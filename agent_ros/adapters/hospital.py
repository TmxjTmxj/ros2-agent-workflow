"""Fixed in-process hospital simulation adapter."""

from __future__ import annotations

import threading
import time
import json
import math
import os
import signal
import subprocess
from collections import deque
from collections.abc import Callable, Mapping
from pathlib import Path
from types import MappingProxyType

from agent_ros.adapters._safety import _BoundedCommandWorker, _CommandResult, _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    HospitalAction,
    Observation,
    RobotAdapter,
)
from agent_ros.safety.outcome import EmergencyStopResult


class HospitalSimulationRuntime:
    """Repository-owned fixed-enum lifecycle that cannot dispatch to hardware."""

    __slots__ = ("_commands", "_lock", "_state")

    def __init__(self) -> None:
        self._commands: deque[HospitalAction] = deque(maxlen=64)
        self._lock = threading.Lock()
        self._state = "idle"

    @property
    def commands(self) -> tuple[HospitalAction, ...]:
        with self._lock:
            return tuple(self._commands)

    def dispatch(self, action: HospitalAction) -> Mapping[str, object]:
        if not isinstance(action, HospitalAction):
            raise AdapterError("PROFILE_INVALID")
        with self._lock:
            if len(self._commands) == self._commands.maxlen:
                raise AdapterError("INTERNAL_ERROR")
            self._commands.append(action)
            if action is HospitalAction.START:
                self._state = "running"
            elif action is HospitalAction.CANCEL:
                self._state = "cancelled"
            elif action is HospitalAction.STOP:
                self._state = "stopped"
            return {"available": True, "state": self._state}

    def start_nowait(self) -> Mapping[str, object]:
        """Reserve START without ever waiting behind another simulation action."""
        if not self._lock.acquire(blocking=False):
            raise AdapterError("INTERNAL_ERROR")
        try:
            if len(self._commands) == self._commands.maxlen:
                raise AdapterError("INTERNAL_ERROR")
            self._commands.append(HospitalAction.START)
            self._state = "running"
            return {"available": True, "state": self._state}
        finally:
            self._lock.release()


class HospitalDeliveryAdapter(RobotAdapter):
    """Expose only the repository-owned simulation lifecycle."""

    def __init__(
        self,
        runtime: HospitalSimulationRuntime,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if type(runtime) is not HospitalSimulationRuntime:
            raise AdapterError("PROFILE_INVALID")
        self._runtime = runtime
        self._clock = clock
        self._simulation_stop_enqueued = 0
        self._safety_channel = _HospitalSimulationEmergencyChannel(self)

    def probe(self) -> AdapterProbe:
        result = self._invoke(HospitalAction.PROBE)
        return AdapterProbe(bool(result.get("available", True)), ("hospital.delivery",))

    def validate(self) -> None:
        self._invoke(HospitalAction.VALIDATE)

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        if task is not HospitalAction.START:
            raise AdapterError("PROFILE_INVALID")
        before_activation = getattr(self, "_before_activation", None)
        if before_activation is not None:
            before_activation()
        return self._activate_owned_start(
            activation_permit,
            self._start_nowait,
        )

    def status(self) -> AdapterStatus:
        return self._status_from(HospitalAction.STATUS)

    def cancel(self) -> AdapterStatus:
        return self._status_from(HospitalAction.CANCEL)

    def stop(self) -> None:
        self._invoke(HospitalAction.STOP)

    def observe(self, source: str) -> Observation:
        if source != "hospital_state":
            raise AdapterError("PROFILE_INVALID")
        result = self._invoke(HospitalAction.OBSERVE)
        return Observation(source, self._clock(), result)

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        return False

    def _emergency_stop_channel(self):
        return self._safety_channel

    def _status_from(self, action: HospitalAction) -> AdapterStatus:
        result = self._invoke(action)
        state = result.get("state")
        if not isinstance(state, str) or not state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(state, values=result)

    def _start_nowait(self) -> AdapterStatus:
        try:
            result = dict(self._runtime.start_nowait())
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        state = result.get("state")
        if not isinstance(state, str) or not state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(state, values=result)

    def _invoke(self, action: HospitalAction) -> dict[str, object]:
        try:
            result = self._runtime.dispatch(action)
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        if not isinstance(result, Mapping):
            raise AdapterError("INTERNAL_ERROR")
        return dict(result)


class _HospitalSimulationEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, adapter: HospitalDeliveryAdapter) -> None:
        super().__init__(hardware_verified=False)
        self._adapter = adapter

    def _preflight(self) -> bool:
        return True

    def _enqueue_zero_disable(self) -> None:
        self._adapter._simulation_stop_enqueued += 1


class RclpyHospitalTransport:
    """Persistent typed ROS control plane for the fixed hospital mission node."""

    __slots__ = (
        "_clients",
        "_closed",
        "_condition",
        "_node",
        "_status",
        "_status_error",
        "_status_received_at",
        "_subscription",
    )

    _SERVICES = {
        HospitalAction.START: "/hospital_mission/start",
        HospitalAction.CANCEL: "/hospital_mission/cancel",
        HospitalAction.STOP: "/hospital_mission/estop",
    }
    _STATUS_FRESHNESS = 1.0

    def __init__(self, node: object) -> None:
        try:
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
        except ImportError:
            raise AdapterError("PROFILE_INVALID") from None
        create_client = getattr(node, "create_client", None)
        create_subscription = getattr(node, "create_subscription", None)
        if not callable(create_client) or not callable(create_subscription):
            raise AdapterError("PROFILE_INVALID")
        self._node = node
        self._condition = threading.Condition()
        self._status: dict[str, object] | None = None
        self._status_error = False
        self._status_received_at: float | None = None
        self._closed = False
        try:
            self._clients = {
                action: create_client(Trigger, service)
                for action, service in self._SERVICES.items()
            }
            self._subscription = create_subscription(
                String, "/hospital_mission/status", self._receive_status, 10
            )
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None

    def trigger(self, action: HospitalAction, timeout: float) -> dict[str, object]:
        client = self._clients.get(action)
        if client is None or self._closed:
            raise AdapterError("PROFILE_INVALID")
        deadline = time.monotonic() + max(0.0, timeout)
        try:
            if not client.wait_for_service(
                timeout_sec=max(0.0, deadline - time.monotonic())
            ):
                raise AdapterError("TIMEOUT")
            request_type = getattr(getattr(client, "srv_type", None), "Request", None)
            if request_type is None:
                from std_srvs.srv import Trigger

                request_type = Trigger.Request
            future = client.call_async(request_type())
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(max(0.0, deadline - time.monotonic())):
                remove_pending = getattr(client, "remove_pending_request", None)
                if callable(remove_pending):
                    try:
                        remove_pending(future)
                    except Exception:
                        pass
                raise AdapterError("TIMEOUT")
            response = future.result()
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("UNSAFE_STATE") from None
        if response is None or getattr(response, "success", None) is not True:
            raise AdapterError("UNSAFE_STATE")
        message = getattr(response, "message", "")
        return {
            "ok": True,
            "success": True,
            "message": message if isinstance(message, str) else "",
        }

    def status(self, timeout: float) -> dict[str, object]:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._condition:
            while True:
                if self._closed or self._status_error:
                    raise AdapterError("INTERNAL_ERROR")
                received = self._status_received_at
                if (
                    self._status is not None
                    and received is not None
                    and time.monotonic() - received <= self._STATUS_FRESHNESS
                ):
                    return dict(self._status)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise AdapterError("TIMEOUT")
                self._condition.wait(remaining)

    def close(self) -> bool:
        with self._condition:
            if self._closed:
                return True
            self._closed = True
            self._condition.notify_all()
        successful = True
        destroy_subscription = getattr(self._node, "destroy_subscription", None)
        if callable(destroy_subscription):
            try:
                successful = destroy_subscription(self._subscription) is not False and successful
            except Exception:
                successful = False
        destroy_client = getattr(self._node, "destroy_client", None)
        if callable(destroy_client):
            for client in self._clients.values():
                try:
                    successful = destroy_client(client) is not False and successful
                except Exception:
                    successful = False
        return successful

    def _receive_status(self, message: object) -> None:
        raw = getattr(message, "data", None)
        parsed: object = None
        if isinstance(raw, str):
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError:
                parsed = None
        valid = (
            isinstance(parsed, dict)
            and all(isinstance(key, str) for key in parsed)
            and isinstance(parsed.get("state"), str)
            and bool(parsed["state"])
        )
        with self._condition:
            if self._closed:
                return
            self._status_error = not valid
            if valid:
                self._status = parsed
                self._status_received_at = time.monotonic()
            self._condition.notify_all()


class HospitalLifecycleClient:
    """Sealed fixed-action client for the repository-owned example lifecycle."""

    __slots__ = (
        "_actions",
        "_cancellation_generation",
        "_emergency_worker",
        "_lock",
        "_start_receipt",
        "_start_process",
        "_started",
        "_stop_latched",
        "_transport",
        "_worker",
    )

    _ROOT = Path(__file__).resolve().parents[2]
    _EXAMPLE_ROOT = _ROOT / "examples" / "hospital_delivery"
    _SCRIPT = _EXAMPLE_ROOT / "scripts" / "codex_project.py"
    _PYTHON = "/usr/bin/python3"
    _RECORDED_ACTIONS = frozenset(
        {
            HospitalAction.VALIDATE,
            HospitalAction.START,
            HospitalAction.CANCEL,
            HospitalAction.STOP,
        }
    )
    _TIMEOUTS = {
        HospitalAction.PROBE: 1.0,
        HospitalAction.VALIDATE: 1.0,
        HospitalAction.START: 130.0,
        HospitalAction.STATUS: 12.0,
        HospitalAction.CANCEL: 12.0,
        HospitalAction.STOP: 15.0,
        HospitalAction.OBSERVE: 12.0,
    }

    def __init__(self, transport: RclpyHospitalTransport | None = None) -> None:
        if transport is not None and type(transport) is not RclpyHospitalTransport:
            raise AdapterError("PROFILE_INVALID")
        self._transport = transport
        self._worker = _BoundedCommandWorker("agent-ros-hospital-lifecycle")
        if not self._worker.start():
            raise AdapterError("PROFILE_INVALID")
        self._emergency_worker = _BoundedCommandWorker("agent-ros-hospital-emergency")
        if not self._emergency_worker.start():
            self._worker.close(1.0)
            raise AdapterError("PROFILE_INVALID")
        self._lock = threading.Lock()
        self._actions: deque[HospitalAction] = deque(maxlen=64)
        self._start_receipt: _CommandResult[object] | None = None
        self._start_process: subprocess.Popen[str] | None = None
        self._started = False
        self._cancellation_generation = 0
        self._stop_latched = False

    @property
    def actions(self) -> tuple[HospitalAction, ...]:
        with self._lock:
            return tuple(self._actions)

    def dispatch(self, action: HospitalAction) -> Mapping[str, object]:
        if not isinstance(action, HospitalAction):
            raise AdapterError("PROFILE_INVALID")
        if action in self._RECORDED_ACTIONS:
            with self._lock:
                if len(self._actions) == self._actions.maxlen:
                    raise AdapterError("INTERNAL_ERROR")
                self._actions.append(action)
        if action is HospitalAction.START:
            return self.start_nowait()
        if action is HospitalAction.STATUS:
            pending = self._start_result()
            if pending is not None:
                return pending
        if action is HospitalAction.PROBE:
            return {"available": self._SCRIPT.is_file(), "state": "available"}
        if action is HospitalAction.VALIDATE:
            if not self._SCRIPT.is_file() or self._SCRIPT.parent.parent != self._EXAMPLE_ROOT:
                raise AdapterError("PROFILE_INVALID")
            return {"available": True, "state": "validated"}
        if action is HospitalAction.STOP:
            self._latch_stop()
        result = self._submit_and_wait(action)
        if action is HospitalAction.STOP:
            self._started = False
        return result

    def start_nowait(self) -> Mapping[str, object]:
        with self._lock:
            if self._stop_latched:
                raise AdapterError("ESTOP_LATCHED")
            receipt = self._start_receipt
            if receipt is not None and not receipt.done.is_set():
                raise AdapterError("UNSAFE_STATE")
            try:
                generation = self._cancellation_generation
                self._start_receipt = self._worker.submit(lambda: self._execute_start(generation))
            except Exception:
                raise AdapterError("UNSAFE_STATE") from None
            self._started = True
        return {"available": True, "state": "starting"}

    def stop_emergency(self) -> None:
        """Send the immediate stop on the independent worker and await it."""
        transport = self._transport
        command = (
            (lambda: transport.trigger(
                HospitalAction.STOP, self._TIMEOUTS[HospitalAction.STOP]
            ))
            if transport is not None
            else (lambda: self._execute_action(HospitalAction.STOP))
        )
        try:
            receipt = self._emergency_worker.submit(command)
        except Exception:
            raise AdapterError("UNSAFE_STATE") from None
        if not receipt.done.wait(self._TIMEOUTS[HospitalAction.STOP]):
            raise AdapterError("TIMEOUT")
        if receipt.error is not None:
            raise AdapterError(getattr(receipt.error, "code", "INTERNAL_ERROR"))
        if not isinstance(receipt.value, Mapping):
            raise AdapterError("INTERNAL_ERROR")
        with self._lock:
            self._started = False

    def close(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        successful = True
        if self._started:
            try:
                self._latch_stop()
                self._submit_and_wait(
                    HospitalAction.STOP,
                    timeout=max(0.0, deadline - time.monotonic()),
                )
                self._started = False
            except AdapterError:
                successful = False
        if self._transport is not None:
            successful = self._transport.close() and successful
        successful = (
            self._worker.close(max(0.0, deadline - time.monotonic())) and successful
        )
        successful = (
            self._emergency_worker.close(max(0.0, deadline - time.monotonic()))
            and successful
        )
        return successful

    def _latch_stop(self) -> None:
        with self._lock:
            self._cancellation_generation += 1
            self._stop_latched = True

    def _start_was_cancelled(self, generation: int) -> bool:
        with self._lock:
            return self._stop_latched or generation != self._cancellation_generation

    def _execute_start(self, generation: int) -> Mapping[str, object]:
        start_result = self._run_fixed(
            ("start", "--timeout", "60"), timeout=120.0, generation=generation
        )
        if start_result is None:
            return self._compensate_cancelled_start()
        if self._start_was_cancelled(generation):
            return self._compensate_cancelled_start()
        if self._transport is None:
            mission_result = self._run_fixed(
                ("mission-start",), timeout=10.0, generation=generation
            )
        else:
            if self._start_was_cancelled(generation):
                return self._compensate_cancelled_start()
            if not self._wait_for_odometry_ready(generation, 10.0):
                return self._compensate_cancelled_start()
            mission_result = self._transport.trigger(HospitalAction.START, 10.0)
        if mission_result is None:
            return self._compensate_cancelled_start()
        if self._start_was_cancelled(generation):
            return self._compensate_cancelled_start()
        return {"available": True, "state": "running"}

    def _wait_for_odometry_ready(self, generation: int, timeout: float) -> bool:
        transport = self._transport
        if transport is None:
            raise AdapterError("PROFILE_INVALID")
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self._start_was_cancelled(generation):
                return False
            remaining = deadline - time.monotonic()
            try:
                status = transport.status(min(0.5, max(0.0, remaining)))
            except AdapterError as exc:
                if exc.code != "TIMEOUT":
                    raise
                continue
            pose = status.get("pose")
            odom_age = status.get("odom_age")
            sim_time = status.get("sim_time")
            ready = (
                isinstance(pose, Mapping)
                and all(
                    not isinstance(pose.get(key), bool)
                    and isinstance(pose.get(key), (int, float))
                    and math.isfinite(float(pose[key]))
                    for key in ("x", "y", "yaw")
                )
                and not isinstance(odom_age, bool)
                and isinstance(odom_age, (int, float))
                and math.isfinite(float(odom_age))
                and 0.0 <= float(odom_age) <= 1.0
                and status.get("feedback_source") == "gazebo_diff_drive_odometry"
                and not isinstance(sim_time, bool)
                and isinstance(sim_time, (int, float))
                and math.isfinite(float(sim_time))
                and float(sim_time) > 0.0
            )
            if ready:
                return True
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        raise AdapterError("TIMEOUT")

    def _compensate_cancelled_start(self) -> Mapping[str, object]:
        try:
            self._run_fixed(("stop",), timeout=self._TIMEOUTS[HospitalAction.STOP])
        finally:
            with self._lock:
                self._started = False
        return {"available": True, "state": "cancelled", "code": "ESTOP_LATCHED"}

    def _start_result(self) -> Mapping[str, object] | None:
        with self._lock:
            receipt = self._start_receipt
        if receipt is None:
            return {"available": True, "state": "idle"}
        if not receipt.done.is_set():
            return {"available": True, "state": "starting"}
        if receipt.error is not None:
            raise AdapterError(getattr(receipt.error, "code", "INTERNAL_ERROR"))
        if isinstance(receipt.value, Mapping) and receipt.value.get("state") == "cancelled":
            return dict(receipt.value)
        return None

    def _submit_and_wait(
        self, action: HospitalAction, *, timeout: float | None = None
    ) -> Mapping[str, object]:
        try:
            receipt = self._worker.submit(lambda: self._execute_action(action))
        except Exception:
            raise AdapterError("UNSAFE_STATE") from None
        wait_timeout = self._TIMEOUTS[action] if timeout is None else max(0.0, timeout)
        if not receipt.done.wait(wait_timeout):
            raise AdapterError("TIMEOUT")
        if receipt.error is not None:
            raise AdapterError(getattr(receipt.error, "code", "INTERNAL_ERROR"))
        if not isinstance(receipt.value, Mapping):
            raise AdapterError("INTERNAL_ERROR")
        return dict(receipt.value)

    def _execute_action(self, action: HospitalAction) -> Mapping[str, object]:
        if self._transport is not None and action in {
            HospitalAction.STATUS,
            HospitalAction.OBSERVE,
        }:
            return self._transport.status(self._TIMEOUTS[action])
        if self._transport is not None and action is HospitalAction.CANCEL:
            payload = self._transport.trigger(action, self._TIMEOUTS[action])
            return {"available": True, "state": "cancelled", "result": payload}
        command = {
            HospitalAction.STATUS: ("mission-status",),
            HospitalAction.CANCEL: ("mission-cancel",),
            HospitalAction.STOP: ("stop",),
            HospitalAction.OBSERVE: ("mission-status",),
        }.get(action)
        if command is None:
            raise AdapterError("PROFILE_INVALID")
        deadline = time.monotonic() + self._TIMEOUTS[action]
        if action is HospitalAction.STOP:
            try:
                # The repository lifecycle lock first cancels a pre-spawn
                # reservation or observes a fully recorded launch. Only then
                # may the exact outer START process group be interrupted.
                payload = self._run_fixed(
                    command, timeout=max(0.0, deadline - time.monotonic())
                )
            finally:
                self._interrupt_inflight_start(deadline)
        else:
            payload = self._run_fixed(command, timeout=self._TIMEOUTS[action])
        if action is HospitalAction.CANCEL:
            return {"available": True, "state": "cancelled", "result": payload}
        if action is HospitalAction.STOP:
            return {"available": True, "state": "stopped", "result": payload}
        status = payload.get("status")
        if isinstance(status, Mapping):
            return dict(status)
        return payload

    def _interrupt_inflight_start(self, deadline: float) -> None:
        """Cancel only the exact START process group and await its owned receipt."""
        with self._lock:
            process = self._start_process
            receipt = self._start_receipt
        if process is not None:
            try:
                if process.poll() is None:
                    self._signal_exact_start_group(process, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except AdapterError:
                raise
            except OSError:
                raise AdapterError("UNSAFE_STATE") from None
        if receipt is not None and not receipt.done.wait(
            max(0.0, deadline - time.monotonic())
        ):
            raise AdapterError("TIMEOUT")

    @staticmethod
    def _signal_exact_start_group(
        process: subprocess.Popen[str], sig: signal.Signals
    ) -> None:
        pgid = os.getpgid(process.pid)
        if pgid != process.pid:
            raise AdapterError("UNSAFE_STATE")
        os.killpg(pgid, sig)

    def _run_fixed(
        self,
        suffix: tuple[str, ...],
        *,
        timeout: float,
        generation: int | None = None,
    ) -> dict[str, object] | None:
        argv = [self._PYTHON, str(self._SCRIPT), *suffix]
        is_initial_start = suffix[:1] == ("start",)
        process: subprocess.Popen[str] | None = None
        try:
            if generation is None:
                process = subprocess.Popen(
                    argv,
                    cwd=self._EXAMPLE_ROOT,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    shell=False,
                )
            else:
                # The latch check and exact START/mission-start spawn share one
                # linearization point. An initial START is also registered as
                # an owned process-group leader before emergency stop can win.
                with self._lock:
                    if (
                        self._stop_latched
                        or generation != self._cancellation_generation
                    ):
                        return None
                    process = subprocess.Popen(
                        argv,
                        cwd=self._EXAMPLE_ROOT,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        shell=False,
                        start_new_session=is_initial_start,
                    )
                    if is_initial_start:
                        self._start_process = process
            stdout, _stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            cleanup_failed = False
            if is_initial_start:
                self._latch_stop()
                try:
                    # Coordinate with the inner lifecycle before touching the
                    # outer wrapper. Its file lock makes a build reservation
                    # cancellable and a spawned launch visible in state first.
                    self._run_fixed(
                        ("stop",), timeout=self._TIMEOUTS[HospitalAction.STOP]
                    )
                except Exception:
                    cleanup_failed = True
                try:
                    self._signal_exact_start_group(process, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception:
                    cleanup_failed = True
            else:
                try:
                    process.kill()
                except Exception:
                    cleanup_failed = True
            try:
                process.communicate(timeout=2.0)
            except Exception:
                cleanup_failed = True
            if cleanup_failed:
                raise AdapterError("CLEANUP_FAILED") from None
            raise AdapterError("TIMEOUT") from None
        except OSError:
            raise AdapterError("TIMEOUT") from None
        finally:
            if is_initial_start and process is not None:
                with self._lock:
                    if self._start_process is process:
                        self._start_process = None
        if generation is not None and self._start_was_cancelled(generation):
            return None
        try:
            payload = _exact_json_object(stdout)
        except AdapterError:
            raise
        if process.returncode != 0 or payload.get("ok") is not True:
            raise AdapterError("UNSAFE_STATE")
        return payload


class HospitalCaseAdapter(RobotAdapter):
    """Focused adapter whose only authority is the fixed hospital lifecycle."""

    __slots__ = ("_client", "_clock", "_safety_channel", "_safety_sequencer", "_runtime_owner_close")

    def __init__(
        self, client: HospitalLifecycleClient, *, clock: Callable[[], float] = time.monotonic
    ) -> None:
        if type(client) is not HospitalLifecycleClient:
            raise AdapterError("PROFILE_INVALID")
        self._client = client
        self._clock = clock
        self._safety_channel = _HospitalCaseEmergencyChannel(client)

    def close(self, timeout: float = 1.0) -> bool:
        """Close the sealed lifecycle client's workers, then base-owned resources."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        successful = True
        try:
            successful = self._client.close(
                max(0.0, deadline - time.monotonic())
            ) and successful
        except Exception:
            successful = False
        return super().close(max(0.0, deadline - time.monotonic())) and successful

    def probe(self) -> AdapterProbe:
        result = self._invoke(HospitalAction.PROBE)
        return AdapterProbe(bool(result.get("available")), ("hospital.delivery",))

    def validate(self) -> None:
        self._invoke(HospitalAction.VALIDATE)

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        if task is not HospitalAction.START:
            raise AdapterError("PROFILE_INVALID")
        return self._activate_owned_start(activation_permit, self._start_nowait)

    def status(self) -> AdapterStatus:
        return self._status_from(HospitalAction.STATUS)

    def cancel(self) -> AdapterStatus:
        return self._status_from(HospitalAction.CANCEL)

    def stop(self) -> None:
        self._invoke(HospitalAction.STOP)

    def observe(self, source: str) -> Observation:
        if source not in {"odometry", "camera", "scan"}:
            raise AdapterError("PROFILE_INVALID")
        result = self._invoke(HospitalAction.OBSERVE)
        if source == "odometry":
            pose = result.get("pose")
            if not isinstance(pose, Mapping):
                raise AdapterError("INTERNAL_ERROR")
            projected: dict[str, float] = {}
            for key in ("x", "y", "yaw"):
                value = pose.get(key)
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                ):
                    raise AdapterError("INTERNAL_ERROR")
                projected[key] = float(value)
            result = projected
        return Observation(source, self._clock(), result)

    def freeze_terminal_evidence(
        self,
        sources: tuple[str, ...],
        terminal_status: AdapterStatus | None = None,
    ) -> Mapping[str, Observation]:
        """Build the hospital terminal snapshot from the status that reported success.

        The status payload already contains the final DiffDrive pose and ROS sim
        time used to declare ``succeeded``. Reusing it avoids a second live
        ROS observation between success and cleanup.
        """
        if not isinstance(sources, tuple) or not all(
            isinstance(source, str) and source for source in sources
        ):
            raise AdapterError("PROFILE_INVALID")
        captured: dict[str, Observation] = dict(
            super().freeze_terminal_evidence(
                tuple(source for source in sources if source != "odometry"),
                terminal_status,
            )
        )
        if "odometry" not in sources:
            return MappingProxyType(captured)
        if terminal_status is None or terminal_status.state != "succeeded":
            raise AdapterError("EVIDENCE_INVALID")
        pose = terminal_status.values.get("pose")
        if not isinstance(pose, Mapping):
            raise AdapterError("EVIDENCE_INVALID")
        projected: dict[str, float] = {}
        for key in ("x", "y", "yaw"):
            value = pose.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise AdapterError("EVIDENCE_INVALID")
            projected[key] = float(value)
        sim_time = terminal_status.values.get("sim_time")
        if (
            isinstance(sim_time, bool)
            or not isinstance(sim_time, (int, float))
            or not math.isfinite(float(sim_time))
            or float(sim_time) < 0.0
        ):
            raise AdapterError("EVIDENCE_INVALID")
        captured["odometry"] = Observation(
            "odometry",
            float(sim_time),
            projected,
        )
        return MappingProxyType(captured)

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        return False

    def _emergency_stop_channel(self):
        return self._safety_channel

    def _start_nowait(self) -> AdapterStatus:
        result = self._invoke(HospitalAction.START)
        return AdapterStatus(str(result.get("state", "starting")), values=result)

    def _status_from(self, action: HospitalAction) -> AdapterStatus:
        result = self._invoke(action)
        raw_state = result.get("state")
        if not isinstance(raw_state, str) or not raw_state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(raw_state.lower(), values=result)

    def _invoke(self, action: HospitalAction) -> dict[str, object]:
        try:
            result = self._client.dispatch(action)
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        if not isinstance(result, Mapping):
            raise AdapterError("INTERNAL_ERROR")
        return dict(result)


class _HospitalCaseEmergencyChannel(_EmergencyStopChannel):
    __slots__ = ("_client", "_receipt", "_stop_lock")

    def __init__(self, client: HospitalLifecycleClient) -> None:
        super().__init__(hardware_verified=False)
        self._client = client
        self._receipt: _CommandResult[object] | None = None
        self._stop_lock = threading.Lock()

    def _preflight(self) -> bool:
        return True

    def _submit_zero(self) -> None:
        # Latch the lifecycle before reporting that the safety command was
        # accepted, then retain the exact worker receipt for `_stop` to verify.
        self._client._latch_stop()
        self._receipt = self._worker.submit(self._execute_zero_disable)

    def _stop(self, timeout: float = 1.0) -> EmergencyStopResult:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._stop_lock:
            self._receipt = None
            result = super()._stop(max(0.0, deadline - time.monotonic()))
            receipt = self._receipt
            completed = (
                receipt is not None
                and receipt.done.wait(max(0.0, deadline - time.monotonic()))
                and receipt.error is None
            )
            if result.safety_command_accepted and not completed:
                return EmergencyStopResult(
                    result.latched,
                    result.activation_quiesced,
                    False,
                    "SAFETY_COMMAND_FAILED",
                )
            return result

    def _enqueue_zero_disable(self) -> None:
        self._client.stop_emergency()


def _exact_json_object(raw: str) -> dict[str, object]:
    try:
        decoder = json.JSONDecoder()
        value, end = decoder.raw_decode(raw.lstrip())
    except (json.JSONDecodeError, TypeError):
        raise AdapterError("INTERNAL_ERROR") from None
    leading = len(raw) - len(raw.lstrip())
    if raw[leading + end :].strip() or not isinstance(value, dict):
        raise AdapterError("INTERNAL_ERROR")
    if not all(isinstance(key, str) for key in value):
        raise AdapterError("INTERNAL_ERROR")
    return value
