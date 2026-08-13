"""Fixed in-process hospital simulation adapter."""

from __future__ import annotations

import threading
import time
import json
import subprocess
from collections import deque
from collections.abc import Callable, Mapping
from pathlib import Path

from agent_ros.adapters._safety import _BoundedCommandWorker, _CommandResult, _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    HospitalAction,
    Observation,
    RobotAdapter,
)


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


class HospitalLifecycleClient:
    """Sealed fixed-action client for the repository-owned example lifecycle."""

    __slots__ = ("_actions", "_emergency_worker", "_lock", "_start_receipt", "_started", "_worker")

    _ROOT = Path(__file__).resolve().parents[2]
    _EXAMPLE_ROOT = _ROOT / "examples" / "hospital_delivery"
    _SCRIPT = _EXAMPLE_ROOT / "scripts" / "codex_project.py"
    _PYTHON = "/usr/bin/python3"
    _TIMEOUTS = {
        HospitalAction.PROBE: 1.0,
        HospitalAction.VALIDATE: 1.0,
        HospitalAction.START: 130.0,
        HospitalAction.STATUS: 12.0,
        HospitalAction.CANCEL: 12.0,
        HospitalAction.STOP: 15.0,
        HospitalAction.OBSERVE: 12.0,
    }

    def __init__(self) -> None:
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
        self._started = False

    @property
    def actions(self) -> tuple[HospitalAction, ...]:
        with self._lock:
            return tuple(self._actions)

    def dispatch(self, action: HospitalAction) -> Mapping[str, object]:
        if not isinstance(action, HospitalAction):
            raise AdapterError("PROFILE_INVALID")
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
        result = self._submit_and_wait(action)
        if action is HospitalAction.STOP:
            self._started = False
        return result

    def start_nowait(self) -> Mapping[str, object]:
        with self._lock:
            receipt = self._start_receipt
            if receipt is not None and not receipt.done.is_set():
                raise AdapterError("UNSAFE_STATE")
            try:
                self._start_receipt = self._worker.submit(
                    lambda: self._execute_action(HospitalAction.START)
                )
            except Exception:
                raise AdapterError("UNSAFE_STATE") from None
            self._started = True
        return {"available": True, "state": "starting"}

    def stop_nowait(self) -> None:
        with self._lock:
            if len(self._actions) == self._actions.maxlen:
                raise AdapterError("INTERNAL_ERROR")
            self._actions.append(HospitalAction.STOP)
        try:
            self._emergency_worker.submit(lambda: self._execute_action(HospitalAction.STOP))
        except Exception:
            raise AdapterError("UNSAFE_STATE") from None

    def close(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        successful = True
        if self._started:
            try:
                self._submit_and_wait(
                    HospitalAction.STOP,
                    timeout=max(0.0, deadline - time.monotonic()),
                )
                self._started = False
            except AdapterError:
                successful = False
        successful = (
            self._worker.close(max(0.0, deadline - time.monotonic())) and successful
        )
        successful = (
            self._emergency_worker.close(max(0.0, deadline - time.monotonic()))
            and successful
        )
        return successful

    def _start_result(self) -> Mapping[str, object] | None:
        with self._lock:
            receipt = self._start_receipt
        if receipt is None:
            return {"available": True, "state": "idle"}
        if not receipt.done.is_set():
            return {"available": True, "state": "starting"}
        if receipt.error is not None:
            raise AdapterError(getattr(receipt.error, "code", "INTERNAL_ERROR"))
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
        if action is HospitalAction.START:
            self._run_fixed(("start", "--timeout", "60"), timeout=120.0)
            self._run_fixed(("mission-start",), timeout=10.0)
            return {"available": True, "state": "running"}
        command = {
            HospitalAction.STATUS: ("mission-status",),
            HospitalAction.CANCEL: ("mission-cancel",),
            HospitalAction.STOP: ("stop",),
            HospitalAction.OBSERVE: ("mission-status",),
        }.get(action)
        if command is None:
            raise AdapterError("PROFILE_INVALID")
        payload = self._run_fixed(command, timeout=self._TIMEOUTS[action])
        if action is HospitalAction.CANCEL:
            return {"available": True, "state": "cancelled", "result": payload}
        if action is HospitalAction.STOP:
            return {"available": True, "state": "stopped", "result": payload}
        status = payload.get("status")
        if isinstance(status, Mapping):
            return dict(status)
        return payload

    def _run_fixed(self, suffix: tuple[str, ...], *, timeout: float) -> dict[str, object]:
        argv = [self._PYTHON, str(self._SCRIPT), *suffix]
        try:
            result = subprocess.run(
                argv,
                cwd=self._EXAMPLE_ROOT,
                capture_output=True,
                text=True,
                timeout=timeout,
                shell=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            raise AdapterError("TIMEOUT") from None
        try:
            payload = _exact_json_object(result.stdout)
        except AdapterError:
            raise
        if result.returncode != 0 or payload.get("ok") is not True:
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
        return Observation(source, self._clock(), result)

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
    __slots__ = ("_client",)

    def __init__(self, client: HospitalLifecycleClient) -> None:
        super().__init__(hardware_verified=False)
        self._client = client

    def _preflight(self) -> bool:
        return True

    def _enqueue_zero_disable(self) -> None:
        self._client.stop_nowait()


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
