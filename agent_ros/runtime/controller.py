"""Generic runtime orchestration across profiles, safety, adapters, audit, and evidence."""

from __future__ import annotations

import json
import os
import time
import threading
from collections.abc import Callable
from pathlib import Path

from agent_ros.adapters._safety import _ActivationIssuer, _ActivationRejected
from agent_ros.adapters.base import AdapterError, HospitalAction, Observation, RobotAdapter
from agent_ros.adapters.hospital import HospitalDeliveryAdapter
from agent_ros.discovery.inference import infer_capabilities
from agent_ros.discovery.ros_graph import RosGraphProbe
from agent_ros.errors import DiscoveryError, ProfileValidationError
from agent_ros.profiles.loader import load_robot_profile, load_task_profile
from agent_ros.profiles.models import RobotProfile, TaskProfile
from agent_ros.runtime.audit import (
    AuditError,
    AuditEvent,
    AuditIntegrityError,
    AuditOperation,
    AuditOutcome,
    AuditWriter,
    _AuditAppendWorker,
    validate_audit_history,
)
from agent_ros.runtime.evidence import EvidenceError, EvidenceReference, EvidenceStore
from agent_ros.safety.gateway import SafetyError, SafetyGateway, SafetyTransition
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.state import SafetyState


_QUARANTINE_TEXT = b"AUDIT_INTEGRITY_COMPROMISED\n"
_PUBLIC_CODES = frozenset({
    "UNSAFE_STATE",
    "PROFILE_INVALID",
    "CONTROLLER_CONFLICT",
    "STALE_FEEDBACK",
    "TIMEOUT",
    "EVIDENCE_INVALID",
    "AUDIT_INTEGRITY_COMPROMISED",
    "ESTOP_LATCHED",
    "OPERATOR_REQUIRED",
    "CLEANUP_FAILED",
})


class RuntimeControllerError(RuntimeError):
    """A stable public code with no underlying ROS, process, or path details."""

    def __init__(self, code: str) -> None:
        self.code = code if code in _PUBLIC_CODES else "UNSAFE_STATE"
        super().__init__(self.code)


AdapterFactory = Callable[[RobotProfile], RobotAdapter]


class RuntimeController:
    """Own exactly one active robot, safety gateway, adapter, and audit writer."""

    def __init__(
        self,
        *,
        profiles_root: Path,
        evidence_dir: Path,
        runtime_dir: Path,
        graph_probe: RosGraphProbe | None = None,
        adapter_factory: AdapterFactory | None = None,
        audit_writer: AuditWriter | None = None,
        clock: Callable[[], float] = time.monotonic,
        boot_id: Callable[[], str] | None = None,
        monitor_interval: float = 0.05,
        cleanup_timeout: float = 1.0,
        monitor_thread_factory: Callable[..., threading.Thread] = threading.Thread,
    ) -> None:
        self._profiles_root = Path(profiles_root)
        self._runtime_dir = Path(runtime_dir)
        self._runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        os.chmod(self._runtime_dir, 0o700)
        self._audit_path = self._runtime_dir / "audit.jsonl"
        self._quarantine_path = self._runtime_dir / "audit.quarantine"
        self._graph_probe = graph_probe or RosGraphProbe()
        self._adapter_factory = adapter_factory
        self._audit_writer = audit_writer or AuditWriter(self._audit_path)
        self._audit_worker = _AuditAppendWorker(self._audit_writer)
        self._clock = clock
        self._boot_id = boot_id
        self._monitor_interval = monitor_interval
        self._cleanup_timeout = cleanup_timeout
        self._monitor_thread_factory = monitor_thread_factory
        self._lock = threading.RLock()
        self._transition_lock = threading.RLock()
        self._transition_condition = threading.Condition(self._transition_lock)
        # Retain the audit-lock name for lifecycle diagnostics; transition and
        # durability coordination intentionally share this one condition.
        self._audit_lock = self._transition_lock
        self._audit_condition = self._transition_condition
        self._next_audit_sequence = 0
        self._pending_audit: dict[
            int,
            tuple[AuditOperation, SafetyTransition, dict[str, object] | None, AuditOutcome],
        ] = {}
        self._audit_failure = False
        self._audit_draining = False
        self._monitor_stop = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._evidence = EvidenceStore(evidence_dir)
        self._profile: RobotProfile | None = None
        self._adapter: RobotAdapter | None = None
        self._gateway: SafetyGateway | None = None
        self._task: TaskProfile | None = None
        self._stage_index = 0
        self._stage_deadline: float | None = None
        self._observed_gateway_state = SafetyState.NEW
        self._activation_issuer = _ActivationIssuer()
        self._physical_estop_bound = False
        self._hardware_safety_verified = False
        self._cleanup_threads: list[threading.Thread] = []
        self._cleanup_start_failed = False
        self._task_cleanup_started = False
        self._cancel_requested = False
        self._last_status = None
        self._report = None
        self._quarantined = self._quarantine_path.exists() or not self._existing_audit_is_valid()
        if not self._audit_worker.start():
            self._audit_failure = True
            self._quarantined = True
        if self._quarantined and not self._quarantine_path.exists():
            self._persist_quarantine()

    @property
    def state(self) -> SafetyState:
        return SafetyState.NEW if self._gateway is None else self._gateway.state

    @property
    def audit_writer(self) -> AuditWriter:
        """Expose identity for composition tests; callers cannot replace the writer."""
        return self._audit_writer

    def discover_robot(self, profile_hint: str | None = None) -> dict[str, object]:
        self._ensure_available()
        if self._gateway is not None:
            raise RuntimeControllerError("UNSAFE_STATE")
        profile_name = profile_hint if profile_hint is not None else self._only_robot_profile_name()
        try:
            profile = load_robot_profile(profile_name, self._profiles_root)
            snapshot = self._graph_probe.probe()
            report = infer_capabilities(snapshot)
            adapter = self._make_adapter(profile)
        except ProfileValidationError:
            raise RuntimeControllerError("PROFILE_INVALID") from None
        except DiscoveryError:
            raise RuntimeControllerError("UNSAFE_STATE") from None
        except RuntimeControllerError:
            raise
        except Exception:
            raise RuntimeControllerError("UNSAFE_STATE") from None
        if report.blocking_warnings:
            raise RuntimeControllerError("CONTROLLER_CONFLICT")
        try:
            adapter._bind_runtime_safety(self._activation_issuer)
            safety_channel = adapter._emergency_stop_channel()
        except Exception:
            raise RuntimeControllerError("PROFILE_INVALID") from None
        gateway = SafetyGateway(
            profile,
            runtime_dir=self._runtime_dir,
            stop_callback=safety_channel._stop,
            clock=self._clock,
            boot_id=self._boot_id,
        )
        self._profile = profile
        self._adapter = adapter
        self._gateway = gateway
        self._report = report
        try:
            bound = adapter.bind_physical_estop(self._physical_estop)
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._physical_estop_bound = bound is True
        try:
            transition = gateway.discover(report)
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        self._append_transition(AuditOperation.DISCOVER, transition)
        self._observed_gateway_state = gateway.state
        return {
            "profile": profile.name,
            "state": gateway.state.value,
            "capabilities": list(report.capability_names),
            "hardware_safety_channel": self._hardware_safety_status(profile),
        }

    def validate_profile(self, profile_name: str) -> dict[str, object]:
        self._ensure_available()
        gateway, adapter, profile = self._active()
        if profile.name != profile_name:
            raise RuntimeControllerError("PROFILE_INVALID")
        before = gateway.state
        try:
            adapter.validate()
            adapter._validate_runtime_safety(profile.mode)
            if profile.mode == "hardware" and (
                not self._physical_estop_bound or isinstance(adapter, HospitalDeliveryAdapter)
            ):
                raise AdapterError("PROFILE_INVALID")
            if profile.mode == "hardware":
                self._hardware_safety_verified = True
            transition = gateway.validate()
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._append_transition(AuditOperation.VALIDATE, transition)
        return {
            "profile": profile.name,
            "state": gateway.state.value,
            "hardware_safety_channel": self._hardware_safety_status(profile),
        }

    def arm_robot(
        self,
        profile_name: str,
        challenge: str | None = None,
        *,
        dry_run: bool = True,
    ) -> dict[str, object]:
        self._ensure_available()
        gateway, _adapter, profile = self._active()
        if profile.name != profile_name:
            raise RuntimeControllerError("PROFILE_INVALID")
        if gateway.state not in {SafetyState.VALIDATED, SafetyState.ARMED}:
            raise RuntimeControllerError("UNSAFE_STATE")
        if dry_run:
            return {"dry_run": True, "profile": profile.name, "state": gateway.state.value}
        try:
            adapter = self._adapter
            assert adapter is not None
            adapter._validate_runtime_safety(profile.mode)
            if profile.mode == "hardware" and not self._physical_estop_bound:
                raise AdapterError("PROFILE_INVALID")
            transition = gateway.arm(challenge)
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        if transition is not None:
            self._append_transition(AuditOperation.ARM, transition)
        return {"profile": profile.name, "state": gateway.state.value}

    def run_task(self, task_name: str, *, dry_run: bool = False) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, adapter, profile = self._active()
            if gateway.state is not SafetyState.ARMED:
                raise RuntimeControllerError("UNSAFE_STATE")
            task = self._load_compatible_task(task_name, profile)
            if dry_run:
                return {"dry_run": True, "profile": profile.name, "task": task.name}
            try:
                permit = self._activation_issuer._issue()
            except _ActivationRejected as exc:
                raise RuntimeControllerError(exc.code) from None
            request = HospitalAction.START if isinstance(adapter, HospitalDeliveryAdapter) else task.stages[0]
        try:
            transition = gateway.start_task()
            self._append_transition(
                AuditOperation.START_TASK,
                transition,
                operation_data={"task": task.name},
            )
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        try:
            start_status = adapter.start(request, permit)
            if not adapter._permit_is_current(permit) or gateway.state is not SafetyState.RUNNING:
                self._start_task_cleanup(adapter)
                raise RuntimeControllerError("ESTOP_LATCHED")
            heartbeat_transition = gateway.heartbeat()
            self._append_transition(AuditOperation.HEARTBEAT, heartbeat_transition)
        except RuntimeControllerError:
            self._latch_adapter_fault()
            raise
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._append_latest_gateway_fault(gateway)
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        if not adapter._permit_is_current(permit) or gateway.state is not SafetyState.RUNNING:
            self._start_task_cleanup(adapter)
            raise RuntimeControllerError("ESTOP_LATCHED")
        with self._lock:
            self._task = task
            self._last_status = start_status
            self._cancel_requested = False
            self._stage_index = 0
            self._stage_deadline = self._clock() + task.stages[0].timeout
            self._observed_gateway_state = gateway.state
            self._task_cleanup_started = False
        try:
            self._start_monitor()
        except RuntimeControllerError:
            self._latch_adapter_fault()
            raise
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        return {"state": gateway.state.value, "task": task.name}

    def task_status(self) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, _adapter, _profile = self._active()
            status = self._last_status
            return {
                "state": gateway.state.value,
                "task": None if self._task is None else self._task.name,
                "hardware_safety_channel": self._hardware_safety_status(_profile),
                **({} if status is None else {"adapter_state": status.state, "code": status.code}),
            }

    def cancel_task(self) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, adapter, _profile = self._active()
            if gateway.state is not SafetyState.RUNNING:
                raise RuntimeControllerError("UNSAFE_STATE")
        try:
            status = adapter.cancel()
            if gateway.state is not SafetyState.RUNNING:
                raise RuntimeControllerError("ESTOP_LATCHED")
            if status.state == "faulted":
                self._latch_adapter_fault()
                raise RuntimeControllerError("UNSAFE_STATE")
            if status.state == "cancelled":
                transition = gateway.cancel()
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except RuntimeControllerError:
            raise
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        with self._lock:
            self._cancel_requested = status.state == "cancelling"
            self._observed_gateway_state = gateway.state
            if status.state == "cancelled":
                self._monitor_stop.set()
        if status.state == "cancelled":
            self._append_transition(AuditOperation.CANCEL, transition)
        return {"state": gateway.state.value, "adapter_state": status.state}

    def emergency_stop(self) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, _adapter, _profile = self._active()
        attempt = gateway.estop_attempt()
        transition = attempt.transition
        if transition is not None:
            self._append_transition(AuditOperation.ESTOP, transition)
        stop_result = attempt.result
        if stop_result is None or not stop_result.successful:
            raise RuntimeControllerError("UNSAFE_STATE")
        with self._lock:
            self._observed_gateway_state = gateway.state
            self._monitor_stop.set()
            return {"state": gateway.state.value}

    def observe(self, source: str) -> Observation:
        self._ensure_available()
        _gateway, adapter, profile = self._active()
        if source not in profile.observation_sources:
            raise RuntimeControllerError("PROFILE_INVALID")
        try:
            return adapter.observe(source)
        except AdapterError as exc:
            raise self._adapter_error(exc) from None
        except Exception:
            raise RuntimeControllerError("UNSAFE_STATE") from None

    def get_evidence(self, report_id: str | None = None) -> EvidenceReference:
        self._ensure_available()
        try:
            return self._evidence.get(report_id)
        except EvidenceError:
            raise RuntimeControllerError("EVIDENCE_INVALID") from None

    def stop_runtime(self) -> dict[str, object]:
        deadline = time.monotonic() + self._cleanup_timeout

        def remaining() -> float:
            return max(0.0, deadline - time.monotonic())

        with self._lock:
            self._monitor_stop.set()
            gateway = self._gateway
            adapter = self._adapter
        if gateway is None:
            result = {"state": SafetyState.NEW.value}
            cleanup_failed = False
        else:
            cleanup_failed = False
            if gateway.state in {SafetyState.RUNNING, SafetyState.ESTOPPED}:
                require_successful_stop = gateway.state is SafetyState.RUNNING
                attempt = gateway.estop_attempt(timeout=remaining())
                transition = attempt.transition
                if transition is not None:
                    self._register_transition(AuditOperation.ESTOP, transition)
                cleanup_failed = (
                    require_successful_stop and not attempt.result.successful
                )
                if adapter is not None:
                    self._start_task_cleanup(adapter)
            cleanup_failed = not gateway.close(timeout=remaining()) or cleanup_failed
            result = {"state": gateway.state.value}
        try:
            self._drain_pending_transitions(timeout=remaining())
        except RuntimeControllerError:
            cleanup_failed = True
        owned = list(self._cleanup_threads)
        monitor = self._monitor_thread
        if monitor is not None:
            owned.append(monitor)
        for thread in owned:
            if thread is threading.current_thread():
                continue
            thread.join(timeout=remaining())
            cleanup_failed = cleanup_failed or thread.is_alive()
        cleanup_failed = cleanup_failed or self._cleanup_start_failed
        try:
            self._drain_pending_transitions(timeout=remaining())
        except RuntimeControllerError:
            cleanup_failed = True
        cleanup_failed = not self._audit_worker.close(remaining()) or cleanup_failed
        if adapter is not None:
            try:
                cleanup_failed = (
                    not adapter.close(remaining())
                    or cleanup_failed
                )
            except Exception:
                cleanup_failed = True
        if cleanup_failed:
            raise RuntimeControllerError("CLEANUP_FAILED")
        return result

    def _load_compatible_task(self, task_name: str, profile: RobotProfile) -> TaskProfile:
        try:
            task = load_task_profile(task_name, self._profiles_root)
        except ProfileValidationError:
            raise RuntimeControllerError("PROFILE_INVALID") from None
        sources = set(profile.observation_sources)
        if (
            task.robot_profile != profile.name
            or not set(task.required_sensors).issubset(sources)
            or not set(task.evidence_sources).issubset(sources)
        ):
            raise RuntimeControllerError("PROFILE_INVALID")
        return task

    def _active(self) -> tuple[SafetyGateway, RobotAdapter, RobotProfile]:
        if self._gateway is None or self._adapter is None or self._profile is None:
            raise RuntimeControllerError("UNSAFE_STATE")
        return self._gateway, self._adapter, self._profile

    def _make_adapter(self, profile: RobotProfile) -> RobotAdapter:
        if self._adapter_factory is None:
            raise RuntimeControllerError("PROFILE_INVALID")
        try:
            adapter = self._adapter_factory(profile)
        except Exception:
            raise RuntimeControllerError("PROFILE_INVALID") from None
        if not isinstance(adapter, RobotAdapter):
            raise RuntimeControllerError("PROFILE_INVALID")
        return adapter

    def _only_robot_profile_name(self) -> str:
        root = self._profiles_root / "robots"
        try:
            names = sorted(path.stem for path in root.glob("*.yaml") if path.is_file())
        except OSError:
            raise RuntimeControllerError("PROFILE_INVALID") from None
        if len(names) != 1:
            raise RuntimeControllerError("PROFILE_INVALID")
        return names[0]

    def _hardware_safety_status(self, profile: RobotProfile) -> str:
        if profile.mode != "hardware":
            return "simulation_only"
        return "verified" if self._hardware_safety_verified else "unverified"

    def _physical_estop(self, asserted: bool) -> None:
        gateway = self._gateway
        if asserted is not True or gateway is None:
            return
        attempt = gateway.observe_physical_estop_attempt(True)
        transition = attempt.transition
        if transition is not None:
            try:
                self._register_transition(AuditOperation.ESTOP, transition)
            except RuntimeControllerError:
                return
        stop_result = attempt.result
        if stop_result is None or not stop_result.successful:
            self._quarantined = True
            self._persist_quarantine()
        self._observed_gateway_state = gateway.state

    def _latch_adapter_fault(self) -> None:
        if self._gateway is not None:
            transition = self._gateway.estop()
            if transition is not None:
                self._append_transition(AuditOperation.ESTOP, transition)
            self._observed_gateway_state = self._gateway.state
            self._monitor_stop.set()

    def _append_latest_gateway_fault(self, gateway: SafetyGateway) -> None:
        transition = next(
            (
                item
                for item in gateway.transitions_from(self._next_audit_sequence)
                if item.state_before is SafetyState.RUNNING
                and item.state_after is SafetyState.FAULTED
            ),
            None,
        )
        if transition is not None:
            self._append_transition(
                AuditOperation.HEARTBEAT,
                transition,
                outcome=AuditOutcome.FAULTED,
            )

    def _start_task_cleanup(self, adapter: RobotAdapter) -> None:
        with self._lock:
            if self._task_cleanup_started:
                return
            self._task_cleanup_started = True

        def cleanup() -> None:
            try:
                adapter.stop()
            except Exception:
                return

        worker = threading.Thread(target=cleanup, name="agent-ros-task-cleanup", daemon=False)
        try:
            worker.start()
        except Exception:
            with self._lock:
                self._cleanup_start_failed = True
            return
        with self._lock:
            self._cleanup_threads.append(worker)

    def _start_monitor(self) -> None:
        self._monitor_stop.clear()
        thread = self._monitor_thread_factory(
            target=self._monitor_loop,
            name="agent-ros-task-monitor",
            daemon=False,
        )
        thread.start()
        self._monitor_thread = thread

    def _monitor_loop(self) -> None:
        while not self._monitor_stop.wait(self._monitor_interval):
            audit_fault = False
            drain_before_return = False
            with self._lock:
                gateway = self._gateway
                if gateway is None:
                    return
                if gateway.state is SafetyState.FAULTED:
                    audit_fault = self._observed_gateway_state is SafetyState.RUNNING
                    self._observed_gateway_state = SafetyState.FAULTED
                elif gateway.state is not SafetyState.RUNNING:
                    drain_before_return = True
            if drain_before_return:
                try:
                    self._drain_pending_transitions()
                except RuntimeControllerError:
                    pass
                return
            if audit_fault:
                try:
                    self._append_latest_gateway_fault(gateway)
                except RuntimeControllerError:
                    pass
                return
            try:
                self._poll_running()
            except RuntimeControllerError:
                return

    def _poll_running(self):
        with self._lock:
            gateway, adapter, _profile = self._active()
            deadline = self._stage_deadline
        if deadline is not None and self._clock() > deadline:
            self._latch_adapter_fault()
            raise RuntimeControllerError("TIMEOUT")
        try:
            status = adapter.status()
            with self._lock:
                self._last_status = status
            if gateway.state is not SafetyState.RUNNING:
                return status
            if status.state == "cancelled" and self._cancel_requested:
                transition = gateway.cancel()
                self._append_transition(AuditOperation.CANCEL, transition)
                self._observed_gateway_state = gateway.state
                self._monitor_stop.set()
                return status
            if status.state in {"failed", "faulted", "rejected", "cancelled"}:
                self._latch_adapter_fault()
                raise RuntimeControllerError("UNSAFE_STATE")
            if status.state == "succeeded" and self._task is not None:
                if self._stage_index + 1 < len(self._task.stages):
                    self._stage_index += 1
                    stage = self._task.stages[self._stage_index]
                    try:
                        stage_permit = self._activation_issuer._issue()
                    except _ActivationRejected as exc:
                        raise RuntimeControllerError(exc.code) from None
                    adapter.start(stage, stage_permit)
                    if not adapter._permit_is_current(stage_permit) or gateway.state is not SafetyState.RUNNING:
                        self._start_task_cleanup(adapter)
                        raise RuntimeControllerError("ESTOP_LATCHED")
                    with self._lock:
                        self._stage_deadline = self._clock() + stage.timeout
                else:
                    self._start_task_cleanup(adapter)
                    transition = gateway.cancel()
                    self._append_transition(AuditOperation.CANCEL, transition)
                    self._observed_gateway_state = gateway.state
                    self._monitor_stop.set()
                    return status
            if gateway.state is SafetyState.RUNNING:
                transition = gateway.heartbeat()
                self._append_transition(AuditOperation.HEARTBEAT, transition)
            return status
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._append_latest_gateway_fault(gateway)
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except RuntimeControllerError:
            raise
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None

    def _append_transition(
        self,
        operation: AuditOperation,
        transition: SafetyTransition,
        *,
        operation_data: dict[str, object] | None = None,
        outcome: AuditOutcome = AuditOutcome.OK,
    ) -> None:
        if not isinstance(transition, SafetyTransition):
            raise RuntimeControllerError("UNSAFE_STATE")
        self._register_transition(operation, transition, operation_data=operation_data, outcome=outcome)
        self._drain_pending_transitions(
            wait_for=transition.sequence,
            timeout=self._cleanup_timeout,
        )

    def _register_transition(
        self,
        operation: AuditOperation,
        transition: SafetyTransition,
        *,
        operation_data: dict[str, object] | None = None,
        outcome: AuditOutcome = AuditOutcome.OK,
    ) -> None:
        if not isinstance(transition, SafetyTransition):
            raise RuntimeControllerError("UNSAFE_STATE")
        gateway = self._gateway
        if gateway is None or not gateway.owns_transition(transition):
            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")
        if transition.stop_result is not None:
            if operation_data is not None:
                raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")
            operation_data = self._stop_result_data(transition.stop_result)
        with self._transition_condition:
            if self._audit_failure:
                raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")
            if transition.sequence < self._next_audit_sequence:
                return
            existing = self._pending_audit.get(transition.sequence)
            item = (operation, transition, operation_data, outcome)
            if existing is not None and existing != item:
                raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")
            self._pending_audit[transition.sequence] = item
            self._transition_condition.notify_all()

    def _drain_pending_transitions(
        self,
        *,
        wait_for: int | None = None,
        timeout: float | None = None,
    ) -> None:
        deadline = time.monotonic() + (
            self._cleanup_timeout if timeout is None else max(0.0, timeout)
        )
        owns_drain = False
        timed_out = False
        try:
            while True:
                with self._transition_condition:
                    while True:
                        if self._audit_failure:
                            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")
                        item = self._pending_audit.get(self._next_audit_sequence)
                        if item is None:
                            item = self._infer_gateway_fault(self._next_audit_sequence)
                        if (
                            wait_for is not None
                            and wait_for < self._next_audit_sequence
                            and (not owns_drain or item is None)
                        ):
                            return
                        if item is not None and (owns_drain or not self._audit_draining):
                            if not owns_drain:
                                self._audit_draining = True
                                owns_drain = True
                            break
                        if (
                            wait_for is None
                            and not self._pending_audit
                            and (owns_drain or not self._audit_draining)
                        ):
                            return
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            timed_out = True
                            break
                        self._transition_condition.wait(remaining)
                    if timed_out:
                        break
                operation, transition, operation_data, outcome = item
                try:
                    self._append_audit(
                        operation,
                        transition.state_before,
                        transition.state_after,
                        operation_data=operation_data,
                        outcome=outcome,
                        timeout=max(0.0, deadline - time.monotonic()),
                    )
                except RuntimeControllerError:
                    with self._transition_condition:
                        self._audit_failure = True
                        self._transition_condition.notify_all()
                    raise
                with self._transition_condition:
                    self._pending_audit.pop(self._next_audit_sequence, None)
                    self._next_audit_sequence += 1
                    self._transition_condition.notify_all()
        finally:
            if owns_drain:
                with self._transition_condition:
                    self._audit_draining = False
                    self._transition_condition.notify_all()
        if timed_out:
            self._record_audit_failure(timeout=0.0)
            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")

    def _infer_gateway_fault(
        self,
        sequence: int,
    ) -> tuple[AuditOperation, SafetyTransition, dict[str, object] | None, AuditOutcome] | None:
        gateway = self._gateway
        if gateway is None:
            return None
        for transition in gateway.transitions_from(sequence):
            if transition.sequence != sequence:
                return None
            if (
                transition.state_before is SafetyState.RUNNING
                and transition.state_after is SafetyState.FAULTED
            ):
                operation_data = (
                    None
                    if transition.stop_result is None
                    else self._stop_result_data(transition.stop_result)
                )
                return (
                    AuditOperation.HEARTBEAT,
                    transition,
                    operation_data,
                    AuditOutcome.FAULTED,
                )
            return None
        return None

    @staticmethod
    def _stop_result_data(result: EmergencyStopResult) -> dict[str, object]:
        return {
            "latched": result.latched,
            "activation_quiesced": result.activation_quiesced,
            "safety_command_accepted": result.safety_command_accepted,
            "code": result.code,
        }

    def _append_audit(
        self,
        operation: AuditOperation,
        before: SafetyState,
        after: SafetyState,
        *,
        operation_data: dict[str, object] | None = None,
        outcome: AuditOutcome = AuditOutcome.OK,
        timeout: float | None = None,
    ) -> None:
        deadline = time.monotonic() + (
            self._cleanup_timeout if timeout is None else max(0.0, timeout)
        )
        try:
            self._audit_worker.append(
                AuditEvent(
                    operation,
                    before,
                    after,
                    outcome,
                    operation_data={} if operation_data is None else operation_data,
                ),
                max(0.0, deadline - time.monotonic()),
            )
        except (AuditIntegrityError, AuditError, OSError):
            self._record_audit_failure(
                timeout=max(0.0, deadline - time.monotonic())
            )
            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED") from None

    def _record_audit_failure(self, *, timeout: float) -> None:
        with self._transition_condition:
            self._audit_failure = True
            self._transition_condition.notify_all()
        self._quarantined = True
        self._persist_quarantine()
        gateway = self._gateway
        if gateway is not None:
            gateway.estop(timeout=max(0.0, timeout))

    def _persist_quarantine(self) -> None:
        temporary = self._runtime_dir / ".audit.quarantine.tmp"
        try:
            descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            try:
                offset = 0
                while offset < len(_QUARANTINE_TEXT):
                    written = os.write(descriptor, _QUARANTINE_TEXT[offset:])
                    if written <= 0:
                        raise OSError
                    offset += written
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, self._quarantine_path)
            directory = os.open(self._runtime_dir, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            self._quarantined = True

    def _existing_audit_is_valid(self) -> bool:
        if not self._audit_path.exists():
            return True
        try:
            raw = self._audit_path.read_bytes()
            validate_audit_history(raw, require_terminal=True)
        except (OSError, AuditError):
            return False
        return True

    def _ensure_available(self) -> None:
        if self._quarantined or self._quarantine_path.exists():
            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED")

    @staticmethod
    def _adapter_error(error: AdapterError) -> RuntimeControllerError:
        return RuntimeControllerError(error.code)

    @staticmethod
    def _safety_error(error: SafetyError) -> RuntimeControllerError:
        code = str(error)
        if code == "PROFILE_UNSUPPORTED":
            code = "PROFILE_INVALID"
        elif code not in {"UNSAFE_STATE", "ESTOP_LATCHED", "OPERATOR_REQUIRED"}:
            code = "UNSAFE_STATE"
        return RuntimeControllerError(code)
