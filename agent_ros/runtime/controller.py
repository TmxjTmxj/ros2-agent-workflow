"""Generic runtime orchestration across profiles, safety, adapters, audit, and evidence."""

from __future__ import annotations

import json
import os
import time
import threading
from collections.abc import Callable
from pathlib import Path

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
    validate_audit_history,
)
from agent_ros.runtime.evidence import EvidenceError, EvidenceReference, EvidenceStore
from agent_ros.safety.gateway import SafetyError, SafetyGateway
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
        self._clock = clock
        self._boot_id = boot_id
        self._monitor_interval = monitor_interval
        self._cleanup_timeout = cleanup_timeout
        self._monitor_thread_factory = monitor_thread_factory
        self._lock = threading.RLock()
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
        self._generation = 0
        self._cancel_requested = False
        self._last_status = None
        self._report = None
        self._quarantined = self._quarantine_path.exists() or not self._existing_audit_is_valid()
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
        gateway = SafetyGateway(
            profile,
            runtime_dir=self._runtime_dir,
            stop_callback=lambda: self._issue_adapter_stop(adapter),
            clock=self._clock,
            boot_id=self._boot_id,
        )
        self._profile = profile
        self._adapter = adapter
        self._gateway = gateway
        self._generation += 1
        self._report = report
        try:
            bound = adapter.bind_physical_estop(self._physical_estop)
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        if profile.mode == "hardware" and bound is not True:
            self._latch_adapter_fault()
            raise RuntimeControllerError("PROFILE_INVALID")
        before = gateway.state
        try:
            gateway.discover(report)
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        self._append_audit(AuditOperation.DISCOVER, before, gateway.state)
        self._observed_gateway_state = gateway.state
        return {
            "profile": profile.name,
            "state": gateway.state.value,
            "capabilities": list(report.capability_names),
        }

    def validate_profile(self, profile_name: str) -> dict[str, object]:
        self._ensure_available()
        gateway, adapter, profile = self._active()
        if profile.name != profile_name:
            raise RuntimeControllerError("PROFILE_INVALID")
        before = gateway.state
        try:
            adapter.validate()
            gateway.validate()
        except AdapterError as exc:
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._append_audit(AuditOperation.VALIDATE, before, gateway.state)
        return {"profile": profile.name, "state": gateway.state.value}

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
        before = gateway.state
        try:
            gateway.arm(challenge)
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        if gateway.state is not before:
            self._append_audit(AuditOperation.ARM, before, gateway.state)
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
            before = gateway.state
            generation = self._generation
            gateway.start_task()
            request = HospitalAction.START if isinstance(adapter, HospitalDeliveryAdapter) else task.stages[0]
        try:
            start_status = adapter.start(request)
            if gateway.state is not SafetyState.RUNNING:
                adapter.stop()
                raise RuntimeControllerError("ESTOP_LATCHED")
            gateway.heartbeat()
        except RuntimeControllerError:
            self._latch_adapter_fault()
            raise
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        with self._lock:
            stale_start = generation != self._generation or gateway.state is not SafetyState.RUNNING
        if stale_start:
            try:
                adapter.stop()
            finally:
                raise RuntimeControllerError("ESTOP_LATCHED")
        with self._lock:
            self._task = task
            self._last_status = start_status
            self._cancel_requested = False
            self._stage_index = 0
            self._stage_deadline = self._clock() + task.stages[0].timeout
            self._observed_gateway_state = gateway.state
        try:
            self._append_audit(
                AuditOperation.START_TASK,
                before,
                gateway.state,
                operation_data={"task": task.name},
            )
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
                **({} if status is None else {"adapter_state": status.state, "code": status.code}),
            }

    def cancel_task(self) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, adapter, _profile = self._active()
            if gateway.state is not SafetyState.RUNNING:
                raise RuntimeControllerError("UNSAFE_STATE")
            before = gateway.state
            generation = self._generation
        try:
            status = adapter.cancel()
            if generation != self._generation or gateway.state is not SafetyState.RUNNING:
                raise RuntimeControllerError("ESTOP_LATCHED")
            if status.state == "faulted":
                self._latch_adapter_fault()
                raise RuntimeControllerError("UNSAFE_STATE")
            if status.state == "cancelled":
                gateway.cancel()
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
            self._append_audit(AuditOperation.CANCEL, before, gateway.state)
        return {"state": gateway.state.value, "adapter_state": status.state}

    def emergency_stop(self) -> dict[str, object]:
        with self._lock:
            self._ensure_available()
            gateway, _adapter, _profile = self._active()
            before = gateway.state
        gateway.estop()
        if gateway.state is not before:
            self._append_audit(AuditOperation.ESTOP, before, gateway.state)
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
        with self._lock:
            self._monitor_stop.set()
            gateway = self._gateway
            adapter = self._adapter
            self._generation += 1
        if gateway is None:
            result = {"state": SafetyState.NEW.value}
            cleanup_failed = False
        else:
            cleanup_failed = False
            if gateway.state is SafetyState.RUNNING:
                completed, status = self._bounded_adapter_call(adapter.cancel if adapter is not None else lambda: None)
                cleanup_failed = not completed
                if completed and getattr(status, "state", None) == "cancelled":
                    try:
                        gateway.cancel()
                    except SafetyError:
                        gateway.estop()
                else:
                    gateway.estop()
            gateway.close()
            result = {"state": gateway.state.value}
        thread = self._monitor_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self._cleanup_timeout)
            cleanup_failed = cleanup_failed or thread.is_alive()
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

    def _physical_estop(self, asserted: bool) -> None:
        gateway = self._gateway
        if asserted is not True or gateway is None:
            return
        before = gateway.state
        gateway.observe_physical_estop(True)
        self._generation += 1
        self._monitor_stop.set()
        if gateway.state is not before:
            try:
                self._append_audit(AuditOperation.ESTOP, before, gateway.state)
            except RuntimeControllerError:
                return
        self._observed_gateway_state = gateway.state

    def _latch_adapter_fault(self) -> None:
        if self._gateway is not None:
            before = self._gateway.state
            self._gateway.estop()
            if self._gateway.state is not before:
                self._append_audit(AuditOperation.ESTOP, before, self._gateway.state)
            self._observed_gateway_state = self._gateway.state
            self._monitor_stop.set()

    def _start_monitor(self) -> None:
        self._monitor_stop.clear()
        self._monitor_thread = self._monitor_thread_factory(target=self._monitor_loop, name="agent-ros-task-monitor", daemon=True)
        self._monitor_thread.start()

    def _monitor_loop(self) -> None:
        while not self._monitor_stop.wait(self._monitor_interval):
            audit_fault = False
            with self._lock:
                gateway = self._gateway
                if gateway is None:
                    return
                if gateway.state is SafetyState.FAULTED:
                    audit_fault = self._observed_gateway_state is SafetyState.RUNNING
                    self._observed_gateway_state = SafetyState.FAULTED
                elif gateway.state is not SafetyState.RUNNING:
                    return
            if audit_fault:
                try:
                    self._append_audit(
                        AuditOperation.HEARTBEAT,
                        SafetyState.RUNNING,
                        SafetyState.FAULTED,
                        outcome=AuditOutcome.FAULTED,
                    )
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
            generation = self._generation
        if deadline is not None and self._clock() > deadline:
            self._latch_adapter_fault()
            raise RuntimeControllerError("TIMEOUT")
        try:
            status = adapter.status()
            with self._lock:
                self._last_status = status
            if generation != self._generation or gateway.state is not SafetyState.RUNNING:
                return status
            if status.state == "cancelled" and self._cancel_requested:
                before = gateway.state
                gateway.cancel()
                self._append_audit(AuditOperation.CANCEL, before, gateway.state)
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
                    adapter.start(stage)
                    if generation != self._generation or gateway.state is not SafetyState.RUNNING:
                        adapter.stop()
                        raise RuntimeControllerError("ESTOP_LATCHED")
                    with self._lock:
                        self._stage_deadline = self._clock() + stage.timeout
                else:
                    before = gateway.state
                    adapter.stop()
                    gateway.cancel()
                    self._append_audit(AuditOperation.CANCEL, before, gateway.state)
                    self._observed_gateway_state = gateway.state
                    self._monitor_stop.set()
                    return status
            if gateway.state is SafetyState.RUNNING:
                before = gateway.state
                gateway.heartbeat()
                self._append_audit(AuditOperation.HEARTBEAT, before, gateway.state)
            return status
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

    def _bounded_adapter_call(self, function: Callable[[], object]) -> tuple[bool, object | None]:
        completed = threading.Event()
        result: list[object] = []

        def invoke() -> None:
            try:
                result.append(function())
            except Exception:
                pass
            finally:
                completed.set()

        threading.Thread(target=invoke, name="agent-ros-cleanup-call", daemon=True).start()
        done = completed.wait(self._cleanup_timeout)
        return done, (result[0] if result else None)

    @staticmethod
    def _issue_adapter_stop(adapter: RobotAdapter) -> None:
        """Dispatch the safety stop without waiting for an adapter's arbitrary locks."""
        entered = threading.Event()

        def invoke() -> None:
            entered.set()
            try:
                adapter.stop()
            except Exception:
                return

        worker = threading.Thread(target=invoke, name="agent-ros-emergency-stop", daemon=True)
        worker.start()
        entered.wait(0.05)

    def _append_audit(
        self,
        operation: AuditOperation,
        before: SafetyState,
        after: SafetyState,
        *,
        operation_data: dict[str, object] | None = None,
        outcome: AuditOutcome = AuditOutcome.OK,
    ) -> None:
        try:
            self._audit_writer.append(AuditEvent(
                operation,
                before,
                after,
                outcome,
                operation_data={} if operation_data is None else operation_data,
            ))
        except (AuditIntegrityError, AuditError, OSError):
            self._quarantined = True
            self._persist_quarantine()
            if self._gateway is not None:
                self._gateway.estop()
            raise RuntimeControllerError("AUDIT_INTEGRITY_COMPROMISED") from None

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
            validate_audit_history(raw)
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
