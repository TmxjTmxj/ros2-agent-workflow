"""Generic runtime orchestration across profiles, safety, adapters, audit, and evidence."""

from __future__ import annotations

import json
import os
import time
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
        self._evidence = EvidenceStore(evidence_dir)
        self._profile: RobotProfile | None = None
        self._adapter: RobotAdapter | None = None
        self._gateway: SafetyGateway | None = None
        self._task: TaskProfile | None = None
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
            stop_callback=adapter.stop,
            clock=self._clock,
            boot_id=self._boot_id,
        )
        self._profile = profile
        self._adapter = adapter
        self._gateway = gateway
        self._report = report
        try:
            adapter.bind_physical_estop(self._physical_estop)
        except Exception:
            gateway.estop()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        before = gateway.state
        try:
            gateway.discover(report)
        except SafetyError as exc:
            raise self._safety_error(exc) from None
        self._append_audit(AuditOperation.DISCOVER, before, gateway.state)
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
        self._ensure_available()
        gateway, adapter, profile = self._active()
        if gateway.state is not SafetyState.ARMED:
            raise RuntimeControllerError("UNSAFE_STATE")
        task = self._load_compatible_task(task_name, profile)
        if dry_run:
            return {"dry_run": True, "profile": profile.name, "task": task.name}
        before = gateway.state
        try:
            gateway.start_task()
            adapter.start(HospitalAction.START if isinstance(adapter, HospitalDeliveryAdapter) else task)
            gateway.heartbeat()
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._task = task
        self._append_audit(
            AuditOperation.START_TASK,
            before,
            gateway.state,
            operation_data={"task": task.name},
        )
        return {"state": gateway.state.value, "task": task.name}

    def task_status(self) -> dict[str, object]:
        self._ensure_available()
        gateway, adapter, _profile = self._active()
        if gateway.state is not SafetyState.RUNNING:
            return {"state": gateway.state.value, "task": None if self._task is None else self._task.name}
        try:
            status = adapter.status()
            before = gateway.state
            gateway.heartbeat()
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._append_audit(AuditOperation.HEARTBEAT, before, gateway.state)
        return {
            "state": gateway.state.value,
            "task": None if self._task is None else self._task.name,
            "adapter_state": status.state,
            "code": status.code,
        }

    def cancel_task(self) -> dict[str, object]:
        self._ensure_available()
        gateway, adapter, _profile = self._active()
        if gateway.state is not SafetyState.RUNNING:
            raise RuntimeControllerError("UNSAFE_STATE")
        before = gateway.state
        try:
            status = adapter.cancel()
            gateway.cancel()
        except AdapterError as exc:
            self._latch_adapter_fault()
            raise self._adapter_error(exc) from None
        except SafetyError as exc:
            self._latch_adapter_fault()
            raise self._safety_error(exc) from None
        except Exception:
            self._latch_adapter_fault()
            raise RuntimeControllerError("UNSAFE_STATE") from None
        self._append_audit(AuditOperation.CANCEL, before, gateway.state)
        return {"state": gateway.state.value, "adapter_state": status.state}

    def emergency_stop(self) -> dict[str, object]:
        self._ensure_available()
        gateway, _adapter, _profile = self._active()
        before = gateway.state
        gateway.estop()
        if gateway.state is not before:
            self._append_audit(AuditOperation.ESTOP, before, gateway.state)
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
        self._ensure_available()
        if self._gateway is None:
            return {"state": SafetyState.NEW.value}
        gateway = self._gateway
        if gateway.state is SafetyState.RUNNING:
            self.cancel_task()
        gateway.close()
        return {"state": gateway.state.value}

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
        if asserted is not True or self._gateway is None:
            return
        before = self._gateway.state
        self._gateway.observe_physical_estop(True)
        if self._gateway.state is not before:
            try:
                self._append_audit(AuditOperation.ESTOP, before, self._gateway.state)
            except RuntimeControllerError:
                # Quarantine and repeated stop have already happened; a callback
                # must not leak failures into an rclpy executor.
                return

    def _latch_adapter_fault(self) -> None:
        if self._gateway is not None:
            before = self._gateway.state
            self._gateway.estop()
            if self._gateway.state is not before:
                self._append_audit(AuditOperation.ESTOP, before, self._gateway.state)

    def _append_audit(
        self,
        operation: AuditOperation,
        before: SafetyState,
        after: SafetyState,
        *,
        operation_data: dict[str, object] | None = None,
    ) -> None:
        try:
            self._audit_writer.append(AuditEvent(
                operation,
                before,
                after,
                AuditOutcome.OK,
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
            if raw and not raw.endswith(b"\n"):
                return False
            for line in raw.splitlines():
                record = json.loads(line)
                if not isinstance(record, dict) or not {
                    "operation", "state", "outcome", "wall_time", "monotonic_time"
                }.issubset(record):
                    return False
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
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
