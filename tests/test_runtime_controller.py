from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    Observation,
    RobotAdapter,
)
from agent_ros.discovery.models import GraphSnapshot
from agent_ros.runtime.audit import AuditIntegrityError, AuditWriter
from agent_ros.runtime.controller import RuntimeController, RuntimeControllerError
from agent_ros.runtime.evidence import EvidenceError, EvidenceStore
from agent_ros.safety.state import SafetyState


class Probe:
    def probe(self) -> GraphSnapshot:
        return GraphSnapshot(
            topics={
                "/cmd_vel": ("geometry_msgs/msg/Twist",),
                "/odom": ("nav_msgs/msg/Odometry",),
            }
        )


class RecordingAdapter(RobotAdapter):
    def __init__(self) -> None:
        self.started = []
        self.stop_count = 0
        self.cancel_count = 0
        self.estop_handler = None
        self.status_error: BaseException | None = None

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("mobile_base.twist",))

    def validate(self) -> None:
        return None

    def start(self, task: object) -> AdapterStatus:
        self.started.append(task)
        return AdapterStatus("running")

    def status(self) -> AdapterStatus:
        if self.status_error is not None:
            raise self.status_error
        return AdapterStatus("running")

    def cancel(self) -> AdapterStatus:
        self.cancel_count += 1
        return AdapterStatus("cancelled")

    def stop(self) -> None:
        self.stop_count += 1

    def observe(self, source: str) -> Observation:
        return Observation(source, 1.0, {"ok": True})

    def bind_physical_estop(self, handler) -> None:
        self.estop_handler = handler


def write_profiles(root: Path, *, required_sensors: list[str] | None = None) -> None:
    (root / "robots").mkdir(parents=True)
    (root / "tasks").mkdir(parents=True)
    robot = {
        "name": "robot",
        "mode": "simulation",
        "namespace": "/robot",
        "frames": {"base": "base_link", "odom": "odom"},
        "adapter": {"kind": "twist"},
        "interfaces": {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        },
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
        "observation_sources": ["odometry", "camera"],
    }
    task = {
        "name": "delivery",
        "robot_profile": "robot",
        "stages": [{
            "name": "destination",
            "goal": {"frame": "odom", "x": 1.0, "y": 2.0, "yaw": 0.0},
            "tolerance": 0.25,
            "timeout": 30.0,
        }],
        "required_sensors": required_sensors if required_sensors is not None else ["odometry"],
        "evidence": {"sources": ["camera"]},
        "recovery_policy": "cancel_and_stop",
    }
    (root / "robots" / "robot.yaml").write_text(yaml.safe_dump(robot), encoding="utf-8")
    (root / "tasks" / "delivery.yaml").write_text(yaml.safe_dump(task), encoding="utf-8")


def controller(tmp_path: Path, adapter: RecordingAdapter, **kwargs) -> RuntimeController:
    profiles = tmp_path / "profiles"
    if not profiles.exists():
        write_profiles(profiles)
    runtime = tmp_path / "runtime"
    return RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
        **kwargs,
    )


def prepare(active: RuntimeController) -> None:
    active.discover_robot("robot")
    active.validate_profile("robot")


def test_runtime_refuses_motion_before_profile_is_discovered_validated_and_armed(tmp_path):
    active = controller(tmp_path, RecordingAdapter())

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.run_task("delivery")


def test_dry_run_checks_compatibility_without_starting_motion(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)

    result = active.run_task("delivery", dry_run=True)

    assert result == {"dry_run": True, "profile": "robot", "task": "delivery"}
    assert adapter.started == []
    assert active.state is SafetyState.ARMED


def test_safe_execution_starts_typed_task_and_refreshes_heartbeat_during_status(tmp_path):
    now = [0.0]
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, clock=lambda: now[0])
    prepare(active)

    assert active.run_task("delivery") == {"state": "RUNNING", "task": "delivery"}
    now[0] = 0.75
    assert active.task_status()["adapter_state"] == "running"
    now[0] = 1.5
    assert active.task_status()["adapter_state"] == "running"
    assert len(adapter.started) == 1
    assert adapter.started[0].name == "delivery"
    active.stop_runtime()


def test_task_requires_profile_sensor_compatibility_before_gateway_start(tmp_path):
    write_profiles(tmp_path / "profiles", required_sensors=["medical_lidar"])
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.run_task("delivery")

    assert active.state is SafetyState.ARMED
    assert adapter.started == []


def test_adapter_fault_is_propagated_as_stable_code_and_latches_estop(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = AdapterError("STALE_FEEDBACK")

    with pytest.raises(RuntimeControllerError, match="STALE_FEEDBACK"):
        active.task_status()

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    operations = [
        json.loads(line)["operation"]
        for line in (tmp_path / "runtime" / "audit.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert operations[-1] == "estop"


def test_unexpected_adapter_failure_is_sanitized_and_stops_motion(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = RuntimeError("secret transport path /home/operator")

    with pytest.raises(RuntimeControllerError) as captured:
        active.task_status()

    assert str(captured.value) == "UNSAFE_STATE"
    assert active.state is SafetyState.ESTOPPED


def test_physical_estop_monitor_is_bound_directly_to_gateway_latch(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")

    assert adapter.estop_handler is not None
    adapter.estop_handler(True)

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3


def test_evidence_store_confines_ids_symlinks_and_resolved_files_to_its_root(tmp_path):
    evidence_root = tmp_path / "evidence"
    evidence_root.mkdir()
    (evidence_root / "report.json").write_text('{"result":"pass"}\n', encoding="utf-8")
    outside = tmp_path / "secret.json"
    outside.write_text('{"secret":true}', encoding="utf-8")
    (evidence_root / "escape.json").symlink_to(outside)
    store = EvidenceStore(evidence_root)

    reference = store.get("report")
    assert reference.report_id == "report"
    assert reference.relative_path == "report.json"
    assert reference.media_type == "application/json"
    assert json.loads(store.read(reference)) == {"result": "pass"}

    for unsafe in ("../secret", "/tmp/secret", "escape"):
        with pytest.raises(EvidenceError, match="EVIDENCE_INVALID"):
            store.get(unsafe)


class CompromisedWriter:
    def append(self, _event) -> None:
        raise AuditIntegrityError()


def test_audit_integrity_compromise_persists_quarantine_across_controller_restart(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    runtime = tmp_path / "runtime"
    adapter = RecordingAdapter()
    compromised = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=CompromisedWriter(),
    )

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        compromised.discover_robot("robot")
    assert (runtime / "audit.quarantine").read_text(encoding="ascii") == "AUDIT_INTEGRITY_COMPROMISED\n"

    restarted = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
    )
    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        restarted.discover_robot("robot")
