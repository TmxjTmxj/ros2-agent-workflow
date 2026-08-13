from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import pytest
import yaml

from agent_ros.adapters._safety import _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    Observation,
    RobotAdapter,
    TwistCommand,
)
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.hospital import HospitalDeliveryAdapter
from agent_ros.adapters.base import HospitalAction
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


class RecordingEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, adapter, *, hardware_verified: bool = True, available: bool = True) -> None:
        super().__init__(hardware_verified=hardware_verified)
        self._adapter = adapter
        self._available = available

    def _preflight(self) -> bool:
        return self._available

    def _enqueue_zero_disable(self) -> None:
        self._adapter.stop_count += 1


class RecordingAdapter(RobotAdapter):
    def __init__(self, *, hardware_verified: bool = True, channel_available: bool = True) -> None:
        self.started = []
        self.stop_count = 0
        self.cancel_count = 0
        self.estop_handler = None
        self.status_error: BaseException | None = None
        self.safety_channel = RecordingEmergencyChannel(
            self,
            hardware_verified=hardware_verified,
            available=channel_available,
        )

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("mobile_base.twist",))

    def validate(self) -> None:
        return None

    def start(self, task: object, safety_token=None) -> AdapterStatus:
        return self._activate_start(safety_token, lambda: self._record_start(task))

    def _record_start(self, task: object) -> AdapterStatus:
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

    def bind_physical_estop(self, handler) -> bool:
        self.estop_handler = handler
        return True

    def _emergency_stop_channel(self):
        return self.safety_channel


def write_profiles(
    root: Path,
    *,
    required_sensors: list[str] | None = None,
    heartbeat_timeout: float = 1.0,
    stage_timeout: float = 30.0,
) -> None:
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
        "safety": {"heartbeat_timeout": heartbeat_timeout, "estop_topic": "/emergency_stop"},
        "observation_sources": ["odometry", "camera"],
    }
    task = {
        "name": "delivery",
        "robot_profile": "robot",
        "stages": [{
            "name": "destination",
            "goal": {"frame": "odom", "x": 1.0, "y": 2.0, "yaw": 0.0},
            "tolerance": 0.25,
            "timeout": stage_timeout,
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
    active = controller(tmp_path, adapter, monitor_interval=0.005)
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
    assert adapter.started[0].name == "destination"
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

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    audit_path = tmp_path / "runtime" / "audit.jsonl"
    operations = []
    deadline = time.monotonic() + 1.0
    while (not operations or operations[-1] != "estop") and time.monotonic() < deadline:
        operations = [json.loads(line)["operation"] for line in audit_path.read_text(encoding="utf-8").splitlines()]
        time.sleep(0.005)
    assert operations[-1] == "estop"


def test_unexpected_adapter_failure_is_sanitized_and_stops_motion(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = RuntimeError("secret transport path /home/operator")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)
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


def test_real_twist_adapter_executes_reviewed_task_stage_through_runtime(tmp_path):
    transport = type("Transport", (), {})()
    transport.commands = []
    transport.odometry = __import__("agent_ros.adapters.base", fromlist=["OdometrySample"]).OdometrySample(
        0.0, 0.0, 0.0, 0.0
    )
    transport.publish = transport.commands.append
    transport.read_odometry = lambda: transport.odometry
    transport.subscribe_estop = lambda handler: setattr(transport, "estop", handler)
    transport.start_waypoint = lambda stage, token=None: setattr(transport, "stage", stage)
    transport.preflight_activation = lambda: True
    transport.waypoint_status = lambda: AdapterStatus("running")
    transport.cancel_waypoint = lambda: transport.commands.extend([TwistCommand.zero()] * 3)
    transport.stop_waypoint = lambda: transport.commands.extend([TwistCommand.zero()] * 3)
    transport.safety_channel = RecordingEmergencyChannel(
        type("Counter", (), {"stop_count": 0})(),
    )
    transport.emergency_channel = lambda: transport.safety_channel
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    runtime = tmp_path / "runtime"
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda profile: TwistAdapter(profile, transport, clock=lambda: 0.0),
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
        clock=lambda: 0.0,
    )
    prepare(active)

    active.run_task("delivery")

    assert active.state is SafetyState.RUNNING
    assert transport.stage.name == "destination"
    active.stop_runtime()


def test_real_nav2_adapter_executes_reviewed_task_stage_through_runtime(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    robot_path = profiles / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text())
    robot["adapter"] = {"kind": "nav2"}
    robot["interfaces"] = {
        "navigation": {"action": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose"}
    }
    robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")

    class NavProbe:
        def probe(self):
            return GraphSnapshot(actions={"/navigate_to_pose": ("nav2_msgs/action/NavigateToPose",)})

    class Transport:
        def __init__(self):
            self.requests = []
            self.stop_count = 0
            self.safety_channel = RecordingEmergencyChannel(self)
        def preflight_activation(self): return True
        def prepare_goal(self, request): return request
        def send_goal(self, goal, token=None): self.requests.append(goal)
        def track_goal(self, future, permit): return None
        def goal_status(self): return {"state": "running"}
        def cancel_goal(self): return None
        def publish_zero(self): return None
        def subscribe_estop(self, handler): self.estop = handler
        def emergency_channel(self): return self.safety_channel

    transport = Transport()
    runtime = tmp_path / "runtime"
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=NavProbe(),
        adapter_factory=lambda profile: Nav2Adapter(profile, transport),
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
    )
    prepare(active)

    active.run_task("delivery")

    assert transport.requests[0].action_type == "nav2_msgs/action/NavigateToPose"
    assert (transport.requests[0].x, transport.requests[0].y) == (1.0, 2.0)
    active.stop_runtime()


def test_real_hospital_adapter_uses_only_fixed_start_action_through_runtime(tmp_path):
    actions = []

    def runner(action):
        actions.append(action)
        return {"state": "running"}

    active = controller(tmp_path, HospitalDeliveryAdapter(runner))
    prepare(active)

    active.run_task("delivery")

    assert actions[-1] is HospitalAction.START
    active.stop_runtime()


def test_hospital_hardware_profile_is_rejected_during_adapter_validation(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    robot_path = profiles / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text())
    robot["mode"] = "hardware"
    robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")
    runtime = tmp_path / "runtime"
    adapter = HospitalDeliveryAdapter(lambda _action: {"state": "running"})
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
    )

    discovered = active.discover_robot("robot")

    assert discovered["hardware_safety_channel"] == "unverified"

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.validate_profile("robot")

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_hardware_profile_cannot_validate_without_a_verified_nonblocking_safety_channel(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    robot_path = profiles / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text())
    robot["mode"] = "hardware"
    robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")
    adapter = RecordingAdapter(hardware_verified=False)
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
    )
    discovered = active.discover_robot("robot")

    assert discovered["hardware_safety_channel"] == "unverified"

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.validate_profile("robot")

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_physical_estop_racing_start_leaves_zero_motion_after_start_returns(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class BlockingAdapter(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait(1.0)
            return self._activate_start(safety_token, lambda: self._record_start(task))

    adapter = BlockingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    errors = []
    starter = threading.Thread(target=lambda: _capture(errors, active.run_task, "delivery"))
    starter.start()
    assert entered.wait(1.0)
    estopper = threading.Thread(target=lambda: adapter.estop_handler(True))
    estopper.start()
    release.set()
    starter.join(1.0)
    estopper.join(1.0)

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3


def test_physical_estop_does_not_wait_for_indefinitely_blocked_start(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class HungStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait()
            return self._activate_start(safety_token, lambda: AdapterStatus("running"))

    adapter = HungStart()
    active = controller(tmp_path, adapter)
    prepare(active)
    starter = threading.Thread(target=lambda: _capture([], active.run_task, "delivery"), daemon=True)
    starter.start()
    assert entered.wait(1.0)

    began = time.monotonic()
    adapter.estop_handler(True)

    assert time.monotonic() - began < 0.2
    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    release.set()


def test_estop_between_start_reservation_and_transport_activation_rejects_start(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class ReservedStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait(1.0)
            return self._activate_start(safety_token, lambda: self._record_start(task))

    adapter = ReservedStart()
    active = controller(tmp_path, adapter)
    prepare(active)
    errors = []
    worker = threading.Thread(target=lambda: _capture(errors, active.run_task, "delivery"))
    worker.start()
    assert entered.wait(1.0)

    adapter.estop_handler(True)
    release.set()
    worker.join(1.0)

    assert adapter.started == []
    assert active.state is SafetyState.ESTOPPED
    assert any(isinstance(error, RuntimeControllerError) for error in errors)


def test_each_stage_activation_uses_a_fresh_internal_safety_reservation(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    task_path = profiles / "tasks" / "delivery.yaml"
    task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    task["stages"].append({
        "name": "return",
        "goal": {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        "tolerance": 0.25,
        "timeout": 30.0,
    })
    task_path.write_text(yaml.safe_dump(task), encoding="utf-8")

    class TwoStageAdapter(RecordingAdapter):
        def __init__(self):
            super().__init__()
            self.status_calls = 0

        def start(self, task, safety_token=None):
            return super().start(task, safety_token)

        def status(self):
            self.status_calls += 1
            return AdapterStatus("succeeded" if self.status_calls == 1 else "running")

    adapter = TwoStageAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.001)
    prepare(active)
    active.run_task("delivery")

    deadline = time.monotonic() + 1.0
    while len(adapter.started) < 2 and time.monotonic() < deadline:
        time.sleep(0.005)

    assert [item.name for item in adapter.started] == ["destination", "return"]
    assert active.state is SafetyState.RUNNING
    active.stop_runtime()


def test_start_transition_is_audited_before_adapter_activation_and_failure_is_continuous(tmp_path):
    class FailedStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            raise AdapterError("STALE_FEEDBACK")

    adapter = FailedStart()
    active = controller(tmp_path, adapter)
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="STALE_FEEDBACK"):
        active.run_task("delivery")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [item["operation"] for item in records[-2:]] == ["start_task", "estop"]
    assert records[-2]["state"] == {"from": "ARMED", "to": "RUNNING"}
    assert records[-1]["state"] == {"from": "RUNNING", "to": "ESTOPPED"}


def test_estop_between_gateway_start_transition_and_durable_append_keeps_audit_continuous(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    runtime = tmp_path / "runtime"

    class BlockingStartWriter(AuditWriter):
        def append(self, event):
            if event.operation.value == "start_task":
                entered.set()
                release.wait(1.0)
            return super().append(event)

    adapter = RecordingAdapter()
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=BlockingStartWriter(runtime / "audit.jsonl"),
    )
    prepare(active)
    start_errors = []
    starter = threading.Thread(target=lambda: _capture(start_errors, active.run_task, "delivery"))
    starter.start()
    assert entered.wait(1.0)

    estopper = threading.Thread(target=lambda: adapter.estop_handler(True))
    estopper.start()
    deadline = time.monotonic() + 1.0
    while active.state is not SafetyState.ESTOPPED and time.monotonic() < deadline:
        time.sleep(0.001)
    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    release.set()
    starter.join(1.0)
    estopper.join(1.0)

    records = [json.loads(line) for line in (runtime / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == ["start_task", "estop"]
    assert records[-2]["state"] == {"from": "ARMED", "to": "RUNNING"}
    assert records[-1]["state"] == {"from": "RUNNING", "to": "ESTOPPED"}

    restarted = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence-2",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    assert restarted.discover_robot("robot")["state"] == "DISCOVERED"
    restarted.stop_runtime()


def test_audit_coordinator_orders_cancel_before_concurrent_estop_by_transition_sequence(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    profiles = tmp_path / "profiles"
    runtime = tmp_path / "runtime"
    write_profiles(profiles)

    class PausedCancelAuditController(RuntimeController):
        def _append_transition(self, operation, transition, **kwargs):
            if operation.value == "cancel":
                entered.set()
                release.wait(1.0)
            return super()._append_transition(operation, transition, **kwargs)

    adapter = RecordingAdapter()
    active = PausedCancelAuditController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
    )
    prepare(active)
    active.run_task("delivery")
    canceler = threading.Thread(target=active.cancel_task)
    canceler.start()
    assert entered.wait(1.0)

    estopper = threading.Thread(target=lambda: adapter.estop_handler(True))
    estopper.start()
    deadline = time.monotonic() + 1.0
    while active.state is not SafetyState.ESTOPPED and time.monotonic() < deadline:
        time.sleep(0.001)
    assert active.state is SafetyState.ESTOPPED
    release.set()
    canceler.join(1.0)
    estopper.join(1.0)
    assert not canceler.is_alive(), active._pending_audit
    assert not estopper.is_alive(), active._pending_audit

    records = [json.loads(line) for line in (runtime / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == ["cancel", "estop"]
    assert records[-2]["state"] == {"from": "RUNNING", "to": "STOPPED"}
    assert records[-1]["state"] == {"from": "STOPPED", "to": "ESTOPPED"}


def test_initial_heartbeat_expiry_audits_fault_before_estop_without_stalling_sequence(tmp_path):
    now = [0.0]

    class ExpiringStart(RecordingAdapter):
        def start(self, task, activation_permit=None):
            result = super().start(task, activation_permit)
            now[0] = 2.0
            return result

    adapter = ExpiringStart()
    active = controller(tmp_path, adapter, clock=lambda: now[0])
    prepare(active)
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, active.run_task, "delivery"),
        daemon=True,
    )
    worker.start()
    worker.join(0.5)

    assert not worker.is_alive(), active._pending_audit
    assert any(isinstance(error, RuntimeControllerError) for error in errors)
    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == ["heartbeat", "estop"]
    assert records[-2]["outcome"] == "faulted"
    assert records[-2]["state"] == {"from": "RUNNING", "to": "FAULTED"}
    assert records[-1]["state"] == {"from": "FAULTED", "to": "ESTOPPED"}


def test_stop_runtime_reports_cleanup_failure_when_monitor_status_never_returns(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class HungStatus(RecordingAdapter):
        def status(self):
            entered.set()
            release.wait()
            return AdapterStatus("running")

    adapter = HungStatus()
    active = controller(tmp_path, adapter, monitor_interval=0.001, cleanup_timeout=0.02)
    prepare(active)
    active.run_task("delivery")
    assert entered.wait(1.0)

    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()

    assert active.state is not SafetyState.RUNNING
    release.set()


def test_stop_runtime_uses_bounded_tracked_task_cleanup_after_nonblocking_safety_enqueue(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class HungStop(RecordingAdapter):
        def stop(self):
            entered.set()
            release.wait()

    adapter = HungStop()
    active = controller(tmp_path, adapter, cleanup_timeout=0.02)
    prepare(active)
    active.run_task("delivery")

    began = time.monotonic()
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()

    assert time.monotonic() - began < 0.2
    assert entered.is_set()
    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    release.set()
    assert active.stop_runtime() == {"state": "ESTOPPED"}


def test_stop_runtime_applies_cleanup_timeout_to_owned_safety_watchdog_join(tmp_path):
    class SlowJoinThread:
        def is_alive(self):
            return True

        def join(self, timeout=None):
            time.sleep(1.0 if timeout is None else timeout)

    active = controller(tmp_path, RecordingAdapter(), cleanup_timeout=0.02)
    prepare(active)
    active._gateway.supervisor._thread = SlowJoinThread()

    began = time.monotonic()
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()

    assert time.monotonic() - began < 0.2


def test_physical_estop_returns_promptly_while_monitor_status_is_blocked(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class HungStatus(RecordingAdapter):
        def status(self):
            entered.set()
            release.wait()
            return AdapterStatus("running")

    adapter = HungStatus()
    active = controller(tmp_path, adapter, monitor_interval=0.001, cleanup_timeout=0.02)
    prepare(active)
    active.run_task("delivery")
    assert entered.wait(1.0)

    began = time.monotonic()
    adapter.estop_handler(True)

    assert time.monotonic() - began < 0.2
    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3
    release.set()
    active.stop_runtime()


def test_monitor_thread_start_failure_latches_stop_and_never_leaks_running(tmp_path):
    adapter = RecordingAdapter()
    class FailedThread:
        def start(self): raise RuntimeError("raw")
    active = controller(tmp_path, adapter, monitor_thread_factory=lambda **_kwargs: FailedThread())
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.run_task("delivery")

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3


def _capture(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_owned_monitor_keeps_healthy_runtime_alive_without_mcp_polling(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")

    time.sleep(0.08)

    assert active.state is SafetyState.RUNNING
    active.stop_runtime()


def test_owned_monitor_latches_stale_adapter_without_mcp_polling(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = AdapterError("STALE_FEEDBACK")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


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


def test_evidence_atomic_write_never_follows_a_predictable_temp_symlink(tmp_path):
    root = tmp_path / "evidence"
    root.mkdir()
    outside = tmp_path / "outside"
    outside.write_text("do-not-touch", encoding="utf-8")
    (root / ".report.json.tmp").symlink_to(outside)
    store = EvidenceStore(root)

    reference = store.write_json("report", {"ok": True})

    assert json.loads(store.read(reference)) == {"ok": True}
    assert outside.read_text(encoding="utf-8") == "do-not-touch"


def test_evidence_constructor_maps_filesystem_errors_without_leaking_paths(tmp_path):
    blocker = tmp_path / "not-a-directory"
    blocker.write_text("x", encoding="utf-8")

    with pytest.raises(EvidenceError) as captured:
        EvidenceStore(blocker / "evidence")

    assert str(captured.value) == "EVIDENCE_INVALID"


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


def test_restart_quarantines_schema_invalid_but_parseable_audit_record(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    (runtime / "audit.jsonl").write_text(
        json.dumps({
            "operation": "start_task",
            "state": {"from": "NEW", "to": "RUNNING"},
            "outcome": "ok",
            "wall_time": 1.0,
            "monotonic_time": 1.0,
            "operation_data": {"payload": "raw"},
            "endpoint_gids": [],
        }) + "\n",
        encoding="utf-8",
    )
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        active.discover_robot("robot")
    assert active.stop_runtime() == {"state": "NEW"}


@pytest.mark.parametrize("mutation", ["oversized", "discontinuous", "bad-first"])
def test_restart_quarantines_bounded_but_impossible_audit_history(tmp_path, mutation):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    writer = AuditWriter(runtime / "audit.jsonl", wall_clock=lambda: 1.0, monotonic_clock=lambda: 1.0)
    from agent_ros.runtime.audit import AuditEvent, AuditOperation, AuditOutcome
    writer.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))
    if mutation == "oversized":
        (runtime / "audit.jsonl").write_bytes(b"{" + b"x" * 5000 + b"}\n")
    else:
        record = json.loads((runtime / "audit.jsonl").read_text())
        if mutation == "discontinuous":
            second = dict(record)
            second["operation"] = "validate"
            second["state"] = {"from": "DISCOVERED", "to": "ARMED"}
            (runtime / "audit.jsonl").write_text(json.dumps(record) + "\n" + json.dumps(record) + "\n")
        else:
            record["operation"] = "validate"
            record["state"] = {"from": "DISCOVERED", "to": "ARMED"}
            (runtime / "audit.jsonl").write_text(json.dumps(record) + "\n")
    active = RuntimeController(
        profiles_root=profiles, evidence_dir=tmp_path / "evidence", runtime_dir=runtime,
        graph_probe=Probe(), adapter_factory=lambda _profile: RecordingAdapter(),
    )

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        active.discover_robot("robot")


def test_clean_stopped_session_allows_new_controller_session_from_new(tmp_path):
    adapter = RecordingAdapter()
    first = controller(tmp_path, adapter)
    prepare(first)
    first.run_task("delivery")
    first.cancel_task()
    first.stop_runtime()

    second = RuntimeController(
        profiles_root=tmp_path / "profiles",
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    result = second.discover_robot("robot")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert result["state"] == "DISCOVERED"
    assert records[-1]["session_id"] != records[0]["session_id"]


def test_restart_quarantines_interleaved_or_replayed_audit_session(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    path = tmp_path / "runtime" / "audit.jsonl"
    records = path.read_text().splitlines()
    path.write_text(records[0] + "\n" + records[1] + "\n" + records[0] + "\n", encoding="utf-8")

    restarted = RuntimeController(
        profiles_root=tmp_path / "profiles", evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime", graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        restarted.discover_robot("robot")


def test_physical_binding_failure_is_estopped_and_audited_once(tmp_path):
    class BrokenBinding(RecordingAdapter):
        def bind_physical_estop(self, handler):
            raise RuntimeError("raw device path")

    active = controller(tmp_path, BrokenBinding())

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.discover_robot("robot")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [item["operation"] for item in records] == ["estop"]
    active.stop_runtime()


def test_quarantined_runtime_still_allows_owned_cleanup_and_join(tmp_path):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")
    (tmp_path / "runtime" / "audit.quarantine").write_text("AUDIT_INTEGRITY_COMPROMISED\n")

    result = active.stop_runtime()

    assert result["state"] == "ESTOPPED"
    assert adapter.stop_count >= 3


def test_owned_monitor_enforces_stage_timeout_without_mcp_polling(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles, stage_timeout=0.02)
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_watchdog_fault_transition_is_audited_once_by_owned_monitor(tmp_path):
    profiles = tmp_path / "profiles"
    write_profiles(profiles, heartbeat_timeout=0.02)
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, monitor_interval=0.08)
    prepare(active)
    active.run_task("delivery")
    time.sleep(0.16)

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    faulted = [item for item in records if item["operation"] == "heartbeat" and item["outcome"] == "faulted"]
    assert active.state is SafetyState.FAULTED
    assert len(faulted) == 1
    active.stop_runtime()
