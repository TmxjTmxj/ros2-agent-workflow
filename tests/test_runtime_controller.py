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
    HospitalAction,
    Observation,
    RobotAdapter,
    TwistCommand,
)
from agent_ros.adapters.hospital import (
    HospitalCaseAdapter,
    HospitalDeliveryAdapter,
    HospitalLifecycleClient,
    HospitalSimulationRuntime,
)
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.discovery.models import Endpoint, GraphSnapshot
from agent_ros.runtime.audit import AuditIntegrityError, AuditOperation, AuditWriter
from agent_ros.runtime.controller import RuntimeController, RuntimeControllerError
from agent_ros.runtime.evidence import EvidenceError, EvidenceStore
from agent_ros.safety.gateway import SafetyStopAttempt, SafetyTransition
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.state import SafetyState


class Probe:
    def probe(self) -> GraphSnapshot:
        return GraphSnapshot(
            topics={
                "/cmd_vel": ("geometry_msgs/msg/Twist",),
                "/odom": ("nav_msgs/msg/Odometry",),
            }
        )


class EmptyProbe:
    def probe(self) -> GraphSnapshot:
        return GraphSnapshot()


class ConflictingProbe:
    def probe(self) -> GraphSnapshot:
        return GraphSnapshot(
            topics={
                "/cmd_vel": ("geometry_msgs/msg/Twist",),
                "/odom": ("nav_msgs/msg/Odometry",),
            },
            topic_endpoints={
                "/cmd_vel": (
                    Endpoint("controller_a", "aa", "publisher"),
                    Endpoint("controller_b", "bb", "publisher"),
                )
            },
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


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.001)
    return predicate()


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
        "stages": [
            {
                "name": "destination",
                "goal": {"frame": "odom", "x": 1.0, "y": 2.0, "yaw": 0.0},
                "tolerance": 0.25,
                "timeout": stage_timeout,
            }
        ],
        "required_sensors": required_sensors if required_sensors is not None else ["odometry"],
        "evidence": {"sources": ["camera"]},
        "recovery_policy": "cancel_and_stop",
    }
    (root / "robots" / "robot.yaml").write_text(yaml.safe_dump(robot), encoding="utf-8")
    (root / "tasks" / "delivery.yaml").write_text(yaml.safe_dump(task), encoding="utf-8")


def write_hospital_profiles(root: Path) -> None:
    write_profiles(root, heartbeat_timeout=301.0, stage_timeout=60.0)
    robot_path = root / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text(encoding="utf-8"))
    robot["name"] = "hospital-amr"
    robot["namespace"] = "/hospital_amr"
    robot["adapter"] = {"kind": "hospital_delivery"}
    robot["limits"] = {
        "max_linear_velocity": 0.22,
        "max_angular_velocity": 1.0,
        "max_linear_acceleration": 0.5,
        "max_angular_acceleration": 1.0,
    }
    robot["observation_sources"] = ["odometry", "camera", "scan"]
    hospital_robot_path = root / "robots" / "hospital-amr.yaml"
    hospital_robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")
    robot_path.unlink()
    task_path = root / "tasks" / "delivery.yaml"
    task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    task["name"] = "hospital-delivery"
    task["robot_profile"] = "hospital-amr"
    task["stages"] = [
        {
            "name": name,
            "goal": {"frame": "world", "x": float(index), "y": 0.0, "yaw": 0.0},
            "tolerance": 0.5,
            "timeout": 60.0,
        }
        for index, name in enumerate(("pharmacy", "ward-2", "laboratory"), start=1)
    ]
    task["evidence"] = {"sources": ["odometry"]}
    hospital_task_path = root / "tasks" / "hospital-delivery.yaml"
    hospital_task_path.write_text(yaml.safe_dump(task), encoding="utf-8")
    task_path.unlink()


class SimTimeHospitalAdapter(HospitalCaseAdapter):
    """Exact production adapter type with a deterministic simulated status source."""

    def __init__(self) -> None:
        super().__init__(HospitalLifecycleClient())
        self.values = {"elapsed": 0.0, "stage_results": []}
        self.test_channel = RecordingEmergencyChannel(self)
        self.stop_count = 0

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("hospital.delivery",))

    def validate(self) -> None:
        return None

    def start(self, task: object, safety_token=None) -> AdapterStatus:
        assert task is HospitalAction.START
        return self._activate_start(
            safety_token,
            lambda: AdapterStatus("running", values=self.values),
        )

    def status(self) -> AdapterStatus:
        return AdapterStatus("running", values=self.values)

    def cancel(self) -> AdapterStatus:
        return AdapterStatus("cancelled", values=self.values)

    def stop(self) -> None:
        self.stop_count += 1

    def _emergency_stop_channel(self):
        return self.test_channel


def controller(
    tmp_path: Path,
    adapter: RecordingAdapter,
    *,
    owner=None,
    **kwargs,
) -> RuntimeController:
    profiles = tmp_path / "profiles"
    if not profiles.exists():
        write_profiles(profiles)
    runtime = tmp_path / "runtime"
    runtime_controller = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
        **kwargs,
    )
    return runtime_controller if owner is None else owner(runtime_controller)


def prepare(active: RuntimeController) -> None:
    active.discover_robot("robot")
    active.validate_profile("robot")


def test_failed_discovery_preserves_factory_cleanup_code_and_controller_ownership(
    tmp_path,
):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)

    class FailingOwnedFactory:
        def __init__(self) -> None:
            self.create_calls = 0
            self.close_timeouts = []
            self.cleanup_allowed = False

        def __call__(self, _profile):
            self.create_calls += 1
            raise AdapterError("CLEANUP_FAILED")

        def close(self, timeout):
            self.close_timeouts.append(timeout)
            return self.cleanup_allowed

    factory = FailingOwnedFactory()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=Probe(),
        adapter_factory=factory,
        cleanup_timeout=0.02,
    )
    try:
        with pytest.raises(RuntimeControllerError) as discovery_failure:
            active.discover_robot("robot")
        assert discovery_failure.value.code == "CLEANUP_FAILED"
        assert active._gateway is None
        assert active._adapter is None

        with pytest.raises(RuntimeControllerError) as first_cleanup:
            active.stop_runtime()
        assert first_cleanup.value.code == "CLEANUP_FAILED"
        assert factory.create_calls == 1
        assert len(factory.close_timeouts) == 1
        assert 0.0 <= factory.close_timeouts[0] <= 0.02

        factory.cleanup_allowed = True
        assert active.stop_runtime() == {"state": "NEW"}
        assert len(factory.close_timeouts) == 2
        assert 0.0 <= factory.close_timeouts[1] <= 0.02
    finally:
        factory.cleanup_allowed = True
        active.stop_runtime()


def test_register_transition_rejects_a_forged_equal_gateway_receipt(tmp_path, runtime_owner):
    active = controller(tmp_path, RecordingAdapter(), owner=runtime_owner)
    active.discover_robot("robot")
    transition = active._gateway.latest_transition
    assert transition is not None
    forged = SafetyTransition(
        transition.sequence,
        transition.state_before,
        transition.state_after,
        transition.stop_result,
    )

    assert forged == transition
    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        active._register_transition(AuditOperation.DISCOVER, forged)

    assert active._pending_audit == {}


def test_runtime_refuses_motion_before_profile_is_discovered_validated_and_armed(tmp_path, runtime_owner):
    active = controller(tmp_path, RecordingAdapter(), owner=runtime_owner)

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.run_task("delivery")


def test_dry_run_checks_compatibility_without_starting_motion(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
    prepare(active)

    result = active.run_task("delivery", dry_run=True)

    assert result == {"dry_run": True, "profile": "robot", "task": "delivery"}
    assert adapter.started == []
    assert active.state is SafetyState.ARMED


def test_safe_execution_starts_typed_task_and_refreshes_heartbeat_during_status(tmp_path, runtime_owner):
    now = [0.0]
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, clock=lambda: now[0])
    prepare(active)

    assert active.run_task("delivery") == {"state": "RUNNING", "task": "delivery"}
    now[0] = 0.75
    assert active.task_status()["adapter_state"] == "running"
    now[0] = 1.5
    assert active.task_status()["adapter_state"] == "running"
    assert len(adapter.started) == 1
    assert adapter.started[0].name == "destination"
    active.stop_runtime()


def test_task_requires_profile_sensor_compatibility_before_gateway_start(tmp_path, runtime_owner):
    write_profiles(tmp_path / "profiles", required_sensors=["medical_lidar"])
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.run_task("delivery")

    assert active.state is SafetyState.ARMED
    assert adapter.started == []


def test_adapter_fault_is_propagated_as_stable_code_and_latches_estop(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = AdapterError("STALE_FEEDBACK")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)

    assert active.state is SafetyState.ESTOPPED
    assert wait_until(lambda: adapter.stop_count >= 3)
    assert adapter.stop_count >= 3
    audit_path = tmp_path / "runtime" / "audit.jsonl"
    operations = []
    deadline = time.monotonic() + 1.0
    while (not operations or operations[-1] != "estop") and time.monotonic() < deadline:
        operations = [json.loads(line)["operation"] for line in audit_path.read_text(encoding="utf-8").splitlines()]
        time.sleep(0.005)
    assert operations[-1] == "estop"


def test_unexpected_adapter_failure_is_sanitized_and_stops_motion(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")
    adapter.status_error = RuntimeError("secret transport path /home/operator")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)
    assert active.state is SafetyState.ESTOPPED


def test_physical_estop_monitor_is_bound_directly_to_gateway_latch(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)
    active.run_task("delivery")

    assert adapter.estop_handler is not None
    adapter.estop_handler(True)

    assert active.state is SafetyState.ESTOPPED
    assert wait_until(lambda: adapter.stop_count >= 3)
    assert adapter.stop_count >= 3


@pytest.mark.parametrize("winner", ["a", "b"])
def test_concurrent_api_emergency_calls_use_only_their_own_stop_result(
    tmp_path,
    runtime_owner,
    winner,
):
    degraded = EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    successful = EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
    active = controller(tmp_path, RecordingAdapter(), owner=runtime_owner)
    prepare(active)
    active.run_task("delivery")
    gateway = active._gateway
    gateway._stop_callback = lambda _timeout: degraded if threading.current_thread().name == "attempt-a" else successful
    paused = "b" if winner == "a" else "a"
    paused_ready = threading.Event()
    release_paused = threading.Event()
    original_stop = gateway._stop_repeatedly

    def ordered_stop(timeout):
        result = original_stop(timeout)
        if threading.current_thread().name == f"attempt-{paused}":
            paused_ready.set()
            assert release_paused.wait(1.0)
        return result

    gateway._stop_repeatedly = ordered_stop
    responses = {}
    errors = {}

    def invoke(label):
        try:
            responses[label] = active.emergency_stop()
        except BaseException as exc:
            errors[label] = exc

    paused_thread = threading.Thread(target=invoke, args=(paused,), name=f"attempt-{paused}")
    winner_thread = threading.Thread(target=invoke, args=(winner,), name=f"attempt-{winner}")
    paused_thread.start()
    assert paused_ready.wait(1.0)
    winner_thread.start()
    winner_thread.join(1.0)
    assert not winner_thread.is_alive()
    release_paused.set()
    paused_thread.join(1.0)

    assert not paused_thread.is_alive()
    assert responses == {"b": {"state": "ESTOPPED"}}
    assert set(errors) == {"a"}
    assert isinstance(errors["a"], RuntimeControllerError)
    assert errors["a"].code == "UNSAFE_STATE"


def test_real_twist_adapter_executes_reviewed_task_stage_through_runtime(tmp_path, runtime_owner):
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
    runtime_owner(active)
    prepare(active)

    active.run_task("delivery")

    assert active.state is SafetyState.RUNNING
    assert transport.stage.name == "destination"
    active.stop_runtime()


def test_real_nav2_adapter_executes_reviewed_task_stage_through_runtime(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    robot_path = profiles / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text())
    robot["adapter"] = {"kind": "nav2"}
    robot["interfaces"] = {"navigation": {"action": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose"}}
    robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")

    class NavProbe:
        def probe(self):
            return GraphSnapshot(actions={"/navigate_to_pose": ("nav2_msgs/action/NavigateToPose",)})

    class Transport:
        def __init__(self):
            self.requests = []
            self.stop_count = 0
            self.safety_channel = RecordingEmergencyChannel(self)

        def preflight_activation(self):
            return True

        def prepare_goal(self, request):
            return request

        def send_goal(self, goal, token=None):
            self.requests.append(goal)

        def track_goal(self, future, permit):
            return None

        def goal_status(self):
            return {"state": "running"}

        def cancel_goal(self):
            return None

        def publish_zero(self):
            return None

        def subscribe_estop(self, handler):
            self.estop = handler

        def emergency_channel(self):
            return self.safety_channel

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
    runtime_owner(active)
    prepare(active)

    active.run_task("delivery")

    assert transport.requests[0].action_type == "nav2_msgs/action/NavigateToPose"
    assert (transport.requests[0].x, transport.requests[0].y) == (1.0, 2.0)
    active.stop_runtime()


def test_real_hospital_adapter_uses_only_fixed_start_action_through_runtime(tmp_path, runtime_owner):
    runtime = HospitalSimulationRuntime()
    active = controller(tmp_path, HospitalDeliveryAdapter(runtime), owner=runtime_owner)
    prepare(active)

    active.run_task("delivery")

    assert runtime.commands[-1] is HospitalAction.START
    active.stop_runtime()


def test_hospital_hardware_profile_is_rejected_during_adapter_validation(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    robot_path = profiles / "robots" / "robot.yaml"
    robot = yaml.safe_load(robot_path.read_text())
    robot["mode"] = "hardware"
    robot_path.write_text(yaml.safe_dump(robot), encoding="utf-8")
    runtime = tmp_path / "runtime"
    adapter = HospitalDeliveryAdapter(HospitalSimulationRuntime())
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        audit_writer=AuditWriter(runtime / "audit.jsonl"),
    )
    runtime_owner(active)

    discovered = active.discover_robot("robot")

    assert discovered["hardware_safety_channel"] == "unverified"

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.validate_profile("robot")

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_hardware_profile_cannot_validate_without_a_verified_nonblocking_safety_channel(tmp_path, runtime_owner):
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
    runtime_owner(active)
    discovered = active.discover_robot("robot")

    assert discovered["hardware_safety_channel"] == "unverified"

    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.validate_profile("robot")

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_physical_estop_racing_start_leaves_zero_motion_after_start_returns(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class BlockingAdapter(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait(1.0)
            return self._activate_start(safety_token, lambda: self._record_start(task))

    adapter = BlockingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner)
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
    assert wait_until(lambda: adapter.stop_count >= 3)
    assert adapter.stop_count >= 3


def test_physical_estop_does_not_wait_for_indefinitely_blocked_start(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class HungStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait()
            return self._activate_start(safety_token, lambda: AdapterStatus("running"))

    adapter = HungStart()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)
    starter = threading.Thread(target=lambda: _capture([], active.run_task, "delivery"))
    starter.start()
    assert entered.wait(1.0)

    began = time.monotonic()
    adapter.estop_handler(True)

    assert time.monotonic() - began < 0.2
    assert active.state is SafetyState.ESTOPPED
    assert wait_until(lambda: adapter.stop_count >= 3)
    assert adapter.stop_count >= 3
    release.set()
    starter.join(1.0)


def test_estop_between_start_reservation_and_transport_activation_rejects_start(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class ReservedStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            entered.set()
            release.wait(1.0)
            return self._activate_start(safety_token, lambda: self._record_start(task))

    adapter = ReservedStart()
    active = controller(tmp_path, adapter, owner=runtime_owner)
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


def test_each_stage_activation_uses_a_fresh_internal_safety_reservation(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    task_path = profiles / "tasks" / "delivery.yaml"
    task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    task["stages"].append(
        {
            "name": "return",
            "goal": {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
            "tolerance": 0.25,
            "timeout": 30.0,
        }
    )
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
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.001)
    prepare(active)
    active.run_task("delivery")

    deadline = time.monotonic() + 1.0
    while len(adapter.started) < 2 and time.monotonic() < deadline:
        time.sleep(0.005)

    assert [item.name for item in adapter.started] == ["destination", "return"]
    assert active.state is SafetyState.RUNNING
    active.stop_runtime()


def test_terminal_evidence_is_frozen_before_success_cleanup_and_served_after_stop(tmp_path, runtime_owner):
    events = []

    class TerminalEvidenceAdapter(RecordingAdapter):
        def __init__(self):
            super().__init__()
            self.status_calls = 0
            self.observe_after_stop_fails = False

        def status(self):
            self.status_calls += 1
            return AdapterStatus("succeeded" if self.status_calls == 1 else "running")

        def observe(self, source):
            if self.observe_after_stop_fails and self.stop_count > 0:
                raise AdapterError("STALE_FEEDBACK")
            events.append(("observe", source, self.stop_count))
            return Observation(source, 99.0, {"frozen": True})

        def stop(self):
            events.append(("stop", self.stop_count))
            super().stop()

    adapter = TerminalEvidenceAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.001)
    prepare(active)
    active.run_task("delivery")

    assert wait_until(lambda: active.state is SafetyState.STOPPED)
    assert wait_until(lambda: adapter.stop_count >= 1)
    assert ("observe", "camera", 0) in events
    assert events.index(("observe", "camera", 0)) < events.index(("stop", 0))

    adapter.observe_after_stop_fails = True
    frozen = active.observe("camera")

    assert frozen.source == "camera"
    assert frozen.timestamp == 99.0
    assert dict(frozen.values) == {"frozen": True}
    assert events.count(("observe", "camera", 0)) == 1
    active.stop_runtime()


def test_terminal_evidence_freeze_failure_latches_estop_and_still_cleans(tmp_path, runtime_owner):
    class BrokenFreezeAdapter(RecordingAdapter):
        def status(self):
            return AdapterStatus("succeeded")

        def observe(self, source):
            raise AdapterError("STALE_FEEDBACK")

    adapter = BrokenFreezeAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.001)
    prepare(active)
    active.run_task("delivery")

    assert wait_until(lambda: active.state is SafetyState.ESTOPPED)
    assert wait_until(lambda: adapter.stop_count >= 1)
    assert not active._terminal_evidence_frozen
    with pytest.raises(RuntimeControllerError, match="STALE_FEEDBACK"):
        active.observe("camera")
    active.stop_runtime()


def test_hospital_terminal_odometry_is_built_from_success_status_without_live_observe(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)

    class TerminalHospitalAdapter(SimTimeHospitalAdapter):
        def __init__(self):
            super().__init__()
            self.live_observe_calls = 0

        def status(self):
            return AdapterStatus(
                "succeeded",
                values={
                    "elapsed": 30.0,
                    "stage_results": [
                        {"elapsed": 10.0},
                        {"elapsed": 20.0},
                        {"elapsed": 30.0},
                    ],
                    "pose": {"x": 1.25, "y": -2.5, "yaw": 0.75},
                    "sim_time": 30.0,
                },
            )

        def observe(self, source):
            self.live_observe_calls += 1
            raise AssertionError("hospital terminal evidence must use the success status")

    adapter = TerminalHospitalAdapter()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
        monitor_interval=0.001,
    )
    runtime_owner(active)
    active.discover_robot("hospital-amr")
    active.validate_profile("hospital-amr")
    active.run_task("hospital-delivery")

    assert wait_until(lambda: active.state is SafetyState.STOPPED)
    assert wait_until(lambda: adapter.stop_count >= 1)
    observation = active.observe("odometry")

    assert adapter.live_observe_calls == 0
    assert observation.source == "odometry"
    assert observation.timestamp == 30.0
    assert dict(observation.values) == {"x": 1.25, "y": -2.5, "yaw": 0.75}
    active.stop_runtime()


def test_start_transition_is_audited_before_adapter_activation_and_failure_is_continuous(tmp_path, runtime_owner):
    class FailedStart(RecordingAdapter):
        def start(self, task, safety_token=None):
            raise AdapterError("STALE_FEEDBACK")

    adapter = FailedStart()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="STALE_FEEDBACK"):
        active.run_task("delivery")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [item["operation"] for item in records[-2:]] == ["start_task", "estop"]
    assert records[-2]["state"] == {"from": "ARMED", "to": "RUNNING"}
    assert records[-1]["state"] == {"from": "RUNNING", "to": "ESTOPPED"}


def test_estop_between_gateway_start_transition_and_durable_append_keeps_audit_continuous(tmp_path, runtime_owner):
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
    runtime_owner(active)
    prepare(active)
    start_errors = []
    starter = threading.Thread(target=lambda: _capture(start_errors, active.run_task, "delivery"))
    starter.start()
    assert entered.wait(1.0)
    assert active._audit_lock.acquire(timeout=0.05)
    active._audit_lock.release()

    estopper = threading.Thread(target=lambda: adapter.estop_handler(True))
    estopper.start()
    deadline = time.monotonic() + 1.0
    while active.state is not SafetyState.ESTOPPED and time.monotonic() < deadline:
        time.sleep(0.001)
    assert active.state is SafetyState.ESTOPPED
    assert wait_until(lambda: adapter.stop_count >= 3)
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
    runtime_owner(restarted)
    assert restarted.discover_robot("robot")["state"] == "DISCOVERED"
    restarted.stop_runtime()


def test_stop_runtime_times_out_waiting_for_concurrent_audit_durability(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    runtime = tmp_path / "runtime"

    class BlockedStartWriter(AuditWriter):
        def append(self, event):
            if event.operation is AuditOperation.START_TASK:
                entered.set()
                release.wait()
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
        audit_writer=BlockedStartWriter(runtime / "audit.jsonl"),
        cleanup_timeout=0.02,
    )
    audit_workers = [thread for thread in threading.enumerate() if thread.name == "agent-ros-audit"]
    assert len(audit_workers) == 1
    assert audit_workers[0].daemon is False
    prepare(active)
    errors = []
    starter = threading.Thread(
        target=lambda: _capture(errors, active.run_task, "delivery"),
    )
    starter.start()
    assert entered.wait(1.0)

    began = time.monotonic()
    try:
        with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
            active.stop_runtime()
    finally:
        release.set()
    elapsed = time.monotonic() - began
    starter.join(1.0)

    assert elapsed < 0.2
    assert not starter.is_alive()
    assert wait_until(lambda: not audit_workers[0].is_alive())
    assert (runtime / "audit.quarantine").read_text(encoding="ascii") == ("AUDIT_INTEGRITY_COMPROMISED\n")


def test_higher_sequence_append_waits_for_delayed_lower_receipt(tmp_path, runtime_owner):
    active = controller(tmp_path, RecordingAdapter(), owner=runtime_owner, cleanup_timeout=0.2)
    prepare(active)
    gateway = active._gateway
    lower = gateway.start_task()
    higher = gateway.heartbeat()
    active._register_transition(AuditOperation.HEARTBEAT, higher)
    errors = []

    def drain_higher():
        try:
            active._drain_pending_transitions(
                wait_for=higher.sequence,
                timeout=0.2,
            )
        except BaseException as exc:
            errors.append(exc)

    waiter = threading.Thread(target=drain_higher)

    waiter.start()
    assert wait_until(lambda: higher.sequence in active._pending_audit)
    waiter.join(0.03)
    assert waiter.is_alive()

    active._register_transition(
        AuditOperation.START_TASK,
        lower,
        operation_data={"task": "delivery"},
    )
    waiter.join(1.0)

    assert not waiter.is_alive()
    assert errors == []
    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == [
        "start_task",
        "heartbeat",
    ]
    estop = gateway.estop(timeout=0.2)
    assert estop is not None
    active._append_transition(AuditOperation.ESTOP, estop)
    assert gateway.close(timeout=0.2)
    assert active._audit_worker.close(0.2)
    assert active._adapter is not None
    assert active._adapter.close(0.2)


def test_stop_runtime_waits_to_entry_deadline_for_pending_sequence_gap(tmp_path):
    active = controller(tmp_path, RecordingAdapter(), cleanup_timeout=0.02)
    prepare(active)
    gateway = active._gateway
    _lower = gateway.start_task()
    higher = gateway.heartbeat()
    active._register_transition(AuditOperation.HEARTBEAT, higher)

    began = time.monotonic()
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()
    elapsed = time.monotonic() - began

    assert elapsed >= 0.015
    assert elapsed < 0.2
    assert higher.sequence in active._pending_audit
    assert (tmp_path / "runtime" / "audit.quarantine").read_text(encoding="ascii") == "AUDIT_INTEGRITY_COMPROMISED\n"


def test_audit_coordinator_orders_cancel_before_concurrent_estop_by_transition_sequence(tmp_path, runtime_owner):
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
    runtime_owner(active)
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


def test_initial_heartbeat_expiry_audits_fault_before_estop_without_stalling_sequence(tmp_path, runtime_owner):
    now = [0.0]

    class ExpiringStart(RecordingAdapter):
        def start(self, task, activation_permit=None):
            result = super().start(task, activation_permit)
            now[0] = 2.0
            return result

    adapter = ExpiringStart()
    active = controller(tmp_path, adapter, owner=runtime_owner, clock=lambda: now[0])
    prepare(active)
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, active.run_task, "delivery"),
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
    expected_stop_result = {
        "activation_quiesced": True,
        "code": "ESTOP_LATCHED",
        "latched": True,
        "safety_command_accepted": True,
    }
    assert records[-2]["operation_data"] == expected_stop_result
    assert records[-1]["operation_data"] == expected_stop_result


def test_physical_estop_drains_unobserved_watchdog_fault_without_waiting_for_monitor(tmp_path, runtime_owner):
    now = [0.0]
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, clock=lambda: now[0], monitor_interval=10.0)
    prepare(active)
    active.run_task("delivery")
    now[0] = 2.0
    assert active._gateway.supervisor.evaluate() is True
    assert active.state is SafetyState.FAULTED
    errors = []
    estopper = threading.Thread(target=lambda: _capture(errors, adapter.estop_handler, True))
    estopper.start()
    estopper.join(0.05)
    blocked = estopper.is_alive()
    if blocked:
        active._append_latest_gateway_fault(active._gateway)
    estopper.join(1.0)

    assert not blocked
    assert not errors
    active.stop_runtime()
    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == ["heartbeat", "estop"]
    assert records[-2]["state"] == {"from": "RUNNING", "to": "FAULTED"}
    assert records[-1]["state"] == {"from": "FAULTED", "to": "ESTOPPED"}


def test_physical_estop_attempt_keeps_degraded_result_when_watchdog_estop_wins(
    tmp_path,
    runtime_owner,
):
    degraded = EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    successful = EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=10.0)
    prepare(active)
    active.run_task("delivery")
    gateway = active._gateway
    gateway._stop_callback = lambda _timeout: (
        degraded if threading.current_thread().name == "physical-a" else successful
    )
    physical_ready = threading.Event()
    release_physical = threading.Event()
    original_stop = gateway._stop_repeatedly

    def ordered_stop(timeout):
        result = original_stop(timeout)
        if threading.current_thread().name == "physical-a":
            physical_ready.set()
            assert release_physical.wait(1.0)
        return result

    gateway._stop_repeatedly = ordered_stop
    errors = []
    physical = threading.Thread(
        target=lambda: _capture(errors, adapter.estop_handler, True),
        name="physical-a",
    )
    physical.start()
    assert physical_ready.wait(1.0)

    gateway._fault("HEARTBEAT_EXPIRED")
    active._latch_adapter_fault()
    release_physical.set()
    physical.join(1.0)

    assert not physical.is_alive()
    assert errors == []
    assert (tmp_path / "runtime" / "audit.quarantine").read_text(encoding="ascii") == "AUDIT_INTEGRITY_COMPROMISED\n"
    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [record["state"] for record in records[-2:]] == [
        {"from": "RUNNING", "to": "FAULTED"},
        {"from": "FAULTED", "to": "ESTOPPED"},
    ]
    assert [record["operation_data"]["code"] for record in records[-2:]] == [
        "ESTOP_LATCHED",
        "ESTOP_LATCHED",
    ]


def test_monitor_never_mistakes_a_newer_estop_for_the_watchdog_fault(tmp_path, runtime_owner):
    now = [0.0]
    entered = threading.Event()
    release = threading.Event()
    profiles = tmp_path / "profiles"
    runtime = tmp_path / "runtime"
    write_profiles(profiles)

    class PausedFaultController(RuntimeController):
        def _append_latest_gateway_fault(self, gateway):
            entered.set()
            release.wait(1.0)
            return super()._append_latest_gateway_fault(gateway)

    adapter = RecordingAdapter()
    active = PausedFaultController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: adapter,
        clock=lambda: now[0],
        monitor_interval=0.001,
    )
    runtime_owner(active)
    prepare(active)
    active.run_task("delivery")
    now[0] = 2.0
    assert active._gateway.supervisor.evaluate() is True
    assert entered.wait(1.0)

    adapter.estop_handler(True)
    release.set()
    assert wait_until(
        lambda: json.loads((runtime / "audit.jsonl").read_text().splitlines()[-1])["operation"] == "estop"
    )

    records = [json.loads(line) for line in (runtime / "audit.jsonl").read_text().splitlines()]
    assert [record["operation"] for record in records[-2:]] == ["heartbeat", "estop"]
    assert records[-2]["state"] == {"from": "RUNNING", "to": "FAULTED"}
    assert records[-1]["state"] == {"from": "FAULTED", "to": "ESTOPPED"}


def test_dead_emergency_worker_fails_closed_and_shutdown_reports_failure(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class FailingEmergencyChannel(RecordingEmergencyChannel):
        def _enqueue_zero_disable(self):
            entered.set()
            release.wait()
            raise RuntimeError("controlled transport failure")

    adapter = RecordingAdapter()
    adapter.safety_channel = FailingEmergencyChannel(adapter)
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")

    assert active.emergency_stop() == {"state": "ESTOPPED"}
    assert entered.wait(1.0)
    release.set()
    assert wait_until(lambda: adapter.safety_channel._worker._failed)
    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.emergency_stop()
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()


def test_full_emergency_queue_fails_closed_without_blocking_caller(tmp_path):
    entered = threading.Event()
    release = threading.Event()

    class BlockingEmergencyChannel(RecordingEmergencyChannel):
        def _enqueue_zero_disable(self):
            entered.set()
            release.wait()

    adapter = RecordingAdapter()
    adapter.safety_channel = BlockingEmergencyChannel(adapter)
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")
    active.emergency_stop()
    assert entered.wait(1.0)

    failure = None
    began = time.monotonic()
    for _ in range(8):
        try:
            active.emergency_stop()
        except RuntimeControllerError as exc:
            failure = exc
            break

    assert time.monotonic() - began < 0.2
    assert failure is not None and failure.code == "UNSAFE_STATE"
    assert active.state is SafetyState.ESTOPPED
    release.set()
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()


def test_stop_runtime_reports_cleanup_failure_when_monitor_status_never_returns(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class HungStatus(RecordingAdapter):
        def status(self):
            entered.set()
            release.wait()
            return AdapterStatus("running")

    adapter = HungStatus()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.001, cleanup_timeout=0.02)
    prepare(active)
    active.run_task("delivery")
    assert entered.wait(1.0)

    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()

    assert active.state is not SafetyState.RUNNING
    release.set()
    assert active.stop_runtime() == {"state": "ESTOPPED"}


def test_shutdown_uses_its_own_degraded_attempt_when_repeated_estop_is_successful(
    tmp_path,
):
    degraded = EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    successful = EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
    active = controller(tmp_path, RecordingAdapter())
    prepare(active)
    active.run_task("delivery")
    gateway = active._gateway
    gateway._stop_callback = lambda _timeout: (
        degraded if threading.current_thread().name == "shutdown-a" else successful
    )
    shutdown_ready = threading.Event()
    release_shutdown = threading.Event()
    original_stop = gateway._stop_repeatedly

    def ordered_stop(timeout):
        result = original_stop(timeout)
        if threading.current_thread().name == "shutdown-a":
            shutdown_ready.set()
            assert release_shutdown.wait(1.0)
        return result

    gateway._stop_repeatedly = ordered_stop
    responses = []
    errors = []

    def stop_runtime():
        try:
            responses.append(active.stop_runtime())
        except BaseException as exc:
            errors.append(exc)

    shutdown = threading.Thread(
        target=stop_runtime,
        name="shutdown-a",
    )

    shutdown.start()
    assert shutdown_ready.wait(1.0)
    repeated = active.emergency_stop()
    release_shutdown.set()
    shutdown.join(1.0)

    assert not shutdown.is_alive()
    assert repeated == {"state": "ESTOPPED"}
    assert responses == []
    assert len(errors) == 1
    assert isinstance(errors[0], RuntimeControllerError)
    assert errors[0].code == "CLEANUP_FAILED"


def test_stop_runtime_uses_bounded_tracked_task_cleanup_after_nonblocking_safety_enqueue(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class HungStop(RecordingAdapter):
        def stop(self):
            entered.set()
            release.wait()

    adapter = HungStop()
    active = controller(tmp_path, adapter, owner=runtime_owner, cleanup_timeout=0.02)
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


def test_stop_runtime_shares_one_entry_deadline_across_owned_components(tmp_path, monkeypatch):
    class FakeMonotonic:
        def __init__(self):
            self.now = 10.0

        def __call__(self):
            return self.now

        def consume(self, duration):
            self.now += duration

    clock = FakeMonotonic()
    offered = []
    result = EmergencyStopResult(True, True, True, "ESTOP_LATCHED")

    class OwnedGateway:
        state = SafetyState.ESTOPPED
        last_stop_result = result
        last_stop_accepted = True

        def estop_attempt(self, *, timeout=1.0):
            offered.append(("estop", timeout))
            clock.consume(timeout)
            return SafetyStopAttempt(None, result)

        def close(self, *, timeout=1.0):
            offered.append(("gateway", timeout))
            clock.consume(timeout)
            return True

        def transitions_from(self, _sequence):
            return ()

    class OwnedAdapter:
        def close(self, timeout=1.0):
            offered.append(("adapter", timeout))
            clock.consume(timeout)
            return True

    active = controller(tmp_path, RecordingAdapter(), cleanup_timeout=0.5)
    active._gateway = OwnedGateway()
    active._adapter = OwnedAdapter()
    active._task_cleanup_started = True
    monkeypatch.setattr("agent_ros.runtime.controller.time.monotonic", clock)

    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()
    assert sum(timeout for _owner, timeout in offered) <= 0.5
    assert offered == [("estop", 0.5), ("gateway", 0.0), ("adapter", 0.0)]


def test_physical_estop_returns_promptly_while_monitor_status_is_blocked(tmp_path, runtime_owner):
    entered = threading.Event()
    release = threading.Event()

    class HungStatus(RecordingAdapter):
        def status(self):
            entered.set()
            release.wait()
            return AdapterStatus("running")

    adapter = HungStatus()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.001, cleanup_timeout=0.02)
    prepare(active)
    active.run_task("delivery")
    assert entered.wait(1.0)

    began = time.monotonic()
    adapter.estop_handler(True)

    assert time.monotonic() - began < 0.2
    assert active.state is SafetyState.ESTOPPED
    assert wait_until(lambda: adapter.stop_count >= 3)
    assert adapter.stop_count >= 3
    release.set()
    active.stop_runtime()


def test_monitor_thread_start_failure_latches_stop_and_never_leaks_running(tmp_path, runtime_owner):
    adapter = RecordingAdapter()

    class FailedThread:
        def start(self):
            raise RuntimeError("raw")

        def join(self, timeout=None):
            raise AssertionError("unstarted monitor joined")

    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_thread_factory=lambda **_kwargs: FailedThread())
    prepare(active)

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.run_task("delivery")

    assert active.state is SafetyState.ESTOPPED
    assert adapter.stop_count >= 3

    assert active.stop_runtime() == {"state": "ESTOPPED"}


def test_cleanup_thread_start_failure_is_reported_without_joining_unstarted_thread(tmp_path, monkeypatch):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter)
    prepare(active)
    active.run_task("delivery")
    original_start = threading.Thread.start

    def fail_cleanup_start(thread):
        if thread.name == "agent-ros-task-cleanup":
            raise RuntimeError("controlled start failure")
        return original_start(thread)

    monkeypatch.setattr(threading.Thread, "start", fail_cleanup_start)

    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()


def _capture(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_owned_monitor_keeps_healthy_runtime_alive_without_mcp_polling(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")

    time.sleep(0.08)

    assert active.state is SafetyState.RUNNING
    active.stop_runtime()


def test_owned_monitor_latches_stale_adapter_without_mcp_polling(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
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
    assert json.loads(store.read(reference, max_bytes=1024)) == {"result": "pass"}

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

    assert json.loads(store.read(reference, max_bytes=1024)) == {"ok": True}
    assert outside.read_text(encoding="utf-8") == "do-not-touch"


def test_evidence_constructor_maps_filesystem_errors_without_leaking_paths(tmp_path):
    blocker = tmp_path / "not-a-directory"
    blocker.write_text("x", encoding="utf-8")

    with pytest.raises(EvidenceError) as captured:
        EvidenceStore(blocker / "evidence")

    assert str(captured.value) == "EVIDENCE_INVALID"


@pytest.mark.parametrize("replacement", [b"different-size", b"XXXXXXXX"])
def test_evidence_snapshot_rejects_replacement_after_reference_resolution(tmp_path, replacement):
    root = tmp_path / "evidence"
    root.mkdir()
    target = root / "report.json"
    target.write_bytes(b"12345678")
    store = EvidenceStore(root)
    reference = store.get("report")
    temporary = root / "replacement"
    temporary.write_bytes(replacement)
    temporary.replace(target)

    with pytest.raises(EvidenceError, match="EVIDENCE_INVALID"):
        store.read(reference, max_bytes=1024)


def test_evidence_snapshot_rejects_growth_while_reading(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    root.mkdir()
    target = root / "report.json"
    target.write_bytes(b"A" * 65536)
    store = EvidenceStore(root)
    reference = store.get("report")
    real_read = __import__("os").read
    calls = []

    def growing_read(descriptor, size):
        chunk = real_read(descriptor, size)
        calls.append(len(chunk))
        if len(calls) == 1:
            with target.open("ab") as handle:
                handle.write(b"B" * 65536)
        return chunk

    monkeypatch.setattr("agent_ros.runtime.evidence.os.read", growing_read)

    with pytest.raises(EvidenceError, match="EVIDENCE_INVALID"):
        store.read(reference, max_bytes=200000)

    assert len(calls) <= 3


def test_evidence_snapshot_rejects_declared_size_above_bound_before_reading(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    root.mkdir()
    target = root / "report.json"
    target.write_bytes(b"A" * 32)
    store = EvidenceStore(root)
    reference = store.get("report")
    reads = []
    monkeypatch.setattr(
        "agent_ros.runtime.evidence.os.read",
        lambda descriptor, size: reads.append((descriptor, size)) or b"",
    )

    with pytest.raises(EvidenceError, match="EVIDENCE_INVALID"):
        store.read(reference, max_bytes=16)

    assert reads == []


class CompromisedWriter:
    def append(self, _event) -> None:
        raise AuditIntegrityError()


def test_audit_integrity_compromise_persists_quarantine_across_controller_restart(tmp_path, runtime_owner):
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
    runtime_owner(restarted)
    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        restarted.discover_robot("robot")
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        compromised.stop_runtime()


def test_restart_quarantines_schema_invalid_but_parseable_audit_record(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    (runtime / "audit.jsonl").write_text(
        json.dumps(
            {
                "operation": "start_task",
                "state": {"from": "NEW", "to": "RUNNING"},
                "outcome": "ok",
                "wall_time": 1.0,
                "monotonic_time": 1.0,
                "operation_data": {"payload": "raw"},
                "endpoint_gids": [],
            }
        )
        + "\n",
        encoding="utf-8",
    )
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    runtime_owner(active)

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        active.discover_robot("robot")
    assert active.stop_runtime() == {"state": "NEW"}


@pytest.mark.parametrize("mutation", ["oversized", "discontinuous", "bad-first"])
def test_restart_quarantines_bounded_but_impossible_audit_history(tmp_path, mutation, runtime_owner):
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
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=runtime,
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    runtime_owner(active)

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        active.discover_robot("robot")


def test_clean_stopped_session_allows_new_controller_session_from_new(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    first = controller(tmp_path, adapter, owner=runtime_owner)
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
    runtime_owner(second)
    result = second.discover_robot("robot")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert result["state"] == "DISCOVERED"
    assert records[-1]["session_id"] != records[0]["session_id"]


def test_restart_quarantines_a_previous_running_session(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    first = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(first)
    first.run_task("delivery")

    restarted = RuntimeController(
        profiles_root=tmp_path / "profiles",
        evidence_dir=tmp_path / "evidence-2",
        runtime_dir=tmp_path / "runtime",
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    runtime_owner(restarted)

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        restarted.discover_robot("robot")
    first.stop_runtime()


def test_restart_quarantines_interleaved_or_replayed_audit_session(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner)
    prepare(active)
    path = tmp_path / "runtime" / "audit.jsonl"
    records = path.read_text().splitlines()
    path.write_text(records[0] + "\n" + records[1] + "\n" + records[0] + "\n", encoding="utf-8")

    restarted = RuntimeController(
        profiles_root=tmp_path / "profiles",
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=Probe(),
        adapter_factory=lambda _profile: RecordingAdapter(),
    )
    runtime_owner(restarted)

    with pytest.raises(RuntimeControllerError, match="AUDIT_INTEGRITY_COMPROMISED"):
        restarted.discover_robot("robot")


def test_physical_binding_failure_is_estopped_and_audited_once(tmp_path, runtime_owner):
    class BrokenBinding(RecordingAdapter):
        def bind_physical_estop(self, handler):
            raise RuntimeError("raw device path")

    active = controller(tmp_path, BrokenBinding(), owner=runtime_owner)

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.discover_robot("robot")

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    assert [item["operation"] for item in records] == ["estop"]
    active.stop_runtime()


def test_quarantined_runtime_still_allows_owned_cleanup_and_join(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")
    (tmp_path / "runtime" / "audit.quarantine").write_text("AUDIT_INTEGRITY_COMPROMISED\n")

    result = active.stop_runtime()

    assert result["state"] == "ESTOPPED"
    assert adapter.stop_count >= 3


def test_owned_monitor_enforces_stage_timeout_without_mcp_polling(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles, stage_timeout=0.02)
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.005)
    prepare(active)
    active.run_task("delivery")

    deadline = time.monotonic() + 1.0
    while active.state is SafetyState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.005)

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def _running_hospital_controller(tmp_path, runtime_owner, now):
    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    adapter = SimTimeHospitalAdapter()
    active = controller(
        tmp_path,
        adapter,
        owner=runtime_owner,
        clock=lambda: now[0],
        monitor_interval=10.0,
    )
    active.discover_robot("hospital-amr")
    active.validate_profile("hospital-amr")
    active.arm_robot("hospital-amr", challenge="ARM hospital-amr", dry_run=False)
    active.run_task("hospital-delivery")
    return active, adapter


def test_hospital_runtime_uses_adapter_sim_time_not_slow_wall_time_for_budget(tmp_path, runtime_owner):
    now = [0.0]
    active, adapter = _running_hospital_controller(tmp_path, runtime_owner, now)
    adapter.values = {
        "elapsed": 90.0,
        "stage_results": [{"stage": "pharmacy", "elapsed": 30.0}],
    }

    # At RTF 0.5, a 90 second mission legitimately uses just over 180 seconds
    # of wall time and must not consume the ROS-time task budget twice.
    now[0] = 180.1
    status = active._poll_running()

    assert status.state == "running"
    assert active.state is SafetyState.RUNNING
    active.stop_runtime()


def test_hospital_runtime_faults_when_current_sim_stage_exceeds_profile_budget(tmp_path, runtime_owner):
    now = [0.0]
    active, adapter = _running_hospital_controller(tmp_path, runtime_owner, now)
    adapter.values = {
        "elapsed": 91.0,
        "stage_results": [{"stage": "pharmacy", "elapsed": 30.0}],
    }

    now[0] = 100.0
    with pytest.raises(RuntimeControllerError, match="TIMEOUT"):
        active._poll_running()

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_hospital_runtime_keeps_separate_bounded_wall_liveness_deadline(tmp_path, runtime_owner):
    now = [0.0]
    active, adapter = _running_hospital_controller(tmp_path, runtime_owner, now)
    adapter.values = {"elapsed": 90.0, "stage_results": []}

    now[0] = 300.001
    with pytest.raises(RuntimeControllerError, match="TIMEOUT"):
        active._poll_running()

    assert active.state is SafetyState.ESTOPPED
    active.stop_runtime()


def test_sealed_hospital_discovery_validates_from_available_fixed_lifecycle_on_empty_graph(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    adapter = SimTimeHospitalAdapter()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
    )
    runtime_owner(active)

    discovered = active.discover_robot("hospital-amr")
    validated = active.validate_profile("hospital-amr")

    assert discovered["capabilities"] == ["mobile_base.twist"]
    assert validated["state"] == "ARMED"
    active.stop_runtime()


def test_hospital_stop_runtime_reaps_inflight_start_with_one_cleanup_deadline(tmp_path, monkeypatch):
    start_entered = threading.Event()
    release_start = threading.Event()
    stop_executed = threading.Event()
    mission_start_executed = threading.Event()

    def fixed_call(self, suffix, *, timeout, generation=None):
        if suffix[0] == "start":
            start_entered.set()
            assert release_start.wait(1.0)
            return {"ok": True, "running": True}
        if suffix[0] == "stop":
            stop_executed.set()
            release_start.set()
            return {"ok": True, "running": False}
        if suffix[0] == "mission-start":
            mission_start_executed.set()
            return {"ok": True, "success": True}
        raise AssertionError(suffix)

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    client = HospitalLifecycleClient()
    adapter = HospitalCaseAdapter(client)
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
        cleanup_timeout=0.8,
        monitor_interval=10.0,
    )
    active.discover_robot("hospital-amr")
    active.validate_profile("hospital-amr")
    active.arm_robot("hospital-amr", challenge="ARM hospital-amr", dry_run=False)
    active.run_task("hospital-delivery")
    assert start_entered.wait(0.2)

    started = time.monotonic()
    result = active.stop_runtime()
    elapsed = time.monotonic() - started

    assert result == {"state": "ESTOPPED"}
    assert elapsed <= 0.8
    assert stop_executed.is_set()
    assert client._start_receipt.done.is_set()
    assert client._start_receipt.error is None
    assert not mission_start_executed.is_set()
    assert not client._worker._thread.is_alive()
    assert not client._emergency_worker._thread.is_alive()


def test_sealed_hospital_empty_graph_fallback_rejects_unavailable_fixed_lifecycle(tmp_path, runtime_owner):
    class UnavailableHospitalAdapter(SimTimeHospitalAdapter):
        close_calls = 0

        def probe(self):
            return AdapterProbe(False, ("hospital.delivery",))

        def close(self, timeout=1.0):
            self.close_calls += 1
            return super().close(timeout)

    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    adapter = UnavailableHospitalAdapter()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
    )
    runtime_owner(active)

    with pytest.raises(RuntimeControllerError, match="UNSAFE_STATE"):
        active.discover_robot("hospital-amr")
    assert adapter.close_calls == 1


def test_sealed_hospital_fallback_never_hides_live_command_publisher_conflict(tmp_path, runtime_owner):
    class CloseTrackingHospitalAdapter(SimTimeHospitalAdapter):
        close_calls = 0

        def close(self, timeout=1.0):
            self.close_calls += 1
            return super().close(timeout)

    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    adapter = CloseTrackingHospitalAdapter()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=ConflictingProbe(),
        adapter_factory=lambda _profile: adapter,
    )
    runtime_owner(active)

    with pytest.raises(RuntimeControllerError, match="CONTROLLER_CONFLICT"):
        active.discover_robot("hospital-amr")
    assert adapter.close_calls == 1


def test_tentative_adapter_cleanup_failure_poison_overrides_discovery_error(
    tmp_path,
):
    class CleanupFailingHospitalAdapter(SimTimeHospitalAdapter):
        close_calls = 0

        def probe(self):
            return AdapterProbe(False, ("hospital.delivery",))

        def close(self, timeout=1.0):
            self.close_calls += 1
            return False

    profiles = tmp_path / "profiles"
    write_hospital_profiles(profiles)
    adapter = CleanupFailingHospitalAdapter()
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
    )
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.discover_robot("hospital-amr")
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.discover_robot("hospital-amr")
    assert adapter.close_calls == 1
    assert HospitalCaseAdapter.close(adapter, 1.0)
    with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
        active.stop_runtime()


def test_empty_graph_never_grants_standard_adapter_profile_capabilities(tmp_path, runtime_owner):
    adapter = RecordingAdapter()
    profiles = tmp_path / "profiles"
    write_profiles(profiles)
    active = RuntimeController(
        profiles_root=profiles,
        evidence_dir=tmp_path / "evidence",
        runtime_dir=tmp_path / "runtime",
        graph_probe=EmptyProbe(),
        adapter_factory=lambda _profile: adapter,
    )
    runtime_owner(active)

    assert active.discover_robot("robot")["capabilities"] == []
    with pytest.raises(RuntimeControllerError, match="PROFILE_INVALID"):
        active.validate_profile("robot")


def test_watchdog_fault_transition_is_audited_once_by_owned_monitor(tmp_path, runtime_owner):
    profiles = tmp_path / "profiles"
    write_profiles(profiles, heartbeat_timeout=0.02)
    adapter = RecordingAdapter()
    active = controller(tmp_path, adapter, owner=runtime_owner, monitor_interval=0.08)
    prepare(active)
    active.run_task("delivery")
    time.sleep(0.16)

    records = [json.loads(line) for line in (tmp_path / "runtime" / "audit.jsonl").read_text().splitlines()]
    faulted = [item for item in records if item["operation"] == "heartbeat" and item["outcome"] == "faulted"]
    assert active.state is SafetyState.FAULTED
    assert len(faulted) == 1
    active.stop_runtime()
