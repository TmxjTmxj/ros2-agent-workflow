from __future__ import annotations

import sys
import subprocess
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

import agent_ros.adapters as adapters_package
from agent_ros.adapters._safety import _ActivationIssuer, _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterStatus,
    HospitalAction,
    NavigationGoal,
    OdometrySample,
    TwistCommand,
    create_adapter,
)
from agent_ros.adapters.hospital import HospitalDeliveryAdapter, HospitalSimulationRuntime
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.profiles.models import RobotProfile
from agent_ros.profiles.models import PoseGoal, TaskStage
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.sequencer import _ActivationRejected, _SafetySequencer
from tests.support.runtime_owners import adapter_owner


def robot_profile(kind: str = "twist", *, mode: str = "simulation") -> RobotProfile:
    interfaces: dict[str, object]
    if kind == "nav2":
        interfaces = {
            "navigation": {
                "action": "/navigate_to_pose",
                "type": "nav2_msgs/action/NavigateToPose",
            }
        }
    else:
        interfaces = {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        }
    return RobotProfile.from_mapping({
        "name": "robot",
        "mode": mode,
        "namespace": "/robot",
        "frames": {"base": "base_link", "odom": "odom"},
        "adapter": {"kind": kind},
        "interfaces": interfaces,
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
        "observation_sources": ["odometry"],
    })


class RecordingEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, enqueue, *, hardware_verified: bool = True, available: bool = True) -> None:
        super().__init__(hardware_verified=hardware_verified)
        self._enqueue = enqueue
        self._available = available

    def _preflight(self) -> bool:
        return self._available

    def _enqueue_zero_disable(self) -> None:
        self._enqueue()


class TwistTransport:
    def __init__(self) -> None:
        self.commands: list[TwistCommand] = []
        self.odometry = OdometrySample(timestamp=0.0, x=1.0, y=2.0, yaw=0.25)
        self.estop_handler = None
        self.started_waypoints = []
        self.state = AdapterStatus("idle")
        self.generation = 0
        self.safety_channel = RecordingEmergencyChannel(self._enqueue_emergency_stop)

    def publish(self, command: TwistCommand) -> None:
        self.commands.append(command)

    def read_odometry(self) -> OdometrySample:
        return self.odometry

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler

    def preflight_activation(self) -> bool:
        return True

    def start_waypoint(self, stage, _permit=None) -> None:
        self.started_waypoints.append(stage)
        self.state = AdapterStatus("running")

    def waypoint_status(self):
        return self.state

    def cancel_waypoint(self):
        self.state = AdapterStatus("cancelled")

    def stop_waypoint(self):
        self.commands.extend([TwistCommand.zero()] * 3)
        self.state = AdapterStatus("stopped")

    def _enqueue_emergency_stop(self):
        self.generation += 1
        self.commands.append(TwistCommand.zero())

    def emergency_channel(self):
        return self.safety_channel


def stage(*, timeout: float = 30.0) -> TaskStage:
    return TaskStage("destination", PoseGoal("odom", 2.0, 2.0, 0.0), 0.1, timeout)


def bind_permit(adapter):
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)
    adapter._validate_runtime_safety("simulation")
    return issuer._issue()


def valid_permit(adapter, owner):
    owner(adapter)
    return bind_permit(adapter)


def test_direct_adapter_context_management_closes_bound_workers():
    transport = TwistTransport()

    with TwistAdapter(robot_profile(), transport, clock=lambda: 0.0) as adapter:
        bind_permit(adapter)
        assert adapter._safety_sequencer.worker_alive

    assert not adapter._safety_sequencer.worker_alive


def test_adapter_close_before_start_is_successful():
    adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)

    assert adapter.close(timeout=0.1)


def test_started_adapter_close_is_idempotent():
    adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)
    bind_permit(adapter)

    assert adapter.close(timeout=0.2)
    assert adapter.close(timeout=0.2)


def test_adapter_context_manager_exposes_close_failure():
    class FailingCloseAdapter(TwistAdapter):
        def close(self, timeout: float = 1.0) -> bool:
            return False

    with pytest.raises(AdapterError, match="CLEANUP_FAILED"):
        with FailingCloseAdapter(
            robot_profile(), TwistTransport(), clock=lambda: 0.0
        ):
            pass


def test_caller_omitting_adapter_close_remains_alive_until_harness_termination():
    program = """
from tests.test_adapters import TwistAdapter, TwistTransport, bind_permit, robot_profile

adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)
bind_permit(adapter)
print("READY", flush=True)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", program],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert process.stdout is not None
        assert process.stdout.readline().strip() == "READY"
        with pytest.raises(subprocess.TimeoutExpired):
            process.wait(timeout=0.1)
    finally:
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)


class CallbackFuture:
    def __init__(self):
        self.callback = None
        self.value = None
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def resolve(self, value):
        self.value = value
        if self.callback is not None:
            self.callback(self)

    def reject(self, error):
        self.error = error
        if self.callback is not None:
            self.callback(self)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeActionClient:
    def __init__(self, node, _action_type, _action_name):
        self.goal_future = CallbackFuture()
        node.action_client = self

    def send_goal_async(self, _goal):
        return self.goal_future


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.action_client = None

    def create_publisher(self, _message_type, _topic, _depth):
        publisher = FakePublisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _message_type, _topic, callback, _depth):
        self.subscriptions.append(callback)
        return callback

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object()))


def install_fake_ros(monkeypatch) -> None:
    class FakeTwist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    class FakeNavigateToPose:
        class Goal:
            def __init__(self):
                self.pose = SimpleNamespace(
                    header=SimpleNamespace(frame_id=None, stamp=None),
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                        orientation=SimpleNamespace(z=0.0, w=0.0),
                    ),
                )

    modules = {
        "geometry_msgs.msg": {"Twist": FakeTwist},
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "std_msgs.msg": {"Bool": type("Bool", (), {})},
        "nav2_msgs.action": {"NavigateToPose": FakeNavigateToPose},
        "rclpy.action": {"ActionClient": FakeActionClient},
    }
    for dotted_name, attributes in modules.items():
        parent_name, child_name = dotted_name.split(".")
        parent = sys.modules.get(parent_name, ModuleType(parent_name))
        child = ModuleType(dotted_name)
        for name, value in attributes.items():
            setattr(child, name, value)
        setattr(parent, child_name, child)
        monkeypatch.setitem(sys.modules, parent_name, parent)
        monkeypatch.setitem(sys.modules, dotted_name, child)


def real_twist_transport(monkeypatch, *, clock=lambda: 0.0):
    from agent_ros.adapters.twist import RclpyTwistTransport

    install_fake_ros(monkeypatch)
    node = FakeNode()
    transport = RclpyTwistTransport(
        node,
        "/cmd_vel",
        "/odom",
        "/emergency_stop",
        limits=robot_profile().limits,
        control_period=0.1,
    )
    transport._clock = clock
    return transport


def real_nav2_transport(monkeypatch, *, clock=lambda: 0.0, cancel_timeout=1.0):
    from agent_ros.adapters.nav2 import RclpyNav2Transport

    install_fake_ros(monkeypatch)
    node = FakeNode()
    transport = RclpyNav2Transport(
        node,
        "/navigate_to_pose",
        "/cmd_vel",
        "/emergency_stop",
        clock=clock,
        cancel_timeout=cancel_timeout,
    )
    return transport


def test_twist_start_accepts_only_a_reviewed_stage(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    adapter.start(stage(), valid_permit(adapter, adapter_owner))

    assert transport.started_waypoints == [stage()]


def _wait_until(predicate, timeout=1.0):
    deadline = __import__("time").monotonic() + timeout
    while not predicate() and __import__("time").monotonic() < deadline:
        __import__("time").sleep(0.001)
    return predicate()


def test_twist_emergency_stop_idempotently_accepts_a_fresh_zero_enqueue(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    valid_permit(adapter, adapter_owner)

    adapter._emergency_stop()
    adapter._emergency_stop()

    assert _wait_until(lambda: len(transport.commands) == 2)
    assert transport.commands == [TwistCommand.zero()] * 2


def test_twist_stale_odometry_stops_with_a_zero_burst_and_reports_stable_code(adapter_owner):
    now = [0.0]
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0], stale_after=1.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    now[0] = 1.01
    transport.waypoint_status = lambda: (_ for _ in ()).throw(AdapterError("STALE_FEEDBACK"))

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert transport.commands[-3:] == [TwistCommand.zero()] * 3


@pytest.mark.parametrize("timestamp", [-2.0, 0.2])
def test_twist_stage_refuses_stale_or_future_odometry_before_any_nonzero_motion(timestamp, adapter_owner):
    transport = TwistTransport()
    transport.odometry = OdometrySample(timestamp=timestamp, x=1.0, y=2.0, yaw=0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0, stale_after=1.0, future_skew=0.05)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.start(stage(), valid_permit(adapter, adapter_owner))

    assert all(command == TwistCommand.zero() for command in transport.commands)


def test_twist_stage_delegates_feedback_control_to_transport_and_status_never_publishes(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    published = list(transport.commands)

    adapter.status()

    assert transport.started_waypoints == [stage()]
    assert transport.commands == published


def test_twist_rejects_direct_command_as_a_public_authority_bypass(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(TwistCommand(0.1, 0.0), valid_permit(adapter, adapter_owner))

    assert transport.commands == []


def test_standard_adapter_start_requires_a_controller_owned_internal_permit():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    for forged in (None, object(), "permit"):
        with pytest.raises(AdapterError, match="PROFILE_INVALID"):
            adapter.start(stage(), forged)

    assert transport.started_waypoints == []
    assert not hasattr(adapters_package, "SafetyToken")


def test_permit_from_a_different_issuer_is_rejected(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter_owner(adapter)
    owner = _ActivationIssuer()
    foreign = _ActivationIssuer()
    adapter._bind_runtime_safety(owner)
    adapter._validate_runtime_safety("simulation")

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(stage(), foreign._issue())

    assert transport.started_waypoints == []


def test_hardware_adapter_rejects_an_unverified_emergency_channel(adapter_owner):
    transport = TwistTransport()
    transport.safety_channel = RecordingEmergencyChannel(
        transport._enqueue_emergency_stop,
        hardware_verified=False,
    )
    adapter = TwistAdapter(robot_profile(mode="hardware"), transport, clock=lambda: 0.0)
    adapter_owner(adapter)
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter._validate_runtime_safety("hardware")


def test_twist_runtime_timer_limits_first_command_acceleration_from_zero(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    commands = []
    transport.publish = commands.append

    transport._control_step()

    assert commands == [TwistCommand(0.05, 0.0)]


@pytest.mark.parametrize("permit_kind", ["missing", "invalid", "foreign"])
def test_twist_timer_without_exact_owned_permit_fails_closed(
    monkeypatch, permit_kind, adapter_owner
):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    foreign = None
    if permit_kind == "missing":
        transport._stage_permit = None
    elif permit_kind == "invalid":
        transport._stage_permit = object()
    else:
        foreign = _ActivationIssuer()
        assert foreign.start()
        transport._stage_permit = foreign.issue()
    commands = []
    transport.publish = commands.append
    try:
        transport._control_step()

        assert commands == []
        assert transport.waypoint_status() == AdapterStatus(
            "faulted", "UNSAFE_STATE"
        )
    finally:
        assert adapter.close(0.2)
        if foreign is not None:
            assert foreign.close(0.2)


def test_adapter_close_latches_before_emergency_close_and_shares_one_deadline():
    close_entered = threading.Event()
    release_close = threading.Event()

    class BlockingCloseChannel(RecordingEmergencyChannel):
        def __init__(self, enqueue) -> None:
            super().__init__(enqueue)
            self.offered: list[float] = []

        def _close(self, timeout: float) -> bool:
            self.offered.append(timeout)
            close_entered.set()
            release_close.wait(timeout)
            return False

        def finish(self) -> bool:
            return super()._close(0.2)

    class RecordingSequencer(_SafetySequencer):
        def __init__(self) -> None:
            super().__init__()
            self.offered: list[float] = []

        def close(self, timeout: float) -> bool:
            self.offered.append(timeout)
            return super().close(timeout)

    transport = TwistTransport()
    channel = BlockingCloseChannel(transport._enqueue_emergency_stop)
    transport.safety_channel = channel
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    sequencer = RecordingSequencer()
    adapter._bind_runtime_safety(sequencer)
    adapter._validate_runtime_safety("simulation")
    old_permit = sequencer.issue()
    close_results = []
    began = time.monotonic()
    closer = threading.Thread(target=lambda: close_results.append(adapter.close(0.05)))
    closer.start()
    try:
        assert close_entered.wait(0.2)
        with pytest.raises(_ActivationRejected):
            sequencer.issue()
        invoked = []
        with pytest.raises(_ActivationRejected):
            sequencer.submit(old_permit, lambda: invoked.append(True), timeout=0.02)
        assert invoked == []

        closer.join(0.2)
        assert not closer.is_alive()
        assert time.monotonic() - began < 0.2
        assert close_results == [False]
        assert len(channel.offered) == 1
        assert len(sequencer.offered) == 1
        assert sum(channel.offered + sequencer.offered) <= 0.055
    finally:
        release_close.set()
        closer.join(0.2)
        assert channel.finish()


def test_twist_timer_snapshot_before_estop_cannot_publish_nonzero_after_estop_returns(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    commands = []
    transport.publish = commands.append
    snapshot = threading.Event()
    release = threading.Event()
    transport._before_publish = lambda: (snapshot.set(), release.wait(1.0))
    worker = threading.Thread(target=transport._control_step)
    worker.start()
    assert snapshot.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert _wait_until(lambda: bool(commands))
    assert commands
    first_zero = commands.index(TwistCommand.zero())
    assert all(command == TwistCommand.zero() for command in commands[first_zero:])


def test_twist_emergency_enqueue_never_waits_for_a_blocked_ros_publish(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    valid_permit(adapter, adapter_owner)
    entered = threading.Event()
    release = threading.Event()
    errors = []

    def blocked_publish(_message):
        entered.set()
        release.wait()

    transport._publisher.publish = blocked_publish
    worker = threading.Thread(target=lambda: _capture(errors, adapter._emergency_stop))
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    release.set()
    worker.join(1.0)

    assert not blocked
    assert not errors


def test_twist_estop_success_waits_for_blocked_nonzero_publish(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    entered = threading.Event()
    release = threading.Event()
    commands = []

    def publish(command):
        if command != TwistCommand.zero():
            entered.set()
            release.wait()
        commands.append(command)

    transport.publish = publish
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert entered.wait(0.2)
        results = []
        stop = threading.Thread(
            target=lambda: results.append(adapter._emergency_stop(0.2))
        )
        stop.start()
        assert stop.is_alive()
        release.set()
        stop.join(0.2)
        timer.join(0.2)

        assert not stop.is_alive()
        assert not timer.is_alive()
        assert results == [EmergencyStopResult(True, True, True, "ESTOP_LATCHED")]
        assert any(command != TwistCommand.zero() for command in commands)
    finally:
        release.set()
        timer.join(0.2)
        assert adapter.close(0.2)


def test_twist_estop_degrades_when_publish_does_not_quiesce(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    entered = threading.Event()
    release = threading.Event()

    def publish(command):
        if command != TwistCommand.zero():
            entered.set()
            release.wait()

    transport.publish = publish
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert entered.wait(0.2)
        began = time.monotonic()

        result = adapter._emergency_stop(0.02)

        assert time.monotonic() - began < 0.2
        assert result == EmergencyStopResult(
            True, False, True, "TRANSPORT_UNQUIESCED"
        )
    finally:
        release.set()
        timer.join(0.2)
        assert not timer.is_alive()
        assert adapter.close(0.2)


class Nav2Transport:
    def __init__(self) -> None:
        self.requests = []
        self.cancel_count = 0
        self.zeros = 0
        self.estop_handler = None
        self.estop_cancelled = False
        self.safety_channel = RecordingEmergencyChannel(self._enqueue_emergency_stop)

    def preflight_activation(self) -> bool:
        return True

    def prepare_goal(self, request):
        return request

    def send_goal(self, goal, _permit=None) -> None:
        self.requests.append(goal)

    def track_goal(self, _future, _permit) -> None:
        return None

    def goal_status(self):
        return {"state": "running"}

    def cancel_goal(self) -> None:
        self.cancel_count += 1

    def publish_zero(self) -> None:
        self.zeros += 1

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler

    def _enqueue_emergency_stop(self):
        self.zeros += 1
        if not self.estop_cancelled:
            self.cancel_count += 1
            self.estop_cancelled = True

    def emergency_channel(self):
        return self.safety_channel


def test_nav2_emits_fixed_action_type_with_structured_goal_values(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(NavigationGoal(frame="map", x=1.25, y=-2.5, yaw=0.75), valid_permit(adapter, adapter_owner))

    request = transport.requests[0]
    assert request.action_type == "nav2_msgs/action/NavigateToPose"
    assert request.action_name == "/navigate_to_pose"
    assert (request.frame, request.x, request.y, request.yaw) == ("map", 1.25, -2.5, 0.75)


def test_nav2_emergency_stop_publishes_independent_zero_and_initiates_cancel(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter, adapter_owner)

    adapter._emergency_stop()

    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1
    assert transport.cancel_count == 1

    adapter._emergency_stop()

    assert _wait_until(lambda: transport.zeros == 2)
    assert transport.zeros == 2
    assert transport.cancel_count == 1


def test_nav2_estop_success_waits_for_send_goal_boundary(adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class BlockingNav2Transport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            entered.set()
            release.wait()
            return super().send_goal(goal, _permit)

    transport = BlockingNav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    starter = threading.Thread(
        target=lambda: _capture(errors, adapter.start, stage(), permit)
    )
    starter.start()
    try:
        assert entered.wait(0.2)
        results = []
        stop = threading.Thread(
            target=lambda: results.append(adapter._emergency_stop(0.2))
        )
        stop.start()
        assert stop.is_alive()
        release.set()
        stop.join(0.2)
        starter.join(0.2)

        assert not stop.is_alive()
        assert not starter.is_alive()
        assert results == [EmergencyStopResult(True, True, True, "ESTOP_LATCHED")]
        assert any(isinstance(error, AdapterError) for error in errors)
    finally:
        release.set()
        starter.join(0.2)
        assert adapter.close(0.2)


def test_nav2_estop_degrades_when_send_goal_does_not_quiesce(adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class BlockingNav2Transport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            entered.set()
            release.wait()
            return super().send_goal(goal, _permit)

    transport = BlockingNav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    starter = threading.Thread(
        target=lambda: _capture(errors, adapter.start, stage(), permit)
    )
    starter.start()
    try:
        assert entered.wait(0.2)
        began = time.monotonic()

        result = adapter._emergency_stop(0.02)

        assert time.monotonic() - began < 0.2
        assert result == EmergencyStopResult(
            True, False, True, "TRANSPORT_UNQUIESCED"
        )
    finally:
        release.set()
        starter.join(0.2)
        assert not starter.is_alive()
        assert adapter.close(0.2)


def test_no_stale_timer_publish_after_successful_estop(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    snapshot = threading.Event()
    release = threading.Event()
    commands = []
    transport.publish = commands.append
    transport._before_publish = lambda: (snapshot.set(), release.wait())
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert snapshot.wait(0.2)

        result = adapter._emergency_stop(0.2)
        release.set()
        timer.join(0.2)

        assert result.successful
        assert not timer.is_alive()
        assert _wait_until(lambda: bool(commands), timeout=0.2)
        assert commands == [TwistCommand.zero()]
    finally:
        release.set()
        timer.join(0.2)
        assert adapter.close(0.2)


def test_late_nav2_goal_response_is_best_effort_cancelled(monkeypatch, adapter_owner):
    class Handle:
        accepted = True

        def __init__(self) -> None:
            self.cancel_count = 0
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()

        def cancel_goal_async(self):
            self.cancel_count += 1
            return self.cancel_future

        def get_result_async(self):
            return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    try:
        result = adapter._emergency_stop(0.2)
        handle = Handle()

        transport._client.goal_future.resolve(handle)

        assert result.successful
        assert handle.cancel_count == 1
        assert transport.goal_status() == {"state": "cancelling"}
    finally:
        assert adapter.close(0.2)


def test_nav2_estop_between_reservation_and_goal_enqueue_rejects_late_start(adapter_owner):
    entered = threading.Event()
    release = threading.Event()
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    adapter._before_activation = lambda: (entered.set(), release.wait(1.0))
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, stage(), permit),
    )
    worker.start()
    assert entered.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert transport.requests == []
    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def test_nav2_goal_callback_registration_happens_after_atomic_enqueue_lock(adapter_owner):
    class ImmediateCompletionTransport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            self.requests.append(goal)
            return object()

        def track_goal(self, _future, _permit):
            self.safety_channel._stop()

    transport = ImmediateCompletionTransport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)

    with pytest.raises(AdapterError, match="ESTOP_LATCHED"):
        adapter.start(stage(), permit)

    assert len(transport.requests) == 1
    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1


def test_nav2_maps_a_reviewed_task_stage_to_the_fixed_goal_shape(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(stage(), valid_permit(adapter, adapter_owner))

    request = transport.requests[0]
    assert (request.frame, request.x, request.y, request.yaw) == ("odom", 2.0, 2.0, 0.0)


def test_nav2_cancellation_calls_cancel_once_then_sends_a_zero_burst():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.cancel()

    assert transport.cancel_count == 1
    assert transport.zeros == 3


def test_nav2_repeated_fail_closed_stop_cancels_a_pending_goal_only_once(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(NavigationGoal(frame="map", x=1.0, y=0.0, yaw=0.0), valid_permit(adapter, adapter_owner))

    adapter.stop()
    adapter.stop()

    assert transport.cancel_count == 1
    assert transport.zeros == 6


def test_adapter_selection_rejects_unknown_or_unimplemented_kinds():
    profile = SimpleNamespace(adapter=SimpleNamespace(kind="raw_shell"))

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        create_adapter(profile, object())


def test_adapter_physical_estop_subscription_is_wired_to_runtime_handler():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport)
    assertions: list[bool] = []

    adapter.bind_physical_estop(assertions.append)
    transport.estop_handler(True)

    assert assertions == [True]


def test_hospital_adapter_accepts_only_fixed_actions_on_owned_simulation_runtime(adapter_owner):
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    assert adapter.start(HospitalAction.START, valid_permit(adapter, adapter_owner)).state == "running"
    assert runtime.commands == (HospitalAction.START,)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start("start", valid_permit(adapter, adapter_owner))


def test_hospital_adapter_rejects_an_arbitrary_callable_runner_at_construction():
    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        HospitalDeliveryAdapter(lambda _action: {"state": "running"})


def test_hospital_simulation_rejects_late_start_after_permit_invalidation(adapter_owner):
    entered = threading.Event()
    release = threading.Event()
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    permit = valid_permit(adapter, adapter_owner)
    adapter._before_activation = lambda: (entered.set(), release.wait(1.0))
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, HospitalAction.START, permit),
    )
    worker.start()
    assert entered.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert HospitalAction.START not in runtime.commands
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def test_hospital_start_never_waits_then_dispatches_late_behind_runtime_lock(adapter_owner):
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    runtime._lock.acquire()
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, HospitalAction.START, permit),
    )
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    adapter._emergency_stop()
    runtime._lock.release()
    worker.join(1.0)

    assert not blocked
    assert HospitalAction.START not in runtime.commands
    assert any(isinstance(error, AdapterError) and error.code == "INTERNAL_ERROR" for error in errors)


def _capture(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_nav2_late_goal_acceptance_is_cancelled_without_returning_to_running(monkeypatch, adapter_owner):
    class Handle:
        accepted = True

        def __init__(self) -> None:
            self.cancel_count = 0
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()

        def cancel_goal_async(self):
            self.cancel_count += 1
            return self.cancel_future

        def get_result_async(self):
            return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport.cancel_goal()
    handle = Handle()

    transport._client.goal_future.resolve(handle)

    assert handle.cancel_count == 1
    assert transport.goal_status() == {"state": "cancelling"}
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    assert transport.goal_status() == {"state": "cancelling"}
    handle.result_future.resolve(type("Result", (), {"status": 5})())
    assert transport.goal_status() == {"state": "cancelled"}


@pytest.mark.parametrize("mode", ["rejected", "exception"])
def test_nav2_cancel_rejection_or_exception_faults_instead_of_claiming_stopped(mode, monkeypatch, adapter_owner):
    class Future:
        def add_done_callback(self, callback): self.callback = callback
        def result(self):
            if mode == "exception": raise RuntimeError("raw")
            return type("Response", (), {"goals_canceling": []})()
    class Handle:
        accepted = True
        def __init__(self): self.future = Future()
        def cancel_goal_async(self): return self.future
        def get_result_async(self): return type("ResultFuture", (), {"add_done_callback": lambda self, cb: None})()

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.future.callback(handle.future)

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_faults_while_confirmation_is_missing(monkeypatch, adapter_owner):
    now = [0.0]
    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport.cancel_goal()
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_remains_active_after_cancel_acceptance_until_terminal_result(monkeypatch, adapter_owner):
    now = [0.0]

    class Handle:
        accepted = True
        def __init__(self):
            self.cancel_future = CallbackFuture()
        def cancel_goal_async(self): return self.cancel_future
        def get_result_async(self): return CallbackFuture()

    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_status_exception_is_stable_and_does_not_recurse_through_normal_stop(adapter_owner):
    transport = Nav2Transport()
    transport.goal_status = lambda: (_ for _ in ()).throw(RuntimeError("raw"))
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter, adapter_owner)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1


def test_nav2_cancelled_goal_result_exception_is_faulted_not_cancelled(monkeypatch, adapter_owner):
    class Handle:
        accepted = True
        def __init__(self):
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()
        def cancel_goal_async(self): return self.cancel_future
        def get_result_async(self): return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.result_future.reject(RuntimeError("raw callback"))

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_emergency_enqueue_never_waits_for_blocked_action_cancel(monkeypatch, adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class Handle:
        accepted = True

        def cancel_goal_async(self):
            entered.set()
            release.wait()
            return CallbackFuture()

        def get_result_async(self):
            return CallbackFuture()

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport._client.goal_future.resolve(Handle())
    errors = []
    worker = threading.Thread(target=lambda: _capture(errors, adapter._emergency_stop))
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    release.set()
    worker.join(1.0)

    assert not blocked
    assert not errors
