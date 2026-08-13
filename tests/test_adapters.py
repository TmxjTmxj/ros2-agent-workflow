from __future__ import annotations

import json
import sys
import threading
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
from agent_ros.adapters.hospital import HospitalDeliveryAdapter
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.profiles.models import RobotProfile
from agent_ros.profiles.models import PoseGoal, TaskStage


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


def valid_permit(adapter):
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)
    adapter._validate_runtime_safety("simulation")
    return issuer._issue()


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


def test_twist_start_accepts_only_a_reviewed_stage():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    adapter.start(stage(), valid_permit(adapter))

    assert transport.started_waypoints == [stage()]


def test_twist_emergency_stop_is_synchronous_idempotent_and_zero_before_return():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    valid_permit(adapter)

    adapter._emergency_stop()
    adapter._emergency_stop()

    assert transport.commands == [TwistCommand.zero()] * 2


def test_twist_stale_odometry_stops_with_a_zero_burst_and_reports_stable_code():
    now = [0.0]
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0], stale_after=1.0)
    adapter.start(stage(), valid_permit(adapter))
    now[0] = 1.01
    transport.waypoint_status = lambda: (_ for _ in ()).throw(AdapterError("STALE_FEEDBACK"))

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert transport.commands[-3:] == [TwistCommand.zero()] * 3


@pytest.mark.parametrize("timestamp", [-2.0, 0.2])
def test_twist_stage_refuses_stale_or_future_odometry_before_any_nonzero_motion(timestamp):
    transport = TwistTransport()
    transport.odometry = OdometrySample(timestamp=timestamp, x=1.0, y=2.0, yaw=0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0, stale_after=1.0, future_skew=0.05)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.start(stage(), valid_permit(adapter))

    assert all(command == TwistCommand.zero() for command in transport.commands)


def test_twist_stage_delegates_feedback_control_to_transport_and_status_never_publishes():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter))
    published = list(transport.commands)

    adapter.status()

    assert transport.started_waypoints == [stage()]
    assert transport.commands == published


def test_twist_rejects_direct_command_as_a_public_authority_bypass():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(TwistCommand(0.1, 0.0), valid_permit(adapter))

    assert transport.commands == []


def test_standard_adapter_start_requires_a_controller_owned_internal_permit():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    for forged in (None, object(), "permit"):
        with pytest.raises(AdapterError, match="PROFILE_INVALID"):
            adapter.start(stage(), forged)

    assert transport.started_waypoints == []
    assert not hasattr(adapters_package, "SafetyToken")


def test_permit_from_a_different_issuer_is_rejected():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    owner = _ActivationIssuer()
    foreign = _ActivationIssuer()
    adapter._bind_runtime_safety(owner)
    adapter._validate_runtime_safety("simulation")

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(stage(), foreign._issue())

    assert transport.started_waypoints == []


def test_hardware_adapter_rejects_an_unverified_emergency_channel():
    transport = TwistTransport()
    transport.safety_channel = RecordingEmergencyChannel(
        transport._enqueue_emergency_stop,
        hardware_verified=False,
    )
    adapter = TwistAdapter(robot_profile(mode="hardware"), transport, clock=lambda: 0.0)
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter._validate_runtime_safety("hardware")


def test_twist_runtime_timer_limits_first_command_acceleration_from_zero(monkeypatch):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter))
    commands = []
    transport.publish = commands.append

    transport._control_step()

    assert commands == [TwistCommand(0.05, 0.0)]


def test_twist_timer_snapshot_before_estop_cannot_publish_nonzero_after_estop_returns(monkeypatch):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter))
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

    assert commands
    first_zero = commands.index(TwistCommand.zero())
    assert all(command == TwistCommand.zero() for command in commands[first_zero:])


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


def test_nav2_emits_fixed_action_type_with_structured_goal_values():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(NavigationGoal(frame="map", x=1.25, y=-2.5, yaw=0.75), valid_permit(adapter))

    request = transport.requests[0]
    assert request.action_type == "nav2_msgs/action/NavigateToPose"
    assert request.action_name == "/navigate_to_pose"
    assert (request.frame, request.x, request.y, request.yaw) == ("map", 1.25, -2.5, 0.75)


def test_nav2_emergency_stop_publishes_independent_zero_and_initiates_cancel():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter)

    adapter._emergency_stop()

    assert transport.zeros == 1
    assert transport.cancel_count == 1

    adapter._emergency_stop()

    assert transport.zeros == 2
    assert transport.cancel_count == 1


def test_nav2_estop_between_reservation_and_goal_enqueue_rejects_late_start():
    entered = threading.Event()
    release = threading.Event()
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter)
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
    assert transport.zeros == 1
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def test_nav2_goal_callback_registration_happens_after_atomic_enqueue_lock():
    class ImmediateCompletionTransport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            self.requests.append(goal)
            return object()

        def track_goal(self, _future, _permit):
            self.safety_channel._stop()

    transport = ImmediateCompletionTransport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter)

    with pytest.raises(AdapterError, match="ESTOP_LATCHED"):
        adapter.start(stage(), permit)

    assert len(transport.requests) == 1
    assert transport.zeros == 1


def test_nav2_maps_a_reviewed_task_stage_to_the_fixed_goal_shape():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(stage(), valid_permit(adapter))

    request = transport.requests[0]
    assert (request.frame, request.x, request.y, request.yaw) == ("odom", 2.0, 2.0, 0.0)


def test_nav2_cancellation_calls_cancel_once_then_sends_a_zero_burst():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.cancel()

    assert transport.cancel_count == 1
    assert transport.zeros == 3


def test_nav2_repeated_fail_closed_stop_cancels_a_pending_goal_only_once():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(NavigationGoal(frame="map", x=1.0, y=0.0, yaw=0.0), valid_permit(adapter))

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


def test_hospital_adapter_accepts_only_fixed_action_enums_and_exactly_one_json_object():
    actions: list[HospitalAction] = []

    def runner(action: HospitalAction) -> str:
        actions.append(action)
        return json.dumps({"state": "running"})

    adapter = HospitalDeliveryAdapter(runner)
    assert adapter.start(HospitalAction.START, valid_permit(adapter)).state == "running"
    assert actions == [HospitalAction.START]

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start("start", valid_permit(adapter))

    broken = HospitalDeliveryAdapter(lambda _action: '{}\n{"second":true}')
    with pytest.raises(AdapterError, match="INTERNAL_ERROR"):
        broken.status()


def test_hospital_simulation_rejects_late_start_after_permit_invalidation():
    entered = threading.Event()
    release = threading.Event()
    actions = []
    adapter = HospitalDeliveryAdapter(lambda action: actions.append(action) or {"state": "running"})
    permit = valid_permit(adapter)
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

    assert actions == []
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def _capture(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_nav2_late_goal_acceptance_is_cancelled_without_returning_to_running(monkeypatch):
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
    adapter.start(stage(), valid_permit(adapter))
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
def test_nav2_cancel_rejection_or_exception_faults_instead_of_claiming_stopped(mode, monkeypatch):
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
    adapter.start(stage(), valid_permit(adapter))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.future.callback(handle.future)

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_faults_while_confirmation_is_missing(monkeypatch):
    now = [0.0]
    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter))
    transport.cancel_goal()
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_remains_active_after_cancel_acceptance_until_terminal_result(monkeypatch):
    now = [0.0]

    class Handle:
        accepted = True
        def __init__(self):
            self.cancel_future = CallbackFuture()
        def cancel_goal_async(self): return self.cancel_future
        def get_result_async(self): return CallbackFuture()

    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_status_exception_is_stable_and_does_not_recurse_through_normal_stop():
    transport = Nav2Transport()
    transport.goal_status = lambda: (_ for _ in ()).throw(RuntimeError("raw"))
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert transport.zeros == 1


def test_nav2_cancelled_goal_result_exception_is_faulted_not_cancelled(monkeypatch):
    class Handle:
        accepted = True
        def __init__(self):
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()
        def cancel_goal_async(self): return self.cancel_future
        def get_result_async(self): return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.result_future.reject(RuntimeError("raw callback"))

    assert transport.goal_status() == {"state": "faulted"}
