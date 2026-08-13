from __future__ import annotations

import json
import threading

import pytest

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


def robot_profile(kind: str = "twist") -> RobotProfile:
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
        "mode": "simulation",
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


class TwistTransport:
    def __init__(self) -> None:
        self.commands: list[TwistCommand] = []
        self.odometry = OdometrySample(timestamp=0.0, x=1.0, y=2.0, yaw=0.25)
        self.estop_handler = None
        self.started_waypoints = []
        self.state = AdapterStatus("idle")

    def publish(self, command: TwistCommand) -> None:
        self.commands.append(command)

    def read_odometry(self) -> OdometrySample:
        return self.odometry

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler

    def start_waypoint(self, stage) -> None:
        self.started_waypoints.append(stage)
        self.state = AdapterStatus("running")

    def waypoint_status(self):
        return self.state

    def cancel_waypoint(self):
        self.state = AdapterStatus("cancelled")

    def stop_waypoint(self):
        self.commands.extend([TwistCommand.zero()] * 3)
        self.state = AdapterStatus("stopped")


def stage(*, timeout: float = 30.0) -> TaskStage:
    return TaskStage("destination", PoseGoal("odom", 2.0, 2.0, 0.0), 0.1, timeout)


def test_twist_start_accepts_only_a_reviewed_stage():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    adapter.start(stage())

    assert transport.started_waypoints == [stage()]


def test_twist_stale_odometry_stops_with_a_zero_burst_and_reports_stable_code():
    now = [0.0]
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0], stale_after=1.0)
    adapter.start(stage())
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
        adapter.start(stage())

    assert all(command == TwistCommand.zero() for command in transport.commands)


def test_twist_stage_delegates_feedback_control_to_transport_and_status_never_publishes():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage())
    published = list(transport.commands)

    adapter.status()

    assert transport.started_waypoints == [stage()]
    assert transport.commands == published


def test_twist_rejects_direct_command_as_a_public_authority_bypass():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(TwistCommand(0.1, 0.0))

    assert transport.commands == []


def test_twist_runtime_timer_limits_first_command_acceleration_from_zero():
    from agent_ros.adapters.twist import RclpyTwistTransport

    transport = object.__new__(RclpyTwistTransport)
    transport._stage = stage()
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    transport._clock = lambda: 0.0
    transport._stale_after = 1.0
    transport._limits = robot_profile().limits
    transport._period = 0.1
    transport._last_command = TwistCommand.zero()
    transport._state = AdapterStatus("running")
    commands = []
    transport.publish = commands.append

    transport._control_step()

    assert commands == [TwistCommand(0.05, 0.0)]


class Nav2Transport:
    def __init__(self) -> None:
        self.requests = []
        self.cancel_count = 0
        self.zeros = 0
        self.estop_handler = None

    def send_goal(self, request) -> None:
        self.requests.append(request)

    def goal_status(self):
        return {"state": "running"}

    def cancel_goal(self) -> None:
        self.cancel_count += 1

    def publish_zero(self) -> None:
        self.zeros += 1

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler


def test_nav2_emits_fixed_action_type_with_structured_goal_values():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(NavigationGoal(frame="map", x=1.25, y=-2.5, yaw=0.75))

    request = transport.requests[0]
    assert request.action_type == "nav2_msgs/action/NavigateToPose"
    assert request.action_name == "/navigate_to_pose"
    assert (request.frame, request.x, request.y, request.yaw) == ("map", 1.25, -2.5, 0.75)


def test_nav2_maps_a_reviewed_task_stage_to_the_fixed_goal_shape():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(stage())

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
    adapter.start(NavigationGoal(frame="map", x=1.0, y=0.0, yaw=0.0))

    adapter.stop()
    adapter.stop()

    assert transport.cancel_count == 1
    assert transport.zeros == 6


def test_adapter_selection_rejects_unknown_or_unimplemented_kinds():
    profile = object.__new__(RobotProfile)
    object.__setattr__(profile, "adapter", type("Config", (), {"kind": "raw_shell"})())

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
    assert adapter.start(HospitalAction.START).state == "running"
    assert actions == [HospitalAction.START]

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start("start")

    broken = HospitalDeliveryAdapter(lambda _action: '{}\n{"second":true}')
    with pytest.raises(AdapterError, match="INTERNAL_ERROR"):
        broken.status()


def test_nav2_late_goal_acceptance_is_cancelled_without_returning_to_running():
    from agent_ros.adapters.nav2 import RclpyNav2Transport

    class Handle:
        accepted = True

        def __init__(self) -> None:
            self.cancel_count = 0
            self.cancel_future = CallbackFuture()

        def cancel_goal_async(self):
            self.cancel_count += 1
            return self.cancel_future

    class CallbackFuture:
        def __init__(self):
            self.callback = None
            self.value = None
            self.error = None
        def add_done_callback(self, callback): self.callback = callback
        def result(self):
            if self.error: raise self.error
            return self.value
        def resolve(self, value):
            self.value = value
            self.callback(self)

    class Future:
        def __init__(self, handle) -> None:
            self.handle = handle

        def result(self):
            return self.handle

    transport = object.__new__(RclpyNav2Transport)
    transport._goal_handle = None
    transport._state = "pending"
    transport._cancel_requested = False
    transport._lock = threading.RLock()
    transport.cancel_goal()
    handle = Handle()

    transport._goal_response(Future(handle))

    assert handle.cancel_count == 1
    assert transport.goal_status() == {"state": "cancelling"}
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    assert transport.goal_status() == {"state": "cancelled"}


@pytest.mark.parametrize("mode", ["rejected", "exception"])
def test_nav2_cancel_rejection_or_exception_faults_instead_of_claiming_stopped(mode):
    from agent_ros.adapters.nav2 import RclpyNav2Transport

    class Future:
        def add_done_callback(self, callback): self.callback = callback
        def result(self):
            if mode == "exception": raise RuntimeError("raw")
            return type("Response", (), {"goals_canceling": []})()
    class Handle:
        accepted = True
        def __init__(self): self.future = Future()
        def cancel_goal_async(self): return self.future

    transport = object.__new__(RclpyNav2Transport)
    transport._goal_handle = Handle()
    transport._state = "running"
    transport._cancel_requested = False
    transport._lock = threading.RLock()

    transport.cancel_goal()
    transport._goal_handle.future.callback(transport._goal_handle.future)

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_faults_while_confirmation_is_missing():
    from agent_ros.adapters.nav2 import RclpyNav2Transport
    now = [0.0]
    transport = object.__new__(RclpyNav2Transport)
    transport._goal_handle = None
    transport._state = "pending"
    transport._cancel_requested = False
    transport._lock = threading.RLock()
    transport._clock = lambda: now[0]
    transport._cancel_timeout = 0.1
    transport._cancel_deadline = None
    transport.cancel_goal()
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}
