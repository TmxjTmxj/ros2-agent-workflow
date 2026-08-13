from __future__ import annotations

import json
import threading

import pytest

from agent_ros.adapters.base import (
    AdapterError,
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

    def publish(self, command: TwistCommand) -> None:
        self.commands.append(command)

    def read_odometry(self) -> OdometrySample:
        return self.odometry

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler


def stage(*, timeout: float = 30.0) -> TaskStage:
    return TaskStage("destination", PoseGoal("odom", 2.0, 2.0, 0.0), 0.1, timeout)


def test_twist_start_clamps_command_to_reviewed_profile_limits():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    adapter.start(TwistCommand(linear_velocity=5.0, angular_velocity=-3.0))

    assert transport.commands == [TwistCommand(linear_velocity=0.5, angular_velocity=-1.0)]


def test_twist_stale_odometry_stops_with_a_zero_burst_and_reports_stable_code():
    now = [0.0]
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0], stale_after=1.0)
    adapter.start(TwistCommand(linear_velocity=0.2, angular_velocity=0.0))
    now[0] = 1.01

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


def test_twist_stage_maps_goal_to_bounded_command_and_limits_acceleration_between_updates():
    now = [0.0]
    transport = TwistTransport()
    transport.odometry = OdometrySample(timestamp=0.0, x=1.0, y=2.0, yaw=0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0])
    adapter.start(stage())
    first = transport.commands[-1]
    now[0] = 0.1
    transport.odometry = OdometrySample(timestamp=0.1, x=1.0, y=2.0, yaw=3.0)

    adapter.status()

    second = transport.commands[-1]
    assert abs(second.linear_velocity - first.linear_velocity) <= 0.05 + 1e-9
    assert abs(second.angular_velocity - first.angular_velocity) <= 0.1 + 1e-9


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

        def cancel_goal_async(self):
            self.cancel_count += 1

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
    assert transport.goal_status() == {"state": "cancelled"}
