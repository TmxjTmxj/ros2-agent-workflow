"""Structured Nav2 NavigateToPose adapter."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    NavigationGoal,
    Observation,
    RobotAdapter,
)
from agent_ros.profiles.models import NAVIGATE_TO_POSE_TYPE, RobotProfile


_ZERO_BURST_COUNT = 3


@dataclass(frozen=True, slots=True)
class NavigateToPoseRequest:
    action_name: str
    action_type: str
    frame: str
    x: float
    y: float
    yaw: float


class Nav2Transport(Protocol):
    def send_goal(self, request: NavigateToPoseRequest) -> None: ...

    def goal_status(self) -> object: ...

    def cancel_goal(self) -> None: ...

    def publish_zero(self) -> None: ...

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None: ...


class Nav2Adapter(RobotAdapter):
    def __init__(self, profile: RobotProfile, transport: Nav2Transport) -> None:
        self._profile = profile
        self._transport = transport
        self._state = "idle"
        self.validate()

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("navigation.nav2",))

    def validate(self) -> None:
        interface = self._profile.interfaces.navigation
        if (
            self._profile.adapter.kind != "nav2"
            or interface is None
            or interface.type != NAVIGATE_TO_POSE_TYPE
            or interface.action is None
        ):
            raise AdapterError("PROFILE_INVALID")

    def start(self, task: object) -> AdapterStatus:
        if not isinstance(task, NavigationGoal):
            raise AdapterError("PROFILE_INVALID")
        interface = self._profile.interfaces.navigation
        assert interface is not None and interface.action is not None
        request = NavigateToPoseRequest(
            action_name=interface.action,
            action_type=NAVIGATE_TO_POSE_TYPE,
            frame=task.frame,
            x=task.x,
            y=task.y,
            yaw=task.yaw,
        )
        try:
            self._transport.send_goal(request)
        except Exception:
            self.stop()
            raise AdapterError("INTERNAL_ERROR") from None
        self._state = "running"
        return AdapterStatus(self._state)

    def status(self) -> AdapterStatus:
        try:
            raw = self._transport.goal_status()
        except Exception:
            self.stop()
            raise AdapterError("STALE_FEEDBACK") from None
        if isinstance(raw, AdapterStatus):
            return raw
        if isinstance(raw, dict) and isinstance(raw.get("state"), str):
            return AdapterStatus(raw["state"])
        raise AdapterError("STALE_FEEDBACK")

    def cancel(self) -> AdapterStatus:
        try:
            self._transport.cancel_goal()
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        finally:
            self.stop()
        self._state = "cancelled"
        return AdapterStatus(self._state)

    def stop(self) -> None:
        for _ in range(_ZERO_BURST_COUNT):
            try:
                self._transport.publish_zero()
            except Exception:
                continue
        self._state = "stopped"

    def observe(self, source: str) -> Observation:
        if source not in self._profile.observation_sources:
            raise AdapterError("PROFILE_INVALID")
        raise AdapterError("STALE_FEEDBACK")

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> None:
        self._transport.subscribe_estop(handler)


class RclpyNav2Transport:
    """Typed rclpy ActionClient transport with no caller-provided action type."""

    def __init__(self, node, action_name: str, command_topic: str, estop_topic: str) -> None:
        try:
            from geometry_msgs.msg import Twist
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from std_msgs.msg import Bool
        except ImportError as exc:
            raise AdapterError("PROFILE_INVALID") from exc
        self._node = node
        self._action_type = NavigateToPose
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._publisher = node.create_publisher(Twist, command_topic, 10)
        self._twist_type = Twist
        self._goal_handle = None
        self._state = "idle"
        self._estop_handlers: list[Callable[[bool], None]] = []

        def estop_callback(message) -> None:
            for handler in tuple(self._estop_handlers):
                handler(bool(message.data))

        self._estop_subscription = node.create_subscription(Bool, estop_topic, estop_callback, 10)

    def send_goal(self, request: NavigateToPoseRequest) -> None:
        goal = self._action_type.Goal()
        goal.pose.header.frame_id = request.frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = request.x
        goal.pose.pose.position.y = request.y
        goal.pose.pose.orientation.z = math.sin(request.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(request.yaw / 2.0)
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self._state = "pending"

    def _goal_response(self, future) -> None:
        self._goal_handle = future.result()
        self._state = "running" if self._goal_handle.accepted else "rejected"

    def goal_status(self) -> object:
        return {"state": self._state}

    def cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def publish_zero(self) -> None:
        self._publisher.publish(self._twist_type())

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None:
        self._estop_handlers.append(handler)
