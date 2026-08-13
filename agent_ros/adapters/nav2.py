"""Structured Nav2 NavigateToPose adapter."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from agent_ros.adapters._safety import _ActivationPermit, _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    NavigationGoal,
    Observation,
    RobotAdapter,
)
from agent_ros.profiles.models import NAVIGATE_TO_POSE_TYPE, RobotProfile, TaskStage


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
    def preflight_activation(self) -> bool: ...

    def prepare_goal(self, request: NavigateToPoseRequest) -> object: ...

    def send_goal(self, goal: object, activation_permit: object = None) -> object: ...

    def track_goal(self, future: object, activation_permit: object) -> None: ...

    def goal_status(self) -> object: ...

    def cancel_goal(self) -> None: ...

    def publish_zero(self) -> None: ...

    def emergency_channel(self) -> _EmergencyStopChannel: ...

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None: ...


class Nav2Adapter(RobotAdapter):
    def __init__(self, profile: RobotProfile, transport: Nav2Transport) -> None:
        self._profile = profile
        self._transport = transport
        self._state = "idle"
        self._cancel_sent = False
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
        try:
            if self._transport.preflight_activation() is not True:
                raise AdapterError("PROFILE_INVALID")
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        if isinstance(task, TaskStage):
            task = NavigationGoal(task.goal.frame, task.goal.x, task.goal.y, task.goal.yaw)
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
            goal = self._transport.prepare_goal(request)
            future = self._activate_start(
                activation_permit,
                lambda: self._transport.send_goal(goal, activation_permit),
            )
            self._transport.track_goal(future, activation_permit)
            if not self._permit_is_current(activation_permit):
                raise AdapterError("ESTOP_LATCHED")
            self._cancel_sent = False
        except AdapterError:
            raise
        except Exception:
            try:
                self._emergency_stop()
            except AdapterError:
                pass
            raise AdapterError("INTERNAL_ERROR") from None
        self._state = "running"
        return AdapterStatus(self._state)

    def status(self) -> AdapterStatus:
        try:
            raw = self._transport.goal_status()
        except Exception:
            self._state = "faulted"
            try:
                self._emergency_stop()
            except AdapterError:
                pass
            raise AdapterError("STALE_FEEDBACK") from None
        if isinstance(raw, AdapterStatus):
            if raw.state == "faulted":
                self._cancel_sent = False
            return raw
        if isinstance(raw, dict) and isinstance(raw.get("state"), str):
            if raw["state"] == "faulted":
                self._cancel_sent = False
            return AdapterStatus(raw["state"])
        raise AdapterError("STALE_FEEDBACK")

    def cancel(self) -> AdapterStatus:
        self.stop()
        return self.status()

    def stop(self) -> None:
        cancel_error = False
        if not self._cancel_sent:
            try:
                self._transport.cancel_goal()
            except Exception:
                cancel_error = True
            self._cancel_sent = True
        for _ in range(_ZERO_BURST_COUNT):
            try:
                self._transport.publish_zero()
            except Exception:
                continue
        if cancel_error:
            self._state = "faulted"
            raise AdapterError("UNSAFE_STATE")
        try:
            state = self.status().state
        except AdapterError:
            self._state = "faulted"
            raise
        self._state = "cancelled" if state == "cancelled" else ("faulted" if state == "faulted" else "cancelling")

    def observe(self, source: str) -> Observation:
        if source not in self._profile.observation_sources:
            raise AdapterError("PROFILE_INVALID")
        raise AdapterError("STALE_FEEDBACK")

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        self._transport.subscribe_estop(handler)
        return True

    def _emergency_stop_channel(self):
        try:
            return self._transport.emergency_channel()
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None


class RclpyNav2Transport:
    """Typed rclpy ActionClient transport with no caller-provided action type."""

    def __init__(self, node, action_name: str, command_topic: str, estop_topic: str, *, cancel_timeout: float = 1.0, clock: Callable[[], float] = time.monotonic) -> None:
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
        self._cancel_requested = False
        import threading
        self._lock = threading.RLock()
        self._clock = clock
        self._cancel_timeout = cancel_timeout
        self._cancel_deadline = None
        self._cancel_generation = 0
        self._goal_generation = 0
        self._goal_permit: _ActivationPermit | None = None
        self._emergency_latched = False
        self._cancel_enqueued_for: tuple[int, int] | None = None
        self._estop_handlers: list[Callable[[bool], None]] = []
        self._safety_channel = _RclpyNav2EmergencyChannel(self)

        def estop_callback(message) -> None:
            for handler in tuple(self._estop_handlers):
                handler(bool(message.data))

        self._estop_subscription = node.create_subscription(Bool, estop_topic, estop_callback, 10)

    def preflight_activation(self) -> bool:
        return callable(getattr(self._client, "send_goal_async", None)) and callable(
            getattr(self._publisher, "publish", None)
        )

    def prepare_goal(self, request: NavigateToPoseRequest) -> object:
        goal = self._action_type.Goal()
        goal.pose.header.frame_id = request.frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = request.x
        goal.pose.pose.position.y = request.y
        goal.pose.pose.orientation.z = math.sin(request.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(request.yaw / 2.0)
        return goal

    def send_goal(self, goal: object, activation_permit: object = None) -> object:
        if type(activation_permit) is not _ActivationPermit:
            raise AdapterError("PROFILE_INVALID")
        self._cancel_requested = False
        self._emergency_latched = False
        self._goal_generation += 1
        generation = self._goal_generation
        self._goal_permit = activation_permit
        self._goal_handle = None
        self._cancel_enqueued_for = None
        future = self._client.send_goal_async(goal)
        self._state = "pending"
        return future

    def track_goal(self, future: object, activation_permit: object) -> None:
        generation = self._goal_generation
        future.add_done_callback(
            lambda completed: self._goal_response(completed, activation_permit, generation)
        )

    def _goal_response(self, future, activation_permit=None, goal_generation: int | None = None) -> None:
        with self._lock:
            try:
                handle = future.result()
                accepted = bool(handle is not None and handle.accepted)
            except Exception:
                self._state = "faulted"
                return
            self._goal_handle = handle
            generation = getattr(self, "_goal_generation", 0) if goal_generation is None else goal_generation
            if accepted:
                try:
                    result_future = handle.get_result_async()
                    result_future.add_done_callback(self._goal_result)
                except Exception:
                    self._state = "faulted"
                    return
            stale = (
                type(activation_permit) is _ActivationPermit
                and not activation_permit._issuer._is_current(activation_permit)
            )
            if self._cancel_requested or getattr(self, "_emergency_latched", False) or stale:
                if accepted:
                    self._begin_cancel(handle, generation)
                else:
                    self._state = "cancelled"
                return
            self._state = "running" if accepted else "rejected"

    def _goal_result(self, future) -> None:
        with self._lock:
            try:
                result = future.result()
                status = getattr(result, "status", None)
            except Exception:
                self._state = "faulted"
                return
            if self._cancel_requested:
                self._state = "cancelled" if status == 5 else "faulted"
            else:
                self._state = "succeeded" if status == 4 else "failed"
            self._cancel_deadline = None

    def goal_status(self) -> object:
        with self._lock:
            deadline = getattr(self, "_cancel_deadline", None)
            clock = getattr(self, "_clock", time.monotonic)
            if self._state == "cancelling" and deadline is not None and clock() > deadline:
                self._state = "faulted"
            return {"state": self._state}

    def cancel_goal(self) -> None:
        with self._lock:
            self._cancel_requested = True
            self._state = "cancelling"
            self._cancel_deadline = getattr(self, "_clock", time.monotonic)() + getattr(self, "_cancel_timeout", 1.0)
            if self._goal_handle is not None:
                self._begin_cancel(self._goal_handle, getattr(self, "_goal_generation", 0))

    def _begin_cancel(self, handle, goal_generation: int | None = None) -> None:
        try:
            key = (id(handle), 0 if goal_generation is None else goal_generation)
            if getattr(self, "_cancel_enqueued_for", None) == key:
                return
            self._cancel_enqueued_for = key
            self._cancel_generation = getattr(self, "_cancel_generation", 0) + 1
            generation = self._cancel_generation
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda completed: self._cancel_response(completed, generation))
        except Exception:
            self._state = "faulted"

    def _cancel_response(self, future, generation: int) -> None:
        with self._lock:
            if generation != getattr(self, "_cancel_generation", generation):
                return
            try:
                response = future.result()
                accepted = bool(getattr(response, "goals_canceling", ()))
            except Exception:
                self._state = "faulted"
                return
            self._state = "cancelling" if accepted else "faulted"
            if not accepted:
                self._cancel_deadline = None

    def publish_zero(self) -> None:
        self._publisher.publish(self._twist_type())

    def emergency_channel(self) -> _EmergencyStopChannel:
        return self._safety_channel

    def _enqueue_emergency_zero(self) -> None:
        self.publish_zero()
        self._cancel_requested = True
        self._emergency_latched = True
        self._state = "cancelling"
        self._cancel_deadline = self._clock() + self._cancel_timeout
        handle = self._goal_handle
        if handle is not None:
            self._begin_cancel(handle, getattr(self, "_goal_generation", 0))

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None:
        self._estop_handlers.append(handler)


class _RclpyNav2EmergencyChannel(_EmergencyStopChannel):
    def __init__(self, transport: RclpyNav2Transport) -> None:
        super().__init__(hardware_verified=True)
        self._transport = transport

    def _preflight(self) -> bool:
        return callable(getattr(self._transport._publisher, "publish", None))

    def _enqueue_zero_disable(self) -> None:
        self._transport._enqueue_emergency_zero()
