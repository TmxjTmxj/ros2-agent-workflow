"""Bounded geometry_msgs/Twist adapter with odometry freshness enforcement."""

from __future__ import annotations

import time
import math
from collections.abc import Callable
from typing import Protocol

from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    Observation,
    OdometrySample,
    RobotAdapter,
    TwistCommand,
)
from agent_ros.profiles.models import ODOMETRY_TYPE, TWIST_TYPE, RobotProfile, TaskStage


_ZERO_BURST_COUNT = 3


class TwistTransport(Protocol):
    def publish(self, command: TwistCommand) -> None: ...

    def read_odometry(self) -> OdometrySample: ...

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None: ...


class TwistAdapter(RobotAdapter):
    def __init__(
        self,
        profile: RobotProfile,
        transport: TwistTransport,
        *,
        clock: Callable[[], float] = time.monotonic,
        stale_after: float | None = None,
        future_skew: float = 0.05,
    ) -> None:
        self._profile = profile
        self._transport = transport
        self._clock = clock
        self._stale_after = stale_after if stale_after is not None else profile.safety.heartbeat_timeout
        self._future_skew = future_skew
        self._state = "idle"
        self._stage: TaskStage | None = None
        self._last_command: TwistCommand | None = None
        self._last_command_time: float | None = None
        self.validate()

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("mobile_base.twist",))

    def validate(self) -> None:
        interfaces = self._profile.interfaces
        if (
            self._profile.adapter.kind != "twist"
            or interfaces.command is None
            or interfaces.command.type != TWIST_TYPE
            or interfaces.odometry is None
            or interfaces.odometry.type != ODOMETRY_TYPE
        ):
            raise AdapterError("PROFILE_INVALID")

    def start(self, task: object) -> AdapterStatus:
        if isinstance(task, TaskStage):
            sample = self._fresh_odometry()
            self._stage = task
            self._publish_desired(self._command_for_stage(task, sample))
            self._state = "running"
            return AdapterStatus(self._state)
        if not isinstance(task, TwistCommand):
            raise AdapterError("PROFILE_INVALID")
        limits = self._profile.limits
        command = TwistCommand(
            linear_velocity=max(-limits.max_linear_velocity, min(limits.max_linear_velocity, task.linear_velocity)),
            angular_velocity=max(-limits.max_angular_velocity, min(limits.max_angular_velocity, task.angular_velocity)),
        )
        self._publish_desired(command, enforce_acceleration=False)
        self._state = "running"
        return AdapterStatus(self._state)

    def status(self) -> AdapterStatus:
        sample = self._fresh_odometry()
        if self._stage is not None:
            distance = math.hypot(self._stage.goal.x - sample.x, self._stage.goal.y - sample.y)
            if distance <= self._stage.tolerance:
                self.stop()
                self._state = "succeeded"
                return AdapterStatus(self._state)
            self._publish_desired(self._command_for_stage(self._stage, sample))
        return AdapterStatus(self._state, values={"odometry_timestamp": sample.timestamp})

    def cancel(self) -> AdapterStatus:
        self.stop()
        self._state = "cancelled"
        return AdapterStatus(self._state)

    def stop(self) -> None:
        for _ in range(_ZERO_BURST_COUNT):
            try:
                self._transport.publish(TwistCommand.zero())
            except Exception:
                continue
        self._state = "stopped"
        self._stage = None
        self._last_command = TwistCommand.zero()
        self._last_command_time = self._clock()

    def observe(self, source: str) -> Observation:
        if source != "odometry" or source not in self._profile.observation_sources:
            raise AdapterError("PROFILE_INVALID")
        sample = self._fresh_odometry()
        return Observation(
            "odometry",
            sample.timestamp,
            {"x": sample.x, "y": sample.y, "yaw": sample.yaw},
        )

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        self._transport.subscribe_estop(handler)
        return True

    def _fresh_odometry(self) -> OdometrySample:
        try:
            sample = self._transport.read_odometry()
        except Exception:
            self.stop()
            raise AdapterError("STALE_FEEDBACK") from None
        age = self._clock() - sample.timestamp if isinstance(sample, OdometrySample) else math.inf
        if (
            not isinstance(sample, OdometrySample)
            or self._stale_after is None
            or age > self._stale_after
            or age < -self._future_skew
        ):
            self.stop()
            raise AdapterError("STALE_FEEDBACK")
        return sample

    def _command_for_stage(self, stage: TaskStage, sample: OdometrySample) -> TwistCommand:
        dx = stage.goal.x - sample.x
        dy = stage.goal.y - sample.y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(heading - sample.yaw), math.cos(heading - sample.yaw))
        linear = min(self._profile.limits.max_linear_velocity, distance)
        if abs(heading_error) > math.pi / 2.0:
            linear = 0.0
        angular = max(
            -self._profile.limits.max_angular_velocity,
            min(self._profile.limits.max_angular_velocity, heading_error),
        )
        return TwistCommand(linear, angular)

    def _publish_desired(self, command: TwistCommand, *, enforce_acceleration: bool = True) -> None:
        now = self._clock()
        if enforce_acceleration and self._last_command is not None and self._last_command_time is not None:
            elapsed = max(0.0, now - self._last_command_time)
            linear_delta = self._profile.limits.max_linear_acceleration * elapsed
            angular_delta = self._profile.limits.max_angular_acceleration * elapsed
            command = TwistCommand(
                max(self._last_command.linear_velocity - linear_delta, min(self._last_command.linear_velocity + linear_delta, command.linear_velocity)),
                max(self._last_command.angular_velocity - angular_delta, min(self._last_command.angular_velocity + angular_delta, command.angular_velocity)),
            )
        try:
            self._transport.publish(command)
        except Exception:
            self.stop()
            raise AdapterError("INTERNAL_ERROR") from None
        self._last_command = command
        self._last_command_time = now


class RclpyTwistTransport:
    """Structured rclpy transport, imported lazily so non-ROS tooling remains usable."""

    def __init__(self, node, command_topic: str, odometry_topic: str, estop_topic: str) -> None:
        try:
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool
        except ImportError as exc:
            raise AdapterError("PROFILE_INVALID") from exc
        self._clock = time.monotonic
        self._twist_type = Twist
        self._publisher = node.create_publisher(Twist, command_topic, 10)
        self._sample: OdometrySample | None = None
        self._estop_handlers: list[Callable[[bool], None]] = []

        def odometry_callback(message) -> None:
            self._sample = OdometrySample(
                self._clock(),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                _yaw_from_quaternion(message.pose.pose.orientation),
            )

        def estop_callback(message) -> None:
            for handler in tuple(self._estop_handlers):
                handler(bool(message.data))

        self._odometry_subscription = node.create_subscription(Odometry, odometry_topic, odometry_callback, 10)
        self._estop_subscription = node.create_subscription(Bool, estop_topic, estop_callback, 10)

    def publish(self, command: TwistCommand) -> None:
        message = self._twist_type()
        message.linear.x = command.linear_velocity
        message.angular.z = command.angular_velocity
        self._publisher.publish(message)

    def read_odometry(self) -> OdometrySample:
        if self._sample is None:
            raise AdapterError("STALE_FEEDBACK")
        return self._sample

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None:
        self._estop_handlers.append(handler)


def _yaw_from_quaternion(value) -> float:
    import math

    siny_cosp = 2.0 * (value.w * value.z + value.x * value.y)
    cosy_cosp = 1.0 - 2.0 * (value.y * value.y + value.z * value.z)
    return math.atan2(siny_cosp, cosy_cosp)
