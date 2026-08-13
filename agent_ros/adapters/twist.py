"""Bounded geometry_msgs/Twist adapter with odometry freshness enforcement."""

from __future__ import annotations

import time
import math
import threading
from collections.abc import Callable
from typing import Protocol

from agent_ros.adapters._safety import (
    _EmergencyStopChannel,
    _HARDWARE_CHANNEL_GUARD,
)
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
from agent_ros.safety.sequencer import (
    _ActivationPermit,
    _ActivationRejected,
    _SafetySequencer,
)


_ZERO_BURST_COUNT = 3


class TwistTransport(Protocol):
    def preflight_activation(self) -> bool: ...

    def read_odometry(self) -> OdometrySample: ...

    def start_waypoint(self, stage: TaskStage, activation_permit: object = None) -> None: ...

    def waypoint_status(self) -> AdapterStatus: ...

    def cancel_waypoint(self) -> None: ...

    def stop_waypoint(self) -> None: ...

    def emergency_channel(self) -> _EmergencyStopChannel: ...

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
        try:
            if self._transport.preflight_activation() is not True:
                raise AdapterError("PROFILE_INVALID")
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        if not isinstance(task, TaskStage):
            raise AdapterError("PROFILE_INVALID")
        self._fresh_odometry()
        try:
            self._activate_start(
                activation_permit,
                lambda: self._transport.start_waypoint(task, activation_permit),
            )
        except AdapterError:
            raise
        except Exception:
            try:
                self._emergency_stop()
            except AdapterError:
                pass
            raise AdapterError("INTERNAL_ERROR") from None
        return AdapterStatus("running")

    def status(self) -> AdapterStatus:
        try:
            status = self._transport.waypoint_status()
        except Exception:
            self.stop()
            raise AdapterError("STALE_FEEDBACK") from None
        if not isinstance(status, AdapterStatus):
            raise AdapterError("STALE_FEEDBACK")
        return status

    def cancel(self) -> AdapterStatus:
        try:
            self._transport.cancel_waypoint()
        except Exception:
            self.stop()
            raise AdapterError("INTERNAL_ERROR") from None
        return AdapterStatus("cancelled")

    def stop(self) -> None:
        try:
            self._transport.stop_waypoint()
        except Exception:
            return

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

    def _emergency_stop_channel(self):
        try:
            return self._transport.emergency_channel()
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None

    def _bind_runtime_safety(self, sequencer) -> None:
        super()._bind_runtime_safety(sequencer)
        if type(self._transport) is RclpyTwistTransport:
            self._transport._bind_safety_sequencer(sequencer)

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



class RclpyTwistTransport:
    """Structured rclpy transport, imported lazily so non-ROS tooling remains usable."""

    def __init__(self, node, command_topic: str, odometry_topic: str, estop_topic: str, limits=None, *, control_period: float = 0.05, stale_after: float = 1.0) -> None:
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
        self._limits = limits
        self._period = control_period
        self._stale_after = stale_after
        self._stage: TaskStage | None = None
        self._stage_permit: _ActivationPermit | None = None
        self._safety_sequencer: _SafetySequencer | None = None
        self._generation = 0
        self._stage_generation = 0
        self._state_lock = threading.RLock()
        self._publish_lock = threading.Lock()
        self._state = AdapterStatus("idle")
        self._last_command = TwistCommand.zero()
        self._timer = node.create_timer(control_period, self._control_step)
        self._estop_handlers: list[Callable[[bool], None]] = []
        self._safety_channel = _RclpyTwistEmergencyChannel(self)

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

    def preflight_activation(self) -> bool:
        return self._limits is not None and callable(getattr(self._publisher, "publish", None))

    def start_waypoint(self, stage: TaskStage, activation_permit: object = None) -> None:
        if type(activation_permit) is not _ActivationPermit:
            raise AdapterError("PROFILE_INVALID")
        self._generation += 1
        self._stage_generation = self._generation
        self._stage_permit = activation_permit
        self._stage = stage
        self._state = AdapterStatus("running")

    def _bind_safety_sequencer(self, sequencer: _SafetySequencer) -> None:
        if type(sequencer) is not _SafetySequencer:
            raise AdapterError("PROFILE_INVALID")
        if self._safety_sequencer is not None and self._safety_sequencer is not sequencer:
            raise AdapterError("PROFILE_INVALID")
        self._safety_sequencer = sequencer

    def waypoint_status(self) -> AdapterStatus:
        return self._state

    def cancel_waypoint(self) -> None:
        self.stop_waypoint()
        self._state = AdapterStatus("cancelled")

    def stop_waypoint(self) -> None:
        self._zero_and_disable("stopped")

    def emergency_channel(self) -> _EmergencyStopChannel:
        return self._safety_channel

    def _enqueue_emergency_zero(self) -> None:
        self._generation += 1
        self._stage = None
        self._stage_permit = None
        self._state = AdapterStatus("estopped")
        self.publish(TwistCommand.zero())
        self._last_command = TwistCommand.zero()

    def _zero_and_disable(self, state: str) -> None:
        with self._state_lock:
            self._generation += 1
            self._stage = None
            self._stage_permit = None
            self._state = AdapterStatus(state)
        with self._publish_lock:
            for _ in range(_ZERO_BURST_COUNT):
                self.publish(TwistCommand.zero())
            self._last_command = TwistCommand.zero()

    def _control_step(self) -> None:
        with self._state_lock:
            stage = self._stage
            sample = self._sample
            generation = self._stage_generation
            permit = getattr(self, "_stage_permit", None)
        if stage is None:
            return
        if sample is None or self._clock() - sample.timestamp > self._stale_after or sample.timestamp - self._clock() > 0.05:
            self.stop_waypoint()
            self._state = AdapterStatus("faulted", "STALE_FEEDBACK")
            return
        distance = math.hypot(stage.goal.x - sample.x, stage.goal.y - sample.y)
        if distance <= stage.tolerance:
            self.stop_waypoint()
            self._state = AdapterStatus("succeeded")
            return
        heading = math.atan2(stage.goal.y - sample.y, stage.goal.x - sample.x)
        error = math.atan2(math.sin(heading - sample.yaw), math.cos(heading - sample.yaw))
        if self._limits is None:
            self.stop_waypoint()
            self._state = AdapterStatus("faulted", "PROFILE_INVALID")
            return
        desired_linear = min(self._limits.max_linear_velocity, distance) if abs(error) <= math.pi / 2 else 0.0
        desired_angular = max(-self._limits.max_angular_velocity, min(self._limits.max_angular_velocity, error))
        linear_delta = self._limits.max_linear_acceleration * self._period
        angular_delta = self._limits.max_angular_acceleration * self._period
        command = TwistCommand(
            max(self._last_command.linear_velocity - linear_delta, min(self._last_command.linear_velocity + linear_delta, desired_linear)),
            max(self._last_command.angular_velocity - angular_delta, min(self._last_command.angular_velocity + angular_delta, desired_angular)),
        )
        before_publish = getattr(self, "_before_publish", None)
        if before_publish is not None:
            before_publish()
        def enqueue() -> None:
            if generation != self._generation or self._stage is None:
                return
            self.publish(command)
            self._last_command = command

        if (
            type(permit) is not _ActivationPermit
            or permit._sequencer is not self._safety_sequencer
        ):
            self._disable_unsafe_stage()
            return
        try:
            permit._sequencer.submit(permit, enqueue, 1.0)
        except _ActivationRejected:
            self._disable_unsafe_stage()

    def _disable_unsafe_stage(self) -> None:
        with self._state_lock:
            self._generation += 1
            self._stage = None
            self._stage_permit = None
            self._state = AdapterStatus("faulted", "UNSAFE_STATE")

    def read_odometry(self) -> OdometrySample:
        if self._sample is None:
            raise AdapterError("STALE_FEEDBACK")
        return self._sample

    def subscribe_estop(self, handler: Callable[[bool], None]) -> None:
        self._estop_handlers.append(handler)


class _RclpyTwistEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, transport: RclpyTwistTransport) -> None:
        super().__init__(hardware_verified=True, construction_guard=_HARDWARE_CHANNEL_GUARD)
        self._transport = transport

    def _preflight(self) -> bool:
        return callable(getattr(self._transport, "publish", None))

    def _enqueue_zero_disable(self) -> None:
        self._transport._enqueue_emergency_zero()


def _yaw_from_quaternion(value) -> float:
    import math

    siny_cosp = 2.0 * (value.w * value.z + value.x * value.y)
    cosy_cosp = 1.0 - 2.0 * (value.y * value.y + value.z * value.z)
    return math.atan2(siny_cosp, cosy_cosp)
