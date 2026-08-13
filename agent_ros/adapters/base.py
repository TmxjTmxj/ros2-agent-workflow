"""Common typed contracts for deterministic robot adapters."""

from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

from agent_ros.safety.outcome import EmergencyStopResult

if TYPE_CHECKING:
    from agent_ros.profiles.models import RobotProfile


class AdapterError(RuntimeError):
    """A stable adapter failure that deliberately omits transport details."""

    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


class HospitalAction(str, Enum):
    """The complete repository-owned hospital controller authority."""

    PROBE = "probe"
    VALIDATE = "validate"
    START = "start"
    STATUS = "status"
    CANCEL = "cancel"
    STOP = "stop"
    OBSERVE = "observe"


@dataclass(frozen=True, slots=True)
class TwistCommand:
    linear_velocity: float
    angular_velocity: float

    def __post_init__(self) -> None:
        if not _finite(self.linear_velocity) or not _finite(self.angular_velocity):
            raise AdapterError("PROFILE_INVALID")

    @classmethod
    def zero(cls) -> "TwistCommand":
        return cls(0.0, 0.0)


@dataclass(frozen=True, slots=True)
class OdometrySample:
    timestamp: float
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.timestamp, self.x, self.y, self.yaw)):
            raise AdapterError("STALE_FEEDBACK")


@dataclass(frozen=True, slots=True)
class NavigationGoal:
    frame: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not isinstance(self.frame, str) or not self.frame or not all(
            _finite(value) for value in (self.x, self.y, self.yaw)
        ):
            raise AdapterError("PROFILE_INVALID")


@dataclass(frozen=True, slots=True)
class AdapterProbe:
    available: bool
    capabilities: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class AdapterStatus:
    state: str
    code: str | None = None
    values: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", MappingProxyType(dict(self.values)))


@dataclass(frozen=True, slots=True)
class Observation:
    source: str
    timestamp: float
    values: Mapping[str, object]

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", MappingProxyType(dict(self.values)))


class RobotAdapter(ABC):
    """The only runtime-facing contract implemented by robot integrations."""

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> bool:
        if not self.close(timeout=1.0):
            raise AdapterError("CLEANUP_FAILED")
        return False

    @abstractmethod
    def probe(self) -> AdapterProbe:
        raise NotImplementedError

    @abstractmethod
    def validate(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> AdapterStatus:
        raise NotImplementedError

    @abstractmethod
    def cancel(self) -> AdapterStatus:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def observe(self, source: str) -> Observation:
        raise NotImplementedError

    @abstractmethod
    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        """Connect a non-agent safety input directly to the runtime latch."""
        raise NotImplementedError

    @abstractmethod
    def _emergency_stop_channel(self):
        """Return the private transport safety channel; never an arbitrary runner."""
        raise NotImplementedError

    def _bind_runtime_safety(self, sequencer) -> None:
        from agent_ros.adapters._safety import _EmergencyStopChannel
        from agent_ros.safety.sequencer import _SafetySequencer

        channel = self._emergency_stop_channel()
        if not isinstance(sequencer, _SafetySequencer) or not isinstance(channel, _EmergencyStopChannel):
            raise AdapterError("PROFILE_INVALID")
        try:
            channel._bind(sequencer)
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None
        self._safety_sequencer = sequencer

    def close(self, timeout: float = 1.0) -> bool:
        """Bounded close for adapter-owned activation and emergency workers."""
        from agent_ros.adapters._safety import _EmergencyStopChannel
        from agent_ros.safety.sequencer import _SafetySequencer

        sequencer = getattr(self, "_safety_sequencer", None)
        if not isinstance(sequencer, _SafetySequencer):
            return True
        deadline = time.monotonic() + max(0.0, timeout)
        sequencer.begin_close()
        channel = self._emergency_stop_channel()
        if not isinstance(channel, _EmergencyStopChannel):
            sequencer.close(max(0.0, deadline - time.monotonic()))
            return False
        channel_closed = channel._close(max(0.0, deadline - time.monotonic()))
        sequencer_closed = sequencer.close(max(0.0, deadline - time.monotonic()))
        return channel_closed and sequencer_closed

    def _validate_runtime_safety(self, mode: str) -> None:
        from agent_ros.adapters._safety import _EmergencyStopChannel

        channel = self._emergency_stop_channel()
        if not isinstance(channel, _EmergencyStopChannel):
            raise AdapterError("PROFILE_INVALID")
        try:
            channel._verify(mode)
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None

    def _activate_start(self, permit: object, enqueue):
        from agent_ros.safety.sequencer import _ActivationRejected, _SafetySequencer

        sequencer = getattr(self, "_safety_sequencer", None)
        if not isinstance(sequencer, _SafetySequencer):
            raise AdapterError("PROFILE_INVALID")
        before_activation = getattr(self, "_before_activation", None)
        if before_activation is not None:
            before_activation()
        try:
            return sequencer.submit(permit, enqueue, 1.0)
        except _ActivationRejected as exc:
            raise AdapterError(exc.code) from None

    def _activate_owned_start(self, permit: object, enqueue):
        from agent_ros.safety.sequencer import _ActivationRejected, _SafetySequencer

        sequencer = getattr(self, "_safety_sequencer", None)
        if not isinstance(sequencer, _SafetySequencer):
            raise AdapterError("PROFILE_INVALID")
        try:
            return sequencer.submit_owned(permit, enqueue)
        except _ActivationRejected as exc:
            raise AdapterError(exc.code) from None

    def _permit_is_current(self, permit: object) -> bool:
        from agent_ros.safety.sequencer import _SafetySequencer

        sequencer = getattr(self, "_safety_sequencer", None)
        return isinstance(sequencer, _SafetySequencer) and sequencer.is_current(permit)

    def _require_current_permit(self, permit: object) -> None:
        from agent_ros.safety.sequencer import _ActivationRejected, _SafetySequencer

        sequencer = getattr(self, "_safety_sequencer", None)
        if not isinstance(sequencer, _SafetySequencer):
            raise AdapterError("PROFILE_INVALID")
        try:
            sequencer.require_current(permit)
        except _ActivationRejected as exc:
            raise AdapterError(exc.code) from None

    def _emergency_stop(self, timeout: float = 1.0) -> EmergencyStopResult:
        try:
            return self._emergency_stop_channel()._stop(timeout)
        except Exception as exc:
            code = getattr(exc, "code", "UNSAFE_STATE")
            raise AdapterError(code) from None


def create_adapter(profile: "RobotProfile", transport: object, **kwargs: Any) -> RobotAdapter:
    """Select only repository-implemented adapter kinds from a reviewed profile."""
    kind = getattr(getattr(profile, "adapter", None), "kind", None)
    if kind == "twist":
        from agent_ros.adapters.twist import TwistAdapter

        return TwistAdapter(profile, transport, **kwargs)
    if kind == "nav2":
        from agent_ros.adapters.nav2 import Nav2Adapter

        return Nav2Adapter(profile, transport, **kwargs)
    raise AdapterError("PROFILE_INVALID")


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
