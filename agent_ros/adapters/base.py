"""Common typed contracts for deterministic robot adapters."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

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

    @abstractmethod
    def probe(self) -> AdapterProbe:
        raise NotImplementedError

    @abstractmethod
    def validate(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def start(self, task: object) -> AdapterStatus:
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
    def bind_physical_estop(self, handler: Callable[[bool], None]) -> None:
        """Connect a non-agent safety input directly to the runtime latch."""
        raise NotImplementedError


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
