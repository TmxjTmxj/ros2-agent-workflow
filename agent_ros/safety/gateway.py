"""Small fail-closed state machine that gates all robot task starts."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from pathlib import Path

from agent_ros.discovery.models import DiscoveryReport
from agent_ros.profiles.models import RobotProfile
from agent_ros.safety.challenge import consume_operator_challenge
from agent_ros.safety.state import SafetyState


class SafetyError(RuntimeError):
    """Stable safety code, intentionally without underlying exception detail."""


_REQUIRED_CAPABILITY = {
    "twist": "mobile_base.twist",
    "nav2": "navigation.nav2",
    "follow_joint_trajectory": "manipulation.follow_joint_trajectory",
}
_STOP_BURST_COUNT = 3


class SafetyGateway:
    """Authorize only reviewed profiles through explicit monotonic state transitions."""

    def __init__(
        self,
        profile: RobotProfile,
        *,
        runtime_dir: Path | None = None,
        stop_callback: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._profile = profile
        self._runtime_dir = Path(runtime_dir) if runtime_dir is not None else None
        self._stop_callback = stop_callback or (lambda: None)
        self._clock = clock
        self._state = SafetyState.NEW
        self._report: DiscoveryReport | None = None
        self._last_heartbeat: float | None = None

    @property
    def state(self) -> SafetyState:
        return self._state

    def discover(self, report: DiscoveryReport) -> None:
        self._require(SafetyState.NEW)
        if not isinstance(report, DiscoveryReport) or report.blocking_warnings:
            raise SafetyError("DISCOVERY_UNSAFE")
        self._report = report
        self._state = SafetyState.DISCOVERED

    def validate(self) -> None:
        self._require(SafetyState.DISCOVERED)
        assert self._report is not None
        capability = _REQUIRED_CAPABILITY[self._profile.adapter.kind]
        if capability not in self._report.capability_names:
            raise SafetyError("PROFILE_UNSUPPORTED")
        self._state = SafetyState.VALIDATED
        if self._profile.mode == "simulation":
            self._state = SafetyState.ARMED

    def arm(self, challenge: str | None = None) -> None:
        if self._profile.mode == "simulation" and self._state is SafetyState.ARMED:
            return
        self._require(SafetyState.VALIDATED)
        if self._profile.mode != "hardware" or self._runtime_dir is None:
            raise SafetyError("HARDWARE_CHALLENGE")
        if not consume_operator_challenge(self._profile.name, self._runtime_dir, challenge or "", monotonic_clock=self._clock):
            raise SafetyError("HARDWARE_CHALLENGE")
        self._state = SafetyState.ARMED

    def start_task(
        self,
        *,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        linear_acceleration: float = 0.0,
        angular_acceleration: float = 0.0,
    ) -> None:
        self._require(SafetyState.ARMED)
        self._check_limit(linear_velocity, self._profile.limits.max_linear_velocity)
        self._check_limit(angular_velocity, self._profile.limits.max_angular_velocity)
        self._check_limit(linear_acceleration, self._profile.limits.max_linear_acceleration)
        self._check_limit(angular_acceleration, self._profile.limits.max_angular_acceleration)
        self._last_heartbeat = self._clock()
        self._state = SafetyState.RUNNING

    def heartbeat(self) -> None:
        self._require(SafetyState.RUNNING)
        timeout = self._profile.safety.heartbeat_timeout
        if timeout is None:
            self._stop_repeatedly()
            self._state = SafetyState.FAULTED
            raise SafetyError("HEARTBEAT_UNCONFIGURED")
        now = self._clock()
        assert self._last_heartbeat is not None
        if now - self._last_heartbeat > timeout:
            self._stop_repeatedly()
            self._state = SafetyState.FAULTED
            raise SafetyError("HEARTBEAT_EXPIRED")
        self._last_heartbeat = now

    def cancel(self) -> None:
        self._require(SafetyState.RUNNING)
        self._stop_repeatedly()
        self._state = SafetyState.STOPPED

    def estop(self) -> None:
        if self._state is SafetyState.ESTOPPED:
            return
        self._stop_repeatedly()
        self._state = SafetyState.ESTOPPED

    def operator_reset(self) -> None:
        if self._state is not SafetyState.ESTOPPED:
            raise SafetyError("UNSAFE_STATE")
        if self._profile.mode == "hardware":
            raise SafetyError("OPERATOR_REQUIRED")
        self._state = SafetyState.NEW
        self._report = None
        self._last_heartbeat = None

    def _require(self, expected: SafetyState) -> None:
        if self._state is SafetyState.ESTOPPED:
            raise SafetyError("ESTOP_LATCHED")
        if self._state is not expected:
            raise SafetyError("UNSAFE_STATE")

    @staticmethod
    def _check_limit(value: float, maximum: float) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > maximum:
            raise SafetyError("MOTION_LIMIT")

    def _stop_repeatedly(self) -> None:
        for _ in range(_STOP_BURST_COUNT):
            try:
                self._stop_callback()
            except Exception:
                # A stop transport failure cannot make a fault recoverable.
                continue
