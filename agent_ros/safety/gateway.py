"""Small fail-closed state machine that gates all robot task starts."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from agent_ros.discovery.models import DiscoveryReport
from agent_ros.profiles.models import RobotProfile
from agent_ros.safety.challenge import consume_operator_challenge
from agent_ros.safety.state import SafetyState
from agent_ros.safety.supervisor import SafetySupervisor


class SafetyError(RuntimeError):
    """Stable safety code, intentionally without underlying exception detail."""


_REQUIRED_CAPABILITY = {
    "twist": "mobile_base.twist",
    "nav2": "navigation.nav2",
    "follow_joint_trajectory": "manipulation.follow_joint_trajectory",
}
_STOP_BURST_COUNT = 3


@dataclass(frozen=True, slots=True)
class SafetyTransition:
    sequence: int
    state_before: SafetyState
    state_after: SafetyState
    safety_enqueue_accepted: bool = True


class SafetyGateway:
    """Authorize only reviewed profiles through explicit monotonic state transitions."""

    def __init__(
        self,
        profile: RobotProfile,
        *,
        runtime_dir: Path | None = None,
        stop_callback: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        boot_id: Callable[[], str] | None = None,
        supervisor_poll_interval: float = 0.05,
    ) -> None:
        self._profile = profile
        self._runtime_dir = Path(runtime_dir) if runtime_dir is not None else None
        self._stop_callback = stop_callback or (lambda: None)
        self._clock = clock
        self._state = SafetyState.NEW
        self._transition_sequence = 0
        self._latest_transition: SafetyTransition | None = None
        self._last_stop_accepted = True
        self._transition_history: deque[SafetyTransition] = deque(maxlen=256)
        self._report: DiscoveryReport | None = None
        self._last_heartbeat: float | None = None
        self._deadline: float | None = None
        self._lock = threading.RLock()
        self._boot_id = boot_id
        self._supervisor = SafetySupervisor(
            clock=self._clock,
            deadline=self._active_deadline,
            on_expired=self._fault_heartbeat_expiry,
            poll_interval=supervisor_poll_interval,
        )

    @property
    def state(self) -> SafetyState:
        with self._lock:
            return self._state

    @property
    def supervisor(self) -> SafetySupervisor:
        """The non-Agent supervisor; runtime owners must call close() during teardown."""
        return self._supervisor

    @property
    def latest_transition(self) -> SafetyTransition | None:
        with self._lock:
            return self._latest_transition

    def transitions_from(self, sequence: int) -> tuple[SafetyTransition, ...]:
        with self._lock:
            return tuple(item for item in self._transition_history if item.sequence >= sequence)

    @property
    def last_stop_accepted(self) -> bool:
        with self._lock:
            return self._last_stop_accepted

    def discover(self, report: DiscoveryReport) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.NEW)
            if not isinstance(report, DiscoveryReport) or report.blocking_warnings:
                raise SafetyError("DISCOVERY_UNSAFE")
            self._report = report
            return self._transition(SafetyState.DISCOVERED)

    def validate(self) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.DISCOVERED)
            assert self._report is not None
            capability = _REQUIRED_CAPABILITY[self._profile.adapter.kind]
            if capability not in self._report.capability_names or not self._interfaces_match(self._report):
                raise SafetyError("PROFILE_UNSUPPORTED")
            target = SafetyState.ARMED if self._profile.mode == "simulation" else SafetyState.VALIDATED
            return self._transition(target)

    def arm(self, challenge: str | None = None) -> SafetyTransition | None:
        with self._lock:
            if self._profile.mode == "simulation" and self._state is SafetyState.ARMED:
                return None
            self._require(SafetyState.VALIDATED)
            if self._profile.mode != "hardware" or self._runtime_dir is None:
                raise SafetyError("HARDWARE_CHALLENGE")
            challenge_args = {"monotonic_clock": self._clock}
            if self._boot_id is not None:
                challenge_args["boot_id"] = self._boot_id
            if not consume_operator_challenge(self._profile.name, self._runtime_dir, challenge or "", **challenge_args):
                raise SafetyError("HARDWARE_CHALLENGE")
            return self._transition(SafetyState.ARMED)

    def start_task(
        self,
        *,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        linear_acceleration: float = 0.0,
        angular_acceleration: float = 0.0,
    ) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.ARMED)
            self._check_limit(linear_velocity, self._profile.limits.max_linear_velocity)
            self._check_limit(angular_velocity, self._profile.limits.max_angular_velocity)
            self._check_limit(linear_acceleration, self._profile.limits.max_linear_acceleration)
            self._check_limit(angular_acceleration, self._profile.limits.max_angular_acceleration)
            self._last_heartbeat = self._clock()
            timeout = self._profile.safety.heartbeat_timeout
            self._deadline = None if timeout is None else self._last_heartbeat + timeout
            transition = self._transition(SafetyState.RUNNING)
            self._supervisor.start()
            return transition

    def heartbeat(self) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.RUNNING)
            timeout = self._profile.safety.heartbeat_timeout
            if timeout is None:
                self._fault("HEARTBEAT_UNCONFIGURED")
                raise SafetyError("HEARTBEAT_UNCONFIGURED")
            now = self._clock()
            assert self._last_heartbeat is not None
            if self._deadline is not None and now > self._deadline:
                self._fault("HEARTBEAT_EXPIRED")
                raise SafetyError("HEARTBEAT_EXPIRED")
            self._last_heartbeat = now
            self._deadline = now + timeout
            return self._transition(SafetyState.RUNNING)

    def cancel(self) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.RUNNING)
            accepted = self._stop_repeatedly()
            transition = self._transition(
                SafetyState.STOPPED,
                safety_enqueue_accepted=accepted,
            )
            self._deadline = None
            self._supervisor.stop()
            return transition

    def estop(self) -> SafetyTransition | None:
        with self._lock:
            return self._latch_estop()

    def observe_physical_estop(self, asserted: bool) -> SafetyTransition | None:
        """Monitor hook for a physical safety circuit; false never clears a latch."""
        if asserted is not True:
            return None
        with self._lock:
            return self._latch_estop()

    def operator_reset(self) -> SafetyTransition:
        with self._lock:
            if self._state is not SafetyState.ESTOPPED:
                raise SafetyError("UNSAFE_STATE")
            if self._profile.mode == "hardware":
                raise SafetyError("OPERATOR_REQUIRED")
            transition = self._transition(SafetyState.NEW)
            self._report = None
            self._last_heartbeat = None
            self._deadline = None
            return transition

    def close(self, *, timeout: float = 1.0) -> bool:
        """Own watchdog lifecycle cleanup; active motion is failed closed before shutdown."""
        with self._lock:
            if self._state is SafetyState.RUNNING:
                self._fault("SUPERVISOR_STOPPED")
            self._supervisor.stop()
        return self._supervisor.join(timeout=timeout)

    def _require(self, expected: SafetyState) -> None:
        if self._state is SafetyState.ESTOPPED:
            raise SafetyError("ESTOP_LATCHED")
        if self._state is not expected:
            raise SafetyError("UNSAFE_STATE")

    @staticmethod
    def _check_limit(value: float, maximum: float) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > maximum:
            raise SafetyError("MOTION_LIMIT")

    def _stop_repeatedly(self) -> bool:
        accepted = True
        for _ in range(_STOP_BURST_COUNT):
            try:
                self._stop_callback()
            except Exception:
                accepted = False
        self._last_stop_accepted = accepted
        return accepted

    def _active_deadline(self) -> float | None:
        with self._lock:
            return self._deadline if self._state is SafetyState.RUNNING else None

    def _fault_heartbeat_expiry(self) -> None:
        with self._lock:
            if self._state is SafetyState.RUNNING and self._deadline is not None and self._clock() > self._deadline:
                self._fault("HEARTBEAT_EXPIRED")

    def _fault(self, _code: str) -> SafetyTransition:
        accepted = self._stop_repeatedly()
        transition = self._transition(
            SafetyState.FAULTED,
            safety_enqueue_accepted=accepted,
        )
        self._deadline = None
        self._supervisor.stop()
        return transition

    def _latch_estop(self) -> SafetyTransition | None:
        accepted = self._stop_repeatedly()
        if self._state is SafetyState.ESTOPPED:
            return None
        transition = self._transition(
            SafetyState.ESTOPPED,
            safety_enqueue_accepted=accepted,
        )
        self._deadline = None
        self._supervisor.stop()
        return transition

    def _transition(
        self,
        state_after: SafetyState,
        *,
        safety_enqueue_accepted: bool = True,
    ) -> SafetyTransition:
        transition = SafetyTransition(
            self._transition_sequence,
            self._state,
            state_after,
            safety_enqueue_accepted,
        )
        self._transition_sequence += 1
        self._state = state_after
        self._latest_transition = transition
        self._transition_history.append(transition)
        return transition

    def _interfaces_match(self, report: DiscoveryReport) -> bool:
        interfaces = self._profile.interfaces
        if self._profile.adapter.kind == "twist":
            return (
                interfaces.command is not None
                and interfaces.odometry is not None
                and self._topic_matches(report, interfaces.command.topic, interfaces.command.type)
                and self._topic_matches(report, interfaces.odometry.topic, interfaces.odometry.type)
            )
        if self._profile.adapter.kind == "nav2":
            return interfaces.navigation is not None and self._action_matches(
                report, interfaces.navigation.action, interfaces.navigation.type
            )
        return interfaces.trajectory is not None and self._action_matches(
            report, interfaces.trajectory.action, interfaces.trajectory.type
        )

    @staticmethod
    def _topic_matches(report: DiscoveryReport, endpoint: str | None, interface_type: str) -> bool:
        return endpoint is not None and interface_type in report.topic_types.get(endpoint, ())

    @staticmethod
    def _action_matches(report: DiscoveryReport, endpoint: str | None, interface_type: str) -> bool:
        return endpoint is not None and interface_type in report.action_types.get(endpoint, ())
