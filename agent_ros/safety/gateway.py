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
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.state import SafetyState
from agent_ros.safety.supervisor import SafetySupervisor


class SafetyError(RuntimeError):
    """Stable safety code, intentionally without underlying exception detail."""


_REQUIRED_CAPABILITY = {
    "twist": "mobile_base.twist",
    "hospital_delivery": "mobile_base.twist",
    "nav2": "navigation.nav2",
    "follow_joint_trajectory": "manipulation.follow_joint_trajectory",
}
_STOP_BURST_COUNT = 3


@dataclass(frozen=True, slots=True)
class SafetyTransition:
    sequence: int
    state_before: SafetyState
    state_after: SafetyState
    stop_result: EmergencyStopResult | None = None

    @property
    def safety_enqueue_accepted(self) -> bool:
        """Compatibility view of the structured emergency result."""
        return True if self.stop_result is None else self.stop_result.safety_command_accepted


@dataclass(frozen=True, slots=True)
class SafetyStopAttempt:
    """The immutable transition and outcome produced by one stop invocation."""

    transition: SafetyTransition | None
    result: EmergencyStopResult


def _successful_stop(_timeout: float) -> EmergencyStopResult:
    return EmergencyStopResult(True, True, True, "ESTOP_LATCHED")


class SafetyGateway:
    """Authorize only reviewed profiles through explicit monotonic state transitions."""

    def __init__(
        self,
        profile: RobotProfile,
        *,
        runtime_dir: Path | None = None,
        stop_callback: Callable[[float], EmergencyStopResult] | None = None,
        clock: Callable[[], float] = time.monotonic,
        boot_id: Callable[[], str] | None = None,
        supervisor_poll_interval: float = 0.05,
    ) -> None:
        self._profile = profile
        self._runtime_dir = Path(runtime_dir) if runtime_dir is not None else None
        self._stop_callback = stop_callback or _successful_stop
        self._clock = clock
        self._state = SafetyState.NEW
        self._transition_sequence = 0
        self._latest_transition: SafetyTransition | None = None
        self._last_stop_result = _successful_stop(0.0)
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

    def owns_transition(self, transition: object) -> bool:
        """Recognize only the exact frozen receipt retained in gateway history."""
        with self._lock:
            return any(item is transition for item in self._transition_history)

    @property
    def last_stop_accepted(self) -> bool:
        with self._lock:
            return self._last_stop_result.safety_command_accepted

    @property
    def last_stop_result(self) -> EmergencyStopResult:
        with self._lock:
            return self._last_stop_result

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
                fault_code = "HEARTBEAT_UNCONFIGURED"
            else:
                now = self._clock()
                assert self._last_heartbeat is not None
                if self._deadline is not None and now > self._deadline:
                    fault_code = "HEARTBEAT_EXPIRED"
                else:
                    self._last_heartbeat = now
                    self._deadline = now + timeout
                    return self._transition(SafetyState.RUNNING)
        self._fault(fault_code)
        raise SafetyError(fault_code)

    def cancel(self, *, timeout: float = 1.0) -> SafetyTransition:
        with self._lock:
            self._require(SafetyState.RUNNING)
        stop_result = self._stop_repeatedly(timeout)
        with self._lock:
            self._require(SafetyState.RUNNING)
            transition = self._transition(
                SafetyState.STOPPED,
                stop_result=stop_result,
            )
            self._deadline = None
            self._supervisor.stop()
            return transition

    def estop(self, *, timeout: float = 1.0) -> SafetyTransition | None:
        """Compatibility API that discards the invocation's stop result."""
        return self.estop_attempt(timeout=timeout).transition

    def estop_attempt(self, *, timeout: float = 1.0) -> SafetyStopAttempt:
        return self._latch_estop_attempt(timeout)

    def observe_physical_estop(self, asserted: bool) -> SafetyTransition | None:
        """Monitor hook for a physical safety circuit; false never clears a latch."""
        if asserted is not True:
            return None
        return self.observe_physical_estop_attempt(True).transition

    def observe_physical_estop_attempt(self, asserted: bool) -> SafetyStopAttempt:
        if asserted is not True:
            raise SafetyError("UNSAFE_STATE")
        return self._latch_estop_attempt(1.0)

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
        deadline = time.monotonic() + max(0.0, timeout)
        with self._lock:
            running = self._state is SafetyState.RUNNING
        if running:
            self._fault(
                "SUPERVISOR_STOPPED",
                timeout=max(0.0, deadline - time.monotonic()),
            )
        with self._lock:
            self._supervisor.stop()
        return self._supervisor.join(timeout=max(0.0, deadline - time.monotonic()))

    def _require(self, expected: SafetyState) -> None:
        if self._state is SafetyState.ESTOPPED:
            raise SafetyError("ESTOP_LATCHED")
        if self._state is not expected:
            raise SafetyError("UNSAFE_STATE")

    @staticmethod
    def _check_limit(value: float, maximum: float) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or abs(value) > maximum
        ):
            raise SafetyError("MOTION_LIMIT")

    def _stop_repeatedly(self, timeout: float) -> EmergencyStopResult:
        deadline = time.monotonic() + max(0.0, timeout)
        results: list[EmergencyStopResult] = []
        for _ in range(_STOP_BURST_COUNT):
            try:
                result = self._stop_callback(max(0.0, deadline - time.monotonic()))
            except Exception:
                result = EmergencyStopResult(
                    True,
                    False,
                    False,
                    "SAFETY_COMMAND_REJECTED",
                )
            if not isinstance(result, EmergencyStopResult):
                result = EmergencyStopResult(
                    True,
                    False,
                    False,
                    "SAFETY_COMMAND_REJECTED",
                )
            results.append(result)
        latched = all(item.latched for item in results)
        quiesced = all(item.activation_quiesced for item in results)
        accepted = all(item.safety_command_accepted for item in results)
        if not quiesced:
            code = "TRANSPORT_UNQUIESCED"
        elif not accepted or not latched:
            code = "SAFETY_COMMAND_REJECTED"
        else:
            code = "ESTOP_LATCHED"
        combined = EmergencyStopResult(latched, quiesced, accepted, code)
        with self._lock:
            self._last_stop_result = combined
        return combined

    def _active_deadline(self) -> float | None:
        with self._lock:
            return self._deadline if self._state is SafetyState.RUNNING else None

    def _fault_heartbeat_expiry(self) -> None:
        with self._lock:
            expired = (
                self._state is SafetyState.RUNNING and self._deadline is not None and self._clock() > self._deadline
            )
        if expired:
            self._fault("HEARTBEAT_EXPIRED")

    def _fault(
        self,
        _code: str,
        *,
        timeout: float = 1.0,
    ) -> SafetyTransition | None:
        stop_result = self._stop_repeatedly(timeout)
        with self._lock:
            if self._state is not SafetyState.RUNNING:
                return None
            transition = self._transition(
                SafetyState.FAULTED,
                stop_result=stop_result,
            )
            self._deadline = None
            self._supervisor.stop()
            return transition

    def _latch_estop_attempt(self, timeout: float) -> SafetyStopAttempt:
        stop_result = self._stop_repeatedly(timeout)
        with self._lock:
            if self._state is SafetyState.ESTOPPED:
                return SafetyStopAttempt(None, stop_result)
            transition = self._transition(
                SafetyState.ESTOPPED,
                stop_result=stop_result,
            )
            self._deadline = None
            self._supervisor.stop()
            return SafetyStopAttempt(transition, stop_result)

    def _transition(
        self,
        state_after: SafetyState,
        *,
        stop_result: EmergencyStopResult | None = None,
    ) -> SafetyTransition:
        transition = SafetyTransition(
            self._transition_sequence,
            self._state,
            state_after,
            stop_result,
        )
        self._transition_sequence += 1
        self._state = state_after
        self._latest_transition = transition
        self._transition_history.append(transition)
        return transition

    def _interfaces_match(self, report: DiscoveryReport) -> bool:
        interfaces = self._profile.interfaces
        if self._profile.adapter.kind in {"twist", "hospital_delivery"}:
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
