"""Deterministic, ROS-independent controller for the hospital mission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any


class RouteValidationError(ValueError):
    """Raised when a mission route cannot be controlled safely."""


class MissionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    ESTOPPED = "ESTOPPED"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class VelocityCommand:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float


@dataclass(frozen=True)
class MissionStage:
    id: str
    name: str
    endpoint: Waypoint
    waypoints: tuple[Waypoint, ...]


@dataclass(frozen=True)
class MissionRoute:
    start: Pose2D
    stages: tuple[MissionStage, ...]


@dataclass(frozen=True)
class ControllerConfig:
    waypoint_tolerance: float = 0.35
    max_linear: float = 1.2
    max_angular: float = 1.6
    angular_kp: float = 1.6
    align_threshold: float = 0.35
    slow_distance: float = 0.8
    mission_timeout: float = 180.0
    odom_timeout: float = 0.5
    obstacle_stop_distance: float = 0.35
    obstacle_fail_after: float = 5.0
    progress_timeout: float = 12.0
    progress_epsilon: float = 0.05
    progress_heading_epsilon: float = 0.05


@dataclass(frozen=True)
class OperationResult:
    accepted: bool
    message: str


def normalize_angle(angle: float) -> float:
    """Normalize to [-pi, pi] while preserving the sign at the boundary."""
    result = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if math.isclose(result, -math.pi) and angle > 0.0:
        return math.pi
    return result


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteValidationError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise RouteValidationError(f"{label} must be finite")
    return result


def _point(raw: Any, label: str) -> Waypoint:
    if not isinstance(raw, list) or len(raw) != 2:
        raise RouteValidationError(f"{label} must contain x and y")
    return Waypoint(
        _finite_number(raw[0], f"{label}.x"),
        _finite_number(raw[1], f"{label}.y"),
    )


def load_route(path: str | Path) -> MissionRoute:
    source = Path(path)
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RouteValidationError(f"cannot load route: {exc}") from exc

    if not isinstance(raw, dict):
        raise RouteValidationError("route root must be an object")
    start_raw = raw.get("start")
    if not isinstance(start_raw, list) or len(start_raw) != 3:
        raise RouteValidationError("start must contain x, y, and yaw")
    start = Pose2D(*(_finite_number(v, f"start[{i}]") for i, v in enumerate(start_raw)))

    stages_raw = raw.get("stages")
    if not isinstance(stages_raw, list) or len(stages_raw) != 3:
        raise RouteValidationError("route must contain exactly three stages")

    stages: list[MissionStage] = []
    seen_ids: set[str] = set()
    for index, item in enumerate(stages_raw):
        if not isinstance(item, dict):
            raise RouteValidationError(f"stage[{index}] must be an object")
        stage_id = item.get("id")
        name = item.get("name")
        if not isinstance(stage_id, str) or not stage_id.strip():
            raise RouteValidationError(f"stage[{index}].id must be non-empty")
        if stage_id in seen_ids:
            raise RouteValidationError(f"duplicate stage id: {stage_id}")
        seen_ids.add(stage_id)
        if not isinstance(name, str) or not name.strip():
            raise RouteValidationError(f"stage[{index}].name must be non-empty")
        waypoint_raw = item.get("waypoints")
        if not isinstance(waypoint_raw, list) or not waypoint_raw:
            raise RouteValidationError(f"stage[{index}] must contain waypoints")
        waypoints = tuple(
            _point(value, f"stage[{index}].waypoints[{wp_index}]")
            for wp_index, value in enumerate(waypoint_raw)
        )
        endpoint = _point(item.get("endpoint"), f"stage[{index}].endpoint")
        if math.hypot(waypoints[-1].x - endpoint.x, waypoints[-1].y - endpoint.y) > 1e-6:
            raise RouteValidationError(f"stage[{index}] final waypoint must match endpoint")
        stages.append(MissionStage(stage_id, name, endpoint, waypoints))

    return MissionRoute(start=start, stages=tuple(stages))


class MissionControllerCore:
    """State and motion calculations with no ROS dependencies."""

    def __init__(self, route: MissionRoute, config: ControllerConfig | None = None):
        self.route = route
        self.config = config or ControllerConfig()
        self.state = MissionState.IDLE
        self.failure_code: str | None = None
        self.started_at: float | None = None
        self.stage_index = 0
        self.waypoint_index = 0
        self.stage_results: list[dict[str, Any]] = []
        self.last_odom_at: float | None = None
        self.last_update_at: float | None = None
        self.obstacle_since: float | None = None
        self._progress_target: tuple[int, int] | None = None
        self._best_distance: float | None = None
        self._best_heading_error: float | None = None
        self._progress_at: float | None = None
        self.failed_stage_id: str | None = None
        self.failed_waypoint_index: int | None = None

    def start(self, now: float) -> OperationResult:
        if self.state is MissionState.RUNNING:
            return OperationResult(False, "mission already running")
        if self.state is MissionState.ESTOPPED:
            return OperationResult(False, "emergency stop is latched")
        self.state = MissionState.RUNNING
        self.failure_code = None
        self.started_at = now
        self.stage_index = 0
        self.waypoint_index = 0
        self.stage_results = []
        self.last_odom_at = None
        self.last_update_at = now
        self.obstacle_since = None
        self._reset_progress()
        self.failed_stage_id = None
        self.failed_waypoint_index = None
        return OperationResult(True, "mission started")

    def cancel(self) -> OperationResult:
        if self.state is not MissionState.RUNNING:
            return OperationResult(False, "mission is not running")
        self.state = MissionState.CANCELLED
        return OperationResult(True, "mission cancelled")

    def estop(self) -> OperationResult:
        if self.state is MissionState.ESTOPPED:
            return OperationResult(False, "emergency stop already latched")
        self.state = MissionState.ESTOPPED
        return OperationResult(True, "emergency stop latched")

    def reset(self) -> OperationResult:
        if self.state is MissionState.RUNNING:
            return OperationResult(False, "cancel the running mission before reset")
        self.state = MissionState.IDLE
        self.failure_code = None
        self.started_at = None
        self.stage_index = 0
        self.waypoint_index = 0
        self.stage_results = []
        self.last_odom_at = None
        self.last_update_at = None
        self.obstacle_since = None
        self._reset_progress()
        self.failed_stage_id = None
        self.failed_waypoint_index = None
        return OperationResult(True, "controller reset")

    def _reset_progress(self) -> None:
        self._progress_target = None
        self._best_distance = None
        self._best_heading_error = None
        self._progress_at = None

    def _fail(self, code: str) -> VelocityCommand:
        self.state = MissionState.FAILED
        self.failure_code = code
        if self.stage_index < len(self.route.stages):
            self.failed_stage_id = self.route.stages[self.stage_index].id
            self.failed_waypoint_index = self.waypoint_index
        return VelocityCommand()

    def update(
        self,
        pose: Pose2D | None,
        now: float,
        front_range: float = math.inf,
        odom_received_at: float | None = None,
    ) -> VelocityCommand:
        self.last_update_at = now
        if self.state is not MissionState.RUNNING:
            return VelocityCommand()

        if self.started_at is not None and now - self.started_at > self.config.mission_timeout:
            return self._fail("MISSION_TIMEOUT")

        if pose is not None:
            self.last_odom_at = now if odom_received_at is None else odom_received_at
            if now - self.last_odom_at > self.config.odom_timeout:
                return self._fail("ODOM_STALE")
        else:
            odom_reference = self.last_odom_at
            if odom_reference is None:
                odom_reference = self.started_at
            if odom_reference is not None and now - odom_reference > self.config.odom_timeout:
                return self._fail("ODOM_STALE")
            return VelocityCommand()

        obstacle_close = math.isfinite(front_range) and front_range < self.config.obstacle_stop_distance
        if obstacle_close:
            if self.obstacle_since is None:
                self.obstacle_since = now
            elif now - self.obstacle_since > self.config.obstacle_fail_after:
                return self._fail("OBSTACLE_BLOCKED")
            return VelocityCommand()
        self.obstacle_since = None

        while self.state is MissionState.RUNNING:
            stage = self.route.stages[self.stage_index]
            target = stage.waypoints[self.waypoint_index]
            distance = math.hypot(target.x - pose.x, target.y - pose.y)
            if distance > self.config.waypoint_tolerance:
                break
            self.waypoint_index += 1
            self._reset_progress()
            if self.waypoint_index < len(stage.waypoints):
                continue
            endpoint_error = math.hypot(stage.endpoint.x - pose.x, stage.endpoint.y - pose.y)
            elapsed = now - self.started_at if self.started_at is not None else 0.0
            self.stage_results.append(
                {
                    "id": stage.id,
                    "name": stage.name,
                    "endpoint_error": endpoint_error,
                    "elapsed": elapsed,
                }
            )
            self.stage_index += 1
            self.waypoint_index = 0
            if self.stage_index == len(self.route.stages):
                self.state = MissionState.SUCCEEDED
                return VelocityCommand()

        target = self.route.stages[self.stage_index].waypoints[self.waypoint_index]
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        absolute_heading_error = abs(heading_error)
        target_identity = (self.stage_index, self.waypoint_index)
        if self._progress_target != target_identity:
            self._progress_target = target_identity
            self._best_distance = distance
            self._best_heading_error = absolute_heading_error
            self._progress_at = now
        elif self._best_distance is not None and distance <= self._best_distance - self.config.progress_epsilon:
            self._best_distance = distance
            self._best_heading_error = absolute_heading_error
            self._progress_at = now
        elif (
            self._best_heading_error is not None
            and absolute_heading_error
            <= self._best_heading_error - self.config.progress_heading_epsilon
        ):
            self._best_heading_error = absolute_heading_error
            self._progress_at = now
        elif self._progress_at is not None and now - self._progress_at > self.config.progress_timeout:
            return self._fail("WAYPOINT_NO_PROGRESS")
        angular = max(
            -self.config.max_angular,
            min(self.config.max_angular, self.config.angular_kp * heading_error),
        )
        if abs(heading_error) > self.config.align_threshold:
            return VelocityCommand(0.0, angular)
        linear = self.config.max_linear * min(1.0, distance / self.config.slow_distance)
        return VelocityCommand(linear, angular)

    def status(self) -> dict[str, Any]:
        stage = self.route.stages[self.stage_index] if self.stage_index < len(self.route.stages) else None
        elapsed = 0.0
        if self.started_at is not None and self.last_update_at is not None:
            elapsed = max(0.0, self.last_update_at - self.started_at)
        return {
            "state": self.state.value,
            "failure_code": self.failure_code,
            "stage_index": self.stage_index,
            "stage_id": stage.id if stage else None,
            "stage_name": stage.name if stage else None,
            "waypoint_index": self.waypoint_index,
            "stage_results": list(self.stage_results),
            "elapsed": elapsed,
            "failed_stage_id": self.failed_stage_id,
            "failed_waypoint_index": self.failed_waypoint_index,
        }
