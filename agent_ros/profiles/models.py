"""Immutable, validated data models for reviewed robot and task profiles."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Mapping

from agent_ros.errors import ProfileValidationError


TWIST_TYPE = "geometry_msgs/msg/Twist"
ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
NAVIGATE_TO_POSE_TYPE = "nav2_msgs/action/NavigateToPose"
FOLLOW_JOINT_TRAJECTORY_TYPE = "control_msgs/action/FollowJointTrajectory"
ADAPTER_KINDS = frozenset({"twist", "nav2", "follow_joint_trajectory"})


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileValidationError(f"{label} must be an object")
    return value


def _only_keys(value: Mapping[str, Any], allowed: set[str], label: str) -> None:
    unexpected = set(value) - allowed
    if unexpected:
        raise ProfileValidationError(f"{label} contains unknown field: {sorted(unexpected)[0]}")


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProfileValidationError(f"{label} must be a non-empty string")
    return value


def _finite_positive(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileValidationError(f"{label} must be finite and positive")
    parsed = float(value)
    if not isfinite(parsed) or parsed <= 0:
        raise ProfileValidationError(f"{label} must be finite and positive")
    return parsed


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileValidationError(f"{label} must be finite")
    parsed = float(value)
    if not isfinite(parsed):
        raise ProfileValidationError(f"{label} must be finite")
    return parsed


@dataclass(frozen=True, slots=True)
class AdapterConfig:
    kind: str


@dataclass(frozen=True, slots=True)
class InterfaceReference:
    type: str
    topic: str | None = None
    action: str | None = None


@dataclass(frozen=True, slots=True)
class RobotInterfaces:
    command: InterfaceReference | None = None
    odometry: InterfaceReference | None = None
    navigation: InterfaceReference | None = None
    trajectory: InterfaceReference | None = None


@dataclass(frozen=True, slots=True)
class MotionLimits:
    max_linear_velocity: float
    max_angular_velocity: float
    max_linear_acceleration: float
    max_angular_acceleration: float


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    heartbeat_timeout: float | None = None
    estop_topic: str | None = None


@dataclass(frozen=True, slots=True)
class RobotProfile:
    name: str
    mode: str
    namespace: str
    frames: tuple[tuple[str, str], ...]
    adapter: AdapterConfig
    interfaces: RobotInterfaces
    limits: MotionLimits
    safety: SafetyConfig
    observation_sources: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object) -> "RobotProfile":
        data = _mapping(value, "robot profile")
        _only_keys(data, {
            "name", "mode", "namespace", "frames", "adapter", "interfaces", "limits", "safety",
            "observation_sources",
        }, "robot profile")
        name = _text(data.get("name"), "robot name")
        mode = data.get("mode")
        if mode not in {"simulation", "hardware"}:
            raise ProfileValidationError("robot mode must be simulation or hardware")
        namespace = _text(data.get("namespace", "/"), "namespace")

        frames_data = _mapping(data.get("frames"), "frames")
        frames = tuple((str(key), _text(item, f"frame {key}")) for key, item in frames_data.items())
        if not frames:
            raise ProfileValidationError("frames must not be empty")

        adapter_data = _mapping(data.get("adapter"), "adapter")
        _only_keys(adapter_data, {"kind"}, "adapter")
        adapter_kind = adapter_data.get("kind")
        if adapter_kind not in ADAPTER_KINDS:
            choices = ", ".join(sorted(ADAPTER_KINDS))
            raise ProfileValidationError(f"adapter kind must be one of: {choices}")
        adapter = AdapterConfig(kind=adapter_kind)

        interfaces_data = _mapping(data.get("interfaces"), "interfaces")
        _only_keys(interfaces_data, {"command", "odometry", "navigation", "trajectory"}, "interfaces")

        def interface(key: str, address: str) -> InterfaceReference | None:
            raw = interfaces_data.get(key)
            if raw is None:
                return None
            item = _mapping(raw, f"interfaces.{key}")
            _only_keys(item, {"type", address}, f"interfaces.{key}")
            return InterfaceReference(
                type=_text(item.get("type"), f"interfaces.{key}.type"),
                **{address: _text(item.get(address), f"interfaces.{key}.{address}")},
            )

        interfaces = RobotInterfaces(
            command=interface("command", "topic"),
            odometry=interface("odometry", "topic"),
            navigation=interface("navigation", "action"),
            trajectory=interface("trajectory", "action"),
        )
        _validate_adapter_interfaces(adapter.kind, interfaces)

        limits_data = _mapping(data.get("limits"), "limits")
        _only_keys(limits_data, {
            "max_linear_velocity", "max_angular_velocity", "max_linear_acceleration",
            "max_angular_acceleration",
        }, "limits")
        limits = MotionLimits(
            max_linear_velocity=_finite_positive(limits_data.get("max_linear_velocity"), "max_linear_velocity"),
            max_angular_velocity=_finite_positive(limits_data.get("max_angular_velocity"), "max_angular_velocity"),
            max_linear_acceleration=_finite_positive(limits_data.get("max_linear_acceleration"), "max_linear_acceleration"),
            max_angular_acceleration=_finite_positive(limits_data.get("max_angular_acceleration"), "max_angular_acceleration"),
        )

        safety_data = _mapping(data.get("safety", {}), "safety")
        _only_keys(safety_data, {"heartbeat_timeout", "estop_topic"}, "safety")
        heartbeat_raw = safety_data.get("heartbeat_timeout")
        safety = SafetyConfig(
            heartbeat_timeout=(None if heartbeat_raw is None else _finite_positive(heartbeat_raw, "heartbeat_timeout")),
            estop_topic=(None if safety_data.get("estop_topic") is None else _text(safety_data["estop_topic"], "estop_topic")),
        )
        if mode == "hardware" and (safety.heartbeat_timeout is None or safety.estop_topic is None):
            raise ProfileValidationError("hardware safety configuration requires heartbeat_timeout and estop_topic")

        sources = data.get("observation_sources", [])
        if not isinstance(sources, list) or not all(isinstance(source, str) and source for source in sources):
            raise ProfileValidationError("observation_sources must be a list of non-empty strings")
        return cls(name, mode, namespace, frames, adapter, interfaces, limits, safety, tuple(sources))


def _validate_adapter_interfaces(kind: str, interfaces: RobotInterfaces) -> None:
    if kind == "twist":
        if interfaces.command is None or interfaces.command.type != TWIST_TYPE:
            raise ProfileValidationError("Twist adapter requires geometry_msgs/msg/Twist command interface")
        if interfaces.odometry is None or interfaces.odometry.type != ODOMETRY_TYPE:
            raise ProfileValidationError("Twist adapter requires nav_msgs/msg/Odometry odometry interface")
    elif kind == "nav2":
        if interfaces.navigation is None or interfaces.navigation.type != NAVIGATE_TO_POSE_TYPE:
            raise ProfileValidationError("Nav2 adapter requires nav2_msgs/action/NavigateToPose action type")
    elif kind == "follow_joint_trajectory":
        if interfaces.trajectory is None or interfaces.trajectory.type != FOLLOW_JOINT_TRAJECTORY_TYPE:
            raise ProfileValidationError("trajectory adapter requires control_msgs/action/FollowJointTrajectory action type")


@dataclass(frozen=True, slots=True)
class PoseGoal:
    frame: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True, slots=True)
class TaskStage:
    name: str
    goal: PoseGoal
    tolerance: float
    timeout: float


@dataclass(frozen=True, slots=True)
class TaskProfile:
    name: str
    robot_profile: str
    stages: tuple[TaskStage, ...]
    required_sensors: tuple[str, ...]
    evidence_sources: tuple[str, ...]
    recovery_policy: str

    @classmethod
    def from_mapping(cls, value: object) -> "TaskProfile":
        data = _mapping(value, "task profile")
        _only_keys(data, {
            "name", "robot_profile", "stages", "required_sensors", "evidence", "recovery_policy",
        }, "task profile")
        name = _text(data.get("name"), "task name")
        robot_profile = _text(data.get("robot_profile"), "robot_profile")
        stages_raw = data.get("stages")
        if not isinstance(stages_raw, list) or not stages_raw:
            raise ProfileValidationError("task stages must be a non-empty ordered list")
        stages: list[TaskStage] = []
        names: set[str] = set()
        for index, raw in enumerate(stages_raw):
            stage = _mapping(raw, f"stages[{index}]")
            _only_keys(stage, {"name", "goal", "tolerance", "timeout"}, f"stages[{index}]")
            stage_name = _text(stage.get("name"), f"stages[{index}].name")
            if stage_name in names:
                raise ProfileValidationError("task stage names must be unique and ordered")
            names.add(stage_name)
            goal_data = _mapping(stage.get("goal"), f"stages[{index}].goal")
            _only_keys(goal_data, {"frame", "x", "y", "yaw"}, f"stages[{index}].goal")
            goal = PoseGoal(
                frame=_text(goal_data.get("frame"), f"stages[{index}].goal.frame"),
                x=_finite(goal_data.get("x"), f"stages[{index}].goal.x"),
                y=_finite(goal_data.get("y"), f"stages[{index}].goal.y"),
                yaw=_finite(goal_data.get("yaw"), f"stages[{index}].goal.yaw"),
            )
            stages.append(TaskStage(
                name=stage_name,
                goal=goal,
                tolerance=_finite_positive(stage.get("tolerance"), f"stages[{index}].tolerance"),
                timeout=_finite_positive(stage.get("timeout"), f"stages[{index}].timeout"),
            ))
        required_sensors = _string_list(data.get("required_sensors", []), "required_sensors")
        evidence = _mapping(data.get("evidence", {}), "evidence")
        _only_keys(evidence, {"sources"}, "evidence")
        evidence_sources = _string_list(evidence.get("sources", []), "evidence.sources")
        recovery_policy = data.get("recovery_policy", "stop")
        if recovery_policy not in {"stop", "cancel_and_stop"}:
            raise ProfileValidationError("recovery_policy must be stop or cancel_and_stop")
        return cls(name, robot_profile, tuple(stages), required_sensors, evidence_sources, recovery_policy)


def _string_list(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) and item for item in value):
        raise ProfileValidationError(f"{label} must be a list of non-empty strings")
    return tuple(value)
