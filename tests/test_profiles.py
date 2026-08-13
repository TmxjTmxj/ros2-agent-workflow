from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from agent_ros.errors import ProfileValidationError
from agent_ros.profiles.loader import load_robot_profile, load_task_profile


def _robot_document(**overrides):
    document = {
        "name": "robot",
        "mode": "simulation",
        "namespace": "/robot",
        "frames": {"base": "base_link", "odom": "odom"},
        "adapter": {"kind": "twist"},
        "interfaces": {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        },
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
        "observation_sources": ["odometry"],
    }
    for key, value in overrides.items():
        document[key] = value
    return document


def _task_document(**overrides):
    document = {
        "name": "task",
        "robot_profile": "robot",
        "stages": [
            {
                "name": "delivery",
                "goal": {"frame": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
                "tolerance": 0.25,
                "timeout": 30.0,
            }
        ],
        "required_sensors": ["odometry"],
        "evidence": {"sources": ["camera"]},
        "recovery_policy": "stop",
    }
    for key, value in overrides.items():
        document[key] = value
    return document


def _write_yaml(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document), encoding="utf-8")


def write_profile(root: Path, **overrides) -> None:
    _write_yaml(root / "robots" / "robot.yaml", _robot_document(**overrides))


def test_hardware_profile_requires_positive_limits(tmp_path):
    write_profile(tmp_path, mode="hardware", limits={
        "max_linear_velocity": 0,
        "max_angular_velocity": 1.0,
        "max_linear_acceleration": 0.5,
        "max_angular_acceleration": 1.0,
    })
    with pytest.raises(ProfileValidationError, match="positive"):
        load_robot_profile("robot", tmp_path)


def test_profile_name_cannot_escape_profile_root(tmp_path):
    with pytest.raises(ProfileValidationError, match="profile name"):
        load_robot_profile("../secret", tmp_path)


def test_robot_profile_is_immutable_and_parses_standard_twist_interfaces(tmp_path):
    write_profile(tmp_path)
    profile = load_robot_profile("robot", tmp_path)

    assert profile.adapter.kind == "twist"
    assert profile.interfaces.command.type == "geometry_msgs/msg/Twist"
    with pytest.raises(AttributeError):
        profile.mode = "hardware"


@pytest.mark.parametrize("invalid_value", [math.inf, math.nan, -0.1, 0])
def test_robot_profile_rejects_non_finite_or_non_positive_limits(tmp_path, invalid_value):
    write_profile(tmp_path, limits={
        "max_linear_velocity": invalid_value,
        "max_angular_velocity": 1.0,
        "max_linear_acceleration": 0.5,
        "max_angular_acceleration": 1.0,
    })
    with pytest.raises(ProfileValidationError, match="finite and positive"):
        load_robot_profile("robot", tmp_path)


def test_profile_rejects_non_finite_numbers_even_in_unrecognized_fields(tmp_path):
    write_profile(tmp_path, unreviewed_limit=math.nan)
    with pytest.raises(ProfileValidationError, match="finite"):
        load_robot_profile("robot", tmp_path)


def test_robot_profile_rejects_unreviewed_fields(tmp_path):
    write_profile(tmp_path, unreviewed_limit=0.1)
    with pytest.raises(ProfileValidationError, match="unknown field"):
        load_robot_profile("robot", tmp_path)


@pytest.mark.parametrize("field", ["namespace", "safety", "observation_sources"])
def test_robot_profile_rejects_schema_required_fields_when_omitted(tmp_path, field):
    document = _robot_document()
    del document[field]
    _write_yaml(tmp_path / "robots" / "robot.yaml", document)

    with pytest.raises(ProfileValidationError):
        load_robot_profile("robot", tmp_path)


@pytest.mark.parametrize(
    "field", ["name", "robot_profile", "stages", "required_sensors", "evidence", "recovery_policy"]
)
def test_task_profile_rejects_schema_required_fields_when_omitted(tmp_path, field):
    document = _task_document()
    del document[field]
    _write_yaml(tmp_path / "tasks" / "task.yaml", document)

    with pytest.raises(ProfileValidationError):
        load_task_profile("task", tmp_path)


def test_twist_profile_requires_odometry(tmp_path):
    write_profile(tmp_path, interfaces={
        "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
    })
    with pytest.raises(ProfileValidationError, match="odometry"):
        load_robot_profile("robot", tmp_path)


def test_nav2_profile_requires_navigate_to_pose_action_type(tmp_path):
    write_profile(tmp_path, adapter={"kind": "nav2"}, interfaces={
        "navigation": {"action": "/navigate_to_pose", "type": "wrong/action/Type"},
    })
    with pytest.raises(ProfileValidationError, match="NavigateToPose"):
        load_robot_profile("robot", tmp_path)


def test_hardware_profile_requires_explicit_safety_configuration(tmp_path):
    write_profile(tmp_path, mode="hardware", safety={})
    with pytest.raises(ProfileValidationError, match="hardware safety"):
        load_robot_profile("robot", tmp_path)


def test_task_profile_requires_non_empty_ordered_stages_and_finite_goal_values(tmp_path):
    _write_yaml(tmp_path / "tasks" / "task.yaml", _task_document(stages=[]))
    with pytest.raises(ProfileValidationError, match="non-empty"):
        load_task_profile("task", tmp_path)

    _write_yaml(tmp_path / "tasks" / "task.yaml", _task_document(stages=[{
        "name": "delivery", "goal": {"frame": "map", "x": math.inf, "y": 2.0, "yaw": 0.0},
        "tolerance": 0.25, "timeout": 30.0,
    }]))
    with pytest.raises(ProfileValidationError, match="finite"):
        load_task_profile("task", tmp_path)


@pytest.mark.parametrize("field, value", [("tolerance", 0), ("timeout", -1)])
def test_task_profile_requires_positive_tolerance_and_timeout(tmp_path, field, value):
    stage = _task_document()["stages"][0]
    stage[field] = value
    _write_yaml(tmp_path / "tasks" / "task.yaml", _task_document(stages=[stage]))
    with pytest.raises(ProfileValidationError, match="positive"):
        load_task_profile("task", tmp_path)


def test_reviewed_hospital_profiles_load_from_repository_profiles_root():
    root = Path(__file__).parents[1] / "profiles"
    robot = load_robot_profile("hospital-amr", root)
    task = load_task_profile("hospital-delivery", root)

    assert robot.mode == "simulation"
    assert robot.adapter.kind == "twist"
    assert tuple(stage.name for stage in task.stages) == (
        "corridor-to-pharmacy",
        "pharmacy-to-ward-2",
        "ward-2-to-laboratory",
    )
