"""Constrained profile loading from the repository-owned profile directory."""

from __future__ import annotations

import re
from math import isfinite
from pathlib import Path
from typing import TypeVar

import yaml
from agent_ros.errors import ProfileValidationError
from agent_ros.profiles.models import RobotProfile, TaskProfile

_PROFILE_NAME = re.compile(r"[a-z0-9][a-z0-9-]*")
_Profile = TypeVar("_Profile", RobotProfile, TaskProfile)


def load_robot_profile(name: str, root: Path) -> RobotProfile:
    return _load(name, root, "robots", RobotProfile)


def load_task_profile(name: str, root: Path) -> TaskProfile:
    return _load(name, root, "tasks", TaskProfile)


def _load(name: str, root: Path, directory: str, model: type[_Profile]) -> _Profile:
    if not isinstance(name, str) or not _PROFILE_NAME.fullmatch(name):
        raise ProfileValidationError("profile name must use lowercase letters, digits, and hyphens only")
    profile_root = (Path(root) / directory).resolve()
    path = (profile_root / f"{name}.yaml").resolve()
    if path.parent != profile_root:
        raise ProfileValidationError("profile name resolves outside profile root")
    try:
        with path.open(encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except FileNotFoundError as exc:
        raise ProfileValidationError(f"profile not found: {name}") from exc
    except yaml.YAMLError as exc:
        raise ProfileValidationError(f"invalid YAML profile: {name}") from exc
    _reject_non_finite_numbers(raw)
    profile = model.from_mapping(raw)
    if profile.name != name:
        raise ProfileValidationError("profile name must match its filename")
    return profile


def _reject_non_finite_numbers(value: object) -> None:
    """YAML permits .nan/.inf, which are never safe profile values."""
    if isinstance(value, float) and not isfinite(value):
        raise ProfileValidationError("profile numeric values must be finite and positive where used as limits")
    if isinstance(value, dict):
        for item in value.values():
            _reject_non_finite_numbers(item)
    elif isinstance(value, list):
        for item in value:
            _reject_non_finite_numbers(item)
