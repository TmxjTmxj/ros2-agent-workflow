"""Access to the packaged default workflow profiles."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def default_profiles_root() -> Path:
    """Return the package-owned directory containing default profiles."""
    return Path(str(files("agent_ros.resources").joinpath("profiles")))
