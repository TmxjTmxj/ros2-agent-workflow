from __future__ import annotations

import json
import subprocess
import sys
import venv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_built_wheel_installs_and_runs_cli_outside_repository(tmp_path):
    dist = tmp_path / "dist"
    builder = tmp_path / "builder"
    venv.EnvBuilder(with_pip=True).create(builder)
    subprocess.run(
        [
            str(builder / "bin" / "python"),
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--wheel-dir",
            str(dist),
            str(ROOT),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    wheel = next(dist.glob("agent_ros-*.whl"))
    environment = tmp_path / "environment"
    venv.EnvBuilder(with_pip=True).create(environment)
    python = environment / "bin" / "python"
    subprocess.run(
        [str(python), "-m", "pip", "install", str(wheel)],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [str(environment / "bin" / "agent-ros"), "--json", "status", "hospital-amr"],
        check=True,
        capture_output=True,
        cwd=tmp_path,
        text=True,
    )

    assert json.loads(completed.stdout) == {"profile": "hospital-amr", "state": "NEW"}
