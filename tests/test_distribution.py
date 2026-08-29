from __future__ import annotations

import asyncio
import json
import os
import subprocess
import tomllib
import venv
from pathlib import Path

from fastmcp import Client
from fastmcp.client.transports import StdioTransport

ROOT = Path(__file__).resolve().parents[1]


def test_readme_documents_the_installed_control_plane():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    assert "Profile → SafetyGateway → Adapter → evidence" in readme
    assert "agent-ros --json status hospital-amr" in readme
    assert "agent-ros-mcp" in readme
    assert "make check" in readme


def test_project_declares_dev_quality_tools():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    tools = set(project["project"]["optional-dependencies"]["dev"])

    assert {"ruff", "mypy", "pytest-cov", "pip-audit", "pre-commit"} <= {
        tool.split("=", 1)[0].split(">", 1)[0] for tool in tools
    }


def test_ci_runs_installed_wheel_smoke_and_quality_gates():
    workflow = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "make smoke-wheel" in workflow
    assert "make check" in workflow
    assert 'install -e ".[dev]"' in workflow


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

    async def verify_installed_mcp() -> None:
        transport = StdioTransport(
            command=str(environment / "bin" / "agent-ros-mcp"),
            args=[],
            cwd=str(tmp_path),
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
            keep_alive=False,
        )
        async with Client(transport) as client:
            assert {tool.name for tool in await client.list_tools()} >= {
                "connection_status",
                "emergency_stop",
                "run_task",
            }

    asyncio.run(verify_installed_mcp())
