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

    assert project["project"]["readme"]["file"] == "README.en.md"
    assert {"ruff", "mypy", "pytest-cov", "pip-audit", "pre-commit"} <= {
        tool.split("=", 1)[0].split(">", 1)[0] for tool in tools
    }


def test_ci_runs_installed_wheel_smoke_and_quality_gates():
    workflow = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "make smoke-wheel" in workflow
    assert "make check" in workflow
    assert 'install -e ".[dev]"' in workflow


def test_release_workflow_validates_metadata_and_artifact_manifest():
    workflow = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")

    assert "twine check dist/*" in workflow
    assert "verify_release_candidate.py --dist-dir dist" in workflow


def test_container_declares_reference_ros_environment():
    dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
    compose = (ROOT / "docker-compose.yml").read_text(encoding="utf-8")
    devcontainer = (ROOT / ".devcontainer" / "devcontainer.json").read_text(encoding="utf-8")
    dockerignore = (ROOT / ".dockerignore").read_text(encoding="utf-8")

    assert "ubuntu:26.04" in dockerfile
    assert "ros-lyrical" in dockerfile
    assert "ros-lyrical-gz-sim-vendor" in dockerfile
    assert "AS control-plane" in dockerfile
    assert "AS hospital-runtime" in dockerfile
    assert "docker-build" in makefile
    assert "docker-control-build" in makefile
    assert "docker-smoke" in makefile
    assert "docker-hospital" in makefile
    assert "AGENT_ROS_UID" in compose
    assert "control-plane:" in compose
    assert "AGENT_ROS_UID" in makefile
    assert '"target": "hospital-runtime"' in devcontainer
    assert "Dockerfile" in devcontainer
    assert "build/" in dockerignore
    assert ".worktrees" in dockerignore


def test_nightly_hospital_workflow_preserves_validated_evidence():
    workflow = (ROOT / ".github" / "workflows" / "nightly-hospital.yml").read_text(encoding="utf-8")
    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")

    assert "workflow_dispatch" in workflow
    assert "cron:" in workflow
    assert "make docker-hospital" in workflow
    assert "make docker-hospital-preflight" in workflow
    assert "make docker-mcp-trace" in workflow
    assert "environment-preflight.json" in makefile
    assert "verify_release_artifacts.py" in workflow
    assert "retention-days: 30" in workflow
    assert "self-hosted" in workflow
    assert "ros-gazebo" in workflow


def test_readmes_link_to_each_other_and_release_docs():
    chinese_readme = (ROOT / "README.md").read_text(encoding="utf-8")
    english_readme = (ROOT / "README.en.md").read_text(encoding="utf-8")

    assert "README.en.md" in chinese_readme
    assert "README.md" in english_readme
    assert (ROOT / "docs" / "RELEASE.md").is_file()
    assert (ROOT / "docs" / "RUNNER.md").is_file()
    assert (ROOT / "docs" / "VERIFICATION-BASELINE.md").is_file()
    assert (ROOT / "docs" / "ADAPTER-MIGRATION.md").is_file()
    assert "docker-hospital-preflight" in chinese_readme
    assert "docker-hospital-preflight" in english_readme
    assert "release-verify" in chinese_readme
    assert "release-verify" in english_readme
    assert "VERIFICATION-BASELINE.md" in chinese_readme
    assert "VERIFICATION-BASELINE.md" in english_readme
    assert "505" not in chinese_readme
    assert "388 root tests passed" in english_readme
    assert "388 个根测试通过" in chinese_readme
    assert '<img src="examples/hospital_delivery/evidence/acceptance-initial.png"' in chinese_readme
    assert '<img src="examples/hospital_delivery/evidence/acceptance-final.png"' in english_readme


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
