#!/usr/bin/env python3
"""Report whether a host can run the ROS/Gazebo hospital container.

This preflight intentionally does not estimate real-time factor or execute a
hospital mission.  The independent acceptance report remains the only evidence
that a runner meets the configured demonstration budget.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from collections.abc import Callable
from pathlib import Path
from typing import Any

CommandRunner = Callable[[list[str]], tuple[int, str, str]]


def _run(command: list[str]) -> tuple[int, str, str]:
    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
    except OSError as exc:
        return 127, "", str(exc)
    return completed.returncode, completed.stdout, completed.stderr


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _cgroup_limits(root: Path) -> dict[str, int | float | None]:
    memory_limit = _read_text(root / "memory.max")
    cpu_limit = _read_text(root / "cpu.max")
    memory_bytes = int(memory_limit) if memory_limit and memory_limit != "max" and memory_limit.isdigit() else None
    cpu_cores: float | None = None
    if cpu_limit:
        quota, *rest = cpu_limit.split()
        if quota != "max" and rest and quota.isdigit() and rest[0].isdigit() and int(rest[0]) > 0:
            cpu_cores = int(quota) / int(rest[0])
    return {"memory_bytes": memory_bytes, "cpu_cores": cpu_cores}


def _container_probe(run: CommandRunner) -> dict[str, Any]:
    command = [
        "docker",
        "compose",
        "run",
        "--rm",
        "--no-deps",
        "agent-ros",
        "bash",
        "-lc",
        "test -f /opt/ros/lyrical/setup.bash && command -v ros2 && command -v gz",
    ]
    code, stdout, stderr = run(command)
    return {
        "ready": code == 0,
        "command": command,
        "details": stdout.strip() if code == 0 else stderr.strip() or stdout.strip(),
    }


def collect_environment(
    *,
    run: CommandRunner = _run,
    cpu_count: Callable[[], int | None] = os.cpu_count,
    path_exists: Callable[[Path], bool] = Path.exists,
    cgroup_root: Path = Path("/sys/fs/cgroup"),
) -> dict[str, Any]:
    """Collect host/runtime facts without starting a hospital mission."""
    code, stdout, stderr = run(["docker", "info", "--format", "{{json .Runtimes}}"])
    runtimes: list[str] = []
    docker_error: str | None = None
    if code == 0:
        try:
            parsed = json.loads(stdout)
            if isinstance(parsed, dict):
                runtimes = sorted(str(name) for name in parsed)
            else:
                docker_error = "Docker returned a non-object runtimes payload"
        except json.JSONDecodeError:
            docker_error = "Docker returned invalid runtimes JSON"
    else:
        docker_error = stderr.strip() or stdout.strip() or "docker info failed"

    gpu_devices = [str(path) for path in (Path("/dev/nvidia0"), Path("/dev/dri")) if path_exists(path)]
    docker_available = code == 0 and docker_error is None
    container_runtime = (
        _container_probe(run)
        if docker_available
        else {"ready": False, "command": None, "details": "Docker daemon is unavailable"}
    )
    return {
        "schema_version": 1,
        "host": {"cpu_count": cpu_count(), "cgroup_limits": _cgroup_limits(cgroup_root), "gpu_devices": gpu_devices},
        "docker": {"available": docker_available, "runtimes": runtimes, "error": docker_error},
        "container_runtime": container_runtime,
        "accelerated_runtime_available": "nvidia" in runtimes and bool(gpu_devices),
        "performance_note": (
            "A passing preflight does not prove RTF. Run the unchanged independent acceptance workflow "
            "to qualify this runner."
        ),
    }


def preflight_exit_code(report: dict[str, Any], *, require_container: bool, require_accelerated_runtime: bool) -> int:
    """Evaluate explicit policies while keeping diagnostic-only preflight non-blocking."""
    if require_container and not report["container_runtime"]["ready"]:
        return 2
    if require_accelerated_runtime and not report["accelerated_runtime_available"]:
        return 2
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    parser.add_argument(
        "--require-container",
        action="store_true",
        help="fail unless ROS/Gazebo commands resolve in the image",
    )
    parser.add_argument(
        "--require-accelerated-runtime",
        action="store_true",
        help="fail unless Docker exposes an NVIDIA runtime and GPU device",
    )
    args = parser.parse_args(argv)
    report = collect_environment()
    encoded = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return preflight_exit_code(
        report,
        require_container=args.require_container,
        require_accelerated_runtime=args.require_accelerated_runtime,
    )


if __name__ == "__main__":
    raise SystemExit(main())
