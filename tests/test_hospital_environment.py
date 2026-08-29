from __future__ import annotations

from pathlib import Path

from scripts.check_hospital_environment import collect_environment, preflight_exit_code


def _runner_with_ros_container(command: list[str]) -> tuple[int, str, str]:
    if command[:2] == ["docker", "info"]:
        return 0, '{"runc": {}, "nvidia": {}}', ""
    if command[:3] == ["docker", "compose", "run"]:
        return 0, "/opt/ros/lyrical/setup.bash\n/usr/bin/ros2\n/usr/bin/gz\n", ""
    raise AssertionError(f"unexpected command: {command}")


def test_preflight_reports_container_capabilities_without_running_a_mission():
    report = collect_environment(
        run=_runner_with_ros_container,
        cpu_count=lambda: 12,
        path_exists=lambda path: path == Path("/dev/nvidia0"),
        cgroup_root=Path("/absent"),
    )

    assert report["docker"]["available"] is True
    assert report["docker"]["runtimes"] == ["nvidia", "runc"]
    assert report["container_runtime"]["ready"] is True
    assert report["accelerated_runtime_available"] is True
    assert report["host"]["cpu_count"] == 12


def test_preflight_reports_absent_docker_without_throwing():
    report = collect_environment(
        run=lambda _command: (1, "", "docker daemon unavailable"),
        cpu_count=lambda: 4,
        path_exists=lambda _path: False,
        cgroup_root=Path("/absent"),
    )

    assert report["docker"]["available"] is False
    assert report["container_runtime"]["ready"] is False
    assert preflight_exit_code(report, require_container=False, require_accelerated_runtime=False) == 0
    assert preflight_exit_code(report, require_container=True, require_accelerated_runtime=False) == 2


def test_preflight_fails_explicit_accelerated_runtime_policy():
    report = collect_environment(
        run=lambda _command: (0, '{"runc": {}}', ""),
        cpu_count=lambda: 8,
        path_exists=lambda _path: False,
        cgroup_root=Path("/absent"),
    )

    assert report["accelerated_runtime_available"] is False
    assert preflight_exit_code(report, require_container=False, require_accelerated_runtime=True) == 2
