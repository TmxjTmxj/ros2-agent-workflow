from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import subprocess

import yaml

from agent_ros.adapters.base import HospitalAction
from agent_ros.adapters.hospital import HospitalDeliveryAdapter, HospitalSimulationRuntime
from agent_ros.profiles.loader import load_robot_profile, load_task_profile


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "hospital_delivery"
PROFILES = ROOT / "profiles"


def test_reviewed_profiles_resolve_every_hospital_case_path():
    robot = load_robot_profile("hospital-amr", PROFILES)
    task = load_task_profile("hospital-delivery", PROFILES)

    assert robot.name == task.robot_profile
    for relative in (
        "config/mission_routes.json",
        "config/ros_gz_bridge.yaml",
        "models/hospital_amr/model.config",
        "models/hospital_amr/model.sdf",
        "worlds/hospital_world_gz10.world",
        "src/smartcar_bringup/package.xml",
        "src/smartcar_bringup/launch/hospital_delivery.launch.py",
        "src/smartcar_bringup/smartcar_bringup/controller_core.py",
        "src/smartcar_bringup/smartcar_bringup/mission_controller.py",
        "scripts/codex_project.py",
        "scripts/capture_camera.py",
        "scripts/verify_acceptance.py",
    ):
        assert (EXAMPLE / relative).is_file(), relative
    assert (ROOT / "scripts" / "demo_hospital.sh").is_file()


def test_task_profile_stage_endpoints_match_the_verified_route():
    task = load_task_profile("hospital-delivery", PROFILES)
    route = json.loads((EXAMPLE / "config" / "mission_routes.json").read_text())

    assert [stage.name for stage in task.stages] == [
        "corridor-to-pharmacy",
        "pharmacy-to-ward-2",
        "ward-2-to-laboratory",
    ]
    assert [(stage.goal.x, stage.goal.y) for stage in task.stages] == [
        tuple(stage["endpoint"]) for stage in route["stages"]
    ]
    assert {stage.goal.frame for stage in task.stages} == {"world"}


def test_bridge_has_exactly_five_scoped_hospital_contact_sources():
    bridges = yaml.safe_load(
        (EXAMPLE / "config" / "ros_gz_bridge.yaml").read_text(encoding="utf-8")
    )
    contacts = [
        bridge
        for bridge in bridges
        if bridge.get("ros_topic_name") == "/hospital_amr/contacts"
    ]

    assert len(contacts) == 5
    assert len({bridge["gz_topic_name"] for bridge in contacts}) == 5
    assert all(
        topic.startswith("/world/hospital_world/model/hospital_amr/")
        and "/sensor/" in topic
        and topic.endswith("/contact")
        for topic in (bridge["gz_topic_name"] for bridge in contacts)
    )


def test_launch_starts_exactly_one_fixed_mission_controller():
    launch_path = EXAMPLE / "src" / "smartcar_bringup" / "launch" / "hospital_delivery.launch.py"
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    mission_nodes = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "Node":
            continue
        values = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in node.keywords
            if keyword.arg in {"package", "executable", "name"}
        }
        if values.get("executable") == "mission_controller":
            mission_nodes.append(values)

    assert mission_nodes == [
        {
            "package": "smartcar_bringup",
            "executable": "mission_controller",
            "name": "hospital_mission_controller",
        }
    ]


def test_hospital_adapter_resolves_only_the_fixed_example_lifecycle_actions():
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)

    adapter.probe()
    adapter.validate()
    adapter.status()
    adapter.cancel()
    adapter.stop()
    adapter.observe("hospital_state")

    assert runtime.commands == (
        HospitalAction.PROBE,
        HospitalAction.VALIDATE,
        HospitalAction.STATUS,
        HospitalAction.CANCEL,
        HospitalAction.STOP,
        HospitalAction.OBSERVE,
    )
    assert {action.value for action in HospitalAction} == {
        "probe",
        "validate",
        "start",
        "status",
        "cancel",
        "stop",
        "observe",
    }


def test_hospital_demo_has_bounded_options_without_starting_ros():
    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "demo_hospital.sh"), "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 0
    assert "--headless" in result.stdout
    assert "--verify" in result.stdout
    assert "--runtime-dir" not in result.stdout


def _run_demo_with_fake_python(tmp_path, *, verify_status: int, stop_status: int):
    env = os.environ.copy()
    env["DEMO_VERIFY_STATUS"] = str(verify_status)
    env["DEMO_STOP_STATUS"] = str(stop_status)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_python = fake_bin / "python3"
    fake_python.write_text("""#!/usr/bin/env bash
      case "$1" in
        *verify_acceptance.py) exit "$DEMO_VERIFY_STATUS" ;;
      esac
      if [[ "$2" == stop ]]; then exit "$DEMO_STOP_STATUS"; fi
      exit 0
    """, encoding="utf-8")
    fake_python.chmod(0o755)
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    return subprocess.run(
        ["bash", str(ROOT / "scripts" / "demo_hospital.sh"), "--headless", "--verify"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=5,
        env=env,
    )


def test_hospital_demo_preserves_verification_failure_through_cleanup(tmp_path):
    result = _run_demo_with_fake_python(tmp_path, verify_status=7, stop_status=0)

    assert result.returncode == 7


def test_hospital_demo_separates_sim_deadline_from_bounded_wall_wait():
    source = (ROOT / "scripts" / "demo_hospital.sh").read_text(encoding="utf-8")

    assert "--timeout 300" in source
    assert "timeout --foreground --signal=TERM --kill-after=15s 360s" in source


def test_hospital_demo_surfaces_cleanup_failure_after_success(tmp_path):
    result = _run_demo_with_fake_python(tmp_path, verify_status=0, stop_status=9)

    assert result.returncode != 0
    assert "cleanup did not complete" in result.stderr


def test_acceptance_report_default_is_example_relative():
    from examples.hospital_delivery.scripts.verify_acceptance import DEFAULT_REPORT

    assert DEFAULT_REPORT == EXAMPLE / "logs" / "acceptance_report.json"


def test_acceptance_validator_is_standalone_from_the_example_root():
    result = subprocess.run(
        ["/usr/bin/python3", "scripts/verify_acceptance.py", "--help"],
        cwd=EXAMPLE,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 0, result.stderr
    assert "--validate" in result.stdout
