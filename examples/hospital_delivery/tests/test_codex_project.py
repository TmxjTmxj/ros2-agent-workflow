import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import pytest
from sensor_msgs.msg import Image

from scripts.capture_camera import CameraCaptureError, image_message_to_png
from scripts.codex_project import (
    _require_exclusive_controller,
    inspect_cmd_vel_publishers,
    load_runtime_state,
    parse_cmd_vel_publisher_endpoints,
    parse_cmd_vel_publishers,
    process_matches,
    record_process,
    terminate_managed_process,
    ProjectLifecycle,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_recorded_process_requires_pid_start_time_and_exact_command_match():
    """Catches PID reuse causing an unrelated process to be treated as managed."""
    proc = subprocess.Popen(["sleep", "30"])
    try:
        record = record_process(proc.pid, name="test-sleeper")
        assert process_matches(record)

        wrong_start = dict(record)
        wrong_start["start_time_ticks"] += 1
        assert not process_matches(wrong_start)
        assert not terminate_managed_process(wrong_start, term_timeout=0.05)
        assert proc.poll() is None

        wrong_command = dict(record)
        wrong_command["cmdline"] = ["not-the-recorded-command"]
        assert not process_matches(wrong_command)
        assert not terminate_managed_process(wrong_command, term_timeout=0.05)
        assert proc.poll() is None

        assert terminate_managed_process(record, term_timeout=1.0)
        proc.wait(timeout=1.0)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def test_load_runtime_state_discards_stale_identity_without_signalling(tmp_path):
    """Catches a stale state file claiming the current process belongs to it."""
    real = record_process(__import__("os").getpid(), name="pytest")
    real["start_time_ticks"] += 1
    state_path = tmp_path / "state.json"
    state_path.write_text(json.dumps({"version": 1, "processes": [real]}))

    state = load_runtime_state(state_path)

    assert state["processes"] == []
    assert state["stale_processes"] == [real]


def test_parse_cmd_vel_publishers_returns_only_publisher_node_names():
    """Catches a subscriber being mistaken for a competing velocity source."""
    output = """Type: geometry_msgs/msg/Twist

Publisher count: 2

Node name: hospital_mission_controller
Node namespace: /
Endpoint type: PUBLISHER
GID: one

Node name: forgotten_cli
Node namespace: /
Endpoint type: PUBLISHER
GID: two

Subscription count: 1

Node name: ros_gz_bridge
Node namespace: /
Endpoint type: SUBSCRIPTION
GID: three
"""

    assert parse_cmd_vel_publishers(output) == [
        "/hospital_mission_controller",
        "/forgotten_cli",
    ]
    assert parse_cmd_vel_publisher_endpoints(output) == [
        {"node": "/hospital_mission_controller", "gid": "one"},
        {"node": "/forgotten_cli", "gid": "two"},
    ]


def test_cmd_vel_inspection_fails_closed_and_exclusive_check_counts_gids(monkeypatch):
    failed = subprocess.CompletedProcess([], 2, "", "graph unavailable")
    monkeypatch.setattr("scripts.codex_project._run_ros", lambda *args, **kwargs: failed)
    assert inspect_cmd_vel_publishers()["ok"] is False

    duplicate = {
        "ok": True,
        "endpoints": [
            {"node": "/hospital_mission_controller", "gid": "one"},
            {"node": "/hospital_mission_controller", "gid": "two"},
        ],
    }
    monkeypatch.setattr("scripts.codex_project.inspect_cmd_vel_publishers", lambda: duplicate)
    with pytest.raises(Exception, match="exactly one"):
        _require_exclusive_controller()


def test_forged_runtime_record_is_not_authorized_to_signal_process():
    proc = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        forged = record_process(proc.pid, name="hospital-delivery-launch")
        assert not terminate_managed_process(
            forged, term_timeout=0.05, require_authorized=True
        )
        assert proc.poll() is None
    finally:
        proc.kill()
        proc.wait()


def test_failed_start_preserves_identity_when_cleanup_is_incomplete(tmp_path, monkeypatch):
    lifecycle = ProjectLifecycle(tmp_path)
    record = {
        "name": "hospital-delivery-launch",
        "pid": 123,
        "start_time_ticks": 1,
        "cmdline": [],
        "cwd": str(PROJECT_ROOT),
        "pgid": 123,
        "owns_process_group": True,
    }
    monkeypatch.setattr(
        "scripts.codex_project.inspect_cmd_vel_publishers",
        lambda: {"ok": True, "endpoints": []},
    )
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **kwargs: {})
    monkeypatch.setattr(
        "scripts.codex_project.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0, "", ""),
    )
    monkeypatch.setattr(
        "scripts.codex_project.subprocess.Popen", lambda *args, **kwargs: type("P", (), {"pid": 123})()
    )
    monkeypatch.setattr("scripts.codex_project.record_process", lambda *args: record)
    monkeypatch.setattr(
        "scripts.codex_project._wait_for_graph",
        lambda timeout: (_ for _ in ()).throw(RuntimeError("readiness failed")),
    )
    monkeypatch.setattr("scripts.codex_project.terminate_managed_process", lambda *args, **kwargs: False)

    with pytest.raises(Exception, match="cleanup incomplete"):
        lifecycle.start()

    state = json.loads(lifecycle.state_path.read_text())
    assert state["processes"] == [record]


def test_status_cli_returns_structured_stopped_state_for_empty_runtime(tmp_path):
    """Catches lifecycle status depending on missing files or human prose."""
    result = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "codex_project.py"),
            "--runtime-dir",
            str(tmp_path),
            "status",
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 0
    assert json.loads(result.stdout) == {
        "ok": True,
        "running": False,
        "managed_processes": [],
        "stale_processes": [],
    }


def test_rgb_image_message_writes_real_png_with_correct_channel_order(tmp_path):
    """Catches blank files and RGB/BGR swaps in Codex camera evidence."""
    msg = Image()
    msg.height = 1
    msg.width = 2
    msg.encoding = "rgb8"
    msg.step = 6
    msg.data = bytes([255, 0, 0, 0, 255, 0])
    output = tmp_path / "camera.png"

    image_message_to_png(msg, output)
    decoded = cv2.imread(str(output), cv2.IMREAD_COLOR)

    assert output.stat().st_size > 20
    assert decoded.shape == (1, 2, 3)
    assert decoded[0, 0].tolist() == [0, 0, 255]
    assert decoded[0, 1].tolist() == [0, 255, 0]


def test_empty_camera_message_is_rejected_instead_of_fabricated(tmp_path):
    """Catches a zero-byte image being reported to Codex as visual evidence."""
    with pytest.raises(CameraCaptureError, match="dimensions"):
        image_message_to_png(Image(), tmp_path / "empty.png")
