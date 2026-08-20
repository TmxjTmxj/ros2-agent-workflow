import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import cv2
import pytest
from sensor_msgs.msg import Image

from scripts.capture_camera import CameraCaptureError, image_message_to_png
from scripts.codex_project import (
    MANAGED_LAUNCH_PREFIX,
    _native_graph_snapshot,
    _require_exclusive_controller,
    _wait_for_graph,
    inspect_cmd_vel_publishers,
    load_runtime_state,
    parse_cmd_vel_publisher_endpoints,
    parse_cmd_vel_publishers,
    process_matches,
    record_process,
    terminate_spawned_launch,
    terminate_managed_process,
    ProjectLifecycle,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _ready_native_graph():
    return {
        "nodes": ["/hospital_mission_controller"],
        "topics": {
            "/cmd_vel": ["geometry_msgs/msg/Twist"],
            "/odom": ["nav_msgs/msg/Odometry"],
            "/scan": ["sensor_msgs/msg/LaserScan"],
            "/camera/image_raw": ["sensor_msgs/msg/Image"],
            "/hospital_amr/contacts": ["ros_gz_interfaces/msg/Contacts"],
            "/hospital_mission/status": ["std_msgs/msg/String"],
        },
        "services": {
            f"/hospital_mission/{name}": ["std_srvs/srv/Trigger"]
            for name in ("start", "cancel", "estop", "reset")
        },
        "actions": {},
        "topic_endpoints": {
            "/cmd_vel": [
                {
                    "node_name": "hospital_mission_controller",
                    "node_namespace": "/",
                    "gid": "01ab",
                    "endpoint_type": "publisher",
                }
            ]
        },
    }


def test_startup_readiness_uses_one_native_graph_snapshot_not_ros2_cli(monkeypatch):
    snapshots = []

    def native_snapshot(timeout, *, environment=None):
        snapshots.append(timeout)
        assert isinstance(environment, dict)
        return _ready_native_graph()

    monkeypatch.setattr("scripts.codex_project._native_graph_snapshot", native_snapshot)
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **_kwargs: {})
    monkeypatch.setattr(
        "scripts.codex_project._run_ros",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("startup readiness must not use ros2 CLI polling")
        ),
    )

    _wait_for_graph(6.0)

    assert len(snapshots) == 1
    assert 0.0 < snapshots[0] <= 6.0


@pytest.mark.parametrize(
    "mutate",
    [
        lambda graph: graph["services"].pop("/hospital_mission/start"),
        lambda graph: graph["topic_endpoints"]["/cmd_vel"].append(
            {
                "node_name": "rogue_controller",
                "node_namespace": "/",
                "gid": "02cd",
                "endpoint_type": "publisher",
            }
        ),
    ],
)
def test_startup_readiness_fails_closed_on_missing_graph_or_controller_conflict(
    monkeypatch, mutate
):
    graph = _ready_native_graph()
    mutate(graph)
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **_kwargs: {})
    monkeypatch.setattr(
        "scripts.codex_project._native_graph_snapshot",
        lambda _timeout, *, environment=None: graph,
    )

    with pytest.raises(Exception):
        _wait_for_graph(0.01)


def test_startup_readiness_retries_while_expected_controller_endpoint_is_not_yet_visible(
    monkeypatch,
):
    first = _ready_native_graph()
    first["topic_endpoints"]["/cmd_vel"] = []
    snapshots = iter((first, _ready_native_graph()))
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **_kwargs: {})
    monkeypatch.setattr(
        "scripts.codex_project._native_graph_snapshot",
        lambda _timeout, *, environment=None: next(snapshots),
    )
    monkeypatch.setattr("scripts.codex_project.time.sleep", lambda _seconds: None)

    _wait_for_graph(1.0)


def test_native_graph_helper_is_fixed_bounded_and_rejects_malformed_output(monkeypatch):
    calls = []

    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0, '{"not":"a closed graph"}', "")

    monkeypatch.setattr("scripts.codex_project.subprocess.run", run)
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **_kwargs: {})

    with pytest.raises(Exception, match="native ROS graph"):
        _native_graph_snapshot(4.0)

    argv, kwargs = calls[0]
    assert argv == [
        str(PROJECT_ROOT.parents[1] / ".venv" / "bin" / "python"),
        "-m",
        "agent_ros.discovery.native_probe",
    ]
    assert kwargs["shell"] is False
    assert kwargs["timeout"] == 4.0


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
    monkeypatch.setattr(
        "scripts.codex_project._process_group_members",
        lambda pgid: [456] if pgid == 123 else [],
    )

    with pytest.raises(Exception, match="cleanup incomplete"):
        lifecycle.start()

    state = json.loads(lifecycle.state_path.read_text())
    assert state["processes"] == [record]


def test_stop_linearizes_with_start_spawn_to_state_handoff(tmp_path, monkeypatch):
    lifecycle = ProjectLifecycle(tmp_path)
    spawned = threading.Event()
    release_record = threading.Event()
    terminated = []
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
        "scripts.codex_project.subprocess.Popen",
        lambda *args, **kwargs: type("P", (), {"pid": 123})(),
    )

    def gated_record(*_args):
        spawned.set()
        assert release_record.wait(1.0)
        return record

    monkeypatch.setattr("scripts.codex_project.record_process", gated_record)
    monkeypatch.setattr("scripts.codex_project._wait_for_graph", lambda timeout: None)
    monkeypatch.setattr(
        "scripts.codex_project.process_matches", lambda candidate: candidate == record
    )
    monkeypatch.setattr(
        "scripts.codex_project.terminate_managed_process",
        lambda candidate, **kwargs: terminated.append(candidate) or True,
    )
    results = []
    start_thread = threading.Thread(target=lambda: results.append(lifecycle.start()))
    stop_thread = threading.Thread(target=lambda: results.append(lifecycle.stop()))
    start_thread.start()
    assert spawned.wait(0.2)
    stop_thread.start()

    # STOP must not pass the spawn-to-state handoff and observe an empty state.
    stop_thread.join(0.05)
    stop_was_blocked = stop_thread.is_alive()
    release_record.set()
    start_thread.join(1.0)
    stop_thread.join(1.0)

    assert stop_was_blocked
    assert not start_thread.is_alive()
    assert not stop_thread.is_alive()
    assert terminated == [record]
    assert json.loads(lifecycle.state_path.read_text())["processes"] == []


def test_stop_holds_authority_until_termination_commit_before_new_start(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    old_record = {
        "name": "hospital-delivery-launch",
        "pid": 111,
        "start_time_ticks": 1,
        "cmdline": [],
        "cwd": str(PROJECT_ROOT),
        "pgid": 111,
        "owns_process_group": True,
    }
    new_record = {**old_record, "pid": 222, "pgid": 222, "start_time_ticks": 2}
    lifecycle.runtime_dir.mkdir(parents=True, exist_ok=True)
    lifecycle.state_path.write_text(
        json.dumps({"version": 1, "processes": [old_record]})
    )
    termination_started = threading.Event()
    release_termination = threading.Event()
    old_terminated = threading.Event()
    started = []
    errors = []

    monkeypatch.setattr(
        "scripts.codex_project.process_matches",
        lambda record: record == new_record
        or (record == old_record and not old_terminated.is_set()),
    )
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda _pgid: [])
    monkeypatch.setattr("scripts.codex_project._service_call", lambda *a, **kw: {})

    def terminate(record, **_kwargs):
        assert record == old_record
        old_terminated.set()
        termination_started.set()
        assert release_termination.wait(1.0)
        return True

    monkeypatch.setattr("scripts.codex_project.terminate_managed_process", terminate)
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
        "scripts.codex_project.subprocess.Popen",
        lambda *args, **kwargs: type("P", (), {"pid": 222})(),
    )
    monkeypatch.setattr("scripts.codex_project.record_process", lambda *args: new_record)
    monkeypatch.setattr("scripts.codex_project._wait_for_graph", lambda _timeout: None)

    def run_start():
        try:
            started.append(lifecycle.start())
        except BaseException as exc:
            errors.append(exc)

    stop_thread = threading.Thread(target=lifecycle.stop)
    start_thread = threading.Thread(target=run_start)
    stop_thread.start()
    assert termination_started.wait(0.2)
    start_thread.start()
    start_thread.join(0.1)
    start_was_blocked = start_thread.is_alive()
    release_termination.set()
    stop_thread.join(1.0)
    start_thread.join(1.0)

    assert start_was_blocked
    assert not stop_thread.is_alive()
    assert not start_thread.is_alive()
    assert errors == []
    assert len(started) == 1
    assert json.loads(lifecycle.state_path.read_text())["processes"] == [new_record]


def test_cancelled_old_start_cleanup_never_overwrites_new_start_authority(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    old_record = {
        "name": "hospital-delivery-launch",
        "pid": 311,
        "start_time_ticks": 1,
        "cmdline": [*MANAGED_LAUNCH_PREFIX, "headless:=true"],
        "cwd": str(PROJECT_ROOT),
        "pgid": 311,
        "owns_process_group": True,
    }
    new_record = {**old_record, "pid": 322, "pgid": 322, "start_time_ticks": 2}
    old_waiting = threading.Event()
    release_old = threading.Event()
    old_dead = threading.Event()
    popen_pids = iter((311, 322))
    errors = []
    fail_new_unlink = [False]
    real_unlink = Path.unlink

    def flaky_unlink(path, *args, **kwargs):
        if path == lifecycle.startup_path and fail_new_unlink[0]:
            raise PermissionError("new startup reservation unlink failed")
        return real_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", flaky_unlink)
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
        "scripts.codex_project.subprocess.Popen",
        lambda *args, **kwargs: type("P", (), {"pid": next(popen_pids)})(),
    )
    monkeypatch.setattr(
        "scripts.codex_project.record_process",
        lambda pid, _name: old_record if pid == 311 else new_record,
    )
    monkeypatch.setattr(
        "scripts.codex_project.process_matches",
        lambda record: record == new_record
        or (record == old_record and not old_dead.is_set()),
    )
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda _pgid: [])
    monkeypatch.setattr("scripts.codex_project._service_call", lambda *a, **kw: {})

    def terminate(record, **_kwargs):
        if record == old_record and not old_dead.is_set():
            old_dead.set()
            return True
        return False

    monkeypatch.setattr("scripts.codex_project.terminate_managed_process", terminate)
    wait_calls = 0

    def wait_for_graph(_timeout):
        nonlocal wait_calls
        wait_calls += 1
        if wait_calls == 1:
            old_waiting.set()
            assert release_old.wait(1.0)
            raise RuntimeError("old launch was stopped")

    monkeypatch.setattr("scripts.codex_project._wait_for_graph", wait_for_graph)

    def start_old():
        try:
            lifecycle.start()
        except BaseException as exc:
            errors.append(exc)

    old_thread = threading.Thread(target=start_old)
    old_thread.start()
    assert old_waiting.wait(0.2)
    assert lifecycle.stop()["ok"] is True

    fail_new_unlink[0] = True
    with pytest.raises(PermissionError, match="new startup reservation unlink failed"):
        lifecycle.start()
    new_token = json.loads(lifecycle.startup_path.read_text())["token"]
    fail_new_unlink[0] = False
    release_old.set()
    old_thread.join(1.0)

    assert not old_thread.is_alive()
    assert len(errors) == 1
    state = json.loads(lifecycle.state_path.read_text())
    assert state["processes"] == [new_record]
    assert state["startup_token"] == new_token


def test_stop_recovers_committed_handoff_after_start_reservation_unlink_failure(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    record = {
        "name": "hospital-delivery-launch",
        "pid": 411,
        "start_time_ticks": 3,
        "cmdline": [*MANAGED_LAUNCH_PREFIX, "headless:=true"],
        "cwd": str(PROJECT_ROOT),
        "pgid": 411,
        "owns_process_group": True,
    }
    unlink_fails = [True]
    real_unlink = Path.unlink

    def flaky_unlink(path, *args, **kwargs):
        if path == lifecycle.startup_path and unlink_fails[0]:
            raise PermissionError("startup reservation unlink failed")
        return real_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", flaky_unlink)
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
        "scripts.codex_project.subprocess.Popen",
        lambda *args, **kwargs: type("P", (), {"pid": 411})(),
    )
    monkeypatch.setattr("scripts.codex_project.record_process", lambda *args: record)
    monkeypatch.setattr("scripts.codex_project.process_matches", lambda item: item == record)
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda _pgid: [])
    monkeypatch.setattr("scripts.codex_project._service_call", lambda *a, **kw: {})
    monkeypatch.setattr(
        "scripts.codex_project.terminate_managed_process",
        lambda item, **kwargs: item == record,
    )

    with pytest.raises(PermissionError, match="unlink failed"):
        lifecycle.start()

    assert lifecycle.startup_path.exists()
    assert json.loads(lifecycle.startup_path.read_text())["phase"] == "handoff"
    assert json.loads(lifecycle.state_path.read_text())["processes"] == [record]
    unlink_fails[0] = False

    stopped = lifecycle.stop()

    assert stopped["ok"] is True
    assert stopped["running"] is False
    assert stopped["unresolved_process_groups"] == []
    assert not lifecycle.startup_path.exists()


def test_phase_only_handoff_never_trusts_unauthorized_state_record(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    token = "handoff-token"
    unauthorized = {
        "name": "not-the-fixed-launch",
        "pid": 511,
        "start_time_ticks": 4,
        "cmdline": ["untrusted"],
        "cwd": str(PROJECT_ROOT),
        "pgid": 511,
        "owns_process_group": True,
    }
    lifecycle.runtime_dir.mkdir(parents=True, exist_ok=True)
    lifecycle._write_startup(
        {"token": token, "cancelled": False, "phase": "handoff"}
    )
    __import__("scripts.codex_project", fromlist=["_write_state"])._write_state(
        lifecycle.state_path, [unauthorized], startup_token=token
    )
    monkeypatch.setattr("scripts.codex_project.process_matches", lambda _record: False)
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda _pgid: [])

    stopped = lifecycle.stop()

    assert stopped["ok"] is False
    assert stopped["running"] is True
    assert lifecycle.startup_path.exists()


def test_phase_only_handoff_preserves_token_until_later_verified_cleanup(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    token = "handoff-token"
    record = {
        "name": "hospital-delivery-launch",
        "pid": 611,
        "start_time_ticks": 5,
        "cmdline": [*MANAGED_LAUNCH_PREFIX, "headless:=true"],
        "cwd": str(PROJECT_ROOT),
        "pgid": 611,
        "owns_process_group": True,
    }
    members = [[612]]
    lifecycle.runtime_dir.mkdir(parents=True, exist_ok=True)
    lifecycle._write_startup(
        {"token": token, "cancelled": False, "phase": "handoff"}
    )
    __import__("scripts.codex_project", fromlist=["_write_state"])._write_state(
        lifecycle.state_path, [record], startup_token=token
    )
    monkeypatch.setattr("scripts.codex_project.process_matches", lambda _record: False)
    monkeypatch.setattr(
        "scripts.codex_project._process_group_members",
        lambda _pgid: list(members[0]),
    )

    first = lifecycle.stop()

    assert first["ok"] is False
    assert first["running"] is True
    assert json.loads(lifecycle.state_path.read_text())["startup_token"] == token
    assert lifecycle.startup_path.exists()

    members[0] = []
    second = lifecycle.stop()

    assert second["ok"] is True
    assert second["running"] is False
    assert not lifecycle.startup_path.exists()
    assert "startup_token" not in json.loads(lifecycle.state_path.read_text())


def test_stop_cancels_build_period_start_before_any_launch_spawn(tmp_path, monkeypatch):
    lifecycle = ProjectLifecycle(tmp_path)
    build_entered = threading.Event()
    release_build = threading.Event()
    spawned = []
    errors = []
    monkeypatch.setattr(
        "scripts.codex_project.inspect_cmd_vel_publishers",
        lambda: {"ok": True, "endpoints": []},
    )
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **kwargs: {})

    def blocked_build(*args, **kwargs):
        build_entered.set()
        assert release_build.wait(1.0)
        return subprocess.CompletedProcess(args[0], 0, "", "")

    def forbidden_spawn(*args, **kwargs):
        spawned.append(args)
        raise AssertionError("cancelled startup must not spawn launch")

    monkeypatch.setattr("scripts.codex_project.subprocess.run", blocked_build)
    monkeypatch.setattr("scripts.codex_project.subprocess.Popen", forbidden_spawn)

    def start():
        try:
            lifecycle.start()
        except BaseException as exc:
            errors.append(exc)

    worker = threading.Thread(target=start)
    worker.start()
    assert build_entered.wait(0.2)

    stopped = lifecycle.stop()
    release_build.set()
    worker.join(1.0)

    assert not worker.is_alive()
    assert stopped["ok"] is True
    assert spawned == []
    assert len(errors) == 1
    assert "cancelled" in str(errors[0])
    assert not lifecycle.startup_path.exists()
    assert load_runtime_state(lifecycle.state_path)["processes"] == []


@pytest.mark.parametrize("failure_point", ["record_identity", "state_publish"])
def test_start_spawn_handoff_failure_terminates_exact_new_launch_group(
    tmp_path, monkeypatch, failure_point
):
    lifecycle = ProjectLifecycle(tmp_path)
    events = []
    record = {
        "name": "hospital-delivery-launch",
        "pid": 321,
        "start_time_ticks": 7,
        "cmdline": [],
        "cwd": str(PROJECT_ROOT),
        "pgid": 321,
        "owns_process_group": True,
    }

    class FixedProcess:
        pid = 321
        returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            events.append(("wait", timeout))
            self.returncode = -15
            return self.returncode

    process = FixedProcess()
    monkeypatch.setattr(
        "scripts.codex_project.inspect_cmd_vel_publishers",
        lambda: {"ok": True, "endpoints": []},
    )
    monkeypatch.setattr("scripts.codex_project._ros_environment", lambda **kwargs: {})
    monkeypatch.setattr(
        "scripts.codex_project.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0, "", ""),
    )
    monkeypatch.setattr("scripts.codex_project.subprocess.Popen", lambda *a, **kw: process)
    monkeypatch.setattr("scripts.codex_project.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "scripts.codex_project.os.killpg",
        lambda pid, sig: events.append(("killpg", pid, sig)),
    )
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda pgid: [])
    if failure_point == "record_identity":
        monkeypatch.setattr(
            "scripts.codex_project.record_process",
            lambda *args: (_ for _ in ()).throw(OSError("identity unavailable")),
        )
    else:
        monkeypatch.setattr("scripts.codex_project.record_process", lambda *args: record)
        monkeypatch.setattr(
            "scripts.codex_project._write_state",
            lambda *args, **kwargs: (_ for _ in ()).throw(
                OSError("state publish failed")
            ),
        )

    with pytest.raises(OSError):
        lifecycle.start()

    assert events[0] == ("killpg", 321, __import__("signal").SIGTERM)
    assert events[1][0] == "wait"
    assert process.returncode is not None
    assert not lifecycle.startup_path.exists()


def test_start_handoff_cleanup_failure_retains_unresolved_reservation(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)

    class FixedProcess:
        pid = 654
        returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            raise subprocess.TimeoutExpired("launch", timeout)

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
        "scripts.codex_project.subprocess.Popen", lambda *a, **kw: FixedProcess()
    )
    monkeypatch.setattr(
        "scripts.codex_project.record_process",
        lambda *args: (_ for _ in ()).throw(OSError("identity unavailable")),
    )
    monkeypatch.setattr("scripts.codex_project.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "scripts.codex_project.os.killpg",
        lambda pid, sig: (_ for _ in ()).throw(PermissionError("not permitted")),
    )

    with pytest.raises(Exception, match="cleanup incomplete"):
        lifecycle.start()

    startup = json.loads(lifecycle.startup_path.read_text())
    assert startup["unresolved_launch"]["pid"] == 654
    status = lifecycle.status()
    assert status["running"] is True
    assert status["unresolved_process_groups"] == [{"pgid": 654, "members": []}]
    stopped = lifecycle.stop()
    assert stopped["ok"] is True
    assert stopped["running"] is False
    assert stopped["unresolved_process_groups"] == []
    assert not lifecycle.startup_path.exists()
    assert not lifecycle.startup_failure_path.exists()
    assert lifecycle.status()["running"] is False


def test_spawn_cleanup_terminates_owned_group_when_leader_already_exited(monkeypatch):
    signals = []
    members = [[778]]

    class ExitedLeader:
        pid = 777

        def poll(self):
            return 0

    monkeypatch.setattr(
        "scripts.codex_project._process_group_members", lambda pgid: list(members[0])
    )

    def killpg(pgid, sig):
        signals.append(sig)
        members[0] = []

    monkeypatch.setattr("scripts.codex_project.os.killpg", killpg)

    assert terminate_spawned_launch(ExitedLeader(), term_timeout=0.0)
    assert signals == [__import__("signal").SIGTERM]


def test_spawn_cleanup_kills_descendants_left_after_term_reaps_leader(monkeypatch):
    signals = []
    members = [[889]]

    class Leader:
        pid = 888
        returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.returncode = -15
            return self.returncode

    leader = Leader()
    monkeypatch.setattr("scripts.codex_project.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "scripts.codex_project._process_group_members", lambda pgid: list(members[0])
    )

    def killpg(pgid, sig):
        signals.append(sig)
        if sig == __import__("signal").SIGKILL:
            members[0] = []

    monkeypatch.setattr("scripts.codex_project.os.killpg", killpg)

    assert terminate_spawned_launch(leader, term_timeout=0.0)
    assert signals == [__import__("signal").SIGTERM, __import__("signal").SIGKILL]


def test_unresolved_reservation_publish_failure_preserves_fail_closed_authority(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)

    class FixedProcess:
        pid = 765

        def poll(self):
            return None

        def wait(self, timeout=None):
            raise subprocess.TimeoutExpired("launch", timeout)

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
        "scripts.codex_project.subprocess.Popen", lambda *a, **kw: FixedProcess()
    )
    monkeypatch.setattr(
        "scripts.codex_project.record_process",
        lambda *args: (_ for _ in ()).throw(OSError("identity unavailable")),
    )
    monkeypatch.setattr("scripts.codex_project.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "scripts.codex_project.os.killpg",
        lambda pid, sig: (_ for _ in ()).throw(PermissionError("not permitted")),
    )
    real_write_startup = lifecycle._write_startup

    def fail_unresolved(payload):
        if "unresolved_launch" in payload:
            raise OSError("reservation publish failed")
        real_write_startup(payload)

    monkeypatch.setattr(lifecycle, "_write_startup", fail_unresolved)
    monkeypatch.setattr(
        lifecycle,
        "_write_startup_failure",
        lambda _pid: (_ for _ in ()).throw(OSError("poison marker failed")),
    )

    with pytest.raises(Exception, match="cleanup incomplete"):
        lifecycle.start()

    assert lifecycle.startup_path.exists()
    assert lifecycle.status()["running"] is True
    stopped = lifecycle.stop()
    assert stopped["ok"] is False
    assert stopped["running"] is True
    assert lifecycle.startup_path.exists()
    with pytest.raises(Exception, match="reserved"):
        lifecycle.start()


def test_stop_retains_stale_leader_record_when_recorded_pgid_members_remain(
    tmp_path, monkeypatch
):
    lifecycle = ProjectLifecycle(tmp_path)
    record = {
        "name": "hospital-delivery-launch",
        "pid": 123,
        "start_time_ticks": 1,
        "cmdline": [*__import__("scripts.codex_project", fromlist=["MANAGED_LAUNCH_PREFIX"]).MANAGED_LAUNCH_PREFIX, "headless:=true"],
        "cwd": str(PROJECT_ROOT),
        "pgid": 123,
        "owns_process_group": True,
    }
    lifecycle.runtime_dir.mkdir(parents=True, exist_ok=True)
    lifecycle.state_path.write_text(json.dumps({"version": 1, "processes": [record]}))
    monkeypatch.setattr("scripts.codex_project.process_matches", lambda _record: False)
    monkeypatch.setattr("scripts.codex_project._process_group_members", lambda pgid: [456] if pgid == 123 else [])
    signalled = []
    monkeypatch.setattr(
        "scripts.codex_project.terminate_managed_process",
        lambda *args, **kwargs: signalled.append(args) or True,
    )

    result = lifecycle.stop()

    assert result["ok"] is False
    assert result["running"] is True
    assert result["unresolved_process_groups"] == [{"pgid": 123, "members": [456]}]
    assert signalled == []
    assert json.loads(lifecycle.state_path.read_text())["processes"] == [record]
    with pytest.raises(Exception, match="unresolved process group"):
        lifecycle.start()


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
        "unresolved_process_groups": [],
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
