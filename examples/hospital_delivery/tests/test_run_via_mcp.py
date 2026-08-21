from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import pytest

import scripts.run_via_mcp as run_via_mcp
from scripts.run_via_mcp import (
    execute_tool_sequence,
    server_environment,
    write_trace_atomic,
)


class ReplayMcpClient:
    def __init__(self, *, fail_tool: str | None = None) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []
        self.fail_tool = fail_tool
        self.statuses = [
            {"state": "RUNNING", "task": "hospital-delivery", "adapter_state": "running"},
            {"state": "STOPPED", "task": "hospital-delivery", "adapter_state": "succeeded"},
        ]

    async def call_tool(self, name, arguments=None, timeout=None):
        arguments = arguments or {}
        self.calls.append((name, arguments))
        if name == self.fail_tool:
            return SimpleNamespace(
                structured_content={
                    "ok": False,
                    "error": {"code": "UNSAFE_STATE", "remediation": "fixed"},
                }
            )
        data = {
            "discover_robot": {"state": "DISCOVERED", "profile": "hospital-amr"},
            "validate_profile": {"state": "ARMED", "profile": "hospital-amr"},
            "arm_robot": {"state": "ARMED", "profile": "hospital-amr"},
            "run_task": {"state": "RUNNING", "task": "hospital-delivery"},
            "observe": {
                "source": "odometry",
                "timestamp": 42.0,
                "values": {"x": 0.1, "y": -0.2, "yaw": 0.3},
            },
            "stop_runtime": {"state": "STOPPED"},
        }.get(name)
        if name == "task_status":
            data = self.statuses.pop(0)
        assert data is not None, name
        return SimpleNamespace(structured_content={"ok": True, "data": data})


@pytest.mark.parametrize(
    ("arguments", "expected_code"),
    [(["--help"], 0), (["--not-a-real-option"], 2)],
)
def test_mcp_runner_cli_arguments_never_start_the_control_plane(
    monkeypatch, capsys, arguments, expected_code
):
    async def forbidden_run():
        raise AssertionError("argument parsing must finish before MCP startup")

    monkeypatch.setattr(run_via_mcp, "_run_stdio", forbidden_run)

    with pytest.raises(SystemExit) as raised:
        run_via_mcp.main(arguments)

    assert raised.value.code == expected_code
    captured = capsys.readouterr()
    assert "usage:" in (captured.out + captured.err)


def test_mcp_agent_sequence_reaches_success_observes_and_explicitly_stops():
    client = ReplayMcpClient()

    trace = asyncio.run(
        execute_tool_sequence(client, wall_timeout=300.0, poll_interval=0.0)
    )

    assert [name for name, _arguments in client.calls] == [
        "discover_robot",
        "validate_profile",
        "arm_robot",
        "run_task",
        "task_status",
        "task_status",
        "observe",
        "stop_runtime",
    ]
    assert client.calls[0][1] == {"profile_hint": "hospital-amr"}
    assert client.calls[2][1] == {
        "profile_name": "hospital-amr",
        "challenge": "hospital_delivery_reviewed",
        "dry_run": False,
    }
    assert client.calls[3][1] == {
        "task_name": "hospital-delivery",
        "dry_run": False,
    }
    assert trace["result"] == "SUCCEEDED"
    assert trace["observation"] == {
        "source": "odometry",
        "values": {"x": 0.1, "y": -0.2, "yaw": 0.3},
    }
    assert trace["independent_acceptance_reference"] == {
        "relative_path": "evidence/acceptance_report.json",
        "role": "separate_independent_acceptance",
    }


def test_mcp_agent_sequence_explicitly_stops_after_tool_failure():
    client = ReplayMcpClient(fail_tool="run_task")

    trace = asyncio.run(
        execute_tool_sequence(client, wall_timeout=300.0, poll_interval=0.0)
    )

    assert [name for name, _arguments in client.calls][-1] == "stop_runtime"
    assert "observe" not in [name for name, _arguments in client.calls]
    assert trace["result"] == "FAILED"
    assert trace["error_code"] == "UNSAFE_STATE"
    assert trace["tools"][-1] == {
        "tool": "stop_runtime",
        "ok": True,
        "state": "STOPPED",
    }


def test_mcp_agent_trace_records_fastmcp_raised_tool_error_without_raw_details():
    class RaisedToolErrorClient(ReplayMcpClient):
        async def call_tool(self, name, arguments=None, timeout=None):
            if name == "discover_robot":
                self.calls.append((name, arguments or {}))
                raise RuntimeError(
                    '{"ok":false,"error":{"code":"CONTROLLER_CONFLICT",'
                    '"remediation":"private detail must not persist"}}'
                )
            return await super().call_tool(name, arguments, timeout)

    trace = asyncio.run(
        execute_tool_sequence(
            RaisedToolErrorClient(), wall_timeout=300.0, poll_interval=0.0
        )
    )

    assert trace["result"] == "FAILED"
    assert trace["error_code"] == "CONTROLLER_CONFLICT"
    assert trace["tools"][0] == {
        "tool": "discover_robot",
        "ok": False,
    }
    assert "private detail" not in json.dumps(trace)


def test_mcp_agent_trace_is_standard_json_and_atomically_replaces_stale_file(tmp_path):
    path = tmp_path / "logs" / "mcp_agent_trace.json"
    path.parent.mkdir()
    path.write_text('{"result":"STALE_PASS"}\n', encoding="utf-8")
    trace = asyncio.run(
        execute_tool_sequence(ReplayMcpClient(), wall_timeout=300.0, poll_interval=0.0)
    )

    write_trace_atomic(path, trace)

    assert json.loads(path.read_text(encoding="utf-8")) == trace
    assert "NaN" not in path.read_text(encoding="utf-8")
    assert list(path.parent.glob(".mcp_agent_trace.json.*")) == []
    assert path.stat().st_mode & 0o077 == 0


def test_mcp_stdio_child_inherits_only_fixed_ros_overlay_environment():
    environment = server_environment({
        "PATH": "/opt/ros/lyrical/bin:/usr/bin",
        "PYTHONPATH": "/opt/ros/lyrical/lib/python3.14/site-packages",
        "AMENT_PREFIX_PATH": "/opt/ros/lyrical",
        "ROS_DISTRO": "lyrical",
        "ROS_DOMAIN_ID": "7",
        "PYTHONNOUSERSITE": "0",
        "PRIVATE_TOKEN": "must-not-cross-stdio-boundary",
    })

    assert environment == {
        "PYTHONUNBUFFERED": "1",
        "PYTHONNOUSERSITE": "1",
        "PATH": "/opt/ros/lyrical/bin:/usr/bin",
        "PYTHONPATH": "/opt/ros/lyrical/lib/python3.14/site-packages",
        "AMENT_PREFIX_PATH": "/opt/ros/lyrical",
        "ROS_DISTRO": "lyrical",
        "ROS_DOMAIN_ID": "7",
    }
