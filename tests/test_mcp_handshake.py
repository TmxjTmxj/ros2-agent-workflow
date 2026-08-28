from __future__ import annotations

import asyncio
import os
import sys
import time
from pathlib import Path

from fastmcp import Client
from fastmcp.client.transports import StdioTransport

ROOT = Path(__file__).resolve().parents[1]
PYTHON = Path(sys.executable)
TOOL_NAMES = {
    "discover_robot",
    "validate_profile",
    "connection_status",
    "list_capabilities",
    "arm_robot",
    "run_task",
    "task_status",
    "cancel_task",
    "emergency_stop",
    "observe",
    "get_evidence",
    "stop_runtime",
}


def _direct_children() -> set[int]:
    children = Path(f"/proc/{os.getpid()}/task/{os.getpid()}/children")
    if not children.exists():
        return set()
    return {int(value) for value in children.read_text().split()}


def _wait_reaped(pid: int, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not Path(f"/proc/{pid}").exists():
            return True
        time.sleep(0.01)
    return not Path(f"/proc/{pid}").exists()


def test_real_stdio_handshake_lists_tools_calls_status_and_reaps_server(tmp_path):
    assert PYTHON == Path(sys.executable)
    before = _direct_children()
    observed_child: set[int] = set()

    async def exercise() -> None:
        transport = StdioTransport(
            command=str(PYTHON),
            args=["-m", "mcp_server.ros2_mcp_server"],
            cwd=str(ROOT),
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
            keep_alive=False,
            log_file=tmp_path / "mcp-stderr.log",
        )
        async with Client(transport) as client:
            observed_child.update(_direct_children() - before)
            tools = await client.list_tools()
            assert {tool.name for tool in tools} == TOOL_NAMES
            status = await client.call_tool("connection_status", timeout=5)
            assert status.structured_content == {
                "ok": True,
                "data": {"state": "NEW"},
            }

    asyncio.run(exercise())

    assert len(observed_child) == 1
    child = observed_child.pop()
    assert _wait_reaped(child), f"stdio MCP child {child} was not reaped"
    assert child not in _direct_children()
