#!/usr/bin/env python3
"""Run the reviewed hospital case through the production FastMCP stdio surface."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
from collections.abc import Mapping
from pathlib import Path
import tempfile
import time


EXAMPLE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = EXAMPLE_ROOT.parents[1]
TRACE_PATH = EXAMPLE_ROOT / "evidence" / "mcp_agent_trace.json"
SERVER_PYTHON = REPOSITORY_ROOT / ".venv" / "bin" / "python"
WALL_TIMEOUT = 600.0
POLL_INTERVAL = 0.5
_ROS_ENVIRONMENT_KEYS = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PATH",
    "PYTHONPATH",
    "RMW_IMPLEMENTATION",
    "ROS_DISTRO",
    "ROS_DOMAIN_ID",
    "ROS_PYTHON_VERSION",
    "ROS_VERSION",
)


class ToolSequenceError(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code if code in {
            "UNSAFE_STATE",
            "PROFILE_INVALID",
            "CONTROLLER_CONFLICT",
            "STALE_FEEDBACK",
            "TIMEOUT",
            "EVIDENCE_INVALID",
            "AUDIT_INTEGRITY_COMPROMISED",
            "ESTOP_LATCHED",
            "OPERATOR_REQUIRED",
            "CLEANUP_FAILED",
        } else "UNSAFE_STATE"
        super().__init__(self.code)


def server_environment(environ: Mapping[str, str]) -> dict[str, str]:
    """Forward only the fixed ROS overlay variables required by the stdio child."""
    result = {"PYTHONUNBUFFERED": "1", "PYTHONNOUSERSITE": "1"}
    for key in _ROS_ENVIRONMENT_KEYS:
        value = environ.get(key)
        if isinstance(value, str) and value:
            result[key] = value
    return result


def _tool_record(name: str, ok: bool, data: Mapping[str, object] | None = None):
    record: dict[str, object] = {"tool": name, "ok": ok}
    if data is not None:
        for key in ("state", "adapter_state", "source"):
            value = data.get(key)
            if isinstance(value, str) and value:
                record[key] = value
        for key in ("elapsed", "stage_index"):
            value = data.get(key)
            if (
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
            ):
                record[key] = value if isinstance(value, int) else float(value)
    return record


async def execute_tool_sequence(
    client,
    *,
    wall_timeout: float = WALL_TIMEOUT,
    poll_interval: float = POLL_INTERVAL,
    monotonic=time.monotonic,
) -> dict[str, object]:
    """Execute the exact reviewed tool sequence and always request explicit cleanup."""
    if (
        isinstance(wall_timeout, bool)
        or not isinstance(wall_timeout, (int, float))
        or not math.isfinite(float(wall_timeout))
        or not 0.0 < float(wall_timeout) <= WALL_TIMEOUT
        or isinstance(poll_interval, bool)
        or not isinstance(poll_interval, (int, float))
        or not math.isfinite(float(poll_interval))
        or float(poll_interval) < 0.0
    ):
        raise ValueError("invalid fixed MCP timing")
    started = monotonic()
    tools: list[dict[str, object]] = []
    observation: dict[str, object] | None = None
    result = "FAILED"
    error_code: str | None = None

    async def call(name: str, arguments: Mapping[str, object] | None = None):
        try:
            response = await client.call_tool(name, dict(arguments or {}), timeout=30)
        except Exception as exc:
            tools.append(_tool_record(name, False))
            code = "UNSAFE_STATE"
            try:
                raised = json.loads(str(exc))
                error = raised.get("error") if isinstance(raised, Mapping) else None
                candidate = error.get("code") if isinstance(error, Mapping) else None
                if isinstance(candidate, str):
                    code = candidate
            except (json.JSONDecodeError, TypeError, AttributeError):
                pass
            raise ToolSequenceError(code) from None
        payload = getattr(response, "structured_content", None)
        if not isinstance(payload, Mapping) or not isinstance(payload.get("ok"), bool):
            tools.append(_tool_record(name, False))
            raise ToolSequenceError("UNSAFE_STATE")
        if payload["ok"] is not True:
            tools.append(_tool_record(name, False))
            error = payload.get("error")
            code = error.get("code") if isinstance(error, Mapping) else None
            raise ToolSequenceError(code if isinstance(code, str) else "UNSAFE_STATE")
        data = payload.get("data")
        if not isinstance(data, Mapping):
            tools.append(_tool_record(name, False))
            raise ToolSequenceError("UNSAFE_STATE")
        tools.append(_tool_record(name, True, data))
        return data

    try:
        await call("discover_robot", {"profile_hint": "hospital-amr"})
        await call("validate_profile", {"profile_name": "hospital-amr"})
        await call(
            "arm_robot",
            {
                "profile_name": "hospital-amr",
                "challenge": "hospital_delivery_reviewed",
                "dry_run": False,
            },
        )
        await call(
            "run_task",
            {"task_name": "hospital-delivery", "dry_run": False},
        )
        while True:
            if monotonic() - started > float(wall_timeout):
                raise ToolSequenceError("TIMEOUT")
            status = await call("task_status")
            if status.get("adapter_state") == "succeeded" and status.get("state") == "STOPPED":
                break
            if status.get("state") in {"FAULTED", "ESTOPPED"}:
                raise ToolSequenceError("UNSAFE_STATE")
            await asyncio.sleep(float(poll_interval))
        observed = await call("observe", {"source": "odometry"})
        values = observed.get("values")
        if not isinstance(values, Mapping) or set(values) != {"x", "y", "yaw"}:
            raise ToolSequenceError("UNSAFE_STATE")
        projected: dict[str, float] = {}
        for key in ("x", "y", "yaw"):
            value = values.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise ToolSequenceError("UNSAFE_STATE")
            projected[key] = float(value)
        observation = {"source": "odometry", "values": projected}
        result = "SUCCEEDED"
    except ToolSequenceError as exc:
        error_code = exc.code
    except Exception:
        error_code = "UNSAFE_STATE"
    finally:
        try:
            await call("stop_runtime")
        except ToolSequenceError:
            result = "FAILED"
            error_code = "CLEANUP_FAILED"
        except Exception:
            tools.append(_tool_record("stop_runtime", False))
            result = "FAILED"
            error_code = "CLEANUP_FAILED"

    wall_elapsed = monotonic() - started
    if not math.isfinite(wall_elapsed) or wall_elapsed < 0.0:
        result = "FAILED"
        error_code = "UNSAFE_STATE"
        wall_elapsed = 0.0
    trace: dict[str, object] = {
        "schema_version": 1,
        "case": "hospital_delivery",
        "control_plane": "production_fastmcp_stdio",
        "authority": "repository_profiles_only",
        "tools": tools,
        "result": result,
        "wall_elapsed_seconds": wall_elapsed,
        "independent_acceptance_reference": {
            "relative_path": "evidence/acceptance_report.json",
            "role": "separate_independent_acceptance",
        },
    }
    if observation is not None:
        trace["observation"] = observation
    if error_code is not None:
        trace["error_code"] = error_code
    return trace


def write_trace_atomic(path: Path, trace: Mapping[str, object]) -> None:
    """Persist a privacy-safe standard-JSON trace without exposing a stale PASS."""
    destination = Path(path)
    destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    encoded = json.dumps(
        dict(trace),
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        allow_nan=False,
    ) + "\n"
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            os.chmod(temporary, 0o600)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        os.chmod(destination, 0o600)
        temporary = None
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


async def _run_stdio() -> dict[str, object]:
    from fastmcp import Client
    from fastmcp.client.transports import StdioTransport

    transport = StdioTransport(
        command=str(SERVER_PYTHON),
        args=["-m", "mcp_server.ros2_mcp_server"],
        cwd=str(REPOSITORY_ROOT),
        env=server_environment(os.environ),
        keep_alive=False,
        log_file=EXAMPLE_ROOT / "logs" / "mcp-stderr.log",
    )
    async with Client(transport) as client:
        return await execute_tool_sequence(client)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    try:
        trace = asyncio.run(_run_stdio())
    except Exception:
        trace = {
            "schema_version": 1,
            "case": "hospital_delivery",
            "control_plane": "production_fastmcp_stdio",
            "authority": "repository_profiles_only",
            "tools": [],
            "result": "FAILED",
            "error_code": "UNSAFE_STATE",
            "wall_elapsed_seconds": 0.0,
            "independent_acceptance_reference": {
                "relative_path": "evidence/acceptance_report.json",
                "role": "separate_independent_acceptance",
            },
        }
    write_trace_atomic(TRACE_PATH, trace)
    print(json.dumps({"ok": trace["result"] == "SUCCEEDED", "result": trace["result"]}))
    return 0 if trace["result"] == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
