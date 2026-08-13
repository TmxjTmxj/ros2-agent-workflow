#!/usr/bin/env python3
"""Bounded Agent-to-ROS 2 MCP control plane.

This module intentionally exposes capability-level operations only.  It has no
tool for shell commands, arbitrary ROS names or payloads, process identifiers,
or filesystem paths.
"""

from __future__ import annotations

import io
import logging
import math
import threading
from collections.abc import Mapping
from contextlib import asynccontextmanager
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Annotated, TypeAlias

from fastmcp import FastMCP
from fastmcp.tools import ToolResult
from fastmcp.utilities.types import Image
from jsonschema import Draft202012Validator
from mcp.types import ToolAnnotations
from PIL import Image as PillowImage
from pydantic import Field

from agent_ros.adapters.base import Observation
from agent_ros.adapters.factory import RclpyAdapterFactory
from agent_ros.runtime import (
    EvidenceError,
    EvidenceReference,
    EvidenceStore,
    RuntimeController,
    RuntimeControllerError,
)


class ProfileName(str, Enum):
    HOSPITAL_AMR = "hospital-amr"


class TaskName(str, Enum):
    HOSPITAL_DELIVERY = "hospital-delivery"


class ObservationSource(str, Enum):
    ODOMETRY = "odometry"
    CAMERA = "camera"
    SCAN = "scan"


Challenge: TypeAlias = Annotated[
    str,
    Field(min_length=1, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
]
ReportId: TypeAlias = Annotated[
    str,
    Field(min_length=1, max_length=64, pattern=r"^[a-z0-9][a-z0-9_-]{0,63}$"),
]

_ROOT = Path(__file__).resolve().parents[1]
_PROFILES_ROOT = _ROOT / "profiles"
_RUNTIME_ROOT = _ROOT / ".runtime"
_EVIDENCE_ROOT = _RUNTIME_ROOT / "evidence"
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_MAX_EVIDENCE_BYTES = 16 * 1024 * 1024
_MAX_IMAGE_PIXELS = 16_000_000
_DEFAULT_TIMEOUT = 15.0
_controller_condition = threading.Condition(threading.RLock())
_controller: RuntimeController | None = None
_evidence_store: EvidenceStore | None = None
_controller_closing = False
_controller_cleanup_failed = False
_logger = logging.getLogger(__name__)

_REMEDIATION = {
    "UNSAFE_STATE": "Inspect connection_status, validate the reviewed profile, and retry safely.",
    "PROFILE_INVALID": "Use a repository-owned profile or task name and resolve validation warnings.",
    "CONTROLLER_CONFLICT": "Remove conflicting command publishers before retrying discovery.",
    "STALE_FEEDBACK": "Restore fresh robot feedback before issuing another operation.",
    "TIMEOUT": "Inspect task_status and evidence, then cancel or stop the runtime if needed.",
    "EVIDENCE_INVALID": "Regenerate evidence in the managed evidence store.",
    "AUDIT_INTEGRITY_COMPROMISED": "Stop operation and have an operator inspect the audit store.",
    "ESTOP_LATCHED": "Keep the robot stopped and follow the operator-only reset procedure.",
    "OPERATOR_REQUIRED": "Have an operator create a fresh out-of-band hardware challenge.",
    "CLEANUP_FAILED": "Keep the robot stopped and inspect managed runtime cleanup.",
}
_STATE = {
    "type": "string",
    "enum": ["NEW", "DISCOVERED", "VALIDATED", "ARMED", "RUNNING", "STOPPED", "ESTOPPED", "FAULTED"],
}
_TEXT = {"type": "string", "minLength": 1}
_ERROR_SCHEMA = {
    "type": "object",
    "properties": {
        "ok": {"const": False},
        "error": {
            "type": "object",
            "properties": {
                "code": {"type": "string", "enum": sorted(_REMEDIATION)},
                "remediation": {"type": "string", "minLength": 1},
            },
            "required": ["code", "remediation"],
            "additionalProperties": False,
        },
    },
    "required": ["ok", "error"],
    "additionalProperties": False,
}


def _object(properties, required):
    return {
        "type": "object",
        "properties": properties,
        "required": list(required),
        "additionalProperties": False,
    }


def _response(data_schema):
    return {
        "type": "object",
        "properties": {
            "ok": {"type": "boolean"},
            "data": data_schema,
            "error": _ERROR_SCHEMA["properties"]["error"],
        },
        "oneOf": [
            _object({"ok": {"const": True}, "data": data_schema}, ("ok", "data")),
            _ERROR_SCHEMA,
        ],
        "additionalProperties": False,
    }


_RESPONSE_SCHEMAS = {
    "discover_robot": _response(_object({
        "profile": _TEXT,
        "state": _STATE,
        "capabilities": {"type": "array", "items": _TEXT},
        "hardware_safety_channel": {"type": "string", "enum": ["simulation_only", "unverified", "verified"]},
    }, ("profile", "state", "capabilities", "hardware_safety_channel"))),
    "validate_profile": _response(_object({
        "profile": _TEXT,
        "state": _STATE,
        "hardware_safety_channel": {"type": "string", "enum": ["simulation_only", "unverified", "verified"]},
    }, ("profile", "state", "hardware_safety_channel"))),
    "connection_status": _response(_object({"state": _STATE}, ("state",))),
    "list_capabilities": _response(_object({
        "capabilities": {"type": "array", "items": _TEXT},
    }, ("capabilities",))),
    "arm_robot": _response(_object({
        "profile": _TEXT,
        "state": _STATE,
        "dry_run": {"type": "boolean"},
    }, ("profile", "state"))),
    "run_task": _response(_object({
        "task": _TEXT,
        "state": _STATE,
        "profile": _TEXT,
        "dry_run": {"type": "boolean"},
    }, ("task",))),
    "task_status": _response(_object({
        "state": _STATE,
        "task": {"oneOf": [_TEXT, {"type": "null"}]},
        "hardware_safety_channel": {"type": "string", "enum": ["simulation_only", "unverified", "verified"]},
        "adapter_state": _TEXT,
        "code": {"oneOf": [_TEXT, {"type": "null"}]},
    }, ("state", "task", "hardware_safety_channel"))),
    "cancel_task": _response(_object({"state": _STATE, "adapter_state": _TEXT}, ("state", "adapter_state"))),
    "emergency_stop": _response(_object({"state": _STATE}, ("state",))),
    "observe": _response(_object({
        "source": {"type": "string", "enum": ["odometry", "camera", "scan"]},
        "timestamp": {"type": "number"},
        "values": _object({
            "x": {"type": "number"},
            "y": {"type": "number"},
            "yaw": {"type": "number"},
        }, ()),
    }, ("source", "timestamp", "values"))),
    "get_evidence": _response(_object({
        "report_id": _TEXT,
        "relative_path": _TEXT,
        "media_type": {"type": "string", "enum": ["application/json", "image/png"]},
        "size": {"type": "integer", "minimum": 0},
    }, ("report_id", "relative_path", "media_type", "size"))),
    "stop_runtime": _response(_object({"state": _STATE}, ("state",))),
}


def get_runtime_controller() -> RuntimeController:
    """Return the one process-owned runtime controller, creating it lazily."""
    global _controller, _evidence_store
    with _controller_condition:
        while _controller_closing:
            _controller_condition.wait()
        if _controller_cleanup_failed:
            raise RuntimeControllerError("CLEANUP_FAILED")
        if _controller is None:
            _evidence_store = EvidenceStore(_EVIDENCE_ROOT)
            adapter_factory = RclpyAdapterFactory()
            _controller = RuntimeController(
                profiles_root=_PROFILES_ROOT,
                evidence_dir=_EVIDENCE_ROOT,
                runtime_dir=_RUNTIME_ROOT,
                adapter_factory=adapter_factory,
                cleanup_timeout=_DEFAULT_TIMEOUT,
            )
        return _controller


def close_runtime_controller() -> bool:
    """Close the singleton before allowing another instance to be created."""
    global _controller, _evidence_store, _controller_closing, _controller_cleanup_failed
    with _controller_condition:
        while _controller_closing:
            _controller_condition.wait()
        controller = _controller
        if controller is None:
            return True
        _controller_closing = True
    successful = False
    try:
        try:
            controller.stop_runtime()
            successful = True
        except RuntimeControllerError:
            successful = False
    finally:
        with _controller_condition:
            if successful:
                _controller = None
                _evidence_store = None
                _controller_cleanup_failed = False
            else:
                _controller_cleanup_failed = True
            _controller_closing = False
            _controller_condition.notify_all()
    return successful


def _default_evidence_store() -> EvidenceStore:
    get_runtime_controller()
    assert _evidence_store is not None
    return _evidence_store


def _annotations(name: str) -> ToolAnnotations:
    read_only = name in {
        "connection_status",
        "list_capabilities",
        "task_status",
        "observe",
        "get_evidence",
    }
    destructive = name in {
        "arm_robot",
        "run_task",
        "cancel_task",
        "emergency_stop",
        "stop_runtime",
    }
    return ToolAnnotations(
        readOnlyHint=read_only,
        destructiveHint=destructive,
        idempotentHint=name in {
            "connection_status",
            "list_capabilities",
            "task_status",
            "observe",
            "get_evidence",
            "emergency_stop",
            "stop_runtime",
        },
        openWorldHint=False,
    )


def _meta(name: str, timeout: float) -> dict[str, object]:
    metadata: dict[str, object] = {
        "timeout_seconds": timeout,
        "authority": "repository_profiles_only",
    }
    if name == "arm_robot":
        metadata.update({
            "hardware_dry_run_default": True,
            "challenge_source": "operator_only",
        })
    if name == "emergency_stop":
        metadata["hardware_reset"] = "operator_only"
    return metadata


def _validate_json_value(value: object) -> object:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return _validate_json_value(asdict(value))
    if isinstance(value, Mapping):
        if not all(isinstance(key, str) for key in value):
            raise ValueError
        return {key: _validate_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_validate_json_value(item) for item in value]
    raise ValueError


def _success(data: object, schema: dict[str, object], *, content=None) -> ToolResult:
    validated = _validate_json_value(data)
    if not isinstance(validated, dict):
        raise ValueError
    structured = {"ok": True, "data": validated}
    Draft202012Validator(schema).validate(structured)
    return ToolResult(
        content=structured if content is None else content,
        structured_content=structured,
    )


def _error(code: str) -> ToolResult:
    stable_code = code if code in _REMEDIATION else "UNSAFE_STATE"
    structured = {
        "ok": False,
        "error": {
            "code": stable_code,
            "remediation": _REMEDIATION[stable_code],
        },
    }
    return ToolResult(content=structured, structured_content=structured, is_error=True)


def _invoke(operation, schema: dict[str, object]) -> ToolResult:
    try:
        return _success(operation(), schema)
    except RuntimeControllerError as exc:
        return _error(exc.code)
    except Exception:
        return _error("UNSAFE_STATE")


def _evidence_result(
    controller: RuntimeController,
    store: EvidenceStore,
    report_id: str | None,
    schema: dict[str, object],
) -> ToolResult:
    try:
        reference = controller.get_evidence(report_id)
        if not isinstance(reference, EvidenceReference):
            raise EvidenceError()
        metadata = {
            "report_id": reference.report_id,
            "relative_path": reference.relative_path,
            "media_type": reference.media_type,
            "size": reference.size,
        }
        if reference.media_type != "image/png":
            return _success(metadata, schema)
        if reference.size < len(_PNG_SIGNATURE) or reference.size > _MAX_EVIDENCE_BYTES:
            raise EvidenceError()
        data = store.read(reference, max_bytes=_MAX_EVIDENCE_BYTES)
        if len(data) != reference.size or not data.startswith(_PNG_SIGNATURE):
            raise EvidenceError()
        try:
            with PillowImage.open(io.BytesIO(data)) as decoded:
                if (
                    decoded.format != "PNG"
                    or decoded.width < 1
                    or decoded.height < 1
                    or decoded.width * decoded.height > _MAX_IMAGE_PIXELS
                ):
                    raise EvidenceError()
                decoded.load()
        except (OSError, ValueError, PillowImage.DecompressionBombError):
            raise EvidenceError() from None
        return _success(metadata, schema, content=[Image(data=data, format="png")])
    except (RuntimeControllerError, EvidenceError) as exc:
        return _error(getattr(exc, "code", "EVIDENCE_INVALID"))
    except Exception:
        return _error("EVIDENCE_INVALID")


def create_server(
    *,
    controller: RuntimeController | None = None,
    evidence_store: EvidenceStore | None = None,
    timeout: float = _DEFAULT_TIMEOUT,
) -> FastMCP:
    """Build the fixed MCP surface around an injected or process singleton runtime."""
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or not 0 < timeout <= 30:
        raise ValueError("timeout must be within (0, 30]")
    operation_timeout = float(timeout)
    controller_for_call = (lambda: controller) if controller is not None else get_runtime_controller
    evidence_for_call = (
        (lambda: evidence_store)
        if evidence_store is not None
        else _default_evidence_store
    )
    capabilities: tuple[str, ...] = ()
    capabilities_lock = threading.Lock()

    @asynccontextmanager
    async def lifespan(_server):
        yield {}
        if controller is None:
            if not close_runtime_controller():
                _logger.error("CLEANUP_FAILED")

    server = FastMCP(
        "agent-ros2",
        version="0.1.0",
        instructions="Use only reviewed profiles and bounded task-level operations.",
        strict_input_validation=True,
        mask_error_details=True,
        lifespan=lifespan,
    )

    def register(name: str, description: str):
        return server.tool(
            name=name,
            description=description,
            annotations=_annotations(name),
            meta=_meta(name, operation_timeout),
            timeout=operation_timeout,
            output_schema=_RESPONSE_SCHEMAS[name],
        )

    @register("discover_robot", "Discover a robot through a reviewed repository profile.")
    def discover_robot(profile_hint: ProfileName | None = None) -> ToolResult:
        nonlocal capabilities

        def operation():
            nonlocal capabilities
            hint = None if profile_hint is None else profile_hint.value
            result = controller_for_call().discover_robot(hint)
            raw = result.get("capabilities", ())
            if not isinstance(raw, (list, tuple)) or not all(isinstance(item, str) for item in raw):
                raise RuntimeControllerError("UNSAFE_STATE")
            with capabilities_lock:
                capabilities = tuple(raw)
            return result

        return _invoke(operation, _RESPONSE_SCHEMAS["discover_robot"])

    @register("validate_profile", "Validate the active reviewed robot profile.")
    def validate_profile(profile_name: ProfileName) -> ToolResult:
        return _invoke(lambda: controller_for_call().validate_profile(profile_name.value), _RESPONSE_SCHEMAS["validate_profile"])

    @register("connection_status", "Read the current bounded runtime connection state.")
    def connection_status() -> ToolResult:
        return _invoke(lambda: {"state": controller_for_call().state.value}, _RESPONSE_SCHEMAS["connection_status"])

    @register("list_capabilities", "List capabilities established by the latest discovery.")
    def list_capabilities() -> ToolResult:
        with capabilities_lock:
            current = list(capabilities)
        return _success({"capabilities": current}, _RESPONSE_SCHEMAS["list_capabilities"])

    @register("arm_robot", "Arm a reviewed profile; hardware remains dry-run by default.")
    def arm_robot(
        profile_name: ProfileName,
        challenge: Challenge,
        dry_run: bool = True,
    ) -> ToolResult:
        return _invoke(
            lambda: controller_for_call().arm_robot(
                profile_name.value,
                challenge,
                dry_run=dry_run,
            ),
            _RESPONSE_SCHEMAS["arm_robot"],
        )

    @register("run_task", "Run a reviewed task profile after safety authorization.")
    def run_task(task_name: TaskName, dry_run: bool = False) -> ToolResult:
        return _invoke(lambda: controller_for_call().run_task(task_name.value, dry_run=dry_run), _RESPONSE_SCHEMAS["run_task"])

    @register("task_status", "Read the active task and adapter status.")
    def task_status() -> ToolResult:
        return _invoke(controller_for_call().task_status, _RESPONSE_SCHEMAS["task_status"])

    @register("cancel_task", "Cancel the active bounded task and stop motion.")
    def cancel_task() -> ToolResult:
        return _invoke(controller_for_call().cancel_task, _RESPONSE_SCHEMAS["cancel_task"])

    @register("emergency_stop", "Latch the runtime emergency stop; no reset is exposed.")
    def emergency_stop() -> ToolResult:
        return _invoke(controller_for_call().emergency_stop, _RESPONSE_SCHEMAS["emergency_stop"])

    @register("observe", "Read one reviewed observation source.")
    def observe(source: ObservationSource) -> ToolResult:
        def operation():
            observation = controller_for_call().observe(source.value)
            if not isinstance(observation, Observation):
                raise RuntimeControllerError("UNSAFE_STATE")
            return {
                "source": observation.source,
                "timestamp": observation.timestamp,
                "values": dict(observation.values),
            }

        return _invoke(operation, _RESPONSE_SCHEMAS["observe"])

    @register("get_evidence", "Read a managed evidence report by opaque identifier.")
    def get_evidence(report_id: ReportId | None = None) -> ToolResult:
        return _evidence_result(controller_for_call(), evidence_for_call(), report_id, _RESPONSE_SCHEMAS["get_evidence"])

    @register("stop_runtime", "Safely stop and close the managed ROS runtime.")
    def stop_runtime() -> ToolResult:
        return _invoke(controller_for_call().stop_runtime, _RESPONSE_SCHEMAS["stop_runtime"])

    return server


mcp = create_server()


if __name__ == "__main__":
    mcp.run(transport="stdio", show_banner=False)
