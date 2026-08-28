from __future__ import annotations

import asyncio
import base64
import threading
import time
import tomllib
from pathlib import Path

import fastmcp
import pytest
from agent_ros.adapters.base import Observation
from agent_ros.runtime import EvidenceReference, EvidenceStore, RuntimeControllerError
from agent_ros.safety.state import SafetyState
from mcp_server import ros2_mcp_server


def test_module_exposes_stdio_main():
    assert callable(ros2_mcp_server.main)


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
READ_ONLY_TOOLS = {
    "connection_status",
    "list_capabilities",
    "task_status",
    "observe",
    "get_evidence",
}
BANNED_INPUT_FRAGMENTS = {"command", "cmd", "topic", "path", "pid", "payload", "message"}
PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


class FakeState:
    value = "NEW"


class FakeController:
    def __init__(self) -> None:
        self.state = FakeState()
        self.arm_calls: list[tuple[str, str, bool]] = []

    def discover_robot(self, profile_hint=None):
        return {
            "profile": profile_hint or "hospital-amr",
            "state": "DISCOVERED",
            "capabilities": ["mobile_base.twist"],
            "hardware_safety_channel": "simulation_only",
        }

    def validate_profile(self, profile_name):
        return {"profile": profile_name, "state": "VALIDATED", "hardware_safety_channel": "simulation_only"}

    def arm_robot(self, profile_name, challenge, *, dry_run=True):
        self.arm_calls.append((profile_name, challenge, dry_run))
        return {"profile": profile_name, "state": "VALIDATED", "dry_run": dry_run}

    def run_task(self, task_name, *, dry_run=False):
        return {"task": task_name, "state": "RUNNING", "dry_run": dry_run}

    def task_status(self):
        return {"state": "NEW", "task": None, "hardware_safety_channel": "simulation_only"}

    def cancel_task(self):
        return {"state": "ARMED", "adapter_state": "cancelled"}

    def emergency_stop(self):
        return {"state": "ESTOPPED"}

    def observe(self, source):
        return Observation(source, 12.5, {"x": 1.0})

    def get_evidence(self, report_id=None):
        identifier = report_id or "latest"
        return EvidenceReference(identifier, f"{identifier}.json", "application/json", 3)

    def stop_runtime(self):
        return {"state": "NEW"}


def _tools(server=None):
    return asyncio.run((server or ros2_mcp_server.mcp).list_tools())


def _call(server, name, arguments=None):
    return asyncio.run(server.call_tool(name, arguments or {}))


def _tool_map(server=None):
    return {tool.name: tool for tool in _tools(server)}


def test_uses_exact_fastmcp_release_and_exposes_exact_bounded_surface():
    assert fastmcp.__version__ == "3.4.7"
    assert set(_tool_map()) == TOOL_NAMES


def test_every_tool_has_closed_world_annotations_and_a_bounded_timeout():
    tools = _tool_map()
    for name, tool in tools.items():
        assert tool.timeout is not None and 0 < tool.timeout <= 30
        assert tool.annotations is not None
        assert tool.annotations.readOnlyHint is (name in READ_ONLY_TOOLS)
        assert tool.annotations.destructiveHint is (
            name in {"arm_robot", "run_task", "cancel_task", "emergency_stop", "stop_runtime"}
        )
        assert tool.annotations.openWorldHint is False


def test_every_tool_has_a_closed_operation_specific_success_schema():
    expected_data_required = {
        "discover_robot": {"profile", "state", "capabilities", "hardware_safety_channel"},
        "validate_profile": {"profile", "state", "hardware_safety_channel"},
        "connection_status": {"state"},
        "list_capabilities": {"capabilities"},
        "arm_robot": {"profile", "state"},
        "run_task": {"task"},
        "task_status": {"state", "task", "hardware_safety_channel"},
        "cancel_task": {"state", "adapter_state"},
        "emergency_stop": {"state"},
        "observe": {"source", "timestamp", "values"},
        "get_evidence": {"report_id", "relative_path", "media_type", "size"},
        "stop_runtime": {"state"},
    }

    def assert_closed_objects(schema):
        if isinstance(schema, dict):
            if schema.get("type") == "object":
                assert schema.get("additionalProperties") is False
            for value in schema.values():
                assert_closed_objects(value)
        elif isinstance(schema, list):
            for value in schema:
                assert_closed_objects(value)

    for name, tool in _tool_map().items():
        schema = tool.output_schema
        assert schema is not None
        assert_closed_objects(schema)
        success = next(branch for branch in schema["oneOf"] if branch["properties"]["ok"].get("const") is True)
        data = success["properties"]["data"]
        assert set(data["required"]) == expected_data_required[name]
        assert set(data["properties"]) >= expected_data_required[name]
    status_schema = _tool_map()["connection_status"].output_schema["oneOf"][0]["properties"]["data"]["properties"][
        "state"
    ]
    assert set(status_schema["enum"]) == {state.value for state in SafetyState}


def test_tool_schemas_use_repository_enums_and_never_grant_raw_authority():
    tools = _tool_map()
    all_argument_names = {
        argument.lower() for tool in tools.values() for argument in tool.parameters.get("properties", {})
    }
    assert not any(fragment in argument for argument in all_argument_names for fragment in BANNED_INPUT_FRAGMENTS)
    assert tools["discover_robot"].parameters["properties"]["profile_hint"]["anyOf"][0]["enum"] == ["hospital-amr"]
    assert tools["validate_profile"].parameters["properties"]["profile_name"]["enum"] == ["hospital-amr"]
    assert tools["run_task"].parameters["properties"]["task_name"]["enum"] == ["hospital-delivery"]
    assert tools["observe"].parameters["properties"]["source"]["enum"] == ["odometry", "camera", "scan"]
    for tool in tools.values():
        assert tool.parameters["additionalProperties"] is False


def test_challenge_and_report_id_are_bounded_opaque_values():
    tools = _tool_map()
    challenge = tools["arm_robot"].parameters["properties"]["challenge"]
    report_id = tools["get_evidence"].parameters["properties"]["report_id"]["anyOf"][0]
    assert challenge["minLength"] == 1
    assert challenge["maxLength"] <= 128
    assert challenge["pattern"]
    assert report_id["maxLength"] <= 64
    assert report_id["pattern"]


def test_hardware_arming_defaults_to_dry_run_and_passes_only_typed_values():
    controller = FakeController()
    server = ros2_mcp_server.create_server(controller=controller)
    schema = _tool_map(server)["arm_robot"].parameters
    assert schema["properties"]["dry_run"]["default"] is True

    result = _call(
        server,
        "arm_robot",
        {"profile_name": "hospital-amr", "challenge": "operator-7"},
    )

    assert result.structured_content["ok"] is True
    assert controller.arm_calls == [("hospital-amr", "operator-7", True)]


def test_discovery_updates_read_only_capability_listing_without_new_authority():
    server = ros2_mcp_server.create_server(controller=FakeController())
    assert _call(server, "list_capabilities").structured_content == {
        "ok": True,
        "data": {"capabilities": []},
    }
    _call(server, "discover_robot", {"profile_hint": "hospital-amr"})
    assert _call(server, "list_capabilities").structured_content == {
        "ok": True,
        "data": {"capabilities": ["mobile_base.twist"]},
    }


def test_runtime_errors_become_stable_structured_results_without_details():
    class FailingController(FakeController):
        def validate_profile(self, profile_name):
            raise RuntimeControllerError("TIMEOUT") from OSError("secret private runtime token")

    server = ros2_mcp_server.create_server(controller=FailingController())
    result = _call(server, "validate_profile", {"profile_name": "hospital-amr"})

    assert result.is_error is True
    assert result.structured_content["ok"] is False
    assert result.structured_content["error"]["code"] == "TIMEOUT"
    assert set(result.structured_content["error"]) == {"code", "remediation"}
    assert "secret" not in str(result.structured_content)
    assert "private runtime token" not in str(result.structured_content)


def test_unexpected_errors_are_masked_as_stable_unsafe_state():
    class FailingController(FakeController):
        def task_status(self):
            raise ValueError("private subprocess output")

    result = _call(ros2_mcp_server.create_server(controller=FailingController()), "task_status")

    assert result.is_error is True
    assert result.structured_content["error"]["code"] == "UNSAFE_STATE"
    assert "private subprocess output" not in str(result.structured_content)


def test_controller_success_with_invalid_operation_shape_fails_closed():
    class InvalidController(FakeController):
        def discover_robot(self, profile_hint=None):
            return {
                "profile": "hospital-amr",
                "capabilities": ["mobile_base.twist"],
                "raw_subprocess": "must not pass",
            }

    result = _call(
        ros2_mcp_server.create_server(controller=InvalidController()),
        "discover_robot",
        {"profile_hint": "hospital-amr"},
    )

    assert result.is_error is True
    assert result.structured_content["error"]["code"] == "UNSAFE_STATE"
    assert "raw_subprocess" not in str(result.structured_content)


def test_controller_is_a_process_singleton():
    assert ros2_mcp_server.get_runtime_controller() is ros2_mcp_server.get_runtime_controller()
    ros2_mcp_server.close_runtime_controller()


def test_singleton_cleanup_failure_is_observable_and_poisoned_instance_is_not_reused(
    monkeypatch,
):
    class FailingController:
        def stop_runtime(self):
            raise RuntimeControllerError("CLEANUP_FAILED")

    with ros2_mcp_server._controller_condition:
        ros2_mcp_server._controller = FailingController()
        ros2_mcp_server._evidence_store = object()
        ros2_mcp_server._controller_closing = False
        ros2_mcp_server._controller_cleanup_failed = False
    try:
        assert ros2_mcp_server.close_runtime_controller() is False
        with pytest.raises(RuntimeControllerError, match="CLEANUP_FAILED"):
            ros2_mcp_server.get_runtime_controller()
    finally:
        with ros2_mcp_server._controller_condition:
            ros2_mcp_server._controller = None
            ros2_mcp_server._evidence_store = None
            ros2_mcp_server._controller_closing = False
            ros2_mcp_server._controller_cleanup_failed = False
            ros2_mcp_server._controller_condition.notify_all()


def test_concurrent_get_waits_for_close_and_creates_only_one_replacement(monkeypatch):
    close_entered = threading.Event()
    release_close = threading.Event()
    created = []

    class BlockingController:
        def stop_runtime(self):
            close_entered.set()
            assert release_close.wait(0.5)
            return {"state": "NEW"}

    class Replacement:
        pass

    def make_controller(**_kwargs):
        replacement = Replacement()
        created.append(replacement)
        return replacement

    monkeypatch.setattr(ros2_mcp_server, "RuntimeController", make_controller)
    monkeypatch.setattr(ros2_mcp_server, "EvidenceStore", lambda _root: object())
    monkeypatch.setattr(ros2_mcp_server, "RclpyAdapterFactory", lambda: object())
    with ros2_mcp_server._controller_condition:
        ros2_mcp_server._controller = BlockingController()
        ros2_mcp_server._evidence_store = object()
        ros2_mcp_server._controller_closing = False
        ros2_mcp_server._controller_cleanup_failed = False
    close_results = []
    get_results = []
    closer = threading.Thread(target=lambda: close_results.append(ros2_mcp_server.close_runtime_controller()))
    getters = [
        threading.Thread(target=lambda: get_results.append(ros2_mcp_server.get_runtime_controller())) for _ in range(2)
    ]
    try:
        closer.start()
        assert close_entered.wait(0.2)
        for getter in getters:
            getter.start()
        time.sleep(0.02)
        assert get_results == []
        release_close.set()
        closer.join(0.5)
        for getter in getters:
            getter.join(0.5)
        assert close_results == [True]
        assert len(created) == 1
        assert get_results == [created[0], created[0]]
    finally:
        release_close.set()
        closer.join(0.5)
        for getter in getters:
            getter.join(0.5)
        with ros2_mcp_server._controller_condition:
            ros2_mcp_server._controller = None
            ros2_mcp_server._evidence_store = None
            ros2_mcp_server._controller_closing = False
            ros2_mcp_server._controller_cleanup_failed = False
            ros2_mcp_server._controller_condition.notify_all()


def test_observation_dataclass_is_converted_to_validated_structured_data():
    result = _call(
        ros2_mcp_server.create_server(controller=FakeController()),
        "observe",
        {"source": "odometry"},
    )
    assert result.structured_content == {
        "ok": True,
        "data": {"source": "odometry", "timestamp": 12.5, "values": {"x": 1.0}},
    }


def test_png_evidence_is_returned_only_after_signature_and_pixel_decode(tmp_path):
    evidence_dir = tmp_path / "evidence"
    evidence_dir.mkdir()
    (evidence_dir / "camera.png").write_bytes(PNG_1X1)
    store = EvidenceStore(evidence_dir)

    class PngController(FakeController):
        def get_evidence(self, report_id=None):
            return store.get("camera")

    server = ros2_mcp_server.create_server(controller=PngController(), evidence_store=store)
    result = _call(server, "get_evidence", {"report_id": "camera"})

    assert result.is_error is False
    assert result.structured_content["ok"] is True
    assert result.content[0].type == "image"
    assert result.content[0].mimeType == "image/png"


@pytest.mark.parametrize("payload", [b"not a png", b"\x89PNG\r\n\x1a\nnot pixels"])
def test_invalid_png_evidence_is_a_stable_error(tmp_path, payload):
    evidence_dir = tmp_path / "evidence"
    evidence_dir.mkdir()
    (evidence_dir / "camera.png").write_bytes(payload)
    store = EvidenceStore(evidence_dir)

    class PngController(FakeController):
        def get_evidence(self, report_id=None):
            return store.get("camera")

    result = _call(
        ros2_mcp_server.create_server(controller=PngController(), evidence_store=store),
        "get_evidence",
        {"report_id": "camera"},
    )

    assert result.is_error is True
    assert result.structured_content["error"]["code"] == "EVIDENCE_INVALID"


def test_oversized_png_is_rejected_before_evidence_store_reads_it():
    class OversizedController(FakeController):
        def get_evidence(self, report_id=None):
            return EvidenceReference("camera", "camera.png", "image/png", 16 * 1024 * 1024 + 1)

    reads = []

    class ReadDetectingStore:
        def read(self, reference):
            reads.append(reference)
            return PNG_1X1

    result = _call(
        ros2_mcp_server.create_server(controller=OversizedController(), evidence_store=ReadDetectingStore()),
        "get_evidence",
        {"report_id": "camera"},
    )

    assert result.is_error is True
    assert result.structured_content["error"]["code"] == "EVIDENCE_INVALID"
    assert reads == []


def test_fastmcp_enforces_enum_and_extra_argument_schema():
    server = ros2_mcp_server.create_server(controller=FakeController())
    with pytest.raises(Exception):
        _call(server, "observe", {"source": "/arbitrary/topic"})
    with pytest.raises(Exception):
        _call(server, "connection_status", {"pid": 123})


def test_codex_example_is_portable_and_launches_the_module_with_bounded_timeouts():
    path = Path(__file__).resolve().parents[1] / ".codex" / "config.toml.example"
    raw = path.read_text(encoding="utf-8")
    config = tomllib.loads(raw)
    server = config["mcp_servers"]["agent_ros"]

    assert ("/" + "home/") not in raw
    assert "<REPOSITORY_ABSOLUTE_PATH>" in raw
    assert server["command"] == "<REPOSITORY_ABSOLUTE_PATH>/.venv/bin/python"
    assert server["args"] == ["-m", "mcp_server.ros2_mcp_server"]
    assert server["cwd"] == "<REPOSITORY_ABSOLUTE_PATH>"
    assert 0 < server["startup_timeout_sec"] <= 30
    assert 0 < server["tool_timeout_sec"] <= 30
