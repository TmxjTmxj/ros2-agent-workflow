from __future__ import annotations

import json
import os
import stat

from agent_ros.runtime.audit import AuditEvent, AuditWriter


def test_append_writes_sanitized_atomic_jsonl_with_time_state_outcome_and_endpoint_gids(tmp_path, monkeypatch):
    audit_path = tmp_path / "audit.jsonl"
    monkeypatch.setenv("ROBOT_TEST_SECRET", "do-not-leak")
    writer = AuditWriter(audit_path, wall_clock=lambda: 1_700_000_000.25, monotonic_clock=lambda: 42.5)

    writer.append(AuditEvent(
        operation="start_task",
        state_before="ARMED",
        state_after="RUNNING",
        outcome="ok",
        operation_data={"task": "delivery", "linear_velocity": 0.25, "token": "never-write"},
        endpoint_gids=("01", "02"),
        error=RuntimeError("ROBOT_TEST_SECRET=do-not-leak /home/alice"),
    ))

    lines = audit_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["wall_time"] == 1_700_000_000.25
    assert record["monotonic_time"] == 42.5
    assert record["state"] == {"from": "ARMED", "to": "RUNNING"}
    assert record["outcome"] == "ok"
    assert record["endpoint_gids"] == ["01", "02"]
    assert record["operation_data"] == {"linear_velocity": 0.25, "task": "delivery"}
    assert record["error_code"] == "INTERNAL_ERROR"
    serialized = audit_path.read_text(encoding="utf-8")
    assert "never-write" not in serialized
    assert "ROBOT_TEST_SECRET" not in serialized
    assert "do-not-leak" not in serialized
    assert "/home/alice" not in serialized
    assert stat.S_IMODE(audit_path.stat().st_mode) == 0o600
    assert os.linesep not in lines[0]


def test_append_only_adds_complete_json_records(tmp_path):
    audit_path = tmp_path / "audit.jsonl"
    writer = AuditWriter(audit_path, wall_clock=lambda: 1.0, monotonic_clock=lambda: 2.0)
    writer.append(AuditEvent("discover", "NEW", "DISCOVERED", "ok"))
    writer.append(AuditEvent("validate", "DISCOVERED", "ARMED", "ok"))

    assert [json.loads(line)["operation"] for line in audit_path.read_text().splitlines()] == [
        "discover", "validate",
    ]
