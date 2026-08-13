from __future__ import annotations

import json
import os
import stat

import pytest

from agent_ros.runtime.audit import AuditError, AuditEvent, AuditOperation, AuditOutcome, AuditWriter
from agent_ros.safety.state import SafetyState


def test_append_writes_sanitized_atomic_jsonl_with_time_state_outcome_and_endpoint_gids(tmp_path, monkeypatch):
    audit_path = tmp_path / "audit.jsonl"
    monkeypatch.setenv("ROBOT_TEST_SECRET", "do-not-leak")
    writer = AuditWriter(audit_path, wall_clock=lambda: 1_700_000_000.25, monotonic_clock=lambda: 42.5)

    writer.append(AuditEvent(
        operation=AuditOperation.START_TASK,
        state_before=SafetyState.ARMED,
        state_after=SafetyState.RUNNING,
        outcome=AuditOutcome.OK,
        operation_data={"task": "delivery", "linear_velocity": 0.25},
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
    assert "ROBOT_TEST_SECRET" not in serialized
    assert "do-not-leak" not in serialized
    assert "/home/alice" not in serialized
    assert stat.S_IMODE(audit_path.stat().st_mode) == 0o600
    assert os.linesep not in lines[0]


def test_append_only_adds_complete_json_records(tmp_path):
    audit_path = tmp_path / "audit.jsonl"
    writer = AuditWriter(audit_path, wall_clock=lambda: 1.0, monotonic_clock=lambda: 2.0)
    writer.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))
    writer.append(AuditEvent(AuditOperation.VALIDATE, SafetyState.DISCOVERED, SafetyState.ARMED, AuditOutcome.OK))

    assert [json.loads(line)["operation"] for line in audit_path.read_text().splitlines()] == [
        "discover", "validate",
    ]


@pytest.mark.parametrize(
    "event",
    [
        AuditEvent("start_task", SafetyState.ARMED, SafetyState.RUNNING, AuditOutcome.OK),
        AuditEvent(AuditOperation.START_TASK, "ARMED", SafetyState.RUNNING, AuditOutcome.OK),
        AuditEvent(AuditOperation.START_TASK, SafetyState.ARMED, SafetyState.RUNNING, "ok"),
        AuditEvent(
            AuditOperation.START_TASK,
            SafetyState.ARMED,
            SafetyState.RUNNING,
            AuditOutcome.OK,
            operation_data={"token": "never-write"},
        ),
        AuditEvent(
            AuditOperation.START_TASK,
            SafetyState.ARMED,
            SafetyState.RUNNING,
            AuditOutcome.OK,
            operation_data={"linear_velocity": "fast"},
        ),
    ],
)
def test_append_rejects_unapproved_audit_shapes_without_writing(tmp_path, event):
    audit_path = tmp_path / "audit.jsonl"

    with pytest.raises(AuditError):
        AuditWriter(audit_path).append(event)
    assert not audit_path.exists()


def test_append_retries_a_real_controlled_short_write_until_the_record_is_complete(tmp_path):
    audit_path = tmp_path / "audit.jsonl"
    calls: list[int] = []

    def short_write(descriptor: int, data: bytes) -> int:
        calls.append(len(data))
        return os.write(descriptor, data[:3])

    writer = AuditWriter(audit_path, write=short_write, wall_clock=lambda: 1.0, monotonic_clock=lambda: 2.0)
    writer.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))

    assert len(calls) > 1
    assert json.loads(audit_path.read_text(encoding="utf-8")) == {
        "endpoint_gids": [],
        "monotonic_time": 2.0,
        "operation": "discover",
        "operation_data": {},
        "outcome": "ok",
        "state": {"from": "NEW", "to": "DISCOVERED"},
        "wall_time": 1.0,
    }
