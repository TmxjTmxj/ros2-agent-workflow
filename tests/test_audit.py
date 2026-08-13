from __future__ import annotations

import json
import os
import stat
import threading

import pytest

from agent_ros.runtime.audit import (
    AuditError,
    AuditEvent,
    AuditIntegrityError,
    AuditOperation,
    AuditOutcome,
    AuditWriter,
    _AuditAppendWorker,
    validate_audit_history,
)
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


def test_audit_worker_failure_after_close_sentinel_still_exits_and_joins():
    entered = threading.Event()
    release = threading.Event()

    class FailingWriter:
        def append(self, _event):
            entered.set()
            release.wait()
            raise AuditError("controlled failure")

    worker = _AuditAppendWorker(FailingWriter())
    assert worker.start()
    event = AuditEvent(
        AuditOperation.DISCOVER,
        SafetyState.NEW,
        SafetyState.DISCOVERED,
        AuditOutcome.OK,
    )
    append_errors = []
    appender = threading.Thread(
        target=lambda: _capture_audit_error(append_errors, worker.append, event, 1.0)
    )
    close_results = []
    closer = threading.Thread(
        target=lambda: close_results.append(worker.close(0.1))
    )

    appender.start()
    assert entered.wait(1.0)
    closer.start()
    assert worker._stop.wait(1.0)
    release.set()
    appender.join(1.0)
    closer.join(0.5)

    close_completed = not closer.is_alive()
    worker_exited = not worker.worker_alive
    if not worker_exited:
        worker._queue.put_nowait(None)
        worker._thread.join(1.0)
        assert not worker.worker_alive

    assert not appender.is_alive()
    assert close_completed
    assert len(append_errors) == 1
    assert close_results == [True]
    assert worker_exited


def _capture_audit_error(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_estop_audit_accepts_only_structured_stop_result_fields(tmp_path):
    audit_path = tmp_path / "audit.jsonl"
    writer = AuditWriter(
        audit_path,
        wall_clock=lambda: 1.0,
        monotonic_clock=lambda: 2.0,
    )

    writer.append(AuditEvent(
        AuditOperation.ESTOP,
        SafetyState.RUNNING,
        SafetyState.ESTOPPED,
        AuditOutcome.OK,
        operation_data={
            "latched": True,
            "activation_quiesced": False,
            "safety_command_accepted": True,
            "code": "TRANSPORT_UNQUIESCED",
        },
    ))

    record = json.loads(audit_path.read_text(encoding="utf-8"))
    assert record["operation_data"] == {
        "activation_quiesced": False,
        "code": "TRANSPORT_UNQUIESCED",
        "latched": True,
        "safety_command_accepted": True,
    }


_STOP_RESULT_DATA = {
    "latched": True,
    "activation_quiesced": True,
    "safety_command_accepted": True,
    "code": "ESTOP_LATCHED",
}


@pytest.mark.parametrize(
    "event",
    [
        AuditEvent(
            AuditOperation.CANCEL,
            SafetyState.RUNNING,
            SafetyState.STOPPED,
            AuditOutcome.OK,
        ),
        AuditEvent(
            AuditOperation.ESTOP,
            SafetyState.RUNNING,
            SafetyState.ESTOPPED,
            AuditOutcome.OK,
        ),
        AuditEvent(
            AuditOperation.HEARTBEAT,
            SafetyState.RUNNING,
            SafetyState.FAULTED,
            AuditOutcome.FAULTED,
        ),
        AuditEvent(
            AuditOperation.HEARTBEAT,
            SafetyState.RUNNING,
            SafetyState.RUNNING,
            AuditOutcome.OK,
            operation_data=_STOP_RESULT_DATA,
        ),
        AuditEvent(
            AuditOperation.CANCEL,
            SafetyState.RUNNING,
            SafetyState.STOPPED,
            AuditOutcome.OK,
            operation_data={
                "latched": True,
                "activation_quiesced": True,
                "safety_command_accepted": True,
            },
        ),
        AuditEvent(
            AuditOperation.ESTOP,
            SafetyState.RUNNING,
            SafetyState.ESTOPPED,
            AuditOutcome.OK,
            operation_data={**_STOP_RESULT_DATA, "extra": False},
        ),
    ],
)
def test_stop_transition_data_rejects_missing_empty_or_mismatched_shapes(tmp_path, event):
    audit_path = tmp_path / "audit.jsonl"

    with pytest.raises(AuditError):
        AuditWriter(audit_path).append(event)

    assert not audit_path.exists()


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
    record = json.loads(audit_path.read_text(encoding="utf-8"))
    assert len(record.pop("session_id")) == 32
    assert record == {
        "endpoint_gids": [],
        "monotonic_time": 2.0,
        "operation": "discover",
        "operation_data": {},
        "outcome": "ok",
        "state": {"from": "NEW", "to": "DISCOVERED"},
        "wall_time": 1.0,
    }


@pytest.mark.parametrize("failure", ["raise", "zero"])
def test_append_rolls_back_a_partial_write_when_the_following_write_fails(tmp_path, failure):
    audit_path = tmp_path / "audit.jsonl"
    initial = AuditWriter(audit_path, wall_clock=lambda: 1.0, monotonic_clock=lambda: 2.0)
    initial.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))
    original = audit_path.read_bytes()
    calls = [0]

    def partial_then_fail(descriptor: int, data: bytes) -> int:
        calls[0] += 1
        if calls[0] == 1:
            return os.write(descriptor, data[:5])
        if failure == "raise":
            raise OSError("controlled write failure")
        return 0

    failing = AuditWriter(audit_path, write=partial_then_fail)
    rejected = AuditEvent(AuditOperation.VALIDATE, SafetyState.DISCOVERED, SafetyState.ARMED, AuditOutcome.OK)

    with pytest.raises(AuditError, match="audit write failed"):
        failing.append(rejected)
    assert audit_path.read_bytes() == original

    initial.append(rejected)
    assert [json.loads(line)["operation"] for line in audit_path.read_text(encoding="utf-8").splitlines()] == [
        "discover", "validate",
    ]


@pytest.mark.parametrize("operation", list(AuditOperation))
@pytest.mark.parametrize("state", list(SafetyState))
def test_each_operation_can_record_rejection_at_every_same_safety_state(tmp_path, operation, state):
    audit_path = tmp_path / "audit.jsonl"
    operation_data = (
        _STOP_RESULT_DATA
        if operation in {AuditOperation.CANCEL, AuditOperation.ESTOP}
        else {}
    )

    AuditWriter(audit_path).append(AuditEvent(
        operation,
        state,
        state,
        AuditOutcome.REJECTED,
        operation_data=operation_data,
    ))

    record = json.loads(audit_path.read_text(encoding="utf-8"))
    assert record["operation"] == operation.value
    assert record["outcome"] == "rejected"
    assert record["state"] == {"from": state.value, "to": state.value}


@pytest.mark.parametrize(
    "operation,before,after",
    [
        (AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED),
        (AuditOperation.VALIDATE, SafetyState.DISCOVERED, SafetyState.ARMED),
        (AuditOperation.ARM, SafetyState.VALIDATED, SafetyState.ARMED),
        (AuditOperation.START_TASK, SafetyState.ARMED, SafetyState.RUNNING),
        (AuditOperation.HEARTBEAT, SafetyState.RUNNING, SafetyState.FAULTED),
        (AuditOperation.CANCEL, SafetyState.RUNNING, SafetyState.STOPPED),
        (AuditOperation.ESTOP, SafetyState.RUNNING, SafetyState.ESTOPPED),
        (AuditOperation.OPERATOR_RESET, SafetyState.ESTOPPED, SafetyState.NEW),
    ],
)
def test_rejected_outcome_cannot_forge_any_cross_state_transition(tmp_path, operation, before, after):
    audit_path = tmp_path / "audit.jsonl"

    with pytest.raises(AuditError, match="invalid audit transition"):
        AuditWriter(audit_path).append(AuditEvent(
            operation,
            before,
            after,
            AuditOutcome.REJECTED,
        ))
    assert not audit_path.exists()


@pytest.mark.parametrize("rollback_failure", ["truncate", "fsync"])
def test_rollback_failure_locks_writer_and_never_exposes_raw_error_data(tmp_path, rollback_failure):
    audit_path = tmp_path / "audit.jsonl"
    initial = AuditWriter(audit_path)
    initial.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))
    calls = [0]

    def partial_then_fail(descriptor: int, data: bytes) -> int:
        calls[0] += 1
        if calls[0] == 1:
            return os.write(descriptor, data[:4])
        raise OSError("super-secret /unsafe/path")

    def fail_truncate(_descriptor: int, _offset: int) -> None:
        raise OSError("super-secret /unsafe/path")

    def fail_fsync(_descriptor: int) -> None:
        raise OSError("super-secret /unsafe/path")

    writer = AuditWriter(
        audit_path,
        write=partial_then_fail,
        truncate=fail_truncate if rollback_failure == "truncate" else os.ftruncate,
        fsync=os.fsync if rollback_failure == "truncate" else fail_fsync,
    )
    event = AuditEvent(AuditOperation.VALIDATE, SafetyState.DISCOVERED, SafetyState.ARMED, AuditOutcome.OK)

    with pytest.raises(AuditIntegrityError) as captured:
        writer.append(event)
    assert str(captured.value) == "AUDIT_INTEGRITY_COMPROMISED"
    assert "secret" not in str(captured.value)
    assert "path" not in str(captured.value)
    writes_after_integrity_failure = calls[0]
    with pytest.raises(AuditIntegrityError):
        writer.append(event)
    assert calls[0] == writes_after_integrity_failure


@pytest.mark.parametrize("previous_state", [SafetyState.RUNNING, SafetyState.FAULTED])
def test_new_session_is_rejected_when_previous_session_was_nonterminal(tmp_path, previous_state):
    audit_path = tmp_path / "audit.jsonl"
    first = AuditWriter(audit_path, wall_clock=lambda: 1.0, monotonic_clock=lambda: 1.0)
    first.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))
    first.append(AuditEvent(AuditOperation.VALIDATE, SafetyState.DISCOVERED, SafetyState.ARMED, AuditOutcome.OK))
    first.append(AuditEvent(AuditOperation.START_TASK, SafetyState.ARMED, SafetyState.RUNNING, AuditOutcome.OK))
    if previous_state is SafetyState.FAULTED:
        first.append(AuditEvent(
            AuditOperation.HEARTBEAT,
            SafetyState.RUNNING,
            SafetyState.FAULTED,
            AuditOutcome.FAULTED,
            operation_data=_STOP_RESULT_DATA,
        ))
    second = AuditWriter(audit_path, wall_clock=lambda: 2.0, monotonic_clock=lambda: 2.0)
    second.append(AuditEvent(AuditOperation.DISCOVER, SafetyState.NEW, SafetyState.DISCOVERED, AuditOutcome.OK))

    with pytest.raises(AuditError, match="invalid audit history"):
        validate_audit_history(audit_path.read_bytes())
