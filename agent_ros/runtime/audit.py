"""Append-only, schema-bound JSONL audit records for safety operations."""

from __future__ import annotations

import fcntl
import json
import math
import os
import re
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from agent_ros.safety.state import SafetyState


_GID = re.compile(r"[A-Za-z0-9:_-]{1,128}\Z")
_TASK_NAME = re.compile(r"[a-z0-9][a-z0-9-]{0,63}\Z")
_ERROR_CODES = frozenset({
    "DISCOVERY_UNSAFE", "ESTOP_LATCHED", "HEARTBEAT_EXPIRED", "HEARTBEAT_UNCONFIGURED",
    "HARDWARE_CHALLENGE", "INTERNAL_ERROR", "MOTION_LIMIT", "OPERATOR_REQUIRED",
    "PROFILE_UNSUPPORTED", "UNSAFE_STATE",
})
_MAX_RECORD_BYTES = 4096


class AuditError(ValueError):
    """A stable failure that leaves unsafe audit input unwritten."""


class AuditIntegrityError(AuditError):
    """The append rollback was not durable; this writer must never be reused."""

    code = "AUDIT_INTEGRITY_COMPROMISED"

    def __init__(self) -> None:
        super().__init__(self.code)


class AuditOperation(str, Enum):
    DISCOVER = "discover"
    VALIDATE = "validate"
    ARM = "arm"
    START_TASK = "start_task"
    HEARTBEAT = "heartbeat"
    CANCEL = "cancel"
    ESTOP = "estop"
    OPERATOR_RESET = "operator_reset"


class AuditOutcome(str, Enum):
    OK = "ok"
    REJECTED = "rejected"
    FAULTED = "faulted"


@dataclass(frozen=True, slots=True)
class AuditEvent:
    operation: AuditOperation
    state_before: SafetyState
    state_after: SafetyState
    outcome: AuditOutcome
    operation_data: Mapping[str, object] = field(default_factory=dict)
    endpoint_gids: tuple[str, ...] = ()
    error: BaseException | None = None
    error_code: str | None = None


_SUCCESS_TRANSITIONS = {
    AuditOperation.DISCOVER: {(SafetyState.NEW, SafetyState.DISCOVERED)},
    AuditOperation.VALIDATE: {(SafetyState.DISCOVERED, SafetyState.VALIDATED), (SafetyState.DISCOVERED, SafetyState.ARMED)},
    AuditOperation.ARM: {(SafetyState.VALIDATED, SafetyState.ARMED)},
    AuditOperation.START_TASK: {(SafetyState.ARMED, SafetyState.RUNNING)},
    AuditOperation.HEARTBEAT: {(SafetyState.RUNNING, SafetyState.RUNNING)},
    AuditOperation.CANCEL: {(SafetyState.RUNNING, SafetyState.STOPPED)},
    AuditOperation.ESTOP: {(state, SafetyState.ESTOPPED) for state in SafetyState if state is not SafetyState.ESTOPPED},
    AuditOperation.OPERATOR_RESET: {(SafetyState.ESTOPPED, SafetyState.NEW)},
}
_FAULTED_TRANSITIONS = {
    AuditOperation.HEARTBEAT: {(SafetyState.RUNNING, SafetyState.FAULTED)},
}


class AuditWriter:
    """Append only fully validated, bounded records using a write-all syscall loop."""

    def __init__(
        self,
        path: Path,
        *,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
        write: Callable[[int, bytes], int] = os.write,
        truncate: Callable[[int, int], None] = os.ftruncate,
        fsync: Callable[[int], None] = os.fsync,
    ) -> None:
        self._path = Path(path)
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._write = write
        self._truncate = truncate
        self._fsync = fsync
        self._integrity_compromised = False

    def append(self, event: AuditEvent) -> None:
        if self._integrity_compromised:
            raise AuditIntegrityError()
        encoded = self._encode(event)
        self._path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        os.chmod(self._path.parent, 0o700)
        lock_path = self._path.with_suffix(self._path.suffix + ".lock")
        try:
            lock_fd = os.open(lock_path, os.O_WRONLY | os.O_CREAT, 0o600)
            try:
                os.fchmod(lock_fd, 0o600)
                fcntl.flock(lock_fd, fcntl.LOCK_EX)
                data_fd = os.open(self._path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
                try:
                    offset = os.lseek(data_fd, 0, os.SEEK_END)
                    try:
                        os.fchmod(data_fd, 0o600)
                        self._write_all(data_fd, encoded)
                        self._fsync(data_fd)
                    except (AuditError, OSError):
                        self._rollback(data_fd, offset)
                        raise AuditError("audit write failed") from None
                finally:
                    os.close(data_fd)
            finally:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)
                os.close(lock_fd)
        except OSError:
            raise AuditError("audit write failed") from None

    @staticmethod
    def validate_record(value: object) -> None:
        """Validate a persisted record through the same event schema and enums."""
        if not isinstance(value, Mapping):
            raise AuditError("invalid audit record")
        allowed = {
            "wall_time", "monotonic_time", "operation", "state", "outcome",
            "operation_data", "endpoint_gids", "error_code",
        }
        if set(value) - allowed or allowed - {"error_code"} - set(value):
            raise AuditError("invalid audit record")
        state = value.get("state")
        if not isinstance(state, Mapping) or set(state) != {"from", "to"}:
            raise AuditError("invalid audit record")
        try:
            if not isinstance(value["endpoint_gids"], list):
                raise AuditError("invalid audit record")
            event = AuditEvent(
                AuditOperation(value["operation"]),
                SafetyState(state["from"]),
                SafetyState(state["to"]),
                AuditOutcome(value["outcome"]),
                operation_data=value["operation_data"],
                endpoint_gids=tuple(value["endpoint_gids"]),
                error_code=value.get("error_code"),
            )
        except (ValueError, TypeError, KeyError):
            raise AuditError("invalid audit record") from None
        _finite_clock(value["wall_time"])
        _finite_clock(value["monotonic_time"])
        _validate_transition(event)
        _validate_operation_data(event.operation, event.operation_data)
        _validate_endpoint_gids(event.endpoint_gids)
        _validate_error(None, event.error_code)

    def _encode(self, event: AuditEvent) -> bytes:
        record = self._record(event)
        encoded = json.dumps(record, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8") + b"\n"
        if len(encoded) > _MAX_RECORD_BYTES:
            raise AuditError("audit record too large")
        return encoded

    def _record(self, event: AuditEvent) -> dict[str, object]:
        if not isinstance(event, AuditEvent):
            raise AuditError("invalid audit event")
        if not isinstance(event.operation, AuditOperation) or not isinstance(event.outcome, AuditOutcome):
            raise AuditError("invalid audit enum")
        if not isinstance(event.state_before, SafetyState) or not isinstance(event.state_after, SafetyState):
            raise AuditError("invalid audit state")
        _validate_transition(event)
        operation_data = _validate_operation_data(event.operation, event.operation_data)
        endpoint_gids = _validate_endpoint_gids(event.endpoint_gids)
        error_code = _validate_error(event.error, event.error_code)
        wall_time = _finite_clock(self._wall_clock())
        monotonic_time = _finite_clock(self._monotonic_clock())
        record: dict[str, object] = {
            "wall_time": wall_time,
            "monotonic_time": monotonic_time,
            "operation": event.operation.value,
            "state": {"from": event.state_before.value, "to": event.state_after.value},
            "outcome": event.outcome.value,
            "operation_data": operation_data,
            "endpoint_gids": list(endpoint_gids),
        }
        if error_code is not None:
            record["error_code"] = error_code
        return record

    def _write_all(self, descriptor: int, data: bytes) -> None:
        offset = 0
        while offset < len(data):
            try:
                written = self._write(descriptor, data[offset:])
            except InterruptedError:
                continue
            except OSError as exc:
                raise AuditError("audit write failed") from exc
            if isinstance(written, bool) or not isinstance(written, int) or written <= 0 or written > len(data) - offset:
                raise AuditError("audit write failed")
            offset += written

    def _rollback(self, descriptor: int, offset: int) -> None:
        try:
            self._truncate(descriptor, offset)
            self._fsync(descriptor)
        except OSError:
            self._integrity_compromised = True
            raise AuditIntegrityError() from None


def _finite_clock(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise AuditError("invalid audit clock")
    return float(value)


def _validate_transition(event: AuditEvent) -> None:
    transition = (event.state_before, event.state_after)
    if event.outcome is AuditOutcome.REJECTED:
        if event.state_before is not event.state_after:
            raise AuditError("invalid audit transition")
        return
    if event.outcome is AuditOutcome.FAULTED:
        if transition not in _FAULTED_TRANSITIONS.get(event.operation, set()):
            raise AuditError("invalid audit transition")
        return
    if transition not in _SUCCESS_TRANSITIONS[event.operation]:
        raise AuditError("invalid audit transition")


def _validate_error(error: BaseException | None, error_code: str | None) -> str | None:
    if error is not None and not isinstance(error, BaseException):
        raise AuditError("invalid audit error")
    if error_code is None:
        return "INTERNAL_ERROR" if error is not None else None
    if error_code not in _ERROR_CODES:
        raise AuditError("invalid audit error")
    return error_code


def _validate_endpoint_gids(value: object) -> tuple[str, ...]:
    if not isinstance(value, tuple) or not all(isinstance(gid, str) and _GID.fullmatch(gid) for gid in value):
        raise AuditError("invalid endpoint gids")
    return value


def _validate_operation_data(operation: AuditOperation, value: object) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise AuditError("invalid audit data")
    data = dict(value)
    if operation is not AuditOperation.START_TASK:
        if data:
            raise AuditError("unexpected audit data")
        return {}
    allowed = {"task", "linear_velocity", "angular_velocity", "linear_acceleration", "angular_acceleration"}
    if set(data) - allowed:
        raise AuditError("unexpected audit data")
    validated: dict[str, object] = {}
    for key, item in data.items():
        if key == "task":
            if not isinstance(item, str) or not _TASK_NAME.fullmatch(item):
                raise AuditError("invalid task data")
            validated[key] = item
        elif isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item):
            validated[key] = float(item)
        else:
            raise AuditError("invalid motion data")
    return validated
