"""Append-only, schema-bound JSONL audit records for safety operations."""

from __future__ import annotations

import fcntl
import json
import math
import os
import queue
import re
import secrets
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from agent_ros.safety.state import SafetyState

_GID = re.compile(r"[A-Za-z0-9:_-]{1,128}\Z")
_TASK_NAME = re.compile(r"[a-z0-9][a-z0-9-]{0,63}\Z")
_ERROR_CODES = frozenset(
    {
        "DISCOVERY_UNSAFE",
        "ESTOP_LATCHED",
        "HEARTBEAT_EXPIRED",
        "HEARTBEAT_UNCONFIGURED",
        "HARDWARE_CHALLENGE",
        "INTERNAL_ERROR",
        "MOTION_LIMIT",
        "OPERATOR_REQUIRED",
        "PROFILE_UNSUPPORTED",
        "UNSAFE_STATE",
    }
)
_STOP_RESULT_CODES = frozenset(
    {
        "ESTOP_LATCHED",
        "SAFETY_COMMAND_REJECTED",
        "TRANSPORT_UNQUIESCED",
    }
)
_STOP_RESULT_KEYS = frozenset(
    {
        "latched",
        "activation_quiesced",
        "safety_command_accepted",
        "code",
    }
)
_MAX_RECORD_BYTES = 4096
_AUDIT_QUEUE_CAPACITY = 256


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
    session_id: str | None = None


_SUCCESS_TRANSITIONS = {
    AuditOperation.DISCOVER: {(SafetyState.NEW, SafetyState.DISCOVERED)},
    AuditOperation.VALIDATE: {
        (SafetyState.DISCOVERED, SafetyState.VALIDATED),
        (SafetyState.DISCOVERED, SafetyState.ARMED),
    },
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
        self._session_id = secrets.token_hex(16)

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
            "wall_time",
            "monotonic_time",
            "operation",
            "state",
            "outcome",
            "operation_data",
            "endpoint_gids",
            "error_code",
            "session_id",
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
                session_id=value["session_id"],
            )
        except (ValueError, TypeError, KeyError):
            raise AuditError("invalid audit record") from None
        _finite_clock(value["wall_time"])
        _finite_clock(value["monotonic_time"])
        _validate_transition(event)
        _validate_operation_data(event, event.operation_data)
        _validate_endpoint_gids(event.endpoint_gids)
        _validate_error(None, event.error_code)
        _validate_session_id(event.session_id)

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
        if event.session_id is not None:
            raise AuditError("audit session is writer-owned")
        _validate_transition(event)
        operation_data = _validate_operation_data(event, event.operation_data)
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
            "session_id": _validate_session_id(self._session_id),
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
            if (
                isinstance(written, bool)
                or not isinstance(written, int)
                or written <= 0
                or written > len(data) - offset
            ):
                raise AuditError("audit write failed")
            offset += written

    def _rollback(self, descriptor: int, offset: int) -> None:
        try:
            self._truncate(descriptor, offset)
            self._fsync(descriptor)
        except OSError:
            self._integrity_compromised = True
            raise AuditIntegrityError() from None


@dataclass(slots=True)
class _AuditAppendReceipt:
    event: AuditEvent
    done: threading.Event = field(default_factory=threading.Event)
    error: BaseException | None = None


class _AuditAppendWorker:
    """Prestarted bounded owner for calls into an audit writer."""

    def __init__(
        self,
        writer: AuditWriter,
        *,
        thread_factory: Callable[..., threading.Thread] = threading.Thread,
    ) -> None:
        self._writer = writer
        self._thread_factory = thread_factory
        self._queue: queue.Queue[_AuditAppendReceipt | None] = queue.Queue(
            # Keep one reserved slot for the terminal marker.
            maxsize=_AUDIT_QUEUE_CAPACITY + 1
        )
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._in_flight: _AuditAppendReceipt | None = None
        self._accepting = False
        self._failed = False
        self._sentinel_queued = False

    def start(self) -> bool:
        with self._lock:
            if self._thread is not None:
                return self._healthy_locked()
            if self._failed:
                return False
            thread = self._thread_factory(
                target=self._run,
                name="agent-ros-audit",
                daemon=False,
            )
            self._accepting = True
            try:
                thread.start()
            except Exception:
                self._accepting = False
                self._failed = True
                return False
            if not thread.is_alive():
                self._accepting = False
                self._failed = True
                return False
            self._thread = thread
            return True

    def append(self, event: AuditEvent, timeout: float) -> None:
        deadline = _deadline(timeout)
        receipt = _AuditAppendReceipt(event)
        with self._lock:
            if not self._healthy_locked():
                raise AuditError("audit write failed")
            if self._queue.qsize() >= _AUDIT_QUEUE_CAPACITY:
                self._fail_locked()
                raise AuditError("audit write failed")
            try:
                self._queue.put_nowait(receipt)
            except queue.Full:
                self._fail_locked()
                raise AuditError("audit write failed") from None
        if not receipt.done.wait(max(0.0, deadline - time.monotonic())):
            with self._lock:
                if not receipt.done.is_set():
                    receipt.error = AuditError("audit write timed out")
                    receipt.done.set()
                    self._fail_locked()
            raise AuditError("audit write timed out")
        if receipt.error is not None:
            if isinstance(receipt.error, (AuditError, OSError)):
                raise receipt.error
            raise AuditError("audit write failed") from None

    def close(self, timeout: float) -> bool:
        deadline = _deadline(timeout)
        no_wait = max(0.0, float(timeout)) == 0.0
        with self._lock:
            self._accepting = False
            thread = self._thread
            if thread is not None and not thread.is_alive() and self._queue.empty() and self._in_flight is None:
                return True
            self._request_stop_locked()
            live_at_zero = no_wait and thread is not None and thread.is_alive()
        if live_at_zero:
            return False
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, deadline - time.monotonic()))
        with self._lock:
            return thread is None or (not thread.is_alive() and self._queue.empty() and self._in_flight is None)

    @property
    def worker_alive(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        while True:
            receipt = self._queue.get()
            if receipt is None:
                with self._lock:
                    self._sentinel_queued = False
                    self._queue.task_done()
                return
            with self._lock:
                self._in_flight = receipt
            error: BaseException | None = None
            try:
                self._writer.append(receipt.event)
            except BaseException as exc:
                error = exc
            finally:
                with self._lock:
                    if not receipt.done.is_set():
                        receipt.error = error
                        receipt.done.set()
                    if error is not None:
                        self._fail_locked()
                    if self._in_flight is receipt:
                        self._in_flight = None
                    self._queue.task_done()

    def _healthy_locked(self) -> bool:
        thread = self._thread
        return self._accepting and not self._failed and thread is not None and thread.is_alive()

    def _fail_locked(self) -> None:
        self._failed = True
        self._accepting = False
        while True:
            try:
                receipt = self._queue.get_nowait()
            except queue.Empty:
                break
            if receipt is None:
                self._sentinel_queued = False
            if receipt is not None and not receipt.done.is_set():
                receipt.error = AuditError("audit write failed")
                receipt.done.set()
            self._queue.task_done()
        self._request_stop_locked()

    def _request_stop_locked(self) -> None:
        self._stop.set()
        if not self._sentinel_queued:
            self._queue.put_nowait(None)
            self._sentinel_queued = True


def validate_audit_history(raw: bytes, *, require_terminal: bool = False) -> None:
    """Validate bounded records plus plausible cross-record state continuity."""
    if raw and not raw.endswith(b"\n"):
        raise AuditError("invalid audit history")
    previous_after: SafetyState | None = None
    current_session: str | None = None
    completed_sessions: set[str] = set()
    for _index, line in enumerate(raw.splitlines()):
        if not line or len(line) + 1 > _MAX_RECORD_BYTES:
            raise AuditError("invalid audit history")
        try:
            record = json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            raise AuditError("invalid audit history") from None
        AuditWriter.validate_record(record)
        session_id = record["session_id"]
        before = SafetyState(record["state"]["from"])
        after = SafetyState(record["state"]["to"])
        if current_session != session_id:
            if session_id in completed_sessions:
                raise AuditError("invalid audit history")
            if current_session is not None:
                if previous_after not in {SafetyState.STOPPED, SafetyState.ESTOPPED}:
                    raise AuditError("invalid audit history")
                completed_sessions.add(current_session)
            current_session = session_id
            previous_after = None
        if previous_after is None and before is not SafetyState.NEW:
            raise AuditError("invalid audit history")
        if previous_after is not None and before is not previous_after:
            raise AuditError("invalid audit history")
        previous_after = after
    if require_terminal and previous_after not in {None, SafetyState.STOPPED, SafetyState.ESTOPPED}:
        raise AuditError("invalid audit history")


def _validate_session_id(value: object) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{32}", value):
        raise AuditError("invalid audit session")
    return value


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


def _validate_operation_data(event: AuditEvent, value: object) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise AuditError("invalid audit data")
    data = dict(value)
    operation = event.operation
    stop_result_required = operation in {AuditOperation.CANCEL, AuditOperation.ESTOP} or (
        operation is AuditOperation.HEARTBEAT and event.outcome is AuditOutcome.FAULTED
    )
    if stop_result_required:
        if set(data) != _STOP_RESULT_KEYS:
            raise AuditError("unexpected audit data")
        if not all(
            type(data[key]) is bool
            for key in (
                "latched",
                "activation_quiesced",
                "safety_command_accepted",
            )
        ):
            raise AuditError("invalid stop data")
        code = data["code"]
        if not isinstance(code, str) or code not in _STOP_RESULT_CODES:
            raise AuditError("invalid stop data")
        return {
            "latched": data["latched"],
            "activation_quiesced": data["activation_quiesced"],
            "safety_command_accepted": data["safety_command_accepted"],
            "code": code,
        }
    if operation is AuditOperation.HEARTBEAT:
        if data:
            raise AuditError("unexpected audit data")
        return {}
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


def _deadline(timeout: float) -> float:
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout):
        raise AuditError("invalid audit timeout")
    return time.monotonic() + max(0.0, timeout)
