"""Append-only, sanitized JSONL audit records for control-plane operations."""

from __future__ import annotations

import fcntl
import json
import math
import os
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Mapping


_SAFE_KEY = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_SENSITIVE_KEY = re.compile(r"(?:token|secret|password|credential|authorization|environment|exception)", re.I)
_SAFE_ERROR_CODE = re.compile(r"[A-Z][A-Z0-9_]{0,63}\Z")
_MAX_RECORD_BYTES = 4096


@dataclass(frozen=True, slots=True)
class AuditEvent:
    operation: str
    state_before: str
    state_after: str
    outcome: str
    operation_data: Mapping[str, object] = field(default_factory=dict)
    endpoint_gids: tuple[str, ...] = ()
    error: BaseException | None = None
    error_code: str | None = None


class AuditWriter:
    """Serializes concise records without exposing raw request or exception data."""

    def __init__(
        self,
        path: Path,
        *,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._path = Path(path)
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock

    def append(self, event: AuditEvent) -> None:
        record = self._record(event)
        encoded = json.dumps(record, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8") + b"\n"
        if len(encoded) > _MAX_RECORD_BYTES:
            raise ValueError("audit record too large")
        self._path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        os.chmod(self._path.parent, 0o700)
        lock_path = self._path.with_suffix(self._path.suffix + ".lock")
        lock_fd = os.open(lock_path, os.O_WRONLY | os.O_CREAT, 0o600)
        try:
            os.fchmod(lock_fd, 0o600)
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
            data_fd = os.open(self._path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
            try:
                os.fchmod(data_fd, 0o600)
                os.write(data_fd, encoded)
                os.fsync(data_fd)
            finally:
                os.close(data_fd)
        finally:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
            os.close(lock_fd)

    def _record(self, event: AuditEvent) -> dict[str, object]:
        if not isinstance(event, AuditEvent):
            raise ValueError("invalid audit event")
        error_code = event.error_code
        if error_code is None and event.error is not None:
            error_code = "INTERNAL_ERROR"
        if error_code is not None and not _SAFE_ERROR_CODE.fullmatch(error_code):
            error_code = "INTERNAL_ERROR"
        record: dict[str, object] = {
            "wall_time": float(self._wall_clock()),
            "monotonic_time": float(self._monotonic_clock()),
            "operation": _safe_label(event.operation),
            "state": {"from": _safe_label(event.state_before), "to": _safe_label(event.state_after)},
            "outcome": _safe_label(event.outcome),
            "operation_data": _sanitize_mapping(event.operation_data),
            "endpoint_gids": [_safe_gid(gid) for gid in event.endpoint_gids if _safe_gid(gid)],
        }
        if error_code is not None:
            record["error_code"] = error_code
        return record


def _safe_label(value: object) -> str:
    if not isinstance(value, str):
        return "INVALID"
    normalized = value.strip()
    if not normalized or len(normalized) > 64 or any(character in normalized for character in "\r\n\x00"):
        return "INVALID"
    return normalized


def _safe_gid(value: object) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9:_-]{1,128}", value):
        return ""
    return value


def _sanitize_mapping(data: Mapping[str, object]) -> dict[str, object]:
    if not isinstance(data, Mapping):
        return {}
    sanitized: dict[str, object] = {}
    for key, value in data.items():
        if not isinstance(key, str) or not _SAFE_KEY.fullmatch(key) or _SENSITIVE_KEY.search(key):
            continue
        safe_value = _sanitize_value(value)
        if safe_value is not None:
            sanitized[key] = safe_value
    return sanitized


def _sanitize_value(value: object) -> object | None:
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        if re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
            return value
        return None
    if isinstance(value, (tuple, list)):
        values = [_sanitize_value(item) for item in value]
        return [item for item in values if item is not None]
    return None
