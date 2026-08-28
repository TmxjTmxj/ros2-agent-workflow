"""Out-of-band, single-use hardware operator challenge records."""

from __future__ import annotations

import fcntl
import hashlib
import hmac
import json
import os
import re
import secrets
import tempfile
import time
from collections.abc import Callable
from pathlib import Path

_PROFILE_NAME = re.compile(r"[a-z0-9][a-z0-9-]*\Z")
_DEFAULT_TTL_SECONDS = 60.0
_MAX_TTL_SECONDS = 300.0


class ChallengeError(ValueError):
    """Raised without exposing filesystem or secret details to control clients."""


def linux_boot_id() -> str:
    """Read Linux's per-boot identifier; challenge creation fails closed if unavailable."""
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    except OSError as exc:
        raise ChallengeError("boot identity unavailable") from exc


def create_operator_challenge(
    profile_name: str,
    runtime_dir: Path,
    *,
    ttl_seconds: float = _DEFAULT_TTL_SECONDS,
    monotonic_clock: Callable[[], float] = time.monotonic,
    boot_id: Callable[[], str] = linux_boot_id,
) -> str:
    """Create one short-lived token for a local operator, never for MCP callers."""
    _validate_profile_name(profile_name)
    if (
        not isinstance(ttl_seconds, (int, float))
        or isinstance(ttl_seconds, bool)
        or not 0 < ttl_seconds <= _MAX_TTL_SECONDS
    ):
        raise ChallengeError("invalid challenge lifetime")
    runtime = _secure_runtime_dir(runtime_dir)
    current_boot_id = _validated_boot_id(boot_id)
    token = secrets.token_urlsafe(32)
    record = {
        "hash": _token_hash(token),
        "profile": profile_name,
        "expires_monotonic": float(monotonic_clock()) + float(ttl_seconds),
        "boot_id": current_boot_id,
        "used": False,
    }
    _write_record(_challenge_path(runtime, profile_name), record)
    return token


def consume_operator_challenge(
    profile_name: str,
    runtime_dir: Path,
    token: str,
    *,
    monotonic_clock: Callable[[], float] = time.monotonic,
    boot_id: Callable[[], str] = linux_boot_id,
) -> bool:
    """Atomically consume an exact, unexpired local operator challenge."""
    if not isinstance(token, str) or not token:
        return False
    try:
        _validate_profile_name(profile_name)
        runtime = _secure_runtime_dir(runtime_dir)
        record_path = _challenge_path(runtime, profile_name)
        lock_path = record_path.with_suffix(record_path.suffix + ".lock")
        lock_fd = os.open(lock_path, os.O_WRONLY | os.O_CREAT, 0o600)
        try:
            os.fchmod(lock_fd, 0o600)
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
            try:
                record = json.loads(record_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                return False
            if not _valid_record(record, profile_name, token, monotonic_clock(), _validated_boot_id(boot_id)):
                return False
            record["used"] = True
            _write_record(record_path, record)
            return True
        finally:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
            os.close(lock_fd)
    except (ChallengeError, OSError):
        return False


def _valid_record(record: object, profile_name: str, token: str, now: float, boot_id: str) -> bool:
    if not isinstance(record, dict):
        return False
    expected = record.get("hash")
    expiry = record.get("expires_monotonic")
    return (
        record.get("profile") == profile_name
        and record.get("boot_id") == boot_id
        and record.get("used") is False
        and isinstance(expected, str)
        and isinstance(expiry, (int, float))
        and not isinstance(expiry, bool)
        and now < expiry
        and hmac.compare_digest(expected, _token_hash(token))
    )


def _token_hash(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _validated_boot_id(provider: Callable[[], str]) -> str:
    try:
        value = provider()
    except Exception as exc:
        raise ChallengeError("boot identity unavailable") from exc
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9-]{8,128}", value):
        raise ChallengeError("boot identity unavailable")
    return value


def _validate_profile_name(profile_name: str) -> None:
    if not isinstance(profile_name, str) or not _PROFILE_NAME.fullmatch(profile_name):
        raise ChallengeError("invalid profile name")


def _secure_runtime_dir(runtime_dir: Path) -> Path:
    runtime = Path(runtime_dir)
    runtime.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(runtime, 0o700)
    return runtime


def _challenge_path(runtime: Path, profile_name: str) -> Path:
    return runtime / f"{profile_name}.challenge.json"


def _write_record(path: Path, record: dict[str, object]) -> None:
    payload = json.dumps(record, sort_keys=True, separators=(",", ":")).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
        os.chmod(path, 0o600)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
