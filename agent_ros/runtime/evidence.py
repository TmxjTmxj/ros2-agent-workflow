"""Structured evidence references confined to a configured runtime directory."""

from __future__ import annotations

import json
import os
import re
import secrets
import stat
from dataclasses import dataclass
from pathlib import Path


_REPORT_ID = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}\Z")
_MEDIA_TYPES = {".json": "application/json", ".png": "image/png"}


class EvidenceError(RuntimeError):
    code = "EVIDENCE_INVALID"

    def __init__(self) -> None:
        super().__init__(self.code)


@dataclass(frozen=True, slots=True)
class EvidenceReference:
    report_id: str
    relative_path: str
    media_type: str
    size: int


class EvidenceStore:
    """Resolve opaque report IDs without exposing arbitrary filesystem authority."""

    def __init__(self, root: Path) -> None:
        try:
            self._root = Path(root).resolve()
            self._root.mkdir(mode=0o700, parents=True, exist_ok=True)
            os.chmod(self._root, 0o700)
            descriptor = self._open_root()
            self._close(descriptor)
        except OSError:
            raise EvidenceError() from None

    def get(self, report_id: str | None = None) -> EvidenceReference:
        if report_id is None:
            candidates = self._safe_candidates()
            if not candidates:
                raise EvidenceError()
            name, info = max(candidates, key=lambda item: (item[1].st_mtime_ns, item[0]))
            report_id = Path(name).stem
        else:
            self._validate_id(report_id)
            candidates = []
            for suffix in _MEDIA_TYPES:
                name = f"{report_id}{suffix}"
                try:
                    info = self._stat_name(name)
                except EvidenceError:
                    continue
                candidates.append((name, info))
            if len(candidates) != 1:
                raise EvidenceError()
            name, info = candidates[0]
        suffix = Path(name).suffix
        return EvidenceReference(report_id, name, _MEDIA_TYPES[suffix], info.st_size)

    def read(self, reference: EvidenceReference) -> bytes:
        if not isinstance(reference, EvidenceReference):
            raise EvidenceError()
        self._validate_id(reference.report_id)
        suffix = Path(reference.relative_path).suffix
        name = f"{reference.report_id}{suffix}"
        if name != reference.relative_path or _MEDIA_TYPES.get(suffix) != reference.media_type:
            raise EvidenceError()
        root_fd = -1
        descriptor = -1
        try:
            root_fd = self._open_root()
            descriptor = os.open(name, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0), dir_fd=root_fd)
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode):
                raise OSError
            chunks = []
            while True:
                chunk = os.read(descriptor, 65536)
                if not chunk:
                    break
                chunks.append(chunk)
            return b"".join(chunks)
        except OSError:
            raise EvidenceError() from None
        finally:
            if descriptor >= 0:
                self._close(descriptor)
            if root_fd >= 0:
                self._close(root_fd)

    def write_json(self, report_id: str, value: object) -> EvidenceReference:
        self._validate_id(report_id)
        if not isinstance(value, (dict, list)):
            raise EvidenceError()
        try:
            encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8") + b"\n"
        except (TypeError, ValueError):
            raise EvidenceError() from None
        destination = f"{report_id}.json"
        temporary = f".{report_id}.{secrets.token_hex(8)}.tmp"
        root_fd = -1
        descriptor = -1
        try:
            root_fd = self._open_root()
            descriptor = os.open(
                temporary,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
                0o600,
                dir_fd=root_fd,
            )
            offset = 0
            while offset < len(encoded):
                written = os.write(descriptor, encoded[offset:])
                if written <= 0:
                    raise OSError
                offset += written
            os.fsync(descriptor)
            self._close(descriptor)
            descriptor = -1
            os.replace(temporary, destination, src_dir_fd=root_fd, dst_dir_fd=root_fd)
            os.fsync(root_fd)
        except OSError:
            try:
                if root_fd >= 0:
                    os.unlink(temporary, dir_fd=root_fd)
            except OSError:
                pass
            raise EvidenceError() from None
        finally:
            if descriptor >= 0:
                self._close(descriptor)
            if root_fd >= 0:
                self._close(root_fd)
        return self.get(report_id)

    def _safe_candidates(self) -> list[tuple[str, os.stat_result]]:
        root_fd = -1
        try:
            root_fd = self._open_root()
            candidates = []
            for name in os.listdir(root_fd):
                path = Path(name)
                if path.suffix not in _MEDIA_TYPES or not _REPORT_ID.fullmatch(path.stem):
                    continue
                try:
                    info = os.stat(name, dir_fd=root_fd, follow_symlinks=False)
                except OSError:
                    continue
                if stat.S_ISREG(info.st_mode):
                    candidates.append((name, info))
            return candidates
        except OSError:
            raise EvidenceError() from None
        finally:
            if root_fd >= 0:
                self._close(root_fd)

    def _stat_name(self, name: str) -> os.stat_result:
        root_fd = -1
        try:
            root_fd = self._open_root()
            info = os.stat(name, dir_fd=root_fd, follow_symlinks=False)
        except OSError:
            raise EvidenceError() from None
        finally:
            if root_fd >= 0:
                self._close(root_fd)
        if not stat.S_ISREG(info.st_mode):
            raise EvidenceError()
        return info

    def _open_root(self) -> int:
        return os.open(self._root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0))

    @staticmethod
    def _close(descriptor: int) -> None:
        try:
            os.close(descriptor)
        except OSError:
            raise EvidenceError() from None

    @staticmethod
    def _validate_id(report_id: object) -> None:
        if not isinstance(report_id, str) or not _REPORT_ID.fullmatch(report_id):
            raise EvidenceError()
