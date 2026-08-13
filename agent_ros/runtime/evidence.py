"""Structured evidence references confined to a configured runtime directory."""

from __future__ import annotations

import json
import os
import re
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
        self._root = Path(root).resolve()
        self._root.mkdir(mode=0o700, parents=True, exist_ok=True)
        os.chmod(self._root, 0o700)

    def get(self, report_id: str | None = None) -> EvidenceReference:
        if report_id is None:
            candidates = self._safe_candidates()
            if not candidates:
                raise EvidenceError()
            path = max(candidates, key=lambda item: (item.stat().st_mtime_ns, item.name))
            report_id = path.stem
        else:
            self._validate_id(report_id)
            candidates = [self._candidate(report_id, suffix) for suffix in _MEDIA_TYPES]
            candidates = [path for path in candidates if path.exists()]
            if len(candidates) != 1:
                raise EvidenceError()
            path = candidates[0]
        self._validate_path(path)
        return EvidenceReference(report_id, path.name, _MEDIA_TYPES[path.suffix], path.stat().st_size)

    def read(self, reference: EvidenceReference) -> bytes:
        if not isinstance(reference, EvidenceReference):
            raise EvidenceError()
        self._validate_id(reference.report_id)
        path = self._candidate(reference.report_id, Path(reference.relative_path).suffix)
        self._validate_path(path)
        if path.name != reference.relative_path or _MEDIA_TYPES.get(path.suffix) != reference.media_type:
            raise EvidenceError()
        try:
            return path.read_bytes()
        except OSError:
            raise EvidenceError() from None

    def write_json(self, report_id: str, value: object) -> EvidenceReference:
        self._validate_id(report_id)
        if not isinstance(value, (dict, list)):
            raise EvidenceError()
        try:
            encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8") + b"\n"
        except (TypeError, ValueError):
            raise EvidenceError() from None
        destination = self._candidate(report_id, ".json")
        temporary = self._root / f".{report_id}.json.tmp"
        try:
            descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            try:
                offset = 0
                while offset < len(encoded):
                    written = os.write(descriptor, encoded[offset:])
                    if written <= 0:
                        raise OSError
                    offset += written
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, destination)
            directory = os.open(self._root, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            try:
                temporary.unlink()
            except OSError:
                pass
            raise EvidenceError() from None
        return self.get(report_id)

    def _safe_candidates(self) -> list[Path]:
        candidates: list[Path] = []
        for path in self._root.iterdir():
            if path.suffix not in _MEDIA_TYPES or not _REPORT_ID.fullmatch(path.stem):
                continue
            try:
                self._validate_path(path)
            except EvidenceError:
                continue
            candidates.append(path)
        return candidates

    def _candidate(self, report_id: str, suffix: str) -> Path:
        if suffix not in _MEDIA_TYPES:
            raise EvidenceError()
        return self._root / f"{report_id}{suffix}"

    def _validate_path(self, path: Path) -> None:
        try:
            if path.is_symlink() or path.resolve(strict=True).parent != self._root:
                raise EvidenceError()
            mode = path.stat().st_mode
        except (OSError, RuntimeError):
            raise EvidenceError() from None
        if not stat.S_ISREG(mode):
            raise EvidenceError()

    @staticmethod
    def _validate_id(report_id: object) -> None:
        if not isinstance(report_id, str) or not _REPORT_ID.fullmatch(report_id):
            raise EvidenceError()
