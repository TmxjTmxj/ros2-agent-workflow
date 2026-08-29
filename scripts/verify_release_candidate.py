#!/usr/bin/env python3
"""Validate local release distributions and their SHA-256 manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
import tomllib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_NAME = "SHA256SUMS.txt"


def _project_version() -> str:
    payload = tomllib.loads((PROJECT_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    return str(payload["project"]["version"])


def _expected_artifacts(version: str) -> set[str]:
    return {
        f"agent_ros-{version}-py3-none-any.whl",
        f"agent_ros-{version}.tar.gz",
    }


def _distribution_artifacts(directory: Path) -> set[str]:
    return {path.name for path in directory.glob("*.whl")} | {path.name for path in directory.glob("*.tar.gz")}


def _manifest_entries(path: Path) -> tuple[dict[str, str], list[str]]:
    entries: dict[str, str] = {}
    errors: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as exc:
        return {}, [f"cannot read SHA-256 manifest: {exc}"]

    for line_number, line in enumerate(lines, start=1):
        fields = line.split(maxsplit=1)
        if len(fields) != 2 or len(fields[0]) != 64 or any(char not in "0123456789abcdef" for char in fields[0]):
            errors.append(f"invalid SHA-256 manifest entry on line {line_number}")
            continue
        digest, name = fields
        if name in entries:
            errors.append(f"duplicate SHA-256 manifest entry: {name}")
            continue
        entries[name] = digest
    return entries, errors


def verify_release_candidate(directory: str | Path) -> int:
    """Return zero only for the exact wheel/sdist pair and matching manifest."""
    root = Path(directory)
    errors: list[str] = []
    if not root.is_dir():
        errors.append(f"distribution directory does not exist: {root}")
    else:
        expected = _expected_artifacts(_project_version())
        actual = _distribution_artifacts(root)
        for name in sorted(expected - actual):
            errors.append(f"missing expected distribution artifact: {name}")
        for name in sorted(actual - expected):
            errors.append(f"unexpected distribution artifact: {name}")

        manifest = root / MANIFEST_NAME
        if not manifest.is_file():
            errors.append(f"missing SHA-256 manifest: {MANIFEST_NAME}")
        else:
            entries, manifest_errors = _manifest_entries(manifest)
            errors.extend(manifest_errors)
            for name in sorted(expected):
                if name not in entries:
                    errors.append(f"missing SHA-256 manifest entry: {name}")
                    continue
                artifact = root / name
                if artifact.is_file():
                    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
                    if entries[name] != digest:
                        errors.append(f"SHA-256 mismatch: {name}")
            for name in sorted(set(entries) - expected):
                errors.append(f"unexpected SHA-256 manifest entry: {name}")

    print(json.dumps({"ok": not errors, "errors": errors}, ensure_ascii=False, allow_nan=False))
    return 0 if not errors else 2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist-dir", type=Path, default=Path("dist"), help="directory containing built distributions")
    return verify_release_candidate(parser.parse_args(argv).dist_dir)


if __name__ == "__main__":
    raise SystemExit(main())
