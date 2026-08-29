#!/usr/bin/env python3
"""Validate a staged hospital demonstration evidence bundle without rewriting it."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

REQUIRED_FILES = (
    "acceptance_report.json",
    "mcp_agent_trace.json",
    "acceptance-initial.png",
    "acceptance-final.png",
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON in {path.name}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required in {path.name}")
    return value


def _validate_success_report(report: dict[str, Any]) -> list[str]:
    """Check exported report claims without resolving paths outside the bundle."""
    errors: list[str] = []
    if report.get("schema_version") != 2:
        errors.append("acceptance report is not schema version 2")
    if report.get("mission_state") != "SUCCEEDED" or report.get("failure_code") is not None:
        errors.append("acceptance report does not record a successful mission")
    if report.get("validation_errors") != []:
        errors.append("acceptance report does not record an empty validation_errors list")
    for field, expected_name in (
        ("initial_camera", "acceptance-initial.png"),
        ("camera", "acceptance-final.png"),
    ):
        camera = report.get(field)
        if not isinstance(camera, dict) or Path(str(camera.get("path", ""))).name != expected_name:
            errors.append(f"acceptance report does not reference {expected_name}")
    return errors


def verify_release_artifacts(directory: str | Path) -> int:
    """Return zero only for a complete, independently valid success evidence bundle."""
    root = Path(directory)
    errors: list[str] = []
    required = {name: root / name for name in REQUIRED_FILES}
    for name, path in required.items():
        if not path.is_file():
            errors.append(f"missing required artifact: {name}")

    acceptance = required["acceptance_report.json"]
    if acceptance.is_file():
        try:
            errors.extend(_validate_success_report(_load_json(acceptance)))
        except ValueError as exc:
            errors.append(str(exc))

    trace = required["mcp_agent_trace.json"]
    if trace.is_file():
        try:
            trace_payload = _load_json(trace)
            if trace_payload.get("schema_version") != 1 or trace_payload.get("result") != "SUCCEEDED":
                errors.append("MCP trace does not record a schema-1 SUCCEEDED result")
        except ValueError as exc:
            errors.append(str(exc))

    for name in ("acceptance-initial.png", "acceptance-final.png"):
        image = required[name]
        if image.is_file():
            try:
                if image.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE:
                    errors.append(f"invalid PNG evidence: {name}")
            except OSError as exc:
                errors.append(f"cannot read PNG evidence {name}: {exc}")

    print(json.dumps({"ok": not errors, "errors": errors}, ensure_ascii=False, allow_nan=False))
    return 0 if not errors else 2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="staged evidence directory")
    return verify_release_artifacts(parser.parse_args(argv).directory)


if __name__ == "__main__":
    raise SystemExit(main())
