"""Capture and validate immutable terminal evidence before adapter cleanup."""

from __future__ import annotations

from collections.abc import Mapping

from agent_ros.adapters.base import AdapterError, AdapterStatus, Observation, RobotAdapter
from agent_ros.runtime.errors import RuntimeControllerError


def capture_terminal_evidence(
    adapter: RobotAdapter,
    sources: tuple[str, ...],
    terminal_status: AdapterStatus | None,
) -> Mapping[str, Observation]:
    """Return a validated terminal snapshot without changing runtime state."""
    try:
        frozen = adapter.freeze_terminal_evidence(sources, terminal_status)
    except AdapterError as exc:
        raise RuntimeControllerError(exc.code) from None
    except Exception:
        raise RuntimeControllerError("EVIDENCE_INVALID") from None
    if not isinstance(frozen, Mapping):
        raise RuntimeControllerError("EVIDENCE_INVALID")
    snapshot: dict[str, Observation] = {}
    for source in sources:
        observation = frozen.get(source)
        if (
            not isinstance(observation, Observation)
            or observation.source != source
            or not isinstance(observation.values, Mapping)
        ):
            raise RuntimeControllerError("EVIDENCE_INVALID")
        snapshot[source] = observation
    return snapshot
