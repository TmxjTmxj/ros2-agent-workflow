"""Stable public runtime errors shared by orchestration boundaries."""

from __future__ import annotations

_PUBLIC_CODES = frozenset(
    {
        "UNSAFE_STATE",
        "PROFILE_INVALID",
        "CONTROLLER_CONFLICT",
        "STALE_FEEDBACK",
        "TIMEOUT",
        "EVIDENCE_INVALID",
        "AUDIT_INTEGRITY_COMPROMISED",
        "ESTOP_LATCHED",
        "OPERATOR_REQUIRED",
        "CLEANUP_FAILED",
    }
)


class RuntimeControllerError(RuntimeError):
    """A stable public code with no underlying ROS, process, or path details."""

    def __init__(self, code: str) -> None:
        self.code = code if code in _PUBLIC_CODES else "UNSAFE_STATE"
        super().__init__(self.code)
