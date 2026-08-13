"""Stable errors exposed by the control-plane boundary."""


class ProfileValidationError(ValueError):
    """Raised when a declarative robot or task profile is unsafe or invalid."""


class DiscoveryError(RuntimeError):
    """Raised when the local ROS graph cannot be safely inspected."""
