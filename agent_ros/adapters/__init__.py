"""Typed robot adapters; no raw ROS or process authority crosses this boundary."""

from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    HospitalAction,
    NavigationGoal,
    Observation,
    OdometrySample,
    RobotAdapter,
    SafetyToken,
    TwistCommand,
    create_adapter,
)
from agent_ros.adapters.hospital import HospitalDeliveryAdapter
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter

__all__ = (
    "AdapterError",
    "AdapterProbe",
    "AdapterStatus",
    "HospitalAction",
    "HospitalDeliveryAdapter",
    "Nav2Adapter",
    "NavigationGoal",
    "Observation",
    "OdometrySample",
    "RobotAdapter",
    "SafetyToken",
    "TwistAdapter",
    "TwistCommand",
    "create_adapter",
)
