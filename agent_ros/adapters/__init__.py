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
    TwistCommand,
    create_adapter,
)
from agent_ros.adapters.hospital import HospitalDeliveryAdapter, HospitalSimulationRuntime
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter

__all__ = (
    "AdapterError",
    "AdapterProbe",
    "AdapterStatus",
    "HospitalAction",
    "HospitalDeliveryAdapter",
    "HospitalSimulationRuntime",
    "Nav2Adapter",
    "NavigationGoal",
    "Observation",
    "OdometrySample",
    "RobotAdapter",
    "TwistAdapter",
    "TwistCommand",
    "create_adapter",
)
