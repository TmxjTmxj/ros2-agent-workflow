from __future__ import annotations

import json
import math
import stat

import pytest

from agent_ros.discovery.models import Capability, DiscoveryReport
from agent_ros.profiles.models import RobotProfile
from agent_ros.safety.challenge import create_operator_challenge
from agent_ros.safety.gateway import SafetyError, SafetyGateway
from agent_ros.safety.state import SafetyState


def robot_profile(mode: str = "simulation", heartbeat_timeout: float | None = 1.0) -> RobotProfile:
    safety = {"heartbeat_timeout": heartbeat_timeout, "estop_topic": "/emergency_stop"}
    return RobotProfile.from_mapping({
        "name": "robot",
        "mode": mode,
        "namespace": "/robot",
        "frames": {"base": "base_link", "odom": "odom"},
        "adapter": {"kind": "twist"},
        "interfaces": {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        },
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": safety,
        "observation_sources": ["odometry"],
    })


def compatible_report() -> DiscoveryReport:
    return DiscoveryReport((Capability("mobile_base.twist", 1.0, ("/cmd_vel", "/odom")),))


def prepared_gateway(profile: RobotProfile, **kwargs) -> SafetyGateway:
    gateway = SafetyGateway(profile, **kwargs)
    gateway.discover(compatible_report())
    gateway.validate()
    return gateway


def test_simulation_auto_arms_after_validating_and_allows_a_bounded_task():
    gateway = prepared_gateway(robot_profile())

    assert gateway.state is SafetyState.ARMED
    gateway.start_task(linear_velocity=0.5, angular_velocity=1.0)
    assert gateway.state is SafetyState.RUNNING


@pytest.mark.parametrize("linear_velocity", [math.nan, math.inf, 0.50001])
def test_start_task_rejects_non_finite_or_out_of_profile_motion_limits(linear_velocity):
    gateway = prepared_gateway(robot_profile())

    with pytest.raises(SafetyError, match="MOTION_LIMIT"):
        gateway.start_task(linear_velocity=linear_velocity, angular_velocity=0.0)
    assert gateway.state is SafetyState.ARMED


def test_hardware_requires_an_exact_out_of_band_challenge_and_rejects_replay(tmp_path):
    profile = robot_profile("hardware")
    token = create_operator_challenge(profile.name, tmp_path, ttl_seconds=30.0)
    record = json.loads((tmp_path / "robot.challenge.json").read_text(encoding="utf-8"))
    assert record["profile"] == "robot"
    assert record["hash"] != token
    assert record["used"] is False
    assert stat.S_IMODE((tmp_path / "robot.challenge.json").stat().st_mode) == 0o600

    first = prepared_gateway(profile, runtime_dir=tmp_path)
    with pytest.raises(SafetyError, match="HARDWARE_CHALLENGE"):
        first.arm("not-the-token")
    assert first.state is SafetyState.VALIDATED

    first.arm(token)
    assert first.state is SafetyState.ARMED
    assert json.loads((tmp_path / "robot.challenge.json").read_text(encoding="utf-8"))["used"] is True

    replay = prepared_gateway(profile, runtime_dir=tmp_path)
    with pytest.raises(SafetyError, match="HARDWARE_CHALLENGE"):
        replay.arm(token)
    assert replay.state is SafetyState.VALIDATED


def test_invalid_transition_is_rejected_fail_closed():
    gateway = SafetyGateway(robot_profile())

    with pytest.raises(SafetyError, match="UNSAFE_STATE"):
        gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    assert gateway.state is SafetyState.NEW


def test_expired_heartbeat_stops_repeatedly_and_latches_fault():
    now = [0.0]
    stops: list[str] = []
    gateway = prepared_gateway(robot_profile(), clock=lambda: now[0], stop_callback=lambda: stops.append("stop"))
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    gateway.heartbeat()
    now[0] = 1.1

    with pytest.raises(SafetyError, match="HEARTBEAT_EXPIRED"):
        gateway.heartbeat()
    assert gateway.state is SafetyState.FAULTED
    assert len(stops) >= 3


def test_missing_simulation_heartbeat_configuration_faults_without_an_assertion_error():
    stops: list[str] = []
    gateway = prepared_gateway(robot_profile(heartbeat_timeout=None), stop_callback=lambda: stops.append("stop"))
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    with pytest.raises(SafetyError, match="HEARTBEAT_UNCONFIGURED"):
        gateway.heartbeat()
    assert gateway.state is SafetyState.FAULTED
    assert len(stops) >= 3


def test_estop_is_latched_and_agent_cannot_reset_hardware_estop(tmp_path):
    profile = robot_profile("hardware")
    token = create_operator_challenge(profile.name, tmp_path)
    stops: list[str] = []
    gateway = prepared_gateway(profile, runtime_dir=tmp_path, stop_callback=lambda: stops.append("stop"))
    gateway.arm(token)
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    gateway.estop()
    assert gateway.state is SafetyState.ESTOPPED
    assert len(stops) >= 3
    with pytest.raises(SafetyError, match="ESTOP_LATCHED"):
        gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    with pytest.raises(SafetyError, match="OPERATOR_REQUIRED"):
        gateway.operator_reset()
    assert gateway.state is SafetyState.ESTOPPED
