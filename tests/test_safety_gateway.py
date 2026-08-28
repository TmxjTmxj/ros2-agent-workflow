from __future__ import annotations

import json
import math
import stat
import threading
from dataclasses import FrozenInstanceError

import pytest
from agent_ros.discovery.models import Capability, DiscoveryReport
from agent_ros.profiles.models import RobotProfile
from agent_ros.safety.challenge import create_operator_challenge
from agent_ros.safety.gateway import SafetyError, SafetyGateway, SafetyTransition
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.state import SafetyState


def robot_profile(mode: str = "simulation", heartbeat_timeout: float | None = 1.0) -> RobotProfile:
    safety = {"heartbeat_timeout": heartbeat_timeout, "estop_topic": "/emergency_stop"}
    return RobotProfile.from_mapping(
        {
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
        }
    )


def compatible_report() -> DiscoveryReport:
    return DiscoveryReport(
        (Capability("mobile_base.twist", 1.0, ("/cmd_vel", "/odom")),),
        topic_types={
            "/cmd_vel": ("geometry_msgs/msg/Twist",),
            "/odom": ("nav_msgs/msg/Odometry",),
        },
    )


def prepared_gateway(profile: RobotProfile, owner, **kwargs) -> SafetyGateway:
    gateway = owner(SafetyGateway(profile, **kwargs))
    gateway.discover(compatible_report())
    gateway.validate()
    return gateway


def accepted_stop(action):
    def stop(_timeout):
        action()
        return EmergencyStopResult(True, True, True, "ESTOP_LATCHED")

    return stop


def test_gateway_recognizes_only_the_exact_transition_receipt(gateway_owner):
    gateway = gateway_owner(SafetyGateway(robot_profile()))

    transition = gateway.discover(compatible_report())
    forged = SafetyTransition(
        transition.sequence,
        transition.state_before,
        transition.state_after,
        transition.stop_result,
    )

    assert forged == transition
    assert gateway.owns_transition(transition)
    assert not gateway.owns_transition(forged)


def test_degraded_estop_is_latched_but_not_reported_as_successful(gateway_owner):
    degraded = EmergencyStopResult(
        True,
        False,
        True,
        "TRANSPORT_UNQUIESCED",
    )
    gateway = prepared_gateway(robot_profile(), gateway_owner, stop_callback=lambda _timeout: degraded)
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    transition = gateway.estop()

    assert transition is not None
    assert transition.state_after is SafetyState.ESTOPPED
    assert transition.stop_result == degraded
    assert transition.stop_result.code == "TRANSPORT_UNQUIESCED"
    assert not transition.stop_result.successful


@pytest.mark.parametrize("winner", ["a", "b"])
def test_concurrent_estop_attempts_keep_their_own_frozen_stop_result(
    winner,
    gateway_owner,
):
    degraded = EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    successful = EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
    gateway = prepared_gateway(robot_profile(), gateway_owner)
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    gateway._stop_callback = lambda _timeout: degraded if threading.current_thread().name == "attempt-a" else successful
    paused = "b" if winner == "a" else "a"
    paused_ready = threading.Event()
    release_paused = threading.Event()
    original_stop = gateway._stop_repeatedly

    def ordered_stop(timeout):
        result = original_stop(timeout)
        if threading.current_thread().name == f"attempt-{paused}":
            paused_ready.set()
            assert release_paused.wait(1.0)
        return result

    gateway._stop_repeatedly = ordered_stop
    attempts = {}
    errors = []

    def invoke(label):
        try:
            attempts[label] = gateway.estop_attempt(timeout=0.5)
        except BaseException as exc:
            errors.append(exc)

    paused_thread = threading.Thread(target=invoke, args=(paused,), name=f"attempt-{paused}")
    winner_thread = threading.Thread(target=invoke, args=(winner,), name=f"attempt-{winner}")
    paused_thread.start()
    assert paused_ready.wait(1.0)
    winner_thread.start()
    winner_thread.join(1.0)
    assert not winner_thread.is_alive()
    release_paused.set()
    paused_thread.join(1.0)

    assert not paused_thread.is_alive()
    assert errors == []
    assert attempts["a"].result == degraded
    assert attempts["b"].result == successful
    assert attempts[winner].transition is not None
    assert attempts[winner].transition.stop_result is attempts[winner].result
    assert attempts[paused].transition is None
    with pytest.raises(FrozenInstanceError):
        attempts["a"].result = successful


def test_simulation_auto_arms_after_validating_and_allows_a_bounded_task(gateway_owner):
    gateway = prepared_gateway(robot_profile(), gateway_owner)

    assert gateway.state is SafetyState.ARMED
    gateway.start_task(linear_velocity=0.5, angular_velocity=1.0)
    assert gateway.state is SafetyState.RUNNING


@pytest.mark.parametrize("linear_velocity", [math.nan, math.inf, 0.50001])
def test_start_task_rejects_non_finite_or_out_of_profile_motion_limits(linear_velocity, gateway_owner):
    gateway = prepared_gateway(robot_profile(), gateway_owner)

    with pytest.raises(SafetyError, match="MOTION_LIMIT"):
        gateway.start_task(linear_velocity=linear_velocity, angular_velocity=0.0)
    assert gateway.state is SafetyState.ARMED


def test_hardware_requires_an_exact_out_of_band_challenge_and_rejects_replay(tmp_path, gateway_owner):
    profile = robot_profile("hardware")
    token = create_operator_challenge(profile.name, tmp_path, ttl_seconds=30.0)
    record = json.loads((tmp_path / "robot.challenge.json").read_text(encoding="utf-8"))
    assert record["profile"] == "robot"
    assert record["hash"] != token
    assert record["used"] is False
    assert stat.S_IMODE((tmp_path / "robot.challenge.json").stat().st_mode) == 0o600

    first = prepared_gateway(profile, gateway_owner, runtime_dir=tmp_path)
    with pytest.raises(SafetyError, match="HARDWARE_CHALLENGE"):
        first.arm("not-the-token")
    assert first.state is SafetyState.VALIDATED

    first.arm(token)
    assert first.state is SafetyState.ARMED
    assert json.loads((tmp_path / "robot.challenge.json").read_text(encoding="utf-8"))["used"] is True

    replay = prepared_gateway(profile, gateway_owner, runtime_dir=tmp_path)
    with pytest.raises(SafetyError, match="HARDWARE_CHALLENGE"):
        replay.arm(token)
    assert replay.state is SafetyState.VALIDATED


def test_invalid_transition_is_rejected_fail_closed(gateway_owner):
    gateway = gateway_owner(SafetyGateway(robot_profile()))

    with pytest.raises(SafetyError, match="UNSAFE_STATE"):
        gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    assert gateway.state is SafetyState.NEW


def test_expired_heartbeat_stops_repeatedly_and_latches_fault(gateway_owner):
    now = [0.0]
    stops: list[str] = []
    gateway = prepared_gateway(
        robot_profile(),
        gateway_owner,
        clock=lambda: now[0],
        stop_callback=accepted_stop(lambda: stops.append("stop")),
    )
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    gateway.heartbeat()
    now[0] = 1.1

    with pytest.raises(SafetyError, match="HEARTBEAT_EXPIRED"):
        gateway.heartbeat()
    assert gateway.state is SafetyState.FAULTED
    assert len(stops) >= 3


def test_independent_watchdog_faults_after_deadline_without_an_agent_heartbeat_call(gateway_owner):
    now = [0.0]
    stops: list[str] = []
    gateway = prepared_gateway(
        robot_profile(),
        gateway_owner,
        clock=lambda: now[0],
        stop_callback=accepted_stop(lambda: stops.append("stop")),
    )
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)
    now[0] = 1.1

    assert gateway.supervisor.evaluate() is True
    assert gateway.state is SafetyState.FAULTED
    assert len(stops) >= 3
    gateway.close()
    assert gateway.supervisor.running is False


def test_watchdog_worker_polls_and_faults_without_manual_evaluation_or_heartbeat(gateway_owner):
    stopped = threading.Event()
    gateway = prepared_gateway(
        robot_profile(heartbeat_timeout=0.01),
        gateway_owner,
        stop_callback=accepted_stop(stopped.set),
        supervisor_poll_interval=0.001,
    )
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    assert stopped.wait(timeout=1.0)
    assert gateway.state is SafetyState.FAULTED
    gateway.close()


def test_missing_simulation_heartbeat_configuration_faults_without_an_assertion_error(gateway_owner):
    stops: list[str] = []
    gateway = prepared_gateway(
        robot_profile(heartbeat_timeout=None),
        gateway_owner,
        stop_callback=accepted_stop(lambda: stops.append("stop")),
    )
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    with pytest.raises(SafetyError, match="HEARTBEAT_UNCONFIGURED"):
        gateway.heartbeat()
    assert gateway.state is SafetyState.FAULTED
    assert len(stops) >= 3


def test_estop_is_latched_and_agent_cannot_reset_hardware_estop(tmp_path, gateway_owner):
    profile = robot_profile("hardware")
    token = create_operator_challenge(profile.name, tmp_path)
    stops: list[str] = []
    gateway = prepared_gateway(
        profile,
        gateway_owner,
        runtime_dir=tmp_path,
        stop_callback=accepted_stop(lambda: stops.append("stop")),
    )
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


def test_hardware_challenge_is_rejected_after_a_boot_identity_change(tmp_path, gateway_owner):
    profile = robot_profile("hardware")
    now = [100.0]
    token = create_operator_challenge(
        profile.name, tmp_path, monotonic_clock=lambda: now[0], boot_id=lambda: "boot-before"
    )
    gateway = prepared_gateway(
        profile,
        gateway_owner,
        runtime_dir=tmp_path,
        clock=lambda: now[0],
        boot_id=lambda: "boot-after",
    )

    with pytest.raises(SafetyError, match="HARDWARE_CHALLENGE"):
        gateway.arm(token)
    assert gateway.state is SafetyState.VALIDATED


@pytest.mark.parametrize(
    "report",
    [
        DiscoveryReport(
            (Capability("mobile_base.twist", 1.0, ("/cmd_vel", "/odom")),),
            topic_types={
                "/cmd_vel": ("wrong/msg/Twist",),
                "/odom": ("nav_msgs/msg/Odometry",),
            },
        ),
        DiscoveryReport(
            (Capability("mobile_base.twist", 1.0, ("/cmd_vel", "/odom")),),
            topic_types={
                "/wrong_cmd_vel": ("geometry_msgs/msg/Twist",),
                "/odom": ("nav_msgs/msg/Odometry",),
            },
        ),
    ],
)
def test_validation_rejects_capability_claims_without_profile_endpoint_and_type_evidence(report, gateway_owner):
    gateway = gateway_owner(SafetyGateway(robot_profile()))
    gateway.discover(report)

    with pytest.raises(SafetyError, match="PROFILE_UNSUPPORTED"):
        gateway.validate()
    assert gateway.state is SafetyState.DISCOVERED


def test_physical_estop_monitor_hook_latches_and_stops_immediately(gateway_owner):
    stops: list[str] = []
    gateway = prepared_gateway(
        robot_profile(),
        gateway_owner,
        stop_callback=accepted_stop(lambda: stops.append("stop")),
    )
    gateway.start_task(linear_velocity=0.1, angular_velocity=0.0)

    gateway.observe_physical_estop(True)

    assert gateway.state is SafetyState.ESTOPPED
    assert len(stops) >= 3
    with pytest.raises(SafetyError, match="ESTOP_LATCHED"):
        gateway.heartbeat()
