import json
import math
from pathlib import Path

import pytest

from smartcar_bringup.controller_core import (
    ControllerConfig,
    MissionControllerCore,
    MissionState,
    Pose2D,
    RouteValidationError,
    load_route,
    normalize_angle,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def write_route(tmp_path: Path, *, stages=None) -> Path:
    if stages is None:
        stages = [
            {
                "id": "pharmacy",
                "name": "走廊到药房取药",
                "endpoint": [2.0, 0.0],
                "waypoints": [[1.0, 0.0], [2.0, 0.0]],
            },
            {
                "id": "ward2",
                "name": "药房到病房2送药",
                "endpoint": [2.0, 2.0],
                "waypoints": [[2.0, 1.0], [2.0, 2.0]],
            },
            {
                "id": "laboratory",
                "name": "病房2到实验室巡视",
                "endpoint": [0.0, 2.0],
                "waypoints": [[1.0, 2.0], [0.0, 2.0]],
            },
        ]
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"start": [0.0, 0.0, 0.0], "stages": stages}))
    return path


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (0.0, 0.0),
        (3.0 * math.pi, math.pi),
        (-3.0 * math.pi, -math.pi),
        (5.0 * math.pi / 2.0, math.pi / 2.0),
    ],
)
def test_normalize_angle_wraps_multi_turn_inputs(raw, expected):
    """Catches angle errors that choose the long direction at ±pi."""
    assert normalize_angle(raw) == pytest.approx(expected)


def test_project_route_has_three_required_stages_and_exact_endpoints():
    """Catches omission or reordering of the agreed delivery stages."""
    route = load_route(PROJECT_ROOT / "config" / "mission_routes.json")

    assert [stage.id for stage in route.stages] == [
        "pharmacy",
        "ward2",
        "laboratory",
    ]
    assert [(stage.endpoint.x, stage.endpoint.y) for stage in route.stages] == [
        (18.5, 3.5),
        (20.0, 8.5),
        (12.0, 14.5),
    ]


def test_load_route_rejects_non_finite_waypoint(tmp_path):
    """Catches NaN coordinates reaching the live controller."""
    stages = [
        {"id": "a", "name": "a", "endpoint": [1, 0], "waypoints": [[1, 0]]},
        {"id": "b", "name": "b", "endpoint": [2, 0], "waypoints": [[2, 0]]},
        {
            "id": "c",
            "name": "c",
            "endpoint": [3, 0],
            "waypoints": [[float("nan"), 0]],
        },
    ]

    with pytest.raises(RouteValidationError, match="finite"):
        load_route(write_route(tmp_path, stages=stages))


def test_load_route_requires_final_waypoint_to_equal_endpoint(tmp_path):
    stages = [
        {"id": "a", "name": "a", "endpoint": [1, 0], "waypoints": [[0.75, 0]]},
        {"id": "b", "name": "b", "endpoint": [2, 0], "waypoints": [[2, 0]]},
        {"id": "c", "name": "c", "endpoint": [3, 0], "waypoints": [[3, 0]]},
    ]

    with pytest.raises(RouteValidationError, match="match endpoint"):
        load_route(write_route(tmp_path, stages=stages))


def test_large_heading_error_rotates_in_place_with_correct_sign(tmp_path):
    """Catches reversed steering and unsafe forward motion while misaligned."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=0.0).accepted

    command = core.update(Pose2D(0.0, 0.0, -math.pi / 2.0), now=0.1)

    assert command.linear == 0.0
    assert command.angular == pytest.approx(1.6)


def test_small_heading_error_drives_and_slows_near_waypoint(tmp_path):
    """Catches full-speed overshoot close to a waypoint."""
    config = ControllerConfig(waypoint_tolerance=0.1)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted

    far = core.update(Pose2D(0.0, 0.0, 0.0), now=0.1)
    near = core.update(Pose2D(0.75, 0.0, 0.0), now=0.2)

    assert far.linear == pytest.approx(1.2)
    assert near.linear == pytest.approx(0.375)
    assert far.angular == near.angular == 0.0


def test_update_skips_waypoint_already_inside_tolerance(tmp_path):
    """Catches a controller needlessly circling an already reached waypoint."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=0.0).accepted

    command = core.update(Pose2D(1.0, 0.0, 0.0), now=0.1)
    status = core.status()

    assert status["stage_id"] == "pharmacy"
    assert status["waypoint_index"] == 1
    assert command.linear > 0.0
    assert command.angular == 0.0


def test_terminal_state_never_commands_motion(tmp_path):
    """Catches velocity leaking after cancellation."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=0.0).accepted
    core.cancel()

    command = core.update(Pose2D(0.0, 0.0, 0.0), now=0.1)

    assert core.state is MissionState.CANCELLED
    assert command.linear == command.angular == 0.0


def test_repeated_start_is_rejected_without_resetting_progress(tmp_path):
    """Catches duplicate Agent calls silently restarting a live mission."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=1.0).accepted
    core.update(Pose2D(1.0, 0.0, 0.0), now=2.0)

    result = core.start(now=3.0)

    assert not result.accepted
    assert core.status()["waypoint_index"] == 1
    assert core.started_at == 1.0


def test_estop_latches_until_explicit_reset(tmp_path):
    """Catches a new mission bypassing an emergency-stop latch."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=0.0).accepted

    assert core.estop().accepted
    assert core.state is MissionState.ESTOPPED
    assert not core.start(now=1.0).accepted
    assert core.update(Pose2D(0.0, 0.0, 0.0), now=1.1).linear == 0.0
    assert core.reset().accepted
    assert core.state is MissionState.IDLE
    assert core.start(now=2.0).accepted


def test_stale_odometry_fails_running_mission(tmp_path):
    """Catches continued motion after position feedback disappears."""
    config = ControllerConfig(odom_timeout=0.5, progress_timeout=20.0)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted
    assert core.update(Pose2D(0.0, 0.0, 0.0), now=0.1).linear > 0.0

    command = core.update(None, now=0.61)

    assert core.state is MissionState.FAILED
    assert core.failure_code == "ODOM_STALE"
    assert command.linear == command.angular == 0.0


def test_cached_pose_does_not_disguise_stale_odometry(tmp_path):
    """Catches a ROS timer refreshing odometry age without a new message."""
    config = ControllerConfig(odom_timeout=0.5, progress_timeout=20.0)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted

    core.update(
        Pose2D(0.0, 0.0, 0.0),
        now=0.1,
        odom_received_at=0.1,
    )
    command = core.update(
        Pose2D(0.0, 0.0, 0.0),
        now=0.61,
        odom_received_at=0.1,
    )

    assert core.state is MissionState.FAILED
    assert core.failure_code == "ODOM_STALE"
    assert command.linear == command.angular == 0.0


def test_mission_timeout_fails_at_180_seconds(tmp_path):
    """Catches missions running past the agreed competition limit."""
    core = MissionControllerCore(load_route(write_route(tmp_path)))
    assert core.start(now=10.0).accepted

    command = core.update(Pose2D(0.0, 0.0, 0.0), now=190.01)

    assert core.state is MissionState.FAILED
    assert core.failure_code == "MISSION_TIMEOUT"
    assert command.linear == command.angular == 0.0


def test_obstacle_stops_immediately_then_fails_after_recovery_window(tmp_path):
    """Catches driving into an obstacle or waiting indefinitely in its path."""
    config = ControllerConfig(obstacle_fail_after=2.0, progress_timeout=20.0)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted

    stopped = core.update(Pose2D(0.0, 0.0, 0.0), now=0.1, front_range=0.2)
    failed = core.update(Pose2D(0.0, 0.0, 0.0), now=2.11, front_range=0.2)

    assert stopped.linear == stopped.angular == 0.0
    assert failed.linear == failed.angular == 0.0
    assert core.state is MissionState.FAILED
    assert core.failure_code == "OBSTACLE_BLOCKED"


def test_no_progress_fails_with_target_identity(tmp_path):
    """Catches a wedged robot burning the whole mission timeout."""
    config = ControllerConfig(progress_timeout=2.0)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted
    core.update(Pose2D(0.0, 0.0, 0.0), now=0.1)

    command = core.update(Pose2D(0.0, 0.0, 0.0), now=2.11)

    assert core.state is MissionState.FAILED
    assert core.failure_code == "WAYPOINT_NO_PROGRESS"
    assert core.status()["failed_stage_id"] == "pharmacy"
    assert core.status()["failed_waypoint_index"] == 0
    assert command.linear == command.angular == 0.0


def test_heading_improvement_counts_as_progress_while_rotating_in_place(tmp_path):
    """A healthy in-place turn must not trip the translation-only watchdog."""
    config = ControllerConfig(progress_timeout=1.0, progress_heading_epsilon=0.05)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted

    for yaw, now in [(1.5, 0.1), (1.0, 0.8), (0.5, 1.5), (0.2, 2.2)]:
        command = core.update(Pose2D(0.0, 0.0, yaw), now=now)
        assert core.state is MissionState.RUNNING
        assert command.angular < 0.0


def test_all_stage_endpoints_record_metrics_and_finish_in_order(tmp_path):
    """Catches skipping or reordering one of the three delivery stages."""
    config = ControllerConfig(waypoint_tolerance=0.05, progress_timeout=20.0)
    core = MissionControllerCore(load_route(write_route(tmp_path)), config=config)
    assert core.start(now=0.0).accepted

    samples = [
        (Pose2D(1.0, 0.0, 0.0), 1.0),
        (Pose2D(2.0, 0.0, 0.0), 2.0),
        (Pose2D(2.0, 1.0, math.pi / 2.0), 3.0),
        (Pose2D(2.0, 2.0, math.pi / 2.0), 4.0),
        (Pose2D(1.0, 2.0, math.pi), 5.0),
        (Pose2D(0.0, 2.0, math.pi), 6.0),
    ]
    for pose, now in samples:
        command = core.update(pose, now=now)

    status = core.status()
    assert core.state is MissionState.SUCCEEDED
    assert command.linear == command.angular == 0.0
    assert [result["id"] for result in status["stage_results"]] == [
        "pharmacy",
        "ward2",
        "laboratory",
    ]
    assert [result["endpoint_error"] for result in status["stage_results"]] == [
        pytest.approx(0.0),
        pytest.approx(0.0),
        pytest.approx(0.0),
    ]
    assert status["elapsed"] == pytest.approx(6.0)
