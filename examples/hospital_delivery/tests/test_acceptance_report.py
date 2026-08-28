import ast
import json
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import pytest
from scripts import verify_acceptance
from scripts.verify_acceptance import is_prohibited_robot_contact, validate_acceptance_report


@pytest.fixture
def valid_report(tmp_path, monkeypatch):
    monkeypatch.setattr(verify_acceptance, "PROJECT_ROOT", tmp_path)
    camera = tmp_path / "camera.png"
    initial_camera = tmp_path / "initial-camera.png"
    assert cv2.imwrite(str(camera), np.zeros((4, 6, 3), dtype=np.uint8))
    assert cv2.imwrite(str(initial_camera), np.ones((4, 6, 3), dtype=np.uint8))
    return {
        "schema_version": 2,
        "metric_source": "diff_drive_odometry",
        "time_domain": "ros_sim_time",
        "mission_state": "SUCCEEDED",
        "failure_code": None,
        "stages": [
            {"id": "pharmacy", "endpoint_error": 0.31, "reached": True, "elapsed": 30.0},
            {"id": "ward2", "endpoint_error": 0.22, "reached": True, "elapsed": 70.0},
            {"id": "laboratory", "endpoint_error": 0.28, "reached": True, "elapsed": 126.0},
        ],
        "elapsed_seconds": 126.0,
        "wall_elapsed_seconds": 189.0,
        "real_time_factor": 2.0 / 3.0,
        "stopped_drift_m": 0.004,
        "cmd_vel_publishers": ["/hospital_mission_controller"],
        "unknown_publishers": [],
        "camera": {
            "path": camera.name,
            "bytes": camera.stat().st_size,
            "width": 6,
            "height": 4,
        },
        "initial_camera": {
            "path": initial_camera.name,
            "bytes": initial_camera.stat().st_size,
            "width": 6,
            "height": 4,
        },
        "odometry_evidence": {
            "topic": "/odom",
            "sample_count": 260,
            "monitor_started_monotonic": 1.0,
            "mission_started_monotonic": 2.0,
            "terminal_monotonic": 128.0,
            "monitor_stopped_monotonic": 131.0,
            "post_terminal_sample_count": 75,
            "monitor_started_sim_time": 9.0,
            "mission_started_sim_time": 10.0,
            "terminal_sim_time": 136.0,
            "monitor_stopped_sim_time": 139.0,
            "monitor_started_wall_monotonic": 1.0,
            "mission_started_wall_monotonic": 2.0,
            "terminal_wall_monotonic": 191.0,
            "monitor_stopped_wall_monotonic": 195.0,
            "initial_odom_pose": {"x": 0.01, "y": -0.01, "yaw": 0.0},
        },
        "route_frame": "world",
        "publisher_evidence": {
            "monitor_started_unix": 1.0,
            "monitor_stopped_unix": 194.0,
            "samples": [
                {
                    "at_unix": 1.0 + index * 0.5,
                    "endpoints": [{"node": "/hospital_mission_controller", "gid": "controller-gid"}],
                }
                for index in range(387)
            ],
            "inspection_errors": [],
            "observed_endpoints": [{"node": "/hospital_mission_controller", "gid": "controller-gid"}],
        },
        "contact_evidence": {
            "topic": "/hospital_amr/contacts",
            "monitor_started_unix": 1.0,
            "monitor_stopped_unix": 194.0,
            "topic_publishers_seen": True,
            "messages": 100,
            "prohibited_contacts": [],
            "configured_sources": verify_acceptance.load_contact_sources(),
            "publisher_samples": [
                {
                    "at_unix": 1.0 + index * 0.5,
                    "endpoints": [{"node": "/hospital_ros_gz_bridge", "gid": f"contact-gid-{gid}"} for gid in range(5)],
                }
                for index in range(387)
            ],
        },
        "safety": {"collision_free": True, "safety_stop_failure": False},
    }


def test_valid_report_passes_and_sets_boolean(valid_report):
    errors = validate_acceptance_report(valid_report)
    assert errors == []
    assert valid_report["passed"] is True
    assert valid_report["validation_errors"] == []


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        (lambda report: report.update(time_domain="wall_monotonic"), "time domain"),
        (lambda report: report.update(wall_elapsed_seconds=-1.0), "wall elapsed"),
        (lambda report: report.update(real_time_factor=float("nan")), "real time factor"),
        (
            lambda report: report["odometry_evidence"].update(monitor_stopped_sim_time=137.0),
            "odometry sim timestamps",
        ),
    ],
)
def test_report_requires_sim_time_domain_and_post_terminal_coverage(valid_report, mutation, expected):
    mutation(valid_report)

    errors = validate_acceptance_report(valid_report)

    assert any(expected in error for error in errors)


def test_each_stage_delta_must_fit_its_sixty_second_budget(valid_report):
    for stage, elapsed in zip(valid_report["stages"], [30.0, 91.0, 126.0], strict=False):
        stage["elapsed"] = elapsed

    errors = validate_acceptance_report(valid_report)

    assert any("stage duration" in error for error in errors)


def test_forged_success_status_cannot_replace_independent_odometry_metrics(valid_report):
    derive = getattr(verify_acceptance, "derive_odom_metrics", None)
    assert callable(derive)
    route = {
        "stages": [
            {"id": "pharmacy", "endpoint": [1.0, 0.0]},
            {"id": "ward2", "endpoint": [2.0, 0.0]},
            {"id": "laboratory", "endpoint": [3.0, 0.0]},
        ]
    }
    samples = [
        {"at_sim_time": 10.0, "at_wall_monotonic": 10.0, "x": 0.0, "y": 0.0},
        {"at_sim_time": 11.0, "at_wall_monotonic": 11.0, "x": 0.1, "y": 0.0},
        {"at_sim_time": 14.0, "at_wall_monotonic": 14.0, "x": 0.1, "y": 0.0},
    ]

    metrics = derive(
        route,
        samples,
        mission_started_sim_time=10.0,
        terminal_sim_time=11.0,
        mission_started_wall_monotonic=10.0,
        terminal_wall_monotonic=11.0,
    )

    assert metrics["metric_source"] == "diff_drive_odometry"
    assert metrics["route_frame"] == "world"
    assert [stage["reached"] for stage in metrics["stages"]] == [False, False, False]
    assert metrics["stages"][0]["endpoint_error"] == pytest.approx(0.9)
    assert metrics["stopped_drift_m"] == pytest.approx(0.0)
    forged = deepcopy(valid_report)
    forged.update(metrics)
    forged["mission_state"] = "SUCCEEDED"
    forged["failure_code"] = None

    errors = validate_acceptance_report(forged)

    assert forged["passed"] is False
    assert any("was not reached" in error for error in errors)


def test_verifier_does_not_import_the_production_route_or_transform():
    source = Path(verify_acceptance.__file__).read_text(encoding="utf-8")
    imported_modules = {node.module for node in ast.walk(ast.parse(source)) if isinstance(node, ast.ImportFrom)}

    assert "smartcar_bringup.controller_core" not in imported_modules


def test_independent_route_parser_rejects_endpoint_waypoint_mutation(tmp_path):
    route = json.loads(verify_acceptance.ROUTE_PATH.read_text(encoding="utf-8"))
    route["stages"][1]["endpoint"][0] += 1.0
    path = tmp_path / "mutated-route.json"
    path.write_text(json.dumps(route), encoding="utf-8")

    with pytest.raises(ValueError, match="final waypoint"):
        verify_acceptance.load_acceptance_route(path)


def test_independent_transform_cross_checks_rotated_world_route():
    route = {
        "start": [12.0, 8.0, np.pi / 2.0],
        "stages": [
            {"id": "pharmacy", "endpoint": [12.0, 9.0]},
            {"id": "ward2", "endpoint": [12.0, 10.0]},
            {"id": "laboratory", "endpoint": [12.0, 11.0]},
        ],
    }
    metrics = verify_acceptance.derive_odom_metrics(
        route,
        [
            {"at_sim_time": 1.0, "at_wall_monotonic": 1.0, "x": 0.0, "y": 0.0},
            {"at_sim_time": 2.0, "at_wall_monotonic": 2.0, "x": 1.0, "y": 0.0},
            {"at_sim_time": 3.0, "at_wall_monotonic": 3.0, "x": 2.0, "y": 0.0},
            {"at_sim_time": 4.0, "at_wall_monotonic": 4.0, "x": 3.0, "y": 0.0},
            {"at_sim_time": 7.0, "at_wall_monotonic": 7.0, "x": 3.0, "y": 0.0},
        ],
        mission_started_sim_time=1.0,
        terminal_sim_time=4.0,
        mission_started_wall_monotonic=1.0,
        terminal_wall_monotonic=4.0,
        tolerance=0.01,
    )

    assert [stage["reached"] for stage in metrics["stages"]] == [True, True, True]
    assert [stage["endpoint_error"] for stage in metrics["stages"]] == pytest.approx([0.0, 0.0, 0.0])


def test_odom_metrics_use_sim_stamps_and_report_wall_clock_rtf():
    route = {
        "stages": [
            {"id": "pharmacy", "endpoint": [1.0, 0.0]},
            {"id": "ward2", "endpoint": [2.0, 0.0]},
            {"id": "laboratory", "endpoint": [3.0, 0.0]},
        ]
    }
    samples = [
        {"at_sim_time": 10.0, "at_wall_monotonic": 20.0, "x": 0.0, "y": 0.0},
        {"at_sim_time": 40.0, "at_wall_monotonic": 65.0, "x": 1.0, "y": 0.0},
        {"at_sim_time": 80.0, "at_wall_monotonic": 125.0, "x": 2.0, "y": 0.0},
        {"at_sim_time": 136.0, "at_wall_monotonic": 209.0, "x": 3.0, "y": 0.0},
        {"at_sim_time": 139.0, "at_wall_monotonic": 213.5, "x": 3.0, "y": 0.0},
    ]

    metrics = verify_acceptance.derive_odom_metrics(
        route,
        samples,
        mission_started_sim_time=10.0,
        terminal_sim_time=136.0,
        mission_started_wall_monotonic=20.0,
        terminal_wall_monotonic=209.0,
        tolerance=0.01,
    )

    assert metrics["time_domain"] == "ros_sim_time"
    assert [stage["elapsed"] for stage in metrics["stages"]] == pytest.approx([30.0, 70.0, 126.0])
    assert metrics["elapsed_seconds"] == pytest.approx(126.0)
    assert metrics["wall_elapsed_seconds"] == pytest.approx(189.0)
    assert metrics["real_time_factor"] == pytest.approx(2.0 / 3.0)


@pytest.mark.parametrize(
    "samples",
    [
        [
            {"at_sim_time": 10.0, "at_wall_monotonic": 20.0, "x": 0.0, "y": 0.0},
            {"at_sim_time": 9.0, "at_wall_monotonic": 21.0, "x": 0.1, "y": 0.0},
        ],
        [
            {"at_sim_time": 0.0, "at_wall_monotonic": 20.0, "x": 0.0, "y": 0.0},
            {"at_sim_time": 1.0, "at_wall_monotonic": 21.0, "x": 0.1, "y": 0.0},
        ],
        [
            {"at_sim_time": "bad", "at_wall_monotonic": 20.0, "x": 0.0, "y": 0.0},
            {"at_sim_time": 11.0, "at_wall_monotonic": 21.0, "x": 0.1, "y": 0.0},
        ],
    ],
)
def test_odom_metrics_reject_invalid_sim_stamps(samples):
    route = {
        "stages": [
            {"id": "pharmacy", "endpoint": [1.0, 0.0]},
            {"id": "ward2", "endpoint": [2.0, 0.0]},
            {"id": "laboratory", "endpoint": [3.0, 0.0]},
        ]
    }

    with pytest.raises(ValueError, match="sim time"):
        verify_acceptance.derive_odom_metrics(
            route,
            samples,
            mission_started_sim_time=10.0,
            terminal_sim_time=11.0,
            mission_started_wall_monotonic=20.0,
            terminal_wall_monotonic=21.0,
        )


@pytest.mark.parametrize(
    ("sec", "nanosec"),
    [(0, 0), (-1, 0), (1, -1), (1, 1_000_000_000), (True, 0), ("1", 0)],
)
def test_ros_stamp_parser_rejects_zero_and_malformed_values(sec, nanosec):
    stamp = type("Stamp", (), {"sec": sec, "nanosec": nanosec})()

    with pytest.raises(ValueError, match="stamp"):
        verify_acceptance.ros_stamp_seconds(stamp)


def test_ros_stamp_parser_converts_valid_stamp_exactly():
    stamp = type("Stamp", (), {"sec": 12, "nanosec": 500_000_000})()

    assert verify_acceptance.ros_stamp_seconds(stamp) == pytest.approx(12.5)


@pytest.mark.parametrize(
    ("first", "second", "prohibited"),
    [
        ("hospital_amr::wheel_left_collision", "ground_plane::collision", False),
        ("hospital_amr::wheel_left_collision", "outer_wall::collision", True),
        ("chair::collision", "hospital_amr::caster_collision", True),
        ("hospital_amr::wheel_left_collision", "hospital_amr::base_chassis", False),
        ("chair::collision", "wall::collision", False),
    ],
)
def test_contact_filter_covers_all_robot_parts(first, second, prohibited):
    assert is_prohibited_robot_contact(first, second) is prohibited


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        (lambda report: report["stages"][1].update(endpoint_error=0.51), "endpoint error"),
        (lambda report: report.update(elapsed_seconds=180.01), "elapsed"),
        (lambda report: report.update(stopped_drift_m=0.021), "drift"),
        (
            lambda report: report["unknown_publishers"].append("/forgotten_cli"),
            "unknown /cmd_vel",
        ),
        (
            lambda report: report["publisher_evidence"]["inspection_errors"].append("boom"),
            "publisher inspection",
        ),
        (
            lambda report: report["publisher_evidence"]["samples"][0]["endpoints"].append(
                {"node": "/late_cli", "gid": "late-gid"}
            ),
            "publisher sample",
        ),
        (
            lambda report: report["publisher_evidence"]["samples"][4]["endpoints"][0].update(gid="replacement-gid"),
            "publisher GID",
        ),
        (
            lambda report: report["contact_evidence"]["prohibited_contacts"].append({"collision": "wall"}),
            "prohibited contact",
        ),
        (
            lambda report: report["contact_evidence"].update(messages=0),
            "contact messages",
        ),
        (lambda report: report.update(mission_state="FAILED"), "mission state"),
        (
            lambda report: report["safety"].update(collision_free=False),
            "collision",
        ),
        (
            lambda report: report["safety"].update(safety_stop_failure=True),
            "safety-stop",
        ),
    ],
)
def test_each_acceptance_invariant_fails_independently(valid_report, mutation, expected):
    report = deepcopy(valid_report)
    mutation(report)

    errors = validate_acceptance_report(report)

    assert report["passed"] is False
    assert any(expected in error for error in errors)


def test_stage_order_and_count_are_strict(valid_report):
    valid_report["stages"] = list(reversed(valid_report["stages"]))
    errors = validate_acceptance_report(valid_report)
    assert any("stage order" in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        (lambda report: report.update(schema_version=1), "schema version"),
        (
            lambda report: report["publisher_evidence"]["samples"].__setitem__(
                slice(None), report["publisher_evidence"]["samples"][:2]
            ),
            "coverage",
        ),
        (
            lambda report: report["publisher_evidence"]["samples"][2].update(at_unix=10.0),
            "ordered",
        ),
        (
            lambda report: report["contact_evidence"].update(monitor_stopped_unix=10.0),
            "contact monitoring",
        ),
    ],
)
def test_evidence_must_cover_the_full_mission_and_stop_window(valid_report, mutation, expected):
    mutation(valid_report)

    errors = validate_acceptance_report(valid_report)

    assert any(expected in error for error in errors)


def test_missing_or_fake_camera_fails(valid_report):
    camera = verify_acceptance.PROJECT_ROOT / valid_report["camera"]["path"]
    camera.unlink()
    errors = validate_acceptance_report(valid_report)
    assert any("camera" in error for error in errors)


def test_missing_or_fake_initial_camera_fails(valid_report):
    camera = verify_acceptance.PROJECT_ROOT / valid_report["initial_camera"]["path"]
    camera.unlink()
    errors = validate_acceptance_report(valid_report)
    assert any("initial camera" in error for error in errors)


def test_png_signature_without_decodable_pixels_fails(valid_report):
    camera = verify_acceptance.PROJECT_ROOT / valid_report["camera"]["path"]
    camera.write_bytes(b"\x89PNG\r\n\x1a\nnot-an-image")
    valid_report["camera"]["bytes"] = camera.stat().st_size

    errors = validate_acceptance_report(valid_report)

    assert any("camera" in error for error in errors)


def test_validate_mode_rejects_report_whose_saved_passed_flag_lies(tmp_path, valid_report):
    valid_report["passed"] = False
    path = tmp_path / "report.json"
    path.write_text(json.dumps(valid_report))

    errors = validate_acceptance_report(json.loads(path.read_text()))

    assert errors == []


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        (lambda report: report.update(elapsed_seconds=-1.0), "elapsed"),
        (lambda report: report.update(stopped_drift_m=-0.1), "drift"),
        (lambda report: report["stages"][0].update(endpoint_error=-0.1), "endpoint error"),
        (lambda report: report.update(elapsed_seconds=float("nan")), "elapsed"),
        (
            lambda report: report["publisher_evidence"].update(monitor_stopped_unix=-1.0),
            "timestamps",
        ),
    ],
)
def test_acceptance_numbers_are_finite_nonnegative_and_forward_moving(valid_report, mutation, expected):
    mutation(valid_report)
    errors = validate_acceptance_report(valid_report)
    assert any(expected in error for error in errors)


def test_validate_mode_rejects_nonstandard_json_nan(tmp_path, valid_report):
    path = tmp_path / "report.json"
    path.write_text(json.dumps(valid_report).replace("126.0", "NaN"), encoding="utf-8")
    load_strict = getattr(verify_acceptance, "load_strict_json", None)
    assert callable(load_strict)

    with pytest.raises(ValueError, match="non-finite"):
        load_strict(path)

    assert verify_acceptance.main(["--validate", str(path)]) == 1


def test_terminal_odometry_requires_a_real_post_stop_window(valid_report):
    valid_report["odometry_evidence"]["monitor_stopped_sim_time"] = 137.0
    valid_report["odometry_evidence"]["post_terminal_sample_count"] = 0

    errors = validate_acceptance_report(valid_report)

    assert any("odometry sim timestamps" in error for error in errors)


def test_all_five_contact_bridge_publishers_must_remain_live(valid_report):
    valid_report["contact_evidence"]["publisher_samples"][100]["endpoints"].pop()

    errors = validate_acceptance_report(valid_report)

    assert any("five configured contact publishers" in error for error in errors)


@pytest.mark.parametrize("elapsed_values", ([70.0, 30.0, 126.0], [30.0, 70.0, 127.0]))
def test_stage_entry_times_are_ordered_and_within_terminal_elapsed(valid_report, elapsed_values):
    for stage, elapsed in zip(valid_report["stages"], elapsed_values, strict=False):
        stage["elapsed"] = elapsed

    errors = validate_acceptance_report(valid_report)

    assert any("stage elapsed order" in error for error in errors)


def test_unobserved_later_stage_serializes_as_standard_json_null(tmp_path):
    route = {
        "stages": [
            {"id": "pharmacy", "endpoint": [1.0, 0.0]},
            {"id": "ward2", "endpoint": [2.0, 0.0]},
            {"id": "laboratory", "endpoint": [3.0, 0.0]},
        ]
    }
    metrics = verify_acceptance.derive_odom_metrics(
        route,
        [
            {"at_sim_time": 10.0, "at_wall_monotonic": 10.0, "x": 0.0, "y": 0.0},
            {"at_sim_time": 11.0, "at_wall_monotonic": 11.0, "x": 1.0, "y": 0.0},
        ],
        mission_started_sim_time=10.0,
        terminal_sim_time=11.0,
        mission_started_wall_monotonic=10.0,
        terminal_wall_monotonic=11.0,
    )
    path = tmp_path / "failed-report.json"

    assert metrics["stages"][1]["endpoint_error"] is None
    verify_acceptance._write_json_atomic(path, metrics)
    assert verify_acceptance.load_strict_json(path) == metrics


@pytest.mark.parametrize(
    "mutation",
    [
        lambda report: report["publisher_evidence"]["samples"].__setitem__(0, None),
        lambda report: report["publisher_evidence"]["samples"][0].update(endpoints=[None]),
        lambda report: report["contact_evidence"]["publisher_samples"][0].update(
            endpoints=[None, None, None, None, None]
        ),
    ],
)
def test_validate_cli_converts_malformed_nested_json_to_failure_without_traceback(
    tmp_path, valid_report, mutation, capsys
):
    mutation(valid_report)
    path = tmp_path / "malformed.json"
    path.write_text(json.dumps(valid_report), encoding="utf-8")

    assert verify_acceptance.main(["--validate", str(path)]) == 1
    captured = capsys.readouterr()
    assert "Traceback" not in captured.err
    saved = verify_acceptance.load_strict_json(path)
    assert saved["schema_version"] == 2
    assert saved["passed"] is False
    assert saved.get("failure_code") in {None, "REPORT_MALFORMED"}


def test_generation_failure_atomically_replaces_stale_pass_report(tmp_path, valid_report, monkeypatch, capsys):
    path = tmp_path / "acceptance.json"
    path.write_text(json.dumps(valid_report), encoding="utf-8")
    monkeypatch.setattr(
        verify_acceptance,
        "generate_acceptance_report",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("private failure text")),
    )

    assert verify_acceptance.main(["--output", str(path)]) == 1
    captured = capsys.readouterr()
    assert "Traceback" not in captured.err
    saved = verify_acceptance.load_strict_json(path)
    assert saved["schema_version"] == 2
    assert saved["passed"] is False
    assert saved["failure_code"] == "ACCEPTANCE_GENERATION_FAILED"
    assert "private failure text" not in path.read_text(encoding="utf-8")
