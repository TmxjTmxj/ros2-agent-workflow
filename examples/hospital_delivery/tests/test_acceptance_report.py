from copy import deepcopy
import json

import cv2
import numpy as np
import pytest

from scripts import verify_acceptance
from scripts.verify_acceptance import is_prohibited_robot_contact, validate_acceptance_report


@pytest.fixture
def valid_report(tmp_path):
    camera = tmp_path / "camera.png"
    initial_camera = tmp_path / "initial-camera.png"
    assert cv2.imwrite(str(camera), np.zeros((4, 6, 3), dtype=np.uint8))
    assert cv2.imwrite(str(initial_camera), np.ones((4, 6, 3), dtype=np.uint8))
    return {
        "schema_version": 2,
        "metric_source": "odometry",
        "mission_state": "SUCCEEDED",
        "failure_code": None,
        "stages": [
            {"id": "pharmacy", "endpoint_error": 0.31},
            {"id": "ward2", "endpoint_error": 0.22},
            {"id": "laboratory", "endpoint_error": 0.28},
        ],
        "elapsed_seconds": 126.0,
        "stopped_drift_m": 0.004,
        "cmd_vel_publishers": ["/hospital_mission_controller"],
        "unknown_publishers": [],
        "camera": {
            "path": str(camera),
            "bytes": camera.stat().st_size,
            "width": 6,
            "height": 4,
        },
        "initial_camera": {
            "path": str(initial_camera),
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
        },
        "publisher_evidence": {
            "monitor_started_unix": 1.0,
            "monitor_stopped_unix": 130.0,
            "samples": [
                {
                    "at_unix": 1.0 + index * 0.5,
                    "endpoints": [
                        {"node": "/hospital_mission_controller", "gid": "controller-gid"}
                    ],
                }
                for index in range(259)
            ],
            "inspection_errors": [],
            "observed_endpoints": [
                {"node": "/hospital_mission_controller", "gid": "controller-gid"}
            ],
        },
        "contact_evidence": {
            "topic": "/hospital_amr/contacts",
            "monitor_started_unix": 1.0,
            "monitor_stopped_unix": 130.0,
            "topic_publishers_seen": True,
            "messages": 100,
            "prohibited_contacts": [],
        },
        "safety": {"collision_free": True, "safety_stop_failure": False},
    }


def test_valid_report_passes_and_sets_boolean(valid_report):
    errors = validate_acceptance_report(valid_report)
    assert errors == []
    assert valid_report["passed"] is True
    assert valid_report["validation_errors"] == []


def test_forged_success_status_cannot_replace_independent_odometry_metrics():
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
        {"at_monotonic": 10.0, "x": 0.0, "y": 0.0},
        {"at_monotonic": 11.0, "x": 0.1, "y": 0.0},
        {"at_monotonic": 14.0, "x": 0.1, "y": 0.0},
    ]

    metrics = derive(route, samples, mission_started_monotonic=10.0, terminal_monotonic=11.0)

    assert metrics["metric_source"] == "odometry"
    assert [stage["reached"] for stage in metrics["stages"]] == [False, False, False]
    assert metrics["stages"][0]["endpoint_error"] == pytest.approx(0.9)
    assert metrics["stopped_drift_m"] == pytest.approx(0.0)


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
            lambda report: report["publisher_evidence"]["samples"][4]["endpoints"][0].update(
                gid="replacement-gid"
            ),
            "publisher GID",
        ),
        (
            lambda report: report["contact_evidence"]["prohibited_contacts"].append(
                {"collision": "wall"}
            ),
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
            lambda report: report["publisher_evidence"]["samples"][2].update(
                at_unix=10.0
            ),
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
    camera = valid_report["camera"]["path"]
    __import__("pathlib").Path(camera).unlink()
    errors = validate_acceptance_report(valid_report)
    assert any("camera" in error for error in errors)


def test_missing_or_fake_initial_camera_fails(valid_report):
    camera = valid_report["initial_camera"]["path"]
    __import__("pathlib").Path(camera).unlink()
    errors = validate_acceptance_report(valid_report)
    assert any("initial camera" in error for error in errors)


def test_png_signature_without_decodable_pixels_fails(valid_report):
    camera = __import__("pathlib").Path(valid_report["camera"]["path"])
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
def test_acceptance_numbers_are_finite_nonnegative_and_forward_moving(
    valid_report, mutation, expected
):
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
