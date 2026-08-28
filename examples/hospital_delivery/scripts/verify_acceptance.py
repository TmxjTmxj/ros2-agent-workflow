#!/usr/bin/env python3
"""Generate or validate real hospital-delivery acceptance evidence."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = PROJECT_ROOT / "logs" / "acceptance_report.json"
EXPECTED_STAGE_IDS = ["pharmacy", "ward2", "laboratory"]
ALLOWED_CMD_VEL_PUBLISHERS = {"/hospital_mission_controller"}
ROUTE_PATH = PROJECT_ROOT / "config" / "mission_routes.json"
BRIDGE_PATH = PROJECT_ROOT / "config" / "ros_gz_bridge.yaml"


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    return converted if math.isfinite(converted) else None


def ros_stamp_seconds(stamp: Any) -> float:
    """Strictly convert a non-zero builtin_interfaces/Time stamp."""
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if (
        isinstance(sec, bool)
        or not isinstance(sec, int)
        or isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
        or sec < 0
        or nanosec < 0
        or nanosec >= 1_000_000_000
        or (sec == 0 and nanosec == 0)
    ):
        raise ValueError("odometry stamp is malformed or zero")
    return sec + nanosec / 1_000_000_000.0


def is_prohibited_robot_contact(first: str, second: str) -> bool:
    """Accept support/internal contacts and reject robot-to-environment contacts."""
    names = {first, second}
    robot_names = [name for name in names if "hospital_amr" in name]
    if len(robot_names) != 1:
        return False
    return not any("ground_plane" in name for name in names)


def _valid_png(camera: Any) -> bool:
    if not isinstance(camera, dict) or not camera.get("path"):
        return False
    try:
        relative = Path(str(camera["path"]))
        if relative.is_absolute() or ".." in relative.parts:
            return False
        path = PROJECT_ROOT / relative
        if (
            not path.is_file()
            or path.suffix.lower() != ".png"
            or path.stat().st_size != int(camera.get("bytes", -1))
            or path.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n"
        ):
            return False
        import cv2

        decoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        return (
            decoded is not None
            and decoded.size > 0
            and decoded.shape[1] == int(camera.get("width", -1))
            and decoded.shape[0] == int(camera.get("height", -1))
        )
    except (OSError, TypeError, ValueError, IndexError):
        return False


def load_strict_json(path: Path) -> dict[str, Any]:
    """Load one standards-compliant JSON object and reject NaN/Infinity."""

    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON number: {value}")

    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError("JSON root must be an object")
    return value


def load_acceptance_route(path: Path = ROUTE_PATH) -> dict[str, Any]:
    """Independently parse the fixed world-frame route used for acceptance."""
    route = load_strict_json(path)
    start = route.get("start")
    if not isinstance(start, list) or len(start) != 3 or any(_number(value) is None for value in start):
        raise ValueError("route start must be three finite numbers")
    stages = route.get("stages")
    if not isinstance(stages, list) or len(stages) != len(EXPECTED_STAGE_IDS):
        raise ValueError("route must contain exactly three stages")
    if [stage.get("id") for stage in stages if isinstance(stage, dict)] != EXPECTED_STAGE_IDS:
        raise ValueError("route stage order is invalid")
    for stage in stages:
        endpoint = stage.get("endpoint")
        waypoints = stage.get("waypoints")
        if (
            not isinstance(endpoint, list)
            or len(endpoint) != 2
            or any(_number(value) is None for value in endpoint)
            or not isinstance(waypoints, list)
            or not waypoints
        ):
            raise ValueError("route endpoint and waypoints must be finite coordinates")
        for waypoint in waypoints:
            if (
                not isinstance(waypoint, list)
                or len(waypoint) != 2
                or any(_number(value) is None for value in waypoint)
            ):
                raise ValueError("route waypoints must be finite coordinates")
        if any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(endpoint, waypoints[-1], strict=False)):
            raise ValueError("route final waypoint must match endpoint")
    return route


def load_contact_sources(path: Path = BRIDGE_PATH) -> list[str]:
    try:
        bridges = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load contact bridge config: {exc}") from exc
    if not isinstance(bridges, list):
        raise ValueError("bridge config must be a list")
    sources = [
        bridge.get("gz_topic_name")
        for bridge in bridges
        if isinstance(bridge, dict)
        and bridge.get("ros_topic_name") == "/hospital_amr/contacts"
        and bridge.get("direction") == "GZ_TO_ROS"
    ]
    if (
        len(sources) != 5
        or any(not isinstance(source, str) or not source for source in sources)
        or len(set(sources)) != 5
    ):
        raise ValueError("bridge config must contain five distinct contact sources")
    return sources


def _world_xy_to_odom(start: list[Any], world_x: float, world_y: float) -> tuple[float, float]:
    start_x, start_y, start_yaw = (float(value) for value in start)
    dx = world_x - start_x
    dy = world_y - start_y
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def derive_odom_metrics(
    route: Any,
    samples: list[dict[str, Any]],
    *,
    mission_started_sim_time: float,
    terminal_sim_time: float,
    mission_started_wall_monotonic: float,
    terminal_wall_monotonic: float,
    tolerance: float = 0.50,
) -> dict[str, Any]:
    """Derive metrics from independent DiffDrive poses and ROS simulation stamps."""
    start = route.get("start", [0.0, 0.0, 0.0])
    stage_rows = [
        (
            stage["id"],
            stage.get("name", stage["id"]),
            float(stage["endpoint"][0]),
            float(stage["endpoint"][1]),
        )
        for stage in route["stages"]
    ]
    if len(stage_rows) != 3:
        raise ValueError("route must contain exactly three stages")
    clock_values = (
        mission_started_sim_time,
        terminal_sim_time,
        mission_started_wall_monotonic,
        terminal_wall_monotonic,
    )
    if any(_number(value) is None for value in clock_values):
        raise ValueError("sim time and wall time bounds must be finite")
    if (
        mission_started_sim_time <= 0.0
        or terminal_sim_time <= mission_started_sim_time
        or mission_started_wall_monotonic < 0.0
        or terminal_wall_monotonic <= mission_started_wall_monotonic
    ):
        raise ValueError("sim time and wall time bounds are invalid")
    all_ordered: list[dict[str, float]] = []
    try:
        for sample in samples:
            row = {
                "at_sim_time": float(sample["at_sim_time"]),
                "at_wall_monotonic": float(sample["at_wall_monotonic"]),
                "x": float(sample["x"]),
                "y": float(sample["y"]),
                "yaw": float(sample.get("yaw", 0.0)),
            }
            if any(not math.isfinite(value) for value in row.values()):
                raise ValueError
            all_ordered.append(row)
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError("odometry sim time samples are malformed") from None
    sim_times = [sample["at_sim_time"] for sample in all_ordered]
    wall_times = [sample["at_wall_monotonic"] for sample in all_ordered]
    if (
        not sim_times
        or any(value <= 0.0 for value in sim_times)
        or any(second <= first for first, second in zip(sim_times, sim_times[1:], strict=False))
        or any(value < 0.0 for value in wall_times)
        or any(second < first for first, second in zip(wall_times, wall_times[1:], strict=False))
    ):
        raise ValueError("odometry sim time samples must be positive and monotonic")
    ordered = [sample for sample in all_ordered if sample["at_sim_time"] >= mission_started_sim_time]
    if not ordered:
        raise ValueError("no odometry samples after mission start")

    stage_metrics: list[dict[str, Any]] = []
    cursor = 0
    terminal_limit = [
        (index, sample) for index, sample in enumerate(ordered) if sample["at_sim_time"] <= terminal_sim_time
    ]
    for stage_id, name, endpoint_x, endpoint_y in stage_rows:
        odom_x, odom_y = _world_xy_to_odom(start, endpoint_x, endpoint_y)
        candidates = terminal_limit[cursor:]
        distances = [math.hypot(sample["x"] - odom_x, sample["y"] - odom_y) for _, sample in candidates]
        reached_offset = next(
            (offset for offset, distance in enumerate(distances) if distance <= tolerance),
            None,
        )
        reached = reached_offset is not None
        if reached:
            absolute_index, entered = candidates[reached_offset]
            endpoint_error = distances[reached_offset]
            cursor = absolute_index + 1
            entered_at = entered["at_sim_time"]
        else:
            endpoint_error = min(distances) if distances else None
            entered_at = None
        stage_metrics.append(
            {
                "id": stage_id,
                "name": name,
                "reached": reached,
                "endpoint_error": endpoint_error,
                "elapsed": (None if entered_at is None else entered_at - mission_started_sim_time),
            }
        )

    terminal_samples = [sample for sample in ordered if sample["at_sim_time"] <= terminal_sim_time]
    terminal_pose = terminal_samples[-1] if terminal_samples else ordered[0]
    stopped_pose = all_ordered[-1]
    stopped_drift = math.hypot(
        stopped_pose["x"] - terminal_pose["x"],
        stopped_pose["y"] - terminal_pose["y"],
    )
    pose_fields = lambda sample: {
        "x": sample["x"],
        "y": sample["y"],
        "yaw": sample["yaw"],
    }
    sim_elapsed = terminal_sim_time - mission_started_sim_time
    wall_elapsed = terminal_wall_monotonic - mission_started_wall_monotonic
    return {
        "metric_source": "diff_drive_odometry",
        "route_frame": "world",
        "time_domain": "ros_sim_time",
        "stages": stage_metrics,
        "elapsed_seconds": sim_elapsed,
        "wall_elapsed_seconds": wall_elapsed,
        "real_time_factor": sim_elapsed / wall_elapsed,
        "terminal_pose": pose_fields(terminal_pose),
        "stopped_pose": pose_fields(stopped_pose),
        "stopped_drift_m": stopped_drift,
        "odometry_evidence": {
            "topic": "/odom",
            "sample_count": len(all_ordered),
            "monitor_started_sim_time": all_ordered[0]["at_sim_time"],
            "mission_started_sim_time": mission_started_sim_time,
            "terminal_sim_time": terminal_sim_time,
            "monitor_stopped_sim_time": all_ordered[-1]["at_sim_time"],
            "monitor_started_wall_monotonic": all_ordered[0]["at_wall_monotonic"],
            "mission_started_wall_monotonic": mission_started_wall_monotonic,
            "terminal_wall_monotonic": terminal_wall_monotonic,
            "monitor_stopped_wall_monotonic": all_ordered[-1]["at_wall_monotonic"],
            "post_terminal_sample_count": sum(sample["at_sim_time"] > terminal_sim_time for sample in all_ordered),
            "initial_odom_pose": pose_fields(all_ordered[0]),
        },
    }


def validate_acceptance_report(report: dict[str, Any]) -> list[str]:
    """Validate every hard acceptance invariant and update pass fields."""
    errors: list[str] = []
    if report.get("schema_version") != 2:
        errors.append(f"schema version is {report.get('schema_version')!r}, expected 2")
    if report.get("mission_state") != "SUCCEEDED":
        errors.append(f"mission state is {report.get('mission_state')!r}, expected SUCCEEDED")
    if report.get("failure_code") is not None:
        errors.append(f"mission failure code is {report.get('failure_code')!r}")
    if report.get("metric_source") != "diff_drive_odometry":
        errors.append("metric source must be independent DiffDrive odometry")
    if report.get("route_frame") != "world":
        errors.append("route frame must be world")
    if report.get("time_domain") != "ros_sim_time":
        errors.append("time domain must be ros_sim_time")

    stages = report.get("stages")
    stage_elapsed_values: list[float] = []
    if (
        not isinstance(stages, list)
        or [item.get("id") for item in stages if isinstance(item, dict)] != EXPECTED_STAGE_IDS
    ):
        errors.append("stage order must be pharmacy, ward2, laboratory exactly once")
    else:
        for stage in stages:
            error = _number(stage.get("endpoint_error"))
            stage_elapsed = _number(stage.get("elapsed"))
            if stage.get("reached") is not True:
                errors.append(f"stage {stage.get('id')} was not reached in odometry")
            if error is None or error < 0.0 or error > 0.50:
                errors.append(f"stage {stage.get('id')} endpoint error {stage.get('endpoint_error')!r} exceeds 0.50 m")
            if stage_elapsed is None or stage_elapsed <= 0.0:
                errors.append(f"stage {stage.get('id')} elapsed time is not positive")
            else:
                stage_elapsed_values.append(stage_elapsed)

    elapsed = _number(report.get("elapsed_seconds"))
    if elapsed is None or elapsed <= 0.0 or elapsed > 180.0:
        errors.append(f"elapsed time {report.get('elapsed_seconds')!r} exceeds 180 s")
    if len(stage_elapsed_values) == len(EXPECTED_STAGE_IDS) and (
        stage_elapsed_values != sorted(stage_elapsed_values)
        or len(set(stage_elapsed_values)) != len(stage_elapsed_values)
        or elapsed is None
        or stage_elapsed_values[-1] > elapsed
    ):
        errors.append("stage elapsed order is invalid or exceeds terminal elapsed time")
    if len(stage_elapsed_values) == len(EXPECTED_STAGE_IDS):
        stage_durations = [
            stage_elapsed_values[0],
            *(second - first for first, second in zip(stage_elapsed_values, stage_elapsed_values[1:], strict=False)),
        ]
        if any(duration <= 0.0 or duration > 60.0 for duration in stage_durations):
            errors.append("stage duration exceeds its 60 s simulation-time budget")
    wall_elapsed = _number(report.get("wall_elapsed_seconds"))
    real_time_factor = _number(report.get("real_time_factor"))
    if wall_elapsed is None or wall_elapsed <= 0.0 or wall_elapsed > 300.0:
        errors.append("wall elapsed time is invalid or exceeds 300 s")
    if (
        real_time_factor is None
        or real_time_factor <= 0.0
        or elapsed is None
        or wall_elapsed is None
        or wall_elapsed <= 0.0
        or not math.isclose(real_time_factor, elapsed / wall_elapsed, rel_tol=1e-6)
    ):
        errors.append("real time factor is invalid")
    drift = _number(report.get("stopped_drift_m"))
    if drift is None or drift < 0.0 or drift > 0.02:
        errors.append(f"stopped drift {report.get('stopped_drift_m')!r} exceeds 0.02 m")

    odometry = report.get("odometry_evidence")
    if not isinstance(odometry, dict):
        errors.append("independent odometry evidence is missing")
    else:
        monitor_started = _number(odometry.get("monitor_started_sim_time"))
        mission_started = _number(odometry.get("mission_started_sim_time"))
        terminal_at = _number(odometry.get("terminal_sim_time"))
        monitor_stopped = _number(odometry.get("monitor_stopped_sim_time"))
        wall_monitor_started = _number(odometry.get("monitor_started_wall_monotonic"))
        wall_mission_started = _number(odometry.get("mission_started_wall_monotonic"))
        wall_terminal = _number(odometry.get("terminal_wall_monotonic"))
        wall_monitor_stopped = _number(odometry.get("monitor_stopped_wall_monotonic"))
        sample_count = odometry.get("sample_count")
        post_terminal_count = odometry.get("post_terminal_sample_count")
        initial = odometry.get("initial_odom_pose")
        if (
            odometry.get("topic") != "/odom"
            or isinstance(sample_count, bool)
            or not isinstance(sample_count, int)
            or sample_count <= 0
            or isinstance(post_terminal_count, bool)
            or not isinstance(post_terminal_count, int)
            or post_terminal_count <= 0
            or None in {monitor_started, mission_started, terminal_at, monitor_stopped}
            or not (monitor_started <= mission_started < terminal_at <= monitor_stopped)
            or monitor_stopped - terminal_at < 2.5
        ):
            errors.append("odometry sim timestamps and sample summary are invalid")
        if None in {
            wall_monitor_started,
            wall_mission_started,
            wall_terminal,
            wall_monitor_stopped,
        } or not (wall_monitor_started <= wall_mission_started < wall_terminal <= wall_monitor_stopped):
            errors.append("odometry wall timestamps are invalid")
        if not isinstance(initial, dict):
            errors.append("initial odometry pose is missing")
        else:
            initial_x = _number(initial.get("x"))
            initial_y = _number(initial.get("y"))
            initial_yaw = _number(initial.get("yaw"))
            if (
                initial_x is None
                or initial_y is None
                or initial_yaw is None
                or math.hypot(initial_x, initial_y) > 0.25
                or abs(initial_yaw) > 0.25
            ):
                errors.append("initial DiffDrive odometry is not spawn-relative")

    unknown = report.get("unknown_publishers")
    if not isinstance(unknown, list) or unknown:
        errors.append(f"unknown /cmd_vel publisher(s): {unknown!r}")
    publishers = report.get("cmd_vel_publishers")
    if not isinstance(publishers, list) or "/hospital_mission_controller" not in publishers:
        errors.append("mission controller is missing from /cmd_vel publishers")

    publisher_evidence = report.get("publisher_evidence")
    if not isinstance(publisher_evidence, dict):
        errors.append("publisher inspection evidence is missing")
    else:
        inspection_errors = publisher_evidence.get("inspection_errors")
        if not isinstance(inspection_errors, list) or inspection_errors:
            errors.append(f"publisher inspection failed: {inspection_errors!r}")
        samples = publisher_evidence.get("samples")
        if not isinstance(samples, list) or not samples:
            errors.append("publisher inspection samples are missing")
        else:
            started = _number(publisher_evidence.get("monitor_started_unix"))
            stopped = _number(publisher_evidence.get("monitor_stopped_unix"))
            times = [_number(sample.get("at_unix")) if isinstance(sample, dict) else None for sample in samples]
            if (
                started is None
                or stopped is None
                or stopped < started
                or any(value is None for value in times)
                or times != sorted(times)
            ):
                errors.append("publisher monitoring timestamps are not finite and ordered")
            else:
                numeric_times = [float(value) for value in times]
                gaps = [second - first for first, second in zip(numeric_times, numeric_times[1:], strict=False)]
                required_duration = (wall_elapsed or 0.0) + 2.5
                if (
                    numeric_times[0] - started > 1.0
                    or stopped - numeric_times[-1] > 1.0
                    or (gaps and max(gaps) > 1.0)
                    or stopped - started < required_duration
                ):
                    errors.append("publisher monitoring coverage is incomplete")
            for sample in samples:
                endpoints = sample.get("endpoints") if isinstance(sample, dict) else None
                endpoint = endpoints[0] if isinstance(endpoints, list) and endpoints else None
                if (
                    not isinstance(endpoints, list)
                    or len(endpoints) != 1
                    or not isinstance(endpoint, dict)
                    or endpoint.get("node") != "/hospital_mission_controller"
                    or not endpoint.get("gid")
                ):
                    errors.append(f"publisher sample is not exclusive: {sample!r}")
                    break
            gids: set[Any] = set()
            for sample in samples:
                if not isinstance(sample, dict):
                    continue
                endpoints = sample.get("endpoints")
                if not isinstance(endpoints, list):
                    continue
                gids.update(
                    endpoint.get("gid") for endpoint in endpoints if isinstance(endpoint, dict) and endpoint.get("gid")
                )
            if len(gids) != 1:
                errors.append("publisher GID changed during monitoring")

    if not _valid_png(report.get("camera")):
        errors.append("camera evidence is missing or is not a valid PNG")
    if not _valid_png(report.get("initial_camera")):
        errors.append("initial camera evidence is missing or is not a valid PNG")
    safety = report.get("safety")
    contact_evidence = report.get("contact_evidence")
    if not isinstance(contact_evidence, dict):
        errors.append("contact evidence is missing")
    else:
        contact_started = _number(contact_evidence.get("monitor_started_unix"))
        contact_stopped = _number(contact_evidence.get("monitor_stopped_unix"))
        publisher_started = (
            _number(publisher_evidence.get("monitor_started_unix")) if isinstance(publisher_evidence, dict) else None
        )
        publisher_stopped = (
            _number(publisher_evidence.get("monitor_stopped_unix")) if isinstance(publisher_evidence, dict) else None
        )
        if (
            contact_started is None
            or contact_stopped is None
            or contact_stopped < contact_started
            or contact_stopped - contact_started < (wall_elapsed or 0.0) + 2.5
            or publisher_started is None
            or publisher_stopped is None
            or abs(contact_started - publisher_started) > 0.1
            or abs(contact_stopped - publisher_stopped) > 0.1
        ):
            errors.append("contact monitoring coverage is incomplete")
        if contact_evidence.get("topic_publishers_seen") is not True:
            errors.append("contact monitor topic was not active")
        contact_messages = contact_evidence.get("messages")
        if isinstance(contact_messages, bool) or not isinstance(contact_messages, int) or contact_messages <= 0:
            errors.append("contact messages were not observed")
        contacts = contact_evidence.get("prohibited_contacts")
        if not isinstance(contacts, list) or contacts:
            errors.append(f"prohibited contact detected: {contacts!r}")
        try:
            expected_contact_sources = load_contact_sources()
        except ValueError as exc:
            errors.append(str(exc))
            expected_contact_sources = []
        configured_sources = contact_evidence.get("configured_sources")
        contact_publisher_samples = contact_evidence.get("publisher_samples")
        if configured_sources != expected_contact_sources:
            errors.append("contact evidence does not identify all five configured sources")
        if not isinstance(contact_publisher_samples, list) or not contact_publisher_samples:
            errors.append("five configured contact publishers were not monitored")
        else:
            expected_gids: set[str] | None = None
            contact_times: list[float] = []
            for sample in contact_publisher_samples:
                at_unix = _number(sample.get("at_unix")) if isinstance(sample, dict) else None
                endpoints = sample.get("endpoints") if isinstance(sample, dict) else None
                if at_unix is None:
                    errors.append("contact publisher timestamps are invalid")
                    break
                contact_times.append(at_unix)
                if not isinstance(endpoints, list) or len(endpoints) != len(expected_contact_sources):
                    errors.append("five configured contact publishers were not live throughout")
                    break
                gids = {
                    endpoint.get("gid")
                    for endpoint in endpoints
                    if isinstance(endpoint, dict)
                    and endpoint.get("node") == "/hospital_ros_gz_bridge"
                    and isinstance(endpoint.get("gid"), str)
                    and endpoint.get("gid")
                }
                if len(gids) != len(expected_contact_sources):
                    errors.append("five configured contact publishers were not live throughout")
                    break
                if expected_gids is None:
                    expected_gids = gids
                elif gids != expected_gids:
                    errors.append("contact publisher identities changed during monitoring")
                    break
            if contact_times:
                gaps = [second - first for first, second in zip(contact_times, contact_times[1:], strict=False)]
                if (
                    contact_times != sorted(contact_times)
                    or contact_started is None
                    or contact_stopped is None
                    or contact_times[0] - contact_started > 1.0
                    or contact_stopped - contact_times[-1] > 1.0
                    or (gaps and max(gaps) > 1.0)
                ):
                    errors.append("contact publisher monitoring coverage is incomplete")
    if not isinstance(safety, dict) or safety.get("collision_free") is not True:
        errors.append("collision-free outcome was not demonstrated")
    if not isinstance(safety, dict) or safety.get("safety_stop_failure") is not False:
        errors.append("safety-stop failure occurred or was not reported")

    report["validation_errors"] = errors
    report["passed"] = not errors
    return errors


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix="acceptance-", suffix=".json", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _failure_report(code: str) -> dict[str, Any]:
    return {
        "schema_version": 2,
        "generated_at_unix": time.time(),
        "mission_state": "FAILED",
        "failure_code": code,
        "validation_errors": [code],
        "passed": False,
    }


def _distance(first: dict[str, Any], second: dict[str, Any]) -> float:
    return math.hypot(float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"]))


def generate_acceptance_report(timeout: float = 300.0, output: Path = DEFAULT_REPORT) -> dict[str, Any]:
    """Arm evidence first, start one mission, then save the complete run."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    try:
        from scripts.capture_camera import capture_one_frame
    except ModuleNotFoundError:
        from capture_camera import capture_one_frame

    output = output.resolve()
    camera_dir = PROJECT_ROOT / "logs"
    timestamp = int(time.time())
    initial_camera = capture_one_frame(camera_dir / f"acceptance-initial-{timestamp}.png", timeout=20.0)

    context = Context()
    rclpy.init(context=context)
    node = Node("codex_acceptance_monitor", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latest: list[dict[str, Any]] = []
    odom_samples: list[dict[str, Any]] = []
    odom_stamp_errors: list[str] = []
    publisher_samples: list[dict[str, Any]] = []
    publisher_inspection_errors: list[str] = []
    contact_messages = 0
    prohibited_contacts: list[dict[str, Any]] = []
    contact_topic_publishers_seen = False
    contact_publisher_samples: list[dict[str, Any]] = []
    configured_contact_sources = load_contact_sources()
    monitor_started_unix = time.time()

    def graph_endpoints(topic: str) -> list[dict[str, str]]:
        infos = node.get_publishers_info_by_topic(topic)
        endpoints = []
        for info in infos:
            namespace = "/" + info.node_namespace.strip("/") if info.node_namespace.strip("/") else ""
            name = f"{namespace}/{info.node_name}".replace("//", "/")
            endpoints.append({"node": name, "gid": bytes(info.endpoint_gid).hex()})
        return endpoints

    def sample_publishers() -> None:
        nonlocal contact_topic_publishers_seen
        try:
            endpoints = graph_endpoints("/cmd_vel")
            publisher_samples.append({"at_unix": time.time(), "endpoints": endpoints})
            contact_endpoints = graph_endpoints("/hospital_amr/contacts")
            contact_publisher_samples.append({"at_unix": time.time(), "endpoints": contact_endpoints})
            contact_topic_publishers_seen = contact_topic_publishers_seen or bool(contact_endpoints)
        except Exception as exc:
            publisher_inspection_errors.append(str(exc))

    def on_status(message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        latest[:] = [status]

    def on_odom(message: Odometry) -> None:
        try:
            sim_stamp = ros_stamp_seconds(message.header.stamp)
        except ValueError as exc:
            odom_stamp_errors.append(str(exc))
            return
        if odom_samples and sim_stamp <= odom_samples[-1]["at_sim_time"]:
            odom_stamp_errors.append("odometry sim time is not strictly increasing")
            return
        pose = message.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        odom_samples.append(
            {
                "at_sim_time": sim_stamp,
                "at_wall_monotonic": time.monotonic(),
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw,
            }
        )

    def on_contacts(message: Contacts) -> None:
        nonlocal contact_messages
        contact_messages += 1
        for contact in message.contacts:
            first = contact.collision1.name
            second = contact.collision2.name
            if not is_prohibited_robot_contact(first, second):
                continue
            evidence = {"at_unix": time.time(), "collision1": first, "collision2": second}
            if evidence not in prohibited_contacts:
                prohibited_contacts.append(evidence)

    subscription = node.create_subscription(String, "/hospital_mission/status", on_status, 10)
    odom_subscription = node.create_subscription(Odometry, "/odom", on_odom, 50)
    contact_subscription = node.create_subscription(Contacts, "/hospital_amr/contacts", on_contacts, 10)
    started_waiting = time.monotonic()
    deadline = started_waiting + timeout
    terminal = None
    mission_started_sim_time: float | None = None
    terminal_sim_time: float | None = None
    mission_started_wall_monotonic: float | None = None
    terminal_wall_monotonic: float | None = None
    next_graph_sample = 0.0
    client = node.create_client(Trigger, "/hospital_mission/start")
    try:
        readiness_deadline = time.monotonic() + 10.0
        while time.monotonic() < readiness_deadline and (not latest or not odom_samples):
            executor.spin_once(timeout_sec=0.1)
        sample_publishers()
        if not latest or latest[0].get("state") != "IDLE" or not odom_samples:
            terminal = dict(latest[0]) if latest else {"state": "NO_STATUS"}
            terminal["failure_code"] = "ODOM_NOT_READY" if not odom_samples else "MISSION_NOT_IDLE"
        elif (
            len(publisher_samples[-1]["endpoints"]) != 1
            or publisher_samples[-1]["endpoints"][0]["node"] != "/hospital_mission_controller"
        ):
            terminal = dict(latest[0])
            terminal["failure_code"] = "CMD_VEL_NOT_EXCLUSIVE"
        elif not client.wait_for_service(timeout_sec=5.0):
            terminal = dict(latest[0])
            terminal["failure_code"] = "MISSION_SERVICE_UNAVAILABLE"
        else:
            future = client.call_async(Trigger.Request())
            start_deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < start_deadline:
                executor.spin_once(timeout_sec=0.1)
            if not future.done() or not future.result().success:
                terminal = dict(latest[0])
                terminal["failure_code"] = "MISSION_START_REJECTED"
            else:
                mission_started_wall_monotonic = time.monotonic()
                mission_started_sim_time = odom_samples[-1]["at_sim_time"]

        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if time.monotonic() >= next_graph_sample:
                sample_publishers()
                next_graph_sample = time.monotonic() + 0.25
            if terminal is not None:
                break
            if not latest:
                continue
            state = latest[0].get("state")
            if state in {"SUCCEEDED", "FAILED", "CANCELLED", "ESTOPPED"}:
                terminal = dict(latest[0])
                terminal_wall_monotonic = time.monotonic()
                terminal_sim_time = odom_samples[-1]["at_sim_time"] if odom_samples else None
                break
        if terminal is None:
            terminal = dict(latest[0]) if latest else {"state": "NO_STATUS"}
            terminal["failure_code"] = terminal.get("failure_code") or "ACCEPTANCE_WALL_TIMEOUT"
        if terminal_wall_monotonic is None:
            terminal_wall_monotonic = time.monotonic()
        if terminal_sim_time is None and odom_samples:
            terminal_sim_time = odom_samples[-1]["at_sim_time"]

        stop_deadline = time.monotonic() + 10.0
        while time.monotonic() < stop_deadline:
            executor.spin_once(timeout_sec=min(0.2, stop_deadline - time.monotonic()))
            if time.monotonic() >= next_graph_sample:
                sample_publishers()
                next_graph_sample = time.monotonic() + 0.25
            if (
                terminal_sim_time is not None
                and odom_samples
                and odom_samples[-1]["at_sim_time"] - terminal_sim_time >= 2.5
            ):
                break
    finally:
        monitor_stopped_unix = time.time()
        node.destroy_client(client)
        node.destroy_subscription(odom_subscription)
        node.destroy_subscription(contact_subscription)
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        node.destroy_node()
        context.shutdown()

    final_camera = capture_one_frame(camera_dir / f"acceptance-final-{timestamp}.png", timeout=20.0)
    for camera in (initial_camera, final_camera):
        camera["path"] = str(Path(camera["path"]).resolve().relative_to(PROJECT_ROOT))
    observed_endpoints: list[dict[str, str]] = []
    for sample in publisher_samples:
        for endpoint in sample["endpoints"]:
            if endpoint not in observed_endpoints:
                observed_endpoints.append(endpoint)
    publishers = sorted({endpoint["node"] for endpoint in observed_endpoints})
    unknown_publishers = sorted(set(publishers).difference(ALLOWED_CMD_VEL_PUBLISHERS))
    if mission_started_wall_monotonic is None:
        mission_started_wall_monotonic = terminal_wall_monotonic
    if mission_started_sim_time is None:
        mission_started_sim_time = terminal_sim_time
    route = load_acceptance_route(ROUTE_PATH)
    try:
        if odom_stamp_errors:
            raise ValueError("; ".join(sorted(set(odom_stamp_errors))))
        if mission_started_sim_time is None or terminal_sim_time is None:
            raise ValueError("simulation time bounds were not observed")
        odom_metrics = derive_odom_metrics(
            route,
            odom_samples,
            mission_started_sim_time=mission_started_sim_time,
            terminal_sim_time=terminal_sim_time,
            mission_started_wall_monotonic=mission_started_wall_monotonic,
            terminal_wall_monotonic=terminal_wall_monotonic,
        )
    except (KeyError, TypeError, ValueError) as exc:
        odom_metrics = {
            "metric_source": "diff_drive_odometry",
            "route_frame": "world",
            "time_domain": "ros_sim_time",
            "stages": [],
            "elapsed_seconds": None,
            "wall_elapsed_seconds": None,
            "real_time_factor": None,
            "terminal_pose": None,
            "stopped_pose": None,
            "stopped_drift_m": None,
            "odometry_evidence": {
                "topic": "/odom",
                "sample_count": len(odom_samples),
                "stamp_errors": odom_stamp_errors,
                "derivation_error": str(exc),
            },
        }
    report: dict[str, Any] = {
        "schema_version": 2,
        "generated_at_unix": time.time(),
        "mission_state": terminal.get("state"),
        "failure_code": terminal.get("failure_code"),
        **odom_metrics,
        "cmd_vel_publishers": publishers,
        "unknown_publishers": unknown_publishers,
        "publisher_evidence": {
            "monitor_started_unix": monitor_started_unix,
            "monitor_stopped_unix": monitor_stopped_unix,
            "samples": publisher_samples,
            "inspection_errors": publisher_inspection_errors,
            "observed_endpoints": observed_endpoints,
        },
        "contact_evidence": {
            "topic": "/hospital_amr/contacts",
            "monitor_started_unix": monitor_started_unix,
            "monitor_stopped_unix": monitor_stopped_unix,
            "topic_publishers_seen": contact_topic_publishers_seen,
            "messages": contact_messages,
            "prohibited_contacts": prohibited_contacts,
            "configured_sources": configured_contact_sources,
            "publisher_samples": contact_publisher_samples,
        },
        "camera": final_camera,
        "initial_camera": initial_camera,
        "safety": {
            "collision_free": contact_topic_publishers_seen and not prohibited_contacts,
            "safety_stop_failure": terminal.get("failure_code") in {"OBSTACLE_BLOCKED", "ODOM_STALE"},
        },
    }
    validate_acceptance_report(report)
    _write_json_atomic(output, report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--validate", type=Path)
    args = parser.parse_args(argv)
    if args.validate:
        try:
            report = load_strict_json(args.validate)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
            return 1
        try:
            errors = validate_acceptance_report(report)
        except Exception:
            report = _failure_report("REPORT_MALFORMED")
            errors = report["validation_errors"]
        _write_json_atomic(args.validate.resolve(), report)
        print(json.dumps({"ok": not errors, "errors": errors}, ensure_ascii=False))
        return 0 if not errors else 1
    try:
        report = generate_acceptance_report(args.timeout, args.output)
    except Exception as exc:
        report = _failure_report("ACCEPTANCE_GENERATION_FAILED")
        try:
            _write_json_atomic(args.output.resolve(), report)
        except Exception:
            args.output.unlink(missing_ok=True)
        print(
            json.dumps(
                {"ok": False, "error": type(exc).__name__, "failure_code": report["failure_code"]},
                ensure_ascii=False,
                allow_nan=False,
            )
        )
        return 1
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
