#!/usr/bin/env python3
"""Generate or validate real hospital-delivery acceptance evidence."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = PROJECT_ROOT / "logs" / "acceptance_report.json"
EXPECTED_STAGE_IDS = ["pharmacy", "ward2", "laboratory"]
ALLOWED_CMD_VEL_PUBLISHERS = {"/hospital_mission_controller"}


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    return converted if math.isfinite(converted) else None


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
        path = Path(str(camera["path"]))
        if not path.is_absolute():
            path = PROJECT_ROOT / path
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


def validate_acceptance_report(report: dict[str, Any]) -> list[str]:
    """Validate every hard acceptance invariant and update pass fields."""
    errors: list[str] = []
    if report.get("schema_version") != 2:
        errors.append(f"schema version is {report.get('schema_version')!r}, expected 2")
    if report.get("mission_state") != "SUCCEEDED":
        errors.append(f"mission state is {report.get('mission_state')!r}, expected SUCCEEDED")
    if report.get("failure_code") is not None:
        errors.append(f"mission failure code is {report.get('failure_code')!r}")

    stages = report.get("stages")
    if not isinstance(stages, list) or [item.get("id") for item in stages if isinstance(item, dict)] != EXPECTED_STAGE_IDS:
        errors.append("stage order must be pharmacy, ward2, laboratory exactly once")
    else:
        for stage in stages:
            error = _number(stage.get("endpoint_error"))
            if error is None or error > 0.50:
                errors.append(
                    f"stage {stage.get('id')} endpoint error {stage.get('endpoint_error')!r} exceeds 0.50 m"
                )

    elapsed = _number(report.get("elapsed_seconds"))
    if elapsed is None or elapsed > 180.0:
        errors.append(f"elapsed time {report.get('elapsed_seconds')!r} exceeds 180 s")
    drift = _number(report.get("stopped_drift_m"))
    if drift is None or drift > 0.02:
        errors.append(f"stopped drift {report.get('stopped_drift_m')!r} exceeds 0.02 m")

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
            times = [
                _number(sample.get("at_unix")) if isinstance(sample, dict) else None
                for sample in samples
            ]
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
                gaps = [
                    second - first
                    for first, second in zip(numeric_times, numeric_times[1:])
                ]
                required_duration = (elapsed or 0.0) + 2.5
                if (
                    numeric_times[0] - started > 1.0
                    or stopped - numeric_times[-1] > 1.0
                    or (gaps and max(gaps) > 1.0)
                    or stopped - started < required_duration
                ):
                    errors.append("publisher monitoring coverage is incomplete")
            for sample in samples:
                endpoints = sample.get("endpoints") if isinstance(sample, dict) else None
                if (
                    not isinstance(endpoints, list)
                    or len(endpoints) != 1
                    or endpoints[0].get("node") != "/hospital_mission_controller"
                    or not endpoints[0].get("gid")
                ):
                    errors.append(f"publisher sample is not exclusive: {sample!r}")
                    break

    if not _valid_png(report.get("camera")):
        errors.append("camera evidence is missing or is not a valid PNG")
    safety = report.get("safety")
    contact_evidence = report.get("contact_evidence")
    if not isinstance(contact_evidence, dict):
        errors.append("contact evidence is missing")
    else:
        contact_started = _number(contact_evidence.get("monitor_started_unix"))
        contact_stopped = _number(contact_evidence.get("monitor_stopped_unix"))
        publisher_started = (
            _number(publisher_evidence.get("monitor_started_unix"))
            if isinstance(publisher_evidence, dict)
            else None
        )
        publisher_stopped = (
            _number(publisher_evidence.get("monitor_stopped_unix"))
            if isinstance(publisher_evidence, dict)
            else None
        )
        if (
            contact_started is None
            or contact_stopped is None
            or contact_stopped < contact_started
            or contact_stopped - contact_started < (elapsed or 0.0) + 2.5
            or publisher_started is None
            or publisher_stopped is None
            or abs(contact_started - publisher_started) > 0.1
            or abs(contact_stopped - publisher_stopped) > 0.1
        ):
            errors.append("contact monitoring coverage is incomplete")
        if contact_evidence.get("topic_publishers_seen") is not True:
            errors.append("contact monitor topic was not active")
        contact_messages = contact_evidence.get("messages")
        if (
            isinstance(contact_messages, bool)
            or not isinstance(contact_messages, int)
            or contact_messages <= 0
        ):
            errors.append("contact messages were not observed")
        contacts = contact_evidence.get("prohibited_contacts")
        if not isinstance(contacts, list) or contacts:
            errors.append(f"prohibited contact detected: {contacts!r}")
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
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _distance(first: dict[str, Any], second: dict[str, Any]) -> float:
    return math.hypot(float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"]))


def generate_acceptance_report(timeout: float = 190.0, output: Path = DEFAULT_REPORT) -> dict[str, Any]:
    """Arm evidence first, start one mission, then save the complete run."""
    import rclpy
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
    minimum_front_range = math.inf
    publisher_samples: list[dict[str, Any]] = []
    publisher_inspection_errors: list[str] = []
    contact_messages = 0
    prohibited_contacts: list[dict[str, Any]] = []
    contact_topic_publishers_seen = False
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
            contact_topic_publishers_seen = contact_topic_publishers_seen or bool(
                graph_endpoints("/hospital_amr/contacts")
            )
        except Exception as exc:
            publisher_inspection_errors.append(str(exc))

    def on_status(message: String) -> None:
        nonlocal minimum_front_range
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        latest[:] = [status]
        front = _number(status.get("front_range"))
        if front is not None:
            minimum_front_range = min(minimum_front_range, front)

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
    contact_subscription = node.create_subscription(
        Contacts, "/hospital_amr/contacts", on_contacts, 10
    )
    started_waiting = time.monotonic()
    deadline = started_waiting + timeout
    terminal = None
    next_graph_sample = 0.0
    client = node.create_client(Trigger, "/hospital_mission/start")
    try:
        readiness_deadline = time.monotonic() + 10.0
        while time.monotonic() < readiness_deadline and not latest:
            executor.spin_once(timeout_sec=0.1)
        sample_publishers()
        if not latest or latest[0].get("state") != "IDLE":
            terminal = dict(latest[0]) if latest else {"state": "NO_STATUS"}
            terminal["failure_code"] = "MISSION_NOT_IDLE"
        elif (
            len(publisher_samples[-1]["endpoints"]) != 1
            or publisher_samples[-1]["endpoints"][0]["node"]
            != "/hospital_mission_controller"
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
                break
        if terminal is None:
            terminal = dict(latest[0]) if latest else {"state": "NO_STATUS"}
            terminal["failure_code"] = terminal.get("failure_code") or "ACCEPTANCE_TIMEOUT"

        before_stop = terminal.get("pose")
        stop_deadline = time.monotonic() + 3.0
        while time.monotonic() < stop_deadline:
            executor.spin_once(timeout_sec=min(0.2, stop_deadline - time.monotonic()))
            if time.monotonic() >= next_graph_sample:
                sample_publishers()
                next_graph_sample = time.monotonic() + 0.25
        after_stop = latest[0].get("pose") if latest else None
    finally:
        monitor_stopped_unix = time.time()
        node.destroy_client(client)
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
    stopped_drift = (
        _distance(before_stop, after_stop)
        if isinstance(before_stop, dict) and isinstance(after_stop, dict)
        else None
    )
    report: dict[str, Any] = {
        "schema_version": 2,
        "generated_at_unix": time.time(),
        "mission_state": terminal.get("state"),
        "failure_code": terminal.get("failure_code"),
        "stages": terminal.get("stage_results", []),
        "elapsed_seconds": terminal.get("elapsed"),
        "terminal_pose": terminal.get("pose"),
        "stopped_pose": after_stop,
        "stopped_drift_m": stopped_drift,
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
        },
        "camera": final_camera,
        "initial_camera": initial_camera,
        "safety": {
            "collision_free": contact_topic_publishers_seen and not prohibited_contacts,
            "safety_stop_failure": terminal.get("failure_code") in {"OBSTACLE_BLOCKED", "ODOM_STALE"},
            "minimum_front_range_m": None if not math.isfinite(minimum_front_range) else minimum_front_range,
        },
    }
    validate_acceptance_report(report)
    _write_json_atomic(output, report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=190.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--validate", type=Path)
    args = parser.parse_args(argv)
    if args.validate:
        try:
            report = json.loads(args.validate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
            return 1
        errors = validate_acceptance_report(report)
        _write_json_atomic(args.validate.resolve(), report)
        print(json.dumps({"ok": not errors, "errors": errors}, ensure_ascii=False))
        return 0 if not errors else 1
    try:
        report = generate_acceptance_report(args.timeout, args.output)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
