from __future__ import annotations

import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

from agent_ros.discovery.inference import infer_capabilities
from agent_ros.discovery.models import Endpoint, GraphSnapshot
from agent_ros.discovery.ros_graph import RosGraphProbe
from agent_ros.discovery import ros_graph
from agent_ros.errors import DiscoveryError


def test_infers_mobile_base_only_with_command_and_feedback():
    snapshot = GraphSnapshot(topics={
        "/cmd_vel": ("geometry_msgs/msg/Twist",),
        "/odom": ("nav_msgs/msg/Odometry",),
    })
    assert infer_capabilities(snapshot).capability_names == ("mobile_base.twist",)


def snapshot_with_two_cmd_vel_publishers() -> GraphSnapshot:
    return GraphSnapshot(
        topics={
            "/cmd_vel": ("geometry_msgs/msg/Twist",),
            "/odom": ("nav_msgs/msg/Odometry",),
        },
        topic_endpoints={
            "/cmd_vel": (
                Endpoint(node_name="controller_a", gid="aa", endpoint_type="publisher"),
                Endpoint(node_name="controller_b", gid="bb", endpoint_type="publisher"),
            ),
        },
    )


def test_duplicate_velocity_publishers_emit_blocking_warning():
    report = infer_capabilities(snapshot_with_two_cmd_vel_publishers())
    assert report.blocking_warnings == ("multiple command publishers on /cmd_vel",)


def test_discovery_infers_standard_capabilities_with_deterministic_confidence():
    snapshot = GraphSnapshot(
        topics={
            "/cmd_vel": ("geometry_msgs/msg/Twist",),
            "/odom": ("nav_msgs/msg/Odometry",),
            "/camera/image_raw": ("sensor_msgs/msg/Image",),
            "/scan": ("sensor_msgs/msg/LaserScan",),
        },
        actions={
            "/navigate_to_pose": ("nav2_msgs/action/NavigateToPose",),
            "/arm/follow_joint_trajectory": ("control_msgs/action/FollowJointTrajectory",),
        },
    )
    report = infer_capabilities(snapshot)

    assert report.capability_names == (
        "mobile_base.twist",
        "navigation.nav2",
        "perception.camera",
        "perception.laser_scan",
        "manipulation.follow_joint_trajectory",
    )
    assert all(capability.confidence == 1.0 for capability in report.capabilities)


def test_discovery_report_preserves_reviewed_endpoint_type_evidence_for_safety_validation():
    report = infer_capabilities(GraphSnapshot(
        topics={
            "/cmd_vel": ("geometry_msgs/msg/Twist",),
            "/odom": ("nav_msgs/msg/Odometry",),
        },
        actions={"/navigate_to_pose": ("nav2_msgs/action/NavigateToPose",)},
    ))

    assert report.topic_types["/cmd_vel"] == ("geometry_msgs/msg/Twist",)
    assert report.topic_types["/odom"] == ("nav_msgs/msg/Odometry",)
    assert report.action_types["/navigate_to_pose"] == ("nav2_msgs/action/NavigateToPose",)


def test_probe_uses_fixed_ros2_argv_and_preserves_duplicate_endpoint_gids():
    calls: list[tuple[str, ...]] = []
    responses = {
        ("ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"): "/controller\n/odometry\n",
        ("ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "0.5"): "/cmd_vel [geometry_msgs/msg/Twist]\n/odom [nav_msgs/msg/Odometry]\n",
        ("ros2", "action", "list", "-t"): "",
        ("ros2", "service", "list", "-t", "--no-daemon", "--spin-time", "0.5"): "",
        ("ros2", "topic", "info", "/cmd_vel", "--verbose", "--no-daemon", "--spin-time", "0.5"): """Type: geometry_msgs/msg/Twist
Publisher count: 2
Node name: controller
Node namespace: /
Endpoint type: PUBLISHER
GID: 01
Node name: controller
Node namespace: /
Endpoint type: PUBLISHER
GID: 02
""",
    }

    def runner(argv):
        normalized = tuple(argv)
        calls.append(normalized)
        return responses[normalized]

    snapshot = RosGraphProbe(runner=runner).probe()

    assert set(calls[:4]) == {
        ("ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"),
        ("ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
        ("ros2", "action", "list", "-t"),
        ("ros2", "service", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
    }
    assert snapshot.nodes == ("/controller", "/odometry")
    assert tuple(endpoint.gid for endpoint in snapshot.topic_endpoints["/cmd_vel"]) == ("01", "02")


def test_probe_collects_independent_fixed_graph_queries_concurrently():
    initial = {
        ("ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"),
        ("ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
        ("ros2", "action", "list", "-t"),
        ("ros2", "service", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
    }
    barrier = threading.Barrier(len(initial))

    def runner(argv):
        normalized = tuple(argv)
        if normalized in initial:
            try:
                barrier.wait(timeout=0.25)
            except threading.BrokenBarrierError:
                raise AssertionError("independent graph queries were serialized") from None
        return ""

    snapshot = RosGraphProbe(runner=runner).probe()

    assert snapshot.nodes == ()
    assert dict(snapshot.topics) == {}


def test_probe_only_inspects_bounded_typed_command_candidates_with_many_topics():
    unrelated = "".join(
        f"/sensor_{index} [sensor_msgs/msg/Image]\n" for index in range(1000)
    )
    topic_list = unrelated + "/cmd_vel [geometry_msgs/msg/Twist]\n"
    calls: list[tuple[str, ...]] = []

    def runner(argv):
        normalized = tuple(argv)
        calls.append(normalized)
        if normalized[:4] == ("ros2", "topic", "list", "-t"):
            return topic_list
        if normalized[:5] == ("ros2", "topic", "info", "/cmd_vel", "--verbose"):
            return ""
        if normalized[:3] in {
            ("ros2", "node", "list"),
            ("ros2", "action", "list"),
            ("ros2", "service", "list"),
        }:
            return ""
        raise AssertionError(normalized)

    RosGraphProbe(runner=runner).probe()

    details = [call for call in calls if call[:3] == ("ros2", "topic", "info")]
    assert len(details) == 1
    assert details[0][3] == "/cmd_vel"


def test_production_graph_command_has_fixed_subprocess_timeout_and_no_shell(monkeypatch):
    observed = []

    def blocked(argv, **kwargs):
        observed.append((tuple(argv), kwargs))
        raise subprocess.TimeoutExpired(argv, kwargs["timeout"])

    monkeypatch.setattr(subprocess, "run", blocked)

    with pytest.raises(DiscoveryError):
        RosGraphProbe(runner=ros_graph._run_ros2).probe()

    assert observed
    assert all(call[1]["shell"] is False for call in observed)
    assert all(0.0 < call[1]["timeout"] <= 6.0 for call in observed)


def _fake_native_modules(events, *, create_error=None, destroy_error=None, spin_error=None):
    class Context:
        def init(self, *, args=None):
            events.append(("context.init", args))

        def try_shutdown(self):
            events.append(("context.try_shutdown",))

    class Node:
        def get_node_names_and_namespaces(self):
            return [("controller", "/"), ("odom", "/hospital")]

        def get_topic_names_and_types(self):
            return [
                ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
                ("/odom", ["nav_msgs/msg/Odometry"]),
            ]

        def get_service_names_and_types(self):
            return [("/hospital_mission/start", ["std_srvs/srv/Trigger"])]

        def get_publishers_info_by_topic(self, topic):
            assert topic == "/cmd_vel"
            return [SimpleNamespace(
                node_name="controller",
                node_namespace="/",
                endpoint_gid=bytes.fromhex("0102"),
                endpoint_type=SimpleNamespace(name="PUBLISHER"),
            )]

        def destroy_node(self):
            events.append(("node.destroy",))
            if destroy_error is not None:
                raise destroy_error

    class Executor:
        def __init__(self, *, context):
            events.append(("executor.init", context))

        def add_node(self, node):
            events.append(("executor.add", node))

        def spin_once(self, *, timeout_sec):
            events.append(("executor.spin_once", timeout_sec))
            if spin_error is not None:
                raise spin_error

        def remove_node(self, node):
            events.append(("executor.remove", node))

        def shutdown(self, *, timeout_sec):
            events.append(("executor.shutdown", timeout_sec))
            return True

    def create_node(name, *, context):
        events.append(("create_node", name, context))
        if create_error is not None:
            raise create_error
        return Node()

    rclpy = SimpleNamespace(create_node=create_node)

    def actions(*, node):
        events.append(("actions", node))
        return [("/navigate_to_pose", ["nav2_msgs/action/NavigateToPose"])]

    return rclpy, Context, Executor, actions


def test_production_probe_uses_one_native_participant_maps_gid_and_cleans_lifecycle(
    monkeypatch,
):
    events = []
    monkeypatch.setattr(
        ros_graph,
        "_native_modules",
        lambda: _fake_native_modules(events),
        raising=False,
    )

    snapshot = ros_graph._probe_native()

    assert snapshot.nodes == ("/controller", "/hospital/odom")
    assert snapshot.topics["/cmd_vel"] == ("geometry_msgs/msg/Twist",)
    assert snapshot.services["/hospital_mission/start"] == ("std_srvs/srv/Trigger",)
    assert snapshot.actions["/navigate_to_pose"] == ("nav2_msgs/action/NavigateToPose",)
    endpoint = snapshot.topic_endpoints["/cmd_vel"][0]
    assert (endpoint.node_name, endpoint.node_namespace, endpoint.gid, endpoint.endpoint_type) == (
        "controller", "/", "0102", "publisher"
    )
    assert [event[0] for event in events].count("context.init") == 1
    assert ("executor.spin_once", 0.5) in events
    assert [event[0] for event in events][-4:] == [
        "executor.remove", "node.destroy", "executor.shutdown", "context.try_shutdown"
    ]


@pytest.mark.parametrize("failure", ["create", "spin", "destroy"])
def test_production_native_probe_start_spin_or_cleanup_failure_is_fail_closed(
    monkeypatch, failure
):
    events = []
    error = RuntimeError(failure)
    modules = _fake_native_modules(
        events,
        create_error=error if failure == "create" else None,
        spin_error=error if failure == "spin" else None,
        destroy_error=error if failure == "destroy" else None,
    )
    monkeypatch.setattr(ros_graph, "_native_modules", lambda: modules, raising=False)

    with pytest.raises(DiscoveryError):
        ros_graph._probe_native()

    assert ("context.try_shutdown",) in events


def test_production_probe_runs_one_fixed_native_helper_subprocess(monkeypatch):
    payload = {
        "nodes": ["/controller"],
        "topics": {"/cmd_vel": ["geometry_msgs/msg/Twist"]},
        "services": {},
        "actions": {},
        "topic_endpoints": {
            "/cmd_vel": [{
                "node_name": "controller",
                "node_namespace": "/",
                "gid": "0102",
                "endpoint_type": "publisher",
            }]
        },
    }
    calls = []

    def completed(argv, **kwargs):
        calls.append((tuple(argv), kwargs))
        return subprocess.CompletedProcess(argv, 0, __import__("json").dumps(payload), "")

    monkeypatch.setattr(subprocess, "run", completed)

    snapshot = RosGraphProbe().probe()

    assert calls[0][0] == (sys.executable, "-m", "agent_ros.discovery.native_probe")
    assert calls[0][1]["shell"] is False
    assert 0.0 < calls[0][1]["timeout"] <= 6.0
    assert snapshot.topic_endpoints["/cmd_vel"][0].gid == "0102"


@pytest.mark.parametrize(
    "result",
    [
        subprocess.CompletedProcess([], 1, "", "private DDS failure"),
        subprocess.CompletedProcess([], 0, "not-json", ""),
        subprocess.CompletedProcess([], 0, '{"nodes":[],"topics":{}}', ""),
    ],
)
def test_production_native_helper_nonzero_or_malformed_json_fails_closed(
    monkeypatch, result
):
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: result)

    with pytest.raises(DiscoveryError):
        RosGraphProbe().probe()


def test_production_native_helper_timeout_is_hard_killable_fail_closed(monkeypatch):
    observed = []

    def blocked(argv, **kwargs):
        observed.append((tuple(argv), kwargs))
        raise subprocess.TimeoutExpired(argv, kwargs["timeout"])

    monkeypatch.setattr(subprocess, "run", blocked)

    with pytest.raises(DiscoveryError):
        RosGraphProbe().probe()

    assert observed[0][0] == (sys.executable, "-m", "agent_ros.discovery.native_probe")
    assert observed[0][1]["shell"] is False
    assert observed[0][1]["timeout"] <= 6.0
