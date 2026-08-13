from __future__ import annotations

from agent_ros.discovery.inference import infer_capabilities
from agent_ros.discovery.models import Endpoint, GraphSnapshot
from agent_ros.discovery.ros_graph import RosGraphProbe


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
        ("ros2", "node", "list"): "/controller\n/odometry\n",
        ("ros2", "topic", "list", "-t"): "/cmd_vel [geometry_msgs/msg/Twist]\n/odom [nav_msgs/msg/Odometry]\n",
        ("ros2", "action", "list", "-t"): "",
        ("ros2", "service", "list", "-t"): "",
        ("ros2", "topic", "info", "/cmd_vel", "--verbose"): """Type: geometry_msgs/msg/Twist
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
        ("ros2", "topic", "info", "/odom", "--verbose"): "",
    }

    def runner(argv):
        normalized = tuple(argv)
        calls.append(normalized)
        return responses[normalized]

    snapshot = RosGraphProbe(runner=runner).probe()

    assert calls[:3] == [
        ("ros2", "node", "list"),
        ("ros2", "topic", "list", "-t"),
        ("ros2", "action", "list", "-t"),
    ]
    assert snapshot.nodes == ("/controller", "/odometry")
    assert tuple(endpoint.gid for endpoint in snapshot.topic_endpoints["/cmd_vel"]) == ("01", "02")
