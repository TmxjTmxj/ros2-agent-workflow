"""Conservative inference of supported standard ROS 2 capabilities."""

from __future__ import annotations

from agent_ros.discovery.models import Capability, DiscoveryReport, GraphSnapshot
from agent_ros.profiles.models import (
    FOLLOW_JOINT_TRAJECTORY_TYPE,
    NAVIGATE_TO_POSE_TYPE,
    ODOMETRY_TYPE,
    TWIST_TYPE,
)


def infer_capabilities(snapshot: GraphSnapshot) -> DiscoveryReport:
    """Infer only recognized interface combinations from a graph snapshot."""
    capabilities: list[Capability] = []
    command_topics = _typed_topics(snapshot.topics, TWIST_TYPE, suffix="cmd_vel")
    odometry_topics = _typed_topics(snapshot.topics, ODOMETRY_TYPE, suffix="odom")
    if command_topics and odometry_topics:
        capabilities.append(Capability(
            "mobile_base.twist", 1.0, (command_topics[0], odometry_topics[0])
        ))
    navigation_actions = _typed_topics(snapshot.actions, NAVIGATE_TO_POSE_TYPE)
    if navigation_actions:
        capabilities.append(Capability("navigation.nav2", 1.0, (navigation_actions[0],)))
    camera_topics = _typed_topics(snapshot.topics, "sensor_msgs/msg/Image")
    if camera_topics:
        capabilities.append(Capability("perception.camera", 1.0, (camera_topics[0],)))
    scan_topics = _typed_topics(snapshot.topics, "sensor_msgs/msg/LaserScan")
    if scan_topics:
        capabilities.append(Capability("perception.laser_scan", 1.0, (scan_topics[0],)))
    trajectory_actions = _typed_topics(snapshot.actions, FOLLOW_JOINT_TRAJECTORY_TYPE)
    if trajectory_actions:
        capabilities.append(Capability(
            "manipulation.follow_joint_trajectory", 1.0, (trajectory_actions[0],)
        ))
    return DiscoveryReport(tuple(capabilities), tuple(_controller_warnings(snapshot, command_topics)))


def _typed_topics(entries, type_name: str, suffix: str | None = None) -> tuple[str, ...]:
    matches = tuple(
        name for name, types in entries.items()
        if type_name in types and (suffix is None or name.rstrip("/").endswith(suffix))
    )
    return matches


def _controller_warnings(snapshot: GraphSnapshot, command_topics: tuple[str, ...]) -> list[str]:
    warnings: list[str] = []
    for topic in command_topics:
        publishers = tuple(
            endpoint for endpoint in snapshot.topic_endpoints.get(topic, ())
            if endpoint.endpoint_type.lower() == "publisher"
        )
        if len(publishers) > 1:
            warnings.append(f"multiple command publishers on {topic}")
    return warnings
