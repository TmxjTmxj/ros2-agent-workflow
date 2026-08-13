import json
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from smartcar_bringup.mission_controller import MissionControllerNode


def write_route(tmp_path):
    route = {
        "start": [0.0, 0.0, 0.0],
        "stages": [
            {"id": "pharmacy", "name": "p", "endpoint": [1, 0], "waypoints": [[1, 0]]},
            {"id": "ward2", "name": "w", "endpoint": [1, 1], "waypoints": [[1, 1]]},
            {"id": "laboratory", "name": "l", "endpoint": [0, 1], "waypoints": [[0, 1]]},
        ],
    }
    path = tmp_path / "route.json"
    path.write_text(json.dumps(route))
    return path


class RosHarness:
    def __init__(self, route_path):
        self.context = Context()
        rclpy.init(context=self.context)
        self.controller = MissionControllerNode(
            route_path=str(route_path),
            context=self.context,
        )
        self.probe = Node("mission_controller_test_probe", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.controller)
        self.executor.add_node(self.probe)
        self.commands = []
        self.statuses = []
        self.probe.create_subscription(Twist, "/cmd_vel", self.commands.append, 10)
        self.probe.create_subscription(
            __import__("std_msgs.msg", fromlist=["String"]).String,
            "/hospital_mission/status",
            lambda msg: self.statuses.append(json.loads(msg.data)),
            10,
        )
        self.odom_pub = self.probe.create_publisher(Odometry, "/odom", 10)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def publish_pose(self, x=0.0, y=0.0, yaw=0.0):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        for _ in range(3):
            self.odom_pub.publish(msg)
            self.spin_for(0.03)

    def trigger(self, service_name):
        client = self.probe.create_client(Trigger, service_name)
        assert client.wait_for_service(timeout_sec=1.0)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        assert future.done()
        self.probe.destroy_client(client)
        return future.result()

    def close(self):
        self.executor.remove_node(self.probe)
        self.executor.remove_node(self.controller)
        self.probe.destroy_node()
        self.controller.close()
        self.controller.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def harness(tmp_path):
    value = RosHarness(write_route(tmp_path))
    try:
        yield value
    finally:
        value.close()


def test_node_exposes_bounded_services_and_publishes_zero_while_idle(harness):
    """Catches a missing control endpoint or motion before mission start."""
    service_names = {name for name, _ in harness.controller.get_service_names_and_types()}
    assert {
        "/hospital_mission/start",
        "/hospital_mission/cancel",
        "/hospital_mission/estop",
        "/hospital_mission/reset",
    }.issubset(service_names)

    harness.spin_for(0.25)

    assert harness.commands
    assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in harness.commands)
    assert harness.statuses[-1]["state"] == "IDLE"


def test_start_uses_real_odometry_and_publishes_schema_and_motion(harness):
    """Catches a ROS adapter that starts but never feeds feedback into control."""
    harness.publish_pose(0.0, 0.0, 0.0)

    response = harness.trigger("/hospital_mission/start")
    # Real odometry is continuous; keep the synthetic stream fresh even when
    # DDS discovery is slow under the full-suite load.
    harness.publish_pose(0.0, 0.0, 0.0)
    harness.spin_for(0.25)

    assert response.success
    assert any(msg.linear.x > 0.0 for msg in harness.commands)
    status = harness.statuses[-1]
    assert status["state"] == "RUNNING"
    assert status["stage_id"] == "pharmacy"
    assert status["pose"] == pytest.approx({"x": 0.0, "y": 0.0, "yaw": 0.0})
    assert status["odom_age"] < 0.5


def test_model_odometry_is_consumed_as_identity_preserving_world_pose(tmp_path):
    route_path = write_route(tmp_path)
    route = json.loads(route_path.read_text())
    route["start"] = [12.0, 8.0, math.pi / 2.0]
    route_path.write_text(json.dumps(route))
    value = RosHarness(route_path)
    try:
        value.publish_pose(1.0, 2.0, 0.25)
        value.spin_for(0.15)
        pose = value.statuses[-1]["pose"]
        assert pose == pytest.approx({"x": 1.0, "y": 2.0, "yaw": 0.25})
    finally:
        value.close()


def test_model_specific_odometry_is_the_only_pose_feedback(harness):
    harness.publish_pose(0.25, 0.5, 0.75)
    harness.spin_for(0.15)

    status = harness.statuses[-1]

    assert status["pose"] == pytest.approx({"x": 0.25, "y": 0.5, "yaw": 0.75})
    assert status["feedback_source"] == "gazebo_model_odometry"


def test_cancel_and_estop_publish_repeated_zero_and_estop_latches(harness):
    """Catches a terminal transition that emits only one lossy stop message."""
    harness.publish_pose()
    assert harness.trigger("/hospital_mission/start").success
    harness.spin_for(0.1)

    assert harness.trigger("/hospital_mission/cancel").success
    harness.spin_for(0.05)
    harness.commands.clear()
    harness.spin_for(0.25)
    cancel_commands = harness.commands
    assert len(cancel_commands) >= 3
    assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in cancel_commands)

    assert harness.trigger("/hospital_mission/start").success
    assert harness.trigger("/hospital_mission/estop").success
    rejected = harness.trigger("/hospital_mission/start")
    assert not rejected.success
    harness.spin_for(0.15)
    assert harness.statuses[-1]["state"] == "ESTOPPED"


def test_rejected_reset_does_not_interrupt_running_command(harness):
    harness.publish_pose()
    assert harness.trigger("/hospital_mission/start").success
    harness.spin_for(0.1)
    before = harness.controller._zero_burst_remaining

    rejected = harness.trigger("/hospital_mission/reset")

    assert not rejected.success
    assert harness.controller._zero_burst_remaining == before
