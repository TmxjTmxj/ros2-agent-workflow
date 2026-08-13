"""Persistent ROS 2 adapter for the hospital mission controller core."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Callable

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .controller_core import (
    MissionControllerCore,
    MissionState,
    Pose2D,
    load_route,
    normalize_angle,
)


TERMINAL_STATES = {
    MissionState.SUCCEEDED,
    MissionState.FAILED,
    MissionState.CANCELLED,
    MissionState.ESTOPPED,
}


def default_route_path() -> Path:
    return Path(get_package_share_directory("smartcar_bringup")) / "config" / "mission_routes.json"


class MissionControllerNode(Node):
    """Own `/cmd_vel` and execute the route at a fixed timer rate."""

    def __init__(
        self,
        *,
        route_path: str | None = None,
        time_fn: Callable[[], float] = time.monotonic,
        context=None,
    ) -> None:
        super().__init__("hospital_mission_controller", context=context)
        route_default = str(route_path or default_route_path())
        self.declare_parameter("route_file", route_default)
        selected_route = str(self.get_parameter("route_file").value)
        self.core = MissionControllerCore(load_route(selected_route))
        self._time_fn = time_fn
        self._pose: Pose2D | None = None
        self._last_odom_received: float | None = None
        self._feedback_source: str | None = None
        self._front_range = math.inf
        self._last_state = self.core.state
        self._zero_burst_remaining = 0

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(String, "/hospital_mission/status", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_service(Trigger, "/hospital_mission/start", self._on_start)
        self.create_service(Trigger, "/hospital_mission/cancel", self._on_cancel)
        self.create_service(Trigger, "/hospital_mission/estop", self._on_estop)
        self.create_service(Trigger, "/hospital_mission/reset", self._on_reset)
        self._timer = self.create_timer(0.05, self._control_tick)

    @staticmethod
    def _yaw_from_orientation(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_odom(self, msg: Odometry) -> None:
        now = self._time_fn()
        p = msg.pose.pose.position
        self._pose = Pose2D(
            float(p.x),
            float(p.y),
            normalize_angle(self._yaw_from_orientation(msg.pose.pose.orientation)),
        )
        self._last_odom_received = now
        self._feedback_source = "gazebo_model_odometry"

    def _on_scan(self, msg: LaserScan) -> None:
        candidates: list[float] = []
        increment = float(msg.angle_increment)
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(value):
                continue
            if value < msg.range_min or value > msg.range_max:
                continue
            angle = float(msg.angle_min) + index * increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if abs(angle) <= math.radians(30.0):
                candidates.append(float(value))
        self._front_range = min(candidates, default=math.inf)

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._pose is None:
            response.success = False
            response.message = "ODOM_NOT_READY"
            return response
        result = self.core.start(self._time_fn())
        response.success = result.accepted
        response.message = result.message
        return response

    def _on_cancel(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self.core.cancel()
        if result.accepted:
            self._zero_burst_remaining = 10
        response.success = result.accepted
        response.message = result.message
        return response

    def _on_estop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self.core.estop()
        self._zero_burst_remaining = 10
        response.success = result.accepted
        response.message = result.message
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self.core.reset()
        if result.accepted:
            self._zero_burst_remaining = 10
        response.success = result.accepted
        response.message = result.message
        return response

    def _publish_command(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _status_document(self, now: float) -> dict:
        status = self.core.status()
        if self._pose is None:
            status["pose"] = None
        else:
            status["pose"] = {"x": self._pose.x, "y": self._pose.y, "yaw": self._pose.yaw}
        status["odom_age"] = (
            None if self._last_odom_received is None else max(0.0, now - self._last_odom_received)
        )
        status["front_range"] = self._front_range if math.isfinite(self._front_range) else None
        status["feedback_source"] = self._feedback_source
        if self.core.stage_index < len(self.core.route.stages):
            stage = self.core.route.stages[self.core.stage_index]
            if self.core.waypoint_index < len(stage.waypoints):
                target = stage.waypoints[self.core.waypoint_index]
                status["current_target"] = {"x": target.x, "y": target.y}
                status["distance_to_waypoint"] = (
                    None
                    if self._pose is None
                    else math.hypot(target.x - self._pose.x, target.y - self._pose.y)
                )
        return status

    def _control_tick(self) -> None:
        now = self._time_fn()
        command = self.core.update(
            self._pose,
            now=now,
            front_range=self._front_range,
            odom_received_at=self._last_odom_received,
        )
        if self.core.state in TERMINAL_STATES and self._last_state not in TERMINAL_STATES:
            self._zero_burst_remaining = 10
        self._last_state = self.core.state
        if self._zero_burst_remaining > 0:
            command = type(command)()
            self._zero_burst_remaining -= 1
        self._publish_command(command.linear, command.angular)
        status_msg = String()
        status_msg.data = json.dumps(self._status_document(now), ensure_ascii=False, allow_nan=False)
        self._status_pub.publish(status_msg)

    def close(self) -> None:
        """Publish a best-effort stop burst before the caller shuts ROS down."""
        if self.context.ok():
            for _ in range(10):
                self._publish_command()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
