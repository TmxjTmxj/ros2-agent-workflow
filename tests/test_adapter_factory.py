from __future__ import annotations

import subprocess
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from agent_ros.adapters.base import AdapterError
from agent_ros.adapters.factory import RclpyAdapterFactory
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.profiles.models import RobotProfile
from mcp_server import ros2_mcp_server


class FakePublisher:
    def publish(self, _message) -> None:
        return None


class FakeNode:
    def __init__(self) -> None:
        self.destroyed = False

    def create_publisher(self, _message_type, _name, _depth):
        return FakePublisher()

    def create_subscription(self, _message_type, _name, callback, _depth):
        return callback

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeContext:
    def __init__(self) -> None:
        self.initialized = False
        self.shutdown_called = False

    def init(self, *, args=None) -> None:
        self.initialized = True

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeExecutor:
    instances = []

    def __init__(self, *, context) -> None:
        self.context = context
        self.nodes = []
        self.spin_started = threading.Event()
        self.stop = threading.Event()
        self.shutdown_timeouts = []
        type(self).instances.append(self)

    def add_node(self, node) -> None:
        self.nodes.append(node)

    def remove_node(self, node) -> None:
        self.nodes.remove(node)

    def spin(self) -> None:
        self.spin_started.set()
        self.stop.wait()

    def shutdown(self, *, timeout_sec) -> bool:
        self.shutdown_timeouts.append(timeout_sec)
        self.stop.set()
        return True


class RetryableShutdownExecutor(FakeExecutor):
    instances = []

    def __init__(self, *, context) -> None:
        super().__init__(context=context)
        self.shutdown_attempts = 0

    def shutdown(self, *, timeout_sec) -> bool:
        self.shutdown_timeouts.append(timeout_sec)
        self.shutdown_attempts += 1
        if self.shutdown_attempts == 1:
            return False
        self.stop.set()
        return True


def _install_fake_ros(monkeypatch) -> None:
    FakeExecutor.instances.clear()

    class FakeTwist:
        def __init__(self) -> None:
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    rclpy = ModuleType("rclpy")
    rclpy.create_node = lambda _name, *, context: FakeNode()
    rclpy_context = ModuleType("rclpy.context")
    rclpy_context.Context = FakeContext
    rclpy_executors = ModuleType("rclpy.executors")
    rclpy_executors.SingleThreadedExecutor = FakeExecutor
    geometry = ModuleType("geometry_msgs")
    geometry_msg = ModuleType("geometry_msgs.msg")
    geometry_msg.Twist = FakeTwist
    geometry.msg = geometry_msg
    nav = ModuleType("nav_msgs")
    nav_msg = ModuleType("nav_msgs.msg")
    nav_msg.Odometry = type("Odometry", (), {})
    nav.msg = nav_msg
    std = ModuleType("std_msgs")
    std_msg = ModuleType("std_msgs.msg")
    std_msg.Bool = type("Bool", (), {})
    std.msg = std_msg
    for name, module in {
        "rclpy": rclpy,
        "rclpy.context": rclpy_context,
        "rclpy.executors": rclpy_executors,
        "geometry_msgs": geometry,
        "geometry_msgs.msg": geometry_msg,
        "nav_msgs": nav,
        "nav_msgs.msg": nav_msg,
        "std_msgs": std,
        "std_msgs.msg": std_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)


def _graph_cli(argv, **kwargs):
    command = tuple(argv)
    values = {
        ("ros2", "node", "list"): "/base_controller\n",
        ("ros2", "topic", "list", "-t"): (
            "/cmd_vel [geometry_msgs/msg/Twist]\n"
            "/odom [nav_msgs/msg/Odometry]\n"
        ),
        ("ros2", "action", "list", "-t"): "",
        ("ros2", "service", "list", "-t"): "",
        ("ros2", "topic", "info", "/cmd_vel", "--verbose"): (
            "Node name: base_controller\n"
            "Node namespace: /\n"
            "Endpoint type: PUBLISHER\n"
            "GID: 01\n"
        ),
        ("ros2", "topic", "info", "/odom", "--verbose"): (
            "Node name: odometry\n"
            "Node namespace: /\n"
            "Endpoint type: PUBLISHER\n"
            "GID: 02\n"
        ),
    }
    return subprocess.CompletedProcess(argv, 0, values[command], "")


def _trajectory_profile() -> RobotProfile:
    return RobotProfile.from_mapping({
        "name": "arm",
        "mode": "simulation",
        "namespace": "/arm",
        "frames": {"base": "base_link"},
        "adapter": {"kind": "follow_joint_trajectory"},
        "interfaces": {
            "trajectory": {
                "action": "/joint_trajectory_controller/follow_joint_trajectory",
                "type": "control_msgs/action/FollowJointTrajectory",
            }
        },
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
        "observation_sources": [],
    })


def _twist_profile() -> RobotProfile:
    return RobotProfile.from_mapping({
        "name": "base",
        "mode": "simulation",
        "namespace": "/base",
        "frames": {"base": "base_link", "odom": "odom"},
        "adapter": {"kind": "twist"},
        "interfaces": {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        },
        "limits": {
            "max_linear_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 1.0,
        },
        "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
        "observation_sources": ["odometry"],
    })


def test_production_singleton_discovers_with_repository_owned_twist_factory(
    monkeypatch, tmp_path
):
    _install_fake_ros(monkeypatch)
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")

    controller = ros2_mcp_server.get_runtime_controller()
    result = controller.discover_robot("hospital-amr")

    assert result["state"] == "DISCOVERED"
    assert isinstance(controller._adapter, TwistAdapter)
    assert len(FakeExecutor.instances) == 1
    assert FakeExecutor.instances[0].spin_started.wait(0.2)
    assert controller._adapter._transport._publisher is not None
    assert ros2_mcp_server.close_runtime_controller() is True
    assert FakeExecutor.instances[0].nodes == []
    assert FakeExecutor.instances[0].context.shutdown_called


def test_production_factory_rejects_unsupported_trajectory_without_starting_ros(
    monkeypatch,
):
    _install_fake_ros(monkeypatch)
    factory = RclpyAdapterFactory()

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        factory(_trajectory_profile())

    assert FakeExecutor.instances == []
    assert factory.close(0.1)


def test_adapter_close_reaps_owned_executor_thread(monkeypatch):
    _install_fake_ros(monkeypatch)
    factory = RclpyAdapterFactory()
    adapter = factory(_twist_profile())
    executor = FakeExecutor.instances[0]
    assert executor.spin_started.wait(0.2)
    owned_thread = factory._thread
    assert owned_thread is not None and owned_thread.is_alive()
    assert owned_thread.daemon is False

    assert adapter.close(0.2)

    owned_thread.join(0.2)
    assert not owned_thread.is_alive()
    assert executor.nodes == []
    assert executor.context.shutdown_called
    assert factory.close(0.1)


def test_factory_retries_only_incomplete_teardown_after_shutdown_failure(monkeypatch):
    _install_fake_ros(monkeypatch)
    RetryableShutdownExecutor.instances.clear()
    monkeypatch.setattr(
        sys.modules["rclpy.executors"],
        "SingleThreadedExecutor",
        RetryableShutdownExecutor,
    )
    factory = RclpyAdapterFactory()
    factory(_twist_profile())
    executor = RetryableShutdownExecutor.instances[0]
    node = executor.nodes[0]
    owned_thread = factory._thread
    assert owned_thread is not None
    try:
        assert factory.close(0.05) is False
        assert owned_thread.is_alive()
        assert executor.shutdown_attempts == 1
        assert executor.nodes == [node]
        assert node.destroyed is False
        assert executor.context.shutdown_called is False

        assert factory.close(0.2) is True
        assert executor.shutdown_attempts == 2
        assert not owned_thread.is_alive()
        assert executor.nodes == []
        assert node.destroyed is True
        assert executor.context.shutdown_called is True
    finally:
        executor.stop.set()
        owned_thread.join(0.2)


def test_singleton_retry_clears_poison_only_after_owned_executor_is_reaped(
    monkeypatch, tmp_path
):
    _install_fake_ros(monkeypatch)
    RetryableShutdownExecutor.instances.clear()
    monkeypatch.setattr(
        sys.modules["rclpy.executors"],
        "SingleThreadedExecutor",
        RetryableShutdownExecutor,
    )
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")
    controller = ros2_mcp_server.get_runtime_controller()
    controller.discover_robot("hospital-amr")
    executor = RetryableShutdownExecutor.instances[0]
    owned_thread = controller._adapter._runtime_owner_close.__self__._thread
    assert owned_thread is not None
    try:
        assert ros2_mcp_server.close_runtime_controller() is False
        assert owned_thread.is_alive()
        with pytest.raises(ros2_mcp_server.RuntimeControllerError, match="CLEANUP_FAILED"):
            ros2_mcp_server.get_runtime_controller()

        assert ros2_mcp_server.close_runtime_controller() is True
        assert executor.shutdown_attempts == 2
        assert not owned_thread.is_alive()
        with ros2_mcp_server._controller_condition:
            assert ros2_mcp_server._controller is None
            assert ros2_mcp_server._controller_cleanup_failed is False
    finally:
        executor.stop.set()
        owned_thread.join(0.2)
        with ros2_mcp_server._controller_condition:
            ros2_mcp_server._controller = None
            ros2_mcp_server._evidence_store = None
            ros2_mcp_server._controller_closing = False
            ros2_mcp_server._controller_cleanup_failed = False
            ros2_mcp_server._controller_condition.notify_all()
