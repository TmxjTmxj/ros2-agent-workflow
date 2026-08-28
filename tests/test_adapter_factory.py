from __future__ import annotations

import json
import subprocess
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest
from agent_ros.adapters import hospital as hospital_adapter
from agent_ros.adapters.base import AdapterError
from agent_ros.adapters.factory import RclpyAdapterFactory
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.profiles.models import RobotProfile
from mcp_server import ros2_mcp_server


class FakePublisher:
    def publish(self, _message) -> None:
        return None


class FakeNode:
    instances = []

    def __init__(self) -> None:
        self.destroyed = False
        self.destroyed_clients = []
        self.destroyed_subscriptions = []
        type(self).instances.append(self)

    def create_publisher(self, _message_type, _name, _depth):
        return FakePublisher()

    def create_subscription(self, message_type, name, callback, _depth):
        if name == "/hospital_mission/status":
            callback(
                message_type(
                    data=json.dumps(
                        {
                            "state": "SUCCEEDED",
                            "elapsed": 30.0,
                            "stage_results": [
                                {"elapsed": 10.0},
                                {"elapsed": 20.0},
                                {"elapsed": 30.0},
                            ],
                            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                        }
                    )
                )
            )
        return callback

    def create_client(self, _service_type, name):
        return FakeClient(name)

    def destroy_client(self, client):
        self.destroyed_clients.append(client)
        return True

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)
        return True

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeContext:
    instances = []

    def __init__(self) -> None:
        self.initialized = False
        self.shutdown_called = False
        self.try_shutdown_calls = 0
        self.shutdown_calls = 0
        type(self).instances.append(self)

    def init(self, *, args=None) -> None:
        self.initialized = True

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        self.shutdown_called = True

    def try_shutdown(self) -> None:
        self.try_shutdown_calls += 1
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


class FakeFuture:
    def __init__(self, value) -> None:
        self._value = value

    def add_done_callback(self, callback) -> None:
        callback(self)

    def result(self):
        return self._value


class FakeClient:
    def __init__(self, name: str) -> None:
        self.name = name

    def wait_for_service(self, *, timeout_sec) -> bool:
        return timeout_sec > 0.0

    def call_async(self, _request):
        return FakeFuture(SimpleNamespace(success=True, message="OK"))


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


class GatedShutdownExecutor(FakeExecutor):
    instances = []
    shutdown_allowed = False

    def __init__(self, *, context) -> None:
        super().__init__(context=context)
        self.shutdown_attempts = 0

    def shutdown(self, *, timeout_sec) -> bool:
        self.shutdown_timeouts.append(timeout_sec)
        self.shutdown_attempts += 1
        if not type(self).shutdown_allowed:
            return False
        self.stop.set()
        return True


def _install_fake_ros(monkeypatch) -> None:
    FakeExecutor.instances.clear()
    FakeContext.instances.clear()
    FakeNode.instances.clear()

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
    std_msg.String = type(
        "String",
        (),
        {"__init__": lambda self, data="": setattr(self, "data", data)},
    )
    std.msg = std_msg
    std_srvs = ModuleType("std_srvs")
    std_srvs_srv = ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {"Request": type("Request", (), {})})
    std_srvs.srv = std_srvs_srv
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
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)


def _graph_cli(argv, **kwargs):
    command = tuple(argv)
    if command == (sys.executable, "-m", "agent_ros.discovery.native_probe"):
        payload = {
            "nodes": ["/base_controller"],
            "topics": {
                "/cmd_vel": ["geometry_msgs/msg/Twist"],
                "/odom": ["nav_msgs/msg/Odometry"],
            },
            "services": {},
            "actions": {},
            "topic_endpoints": {
                "/cmd_vel": [
                    {
                        "node_name": "base_controller",
                        "node_namespace": "/",
                        "gid": "01",
                        "endpoint_type": "publisher",
                    }
                ]
            },
        }
        return subprocess.CompletedProcess(argv, 0, json.dumps(payload), "")
    values = {
        ("ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"): "/base_controller\n",
        ("ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "0.5"): (
            "/cmd_vel [geometry_msgs/msg/Twist]\n/odom [nav_msgs/msg/Odometry]\n"
        ),
        ("ros2", "action", "list", "-t"): "",
        ("ros2", "service", "list", "-t", "--no-daemon", "--spin-time", "0.5"): "",
        ("ros2", "topic", "info", "/cmd_vel", "--verbose", "--no-daemon", "--spin-time", "0.5"): (
            "Node name: base_controller\nNode namespace: /\nEndpoint type: PUBLISHER\nGID: 01\n"
        ),
    }
    return subprocess.CompletedProcess(argv, 0, values[command], "")


def _trajectory_profile() -> RobotProfile:
    return RobotProfile.from_mapping(
        {
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
        }
    )


def _twist_profile() -> RobotProfile:
    return RobotProfile.from_mapping(
        {
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
        }
    )


def _hospital_profile() -> RobotProfile:
    return RobotProfile.from_mapping(
        {
            "name": "hospital-amr",
            "mode": "simulation",
            "namespace": "/hospital_amr",
            "frames": {"base": "base_link", "odom": "odom"},
            "adapter": {"kind": "hospital_delivery"},
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
            "observation_sources": ["odometry", "camera", "scan"],
        }
    )


def test_production_singleton_discovers_with_repository_owned_hospital_factory(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch)
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")

    controller = ros2_mcp_server.get_runtime_controller()
    result = controller.discover_robot("hospital-amr")

    assert result["state"] == "DISCOVERED"
    adapter_type = getattr(hospital_adapter, "HospitalCaseAdapter", None)
    assert adapter_type is not None
    assert isinstance(controller._adapter, adapter_type)
    assert len(FakeExecutor.instances) == 1
    assert ros2_mcp_server.close_runtime_controller() is True


def test_production_singleton_runs_only_the_fixed_hospital_lifecycle(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch)
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")
    fixed_calls = []

    def fixed_call(self, suffix, *, timeout, generation=None):
        fixed_calls.append((suffix, timeout))
        if suffix == ("mission-status",):
            return {
                "ok": True,
                "status": {
                    "state": "SUCCEEDED",
                    "elapsed": 30.0,
                    "stage_results": [
                        {"elapsed": 10.0},
                        {"elapsed": 20.0},
                        {"elapsed": 30.0},
                    ],
                },
            }
        return {"ok": True}

    monkeypatch.setattr(hospital_adapter.HospitalLifecycleClient, "_run_fixed", fixed_call)
    controller = ros2_mcp_server.get_runtime_controller()
    try:
        assert controller.discover_robot("hospital-amr")["state"] == "DISCOVERED"
        assert controller.validate_profile("hospital-amr")["state"] == "ARMED"
        assert controller.run_task("hospital-delivery")["state"] == "RUNNING"
        deadline = __import__("time").monotonic() + 2.0
        while __import__("time").monotonic() < deadline:
            if controller.task_status().get("adapter_state") == "succeeded":
                break
            __import__("time").sleep(0.01)

        assert (("start", "--timeout", "60"), 120.0) in fixed_calls
        assert all(suffix != ("mission-start",) for suffix, _timeout in fixed_calls)
        assert all(suffix != ("mission-status",) for suffix, _timeout in fixed_calls)
        assert len(FakeExecutor.instances) == 1
    finally:
        assert ros2_mcp_server.close_runtime_controller() is True


def test_production_factory_seals_hospital_client_and_owns_persistent_rclpy(monkeypatch):
    _install_fake_ros(monkeypatch)
    client_type = getattr(hospital_adapter, "HospitalLifecycleClient", None)
    adapter_type = getattr(hospital_adapter, "HospitalCaseAdapter", None)
    assert client_type is not None
    assert adapter_type is not None

    factory = RclpyAdapterFactory()
    adapter = factory(_hospital_profile())

    assert isinstance(adapter, adapter_type)
    assert type(adapter._client) is client_type
    assert len(FakeExecutor.instances) == 1
    assert factory._thread is not None and factory._thread.is_alive()
    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter_type(lambda _action: {"state": "running"})
    assert factory.close(0.5)
    assert not factory._thread.is_alive()


def test_production_factory_rejects_unsupported_trajectory_without_starting_ros(
    monkeypatch,
):
    _install_fake_ros(monkeypatch)
    factory = RclpyAdapterFactory()

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        factory(_trajectory_profile())

    assert FakeExecutor.instances == []
    assert factory.close(0.1)


def test_create_node_failure_releases_initialized_context(monkeypatch):
    _install_fake_ros(monkeypatch)

    def fail_create_node(_name, *, context):
        assert context.initialized
        raise RuntimeError("create node failed")

    monkeypatch.setattr(sys.modules["rclpy"], "create_node", fail_create_node)
    factory = RclpyAdapterFactory()

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        factory(_twist_profile())

    context = FakeContext.instances[0]
    assert context.try_shutdown_calls == 1
    assert context.shutdown_calls == 0
    assert factory.close(0.1)


def test_executor_construction_failure_releases_node_and_context(monkeypatch):
    _install_fake_ros(monkeypatch)

    class FailingExecutorConstructor:
        def __init__(self, *, context):
            assert context.initialized
            raise RuntimeError("executor construction failed")

    monkeypatch.setattr(
        sys.modules["rclpy.executors"],
        "SingleThreadedExecutor",
        FailingExecutorConstructor,
    )
    factory = RclpyAdapterFactory()

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        factory(_twist_profile())

    node = FakeNode.instances[0]
    context = FakeContext.instances[0]
    assert node.destroyed is True
    assert context.try_shutdown_calls == 1
    assert factory.close(0.1)


def test_add_node_failure_releases_unstarted_executor_node_and_context(monkeypatch):
    _install_fake_ros(monkeypatch)

    class FailingAddExecutor(FakeExecutor):
        instances = []

        def add_node(self, node) -> None:
            raise RuntimeError("add node failed")

    monkeypatch.setattr(
        sys.modules["rclpy.executors"],
        "SingleThreadedExecutor",
        FailingAddExecutor,
    )
    factory = RclpyAdapterFactory()

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        factory(_twist_profile())

    executor = FailingAddExecutor.instances[0]
    node = FakeNode.instances[0]
    context = FakeContext.instances[0]
    assert len(executor.shutdown_timeouts) == 1
    assert factory._thread is None
    assert node.destroyed is True
    assert context.try_shutdown_calls == 1
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


def test_failed_startup_cleanup_is_retried_before_any_new_runtime_is_created(
    monkeypatch,
):
    _install_fake_ros(monkeypatch)
    GatedShutdownExecutor.instances.clear()
    GatedShutdownExecutor.shutdown_allowed = False
    monkeypatch.setattr(
        sys.modules["rclpy.executors"],
        "SingleThreadedExecutor",
        GatedShutdownExecutor,
    )
    from agent_ros.adapters import factory as factory_module

    real_twist_adapter = factory_module.TwistAdapter
    construction_attempts = 0

    def fail_first_adapter_construction(*args, **kwargs):
        nonlocal construction_attempts
        construction_attempts += 1
        if construction_attempts == 1:
            raise RuntimeError("private ROS adapter construction detail")
        return real_twist_adapter(*args, **kwargs)

    monkeypatch.setattr(factory_module, "TwistAdapter", fail_first_adapter_construction)
    factory = RclpyAdapterFactory()
    first_thread = None
    try:
        with pytest.raises(AdapterError) as first_failure:
            factory(_twist_profile())
        assert first_failure.value.code == "CLEANUP_FAILED"
        assert len(GatedShutdownExecutor.instances) == 1
        first_executor = GatedShutdownExecutor.instances[0]
        first_thread = factory._thread
        assert first_thread is not None and first_thread.is_alive()
        old_resources = (factory._context, factory._node, factory._executor, factory._thread)

        with pytest.raises(AdapterError) as retry_failure:
            factory(_twist_profile())
        assert retry_failure.value.code == "CLEANUP_FAILED"
        assert len(GatedShutdownExecutor.instances) == 1
        assert (factory._context, factory._node, factory._executor, factory._thread) == old_resources
        assert first_executor.shutdown_attempts == 2
        assert construction_attempts == 1

        GatedShutdownExecutor.shutdown_allowed = True
        adapter = factory(_twist_profile())

        assert isinstance(adapter, TwistAdapter)
        assert len(GatedShutdownExecutor.instances) == 2
        assert construction_attempts == 2
        assert not first_thread.is_alive()
        assert factory.close(0.2)
    finally:
        GatedShutdownExecutor.shutdown_allowed = True
        for executor in GatedShutdownExecutor.instances:
            executor.stop.set()
        if first_thread is not None:
            first_thread.join(0.2)
        factory.close(0.2)


def test_singleton_keeps_partial_startup_poisoned_until_factory_cleanup_succeeds(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch)
    from agent_ros.adapters import factory as factory_module

    real_adapter_type = factory_module.HospitalCaseAdapter
    close_allowed = False
    close_attempts = 0

    class GatedCloseAdapter(real_adapter_type):
        """Construction succeeds; close fails until allowed, then delegates."""

        def close(self, timeout: float = 1.0) -> bool:
            nonlocal close_attempts
            close_attempts += 1
            if not close_allowed:
                return False
            return super().close(timeout)

    monkeypatch.setattr(factory_module, "HospitalCaseAdapter", GatedCloseAdapter)
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")
    controller = ros2_mcp_server.get_runtime_controller()
    try:
        controller.discover_robot("hospital-amr")
        assert ros2_mcp_server.close_runtime_controller() is False
        assert close_attempts == 1
        with pytest.raises(ros2_mcp_server.RuntimeControllerError) as poisoned:
            ros2_mcp_server.get_runtime_controller()
        assert poisoned.value.code == "CLEANUP_FAILED"

        close_allowed = True
        assert ros2_mcp_server.close_runtime_controller() is True
        assert close_attempts == 2
        with ros2_mcp_server._controller_condition:
            assert ros2_mcp_server._controller is None
            assert ros2_mcp_server._controller_cleanup_failed is False
    finally:
        close_allowed = True
        with ros2_mcp_server._controller_condition:
            ros2_mcp_server._controller = None
            ros2_mcp_server._evidence_store = None
            ros2_mcp_server._controller_closing = False
            ros2_mcp_server._controller_cleanup_failed = False
            ros2_mcp_server._controller_condition.notify_all()


def test_singleton_retry_clears_poison_only_after_owned_executor_is_reaped(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch)
    from agent_ros.adapters import factory as factory_module

    real_adapter_type = factory_module.HospitalCaseAdapter
    construction_attempts = 0
    close_allowed = False

    class FailFirstConstructionAdapter(real_adapter_type):
        """Construction fails once; the retry constructs and closes cleanly."""

        def __init__(self, client, *, clock=None):
            nonlocal construction_attempts
            construction_attempts += 1
            if construction_attempts == 1:
                raise AdapterError("PROFILE_INVALID")
            if clock is None:
                import time as _time

                clock = _time.monotonic
            super().__init__(client, clock=clock)

        def close(self, timeout: float = 1.0) -> bool:
            nonlocal close_allowed
            if not close_allowed:
                return False
            return super().close(timeout)

    monkeypatch.setattr(factory_module, "HospitalCaseAdapter", FailFirstConstructionAdapter)
    monkeypatch.setattr(subprocess, "run", _graph_cli)
    monkeypatch.setattr(ros2_mcp_server, "_RUNTIME_ROOT", tmp_path / "runtime")
    monkeypatch.setattr(ros2_mcp_server, "_EVIDENCE_ROOT", tmp_path / "evidence")
    controller = ros2_mcp_server.get_runtime_controller()
    try:
        with pytest.raises(ros2_mcp_server.RuntimeControllerError) as first_failure:
            controller.discover_robot("hospital-amr")
        assert first_failure.value.code == "PROFILE_INVALID"
        assert construction_attempts == 1

        close_allowed = True
        result = controller.discover_robot("hospital-amr")
        assert result["state"] == "DISCOVERED"
        assert construction_attempts == 2

        assert ros2_mcp_server.close_runtime_controller() is True
        with ros2_mcp_server._controller_condition:
            assert ros2_mcp_server._controller is None
            assert ros2_mcp_server._controller_cleanup_failed is False
    finally:
        close_allowed = True
        with ros2_mcp_server._controller_condition:
            ros2_mcp_server._controller = None
            ros2_mcp_server._evidence_store = None
            ros2_mcp_server._controller_closing = False
            ros2_mcp_server._controller_cleanup_failed = False
            ros2_mcp_server._controller_condition.notify_all()
