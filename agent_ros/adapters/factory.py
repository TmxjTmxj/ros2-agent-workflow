"""Repository-owned production composition for structured rclpy adapters."""

from __future__ import annotations

import threading
import time

from agent_ros.adapters.base import AdapterError, RobotAdapter
from agent_ros.adapters.hospital import HospitalCaseAdapter, HospitalLifecycleClient
from agent_ros.adapters.nav2 import Nav2Adapter, RclpyNav2Transport
from agent_ros.adapters.twist import RclpyTwistTransport, TwistAdapter
from agent_ros.profiles.models import TWIST_TYPE, RobotProfile


class RclpyAdapterFactory:
    """Own one rclpy context, node, executor, and adapter per controller."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._context = None
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._adapter: RobotAdapter | None = None
        self._hospital_client: HospitalLifecycleClient | None = None
        self._executor_shutdown = True
        self._thread_joined = True
        self._node_removed = True
        self._node_destroyed = True
        self._context_shutdown = True
        self._closed = False

    def __call__(self, profile: RobotProfile) -> RobotAdapter:
        with self._lock:
            if self._closed or self._adapter is not None:
                raise AdapterError("PROFILE_INVALID")
            deadline = time.monotonic() + 1.0
            if self._owns_resources():
                if not self._close_locked(max(0.0, deadline - time.monotonic())):
                    raise AdapterError("CLEANUP_FAILED")
                self._reset_resources()
            if not isinstance(profile, RobotProfile):
                raise AdapterError("PROFILE_INVALID")
            constructor = self._adapter_constructor(profile)
            try:
                if profile.adapter.kind == "hospital_delivery":
                    client = HospitalLifecycleClient()
                    self._hospital_client = client
                    adapter = constructor(client)
                else:
                    self._start_runtime()
                    adapter = constructor(self._node)
                adapter._bind_runtime_owner(self.close)
                self._adapter = adapter
                return adapter
            except AdapterError:
                if not self._close_locked(max(0.0, deadline - time.monotonic())):
                    raise AdapterError("CLEANUP_FAILED") from None
                self._reset_resources()
                raise
            except Exception:
                if not self._close_locked(max(0.0, deadline - time.monotonic())):
                    raise AdapterError("CLEANUP_FAILED") from None
                self._reset_resources()
                raise AdapterError("PROFILE_INVALID") from None

    def close(self, timeout: float = 1.0) -> bool:
        if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
            return False
        with self._lock:
            if self._closed:
                return True
            successful = self._close_locked(max(0.0, float(timeout)))
            if successful:
                self._closed = True
            return successful

    def _adapter_constructor(self, profile: RobotProfile):
        estop_topic = profile.safety.estop_topic
        if not isinstance(estop_topic, str) or not estop_topic:
            raise AdapterError("PROFILE_INVALID")
        if profile.adapter.kind == "twist":
            command = profile.interfaces.command
            odometry = profile.interfaces.odometry
            if (
                command is None
                or command.type != TWIST_TYPE
                or command.topic is None
                or odometry is None
                or odometry.topic is None
                or profile.safety.heartbeat_timeout is None
            ):
                raise AdapterError("PROFILE_INVALID")
            return lambda node: TwistAdapter(
                profile,
                RclpyTwistTransport(
                    node,
                    command.topic,
                    odometry.topic,
                    estop_topic,
                    limits=profile.limits,
                    stale_after=profile.safety.heartbeat_timeout,
                ),
            )
        if profile.adapter.kind == "hospital_delivery":
            if (
                profile.name != "hospital-amr"
                or profile.mode != "simulation"
                or profile.namespace != "/hospital_amr"
            ):
                raise AdapterError("PROFILE_INVALID")
            return lambda client: HospitalCaseAdapter(client)
        if profile.adapter.kind == "nav2":
            navigation = profile.interfaces.navigation
            command = profile.interfaces.command
            if (
                navigation is None
                or navigation.action is None
                or command is None
                or command.type != TWIST_TYPE
                or command.topic is None
            ):
                raise AdapterError("PROFILE_INVALID")
            return lambda node: Nav2Adapter(
                profile,
                RclpyNav2Transport(
                    node,
                    navigation.action,
                    command.topic,
                    estop_topic,
                ),
            )
        raise AdapterError("PROFILE_INVALID")

    def _start_runtime(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
        except ImportError:
            raise AdapterError("PROFILE_INVALID") from None
        context = Context()
        context.init(args=None)
        self._context = context
        self._context_shutdown = False
        node = rclpy.create_node("agent_ros_runtime", context=context)
        self._node = node
        self._node_destroyed = False
        executor = SingleThreadedExecutor(context=context)
        self._executor = executor
        self._executor_shutdown = False
        executor.add_node(node)
        self._node_removed = False
        entered = threading.Event()

        def spin() -> None:
            entered.set()
            executor.spin()

        thread = threading.Thread(
            target=spin,
            name="agent-ros-rclpy-executor",
            daemon=False,
        )
        try:
            thread.start()
        except Exception:
            raise AdapterError("PROFILE_INVALID") from None
        self._thread = thread
        self._thread_joined = False
        if not entered.wait(1.0) or not thread.is_alive():
            raise AdapterError("PROFILE_INVALID")

    def _close_locked(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        client = self._hospital_client
        if client is not None:
            if not client.close(max(0.0, deadline - time.monotonic())):
                return False
            self._hospital_client = None
        executor = self._executor
        thread = self._thread
        node = self._node
        context = self._context
        if executor is not None and not self._executor_shutdown:
            try:
                result = executor.shutdown(timeout_sec=max(0.0, deadline - time.monotonic()))
                if result is False:
                    return False
                self._executor_shutdown = True
            except Exception:
                return False
        if thread is not None and not self._thread_joined:
            if thread is threading.current_thread():
                return False
            thread.join(max(0.0, deadline - time.monotonic()))
            if thread.is_alive():
                return False
            self._thread_joined = True
        if executor is not None and node is not None and not self._node_removed:
            try:
                executor.remove_node(node)
                self._node_removed = True
            except Exception:
                return False
        if node is not None and not self._node_destroyed:
            try:
                node.destroy_node()
                self._node_destroyed = True
            except Exception:
                return False
        if context is not None and not self._context_shutdown:
            try:
                try_shutdown = getattr(context, "try_shutdown", None)
                if callable(try_shutdown):
                    try_shutdown()
                else:
                    context.shutdown()
                self._context_shutdown = True
            except Exception:
                return False
        return True

    def _owns_resources(self) -> bool:
        return any(
            resource is not None
            for resource in (
                self._context,
                self._node,
                self._executor,
                self._thread,
                self._hospital_client,
            )
        )

    def _reset_resources(self) -> None:
        self._context = None
        self._node = None
        self._executor = None
        self._thread = None
        self._adapter = None
        self._hospital_client = None
        self._executor_shutdown = True
        self._thread_joined = True
        self._node_removed = True
        self._node_destroyed = True
        self._context_shutdown = True


__all__ = ("RclpyAdapterFactory",)
