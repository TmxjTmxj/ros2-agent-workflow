"""Repository-owned production composition for structured rclpy adapters."""

from __future__ import annotations

import threading
import time

from agent_ros.adapters.base import AdapterError, RobotAdapter
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
        self._closed = False

    def __call__(self, profile: RobotProfile) -> RobotAdapter:
        if not isinstance(profile, RobotProfile):
            raise AdapterError("PROFILE_INVALID")
        constructor = self._adapter_constructor(profile)
        with self._lock:
            if self._closed or self._adapter is not None:
                raise AdapterError("PROFILE_INVALID")
            try:
                self._start_runtime()
                adapter = constructor(self._node)
                adapter._bind_runtime_owner(self.close)
                self._adapter = adapter
                return adapter
            except AdapterError:
                self._close_locked(1.0)
                raise
            except Exception:
                self._close_locked(1.0)
                raise AdapterError("PROFILE_INVALID") from None

    def close(self, timeout: float = 1.0) -> bool:
        if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
            return False
        with self._lock:
            return self._close_locked(max(0.0, float(timeout)))

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
        node = rclpy.create_node("agent_ros_runtime", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        entered = threading.Event()

        def spin() -> None:
            entered.set()
            executor.spin()

        thread = threading.Thread(
            target=spin,
            name="agent-ros-rclpy-executor",
            daemon=False,
        )
        self._context = context
        self._node = node
        self._executor = executor
        self._thread = thread
        try:
            thread.start()
        except Exception:
            self._close_locked(1.0)
            raise AdapterError("PROFILE_INVALID") from None
        if not entered.wait(1.0) or not thread.is_alive():
            self._close_locked(1.0)
            raise AdapterError("PROFILE_INVALID")

    def _close_locked(self, timeout: float) -> bool:
        if self._closed:
            return True
        self._closed = True
        deadline = time.monotonic() + timeout
        successful = True
        executor = self._executor
        thread = self._thread
        node = self._node
        context = self._context
        if executor is not None:
            try:
                result = executor.shutdown(timeout_sec=max(0.0, deadline - time.monotonic()))
                successful = result is not False and successful
            except Exception:
                successful = False
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, deadline - time.monotonic()))
            successful = not thread.is_alive() and successful
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                successful = False
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                successful = False
        if context is not None:
            try:
                context.shutdown()
            except Exception:
                successful = False
        return successful


__all__ = ("RclpyAdapterFactory",)
