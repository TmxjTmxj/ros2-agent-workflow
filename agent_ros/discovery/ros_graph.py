"""Read-only ROS graph collection through a killable native helper boundary."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor

from agent_ros.discovery.models import Endpoint, GraphSnapshot
from agent_ros.errors import DiscoveryError

Runner = Callable[[Sequence[str]], object]
_TYPED_LINE = re.compile(r"^(?P<name>\S+)\s+\[(?P<types>[^\]]+)\]\s*$")
_GRAPH_COMMAND_TIMEOUT = 6.0
_MAX_COMMAND_TOPICS = 16


class RosGraphProbe:
    """Collect a graph snapshot without accepting a caller-provided command."""

    def __init__(self, runner: Runner | None = None) -> None:
        self._runner = runner

    def probe(self) -> GraphSnapshot:
        if self._runner is None:
            return _run_native_helper()
        return self._probe_cli()

    def _probe_cli(self) -> GraphSnapshot:
        commands = {
            "nodes": ("ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"),
            "topics": ("ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
            "actions": ("ros2", "action", "list", "-t"),
            "services": ("ros2", "service", "list", "-t", "--no-daemon", "--spin-time", "0.5"),
        }
        with ThreadPoolExecutor(
            max_workers=_MAX_COMMAND_TOPICS,
            thread_name_prefix="agent-ros-graph",
        ) as pool:
            initial = {name: pool.submit(self._output, argv) for name, argv in commands.items()}
            output = {name: future.result() for name, future in initial.items()}
            nodes = tuple(line for line in output["nodes"].splitlines() if line)
            topics = _parse_typed_lines(output["topics"])
            actions = _parse_typed_lines(output["actions"])
            services = _parse_typed_lines(output["services"])
            command_topics = tuple(
                topic
                for topic, types in topics.items()
                if topic.rstrip("/").endswith("cmd_vel") and "geometry_msgs/msg/Twist" in types
            )
            if len(command_topics) > _MAX_COMMAND_TOPICS:
                raise DiscoveryError("too many command topics for bounded inspection")
            endpoint_futures = {
                topic: pool.submit(
                    self._output,
                    (
                        "ros2",
                        "topic",
                        "info",
                        topic,
                        "--verbose",
                        "--no-daemon",
                        "--spin-time",
                        "0.5",
                    ),
                )
                for topic in command_topics
            }
            endpoints = {topic: _parse_endpoints(future.result()) for topic, future in endpoint_futures.items()}
        return GraphSnapshot(
            nodes=nodes,
            topics=topics,
            services=services,
            actions=actions,
            topic_endpoints=endpoints,
        )

    def _output(self, argv: tuple[str, ...]) -> str:
        if self._runner is None:
            raise DiscoveryError("CLI graph test seam is unavailable")
        try:
            result = self._runner(argv)
        except (OSError, subprocess.SubprocessError) as exc:
            raise DiscoveryError("ROS graph probe failed") from exc
        if isinstance(result, str):
            return result
        stdout = getattr(result, "stdout", None)
        if isinstance(stdout, str):
            return stdout
        raise DiscoveryError("ROS graph probe runner returned no text output")


def _native_modules():
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from ros2action.api import get_action_names_and_types
    except ImportError as exc:
        raise DiscoveryError("native ROS graph API is unavailable") from exc
    return rclpy, Context, SingleThreadedExecutor, get_action_names_and_types


def _run_native_helper() -> GraphSnapshot:
    argv = (sys.executable, "-m", "agent_ros.discovery.native_probe")
    try:
        completed = subprocess.run(
            list(argv),
            check=False,
            capture_output=True,
            text=True,
            shell=False,
            timeout=6.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise DiscoveryError("native ROS graph helper failed") from exc
    if completed.returncode != 0:
        raise DiscoveryError("native ROS graph helper failed")
    try:
        decoder = json.JSONDecoder()
        document, end = decoder.raw_decode(completed.stdout.lstrip())
        if completed.stdout.lstrip()[end:].strip():
            raise ValueError
        return _snapshot_from_document(document)
    except (json.JSONDecodeError, TypeError, ValueError, DiscoveryError):
        raise DiscoveryError("native ROS graph helper returned invalid evidence") from None


def _probe_native() -> GraphSnapshot:
    """Read one bounded graph snapshot through one owned DDS participant."""
    context = None
    node = None
    executor = None
    node_added = False
    snapshot = None
    failure: BaseException | None = None
    cleanup_failed = False
    try:
        rclpy, Context, Executor, get_actions = _native_modules()
        context = Context()
        context.init(args=None)
        node = rclpy.create_node("agent_ros_graph_probe", context=context)
        executor = Executor(context=context)
        executor.add_node(node)
        node_added = True
        executor.spin_once(timeout_sec=0.5)
        nodes = tuple(_full_node_name(name, namespace) for name, namespace in node.get_node_names_and_namespaces())
        topics = _typed_mapping(node.get_topic_names_and_types())
        services = _typed_mapping(node.get_service_names_and_types())
        actions = _typed_mapping(get_actions(node=node))
        command_topics = tuple(
            topic
            for topic, types in topics.items()
            if topic.rstrip("/").endswith("cmd_vel") and "geometry_msgs/msg/Twist" in types
        )
        if len(command_topics) > _MAX_COMMAND_TOPICS:
            raise DiscoveryError("too many command topics for bounded inspection")
        endpoints = {
            topic: tuple(_native_endpoint(info) for info in node.get_publishers_info_by_topic(topic))
            for topic in command_topics
        }
        snapshot = GraphSnapshot(
            nodes=nodes,
            topics=topics,
            services=services,
            actions=actions,
            topic_endpoints=endpoints,
        )
    except BaseException as exc:
        failure = exc
    finally:
        if executor is not None and node is not None and node_added:
            try:
                executor.remove_node(node)
            except Exception:
                cleanup_failed = True
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                cleanup_failed = True
        if executor is not None:
            try:
                if executor.shutdown(timeout_sec=1.0) is False:
                    cleanup_failed = True
            except Exception:
                cleanup_failed = True
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                cleanup_failed = True
    if failure is not None or cleanup_failed or snapshot is None:
        if isinstance(failure, DiscoveryError):
            raise failure
        raise DiscoveryError("native ROS graph probe failed") from failure
    return snapshot


def _typed_mapping(entries) -> dict[str, tuple[str, ...]]:
    result: dict[str, tuple[str, ...]] = {}
    for name, types in entries:
        if not isinstance(name, str) or not name:
            raise DiscoveryError("native graph name is invalid")
        values = tuple(types)
        if not values or not all(isinstance(value, str) and value for value in values):
            raise DiscoveryError("native graph type is invalid")
        result[name] = values
    return result


def _full_node_name(name: object, namespace: object) -> str:
    if not isinstance(name, str) or not name or not isinstance(namespace, str):
        raise DiscoveryError("native node identity is invalid")
    prefix = namespace.rstrip("/")
    return f"{prefix}/{name}" if prefix else f"/{name}"


def _native_endpoint(info) -> Endpoint:
    name = getattr(info, "node_name", None)
    namespace = getattr(info, "node_namespace", None)
    endpoint_type = getattr(getattr(info, "endpoint_type", None), "name", None)
    gid = getattr(info, "endpoint_gid", None)
    if not isinstance(name, str) or not name or not isinstance(namespace, str) or not isinstance(endpoint_type, str):
        raise DiscoveryError("native endpoint identity is invalid")
    try:
        gid_text = bytes(gid).hex()
    except (TypeError, ValueError):
        raise DiscoveryError("native endpoint GID is invalid") from None
    if not gid_text:
        raise DiscoveryError("native endpoint GID is invalid")
    return Endpoint(name, gid_text, endpoint_type.lower(), namespace)


def _snapshot_document(snapshot: GraphSnapshot) -> dict[str, object]:
    return {
        "nodes": list(snapshot.nodes),
        "topics": {name: list(types) for name, types in snapshot.topics.items()},
        "services": {name: list(types) for name, types in snapshot.services.items()},
        "actions": {name: list(types) for name, types in snapshot.actions.items()},
        "topic_endpoints": {
            topic: [
                {
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "gid": endpoint.gid,
                    "endpoint_type": endpoint.endpoint_type,
                }
                for endpoint in endpoints
            ]
            for topic, endpoints in snapshot.topic_endpoints.items()
        },
    }


def _snapshot_from_document(document: object) -> GraphSnapshot:
    required = {"nodes", "topics", "services", "actions", "topic_endpoints"}
    if not isinstance(document, Mapping) or set(document) != required:
        raise DiscoveryError("native graph document is not closed")
    nodes = document["nodes"]
    if not isinstance(nodes, list) or not all(isinstance(node, str) and node.startswith("/") for node in nodes):
        raise DiscoveryError("native graph nodes are invalid")

    def types_map(value: object) -> dict[str, tuple[str, ...]]:
        if not isinstance(value, Mapping):
            raise DiscoveryError("native graph interfaces are invalid")
        result: dict[str, tuple[str, ...]] = {}
        for name, types in value.items():
            if (
                not isinstance(name, str)
                or not name.startswith("/")
                or not isinstance(types, list)
                or not types
                or not all(isinstance(item, str) and item for item in types)
            ):
                raise DiscoveryError("native graph interfaces are invalid")
            result[name] = tuple(types)
        return result

    topics = types_map(document["topics"])
    services = types_map(document["services"])
    actions = types_map(document["actions"])
    raw_endpoints = document["topic_endpoints"]
    if not isinstance(raw_endpoints, Mapping):
        raise DiscoveryError("native graph endpoints are invalid")
    endpoints: dict[str, tuple[Endpoint, ...]] = {}
    endpoint_keys = {"node_name", "node_namespace", "gid", "endpoint_type"}
    for topic, values in raw_endpoints.items():
        if not isinstance(topic, str) or topic not in topics or not isinstance(values, list):
            raise DiscoveryError("native graph endpoints are invalid")
        parsed = []
        for value in values:
            if not isinstance(value, Mapping) or set(value) != endpoint_keys:
                raise DiscoveryError("native graph endpoints are invalid")
            if not all(isinstance(value[key], str) for key in endpoint_keys):
                raise DiscoveryError("native graph endpoints are invalid")
            if (
                not value["node_name"]
                or not value["gid"]
                or value["endpoint_type"] not in {"publisher", "subscription"}
            ):
                raise DiscoveryError("native graph endpoints are invalid")
            parsed.append(
                Endpoint(
                    value["node_name"],
                    value["gid"],
                    value["endpoint_type"],
                    value["node_namespace"],
                )
            )
        endpoints[topic] = tuple(parsed)
    return GraphSnapshot(
        nodes=tuple(nodes),
        topics=topics,
        services=services,
        actions=actions,
        topic_endpoints=endpoints,
    )


def _run_ros2(argv: Sequence[str]) -> str:
    completed = subprocess.run(
        list(argv),
        check=True,
        capture_output=True,
        text=True,
        shell=False,
        timeout=_GRAPH_COMMAND_TIMEOUT,
    )
    return completed.stdout


def _parse_typed_lines(output: str) -> dict[str, tuple[str, ...]]:
    entries: dict[str, tuple[str, ...]] = {}
    for line in output.splitlines():
        match = _TYPED_LINE.match(line.strip())
        if match is None:
            continue
        entries[match.group("name")] = tuple(item.strip() for item in match.group("types").split(",") if item.strip())
    return entries


def _parse_endpoints(output: str) -> tuple[Endpoint, ...]:
    records: list[Endpoint] = []
    current: dict[str, str] = {}
    for line in output.splitlines():
        if ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        if key == "Node name" and current:
            _append_endpoint(records, current)
            current = {}
        if key in {"Node name", "Node namespace", "Endpoint type", "GID"}:
            current[key] = value
    _append_endpoint(records, current)
    return tuple(records)


def _append_endpoint(records: list[Endpoint], current: dict[str, str]) -> None:
    node_name = current.get("Node name")
    endpoint_type = current.get("Endpoint type")
    if node_name is None or endpoint_type is None:
        return
    # The GID is an identity, not a display label. Keep records separate even
    # where a CLI implementation omits it, rather than deduplicating nodes.
    gid = current.get("GID", "")
    records.append(
        Endpoint(
            node_name=node_name,
            node_namespace=current.get("Node namespace", ""),
            gid=gid,
            endpoint_type=endpoint_type.lower(),
        )
    )
