"""Read-only ROS graph collection using fixed, non-shell CLI arguments."""

from __future__ import annotations

import re
import subprocess
from collections.abc import Callable, Sequence
from typing import Any

from agent_ros.discovery.models import Endpoint, GraphSnapshot
from agent_ros.errors import DiscoveryError


Runner = Callable[[Sequence[str]], object]
_TYPED_LINE = re.compile(r"^(?P<name>\S+)\s+\[(?P<types>[^\]]+)\]\s*$")


class RosGraphProbe:
    """Collect a graph snapshot without accepting a caller-provided command."""

    def __init__(self, runner: Runner | None = None) -> None:
        self._runner = runner or _run_ros2

    def probe(self) -> GraphSnapshot:
        nodes = tuple(line for line in self._output(("ros2", "node", "list")).splitlines() if line)
        topics = _parse_typed_lines(self._output(("ros2", "topic", "list", "-t")))
        actions = _parse_typed_lines(self._output(("ros2", "action", "list", "-t")))
        services = _parse_typed_lines(self._output(("ros2", "service", "list", "-t")))
        endpoints = {
            topic: _parse_endpoints(self._output(("ros2", "topic", "info", topic, "--verbose")))
            for topic in topics
        }
        return GraphSnapshot(
            nodes=nodes,
            topics=topics,
            services=services,
            actions=actions,
            topic_endpoints=endpoints,
        )

    def _output(self, argv: tuple[str, ...]) -> str:
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


def _run_ros2(argv: Sequence[str]) -> str:
    completed = subprocess.run(
        list(argv),
        check=True,
        capture_output=True,
        text=True,
        shell=False,
    )
    return completed.stdout


def _parse_typed_lines(output: str) -> dict[str, tuple[str, ...]]:
    entries: dict[str, tuple[str, ...]] = {}
    for line in output.splitlines():
        match = _TYPED_LINE.match(line.strip())
        if match is None:
            continue
        entries[match.group("name")] = tuple(
            item.strip() for item in match.group("types").split(",") if item.strip()
        )
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
    records.append(Endpoint(
        node_name=node_name,
        node_namespace=current.get("Node namespace", ""),
        gid=gid,
        endpoint_type=endpoint_type.lower(),
    ))
