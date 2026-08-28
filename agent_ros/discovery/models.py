"""Typed snapshots and reports for a read-only ROS graph inspection."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType


def _frozen_mapping(value: Mapping[str, tuple[str, ...]]) -> Mapping[str, tuple[str, ...]]:
    return MappingProxyType({key: tuple(item) for key, item in value.items()})


@dataclass(frozen=True, slots=True)
class Endpoint:
    """One graph endpoint, identified independently even if node names repeat."""

    node_name: str
    gid: str
    endpoint_type: str
    node_namespace: str = ""


@dataclass(frozen=True, slots=True)
class GraphSnapshot:
    nodes: tuple[str, ...] = ()
    topics: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    services: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    topic_endpoints: Mapping[str, tuple[Endpoint, ...]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "nodes", tuple(self.nodes))
        object.__setattr__(self, "topics", _frozen_mapping(self.topics))
        object.__setattr__(self, "services", _frozen_mapping(self.services))
        object.__setattr__(self, "actions", _frozen_mapping(self.actions))
        object.__setattr__(
            self,
            "topic_endpoints",
            MappingProxyType({key: tuple(item) for key, item in self.topic_endpoints.items()}),
        )


@dataclass(frozen=True, slots=True)
class Capability:
    name: str
    confidence: float
    evidence: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class DiscoveryReport:
    capabilities: tuple[Capability, ...]
    blocking_warnings: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    topic_types: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    action_types: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "capabilities", tuple(self.capabilities))
        object.__setattr__(self, "blocking_warnings", tuple(self.blocking_warnings))
        object.__setattr__(self, "warnings", tuple(self.warnings))
        object.__setattr__(self, "topic_types", _frozen_mapping(self.topic_types))
        object.__setattr__(self, "action_types", _frozen_mapping(self.action_types))

    @property
    def capability_names(self) -> tuple[str, ...]:
        return tuple(capability.name for capability in self.capabilities)
