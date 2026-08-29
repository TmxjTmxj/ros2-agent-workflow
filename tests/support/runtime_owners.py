"""Explicit lifecycle owners for tests that start runtime workers."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from typing import TypeVar

import pytest
from agent_ros.adapters._safety import _ActivationIssuer
from agent_ros.adapters.base import RobotAdapter
from agent_ros.runtime.controller import RuntimeController
from agent_ros.safety.gateway import SafetyGateway
from agent_ros.safety.supervisor import SafetySupervisor

_T = TypeVar("_T")


def bind_simulation_activation(adapter: RobotAdapter) -> object:
    """Bind a deterministic simulation-only activation issuer for contract tests."""
    if not isinstance(adapter, RobotAdapter):
        raise TypeError("adapter must implement RobotAdapter")
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)
    adapter._validate_runtime_safety("simulation")
    return issuer._issue()


@contextmanager
def owned_adapter(adapter: RobotAdapter) -> Iterator[RobotAdapter]:
    """Close exactly one explicitly claimed adapter and expose close failure."""
    try:
        yield adapter
    finally:
        assert adapter.close(timeout=0.5)


@contextmanager
def owned_runtime(runtime: RuntimeController) -> Iterator[RuntimeController]:
    """Stop exactly one explicitly claimed controller without hiding errors."""
    try:
        yield runtime
    finally:
        runtime.stop_runtime()


@contextmanager
def owned_gateway(gateway: SafetyGateway) -> Iterator[SafetyGateway]:
    """Close exactly one explicitly claimed gateway and expose join failure."""
    try:
        yield gateway
    finally:
        assert gateway.close(timeout=0.5)


@contextmanager
def owned_supervisor(supervisor: SafetySupervisor) -> Iterator[SafetySupervisor]:
    """Stop and join exactly one explicitly claimed supervisor."""
    try:
        yield supervisor
    finally:
        supervisor.stop()
        assert supervisor.join(timeout=0.5)


def _owner_fixture(context_manager):
    with ExitStack() as stack:

        def own(value: _T) -> _T:
            return stack.enter_context(context_manager(value))

        yield own


@pytest.fixture
def adapter_owner():
    yield from _owner_fixture(owned_adapter)


@pytest.fixture
def runtime_owner():
    yield from _owner_fixture(owned_runtime)


@pytest.fixture
def gateway_owner():
    yield from _owner_fixture(owned_gateway)


@pytest.fixture
def supervisor_owner():
    yield from _owner_fixture(owned_supervisor)
