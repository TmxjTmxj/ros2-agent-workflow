"""Reusable safety and lifecycle assertions for deterministic adapters."""

from __future__ import annotations

from collections.abc import Mapping

from agent_ros.adapters.base import AdapterError, AdapterProbe, AdapterStatus, Observation, RobotAdapter

from tests.support.runtime_owners import bind_simulation_activation


def assert_adapter_contract(adapter: RobotAdapter, task: object, sources: tuple[str, ...]) -> None:
    """Assert the minimum fail-closed lifecycle shared by reviewed adapters."""
    assert isinstance(adapter, RobotAdapter), "adapter must implement RobotAdapter"
    assert isinstance(sources, tuple) and sources, "contract needs one evidence source"

    _assert_start_rejects_missing_permit(adapter, task)
    adapter.validate()
    assert isinstance(adapter.probe(), AdapterProbe), "probe must return AdapterProbe"

    permit = bind_simulation_activation(adapter)
    started = adapter.start(task, permit)
    assert isinstance(started, AdapterStatus), "start must return AdapterStatus"

    snapshot = adapter.freeze_terminal_evidence(sources, started)
    assert isinstance(snapshot, Mapping), "terminal evidence must be a mapping"
    for source in sources:
        observation = snapshot.get(source)
        assert isinstance(observation, Observation), "terminal evidence must contain Observation values"
        assert observation.source == source, "terminal evidence source must match its key"
        assert isinstance(observation.values, Mapping), "terminal evidence values must be a mapping"

    assert isinstance(adapter.cancel(), AdapterStatus), "cancel must return AdapterStatus"
    assert isinstance(adapter.cancel(), AdapterStatus), "repeated cancel must remain safe"
    adapter.stop()
    adapter.stop()

    emergency = adapter._emergency_stop(timeout=0.5)
    assert emergency.latched, "emergency stop must latch the adapter"
    try:
        adapter.start(task, permit)
    except AdapterError as exc:
        assert exc.code == "ESTOP_LATCHED", "latched adapter must reject a prior activation permit"
    else:
        raise AssertionError("latched adapter accepted an activation permit")

    assert adapter.close(timeout=0.5), "adapter close must finish within its bound"


def _assert_start_rejects_missing_permit(adapter: RobotAdapter, task: object) -> None:
    try:
        adapter.start(task)
    except AdapterError:
        return
    raise AssertionError("adapter accepted start without an activation permit")
