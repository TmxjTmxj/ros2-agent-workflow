"""Lifecycle ownership shared by tests that construct runtime workers."""

from __future__ import annotations

import pytest

from agent_ros.adapters.base import RobotAdapter
from agent_ros.runtime.controller import RuntimeController, RuntimeControllerError
from agent_ros.safety.supervisor import SafetySupervisor


@pytest.fixture(autouse=True)
def close_owned_runtime_threads(monkeypatch):
    runtimes: list[RuntimeController] = []
    supervisors: list[SafetySupervisor] = []
    adapters: list[RobotAdapter] = []
    runtime_init = RuntimeController.__init__
    supervisor_init = SafetySupervisor.__init__
    bind_runtime_safety = RobotAdapter._bind_runtime_safety

    def tracked_runtime_init(self, *args, **kwargs):
        runtime_init(self, *args, **kwargs)
        runtimes.append(self)

    def tracked_supervisor_init(self, *args, **kwargs):
        supervisor_init(self, *args, **kwargs)
        supervisors.append(self)

    def tracked_bind_runtime_safety(self, issuer):
        bind_runtime_safety(self, issuer)
        if self not in adapters:
            adapters.append(self)

    monkeypatch.setattr(RuntimeController, "__init__", tracked_runtime_init)
    monkeypatch.setattr(SafetySupervisor, "__init__", tracked_supervisor_init)
    monkeypatch.setattr(RobotAdapter, "_bind_runtime_safety", tracked_bind_runtime_safety)
    yield
    for runtime in reversed(runtimes):
        try:
            runtime.stop_runtime()
        except RuntimeControllerError:
            pass
    for supervisor in reversed(supervisors):
        supervisor.stop()
        supervisor.join(timeout=1.0)
    for adapter in reversed(adapters):
        adapter.close(timeout=1.0)
