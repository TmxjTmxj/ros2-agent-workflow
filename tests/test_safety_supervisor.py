from __future__ import annotations

import threading

import pytest
from agent_ros.safety.supervisor import SafetySupervisor


def test_supervisor_owner_stops_and_joins_started_worker(supervisor_owner):
    supervisor = supervisor_owner(
        SafetySupervisor(
            clock=lambda: 0.0,
            deadline=lambda: None,
            on_expired=lambda: None,
            poll_interval=0.001,
        )
    )

    supervisor.start()

    assert supervisor.running


def test_supervisor_start_failure_does_not_register_an_unstarted_thread(monkeypatch):
    supervisor = SafetySupervisor(
        clock=lambda: 0.0,
        deadline=lambda: None,
        on_expired=lambda: None,
    )

    def fail_start(_thread):
        raise RuntimeError("controlled start failure")

    monkeypatch.setattr(threading.Thread, "start", fail_start)

    with pytest.raises(RuntimeError, match="controlled start failure"):
        supervisor.start()

    supervisor.stop()
    assert supervisor.join(timeout=0.1)
