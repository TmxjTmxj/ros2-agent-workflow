from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Generic, TypeVar

import pytest

from agent_ros.adapters._safety import _ActivationRejected
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.sequencer import _SafetySequencer


_T = TypeVar("_T")


class ThreadCall(Generic[_T]):
    def __init__(self, function: Callable[[], _T]) -> None:
        self._value: _T | None = None
        self._error: BaseException | None = None

        def invoke() -> None:
            try:
                self._value = function()
            except BaseException as exc:
                self._error = exc

        self._thread = threading.Thread(target=invoke)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def result(self, timeout: float) -> _T:
        self._thread.join(timeout)
        assert not self._thread.is_alive(), "test call did not finish before its deadline"
        if self._error is not None:
            raise self._error
        return self._value  # type: ignore[return-value]


def started_sequencer() -> _SafetySequencer:
    sequencer = _SafetySequencer()
    assert sequencer.start()
    return sequencer


def test_estop_waits_for_recorded_inflight_command_then_succeeds():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    permit = sequencer.issue()
    call = ThreadCall(
        lambda: sequencer.submit(
            permit,
            lambda: (entered.set(), release.wait())[0],
            timeout=0.5,
        )
    )
    try:
        assert entered.wait(0.2)
        stop = ThreadCall(
            lambda: sequencer.latch_and_quiesce(lambda: None, timeout=0.2)
        )
        assert stop.is_alive()
        release.set()
        result = stop.result(0.2)
        assert result == EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
        call.result(0.2)
    finally:
        release.set()
        assert sequencer.close(0.2)


def test_estop_returns_degraded_result_when_inflight_command_misses_deadline():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    call = ThreadCall(
        lambda: sequencer.submit(
            sequencer.issue(),
            lambda: (entered.set(), release.wait())[0],
            timeout=0.5,
        )
    )
    try:
        assert entered.wait(0.2)
        began = time.monotonic()

        result = sequencer.latch_and_quiesce(lambda: None, timeout=0.02)

        elapsed = time.monotonic() - began
        assert 0.01 <= elapsed < 0.2
        assert result == EmergencyStopResult(
            True, False, True, "TRANSPORT_UNQUIESCED"
        )
    finally:
        release.set()
        call.result(0.2)
        assert sequencer.close(0.2)


def test_latch_rejects_every_queued_command_without_invoking_it():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    invoked: list[str] = []
    active = ThreadCall(
        lambda: sequencer.submit(
            sequencer.issue(),
            lambda: (entered.set(), release.wait())[0],
            timeout=0.5,
        )
    )
    queued_started = threading.Event()
    queued = ThreadCall(
        lambda: (queued_started.set(), sequencer.submit(
            sequencer.issue(), lambda: invoked.append("stale"), timeout=0.5
        ))[1]
    )
    try:
        assert entered.wait(0.2)
        assert queued_started.wait(0.2)
        deadline = time.monotonic() + 0.5
        while sequencer.pending_count < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert sequencer.pending_count == 1
        result = sequencer.latch_and_quiesce(lambda: None, timeout=0.02)

        with pytest.raises(_ActivationRejected, match="ESTOP_LATCHED"):
            queued.result(1.0)
        assert invoked == []
        assert result.code == "TRANSPORT_UNQUIESCED"
    finally:
        release.set()
        active.result(0.2)
        assert sequencer.close(0.2)


def test_stale_and_foreign_permits_are_rejected():
    owner = started_sequencer()
    foreign = started_sequencer()
    stale = owner.issue()
    try:
        owner.latch_and_quiesce(lambda: None, timeout=0.1)

        with pytest.raises(_ActivationRejected, match="ESTOP_LATCHED"):
            owner.submit(stale, lambda: None, timeout=0.1)
        with pytest.raises(_ActivationRejected, match="PROFILE_INVALID"):
            owner.submit(foreign.issue(), lambda: None, timeout=0.1)
        with pytest.raises(_ActivationRejected, match="ESTOP_LATCHED"):
            owner.issue()
    finally:
        assert owner.close(0.2)
        assert foreign.close(0.2)


def test_activation_queue_has_capacity_sixteen_and_fails_closed_when_full():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    active = ThreadCall(
        lambda: sequencer.submit(
            sequencer.issue(),
            lambda: (entered.set(), release.wait())[0],
            timeout=0.5,
        )
    )
    queued: list[ThreadCall[None]] = []
    try:
        assert entered.wait(0.2)
        for _ in range(16):
            queued.append(
                ThreadCall(
                    lambda: sequencer.submit(
                        sequencer.issue(), lambda: None, timeout=0.5
                    )
                )
            )
        deadline = time.monotonic() + 0.2
        while sequencer.pending_count != 16 and time.monotonic() < deadline:
            threading.Event().wait(0.001)
        assert sequencer.pending_count == 16

        with pytest.raises(_ActivationRejected, match="UNSAFE_STATE"):
            sequencer.submit(sequencer.issue(), lambda: None, timeout=0.1)

        sequencer.latch_and_quiesce(lambda: None, timeout=0.02)
        for call in queued:
            with pytest.raises(_ActivationRejected, match="ESTOP_LATCHED"):
                call.result(0.2)
    finally:
        release.set()
        active.result(0.2)
        assert sequencer.close(0.2)


def test_start_failure_is_reported_without_registering_a_worker():
    class StartFailedThread:
        daemon = False

        def start(self) -> None:
            raise RuntimeError("controlled start failure")

        def is_alive(self) -> bool:
            return False

    sequencer = _SafetySequencer(
        thread_factory=lambda **_kwargs: StartFailedThread()
    )

    assert not sequencer.start()
    with pytest.raises(_ActivationRejected, match="UNSAFE_STATE"):
        sequencer.issue()
    assert sequencer.close(0.1)


def test_dead_worker_rejects_submission():
    die = threading.Event()
    running = threading.Event()

    def dead_worker_factory(**_kwargs):
        return threading.Thread(
            target=lambda: (running.set(), die.wait()),
            name="controlled-dead-activation-worker",
            daemon=False,
        )

    sequencer = _SafetySequencer(thread_factory=dead_worker_factory)
    assert sequencer.start()
    assert running.wait(0.2)
    permit = sequencer.issue()
    die.set()
    deadline = time.monotonic() + 0.2
    while sequencer.worker_alive and time.monotonic() < deadline:
        threading.Event().wait(0.001)
    assert not sequencer.worker_alive

    with pytest.raises(_ActivationRejected, match="UNSAFE_STATE"):
        sequencer.submit(permit, lambda: None, timeout=0.1)
    assert not sequencer.close(0.1)


def test_repeated_estop_remains_latched_and_accepts_each_zero_intent():
    sequencer = started_sequencer()
    accepted: list[int] = []
    try:
        first = sequencer.latch_and_quiesce(
            lambda: accepted.append(1), timeout=0.1
        )
        second = sequencer.latch_and_quiesce(
            lambda: accepted.append(2), timeout=0.1
        )

        assert first == EmergencyStopResult(True, True, True, "ESTOP_LATCHED")
        assert second == first
        assert accepted == [1, 2]
    finally:
        assert sequencer.close(0.2)


def test_submit_result_wait_timeout_latches_and_rejects_later_activation():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    try:
        with pytest.raises(_ActivationRejected, match="TIMEOUT"):
            sequencer.submit(
                sequencer.issue(),
                lambda: (entered.set(), release.wait())[0],
                timeout=0.02,
            )
        assert entered.is_set()
        with pytest.raises(_ActivationRejected, match="ESTOP_LATCHED"):
            sequencer.issue()
    finally:
        release.set()
        assert sequencer.close(0.2)


def test_late_transport_completion_cannot_overwrite_timed_out_receipt():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    call = ThreadCall(
        lambda: sequencer.submit(
            sequencer.issue(),
            lambda: (entered.set(), release.wait(), "late")[2],
            timeout=0.02,
        )
    )
    try:
        assert entered.wait(0.2)
        receipt = sequencer._in_flight
        assert receipt is not None
        with pytest.raises(_ActivationRejected, match="TIMEOUT"):
            call.result(0.2)

        release.set()
        deadline = time.monotonic() + 0.2
        while sequencer._in_flight is not None and time.monotonic() < deadline:
            threading.Event().wait(0.001)

        assert sequencer._in_flight is None
        assert isinstance(receipt.error, _ActivationRejected)
        assert receipt.error.code == "TIMEOUT"
        assert receipt.value is None
    finally:
        release.set()
        assert sequencer.close(0.2)


def test_close_is_bounded_while_transport_is_blocked():
    sequencer = started_sequencer()
    entered = threading.Event()
    release = threading.Event()
    call = ThreadCall(
        lambda: sequencer.submit(
            sequencer.issue(),
            lambda: (entered.set(), release.wait())[0],
            timeout=0.5,
        )
    )
    assert entered.wait(0.2)
    began = time.monotonic()

    assert not sequencer.close(0.02)

    assert time.monotonic() - began < 0.2
    release.set()
    call.result(0.2)
    assert sequencer.close(0.2)


def test_activation_worker_is_non_daemon_and_close_stops_it():
    sequencer = started_sequencer()
    try:
        assert sequencer.submit(
            sequencer.issue(),
            lambda: threading.current_thread().daemon,
            timeout=0.1,
        ) is False
    finally:
        assert sequencer.close(0.2)
