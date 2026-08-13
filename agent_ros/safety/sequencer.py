"""Linearizable ordering for activation commands and emergency latching."""

from __future__ import annotations

import math
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Generic, TypeVar

from agent_ros.safety.outcome import EmergencyStopResult


_T = TypeVar("_T")
_QUEUE_CAPACITY = 16
_PERMIT_CONSTRUCTION_GUARD = object()


class _ActivationRejected(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


@dataclass(slots=True)
class _Receipt(Generic[_T]):
    generation: int
    command: Callable[[], _T]
    done: threading.Event = field(default_factory=threading.Event)
    value: _T | None = None
    error: BaseException | None = None


class _ActivationPermit:
    """Sequencer-owned capability; no caller can construct a valid permit."""

    __slots__ = ("_generation", "_nonce", "_sequencer")

    def __init__(
        self,
        sequencer: "_SafetySequencer",
        generation: int,
        nonce: object,
        construction_guard: object,
    ) -> None:
        if construction_guard is not _PERMIT_CONSTRUCTION_GUARD:
            raise TypeError("activation permits are sequencer-owned")
        self._sequencer = sequencer
        self._generation = generation
        self._nonce = nonce

    def _activate(self, command: Callable[[], _T], timeout: float = 1.0) -> _T:
        return self._sequencer.submit(self, command, timeout)


class _SafetySequencer:
    """Own a bounded activation worker and linearize it with emergency latch."""

    __slots__ = (
        "_accepting",
        "_closed",
        "_condition",
        "_failed",
        "_generation",
        "_in_flight",
        "_latched",
        "_nonce",
        "_queue",
        "_thread",
        "_thread_factory",
    )

    def __init__(self, *, thread_factory: Callable[..., object] = threading.Thread) -> None:
        self._condition = threading.Condition()
        self._queue: queue.Queue[_Receipt[object]] = queue.Queue(
            maxsize=_QUEUE_CAPACITY
        )
        self._thread: object | None = None
        self._thread_factory = thread_factory
        self._accepting = False
        self._closed = False
        self._failed = False
        self._generation = 0
        self._latched = False
        self._nonce = object()
        self._in_flight: _Receipt[object] | None = None

    def start(self) -> bool:
        with self._condition:
            if self._thread is not None:
                return self._worker_ready_locked()
            if self._closed or self._failed:
                return False
            thread = self._thread_factory(
                target=self._run,
                name="agent-ros-activation",
                daemon=False,
            )
            try:
                thread.start()
            except Exception:
                self._failed = True
                return False
            if not thread.is_alive():
                self._failed = True
                return False
            self._thread = thread
            self._accepting = True
            self._condition.notify_all()
            return True

    def issue(self) -> _ActivationPermit:
        with self._condition:
            if self._closed or self._failed:
                raise _ActivationRejected("UNSAFE_STATE")
            if self._thread is not None and not self._worker_ready_locked():
                raise _ActivationRejected("UNSAFE_STATE")
            if self._latched:
                raise _ActivationRejected("ESTOP_LATCHED")
            return _ActivationPermit(
                self,
                self._generation,
                self._nonce,
                _PERMIT_CONSTRUCTION_GUARD,
            )

    def submit(
        self,
        permit: object,
        command: Callable[[], _T],
        timeout: float,
    ) -> _T:
        deadline = self._deadline(timeout)
        if not callable(command):
            raise _ActivationRejected("PROFILE_INVALID")
        with self._condition:
            self._require_owned_locked(permit)
            if not self._worker_ready_locked():
                raise _ActivationRejected("UNSAFE_STATE")
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")
            if self._queue.full():
                self._latch_locked("ESTOP_LATCHED")
                raise _ActivationRejected("UNSAFE_STATE")
            receipt: _Receipt[_T] = _Receipt(permit._generation, command)
            self._queue.put_nowait(receipt)  # type: ignore[arg-type]
            self._condition.notify()

        remaining = max(0.0, deadline - time.monotonic())
        if not receipt.done.wait(remaining):
            with self._condition:
                if not receipt.done.is_set():
                    self._latch_locked("ESTOP_LATCHED")
                    receipt.error = _ActivationRejected("TIMEOUT")
                    receipt.done.set()
                    self._condition.notify_all()
                    raise _ActivationRejected("TIMEOUT")
        if receipt.error is not None:
            raise receipt.error
        return receipt.value  # type: ignore[return-value]

    def submit_owned(self, permit: object, command: Callable[[], _T]) -> _T:
        """Run one reviewed, nonblocking in-memory operation at the boundary."""
        if not callable(command):
            raise _ActivationRejected("PROFILE_INVALID")
        with self._condition:
            self._require_owned_locked(permit)
            if not self._worker_ready_locked():
                raise _ActivationRejected("UNSAFE_STATE")
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")
            return command()

    def latch_and_quiesce(
        self,
        submit_zero: Callable[[], object],
        timeout: float,
    ) -> EmergencyStopResult:
        deadline = self._deadline(timeout)
        if not callable(submit_zero):
            return EmergencyStopResult(True, True, False, "SAFETY_COMMAND_REJECTED")
        with self._condition:
            self._latch_locked("ESTOP_LATCHED")
            try:
                submit_zero()
                safety_command_accepted = True
            except BaseException:
                safety_command_accepted = False
            while self._in_flight is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            activation_quiesced = self._in_flight is None

        if not activation_quiesced:
            code = "TRANSPORT_UNQUIESCED"
        elif safety_command_accepted:
            code = "ESTOP_LATCHED"
        else:
            code = "SAFETY_COMMAND_REJECTED"
        return EmergencyStopResult(
            True,
            activation_quiesced,
            safety_command_accepted,
            code,
        )

    def close(self, timeout: float) -> bool:
        deadline = self._deadline(timeout)
        with self._condition:
            self._closed = True
            self._accepting = False
            self._generation += 1
            self._latched = True
            self._reject_queued_locked("UNSAFE_STATE")
            thread = self._thread
            self._condition.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, deadline - time.monotonic()))
        with self._condition:
            if thread is None:
                return True
            return (
                not thread.is_alive()
                and self._queue.empty()
                and self._in_flight is None
                and not self._failed
            )

    def is_current(self, permit: object) -> bool:
        with self._condition:
            try:
                self._require_owned_locked(permit)
            except _ActivationRejected:
                return False
            return (
                self._worker_ready_locked()
                and not self._latched
                and permit._generation == self._generation
            )

    def require_current(self, permit: object) -> None:
        with self._condition:
            self._require_owned_locked(permit)
            if not self._worker_ready_locked():
                raise _ActivationRejected("UNSAFE_STATE")
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")

    @property
    def pending_count(self) -> int:
        with self._condition:
            return self._queue.qsize()

    @property
    def worker_alive(self) -> bool:
        with self._condition:
            return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._queue.empty() and self._accepting:
                    self._condition.wait()
                if self._queue.empty():
                    if not self._accepting:
                        return
                    continue
                receipt = self._queue.get_nowait()
                if self._latched or receipt.generation != self._generation:
                    receipt.error = _ActivationRejected("ESTOP_LATCHED")
                    receipt.done.set()
                    self._queue.task_done()
                    continue
                self._in_flight = receipt

            try:
                receipt.value = receipt.command()
            except BaseException as exc:
                receipt.error = exc
                with self._condition:
                    self._failed = True
                    self._accepting = False
                    self._generation += 1
                    self._latched = True
                    self._reject_queued_locked("UNSAFE_STATE")
            finally:
                with self._condition:
                    if self._in_flight is receipt:
                        self._in_flight = None
                    receipt.done.set()
                    self._queue.task_done()
                    self._condition.notify_all()

    def _latch_locked(self, queued_code: str) -> None:
        self._generation += 1
        self._latched = True
        self._reject_queued_locked(queued_code)
        self._condition.notify_all()

    def _reject_queued_locked(self, code: str) -> None:
        while not self._queue.empty():
            receipt = self._queue.get_nowait()
            receipt.error = _ActivationRejected(code)
            receipt.done.set()
            self._queue.task_done()

    def _worker_ready_locked(self) -> bool:
        thread = self._thread
        if (
            thread is None
            or not self._accepting
            or self._closed
            or self._failed
            or not thread.is_alive()
        ):
            if thread is not None and not thread.is_alive() and not self._closed:
                self._failed = True
                self._accepting = False
                self._latch_locked("UNSAFE_STATE")
            return False
        return True

    def _require_owned_locked(self, permit: object) -> None:
        if (
            type(permit) is not _ActivationPermit
            or permit._sequencer is not self
            or permit._nonce is not self._nonce
        ):
            raise _ActivationRejected("PROFILE_INVALID")

    @staticmethod
    def _deadline(timeout: float) -> float:
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout)
        ):
            raise _ActivationRejected("PROFILE_INVALID")
        return time.monotonic() + max(0.0, timeout)

    # Compatibility for Task 2's controller migration.
    def _start(self) -> None:
        if not self.start():
            raise _ActivationRejected("PROFILE_INVALID")

    def _issue(self) -> _ActivationPermit:
        return self.issue()

    def _activate(self, permit: object, command: Callable[[], _T]) -> _T:
        return self.submit(permit, command, 1.0)

    def _activate_owned(self, permit: object, command: Callable[[], _T]) -> _T:
        return self.submit_owned(permit, command)

    def _is_current(self, permit: object) -> bool:
        return self.is_current(permit)

    def _require_current(self, permit: object) -> None:
        self.require_current(permit)

    def _close(self, timeout: float) -> bool:
        return self.close(timeout)


# Temporary import-compatible name for the separately scheduled controller task.
_ActivationIssuer = _SafetySequencer
