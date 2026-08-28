"""Private, owned activation and emergency command queues."""

from __future__ import annotations

import queue
import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Generic, TypeVar

from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.sequencer import (
    _ActivationIssuer,  # noqa: F401 - compatibility re-export for the adapter boundary
    _ActivationPermit,  # noqa: F401 - compatibility re-export for the adapter boundary
    _ActivationRejected,
    _SafetySequencer,
)

_T = TypeVar("_T")
_HARDWARE_CHANNEL_GUARD = object()
_QUEUE_CAPACITY = 16


@dataclass(slots=True)
class _CommandResult(Generic[_T]):
    done: threading.Event = field(default_factory=threading.Event)
    value: _T | None = None
    error: BaseException | None = None

    def wait(self) -> _T:
        self.done.wait()
        if self.error is not None:
            raise self.error
        return self.value  # type: ignore[return-value]


class _BoundedCommandWorker:
    """A fixed-capacity, prestarted owner for calls into an unknown transport."""

    __slots__ = (
        "_accepting",
        "_failed",
        "_lock",
        "_queue",
        "_stop",
        "_thread",
        "_thread_name",
    )

    def __init__(self, thread_name: str) -> None:
        self._queue: queue.Queue[tuple[Callable[[], object], _CommandResult[object]]] = queue.Queue(
            maxsize=_QUEUE_CAPACITY
        )
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._accepting = False
        self._failed = False
        self._thread_name = thread_name

    def start(self) -> bool:
        with self._lock:
            if self._thread is not None:
                return self._accepting and self._thread.is_alive() and not self._failed
            thread = threading.Thread(target=self._run, name=self._thread_name, daemon=False)
            try:
                thread.start()
            except Exception:
                self._failed = True
                return False
            self._thread = thread
            self._accepting = True
            return True

    def submit(self, command: Callable[[], _T]) -> _CommandResult[_T]:
        if not callable(command):
            raise _ActivationRejected("PROFILE_INVALID")
        result: _CommandResult[_T] = _CommandResult()
        with self._lock:
            thread = self._thread
            if not self._accepting or self._failed or thread is None or not thread.is_alive():
                raise _ActivationRejected("UNSAFE_STATE")
            try:
                self._queue.put_nowait((command, result))
            except queue.Full:
                self._failed = True
                raise _ActivationRejected("UNSAFE_STATE") from None
        return result

    def close(self, timeout: float) -> bool:
        with self._lock:
            self._accepting = False
            self._stop.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, timeout))
        return thread is None or (not thread.is_alive() and self._queue.empty() and not self._failed)

    def _run(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                command, result = self._queue.get(timeout=0.01)
            except queue.Empty:
                continue
            try:
                result.value = command()
            except BaseException as exc:
                result.error = exc
                with self._lock:
                    self._failed = True
            finally:
                result.done.set()
                self._queue.task_done()


class _EmergencyStopChannel(ABC):
    """A private bounded enqueue whose worker owns all transport calls."""

    __slots__ = (
        "_hardware_verified",
        "_sequencer",
        "_verified",
        "_worker",
        "__weakref__",
    )

    def __init__(self, *, hardware_verified: bool, construction_guard: object = None) -> None:
        self._hardware_verified = hardware_verified is True and construction_guard is _HARDWARE_CHANNEL_GUARD
        self._sequencer: _SafetySequencer | None = None
        self._verified = False
        self._worker = _BoundedCommandWorker("agent-ros-emergency")

    def _bind(self, sequencer: _SafetySequencer) -> None:
        if not isinstance(sequencer, _SafetySequencer):
            raise _ActivationRejected("PROFILE_INVALID")
        if self._sequencer is not None and self._sequencer is not sequencer:
            raise _ActivationRejected("PROFILE_INVALID")
        sequencer._start()
        self._sequencer = sequencer

    def _verify(self, mode: str) -> None:
        if mode not in {"simulation", "hardware"}:
            raise _ActivationRejected("PROFILE_INVALID")
        try:
            available = self._preflight()
        except Exception:
            raise _ActivationRejected("PROFILE_INVALID") from None
        if available is not True or (mode == "hardware" and not self._hardware_verified) or not self._worker.start():
            raise _ActivationRejected("PROFILE_INVALID")
        self._verified = True

    def _stop(self, timeout: float = 1.0) -> EmergencyStopResult:
        sequencer = self._sequencer
        if sequencer is None or not self._verified:
            raise _ActivationRejected("UNSAFE_STATE")
        return sequencer.latch_and_quiesce(
            self._submit_zero,
            timeout,
        )

    def _submit_zero(self) -> None:
        self._worker.submit(self._execute_zero_disable)

    def _close(self, timeout: float) -> bool:
        return self._worker.close(timeout)

    @abstractmethod
    def _preflight(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def _enqueue_zero_disable(self) -> None:
        """Execute the fixed zero/disable operation on the owned worker."""
        raise NotImplementedError

    def _execute_zero_disable(self) -> None:
        self._enqueue_zero_disable()
