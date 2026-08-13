"""Private, owned activation and emergency command queues."""

from __future__ import annotations

import queue
import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Generic, TypeVar


_T = TypeVar("_T")
_PERMIT_CONSTRUCTION_GUARD = object()
_HARDWARE_CHANNEL_GUARD = object()
_QUEUE_CAPACITY = 16


class _ActivationRejected(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


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
        "_accepting", "_failed", "_lock", "_queue", "_stop", "_thread", "_thread_name",
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
        return thread is None or (
            not thread.is_alive() and self._queue.empty() and not self._failed
        )

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


class _ActivationPermit:
    """Issuer-bound reservation; callers cannot construct a valid instance."""

    __slots__ = ("_generation", "_issuer", "_nonce")

    def __init__(
        self,
        issuer: "_ActivationIssuer",
        generation: int,
        nonce: object,
        construction_guard: object,
    ) -> None:
        if construction_guard is not _PERMIT_CONSTRUCTION_GUARD:
            raise TypeError("activation permits are issuer-owned")
        self._issuer = issuer
        self._generation = generation
        self._nonce = nonce

    def _activate(self, enqueue: Callable[[], _T]) -> _T:
        return self._issuer._activate(self, enqueue)


class _ActivationIssuer:
    """Atomically orders activation submission against safety invalidation."""

    __slots__ = ("_generation", "_latched", "_lock", "_nonce", "_worker")

    def __init__(self) -> None:
        self._generation = 0
        self._latched = False
        self._lock = threading.Lock()
        self._nonce = object()
        self._worker = _BoundedCommandWorker("agent-ros-activation")

    def _start(self) -> None:
        if not self._worker.start():
            raise _ActivationRejected("PROFILE_INVALID")

    def _close(self, timeout: float) -> bool:
        return self._worker.close(timeout)

    def _issue(self) -> _ActivationPermit:
        with self._lock:
            if self._latched:
                raise _ActivationRejected("ESTOP_LATCHED")
            return _ActivationPermit(
                self,
                self._generation,
                self._nonce,
                _PERMIT_CONSTRUCTION_GUARD,
            )

    def _activate(self, permit: object, enqueue: Callable[[], _T]) -> _T:
        with self._lock:
            self._require_owned(permit)
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")

            def guarded_enqueue() -> _T:
                with self._lock:
                    if self._latched or permit._generation != self._generation:
                        raise _ActivationRejected("ESTOP_LATCHED")
                return enqueue()

            result = self._worker.submit(guarded_enqueue)
        return result.wait()

    def _activate_owned(self, permit: object, enqueue: Callable[[], _T]) -> _T:
        """Run one repository-owned, fixed-memory queue insertion atomically."""
        with self._lock:
            self._require_owned(permit)
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")
            return enqueue()

    def _is_current(self, permit: object) -> bool:
        with self._lock:
            try:
                self._require_owned(permit)
            except _ActivationRejected:
                return False
            return not self._latched and permit._generation == self._generation

    def _require_current(self, permit: object) -> None:
        with self._lock:
            self._require_owned(permit)
            if self._latched or permit._generation != self._generation:
                raise _ActivationRejected("ESTOP_LATCHED")

    def _invalidate_and_submit(self, submit: Callable[[], None]) -> None:
        with self._lock:
            self._generation += 1
            self._latched = True
            submit()

    def _require_owned(self, permit: object) -> None:
        if (
            type(permit) is not _ActivationPermit
            or permit._issuer is not self
            or permit._nonce is not self._nonce
        ):
            raise _ActivationRejected("PROFILE_INVALID")


class _EmergencyStopChannel(ABC):
    """A private bounded enqueue whose worker owns all transport calls."""

    __slots__ = (
        "_hardware_verified", "_issuer", "_verified", "_worker", "__weakref__",
    )

    def __init__(self, *, hardware_verified: bool, construction_guard: object = None) -> None:
        self._hardware_verified = (
            hardware_verified is True and construction_guard is _HARDWARE_CHANNEL_GUARD
        )
        self._issuer: _ActivationIssuer | None = None
        self._verified = False
        self._worker = _BoundedCommandWorker("agent-ros-emergency")

    def _bind(self, issuer: _ActivationIssuer) -> None:
        if not isinstance(issuer, _ActivationIssuer):
            raise _ActivationRejected("PROFILE_INVALID")
        if self._issuer is not None and self._issuer is not issuer:
            raise _ActivationRejected("PROFILE_INVALID")
        issuer._start()
        self._issuer = issuer

    def _verify(self, mode: str) -> None:
        if mode not in {"simulation", "hardware"}:
            raise _ActivationRejected("PROFILE_INVALID")
        try:
            available = self._preflight()
        except Exception:
            raise _ActivationRejected("PROFILE_INVALID") from None
        if (
            available is not True
            or (mode == "hardware" and not self._hardware_verified)
            or not self._worker.start()
        ):
            raise _ActivationRejected("PROFILE_INVALID")
        self._verified = True

    def _stop(self) -> None:
        issuer = self._issuer
        if issuer is None or not self._verified:
            raise _ActivationRejected("UNSAFE_STATE")
        try:
            issuer._invalidate_and_submit(
                lambda: self._worker.submit(self._execute_zero_disable)
            )
        except _ActivationRejected:
            raise
        except Exception:
            raise _ActivationRejected("UNSAFE_STATE") from None

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
