"""Private activation and emergency-enqueue primitives for runtime adapters."""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import TypeVar


_T = TypeVar("_T")
_PERMIT_CONSTRUCTION_GUARD = object()


class _ActivationRejected(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


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
    """Own one short lock shared by activation enqueue and safety invalidation."""

    __slots__ = ("_generation", "_latched", "_lock", "_nonce")

    def __init__(self) -> None:
        self._generation = 0
        self._latched = False
        self._lock = threading.Lock()
        self._nonce = object()

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
            # The private adapter transport contract permits only a preflighted,
            # nonblocking enqueue while this lock is held.
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

    def _invalidate_and_enqueue(self, enqueue: Callable[[], None]) -> None:
        with self._lock:
            self._generation += 1
            self._latched = True
            # Safety channels are preflighted nonblocking enqueue/disable
            # primitives. No task, action, or arbitrary runner executes here.
            enqueue()

    def _require_owned(self, permit: object) -> None:
        if (
            type(permit) is not _ActivationPermit
            or permit._issuer is not self
            or permit._nonce is not self._nonce
        ):
            raise _ActivationRejected("PROFILE_INVALID")


class _EmergencyStopChannel(ABC):
    """Private, preflighted nonblocking zero/disable enqueue boundary."""

    __slots__ = ("_hardware_verified", "_issuer", "_verified")

    def __init__(self, *, hardware_verified: bool) -> None:
        self._hardware_verified = hardware_verified is True
        self._issuer: _ActivationIssuer | None = None
        self._verified = False

    def _bind(self, issuer: _ActivationIssuer) -> None:
        if not isinstance(issuer, _ActivationIssuer):
            raise _ActivationRejected("PROFILE_INVALID")
        if self._issuer is not None and self._issuer is not issuer:
            raise _ActivationRejected("PROFILE_INVALID")
        self._issuer = issuer

    def _verify(self, mode: str) -> None:
        if mode not in {"simulation", "hardware"}:
            raise _ActivationRejected("PROFILE_INVALID")
        try:
            available = self._preflight()
        except Exception:
            raise _ActivationRejected("PROFILE_INVALID") from None
        if available is not True or (mode == "hardware" and not self._hardware_verified):
            raise _ActivationRejected("PROFILE_INVALID")
        self._verified = True

    def _stop(self) -> None:
        issuer = self._issuer
        if issuer is None or not self._verified:
            raise _ActivationRejected("UNSAFE_STATE")
        try:
            issuer._invalidate_and_enqueue(self._enqueue_zero_disable)
        except _ActivationRejected:
            raise
        except Exception:
            raise _ActivationRejected("UNSAFE_STATE") from None

    @abstractmethod
    def _preflight(self) -> bool:
        """Return true only when enqueue is ready and is known not to wait."""
        raise NotImplementedError

    @abstractmethod
    def _enqueue_zero_disable(self) -> None:
        """Enqueue one zero/disable command without waiting for task locks."""
        raise NotImplementedError
