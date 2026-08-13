"""Fixed in-process hospital simulation adapter."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable, Mapping

from agent_ros.adapters._safety import _EmergencyStopChannel
from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    HospitalAction,
    Observation,
    RobotAdapter,
)


class HospitalSimulationRuntime:
    """Repository-owned fixed-enum lifecycle that cannot dispatch to hardware."""

    __slots__ = ("_commands", "_lock", "_state")

    def __init__(self) -> None:
        self._commands: deque[HospitalAction] = deque(maxlen=64)
        self._lock = threading.Lock()
        self._state = "idle"

    @property
    def commands(self) -> tuple[HospitalAction, ...]:
        with self._lock:
            return tuple(self._commands)

    def dispatch(self, action: HospitalAction) -> Mapping[str, object]:
        if not isinstance(action, HospitalAction):
            raise AdapterError("PROFILE_INVALID")
        with self._lock:
            if len(self._commands) == self._commands.maxlen:
                raise AdapterError("INTERNAL_ERROR")
            self._commands.append(action)
            if action is HospitalAction.START:
                self._state = "running"
            elif action is HospitalAction.CANCEL:
                self._state = "cancelled"
            elif action is HospitalAction.STOP:
                self._state = "stopped"
            return {"available": True, "state": self._state}

    def start_nowait(self) -> Mapping[str, object]:
        """Reserve START without ever waiting behind another simulation action."""
        if not self._lock.acquire(blocking=False):
            raise AdapterError("INTERNAL_ERROR")
        try:
            if len(self._commands) == self._commands.maxlen:
                raise AdapterError("INTERNAL_ERROR")
            self._commands.append(HospitalAction.START)
            self._state = "running"
            return {"available": True, "state": self._state}
        finally:
            self._lock.release()


class HospitalDeliveryAdapter(RobotAdapter):
    """Expose only the repository-owned simulation lifecycle."""

    def __init__(
        self,
        runtime: HospitalSimulationRuntime,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if type(runtime) is not HospitalSimulationRuntime:
            raise AdapterError("PROFILE_INVALID")
        self._runtime = runtime
        self._clock = clock
        self._simulation_stop_enqueued = 0
        self._safety_channel = _HospitalSimulationEmergencyChannel(self)

    def probe(self) -> AdapterProbe:
        result = self._invoke(HospitalAction.PROBE)
        return AdapterProbe(bool(result.get("available", True)), ("hospital.delivery",))

    def validate(self) -> None:
        self._invoke(HospitalAction.VALIDATE)

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        if task is not HospitalAction.START:
            raise AdapterError("PROFILE_INVALID")
        before_activation = getattr(self, "_before_activation", None)
        if before_activation is not None:
            before_activation()
        return self._activate_owned_start(
            activation_permit,
            self._start_nowait,
        )

    def status(self) -> AdapterStatus:
        return self._status_from(HospitalAction.STATUS)

    def cancel(self) -> AdapterStatus:
        return self._status_from(HospitalAction.CANCEL)

    def stop(self) -> None:
        self._invoke(HospitalAction.STOP)

    def observe(self, source: str) -> Observation:
        if source != "hospital_state":
            raise AdapterError("PROFILE_INVALID")
        result = self._invoke(HospitalAction.OBSERVE)
        return Observation(source, self._clock(), result)

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        return False

    def _emergency_stop_channel(self):
        return self._safety_channel

    def _status_from(self, action: HospitalAction) -> AdapterStatus:
        result = self._invoke(action)
        state = result.get("state")
        if not isinstance(state, str) or not state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(state, values=result)

    def _start_nowait(self) -> AdapterStatus:
        try:
            result = dict(self._runtime.start_nowait())
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        state = result.get("state")
        if not isinstance(state, str) or not state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(state, values=result)

    def _invoke(self, action: HospitalAction) -> dict[str, object]:
        try:
            result = self._runtime.dispatch(action)
        except AdapterError:
            raise
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        if not isinstance(result, Mapping):
            raise AdapterError("INTERNAL_ERROR")
        return dict(result)


class _HospitalSimulationEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, adapter: HospitalDeliveryAdapter) -> None:
        super().__init__(hardware_verified=False)
        self._adapter = adapter

    def _preflight(self) -> bool:
        return True

    def _enqueue_zero_disable(self) -> None:
        self._adapter._simulation_stop_enqueued += 1
