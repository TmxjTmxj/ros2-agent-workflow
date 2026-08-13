"""Fixed-authority bridge to the repository-owned hospital lifecycle."""

from __future__ import annotations

import json
import time
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


HospitalRunner = Callable[[HospitalAction], str | Mapping[str, object]]


class HospitalDeliveryAdapter(RobotAdapter):
    """Invoke fixed lifecycle actions; never accept argv, shell text, paths, or payloads."""

    def __init__(self, runner: HospitalRunner, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._runner = runner
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
        # This adapter is simulation-only. The final permit check rejects a
        # queued late START, but no claim is made about physical actuation.
        self._require_current_permit(activation_permit)
        return self._status_from(HospitalAction.START)

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
        # Repository-owned HospitalRunner is a simulation lifecycle only.
        return False

    def _emergency_stop_channel(self):
        return self._safety_channel

    def _status_from(self, action: HospitalAction) -> AdapterStatus:
        result = self._invoke(action)
        state = result.get("state")
        if not isinstance(state, str) or not state:
            raise AdapterError("INTERNAL_ERROR")
        return AdapterStatus(state, values=result)

    def _invoke(self, action: HospitalAction) -> dict[str, object]:
        if not isinstance(action, HospitalAction):
            raise AdapterError("PROFILE_INVALID")
        try:
            raw = self._runner(action)
        except Exception:
            raise AdapterError("INTERNAL_ERROR") from None
        if isinstance(raw, Mapping):
            result = dict(raw)
        elif isinstance(raw, str):
            result = _exact_json_object(raw)
        else:
            raise AdapterError("INTERNAL_ERROR")
        if not all(isinstance(key, str) for key in result):
            raise AdapterError("INTERNAL_ERROR")
        return result


def _exact_json_object(raw: str) -> dict[str, object]:
    try:
        decoder = json.JSONDecoder()
        value, end = decoder.raw_decode(raw.lstrip())
    except (json.JSONDecodeError, TypeError):
        raise AdapterError("INTERNAL_ERROR") from None
    leading = len(raw) - len(raw.lstrip())
    if raw[leading + end:].strip() or not isinstance(value, dict):
        raise AdapterError("INTERNAL_ERROR")
    return value


class _HospitalSimulationEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, adapter: HospitalDeliveryAdapter) -> None:
        super().__init__(hardware_verified=False)
        self._adapter = adapter

    def _preflight(self) -> bool:
        return True

    def _enqueue_zero_disable(self) -> None:
        # Fixed in-process simulation lifecycle marker; never invoke runner.
        self._adapter._simulation_stop_enqueued += 1
