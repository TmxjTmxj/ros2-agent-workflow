"""Fixed-authority bridge to the repository-owned hospital lifecycle."""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Mapping

from agent_ros.adapters.base import (
    AdapterError,
    AdapterProbe,
    AdapterStatus,
    HospitalAction,
    Observation,
    RobotAdapter,
    SafetyToken,
)


HospitalRunner = Callable[[HospitalAction], str | Mapping[str, object]]


class HospitalDeliveryAdapter(RobotAdapter):
    """Invoke fixed lifecycle actions; never accept argv, shell text, paths, or payloads."""

    def __init__(self, runner: HospitalRunner, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._runner = runner
        self._clock = clock

    def probe(self) -> AdapterProbe:
        result = self._invoke(HospitalAction.PROBE)
        return AdapterProbe(bool(result.get("available", True)), ("hospital.delivery",))

    def validate(self) -> None:
        self._invoke(HospitalAction.VALIDATE)

    def start(self, task: object, safety_token: SafetyToken | None = None) -> AdapterStatus:
        if task is not HospitalAction.START or not isinstance(safety_token, SafetyToken):
            raise AdapterError("PROFILE_INVALID")
        if not safety_token.is_valid():
            raise AdapterError("ESTOP_LATCHED")
        return self._status_from(HospitalAction.START)

    def status(self) -> AdapterStatus:
        return self._status_from(HospitalAction.STATUS)

    def cancel(self) -> AdapterStatus:
        return self._status_from(HospitalAction.CANCEL)

    def stop(self) -> None:
        self._invoke(HospitalAction.STOP)

    def emergency_stop(self) -> None:
        emergency = getattr(self._runner, "emergency_stop", None)
        if emergency is None:
            self._invoke(HospitalAction.STOP)
            return
        try:
            emergency()
        except Exception:
            raise AdapterError("UNSAFE_STATE") from None

    def observe(self, source: str) -> Observation:
        if source != "hospital_state":
            raise AdapterError("PROFILE_INVALID")
        result = self._invoke(HospitalAction.OBSERVE)
        return Observation(source, self._clock(), result)

    def bind_physical_estop(self, handler: Callable[[bool], None]) -> bool:
        binder = getattr(self._runner, "subscribe_estop", None)
        emergency = getattr(self._runner, "emergency_stop", None)
        if binder is not None and emergency is not None:
            binder(handler)
            return True
        return False

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
