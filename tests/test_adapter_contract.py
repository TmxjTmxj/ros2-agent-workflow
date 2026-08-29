from __future__ import annotations

import pytest
from agent_ros.adapters.base import AdapterProbe, AdapterStatus, HospitalAction, Observation, RobotAdapter
from agent_ros.adapters.hospital import HospitalDeliveryAdapter, HospitalSimulationRuntime

from tests.contracts.adapter_contract import assert_adapter_contract


class UnsafeFakeAdapter(RobotAdapter):
    """Deliberately violates the activation boundary for a contract negative test."""

    def probe(self) -> AdapterProbe:
        return AdapterProbe(True, ("fake",))

    def validate(self) -> None:
        return None

    def start(self, task: object, activation_permit: object = None) -> AdapterStatus:
        return AdapterStatus("running")

    def status(self) -> AdapterStatus:
        return AdapterStatus("running")

    def cancel(self) -> AdapterStatus:
        return AdapterStatus("cancelled")

    def stop(self) -> None:
        return None

    def observe(self, source: str) -> Observation:
        return Observation(source, 0.0, {})

    def bind_physical_estop(self, handler) -> bool:
        return False

    def _emergency_stop_channel(self):
        raise AssertionError("unsafe adapter must fail before safety setup")


def test_contract_rejects_adapter_that_starts_without_permit():
    with pytest.raises(AssertionError, match="activation permit"):
        assert_adapter_contract(UnsafeFakeAdapter(), task=object(), sources=("odometry",))


def test_contract_covers_the_in_memory_hospital_demonstration():
    adapter = HospitalDeliveryAdapter(HospitalSimulationRuntime())

    assert_adapter_contract(adapter, task=HospitalAction.START, sources=("hospital_state",))
