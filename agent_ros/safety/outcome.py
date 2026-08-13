from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class EmergencyStopResult:
    latched: bool
    activation_quiesced: bool
    safety_command_accepted: bool
    code: str

    @property
    def successful(self) -> bool:
        return (
            self.latched
            and self.activation_quiesced
            and self.safety_command_accepted
        )
