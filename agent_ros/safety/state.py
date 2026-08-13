"""Explicit, fail-closed states for robot control authorization."""

from __future__ import annotations

from enum import Enum


class SafetyState(str, Enum):
    NEW = "NEW"
    DISCOVERED = "DISCOVERED"
    VALIDATED = "VALIDATED"
    ARMED = "ARMED"
    RUNNING = "RUNNING"
    STOPPED = "STOPPED"
    FAULTED = "FAULTED"
    ESTOPPED = "ESTOPPED"
