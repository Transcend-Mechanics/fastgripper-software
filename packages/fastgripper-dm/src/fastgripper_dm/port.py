"""Contracts between the gripper controller and whatever owns the motor."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

POS_WINDOW = 12.5          # DM-J4310 MIT feedback window: +/-12.5 rad
SPAN = 2 * POS_WINDOW      # 25 rad per wrap

from .damiao.dm4310 import FAULT_CODES  # single fault table for the whole package


@dataclass(frozen=True)
class Feedback:
    position: float     # rad, output shaft, WRAPPED into +/-pos_window
    velocity: float     # rad/s
    torque: float       # Nm at the motor
    error_code: int     # DM status nibble
    t: float            # time.monotonic() of the reading

    @property
    def faulted(self) -> bool:
        return self.error_code >= 0x8

    @property
    def enabled(self) -> bool:
        return self.error_code == 0x1

    @property
    def fault_text(self) -> str:
        return FAULT_CODES.get(self.error_code, f"unknown ({self.error_code:#x})")


@dataclass(frozen=True)
class MitCommand:
    pos: float = 0.0
    vel: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0


class PortError(RuntimeError):
    """The port could not complete a transaction within its retry budget."""


class BusDead(PortError):
    """The bus opened but passes no frames (nothing ACKs)."""


class MotorPort(Protocol):
    pos_window: float

    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def command(self, cmd: MitCommand) -> Feedback: ...
    def read(self) -> Feedback: ...
    def clear_error(self) -> None: ...
    def close(self) -> None: ...
