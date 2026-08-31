"""Synchronous MotorPort over a dedicated CAN channel (python-can)."""

from __future__ import annotations

import time

from ..port import Feedback, MitCommand, PortError
from .dm4310 import DM4310


class PortFault(PortError):
    def __init__(self, code: int, text: str):
        super().__init__(text)
        self.code = code


class DamiaoCanPort:
    pos_window = 12.5

    def __init__(self, bus, motor_id: int, master_id: int, retry_budget_s: float = 0.2):
        self.motor = DM4310(bus, can_id=motor_id, master_id=master_id)
        self.retry_budget_s = retry_budget_s
        self._closed = False

    def enable(self) -> None:
        self.motor.enable()

    def disable(self) -> None:
        self.motor.disable()

    def clear_error(self) -> None:
        self.motor.clear_error()

    def _txrx(self, cmd: MitCommand) -> Feedback:
        deadline = time.monotonic() + self.retry_budget_s
        while True:
            self.motor.mit_control(cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau)
            raw = self.motor.read_feedback(timeout=min(0.05, self.retry_budget_s))
            if raw is not None:
                return Feedback(position=raw.position, velocity=raw.velocity,
                                torque=raw.torque, error_code=raw.error, t=time.monotonic())
            if time.monotonic() > deadline:
                raise PortError(f"motor 0x{self.motor.can_id:02X}: no feedback within "
                                f"{self.retry_budget_s:.2f}s")

    def _recover(self) -> Feedback | None:
        for _ in range(10):
            self.motor.clear_error()
            time.sleep(0.01)
            self.motor.enable()
            try:
                fb = self._txrx(MitCommand())
            except PortError:
                time.sleep(0.3)
                continue
            if not fb.faulted:
                return fb
            time.sleep(0.3)
        return None

    def command(self, cmd: MitCommand) -> Feedback:
        fb = self._txrx(cmd)
        if fb.faulted:
            recovered = self._recover()
            if recovered is None:
                raise PortFault(fb.error_code,
                                f"motor fault '{fb.fault_text}' did not clear after 10 attempts")
            return recovered
        return fb

    def read(self) -> Feedback:
        return self.command(MitCommand())    # zero gains: no motion

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for _ in range(3):
            try:
                self.motor.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                break
            time.sleep(0.02)
        for _ in range(5):
            try:
                self.motor.disable()
                self.motor.read_feedback(timeout=0.1)
            except Exception:
                break
            time.sleep(0.05)
