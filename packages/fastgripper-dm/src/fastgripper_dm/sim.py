"""Simulated worm gripper implementing MotorPort, for hardware-free tests.

The torque model is exactly the motor's (tau = kd*(v_cmd - v) + tau_ff) so a
"cap never exceeded" assertion is meaningful, not vacuous."""

from __future__ import annotations

import time

from .port import POS_WINDOW, SPAN, Feedback, MitCommand, PortError


class SimulatedWormGripper:
    pos_window = POS_WINDOW

    def __init__(self, stop_open: float = -31.0, stop_closed: float = +3.0, start: float = +1.0,
                 friction: float = 0.25, dt: float = 0.02,
                 stop_stiffness: float = 2.0, inertia: float = 0.01):
        assert stop_open < start < stop_closed
        self.stop_open, self.stop_closed = stop_open, stop_closed
        self.true_position = start
        self.velocity = 0.0
        self.friction, self.dt = friction, dt
        self.stop_stiffness, self.inertia = stop_stiffness, inertia
        self._enabled = False
        self._fault = 0
        self._drops = 0
        self._torque = 0.0
        self.max_abs_torque = 0.0
        self._t = 0.0

    # --- test hooks ---
    def drop_next(self, n: int) -> None:
        self._drops = n

    def inject_fault(self, code: int) -> None:
        self._fault = code
        self._enabled = False

    # --- MotorPort ---
    def enable(self) -> None:
        if not self._fault:
            self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def clear_error(self) -> None:
        self._fault = 0

    def close(self) -> None:
        self.disable()

    def read(self) -> Feedback:
        return self._feedback()

    def command(self, cmd: MitCommand) -> Feedback:
        if self._drops > 0:
            self._drops -= 1
            raise PortError("simulated dropped frame")
        if self._enabled and not self._fault:
            # ONE velocity integrator, driven by the PREVIOUS step's torque, so the
            # feedback pair (velocity, torque) is self-consistent: reported torque
            # is kd*(v_cmd - reported_velocity) exactly, and the controller clamps
            # its next v_cmd against that same reported velocity -- which is what
            # makes the max_abs_torque <= TMAX assertion a real guarantee.
            tau_net = self._torque \
                - self.friction * (1 if self.velocity > 0 else -1 if self.velocity < 0 else 0)
            if self.true_position > self.stop_closed:
                tau_net -= self.stop_stiffness * (self.true_position - self.stop_closed)
            elif self.true_position < self.stop_open:
                tau_net -= self.stop_stiffness * (self.true_position - self.stop_open)
            self.velocity += (tau_net / self.inertia) * self.dt
            new_pos = self.true_position + self.velocity * self.dt
            hi, lo = self.stop_closed + 0.35, self.stop_open - 0.35
            hit_stop = new_pos >= hi or new_pos <= lo
            new_pos = min(hi, max(lo, new_pos))
            if hit_stop:
                self.velocity = 0.0
            self.true_position = new_pos
            self._torque = cmd.kd * (cmd.vel - self.velocity) + cmd.tau \
                + cmd.kp * (cmd.pos - self._wrapped())
            self.max_abs_torque = max(self.max_abs_torque, abs(self._torque))
        else:
            self.velocity = 0.0
            self._torque = 0.0
        self._t += self.dt
        return self._feedback()

    def _wrapped(self) -> float:
        return ((self.true_position + POS_WINDOW) % SPAN) - POS_WINDOW

    def _feedback(self) -> Feedback:
        code = self._fault if self._fault else (1 if self._enabled else 0)
        return Feedback(position=self._wrapped(), velocity=self.velocity,
                        torque=self._torque, error_code=code, t=self._t or time.monotonic())
