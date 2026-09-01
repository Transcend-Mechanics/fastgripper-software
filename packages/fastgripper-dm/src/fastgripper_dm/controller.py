"""The gripper control algorithm, once, with no I/O.

Ported from so101_teleop.py v7 (the hardware-validated version with the
torque cap): software P velocity loop + per-tick torque clamp + stall hold.
The owner of the bus loops `fb = port.command(controller.tick(fb, dt))`
(standalone) or merges the returned command into its own array (adapters)."""

from __future__ import annotations

from .port import Feedback, MitCommand
from .profile import GripperProfile
from .tracker import MultiTurnTracker


class GripperController:
    RELEASE_EPS = 0.05   # rad; matches the live teleop's trigger-backoff release

    def __init__(self, entry: dict, profile: GripperProfile):
        profile.validate()
        if "open" not in entry or "closed" not in entry:
            raise ValueError("cal entry needs open/closed marks -- calibrate first")
        self.entry = entry
        self.profile = profile
        self.kd = profile.tmax_nm / profile.vmax
        self._lo = min(entry["open"], entry["closed"])
        self._hi = max(entry["open"], entry["closed"])
        self.tracker = MultiTurnTracker()
        self.goal: float | None = None
        self.stalled = False
        self._stall_t = 0.0
        # Last *requested* goal (pre-stall-clamp), used only to detect a
        # retreating goal while latched -- self.goal itself gets pinned to
        # the current position on stall (see tick()), so it can't be used
        # for that comparison once latched.
        self._requested_goal: float | None = None

    # --- goals ---
    def goto_rad(self, pos: float) -> None:
        new_goal = min(self._hi, max(self._lo, pos))
        if self.stalled:
            # Latched on an obstruction: a goal that stays within RELEASE_EPS
            # of the last requested goal is the trigger still held (or
            # re-asserted every tick by the live per-tick goto) -- ignore it
            # and keep the latch. Only a goal that has backed off by at
            # least RELEASE_EPS (the live stall_clip semantics,
            # so101_teleop.py:389-395) releases the latch.
            prev = self._requested_goal if self._requested_goal is not None else new_goal
            if abs(new_goal - prev) >= self.RELEASE_EPS:
                self.stalled = False
                self._stall_t = 0.0
                self.goal = new_goal
            self._requested_goal = new_goal
            return
        if self.goal is not None and abs(new_goal - self.goal) < 1e-9:
            self._requested_goal = new_goal
            return                            # idempotent re-goto: keep stall timer
        self.goal = new_goal
        self._requested_goal = new_goal
        self.stalled = False
        self._stall_t = 0.0

    def goto_frac(self, frac: float) -> None:
        frac = min(1.0, max(0.0, frac))
        self.goto_rad(self.entry["closed"] + frac * (self.entry["open"] - self.entry["closed"]))

    def open(self) -> None:
        self.goto_frac(1.0)

    def close(self) -> None:
        self.goto_frac(0.0)

    def hold(self) -> None:
        if self.position is not None:
            self.goal = self.position

    # --- state ---
    @property
    def position(self) -> float | None:
        return self.tracker.position if self.tracker.seen else None

    @property
    def at_goal(self) -> bool:
        return (self.goal is not None and self.position is not None
                and abs(self.goal - self.position) < 0.05)

    def adopt_park(self, last_position: float) -> None:
        self.tracker = MultiTurnTracker(start_unwrapped=last_position)

    def anchor(self, known_position: float) -> None:
        self.tracker.anchor(known_position)

    def park_fields(self) -> dict:
        return {"last_position": self.tracker.position, "last_wrapped": self.tracker.wrapped}

    # --- the tick ---
    def tick(self, fb: Feedback, dt: float) -> MitCommand:
        if fb.faulted:
            return MitCommand()
        pos = self.tracker.update(fb.position)
        if self.goal is None or self.stalled:
            return MitCommand(kd=self.kd)          # damp to zero velocity (hold)
        p = self.profile
        v = p.sw_kp * (self.goal - pos)
        v = min(p.vmax, max(-p.vmax, v))
        dv = p.tmax_nm / self.kd                    # = vmax by construction
        # Clamps against fb.velocity, which is one integration step stale relative
        # to the velocity the motor (or sim) uses to compute the torque it reports
        # next tick -- the <= tmax bound is enforced by the motor's own kd law
        # (tau = kd*(v_cmd - v_actual), a self-correcting relationship), not by
        # strict per-tick algebra here, and is verified empirically in tests.
        # Revisit this comment if vmax, sw_kp, or the loop dt change materially.
        v = min(fb.velocity + dv, max(fb.velocity - dv, v))
        # stall detection on MEASURED torque (current sense), like the bench code
        if abs(fb.torque) > p.stall_torque_frac * p.tmax_nm and not self.at_goal:
            self._stall_t += dt
            if self._stall_t > p.stall_time_s:
                self.stalled = True
                self.goal = pos
                return MitCommand(kd=self.kd)
        else:
            self._stall_t = 0.0
        return MitCommand(vel=v, kd=self.kd)
