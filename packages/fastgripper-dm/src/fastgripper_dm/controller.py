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
        # The goal in effect when the latch engaged, and the sign of the
        # travel that produced it (+1 = was closing, -1 = was opening).
        # self.goal gets pinned to the current position on stall (see tick()),
        # so the release test has to be against this frozen goal instead.
        self._stall_goal: float | None = None
        self._stall_dir = 0.0

    # --- goals ---
    def goto_rad(self, pos: float) -> None:
        new_goal = min(self._hi, max(self._lo, pos))
        if self.stalled:
            # Latched on an obstruction. Release ONLY when the new goal has
            # retreated at least RELEASE_EPS from the goal that was in effect
            # when the latch engaged, in the direction AWAY from the travel
            # that produced the stall (the live stall_clip semantics,
            # so101_teleop.py:389-395). Deliberately not abs(): a goal that
            # pushes deeper into the obstruction, however far, keeps the latch.
            # Measuring against the frozen _stall_goal (not the last goal) is
            # what lets a slow, quantized trigger release accumulate.
            retreat = (self._stall_goal - new_goal) * self._stall_dir
            if retreat >= self.RELEASE_EPS:
                self.stalled = False
                self._stall_goal = None
                self._stall_dir = 0.0
                self._stall_t = 0.0
                self.goal = new_goal
            return
        # Setting a goal never touches the stall timer. The timer measures
        # sustained high MEASURED torque away from the goal, and tick() already
        # zeroes it the moment the torque falls off -- which is exactly what
        # happens when the operator really does back off an obstruction. Letting
        # a goal edit re-zero it instead put the timer at the mercy of the
        # trigger encoder: ~0.027 rad/tick of quantization plus dither means a
        # merely-held trigger re-commands a slightly different goal every tick,
        # the 0.4 s timer never expires, the latch never engages, and the motor
        # pushes into a grasped object at the full 2.0 Nm cap indefinitely.
        self.goal = new_goal

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
                self._stall_goal = self.goal
                self._stall_dir = 1.0 if self.goal > pos else -1.0
                self.goal = pos
                return MitCommand(kd=self.kd)
        else:
            self._stall_t = 0.0
        return MitCommand(vel=v, kd=self.kd)
