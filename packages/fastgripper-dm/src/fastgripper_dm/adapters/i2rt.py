"""FastGripper on an i2rt YAM arm: the gripper rides the arm's MotorChainRobot.

The adapter touches the ROBOT READ SURFACE ONLY (`get_joint_pos`,
`get_observations`); the loop owner (the teleop / robot server client) is the
single writer via `command_joint_state` — i2rt's server thread consumes it
(spec §3.2). Requires the robot built with
`gripper_limits_override = [-POS_WINDOW, +POS_WINDOW]` so
`get_joint_pos()[idx] * SPAN - POS_WINDOW` is the exact wrapped shaft angle
(so101_teleop.py:303-306, 398). Faults on this path are handled by i2rt's own
recovery; Feedback.error_code is reported as 1 (enabled).

i2rt itself is only needed by the CALLER (to build the robot):
    pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"
This module never imports i2rt.
"""

from __future__ import annotations

import time

from ..calstore import default_cal_path, entry_profile, get_entry, load_store, save_store
from ..controller import GripperController
from ..port import Feedback, MitCommand, POS_WINDOW, SPAN


def to_i2rt_command(cmd: MitCommand, pos_placeholder_norm: float) -> tuple[float, float, float, float]:
    """Convert a real-unit MitCommand into i2rt's normalized command space.

    The JointMapper multiplies the gripper velocity by the joint range
    (= SPAN), so divide here (so101_teleop.py:451). Position is ignored at
    kp=0 but must be a valid normalized value — pass the current one."""
    return (pos_placeholder_norm, cmd.vel / SPAN, cmd.kp, cmd.kd)


class I2rtGripper:
    def __init__(self, robot, joint_index: int, gripper: str | None = None,
                 cal_path: str | None = None):
        self.robot = robot
        self.idx = joint_index
        self.cal_path = cal_path or default_cal_path()
        self._store = load_store(self.cal_path)
        self._name, self.entry = get_entry(self._store, gripper)
        self.ctrl = GripperController(self.entry, entry_profile(self.entry))
        self.last_feedback: Feedback | None = None
        self._connected = False
        # True once this session has put the multi-turn tracker into the same
        # frame the cal store uses (park adoption, or explicit homing). Until
        # then the tracker's zero is wherever the shaft happened to boot, and
        # nothing derived from it may be written back to the store.
        self._anchored = False

    # --- read side ---
    def _wrapped(self) -> float:
        return float(self.robot.get_joint_pos()[self.idx]) * SPAN - POS_WINDOW

    def _feedback(self) -> Feedback:
        obs = self.robot.get_observations()
        return Feedback(position=self._wrapped(),
                        velocity=float(obs["gripper_vel"][0]) * SPAN,
                        torque=float(obs["gripper_eff"][0]),
                        error_code=1, t=time.monotonic())

    # --- lifecycle ---
    def connect(self, home: str = "auto") -> None:
        # Enforce the load-bearing invariant: the robot must be built with
        # gripper_limits_override = [-POS_WINDOW, +POS_WINDOW] or every
        # position/velocity is silently mis-scaled (JointMapper divides by
        # the limits span). get_robot_info exposes the limits.
        try:
            lim = self.robot.get_robot_info()["gripper_limits"]
            span = float(lim[1]) - float(lim[0])
        except Exception as e:
            raise ValueError(f"robot does not expose gripper_limits: {e}") from e
        if abs(span - SPAN) > 1e-6:
            raise ValueError(
                f"robot gripper_limits span {span} != {SPAN}: build the robot with "
                f"gripper_limits_override=[-{POS_WINDOW}, +{POS_WINDOW}]")
        boot = self._wrapped()
        if home == "auto":
            lw = self.entry.get("last_wrapped")
            tol = self.ctrl.profile.park_tolerance_rad
            if lw is None or min(abs(boot - lw) % SPAN, SPAN - abs(boot - lw) % SPAN) > tol:
                raise ValueError(
                    f"gripper is not at its park (wrapped {boot:+.2f} vs saved {lw}) -- "
                    f"run `fastgripper-dm autocal home --gripper {self._name}` on a dedicated "
                    f"channel, or fix the cal entry; no stall homing on a shared chain")
            self.ctrl.adopt_park(self.entry["last_position"])
            self._anchored = True
        elif home == "off":
            pass                       # leaves the tracker unanchored
        else:
            raise ValueError(f"unsupported home mode on a shared chain: {home!r}")
        self.ctrl.tick(self._feedback(), 0.0)
        self.ctrl.hold()
        self._connected = True

    # --- per-cycle ---
    def tick(self, dt: float) -> MitCommand:
        self.last_feedback = self._feedback()
        return self.ctrl.tick(self.last_feedback, dt)

    @property
    def velocity(self) -> float:
        """Last measured shaft velocity, rad/s (for telemetry)."""
        return self.last_feedback.velocity

    @property
    def torque(self) -> float:
        """Last measured motor torque, Nm (for telemetry)."""
        return self.last_feedback.torque

    def command_tuple(self, dt: float) -> tuple[float, float, float, float]:
        cmd = self.tick(dt)
        return to_i2rt_command(cmd, float(self.robot.get_joint_pos()[self.idx]))

    # --- goals / state (delegation) ---
    def goto(self, frac: float) -> None: self.ctrl.goto_frac(frac)
    def open(self) -> None: self.ctrl.open()
    def close(self) -> None: self.ctrl.close()
    def hold(self) -> None: self.ctrl.hold()

    @property
    def position(self): return self.ctrl.position

    @property
    def goal(self): return self.ctrl.goal

    @property
    def stalled(self) -> bool: return self.ctrl.stalled

    def anchor(self, known_position: float) -> None:
        """Declare the shaft's true multi-turn position (explicit homing).

        This is what makes a `home="off"` session's park() safe to save."""
        self.ctrl.anchor(known_position)
        self._anchored = True

    def park(self) -> None:
        self.ctrl.hold()               # stop moving, anchored or not
        if not self._anchored:
            # home="off": the tracker is not anchored, so last_position would be
            # an arbitrary frame. Saving it lets the NEXT session adopt it and
            # drive the (non-back-drivable) worm into a hard stop at the full
            # torque cap. Stop moving, save nothing.
            print("park: tracker is not anchored (home=off) -- not saving "
                  "last_position; re-home before the next session")
            return
        if self.ctrl.tracker.seen:
            self.entry.update(self.ctrl.park_fields())
            self.entry["parked_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            save_store(self.cal_path, self._store)
