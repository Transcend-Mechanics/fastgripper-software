"""Minimal programmatic control: drive the gripper to a percentage and exit.

This is the ~80-line version of what pad.py/gui.py do — a starting point for
wiring the gripper into your own Python code (teleop glue, LeRobot hooks,
scripts). Velocity-mode MIT control with a software position loop; the worm
gear holds position when we stop commanding.

Usage:
    fastgripper-drive 40                 # go to 40% closed (0=open, 100=closed)
    fastgripper-drive 40 --interface socketcan --channel can1

Safety: same rules as the other tools — calibrated cal entry required; the
tool refuses to run if the restored position is outside the calibrated range
(stale state -> run `fastgripper-autocal home` first, jaws empty).
"""

import argparse
import time

from ..calstore import default_cal_path, get_entry, load_store, resolve_ids, save_store
from ..damiao.canbus import add_bus_args, open_bus
from ..damiao.dm4310 import DM4310, MultiTurnTracker

LOOP_HZ = 50.0
SW_KP = 4.0        # software position loop gain: v_cmd = SW_KP * error (1/s)
VMAX = 4.0         # rad/s speed cap
KD = 1.0           # motor-side velocity damping
TMAX = 1.0         # N*m grip torque cap
GOAL_EPS = 0.05    # rad: close enough -> stop, worm holds
STALL_SPEED = 0.1  # rad/s: commanded-but-not-moving below this...
STALL_TIME = 0.7   # ...for this long -> stop (obstruction / endpoint)
TIMEOUT_S = 30.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(parser)
    parser.add_argument("pct", type=float, help="target, 0=open .. 100=closed")
    parser.add_argument("--cal", default=None,
                        help="cal store path (default: ./gripper_cal.json if present, "
                             "else ~/.config/fastgripper/)")
    parser.add_argument("--gripper", default=None, help="cal-store entry name")
    args = parser.parse_args()
    args.cal = args.cal or default_cal_path()

    store = load_store(args.cal)
    name, entry = get_entry(store, args.gripper)
    if entry.get("open") is None or entry.get("last_position") is None:
        raise SystemExit(f"cal entry '{name}' incomplete — run fastgripper-autocal "
                         f"full (or home) first")
    motor_id, master_id = resolve_ids(args, entry)
    pct_goal = max(0.0, min(100.0, args.pct))
    if pct_goal != args.pct:
        print(f"target clamped to {pct_goal:.0f}% (valid range 0..100)")
    goal = entry["open"] + (pct_goal / 100.0) * (entry["closed"] - entry["open"])

    with open_bus(args.interface, args.channel) as bus:
        motor = DM4310(bus, can_id=motor_id, master_id=master_id)
        tracker = MultiTurnTracker(start_unwrapped=entry["last_position"])
        fb = None
        for _ in range(20):
            motor.enable()
            fb = motor.read_feedback(timeout=0.25)
            if fb:
                break
        if fb is None:
            for _ in range(3):
                motor.disable()
                time.sleep(0.02)
            raise SystemExit("no feedback — check power / wiring / channel / IDs")
        tracker.update(fb.position)

        lo, hi = sorted((entry["open"], entry["closed"]))
        if not (lo - 1.5 <= tracker.position <= hi + 1.5):
            motor.disable()
            raise SystemExit(f"restored position {tracker.position:+.1f} rad is outside "
                             f"[{lo:+.1f}, {hi:+.1f}] — stale state; run fastgripper-autocal home")

        deadline = time.monotonic() + TIMEOUT_S
        stall_since = None
        try:
            while time.monotonic() < deadline:
                t0 = time.monotonic()
                err = goal - tracker.position
                if abs(err) < GOAL_EPS:
                    break
                vel = max(-VMAX, min(VMAX, SW_KP * err))
                # torque cap: |kd*(v_cmd - v_actual)| <= TMAX
                vel = max(fb.velocity - TMAX / KD, min(fb.velocity + TMAX / KD, vel))
                motor.mit_control(0.0, vel, 0.0, KD, 0.0)
                new_fb = motor.read_feedback(timeout=0.05)
                if new_fb:
                    fb = new_fb
                    tracker.update(fb.position)
                    # stall guard: commanded but not moving = obstruction/endpoint
                    if vel != 0.0 and abs(fb.velocity) < STALL_SPEED:
                        if stall_since is None:
                            stall_since = t0
                        elif t0 - stall_since > STALL_TIME:
                            print("stalled — stopping (obstruction, or jaws at their limit)")
                            break
                    else:
                        stall_since = None
                time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t0)))
        finally:
            for _ in range(3):
                motor.mit_control(0.0, 0.0, 0.0, KD, 0.0)
                time.sleep(0.02)
            for _ in range(5):
                motor.disable()
                motor.read_feedback(timeout=0.1)
                time.sleep(0.02)
            entry["last_position"] = tracker.position
            entry["last_wrapped"] = tracker.wrapped
            save_store(args.cal, store)

    pct = (tracker.position - entry["open"]) / (entry["closed"] - entry["open"]) * 100.0
    print(f"done: {pct:.1f}% closed ({tracker.position:+.2f} rad)")


if __name__ == "__main__":
    main()


def cli() -> None:
    """Console-script entry: main() + fast exit (see fastgripper/_cli.py)."""
    from ._cli import run
    run(main)
