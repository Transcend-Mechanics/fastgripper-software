"""Guided gripper range calibration for the DM-J4310 worm-gear gripper.

You jog the motor with the keyboard while watching the gripper, and mark the
two ends of its travel yourself — the manual fallback for when autocal.py
can't be tuned for a unit. Positions are tracked across the motor's
±12.5 rad wrap window and saved to the gripper_cal.json store the other
tools read.

Keys:
    a / d : HOLD to jog open / closed
            (dead-man: releasing the key stops the motor)
    f     : toggle slow/FAST jog speed
    r     : reset the peak-torque readout
    space : stop jogging immediately
    o     : mark current position as OPEN
    c     : mark current position as CLOSED
    q     : save marks + exit               Ctrl-C: abort without saving

The worm gear can't be back-driven, so the calibration stays valid across
power cycles — as long as nobody moves the mechanism by hand while off.

Usage:
    python3 calibrate.py --interface socketcan --channel can1
    python3 calibrate.py --interface slcan --channel /dev/tty.usbmodemXXXX
"""

import argparse
import select
import sys
import termios
import time
import tty

from ..calstore import default_cal_path, get_entry, load_store, resolve_ids, save_store
from ..damiao.canbus import add_bus_args, open_bus
from ..damiao.dm4310 import DM4310, MultiTurnTracker

CAL_FILE = "gripper_cal.json"
JOG_SLOW = 1.5   # rad/s at the motor output shaft
JOG_FAST = 4.0
HOLD_TIMEOUT_S = 0.45  # jog stops this long after the last held-key repeat
                       # (must exceed the OS key-repeat initial delay)
TORQUE_LIMIT = 3.0   # N*m: sustained torque above this stops the jog
TORQUE_TRIP_S = 0.25
LOOP_HZ = 50


def read_key() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(parser)
    parser.add_argument("--cal", default=None, help="cal store path (default: ./gripper_cal.json if present, else ~/.config/fastgripper/)")
    parser.add_argument("--gripper", default=None,
                        help="entry name in the cal file (default: the existing entry "
                             "when there is exactly one, else 'default')")
    args = parser.parse_args()
    args.cal = args.cal or default_cal_path()

    if not sys.stdin.isatty():
        raise SystemExit("calibrate.py needs a real terminal for keyboard jogging — "
                         "run it in a Terminal window, not through a piped shell")

    # Resolve the cal-store entry up front: reuse the single existing entry
    # (and its motor IDs) rather than silently creating a second entry that
    # breaks name-less auto-resolution for every other tool.
    store = load_store(args.cal)
    if args.gripper is None:
        if len(store["grippers"]) > 1:
            get_entry(store, None)  # raises with the pick-a-name message
        args.gripper = next(iter(store["grippers"]), "default")
    entry = store["grippers"].get(args.gripper, {})
    motor_id, master_id = resolve_ids(args, entry)
    print(f"gripper entry: {args.gripper} (motor 0x{motor_id:02X} / feedback 0x{master_id:02X})")

    marks: dict[str, float] = {}
    vel = 0.0
    fast = False
    peak_torque = 0.0
    over_torque_since: float | None = None

    with open_bus(args.interface, args.channel) as bus:
        motor = DM4310(bus, can_id=motor_id, master_id=master_id)
        tracker = MultiTurnTracker()

        # enable + get first position fix (retry: first frames can be lossy)
        fb = None
        for _ in range(20):
            motor.enable()
            fb = motor.read_feedback(timeout=0.25)
            if fb:
                break
        if fb is None:
            # don't leave the motor enabled on the way out
            for _ in range(3):
                motor.disable()
                time.sleep(0.02)
            raise SystemExit("no feedback from motor — check power / wiring / charger unplugged")
        tracker.update(fb.position)
        print(__doc__.split("Usage:")[0])
        print(f"start position: {tracker.position:+.2f} rad — jog away!\n")

        old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        last_jog_key = 0.0
        try:
            while True:
                t0 = time.monotonic()
                key = read_key()
                if key in ("a", "A", "d", "D"):
                    speed = JOG_FAST if fast else JOG_SLOW
                    vel = -speed if key.lower() == "a" else speed
                    last_jog_key = t0
                elif key in ("f", "F"):
                    fast = not fast
                    if vel != 0.0:
                        vel = (JOG_FAST if fast else JOG_SLOW) * (1 if vel > 0 else -1)
                elif key == " ":
                    vel = 0.0
                elif key == "r":
                    peak_torque = 0.0
                elif key == "o":
                    marks["open"] = tracker.position
                    print(f"\nmarked OPEN   = {marks['open']:+.2f} rad")
                elif key == "c":
                    marks["closed"] = tracker.position
                    print(f"\nmarked CLOSED = {marks['closed']:+.2f} rad")
                elif key == "q":
                    break

                # dead-man: jog only while a key is held. Key autorepeat keeps
                # feeding characters during a hold; when they stop, so do we.
                if vel != 0.0 and t0 - last_jog_key > HOLD_TIMEOUT_S:
                    vel = 0.0

                # velocity command via MIT (kp=0, kd damps to the target vel)
                motor.mit_control(0.0, vel, 0.0, 1.0, 0.0)
                fb = motor.read_feedback(timeout=0.05)
                if fb:
                    tracker.update(fb.position)
                    peak_torque = max(peak_torque, abs(fb.torque))
                    # torque guard: stop driving into a jam
                    if vel != 0.0 and abs(fb.torque) > TORQUE_LIMIT:
                        if over_torque_since is None:
                            over_torque_since = time.monotonic()
                        elif time.monotonic() - over_torque_since > TORQUE_TRIP_S:
                            vel = 0.0
                            over_torque_since = None
                            print(f"\n!! torque {fb.torque:+.2f} Nm > {TORQUE_LIMIT} — jog stopped")
                    else:
                        over_torque_since = None
                    turns = tracker.position / (2 * 3.14159265)
                    print(f"\r{tracker.position:+7.2f}rad {turns:+5.1f}t "
                          f"v{fb.velocity:+4.1f} T{fb.torque:+5.2f}Nm pk{peak_torque:5.2f} "
                          f"jog{vel:+4.1f} [{'FAST' if fast else 'slow'}] ",
                          end="", flush=True)

                time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            print("\naborted (Ctrl-C) — nothing saved")
            marks.clear()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            # stop, then disable with retries + verify (a single lost frame
            # otherwise leaves the motor enabled and holding)
            for _ in range(3):
                motor.mit_control(0.0, 0.0, 0.0, 1.0, 0.0)
                time.sleep(0.02)
            disabled = False
            for _ in range(5):
                motor.disable()
                fb = motor.read_feedback(timeout=0.1)
                time.sleep(0.05)
                disabled = True
            print("\nmotor disable sent" if disabled else "\nWARNING: disable may not have landed")

    if "open" in marks and "closed" in marks:
        store = load_store(args.cal)  # reload: another tool may have written meanwhile
        cal = store["grippers"].setdefault(args.gripper, {})
        cal.update(**{
            "motor_id": motor_id, "master_id": master_id,
            "open": marks["open"],
            "closed": marks["closed"],
            "last_position": tracker.position,
            # raw motor-frame position (inside +/-12.5 rad) at save time: lets
            # cal_doctor.py verify a restored turn count against a live reading
            "last_wrapped": tracker.wrapped,
            "span": abs(marks["closed"] - marks["open"]),
            "method": "keyboard",
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        })
        save_store(args.cal, store)
        span_turns = cal["span"] / (2 * 3.14159265)
        print(f"saved entry '{args.gripper}' in {args.cal}: open={cal['open']:+.2f} closed={cal['closed']:+.2f} "
              f"({span_turns:.1f} turns of travel)")
        if cal["span"] > MultiTurnTracker.SPAN:
            print(f"note: travel exceeds one {MultiTurnTracker.SPAN:.0f} rad encoder window "
                  f"(normal for this gripper) — position restore relies on last_position; "
                  f"if the mechanism is ever moved by hand, verify with cal_doctor.py")
    else:
        missing = {"open", "closed"} - marks.keys()
        print(f"NOT saved — missing marks: {', '.join(missing)}")


if __name__ == "__main__":
    main()


def cli() -> None:
    """Console-script entry: main() + fast exit (see fastgripper/_cli.py)."""
    from ._cli import run
    run(main)
