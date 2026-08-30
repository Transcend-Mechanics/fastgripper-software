"""Turn-alias doctor for the DM4310 worm-gear gripper.

The motor only reports position inside a ±12.5 rad window; gripper travel is
wider than one 25 rad window, so a saved `last_position` that is stale (the
mechanism moved while no driver was running) makes every tool pick the wrong
encoder revolution — symptoms: close stops early / open overruns, or vice
versa. This tool reads ONE live feedback frame (enable → read → disable; no
motion is commanded), lists every physically-possible interpretation of it,
and checks the calibration file's saved state against them.

Usage:
    python cal_doctor.py --interface gs_usb --motor_id 0x07 --master_id 0x17
    python cal_doctor.py ... --fix        # write the recommended last_position

IMPORTANT: the doctor can only tell WHERE the shaft could be; if more than one
candidate lies inside the calibrated range, it needs YOU to say which matches
what the jaws visibly look like (percentages are printed for exactly that).
Never --fix based on a guess; recalibrate instead.
"""

import argparse
import sys
import time

from ..calstore import default_cal_path, get_entry, load_store, resolve_ids, save_store
from ..damiao.canbus import add_bus_args, open_bus
from ..damiao.dm4310 import DM4310, MultiTurnTracker

SPAN = MultiTurnTracker.SPAN  # 25 rad
RANGE_MARGIN = 1.5            # rad of slack beyond the open/closed marks


def read_wrapped_once(args) -> float:
    """Enable, grab one feedback frame, disable. No motion is commanded."""
    with open_bus(args.interface, args.channel) as bus:
        motor = DM4310(bus, can_id=args.motor_id, master_id=args.master_id)
        fb = None
        try:
            for _ in range(20):
                motor.enable()
                fb = motor.read_feedback(timeout=0.25)
                if fb:
                    break
        finally:
            for _ in range(5):
                motor.disable()
                motor.read_feedback(timeout=0.1)
                time.sleep(0.02)
        if fb is None:
            raise SystemExit("no feedback from motor — check power / wiring / IDs")
        if fb.faulted:
            print(f"WARNING: motor reports fault: {fb.error_text}")
        return fb.position


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(parser)
    parser.add_argument("--cal", default=None, help="cal store path (default: ./gripper_cal.json if present, else ~/.config/fastgripper/)")
    parser.add_argument("--gripper", default=None,
                        help="cal entry name (optional if the file has exactly one)")
    parser.add_argument("--fix", action="store_true",
                        help="write the recommended last_position back to the cal file")
    args = parser.parse_args()
    args.cal = args.cal or default_cal_path()

    store = load_store(args.cal)
    name, cal = get_entry(store, args.gripper)
    print(f"gripper entry: {name}")
    args.motor_id, args.master_id = resolve_ids(args, cal)
    lo, hi = sorted((cal["open"], cal["closed"]))
    saved = cal.get("last_position")
    span = hi - lo

    wrapped = read_wrapped_once(args)
    print(f"\ncalibration: open={cal['open']:+.2f}  closed={cal['closed']:+.2f}  "
          f"span={span:.2f} rad ({span / SPAN:.2f} encoder windows)")
    print(f"saved last_position: {saved:+.3f} rad" if saved is not None
          else "saved last_position: MISSING")
    print(f"live wrapped reading: {wrapped:+.3f} rad (window ±{SPAN / 2:.1f})\n")

    # every unwrapped position consistent with the live reading, near the range
    candidates = []
    k_lo = int((lo - RANGE_MARGIN - wrapped) // SPAN)
    k_hi = int((hi + RANGE_MARGIN - wrapped) // SPAN) + 1
    for k in range(k_lo, k_hi + 1):
        c = wrapped + k * SPAN
        if lo - RANGE_MARGIN <= c <= hi + RANGE_MARGIN:
            pct = (c - cal["open"]) / (cal["closed"] - cal["open"]) * 100.0
            candidates.append((c, pct))

    if not candidates:
        print("NO candidate inside the calibrated range — the mechanism has been")
        print("moved far outside its marks, or the calibration file is for a")
        print("different motor. Recalibrate.")
        sys.exit(1)

    print("physically possible positions (pick by LOOKING at the jaws):")
    match = None
    for c, pct in candidates:
        tag = ""
        if saved is not None and abs(c - saved) < 0.2:
            tag = "  <-- matches saved state"
            match = c
        print(f"  {c:+9.3f} rad  =  {pct:6.1f}% closed{tag}")

    if match is not None:
        print("\nOK: saved state is consistent with the live reading.")
        print("(If behavior is still wrong, the jaws must visibly disagree with")
        print(f" the matched candidate's percentage — then recalibrate.)")
        sys.exit(0)

    print("\nSAVED STATE IS STALE — it does not match any interpretation of the")
    print("live reading. The mechanism moved while no driver was tracking it.")
    if len(candidates) == 1:
        c, pct = candidates[0]
        print(f"Exactly one candidate is in range: {c:+.3f} rad ({pct:.1f}% closed).")
        if args.fix:
            cal["last_position"] = c
            cal["last_wrapped"] = wrapped
            save_store(args.cal, store)
            print(f"WROTE last_position={c:+.3f} to {args.cal}")
        else:
            print("Confirm the jaws visually match that percentage, then rerun with --fix.")
    else:
        print("Multiple candidates are in range — the doctor cannot choose for you.")
        print("Compare the percentages above with the actual jaws; edit last_position")
        print("to the matching value, or recalibrate if unsure.")
    sys.exit(2)


if __name__ == "__main__":
    main()


def cli() -> None:
    """Console-script entry: main() + fast exit (see fastgripper/_cli.py)."""
    from ._cli import run
    run(main)
