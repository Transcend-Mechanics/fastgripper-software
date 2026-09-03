"""Hardstop autocalibration for the DM4310 worm-gear gripper.

Requires a gripper WITH designed-in hardstops (safe to probe gently). Probes
the stops at low speed under a hard torque cap, double-touches each datum for
repeatability, verifies the measured span, and only then writes the cal file.
Headless-friendly (no TTY needed with --yes); works on gs_usb, slcan, or
socketcan via canbus.py.

Modes:
  full  : probe closed stop, then open stop; write new open/closed marks
          (stops minus --margin), span, and stop datums to the cal file.
  home  : probe the closed stop only and re-anchor last_position against the
          stop_closed datum recorded by a previous full/touch run. Marks
          unchanged. ~20 s; the demo "autocal" / turn-alias recovery command.
  touch : closed-stop double-touch TEST — probes, reports the datum and how it
          compares to the calibrated closed mark, backs off, writes NOTHING
          unless --write (which records stop_closed so home mode works).
          Use this to validate probe tuning one direction at a time.

Usage:
  python autocal.py full --expected_span 31.1 --interface socketcan --channel can_right \
      --motor_id 0x20 --master_id 0x30
  python autocal.py home --interface gs_usb --motor_id 0x07 --master_id 0x17

SAFETY: jaws must be EMPTY (an object in the jaws either gets squeezed or
fakes a stall — the double-touch + span checks catch most of this, but do not
rely on them). Never run while another process owns the motor.
"""

import argparse
import math
import sys
import time

from ..calstore import default_cal_path, get_entry, load_store, resolve_ids, save_store
from ..damiao.canbus import add_bus_args, open_bus
from ..damiao.dm4310 import DM4310, MultiTurnTracker
from ..profile import GripperProfile

# Usable travel assumed below the closed mark when probing closed-only.
# Conservative UNDERestimate of real stop-to-stop travel: if this is too
# large, a 0% (open) command drives the jaws into the open stop.
DEFAULT_SPAN_FROM_CLOSED = 30.0

# Homing guards. A legitimate re-anchor corrects a whole-turn (2*pi) boot
# ambiguity plus probe scatter; a false contact produces something far larger.
HOME_MAX_REANCHOR_RAD = 3 * 6.283185 + 1.0   # ~3 turns of slack
HOME_MARK_SLACK_RAD = 2.0                    # tolerance outside the cal marks

LOOP_HZ = 50.0


class Abort(Exception):
    pass


class Rig:
    """Motor + tracker + probe primitives sharing one control loop style."""

    def __init__(self, motor: DM4310, args):
        self.motor = motor
        self.args = args
        self.tracker = MultiTurnTracker()
        self.fb = None

    def step(self, vel: float) -> None:
        """One 50 Hz cycle: torque-capped velocity command + feedback."""
        if self.fb is not None:
            lo = self.fb.velocity - self.args.probe_tmax / self.args.kd
            hi = self.fb.velocity + self.args.probe_tmax / self.args.kd
            vel = max(lo, min(hi, vel))
        self.motor.mit_control(0.0, vel, 0.0, self.args.kd, 0.0)
        fb = self.motor.read_feedback(timeout=0.05)
        if fb:
            if fb.faulted:
                raise Abort(f"motor fault: {fb.error_text}")
            self.fb = fb
            self.tracker.update(fb.position)

    def probe(self, direction: int, label: str, vel: float | None = None,
              contact_torque: float | None = None) -> float:
        """Drive until contact. Two triggers, whichever first: sustained
        velocity stall (rigid hardstop), or sustained torque above threshold
        (compliant stop, e.g. jaw tips; fires at contact onset, before force
        builds). Returns the position. vel/contact_torque default to the
        tuned flags; the fast seek pass overrides both."""
        vel = vel if vel is not None else self.args.probe_vel
        if contact_torque is None:
            contact_torque = self.args.contact_torque
        start = self.tracker.position
        stall_since = contact_since = None
        free_torques: list[float] = []
        t_end = time.monotonic() + self.args.probe_timeout
        while True:
            t0 = time.monotonic()
            if t0 > t_end:
                raise Abort(f"{label}: no contact within {self.args.probe_timeout:.0f}s")
            self.step(direction * vel)
            if abs(self.tracker.position - start) > self.args.max_travel:
                raise Abort(f"{label}: traveled {self.args.max_travel:.1f} rad without "
                            f"stalling — wrong direction, no stop, or decoupled mechanism")
            if self.fb:
                moving = abs(self.fb.velocity) >= self.args.stall_speed
                if moving:
                    free_torques.append(abs(self.fb.torque))

                if contact_torque and contact_torque > 0 and \
                        abs(self.fb.torque) > contact_torque:
                    contact_since = contact_since or t0
                    if t0 - contact_since > self.args.contact_time:
                        pos = self.tracker.position
                        print(f"  {label}: torque contact ({self.fb.torque:+.2f} Nm) "
                              f"at {pos:+8.3f} rad")
                        self._report_free_torque(free_torques)
                        return pos
                else:
                    contact_since = None

                if not moving:
                    stall_since = stall_since or t0
                    if t0 - stall_since > self.args.stall_time:
                        pos = self.tracker.position
                        print(f"  {label}: stall at {pos:+8.3f} rad")
                        self._report_free_torque(free_torques)
                        return pos
                else:
                    stall_since = None
            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t0)))

    @staticmethod
    def _report_free_torque(samples: list[float]) -> None:
        """Free-running torque stats — the basis for tuning --contact_torque."""
        if len(samples) < 10:
            return
        samples = sorted(samples)
        med = samples[len(samples) // 2]
        p95 = samples[int(len(samples) * 0.95)]
        print(f"    free-run torque: median {med:.2f} / p95 {p95:.2f} Nm "
              f"(set --contact_torque a bit above p95)")

    def move_by(self, delta: float, label: str) -> None:
        """Open-loop-ish relative move (P-on-error velocity), for back-offs."""
        goal = self.tracker.position + delta
        t_end = time.monotonic() + self.args.probe_timeout
        while abs(goal - self.tracker.position) > 0.1:
            t0 = time.monotonic()
            if t0 > t_end:
                raise Abort(f"{label}: back-off did not complete")
            err = goal - self.tracker.position
            vmax = max(self.args.seek_vel, self.args.probe_vel)
            vel = max(-vmax, min(vmax, 4.0 * err))
            self.step(vel)
            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t0)))

    def double_touch(self, direction: int, label: str) -> float:
        """Touch a stop twice from the same direction; require agreement.
        Touch 1 is a fast seek (padded torque threshold — friction rises with
        speed); touch 2 is slow and is the datum."""
        seek_thresh = None
        if self.args.contact_torque and self.args.contact_torque > 0:
            seek_thresh = self.args.contact_torque + self.args.seek_torque_pad
        t1 = self.probe(direction, f"{label} seek", vel=self.args.seek_vel,
                        contact_torque=seek_thresh)
        self.move_by(-direction * self.args.backoff, f"{label} back-off")
        t2 = self.probe(direction, f"{label} touch")
        if self.args.single_touch:
            # trust the slow touch without the agreement check (toggle for
            # high/lumpy-friction units where the check keeps false-aborting)
            if abs(t1 - t2) > self.args.touch_tol:
                print(f"    note: seek/touch disagree by {abs(t1 - t2):.3f} rad "
                      f"(--single_touch: accepting the slow touch)")
            return t2
        if abs(t1 - t2) > self.args.touch_tol:
            raise Abort(f"{label}: touches disagree by {abs(t1 - t2):.3f} rad "
                        f"(> {self.args.touch_tol}) — soft obstruction or debris? "
                        f"clear the jaws and rerun")
        return t2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["full", "home", "touch"])
    parser.add_argument("--write", action="store_true",
                        help="touch mode: record the measured stop_closed datum "
                             "(and last_position) into the cal file")
    add_bus_args(parser)
    parser.add_argument("--cal", default=None, help="cal store path (default: $FASTGRIPPER_DM_HOME or ~/.config/fastgripper-dm/gripper_cal.json)")
    parser.add_argument("--gripper", default=None,
                        help="cal entry name (optional if the file has exactly one; "
                             "full mode creates the entry if it doesn't exist)")
    # One source of truth for the values GripperProfile also carries: the probe
    # caps were re-measured on 2026-09-02 (fastgripper-dm 0.1.1) and drifted apart
    # from these defaults, which then false-contacted on friction. Still overridable.
    _d = GripperProfile()
    parser.add_argument("--close_dir", type=int, choices=[1, -1], default=_d.close_dir,
                        help="motor velocity sign that closes the jaws (default +1)")
    parser.add_argument("--probe_vel", type=float, default=_d.probe_vel,
                        help="slow datum-touch speed, rad/s")
    parser.add_argument("--seek_vel", type=float, default=_d.seek_vel,
                        help="fast seek speed for the first touch and traverses, rad/s "
                             "(set equal to --probe_vel for all-slow behavior)")
    parser.add_argument("--seek_torque_pad", type=float, default=0.05,
                        help="Nm added to --contact_torque during the fast seek only "
                             "(friction torque rises with speed; bench 2026-07-11: "
                             "fast free-run mid-0.2s, max 0.28 -> seek trigger 0.35)")
    parser.add_argument("--probe_tmax", type=float, default=_d.probe_tmax,
                        help="probe torque cap, Nm at the motor (worm multiplies!); "
                             "must stay above --contact_torque or the probe can never "
                             "reach its own trigger")
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--stall_speed", type=float, default=0.1, help="rad/s")
    parser.add_argument("--stall_time", type=float, default=0.3, help="s")
    parser.add_argument("--contact_torque", type=float, default=_d.contact_torque,
                        help="Nm: trigger on sustained torque above this (bench-tuned "
                             "2026-09-02: free-run p95 0.36-0.44 on this unit, so the "
                             "old 0.30 default triggered on friction at the start "
                             "position). Pass 0 to disable and use velocity-stall only.")
    parser.add_argument("--contact_time", type=float, default=0.12,
                        help="s of sustained over-threshold torque to confirm contact")
    parser.add_argument("--backoff", type=float, default=_d.backoff, help="rad between touches")
    parser.add_argument("--touch_tol", type=float, default=_d.touch_tol,
                        help="max disagreement between double-touches, rad")
    parser.add_argument("--margin", type=float, default=_d.margin,
                        help="marks sit this far inside the physical stops, rad")
    parser.add_argument("--expected_span", type=float, default=None,
                        help="stop-to-stop distance from CAD/previous cal, rad (full mode)")
    parser.add_argument("--span_tol", type=float, default=0.5, help="rad")
    parser.add_argument("--max_travel", type=float, default=40.0,
                        help="abort if a single probe travels farther than this, rad")
    parser.add_argument("--probe_timeout", type=float, default=90.0, help="s per probe")
    parser.add_argument("--yes", action="store_true", help="skip the jaws-empty prompt")
    parser.add_argument("--single_touch", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="skip the double-touch agreement check; the slow touch "
                             "is the datum. DEFAULT ON: this mechanism's friction is "
                             "lumpy enough that double-touch false-aborts. "
                             "--no-single_touch restores the agreement check.")
    parser.add_argument("--closed_only", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="full mode: probe ONLY the closed stop; derive the open "
                             "mark as closed - close_dir * span (no traverse to the "
                             "open end). DEFAULT ON: the open end has no qualified "
                             "hard stop. --no-closed_only probes both stops.")
    parser.add_argument("--span_from_closed", type=float, default=None,
                        help="RANGE OF MOTION: rad of usable travel below the closed "
                             "mark. MUST be a conservative UNDERestimate of the real "
                             "travel, or 0%% commands drive into the open stop. "
                             "Falls back to the cal entry's saved span, then to "
                             f"{DEFAULT_SPAN_FROM_CLOSED} rad.")
    args = parser.parse_args()
    args.cal = args.cal or default_cal_path()

    store = load_store(args.cal)
    # range of motion: CLI > per-gripper saved value > package default
    if args.span_from_closed is None:
        saved = store["grippers"].get(args.gripper or "", {})
        args.span_from_closed = float(
            saved.get("span_from_closed", DEFAULT_SPAN_FROM_CLOSED))
        print(f"(range of motion: {args.span_from_closed} rad "
              f"{'from cal entry' if 'span_from_closed' in saved else '(package default)'})")
    if args.mode == "full" and args.gripper is not None and \
            args.gripper not in store["grippers"]:
        store["grippers"][args.gripper] = {}   # full mode may create a new entry
        name, cal = args.gripper, store["grippers"][args.gripper]
    elif args.mode == "full" and not store["grippers"]:
        name, cal = "default", store["grippers"].setdefault("default", {})
    else:
        name, cal = get_entry(store, args.gripper)
    print(f"gripper entry: {name}")

    if args.mode == "home" and "stop_closed" not in cal:
        raise SystemExit(f"entry '{name}' has no stop_closed datum — run 'autocal.py full' "
                         f"once (keyboard-calibrated entries can't be homed against a stop)")

    if args.mode == "full" and args.expected_span is None:
        args.expected_span = cal.get("stop_span")
        if args.expected_span is not None:
            print(f"using expected span from cal file: {args.expected_span:.2f} rad")
        else:
            print("WARNING: no --expected_span and none in the cal file — the span "
                  "sanity check is OFF for this run. Give the CAD value next time.")

    if not args.yes:
        if not sys.stdin.isatty():
            raise SystemExit("no terminal for the jaws-empty confirmation — pass --yes "
                             "after checking the mechanism is clear (headless/Jetson use)")
        if input("Jaws empty and mechanism clear? [y/N] ").strip().lower() != "y":
            raise SystemExit("aborted")

    with open_bus(args.interface, args.channel) as bus:
        motor_id, master_id = resolve_ids(args, cal)
        motor = DM4310(bus, can_id=motor_id, master_id=master_id)
        rig = Rig(motor, args)
        if args.mode == "touch" and cal.get("last_position") is not None:
            # report in the calibration's frame (worm gear can't back-drive,
            # so anchoring from the saved position is valid unless it's stale)
            rig.tracker = MultiTurnTracker(start_unwrapped=cal["last_position"])

        fb = None
        for _ in range(20):
            motor.clear_error()
            time.sleep(0.01)
            motor.enable()
            fb = motor.read_feedback(timeout=0.25)
            if fb:
                break
        if fb is None:
            for _ in range(3):
                motor.disable()
                time.sleep(0.02)
            raise SystemExit("no feedback from motor — check power / wiring / IDs")
        rig.fb = fb
        rig.tracker.update(fb.position)

        ok = False
        try:
            # entry's stored direction wins unless the CLI overrides it
            d = args.close_dir if args.close_dir != 1 else int(cal.get("close_dir", 1))
            print(f"probing closed stop (dir {d:+d}, {args.probe_vel} rad/s, "
                  f"cap {args.probe_tmax} Nm)...")
            stop_closed = rig.double_touch(d, "closed")

            if args.mode == "touch":
                if "closed" in cal:
                    delta = stop_closed - cal["closed"]
                    print(f"datum vs calibrated closed mark: {delta:+.3f} rad")
                    if abs(abs(delta) - MultiTurnTracker.SPAN) < 1.5:
                        print("  NOTE: that's ~one 25 rad encoder window — the saved "
                              "last_position was probably turn-aliased; a --write run "
                              "(or cal_doctor.py) will fix the anchor")
                rig.move_by(-d * (args.margin + args.backoff), "park back-off")
                if args.write and cal:
                    cal["stop_closed"] = stop_closed
                    cal["last_position"] = rig.tracker.position
                    cal["last_wrapped"] = rig.tracker.wrapped
                    cal["touched_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    save_store(args.cal, store)
                    print(f"wrote stop_closed={stop_closed:+.3f} to {args.cal} "
                          f"(home mode is now available)")
                else:
                    print("dry run — nothing written (add --write to record the datum)")
                ok = True
            elif args.mode == "home":
                # datum known in the cal frame; shift this session onto it
                offset = cal["stop_closed"] - stop_closed

                # SANITY GUARD (required on every homing path). `home` probes
                # only ONE stop, so nothing else constrains the re-anchor: a
                # probe that false-triggers on friction instead of the stop
                # yields an arbitrary offset, and writing it silently corrupts
                # position knowledge. Bench 2026-07-28: a probe triggered on a
                # friction hump (threshold 0.42 Nm vs measured free-run p95
                # 0.51) and produced last_position 16.7 rad BEYOND the closed
                # stop -- physically impossible, written without complaint.
                #
                # The boot encoder ambiguity is a whole number of output turns
                # (2*pi), so a legitimate re-anchor is a small number of turns
                # plus probe scatter. Anything larger means the probe missed.
                turns = offset / (2 * math.pi)
                if abs(offset) > HOME_MAX_REANCHOR_RAD:
                    raise Abort(
                        f"re-anchor offset {offset:+.2f} rad ({turns:+.1f} turns) "
                        f"exceeds the {HOME_MAX_REANCHOR_RAD:.1f} rad limit -- the "
                        f"probe almost certainly triggered on friction, not the "
                        f"stop. Nothing written. Raise --contact_torque above the "
                        f"printed free-run p95 (and --probe_tmax ~0.3 above that) "
                        f"and rerun, or re-calibrate with `fastgripper-dm calibrate`.")

                rig.move_by(-d * (args.margin + args.backoff), "park back-off")
                anchored = rig.tracker.position + offset
                lo = min(cal["open"], cal["closed"]) - HOME_MARK_SLACK_RAD
                hi = max(cal["open"], cal["closed"]) + HOME_MARK_SLACK_RAD
                if not (lo <= anchored <= hi):
                    raise Abort(
                        f"homed position {anchored:+.2f} rad falls outside the "
                        f"calibrated range [{lo:+.2f}, {hi:+.2f}] -- refusing to "
                        f"write. Re-calibrate with `fastgripper-ur jog`.")
                cal["last_position"] = anchored
                cal["last_wrapped"] = rig.tracker.wrapped
                cal["homed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
                print(f"re-anchored: last_position={cal['last_position']:+.3f} rad "
                      f"(offset {offset:+.3f})")
            elif args.closed_only:
                closed_mark = stop_closed - d * args.margin
                open_mark = closed_mark - d * args.span_from_closed
                print(f"closed-only: open mark derived at {open_mark:+.3f} rad "
                      f"(closed {closed_mark:+.3f} - {args.span_from_closed} rad)")
                cal.update(
                    motor_id=motor_id, master_id=master_id, close_dir=d,
                    open=open_mark, closed=closed_mark,
                    span=args.span_from_closed,
                    span_from_closed=args.span_from_closed,
                    stop_closed=stop_closed,
                    # open stop never measured; recorded as the derived mark
                    # minus margin so range guards stay conservative
                    stop_open=open_mark - d * args.margin,
                    stop_span=args.span_from_closed + 2 * args.margin,
                    method="autocal_closed_only",
                    calibrated_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                )
                # park at the closed mark (we're at the stop; back off margin)
                rig.move_by(-d * args.margin, "park at closed mark")
                cal["last_position"] = rig.tracker.position
                cal["last_wrapped"] = rig.tracker.wrapped
            else:
                rig.move_by(-d * args.backoff, "leave closed stop")
                print("traversing to open stop...")
                stop_open = rig.double_touch(-d, "open")
                span = abs(stop_closed - stop_open)
                print(f"measured stop-to-stop span: {span:.3f} rad "
                      f"({span / MultiTurnTracker.SPAN:.2f} encoder windows)")
                if args.expected_span is not None and abs(span - args.expected_span) > args.span_tol:
                    raise Abort(f"span {span:.2f} != expected {args.expected_span:.2f} "
                                f"(±{args.span_tol}) — object in jaws or mechanical "
                                f"problem; nothing written")
                cal.update(
                    motor_id=motor_id, master_id=master_id, close_dir=d,
                    open=stop_open + d * args.margin,
                    closed=stop_closed - d * args.margin,
                    span=abs((stop_closed - d * args.margin) - (stop_open + d * args.margin)),
                    stop_open=stop_open, stop_closed=stop_closed, stop_span=span,
                    method="autocal",
                    calibrated_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                )
                # park just inside the open mark
                rig.move_by(d * args.margin, "park")
                cal["last_position"] = rig.tracker.position
                cal["last_wrapped"] = rig.tracker.wrapped

            if args.mode != "touch":  # touch handles its own (optional) write
                save_store(args.cal, store)
                print(f"wrote entry '{name}' in {args.cal}")
            ok = True
        except Abort as e:
            print(f"\nABORT: {e}")
        finally:
            for _ in range(3):
                motor.mit_control(0.0, 0.0, 0.0, args.kd, 0.0)
                time.sleep(0.02)
            for _ in range(5):
                motor.disable()
                motor.read_feedback(timeout=0.1)
                time.sleep(0.05)
            print("motor disabled")

    sys.exit(0 if ok else 1)


def cli() -> None:
    from ._cli import run
    run(main)


if __name__ == "__main__":
    cli()
