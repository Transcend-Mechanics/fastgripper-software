"""SO-101 leader -> YAM follower teleop bridge (joint-space, delta mode)
with the FastGripper i2rt adapter driving the gripper (motor 7).

Arm: delta mode -- both arms' poses at startup are the reference; SO-101
joint deltas steer the mapped YAM joints. No calibration, no startup jump.

Gripper (motor 7): the multi-turn worm-gripper math (position window
unwrap, software velocity loop, torque cap, stall latch) lives in
`fastgripper_dm.adapters.i2rt.I2rtGripper` now -- this script only converts
the SO-101 trigger reading into a 0..1 "open fraction" and hands it to
`grip.goto()` every cycle. Needs a fastgripper-dm cal store (gripper_cal.json,
`--gripper` entry name) and an absolute SO-101 trigger calibration
(so101_trigger_cal.json) -- there is no delta-mode trigger fallback anymore;
run the trigger calibration tool first if that file is missing.

CONVENTION: the trigger fraction below is 0 = released (open) .. 1 =
squeezed (closed); the controller's `goto_frac` is 0 = closed .. 1 = open.
The boundary between the two conventions is inverted explicitly at the
`grip.goto(1.0 - frac_squeeze)` call -- do not "simplify" that away.

Joint map (SO-101 -> YAM): pan->J1  lift->J2  elbow->J3  wristflex->J4
                           wristroll->J6  trigger->gripper (J5 held)

Usage:
  I2RT_CAN_BUSTYPE=gs_usb .venv/bin/python -u bench/yam/so101_teleop.py \
      --leader_port /dev/cu.usbmodemXXXX --seconds 180
  (add --no_gripper to run the 6-dof arm only)

A lost/uncertain gripper park refuses to start under `--home auto` (the
default) with a message naming the fix: `fastgripper-dm autocal home
--gripper <name>` on a dedicated channel (the gripper motor ALONE on the
bus), or re-anchor the cal entry by hand. For an emergency session where
neither is possible, `--home off` skips the park check entirely -- the
gripper's frame is then unanchored (jaws-by-eye only; do not trust
`goto()` targets in that mode).
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import scservo_sdk as scs

from fastgripper_dm.adapters.i2rt import I2rtGripper
from fastgripper_dm.port import POS_WINDOW

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import GripperType

# Die WITHOUT interpreter finalization on any exit: teardown crashes in
# libusb on macOS (SIGABRT/SIGSEGV during GC) and can poison the adapter for
# the next session. Hardware cleanup runs in the shutdown path below first.


def _fast_excepthook(exc_type, exc, tb):
    import traceback as _traceback
    _traceback.print_exception(exc_type, exc, tb)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(130 if exc_type is KeyboardInterrupt else 1)


sys.excepthook = _fast_excepthook

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- SO-101 leader ----------------------------------------------------------
TICKS_PER_REV = 4096
PRESENT_POSITION_ADDR = 56  # STS3215
SO_IDS = [1, 2, 3, 4, 5, 6]

# so_index -> (yam_joint_index, sign, scale) for the ARM joints
JOINT_MAP = {
    0: (0, -1.0, 1.0),  # shoulder_pan  -> J1
    1: (1, +1.0, 1.0),  # shoulder_lift -> J2
    2: (2, -1.0, 1.0),  # elbow_flex    -> J3 (J3 range is one-sided 0..3.66)
    3: (3, -1.0, 1.0),  # wrist_flex    -> J4
    4: (5, -1.0, 1.0),  # wrist_roll    -> J6
}
TRIGGER_SO_INDEX = 5

MAX_DELTA_RAD = 1.8   # arm: max deviation from YAM start pose per joint
MAX_STEP_RAD = 0.05   # arm: max change per cycle (rate limit)
LOOP_HZ = 50.0

parser = argparse.ArgumentParser()
parser.add_argument("--leader_port", type=str, required=True)
parser.add_argument("--gripper", type=str, default="yam",
                    help="gripper entry name in the fastgripper-dm cal file")
parser.add_argument("--baud", type=int, default=1000000)
parser.add_argument("--channel", type=str, default="0")
parser.add_argument("--seconds", type=float, default=None,
                    help="optional session limit; default runs until Ctrl-C "
                         "(matching stock i2rt behavior)")
parser.add_argument("--ee_mass", type=float, default=0.4)
parser.add_argument("--no_gripper", action="store_true")
parser.add_argument("--home", choices=["auto", "off"], default="auto",
                    help="'auto' (default) refuses to start on a park "
                         "mismatch; 'off' skips the park check for an "
                         "emergency, frame-unanchored session")
parser.add_argument(
    "--cal_file",
    type=str,
    default=os.path.join(HERE, "gripper_cal.json"),
)
parser.add_argument(
    "--log_dir",
    type=str,
    default=os.path.join(HERE, "logs"),
)
args = parser.parse_args()

os.makedirs(args.log_dir, exist_ok=True)
log_path = os.path.join(args.log_dir, time.strftime("teleop_%Y%m%d_%H%M%S.csv"))
log_file = open(log_path, "w")
log_file.write("time_s,trigger_rad,goal_rad,gripper_pos_rad,gripper_vel_rad_s,"
               "torque_nm,stall_hold\n")
print(f"logging to {log_path}")

TRIGGER_CAL_FILE = os.path.join(HERE, "so101_trigger_cal.json")

with_gripper = not args.no_gripper
trig_cal = None
if with_gripper:
    try:
        trig_cal = json.load(open(TRIGGER_CAL_FILE))
    except OSError:
        raise SystemExit(
            f"{TRIGGER_CAL_FILE} not found -- an absolute trigger calibration "
            "is required (the delta-mode fallback was dropped; see the module "
            "docstring). Generate it with so101_trigger_cal.py, or pass "
            "--no_gripper to run the 6-dof arm only.")
    print(f"trigger cal loaded: released={trig_cal['released']:+.3f} rad "
          f"squeezed={trig_cal['squeezed']:+.3f} rad")

# ---- keep the host awake for the life of this process -----------------------
# An idle-sleeping Mac froze the control loop for 91 s on 2026-08-29; the
# arm's 8 s CAN watchdog latched every motor. `caffeinate -w <pid>` holds
# display/idle/system sleep off and exits with us -- no timers to re-arm.
if sys.platform == "darwin":
    import subprocess
    try:
        subprocess.Popen(["caffeinate", "-dims", "-w", str(os.getpid())],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass

# ---- SO-101 -----------------------------------------------------------------
packet = scs.PacketHandler(0)
port = None
reader = None


def _open_so101() -> None:
    """(Re)open the leader's serial port and rebuild the sync reader."""
    global port, reader
    if port is not None:
        try:
            port.closePort()
        except Exception:
            pass
    port = scs.PortHandler(args.leader_port)
    assert port.openPort(), f"cannot open {args.leader_port}"
    assert port.setBaudRate(args.baud), "cannot set baud"
    reader = scs.GroupSyncRead(port, packet, PRESENT_POSITION_ADDR, 2)
    for sid in SO_IDS:
        reader.addParam(sid)


_open_so101()


def _reopen_so101(timeout_s: float = 6.0) -> bool:
    """The leader's USB serial chip re-enumerates when its (bus-powered) hub
    sags -- seen live 2026-08-29 whenever the SO-101 pair's follower powered
    up on the same hub. The old file descriptor is then dead for good; the
    device comes back on the same path within a few seconds. Reopen by path
    instead of dying; the YAM holds its last goal meanwhile."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(0.25)
        if not os.path.exists(args.leader_port):
            continue
        try:
            _open_so101()
            _read_so101_once()
            print(f"\nSO-101 leader reconnected after {time.time() - t0:.1f}s")
            return True
        except Exception:
            continue
    return False


def _read_so101_once() -> np.ndarray:
    if reader.txRxPacket() != scs.COMM_SUCCESS:
        raise IOError("SO-101 sync read failed")
    out = []
    for sid in SO_IDS:
        if not reader.isAvailable(sid, PRESENT_POSITION_ADDR, 2):
            raise IOError(f"servo {sid} not available")
        out.append(reader.getData(sid, PRESENT_POSITION_ADDR, 2))
    return np.array(out) * (2 * np.pi / TICKS_PER_REV)


def read_so101(max_tries: int = 25) -> np.ndarray:
    """Feetech sync reads hiccup transiently; retry ~0.5s before giving up."""
    for attempt in range(max_tries):
        try:
            return _read_so101_once()
        except IOError:
            if attempt == max_tries - 1:
                print("\nSO-101 leader not answering -- reopening its port...")
                if _reopen_so101():
                    return _read_so101_once()
                raise
            time.sleep(0.02)


s_prev = read_so101()
print("SO-101 online, joints (rad):", np.round(s_prev, 3))

# ---- pre-flight: joint limits ------------------------------------------------
# Read each arm joint BEFORE constructing the robot: an out-of-bounds joint
# makes robot init raise mid-USB-transaction, which wedges the adapter. Exit
# with instructions instead.
PREFLIGHT_LIMITS = {1: (-2.6, 3.0), 2: (-0.1, 3.6), 3: (-0.1, 3.6),
                    4: (-1.55, 1.55), 5: (-1.55, 1.55), 6: (-2.2, 2.2)}
JOINT_HINT = {6: "twist the whole gripper assembly (doorknob motion)",
              5: "tilt the wrist up/down", 4: "rotate the forearm"}
from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface, MotorType

_pf = DMSingleMotorCanInterface(channel=args.channel, control_mode=ControlMode.MIT)
_pf_bad = []
for _mid, (_lo, _hi) in PREFLIGHT_LIMITS.items():
    try:
        _info = _pf.motor_on(_mid, MotorType.DM4310)
        _pf.motor_off(_mid)
    except Exception:
        print(f"pre-flight: motor {_mid} not answering -- check power/wiring")
        _pf.close()
        sys.exit(1)
    if not (_lo <= _info.position <= _hi):
        _pf_bad.append((_mid, _info.position, _lo, _hi))
_pf.close()
if _pf_bad:
    for _mid, _pos, _lo, _hi in _pf_bad:
        hint = JOINT_HINT.get(_mid, "move this joint")
        direction = "positive" if _pos < _lo else "negative"
        print(f"PRE-FLIGHT FAIL: J{_mid} at {_pos:+.2f} rad, outside [{_lo}, {_hi}].")
        print(f"  -> hand-move it in the {direction} direction: {hint}")
    sys.exit(1)
print("pre-flight OK: all joints inside limits")

# ---- YAM --------------------------------------------------------------------
# Gripper limits set to the raw feedback window so the i2rt adapter's
# normalized<->wrapped-rad conversion is exact (I2rtGripper.connect()
# enforces this).
robot = get_yam_robot(
    channel=args.channel,
    gripper_type=GripperType.LINEAR_4310 if with_gripper else GripperType.NO_GRIPPER,
    gripper_limits_override=np.array([-POS_WINDOW, POS_WINDOW]) if with_gripper else None,
    zero_gravity_mode=False,
    ee_mass=args.ee_mass,
)
n = robot.num_dofs()
y_start = robot.get_joint_pos().copy()
print(f"YAM online, {n} dofs at", np.round(y_start, 3))

# If a mapped joint rests at/near its limit, neutral leader pose would have
# zero travel in one direction (e.g. J4 parks at +1.61 vs a +1.57 limit).
# Nudge the start pose inside the limits; the rate limiter glides it there.
YAM_LIMITS = {0: (-2.618, 3.054), 1: (0.0, 3.65), 2: (0.0, 3.665),
              3: (-1.5708, 1.5708), 4: (-1.5708, 1.5708), 5: (-2.094, 2.094)}
LIMIT_MARGIN = 0.15
# wrist flex centers at startup so a neutral leader = a neutral follower
# wrist with full travel both ways (a wrist-only motion, low risk)
START_CENTERED = {3: 0.0}
for _, (yam_j, _s, _sc) in JOINT_MAP.items():
    lo, hi = YAM_LIMITS[yam_j]
    if yam_j in START_CENTERED:
        nudged = START_CENTERED[yam_j]
    else:
        nudged = float(np.clip(y_start[yam_j], lo + LIMIT_MARGIN, hi - LIMIT_MARGIN))
    if abs(nudged - y_start[yam_j]) > 1e-6:
        print(f"J{yam_j + 1} starts at {y_start[yam_j]:+.2f} -- "
              f"gliding to {nudged:+.2f} at session start")
        y_start[yam_j] = nudged

GRIP = n - 1 if with_gripper else None
kp = robot._kp.copy()
kd = robot._kd.copy()

grip = None
if with_gripper:
    grip = I2rtGripper(robot, joint_index=GRIP, gripper=args.gripper,
                       cal_path=args.cal_file)
    try:
        grip.connect(home=args.home)
    except ValueError as e:
        robot.close()
        port.closePort()
        raise SystemExit(f"gripper connect failed: {e}")
    print(f"gripper: connected, position={grip.position:+.2f} rad")

s_ref = read_so101()
s_unwrapped = s_ref.copy()
target = y_start.copy()
print_ctr = 0
log_ctr = 0

print(f"\nTELEOP LIVE for {args.seconds:.0f}s\n" if args.seconds
      else "\nTELEOP LIVE -- Ctrl-C to stop\n")
t0 = time.monotonic()
try:
    while args.seconds is None or time.monotonic() - t0 < args.seconds:
        t_loop = time.monotonic()
        s_now = read_so101()
        step = np.mod(s_now - s_prev + np.pi, 2 * np.pi) - np.pi
        s_unwrapped = s_unwrapped + step
        s_prev = s_now

        # --- arm joints: delta mode ---
        desired = y_start.copy()
        for so_i, (yam_j, sign, scale) in JOINT_MAP.items():
            delta = sign * scale * (s_unwrapped[so_i] - s_ref[so_i])
            desired[yam_j] = y_start[yam_j] + np.clip(delta, -MAX_DELTA_RAD, MAX_DELTA_RAD)
        target = target + np.clip(desired - target, -MAX_STEP_RAD, MAX_STEP_RAD)

        vel = np.zeros(n)
        if with_gripper:
            frac_squeeze = (s_now[TRIGGER_SO_INDEX] - trig_cal["released"]) / (
                trig_cal["squeezed"] - trig_cal["released"])          # 0 released .. 1 squeezed (today's math)
            grip.goto(1.0 - max(0.0, min(1.0, frac_squeeze)))          # controller: 0=closed, 1=open
            pos_n, vel_n, kp_g, kd_g = grip.command_tuple(1.0 / LOOP_HZ)
            target[GRIP], vel[GRIP], kp[GRIP], kd[GRIP] = pos_n, vel_n, kp_g, kd_g

            print_ctr += 1
            if print_ctr >= 5:  # ~10Hz telemetry
                if sys.stdout.isatty():
                    print(f"\rgrip {grip.position:+7.2f} -> {grip.goal:+7.2f} rad   "
                          f"vel {grip.velocity:+5.1f}   torque {grip.torque:+5.2f} Nm   "
                          f"{'STALLED' if grip.stalled else '       '}",
                          end="", flush=True)
                else:  # machine-readable stream for the GUI
                    print(f"TLM pos={grip.position:.3f} goal={grip.goal:.3f} "
                          f"vel={grip.velocity:.2f} eff={grip.torque:.3f}", flush=True)
                print_ctr = 0

            log_file.write(f"{time.monotonic() - t0:.3f},"
                           f"{s_now[TRIGGER_SO_INDEX]:.4f},{grip.goal:.4f},"
                           f"{grip.position:.4f},{grip.velocity:.3f},{grip.torque:.4f},"
                           f"{int(grip.stalled)}\n")
            log_ctr += 1
            if log_ctr >= 50:  # flush once a second
                log_file.flush()
                log_ctr = 0

        robot.command_joint_state({"pos": target, "vel": vel, "kp": kp, "kd": kd})

        dt = time.monotonic() - t_loop
        time.sleep(max(0.0, 1.0 / LOOP_HZ - dt))
except KeyboardInterrupt:
    pass
finally:
    print("\nshutting down...")
    if with_gripper and grip is not None and getattr(grip, "_connected", False):
        try:
            grip.park()
            print(f"park saved: {grip.position:+.2f} rad")
        except Exception as e:
            print(f"WARNING: gripper park not saved: {e}")
    log_file.close()
    print(f"session log: {log_path}")
    robot.close()
    port.closePort()
    print("done, torques zeroed")

# skip interpreter finalization (see excepthook note near the imports)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
