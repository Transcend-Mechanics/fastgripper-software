# YAM + SO-101 Teleop Runbook

All commands run from the repo root unless noted. The bench venv lives in
`bench/.venv` (see `bench/README.md` for setup) and has both
`fastgripper-dm` and `i2rt` installed.

The gripper math (multi-turn unwrap, software velocity loop, torque cap,
stall-and-hold latch) now lives in the `fastgripper-dm` package's i2rt
adapter (`fastgripper_dm.adapters.i2rt.I2rtGripper`) — `so101_teleop.py`
only converts the SO-101 trigger reading into an open-fraction and calls
`grip.goto()` each cycle. Trigger mapping is **absolute only**: the
delta-mode trigger fallback from the old standalone script was dropped in
the bench migration (no calibration = hard error, not a silent fallback).
Bus/motor diagnostics, calibration, and homing now go through the
`fastgripper-dm` CLI instead of the old standalone scripts.

**Two different CAN adapters are in play — check which one is plugged in first**,
because the bustype and channel differ. `ls /dev/tty.usbmodem*`:

| Adapter | USB ID | Bustype | Channel |
|---|---|---|---|
| CANable2, **slcan** firmware (Openlight) | `16d0:117e` | `slcan` | the `/dev/tty.usbmodem*` port |
| CANable2, **candlelight** firmware | `1d50:606f` | `gs_usb` | `0` (USB index, no serial port) |

Session setup — edit these two lines to match the adapter in hand:

```bash
PY=bench/.venv/bin/python
export I2RT_CAN_BUSTYPE=slcan               # or gs_usb
CAN=/dev/tty.usbmodemXXXXXXXXXXX            # or 0 for gs_usb
LEADER=/dev/cu.usbmodemYYYYYYYYYYY          # SO-101 (CH343, 1a86:55d3)
```

The `fastgripper-dm` CLI tools default `--interface auto`, which prefers a
serial/slcan adapter and falls back to gs_usb, so they usually need no
`--interface`/`--channel` flags once `fastgripper-dm setup` has recorded
your bus (see `fastgripper-dm --help`).

## Preflight (run before any session)

```bash
fastgripper-dm preflight --gripper yam        # no-motion go/no-go: bus, motors, fault, cal entry
```

Go/no-go in ~5 s, no motion. Each FAIL line names the fix (replug adapter /
power-cycle 24 V PSU / …). Low-level equivalent: `fastgripper-dm doctor`.
Motors 1–6 = arm, motor 7 = gripper DM4310 (ID `0x07`, feedback `0x17`).

`so101_teleop.py` itself still runs its own arm-joint preflight (soft-limit
check on motors 1–6) before building the robot — see "Joint limit violation"
below.

## Quick jog / roll-call (verify motion before teleop)

```bash
$PY -u bench/tools/wiggle_joints.py --channel $CAN        # YAM: each joint ±0.1 rad, 4 s each
$PY bench/so101/so101_monitor.py --leader_port $LEADER --seconds 60   # leader: which servo moves
```

`wiggle_joints.py` proves the follower moves and confirms which index is
which joint; `so101_monitor.py` is the leader-side equivalent (move ONE
joint at a time and watch which delta grows) — that's how `JOINT_MAP` was
verified.

## Teleop GUI (recommended)

```bash
cd bench/yam && ../.venv/bin/python teleop_gui.py
```

One window: START/STOP button (clean SIGINT shutdown), session length,
live gripper position gauge (with goal marker), torque bar with the 2 Nm
cap marked, and the teleop's log stream. The GUI runs `so101_teleop.py`
as a subprocess and never touches the CAN bus itself. Edit `LEADER_PORT`
and `CAL_FILE` at the top of `teleop_gui.py` for your bench (or point it
at `bench/local.toml`'s `leader_*` entries).

## Teleop (command line)

```bash
$PY -u bench/yam/so101_teleop.py --leader_port $LEADER --channel $CAN     # runs until Ctrl-C
```

- Arm: delta mode — both arms' start poses are "neutral"; joints near a limit
  auto-glide inward at startup for two-way travel.
- Gripper: absolute only — trigger released = open, squeezed = closed
  (needs `so101_trigger_cal.json`; there is no delta-mode fallback).
- Status line prints gripper position/goal/velocity/**torque** at ~2 Hz
  (machine-readable `TLM ...` line when stdout isn't a TTY, for the GUI).
- `--no_gripper` runs the 6-dof arm only. Ctrl-C exits cleanly.
- `--home {auto,off}` (default `auto`): `auto` refuses to start on a park
  mismatch between the saved cal entry and the gripper's actual position —
  fix with `fastgripper-dm autocal home --gripper yam` on a dedicated
  channel (the gripper motor ALONE on the bus), or correct the cal entry by
  hand. `--home off` skips the check for an emergency, frame-unanchored
  session (jaws-by-eye only — do not trust `goto()` targets in that mode).
- On exit the gripper park position is saved automatically (do not move the
  jaws by hand between sessions).

### Tuning constants

Gripper force/speed/gain constants (`vmax`, `sw_kp`, `tmax`, derived `kd`)
now live in the `fastgripper-dm` gripper profile (the cal entry's
`profile` block, or the package defaults — see
`fastgripper_dm.profile.GripperProfile`), not at the top of
`so101_teleop.py`. Arm tuning constants are still local to the script:

| Constant | Current | Meaning |
|---|---|---|
| `MAX_STEP_RAD` | 0.05 | arm rate limit per 50 Hz cycle |
| `MAX_DELTA_RAD` | 1.8 | arm max deviation from start pose |
| `JOINT_MAP` | — | per-joint sign/scale for the arm |

Torque physics (unchanged): motor torque = `kd × (commanded − actual
velocity)`. The controller clamps commanded velocity so grip force can
never exceed the profile's `tmax_nm` regardless of speed settings. The worm
gear multiplies motor torque into much larger jaw force.

## Gripper tool (no leader arm needed)

```bash
fastgripper-dm open      --gripper yam   # glide fully open
fastgripper-dm close     --gripper yam   # glide fully closed
fastgripper-dm goto --pct 30 --gripper yam   # 30% closed
fastgripper-dm home       --gripper yam   # re-anchor via closed stop
fastgripper-dm calibrate --gripper yam   # keyboard jog + mark ends
```

- `calibrate` needs a real terminal: hold `a`/`d` to jog (dead-man), `o`/`c`
  mark open/closed, `q` saves `gripper_cal.json`.
- `home` fixes a stale/unknown gripper position (e.g. after a crashed session
  or if the jaws were moved by hand while powered off): drives slowly into the
  closed stop, re-anchors, backs off.
- Every mode saves the park position on exit.

## One-time calibrations (redo only if hardware changes)

```bash
# trigger endpoints (absolute gripper mapping)
$PY bench/so101/so101_trigger_cal.py --leader_port $LEADER

# assign a factory DM4310 the gripper identity 0x07/0x17
# (motor must be ALONE on the bus: harness straight into the motor)
fastgripper-dm id            # configure
fastgripper-dm id --verify   # after a power cycle, confirm it stuck
```

⚠️ Both DM4310s are configured `0x07` — never put both on the bus at once
(duplicate IDs poison the whole bus and look exactly like broken wiring).

## "Joint limit violation detected" at startup

The arm is physically resting with a joint past its soft limit (usually the
wrist after a session ends near end-of-travel). Fix: hand-move that joint
back toward the middle of its range and relaunch. The follow-on libusb
SIGABRT crash dump is cosmetic — but replug the adapter USB-C afterwards,
since the messy exit usually wedges it.

## When the bus acts up

Symptoms: pings return `[]` or a partial list, "no feedback from motor".

1. USB-C replug of the adapter (software reset is usually NOT enough)
2. If still bad: re-seat the CAN wires in the blue Wago levers (tug test),
   PSU off 5 s / on, then replug USB-C
3. Verify with **two** consecutive clean pings before launching anything
   (`fastgripper-dm doctor`)
4. Diagnostic: if a raw TX gets no echo, nothing is ACKing — wiring/motor
   side; if the adapter won't enumerate, it's USB side

## Ports

Suffixes are per-device serial numbers, but re-check with `ls /dev/cu.usbmodem*`
if anything was replugged. Fill the real paths into `bench/local.toml`
(git-ignored); `bench/local.toml.example` shows the format.

- CAN adapter (slcan build): CANable2 `16d0:117e`, "Openlight Labs"
- CAN adapter (candlelight build): no serial port at all; `1d50:606f`, channel `0`
- SO-101 leader: CH343 `1a86:55d3`

Identify an unknown port with `system_profiler SPUSBDataType | grep -i -A6 canable`.

## Diagnostic gotcha: TX echo on slcan

`fastgripper-dm doctor` reports `TX echo (bus ACK): NO` on the slcan adapter
even when the bus is perfectly healthy — the echo check is written for
candlelight's TX-echo behavior, which slcan does not reproduce. **On slcan,
ignore the echo line and the verdict that follows it; trust the per-motor
replies.** Motors answering = bus fine. (On gs_usb the echo line is
meaningful.)
