# YAM + SO-101 teleop (adapter mode)

This is the **adapter path**, not the standalone quickstarts. The gripper's
DM-J4310 doesn't get its own CAN channel here — it rides motor 7 on an
i2rt YAM arm's `MotorChainRobot` alongside the six arm joints. `fastgripper-dm`
supplies the gripper math (multi-turn position unwrap, software velocity
loop, torque cap, stall-and-hold latch) through
`fastgripper_dm.adapters.i2rt.I2rtGripper`; the arm side (motion, CAN bus,
gravity comp) stays i2rt's. If you're driving a gripper alone on its own
CAN channel, use `quickstart-linux.md` or `quickstart-mac.md` instead — this
page assumes a YAM arm is already in the loop.

The bench reference implementation is `bench/yam/so101_teleop.py`: an SO-101
leader arm teleoperates the YAM follower arm (joint-space, delta mode), and
the SO-101's trigger drives the gripper open fraction through the adapter.

## Install

```sh
pip install fastgripper-dm
pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"
```

`fastgripper-dm` never imports i2rt itself — the adapter only touches the
`robot` object it's handed. i2rt is needed by whatever builds that robot
(your teleop script, or `bench/yam/so101_teleop.py`).

On macOS, apply the gs_usb fix into your venv once:

```sh
patches/setup-mac.sh <path-to-venv>
```

(see `patches/README.md` for what that patches, and `quickstart-mac.md` for
the rest of the macOS gs_usb behaviors — adapter wedging, `os._exit`
cleanup, idle-sleep freezing the loop).

## The `gripper_limits_override` requirement

`I2rtGripper` reads the gripper's position off `robot.get_joint_pos()` and
converts it from i2rt's normalized `[0, 1]` joint space back into real
wrapped-shaft radians:

```python
float(self.robot.get_joint_pos()[self.idx]) * SPAN - POS_WINDOW
```

That conversion is only correct if the robot's gripper joint limits span
exactly `SPAN` (`fastgripper_dm.port.SPAN`), centered on zero. Build the
robot with `gripper_limits_override` set accordingly — from
`bench/yam/so101_teleop.py`:

```python
robot = get_yam_robot(
    channel=args.channel,
    gripper_type=GripperType.LINEAR_4310 if with_gripper else GripperType.NO_GRIPPER,
    gripper_limits_override=np.array([-POS_WINDOW, POS_WINDOW]) if with_gripper else None,
    zero_gravity_mode=False,
    ee_mass=args.ee_mass,
)
```

`I2rtGripper.connect()` enforces this itself: it reads
`robot.get_robot_info()["gripper_limits"]` and raises if the span doesn't
match `SPAN`, naming the required override in the error. Get this wrong (or
build the robot without a gripper limits override) and every position and
velocity the adapter reports is silently mis-scaled — the check exists so
that fails loud instead of quiet.

## Running the bench teleop script

```sh
I2RT_CAN_BUSTYPE=gs_usb .venv/bin/python -u bench/yam/so101_teleop.py \
    --leader_port <so101-leader-port> --seconds 180
```

Flags on `so101_teleop.py`:

| Flag | Default | Meaning |
|---|---|---|
| `--leader_port` | required | SO-101 leader serial port |
| `--gripper` | `yam` | gripper entry name in the `fastgripper-dm` cal file |
| `--channel` | `0` | YAM CAN channel |
| `--baud` | `1000000` | SO-101 leader baud rate |
| `--seconds` | none | optional session limit; default runs until Ctrl-C |
| `--no_gripper` | off | run the 6-dof arm only, skip the gripper adapter |
| `--home` | `auto` | `auto` refuses to start on a park mismatch; `off` skips the check (see below) |
| `--cal_file` | `bench/yam/gripper_cal.json` | fastgripper-dm cal store |

Requires a `fastgripper-dm` cal store (`gripper_cal.json`, the `--gripper`
entry) and an absolute SO-101 trigger calibration
(`bench/yam/so101_trigger_cal.json`) — trigger mapping is absolute only,
there is no delta-mode fallback; a missing trigger cal is a hard error, not
a silent behavior change. See `bench/yam/TELEOP.md` for the full runbook
(CAN adapter identification, preflight, the teleop GUI, tuning constants,
and troubleshooting) and `bench/yam/running-the-yam-gotchas.md` for
hardware-validated gotchas (multi-turn gripper, bus wedging, motor
identity).

## Park / homing

The DM-J4310's absolute encoder only covers ±12.5 rad; the gripper's ~33 rad
of travel is unwrapped in software, and that turn count doesn't survive a
power cycle. `I2rtGripper.connect()` re-establishes it from the saved park
position (the cal entry's `last_position` — the worm gear can't back-drive,
so a saved park is trusted exactly). Under the default `--home auto`, if the
gripper's current wrapped position doesn't match the saved park within
tolerance, `connect()` refuses to start rather than silently trusting a
stale wrap count, and its error names the fix:

```sh
fastgripper-dm autocal home --gripper <name>
```

Run that on a **dedicated channel** — the gripper motor alone on the bus —
never on the shared arm chain; there is no stall homing on a shared chain.
For an emergency session where that isn't possible, `--home off` skips the
park check entirely, but the gripper's frame is then unanchored (jaws-by-eye
only — don't trust `goto()` targets in that mode). On a clean exit, the
script saves the park position automatically; don't move the jaws by hand
between sessions.

Stuck? `fastgripper-dm doctor` then `docs/fastgripper-dm/troubleshooting.md`,
or `bench/yam/running-the-yam-gotchas.md` for YAM-specific bus/motor issues.
