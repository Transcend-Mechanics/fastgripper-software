# Running the YAM bench — gotchas & folklore

For anyone (human or Claude) driving the YAM arm + worm gripper from this
bench. Every item below was learned the hard way and hardware-validated;
ignore at your peril. Originally written 2026-07-28 for handoff to
`ur-test`; updated for the `bench/` + `fastgripper-dm` package layout.

## The one-page version

```bash
# health check FIRST, always (5 s, no motion):
fastgripper-dm doctor --interface gs_usb --channel 0

# manual gripper calibration (keyboard jog, live torque+RPM):
I2RT_CAN_BUSTYPE=gs_usb I2RT_CAN_RESPONSE_TIMEOUT=0.2 \
  fastgripper-dm calibrate --gripper yam --interface gs_usb --channel 0

# teleop (runs until Ctrl-C):
I2RT_CAN_BUSTYPE=gs_usb I2RT_CAN_RESPONSE_TIMEOUT=0.2 \
  bench/.venv/bin/python -u bench/yam/so101_teleop.py --leader_port /dev/cu.usbmodemXXXX
```

Recovery ritual after ANY crash/weirdness, in order:
1. `fastgripper-dm doctor` — it names the failure class for you.
2. No TX echo → unplug/replug the adapter USB. (LED: green at rest =
   healthy, blue at rest = leaked channel = replug.)
3. Motor LED flashing red → power-cycle the 24 V PSU (adapter replug does
   NOT clear a latched motor fault).
4. NEVER `dev.reset()` the adapter in software — on this Mac it knocks the
   device off USB entirely ([Errno 19]) until physical replug.

## Environment gotchas

- **Always use the bench venv** (`bench/.venv`, see `bench/README.md`) —
  system Python lacks pyusb and the darwin-patched gs_usb package; failures
  look like "no adapter found".
- **The CAN adapter creates NO serial port.** It's candlelight firmware =
  raw USB (VID 0x1D50, PID 0x606F). Detect with
  `system_profiler SPUSBDataType | grep canable`, never by scanning
  /dev/tty*. Any `/dev/cu.usbmodem*` you see is the SO-101 leader's serial
  chip — a different device.
- **Always pass `--interface gs_usb --channel 0` explicitly** when it
  matters. `fastgripper-dm`'s `auto` mode prefers serial ports and will
  happily open the SO-101's serial port as a CAN bus (silent garbage).
- **The SO-101 leader's port path drifts across replugs**
  (`/dev/cu.usbmodem...` with varying digits). `ls /dev/cu.usbmodem*`
  before every teleop session.
- **`I2RT_CAN_RESPONSE_TIMEOUT=0.2` on every i2rt-chain command.** Stock
  i2rt waits only 10 ms per motor reply; macOS gs_usb delivery is bursty
  and misses that window. Without the env var, chain init dies with
  "fail to communicate with the motor 1".
- Keep the CAN adapter on its own USB port, not a hub shared with the
  serial devices (suspected enumeration disturbance when serial opens).

## The frame-hygiene law (root cause of a week of "wedges")

**Every CAN frame you cause must be consumed, and the bus must be read dry
before the channel is closed.** Closing a gs_usb channel with unread frames
in the adapter's pipeline wedges its TX path until physical replug. The
`fastgripper-dm` package's `damiao/canbus.py` (all bus-opening tools in
`bench/` route through it) drains-on-shutdown and the patched i2rt drains
before motor bring-up. If you write NEW code that opens the bus: read after
every send, and drain before close. Relatedly, all CLI tools exit via
`os._exit()` after cleanup — Python interpreter finalization crashes inside
libusb on macOS (SIGABRT) and can poison the adapter for the next session.
Do the same in new tools.

## Motor identity gotchas

- The YAM chain hardcodes the gripper motor at **CAN 0x07 / feedback 0x17**
  (i2rt `get_robot.py` appends `[0x07, ...]`). A factory-fresh DM-J4310 is
  **0x01 — which collides with arm joint J1**. Provision with
  `fastgripper-dm id` with the motor ALONE on the bus (the CLI enforces it).
- Factory master_id is 0x00, so a fresh motor answers on arbitration 0x000.
- Motors re-ID'd above 0x0F overflow the status nibble in feedback byte 0;
  `dm4310.py:decode_feedback` normalizes this — replicate if writing a new
  decoder.
- DM motors have `timeout=0` (no CAN watchdog): they hold their last
  command forever if the host dies. Every exit path must disable motors.

## The multi-turn gripper (why everything is complicated)

- The DM-J4310 reports position only inside ±12.5 rad; gripper travel is
  ~33 rad (5+ turns). The `fastgripper-dm` package's tracker unwraps in
  software; **the turn count dies at power-off** (encoder is absolute only
  mod ~2π). Absolute position is re-established from the saved park
  (`gripper_cal.json`, `last_position` — worm can't back-drive, so the park
  is trusted exactly) or by homing against the closed stop
  (`fastgripper-dm autocal home`, ~20 s). The i2rt teleop adapter
  (`fastgripper_dm.adapters.i2rt.I2rtGripper`) enforces this at `connect()`
  time: `--home auto` (the default) refuses to start on a park mismatch
  rather than silently trusting a stale wrap count.
- Calibration store: `gripper_cal.json`, format 2, named entries (`yam`,
  `bench`, `hexapod_fr`, ...). The bench teleop defaults to entry `yam`.
- If "close stops early / open overruns": stale turn state. Fix =
  `fastgripper-dm autocal home`, never hand-math.
- This unit's friction is HIGH and LUMPY (free-run median ~0.3-0.5,
  p95 up to ~0.6+ Nm, worse near closed). Autocal needs
  `--contact_torque` above the printed free-run p95 and `--probe_tmax`
  ~0.3-0.4 above that, or it false-triggers "contact" mid-travel.
  Escape hatches: `--single_touch` (skip double-touch abort),
  `--closed_only --span_from_closed 33.5` (probe closed stop only, derive
  open; the 33.5 MUST be a conservative underestimate of true travel).
- The closed stop is compliant (flexures squish) — the closed mark depends
  on probe torque. `--margin` (default 0.75 rad) sets how far inside the
  stop the usable mark sits; raise it if "closed" over-squeezes.

## Teleop gotchas

- **Delta mode**: both arms' poses AT LAUNCH are the zero reference. Pose
  the SO-101 to roughly mirror the YAM before starting, or live with the
  offset all session. Wrist roll is the worst for this (can't eyeball a
  cylinder's rotation) — expect asymmetric range on roll.
- Pre-flight refuses to start if any YAM joint is outside soft limits —
  hand-nudge the named joint slightly and relaunch. It's usually parked
  0.05 rad past a limit, not broken.
- Trigger→jaw is **absolute only** (needs `bench/yam/so101_trigger_cal.json`;
  re-run the trigger calibration if the leader hardware changes). The old
  delta-mode fallback (rebase from trigger deltas when no cal file is
  present) was dropped in the bench migration — a missing trigger cal is
  now a hard error, not a silent behavior change.
- Runs until Ctrl-C (no timer). Clean exit saves the gripper park.
- Gripper force/speed constants (`tmax`, `vmax`, etc.) now live in the
  `fastgripper-dm` gripper profile (cal entry `profile` block / package
  defaults), not at the top of `so101_teleop.py`. **Do not run high TMAX
  values (>2 Nm) unattended — torque transients snapped the v1 worm.** kd is
  derived as `tmax/vmax` so the cap holds through impacts; jaw MOMENTUM is
  not capped and grows with speed.
- During gripper-only CLI sessions the ARM is PD-held at its startup pose
  (kp=15) — a limp arm drifts upward under i2rt's always-on gravity comp.

## Diagnostic toolbox

| Tool | What it answers | Motion? |
|---|---|---|
| `fastgripper-dm doctor` | echo? which motors alive? fault codes decoded (undervoltage / comm-loss / etc.) | none |
| `bench/tools/bus_dump.py` | raw frame stream — stale floods, missing replies | none (one poke) |
| `fastgripper-dm cal-doctor` | which turn interpretation matches the saved state | none |
| `fastgripper-dm autocal touch` | probe tuning dry-run; prints free-run torque stats | gentle |
| `bench/tools/rpm_stats.py` | motor speed distribution from teleop CSVs (RPM) | none |
| `I2RT_CAN_DEBUG=1` | per-frame send/skip/got trace during chain init | — |

## Known-good reference values (this bench, 2026-07-28)

- Bus: classic CAN 2.0, 1 Mbit/s, 11-bit IDs. Arm 0x01-0x06 (replies
  0x11-0x16), gripper 0x07/0x17.
- Cal entry `yam`: closed +31.49, open −2.01 (closed-only, span 33.5),
  method `autocal_closed_only`.
- Gripper profile: `vmax 24`, `sw_kp 24`; `tmax` has been experimented
  upward — check the cal entry's `profile` block before trusting it.
- Motor speed demand (measured): median ~100 RPM, p95 ~165, peak ~180
  while moving. A <100 RPM motor will feel sluggish on this mechanism.
