# fastgripper-dm — design spec

Date: 2026-08-30 · Status: rev 2, post-review · Owner: Alex
Review record: `review-correctness-2026-08-30.md` (inline report, applied),
`review-architecture-2026-08-30.md` (applied), `harvest-audit-2026-08-30.md`.

## 1. What this is

`fastgripper-dm` is the one Python package that drives the FastGripper
worm-gear parallel gripper when its actuator is a Damiao DM‑J4310 on CAN.
It replaces `fastgripper-openarm`, `fastgripper-yam`, and the DM‑4310 parts
of the `YAM Test` / `dm-j4310-test` / `gripper-openarm-portable` bench
folders, which today hold 2–8 diverged copies of the same modules.

It lives in a new monorepo, `fastgripper-software`, alongside the unchanged
SO‑101 LeRobot plugin `lerobot-robot-fastgripper` (Feetech STS3215 — a
different motor; it is *not* rewritten on top of this package).

The gripper is the same on every arm. What differs per arm is only **who
owns the CAN bus**:

| Mode | Who owns the bus | How the controller is driven | Arms |
| --- | --- | --- | --- |
| standalone | we do — a CAN channel dedicated to the gripper | `DamiaoCanPort` (python-can): synchronous command → reply | OpenArm, bench, any arm with a spare bus, the grippers now in manufacture |
| adapter | the arm's SDK (its own control thread, one command array for all joints) | the SDK's loop owner calls `controller.tick()` and merges the returned command into its own array; i2rt now, MakerMods/Almond later | YAM; MakerMods, Almond |

**Platforms:** Linux + SocketCAN is the shipped, first-class path. macOS +
gs_usb is the development bench and must keep working, with its quirks
isolated behind the port and in `patches/`. **Windows is unsupported in v0.**

### Goals

1. One implementation of the control algorithm, tested without hardware.
2. `pip install fastgripper-dm` → `fastgripper-dm preflight` → `calibrate` →
   `open/close/goto` on a standalone gripper, Linux, from the public docs
   alone (drill "cold start").
3. The YAM bench teleops from this package through the i2rt adapter before
   the old `YAM Test` folder is retired.
4. Every bench-won fix of the last two months (adapter wedge, drain on
   shutdown, no software USB reset on macOS, status-nibble normalisation
   for IDs > 0x0F, closed-only autocal, single-touch, home sanity guards,
   park/restore trust, preflight, usb watch/soak) lands once, here.
5. Adding an arm = one adapter file + one docs page + one drill run.
6. A dead host can never leave the motor pushing: the motor's own
   comm-loss watchdog is mandatory (§3.6).

### Non-goals (v0)

- UR (`fastgripper-ur`, `gripperd`) — later; its `autocal` improvements are
  harvested now, the UR daemon is not.
- MakerMods / Almond adapters — designed for, not built; each starts with a
  hardware question (flange, CAN IDs, bitrate) to the vendor.
- Rewriting `lerobot-robot-fastgripper` on this package.
- **Two grippers on one bus / one process (bimanual).** The cal store
  supports named entries with per-entry motor IDs; the CLI, `setup`, and
  port ownership are single-gripper in v0. Design rule for later: one
  `DamiaoCanPort` per bus, N controllers multiplexed by motor ID.
- GUIs beyond porting the existing `gui`/`pad` tools as-is.
- A ROS node. Windows.

## 2. Monorepo layout

```
fastgripper-software/
  packages/
    fastgripper-dm/                 this spec; own pyproject, own uv.lock, own tests
    lerobot-robot-fastgripper/      moved verbatim; own pyproject, own uv.lock, own tests
  bench/
    yam/        so101_teleop.py (on the adapter), teleop_gui.py, gripper_cal.json, so101_trigger_cal.json, TELEOP.md, gotchas
    so101/      trigger_probe.py, leader_drop_probe.py
    tools/      bus_dump.py, wiggle_joints.py, rpm_stats.py (not promoted into the package)
  patches/
    i2rt/       git submodule → Transcend-Mechanics/i2rt, branch `fastgripper` (§8)
    gs_usb-darwin.patch, ruckig-build.patch, setup-mac.sh
  drills/
    README.md, cards/, drill                                  (§9)
  docs/
    fastgripper-dm/ (quickstart per mode, per-arm pages, troubleshooting incl. usb-serial-drops, watchdog)
    lerobot-robot-fastgripper/ (moved)
  .github/workflows/
    ci.yml              matrix: {fastgripper-dm: py3.10/3.12 ubuntu+macos, lerobot-robot-fastgripper: py3.12 ubuntu}
    release-dm.yml      tags `dm-v*`;  `working-directory: packages/fastgripper-dm`; `uv build`; environment `pypi-dm`
    release-lerobot.yml tags `lerobot-v*`; working-directory its package; environment `pypi-lerobot`
  Makefile              dev shortcuts (`make dm-test`, `make lerobot-test`, `make bench-setup-mac`)
```

**Not a uv workspace.** The two packages need incompatible environments
(`lerobot[feetech]>=0.6,<0.7` + torch, Python ≥ 3.12 vs `python-can`,
Python ≥ 3.10) and a workspace would force one lockfile, one venv, and the
intersection of `requires-python`. Each package is resolved, locked, tested
and released independently; the repo root has no `pyproject.toml`.

Release: PyPI trusted publishing per package — each PyPI project gets its
own GitHub environment (`pypi-dm`, `pypi-lerobot`, plus `testpypi-*` for the
`workflow_dispatch` dry run), and each workflow gates on
`startsWith(github.ref, 'refs/tags/dm-v')` / `…/lerobot-v`.

Public/private: the repo is public. Rule: **no device serial numbers or
bench paths in the repo** — `usb_stress.py`'s hard-coded
`/dev/cu.usbmodem5C4C…` map is replaced by VID/PID discovery + a
git-ignored `bench/local.toml`; a CI grep for `usbmodem|/dev/cu\.` over
`bench/` and `packages/` fails the build. Drill *logs* (serials, timings,
gripes) go to a private `bench-drills` repo.

`.venv`, `logs/`, datasets, checkpoints git-ignored from day one.

## 3. Package `fastgripper-dm`

Distribution `fastgripper-dm`, import `fastgripper_dm`, Python ≥ 3.10 on
Linux (3.12 on the Mac bench because of i2rt/ruckig). Console script
`fastgripper-dm`.

```
fastgripper_dm/
  __init__.py          FastGripper facade, __version__
  port.py              MotorPort protocol, Feedback, MitCommand, PortError
  controller.py        GripperController: pure tick() core + run loop for owners of a sync port
  tracker.py           multi-turn unwrap (from so101_teleop.Tracker / fastgripper_yam.Tracker)
  calstore.py          format 2 named entries + profile block; atomic write
  profile.py           GripperProfile dataclass, presets (openarm, yam, ur)
  damiao/
    protocol.py        MIT frame encode/decode; status nibble = ((d[0] - (can_id & 0xFF)) >> 4) & 0xF for re-ID'd motors (dm4310.py:71)
    canbus.py          open_bus(interface, channel): socketcan | gs_usb | slcan | auto; TX-echo alive check; drain-on-shutdown + USB dispose; never dev.reset()
    can_port.py        DamiaoCanPort(MotorPort) — synchronous
    config_tool.py     motor id / master id set+verify (motor alone on bus, enforced); RID registers incl. timeout (§3.6)
  adapters/
    i2rt.py            I2rtGripper: tick-based adapter for a shared DMChainCanInterface (§3.2)
  tools/
    autocal.py, calibrate.py, drive.py, pad.py, gui.py, doctor.py, preflight.py, _cli.py
  cli.py               setup | preflight | doctor [usb soak|watch] | id | calibrate | autocal | home | open | close | goto | status | drive | pad | gui
  sim.py               SimulatedWormGripper(MotorPort) for tests
```

### 3.1 `MotorPort` — what a bus owner provides (standalone mode)

```python
@dataclass
class Feedback:
    position: float      # rad, output shaft, WRAPPED into ±pos_window (the port guarantees this)
    velocity: float      # rad/s
    torque: float        # Nm at the motor
    error_code: int      # DM status nibble: 0 disabled, 1 enabled, ≥8 fault
    t: float             # monotonic timestamp of this reading

@dataclass
class MitCommand:
    pos: float; vel: float; kp: float; kd: float; tau: float

class MotorPort(Protocol):
    pos_window: float                                  # ±12.5 rad for DM-J4310 MIT mode
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def command(self, cmd: MitCommand) -> Feedback: ... # sync: the reply to THIS frame
    def read(self) -> Feedback: ...                     # no motion (zero-gain command on DM)
    def clear_error(self) -> None: ...
    def close(self) -> None: ...                        # drain + release; idempotent
```

Rules: a port never interprets the gripper (no unwrap, no stall logic). It
retries a dropped frame within its own budget (≤ 0.2 s on macOS gs_usb) and
only then raises `PortError`. `close()` drains before shutting down and on
macOS never calls `dev.reset()`. `DamiaoCanPort` also implements fault
recovery: on `error_code ≥ 8`, `clear_error` + `enable` with backoff
(10 × 0.3 s), mirroring i2rt's `_try_recover_motors`.

### 3.2 `GripperController` — the algorithm, once; pure core

The controller has **no I/O in its core**:

```python
class GripperController:
    def __init__(self, cal_entry, profile, tracker=None): ...
    def tick(self, fb: Feedback, dt: float) -> MitCommand   # pure: unwrap, P-loop, cap, stall, homing state machine
    def run(self, port: MotorPort, hz=50)                    # convenience loop for owners of a SYNC port (CLI tools)
    # goals / queries
    def goto(self, frac), open(), close(), home(), park(); state, faulted, stalled
```

- **Standalone:** `run()` loops `fb = port.command(self.tick(fb, dt))`.
- **Adapter (i2rt, YAM):** i2rt's `DMChainCanInterface` runs its own
  250 Hz thread and `set_commands()` replaces the commands of **all**
  joints each call (dm_driver.py:744-765) and returns *cached* state
  (dm_driver.py:715-734). So no per-frame port is possible. The adapter
  `I2rtGripper` wraps the chain **read side only**: it derives the wrapped
  position as `get_joint_pos(idx) * SPAN − POS_WINDOW`, which requires the
  chain to be built with `gripper_limits_override = [−12.5, +12.5]`
  (so101_teleop.py:303-306, 339, 398; gripper.py:113, 142) — i2rt's own
  `absolute_positions` are already unwrapped and must not be fed to the
  tracker. The **owner of the chain** (the YAM teleop, or a robot class)
  calls `cmd = gripper.tick(fb, dt)` each cycle and merges `cmd` into its
  full 7-joint command array before `command_joint_state` / `set_commands`.
  The returned feedback is the latest cached state, which is fine for a
  P-loop on measured state; the design does not assume reply-to-this-frame
  in adapter mode. Exactly one writer to the chain — the owner; the adapter
  never calls `set_commands` itself. `motor_chain_robot`'s background loop
  and a controller `run()` are never active on the same chain.

Algorithm (ported from `so101_teleop.py` v7, the only copy with the torque
cap; `fastgripper_yam/gripper.py` shares the tracker and velocity mode but
has no cap and different gains, and is superseded):

- **Multi-turn tracking** (`tracker.py`): stroke ≈ 5.8 turns (≈ 33–37 rad)
  exceeds the ±12.5 rad window; consecutive readings that jump by more
  than half-span are wraps (dm4310.py:112-118, so101_teleop.py:113-118).
- **Velocity mode:** `kp = 0, kd = KD, pos = 0, tau = 0, vel = v_cmd`;
  torque ≈ `KD · (v_cmd − v_actual)`.
- **Software position loop:** `v_cmd = clip(SW_KP · (goal − pos), ±VMAX)`.
- **Hard torque cap:** `KD = TMAX / VMAX`, plus per-tick clamp of `v_cmd`
  to `v_actual ± TMAX/KD` (so101_teleop.py:410-411), so torque never
  exceeds `TMAX` through impacts. Defaults `VMAX 24 rad/s`, `SW_KP 24`,
  `TMAX 2.0 Nm` (documented cap; > 2 Nm snapped the v1 worm); `TMAX` is a
  profile field and `preflight` refuses a profile above the cap.
- **Stall / grasp hold:** `|v_actual| < STALL_V` for `STALL_T` with the goal
  unmet ⇒ stalled; hold at the cap.
- **Homing** (from URtest `autocal.py`, the only copy with the home-mode
  sanity guards): probe toward the closed stop, detect contact by torque
  (`contact_torque` above the unit's free-run p95), back off `margin`,
  anchor; **reject** a re-anchor > `HOME_MAX_REANCHOR_RAD` (~3 turns) or a
  homed position outside the cal marks ± `HOME_MARK_SLACK_RAD`
  (URtest autocal.py:337-369). `closed_only + span_from_closed` derives
  open from a conservative span; double-touch verification unless
  `single_touch`. All of these are **profile** fields (§3.3).
- **Park / restore.** Decision: the controller trusts `last_position`
  **exactly** (so101_teleop.py:111 semantics) because the worm cannot
  back-drive. The window-rounding tracker used by autocal today
  (dm4310.py:111, `round((pending − wrapped)/SPAN)·SPAN`) is **retired**;
  autocal uses the controller's tracker. **New, not ported:** at connect
  the wrapped boot reading must agree with `last_wrapped` within
  `park_tolerance_rad` (profile; default 0.35 rad ≈ the observed settle
  after torque-off) or `auto` mode stall-homes (default) / refuses
  (`auto_fallback="error"`). Tested in sim; verified on the bench in the
  damage-control drill.
- **Percent API:** 0 = closed mark, 1 = open mark, in the unwrapped frame;
  jaw direction via `close_dir`.

### 3.3 `calstore` + profile — format 2, named entries

`gripper_cal.json`: `{"format": 2, "grippers": {name: entry}}`; entry =
`open, closed, last_position, last_wrapped, span, stop_open, stop_closed,
stop_span, span_from_closed, method, calibrated_at, touched_at, homed_at,
motor_id, master_id, close_dir, profile`. Atomic write (tmp + fsync +
`os.replace`, from URtest `calstore.py:41-46`). Legacy flat files upgrade
to `default` on load (existing behaviour).

`profile` block (§3.3a of rev 1, folded in): `closed_only, single_touch,
span_from_closed, contact_torque, probe_tmax, margin, tmax_nm, vmax, sw_kp,
park_tolerance_rad, watchdog_ms`. Package defaults describe a
two-hardstop gripper (`closed_only=false, single_touch=false`); presets
`openarm`, `yam` (span 33.5), `ur` (closed_only, single_touch, span 30.0)
are selectable at `setup`. Tools read the profile; CLI flags override per
run. No physical constants at module level.

**Location precedence (same for `gripper_cal.json` and `config.json`):**
`--cal PATH` / `--config PATH` explicit → `$FASTGRIPPER_DM_HOME` →
`~/.config/fastgripper-dm/`. **No cwd lookup** (a stale file in the working
directory silently shadowing the real calibration drives `goto` toward
wrong stops). `calibrate --gripper` defaults to `None` = reuse the single
existing entry, never silently create a second.

### 3.4 CLI

```
fastgripper-dm setup      --interface socketcan --channel can0 [--profile openarm] [--motor-id 0x07]
                          writes config.json; WRITES AND VERIFIES the motor watchdog (§3.6)
fastgripper-dm preflight  adapter present · TX echo · motor answers · no latched fault · cal entry + park present
                          · entry.motor_id == the id that answered · profile.tmax ≤ 2.0 · watchdog register == profile.watchdog_ms
fastgripper-dm doctor     bus_ping with fault decode; `doctor usb watch|soak`
fastgripper-dm id         set|verify motor id / master id (motor must be alone on the bus; enforced by probing 1..8)
fastgripper-dm calibrate  keyboard jog, mark open/closed (real TTY)
fastgripper-dm autocal    full|home|touch; profile-driven defaults, flags override
fastgripper-dm home | open | close | goto <pct> | status
fastgripper-dm drive | pad | gui
```

Every tool exits through the shared `_cli.run` wrapper (disable motor →
drain → `os._exit`), never inline `os._exit`. On macOS the long-running
tools spawn `caffeinate -dims -w <pid>`.

### 3.5 Facade

```python
from fastgripper_dm import FastGripper
g = FastGripper.standalone(interface="socketcan", channel="can0", gripper="default")
with g:                       # connect(home="auto") … disconnect() parks + disables
    g.home(); g.goto(0.4); g.close(); print(g.state)

from fastgripper_dm.adapters.i2rt import I2rtGripper
grip = I2rtGripper(chain, joint_index=6, gripper="yam")   # read side only
cmd = grip.tick(dt)                                        # owner merges cmd into its command array
```

### 3.6 Motor watchdog — mandatory

A DM‑J4310 in MIT velocity mode keeps executing the last command until a
new frame or power loss; the gripper motor on the YAM bench has RID 9
`timeout = 0` (disabled) while the arm motors have 8000 ms. If the host is
SIGKILLed, panics, or its USB is yanked (the damage-control drill), a
disabled watchdog means the worm keeps pushing into a stop at `TMAX` until
someone pulls the power. Therefore:

- `setup` writes RID 9 to `profile.watchdog_ms` (default **500 ms**; the
  50 Hz loop sends every 20 ms, so this tolerates a 25-cycle stall) and
  reads it back; a mismatch fails `setup`.
- `preflight` verifies the register each run — NO-GO on mismatch or 0.
- A watchdog fault (`loss communication`) recovers through the port's
  clear/enable path once frames resume.
- Docs state the trade-off plainly: a host stall > 500 ms drops torque
  (the jaws hold anyway — the worm self-locks), which is the safe direction.

## 4. Behaviour on failure

| Situation | Behaviour |
| --- | --- |
| dropped CAN frame | port retries within budget; controller unaware |
| motor latched fault (comm-loss, overload) | port `clear_error` + `enable` with backoff (10 × 0.3 s); controller pauses the goal, `state.faulted` |
| adapter wedge (TX, no echo) | `preflight`/`doctor` name it and the fix (PSU on? replug adapter); never software-reset the adapter |
| park mismatch / missing | stall-home (default) or refuse (`auto_fallback="error"`) |
| host process killed / USB yanked | motor watchdog (§3.6) disables within `watchdog_ms`; worm self-locks; next `preflight` shows the latched fault and clears it on connect |
| host stall > watchdog | same as above, then automatic recovery when frames resume |
| Ctrl‑C / normal exit | park → disable → drain → `os._exit` via `_cli.run` |

## 5. Testing

- **Unit, no hardware:** `sim.SimulatedWormGripper` implements `MotorPort`
  with a ±12.5 rad wrapping window, viscous friction, compliant hard stops,
  and **torque computed exactly as the motor does**,
  `KD · (v_cmd − v_actual)` (so the cap assertion is not vacuous), plus
  injectable frame drops and faults. Tests: unwrap across wraps both ways;
  torque cap never exceeded (asserted on every simulated command); stall
  detection; homing finds the stop within `margin` and the sanity guards
  reject bad re-anchors; closed-only span derivation; park/restore accept,
  reject-then-stall-home, and refuse cases; controller survives injected
  port faults; cal store round-trip, legacy upgrade, entry selection, no
  cwd lookup; profile presets; CLI parsing; `tick()` is pure (same inputs →
  same command, no I/O).
- **Protocol:** encode/decode golden tests from captured frames, including
  a re-ID'd motor (0x20) and the status-nibble normalisation.
- **Adapter:** `I2rtGripper` tested against a fake chain exposing
  `get_joint_pos`/cached state; asserts it never calls `set_commands`.
- **Hardware drills** (`drills/cards/`): cold-start-linux, cold-start-mac,
  beat-to-quarters (< 3 min daily start), damage-control (SIGKILL the tool
  mid-move → motor stops within `watchdog_ms`; yank USB; hand-move jaws
  while off → recover from docs only), yam-teleop-from-monorepo. Pass
  criteria and time limits on each card.

## 6. Migration / harvest plan (order matters)

0. **Capture the hidden patches.** In `YAM Test/i2rt`: commit the 5
   uncommitted modified files (`can_interface.py`, `utils.py`,
   `motor_config_tool/*`) on top of `f732e4f` into branch `fastgripper`,
   push to the `Transcend-Mechanics/i2rt` fork, diff-verify against the
   working venv. Extract the venv `gs_usb` darwin patch
   (`gs_usb/gs_usb.py:56`) and the ruckig build patch into `patches/`.
   Nothing else starts until this is done — it is the only copy.
1. Skeleton, `port.py`, `tracker.py`, `controller.py`, `sim.py`,
   `profile.py`, `calstore.py` with tests — no hardware.
2. `damiao/protocol.py` + `canbus.py` (canonical `YAM Test/canbus.py`) +
   `DamiaoCanPort` + `config_tool` (watchdog); `doctor`/`preflight`/`usb`.
3. Standalone CLI end-to-end on the bench gripper (Mac gs_usb), then a Linux
   SocketCAN box (drill cold-start-linux) — **`dm-v0.1.0`**, the release for
   the grippers now in manufacture.
4. `adapters/i2rt.py` on the fork; `bench/yam/so101_teleop.py` switched to
   `I2rtGripper.tick()`; YAM drill passes → `YAM Test` folder archived.
5. Move `lerobot-robot-fastgripper` in verbatim; CI matrix; the two release
   workflows; serial-number grep gate.
6. Archive `fastgripper-openarm`, `fastgripper-yam`, `dm-j4310-test`,
   `gripper-openarm-portable`, `SO101-software`, `gripper-software` with a
   README pointing here. `URtest` stays (UR later).

Harvest bases and merges: Appendix A.

## 7. Cross-arm usage (what a customer sees)

`pip install fastgripper-dm` for standalone. Same verbs and the same Python
object on every arm; only `setup --interface/--channel` differs.

**YAM / i2rt:** the fork is **not on PyPI** and PyPI rejects git-URL
dependencies in metadata, so there is no `[i2rt]` extra in v0. The YAM docs
page gives the explicit step
`pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"`
and `fastgripper_dm.adapters.i2rt` imports i2rt lazily with a clear error
if absent. Publishing the fork as `i2rt-fastgripper` on PyPI is a later
option, contingent on i2rt's licence (check before step 4).

Docs: one quickstart per mode (standalone Linux, standalone Mac bench,
YAM), one troubleshooting section (USB serial/CAN drops, wedge, faults,
watchdog, park), one page per future arm.

## 8. Patched dependencies (make the hidden explicit)

Today the YAM works only because of: an i2rt clone with **one commit and
five uncommitted files** of patches (drain before bring-up, response-id
matching, `I2RT_CAN_RESPONSE_TIMEOUT`, gs_usb backend selection,
exception-safe recovery); a `gs_usb` PyPI package hand-patched inside a venv
(`is_kernel_driver_active` guard on darwin, `gs_usb.py:56`); a ruckig wheel
built from a patched sdist. Step 0 of §6 makes each reproducible: i2rt →
fork branch `fastgripper` (submodule in `patches/i2rt`, pip-installed by
URL per §7); gs_usb and ruckig → `patches/*.patch` applied by
`patches/setup-mac.sh`. Linux needs neither patch.

## 9. Drills

A drill is a card (public, `drills/cards/<name>.md`: objective,
preconditions, steps citing only public docs, pass criteria, time limit,
what to log) run from an empty folder under
`~/Documents/Transcend/Drills/<date>-<name>/` with a fresh venv.
`drills/drill` (≈150 lines) does `start <name>` (folder, venv, clock, env
snapshot: python, pip freeze, USB tree, preflight output), `note "…"`,
`finish pass|fail` (summary + issue list). Logs → private `bench-drills`
repo. Public/private checklist: no serial numbers, device paths, or
timings in the public repo (CI grep enforces the first two). The first run
of each drill is expected to fail; the issue list is the backlog.

## 10. Decisions log

- Names: `fastgripper-dm` / `fastgripper_dm`; monorepo `fastgripper-software`.
- No uv workspace; independent packages, independent lockfiles.
- No `[i2rt]` extra on PyPI; manual git install documented.
- Watchdog mandatory, 500 ms default, written by `setup`, verified by `preflight`.
- Park semantics: exact trust of `last_position`; rounding tracker retired.
- Profiles replace module-level physical constants; `_cli.run` is the one exit path.
- One gripper per bus/process in v0.
- `bench/yam` cal JSONs live in the public repo (no secrets; serials do not).
- Open: `MAX_DELTA_RAD` for the YAM teleop (1.8 vs 2.5) — bench decision.

## Appendix A — harvest audit (summary; full report: `harvest-audit-2026-08-30.md`)

| module | canonical base | merge in | discard |
| --- | --- | --- | --- |
| `canbus.py` | `YAM Test/canbus.py` (160 L): auto detect, open-without-reset + TX-echo alive check, drain-before-shutdown + USB-handle dispose, never `dev.reset()` | nothing | `gripper-openarm-portable` (88 L) and `dm-j4310-test` (72 L) **hard-reset the adapter on open** — the proven wedge cause; `fastgripper-openarm` (126 L) lacks the USB dispose |
| `dm4310.py` | any 204 L copy (status-nibble normalisation for IDs > 0x0F) | nothing | `dm-j4310-test` (199 L) |
| `calstore.py` | `fastgripper-openarm` (79 L: `default_cal_path`, makedirs) | atomic write from `URtest/calstore.py:41-46` | — |
| `calibrate.py` | `fastgripper-openarm` (216 L: entry resolution, `--gripper None`, peak torque, fast jog, `last_wrapped`) | nothing | `YAM Test`/`URtest` 195 L (talks to `args.motor_id` directly — the re-ID'd-gripper-talks-to-0x01 bug) |
| `autocal.py` | `URtest/fastgripper_ur` (461 L: home sanity guards L337-369, span fallback ladder, saves `span_from_closed`) | `default_cal_path()` from `fastgripper-openarm/autocal.py:176,226`; tracker swapped for the controller's (§3.2) | — |

Tool pairs folded into one each: `doctor.py`/`cal_doctor.py`,
`gui.py`/`gripper_gui.py`, `pad.py`/`gripper_pad.py`. Relative imports
throughout; flat-import bench copies retired.

Bench values that become configuration (§3.3): absolute cal path
(`gripper_tool.py:67`); motor IDs 0x07/0x17 (`set_gripper_id.py`,
`preflight.py:31`); gs_usb VID/PID table and `/dev/tty.usbmodem*` glob
(defaults, exposed); `span_from_closed` 33.5/30.0/31.1; app name in
`default_cal_path()`; `usb_stress.py:23-25` serial map → discovery.
