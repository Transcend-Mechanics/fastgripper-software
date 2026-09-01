# fastgripper-dm Plan 2: i2rt adapter + YAM migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The YAM bench teleops from the monorepo through `fastgripper_dm.adapters.i2rt`, the i2rt fork is public and pip-installable, the bench tools live in `bench/` with no device serials, and the `YAM Test` folder can be archived.

**Architecture:** `I2rtGripper` wraps an i2rt `MotorChainRobot` **read-side only** (position from `get_joint_pos()[idx]*SPAN − POS_WINDOW`, velocity/torque from `get_observations()`), runs the existing `GripperController.tick()` and returns real-unit `MitCommand`s; the loop **owner** (the teleop) converts with `to_i2rt_command` (`vel/SPAN`) and merges into its 7-joint `command_joint_state` call — i2rt's server thread stays the single writer. This is exactly what the proven `so101_teleop.py` does today, refactored onto the shared controller.

**Tech Stack:** Python 3.12 (i2rt/ruckig constraint), the published `fastgripper-dm` package (editable), i2rt fork branch `fastgripper`, pytest with a fake `MotorChainRobot` surface.

**Spec:** `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md` (rev 3), §3.2 adapter, §5 adapter tests, §6 step 4, §7 install path, §8 patches.

## Global Constraints

- The adapter NEVER calls `set_commands`, `command_joint_state`, or any write method on the robot — the owner writes; tests assert it.
- All real units at the adapter boundary: `Feedback` in rad, rad/s, Nm; the ×SPAN / ÷SPAN conversions live in exactly two places: `I2rtGripper._feedback()` (read) and `to_i2rt_command` (write). SPAN = 25.0, POS_WINDOW = 12.5, imported from `fastgripper_dm.port`.
- `error_code` on this path is reported as 1 (enabled) — fault handling is i2rt's `_try_recover_motors` (spec §3.2); document it in the adapter docstring.
- i2rt imports are lazy (module import must not require i2rt; a clear `ImportError` message names the install command).
- Install command documented everywhere as: `pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"` — no `[i2rt]` extra (PyPI forbids git deps).
- No device serial numbers or `/dev/cu.usbmodem<digits>` in the repo (`make gate` must stay green); bench-specific ports go in git-ignored `bench/local.toml`.
- The bench runs `gripper_limits_override = np.array([-POS_WINDOW, POS_WINDOW])` via `get_yam_robot` — the adapter asserts the mapping is exact by checking a round-trip at connect (see Task 1).
- Nothing in `YAM Test/` is deleted by this plan — Task 4 archives it only after the human confirms the drill passed.
- Commit trailer on every commit:
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01WamdJVNm9PA62Q8pJ4d25b`

## File Structure

```
packages/fastgripper-dm/src/fastgripper_dm/adapters/__init__.py
packages/fastgripper-dm/src/fastgripper_dm/adapters/i2rt.py      I2rtGripper, to_i2rt_command
packages/fastgripper-dm/tests/test_adapter_i2rt.py               FakeYamRobot + unit tests
bench/yam/so101_teleop.py        the YAM teleop, gripper block on the adapter
bench/yam/teleop_gui.py, TELEOP.md, running-the-yam-gotchas.md   copied, paths updated
bench/yam/gripper_cal.json, so101_trigger_cal.json               bench cal data (no secrets)
bench/so101/trigger_probe.py, leader_drop_probe.py               copied from SO101-software/scripts
bench/tools/usb_stress.py        de-serialized (reads bench/local.toml), bus_dump.py, wiggle_joints.py, rpm_stats.py
bench/local.toml.example         documented port map template
docs/fastgripper-dm/yam.md       install + run the YAM from this repo
```

---

### Task 0: Publish the i2rt fork (outward-facing — controller/human runs it)

The branch exists locally (`YAM Test/i2rt`, commits `f732e4f` + `4595129` on top of `https://github.com/i2rt-robotics/i2rt` main). Create the org fork and push.

- [ ] **Step 1:** `gh repo fork i2rt-robotics/i2rt --org Transcend-Mechanics --clone=false`
- [ ] **Step 2:** `cd "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt" && git remote add transcend https://github.com/Transcend-Mechanics/i2rt.git && git push transcend fastgripper`
- [ ] **Step 3:** Verify installability in a scratch venv (Python 3.12):
  `uv venv -q --python 3.12 /tmp/i2rt-check && VIRTUAL_ENV=/tmp/i2rt-check uv pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"` → `python -c "import i2rt; from i2rt.robots.get_robot import get_yam_robot; print('fork OK')"` (build deps like ruckig may fail on macOS — acceptable; record the result. On failure, fall back to verifying `pip download` fetches the branch.)
- [ ] **Step 4:** No repo commit needed (patches/README.md already documents the fork).

### Task 1a: Controller stall contract (required by the teleop's per-tick goto)

The live teleop calls `goto()` every cycle; `goto_rad` currently resets the
stall timer/latch unconditionally (controller.py:33-34), which makes the
stall latch unreachable and would deliver SUSTAINED tmax into a grasp
(review B2). Port the live `stall_clip` semantics into the controller:

**Files:** Modify `packages/fastgripper-dm/src/fastgripper_dm/controller.py`; append tests to `tests/test_controller.py`.

- [ ] **Step 1: Tests** (append):
```python
def test_per_tick_regoto_does_not_reset_stall(  ):
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for _ in range(int(4.0 / 0.02)):
        c.goto_frac(0.0)                      # trigger held: re-goto EVERY tick
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled                          # latch must still engage
    assert sim.max_abs_torque <= 2.0 + 1e-6


def test_retreating_goal_releases_stall():
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for _ in range(int(4.0 / 0.02)):
        c.goto_frac(0.0)
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled
    stall_pos = c.position
    c.goto_frac(0.0)                          # same squeeze: stays latched
    assert c.stalled
    c.goto_frac(1.0)                          # trigger released: unlatch
    assert not c.stalled
    for _ in range(int(4.0 / 0.02)):
        fb = sim.command(c.tick(fb, 0.02))
    assert c.position > stall_pos + 0.5       # moved away (open is +? no: open=-30) --
    # direction: ENTRY open=-30 < closed=2; retreat is toward open (more negative)
```
Fix the final assertion's direction before committing: with ENTRY open=-30.0, retreat means `c.position < stall_pos - 0.5`.

- [ ] **Step 2: Implement** in `goto_rad`:
```python
    RELEASE_EPS = 0.05   # rad; matches the live teleop's trigger-backoff release

    def goto_rad(self, pos: float) -> None:
        new_goal = min(self._hi, max(self._lo, pos))
        if self.stalled:
            # Latched on an obstruction: ignore goals that push further toward
            # the stall; release only when the goal retreats by RELEASE_EPS
            # (the live stall_clip semantics, so101_teleop.py:389-395).
            here = self.position if self.position is not None else new_goal
            toward_stall = abs(new_goal - here) < self.RELEASE_EPS or \
                (self.goal is not None and abs(new_goal - self.goal) < self.RELEASE_EPS)
            if toward_stall:
                return
        if self.goal is not None and abs(new_goal - self.goal) < 1e-9:
            return                            # idempotent re-goto: keep stall timer
        self.goal = new_goal
        self.stalled = False
        self._stall_t = 0.0
```
(The exact predicate is the implementer's to refine against the two tests: held-squeeze must stay latched; a released trigger — goal moving ≥ RELEASE_EPS away from the stall goal — must unlatch.)

- [ ] **Step 3:** `uv run pytest tests/test_controller.py tests/test_sim.py -q` all green (existing 6 controller tests must still pass — note `test_stall_on_obstruction_holds` calls `close()` once, unaffected).
- [ ] **Step 4:** Commit.

### Task 1: `adapters/i2rt.py` + fake-robot tests

**Files:**
- Create: `packages/fastgripper-dm/src/fastgripper_dm/adapters/__init__.py` (empty), `.../adapters/i2rt.py`, `packages/fastgripper-dm/tests/test_adapter_i2rt.py`

**Interfaces:**
- Consumes: `GripperController` (tick/goto_frac/open/close/hold/position/stalled/park_fields/adopt_park), `calstore` (default_cal_path/load_store/save_store/get_entry/entry_profile), `port` (Feedback, MitCommand, SPAN, POS_WINDOW).
- Produces:
  ```python
  def to_i2rt_command(cmd: MitCommand, pos_placeholder_norm: float) -> tuple[float, float, float, float]
      # returns (pos_norm, vel_norm, kp, kd): vel_norm = cmd.vel / SPAN; pos passthrough placeholder
  class I2rtGripper:
      def __init__(self, robot, joint_index: int, gripper: str | None = None,
                   cal_path: str | None = None): ...
      def connect(self, home: str = "auto") -> None   # park adopt or ValueError (no stall homing on a shared chain in v1 — the bench autocal covers it)
      def tick(self, dt: float) -> MitCommand          # reads robot, runs controller.tick
      def command_tuple(self, dt: float) -> tuple[float, float, float, float]  # tick + to_i2rt_command
      goto(frac) / open() / close() / hold(); position; stalled; goal
      def park(self) -> None                           # writes park_fields + parked_at, atomic save
  ```

- [ ] **Step 1: Write the failing tests**

`tests/test_adapter_i2rt.py`:
```python
import json

import numpy as np
import pytest

from fastgripper_dm.adapters.i2rt import I2rtGripper, to_i2rt_command
from fastgripper_dm.port import MitCommand, POS_WINDOW, SPAN


class FakeYamRobot:
    """The MotorChainRobot surface the adapter is allowed to touch, plus
    write methods that record abuse."""

    def __init__(self, n=7, grip_norm=0.5, grip_vel_norm=0.0, grip_eff=0.0):
        self.n = n
        self.grip_norm = grip_norm
        self.grip_vel_norm = grip_vel_norm
        self.grip_eff = grip_eff
        self.writes = []

    def get_joint_pos(self):
        out = np.zeros(self.n)
        out[-1] = self.grip_norm
        return out

    def get_observations(self):
        return {"joint_pos": np.zeros(self.n - 1), "joint_vel": np.zeros(self.n - 1),
                "joint_eff": np.zeros(self.n - 1),
                "gripper_vel": np.array([self.grip_vel_norm]),
                "gripper_eff": np.array([self.grip_eff])}

    def get_robot_info(self):
        return {"gripper_limits": (-POS_WINDOW, POS_WINDOW)}

    def command_joint_state(self, *a, **k):
        self.writes.append(("command_joint_state", a, k))

    def set_commands(self, *a, **k):
        self.writes.append(("set_commands", a, k))


def entry_file(tmp_path, entry):
    p = tmp_path / "gripper_cal.json"
    p.write_text(json.dumps({"format": 2, "grippers": {"yam": entry}}))
    return str(p)


ENTRY = {"open": -30.0, "closed": 2.0, "last_position": 1.0, "last_wrapped": 1.0}


def connected(tmp_path, grip_norm=None, entry=None):
    entry = dict(entry or ENTRY)
    # choose the robot's normalized position to match last_wrapped exactly:
    # norm = (wrapped + POS_WINDOW) / SPAN
    if grip_norm is None:
        grip_norm = (entry["last_wrapped"] + POS_WINDOW) / SPAN
    robot = FakeYamRobot(grip_norm=grip_norm)
    g = I2rtGripper(robot, joint_index=6, gripper="yam", cal_path=entry_file(tmp_path, entry))
    g.connect()
    return robot, g


def test_feedback_scaling_round_trip(tmp_path):
    robot, g = connected(tmp_path)
    robot.grip_vel_norm = 0.2                      # normalized
    robot.grip_eff = 0.7                           # already real Nm
    g.tick(0.02)
    fb = g.last_feedback
    assert g.position == pytest.approx(1.0, abs=1e-6)          # adopted park exactly
    assert fb.velocity == pytest.approx(0.2 * SPAN)            # ×SPAN read scaling (5.0 rad/s)
    assert fb.torque == pytest.approx(0.7)                     # eff is already real Nm
    assert fb.position == pytest.approx(1.0, abs=1e-6)         # ×SPAN − POS_WINDOW
    assert g.velocity == pytest.approx(5.0) and g.torque == pytest.approx(0.7)


def test_connect_rejects_wrong_gripper_limits(tmp_path):
    robot = FakeYamRobot(grip_norm=(ENTRY["last_wrapped"] + POS_WINDOW) / SPAN)
    robot.get_robot_info = lambda: {"gripper_limits": (0.0, 3.66)}   # arm default: wrong
    g = I2rtGripper(robot, joint_index=6, gripper="yam", cal_path=entry_file(tmp_path, dict(ENTRY)))
    with pytest.raises(ValueError, match="gripper_limits"):
        g.connect()


def test_to_i2rt_command_divides_vel_by_span(tmp_path):
    cmd = MitCommand(vel=5.0, kd=0.0833)
    pos_n, vel_n, kp, kd = to_i2rt_command(cmd, pos_placeholder_norm=0.4)
    assert vel_n == pytest.approx(5.0 / SPAN)
    assert pos_n == 0.4 and kp == 0.0 and kd == pytest.approx(0.0833)


def test_adapter_never_writes_to_the_robot(tmp_path):
    robot, g = connected(tmp_path)
    g.open()
    for _ in range(50):
        g.command_tuple(0.02)
    assert robot.writes == []


def test_connect_refuses_park_mismatch(tmp_path):
    # robot sits 3 rad (wrapped) away from last_wrapped -> no silent adoption
    with pytest.raises(ValueError, match="park"):
        connected(tmp_path, grip_norm=(4.0 + POS_WINDOW) / SPAN)


def test_park_persists(tmp_path):
    entry = dict(ENTRY)
    robot, g = connected(tmp_path)
    g.close()
    g.tick(0.02)
    g.park()
    saved = json.loads(open(g.cal_path).read())["grippers"]["yam"]
    assert saved["last_position"] == pytest.approx(g.position)
    assert "parked_at" in saved


def test_lazy_import_message():
    # importing the module must not require i2rt; only robot-side helpers may
    import fastgripper_dm.adapters.i2rt as m
    assert hasattr(m, "I2rtGripper")
```

- [ ] **Step 2: Run to verify failure** — `cd packages/fastgripper-dm && uv run pytest tests/test_adapter_i2rt.py -q` → ModuleNotFoundError.

- [ ] **Step 3: Implement**

`src/fastgripper_dm/adapters/i2rt.py`:
```python
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
        elif home == "off":
            pass
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

    def park(self) -> None:
        if self.ctrl.tracker.seen:
            self.entry.update(self.ctrl.park_fields())
            self.entry["parked_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            save_store(self.cal_path, self._store)
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_adapter_i2rt.py -q` → 7 passed; full suite green (82 expected incl. Task 1a's two). (numpy is needed by the tests only: add `numpy>=1.24` to the dev extra in pyproject, not runtime deps.)
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: i2rt adapter (I2rtGripper, to_i2rt_command) with fake-robot tests"`

### Task 2: bench/ migration (teleop on the adapter, tools de-serialized)

**Files:**
- Create: `bench/yam/so101_teleop.py` (from `YAM Test/so101_teleop.py`, gripper block replaced), `bench/yam/teleop_gui.py`, `bench/yam/TELEOP.md`, `bench/yam/running-the-yam-gotchas.md`, `bench/yam/gripper_cal.json`, `bench/yam/so101_trigger_cal.json`, `bench/so101/trigger_probe.py`, `bench/so101/leader_drop_probe.py`, `bench/tools/usb_stress.py`, `bench/tools/bus_dump.py`, `bench/tools/wiggle_joints.py`, `bench/tools/rpm_stats.py`, `bench/local.toml.example`, `bench/README.md`

**Interfaces:**
- Consumes: `I2rtGripper` (connect/tick/command_tuple/goto/park/stalled/position), Task 1.

- [ ] **Step 1:** Copy the files from `YAM Test/` and `SO101-software/scripts/` verbatim, then apply these changes:
  - `bench/yam/so101_teleop.py`: delete the in-file `MultiTurnTracker`, the gripper constants block (`POS_WINDOW…GRIPPER_KD`), and the cal-store handling for the gripper; instead:
    ```python
    from fastgripper_dm.adapters.i2rt import I2rtGripper
    grip = I2rtGripper(robot, joint_index=GRIP, gripper=args.gripper,
                       cal_path=args.cal_file)
    grip.connect(home="auto")
    ```
    In the 50 Hz loop, replace the whole gripper section — with the CONVENTION FLIP made explicit
    (review B1): today's trigger math produces frac 0 = open, 1 = closed; the controller's
    `goto_frac` is 0 = closed, 1 = open, so INVERT at the boundary:
    ```python
    frac_squeeze = (s_now[TRIGGER_SO_INDEX] - trig_cal["released"]) / (
        trig_cal["squeezed"] - trig_cal["released"])          # 0 released .. 1 squeezed (today's math)
    grip.goto(1.0 - max(0.0, min(1.0, frac_squeeze)))          # controller: 0=closed, 1=open
    pos_n, vel_n, kp_g, kd_g = grip.command_tuple(1.0 / LOOP_HZ)
    target[GRIP], vel[GRIP], kp[GRIP], kd[GRIP] = pos_n, vel_n, kp_g, kd_g
    ```
    Stall-hold relies on Task 1a's controller contract (per-tick idempotent re-goto keeps the
    latch; a released trigger retreats the goal ≥ RELEASE_EPS and unlatches) — the live
    `stall_clip` block is deleted, and the drill (Task 4) verifies grasp-and-hold on hardware.
    **Telemetry line must stay byte-compatible with the GUI regex** (`teleop_gui.py:26`):
    `print(f"TLM pos={grip.position:.3f} goal={grip.goal:.3f} vel={grip.velocity:.2f} eff={grip.torque:.3f}", flush=True)`
    (interactive branch keeps today's human-readable format using the same four values).
    **Exit structure is mandatory** (review M4): keep the `try: ... except KeyboardInterrupt: pass
    finally:` shape; `grip.park()` goes in the `finally` (guarded on a connected grip), followed by
    `robot.close()`, log close/flush, and the trailing `os._exit(0)` teardown — Ctrl-C must save
    the park exactly as today (so101_teleop.py:458-478).
    The teleop takes `--home {auto,off}` (default auto). `auto` refuses on park mismatch with the
    adapter's message; document in TELEOP.md that a lost park is recovered with
    `fastgripper-dm autocal home` on a dedicated channel, or `--home off` for a
    frame-unanchored emergency session (jaws-by-eye; no goto trust). (Review m3.)
  - `bench/tools/usb_stress.py`: replace the hard-coded `PORTS` map with a loader: read `bench/local.toml` (`[ports] leader_1 = "..."` etc.) via `tomllib`; error with a pointer to `bench/local.toml.example` when absent. No serials in the example (placeholders like `/dev/cu.usbmodemXXXXXXXXXXX`). Also rewrite its flat imports for the new home (review m1): `from canbus import open_bus` → `from fastgripper_dm.damiao.canbus import open_bus`; verify `can`/`scservo_sdk`/`usb` import in the bench venv (an import smoke run, not just py_compile).
  - `bench/local.toml.example` + `bench/README.md`: document the port map, the venv (`uv venv --python 3.12; pip install fastgripper-dm "i2rt @ git+…@fastgripper"` + `patches/setup-mac.sh`), and that `bench/local.toml` is git-ignored.
  - `bench/yam/TELEOP.md` / gotchas: update paths (`bench/yam/...`), the venv instructions, and note the adapter now owns the gripper math.
- [ ] **Step 2:** `make gate` → green (no serials). `python -m py_compile bench/yam/so101_teleop.py bench/tools/*.py bench/so101/*.py` → clean.
- [ ] **Step 3:** Commit.

### Task 3: `docs/fastgripper-dm/yam.md`

- [ ] **Step 1:** Write the page: what the YAM path is (adapter mode), install (`pip install fastgripper-dm` + the i2rt fork URL + `patches/setup-mac.sh` on macOS), the `gripper_limits_override` requirement with the exact `get_yam_robot(...)` call, run instructions for `bench/yam/so101_teleop.py`, park/homing notes (`autocal home` on a dedicated channel when the park is lost), link from `docs/fastgripper-dm/quickstart-linux.md` and the repo README.
- [ ] **Step 2:** Commit.

### Task 4: YAM drill + archive (hardware; human present — STOPS the run)

- [ ] `bench/` venv built per `bench/README.md`; `fastgripper-dm preflight` GO on the YAM bus.
- [ ] Teleop session from `bench/yam/so101_teleop.py`: arm tracks, trigger→jaw absolute mapping correct, stall-hold on grasp, Ctrl-C saves park; a second session auto-restores with no homing motion.
- [ ] Only after the human confirms: move `YAM Test/` to `~/Documents/Transcend/Repos/_archive/YAM Test/` (no deletion), leave a README pointer. Update memory files.

## Self-review

1. Spec coverage: §3.2 adapter (T1: read-side, tick, to_i2rt_command, single-writer, error_code=1), §5 adapter tests (T1 tests a/b/c: scaling round-trip, vel/SPAN, never-writes), §6 step 4 (T0 fork push, T2 teleop switch, T4 drill+archive), §7 install (T0/T3), §2 serial gate (T2). Homing on the shared chain deliberately refuses (spec's stall-home is standalone-only; `autocal home` covers re-anchoring) — documented in T1/T3.
2. Placeholders: none — every step carries code or exact commands.
3. Type consistency: `to_i2rt_command(cmd, pos_placeholder_norm) -> (pos_n, vel_n, kp, kd)` matches T2's unpacking order; `I2rtGripper(robot, joint_index, gripper, cal_path)` consistent across T1/T2; `command_tuple` used in T2.
