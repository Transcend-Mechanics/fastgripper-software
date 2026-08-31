# fastgripper-dm v0.1.0 (standalone) Implementation Plan

> **STATUS: Tasks 0–1 already executed via the v0.0.1 "harvest" commits (skeleton, CI, gate, patches captured); the repo state supersedes their steps. Execution starts at Task 2.**
>
> **Scope note (deviation from spec §3.2, deliberate):** v0.1 ships the `GripperController`/`MotorPort`/facade path for the customer API (`open/close/goto/home/status`, park/restore, torque cap) and keeps `calibrate`/`autocal` on their proven direct-DM4310 implementations. Migrating those two tools onto the controller happens with the i2rt adapter work (Plan 2), where the shared core pays off. One algorithm still ends up in one place; it just gets there in two steps.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship `fastgripper-dm` v0.1.0 — one tested Python package that drives the DM‑J4310 worm-gear gripper on a dedicated CAN channel (standalone mode), with preflight, doctor, id/watchdog provisioning, autocal, keyboard calibrate, and a `FastGripper` facade — from a new monorepo, on Linux/SocketCAN first and the macOS/gs_usb bench second.

**Architecture:** A pure `GripperController.tick(feedback, dt) -> MitCommand` core (multi-turn tracking, software P velocity loop, hard torque cap, stall hold, homing state machine, park/restore) drives any `MotorPort`. `DamiaoCanPort` is the synchronous python-can port; `SimulatedWormGripper` is the test port. Tools and the CLI orchestrate the controller over a port; per-gripper physical constants live in a `GripperProfile` stored inside the cal entry.

**Tech Stack:** Python ≥ 3.10, `python-can` ≥ 4.0 (socketcan / gs_usb / slcan), `pyusb` (gs_usb on macOS), `pytest`, `uv` for env/lock/build, GitHub Actions + PyPI trusted publishing.

**Spec:** `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md` (rev 3). Plans 2–4 (i2rt adapter + YAM migration; LeRobot move + release; drills) follow this one.

## Global Constraints

- Distribution `fastgripper-dm`, import `fastgripper_dm`, console script `fastgripper-dm`. Package dir `packages/fastgripper-dm/` with `src/` layout.
- `requires-python = ">=3.10"`; runtime deps exactly `python-can>=4.0` and `pyusb>=1.2`; dev deps `pytest`. No git-URL dependencies anywhere in metadata.
- Not a uv workspace: the repo root has no `pyproject.toml`; each package locks independently.
- Every physical constant that differs per gripper is a `GripperProfile` field — never a module-level constant: `closed_only, single_touch, span_from_closed, contact_torque, probe_tmax, probe_vel, seek_vel, margin, backoff, touch_tol, tmax_nm, vmax, sw_kp, park_tolerance_rad, watchdog_ms`.
- `TMAX_CAP = 2.0` Nm: any profile with `tmax_nm > 2.0` is refused by `preflight` and by `GripperController.__init__`.
- Motor watchdog (RID 9, milliseconds) is written by `setup` and verified by `preflight`; `0` is NO-GO.
- Cal store is format 2: `{"format": 2, "grippers": {name: entry}}`; entry fields `open, closed, last_position, last_wrapped, span, stop_open, stop_closed, stop_span, span_from_closed, method, calibrated_at, touched_at, homed_at, motor_id, master_id, close_dir, profile`. Atomic writes only.
- File locations: explicit `--cal/--config PATH` → `$FASTGRIPPER_DM_HOME/` → `~/.config/fastgripper-dm/`. **Never** the current working directory.
- Every CLI exits through `fastgripper_dm.tools._cli.run` (motor disable → bus drain → `os._exit`). Never inline `os._exit`.
- `canbus.open_bus` never calls `usb.core.Device.reset()`; a dead gs_usb bus raises `BusDead` (a `PortError`), never `SystemExit`.
- No device serial numbers or `/dev/cu.usbmodem…` paths in the repo (CI grep gate, Task 1).
- Commit after every task with the trailer lines: `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01WamdJVNm9PA62Q8pJ4d25b`.
- Out of scope for this plan (Plan 1b): `tools/pad.py`, `tools/gui.py` ports; Plan 2: i2rt adapter; Plan 3: LeRobot move + `release-lerobot.yml`; Plan 4: drills.

## File Structure

```
fastgripper-software/
  .gitignore, Makefile, .github/workflows/ci.yml
  patches/README.md, patches/gs_usb-darwin.patch, patches/ruckig-build.md, patches/setup-mac.sh
  packages/fastgripper-dm/
    pyproject.toml, README.md, uv.lock
    src/fastgripper_dm/
      __init__.py        FastGripper facade, __version__
      port.py            POS_WINDOW, SPAN, Feedback, MitCommand, PortError, BusDead, MotorPort
      tracker.py         MultiTurnTracker
      profile.py         GripperProfile, PRESETS, TMAX_CAP
      calstore.py        config_home, default_cal_path, default_config_path, load_store, save_store, get_entry, entry_profile, resolve_ids
      sim.py             SimulatedWormGripper
      controller.py      GripperController, ControllerState, Mode
      damiao/__init__.py
      damiao/protocol.py float_to_uint, uint_to_float, encode_mit, decode_feedback, special_frame, FAULT_CODES, RawFeedback
      damiao/canbus.py   open_bus, add_bus_args
      damiao/can_port.py DamiaoCanPort
      damiao/config_tool.py RegisterClient (read/write/save registers), set_motor_id, read_watchdog_ms, set_watchdog_ms, probe_ids
      tools/__init__.py
      tools/_cli.py      run(main)
      tools/preflight.py collect + evaluate + run_preflight
      tools/doctor.py    bus_ping (+ usb watch/soak reuse from preflight collectors)
      tools/autocal.py   Rig on controller primitives; full|home|touch
      tools/calibrate.py keyboard jog
      tools/drive.py     goto percentage and exit
      cli.py             argparse front door, config.json handling
    tests/
      test_port.py, test_tracker.py, test_profile_calstore.py, test_sim.py, test_controller.py,
      test_protocol.py, test_canbus.py, test_can_port.py, test_config_tool.py, test_preflight.py,
      test_doctor.py, test_autocal.py, test_calibrate.py, test_cli.py, test_facade.py
  docs/fastgripper-dm/quickstart-linux.md, quickstart-mac.md, troubleshooting.md, watchdog.md
```

Each source file has one responsibility; tools never touch `python-can` directly — only through `DamiaoCanPort`/`open_bus`.

---

### Task 0: Capture the hidden patches (i2rt fork branch, gs_usb + ruckig patches)

The YAM bench works only because of uncommitted edits. This task makes them durable before anything else is built. No package code.

**Files:**
- Modify (commit): `/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt` working tree → branch `fastgripper`
- Create: `patches/README.md`, `patches/gs_usb-darwin.patch`, `patches/ruckig-build.md`, `patches/setup-mac.sh`

**Interfaces:**
- Produces: git branch `fastgripper` on the i2rt clone containing every local change; `patches/setup-mac.sh` that applies the gs_usb patch into a given venv.

- [ ] **Step 1: Inspect the uncommitted i2rt changes**

Run:
```bash
cd "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt" && git status --short && git diff --stat && git log --oneline origin/main..HEAD
```
Expected: 5 modified files (`i2rt/motor_config_tool/{ping_motors,set_timeout,set_zero}.py`, `i2rt/motor_drivers/{can_interface,utils}.py`), 1 commit ahead (`f732e4f`).

- [ ] **Step 2: Commit them on a named branch**

```bash
cd "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt"
git checkout -b fastgripper
git add -A i2rt/
git commit -m "fastgripper bench patches: gs_usb backend selection, drain before bring-up, response-id matching, I2RT_CAN_RESPONSE_TIMEOUT, config-tool fixes

Captured from the working tree that has run the YAM bench since 2026-07-21."
git log --oneline origin/main..HEAD
```
Expected: 2 commits ahead of `origin/main`, clean `git status`.

- [ ] **Step 3: Verify the branch reproduces the working code**

```bash
cd "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt" && git diff fastgripper --stat | tail -1
```
Expected: no output (working tree identical to branch). Do **not** push yet — the `Transcend-Mechanics/i2rt` fork is created in Plan 2; record the branch name in `patches/README.md`.

- [ ] **Step 4: Extract the gs_usb darwin patch from the venv**

```bash
cd "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt"
V=.venv/lib/python3.12/site-packages/gs_usb
PIPV=$(.venv/bin/pip show gs_usb | awk '/^Version/{print $2}')
mkdir -p /tmp/gs_usb_pristine && cd /tmp/gs_usb_pristine && "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt/.venv/bin/pip" download --no-deps --no-binary :all: "gs_usb==$PIPV" -q && tar xzf gs_usb-*.tar.gz
diff -ru /tmp/gs_usb_pristine/gs_usb-*/gs_usb "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/i2rt/$V" > ~/Documents/Transcend/Repos/fastgripper-software/patches/gs_usb-darwin.patch; true
head -20 ~/Documents/Transcend/Repos/fastgripper-software/patches/gs_usb-darwin.patch
```
Expected: a unified diff whose only hunk guards `is_kernel_driver_active` on darwin (around `gs_usb.py:56`). If `pip download` fails (no sdist), instead copy the patched file's hunk by hand: `git diff --no-index` between the wheel-installed file (reinstall into a scratch venv) and the patched one.

- [ ] **Step 5: Write `patches/setup-mac.sh` and `patches/README.md`**

`patches/setup-mac.sh`:
```bash
#!/usr/bin/env bash
# Apply the macOS-only fixes into a venv so gs_usb works on Darwin.
# Usage: patches/setup-mac.sh /path/to/.venv
set -euo pipefail
VENV="${1:?venv path}"
PY="$VENV/bin/python"
SITE="$($PY -c 'import gs_usb, os; print(os.path.dirname(gs_usb.__file__))')"
patch -p1 -N -d "$SITE" < "$(dirname "$0")/gs_usb-darwin.patch" || echo "gs_usb patch already applied"
echo "gs_usb patched in $SITE"
echo "ruckig: see patches/ruckig-build.md (only needed for the i2rt/YAM bench, not for fastgripper-dm)"
```

`patches/ruckig-build.md`:
```markdown
# ruckig on macOS (i2rt/YAM bench only)

The PyPI sdist for ruckig 0.15.3 fails to build on macOS because its
`pyproject.toml` uses `cmake.targets`; the fix is to rename that key to
`build.targets` and build a wheel:

    pip download --no-binary :all: ruckig==0.15.3
    tar xzf ruckig-0.15.3.tar.gz && cd ruckig-0.15.3
    sed -i '' 's/cmake.targets/build.targets/' pyproject.toml
    pip wheel . -w ../wheels && pip install ../wheels/ruckig-*.whl

`fastgripper-dm` itself does not depend on ruckig.
```

`patches/README.md`:
```markdown
# patches/

Reproducible copies of every out-of-tree fix the bench depends on.

| what | where | applied by |
| --- | --- | --- |
| i2rt bench patches | branch `fastgripper` in the i2rt clone (`YAM Test/i2rt`); pushed to `Transcend-Mechanics/i2rt` in Plan 2 | `pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"` |
| gs_usb darwin guard | `gs_usb-darwin.patch` | `patches/setup-mac.sh <venv>` |
| ruckig macOS build | `ruckig-build.md` | manual, i2rt bench only |

Linux needs none of these.
```

- [ ] **Step 6: Commit**

```bash
cd ~/Documents/Transcend/Repos/fastgripper-software && chmod +x patches/setup-mac.sh && git add patches && git commit -m "patches: capture gs_usb darwin fix, ruckig build note, i2rt branch pointer"
```

---

### Task 1: Package skeleton, CI, serial-number gate

**Files:**
- Create: `.gitignore`, `Makefile`, `.github/workflows/ci.yml`, `packages/fastgripper-dm/pyproject.toml`, `packages/fastgripper-dm/README.md`, `packages/fastgripper-dm/src/fastgripper_dm/__init__.py`, `packages/fastgripper-dm/src/fastgripper_dm/damiao/__init__.py`, `packages/fastgripper-dm/src/fastgripper_dm/tools/__init__.py`, `packages/fastgripper-dm/tests/test_package.py`

**Interfaces:**
- Produces: `fastgripper_dm.__version__ == "0.1.0.dev0"`; `make dm-test` runs the package tests; CI runs them on ubuntu + macos, py3.10 + 3.12, and fails on serial numbers.

- [ ] **Step 1: Write the failing test**

`packages/fastgripper-dm/tests/test_package.py`:
```python
import fastgripper_dm


def test_version_string():
    assert fastgripper_dm.__version__ == "0.1.0.dev0"
```

- [ ] **Step 2: Create the package files**

`packages/fastgripper-dm/pyproject.toml`:
```toml
[project]
name = "fastgripper-dm"
version = "0.1.0.dev0"
description = "FastGripper worm-gear gripper driver for Damiao DM-J4310 actuators on CAN"
readme = "README.md"
requires-python = ">=3.10"
license = { text = "Apache-2.0" }
dependencies = ["python-can>=4.0", "pyusb>=1.2"]

[project.optional-dependencies]
dev = ["pytest>=8"]

[project.scripts]
fastgripper-dm = "fastgripper_dm.cli:main"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.hatch.build.targets.wheel]
packages = ["src/fastgripper_dm"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

`packages/fastgripper-dm/src/fastgripper_dm/__init__.py`:
```python
"""fastgripper-dm: FastGripper worm-gear gripper on a Damiao DM-J4310 over CAN."""

__version__ = "0.1.0.dev0"
```

`packages/fastgripper-dm/src/fastgripper_dm/damiao/__init__.py` and `.../tools/__init__.py`: empty files.

`packages/fastgripper-dm/README.md`:
```markdown
# fastgripper-dm

Driver, calibration, and CLI for the FastGripper worm-gear gripper when its
actuator is a Damiao DM-J4310 on a dedicated CAN channel.

    pip install fastgripper-dm
    fastgripper-dm setup --interface socketcan --channel can0
    fastgripper-dm preflight
    fastgripper-dm calibrate
    fastgripper-dm open / close / goto 40

Docs: ../../docs/fastgripper-dm/
```

`.gitignore` (repo root):
```
.venv/
venv/
__pycache__/
*.pyc
dist/
build/
*.egg-info/
logs/
datasets/
checkpoints/
bench/local.toml
.DS_Store
```

`Makefile` (repo root):
```make
.PHONY: dm-sync dm-test dm-lint gate
dm-sync:
	cd packages/fastgripper-dm && uv sync --extra dev
dm-test: dm-sync
	cd packages/fastgripper-dm && uv run pytest -q
gate:
	@! grep -rnE 'usbmodem|/dev/cu\.' packages bench 2>/dev/null || (echo "serial numbers / bench device paths found" && exit 1)
```

`.github/workflows/ci.yml`:
```yaml
name: ci
on: [push, pull_request]
jobs:
  gate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: make gate
  fastgripper-dm:
    strategy:
      matrix:
        os: [ubuntu-latest, macos-latest]
        python: ["3.10", "3.12"]
    runs-on: ${{ matrix.os }}
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/setup-uv@v3
      - run: cd packages/fastgripper-dm && uv python install ${{ matrix.python }} && uv sync --python ${{ matrix.python }} --extra dev && uv run pytest -q
```

- [ ] **Step 3: Run the test**

Run: `cd ~/Documents/Transcend/Repos/fastgripper-software && make dm-test`
Expected: `1 passed`, and `packages/fastgripper-dm/uv.lock` created.

- [ ] **Step 4: Run the gate**

Run: `make gate`
Expected: no output, exit 0.

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "fastgripper-dm: package skeleton, CI matrix, serial-number gate"
```

---

### Task 2: `port.py` — the contracts

**Files:**
- Create: `packages/fastgripper-dm/src/fastgripper_dm/port.py`, `packages/fastgripper-dm/tests/test_port.py`

**Interfaces:**
- Produces:
  ```python
  POS_WINDOW: float = 12.5; SPAN: float = 25.0
  @dataclass(frozen=True) class Feedback: position: float; velocity: float; torque: float; error_code: int; t: float
      faulted -> bool (error_code >= 8); enabled -> bool (error_code == 1)
  @dataclass(frozen=True) class MitCommand: pos=0.0; vel=0.0; kp=0.0; kd=0.0; tau=0.0
  class PortError(RuntimeError); class BusDead(PortError)
  class MotorPort(Protocol): pos_window: float; enable(); disable(); command(cmd) -> Feedback; read() -> Feedback; clear_error(); close()
  FAULT_CODES: dict[int, str]
  ```

- [ ] **Step 1: Write the failing test**

`tests/test_port.py`:
```python
from fastgripper_dm.port import FAULT_CODES, SPAN, POS_WINDOW, Feedback, MitCommand, BusDead, PortError


def test_constants():
    assert POS_WINDOW == 12.5 and SPAN == 25.0


def test_feedback_flags():
    ok = Feedback(position=0.0, velocity=0.0, torque=0.0, error_code=1, t=0.0)
    bad = Feedback(position=0.0, velocity=0.0, torque=0.0, error_code=0xD, t=0.0)
    assert ok.enabled and not ok.faulted
    assert bad.faulted and bad.fault_text == "communication loss"


def test_mitcommand_defaults_are_zero_gain():
    c = MitCommand(vel=1.0, kd=0.05)
    assert (c.pos, c.kp, c.tau) == (0.0, 0.0, 0.0)


def test_busdead_is_a_porterror():
    assert issubclass(BusDead, PortError) and FAULT_CODES[0x1] == "enabled"
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd packages/fastgripper-dm && uv run pytest tests/test_port.py -q`
Expected: `ModuleNotFoundError: fastgripper_dm.port`

- [ ] **Step 3: Implement**

`src/fastgripper_dm/port.py`:
```python
"""Contracts between the gripper controller and whatever owns the motor."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

POS_WINDOW = 12.5          # DM-J4310 MIT feedback window: +/-12.5 rad
SPAN = 2 * POS_WINDOW      # 25 rad per wrap

FAULT_CODES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "MOS overtemperature",
    0xC: "motor coil overtemperature",
    0xD: "communication loss",
    0xE: "overload",
}


@dataclass(frozen=True)
class Feedback:
    position: float     # rad, output shaft, WRAPPED into +/-pos_window
    velocity: float     # rad/s
    torque: float       # Nm at the motor
    error_code: int     # DM status nibble
    t: float            # time.monotonic() of the reading

    @property
    def faulted(self) -> bool:
        return self.error_code >= 0x8

    @property
    def enabled(self) -> bool:
        return self.error_code == 0x1

    @property
    def fault_text(self) -> str:
        return FAULT_CODES.get(self.error_code, f"unknown ({self.error_code:#x})")


@dataclass(frozen=True)
class MitCommand:
    pos: float = 0.0
    vel: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0


class PortError(RuntimeError):
    """The port could not complete a transaction within its retry budget."""


class BusDead(PortError):
    """The bus opened but passes no frames (nothing ACKs)."""


class MotorPort(Protocol):
    pos_window: float

    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def command(self, cmd: MitCommand) -> Feedback: ...
    def read(self) -> Feedback: ...
    def clear_error(self) -> None: ...
    def close(self) -> None: ...
```

- [ ] **Step 4: Run to verify it passes** — `uv run pytest tests/test_port.py -q` → `4 passed`.

- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: port contracts (Feedback, MitCommand, MotorPort, PortError)"`

---

### Task 3: `tracker.py` — multi-turn unwrap with exact park adoption

**Files:**
- Create: `src/fastgripper_dm/tracker.py`, `tests/test_tracker.py`

**Interfaces:**
- Produces: `class MultiTurnTracker(span: float = SPAN, start_unwrapped: float | None = None)`; `update(wrapped) -> float`; `position: float`; `wrapped: float`; `anchor(known_position: float) -> None`; `seen: bool`.

- [ ] **Step 1: Write the failing tests**

`tests/test_tracker.py`:
```python
import pytest
from fastgripper_dm.port import SPAN
from fastgripper_dm.tracker import MultiTurnTracker


def test_first_update_defines_position():
    t = MultiTurnTracker()
    assert t.update(3.0) == 3.0 and t.position == 3.0 and t.wrapped == 3.0


def test_wrap_forward_and_back():
    t = MultiTurnTracker()
    t.update(12.0)
    assert t.update(-12.0) == pytest.approx(13.0)      # jumped down by 24 -> one wrap up
    assert t.update(-11.0) == pytest.approx(14.0)
    assert t.update(12.0) == pytest.approx(12.0)       # jumped up by 23 -> one wrap down


def test_small_moves_never_wrap():
    t = MultiTurnTracker()
    t.update(0.0)
    for x in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0):
        t.update(x)
    assert t.position == pytest.approx(12.0)


def test_exact_park_adoption():
    t = MultiTurnTracker(start_unwrapped=-22.16)
    assert t.update(2.84) == pytest.approx(-22.16)     # offset = -25.0 exactly, no rounding
    t2 = MultiTurnTracker(start_unwrapped=-20.0)
    assert t2.update(2.84) == pytest.approx(-20.0)     # NOT rounded to a window multiple


def test_anchor_shifts_frame():
    t = MultiTurnTracker()
    t.update(1.0)
    t.anchor(31.0)
    assert t.position == pytest.approx(31.0)
    t.update(1.5)
    assert t.position == pytest.approx(31.5)


def test_position_before_update_raises():
    t = MultiTurnTracker()
    assert not t.seen
    with pytest.raises(RuntimeError):
        _ = t.position
```

- [ ] **Step 2: Run to verify failure** — `uv run pytest tests/test_tracker.py -q` → ModuleNotFoundError.

- [ ] **Step 3: Implement**

`src/fastgripper_dm/tracker.py`:
```python
"""Unwrap the DM-J4310's +/-12.5 rad feedback window into a continuous shaft angle.

The worm gear cannot be back-driven, so a saved park position is adopted
EXACTLY (offset = park - first_wrapped): the motor's own frame resets modulo
2*pi on every power cycle, so modular reconciliation is never safe. Ported
from so101_teleop.py (v7); the window-rounding variant in dm4310.py is
retired.
"""

from __future__ import annotations

from .port import SPAN


class MultiTurnTracker:
    def __init__(self, span: float = SPAN, start_unwrapped: float | None = None):
        self.span = span
        self._offset = 0.0
        self._last: float | None = None
        self._pending = start_unwrapped

    @property
    def seen(self) -> bool:
        return self._last is not None

    def update(self, wrapped: float) -> float:
        if self._last is None:
            self._last = wrapped
            if self._pending is not None:
                self._offset = self._pending - wrapped
                self._pending = None
        else:
            delta = wrapped - self._last
            if delta > self.span / 2:
                self._offset -= self.span
            elif delta < -self.span / 2:
                self._offset += self.span
            self._last = wrapped
        return self.position

    def anchor(self, known_position: float) -> None:
        """Declare the current shaft angle to be `known_position` (homing)."""
        if self._last is None:
            self._pending = known_position
            return
        self._offset = known_position - self._last

    @property
    def position(self) -> float:
        if self._last is None:
            raise RuntimeError("no feedback seen yet")
        return self._last + self._offset

    @property
    def wrapped(self) -> float:
        if self._last is None:
            raise RuntimeError("no feedback seen yet")
        return self._last
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_tracker.py -q` → `6 passed`.

- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: multi-turn tracker with exact park adoption"`

---

### Task 4: `profile.py` — per-gripper physical constants

**Files:**
- Create: `packages/fastgripper-dm/src/fastgripper_dm/profile.py`, `packages/fastgripper-dm/tests/test_profile.py`
- Modify: `packages/fastgripper-dm/src/fastgripper_dm/calstore.py` (append `entry_profile`)

**Interfaces:**
- Produces:
  ```python
  TMAX_CAP: float = 2.0
  @dataclass class GripperProfile: closed_only=False; single_touch=False; span_from_closed=30.0;
      contact_torque=0.30; probe_tmax=0.5; probe_vel=0.8; seek_vel=2.5; margin=0.75; backoff=2.0;
      touch_tol=0.2; tmax_nm=2.0; vmax=24.0; sw_kp=24.0; stall_torque_frac=0.75; stall_time_s=0.4;
      park_tolerance_rad=0.35; watchdog=8000; close_dir=1
      to_dict() -> dict; from_dict(d) -> GripperProfile (unknown keys ignored); validate() raises ValueError
  PRESETS: dict[str, GripperProfile]  # "openarm", "yam" (span 33.5), "ur" (closed_only, single_touch)
  calstore.entry_profile(entry: dict) -> GripperProfile   # entry.get("profile", {}) merged over defaults
  ```

- [ ] **Step 1: Write the failing tests**

`tests/test_profile.py`:
```python
import pytest
from fastgripper_dm.calstore import entry_profile
from fastgripper_dm.profile import PRESETS, TMAX_CAP, GripperProfile


def test_defaults_describe_a_two_hardstop_gripper():
    p = GripperProfile()
    assert not p.closed_only and not p.single_touch and p.tmax_nm == 2.0 and p.watchdog == 8000


def test_presets():
    assert PRESETS["yam"].span_from_closed == 33.5
    assert PRESETS["ur"].closed_only and PRESETS["ur"].single_touch and PRESETS["ur"].span_from_closed == 30.0


def test_roundtrip_ignores_unknown_keys():
    p = GripperProfile.from_dict({"tmax_nm": 1.5, "someday_field": 1})
    assert p.tmax_nm == 1.5
    assert GripperProfile.from_dict(p.to_dict()) == p


def test_validate_refuses_tmax_above_cap():
    with pytest.raises(ValueError, match="tmax_nm"):
        GripperProfile(tmax_nm=TMAX_CAP + 0.1).validate()
    GripperProfile(tmax_nm=TMAX_CAP).validate()


def test_entry_profile_merges_over_defaults():
    p = entry_profile({"profile": {"span_from_closed": 33.5}})
    assert p.span_from_closed == 33.5 and p.tmax_nm == 2.0
    assert entry_profile({}) == GripperProfile()
```

- [ ] **Step 2: Run to verify failure** — `uv run pytest tests/test_profile.py -q` → ModuleNotFoundError.

- [ ] **Step 3: Implement**

`src/fastgripper_dm/profile.py`:
```python
"""Per-gripper physical constants. Every value that differed between the old
copies (closed_only, single_touch, span, torque thresholds) is a profile
field, never a module constant. Stored inside the cal entry under "profile"."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields

TMAX_CAP = 2.0   # Nm: documented worm ceiling; >2 Nm transients snapped the v1 worm


@dataclass
class GripperProfile:
    closed_only: bool = False
    single_touch: bool = False
    span_from_closed: float = 30.0     # rad; conservative UNDERestimate of travel
    contact_torque: float = 0.30       # Nm; must exceed the unit's free-run p95
    probe_tmax: float = 0.5            # Nm cap during probing
    probe_vel: float = 0.8             # rad/s slow datum touch
    seek_vel: float = 2.5              # rad/s fast seek
    margin: float = 0.75               # rad inside the physical stops
    backoff: float = 2.0               # rad between touches
    touch_tol: float = 0.2             # rad double-touch agreement
    tmax_nm: float = 2.0               # hard grip-torque cap
    vmax: float = 24.0                 # rad/s software velocity limit
    sw_kp: float = 24.0                # software position-loop gain (1/s)
    stall_torque_frac: float = 0.75    # stall when |torque| > frac * tmax_nm ...
    stall_time_s: float = 0.4          # ... for this long
    park_tolerance_rad: float = 0.35   # PROVISIONAL: boot wrapped vs last_wrapped (spec §10)
    watchdog: int = 8000               # RID 9 raw value; unit under investigation (2026-08-30)
    close_dir: int = 1                 # velocity sign that closes the jaws

    def validate(self) -> None:
        if self.tmax_nm > TMAX_CAP:
            raise ValueError(f"profile tmax_nm {self.tmax_nm} exceeds the {TMAX_CAP} Nm cap")
        if self.close_dir not in (1, -1):
            raise ValueError("close_dir must be +1 or -1")
        if self.span_from_closed <= 0 or self.vmax <= 0 or self.sw_kp <= 0:
            raise ValueError("span_from_closed, vmax, sw_kp must be positive")

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "GripperProfile":
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


PRESETS = {
    "openarm": GripperProfile(),
    "yam": GripperProfile(span_from_closed=33.5),
    "ur": GripperProfile(closed_only=True, single_touch=True, span_from_closed=30.0),
}
```

Append to `src/fastgripper_dm/calstore.py`:
```python
def entry_profile(entry: dict):
    """The entry's profile block merged over package defaults."""
    from .profile import GripperProfile

    return GripperProfile.from_dict(entry.get("profile", {}) or {})
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_profile.py -q` → `5 passed`.
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: GripperProfile with presets; entry_profile"`

---

### Task 5: `sim.py` — simulated worm gripper (the test port)

**Files:**
- Create: `src/fastgripper_dm/sim.py`, `tests/test_sim.py`

**Interfaces:**
- Produces:
  ```python
  class SimulatedWormGripper(MotorPort):
      def __init__(self, stop_open=-31.0, stop_closed=+3.0, start=+1.0, friction=0.25,
                   tau_response=0.05, dt=0.02): ...
      pos_window = 12.5
      # MotorPort methods; plus test hooks:
      true_position: float          # unwrapped ground truth
      max_abs_torque: float         # peak |torque| ever produced
      drop_next(n: int)             # next n command()s raise PortError
      inject_fault(code: int)       # feedback reports this error_code until clear_error()
  ```
  Torque model is EXACTLY the motor's: `tau = kd * (vel_cmd - velocity) + tau_ff`; a first-order
  lag (`tau_response`) drives velocity toward `vel_cmd`; friction opposes motion; hard stops are
  springs (2 Nm/rad past the stop) the shaft cannot pass. `command()` returns feedback with
  `position` wrapped into ±12.5.

- [ ] **Step 1: Write the failing tests**

`tests/test_sim.py`:
```python
import pytest
from fastgripper_dm.port import MitCommand, PortError, SPAN
from fastgripper_dm.sim import SimulatedWormGripper


def run(sim, cmd, n):
    fb = sim.read()
    for _ in range(n):
        fb = sim.command(cmd)
    return fb


def test_moves_and_wraps():
    sim = SimulatedWormGripper(start=12.0)
    sim.enable()
    fb0 = sim.read()
    fb = run(sim, MitCommand(vel=5.0, kd=1.0), 200)      # 200 * 20 ms = 4 s at ~5 rad/s
    assert sim.true_position > 12.5                       # crossed the window edge
    assert -12.5 <= fb.position <= 12.5                   # feedback stays wrapped
    assert fb.position == pytest.approx(((sim.true_position + 12.5) % SPAN) - 12.5, abs=1e-6)
    assert fb0.position == pytest.approx(12.0)


def test_hard_stop_stalls_the_shaft():
    sim = SimulatedWormGripper(stop_closed=3.0, start=2.0)
    sim.enable()
    run(sim, MitCommand(vel=4.0, kd=1.0), 500)
    assert sim.true_position < 3.4                        # cannot pass the stop (+ small compliance)
    fb = sim.command(MitCommand(vel=4.0, kd=1.0))
    assert abs(fb.velocity) < 0.05                        # stalled
    assert fb.torque > 0.5                                 # pushing


def test_torque_is_kd_times_velocity_error():
    sim = SimulatedWormGripper(start=0.0, friction=0.0)
    sim.enable()
    fb = sim.command(MitCommand(vel=2.0, kd=0.5))
    assert fb.torque == pytest.approx(0.5 * (2.0 - fb.velocity), abs=1e-6)


def test_drop_and_fault_injection():
    sim = SimulatedWormGripper()
    sim.enable()
    sim.drop_next(2)
    with pytest.raises(PortError):
        sim.command(MitCommand())
    with pytest.raises(PortError):
        sim.command(MitCommand())
    sim.command(MitCommand())                              # third works
    sim.inject_fault(0xD)
    assert sim.read().faulted
    sim.clear_error()
    assert not sim.read().faulted


def test_disabled_motor_does_not_move():
    sim = SimulatedWormGripper(start=0.0)
    run(sim, MitCommand(vel=5.0, kd=1.0), 50)              # never enabled
    assert sim.true_position == pytest.approx(0.0)
```

- [ ] **Step 2: Run to verify failure.**

- [ ] **Step 3: Implement**

`src/fastgripper_dm/sim.py`:
```python
"""Simulated worm gripper implementing MotorPort, for hardware-free tests.

The torque model is exactly the motor's (tau = kd*(v_cmd - v) + tau_ff) so a
"cap never exceeded" assertion is meaningful, not vacuous."""

from __future__ import annotations

import time

from .port import POS_WINDOW, SPAN, Feedback, MitCommand, PortError


class SimulatedWormGripper:
    pos_window = POS_WINDOW

    def __init__(self, stop_open: float = -31.0, stop_closed: float = +3.0, start: float = +1.0,
                 friction: float = 0.25, tau_response: float = 0.05, dt: float = 0.02,
                 stop_stiffness: float = 2.0, inertia: float = 0.01):
        assert stop_open < start < stop_closed
        self.stop_open, self.stop_closed = stop_open, stop_closed
        self.true_position = start
        self.velocity = 0.0
        self.friction, self.tau_response, self.dt = friction, tau_response, dt
        self.stop_stiffness, self.inertia = stop_stiffness, inertia
        self._enabled = False
        self._fault = 0
        self._drops = 0
        self._torque = 0.0
        self.max_abs_torque = 0.0
        self._t = 0.0

    # --- test hooks ---
    def drop_next(self, n: int) -> None:
        self._drops = n

    def inject_fault(self, code: int) -> None:
        self._fault = code
        self._enabled = False

    # --- MotorPort ---
    def enable(self) -> None:
        if not self._fault:
            self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def clear_error(self) -> None:
        self._fault = 0

    def close(self) -> None:
        self.disable()

    def read(self) -> Feedback:
        return self._feedback()

    def command(self, cmd: MitCommand) -> Feedback:
        if self._drops > 0:
            self._drops -= 1
            raise PortError("simulated dropped frame")
        if self._enabled and not self._fault:
            tau = cmd.kd * (cmd.vel - self.velocity) + cmd.tau \
                + cmd.kp * (cmd.pos - self._wrapped())
            # friction opposes motion; stops push back
            tau_net = tau - self.friction * (1 if self.velocity > 0 else -1 if self.velocity < 0 else 0)
            if self.true_position > self.stop_closed:
                tau_net -= self.stop_stiffness * (self.true_position - self.stop_closed)
            elif self.true_position < self.stop_open:
                tau_net -= self.stop_stiffness * (self.true_position - self.stop_open)
            self.velocity += (tau_net / self.inertia) * self.dt
            # first-order lag toward the commanded velocity dominates (motor's own loop)
            alpha = min(1.0, self.dt / max(self.tau_response, 1e-6))
            self.velocity += alpha * (cmd.vel - self.velocity) if cmd.kd > 0 else 0.0
            new_pos = self.true_position + self.velocity * self.dt
            # the shaft may compress the stop slightly but not pass it
            new_pos = min(new_pos, self.stop_closed + 0.35)
            new_pos = max(new_pos, self.stop_open - 0.35)
            if new_pos in (self.stop_closed + 0.35, self.stop_open - 0.35):
                self.velocity = 0.0
            self.true_position = new_pos
            self._torque = tau
            self.max_abs_torque = max(self.max_abs_torque, abs(tau))
        else:
            self.velocity = 0.0
            self._torque = 0.0
        self._t += self.dt
        return self._feedback()

    def _wrapped(self) -> float:
        return ((self.true_position + POS_WINDOW) % SPAN) - POS_WINDOW

    def _feedback(self) -> Feedback:
        code = self._fault if self._fault else (1 if self._enabled else 0)
        return Feedback(position=self._wrapped(), velocity=self.velocity,
                        torque=self._torque, error_code=code, t=self._t or time.monotonic())
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_sim.py -q` → `5 passed`. If the stop test is flaky, tune `stop_stiffness`/`inertia` in the sim (not the test's assertions).
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: simulated worm gripper port for hardware-free tests"`

---

### Task 6: `controller.py` — the pure tick core

**Files:**
- Create: `src/fastgripper_dm/controller.py`, `tests/test_controller.py`

**Interfaces:**
- Consumes: `Feedback, MitCommand, PortError` (Task 2), `MultiTurnTracker` (Task 3), `GripperProfile` (Task 4).
- Produces:
  ```python
  class GripperController:
      def __init__(self, entry: dict, profile: GripperProfile): ...   # entry needs open/closed marks
      def tick(self, fb: Feedback, dt: float) -> MitCommand            # pure; no I/O
      def goto_frac(self, frac: float) -> None    # 0 = closed mark, 1 = open mark
      def goto_rad(self, pos: float) -> None
      def open(self) -> None; def close(self) -> None; def hold(self) -> None
      position: float | None      # unwrapped rad (None before first tick)
      goal: float | None
      stalled: bool               # sustained measured torque near the cap with goal unmet
      at_goal: bool               # |goal - position| < 0.05 rad
      def park_fields(self) -> dict   # {"last_position": ..., "last_wrapped": ...}
      def adopt_park(self, last_position: float) -> None   # trust the saved park exactly
      def anchor(self, known_position: float) -> None      # homing re-anchor
  ```
  `tick` implements, from `so101_teleop.py` v7: software P loop `v = clip(sw_kp*(goal-pos), ±vmax)`,
  torque clamp `v = clip(v, v_meas - tmax/kd, v_meas + tmax/kd)` with `kd = tmax_nm/vmax`, stall
  detection on measured torque (`|tau| > stall_torque_frac*tmax` for `stall_time_s` ⇒ hold here),
  goal clamped into [min(open,closed), max(open,closed)]. Returns `MitCommand(vel=v, kd=kd)`
  (kp=0, pos=0, tau=0). Raises `ValueError` at construction if the profile fails `validate()` or
  the entry lacks marks.

- [ ] **Step 1: Write the failing tests**

`tests/test_controller.py`:
```python
import pytest
from fastgripper_dm.controller import GripperController
from fastgripper_dm.port import MitCommand
from fastgripper_dm.profile import GripperProfile
from fastgripper_dm.sim import SimulatedWormGripper

ENTRY = {"open": -30.0, "closed": 2.0}


def make(sim=None, **prof):
    sim = sim or SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=1.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile(**prof))
    c.adopt_park(1.0)          # sim starts at 1.0; trust it exactly
    return sim, c


def run(sim, c, seconds):
    fb = sim.read()
    for _ in range(int(seconds / 0.02)):
        fb = sim.command(c.tick(fb, 0.02))
    return fb


def test_reaches_a_goal_without_exceeding_the_torque_cap():
    sim, c = make()
    c.goto_frac(1.0)                        # fully open = -30.0
    run(sim, c, 6.0)
    assert c.position == pytest.approx(-30.0, abs=0.1)
    assert c.at_goal
    assert sim.max_abs_torque <= 2.0 + 1e-6   # THE cap assertion


def test_frac_mapping():
    sim, c = make()
    c.goto_frac(0.0)
    assert c.goal == pytest.approx(2.0)      # 0 = closed mark
    c.goto_frac(0.5)
    assert c.goal == pytest.approx(-14.0)


def test_goal_is_clamped_to_marks():
    _, c = make()
    c.goto_rad(+10.0)
    assert c.goal == pytest.approx(2.0)
    c.goto_rad(-99.0)
    assert c.goal == pytest.approx(-30.0)


def test_stall_on_obstruction_holds():
    # closed stop at 3.0 but the mark is 2.0; put a virtual object by moving the stop inward
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    c.close()                                # goal 2.0, but the "object" is at -5
    run(sim, c, 4.0)
    assert c.stalled
    assert c.position == pytest.approx(-5.0, abs=0.6)
    assert sim.max_abs_torque <= 2.0 + 1e-6


def test_tick_is_pure_and_survives_fault_feedback():
    sim, c = make()
    c.open()
    fb = sim.read()
    a = c.tick(fb, 0.02)
    # rebuild an identical controller: same inputs -> same command
    c2 = GripperController(dict(ENTRY), GripperProfile())
    c2.adopt_park(1.0)
    c2.open()
    assert c2.tick(fb, 0.02) == a
    sim.inject_fault(0xD)
    cmd = c.tick(sim.read(), 0.02)
    assert cmd == MitCommand()               # zero everything while faulted


def test_profile_validation_at_construction():
    with pytest.raises(ValueError):
        GripperController(dict(ENTRY), GripperProfile(tmax_nm=9.0))
    with pytest.raises(ValueError):
        GripperController({}, GripperProfile())
```

- [ ] **Step 2: Run to verify failure.**

- [ ] **Step 3: Implement**

`src/fastgripper_dm/controller.py`:
```python
"""The gripper control algorithm, once, with no I/O.

Ported from so101_teleop.py v7 (the hardware-validated version with the
torque cap): software P velocity loop + per-tick torque clamp + stall hold.
The owner of the bus loops `fb = port.command(controller.tick(fb, dt))`
(standalone) or merges the returned command into its own array (adapters)."""

from __future__ import annotations

from .port import Feedback, MitCommand
from .profile import GripperProfile
from .tracker import MultiTurnTracker


class GripperController:
    def __init__(self, entry: dict, profile: GripperProfile):
        profile.validate()
        if "open" not in entry or "closed" not in entry:
            raise ValueError("cal entry needs open/closed marks -- calibrate first")
        self.entry = entry
        self.profile = profile
        self.kd = profile.tmax_nm / profile.vmax
        self._lo = min(entry["open"], entry["closed"])
        self._hi = max(entry["open"], entry["closed"])
        self.tracker = MultiTurnTracker()
        self.goal: float | None = None
        self.stalled = False
        self._stall_t = 0.0

    # --- goals ---
    def goto_rad(self, pos: float) -> None:
        self.goal = min(self._hi, max(self._lo, pos))
        self.stalled = False
        self._stall_t = 0.0

    def goto_frac(self, frac: float) -> None:
        frac = min(1.0, max(0.0, frac))
        self.goto_rad(self.entry["closed"] + frac * (self.entry["open"] - self.entry["closed"]))

    def open(self) -> None:
        self.goto_frac(1.0)

    def close(self) -> None:
        self.goto_frac(0.0)

    def hold(self) -> None:
        if self.position is not None:
            self.goal = self.position

    # --- state ---
    @property
    def position(self) -> float | None:
        return self.tracker.position if self.tracker.seen else None

    @property
    def at_goal(self) -> bool:
        return (self.goal is not None and self.position is not None
                and abs(self.goal - self.position) < 0.05)

    def adopt_park(self, last_position: float) -> None:
        self.tracker = MultiTurnTracker(start_unwrapped=last_position)

    def anchor(self, known_position: float) -> None:
        self.tracker.anchor(known_position)

    def park_fields(self) -> dict:
        return {"last_position": self.tracker.position, "last_wrapped": self.tracker.wrapped}

    # --- the tick ---
    def tick(self, fb: Feedback, dt: float) -> MitCommand:
        if fb.faulted:
            return MitCommand()
        pos = self.tracker.update(fb.position)
        if self.goal is None or self.stalled:
            return MitCommand(kd=self.kd)          # damp to zero velocity (hold)
        p = self.profile
        v = p.sw_kp * (self.goal - pos)
        v = min(p.vmax, max(-p.vmax, v))
        dv = p.tmax_nm / self.kd                    # = vmax by construction
        v = min(fb.velocity + dv, max(fb.velocity - dv, v))
        # stall detection on MEASURED torque (current sense), like the bench code
        if abs(fb.torque) > p.stall_torque_frac * p.tmax_nm and not self.at_goal:
            self._stall_t += dt
            if self._stall_t > p.stall_time_s:
                self.stalled = True
                self.goal = pos
                return MitCommand(kd=self.kd)
        else:
            self._stall_t = 0.0
        return MitCommand(vel=v, kd=self.kd)
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_controller.py -q` → `6 passed`. Tune only the sim's physical constants if a physics test is marginal; the cap assertion and frac mapping must pass as written.
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: GripperController pure tick core with torque cap and stall hold"`

---

### Task 7: `damiao/can_port.py` — the synchronous hardware port

**Files:**
- Create: `src/fastgripper_dm/damiao/can_port.py`, `tests/test_can_port.py`

**Interfaces:**
- Consumes: `DM4310`, `decode_feedback` (existing `damiao/dm4310.py`), `port.py` contracts.
- Produces:
  ```python
  class DamiaoCanPort:                     # implements MotorPort
      def __init__(self, bus, motor_id: int, master_id: int, retry_budget_s: float = 0.2): ...
      pos_window = 12.5
      # command(): mit frame + wait for THIS motor's reply; retries within budget then PortError
      # on fb.faulted: clear_error + enable with backoff (10 x 0.3 s) inside recover(); command()
      #   raises PortFault (subclass of PortError, carries the code) so the caller can decide
      # read(): zero-gain command (kd=0) -> feedback, no motion
      # close(): stop command x3, disable x5 with feedback reads, idempotent
  class PortFault(PortError): code: int
  ```

- [ ] **Step 1: Write the failing tests**

`tests/test_can_port.py`:
```python
import pytest
import can
from fastgripper_dm.damiao.can_port import DamiaoCanPort, PortFault
from fastgripper_dm.damiao.dm4310 import DM4310, float_to_uint
from fastgripper_dm.port import MitCommand, PortError


class FakeMotorBus:
    """Replies to MIT/special frames like a DM at motor_id, feedback on master_id."""

    def __init__(self, motor_id=0x07, master_id=0x17, status=0x1, position=1.0):
        self.motor_id, self.master_id = motor_id, master_id
        self.status, self.position = status, position
        self.rx = []
        self.sent = []

    def _feedback(self):
        p = float_to_uint(self.position, -12.5, 12.5, 16)
        d = [((self.status << 4) | (self.motor_id & 0x0F)) & 0xFF, p >> 8, p & 0xFF, 0x80, 0x00, 0x00, 25, 26]
        return can.Message(arbitration_id=self.master_id, data=bytes(d), is_rx=True)

    def send(self, msg):
        self.sent.append(msg)
        if msg.arbitration_id == self.motor_id:
            if msg.data[-1] == 0xFB:      # clear_error
                self.status = 0x0
                return
            if msg.data[-1] == 0xFC:      # enable
                self.status = 0x1
            self.rx.append(self._feedback())

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None


def test_command_returns_this_motors_feedback():
    bus = FakeMotorBus(position=2.5)
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.enable()
    fb = port.command(MitCommand(vel=1.0, kd=0.5))
    assert fb.position == pytest.approx(2.5, abs=1e-3) and fb.enabled


def test_silent_motor_raises_porterror_within_budget():
    bus = FakeMotorBus()
    bus.send = lambda msg: None            # nothing ever answers
    port = DamiaoCanPort(bus, 0x07, 0x17, retry_budget_s=0.05)
    with pytest.raises(PortError):
        port.command(MitCommand())


def test_fault_triggers_recovery_then_portfault():
    bus = FakeMotorBus(status=0xD)         # latched comm loss; FB/FC path clears it
    port = DamiaoCanPort(bus, 0x07, 0x17, retry_budget_s=0.05)
    fb = port.command(MitCommand())        # recover() clears + enables -> healthy reply
    assert fb.enabled
    assert any(m.data[-1] == 0xFB for m in bus.sent)   # clear_error was sent


def test_read_is_zero_gain():
    bus = FakeMotorBus()
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.read()
    mit = [m for m in bus.sent if m.arbitration_id == 0x07 and m.data[-1] not in (0xFB, 0xFC, 0xFD)]
    assert mit, "read must send a command frame"
    # kd bits (byte 5 high nibble + byte 6 low? -- decode: kd occupies data[5]<<4|data[6]>>4)
    d = mit[-1].data
    assert (d[5] << 4) | (d[6] >> 4) == 0  # kd == 0


def test_close_disables():
    bus = FakeMotorBus()
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.enable()
    port.close()
    assert any(m.data[-1] == 0xFD for m in bus.sent)
    port.close()                            # idempotent
```

- [ ] **Step 2: Run to verify failure.**

- [ ] **Step 3: Implement**

`src/fastgripper_dm/damiao/can_port.py`:
```python
"""Synchronous MotorPort over a dedicated CAN channel (python-can)."""

from __future__ import annotations

import time

from ..port import Feedback, MitCommand, PortError
from .dm4310 import DM4310


class PortFault(PortError):
    def __init__(self, code: int, text: str):
        super().__init__(text)
        self.code = code


class DamiaoCanPort:
    pos_window = 12.5

    def __init__(self, bus, motor_id: int, master_id: int, retry_budget_s: float = 0.2):
        self.motor = DM4310(bus, can_id=motor_id, master_id=master_id)
        self.retry_budget_s = retry_budget_s
        self._closed = False

    def enable(self) -> None:
        self.motor.enable()

    def disable(self) -> None:
        self.motor.disable()

    def clear_error(self) -> None:
        self.motor.clear_error()

    def _txrx(self, cmd: MitCommand) -> Feedback:
        deadline = time.monotonic() + self.retry_budget_s
        while True:
            self.motor.mit_control(cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau)
            raw = self.motor.read_feedback(timeout=min(0.05, self.retry_budget_s))
            if raw is not None:
                return Feedback(position=raw.position, velocity=raw.velocity,
                                torque=raw.torque, error_code=raw.error, t=time.monotonic())
            if time.monotonic() > deadline:
                raise PortError(f"motor 0x{self.motor.can_id:02X}: no feedback within "
                                f"{self.retry_budget_s:.2f}s")

    def _recover(self) -> Feedback | None:
        for _ in range(10):
            self.motor.clear_error()
            time.sleep(0.01)
            self.motor.enable()
            try:
                fb = self._txrx(MitCommand())
            except PortError:
                time.sleep(0.3)
                continue
            if not fb.faulted:
                return fb
            time.sleep(0.3)
        return None

    def command(self, cmd: MitCommand) -> Feedback:
        fb = self._txrx(cmd)
        if fb.faulted:
            recovered = self._recover()
            if recovered is None:
                raise PortFault(fb.error_code,
                                f"motor fault '{fb.fault_text}' did not clear after 10 attempts")
            return recovered
        return fb

    def read(self) -> Feedback:
        return self.command(MitCommand())    # zero gains: no motion

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for _ in range(3):
            try:
                self.motor.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                break
            time.sleep(0.02)
        for _ in range(5):
            try:
                self.motor.disable()
                self.motor.read_feedback(timeout=0.1)
            except Exception:
                break
            time.sleep(0.05)
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_can_port.py -q` → `5 passed`.
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: DamiaoCanPort with retry budget and fault recovery"`

---

### Task 8: `FastGripper` facade — connect modes, park/restore, context manager

**Files:**
- Modify: `src/fastgripper_dm/__init__.py`
- Create: `src/fastgripper_dm/facade.py`, `tests/test_facade.py`

**Interfaces:**
- Consumes: everything above plus `calstore` (load/save/get_entry/entry_profile) and `canbus.open_bus`.
- Produces:
  ```python
  class FastGripper:
      @classmethod
      def standalone(cls, interface=None, channel=None, gripper=None, cal_path=None,
                     home="auto") -> "FastGripper"
          # reads config.json for interface/channel/motor ids when omitted
      def connect(self) -> None
          # home="auto": if last_wrapped present and |boot.position - last_wrapped| (wrapped
          #   distance) <= profile.park_tolerance_rad -> adopt_park(last_position); else if
          #   auto_fallback (default) -> home_against_stop(); "assume_closed": anchor at the
          #   closed mark; "off": adopt nothing (tracker starts at 0-frame)
      def home_against_stop(self) -> None
          # probe toward close_dir at probe_vel under probe_tmax until torque/stall contact;
          # requires entry["stop_closed"]; applies the URtest sanity guards
          # (re-anchor <= ~3 turns; homed position inside marks +/- 2.0 rad) or raises HomingError
      def goto(self, frac) / open() / close(); state -> dict; stalled/at_goal passthrough
      def park(self) -> None      # writes park_fields + timestamps into the entry; atomic save
      def disconnect(self) -> None  # hold -> park -> port.close(); idempotent
      __enter__/__exit__
  class HomingError(RuntimeError)
  # __init__.py re-exports FastGripper, GripperProfile, __version__ = "0.1.0.dev0"
  ```
  The run loop lives here: `_run_until(predicate, timeout)` loops
  `fb = port.command(ctrl.tick(fb, dt))` at 50 Hz.

- [ ] **Step 1: Write the failing tests**

`tests/test_facade.py`:
```python
import json
import pytest
from fastgripper_dm.facade import FastGripper, HomingError
from fastgripper_dm.sim import SimulatedWormGripper


def make_store(tmp_path, entry):
    p = tmp_path / "gripper_cal.json"
    p.write_text(json.dumps({"format": 2, "grippers": {"default": entry}}))
    return str(p)


def sim_gripper(tmp_path, entry, start=1.0, home="auto"):
    cal = make_store(tmp_path, entry)
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=start)
    g = FastGripper(port=sim, cal_path=cal, gripper="default", home=home)  # test ctor: inject port
    return sim, g


ENTRY = {"open": -30.0, "closed": 2.0, "stop_closed": 3.0,
         "last_position": 1.0, "last_wrapped": 1.0}


def test_auto_restores_when_park_matches(tmp_path):
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=1.0)
    g.connect()
    assert g.position == pytest.approx(1.0, abs=0.05)


def test_auto_stall_homes_on_mismatch(tmp_path):
    # jaws "moved by hand": sim starts 2 rad away from the park
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=-1.0)
    g.connect()                                  # falls back to homing against stop_closed=3.0
    # after homing, the tracker frame must agree with ground truth at the stop datum
    g.goto(0.0)                                   # close to the mark (2.0)
    g.wait(timeout=8.0)
    assert sim.true_position == pytest.approx(2.0, abs=0.3)


def test_homing_guard_rejects_absurd_reanchor(tmp_path):
    entry = dict(ENTRY)
    entry["stop_closed"] = 90.0                   # nonsense datum -> re-anchor > 3 turns
    sim, g = sim_gripper(tmp_path, entry, start=-1.0)
    with pytest.raises(HomingError):
        g.connect()


def test_park_persists_and_next_auto_restores(tmp_path):
    cal = make_store(tmp_path, dict(ENTRY))
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=1.0)
    with FastGripper(port=sim, cal_path=cal, gripper="default") as g:
        g.goto(0.5)
        g.wait(timeout=8.0)
    saved = json.load(open(cal))["grippers"]["default"]
    assert saved["last_position"] == pytest.approx(sim.true_position, abs=0.05)
    # second session, same sim position: auto restore, no homing motion
    pos_before = sim.true_position
    g2 = FastGripper(port=sim, cal_path=cal, gripper="default")
    g2.connect()
    assert sim.true_position == pytest.approx(pos_before, abs=0.05)
    assert g2.position == pytest.approx(saved["last_position"], abs=0.05)


def test_missing_marks_raise(tmp_path):
    sim, g = sim_gripper(tmp_path, {"stop_closed": 3.0}, start=1.0)
    with pytest.raises(ValueError):
        g.connect()
```

- [ ] **Step 2: Run to verify failure.**

- [ ] **Step 3: Implement**

`src/fastgripper_dm/facade.py`:
```python
"""FastGripper: the one object a user drives. Owns a MotorPort, a
GripperController, and the cal entry's park lifecycle."""

from __future__ import annotations

import time

from .calstore import default_cal_path, entry_profile, get_entry, load_store, save_store
from .controller import GripperController
from .port import Feedback, MitCommand, MotorPort, PortError, SPAN
from .tracker import MultiTurnTracker


class HomingError(RuntimeError):
    pass


def _wrapped_dist(a: float, b: float) -> float:
    d = abs(a - b) % SPAN
    return min(d, SPAN - d)


class FastGripper:
    def __init__(self, port: MotorPort | None = None, interface: str | None = None,
                 channel: str | None = None, gripper: str | None = None,
                 cal_path: str | None = None, home: str = "auto", auto_fallback: str = "stall"):
        self._given_port = port
        self._interface, self._channel = interface, channel
        self._gripper_name = gripper
        self._cal_path = cal_path or default_cal_path()
        self._home_mode = home
        self._auto_fallback = auto_fallback
        self.port: MotorPort | None = port
        self.ctrl: GripperController | None = None
        self._store = None
        self._entry = None
        self._connected = False

    @classmethod
    def standalone(cls, interface=None, channel=None, gripper=None, cal_path=None, home="auto"):
        from .cli import _load_config

        cfg = _load_config()
        return cls(interface=interface or cfg.get("interface", "auto"),
                   channel=channel or cfg.get("channel"),
                   gripper=gripper, cal_path=cal_path, home=home)

    # --- lifecycle ---
    def connect(self) -> None:
        self._store = load_store(self._cal_path)
        name, self._entry = get_entry(self._store, self._gripper_name)
        self._gripper_name = name
        profile = entry_profile(self._entry)
        self.ctrl = GripperController(self._entry, profile)
        if self.port is None:
            from .cli import _load_config
            from .damiao.canbus import open_bus

            cfg = _load_config()
            bus = open_bus(self._interface, self._channel)
            from .damiao.can_port import DamiaoCanPort

            self.port = DamiaoCanPort(bus, int(self._entry.get("motor_id", cfg.get("motor_id", 0x01))),
                                      int(self._entry.get("master_id", cfg.get("master_id", 0x00))))
        self.port.enable()
        boot = self.port.read()
        mode = self._home_mode
        if mode == "auto":
            lw = self._entry.get("last_wrapped")
            if lw is not None and _wrapped_dist(boot.position, lw) <= profile.park_tolerance_rad:
                self.ctrl.adopt_park(self._entry["last_position"])
                self.ctrl.tick(boot, 0.0)
            elif self._auto_fallback == "stall":
                self.home_against_stop()
            else:
                raise HomingError(
                    f"gripper is not where it was parked (wrapped {boot.position:+.2f} vs "
                    f"saved {lw}) -- run home_against_stop() / `autocal home`, or connect "
                    f"with home='assume_closed' after closing the jaws by hand")
        elif mode == "assume_closed":
            self.ctrl.tick(boot, 0.0)
            self.ctrl.anchor(self._entry["closed"])
        elif mode == "stall":
            self.home_against_stop()
        elif mode == "off":
            self.ctrl.tick(boot, 0.0)
        else:
            raise ValueError(f"unknown home mode '{mode}'")
        self.ctrl.hold()
        self._connected = True

    def home_against_stop(self) -> None:
        """Probe toward the closed stop under the profile's probe caps, anchor
        against the recorded stop_closed datum, with the URtest sanity guards."""
        if "stop_closed" not in self._entry:
            raise HomingError("entry has no stop_closed datum -- run `autocal full` once")
        p = self.ctrl.profile
        d = p.close_dir
        probe = GripperController({"open": -1e9, "closed": 1e9}, p)  # unclamped goals for probing
        probe.tracker = MultiTurnTracker()
        fb = self.port.read()
        probe.tracker.update(fb.position)
        start = probe.tracker.position
        t_end = time.monotonic() + 90.0
        contact_since = None
        kd = 1.0
        while True:
            now = time.monotonic()
            if now > t_end:
                raise HomingError("no contact within 90 s")
            v = d * p.probe_vel
            v = min(fb.velocity + p.probe_tmax / kd, max(fb.velocity - p.probe_tmax / kd, v))
            fb = self.port.command(MitCommand(vel=v, kd=kd))
            probe.tracker.update(fb.position)
            if abs(probe.tracker.position - start) > 40.0:
                raise HomingError("traveled 40 rad without contact -- wrong close_dir or no stop")
            contact = abs(fb.torque) > p.contact_torque or abs(fb.velocity) < 0.1
            if contact:
                contact_since = contact_since or now
                if now - contact_since > 0.3:
                    break
            else:
                contact_since = None
            time.sleep(0.02)
        stop_here = probe.tracker.position
        offset = self._entry["stop_closed"] - stop_here
        if abs(offset) > 3 * 6.283185 + 1.0:
            raise HomingError(f"re-anchor offset {offset:+.2f} rad exceeds ~3 turns -- probe "
                              f"likely triggered on friction; raise contact_torque and retry")
        anchored = stop_here + offset
        lo = min(self._entry["open"], self._entry["closed"]) - 2.0
        hi = max(self._entry["open"], self._entry["closed"]) + 2.0
        if not (lo <= anchored <= hi):
            raise HomingError(f"homed position {anchored:+.2f} outside calibrated range -- recalibrate")
        # back off the stop, then hand the anchored frame to the main controller
        back_goal = anchored - d * p.margin
        self.ctrl.adopt_park(anchored)
        self.ctrl.tick(fb, 0.0)
        self.ctrl.goto_rad(back_goal)
        self.wait(timeout=10.0)

    # --- motion ---
    def _run_until(self, predicate, timeout: float) -> None:
        fb = self.port.read()
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            fb = self.port.command(self.ctrl.tick(fb, 0.02))
            if predicate():
                return
            time.sleep(max(0.0, 0.02 - (time.monotonic() - t0)))
        raise TimeoutError("gripper did not reach the goal in time")

    def wait(self, timeout: float = 10.0) -> None:
        self._run_until(lambda: self.ctrl.at_goal or self.ctrl.stalled, timeout)

    def goto(self, frac: float) -> None:
        self.ctrl.goto_frac(frac)

    def open(self) -> None:
        self.ctrl.open()

    def close(self) -> None:
        self.ctrl.close()

    @property
    def position(self):
        return self.ctrl.position if self.ctrl else None

    @property
    def stalled(self) -> bool:
        return bool(self.ctrl and self.ctrl.stalled)

    # --- park / teardown ---
    def park(self) -> None:
        if self.ctrl and self.ctrl.tracker.seen:
            self._entry.update(self.ctrl.park_fields())
            self._entry["parked_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            save_store(self._cal_path, self._store)

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False
        try:
            self.park()
        finally:
            try:
                self.port.close()
            except PortError:
                pass

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
```

`src/fastgripper_dm/__init__.py` becomes:
```python
"""fastgripper-dm: FastGripper worm-gear gripper on a Damiao DM-J4310 over CAN."""

from .facade import FastGripper, HomingError
from .profile import GripperProfile, PRESETS, TMAX_CAP

__version__ = "0.1.0.dev0"
__all__ = ["FastGripper", "HomingError", "GripperProfile", "PRESETS", "TMAX_CAP", "__version__"]
```

- [ ] **Step 4: Run to verify pass** — `uv run pytest tests/test_facade.py -q` → `5 passed`; full suite still green.
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: FastGripper facade with auto/stall/assume_closed connect and park lifecycle"`

---

### Task 9: CLI verbs on the facade (`open|close|goto|home|status`), retire `drive`'s direct path

**Files:**
- Modify: `src/fastgripper_dm/cli.py`, `src/fastgripper_dm/tools/drive.py` (delete body, delegate), `tests/test_harvest.py` (CLI list)
- Create: `tests/test_cli_verbs.py`

**Interfaces:**
- Produces: `fastgripper-dm open | close | goto <pct 0-100> | home | status [--gripper NAME]`, all through `FastGripper.standalone()` and the `_cli.run` exit path. `drive <pct>` stays as an alias for `goto`. `status` prints position (rad + %), marks, stalled flag, and never moves.

- [ ] **Step 1: Write the failing test**

`tests/test_cli_verbs.py`:
```python
import subprocess
import sys

import pytest


@pytest.mark.parametrize("sub", ["open", "close", "goto", "home", "status", "drive"])
def test_verbs_parse(sub):
    argv = [sys.executable, "-m", "fastgripper_dm.cli", sub, "-h"]
    out = subprocess.run(argv, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr


def test_goto_requires_a_percentage():
    out = subprocess.run([sys.executable, "-m", "fastgripper_dm.cli", "goto"],
                        capture_output=True, text=True)
    assert out.returncode != 0
```

- [ ] **Step 2: Run to verify failure** (unknown subcommands).

- [ ] **Step 3: Implement.** In `cli.py`:

```python
def cmd_motion(args, extra):
    from .facade import FastGripper

    frac = {"open": 1.0, "close": 0.0}.get(args.cmd)
    if frac is None:
        frac = args.pct / 100.0
    from .tools._cli import run

    def go():
        with FastGripper.standalone(gripper=args.gripper) as g:
            g.goto(frac)
            g.wait(timeout=15.0)
            print(f"at {g.position:+.2f} rad" + (" (stalled/holding)" if g.stalled else ""))
    run(go)


def cmd_home(args, extra):
    from .facade import FastGripper
    from .tools._cli import run

    def go():
        with FastGripper.standalone(gripper=args.gripper, home="stall") as g:
            print(f"homed; at {g.position:+.2f} rad")
    run(go)


def cmd_status(args, extra):
    from .facade import FastGripper
    from .tools._cli import run

    def go():
        g = FastGripper.standalone(gripper=args.gripper, home="off")
        g.connect()
        e = g._entry
        pos = g.position
        pct = 100.0 * (pos - e["closed"]) / (e["open"] - e["closed"]) if pos is not None else None
        print(f"entry '{g._gripper_name}': open {e['open']:+.2f} closed {e['closed']:+.2f} rad")
        print(f"position (this window): {pos:+.2f} rad" + (f" ~ {pct:.0f}% open" if pct is not None else ""))
        print("NOTE: 'off' mode has no absolute frame; use preflight/park for trusted state")
        g.port.disable()
        g.port.close()
    run(go)
```

Parser additions (in `main()`):
```python
    for name in ("open", "close"):
        p = sub.add_parser(name, help=f"{name} fully (torque-capped)")
        p.add_argument("--gripper", default=None)
    for name in ("goto", "drive"):
        p = sub.add_parser(name, help="go to a percentage open (0 = closed, 100 = open)")
        p.add_argument("pct", type=float)
        p.add_argument("--gripper", default=None)
    p = sub.add_parser("home", help="stall-home against the closed stop, then hold just off it")
    p.add_argument("--gripper", default=None)
    p = sub.add_parser("status", help="print entry marks and the current wrapped position (no motion)")
    p.add_argument("--gripper", default=None)
```
Dispatch: `open/close/goto/drive → cmd_motion`, `home → cmd_home`, `status → cmd_status`. Remove `drive` from the pass-through tool list and delete `tools/drive.py`'s direct-DM4310 body (keep the file as `from ..cli import main` shim or delete it and the old import).

- [ ] **Step 4: Run** — `uv run pytest -q` all green (update `test_harvest.py`'s CLI-subcommand list to the new set).
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fastgripper-dm: open/close/goto/home/status verbs on the FastGripper facade"`

---

### Task 10: preflight upgrades — profile cap, entry↔motor binding, park check

**Files:**
- Modify: `src/fastgripper_dm/tools/preflight.py`, `tests/test_harvest.py`

**Interfaces:**
- `evaluate(...)` gains `entry_motor_id: int | None` and `profile_tmax: float | None` params:
  - FAIL when `entry_motor_id` is set and differs from the answering motor id (wrong `--gripper` for this motor);
  - FAIL when `profile_tmax > TMAX_CAP`;
  - WARN "park missing → will stall-home" instead of nothing;
  - existing checks unchanged.

- [ ] **Step 1: Extend the tests** (in `test_harvest.py`):

```python
def test_preflight_entry_motor_binding():
    from fastgripper_dm.tools.preflight import evaluate
    f = evaluate(True, 0, 8000, 8000, {"open": -20.0, "closed": 3.0, "last_position": 1.0,
                                       "motor_id": 8}, entry_motor_id=8, answered_id=7)
    assert any(x.level == "FAIL" and "entry" in x.text for x in f)


def test_preflight_profile_cap():
    from fastgripper_dm.tools.preflight import evaluate
    f = evaluate(True, 0, 8000, 8000, {"open": -20.0, "closed": 3.0, "last_position": 1.0},
                 profile_tmax=2.5)
    assert any(x.level == "FAIL" and "tmax" in x.text for x in f)
```

- [ ] **Step 2: Run to verify failure** (unknown kwargs).
- [ ] **Step 3: Implement** — add the two parameters (default `None`, `answered_id=None`) and findings:

```python
    if entry_motor_id is not None and answered_id is not None and entry_motor_id != answered_id:
        out.append(Finding("FAIL", f"cal: entry is for motor 0x{entry_motor_id:02X} but "
                           f"0x{answered_id:02X} answered -- wrong --gripper for this unit"))
    if profile_tmax is not None and profile_tmax > TMAX_CAP:
        out.append(Finding("FAIL", f"profile: tmax_nm {profile_tmax} exceeds the {TMAX_CAP} Nm cap"))
```
`run_preflight` passes `entry.get("motor_id")`, the id that answered, and `entry_profile(entry).tmax_nm`.

- [ ] **Step 4: Run** — full suite green. **Step 5: Commit.**

---

### Task 11: Bench validation + watchdog-unit experiment (hardware; human present)

No code. A checklist executed on the Mac bench with gripper #2 (or #1), recorded in the commit message of a `docs/fastgripper-dm/bench-validation-v0.1.md` note.

- [ ] `fastgripper-dm preflight` → GO
- [ ] `fastgripper-dm status`; `open`; `close`; `goto 50` — motion correct, no faults, park saved on exit
- [ ] Park/restore: `close`, unplug USB mid-`open` (SIGKILL the process), replug, `preflight`, `open` → auto restore or clean stall-home; no wrong-direction motion
- [ ] Grip test: object in jaws, `close` → stalls and holds at cap; `open` releases
- [ ] **Watchdog unit experiment:** `watchdog --set 2000`, jog via `calibrate` at 50 Hz for 30 s.
      Faults ⇒ unit is sub-ms (8000 ≈ 80 ms if 10 µs); no faults ⇒ try 500 again to reconfirm, then
      bisect (1000, 4000). Record the value/behaviour table in the bench note and set
      `GripperProfile.watchdog`'s comment to the measured unit. Restore 8000 afterwards.
- [ ] `make dm-test && make gate` green; commit the bench note.

---

### Task 12: Release `dm-v0.1.0` (TestPyPI → PyPI)

**Files:**
- Create: `.github/workflows/release-dm.yml`
- Modify: `packages/fastgripper-dm/pyproject.toml` (version `0.1.0`), `src/fastgripper_dm/__init__.py`

- [ ] **Step 1:** `release-dm.yml` (adapted from fastgripper-lerobot's proven workflow):

```yaml
name: release-dm
on:
  push:
    tags: ["dm-v*"]
  workflow_dispatch: {}          # dry run -> TestPyPI
jobs:
  build:
    runs-on: ubuntu-latest
    defaults: { run: { working-directory: packages/fastgripper-dm } }
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/setup-uv@v3
      - run: uv build
      - uses: actions/upload-artifact@v4
        with: { name: dist, path: packages/fastgripper-dm/dist/ }
  testpypi:
    if: github.event_name == 'workflow_dispatch'
    needs: build
    runs-on: ubuntu-latest
    environment: testpypi-dm
    permissions: { id-token: write }
    steps:
      - uses: actions/download-artifact@v4
        with: { name: dist, path: dist/ }
      - uses: pypa/gh-action-pypi-publish@release/v1
        with: { repository-url: "https://test.pypi.org/legacy/" }
  pypi:
    if: startsWith(github.ref, 'refs/tags/dm-v')
    needs: build
    runs-on: ubuntu-latest
    environment: pypi-dm
    permissions: { id-token: write }
    steps:
      - uses: actions/download-artifact@v4
        with: { name: dist, path: dist/ }
      - uses: pypa/gh-action-pypi-publish@release/v1
```

- [ ] **Step 2 (browser, human):** register pending publishers on test.pypi.org and pypi.org for project `fastgripper-dm`, owner `Transcend-Mechanics`, repo `fastgripper-software`, workflow `release-dm.yml`, environments `testpypi-dm` / `pypi-dm`; create both environments in GitHub repo settings (follow `fastgripper-lerobot/docs/RELEASING.md`).
- [ ] **Step 3:** bump version to `0.1.0` in `pyproject.toml` + `__init__.py`; commit.
- [ ] **Step 4:** `workflow_dispatch` the dry run → verify install from TestPyPI in a scratch venv (`pip install -i https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple fastgripper-dm`).
- [ ] **Step 5:** `git tag dm-v0.1.0 && git push --tags` → verify `pip install fastgripper-dm` from PyPI; update README/quickstarts to drop the git-URL install.
- [ ] **Step 6: Commit** any doc changes.

---

## Self-review checklist (executed when the plan is complete)

1. Spec coverage: §3.1 port (T2, T7), §3.2 controller/tick + cap + stall + park semantics (T3, T6, T8), §3.3 calstore/profile (T4 + harvest), §3.3a profiles (T4), §3.4 CLI (harvest + T9), §3.6 watchdog (harvest `setup` + T10/T11 unit experiment), §4 failure table (T7 recovery, T8 fallback, harvest canbus), §5 sim/tests (T5, per-task tests), §6 steps 0–3 (Task 0 done; T12 = release), §7/§8 (patches done; docs in T12), deferred per scope note: autocal/calibrate on controller, adapters (Plan 2), drills (Plan 4).
2. No placeholders: every code step carries complete code.
3. Type consistency: `MitCommand`/`Feedback` fields, `GripperProfile` field names, `entry_profile`, `park_fields`, `FastGripper` ctor kwargs match across tasks 4–10.
