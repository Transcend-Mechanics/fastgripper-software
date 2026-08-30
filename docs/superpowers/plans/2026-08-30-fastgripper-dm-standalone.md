# fastgripper-dm v0.1.0 (standalone) Implementation Plan

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
