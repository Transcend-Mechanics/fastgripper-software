# bench/ — hardware-in-the-loop tools

Everything under `bench/` talks to real hardware (a YAM arm, an SO-101
leader, CAN adapters, serial ports) and is exercised by hand on a physical
rig, not by `make dm-test`. It is **not** shipped as part of the
`fastgripper-dm` package — it consumes the package from its own venv.

```
bench/
  README.md              this file
  local.toml.example     template for bench/local.toml (git-ignored)
  yam/                    SO-101 -> YAM teleop bridge + GUI + cal files
  so101/                  SO-101-leader-only diagnostics (no YAM needed)
  tools/                  bus/USB diagnostics shared across rigs
```

## Setup (once per bench)

```bash
cd bench
uv venv --python 3.12
uv pip install fastgripper-dm "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"
../patches/setup-mac.sh .venv     # macOS only: patches gs_usb for Darwin
```

(`fastgripper-dm` is also installable from this repo's own venv at
`packages/fastgripper-dm/.venv` if you're developing the package and the
bench in the same session — either venv works as long as it has both
`fastgripper-dm` and `i2rt` installed.)

Then set up this bench's device map:

```bash
cp bench/local.toml.example bench/local.toml
# edit bench/local.toml with this rig's real serial ports (ls /dev/cu.usbmodem*)
```

`bench/local.toml` is git-ignored — it holds bench-specific serial numbers
that must never land in the repo (see `make gate`, which fails the build if
one shows up anywhere under `packages/` or `bench/`).

## What's here

- **`yam/`** — the SO-101-leader -> YAM-follower teleop bridge
  (`so101_teleop.py`), its GUI (`teleop_gui.py`), the runbook (`TELEOP.md`),
  hardware gotchas (`running-the-yam-gotchas.md`), and the gripper/trigger
  calibration files the teleop reads by default.
- **`so101/`** — SO-101 leader-only diagnostics that don't need a YAM
  follower: `trigger_probe.py` (live trigger-to-gripper-% readout) and
  `leader_drop_probe.py` (does follower motion knock the leader off USB).
- **`tools/`** — cross-rig diagnostics: `usb_stress.py` (soak/watch USB
  links, driven by `bench/local.toml`), `bus_dump.py` (raw CAN frame dump),
  `wiggle_joints.py` (YAM joint roll-call), `rpm_stats.py` (gripper-motor
  RPM stats from teleop CSVs).

See `bench/yam/TELEOP.md` for the full teleop runbook.
