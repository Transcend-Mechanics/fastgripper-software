# DM-J4310 Duplicated-Module Consolidation Audit

**Date:** 2026-08-30
**Scope:** Map every copy of the shared DM-J4310 gripper Python modules across the local repos, identify the fixes each copy carries, and produce a per-module merge plan toward ONE canonical package.

All paths below are relative to `/Users/alexmcleod/Documents/Transcend/Repos/`. Findings come from reading the actual files and pairwise `diff`. `build/` and `.venv/` were ignored.

**Excluded as unrelated:** `data-pipeline-skunkworks/env/calibrate.py` (277 lines) is NOT a gripper module — it is a data-pipeline "witness/falsification calibration gate" (`python -m env.calibrate <task_id>`). It shares only the filename. Do not merge it.

---

## Dependency graph

Leaf modules with no internal dependencies: `calstore.py`, `dm4310.py`, `canbus.py` (canbus depends on the third-party `can` and `usb` packages only). `autocal.py` and `calibrate.py` each import all three leaves. Every tool script imports the four shared modules.

**Two import styles exist — this is the central packaging decision:**

- **Flat / script style** (`from canbus import …`): `dm-j4310-test/*`, `YAM Test/*`, `YAM Test/gripper-openarm-portable/*`, `URtest/*.py` (top level).
- **Package / relative style** (`from .canbus import …`): `URtest/fastgripper_ur/*`, `fastgripper-openarm/fastgripper/*` (this one also has an `__init__.py` re-exporting the shared API and a `_cli.py` console wrapper).

Consolidation must standardize on one style (recommended: relative imports inside a real package, with thin top-level shims if any legacy flat script must keep working).

Representative importers found:
- canbus: `bus_ping.py`, `bus_dump.py`, `usb_stress.py`, `preflight.py`, `smoke_test.py`, `probe_motor.py`, all `autocal`/`calibrate`/`gui`/`pad`/`doctor`/`drive`/`gripperd` copies.
- dm4310: same set plus `hello_world.py`, `rpm_stats.py` consumers.
- calstore: `autocal`, `calibrate`, `cal_doctor`/`doctor`, `gripper_tool`, `gripper_gui`/`gui`, `gripper_pad`/`pad`, `gripperd`, `so101_teleop.py`, `teleop_gui.py`.
- `fastgripper-openarm/fastgripper/__init__.py` re-exports `calstore`, `canbus`, `dm4310` symbols.
- `URtest/fastgripper_ur/cli.py` dispatches to `.autocal:main` and `.calibrate:main`.

---

## canbus.py

| copy | lines | distinguishing features / fixes |
|---|---|---|
| `YAM Test` = `URtest` = `URtest/fastgripper_ur` | 160 | **Canonical.** Auto interface detection (socketcan / slcan / gs_usb); slcan + socketcan + gs_usb support; gs_usb `index=0` addressing; open-WITHOUT-reset then TX-echo **alive check** (`_bus_alive`, L86-99); **drain-before-shutdown** that also **disposes the USB handle** (`os._exit`-safe, L121-123); **never** software-resets — raises a physical-recovery `SystemExit` instead (L141-149). The three copies are byte-identical. |
| `fastgripper-openarm/fastgripper` | 126 | Same safe open+alive+drain approach, but **no USB-handle dispose** inside the drain (L95), no `_hard_reset`/`_open` helper functions; generic `--motor_id` help text ("normally read from the cal file entry"); "24V supply" wording. Behaviourally safe, just older/thinner. |
| `YAM Test/gripper-openarm-portable` | 88 | **Regressed / dangerous.** Preemptively calls `dev.reset()` on every open (L65-77) — the exact behaviour YAM's own comments prove wedges the adapter off the USB bus (`[Errno 19]` until physical replug). Has drain-on-shutdown but **no alive check, no USB dispose**. Do not use. |
| `dm-j4310-test` | 72 | Oldest. No socketcan; hard-resets on open; no drain, no alive check. SH-C31A-specific docstring. |

**Base: `YAM Test/canbus.py` (160).** It is a strict superset of every safe behaviour. Nothing needs merging in. The 88- and 72-line hard-reset-on-open copies must be **discarded**, not merged — their reset-on-open logic is the documented failure cascade.

---

## dm4310.py

| copy | lines | distinguishing features / fixes |
|---|---|---|
| `YAM Test` = `URtest` = `URtest/fastgripper_ur` = `fastgripper-openarm/fastgripper` = `YAM Test/gripper-openarm-portable` | 204 | **Canonical.** `decode_feedback(msg, can_id=…)` performs **sign-magnitude / large-ID normalization** (L65-77): byte 0 is `(status << 4) + motor_id`, so a motor re-ID'd above `0x0F` (e.g. `0x20`) overflows the 4-bit ID field into the status nibble; subtracting the known `can_id` recovers status for both small and large IDs. `MotorFrame.read()` passes `can_id=self.can_id` (L198). All five copies byte-identical. |
| `dm-j4310-test` | 199 | Missing the normalization — `decode_feedback(msg)` reads `motor_id=d[0] & 0x0F`, `error=d[0] >> 4`, which mis-decodes any motor re-ID'd above `0x0F`. |

**Base: any 204-line copy.** Discard the 199-line `dm-j4310-test` copy.

---

## calstore.py — requires a real merge

| copy | lines | distinguishing features / fixes |
|---|---|---|
| `YAM Test` = `YAM Test/gripper-openarm-portable` | 68 | Format-2 named-entry store; legacy flat-file migration in `load_store`; `get_entry` (name optional iff one entry); `resolve_ids` (entry IDs win unless CLI overrides). **Plain, non-atomic `save_store`.** |
| `URtest` = `URtest/fastgripper_ur` | 74 | Adds **atomic write**: `tmp` file + `f.flush()` + `os.fsync()` + `os.replace()` (L41-46) — a crash mid-save cannot destroy the only calibration. |
| `fastgripper-openarm/fastgripper` | 79 | Adds **`default_cal_path()`** (`./gripper_cal.json` if present, else `~/.config/fastgripper/gripper_cal.json`, L28-35) so pip-installed console commands work from any directory; adds `os.makedirs(parent, …)` in `save_store`. **But reverts to a plain, non-atomic write.** |

**No single copy has both the atomic write and `default_cal_path()`.** That gap IS the merge.

**Base: `fastgripper-openarm/fastgripper/calstore.py` (79).** **Merge the atomic-write hunk from `URtest/calstore.py:41-46` into its `save_store`** while keeping `fastgripper-openarm`'s `os.makedirs(parent)`. Keep `default_cal_path()`. Result: named entries + legacy migration + `default_cal_path` + `makedirs` + atomic fsync/replace write.

---

## calibrate.py

| copy | lines | distinguishing features / fixes |
|---|---|---|
| `fastgripper-openarm/fastgripper` | 216 | **Most complete.** Up-front cal-entry resolution (`--gripper default=None` reuses the single existing entry rather than always writing `"default"`; raises the pick-a-name message when >1 entry; resolves motor IDs via `get_entry`/`resolve_ids`); `default_cal_path()` for `--cal`; peak-torque readout + `f` fast-toggle + `r` reset; saves `last_wrapped`, `span`, `method`, span-exceeds-encoder-window note; reloads store before write ("another tool may have written meanwhile"); `cli()` console entry via `_cli.run`. Relative imports. |
| `YAM Test/gripper-openarm-portable` | 209 | Same up-front entry resolution (`default=None` reuse), but **flat imports**, **no** `default_cal_path`, **no** `cli()`. |
| `URtest/fastgripper_ur` | 206 | Resolves IDs, but via a `try/except SystemExit` fallback and keeps `--gripper default="default"` (weaker — can silently spawn a second entry). Relative imports. |
| `YAM Test` = `URtest` | 195 | **No ID resolution from the entry** — talks to `args.motor_id` directly. This is the "re-ID'd gripper silently talks to `0x01` and reports no feedback" bug. peak-torque/`f`/`r`/multi-gripper present. Flat imports. |
| `dm-j4310-test` | 169 | Oldest: **no calstore** (writes a flat format-1 file directly with `json.dump`), `key.isupper()` fast-jog (A/D), no peak-torque, no `f`/`r`, no multi-gripper, no span-window note. |

**Base: `fastgripper-openarm/fastgripper/calibrate.py` (216).** It already carries every fix. The only open item is the import style (settled by the packaging decision above).

---

## autocal.py — requires a merge + a human decision

| copy | lines | distinguishing features / fixes |
|---|---|---|
| `URtest/fastgripper_ur` | 461 | **Most advanced.** `home`-mode **sanity guards** (the 2026-07-28 friction-hump fix, L337-369): rejects a re-anchor offset beyond `HOME_MAX_REANCHOR_RAD` (~3 turns) and rejects a homed position outside the calibrated marks ± `HOME_MARK_SLACK_RAD` — a probe that false-triggers on friction can no longer silently corrupt position knowledge. `single_touch` and `closed_only` are `BooleanOptionalAction` **DEFAULT ON**; `span_from_closed` fallback ladder (CLI → cal entry's saved value → `DEFAULT_SPAN_FROM_CLOSED=30.0`); saves `span_from_closed` into the cal entry; adds `import math`; `os._exit` fast-exit (inline); null-safe `.get() is None` checks. Relative imports. |
| `YAM Test` = `URtest` | 404 | `single_touch` and `closed_only`/`span_from_closed` present but **opt-in** flags (`span_from_closed` default `33.5`); inline `os._exit` fast-exit; **no home sanity guards**; non-null-safe `"key" in cal` checks. Flat imports. |
| `fastgripper-openarm/fastgripper` | 392 | 404-feature set **minus** the inline `os._exit` — replaced by `cli()` + `_cli.run` wrapper; adds `default_cal_path()` for `--cal`; null-safe `.get() is None` checks; "blank template entries" message. **No home guards, no defaults-on.** Relative imports. |
| `YAM Test/gripper-openarm-portable` | 346 | Minimal/oldest: **no** `single_touch`, **no** `closed_only`/`span_from_closed`, **no** fast-exit (plain `main()`); has the null-safe `.get() is None` checks. |

**Base: `URtest/fastgripper_ur/autocal.py` (461).** It is the only copy with the home-mode sanity guards and the span fallback ladder. **Merge from `fastgripper-openarm/fastgripper/autocal.py`: `default_cal_path()` for the `--cal` default (L176, L226)** so packaged installs work outside the repo directory. Then resolve the fast-exit conflict (see below).

---

## Behavioural conflicts a human must decide

1. **autocal `closed_only` / `single_touch` default (461 vs everything else).** The `URtest/fastgripper_ur` copy defaults **both ON**: it never probes the open stop (`closed_only`) and skips the double-touch agreement check (`single_touch`). That is correct for the UR gripper ("the open end has no qualified hard stop", "friction lumpy enough that double-touch false-aborts") but WRONG for grippers with two real hardstops, which would get half-calibrated. → Must become **per-gripper / per-profile configuration**, not a package-wide hard default.

2. **Fast-exit mechanism.** Inline `os._exit(code)` after `main()` (461, 404) vs the `cli()` + `_cli.run` wrapper (`fastgripper-openarm`, shared `_cli.py`). Both exist to dodge the libusb-at-interpreter-finalization SIGABRT on macOS. Pick ONE; the `_cli.run` wrapper is cleaner and already reused across `autocal`/`calibrate`/`gui`/`pad`/`doctor`/`drive`.

3. **`span_from_closed` default value.** `33.5` rad (YAM/URtest 404) vs `30.0` rad (`DEFAULT_SPAN_FROM_CLOSED`, UR 461). This is a physical range-of-motion value and MUST be per-gripper config, never a module constant — an overestimate drives `0%` commands into the open stop.

4. **calibrate `--gripper` default.** `None`-reuse-existing-entry (216 / 209) vs `"default"` (206 / 195). The `None` behaviour is safer (prevents a second silent entry). Adopt `None`.

---

## Hard-coded bench-specific values to externalize as configuration

- **Absolute cal-file path:** `YAM Test/gripper_tool.py:67` — `CAL_FILE = "/Users/alexmcleod/Documents/Transcend/Repos/YAM Test/gripper_cal.json"`. Replace with `default_cal_path()`.
- **Relative cal file:** `YAM Test/calibrate.py:37` — `CAL_FILE = "gripper_cal.json"` (also `--cal` default). Replace with `default_cal_path()`.
- **Motor IDs `0x07` / `0x17` (YAM gripper):** hard-coded in `set_gripper_id.py` (`--new_id` default `0x07`, L80-81; docstring identity "CAN ID 0x07, feedback 0x17"), `preflight.py:31` (`GRIPPER_ID = 0x07`), and `canbus.py` `--motor_id`/`--master_id` help text. Alt profile `0x20` / `0x30` is threaded through `set_gripper_id.py` examples (L13-16). → per-gripper config.
- **gs_usb VID/PID table** `[(0x1D50, 0x606F), (0x1209, 0x2323)]` — repeated in every `canbus.py`. Fine as a default, but should be surfaced as config for other adapters.
- **Device path glob** `/dev/tty.usbmodem*` and slcan example `/dev/tty.usbmodem2101` in `canbus.py`. Reasonable default; expose for non-macOS/other adapters.
- **App-name in config path:** `~/.config/fastgripper/gripper_cal.json` — the literal `fastgripper` is baked into `default_cal_path()`.
- **`span_from_closed` / `--expected_span`** physical constants (`33.5`, `30.0`, `31.1` in autocal usage examples) — per-gripper config.

---

## Single-copy modules (one line each)

**`fastgripper-openarm/fastgripper/`:**
- `doctor.py` (140) — turn-alias doctor for the DM4310 gripper (packaged twin of YAM `cal_doctor.py`).
- `drive.py` (129) — minimal programmatic control: drive the gripper to a percentage and exit.
- `pad.py` (330) — gamepad/keyboard control for one or two DM4310 grippers (packaged twin of YAM `gripper_pad.py`).
- `gui.py` (290) — standalone slider + buttons + live-telemetry GUI (packaged twin of YAM `gripper_gui.py`).
- `_cli.py` (28) — console-entry wrapper: run a tool's `main()` and exit via `os._exit`.

**`YAM Test/`:**
- `bus_ping.py` (104) — 5-second CAN bus triage: who is actually alive.
- `bus_dump.py` (78) — raw CAN frame dump for a few seconds.
- `cal_doctor.py` (133) — turn-alias doctor for the DM4310 gripper.
- `usb_stress.py` (153) — USB link stress tools for the two-rig bench (no motion).
- `preflight.py` (254) — go/no-go preflight for the YAM + SO-101 leader teleop rig (no motion).
- `gripper_tool.py` (296) — all-in-one worm-gear gripper tool, i2rt-native (own bus code, but imports `calstore`).
- `gripper_gui.py` (276) — standalone slider + buttons + live-telemetry GUI.
- `gripper_pad.py` (321) — gamepad/keyboard control for one or two DM4310 grippers.
- `set_gripper_id.py` (147) — assign the gripper DM4310 its chain identity (CAN ID `0x07`, feedback `0x17`).
- `wiggle_joints.py` (49) — joint roll-call: wiggle each YAM joint one at a time.
- `rpm_stats.py` (86) — motor RPM statistics from teleop session CSVs (motor-viability analysis).
- `float_test.py` (44) — zero-gravity float test: gravity compensation on the bare arm.
- `bench_can_read_rate.py` (53) — measure CAN state-read round-trip rate before enabling control.

**Note:** `doctor.py`/`cal_doctor.py`, `gui.py`/`gripper_gui.py`, and `pad.py`/`gripper_pad.py` are themselves near-duplicate pairs split only by import style. Each pair should be folded into one canonical tool alongside the consolidated shared modules.

---

## Summary of canonical bases

| module | canonical base | merges needed |
|---|---|---|
| canbus.py | `YAM Test/canbus.py` (160) | none — discard the reset-on-open copies |
| dm4310.py | any 204-line copy | none — discard `dm-j4310-test` (199) |
| calstore.py | `fastgripper-openarm` (79) | + atomic write from `URtest` (74) L41-46 |
| calibrate.py | `fastgripper-openarm` (216) | none (import style only) |
| autocal.py | `URtest/fastgripper_ur` (461) | + `default_cal_path()` from `fastgripper-openarm` (392); resolve fast-exit + defaults-on conflicts |

No files were modified in producing this audit.
