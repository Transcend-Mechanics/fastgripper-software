# Changelog

## 0.1.1 — 2026-09-02

Ports the hot-patches found and validated on real hardware (DM-J4310 worm gripper,
gs_usb on macOS) after 0.1.0 shipped. No API, CLI or convention changes.

- **Shut the CAN bus down on every teardown.** `FastGripper.disconnect()`,
  `_safe_disable_close()` and the `status` verb now call `bus.shutdown()` after
  closing the port. Left to GC at interpreter exit, libusb aborts on macOS and
  the adapter is wedged ("bus opened but passes NO frames") for the next session.
- **`tools._cli.run` catches `HomingError`** like `PortError`: message to stderr,
  exit 1 via `os._exit`. An escaping `HomingError` reached the crash path above.
- **Re-home window alias fixed.** `home_against_stop()` folds the re-anchor offset
  into the nearest ±12.5 rad window before the "> 3 turns → friction" check, and
  anchors on `stop_closed` itself. Every re-home used to abort with
  "re-anchor offset ±25.0 rad exceeds ~3 turns" with the jaws physically at the stop.
- **Drift-corrected park adoption.** `connect(home="auto")` adopts
  `last_position + drift` (drift folded into one window) instead of the stale saved
  value. The mechanism relaxes 1.7–1.8 rad after a hard close and disable; adopting
  the stale value put the closed goal past the physical stop and pushed at the
  torque cap until timeout.
- **Profile defaults re-measured** (2026-09-02): `park_tolerance_rad` 0.35 → 3.0,
  `contact_torque` 0.30 → 0.8, `probe_tmax` 0.5 → 1.0, `probe_vel` 0.8 → 2.0. The
  old probe could produce at most 0.48 Nm, below this unit's free-run friction p95
  of 0.36–0.44 Nm, so it declared contact at the start position. `validate()` now
  requires `contact_torque < probe_tmax <= TMAX_CAP`.
- **Park guard on the facade.** `FastGripper` tracks whether the session anchored
  the tracker; `park()` warns and saves nothing when it did not (`home="off"`).
  Mirrors the guard already in `adapters/i2rt.py`.

## 0.1.0

Controller/port refactor: `FastGripper` facade, `GripperController`, `MotorPort`,
profiles, the torque-driven stall contract, and the i2rt/YAM adapter.
