# Final whole-branch review — fastgripper-dm v0.1 standalone (aaf0b82..e921726)

Reviewer: final whole-branch pass. Scope: 12 commits, tasks 2–10 (tasks 11–12 are
hardware/release, not in this branch). Verified against actual code in the working
tree; suite re-run locally = **73 passed** (42.6 s). Claims below cite file:line in
`packages/fastgripper-dm/`.

Verdict: **mergeable after one small safety hardening + one dead-import cleanup.**
No correctness or spec-compliance defects at the cross-task seams. The torque cap,
park/restore, BusDead unification, drive-convention flip, and preflight upgrades all
land as the spec/plan require.

---

## Findings

### Important

**I1. `_safe_disable_close` catches only `PortError`, so a library exception from
`disable()` can leave the motor enabled — the exact invariant spec §4 forbids.**
`src/fastgripper_dm/facade.py:99-107`.
The connect() failure path (facade.py:570-574) exists precisely to guarantee "no
failure path may leave the motor enabled." But `_safe_disable_close` guards each call
with `except PortError`. `DamiaoCanPort.disable()` → `DM4310.disable()` → `_send()` →
`bus.send()` is **not** wrapped and, on a degrading bus, raises `can.CanError`
(a plain `Exception`, not `PortError`). That exception escapes the first `try`, skips
`self.port.close()` entirely (which *is* robust — `can_port.py:427-443` swallows
`Exception`), and masks the original boot/homing error. Result: the enable performed at
facade.py:544 is never paired with a disable.
Reachability is narrow (a bus dead enough for `disable()` to throw would usually have
thrown at the pre-try `enable()` first), but this is the one safety invariant the whole
teardown design turns on, and the fix is one word.
**Fix:** broaden both guards to `except Exception:` (matching what `close()` already
does):
```python
def _safe_disable_close(self) -> None:
    try:
        self.port.disable()
    except Exception:
        pass
    try:
        self.port.close()
    except Exception:
        pass
```

### Minor

**M1. Dead import `Feedback` in the facade.** `src/fastgripper_dm/facade.py:10`
imports `Feedback` from `.port`; it is never referenced (confirmed by AST scan — the
name appears only on the import line). Drop it. Introduced by this branch.

**M2. `cmd_status` divides by `(open − closed)` with no zero-guard.**
`src/fastgripper_dm/cli.py` (`cmd_status`, the `pct = ... / (e["open"] - e["closed"])`
line). A degenerate entry with `open == closed` raises `ZeroDivisionError`. Marks are
guaranteed present (GripperController would have raised otherwise), so this only bites a
malformed cal file. The adjacent `if pos is not None` guard is effectively dead — after
an `home="off"` connect the tracker has always seen a frame, so `position` is never
`None`. Ledger-deferred; safe to leave, but a `(open != closed)` guard is a one-liner.

**M3. `wait()`/`_run_until` raises bare `TimeoutError`, which `_cli.run` does not
catch.** `facade.py:_run_until` raises `TimeoutError`; `tools/_cli.py:run` only handles
`PortError`/`SystemExit`/`KeyboardInterrupt`. So `goto`/`open`/`close` on a gripper that
can't reach its goal exit with a raw traceback rather than a clean message. Safety is
unaffected — the `with` block's `__exit__` → `disconnect()` still disables the motor
during unwind. Consider mapping the timeout to a `PortError`/`SystemExit` in `cmd_motion`
or teaching `run` about `TimeoutError`.

**M4. Homing facade tests run ~10 s each on real 50 Hz sleeps.** `tests/test_facade.py`
(`test_auto_stall_homes_on_mismatch`, `test_homing_guard_rejects_absurd_reanchor`,
`test_failed_connect_leaves_port_disabled`) drive `home_against_stop`/`wait` at wall-clock
`time.sleep(0.02)`; the suite is 42.6 s largely because of these. Ledger-deferred and
fine for merge; add a dt/time-scale hook before the suite grows so CI stays fast.

**M5. Exhausted-recovery → `PortFault` path and backoff timing are untested.**
`src/fastgripper_dm/damiao/can_port.py:399-422` (`_recover` returning `None` →
`command` raising `PortFault`) has no test — matches the brief's given suite. The path is
simple and its safety is covered by facade teardown, so this is acceptable to defer, but
it is the one branch of the port's fault machine with zero coverage.

**M6 (by-design, note only). preflight `answered_id` is always the configured
`motor_id`.** `tools/preflight.py:123` passes `answered_id=motor_id if status is not
None else None`, and `collect()` (preflight.py:88) only accepts frames whose low nibble
matches `motor_id`. So the entry↔motor FAIL (preflight.py:47-49) really compares
`entry["motor_id"]` against the *configured* id, not the id that physically answered; it
cannot detect a different physical motor squatting on the same id. Correct for
single-motor preflight and matches spec §3.4 intent, but the FAIL text "0xNN answered"
slightly overstates what was verified. Leave as-is.

**M7 (pre-existing, out of branch scope). Unused `DEFAULT_WATCHDOG_MS` import.**
`src/fastgripper_dm/cli.py:68` (inside `cmd_setup`) imports `DEFAULT_WATCHDOG_MS` and
never uses it. Not introduced or touched by this branch's diff; flagged only for the
record. Sweep opportunistically.

---

## Must-fix before merge

1. **I1** — broaden `_safe_disable_close` to `except Exception` (facade.py:99-107).
   One-word change; upholds the spec §4 motor-never-left-enabled invariant.
2. **M1** — remove the dead `Feedback` import (facade.py:10). Trivial, keeps the new
   module clean.

Everything else (M2–M7) may ship as deferred minors; none blocks merge.

---

## Verified sound

- **Fault table is single-sourced.** `port.py:723` imports `FAULT_CODES` from
  `damiao/dm4310.py:24-34`; `0x1 → "enabled"`, `0xD → "communication loss"` match the
  `test_port.py` / `test_harvest.py` assertions. No duplicate table, no import cycle
  (dm4310 imports only `can`).
- **`DamiaoCanPort` ↔ `DM4310` contract.** `can_port.py` calls `mit_control`,
  `read_feedback(timeout)→.position/.velocity/.torque/.error`, `enable/disable/
  clear_error`, `can_id` — all present with matching signatures (dm4310.py DM4310 class).
- **Torque cap (the central safety property).** Controller sets `kd = tmax_nm/vmax`
  and clamps `v` to `fb.velocity ± tmax/kd` (`controller.py:328-337`); the sim computes
  torque exactly as the motor (`kd*(v_cmd − reported_v)`, `sim.py:919`), so
  `max_abs_torque ≤ 2.0` is a real assertion and passes in `test_controller.py`. The
  cap-invariant comment was correctly softened to "empirical under sim constants, not
  strict per-tick algebra" (Task 6 fix 0a47b9f) — the code no longer overstates.
- **Park / restore semantics.** Exact adoption via the non-rounding
  `MultiTurnTracker` (`tracker.py:1247-1260`, offset = `park − first_wrapped`, no
  `round(...)`); wrapped-distance check `_wrapped_dist` (facade.py:99 region /
  facade.py:497) is a correct circular metric (`min(d, SPAN−d)`) compared to
  `profile.park_tolerance_rad`. `auto`/`stall`/`assume_closed`/`off` modes all resolve
  and are covered by `test_facade.py`.
- **Disconnect ordering + failed-connect leak.** `disconnect()` does hold → park →
  `port.close()` in try/finally (facade.py:686-698, fix 07e7246); the failed-connect
  path disables+closes before propagating and is asserted by
  `test_failed_connect_leaves_port_disabled`. (I1 hardens the *breadth* of that guard,
  not its presence.)
- **CLI ↔ facade threading.** `open/close/goto/drive/home/status` all thread
  `--gripper` and `--cal` into `FastGripper.standalone(gripper=…, cal_path=…, home=…)`;
  parsers add both flags to every verb (cli.py:197-211). `drive` is a true alias for
  `goto`. The **0 = closed / 100 = open** convention now matches spec §3.2 and
  `docs/fastgripper-dm/quickstart-linux.md:55-56` (docs were already correct; code now
  agrees). `tools/drive.py` deleted with no dangling console script — the only entry
  point is `fastgripper-dm = fastgripper_dm.cli:main`.
- **Version pin.** `__init__.py` re-exports the facade/profile symbols but keeps
  `__version__ = "0.0.1"`; `test_harvest.py:18` asserts `"0.0.1"`. Import smoke test
  clean, `__all__` correct.
- **BusDead unification + preflight.** One `BusDead(PortError)` in `port.py:760`;
  `canbus.py` now raises it; `_cli.run` prints `PortError` and exits 1; `run_preflight`
  catches `(PortError, SystemExit)`. The watchdog-precedence fix `watchdog_want_for`
  (preflight.py:24-29) correctly falls to `cfg`/8000 when the entry carries no `profile`
  block, and is discriminatingly tested (`test_watchdog_want_default_8000`).
- **Suite.** 73 passed locally, matching the controller's report.
