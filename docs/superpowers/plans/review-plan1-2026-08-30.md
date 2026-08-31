# Adversarial review — fastgripper-dm v0.1.0 standalone implementation plan

Plan reviewed: `docs/superpowers/plans/2026-08-30-fastgripper-dm-standalone.md`
Spec: `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md` (rev 3)
Repo state verified against: `packages/fastgripper-dm/**` (v0.0.1 harvest already committed)
Reviewer: senior adversarial review, claims traced to code. Date 2026-08-30.

Severity counts: **3 blockers, 6 major, 8 minor.**

Line numbers "plan Lnnn" refer to the plan file. Repo citations are `path:line`.

---

## BLOCKERS

### B1 — Task 5 `test_moves_and_wraps` cannot construct the sim (AssertionError), and its logic contradicts the default stops
- Plan L756: `sim = SimulatedWormGripper(start=12.0)` overrides only `start`.
- Plan L828 (`__init__`): `assert stop_open < start < stop_closed`. Defaults are `stop_open=-31.0, stop_closed=+3.0` (plan L825). So the assertion is `-31.0 < 12.0 < 3.0` → `12.0 < 3.0` is **False → AssertionError at construction**. The test dies before `enable()`.
- Even if it constructed: the test drives `MitCommand(vel=5.0)` (positive) and asserts `sim.true_position > 12.5` (plan L761). In this sim positive motion is the *closed* direction (controller marks: open −30, closed +2; sim `make()` uses `stop_closed=3.0`, plan L956). The closed stop at `+3.0` clamps travel at `stop_closed+0.35 = 3.35` (plan L885), so `true_position` can never reach 12.5. The test is inconsistent with the sim's own stop model in two independent ways.
- **Fix:** give the test room in the travel direction it actually exercises. To cross the +12.5 window edge the closed stop must be beyond it, e.g. `SimulatedWormGripper(start=12.0, stop_closed=40.0)`; or, better, exercise wrapping the way the hardware does — drive negative from `start=+1.0` toward the open stop and assert `sim.true_position < -12.5` and that `fb.position` stays wrapped. Pick one and make the assertion match the driven direction.

### B2 — Task 5 `test_torque_is_kd_times_velocity_error` cannot pass; the sim's velocity model is not self-consistent
- Plan L776-780:
  ```python
  sim = SimulatedWormGripper(start=0.0, friction=0.0); sim.enable()
  fb = sim.command(MitCommand(vel=2.0, kd=0.5))
  assert fb.torque == pytest.approx(0.5 * (2.0 - fb.velocity), abs=1e-6)
  ```
- Trace of `command()` (plan L866-896) with `inertia=0.01`, `tau_response=0.05`, `dt=0.02`, starting `velocity=0`:
  1. `tau = cmd.kd*(cmd.vel - self.velocity) + … = 0.5*(2.0 - 0.0) = 1.0` — computed against the **pre-update** velocity (0). `self._torque = tau = 1.0`, so `fb.torque = 1.0`.
  2. Update #1 (torque/inertia): `velocity += (tau_net/inertia)*dt = (1.0/0.01)*0.02 = 2.0` → `velocity = 2.0`.
  3. Update #2 (first-order lag, plan L882): `alpha = min(1, 0.02/0.05) = 0.4`; `velocity += 0.4*(2.0 - 2.0) = 0` → `velocity = 2.0`.
  4. `fb.velocity = 2.0`.
- Assertion evaluates `1.0 == approx(0.5*(2.0 - 2.0)) = 0.0` → **False. Test fails.** Torque is computed against the velocity *before* the step; the feedback reports the velocity *after* the step; the two updates make them diverge, so `torque == kd*(vel_cmd − fb.velocity)` cannot hold.
- This is the core "not self-consistent" defect the review brief flagged. It is compounded because a naive fix collides with Task 6: the Task 6 cap assertion `sim.max_abs_torque <= 2.0` (plan L976) holds **only because** the sim computes torque against the previous-frame velocity that the controller clamped against (`v_cmd ∈ v_prev ± dv`, plan L1115). If you switch the sim to report torque against the post-update velocity to satisfy B2, `|v_cmd − v_new|` can exceed `dv` and the cap assertion in Task 6 can break. So the two tests pull the model in opposite directions as written.
- **Fix:** commit to one coherent single-velocity model. Recommended: evolve velocity **only** via `tau_net/inertia` (delete the update-#2 first-order-lag block, plan L881-882 — it double-drives velocity and has no physical basis alongside the inertia integrator), and define the reported torque and reported velocity to be the same pair the controller will see next tick. Concretely: integrate velocity from the *previous* step's torque, then set `self._torque = cmd.kd*(cmd.vel - self.velocity) + cmd.tau + cmd.kp*(cmd.pos - self._wrapped())` using the just-updated `self.velocity`, and return that. Then re-derive the Task 6 cap guarantee: the controller must clamp `v_cmd` against the **same** velocity the sim reports (it does — `fb.velocity` is last frame's post-update value), so `|v_cmd − v| ≤ dv` still bounds `|tau| ≤ TMAX`. Re-run both `test_torque_is_kd_times_velocity_error` and `test_reaches_a_goal_without_exceeding_the_torque_cap` together and prove both hold before calling Task 5/6 done.

### B3 — Task 8 sets `__version__ = "0.1.0.dev0"`, which breaks the existing `test_harvest.test_version` (asserts `"0.0.1"`); Task 8 claims "full suite still green"
- Repo: `src/fastgripper_dm/__init__.py:8` → `__version__ = "0.0.1"`; `pyproject.toml` version `0.0.1`; `tests/test_harvest.py:18` → `assert fastgripper_dm.__version__ == "0.0.1"`.
- Plan L1654-1662 (Task 8, "`__init__.py` becomes …") sets `__version__ = "0.1.0.dev0"`, and plan L1664 asserts "full suite still green." That is **false**: `test_harvest.test_version` fails the moment `__init__` changes.
- The `0.1.0.dev0` premise threads through the plan (Global Constraints implicitly, Task 1 L232/L195) but **never matched the harvested repo**, which shipped `0.0.1`. Task 1 is marked superseded, so its `0.1.0.dev0` is dead text; but Task 8 actively rewrites `__init__` to it.
- Task 12 (plan L1880) then bumps `pyproject` + `__init__` to `0.1.0` — and **also** never updates `test_harvest.test_version`, so the version test breaks again at release regardless.
- **Fix:** Task 8's `__init__` must keep `__version__ = "0.0.1"` (only re-export `FastGripper`/`HomingError`/profile symbols; do not touch the version). Do the single version bump in Task 12, and add an explicit step there to update `tests/test_harvest.py:18` to the released string (and keep `pyproject.toml` in lockstep). State in Task 8 that the version is intentionally left at `0.0.1`.

---

## MAJOR

### M1 — `BusDead` is contradictory: the shipped `canbus.BusDead` is a `SystemExit`, but the plan asserts it is a `PortError` and never `SystemExit`; the facade leaks it into library callers
- Repo `damiao/canbus.py:13`: `class BusDead(SystemExit)`. `open_bus` raises it on a dead/wedged adapter (`canbus.py:143`, `:158`).
- Plan Global Constraint L28: "`canbus.open_bus` … a dead gs_usb bus raises `BusDead` (a `PortError`), **never** `SystemExit`." Plan L430 (Task 2 `port.py`) also defines a **second** `class BusDead(PortError)`. So two incompatible `BusDead` classes exist and the constraint is false against the actual `canbus.py`.
- Consequence: `FastGripper.connect()` (plan L1511) calls `bus = open_bus(...)` with **no** try/except. A dead bus raises the `SystemExit`-based `canbus.BusDead`, which propagates out of a library call — exactly the behavior the spec/§3.1 and Global Constraint L28 say must not happen for library callers. (Preflight relies on the SystemExit flavor: `preflight.py:105` catches `except SystemExit` to build `bus_error`.)
- **Fix:** decide one contract and make the plan enforce it. Either (a) change `canbus.BusDead` to subclass `PortError` and update `preflight.run_preflight` to catch `PortError`/`BusDead` instead of `SystemExit` (add this as an explicit step in the plan, since Task 2's `port.BusDead(PortError)` currently just shadows it), or (b) have `FastGripper.connect()` wrap `open_bus` and translate `canbus.BusDead → port.BusDead`. Whichever you pick, delete the duplicate class or make `port.BusDead` the one `canbus` raises, and correct Global Constraint L28.

### M2 — Global Constraints/Task 1 dependency list is stale vs the committed `pyproject.toml`
- Plan Global Constraint L20: "runtime deps exactly `python-can>=4.0` and `pyusb>=1.2`." Task 1 pyproject snippet L209: `dependencies = ["python-can>=4.0", "pyusb>=1.2"]`.
- Repo `pyproject.toml`: `dependencies = ["python-can>=4.0,<4.6", "pyusb>=1.2", "gs_usb>=0.3.0"]`. Git history: "pin python-can <4.6 (4.6.1 wedged the adapter)" and "gs_usb is a real dependency."
- The `<4.6` cap and `gs_usb` are load-bearing (bench-proven). The plan's "exactly" wording is wrong and, if any later task regenerates metadata from the plan text, would silently drop both.
- **Fix:** update Global Constraint L20 and the Task 1 snippet to `python-can>=4.0,<4.6`, `pyusb>=1.2`, `gs_usb>=0.3.0`, and drop the word "exactly."

### M3 — Profile field is named `watchdog`, contradicting the plan's own Global Constraints, spec §3.3/§3.4, and the harvested default; plus an unreconciled 500-vs-8000 default
- Task 4 `GripperProfile` uses `watchdog: int = 8000` (plan L675). But Global Constraint L22 lists the field as **`watchdog_ms`**, spec §3.3 lists `watchdog_ms`, and spec §3.4 (L286) says preflight verifies "watchdog register == `profile.watchdog_ms`." A field named `watchdog` makes any `profile.watchdog_ms` access an `AttributeError`.
- Default value conflict: Task 4 default `8000` matches the repo (`config_tool.py:28 DEFAULT_WATCHDOG_MS = 8000`, `cli.py:138 --watchdog_ms default 8000`) — but spec §3.6 (L337) and Decisions §10 (L457) still say "Default **500 ms**", and `preflight.run_preflight` falls back to `cfg.get("watchdog_ms", 500)` (`preflight.py:92`). `500` faulted continuously at 50 Hz per the git log, so `500` is known-wrong.
- **Fix:** pick one field name — recommend `watchdog_ms` to match Global Constraints/spec — and use it consistently in Task 4, Task 8/10, and any preflight change. Update spec §3.6/§10 and the `preflight.py:92` fallback default from `500` to `8000`. Note the unit is still uncertain (Task 11 experiment) — that's fine, but the *plan* should not carry two different numbers.

### M4 — Task 9 silently inverts the `drive`/`goto` percentage convention vs the shipped `drive` tool, and doesn't reconcile the pre-existing docs contradiction
- New behavior (plan L1709-1718, and spec §3.2 "0 = closed, 1 = open"): `frac = pct/100`; `goto_frac(frac) = closed + frac*(open − closed)` (plan L1073). So `goto 0 → closed`, `goto 100 → open`.
- Harvested `tools/drive.py`: `goal = open + (pct/100)*(closed − open)` (`drive.py:56`), docstring "0=open, 100=closed" (`drive.py:9`, `:39`). So the shipped `drive 0 → open`, `drive 100 → closed` — the **opposite** of the new `goto`/`drive`.
- Meanwhile `docs/fastgripper-dm/quickstart-linux.md:55-56` documents `drive 0 # closed` / `drive 100 # open` — which matches the NEW convention and **contradicts the shipped `drive.py` code**. So v0.0.1 already ships a tool whose behavior is inverted from its own quickstart; the plan neither notices nor calls it out.
- Net: Task 9 flips `drive`'s runtime numbers (0 and 100 swap meaning). It happens to align with the quickstart and spec, but the change is a breaking behavior change for anyone who used v0.0.1 `drive` per its code/docstring.
- **Fix:** in Task 9, explicitly state that `drive`/`goto` adopt the spec §3.2 convention (0=closed, 100=open), that this inverts the harvested `drive.py`, and add a Step-4 verification that `quickstart-linux.md:55-56` (already correct) and any other doc agree. Note that the old `drive.py` docstring is removed with the body.

### M5 — Spec §3.4 preflight check "watchdog register == `profile.watchdog_ms`" is not implemented and Task 10 does not add it
- Spec §3.4 (L286): preflight must verify the motor's watchdog register equals `profile.watchdog_ms`. Current `preflight.run_preflight` compares against `cfg.get("watchdog_ms", 500)` (`preflight.py:92`) — a **config** value, not the profile — and Task 10 (plan L1771-1812) only adds the tmax cap and entry↔motor-id checks. The profile-watchdog binding is nowhere.
- **Fix:** either add to Task 10 a step that passes `entry_profile(entry).watchdog_ms` as `watchdog_want` into `evaluate` (superseding the config fallback), or explicitly defer §3.4's profile-watchdog check with a spec note. Don't leave it silently unmet while the self-review (plan L1889) claims §3.6 is covered.

### M6 — Spec §10 mandates measuring `park_tolerance_rad` drift before `dm-v0.1.0`; Task 11 omits the measurement, and Task 12 releases anyway
- Spec §10 (L462-464): "`park_tolerance_rad` — provisional 0.35 rad; **measure drift over power cycles in the damage-control drill and set above max observed, before `dm-v0.1.0`**." Profile default carries the caveat (plan L674: "PROVISIONAL … spec §10").
- Task 11 (plan L1816-1828) has the **watchdog-unit** experiment and a one-shot park/restore functional check, but **no** N-cycle shaft-drift measurement to set `park_tolerance_rad`. Task 12 then releases `dm-v0.1.0` (plan L1832) with the tolerance still at the unmeasured provisional 0.35.
- **Fix:** add a Task 11 step: power-cycle the parked gripper N times (≥10), record wrapped-boot vs `last_wrapped` drift each cycle, and set `GripperProfile.park_tolerance_rad` above the max observed before Task 12. Make Task 12 depend on it.

---

## MINOR

### m1 — New CLI verbs drop bus/`--cal`/id overrides the harvested `drive` exposed
Task 9's `open/close/goto/home/status` add only `--gripper` (plan L1752-1762) and route through `FastGripper.standalone(gripper=…)` (plan L1715), which reads `config.json`. The harvested `drive.py` accepted `--interface/--channel/--cal/--motor_id/--master_id` via `add_bus_args` (`drive.py:38`, `:40`). The documented flow is setup-first, so `quickstart-linux.md` is safe, but `--cal PATH` (a spec §3.3 location-precedence requirement) is now unreachable from these verbs. Consider threading `--cal` through `cmd_motion/cmd_status` into `FastGripper.standalone(cal_path=…)`, or note the deferral.

### m2 — The "retired" rounding tracker still ships and is load-bearing in v0.1
Task 3's `tracker.py` docstring says the `dm4310.py` window-rounding tracker "is retired" (plan L523-524), and spec §3.2 (L246-247) says "autocal uses the controller's tracker." But per the deliberate scope note (plan L5), autocal/calibrate/drive stay on the direct-DM4310 path: `tools/autocal.py:37` still `from ..damiao.dm4310 import DM4310, MultiTurnTracker` and uses it at `:62/:291/:321`; `tools/drive.py:22` likewise. So `dm4310.MultiTurnTracker` (rounding, `dm4310.py:86-130`) remains in the shipped package. The "retired" wording overstates; soften the Task 3 comment to "retired on the controller path; still used by autocal/drive until Plan 2."

### m3 — Task 5 hard-stop test relies on float equality that only works by luck of `min()` returning the literal
Plan L885-888: `new_pos = min(new_pos, self.stop_closed + 0.35)` then `if new_pos in (self.stop_closed + 0.35, self.stop_open - 0.35): self.velocity = 0.0`. This works because `min` returns the exact `stop_closed+0.35` operand, which equals the recomputed literal in the `in` check. It functions, but it is fragile (any refactor that stores the clamp bound in a differently-rounded variable silently breaks the stall). Prefer an explicit flag: set `hit_stop = new_pos >= self.stop_closed + 0.35 or new_pos <= self.stop_open - 0.35` at clamp time and zero velocity on that.

### m4 — `FAULT_CODES` and `Feedback` duplicated between `dm4310.py` and Task 2 `port.py`
`dm4310.py:24-34` defines `FAULT_CODES` and `dm4310.py:46` a `Feedback`; Task 2 `port.py` (plan L383-414) defines its own `FAULT_CODES` and a different `Feedback`. The field sets differ (dm4310: `motor_id/error/temp_*`; port: `error_code/t`), so the duplication is defensible, but the two `FAULT_CODES` dicts can drift. Consider having `port.py` import `FAULT_CODES` from `dm4310` (or vice versa) so there is one table. Acceptable to leave as-is if noted.

### m5 — Task 1's pyproject/version snippet no longer matches the harvested repo
Beyond M2's deps: Task 1 pyproject (plan L204) shows `version = "0.1.0.dev0"` and dev dep `pytest>=8`; the repo shows `version = "0.0.1"`. Task 1 is superseded, so this is documentation drift only, but it feeds the B3 version confusion. Add a one-line note that the live `pyproject.toml` (0.0.1, pinned can, gs_usb) is authoritative.

### m6 — `home_against_stop` builds a throwaway `GripperController` only to steal its tracker
Plan L1550-1551: `probe = GripperController({"open": -1e9, "closed": 1e9}, p)` then immediately `probe.tracker = MultiTurnTracker()`. The controller object is never otherwise used. Harmless (the ctor accepts those marks — validate() only caps tmax and checks `close_dir`/positivity, plan L678-684), but wasteful and misleading. Use a bare `MultiTurnTracker()` local instead.

### m7 — CI/release use `astral-sh/setup-uv@v3` while the proven lerobot workflow uses `@v4`
Task 1 `ci.yml` (plan L298) and Task 12 `release-dm.yml` (plan L1852) pin `setup-uv@v3`; `fastgripper-lerobot/.github/workflows/release.yml` uses `@v4`. Not a bug, but align to the proven `@v4` for consistency.

### m8 — Status verb enables the motor
Task 9 `cmd_status` uses `home="off"`, but `connect()` still calls `self.port.enable()` (plan L1516) before reading, then `cmd_status` disables at the end (plan L1745). "Never moves" holds (zero-gain), but the motor is briefly energized during a `status`. Fine to keep; note it in the help text if silence-on-status matters.

---

## Checked and sound

- **Task 7 `DamiaoCanPort` against the real `DM4310` API.** `dm4310.py` exports `float_to_uint`/`uint_to_float`/`decode_feedback` and a `DM4310` with `enable/disable/clear_error` (`dm4310.py:151`, added), `mit_control` (`:160`), and `read_feedback(timeout=…) -> Feedback|None` returning `.position/.velocity/.torque/.error` (`:191-209`). Task 7's `_txrx`/`_recover`/`command`/`read`/`close` (plan L1275-1332) use exactly these; the `FakeMotorBus` tests trace correctly (MIT frame last byte `0xFF` is not mistaken for a special `0xFB/0xFC/0xFD`; `test_read_is_zero_gain`'s kd-bit extraction computes 0).
- **Task 6 controller math and cap.** `kd = tmax/vmax`, `dv = tmax/kd = vmax`, per-tick clamp `v ∈ fb.velocity ± dv` (plan L1113-1115) is identical to `so101_teleop.py:408-411` (`dv = GRIPPER_TMAX/GRIPPER_KD`, `v = clip(v, g_vel-dv, g_vel+dv)`), and stall on measured torque `|tau| > 0.75*tmax` for `0.4 s` matches `so101_teleop.py`. The cap clamp genuinely binds on fast opposite-sign impacts, so `max_abs_torque <= 2.0` is a real constraint, not vacuous. `test_frac_mapping`, `test_goal_is_clamped_to_marks`, `test_tick_is_pure…` (float determinism holds — same code path, same ops), and `test_profile_validation_at_construction` all trace to pass.
- **No circular import between `cli` and `facade`.** `facade.standalone`/`connect` import `cli._load_config` lazily inside function bodies (plan L1492, L1507); `cli.cmd_motion/home/status` import `facade` lazily inside function bodies (plan L1707 etc.). Neither imports the other at module top, so import order is safe.
- **`open_bus` returns a `can.BusABC` directly** (`canbus.py:18`), not a context manager, so the facade's `bus = open_bus(...)` without `with` (plan L1511) works. (It never shuts the bus down — a minor leak papered over by `os._exit`; acceptable.)
- **Task 12 `release-dm.yml` paths are correct.** `defaults.run.working-directory: packages/fastgripper-dm` applies only to `run:` steps, so `uv build` (a `run:` step) produces `packages/fastgripper-dm/dist/`, while `upload-artifact` (a `uses:` step) resolves its `path: packages/fastgripper-dm/dist/` from the repo root — they match. Consistent with the proven `fastgripper-lerobot/.github/workflows/release.yml` (which has no subdir, so uses `dist/`). The `workflow_dispatch → testpypi`, tag-gated `pypi`, and OIDC `id-token: write` structure mirror the proven workflow.
- **Task 10 `evaluate` signature growth is backward-compatible.** New params (`entry_motor_id`, `answered_id`, `profile_tmax`) added with defaults after the existing `bus_error=None` (`preflight.py:25`) preserve every positional call in `test_harvest.py:127/132/137/139/144`; the two new Task 10 tests pass their new args by keyword.
- **Tasks 0-1 artifacts exist as claimed.** `patches/` holds `README.md`, `gs_usb-darwin.patch`, `ruckig-build.md`, `setup-mac.sh`; `.github/workflows/ci.yml` present; `Makefile` has `dm-sync`/`dm-test`/`gate`; `damiao/config_tool.py` exports `RegisterClient`/`set_motor_id`/`read_watchdog_ms`/`set_watchdog_ms`/`probe`/`probe_ids`. `port.py`/`tracker.py`/`profile.py`/`sim.py`/`controller.py`/`facade.py` do not yet exist, so Tasks 2-8 do not collide with existing files; Task 4 appending `entry_profile` to `calstore.py` is additive (not currently present).
- **Park-check window aliasing is a known, accepted limitation, not overstated.** `_wrapped_dist` (plan L1469-1471) compares wrapped boot vs `last_wrapped` mod SPAN, so it cannot detect a full-window (25 rad) hand-move — but spec §3.2 accepts this (the worm self-locks and cannot be back-driven a full window while unpowered), and the plan carries the "PROVISIONAL/unmeasured" caveat rather than claiming full safety. Observation only; the M6 drift measurement is the real gap.
