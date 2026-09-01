# Adversarial review — Plan 2 (i2rt adapter + YAM migration)

Plan: `docs/superpowers/plans/2026-08-31-fastgripper-dm-i2rt-yam.md`
Spec: `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md` (rev 3)
Reviewed against merged v0.1 core (`controller.py`, `calstore.py`, `port.py`,
`profile.py`, `tracker.py`), the live teleop (`YAM Test/so101_teleop.py`),
`YAM Test/teleop_gui.py`, `YAM Test/usb_stress.py`, and the i2rt fork
(`motor_chain_robot.py`, `robots/utils.py`).

Verdict: **NOT sound as written.** 2 blockers, 4 majors, 3 minors. The
adapter code in Task 1 is largely correct against the real API, but Task 2's
teleop rewrite has two behaviour-inverting/behaviour-losing defects, the
adapter omits a safety check the plan itself promises, and one of the three
required adapter tests is vacuous.

---

## BLOCKERS

### B1. Trigger fraction is inverted — `grip.goto(trigger_frac)` opens when the operator squeezes
- **Plan location:** Task 2, Step 1, lines 318–325 (`grip.goto(trigger_frac)`, "same trigger math as today").
- **Evidence:**
  - Live teleop absolute mapping: `frac = (s_now − released)/(squeezed − released)`; `grip_goal = clip(cal["open"] + frac*stroke, …)`, `stroke = closed − open` (so101_teleop.py:379–382, 376). So today **frac = 0 → open**, **frac = 1 → closed** (comment confirms "released=open, squeezed=closed", so101_teleop.py:170).
  - Controller: `goto_frac(f): goto_rad(closed + f*(open − closed))` (controller.py:36–38) → **f = 0 → closed**, **f = 1 → open** (spec §3.2 "0 = closed mark, 1 = open mark").
  - The two conventions are exact opposites. Solving `open + frac·(closed−open) = closed + f'·(open−closed)` gives **f' = 1 − frac**.
- **Impact:** Passing the today-math `frac` straight into `grip.goto()` inverts the jaw: releasing the trigger commands *closed*, squeezing commands *open*. This is a live-hardware safety/usability defect, not cosmetic.
- **Fix:** In Task 2 pass `grip.goto(1.0 - frac)` (or recompute frac in the controller's closed=0/open=1 convention), and state the convention explicitly in the plan so the implementer can't reintroduce it. Add a note that `I2rtGripper` exposes only `goto(frac)` (no `goto_rad`), so the teleop must convert to a fraction rather than pass an absolute rad goal.

### B2. Controller stall-hold can never fire — per-tick `goto()` resets the stall timer, and the `stall_clip` grip-and-hold is dropped with no replacement
- **Plan location:** Task 2, Step 1, lines 318–325; claim at line 325 "Stall-hold/rebase semantics come from the controller."
- **Evidence:**
  - The loop calls `grip.goto(trigger_frac)` **every cycle** (plan line 322), which routes through `goto_frac → goto_rad`, and `goto_rad` unconditionally sets `self.stalled = False; self._stall_t = 0.0` on every call (controller.py:33–34).
  - Controller stall detection requires `_stall_t` to accumulate past `stall_time_s` (0.4 s) across consecutive ticks (controller.py:89–90). Because `goto()` zeroes `_stall_t` immediately before every `tick()`, `_stall_t` never exceeds one `dt` (0.02 s). **The stall latch is unreachable.**
  - With no latch, a grasp holds by the P-loop: `v_cmd = sw_kp·(goal − pos)` saturates at `vmax` while the jaw is blocked, so `torque = kd·(vmax − v_actual) ≈ kd·vmax = TMAX` is delivered **continuously** (controller.py:76–86). The live code instead forced `v = 0` on stall and held via the self-locking worm at ~0 torque (so101_teleop.py:426–438), and additionally clamped the *goal* to the stall point until the trigger retreated (`stall_clip`, so101_teleop.py:389–395, 429–430). Both behaviours are lost.
- **Impact:** (1) Sustained TMAX (2.0 Nm) into a grasped object/worm instead of backing off — the running-the-yam gotchas record that ">2 Nm transients snapped the v1 worm" and this is *sustained* 2 Nm. (2) Task 4's drill pass criterion "stall-hold on grasp" (plan line 340) will not be met. (3) The plan's line-325 claim that stall-hold comes from the controller is false under this call pattern.
- **Fix (pick one, state it in the plan):** either (a) only call `grip.goto()` when the target changes, so the stall timer can accumulate while the operator holds; or (b) re-introduce a teleop-side `stall_clip`/trigger-backoff equivalent; or (c) change the driving contract so the controller's stall state is not reset by an idempotent re-goto. Option (a) alone still resets on any trigger jitter, so (b) or (c) is safer. Whichever is chosen, add a test that a blocked jaw held past `stall_time_s` produces the hold command.

---

## MAJORS

### M1. Adapter omits the connect-time mapping round-trip check the plan's Global Constraints promise
- **Plan location:** Global Constraints line 21 ("the adapter asserts the mapping is exact by checking a round-trip at connect (see Task 1)") vs. the actual `connect()` code, Task 1 Step 3, lines 250–267.
- **Evidence:** `connect()` checks only the park tolerance; it never verifies that the robot was built with `gripper_limits_override = [−POS_WINDOW, +POS_WINDOW]`. The `×SPAN − POS_WINDOW` read (i2rt.py `_wrapped`, plan line 240) and the `÷SPAN` write (`to_i2rt_command`, plan line 224) are correct **only** because `JointMapper.joint_range == SPAN`: `to_robot_joint_vel_space` multiplies by `joint_range` and `to_command_joint_pos_space` normalises by it (utils.py:502, 511, 520), and `joint_range = limits[:,1] − limits[:,0] = 25 = SPAN` for the override. If a caller builds the robot with the arm's default gripper limits (any other range), every position and velocity is silently mis-scaled with no error.
- **Impact:** The single load-bearing invariant of the whole adapter is unenforced despite the plan claiming it is enforced.
- **Fix:** In `connect()`, read `self.robot.get_robot_info()["gripper_limits"]` (available, motor_chain_robot.py:303) and assert `max − min == SPAN` (and, ideally, the window is symmetric ±POS_WINDOW); raise a clear `ValueError` otherwise. Add a test with a fake robot reporting wrong limits.

### M2. GUI telemetry breaks — `teleop_gui.py` requires `vel=`/`eff=` fields the adapter cannot supply
- **Plan location:** Task 2, Step 1, line 325 ("Telemetry uses `grip.position`, `grip.goal`, `grip.stalled`"); Task 2 copies `teleop_gui.py` (plan line 305/328).
- **Evidence:** The GUI parses stdout with `TLM_RE = re.compile(r"TLM pos=([-\d.]+) goal=([-\d.]+) vel=([-\d.]+) eff=([-\d.]+)")` (teleop_gui.py:26) and skips any line that doesn't match. The live teleop emits exactly that (so101_teleop.py:420–421) using `g_vel`/`g_eff` from `robot.get_observations()`. The `I2rtGripper` surface in Task 1 exposes `position`, `goal`, `stalled` but **no measured velocity or torque accessor** — the plan's telemetry note omits `vel`/`eff` entirely.
- **Impact:** If the implementer builds the TLM line from adapter properties only, the `vel=`/`eff=` tokens vanish, the regex fails to match, and the GUI shows no telemetry (or the teleop must reach past the adapter into `robot.get_observations()` and duplicate the read the adapter already performs internally).
- **Fix:** Add `velocity`/`torque` (or a `feedback`) property to `I2rtGripper` returning the last `_feedback()` values, and require Task 2 to keep the exact `TLM pos=… goal=… vel=… eff=…` line. Note `_feedback()` is currently recomputed inside `tick()`; expose the cached last reading rather than double-reading the robot.

### M3. `test_feedback_scaling_round_trip` is vacuous — it does not verify the velocity `×SPAN` scaling (spec §5a)
- **Plan location:** Task 1, Step 1, lines 138–146.
- **Evidence:** Hand-trace with `ENTRY`/defaults: after connect the goal is held at park (pos 1.0), so `v = sw_kp·(1.0 − 1.0) = 0`; the velocity clamp `min(fb.vel+dv, max(fb.vel−dv, v))` with `dv = vmax = 24` leaves `cmd.vel = 0` regardless of `fb.velocity` (controller.py:76–86). The assertion `abs(cmd.vel − 5.0) <= vmax + 1e-9` → `5.0 <= 24` passes. Critically it **also passes if the adapter drops the `×SPAN`** (then `fb.velocity = 0.2`, `cmd.vel` still 0): the test cannot distinguish correct from missing scaling. Spec §5(a) explicitly requires "a known normalised … `gripper_vel` round-trips to the expected real-unit Feedback (… `× SPAN`)"; this test does not exercise it.
- **Impact:** The one test that is supposed to guard the velocity read-scaling proves nothing about it. The `torque` (`gripper_eff`) pass-through is likewise unasserted.
- **Fix:** Assert on `g._feedback()` directly: `velocity == pytest.approx(0.2*SPAN)` (== 5.0) and `torque == pytest.approx(0.7)`; keep the park-adoption `position == 1.0` check. (Position-scaling `×SPAN − POS_WINDOW` is exercised indirectly by the park check but a direct `_feedback().position` assertion is cheap.)

### M4. Ctrl-C park save is not guaranteed by the plan
- **Plan location:** Task 2, Step 1, line 325 ("On exit: `grip.park()` replaces the manual cal-file write").
- **Evidence:** The live teleop saves the park inside `finally:` so it runs on both `KeyboardInterrupt` and exceptions (so101_teleop.py:458–468), and Task 4's drill criterion is "Ctrl-C saves park" (plan line 340). The plan says only "on exit" and does not require preserving the `try/except KeyboardInterrupt/finally` structure (nor the `sys.excepthook`/`os._exit` teardown that must run *after* the finally).
- **Impact:** An implementer who puts `grip.park()` on the normal-return path loses park-save on Ctrl-C, failing the drill and the bench's core "second session auto-restores" property.
- **Fix:** State explicitly: keep the `try/except KeyboardInterrupt/finally` shape; call `grip.park()` in `finally` (guarded by `grip` being connected); preserve `robot.close()`, log flush, and the `os._exit(0)` teardown.

---

## MINORS

### m1. De-serialised `usb_stress.py` still has flat imports that will fail at runtime
- **Plan location:** Task 2, Step 1, line 326 (usb_stress port-map change), Step 2 line 329 (`py_compile` check).
- **Evidence:** `soak()` does `from canbus import open_bus` plus `import can`, `import scservo_sdk` (usb_stress.py, inside `soak`). In `bench/tools/`, `from canbus import open_bus` no longer resolves (canbus now lives at `fastgripper_dm.damiao.canbus`, spec §3). `python -m py_compile` (the plan's only check) compiles without importing, so it passes while `usb_stress.py soak` breaks on first run.
- **Fix:** Have Task 2 rewrite the import to `from fastgripper_dm.damiao.canbus import open_bus` (and confirm `can`/`scservo_sdk` are in the bench venv). The port-map `tomllib` change is otherwise fine and preserves the three named ports + VID/PID `0x1D50/0x606F` + soak/watch structure.

### m2. Spec §3.5 facade example contradicts the real API (out of plan scope, but fix the spec)
- **Evidence:** Spec §3.5 shows `robot.command_joint_state(pos=pos, vel=vel, kp=kp, kd=kd)` (kwargs) and `vel[6], pos[6], kp[6], kd[6] = to_i2rt_command(cmd)` (single-arg, vel-first). The real signature is `command_joint_state(joint_state: Dict[...])` taking one dict with `"pos"/"vel"/"kp"/"kd"` (motor_chain_robot.py:545–560; live call so101_teleop.py:454), and the plan's `to_i2rt_command(cmd, pos_placeholder_norm) → (pos, vel, kp, kd)` takes two args, pos-first. The **plan** is internally consistent (Task 1 def matches Task 2 unpack order, plan lines 224 / 322–323); only the spec snippet is wrong.
- **Fix:** Correct spec §3.5 to the dict call and the two-arg `to_i2rt_command`. No plan change required.

### m3. `park_tolerance_rad = 0.35` is provisional/unmeasured yet hard-gates bench startup with no soft fallback
- **Plan location:** Task 1 `connect()` lines 250–260; profile default profile.py:29; spec §10 open item.
- **Evidence:** On the shared chain the adapter has no stall-home, so a boot reading >0.35 rad from `last_wrapped` makes `connect()` raise, and the only recovery is manual `autocal home` on a dedicated channel (plan lines 256–259). The live teleop never refused — it adopted `last_position` unconditionally (so101_teleop.py:340–341). The 0.35 rad value is explicitly "PROVISIONAL … unmeasured" (spec §3.2, §10).
- **Impact:** Risk that Task 4's "second session auto-restores with no homing motion" fails on first real power-cycle if drift exceeds the guessed tolerance, blocking the bench where it previously worked.
- **Fix:** Sequence the drill to *measure* power-cycle drift before trusting the guard (spec already calls for this), and/or give the bench teleop a documented `--force-adopt`/`home="off"` escape hatch (the adapter already supports `home="off"`, plan line 261) so a lost park doesn't hard-stop a session.

---

## Checked and sound (no action)
- Task 1 adapter API names all resolve: `ctrl.profile` (controller.py:21), `ctrl.profile.park_tolerance_rad` (profile.py:29), `tracker.seen` (tracker.py:22), `adopt_park`/`park_fields` (controller.py:60,66), `goto_frac/open/close/hold/position/goal/stalled` (controller.py), and the `calstore` imports `default_cal_path/entry_profile/get_entry/load_store/save_store` (calstore.py). `GripperController(entry, profile)` matches the two-arg ctor.
- `get_entry` returns the live entry dict, so `self.entry.update(...)` + `save_store(self._store)` in `park()` persists correctly via aliasing (calstore.py:69–72).
- Park-mismatch wrap formula is correct: `min(|d|%SPAN, SPAN−|d|%SPAN)` gives 1.0 for boot=−12/lw=+12 and 0 for the matched case; `test_connect_refuses_park_mismatch` (boot 4 vs lw 1 → 3 > 0.35) raises with a message containing "park" matching the regex.
- `to_i2rt_command` return order `(pos, vel, kp, kd)` matches Task 2's unpack `target[GRIP], vel[GRIP], kp[GRIP], kd[GRIP]`; `kd` value `tmax/vmax = 2.0/24 = 0.0833` equals the live `GRIPPER_KD`.
- Velocity scaling direction is right: JointMapper multiplies command vel by `joint_range` and divides feedback vel by it (utils.py:497–511); with the ±POS_WINDOW override `joint_range == SPAN`, matching the live `vel/SPAN` and `g_vel*SPAN` (so101_teleop.py:408, 442). (The missing *enforcement* of that invariant is M1.)
- numpy is tests-only: the adapter module imports `time`, `calstore`, `controller`, `port` — no numpy; `float(arr[idx])` needs none. Plan's "add numpy to the dev extra" is correct (plan line 299).
- Test count: suite currently collects **73** (verified via `pytest --collect-only`); +6 adapter tests = 79, matching the plan.
- `get_observations` exposes `gripper_vel`/`gripper_eff` when a gripper index is set (motor_chain_robot.py:592–593), matching the adapter's reads and the fake robot's keys.
- Task 0: `gh repo fork i2rt-robotics/i2rt --org Transcend-Mechanics --clone=false`, `git remote add … && git push transcend fastgripper`, and the PEP 508 `"i2rt @ git+https://…@fastgripper"` install URL are all valid.
