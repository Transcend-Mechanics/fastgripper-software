# Adversarial review — `fastgripper-dm` design spec

**Reviewer lens:** architecture, packaging, safety, edge cases
**Spec:** `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md`
**Date:** 2026-08-30
**Tally:** 2 blockers · 4 majors · 6 minors

---

## Blockers

### B1. `[i2rt]` extra as a git dependency makes the package unpublishable / uninstallable from PyPI
§7 promises `pip install fastgripper-dm[i2rt]` and §8 says the fork is "the `[i2rt]` extra's dependency," consumed from `Transcend-Mechanics/i2rt` on branch `fastgripper`. PyPI **rejects any uploaded distribution whose metadata contains a PEP 508 direct reference** (`i2rt @ git+https://…`) — the `twine`/`gh-action-pypi-publish` upload fails with *"Invalid distribution metadata: … direct reference."* Upstream `i2rt` is not on PyPI, so even if the metadata were accepted, `pip install fastgripper-dm[i2rt]` from a public index could not resolve `i2rt`.

This breaks Goal #2/§7: "YAM from the public docs alone, `pip install`" is not achievable as written.

**Fix (spec text §7/§8):** choose and state one of:
- Publish the fork to PyPI under a distinct name (e.g. `i2rt-fastgripper`) and set `[i2rt] = ["i2rt-fastgripper==<pin>"]`; or
- Drop the extra and document `[i2rt]` as an explicit user-run step: `pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"`, clearly labeled "not on PyPI, installed by hand." An extra may not carry the URL.

Evidence: `fastgripper-lerobot/.github/workflows/release.yml:2` (`tags: ["v*"]`, `pypa/gh-action-pypi-publish@release/v1`); spec §7 line ~267, §8 lines ~281-283.

### B2. Motor can apply torque after the host process dies — no watchdog is provisioned
§3.4/§4 acknowledge "DM has no watchdog by default" and rely on every *graceful* exit calling `disable`. But on `SIGKILL` / host power-yank / OS panic — exactly the `damage-control` drill (§5: "yank USB / kill terminal") — no `disable` frame is sent, and a DM‑J4310 in MIT velocity mode continues executing the **last commanded velocity** until a new frame arrives or power is cut. With no comm-loss timeout register set, it grinds into a stop at the `TMAX` cap indefinitely. `damiao/config_tool.py` can read/write the RID timeout register (§3.1, §8), but nothing **mandates** setting it.

The `TMAX 2.0 Nm` cap (§3.2) limits the *severity* of the grind but does not stop it; the worm can't back-drive, so a stalled 2 Nm push against a hard stop persists until power removal.

**Fix (spec text §3.4 + §4):** `setup` must write a nonzero comm-loss timeout to the motor's RID timeout register and `preflight` must **verify** it as a go/no-go item, so a dead host auto-disables the actuator. Promote this from an optional `config_tool` capability to a mandated step; add a row to the §4 failure table for "host process killed → motor watchdog disables within Tms."

Evidence: spec §3.1 (`config_tool … read/write RID registers (timeout etc.)`), §3.4 line ~206 ("DM has no watchdog by default"), §4 line ~226, §5 damage-control drill.

---

## Majors

### M1. A single uv workspace over lerobot(+torch) and fastgripper-dm(python-can) is the wrong structure
A uv workspace resolves **all** members into **one lockfile and one shared venv**, with a single `requires-python` equal to the *intersection* of members. The two members are structurally incompatible for that:
- `lerobot-robot-fastgripper/pyproject.toml`: `requires-python = ">=3.12"`, `dependencies = ["lerobot[feetech]>=0.6.0,<0.7"]` (heavy torch stack).
- `fastgripper-dm` (spec §3): `requires-python >= 3.10`, deps `python-can` only.

Consequences:
1. The shared dev/CI venv always drags in torch/lerobot even to touch the gripper package.
2. fastgripper-dm's 3.10/3.11 support is never exercised in-workspace (intersection forces ≥3.12).
3. Any transitive pin conflict between torch's world and python-can's makes the workspace fail to co-resolve at all — a hard stop, not a warning.

§2 explicitly wants "own deps, own tests, own release tag" — precisely what one workspace lockfile does **not** provide.

**Fix (spec text §2):** do **not** use a single uv workspace. Keep two independent packages in the monorepo, each with its own `pyproject.toml` and its own lockfile (a monorepo, not a `[tool.uv.workspace]`). If you want `uv` ergonomics, omit the workspace table (or exclude the lerobot package from default sync). Rewrite the "uv workspace root (no code)" line and the "shared venv" assumption. (Note: PyPI end-users are unaffected — they `pip install` one package — so this is a dev/CI-environment defect, not an end-user one.)

Evidence: `fastgripper-lerobot/pyproject.toml` (`requires-python = ">=3.12"`, `lerobot[feetech]>=0.6.0,<0.7`); spec §2 lines ~76, ~82-84, ~54-77.

### M2. Release workflow needs more than "copy release.yml"; the current one won't tag per-package
`release.yml:2` triggers on `tags: ["v*"]`, does a root `uv build`, and publishes to a single PyPI project via `environment: pypi`. The spec wants `dm-v0.2.0` / `lerobot-v0.2.0` tags (§2). Copying it verbatim means:
- A `dm-v0.2.0` tag does not match `v*` → no publish.
- A `v*` tag would build/publish for *both* packages into one project.
- A bare `uv build` at the workspace/monorepo root has no `[project]` to build → fails or is ambiguous.

**Fix (spec text §2):** spell out per-package workflow changes explicitly:
- Tag filter `dm-v*` (resp. `lerobot-v*`), and gate publish with `if: startsWith(github.ref, 'refs/tags/dm-v')`.
- Build the specific package: `uv build --package fastgripper-dm` (resp. `lerobot-robot-fastgripper`).
- Distinct GitHub `environment:` per package (`pypi-dm` / `pypi-lerobot`) — each PyPI project has its own trusted-publisher (OIDC) config, so they cannot share one `environment: pypi`.
- Keep the `workflow_dispatch` → TestPyPI dry-run path per package.

Evidence: `fastgripper-lerobot/.github/workflows/release.yml:2` (`tags: ["v*"]`), lines for `environment: pypi` and root `uv build`; spec §2 lines ~83-84.

### M3. Bench serial numbers leak into the public repo
§2 places `bench/tools/usb_stress.py` in the **public** monorepo; §9/§2 route only *logs* (with serials) to the private `bench-drills` repo. But the actual `usb_stress.py` hard-codes real device paths/serials:
```
23:    "leader_1": "/dev/cu.usbmodem5C4C1268341",
24:    "leader_2": "/dev/cu.usbmodem5C4C1269411",
25:    "follower_2": "/dev/cu.usbmodem5C4C1267161",
```
Committing this file publicly leaks bench hardware identifiers. (The file already has VID/PID auto-discovery at line 32, `idVendor=0x1D50, idProduct=0x606F`, so the hard-coded map is unnecessary.)

**Fix (spec text §2/§9):** before promoting any `bench/tools/*` to the public repo, parameterize device selection (auto-discover via VID/PID, or read from a git-ignored config/env). Add "no hard-coded device serials/paths" to the public/private split checklist in §9, and require a grep sweep of `bench/` for `usbmodem`/`/dev/cu.`/serials, not just this one file.

Evidence: `YAM Test/usb_stress.py:23-25` (hard-coded serials), `:32` (VID/PID discovery already present); spec §2 lines ~64, ~79-81, §9.

### M4. The i2rt fork won't capture all the patches — most are uncommitted working-tree edits
§8 says the fork will carry "the patches as commits on branch `fastgripper`." In the actual clone (`YAM Test/i2rt`), only **one** commit exists beyond `origin/main`:
```
f732e4f dm_driver: make motor recovery exception-safe with backoff
```
The other patches §8 enumerates (drain-before-bring-up, response-id matching, `I2RT_CAN_RESPONSE_TIMEOUT`, gs_usb backend selection) live as **uncommitted** working-tree modifications:
```
 M i2rt/motor_config_tool/ping_motors.py
 M i2rt/motor_config_tool/set_timeout.py
 M i2rt/motor_config_tool/set_zero.py
 M i2rt/motor_drivers/can_interface.py
 M i2rt/motor_drivers/utils.py
```
A naïve "push the branch to the fork" loses every one of these — i.e. the whole reason the YAM works today.

**Fix (spec text §6/§8):** add an explicit, ordered step: *commit the current working-tree diff into the fork's `fastgripper` branch and diff-verify it against this venv/clone* before `adapters/i2rt.py` or the `[i2rt]` extra depends on the fork. Make it a precondition of §6 step 4 (see m2).

Evidence: `git -C "YAM Test/i2rt" log origin/main..HEAD` → single commit `f732e4f`; `git -C … status -s` → 5 modified, uncommitted files; spec §8 lines ~275-283.

---

## Minors

### m1. Cal-store cwd precedence is a footgun, and inconsistent with `config.json`
§3.3 resolves `gripper_cal.json` as `$FASTGRIPPER_DM_CAL → ./gripper_cal.json → ~/.config/fastgripper-dm/gripper_cal.json`. Running from any directory that happens to hold a stale `gripper_cal.json` silently shadows the real calibration — dangerous for a device whose cal marks drive `goto` targets toward hard stops. Meanwhile §3.4's `config.json` lives **only** in `~/.config` (no cwd, no env override), so the two files follow different precedence rules.

**Fix (spec text §3.3):** drop cwd from the default cal search (or require an explicit `--cal PATH` / `--local` opt-in), and make both files use the same precedence order (env → `~/.config`, with an explicit flag for a local file).

Evidence: spec §3.3 line ~188 vs §3.4 line ~194.

### m2. Migration order (§6) assumes an artifact it never schedules
§6 step 4 (`adapters/i2rt.py`, YAM drill, retire YAM Test) depends on the i2rt fork/submodule of §8, but *creating and committing that fork* (see M4) is not a numbered step. As written, step 4's pass criteria can't be reached.

**Fix (spec text §6):** insert "create & commit-verify the i2rt fork (per M4)" as step 0, or as the explicit first half of step 4, before `adapters/i2rt.py`.

Evidence: spec §6 step 4 (line ~259) vs §8.

### m3. `preflight` doesn't bind the selected cal entry to the connected motor
§3.4 `preflight` checks "cal entry + park" but not that the entry's `motor_id`/`master_id` (§3.3) match the motor actually answering on the bus. With named entries + `--gripper`, selecting the wrong entry (e.g. a bimanual left-hand entry against the right motor) yields a wrong span / `close_dir`; `goto` then drives toward a bogus target. Torque-capped, but still a grind into a stop.

**Fix (spec text §3.4):** add to `preflight`'s go/no-go list: "selected cal entry `motor_id` == the id that answered TX echo."

Evidence: spec §3.4 line ~196 (`cal entry + park`), §3.3 entry fields include `motor_id, master_id`.

### m4. Multi-gripper (bimanual: two DMs on one bus) is only half-designed
The *data model* supports it — per-entry `motor_id`/`master_id` (§3.3) and named entries — but the *plumbing* doesn't: `setup` saves a single global port/channel (§3.4), the CLI verbs address only `--gripper`, and two `DamiaoCanPort`s on one channel raise an unaddressed "who owns the port / concurrent writers" question. §1 lists bimanual-capable arms.

**Fix (spec text §3.4/§10):** state the v0 scope explicitly — either "one gripper per process/bus in v0" (and move bimanual to non-goals), or define the two-controllers-one-bus port-ownership rule.

Evidence: spec §3.3 (per-entry ids), §3.4 (single-channel `setup`), §1 table.

### m5. i2rt threading ownership is ambiguous — risk of two writers
§3.1/§3.2 say the adapter wraps "an i2rt `DMChainCanInterface` / `motor_chain_robot`." The former is synchronous (single owner — safe with the teleop's `step()`); `motor_chain_robot` runs its **own** background control thread, which would mean two writers to the same motor when `port.command()` also sends frames. §3.2 says the YAM teleop calls `step()` from its own loop, compounding the ambiguity.

**Fix (spec text §3.1):** pin the adapter to the synchronous `DMChainCanInterface` and explicitly forbid running i2rt's `motor_chain_robot` control loop concurrently; state who owns the port in adapter mode.

Evidence: spec §3.1 line ~106 (`wraps an i2rt DMChainCanInterface / motor_chain_robot`), §3.2 line ~180 (teleop calls `step()`).

### m6. Windows is silent
§1 declares Linux/SocketCAN first-class and macOS/gs_usb the bench; Windows is never mentioned. python-can supports Windows, but `os._exit` (libusb SIGABRT) and `caffeinate -w` (§3.4, §4) are macOS-specific, and there's no SocketCAN on Windows.

**Fix (spec text §1):** one line — "Windows unsupported in v0" (or name the intended path).

Evidence: spec §1 lines ~26-28, §3.4 line ~205-206, §4 line ~226.

---

## Checked and found sound

- **Per-package version/tag scheme** (`dm-v*` / `lerobot-v*`, §2) is coherent *once M2's workflow mechanics are fixed* — the naming itself is fine.
- **OIDC trusted publishing** (`id-token: write` + `pypa/gh-action-pypi-publish@release/v1`, no stored tokens) is the correct, token-less approach; reuse the pattern per package. (`release.yml`.)
- **Keeping `lerobot-robot-fastgripper` a separate package** (not rewritten on `fastgripper-dm`, §1 non-goals) is correct — different motor (Feetech STS3215 vs DM‑J4310) and a deliberately narrow lerobot pin (`fastgripper-lerobot/pyproject.toml`).
- **Torque cap through impacts** (`KD = TMAX/VMAX` plus per-tick clamp of `v_cmd` to `v_actual ± TMAX/KD`, §3.2) genuinely bounds worm torque during homing/goto, *given a live control loop* (B2 is about the loop dying, not this math).
- **gs_usb macOS patch** is real and correctly scoped to `patches/` — venv `gs_usb/gs_usb.py:56` carries the darwin guard on `is_kernel_driver_active`, consistent with §8.
- **The port abstraction** (`MotorPort` protocol, §3.1) cleanly isolates the standalone/adapter split; "a port never interprets the gripper" is the right boundary.

---

### Files & evidence cited
- Spec: `docs/superpowers/specs/2026-08-30-fastgripper-dm-design.md` (§ and line refs above)
- `fastgripper-lerobot/.github/workflows/release.yml:2` (`tags: ["v*"]`), `environment: pypi`, root `uv build`
- `fastgripper-lerobot/pyproject.toml` (`requires-python = ">=3.12"`, `lerobot[feetech]>=0.6.0,<0.7`)
- `fastgripper-openarm/pyproject.toml` (`python-can>=4.0`, `requires-python >=3.10`)
- `YAM Test/usb_stress.py:23-25` (hard-coded serials), `:32` (VID/PID `0x1D50/0x606F` discovery)
- `YAM Test/i2rt`: `git log origin/main..HEAD` → single commit `f732e4f`; `git status -s` → 5 uncommitted modified files
- venv `gs_usb/gs_usb.py:56` (darwin `is_kernel_driver_active` guard)
- PyPI PEP 508 direct-reference upload policy (metadata with URL requirements is rejected)
