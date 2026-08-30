# Adversarial review — correctness against existing code (rev 1)

Reviewer lens: correctness of the spec's claims vs the code it says it ports.
Delivered inline 2026-08-30; recorded here. All items applied in rev 2.

Blockers
1. §3.1/§3.2 I2rtChainPort.command() cannot be synchronous: i2rt DMChainCanInterface runs its own 250 Hz thread (dm_driver.py:538-546, 551-630); set_commands() queues and returns cached read_states() (dm_driver.py:744-767, 715-734). → rev 2 §3.2: tick-based adapter, owner merges the command.
2. §3.2 "YAM teleop calls step()" infeasible: set_commands replaces ALL joint commands (dm_driver.py:744-765); so101_teleop builds a full 7-element command (so101_teleop.py:454, 332-338, 442-443). → rev 2 §3.2.

Majors
3. Feedback.position "raw as reported" wrong on i2rt: read_states().pos is already unwrapped (dm_driver.py:484-513, 728); wrapped value only via get_joint_pos()*SPAN−POS_WINDOW with gripper_limits_override=[−12.5,12.5] (so101_teleop.py:303-306,339,398; gripper.py:113,142). → rev 2 §3.1/§3.2.
4. Park/restore: so101_teleop adopts exactly (so101_teleop.py:111) but dm4310.MultiTurnTracker rounds to the nearest window (dm4310.py:111), used by autocal. → rev 2: rounding tracker retired, exact adoption chosen.
5. last_wrapped boot check is new (never read for validation: so101_teleop.py:466; autocal.py:326,395). → rev 2 labels it new, tested in sim + drill.
6. Torque cap exists only in so101_teleop (410-411; constants 73,77,81,91); fastgripper_yam has no cap, KD 1.0, SW_KP 4.0, GLIDE_VMAX 3.0 (gripper.py:31-40,200-227). → rev 2 §3.2.

Minors
7. caffeinate flags are `-dims -w` (so101_teleop.py:186). → fixed.
8. clear_error+re-enable backoff is i2rt-only (dm_driver.py:632-686); standalone autocal aborts on fault (autocal.py:73-75). → rev 2: DamiaoCanPort implements recovery.
9. "sign-magnitude" mischaracterised; fix is ((d[0]-(can_id&0xFF))>>4)&0x0F (dm4310.py:71). → fixed.
10. Format-2 schema missing span_from_closed, touched_at, homed_at (autocal.py:383,327,371); path precedence is new. → fixed.
11. preflight TMAX check today greps so101_teleop.py (preflight.py:220-222). → rev 2: reads profile.tmax.
12. autocal flag defaults diverge (YAM opt-in, span 33.5, autocal.py:215-225; URtest defaults ON, span cal→30.0, URtest autocal.py:226-243) and home guards undescribed (URtest autocal.py:46-48,350-368). → rev 2 profiles + guards.

Checked and correct: MIT velocity control + software P-loop; cap formula; tracker unwrap on jump>SPAN/2; closed_only derivation; calstore format 2 + legacy upgrade; `id` alone-on-bus enforcement (set_gripper_id.py:102-108, 88); os._exit discipline; Python ≥3.10 claim; sim feasibility (torque must be modelled as KD·(v_cmd−v_actual)).
