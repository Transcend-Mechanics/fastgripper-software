# Troubleshooting

Run `fastgripper-dm preflight` first. Each FAIL line names the fix. Then:

| Symptom | Cause | Fix |
| --- | --- | --- |
| `no TX echo -- nothing ACKs` | motor PSU off, CAN H/L open, or (macOS) wedged adapter | PSU on; tug-test the connector; replug the adapter USB |
| motor silent, echo present | wrong `--motor_id`, or the motor latched a fault | `fastgripper-dm doctor --ids 1,2,3,4,5,6,7`; `id --verify`; power-cycle the motor supply |
| `latched fault 0xd` (communication loss) | the watchdog tripped: host stalled/crashed | power-cycle the motor supply; check what stalled the host |
| `watchdog: DISABLED (0)` | factory motor | `fastgripper-dm setup` |
| `re-anchor offset … exceeds limit` in `autocal home` | probe triggered on friction, not the stop | raise `--contact_torque` above the printed free-run p95, `--probe_tmax` ~0.3 above that |
| "closed" stops early / "open" overruns | stale turn state after a hand-move while off | `fastgripper-dm autocal home` (never hand-edit the cal) |
| `Device not configured` / adapter vanished | USB power/hub/cable (bus-powered hubs, damaged dongles) | direct port or powered hub; see the FastGripper LeRobot repo's `usb-serial-drops` guide — same mechanics |
