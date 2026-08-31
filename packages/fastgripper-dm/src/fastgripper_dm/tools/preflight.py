"""No-motion go/no-go for a standalone FastGripper on its own CAN channel.

Checks: adapter/bus opens · TX echo (something ACKs) · the configured motor
answers · no latched fault · watchdog register == configured (and != 0) ·
cal entry present with open/closed marks and a park position.
Each FAIL names the fix.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from ..calstore import default_cal_path, load_store

TMAX_CAP = 2.0


@dataclass
class Finding:
    level: str  # FAIL / WARN / OK
    text: str


def evaluate(echo: bool | None, status: int | None, watchdog: int | None, watchdog_want: int,
             entry: dict | None, bus_error: str | None = None) -> list[Finding]:
    out: list[Finding] = []
    if bus_error:
        out.append(Finding("FAIL", f"bus: {bus_error}"))
        return out
    if not echo:
        out.append(Finding("FAIL", "bus: no TX echo -- nothing ACKs: motor PSU off / CAN wiring / adapter wedged (replug USB)"))
    if status is None:
        out.append(Finding("FAIL", "motor: no reply -- power, wiring, or wrong --motor_id (run `fastgripper-dm doctor` / `id --verify`)"))
    elif status >= 0x8:
        out.append(Finding("FAIL", f"motor: latched fault {status:#x} -- power-cycle the motor supply, then rerun"))
    else:
        out.append(Finding("OK", f"motor answers ({'enabled' if status == 1 else 'disabled'})"))
    if watchdog is None:
        out.append(Finding("FAIL", "watchdog: could not read RID 9"))
    elif watchdog == 0:
        out.append(Finding("FAIL", "watchdog: DISABLED (0) -- a dead host would leave the motor pushing; run `fastgripper-dm setup`"))
    elif watchdog != watchdog_want:
        out.append(Finding("WARN", f"watchdog: {watchdog} ms (configured {watchdog_want} ms) -- run `fastgripper-dm setup`"))
    else:
        out.append(Finding("OK", f"watchdog: {watchdog} ms"))
    if not entry:
        out.append(Finding("FAIL", "cal: no gripper entry -- run `fastgripper-dm calibrate` or `autocal full`"))
    elif "open" not in entry or "closed" not in entry:
        out.append(Finding("FAIL", "cal: entry has no open/closed marks -- calibrate"))
    elif entry.get("last_position") is None:
        out.append(Finding("WARN", "cal: no park position -- run `fastgripper-dm autocal home` before goto"))
    else:
        out.append(Finding("OK", f"cal: open {entry['open']:+.2f} closed {entry['closed']:+.2f} park {entry['last_position']:+.2f} rad"))
    return out


def collect(bus, motor_id: int, master_id: int):
    """Returns (echo, status_nibble | None, watchdog_ms | None)."""
    import can

    from ..damiao.config_tool import RegisterClient

    echo = False
    status = None
    bus.send(can.Message(arbitration_id=motor_id, data=[0xFF] * 7 + [0xFD], is_extended_id=False))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        m = bus.recv(timeout=0.05)
        if m is None:
            continue
        if not m.is_rx:
            echo = True
            continue
        if m.is_error_frame or len(m.data) != 8 or (m.data[0] & 0x0F) != (motor_id & 0x0F):
            continue
        status = ((m.data[0] - (motor_id & 0xFF)) >> 4) & 0x0F
        break
    try:
        watchdog = int(RegisterClient(bus).read(motor_id, "timeout"))
    except Exception:
        watchdog = None
    return echo, status, watchdog


def run_preflight(cfg: dict, interface: str | None = None, channel: str | None = None) -> bool:
    from ..calstore import get_entry
    from ..damiao.canbus import open_bus
    from ..port import PortError

    motor_id = int(cfg.get("motor_id", 0x01))
    master_id = int(cfg.get("master_id", 0x00))
    want = int(cfg.get("watchdog_ms", 500))
    store = load_store(default_cal_path())
    entry = None
    if store["grippers"]:
        try:
            _, entry = get_entry(store, cfg.get("gripper"))
        except SystemExit:
            entry = None
    bus_error = None
    echo = status = watchdog = None
    try:
        with open_bus(interface or cfg.get("interface", "auto"), channel or cfg.get("channel")) as bus:
            echo, status, watchdog = collect(bus, motor_id, master_id)
    except (PortError, SystemExit) as e:
        bus_error = str(e).splitlines()[0]
    findings = evaluate(echo, status, watchdog, want, entry, bus_error)
    for f in findings:
        print(f"  [{f.level:4}] {f.text}")
    ok = not any(f.level == "FAIL" for f in findings)
    print("preflight:", "GO" if ok else "NO-GO")
    return ok
