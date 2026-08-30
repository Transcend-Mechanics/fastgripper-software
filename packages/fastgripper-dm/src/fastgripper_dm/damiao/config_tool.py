"""DM-J4310 register access over plain python-can: motor id, master id, and
the CAN comm-loss watchdog (RID 9, milliseconds).

Frames (DM "special message" protocol, arbitration id 0x7FF):
  read : [motor_id, 0x00, 0x33, rid, 0, 0, 0, 0]  -> reply data[4:8] = value
  write: [motor_id, 0x00, 0x55, rid, v0, v1, v2, v3]
  save : [motor_id, 0x00, 0xAA, rid, 0, 0, 0, 0]   (persist to flash)
Values are little-endian uint32 (id/master_id/timeout/sw_ver) or float32.

Ported from set_gripper_id.py + i2rt.motor_config_tool.utils, without the
i2rt dependency so the standalone package needs only python-can.
"""

from __future__ import annotations

import struct
import time

import can

CONFIG_ID = 0x7FF
REGISTERS = {           # name -> (rid, fmt)
    "master_id": (7, "<I"),
    "id": (8, "<I"),
    "timeout": (9, "<I"),   # comm-loss watchdog, ms; 0 = disabled
    "sw_ver": (14, "<I"),
}
DEFAULT_WATCHDOG_MS = 500


class RegisterClient:
    def __init__(self, bus: can.BusABC, response_timeout: float = 0.2, retries: int = 5):
        self.bus = bus
        self.response_timeout = response_timeout
        self.retries = retries

    def _txrx(self, motor_id: int, data: list[int], required: bool = True,
              accept_ids: tuple[int, ...] = ()) -> can.Message | None:
        """Send a special frame; return the motor's reply. Replies arrive on 0x7FF with
        data[0] == motor_id and data[2] == op; some firmware answers the flash-save
        (0xAA) inconsistently, so callers may pass required=False and verify by read-back."""
        last = None
        for _ in range(self.retries):
            self.bus.send(can.Message(arbitration_id=CONFIG_ID, data=bytes(data), is_extended_id=False))
            t0 = time.monotonic()
            while time.monotonic() - t0 < self.response_timeout:
                m = self.bus.recv(timeout=0.02)
                if m is None or not m.is_rx or len(m.data) != 8:
                    continue
                ok_ids = {motor_id & 0xFF, *(i & 0xFF for i in accept_ids)}
                if m.data[0] in ok_ids and m.data[2] == data[2]:
                    return m
                last = m
            time.sleep(0.01)
        if not required:
            return None
        raise TimeoutError(f"motor 0x{motor_id:02X}: no register reply (last frame: {last})")

    def read(self, motor_id: int, name: str):
        rid, fmt = REGISTERS[name]
        m = self._txrx(motor_id, [motor_id & 0xFF, 0x00, 0x33, rid, 0, 0, 0, 0])
        return struct.unpack(fmt, bytes(m.data[4:8]))[0]

    def write(self, motor_id: int, name: str, value) -> None:
        rid, fmt = REGISTERS[name]
        payload = list(struct.pack(fmt, value))
        # the reply to an id write already carries the NEW id (live 2026-08-30:
        # "07 00 55 08 07 00 00 00" on the new master id), so accept it too
        accept = (int(value),) if name == "id" else ()
        self._txrx(motor_id, [motor_id & 0xFF, 0x00, 0x55, rid] + payload, accept_ids=accept)

    def save(self, motor_id: int, name: str) -> bool:
        """Persist a register to flash. Returns True if the motor acknowledged; a silent
        save is not an error by itself -- verify with a read-back after a power cycle."""
        rid, _ = REGISTERS[name]
        return self._txrx(motor_id, [motor_id & 0xFF, 0x00, 0xAA, rid, 0, 0, 0, 0], required=False) is not None

    def probe(self, motor_id: int) -> bool:
        try:
            self.read(motor_id, "sw_ver")
            return True
        except TimeoutError:
            return False


def probe_ids(client: RegisterClient, ids=range(1, 9)) -> list[int]:
    return [i for i in ids if client.probe(i)]


def read_watchdog_ms(client: RegisterClient, motor_id: int) -> int:
    return int(client.read(motor_id, "timeout"))


def set_watchdog_ms(client: RegisterClient, motor_id: int, ms: int = DEFAULT_WATCHDOG_MS) -> int:
    """Write + persist + read back the comm-loss watchdog. Returns the value read back."""
    client.write(motor_id, "timeout", int(ms))
    client.save(motor_id, "timeout")
    got = read_watchdog_ms(client, motor_id)
    if got != int(ms):
        raise RuntimeError(f"watchdog write failed: wrote {ms}, motor reports {got}")
    return got


def set_motor_id(client: RegisterClient, old_id: int, new_id: int, master_id: int | None = None) -> tuple[int, int]:
    """Re-ID a motor that must be ALONE on the bus (enforced). Returns (id, master_id) read back."""
    master = master_id if master_id is not None else new_id + 0x10
    if new_id == master:
        raise ValueError("new_id and master_id must differ (feedback would collide with commands)")
    others = [i for i in probe_ids(client) if i not in (old_id, new_id)]
    if others:
        raise RuntimeError(f"other motors {others} are answering -- disconnect everything except the gripper motor")
    if client.probe(new_id):
        return int(client.read(new_id, "id")), int(client.read(new_id, "master_id"))
    if not client.probe(old_id):
        raise RuntimeError(f"no motor at 0x{old_id:02X} and none at 0x{new_id:02X}: check power and CAN wiring")
    client.write(old_id, "master_id", master)
    client.save(old_id, "master_id")
    client.write(old_id, "id", new_id)
    for target in (old_id, new_id):        # after the id write the motor may answer on either
        if client.save(target, "id"):
            break
    got_id, got_master = int(client.read(new_id, "id")), int(client.read(new_id, "master_id"))
    if (got_id, got_master) != (new_id, master):
        raise RuntimeError(f"id write failed: wanted 0x{new_id:02X}/0x{master:02X}, motor reports 0x{got_id:02X}/0x{got_master:02X}")
    return got_id, got_master
