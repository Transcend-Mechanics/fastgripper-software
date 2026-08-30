"""Minimal driver for the Damiao DM-J4310 motor over python-can.

Implements the MIT-mode protocol from the DM-J4310-2EC V1.1 manual:
  - special commands (enable / disable / set zero)
  - MIT control frame packing (16-bit pos, 12-bit vel/kp/kd/torque)
  - feedback frame decoding

All scaling constants are the motor's factory defaults; if you change
P_MAX / V_MAX / T_MAX with the Damiao debugging assistant, mirror them here.
"""

import time
from dataclasses import dataclass

import can

# Factory default scaling ranges for DM-J4310 (change here if reconfigured).
P_MAX = 12.5   # rad, position range is [-P_MAX, +P_MAX]
V_MAX = 30.0   # rad/s
T_MAX = 10.0   # N*m
KP_MAX = 500.0
KD_MAX = 5.0

FAULT_CODES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "MOS overtemperature",
    0xC: "motor coil overtemperature",
    0xD: "communication loss",
    0xE: "overload",
}


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    return x_min + u * (x_max - x_min) / ((1 << bits) - 1)


@dataclass
class Feedback:
    motor_id: int
    error: int          # status nibble: 0 disabled, 1 enabled, 8+ faults (see FAULT_CODES)
    position: float     # rad, output shaft
    velocity: float     # rad/s
    torque: float       # N*m
    temp_mos: int       # deg C
    temp_rotor: int     # deg C

    @property
    def error_text(self) -> str:
        return FAULT_CODES.get(self.error, f"unknown ({self.error:#x})")

    @property
    def faulted(self) -> bool:
        return self.error >= 0x8


def decode_feedback(msg: can.Message, can_id: int | None = None) -> Feedback:
    # Byte 0 is (status << 4) + motor ID. A motor re-ID'd above 0x0F overflows
    # the 4-bit ID field into the status nibble (observed at ID 0x20: byte 0 is
    # 0x20 disabled / 0x30 enabled), so pass the motor's can_id to normalize —
    # subtracting it recovers the status nibble for small and large IDs alike.
    d = msg.data
    error = ((d[0] - (can_id & 0xFF)) >> 4) & 0x0F if can_id is not None else d[0] >> 4
    pos_raw = (d[1] << 8) | d[2]
    vel_raw = (d[3] << 4) | (d[4] >> 4)
    tau_raw = ((d[4] & 0x0F) << 8) | d[5]
    return Feedback(
        motor_id=can_id if can_id is not None else d[0] & 0x0F,
        error=error,
        position=uint_to_float(pos_raw, -P_MAX, P_MAX, 16),
        velocity=uint_to_float(vel_raw, -V_MAX, V_MAX, 12),
        torque=uint_to_float(tau_raw, -T_MAX, T_MAX, 12),
        temp_mos=d[6],
        temp_rotor=d[7],
    )


class MultiTurnTracker:
    """Unwrap the motor's ±P_MAX position window into a continuous position.

    The DM4310 reports position only within ±12.5 rad (~±2 output turns); a
    worm-gear gripper spans many turns, so we count window wrap-arounds.
    Feed every feedback frame in; poll fast enough that the shaft can't move
    more than half a window (12.5 rad) between samples — trivial at any sane
    poll rate and jog speed.
    """

    SPAN = 2 * P_MAX  # 25 rad

    def __init__(self, start_unwrapped: float | None = None):
        self._wraps = 0
        self._last_wrapped: float | None = None
        self._start_offset = 0.0
        self._pending_start = start_unwrapped

    def update(self, wrapped: float) -> float:
        if self._last_wrapped is None:
            self._last_wrapped = wrapped
            if self._pending_start is not None:
                # Reconcile with a saved unwrapped position: the worm gear
                # can't be back-driven, so the shaft is where we left it —
                # pick the wrap count that matches the saved value.
                self._start_offset = round((self._pending_start - wrapped) / self.SPAN) * self.SPAN
        else:
            delta = wrapped - self._last_wrapped
            if delta > self.SPAN / 2:
                self._wraps -= 1
            elif delta < -self.SPAN / 2:
                self._wraps += 1
            self._last_wrapped = wrapped
        return self.position

    @property
    def position(self) -> float:
        assert self._last_wrapped is not None, "no feedback seen yet"
        return self._last_wrapped + self._wraps * self.SPAN + self._start_offset

    @property
    def wrapped(self) -> float:
        """Last raw motor-frame position (inside ±P_MAX)."""
        assert self._last_wrapped is not None, "no feedback seen yet"
        return self._last_wrapped


class DM4310:
    def __init__(self, bus: can.BusABC, can_id: int = 0x01, master_id: int = 0x00):
        self.bus = bus
        self.can_id = can_id
        self.master_id = master_id

    def _send(self, arbitration_id: int, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False))

    def _special(self, cmd: int) -> None:
        self._send(self.can_id, bytes([0xFF] * 7 + [cmd]))

    def enable(self) -> None:
        self._special(0xFC)

    def disable(self) -> None:
        self._special(0xFD)

    def set_zero(self) -> None:
        """Save the current position as zero (within the single +/-P_MAX window)."""
        self._special(0xFE)

    def mit_control(self, position: float, velocity: float, kp: float, kd: float, torque: float) -> None:
        """Send one MIT-mode control frame. The motor replies with a feedback frame."""
        p = float_to_uint(position, -P_MAX, P_MAX, 16)
        v = float_to_uint(velocity, -V_MAX, V_MAX, 12)
        kp_u = float_to_uint(kp, 0.0, KP_MAX, 12)
        kd_u = float_to_uint(kd, 0.0, KD_MAX, 12)
        t = float_to_uint(torque, -T_MAX, T_MAX, 12)
        data = bytes([
            p >> 8,
            p & 0xFF,
            v >> 4,
            ((v & 0x0F) << 4) | (kp_u >> 8),
            kp_u & 0xFF,
            kd_u >> 4,
            ((kd_u & 0x0F) << 4) | (t >> 8),
            t & 0xFF,
        ])
        self._send(self.can_id, data)

    def speed_control(self, velocity: float) -> None:
        """Speed-mode command: CAN ID 0x200+ID, one little-endian float (rad/s).
        Ignored unless the motor is configured in Speed mode."""
        import struct
        self._send(0x200 + self.can_id, struct.pack("<f", velocity))

    def pos_speed_control(self, position: float, velocity: float) -> None:
        """Position-speed-mode command: CAN ID 0x100+ID, two LE floats (rad, rad/s).
        Ignored unless the motor is configured in Position-Speed mode."""
        import struct
        self._send(0x100 + self.can_id, struct.pack("<ff", position, velocity))

    def read_feedback(self, timeout: float = 1.0, debug: bool = False) -> Feedback | None:
        """Wait for the next feedback frame from this motor's master ID.

        Polls with SHORT recv timeouts: long blocking reads through
        pyusb/libusb on macOS reliably miss frames that short polls catch.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=0.02)
            if msg is None:
                continue
            if msg.arbitration_id == self.master_id and len(msg.data) == 8 and msg.is_rx:
                return decode_feedback(msg, can_id=self.can_id)
            if debug:
                kind = "rx" if msg.is_rx else "tx echo"
                print(f"    [debug] skipped {kind}: id={msg.arbitration_id:#05x} data={msg.data.hex(' ')}")
        if debug:
            print(f"    [debug] no feedback within {timeout}s (short-poll)")
        return None
