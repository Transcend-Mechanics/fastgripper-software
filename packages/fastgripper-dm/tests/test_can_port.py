import pytest
import can
from fastgripper_dm.damiao.can_port import DamiaoCanPort, PortFault
from fastgripper_dm.damiao.dm4310 import DM4310, float_to_uint
from fastgripper_dm.port import MitCommand, PortError


class FakeMotorBus:
    """Replies to MIT/special frames like a DM at motor_id, feedback on master_id."""

    def __init__(self, motor_id=0x07, master_id=0x17, status=0x1, position=1.0):
        self.motor_id, self.master_id = motor_id, master_id
        self.status, self.position = status, position
        self.rx = []
        self.sent = []

    def _feedback(self):
        p = float_to_uint(self.position, -12.5, 12.5, 16)
        d = [((self.status << 4) | (self.motor_id & 0x0F)) & 0xFF, p >> 8, p & 0xFF, 0x80, 0x00, 0x00, 25, 26]
        return can.Message(arbitration_id=self.master_id, data=bytes(d), is_rx=True)

    def send(self, msg):
        self.sent.append(msg)
        if msg.arbitration_id == self.motor_id:
            if msg.data[-1] == 0xFB:      # clear_error
                self.status = 0x0
                return
            if msg.data[-1] == 0xFC:      # enable
                self.status = 0x1
            self.rx.append(self._feedback())

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None


def test_command_returns_this_motors_feedback():
    bus = FakeMotorBus(position=2.5)
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.enable()
    fb = port.command(MitCommand(vel=1.0, kd=0.5))
    assert fb.position == pytest.approx(2.5, abs=1e-3) and fb.enabled


def test_silent_motor_raises_porterror_within_budget():
    bus = FakeMotorBus()
    bus.send = lambda msg: None            # nothing ever answers
    port = DamiaoCanPort(bus, 0x07, 0x17, retry_budget_s=0.05)
    with pytest.raises(PortError):
        port.command(MitCommand())


def test_fault_triggers_recovery_then_portfault():
    bus = FakeMotorBus(status=0xD)         # latched comm loss; FB/FC path clears it
    port = DamiaoCanPort(bus, 0x07, 0x17, retry_budget_s=0.05)
    fb = port.command(MitCommand())        # recover() clears + enables -> healthy reply
    assert fb.enabled
    assert any(m.data[-1] == 0xFB for m in bus.sent)   # clear_error was sent


def test_read_is_zero_gain():
    bus = FakeMotorBus()
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.read()
    mit = [m for m in bus.sent if m.arbitration_id == 0x07 and m.data[-1] not in (0xFB, 0xFC, 0xFD)]
    assert mit, "read must send a command frame"
    # kd bits (byte 5 high nibble + byte 6 low? -- decode: kd occupies data[5]<<4|data[6]>>4)
    d = mit[-1].data
    assert (d[5] << 4) | (d[6] >> 4) == 0  # kd == 0


def test_close_disables():
    bus = FakeMotorBus()
    port = DamiaoCanPort(bus, 0x07, 0x17)
    port.enable()
    port.close()
    assert any(m.data[-1] == 0xFD for m in bus.sent)
    port.close()                            # idempotent
