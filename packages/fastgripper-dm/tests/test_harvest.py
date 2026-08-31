"""Hardware-free smoke tests for the v0.0.x harvest release."""

import json
import struct
import subprocess
import sys

import can
import pytest

import fastgripper_dm
from fastgripper_dm import calstore
from fastgripper_dm.damiao import config_tool, dm4310
from fastgripper_dm.tools.preflight import evaluate


def test_version():
    assert fastgripper_dm.__version__ == "0.0.1"


# ---- protocol -------------------------------------------------------------

def test_float_uint_roundtrip():
    for x in (-12.5, -1.0, 0.0, 3.3, 12.5):
        u = dm4310.float_to_uint(x, -12.5, 12.5, 16)
        assert dm4310.uint_to_float(u, -12.5, 12.5, 16) == pytest.approx(x, abs=1e-3)


def _frame(data):
    return can.Message(arbitration_id=0x17, data=bytes(data), is_extended_id=False, is_rx=True)


def test_decode_status_nibble_normalises_large_ids():
    # motor re-ID'd to 0x20: byte 0 is 0x20 disabled / 0x30 enabled
    pos = dm4310.float_to_uint(0.0, -12.5, 12.5, 16)
    body = [pos >> 8, pos & 0xFF, 0x80, 0x08, 0x00, 0x00, 25, 26]
    assert dm4310.decode_feedback(_frame([0x20] + body[1:]), can_id=0x20).error == 0
    assert dm4310.decode_feedback(_frame([0x30] + body[1:]), can_id=0x20).error == 1
    assert dm4310.decode_feedback(_frame([0xD7] + body[1:]), can_id=0x07).error == 0xD
    assert dm4310.decode_feedback(_frame([0xD7] + body[1:]), can_id=0x07).faulted


# ---- calstore -------------------------------------------------------------

def test_cal_path_never_uses_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "gripper_cal.json").write_text("{}")
    monkeypatch.setenv("FASTGRIPPER_DM_HOME", str(tmp_path / "home"))
    assert calstore.default_cal_path() == str(tmp_path / "home" / "gripper_cal.json")
    monkeypatch.delenv("FASTGRIPPER_DM_HOME")
    assert "fastgripper-dm" in calstore.default_cal_path() and "home" not in calstore.default_cal_path()


def test_save_is_atomic_and_upgrades_legacy(tmp_path):
    p = tmp_path / "sub" / "gripper_cal.json"
    p.parent.mkdir()
    p.write_text(json.dumps({"open": -20.0, "closed": 3.0}))       # legacy flat file
    store = calstore.load_store(str(p))
    assert store == {"format": 2, "grippers": {"default": {"open": -20.0, "closed": 3.0}}}
    calstore.save_store(str(p), store)
    assert not (tmp_path / "sub" / "gripper_cal.json.tmp").exists()
    assert json.load(open(p))["format"] == 2
    name, entry = calstore.get_entry(store)
    assert name == "default" and entry["closed"] == 3.0


# ---- config tool against a fake bus --------------------------------------

class FakeBus:
    """Answers register reads/writes/saves like a DM motor at `motor_id`."""

    def __init__(self, motor_id=0x01, regs=None):
        self.motor_id = motor_id
        self.regs = regs or {7: 0x00, 8: motor_id, 9: 0, 14: 925970485}
        self.rx = []

    def send(self, msg):
        d = msg.data
        mid, op, rid = d[0], d[2], d[3]
        if mid != self.motor_id:
            return  # nobody home at that id
        if op == 0x55:
            self.regs[rid] = struct.unpack("<I", bytes(d[4:8]))[0]
            if rid == 8:
                self.motor_id = self.regs[8]
        reply = [mid, 0x00, op, rid] + list(struct.pack("<I", self.regs[rid]))
        self.rx.append(can.Message(arbitration_id=0x7FF, data=bytes(reply), is_rx=True))

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None


def test_register_client_read_write_save():
    bus = FakeBus(motor_id=0x07, regs={7: 0x17, 8: 0x07, 9: 0, 14: 1})
    c = config_tool.RegisterClient(bus, response_timeout=0.01, retries=2)
    assert c.read(0x07, "timeout") == 0
    assert config_tool.set_watchdog_ms(c, 0x07, 500) == 500
    assert config_tool.read_watchdog_ms(c, 0x07) == 500
    assert not c.probe(0x02) and c.probe(0x07)


def test_set_motor_id_reids_a_factory_motor():
    bus = FakeBus(motor_id=0x01)
    c = config_tool.RegisterClient(bus, response_timeout=0.01, retries=2)
    assert config_tool.set_motor_id(c, 0x01, 0x07) == (0x07, 0x17)
    assert bus.motor_id == 0x07 and bus.regs[7] == 0x17


def test_set_motor_id_refuses_when_others_answer():
    class TwoMotors(FakeBus):
        def send(self, msg):
            if msg.data[0] in (0x01, 0x03):
                self.motor_id = msg.data[0]
            super().send(msg)
    c = config_tool.RegisterClient(TwoMotors(), response_timeout=0.01, retries=1)
    with pytest.raises(RuntimeError, match="other motors"):
        config_tool.set_motor_id(c, 0x01, 0x07)


# ---- preflight verdicts --------------------------------------------------

def _levels(f):
    return [x.level for x in f]


def test_preflight_go():
    f = evaluate(True, 0, 500, 500, {"open": -20.0, "closed": 3.0, "last_position": 1.0})
    assert "FAIL" not in _levels(f) and "WARN" not in _levels(f)


def test_preflight_watchdog_disabled_is_no_go():
    f = evaluate(True, 0, 0, 500, {"open": -20.0, "closed": 3.0, "last_position": 1.0})
    assert any(x.level == "FAIL" and "watchdog" in x.text for x in f)


def test_preflight_no_echo_and_fault():
    f = evaluate(False, None, None, 500, None)
    assert sum(x.level == "FAIL" for x in f) >= 3
    f2 = evaluate(True, 0xD, 500, 500, {"open": -20.0, "closed": 3.0, "last_position": 1.0})
    assert any("latched fault" in x.text for x in f2)


def test_preflight_bus_error_short_circuits():
    f = evaluate(None, None, None, 500, None, bus_error="bus opened but passes NO frames")
    assert len(f) == 1 and f[0].level == "FAIL"


def test_preflight_entry_motor_binding():
    f = evaluate(True, 0, 8000, 8000, {"open": -20.0, "closed": 3.0, "last_position": 1.0,
                                       "motor_id": 8}, entry_motor_id=8, answered_id=7)
    assert any(x.level == "FAIL" and "entry" in x.text for x in f)


def test_preflight_profile_cap():
    f = evaluate(True, 0, 8000, 8000, {"open": -20.0, "closed": 3.0, "last_position": 1.0},
                 profile_tmax=2.5)
    assert any(x.level == "FAIL" and "tmax" in x.text for x in f)


def test_watchdog_want_default_8000():
    from fastgripper_dm.tools.preflight import watchdog_want_for
    # no entry, no cfg → 8000
    assert watchdog_want_for(None, {}) == 8000
    # no entry, cfg set → cfg value
    assert watchdog_want_for(None, {"watchdog_ms": 5000}) == 5000
    # entry WITHOUT profile block, cfg set → cfg wins (catches precedence bug)
    assert watchdog_want_for({"open": -20.0, "closed": 3.0}, {"watchdog_ms": 500}) == 500
    # entry WITH profile block, cfg set → profile wins
    assert watchdog_want_for({"open": -20.0, "closed": 3.0, "profile": {"watchdog_ms": 4000}},
                             {"watchdog_ms": 500}) == 4000
    # entry without profile, no cfg → 8000 default
    assert watchdog_want_for({"open": -20.0, "closed": 3.0}, {}) == 8000


# ---- CLI parses -----------------------------------------------------------

@pytest.mark.parametrize("sub", ["setup", "preflight", "watchdog", "id", "doctor", "calibrate", "autocal", "cal-doctor", "open", "close", "goto", "drive", "home", "status"])
def test_cli_help(sub):
    out = subprocess.run([sys.executable, "-m", "fastgripper_dm.cli", sub, "-h"], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
