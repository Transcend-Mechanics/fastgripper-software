import subprocess
import sys

import pytest


@pytest.mark.parametrize("sub", ["open", "close", "goto", "home", "status", "drive"])
def test_verbs_parse(sub):
    argv = [sys.executable, "-m", "fastgripper_dm.cli", sub, "-h"]
    out = subprocess.run(argv, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr


def test_goto_requires_a_percentage():
    out = subprocess.run([sys.executable, "-m", "fastgripper_dm.cli", "goto"],
                        capture_output=True, text=True)
    assert out.returncode != 0


# --- 0.1.1: teardown paths --------------------------------------------------

def test_run_catches_homing_error_and_exits_1(monkeypatch):
    from fastgripper_dm.facade import HomingError
    from fastgripper_dm.tools import _cli

    codes = []
    monkeypatch.setattr(_cli.os, "_exit", codes.append)

    def boom():
        raise HomingError("no contact within 90 s")

    _cli.run(boom)
    assert codes == [1]


def test_run_still_exits_0_on_success(monkeypatch):
    from fastgripper_dm.tools import _cli

    codes = []
    monkeypatch.setattr(_cli.os, "_exit", codes.append)
    _cli.run(lambda: None)
    assert codes == [0]


def test_cmd_status_shuts_the_bus_down_without_saving_a_park(tmp_path, monkeypatch, capsys):
    """`status` uses home='off', so it must NOT go through disconnect() (which
    would save a park from an unanchored frame) but MUST still shut the bus down."""
    import json
    from argparse import Namespace

    from fastgripper_dm import cli as cli_mod
    from fastgripper_dm.facade import FastGripper
    from fastgripper_dm.tools import _cli
    from test_facade import ENTRY, FakePort   # same test dir; pytest prepends it to sys.path

    cal = tmp_path / "gripper_cal.json"
    cal.write_text(json.dumps({"format": 2, "grippers": {"default": dict(ENTRY)}}))
    port = FakePort(wrapped=1.0)

    def fake_standalone(cls, **kw):
        return FastGripper(port=port, cal_path=str(cal), gripper="default", home=kw.get("home"))

    monkeypatch.setattr(FastGripper, "standalone", classmethod(fake_standalone))
    codes = []
    monkeypatch.setattr(_cli.os, "_exit", codes.append)

    before = json.loads(cal.read_text())
    cli_mod.cmd_status(Namespace(gripper=None, cal=str(cal)), [])

    assert codes == [0]
    assert not port.enabled and port.closed
    assert port.motor.bus.shutdown.call_count == 1
    assert json.loads(cal.read_text()) == before      # no park written from 'off'
    assert "no absolute frame" in capsys.readouterr().out
