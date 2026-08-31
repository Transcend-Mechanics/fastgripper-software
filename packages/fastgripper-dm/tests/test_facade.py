import json
import pytest
from fastgripper_dm.facade import FastGripper, HomingError
from fastgripper_dm.sim import SimulatedWormGripper


def make_store(tmp_path, entry):
    p = tmp_path / "gripper_cal.json"
    p.write_text(json.dumps({"format": 2, "grippers": {"default": entry}}))
    return str(p)


def sim_gripper(tmp_path, entry, start=1.0, home="auto"):
    cal = make_store(tmp_path, entry)
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=start)
    g = FastGripper(port=sim, cal_path=cal, gripper="default", home=home)  # test ctor: inject port
    return sim, g


ENTRY = {"open": -30.0, "closed": 2.0, "stop_closed": 3.0,
         "last_position": 1.0, "last_wrapped": 1.0}


def test_auto_restores_when_park_matches(tmp_path):
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=1.0)
    g.connect()
    assert g.position == pytest.approx(1.0, abs=0.05)


def test_auto_stall_homes_on_mismatch(tmp_path):
    # jaws "moved by hand": sim starts 2 rad away from the park
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=-1.0)
    g.connect()                                  # falls back to homing against stop_closed=3.0
    # after homing, the tracker frame must agree with ground truth at the stop datum
    g.goto(0.0)                                   # close to the mark (2.0)
    g.wait(timeout=8.0)
    assert sim.true_position == pytest.approx(2.0, abs=0.3)


def test_homing_guard_rejects_absurd_reanchor(tmp_path):
    entry = dict(ENTRY)
    entry["stop_closed"] = 90.0                   # nonsense datum -> re-anchor > 3 turns
    sim, g = sim_gripper(tmp_path, entry, start=-1.0)
    with pytest.raises(HomingError):
        g.connect()


def test_park_persists_and_next_auto_restores(tmp_path):
    cal = make_store(tmp_path, dict(ENTRY))
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=1.0)
    with FastGripper(port=sim, cal_path=cal, gripper="default") as g:
        g.goto(0.5)
        g.wait(timeout=8.0)
    saved = json.load(open(cal))["grippers"]["default"]
    assert saved["last_position"] == pytest.approx(sim.true_position, abs=0.05)
    # second session, same sim position: auto restore, no homing motion
    pos_before = sim.true_position
    g2 = FastGripper(port=sim, cal_path=cal, gripper="default")
    g2.connect()
    assert sim.true_position == pytest.approx(pos_before, abs=0.05)
    assert g2.position == pytest.approx(saved["last_position"], abs=0.05)


def test_missing_marks_raise(tmp_path):
    sim, g = sim_gripper(tmp_path, {"stop_closed": 3.0}, start=1.0)
    with pytest.raises(ValueError):
        g.connect()


def test_failed_connect_leaves_port_disabled(tmp_path):
    # nonsense stop_closed datum -> home_against_stop raises HomingError after
    # the port has already been enabled; the motor must not be left enabled.
    entry = dict(ENTRY)
    entry["stop_closed"] = 90.0
    sim, g = sim_gripper(tmp_path, entry, start=-1.0)
    with pytest.raises(HomingError):
        g.connect()
    assert sim.read().error_code == 0
