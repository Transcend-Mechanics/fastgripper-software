import pytest
from fastgripper_dm.controller import GripperController
from fastgripper_dm.port import MitCommand
from fastgripper_dm.profile import GripperProfile
from fastgripper_dm.sim import SimulatedWormGripper

ENTRY = {"open": -30.0, "closed": 2.0}


def make(sim=None, **prof):
    sim = sim or SimulatedWormGripper(stop_open=-31.0, stop_closed=3.0, start=1.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile(**prof))
    c.adopt_park(1.0)          # sim starts at 1.0; trust it exactly
    return sim, c


def run(sim, c, seconds):
    fb = sim.read()
    for _ in range(int(seconds / 0.02)):
        fb = sim.command(c.tick(fb, 0.02))
    return fb


def test_reaches_a_goal_without_exceeding_the_torque_cap():
    sim, c = make()
    c.goto_frac(1.0)                        # fully open = -30.0
    run(sim, c, 6.0)
    assert c.position == pytest.approx(-30.0, abs=0.1)
    assert c.at_goal
    assert sim.max_abs_torque <= 2.0 + 1e-6   # THE cap assertion


def test_frac_mapping():
    sim, c = make()
    c.goto_frac(0.0)
    assert c.goal == pytest.approx(2.0)      # 0 = closed mark
    c.goto_frac(0.5)
    assert c.goal == pytest.approx(-14.0)


def test_goal_is_clamped_to_marks():
    _, c = make()
    c.goto_rad(+10.0)
    assert c.goal == pytest.approx(2.0)
    c.goto_rad(-99.0)
    assert c.goal == pytest.approx(-30.0)


def test_stall_on_obstruction_holds():
    # closed stop at 3.0 but the mark is 2.0; put a virtual object by moving the stop inward
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    c.close()                                # goal 2.0, but the "object" is at -5
    run(sim, c, 4.0)
    assert c.stalled
    assert c.position == pytest.approx(-5.0, abs=0.6)
    assert sim.max_abs_torque <= 2.0 + 1e-6


def test_tick_is_pure_and_survives_fault_feedback():
    sim, c = make()
    c.open()
    fb = sim.read()
    a = c.tick(fb, 0.02)
    # rebuild an identical controller: same inputs -> same command
    c2 = GripperController(dict(ENTRY), GripperProfile())
    c2.adopt_park(1.0)
    c2.open()
    assert c2.tick(fb, 0.02) == a
    sim.inject_fault(0xD)
    cmd = c.tick(sim.read(), 0.02)
    assert cmd == MitCommand()               # zero everything while faulted


def test_profile_validation_at_construction():
    with pytest.raises(ValueError):
        GripperController(dict(ENTRY), GripperProfile(tmax_nm=9.0))
    with pytest.raises(ValueError):
        GripperController({}, GripperProfile())
