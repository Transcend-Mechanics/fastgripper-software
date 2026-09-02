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


def test_per_tick_regoto_does_not_reset_stall():
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for _ in range(int(4.0 / 0.02)):
        c.goto_frac(0.0)                      # trigger held: re-goto EVERY tick
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled                          # latch must still engage
    assert sim.max_abs_torque <= 2.0 + 1e-6


def test_retreating_goal_releases_stall():
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for _ in range(int(4.0 / 0.02)):
        c.goto_frac(0.0)
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled
    stall_pos = c.position
    c.goto_frac(0.0)                          # same squeeze: stays latched
    assert c.stalled
    c.goto_frac(1.0)                          # trigger released: unlatch
    assert not c.stalled
    for _ in range(int(4.0 / 0.02)):
        fb = sim.command(c.tick(fb, 0.02))
    # ENTRY open=-30.0, closed=2.0; retreat (toward open) means position DECREASES
    assert c.position < stall_pos - 0.5       # moved away from the stall


def stalled_on_object():
    """A controller latched on an object at -5.0, closing toward +1.0."""
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for _ in range(int(4.0 / 0.02)):
        c.goto_rad(1.0)
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled
    return sim, c, fb


def test_stall_latches_despite_per_tick_goal_jitter():
    # The live trigger quantizes to ~0.027 rad/tick and dithers; under the old
    # contract every jittered goto reset the 0.4 s stall timer, so the latch
    # never engaged and the motor kept pushing at the 2.0 Nm cap.
    sim = SimulatedWormGripper(stop_open=-31.0, stop_closed=-5.0, start=-10.0)
    sim.enable()
    c = GripperController(dict(ENTRY), GripperProfile())
    c.adopt_park(-10.0)
    fb = sim.read()
    for i in range(int(4.0 / 0.02)):
        c.goto_rad(1.0 + (0.03 if i % 2 else -0.03))   # +/-0.03 rad jitter
        fb = sim.command(c.tick(fb, 0.02))
    assert c.stalled
    assert sim.max_abs_torque <= 2.0 + 1e-6


def test_deeper_goal_while_latched_stays_latched():
    _, c, _ = stalled_on_object()
    c.goto_rad(1.0 + 0.5)          # squeeze HARDER (toward the stall): no unlatch
    assert c.stalled
    c.goto_rad(2.0)                # all the way to the closed mark: still no unlatch
    assert c.stalled


def test_retreat_of_one_eps_unlatches():
    _, c, _ = stalled_on_object()
    c.goto_rad(1.0 - GripperController.RELEASE_EPS)
    assert not c.stalled
    assert c.goal == pytest.approx(0.95)


def test_slow_staircase_retreat_eventually_unlatches():
    # Trigger released one encoder tick (0.027 rad) at a time: each step alone
    # is under RELEASE_EPS, but retreat is measured from the goal in effect at
    # the latch, so the cumulative backoff releases it.
    _, c, _ = stalled_on_object()
    goal = 1.0
    for _ in range(2):
        goal -= 0.027
        c.goto_rad(goal)
    assert not c.stalled            # cumulative retreat 0.054 >= 0.05
    assert c.goal == pytest.approx(goal)


def test_one_staircase_step_is_not_enough():
    _, c, _ = stalled_on_object()
    c.goto_rad(1.0 - 0.027)
    assert c.stalled
