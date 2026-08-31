import pytest
from fastgripper_dm.port import MitCommand, PortError, SPAN
from fastgripper_dm.sim import SimulatedWormGripper


def run(sim, cmd, n):
    fb = sim.read()
    for _ in range(n):
        fb = sim.command(cmd)
    return fb


def test_moves_and_wraps():
    # drive toward the OPEN stop (negative) -- the direction with >12.5 rad of room
    sim = SimulatedWormGripper(start=1.0)                 # stops at -31 / +3
    sim.enable()
    fb0 = sim.read()
    fb = run(sim, MitCommand(vel=-5.0, kd=1.0), 250)      # 250 * 20 ms = 5 s at ~5 rad/s
    assert sim.true_position < -12.5                      # crossed the window edge
    assert -12.5 <= fb.position <= 12.5                   # feedback stays wrapped
    assert fb.position == pytest.approx(((sim.true_position + 12.5) % SPAN) - 12.5, abs=1e-6)
    assert fb0.position == pytest.approx(1.0)


def test_hard_stop_stalls_the_shaft():
    sim = SimulatedWormGripper(stop_closed=3.0, start=2.0)
    sim.enable()
    run(sim, MitCommand(vel=4.0, kd=1.0), 500)
    assert sim.true_position < 3.4                        # cannot pass the stop (+ small compliance)
    fb = sim.command(MitCommand(vel=4.0, kd=1.0))
    assert abs(fb.velocity) < 0.05                        # stalled
    assert fb.torque > 0.5                                 # pushing


def test_torque_is_kd_times_velocity_error():
    sim = SimulatedWormGripper(start=0.0, friction=0.0)
    sim.enable()
    for _ in range(5):                                     # any step, any state
        fb = sim.command(MitCommand(vel=2.0, kd=0.5))
        assert fb.torque == pytest.approx(0.5 * (2.0 - fb.velocity), abs=1e-9)


def test_drop_and_fault_injection():
    sim = SimulatedWormGripper()
    sim.enable()
    sim.drop_next(2)
    with pytest.raises(PortError):
        sim.command(MitCommand())
    with pytest.raises(PortError):
        sim.command(MitCommand())
    sim.command(MitCommand())                              # third works
    sim.inject_fault(0xD)
    assert sim.read().faulted
    sim.clear_error()
    assert not sim.read().faulted


def test_disabled_motor_does_not_move():
    sim = SimulatedWormGripper(start=0.0)
    run(sim, MitCommand(vel=5.0, kd=1.0), 50)              # never enabled
    assert sim.true_position == pytest.approx(0.0)
