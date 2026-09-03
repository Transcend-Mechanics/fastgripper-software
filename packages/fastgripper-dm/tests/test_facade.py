import json
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from fastgripper_dm.facade import FastGripper, HomingError
from fastgripper_dm.port import POS_WINDOW, SPAN, Feedback, PortError
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
    # jaws "moved by hand": sim starts 6 rad away from the park, past the
    # 3.0 rad park_tolerance_rad
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=-5.0)
    g.connect()                                  # falls back to homing against stop_closed=3.0
    # after homing, the tracker frame must agree with ground truth at the stop datum
    g.goto(0.0)                                   # close to the mark (2.0)
    g.wait(timeout=8.0)
    # 0.5 rad, not 0.3: homing anchors on the stop_closed datum, and the 0.1.1
    # probe (probe_vel 2.0) penetrates the sim's 0.35-rad-wide compliant stop to
    # its clamp before |velocity| drops, so the adopted frame sits ~0.35 rad from
    # ground truth. On a rigid hardware stop the probe stalls at the datum itself.
    assert sim.true_position == pytest.approx(2.0, abs=0.5)


def test_homing_guard_rejects_a_datum_the_probe_cannot_reach(tmp_path):
    # Nonsense datum: the probe stops 11.65 rad (folded) from where stop_closed
    # claims the stop is, well past park_tolerance_rad -> the friction guard fires.
    entry = dict(ENTRY)
    entry["stop_closed"] = 90.0
    sim, g = sim_gripper(tmp_path, entry, start=-5.0)
    with pytest.raises(HomingError, match="probably friction"):
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
    sim, g = sim_gripper(tmp_path, entry, start=-5.0)
    with pytest.raises(HomingError):
        g.connect()
    assert sim.read().error_code == 0


# --- 0.1.1: bus shutdown, park drift, re-home aliasing, park guard ----------

class FakePort:
    """MotorPort that reports a fixed wrapped position and never moves.

    `.motor.bus` is a MagicMock so the teardown paths' `bus.shutdown()` is
    observable, exactly as DamiaoCanPort exposes the python-can bus."""

    pos_window = POS_WINDOW

    def __init__(self, wrapped=0.0, fail_read=False, fail_enable=False, fail_disable=False):
        self.wrapped = wrapped
        self.fail_read = fail_read
        self.fail_enable = fail_enable
        self.fail_disable = fail_disable
        self.enabled = False
        self.closed = False
        self.motor = SimpleNamespace(bus=MagicMock())

    def enable(self):
        if self.fail_enable:
            raise PortError("simulated enable failure")
        self.enabled = True

    def disable(self):
        self.enabled = False
        if self.fail_disable:
            raise PortError("simulated disable failure")

    def clear_error(self):
        pass

    def close(self):
        self.closed = True

    def _fb(self):
        return Feedback(position=self.wrapped, velocity=0.0, torque=0.0,
                        error_code=1 if self.enabled else 0, t=time.monotonic())

    def read(self):
        if self.fail_read:
            raise PortError("simulated read failure")
        return self._fb()

    def command(self, cmd):
        return self._fb()


class AliasPort(FakePort):
    """Moves toward a hard stop under commanded velocity, reporting WRAPPED
    positions -- so a true position outside +/-12.5 rad makes the probe's
    tracker frame differ from the entry frame by a whole SPAN."""

    STEP = 0.02  # s of simulated travel per command (the facade's loop period)

    def __init__(self, start, stop_true):
        super().__init__()
        self.true = start
        self.stop_true = stop_true
        self.velocity = 0.0

    def _wrap(self, x):
        return ((x + POS_WINDOW) % SPAN) - POS_WINDOW

    def _fb(self):
        return Feedback(position=self._wrap(self.true), velocity=self.velocity,
                        torque=0.0, error_code=1, t=time.monotonic())

    def command(self, cmd):
        target = self.true + cmd.vel * self.STEP
        target = min(self.stop_true, target)
        self.velocity = (target - self.true) / self.STEP
        self.true = target
        return self._fb()


def auto_gripper(tmp_path, entry, wrapped, **kw):
    port = FakePort(wrapped=wrapped)
    g = FastGripper(port=port, cal_path=make_store(tmp_path, entry),
                    gripper="default", **kw)
    return port, g


def test_disconnect_shuts_the_bus_down_once(tmp_path):
    port, g = auto_gripper(tmp_path, dict(ENTRY), wrapped=1.0)
    g.connect()
    g.disconnect()
    assert port.closed
    assert port.motor.bus.shutdown.call_count == 1


def test_failed_connect_shuts_the_bus_down_once(tmp_path):
    port = FakePort(wrapped=1.0, fail_read=True)
    g = FastGripper(port=port, cal_path=make_store(tmp_path, dict(ENTRY)), gripper="default")
    with pytest.raises(PortError):
        g.connect()
    assert not port.enabled                       # motor disabled on the failure path
    assert port.motor.bus.shutdown.call_count == 1


def test_failed_enable_shuts_the_bus_down_once(tmp_path):
    # enable() sits inside connect()'s teardown guard: a CanError there must not
    # escape with the bus left to be collected at interpreter exit.
    port = FakePort(wrapped=1.0, fail_enable=True)
    g = FastGripper(port=port, cal_path=make_store(tmp_path, dict(ENTRY)), gripper="default")
    with pytest.raises(PortError, match="enable"):
        g.connect()
    assert port.closed
    assert port.motor.bus.shutdown.call_count == 1


def test_teardown_shuts_the_bus_down_even_when_disable_raises(tmp_path):
    port = FakePort(wrapped=1.0, fail_read=True, fail_disable=True)
    g = FastGripper(port=port, cal_path=make_store(tmp_path, dict(ENTRY)), gripper="default")
    with pytest.raises(PortError):
        g.connect()
    assert port.closed                            # close() still ran
    assert port.motor.bus.shutdown.call_count == 1


def test_teardown_shuts_down_a_bus_opened_before_the_port_existed(tmp_path):
    # open_bus() succeeded but the port construction failed: self.port is still
    # None, and the only handle on the bus is the one connect() stashed.
    g = FastGripper(cal_path=make_store(tmp_path, dict(ENTRY)), gripper="default")
    bus = MagicMock()
    g._own_bus = bus
    g._safe_disable_close()                       # must not raise on port=None
    assert bus.shutdown.call_count == 1


def test_shutdown_bus_tolerates_a_port_without_one(tmp_path):
    # SimulatedWormGripper has no .motor: teardown must not raise.
    sim, g = sim_gripper(tmp_path, dict(ENTRY), start=1.0)
    g.connect()
    g.disconnect()


def test_shutdown_bus_swallows_a_dead_bus(tmp_path):
    port, g = auto_gripper(tmp_path, dict(ENTRY), wrapped=1.0)
    port.motor.bus.shutdown.side_effect = RuntimeError("libusb is gone")
    g.connect()
    g.disconnect()                                # must not propagate
    assert port.motor.bus.shutdown.call_count == 1


@pytest.mark.parametrize("drift", [1.7, -1.7])
def test_auto_adopts_the_park_corrected_by_drift(tmp_path, drift):
    # The mechanism relaxes while disabled: boot sits `drift` rad off the saved
    # wrapped park, so the adopted multi-turn position must move with it.
    entry = dict(ENTRY)
    port, g = auto_gripper(tmp_path, entry, wrapped=entry["last_wrapped"] + drift)
    g.connect()
    assert g.position == pytest.approx(entry["last_position"] + drift, abs=1e-6)


def test_auto_drift_folds_across_the_wrap(tmp_path):
    # lw and boot straddle the +/-12.5 window edge: the raw difference is -24.8,
    # the physical drift is +0.2.
    entry = dict(ENTRY, last_wrapped=12.4, last_position=12.4)
    port, g = auto_gripper(tmp_path, entry, wrapped=-12.4)
    g.connect()
    assert g.position == pytest.approx(12.6, abs=1e-6)


def test_rehome_accepts_a_whole_span_alias(tmp_path):
    # True position -14 wraps to +11, so the probe's tracker frame is one SPAN
    # away from the entry frame: the raw re-anchor offset is ~-25 rad. Before the
    # fold, this aborted with "exceeds ~3 turns" with the jaws AT the stop.
    # probe_vel is raised only to keep the 17 rad of travel quick in the test
    entry = dict(ENTRY, profile={"probe_vel": 20.0})
    port = AliasPort(start=-14.0, stop_true=3.0)
    g = FastGripper(port=port, cal_path=make_store(tmp_path, entry),
                    gripper="default", home="stall")
    g.connect()                                   # must not raise HomingError
    # Anchored on the datum itself, then backed off by margin (0.75), clamped by
    # goto_rad to the calibrated closed mark (2.0). The frame must now agree with
    # ground truth -- an unfolded alias would leave it 25 rad out.
    assert g.position == pytest.approx(entry["closed"], abs=0.05)
    assert port.true == pytest.approx(g.position, abs=0.05)


def test_rehome_still_rejects_a_probe_that_stopped_on_friction(tmp_path):
    # Probe stalls 5 rad short of the datum (friction, not the stop). 5 rad is
    # inside one window, so only the folded-offset guard can catch it.
    entry = dict(ENTRY, profile={"probe_vel": 20.0})
    port = AliasPort(start=-8.0, stop_true=-2.0)   # real stop 5 rad short of 3.0
    g = FastGripper(port=port, cal_path=make_store(tmp_path, entry),
                    gripper="default", home="stall")
    with pytest.raises(HomingError, match="probably friction"):
        g.connect()


def test_park_saves_nothing_when_the_tracker_is_unanchored(tmp_path):
    cal = make_store(tmp_path, dict(ENTRY))
    port = FakePort(wrapped=1.0)
    g = FastGripper(port=port, cal_path=cal, gripper="default", home="off")
    g.connect()
    before = json.load(open(cal))
    g.disconnect()
    assert json.load(open(cal)) == before         # no last_position, no parked_at
    assert port.motor.bus.shutdown.call_count == 1


def test_park_saves_when_the_park_was_adopted(tmp_path):
    cal = make_store(tmp_path, dict(ENTRY))
    port = FakePort(wrapped=1.0)
    g = FastGripper(port=port, cal_path=cal, gripper="default", home="auto")
    g.connect()
    g.disconnect()
    saved = json.load(open(cal))["grippers"]["default"]
    assert saved["last_position"] == pytest.approx(1.0, abs=1e-6)
    assert "parked_at" in saved
