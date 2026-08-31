import pytest
from fastgripper_dm.port import SPAN
from fastgripper_dm.tracker import MultiTurnTracker


def test_first_update_defines_position():
    t = MultiTurnTracker()
    assert t.update(3.0) == 3.0 and t.position == 3.0 and t.wrapped == 3.0


def test_wrap_forward_and_back():
    t = MultiTurnTracker()
    t.update(12.0)
    assert t.update(-12.0) == pytest.approx(13.0)      # jumped down by 24 -> one wrap up
    assert t.update(-11.0) == pytest.approx(14.0)
    assert t.update(12.0) == pytest.approx(12.0)       # jumped up by 23 -> one wrap down


def test_small_moves_never_wrap():
    t = MultiTurnTracker()
    t.update(0.0)
    for x in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0):
        t.update(x)
    assert t.position == pytest.approx(12.0)


def test_exact_park_adoption():
    t = MultiTurnTracker(start_unwrapped=-22.16)
    assert t.update(2.84) == pytest.approx(-22.16)     # offset = -25.0 exactly, no rounding
    t2 = MultiTurnTracker(start_unwrapped=-20.0)
    assert t2.update(2.84) == pytest.approx(-20.0)     # NOT rounded to a window multiple


def test_anchor_shifts_frame():
    t = MultiTurnTracker()
    t.update(1.0)
    t.anchor(31.0)
    assert t.position == pytest.approx(31.0)
    t.update(1.5)
    assert t.position == pytest.approx(31.5)


def test_position_before_update_raises():
    t = MultiTurnTracker()
    assert not t.seen
    with pytest.raises(RuntimeError):
        _ = t.position
