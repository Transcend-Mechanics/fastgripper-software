import pytest
from fastgripper_dm.calstore import entry_profile
from fastgripper_dm.profile import PRESETS, TMAX_CAP, GripperProfile


def test_defaults_describe_a_two_hardstop_gripper():
    p = GripperProfile()
    assert not p.closed_only and not p.single_touch and p.tmax_nm == 2.0 and p.watchdog_ms == 8000


def test_presets():
    assert PRESETS["yam"].span_from_closed == 33.5
    assert PRESETS["ur"].closed_only and PRESETS["ur"].single_touch and PRESETS["ur"].span_from_closed == 30.0


def test_roundtrip_ignores_unknown_keys():
    p = GripperProfile.from_dict({"tmax_nm": 1.5, "someday_field": 1})
    assert p.tmax_nm == 1.5
    assert GripperProfile.from_dict(p.to_dict()) == p


def test_validate_refuses_tmax_above_cap():
    with pytest.raises(ValueError, match="tmax_nm"):
        GripperProfile(tmax_nm=TMAX_CAP + 0.1).validate()
    GripperProfile(tmax_nm=TMAX_CAP).validate()


def test_entry_profile_merges_over_defaults():
    p = entry_profile({"profile": {"span_from_closed": 33.5}})
    assert p.span_from_closed == 33.5 and p.tmax_nm == 2.0
    assert entry_profile({}) == GripperProfile()
