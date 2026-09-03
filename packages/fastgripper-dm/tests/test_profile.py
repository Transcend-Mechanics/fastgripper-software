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


def test_defaults_carry_the_measured_probe_caps():
    # 2026-09-02 bench: free-run friction p95 0.36-0.44 Nm, relaxation after a
    # hard close 1.7-1.8 rad.
    p = GripperProfile()
    assert p.contact_torque == 0.8 and p.probe_tmax == 1.0 and p.probe_vel == 2.0
    assert p.park_tolerance_rad == 3.0
    p.validate()


def test_validate_refuses_a_probe_that_cannot_beat_its_own_threshold():
    with pytest.raises(ValueError, match="probe_tmax"):
        GripperProfile(contact_torque=1.2, probe_tmax=1.0).validate()
    with pytest.raises(ValueError, match="probe_tmax"):
        GripperProfile(contact_torque=1.0, probe_tmax=1.0).validate()   # equal is not enough
    with pytest.raises(ValueError, match="probe_tmax"):
        GripperProfile(contact_torque=0.5, probe_tmax=TMAX_CAP + 0.1).validate()
    GripperProfile(contact_torque=0.5, probe_tmax=TMAX_CAP).validate()


def test_presets_validate():
    for name, p in PRESETS.items():
        p.validate()
