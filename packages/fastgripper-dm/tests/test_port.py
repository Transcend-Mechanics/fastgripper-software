from fastgripper_dm.port import FAULT_CODES, SPAN, POS_WINDOW, Feedback, MitCommand, BusDead, PortError


def test_constants():
    assert POS_WINDOW == 12.5 and SPAN == 25.0


def test_feedback_flags():
    ok = Feedback(position=0.0, velocity=0.0, torque=0.0, error_code=1, t=0.0)
    bad = Feedback(position=0.0, velocity=0.0, torque=0.0, error_code=0xD, t=0.0)
    assert ok.enabled and not ok.faulted
    assert bad.faulted and bad.fault_text == "communication loss"


def test_mitcommand_defaults_are_zero_gain():
    c = MitCommand(vel=1.0, kd=0.05)
    assert (c.pos, c.kp, c.tau) == (0.0, 0.0, 0.0)


def test_busdead_is_a_porterror():
    assert issubclass(BusDead, PortError) and FAULT_CODES[0x1] == "enabled"
