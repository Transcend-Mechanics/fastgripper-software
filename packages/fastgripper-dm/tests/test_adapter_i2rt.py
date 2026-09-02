import json

import numpy as np
import pytest

from fastgripper_dm.adapters.i2rt import I2rtGripper, to_i2rt_command
from fastgripper_dm.port import MitCommand, POS_WINDOW, SPAN


class FakeYamRobot:
    """The MotorChainRobot surface the adapter is allowed to touch, plus
    write methods that record abuse."""

    def __init__(self, n=7, grip_norm=0.5, grip_vel_norm=0.0, grip_eff=0.0):
        self.n = n
        self.grip_norm = grip_norm
        self.grip_vel_norm = grip_vel_norm
        self.grip_eff = grip_eff
        self.writes = []

    def get_joint_pos(self):
        out = np.zeros(self.n)
        out[-1] = self.grip_norm
        return out

    def get_observations(self):
        return {"joint_pos": np.zeros(self.n - 1), "joint_vel": np.zeros(self.n - 1),
                "joint_eff": np.zeros(self.n - 1),
                "gripper_vel": np.array([self.grip_vel_norm]),
                "gripper_eff": np.array([self.grip_eff])}

    def get_robot_info(self):
        return {"gripper_limits": (-POS_WINDOW, POS_WINDOW)}

    def command_joint_state(self, *a, **k):
        self.writes.append(("command_joint_state", a, k))

    def set_commands(self, *a, **k):
        self.writes.append(("set_commands", a, k))


def entry_file(tmp_path, entry):
    p = tmp_path / "gripper_cal.json"
    p.write_text(json.dumps({"format": 2, "grippers": {"yam": entry}}))
    return str(p)


ENTRY = {"open": -30.0, "closed": 2.0, "last_position": 1.0, "last_wrapped": 1.0}


def connected(tmp_path, grip_norm=None, entry=None):
    entry = dict(entry or ENTRY)
    # choose the robot's normalized position to match last_wrapped exactly:
    # norm = (wrapped + POS_WINDOW) / SPAN
    if grip_norm is None:
        grip_norm = (entry["last_wrapped"] + POS_WINDOW) / SPAN
    robot = FakeYamRobot(grip_norm=grip_norm)
    g = I2rtGripper(robot, joint_index=6, gripper="yam", cal_path=entry_file(tmp_path, entry))
    g.connect()
    return robot, g


def test_feedback_scaling_round_trip(tmp_path):
    robot, g = connected(tmp_path)
    robot.grip_vel_norm = 0.2                      # normalized
    robot.grip_eff = 0.7                           # already real Nm
    g.tick(0.02)
    fb = g.last_feedback
    assert g.position == pytest.approx(1.0, abs=1e-6)          # adopted park exactly
    assert fb.velocity == pytest.approx(0.2 * SPAN)            # ×SPAN read scaling (5.0 rad/s)
    assert fb.torque == pytest.approx(0.7)                     # eff is already real Nm
    assert fb.position == pytest.approx(1.0, abs=1e-6)         # ×SPAN − POS_WINDOW
    assert g.velocity == pytest.approx(5.0) and g.torque == pytest.approx(0.7)


def test_connect_rejects_wrong_gripper_limits(tmp_path):
    robot = FakeYamRobot(grip_norm=(ENTRY["last_wrapped"] + POS_WINDOW) / SPAN)
    robot.get_robot_info = lambda: {"gripper_limits": (0.0, 3.66)}   # arm default: wrong
    g = I2rtGripper(robot, joint_index=6, gripper="yam", cal_path=entry_file(tmp_path, dict(ENTRY)))
    with pytest.raises(ValueError, match="gripper_limits"):
        g.connect()


def test_to_i2rt_command_divides_vel_by_span(tmp_path):
    cmd = MitCommand(vel=5.0, kd=0.0833)
    pos_n, vel_n, kp, kd = to_i2rt_command(cmd, pos_placeholder_norm=0.4)
    assert vel_n == pytest.approx(5.0 / SPAN)
    assert pos_n == 0.4 and kp == 0.0 and kd == pytest.approx(0.0833)


def test_adapter_never_writes_to_the_robot(tmp_path):
    robot, g = connected(tmp_path)
    g.open()
    for _ in range(50):
        g.command_tuple(0.02)
    assert robot.writes == []


def test_connect_refuses_park_mismatch(tmp_path):
    # robot sits 3 rad (wrapped) away from last_wrapped -> no silent adoption
    with pytest.raises(ValueError, match="park"):
        connected(tmp_path, grip_norm=(4.0 + POS_WINDOW) / SPAN)


def test_park_persists(tmp_path):
    entry = dict(ENTRY)
    robot, g = connected(tmp_path)
    g.close()
    g.tick(0.02)
    g.park()
    saved = json.loads(open(g.cal_path).read())["grippers"]["yam"]
    assert saved["last_position"] == pytest.approx(g.position)
    assert "parked_at" in saved


def test_lazy_import_message():
    # importing the module must not require i2rt; only robot-side helpers may
    import fastgripper_dm.adapters.i2rt as m
    assert hasattr(m, "I2rtGripper")


def test_home_off_does_not_poison_the_cal_store(tmp_path, capsys):
    # home="off" never anchors the multi-turn tracker, so park()'s
    # last_position would be an arbitrary frame. Adopting it next session can
    # drive the worm into a hard stop at the full torque cap -- so park() must
    # refuse to save it.
    path = entry_file(tmp_path, dict(ENTRY))
    before = open(path).read()
    robot = FakeYamRobot(grip_norm=(ENTRY["last_wrapped"] + POS_WINDOW) / SPAN)
    g = I2rtGripper(robot, joint_index=6, gripper="yam", cal_path=path)
    g.connect(home="off")
    g.tick(0.02)
    g.park()
    assert open(path).read() == before          # cal store untouched
    assert "not anchored" in capsys.readouterr().out


def test_home_auto_still_persists_park(tmp_path):
    robot, g = connected(tmp_path)              # home="auto" -> adopt_park
    g.tick(0.02)
    g.park()
    saved = json.loads(open(g.cal_path).read())["grippers"]["yam"]
    assert saved["last_position"] == pytest.approx(g.position)
