"""Joint roll-call: wiggle each YAM joint one at a time so a human can
identify what each index physically does. Small sine (+/-0.1 rad), 4s per
joint, returns to start pose between joints.

Usage:
  I2RT_CAN_BUSTYPE=gs_usb .venv/bin/python -u bench/tools/wiggle_joints.py --channel 0
"""

import argparse
import time

import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import GripperType

parser = argparse.ArgumentParser()
parser.add_argument("--channel", type=str, default="0")
parser.add_argument("--amplitude", type=float, default=0.1)
parser.add_argument("--period", type=float, default=2.0)
parser.add_argument("--cycles", type=int, default=2)
args = parser.parse_args()

robot = get_yam_robot(
    channel=args.channel,
    gripper_type=GripperType.NO_GRIPPER,
    zero_gravity_mode=False,
    ee_mass=0.4,
)
start = robot.get_joint_pos().copy()
n = robot.num_dofs()
print(f"\n{n} dofs, holding at", np.round(start, 3))
time.sleep(1.0)

for j in range(n):
    print(f"\n>>> JOINT {j + 1} moving now -- watch the arm <<<")
    t0 = time.monotonic()
    dur = args.period * args.cycles
    while (t := time.monotonic() - t0) < dur:
        target = start.copy()
        target[j] += args.amplitude * np.sin(2 * np.pi * t / args.period)
        robot.command_joint_pos(target)
        time.sleep(0.02)
    robot.command_joint_pos(start)
    time.sleep(1.0)

print("\nroll-call complete, shutting down")
robot.close()
print("done")
