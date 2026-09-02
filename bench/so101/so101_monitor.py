"""Leader-only diagnostic: stream which SO-101 servo moves. No YAM involved.

Move one leader joint at a time; every 0.3s this prints the per-servo delta
from start (rad) with the biggest mover flagged. Used to derive/verify
so101_teleop.py's JOINT_MAP: wiggle one leader joint and see which index
grows.

Usage:
  bench/.venv/bin/python bench/so101/so101_monitor.py --leader_port /dev/cu.usbmodemXXXX
"""

import argparse
import time

import numpy as np
import scservo_sdk as scs

TICKS_PER_REV = 4096
PRESENT_POSITION_ADDR = 56
SO_IDS = [1, 2, 3, 4, 5, 6]

parser = argparse.ArgumentParser()
parser.add_argument("--leader_port", type=str, required=True)
parser.add_argument("--seconds", type=float, default=60.0)
args = parser.parse_args()

port = scs.PortHandler(args.leader_port)
assert port.openPort() and port.setBaudRate(1000000)
packet = scs.PacketHandler(0)
reader = scs.GroupSyncRead(port, packet, PRESENT_POSITION_ADDR, 2)
for sid in SO_IDS:
    reader.addParam(sid)


def read_all():
    if reader.txRxPacket() != scs.COMM_SUCCESS:
        raise IOError("sync read failed")
    return np.array([reader.getData(s, PRESENT_POSITION_ADDR, 2) for s in SO_IDS]) * (
        2 * np.pi / TICKS_PER_REV
    )


start = read_all()
print("servo IDs:", SO_IDS, "-- move ONE joint at a time\n")
t0 = time.monotonic()
while time.monotonic() - t0 < args.seconds:
    d = read_all() - start
    d = np.mod(d + np.pi, 2 * np.pi) - np.pi
    big = int(np.argmax(np.abs(d)))
    flag = f"  <-- servo {SO_IDS[big]} moving" if np.abs(d[big]) > 0.05 else ""
    print("delta:", np.round(d, 2), flag)
    time.sleep(0.3)
port.closePort()
print("done")
