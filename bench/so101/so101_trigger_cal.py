"""One-time SO-101 trigger calibration for absolute gripper mapping.

Records the trigger's RELEASED and SQUEEZED positions so teleop can map
trigger position -> jaw position absolutely (released = open, squeezed =
closed). After this, gripper direction is correct by definition and there is
no start-position linking ritual.

Run interactively:
  bench/.venv/bin/python bench/so101/so101_trigger_cal.py \
      --leader_port /dev/cu.usbmodemXXXX

Writes bench/yam/so101_trigger_cal.json (the file bench/yam/so101_teleop.py
reads by default) unless --out_file overrides it.
"""

import argparse
import json
import os
import time

import numpy as np
import scservo_sdk as scs

TICKS_PER_REV = 4096
PRESENT_POSITION_ADDR = 56
TRIGGER_ID = 6
DEFAULT_OUT_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "yam", "so101_trigger_cal.json")

parser = argparse.ArgumentParser()
parser.add_argument("--leader_port", type=str, required=True)
parser.add_argument("--out_file", type=str, default=DEFAULT_OUT_FILE)
args = parser.parse_args()

port = scs.PortHandler(args.leader_port)
assert port.openPort() and port.setBaudRate(1000000)
packet = scs.PacketHandler(0)


def read_trigger() -> float:
    for _ in range(20):
        pos, res, err = packet.read2ByteTxRx(port, TRIGGER_ID, PRESENT_POSITION_ADDR)
        if res == scs.COMM_SUCCESS:
            return pos * (2 * np.pi / TICKS_PER_REV)
        time.sleep(0.02)
    raise SystemExit("cannot read trigger servo")


def settle_read(label: str) -> float:
    input(f"{label} the trigger fully, hold it there, then press Enter... ")
    vals = [read_trigger() for _ in range(10)]
    v = float(np.median(vals))
    print(f"  recorded {v:+.3f} rad")
    return v


released = settle_read("RELEASE")
squeezed = settle_read("SQUEEZE")
if abs(squeezed - released) < 0.1:
    raise SystemExit("trigger barely moved between the two reads -- try again")

with open(args.out_file, "w") as f:
    json.dump({"released": released, "squeezed": squeezed,
               "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)
print(f"saved {args.out_file} (travel {abs(squeezed - released):.2f} rad)")
port.closePort()
