"""Print the leader's commanded gripper % live. Squeeze the trigger fully:
if the number doesn't reach ~0.0, the leader calibration is the culprit."""

import json
import pathlib
import time

from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

cfg = json.loads((pathlib.Path.home() / ".config/fastgripper/config.json").read_text())
leader = SOLeader(SOLeaderTeleopConfig(port=cfg["leader_port"], id=cfg["leader_id"]))
leader.connect(calibrate=False)
print("Squeeze the trigger fully closed, then fully open. Ctrl-C to stop.")
try:
    while True:
        print(f"\rleader gripper.pos = {leader.get_action()['gripper.pos']:6.2f} %", end="")
        time.sleep(0.05)
except KeyboardInterrupt:
    print()
finally:
    leader.disconnect()
