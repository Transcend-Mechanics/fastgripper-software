"""Does follower MOTION knock the leader's USB serial device off the bus?

Reads the leader at 60 Hz the whole time (like teleop). Phase 1: follower
idle (10 s). Phase 2: follower wrist_roll (id 5) wiggles +/-WIGGLE ticks
around its current position (20 s). Phase 3: idle again (10 s). Reports
the exact moment the leader read fails / the device node vanishes.

Make sure the follower arm is clear of obstacles before running.
Usage: python bench/so101/leader_drop_probe.py   (uses ~/.config/fastgripper/config.json)
"""

import json
import os
import pathlib
import threading
import time

import scservo_sdk as scs

cfg = json.loads((pathlib.Path.home() / ".config/fastgripper/config.json").read_text())
LEADER = cfg["leader_port"].replace("/dev/tty.", "/dev/cu.")
FOLLOWER = cfg["follower_port"].replace("/dev/tty.", "/dev/cu.")
WIGGLE = 120          # ticks (~10 deg)
MOTOR = 5             # wrist_roll
ADDR_TORQUE, ADDR_GOAL, ADDR_POS = 40, 42, 56

phase = {"name": "idle-before"}
t0 = time.time()
events = []


def leader_reader():
    ph = scs.PortHandler(LEADER)
    ph.openPort()
    ph.setBaudRate(1000000)
    pk = scs.PacketHandler(0)
    rd = scs.GroupSyncRead(ph, pk, ADDR_POS, 2)
    for i in range(1, 7):
        rd.addParam(i)
    ok = 0
    try:
        while phase["name"] != "done":
            r = rd.txRxPacket()
            if r == scs.COMM_SUCCESS:
                ok += 1
            else:
                events.append(f"t={time.time()-t0:6.2f}s [{phase['name']}] leader soft fail: "
                              f"{pk.getTxRxResult(r)} node_exists={os.path.exists(LEADER)}")
                if not os.path.exists(LEADER):
                    events.append(f"t={time.time()-t0:6.2f}s [{phase['name']}] LEADER DEVICE VANISHED")
                    break
            time.sleep(1 / 60)
    except Exception as e:
        events.append(f"t={time.time()-t0:6.2f}s [{phase['name']}] LEADER EXCEPTION: {str(e)[:80]}")
    events.append(f"leader clean reads: {ok}")
    try:
        ph.closePort()
    except Exception:
        pass


th = threading.Thread(target=leader_reader, daemon=True)
th.start()

fp = scs.PortHandler(FOLLOWER)
fp.openPort()
fp.setBaudRate(1000000)
fk = scs.PacketHandler(0)

time.sleep(10)
phase["name"] = "MOVING"
centre, _, _ = fk.read2ByteTxRx(fp, MOTOR, ADDR_POS)
print(f"follower wrist_roll centre = {centre}; wiggling +/-{WIGGLE} for 20 s")
fk.write1ByteTxRx(fp, MOTOR, ADDR_TORQUE, 1)
try:
    t1 = time.time()
    while time.time() - t1 < 20 and th.is_alive():
        for target in (centre + WIGGLE, centre - WIGGLE):
            fk.write2ByteTxRx(fp, MOTOR, ADDR_GOAL, target)
            time.sleep(0.5)
finally:
    fk.write2ByteTxRx(fp, MOTOR, ADDR_GOAL, centre)
    time.sleep(0.5)
    fk.write1ByteTxRx(fp, MOTOR, ADDR_TORQUE, 0)
    fp.closePort()

phase["name"] = "idle-after"
time.sleep(10 if th.is_alive() else 0)
phase["name"] = "done"
th.join(timeout=3)

print("\n".join(events) if events else "no events")
print("VERDICT:", "leader dropped DURING follower motion -> electrical coupling (power/ground)"
      if any("MOVING" in e and ("VANISHED" in e or "EXCEPTION" in e) for e in events)
      else "leader survived follower motion")
