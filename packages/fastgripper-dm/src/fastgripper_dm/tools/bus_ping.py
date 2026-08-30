"""5-second CAN bus triage: who is actually alive?

Sends a harmless disable frame to each motor ID and reports:
  - TX echo:   present = the frame was ACKed by SOMEONE (bus electrically OK)
               absent  = nobody ACKed (adapter wedge / wiring / power) -> replug adapter USB
  - feedback:  which motors answered (arm 1-6 reply on 0x11-0x16, gripper 0x07 on 0x17)

Interpretation:
  no echo at all          -> adapter wedged or bus dead: replug adapter USB, check wiring/power
  echo but arm silent     -> arm motors latched comm-fault or unpowered: POWER-CYCLE THE 24V PSU
  echo but gripper silent -> gripper power/IDs
  everyone answers        -> bus is fine; the problem is elsewhere

Usage: fastgripper-dm doctor [--interface socketcan --channel can0] [--ids 1,2,3,7]
"""

import argparse
import time

from ..damiao.canbus import add_bus_args, open_bus
from ..damiao.dm4310 import FAULT_CODES

ARM_IDS: list[int] = []      # optional arm motors to include (--ids)
GRIPPER_ID = 0x07            # overridden by --motor_id


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(parser)
    parser.add_argument("--ids", default=None,
                        help="comma-separated extra motor IDs to ping (e.g. arm joints 1,2,3,4,5,6)")
    args = parser.parse_args()
    global ARM_IDS, GRIPPER_ID
    GRIPPER_ID = args.motor_id
    ARM_IDS = [i for i in ([int(x, 0) for x in args.ids.split(",")] if args.ids else []) if i != GRIPPER_ID]

    with open_bus(args.interface, args.channel) as bus:
        echo_seen = False
        answered: dict[int, tuple[int, int]] = {}  # cmd_id -> (feedback arb id, status nibble)
        for cmd_id in ARM_IDS + [GRIPPER_ID]:
            import can
            bus.send(can.Message(arbitration_id=cmd_id,
                                 data=[0xFF] * 7 + [0xFD], is_extended_id=False))
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.25:
                m = bus.recv(timeout=0.05)
                if m is None:
                    continue
                if not m.is_rx:
                    echo_seen = True          # our own frame, ACKed on the wire
                    continue
                # byte 0 = (status << 4) + motor id; normalize against the
                # command id so re-ID'd motors (> 0x0F) decode correctly
                status = ((m.data[0] - (cmd_id & 0xFF)) >> 4) & 0x0F if len(m.data) == 8 else -1
                answered[cmd_id] = (m.arbitration_id, status)
                break

        print(f"TX echo (bus ACK): {'YES' if echo_seen else 'NO'}")
        any_fault = False
        for cmd_id in ARM_IDS + [GRIPPER_ID]:
            tag = "arm" if cmd_id in ARM_IDS else "gripper"
            if cmd_id in answered:
                arb, status = answered[cmd_id]
                text = FAULT_CODES.get(status, f"status {status:#x}")
                mark = ""
                if status >= 0x8:
                    mark = "  <-- FAULT"
                    any_fault = True
                print(f"  motor 0x{cmd_id:02X} ({tag}): answered on 0x{arb:03X}  [{text}]{mark}")
            else:
                print(f"  motor 0x{cmd_id:02X} ({tag}): SILENT")
        if any_fault:
            print("\n-> FAULTED motor(s) above. Fault names decode the cause:")
            print("   undervoltage = 24V delivery sagging (loose connector / PSU limit)")
            print("   communication loss = motor's CAN watchdog tripped")
            print("   Clear a latched fault by power-cycling the 24V PSU.")

        if not echo_seen:
            print("\n-> NO ACK on the wire: adapter wedge or wiring/power. "
                  "Replug the adapter USB and re-run.")
        elif not any(i in answered for i in ARM_IDS):
            print("\n-> Bus alive but the ARM is silent: latched comm-fault or no arm power. "
                  "POWER-CYCLE THE 24V PSU, then re-run.")
        elif GRIPPER_ID not in answered:
            print("\n-> Bus + arm alive but the gripper is silent: check its power tap / IDs.")
        else:
            print("\n-> Everything answers. The bus is fine.")


def cli() -> None:
    from ._cli import run
    run(main)


if __name__ == "__main__":
    cli()
