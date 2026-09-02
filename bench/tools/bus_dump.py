"""Raw CAN frame dump: open the bus and print EVERYTHING for a few seconds.

Run this immediately after a suspect session (e.g. autocal) to see what state
the bus/adapter is really in. Phase 1 listens passively (stale backlog shows
up here as a flood of old feedback frames). Phase 2 sends one enable-status
poke to motor 1 and shows every frame that follows, so you can see whether
its reply arrives and what it's buried under.

Usage: .venv/bin/python bench/tools/bus_dump.py --interface gs_usb --channel 0
"""

import argparse
import time

import can

from fastgripper_dm.damiao.canbus import add_bus_args, open_bus


def dump(bus, seconds: float, label: str) -> int:
    print(f"--- {label} ({seconds:.1f}s) ---")
    n = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        m = bus.recv(timeout=0.05)
        if m is None:
            continue
        n += 1
        kind = "rx" if m.is_rx else "TXECHO"
        print(f"  {time.monotonic() - t0:7.3f}s  id=0x{m.arbitration_id:03X} "
              f"[{kind}] data={m.data.hex(' ')}")
        if n >= 200:
            print("  ... (200 frames shown, still flowing -- flood confirmed)")
            break
    if n == 0:
        print("  (no frames)")
    return n


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(parser)
    args = parser.parse_args()

    with open_bus(args.interface, args.channel) as bus:
        stale = dump(bus, 2.0, "phase 1: passive listen (stale backlog?)")
        print(f"\nstale frames seen: {stale}\n")

        print("--- phase 2: poke motor 0x01 (harmless disable), watch replies ---")
        bus.send(can.Message(arbitration_id=0x01, data=[0xFF] * 7 + [0xFD],
                             is_extended_id=False))
        dump(bus, 1.0, "after poke")

    print("\nInterpretation:")
    print("  phase 1 flood of 0x017 (or other) frames = stale backlog from the")
    print("  previous session -> chain init drowns; the drain fix handles it.")
    print("  phase 1 empty + phase 2 shows 0x011 reply = bus healthy right now.")
    print("  phase 2 no reply at all = motor 1 truly not responding (fault/power).")


if __name__ == "__main__":
    import os
    import sys
    code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)
            code = 1
        else:
            code = e.code if e.code is not None else 0
    except KeyboardInterrupt:
        code = 130
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
