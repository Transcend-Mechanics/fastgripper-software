"""fastgripper-dm — command-line front door.

    fastgripper-dm setup      save interface/channel (+ motor ids) and write the motor watchdog
    fastgripper-dm preflight  no-motion go/no-go: bus echo, motor answers, fault, watchdog, cal entry
    fastgripper-dm doctor     bus ping with fault decode (which motors are alive?)
    fastgripper-dm id         re-ID a factory motor (must be alone on the bus) / --verify
    fastgripper-dm watchdog   read or set the comm-loss watchdog (ms)
    fastgripper-dm calibrate  keyboard jog, mark open/closed (real terminal)
    fastgripper-dm autocal    full|home|touch hardstop calibration
    fastgripper-dm drive      go to a percentage and exit
    fastgripper-dm cal-doctor turn-alias diagnosis of the saved calibration

Saved settings live in $FASTGRIPPER_DM_HOME or ~/.config/fastgripper-dm/
(config.json + gripper_cal.json). The tools take --interface/--channel/
--motor_id/--master_id explicitly; `setup` records them so you can omit them.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from .calstore import default_config_path


def _load_config() -> dict:
    p = default_config_path()
    if os.path.exists(p):
        return json.load(open(p))
    return {}


def _save_config(cfg: dict) -> None:
    p = default_config_path()
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp, p)


def _bus_argv(cfg: dict, extra: list[str]) -> list[str]:
    """Prefix saved bus/motor settings unless the caller passed them explicitly."""
    out: list[str] = []
    for key, flag in (("interface", "--interface"), ("channel", "--channel"),
                      ("motor_id", "--motor_id"), ("master_id", "--master_id")):
        if key in cfg and flag not in extra:
            out += [flag, str(cfg[key])]
    return out + extra


def _run_tool(tool_main, argv: list[str], prog: str) -> None:
    from .tools._cli import run

    sys.argv = [prog] + argv
    run(tool_main)


def cmd_setup(args, extra):
    from .damiao.canbus import open_bus
    from .damiao.config_tool import DEFAULT_WATCHDOG_MS, RegisterClient, read_watchdog_ms, set_watchdog_ms

    cfg = _load_config()
    for key in ("interface", "channel", "motor_id", "master_id"):
        v = getattr(args, key)
        if v is not None:
            cfg[key] = v
    if "interface" not in cfg:
        raise SystemExit("setup needs --interface (socketcan|gs_usb|slcan) and --channel")
    cfg.setdefault("motor_id", 0x01)
    cfg.setdefault("master_id", 0x00)
    cfg["watchdog_ms"] = args.watchdog_ms
    _save_config(cfg)
    print(f"saved {default_config_path()}: {cfg}")
    with open_bus(cfg["interface"], cfg.get("channel")) as bus:
        c = RegisterClient(bus)
        mid = int(cfg["motor_id"])
        if not c.probe(mid):
            raise SystemExit(f"motor 0x{mid:02X} does not answer -- check power/wiring, or run `fastgripper-dm id`")
        before = read_watchdog_ms(c, mid)
        got = before if before == args.watchdog_ms else set_watchdog_ms(c, mid, args.watchdog_ms)
        print(f"motor 0x{mid:02X} watchdog: {before} -> {got} ms (verified)")


def cmd_watchdog(args, extra):
    from .damiao.canbus import open_bus
    from .damiao.config_tool import RegisterClient, read_watchdog_ms, set_watchdog_ms

    cfg = _load_config()
    mid = int(args.motor_id if args.motor_id is not None else cfg.get("motor_id", 0x01))
    with open_bus(args.interface or cfg.get("interface", "auto"), args.channel or cfg.get("channel")) as bus:
        c = RegisterClient(bus)
        if args.set is None:
            print(f"motor 0x{mid:02X} watchdog = {read_watchdog_ms(c, mid)} ms")
        else:
            print(f"motor 0x{mid:02X} watchdog = {set_watchdog_ms(c, mid, args.set)} ms (verified)")


def cmd_id(args, extra):
    from .damiao.canbus import open_bus
    from .damiao.config_tool import RegisterClient, set_motor_id

    cfg = _load_config()
    with open_bus(args.interface or cfg.get("interface", "auto"), args.channel or cfg.get("channel")) as bus:
        c = RegisterClient(bus)
        if args.verify:
            for reg in ("id", "master_id", "timeout", "sw_ver"):
                print(f"motor 0x{args.new_id:02X} {reg} = {c.read(args.new_id, reg)}")
            return
        got_id, got_master = set_motor_id(c, args.old_id, args.new_id, args.master_id)
        print(f"SUCCESS: motor is now 0x{got_id:02X} / feedback 0x{got_master:02X}")
        cfg["motor_id"], cfg["master_id"] = got_id, got_master
        _save_config(cfg)
        print(f"saved to {default_config_path()}")


def cmd_preflight(args, extra):
    from .tools.preflight import run_preflight

    cfg = _load_config()
    ok = run_preflight(cfg, interface=args.interface, channel=args.channel)
    raise SystemExit(0 if ok else 1)


def main() -> None:
    parser = argparse.ArgumentParser(prog="fastgripper-dm", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    hexint = lambda s: int(s, 0)  # noqa: E731

    p = sub.add_parser("setup", help="save bus/motor settings and write the motor watchdog")
    p.add_argument("--interface", choices=["gs_usb", "slcan", "socketcan"])
    p.add_argument("--channel")
    p.add_argument("--motor_id", type=hexint)
    p.add_argument("--master_id", type=hexint)
    p.add_argument("--watchdog_ms", type=int, default=500)

    p = sub.add_parser("preflight", help="no-motion go/no-go")
    p.add_argument("--interface"); p.add_argument("--channel")

    p = sub.add_parser("watchdog", help="read/set the comm-loss watchdog (ms)")
    p.add_argument("--set", type=int, default=None)
    p.add_argument("--interface"); p.add_argument("--channel"); p.add_argument("--motor_id", type=hexint)

    p = sub.add_parser("id", help="re-ID a factory motor (motor ALONE on the bus)")
    p.add_argument("--old_id", type=hexint, default=0x01)
    p.add_argument("--new_id", type=hexint, default=0x07)
    p.add_argument("--master_id", type=hexint, default=None, help="default new_id + 0x10")
    p.add_argument("--verify", action="store_true")
    p.add_argument("--interface"); p.add_argument("--channel")

    for name in ("doctor", "calibrate", "autocal", "drive", "cal-doctor"):
        sub.add_parser(name, help=f"run `{name}` (pass its own flags after the name; -h for help)")

    args, extra = parser.parse_known_args()
    if args.cmd == "setup":
        return cmd_setup(args, extra)
    if args.cmd == "preflight":
        return cmd_preflight(args, extra)
    if args.cmd == "watchdog":
        return cmd_watchdog(args, extra)
    if args.cmd == "id":
        return cmd_id(args, extra)

    cfg = _load_config()
    argv = _bus_argv(cfg, extra)
    if args.cmd == "doctor":
        from .tools.bus_ping import main as tool
    elif args.cmd == "calibrate":
        from .tools.calibrate import main as tool
    elif args.cmd == "autocal":
        from .tools.autocal import main as tool
    elif args.cmd == "drive":
        from .tools.drive import main as tool
    else:
        from .tools.cal_doctor import main as tool
    _run_tool(tool, argv, f"fastgripper-dm {args.cmd}")


if __name__ == "__main__":
    main()
