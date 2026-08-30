"""Multi-gripper calibration store.

Format v2 — one gripper_cal.json, named entries, each self-describing:

    {
      "format": 2,
      "grippers": {
        "right": {
          "motor_id": 32, "master_id": 48,
          "open": ..., "closed": ..., "span": ...,
          "last_position": ..., "last_wrapped": ...,
          "stop_open": ..., "stop_closed": ..., "stop_span": ...,
          "calibrated_at": "..."
        }
      }
    }

Legacy flat single-gripper files (open/closed at top level) load as an entry
named "default" and are upgraded to v2 on the next save. Tools select an
entry with --gripper <name>; when the file has exactly one entry the name is
optional.
"""

import json
import os


def config_home() -> str:
    """$FASTGRIPPER_DM_HOME, else ~/.config/fastgripper-dm. Never the cwd: a
    stale gripper_cal.json in the working directory silently shadowing the
    real calibration would drive goto targets toward the wrong stops."""
    return os.environ.get("FASTGRIPPER_DM_HOME") or os.path.join(
        os.path.expanduser("~"), ".config", "fastgripper-dm")


def default_cal_path() -> str:
    return os.path.join(config_home(), "gripper_cal.json")


def default_config_path() -> str:
    return os.path.join(config_home(), "config.json")


def load_store(path: str) -> dict:
    """Load a cal store, migrating legacy flat files in memory."""
    try:
        raw = json.load(open(path))
    except OSError:
        return {"format": 2, "grippers": {}}
    if "grippers" in raw:
        return raw
    # legacy flat single-gripper file
    return {"format": 2, "grippers": {"default": raw}}


def save_store(path: str, store: dict) -> None:
    """Atomic write: a crash mid-save must not destroy the only calibration."""
    store["format"] = 2
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(store, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def get_entry(store: dict, name: str | None = None) -> tuple[str, dict]:
    """Resolve (name, entry). Name optional iff exactly one entry exists."""
    g = store["grippers"]
    if name is not None:
        if name not in g:
            raise SystemExit(f"no gripper named '{name}' in the cal file — "
                             f"have: {', '.join(sorted(g)) or '(none)'}")
        return name, g[name]
    if len(g) == 1:
        return next(iter(g.items()))
    if not g:
        raise SystemExit("cal file has no gripper entries — calibrate first "
                         "(calibrate.py or autocal.py full)")
    raise SystemExit(f"cal file has multiple grippers ({', '.join(sorted(g))}) — "
                     f"pick one with --gripper <name>")


def resolve_ids(args, entry: dict) -> tuple[int, int]:
    """Motor IDs: the entry's own IDs win unless the CLI overrides them.
    (add_bus_args defaults are 0x01/0x00 — treated as 'not specified'.)"""
    cli_set = not (args.motor_id == 0x01 and args.master_id == 0x00)
    if cli_set or "motor_id" not in entry:
        return args.motor_id, args.master_id
    return int(entry["motor_id"]), int(entry["master_id"])
