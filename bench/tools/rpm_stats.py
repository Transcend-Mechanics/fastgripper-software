"""Motor RPM statistics from teleop session CSVs — motor-viability analysis.

Reads the gripper velocity column (rad/s, 50 Hz) from teleop logs and reports
the speed distribution in RPM, plus what fraction of actual commanded motion
a slower motor (e.g. a ~30 RPM Dynamixel) could have kept up with.

Usage:
  python bench/tools/rpm_stats.py                      # latest teleop log
  python bench/tools/rpm_stats.py bench/yam/logs/teleop_*.csv    # specific/multiple sessions
  python bench/tools/rpm_stats.py --threshold 30 --threshold 60
"""

import argparse
import csv
import glob
import math
import os
import sys

RAD_S_TO_RPM = 60.0 / (2 * math.pi)   # ~9.549
MOVING_RPM = 1.0                       # below this = holding, excluded from stats


def analyze(path: str, thresholds: list[float]) -> list[float]:
    rpms = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                v = abs(float(row["gripper_vel_rad_s"])) * RAD_S_TO_RPM
            except (KeyError, ValueError):
                continue
            rpms.append(v)
    moving = sorted(r for r in rpms if r > MOVING_RPM)
    print(f"\n{os.path.basename(path)}: {len(rpms)} samples, "
          f"{len(moving)} moving ({len(moving) / max(1, len(rpms)) * 100:.0f}% of session)")
    if not moving:
        print("  (no motion in this session)")
        return moving
    def pct(p):
        return moving[min(len(moving) - 1, int(len(moving) * p))]
    print(f"  while moving:  mean {sum(moving) / len(moving):6.1f} RPM   "
          f"median {pct(0.50):6.1f}   p95 {pct(0.95):6.1f}   max {moving[-1]:6.1f}")
    for t in thresholds:
        over = sum(1 for r in moving if r > t)
        print(f"  above {t:5.1f} RPM: {over / len(moving) * 100:5.1f}% of moving time"
              + ("   <-- a motor capped here misses this fraction of commanded speed"
                 if over else "   <-- a motor capped here would have kept up fully"))
    return moving


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="*", help="teleop CSVs (default: latest)")
    parser.add_argument("--threshold", type=float, action="append",
                        help="viability threshold(s) in RPM (default: 30, 60, 100)")
    parser.add_argument("--logdir", default=os.path.join(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))), "yam", "logs"))
    args = parser.parse_args()
    thresholds = args.threshold or [30.0, 60.0, 100.0]

    files = args.files
    if not files:
        candidates = sorted(glob.glob(os.path.join(args.logdir, "teleop_*.csv")))
        if not candidates:
            sys.exit(f"no teleop logs in {args.logdir}")
        files = [candidates[-1]]

    all_moving = []
    for path in files:
        all_moving.extend(analyze(path, thresholds))

    if len(files) > 1 and all_moving:
        all_moving.sort()
        print(f"\nCOMBINED ({len(files)} sessions, {len(all_moving)} moving samples):")
        print(f"  median {all_moving[len(all_moving) // 2]:6.1f} RPM   "
              f"p95 {all_moving[int(len(all_moving) * 0.95)]:6.1f}   "
              f"max {all_moving[-1]:6.1f}")

    print("\nreference: full stroke (~33.5 rad = 5.3 turns) takes "
          "10.7 s at 30 RPM, 5.3 s at 60 RPM, 1.4 s at the current "
          "teleop cap (24 rad/s = 229 RPM)")


if __name__ == "__main__":
    main()
