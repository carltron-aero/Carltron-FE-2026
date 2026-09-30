#!/usr/bin/env python3
"""Standalone DriveBase steering-center calibration helper.

The script runs short straight-drive segments with selected steering trims.
Pick the trim that produces the straightest physical path, then use that trim
as DriveConfig.steering_center_trim / production CLI default later.

It commands hardware. Run only in a safe open area.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = REPO_ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))


def parse_trim_list(text: str) -> list[float]:
    trims = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not trims:
        raise argparse.ArgumentTypeError("trim list must contain at least one value")
    if any(abs(t) > 0.25 for t in trims):
        raise argparse.ArgumentTypeError("trim values should stay within [-0.25, 0.25] for this calibration")
    return trims


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Timed steering center trim calibration for DriveBase.")
    parser.add_argument("--trims", type=parse_trim_list, default=[-0.04, -0.02, 0.0, 0.02, 0.04], help="Comma-separated steering trim candidates")
    parser.add_argument("--speed", type=float, default=0.12, help="Target speed in m/s")
    parser.add_argument("--duration-s", type=float, default=2.5, help="Motion duration per trim")
    parser.add_argument("--pause-s", type=float, default=1.0, help="Coast pause between trims")
    parser.add_argument("--countdown-s", type=float, default=2.0, help="Countdown after prompt before motion starts")
    parser.add_argument("--sample-period-s", type=float, default=0.05, help="Telemetry print/log period")
    parser.add_argument("--repeat", type=int, default=1, help="How many times to run the whole trim list")
    parser.add_argument("--reverse", action="store_true", help="Run in reverse instead of forward")
    parser.add_argument("--output", default="", help="CSV output path; default is /tmp/drivebase_steering_center_<timestamp>.csv")
    parser.add_argument("--no-debug", dest="debug", action="store_false", help="Disable DriveBase debug prints")
    parser.set_defaults(debug=True)
    return parser


def normalize_argv(argv: list[str] | None) -> list[str] | None:
    if argv is None:
        argv = sys.argv[1:]
    out: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--trims" and i + 1 < len(argv):
            out.append(f"--trims={argv[i + 1]}")
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def output_path(args) -> Path:
    if args.output:
        return Path(args.output).expanduser()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("/tmp") / f"drivebase_steering_center_{stamp}.csv"


def validate_args(args) -> None:
    if args.speed <= 0.0:
        raise SystemExit("--speed must be > 0")
    if args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be > 0")
    if args.pause_s < 0.0 or args.countdown_s < 0.0:
        raise SystemExit("--pause-s and --countdown-s must be >= 0")
    if args.sample_period_s <= 0.0:
        raise SystemExit("--sample-period-s must be > 0")
    if args.repeat <= 0:
        raise SystemExit("--repeat must be > 0")


def countdown(seconds: float) -> None:
    end_t = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = end_t - time.monotonic()
        if remaining <= 0.0:
            break
        print(f"  starting in {remaining:0.1f}s", end="\r", flush=True)
        time.sleep(min(0.2, remaining))
    print("  starting now        ")


def telemetry_row(segment: str, trim: float, command_speed: float, db, elapsed: float) -> dict:
    telem = db.get_telemetry()
    return {
        "t_monotonic": f"{time.monotonic():.6f}",
        "segment": segment,
        "elapsed_s": f"{elapsed:.4f}",
        "steering_trim": f"{trim:.5f}",
        "command_speed_mps": f"{command_speed:.4f}",
        "mode": telem.get("mode", ""),
        "submode": telem.get("submode", ""),
        "target_user_mps": f"{float(telem.get('target_user_mps', 0.0) or 0.0):.4f}",
        "target_ramped_mps": f"{float(telem.get('target_ramped_mps', 0.0) or 0.0):.4f}",
        "measured_mps": f"{float(telem.get('measured_mps', 0.0) or 0.0):.4f}",
        "position_m": f"{float(telem.get('position_m', 0.0) or 0.0):.5f}",
        "count": int(telem.get("count", 0) or 0),
        "last_duty": "" if telem.get("last_duty") is None else f"{float(telem.get('last_duty')):.4f}",
        "actuating": bool(telem.get("actuating", False)),
    }


def print_sample(row: dict) -> None:
    duty = row["last_duty"] if row["last_duty"] != "" else "--"
    print(
        f"  t={float(row['elapsed_s']):4.1f}s "
        f"trim={float(row['steering_trim']):+.3f} "
        f"v={float(row['measured_mps']):+.3f} "
        f"pos={float(row['position_m']):+.3f} "
        f"cnt={row['count']:>6} "
        f"{row['mode']}/{row['submode']} "
        f"u={duty}"
    )


def run_segment(args, db, writer: csv.DictWriter, repeat_idx: int, trim: float) -> None:
    direction = -1.0 if args.reverse else 1.0
    command_speed = direction * float(args.speed)
    segment = f"repeat_{repeat_idx + 1}_trim_{trim:+.3f}"
    print("\n" + "=" * 80)
    print(f"{segment}: trim={trim:+.4f}, speed={command_speed:+.3f} m/s, duration={args.duration_s:.1f}s")
    print("Mark a straight start line, place the robot with the same heading each run, then press Enter.")
    input("Press Enter to start this timed trim segment...")
    countdown(args.countdown_s)

    db.set_steering(0.0)
    db.set_target_speed(command_speed)
    start_t = time.monotonic()
    next_sample_t = start_t
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start_t
            if elapsed >= args.duration_s:
                break
            if now >= next_sample_t:
                row = telemetry_row(segment, trim, command_speed, db, elapsed)
                writer.writerow(row)
                print_sample(row)
                next_sample_t += max(0.01, float(args.sample_period_s))
            time.sleep(0.005)
    finally:
        db.coast()
        db.set_steering(0.0)
        print(f"Segment complete. Coasting for {args.pause_s:.1f}s.")
        time.sleep(max(0.0, float(args.pause_s)))


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(normalize_argv(argv))
    validate_args(args)

    from robot_core.drivebase import DriveBase, DriveConfig

    csv_path = output_path(args)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    print("DriveBase steering center calibration")
    print(f"Trims: {', '.join(f'{t:+.3f}' for t in args.trims)}")
    print(f"CSV: {csv_path}")
    print("This script commands hardware. Use a safe straight test lane.")

    db = None
    try:
        for repeat_idx in range(args.repeat):
            for trim in args.trims:
                cfg = dataclasses.replace(DriveConfig(debug=bool(args.debug)), steering_center_trim=float(trim))
                if db is not None:
                    db.shutdown()
                    db = None
                    time.sleep(0.2)
                db = DriveBase(cfg)
                db.start()
                time.sleep(0.3)
                mode = "a" if csv_path.exists() else "w"
                with csv_path.open(mode, newline="") as f:
                    probe = telemetry_row("header", trim, 0.0, db, 0.0)
                    writer = csv.DictWriter(f, fieldnames=list(probe.keys()))
                    if mode == "w":
                        writer.writeheader()
                    run_segment(args, db, writer, repeat_idx, float(trim))
        print("\nAll trim segments complete.")
        print(f"Log written to {csv_path}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted. Coasting and shutting down.")
        return 130
    finally:
        if db is not None:
            try:
                db.coast()
                db.set_steering(0.0)
                time.sleep(0.15)
                db.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
