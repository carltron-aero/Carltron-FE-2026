#!/usr/bin/env python3
"""Standalone DriveBase low-speed creep test suite.

This script intentionally bypasses perception, navigation, and the web UI. It
exists to tune DriveBase low-speed behavior with controlled timed segments.

It commands hardware. Run only when the robot is in a safe test area.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
import time
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = REPO_ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))


DEFAULT_SPEEDS = (0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18)


def parse_speed_list(text: str) -> list[float]:
    vals: list[float] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        vals.append(float(part))
    if not vals:
        raise argparse.ArgumentTypeError("speed list must contain at least one value")
    if any(v <= 0.0 for v in vals):
        raise argparse.ArgumentTypeError("all speeds must be positive; use --reverse or --both-directions for reverse")
    return vals


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Timed low-speed DriveBase creep tests. This script commands robot drive hardware.",
    )
    parser.add_argument(
        "mode",
        choices=("straight", "circle", "reverse", "hold"),
        help="Test mode: straight, fixed-steering circle, reverse straight, or repeated same-command hold.",
    )
    parser.add_argument("--speeds", type=parse_speed_list, default=list(DEFAULT_SPEEDS), help="Comma-separated positive speed list in m/s")
    parser.add_argument("--speed", type=float, default=0.08, help="Speed used by hold mode")
    parser.add_argument("--hold-repeats", type=int, default=6, help="Number of repeated segments in hold mode")
    parser.add_argument("--duration-s", type=float, default=4.0, help="Motion duration per segment")
    parser.add_argument("--pause-s", type=float, default=1.0, help="Coast pause between segments")
    parser.add_argument("--countdown-s", type=float, default=2.0, help="Countdown after prompt before motion starts")
    parser.add_argument("--sample-period-s", type=float, default=0.05, help="Telemetry print/log period")
    parser.add_argument("--steering", type=float, default=0.72, help="Normalized steering used by circle mode")
    parser.add_argument("--hold-steering", type=float, default=0.0, help="Normalized steering used by hold mode")
    parser.add_argument("--both-directions", action="store_true", help="Run each speed forward and reverse")
    parser.add_argument("--reverse", action="store_true", help="Run selected mode in reverse direction")
    parser.add_argument("--output", default="", help="CSV output path; default is /tmp/drivebase_creep_<timestamp>.csv")
    parser.add_argument("--no-debug", dest="debug", action="store_false", help="Disable DriveBase debug prints")
    parser.set_defaults(debug=True)

    # Direct DriveConfig tuning overrides used for creep investigation.
    parser.add_argument("--creep-threshold-mps", type=float, default=None)
    parser.add_argument("--breakaway-detect-counts", type=int, default=None)
    parser.add_argument("--breakaway-duty-start", type=float, default=None)
    parser.add_argument("--breakaway-duty-max", type=float, default=None)
    parser.add_argument("--breakaway-duty-ramp-per-s", type=float, default=None)
    parser.add_argument("--hold-decay-overspeed-margin-mps", type=float, default=None)
    parser.add_argument("--hold-min", type=float, default=None)
    parser.add_argument("--hold-capture-factor", type=float, default=None)
    parser.add_argument("--creep-corr-clamp", type=float, default=None)
    parser.add_argument("--creep-duty-slew-per-s", type=float, default=None)
    parser.add_argument("--stall-timeout-s", type=float, default=None)
    parser.add_argument("--v-slew-mps2", type=float, default=None)
    parser.add_argument("--speed-ff-profile", dest="speed_ff_profile", action="store_true", default=None, help="Force calibrated speed-to-duty feedforward profile on")
    parser.add_argument("--no-speed-ff-profile", dest="speed_ff_profile", action="store_false", help="Force legacy linear feedforward")
    parser.add_argument("--speed-ff-correction-clamp", type=float, default=None, help="Override calibrated profile PI trim clamp")
    return parser


def apply_config_overrides(base_cfg, args):
    updates = {"debug": bool(args.debug)}
    mapping = {
        "creep_threshold_mps": args.creep_threshold_mps,
        "breakaway_detect_counts": args.breakaway_detect_counts,
        "breakaway_duty_start": args.breakaway_duty_start,
        "breakaway_duty_max": args.breakaway_duty_max,
        "breakaway_duty_ramp_per_s": args.breakaway_duty_ramp_per_s,
        "hold_decay_overspeed_margin_mps": args.hold_decay_overspeed_margin_mps,
        "hold_min": args.hold_min,
        "hold_capture_factor": args.hold_capture_factor,
        "creep_corr_clamp": args.creep_corr_clamp,
        "creep_duty_slew_per_s": args.creep_duty_slew_per_s,
        "stall_timeout_s": args.stall_timeout_s,
        "v_slew_mps2": args.v_slew_mps2,
        "speed_ff_profile_enable": args.speed_ff_profile,
        "speed_ff_correction_clamp": args.speed_ff_correction_clamp,
    }
    for key, value in mapping.items():
        if value is not None:
            updates[key] = value
    return dataclasses.replace(base_cfg, **updates)


def output_path(args) -> Path:
    if args.output:
        return Path(args.output).expanduser()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("/tmp") / f"drivebase_creep_{stamp}.csv"


def countdown(seconds: float) -> None:
    end_t = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = end_t - time.monotonic()
        if remaining <= 0.0:
            break
        print(f"  starting in {remaining:0.1f}s", end="\r", flush=True)
        time.sleep(min(0.2, remaining))
    print("  starting now        ")


def segment_plan(args) -> list[dict]:
    if args.mode == "hold":
        speed = abs(float(args.speed))
        if speed <= 0.0:
            raise SystemExit("--speed must be > 0 for hold mode")
        sign = -1.0 if args.reverse else 1.0
        return [
            {
                "name": f"hold_{i + 1}",
                "speed": sign * speed,
                "steering": float(args.hold_steering),
            }
            for i in range(max(1, int(args.hold_repeats)))
        ]

    base_steering = float(args.steering) if args.mode == "circle" else 0.0
    base_sign = -1.0 if args.mode == "reverse" or args.reverse else 1.0
    signs = (1.0, -1.0) if args.both_directions else (base_sign,)
    plan: list[dict] = []
    for sign in signs:
        for speed in args.speeds:
            direction = "reverse" if sign < 0.0 else "forward"
            plan.append(
                {
                    "name": f"{args.mode}_{direction}_{speed:.3f}",
                    "speed": sign * abs(float(speed)),
                    "steering": base_steering,
                }
            )
    return plan


def telemetry_row(segment: dict, db, t0: float, elapsed: float) -> dict:
    telem = db.get_telemetry()
    return {
        "t_monotonic": f"{time.monotonic():.6f}",
        "segment": segment["name"],
        "elapsed_s": f"{elapsed:.4f}",
        "command_speed_mps": f"{segment['speed']:.4f}",
        "steering_norm": f"{segment['steering']:.4f}",
        "mode": telem.get("mode", ""),
        "submode": telem.get("submode", ""),
        "target_user_mps": f"{float(telem.get('target_user_mps', 0.0) or 0.0):.4f}",
        "target_ramped_mps": f"{float(telem.get('target_ramped_mps', 0.0) or 0.0):.4f}",
        "measured_mps": f"{float(telem.get('measured_mps', 0.0) or 0.0):.4f}",
        "rate_mps": f"{float(telem.get('rate_mps', 0.0) or 0.0):.4f}",
        "position_m": f"{float(telem.get('position_m', 0.0) or 0.0):.5f}",
        "count": int(telem.get("count", 0) or 0),
        "last_duty": "" if telem.get("last_duty") is None else f"{float(telem.get('last_duty')):.4f}",
        "actuating": bool(telem.get("actuating", False)),
        "run_elapsed_s": f"{time.monotonic() - t0:.4f}",
    }


def print_sample(row: dict) -> None:
    duty = row["last_duty"] if row["last_duty"] != "" else "--"
    print(
        f"  t={float(row['elapsed_s']):5.2f}s "
        f"cmd={float(row['command_speed_mps']):+0.3f} "
        f"v={float(row['measured_mps']):+0.3f} "
        f"pos={float(row['position_m']):+0.3f} "
        f"cnt={row['count']:>6} "
        f"{row['mode']}/{row['submode']} "
        f"u={duty}"
    )


def run_segment(args, db, writer: csv.DictWriter, segment: dict, run_t0: float) -> None:
    print("\n" + "=" * 80)
    print(
        f"Segment {segment['name']}: speed={segment['speed']:+.3f} m/s, "
        f"steering={segment['steering']:+.3f}, duration={args.duration_s:.1f}s"
    )
    input("Place robot safely, then press Enter to start this timed segment...")
    countdown(args.countdown_s)

    db.set_steering(float(segment["steering"]))
    db.set_target_speed(float(segment["speed"]))

    start_t = time.monotonic()
    next_sample_t = start_t
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start_t
            if elapsed >= args.duration_s:
                break
            if now >= next_sample_t:
                row = telemetry_row(segment, db, run_t0, elapsed)
                writer.writerow(row)
                print_sample(row)
                next_sample_t += max(0.01, float(args.sample_period_s))
            time.sleep(0.005)
    finally:
        db.coast()
        db.set_steering(0.0)
        print(f"Segment complete. Coasting for {args.pause_s:.1f}s.")
        time.sleep(max(0.0, float(args.pause_s)))


def validate_args(args) -> None:
    if args.duration_s <= 0:
        raise SystemExit("--duration-s must be > 0")
    if args.pause_s < 0 or args.countdown_s < 0:
        raise SystemExit("--pause-s and --countdown-s must be >= 0")
    if args.sample_period_s <= 0:
        raise SystemExit("--sample-period-s must be > 0")
    if not -1.0 <= args.steering <= 1.0:
        raise SystemExit("--steering must be in [-1, 1]")
    if not -1.0 <= args.hold_steering <= 1.0:
        raise SystemExit("--hold-steering must be in [-1, 1]")
    if args.hold_repeats <= 0:
        raise SystemExit("--hold-repeats must be > 0")
    if args.breakaway_detect_counts is not None and args.breakaway_detect_counts <= 0:
        raise SystemExit("--breakaway-detect-counts must be > 0")
    if args.creep_corr_clamp is not None and args.creep_corr_clamp < 0:
        raise SystemExit("--creep-corr-clamp must be >= 0")
    if args.speed_ff_correction_clamp is not None and args.speed_ff_correction_clamp < 0:
        raise SystemExit("--speed-ff-correction-clamp must be >= 0")


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args)

    from robot_core.drivebase import DriveBase, DriveConfig

    cfg = apply_config_overrides(DriveConfig(), args)
    segments = segment_plan(args)
    csv_path = output_path(args)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    print("DriveBase creep suite")
    print(f"Mode: {args.mode}")
    print(f"Segments: {len(segments)}")
    print(f"CSV: {csv_path}")
    print("This script commands hardware. Keep the robot clear before pressing Enter.")

    db = DriveBase(cfg)
    fieldnames = list(
        telemetry_row({"name": "header", "speed": 0.0, "steering": 0.0}, db, time.monotonic(), 0.0).keys()
    )

    try:
        db.start()
        time.sleep(0.3)
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            run_t0 = time.monotonic()
            for segment in segments:
                run_segment(args, db, writer, segment, run_t0)
                f.flush()
        print("\nAll segments complete.")
        print(f"Log written to {csv_path}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted. Coasting and shutting down.")
        return 130
    finally:
        try:
            db.coast()
            db.set_steering(0.0)
            time.sleep(0.15)
            db.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
