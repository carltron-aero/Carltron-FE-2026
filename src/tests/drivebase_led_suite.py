#!/usr/bin/env python3
"""Standalone DriveBase LED status test.

This script exercises only the front/rear LED outputs used by the production
status thread. It does not start the DriveBase control loop and does not command
the drivetrain.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = REPO_ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))


def clamp_level(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def triangle01(phase: float) -> float:
    phase = phase % 1.0
    if phase < 0.5:
        return phase * 2.0
    return (1.0 - phase) * 2.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test front/rear LED modes without commanding robot drive hardware.",
    )
    parser.add_argument("--front-base", type=float, default=0.03, help="Front baseline brightness")
    parser.add_argument("--rear-base", type=float, default=0.10, help="Rear baseline brightness")
    parser.add_argument("--front-confirmation", type=float, default=0.80, help="Front confirmation brightness")
    parser.add_argument("--rear-pulse", type=float, default=1.0, help="Rear pulse brightness")
    parser.add_argument("--breathe-period-s", type=float, default=2.2, help="Waiting-state rear breathe period")
    parser.add_argument("--blink-period-s", type=float, default=2.0, help="Normal rear blink period")
    parser.add_argument("--blink-on-s", type=float, default=0.10, help="Normal rear pulse duration")
    parser.add_argument("--mode-duration-s", type=float, default=3.0, help="Duration for each animated mode")
    parser.add_argument("--hold-duration-s", type=float, default=1.6, help="Duration for each static confirmation mode")
    parser.add_argument("--step-s", type=float, default=0.02, help="LED update interval for animations")
    parser.add_argument("--no-mirror-leds", action="store_true", help="Disable GPIO5/GPIO6 mirror LEDs; only front hpwm and rear GPIO11 are driven")
    parser.add_argument("--skip-countdown", action="store_true", help="Start immediately without the safety countdown")
    return parser


def validate_args(parser: argparse.ArgumentParser, args) -> None:
    for name in ("front_base", "rear_base", "front_confirmation", "rear_pulse"):
        if not 0.0 <= float(getattr(args, name)) <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be between 0 and 1")
    if min(args.breathe_period_s, args.blink_period_s, args.blink_on_s, args.mode_duration_s, args.hold_duration_s, args.step_s) <= 0:
        parser.error("timing values must be > 0")


def countdown(skip: bool) -> None:
    if skip:
        return
    print("LED test only. Drivetrain is not started, but keep the robot safe.")
    for remaining in range(3, 0, -1):
        print(f"Starting in {remaining}...")
        time.sleep(1.0)


def set_leds(db, front: float, rear: float) -> None:
    db.set_front_led(clamp_level(front))
    db.set_rear_led(clamp_level(rear))


def run_for(duration_s: float, step_s: float, callback) -> None:
    start = time.monotonic()
    next_t = start
    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= duration_s:
            break
        callback(elapsed)
        next_t += step_s
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_t = time.monotonic()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from robot_core.drivebase import DriveBase

    countdown(bool(args.skip_countdown))

    db = DriveBase()
    if bool(args.no_mirror_leds):
        setter = getattr(db, "set_mirror_leds_enabled", None)
        if callable(setter):
            setter(False)
            print("GPIO5/GPIO6 mirror LEDs disabled for this test")
        else:
            print("DriveBase does not expose mirror LED control; continuing with default LED mapping")
    try:
        front_base = clamp_level(args.front_base)
        rear_base = clamp_level(args.rear_base)
        front_confirmation = clamp_level(args.front_confirmation)
        rear_pulse = clamp_level(args.rear_pulse)

        print("1/6 waiting_for_start: front base, rear breathing")
        run_for(
            args.mode_duration_s,
            args.step_s,
            lambda elapsed: set_leds(
                db,
                front_base,
                triangle01(elapsed / max(0.05, args.breathe_period_s)),
            ),
        )

        print("2/6 normal_running: front base, rear short pulse every blink period")
        run_for(
            args.mode_duration_s,
            args.step_s,
            lambda elapsed: set_leds(
                db,
                front_base,
                rear_pulse if math.fmod(elapsed, args.blink_period_s) < args.blink_on_s else rear_base,
            ),
        )

        print("3/6 color_confirmation base attempt: front confirmation, rear normal base")
        set_leds(db, front_confirmation, rear_base)
        time.sleep(args.hold_duration_s)

        print("4/6 color_confirmation underexposed retry: front confirmation, rear normal base")
        set_leds(db, front_confirmation, rear_base)
        time.sleep(args.hold_duration_s)

        print("5/6 color_confirmation overexposed retry: front base, rear normal base")
        set_leds(db, front_base, rear_base)
        time.sleep(args.hold_duration_s)

        print("6/6 shutdown/off")
        set_leds(db, 0.0, 0.0)
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted; turning LEDs off")
    finally:
        try:
            set_leds(db, 0.0, 0.0)
        except Exception:
            pass
        try:
            db.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
