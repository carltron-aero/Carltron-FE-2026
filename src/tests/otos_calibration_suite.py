#!/usr/bin/env python3
"""
Manual calibration suite for SparkFun Qwiic OTOS.

This script only initializes and reads the OTOS. It does not import DriveBase,
does not command motors/steering, and does not touch GPIO/PWM.

Recommended order:
  1. angular: calibrate angular scalar first
  2. linear : calibrate linear scalar second
  3. live   : inspect readings after applying scalars/offset

Coordinate convention for offsets:
  +x = forward from robot reference point, usually rear axle midpoint
  +y = left
  +h = counterclockwise mounting angle in degrees
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional

import qwiic_otos


SCALAR_MIN = 0.872
SCALAR_MAX = 1.127
DEFAULT_RATE_HZ = 100.0


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    h: float
    vx: float
    vy: float
    vh: float
    ax: float
    ay: float
    ah: float


def clamp_scalar(value: float) -> float:
    return max(SCALAR_MIN, min(SCALAR_MAX, value))


def scalar_from_actual_measured(actual: float, measured: float) -> float:
    if abs(measured) < 1e-9:
        raise ValueError("Measured value is too close to zero")
    return actual / measured


def wrap_delta_deg(curr: float, prev: float) -> float:
    return (curr - prev + 180.0) % 360.0 - 180.0


def unit_constant(name: str, class_name: str):
    if hasattr(qwiic_otos, name):
        return getattr(qwiic_otos, name)
    return getattr(qwiic_otos.QwiicOTOS, class_name)


def make_pose(x: float, y: float, h: float):
    return qwiic_otos.Pose2D(x, y, h)


def pose_dict(pose) -> dict:
    return {"x": float(pose.x), "y": float(pose.y), "h": float(pose.h)}


def require_not_false(result, label: str) -> None:
    """Some qwiic_otos write methods return None on success, False on validation failure."""
    if result is False:
        raise RuntimeError(f"{label} failed")


def setup_otos(args) -> qwiic_otos.QwiicOTOS:
    otos = qwiic_otos.QwiicOTOS()
    if not otos.is_connected():
        raise RuntimeError("OTOS not connected")
    if not otos.begin():
        raise RuntimeError("OTOS begin() failed")

    require_not_false(
        otos.setLinearUnit(unit_constant("kLinearUnitMeters", "kLinearUnitMeters")),
        "setLinearUnit()",
    )
    require_not_false(
        otos.setAngularUnit(unit_constant("kAngularUnitDegrees", "kAngularUnitDegrees")),
        "setAngularUnit()",
    )

    offset = make_pose(args.offset_x, args.offset_y, args.offset_h)
    require_not_false(otos.setOffset(offset), "setOffset()")

    require_not_false(otos.setLinearScalar(args.linear_scalar), "setLinearScalar()")
    require_not_false(otos.setAngularScalar(args.angular_scalar), "setAngularScalar()")

    if args.calibrate_imu:
        print("Keep robot perfectly still: calibrating OTOS IMU...")
        require_not_false(otos.calibrateImu(), "calibrateImu()")

    if args.reset:
        require_not_false(otos.resetTracking(), "resetTracking()")
        zero = make_pose(0.0, 0.0, 0.0)
        require_not_false(otos.setPosition(zero), "setPosition()")

    return otos


def read_sample(otos: qwiic_otos.QwiicOTOS, t0: float) -> PoseSample:
    try:
        pos, vel, acc = otos.getPosVelAcc()
    except Exception:
        pos = otos.getPosition()
        vel = otos.getVelocity()
        acc = otos.getAcceleration()
    return PoseSample(
        t=time.monotonic() - t0,
        x=float(pos.x),
        y=float(pos.y),
        h=float(pos.h),
        vx=float(vel.x),
        vy=float(vel.y),
        vh=float(vel.h),
        ax=float(acc.x),
        ay=float(acc.y),
        ah=float(acc.h),
    )


def wait_for_enter(message: str) -> None:
    input(message)


def sample_until_enter(otos: qwiic_otos.QwiicOTOS, rate_hz: float) -> list[PoseSample]:
    done = threading.Event()

    def _waiter():
        input("Press Enter when finished...\n")
        done.set()

    thread = threading.Thread(target=_waiter, daemon=True)
    thread.start()

    samples: list[PoseSample] = []
    t0 = time.monotonic()
    period = 1.0 / rate_hz
    next_t = time.monotonic()
    while not done.is_set():
        samples.append(read_sample(otos, t0))
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    samples.append(read_sample(otos, t0))
    return samples


def sample_until_enter_with_t0(
    otos: qwiic_otos.QwiicOTOS,
    rate_hz: float,
    t0: float,
    prompt: str = "Press Enter when finished...\n",
) -> list[PoseSample]:
    done = threading.Event()

    def _waiter():
        input(prompt)
        done.set()

    thread = threading.Thread(target=_waiter, daemon=True)
    thread.start()

    samples: list[PoseSample] = []
    period = 1.0 / rate_hz
    next_t = time.monotonic()
    while not done.is_set():
        samples.append(read_sample(otos, t0))
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    samples.append(read_sample(otos, t0))
    return samples


def sample_for_duration(
    otos: qwiic_otos.QwiicOTOS,
    rate_hz: float,
    duration_s: float,
) -> list[PoseSample]:
    samples: list[PoseSample] = []
    t0 = time.monotonic()
    period = 1.0 / rate_hz
    next_t = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        samples.append(read_sample(otos, t0))
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    samples.append(read_sample(otos, t0))
    return samples


def continuous_heading_delta(samples: Iterable[PoseSample]) -> float:
    iterator = iter(samples)
    first = next(iterator, None)
    if first is None:
        return 0.0
    prev = first.h
    total = 0.0
    for sample in iterator:
        total += wrap_delta_deg(sample.h, prev)
        prev = sample.h
    return total


def path_distance(samples: list[PoseSample]) -> float:
    if len(samples) < 2:
        return 0.0
    dist = 0.0
    prev = samples[0]
    for sample in samples[1:]:
        dist += math.hypot(sample.x - prev.x, sample.y - prev.y)
        prev = sample
    return dist


def final_displacement(samples: list[PoseSample]) -> float:
    if not samples:
        return 0.0
    first = samples[0]
    last = samples[-1]
    return math.hypot(last.x - first.x, last.y - first.y)


def displacement_components(start: PoseSample, end: PoseSample, heading_deg: float) -> tuple[float, float]:
    dx = end.x - start.x
    dy = end.y - start.y
    h = math.radians(heading_deg)
    forward = dx * math.cos(h) + dy * math.sin(h)
    lateral = -dx * math.sin(h) + dy * math.cos(h)
    return forward, lateral


def write_csv(path: Optional[Path], samples: list[PoseSample]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(samples[0]).keys()))
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))
    print(f"Wrote samples: {path}")


def write_directional_csv(path: Optional[Path], legs: list[dict]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    sample_fields = list(asdict(PoseSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)).keys())
    fieldnames = ["leg_index", "leg_name", "sample_index"] + sample_fields
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for leg in legs:
            for sample_index, sample in enumerate(leg["samples"]):
                row = {
                    "leg_index": leg["index"],
                    "leg_name": leg["name"],
                    "sample_index": sample_index,
                }
                row.update(asdict(sample))
                writer.writerow(row)
    print(f"Wrote samples: {path}")


def write_result(path: Optional[Path], result: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(result, f, indent=2, sort_keys=True)
        f.write("\n")
    print(f"Wrote result: {path}")


def print_setup(otos: qwiic_otos.QwiicOTOS, args=None) -> None:
    print("OTOS setup:")
    try:
        hw, fw = otos.getVersionInfo()
        print(f"  version: hw={hw}, fw={fw}")
    except Exception as e:
        print(f"  version: unavailable ({e})")
    try:
        print(f"  linear_scalar={otos.getLinearScalar():.6f}")
        print(f"  angular_scalar={otos.getAngularScalar():.6f}")
    except Exception as e:
        print(f"  scalars: unavailable ({e})")
    if args is not None:
        print(
            "  requested_offset="
            f"{{'x': {args.offset_x}, 'y': {args.offset_y}, 'h': {args.offset_h}}}"
        )
    try:
        readback = pose_dict(otos.getOffset())
        print(f"  readback_offset={readback}")
        if args is not None:
            dx = abs(readback["x"] - args.offset_x)
            dy = abs(readback["y"] - args.offset_y)
            dh = abs(wrap_delta_deg(readback["h"], args.offset_h))
            if dx > 0.002 or dy > 0.002 or dh > 1.0:
                print(
                    "  WARNING: offset readback differs from requested offset. "
                    "If live axes behave correctly, continue testing; otherwise check library/firmware support."
                )
    except Exception as e:
        print(f"  readback_offset: unavailable ({e})")


def cmd_angular(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print("Angular calibration:")
    print(f"  actual rotation: {args.actual_deg:.3f} deg")
    print("  Rotate the robot around the chosen robot origin.")
    print("  Use several full turns when possible; angular scalar should be calibrated first.")
    wait_for_enter("Press Enter to reset tracking and start sampling...")
    otos.resetTracking()
    otos.setPosition(make_pose(0.0, 0.0, 0.0))

    if args.duration > 0:
        samples = sample_for_duration(otos, args.rate, args.duration)
    else:
        samples = sample_until_enter(otos, args.rate)

    measured = continuous_heading_delta(samples)
    raw_scalar = scalar_from_actual_measured(args.actual_deg, measured)
    scalar = clamp_scalar(raw_scalar)
    last = samples[-1]
    result = {
        "mode": "angular",
        "actual_deg": args.actual_deg,
        "measured_deg": measured,
        "raw_angular_scalar": raw_scalar,
        "clamped_angular_scalar": scalar,
        "final_pose": {"x": last.x, "y": last.y, "h": last.h},
        "final_displacement_m": final_displacement(samples),
        "path_distance_m": path_distance(samples),
        "input_linear_scalar": args.linear_scalar,
        "input_angular_scalar": args.angular_scalar,
        "offset": {"x": args.offset_x, "y": args.offset_y, "h": args.offset_h},
        "samples": len(samples),
        "duration_s": samples[-1].t if samples else 0.0,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    if raw_scalar != scalar:
        print(f"WARNING: scalar was outside OTOS range {SCALAR_MIN}..{SCALAR_MAX} and was clamped")
    write_csv(args.csv, samples)
    write_result(args.out, result)


def cmd_linear(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print("Linear calibration:")
    print(f"  actual straight distance: {args.actual_m:.4f} m")
    print("  Apply the already-calibrated angular scalar for best results.")
    print("  Move as straight as possible, then stop before pressing Enter.")
    wait_for_enter("Press Enter to reset tracking and start sampling...")
    otos.resetTracking()
    otos.setPosition(make_pose(0.0, 0.0, 0.0))

    if args.duration > 0:
        samples = sample_for_duration(otos, args.rate, args.duration)
    else:
        samples = sample_until_enter(otos, args.rate)

    displacement = final_displacement(samples)
    distance = path_distance(samples)
    measured = distance if args.use_path_distance else displacement
    raw_scalar = scalar_from_actual_measured(args.actual_m, measured)
    scalar = clamp_scalar(raw_scalar)
    last = samples[-1]
    result = {
        "mode": "linear",
        "actual_m": args.actual_m,
        "measured_m": measured,
        "final_displacement_m": displacement,
        "path_distance_m": distance,
        "raw_linear_scalar": raw_scalar,
        "clamped_linear_scalar": scalar,
        "input_linear_scalar": args.linear_scalar,
        "input_angular_scalar": args.angular_scalar,
        "offset": {"x": args.offset_x, "y": args.offset_y, "h": args.offset_h},
        "final_pose": {"x": last.x, "y": last.y, "h": last.h},
        "samples": len(samples),
        "duration_s": last.t,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    if raw_scalar != scalar:
        print(f"WARNING: scalar was outside OTOS range {SCALAR_MIN}..{SCALAR_MAX} and was clamped")
    write_csv(args.csv, samples)
    write_result(args.out, result)


def cmd_live(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print("Live OTOS stream. Ctrl+C to stop.")
    t0 = time.monotonic()
    period = 1.0 / args.rate
    try:
        while True:
            sample = read_sample(otos, t0)
            print(
                f"t={sample.t:8.3f}s "
                f"pos=({sample.x:+8.4f} m, {sample.y:+8.4f} m, {sample.h:+8.2f} deg) "
                f"vel=({sample.vx:+8.4f} m/s, {sample.vy:+8.4f} m/s, {sample.vh:+8.2f} deg/s)"
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print()


def cmd_validate(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print("Validation run:")
    print("  Reset, move through any test path, stop, then press Enter.")
    wait_for_enter("Press Enter to reset tracking and start sampling...")
    otos.resetTracking()
    otos.setPosition(make_pose(0.0, 0.0, 0.0))
    samples = sample_until_enter(otos, args.rate)
    last = samples[-1]
    heading_delta = continuous_heading_delta(samples)
    result = {
        "mode": "validate",
        "final_pose": {"x": last.x, "y": last.y, "h": last.h},
        "path_distance_m": path_distance(samples),
        "final_displacement_m": final_displacement(samples),
        "continuous_heading_delta_deg": heading_delta,
        "input_linear_scalar": args.linear_scalar,
        "input_angular_scalar": args.angular_scalar,
        "offset": {"x": args.offset_x, "y": args.offset_y, "h": args.offset_h},
        "samples": len(samples),
        "duration_s": last.t,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    write_csv(args.csv, samples)
    write_result(args.out, result)


def cmd_directional_linear(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print("Directional linear push test:")
    print(f"  legs: {args.directions}")
    print(f"  nominal straight distance per leg: {args.actual_m:.4f} m")
    print("  OTOS is initialized once and tracking is reset once at the beginning.")
    print("  For each leg: align the robot, press Enter, push straight, stop, then press Enter again.")
    wait_for_enter("Press Enter to reset tracking and start the first leg setup...")
    require_not_false(otos.resetTracking(), "resetTracking()")
    require_not_false(otos.setPosition(make_pose(0.0, 0.0, 0.0)), "setPosition()")

    directions = [part.strip() for part in args.directions.split(",") if part.strip()]
    if not directions:
        raise ValueError("--directions must contain at least one label")

    run_t0 = time.monotonic()
    legs: list[dict] = []
    first_sample: Optional[PoseSample] = None
    for leg_index, name in enumerate(directions, start=1):
        print()
        print(f"Leg {leg_index}/{len(directions)}: {name}")
        print("  Align the robot for this direction and keep it still.")
        wait_for_enter("  Press Enter to start sampling this push...")
        samples = sample_until_enter_with_t0(
            otos,
            args.rate,
            run_t0,
            prompt=f"  Push {args.actual_m:.3f} m straight, stop, then press Enter to finish leg {leg_index}...\n",
        )
        if not samples:
            continue
        if first_sample is None:
            first_sample = samples[0]
        start = samples[0]
        end = samples[-1]
        forward, lateral = displacement_components(start, end, start.h)
        displacement = final_displacement(samples)
        distance = path_distance(samples)
        heading_delta = continuous_heading_delta(samples)
        scalar_forward = args.actual_m / abs(forward) if abs(forward) > 1e-9 else None
        scalar_path = args.actual_m / distance if distance > 1e-9 else None
        leg = {
            "index": leg_index,
            "name": name,
            "samples": samples,
            "summary": {
                "index": leg_index,
                "name": name,
                "actual_m": args.actual_m,
                "start_pose": {"x": start.x, "y": start.y, "h": start.h},
                "end_pose": {"x": end.x, "y": end.y, "h": end.h},
                "delta_pose": {
                    "x": end.x - start.x,
                    "y": end.y - start.y,
                    "h": wrap_delta_deg(end.h, start.h),
                },
                "measured_forward_m": forward,
                "measured_lateral_m": lateral,
                "final_displacement_m": displacement,
                "path_distance_m": distance,
                "continuous_heading_delta_deg": heading_delta,
                "forward_error_m": forward - args.actual_m,
                "lateral_error_m": lateral,
                "lateral_error_per_m": lateral / args.actual_m if args.actual_m else 0.0,
                "raw_linear_scalar_from_forward_abs": scalar_forward,
                "raw_linear_scalar_from_path": scalar_path,
                "samples": len(samples),
                "duration_s": end.t - start.t,
            },
        }
        legs.append(leg)
        summary = leg["summary"]
        print(
            "  result: "
            f"forward={summary['measured_forward_m']:+.4f}m "
            f"lateral={summary['measured_lateral_m']:+.4f}m "
            f"heading_delta={summary['continuous_heading_delta_deg']:+.2f}deg "
            f"path={summary['path_distance_m']:.4f}m"
        )

    if not legs:
        raise RuntimeError("No leg samples captured")

    last_sample = legs[-1]["samples"][-1]
    if first_sample is None:
        first_sample = legs[0]["samples"][0]
    total_heading_delta = continuous_heading_delta([sample for leg in legs for sample in leg["samples"]])
    result = {
        "mode": "directional-linear",
        "actual_m": args.actual_m,
        "directions": directions,
        "input_linear_scalar": args.linear_scalar,
        "input_angular_scalar": args.angular_scalar,
        "offset": {"x": args.offset_x, "y": args.offset_y, "h": args.offset_h},
        "legs": [leg["summary"] for leg in legs],
        "final_pose": {"x": last_sample.x, "y": last_sample.y, "h": last_sample.h},
        "total_delta_pose": {
            "x": last_sample.x - first_sample.x,
            "y": last_sample.y - first_sample.y,
            "h": wrap_delta_deg(last_sample.h, first_sample.h),
        },
        "total_displacement_m": math.hypot(last_sample.x - first_sample.x, last_sample.y - first_sample.y),
        "total_continuous_heading_delta_deg": total_heading_delta,
        "samples": sum(len(leg["samples"]) for leg in legs),
        "duration_s": last_sample.t - first_sample.t,
    }
    print()
    print(json.dumps(result, indent=2, sort_keys=True))
    write_directional_csv(args.csv, legs)
    write_result(args.out, result)


def cmd_benchmark(args) -> None:
    otos = setup_otos(args)
    print_setup(otos, args)
    print()
    print(f"Benchmarking burst reads for {args.duration:.2f}s...")
    periods = []
    end_t = time.monotonic() + args.duration
    last = time.monotonic()
    count = 0
    while time.monotonic() < end_t:
        read_sample(otos, last)
        now = time.monotonic()
        periods.append(now - last)
        last = now
        count += 1
    hz = count / args.duration
    result = {
        "mode": "benchmark",
        "duration_s": args.duration,
        "reads": count,
        "hz": hz,
        "period_ms_min": min(periods) * 1000.0,
        "period_ms_mean": statistics.mean(periods) * 1000.0,
        "period_ms_median": statistics.median(periods) * 1000.0,
        "period_ms_max": max(periods) * 1000.0,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    write_result(args.out, result)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Sampling rate in Hz")
    parser.add_argument("--linear-scalar", type=float, default=1.0, help="Applied OTOS linear scalar")
    parser.add_argument("--angular-scalar", type=float, default=1.0, help="Applied OTOS angular scalar")
    parser.add_argument("--offset-x", type=float, default=0.0, help="OTOS x offset from robot origin in meters")
    parser.add_argument("--offset-y", type=float, default=0.0, help="OTOS y offset from robot origin in meters")
    parser.add_argument("--offset-h", type=float, default=0.0, help="OTOS heading offset in degrees")
    parser.add_argument("--calibrate-imu", action="store_true", help="Run OTOS IMU calibration before the command")
    parser.add_argument("--no-reset", dest="reset", action="store_false", help="Do not reset tracking during setup")
    parser.set_defaults(reset=True)
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV file for sampled readings")
    parser.add_argument("--out", type=Path, default=None, help="Optional JSON file for computed result")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manual SparkFun Qwiic OTOS calibration suite",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("angular", help="Compute angular scalar from a known rotation")
    add_common_args(p)
    p.add_argument("--actual-deg", type=float, required=True, help="Actual signed rotation in degrees")
    p.add_argument("--duration", type=float, default=0.0, help="Fixed sampling duration; 0 means press Enter to finish")
    p.set_defaults(func=cmd_angular)

    p = sub.add_parser("linear", help="Compute linear scalar from a known straight movement")
    add_common_args(p)
    p.add_argument("--actual-m", type=float, required=True, help="Actual straight distance in meters")
    p.add_argument(
        "--use-path-distance",
        action="store_true",
        help="Use integrated path length instead of final displacement; normally leave off for straight tests",
    )
    p.add_argument("--duration", type=float, default=0.0, help="Fixed sampling duration; 0 means press Enter to finish")
    p.set_defaults(func=cmd_linear)

    p = sub.add_parser("live", help="Print live OTOS position/velocity")
    add_common_args(p)
    p.set_defaults(func=cmd_live)

    p = sub.add_parser("validate", help="Record and summarize an arbitrary validation path")
    add_common_args(p)
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("directional-linear", help="Record multiple straight manual pushes in one OTOS session")
    add_common_args(p)
    p.add_argument("--actual-m", type=float, default=2.0, help="Nominal straight distance for each push")
    p.add_argument(
        "--directions",
        default="0deg,90deg,180deg,270deg",
        help="Comma-separated leg labels; tracking is continuous, labels are for reporting",
    )
    p.set_defaults(func=cmd_directional_linear)

    p = sub.add_parser("benchmark", help="Measure maximum read throughput")
    add_common_args(p)
    p.add_argument("--duration", type=float, default=5.0, help="Benchmark duration in seconds")
    p.set_defaults(func=cmd_benchmark)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.rate <= 0:
        parser.error("--rate must be > 0")
    if not (SCALAR_MIN <= args.linear_scalar <= SCALAR_MAX):
        parser.error(f"--linear-scalar must be in {SCALAR_MIN}..{SCALAR_MAX}")
    if not (SCALAR_MIN <= args.angular_scalar <= SCALAR_MAX):
        parser.error(f"--angular-scalar must be in {SCALAR_MIN}..{SCALAR_MAX}")

    if getattr(args, "actual_m", 1.0) <= 0:
        parser.error("--actual-m must be > 0")

    try:
        args.func(args)
    except KeyboardInterrupt:
        print()
        return 130
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
