#!/usr/bin/env python3
"""Analyze DriveBase timing diagnostics from a drive_debug.csv run log."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import median


def find_csv(path: Path) -> Path:
    if path.is_dir():
        candidate = path / "drive_debug.csv"
        if candidate.exists():
            return candidate
    if path.exists():
        return path
    latest = Path("logs/latest/drive_debug.csv")
    if latest.exists():
        return latest
    raise FileNotFoundError(f"no drive_debug.csv found for {path}")


def f(row: dict, key: str, default=float("nan")) -> float:
    try:
        value = row.get(key, "")
        if value in ("", None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def pct(values: list[float], q: float) -> float:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return float("nan")
    idx = min(len(vals) - 1, max(0, int(round((len(vals) - 1) * q))))
    return vals[idx]


def fmt(v: float) -> str:
    return "n/a" if not math.isfinite(v) else f"{v:.2f}"


def summarize(rows: list[dict], label: str) -> None:
    if not rows:
        return
    loop = [f(r, "loop_ms") for r in rows]
    over = [f(r, "drive_loop_overrun_ms") for r in rows]
    target_period = median([f(r, "drive_loop_target_period_ms") for r in rows if math.isfinite(f(r, "drive_loop_target_period_ms"))] or [float("nan")])
    effective_hz = 1000.0 / pct(loop, 0.50) if pct(loop, 0.50) > 0 else float("nan")
    print(f"\n{label}: rows={len(rows)} target_period_ms={fmt(target_period)} median_hz={fmt(effective_hz)}")
    print(f"  loop_ms      p50={fmt(pct(loop,0.50))} p95={fmt(pct(loop,0.95))} p99={fmt(pct(loop,0.99))} max={fmt(max([v for v in loop if math.isfinite(v)] or [float('nan')]))}")
    print(f"  overrun_ms   p50={fmt(pct(over,0.50))} p95={fmt(pct(over,0.95))} p99={fmt(pct(over,0.99))} max={fmt(max([v for v in over if math.isfinite(v)] or [float('nan')]))}")

    fields = [
        "drive_t_motion_command_ms",
        "drive_t_set_steering_ms",
        "drive_t_db_set_steering_call_ms",
        "drive_t_db_set_steering_lock_wait_ms",
        "drive_t_db_set_steering_pwm_ms",
        "drive_t_db_steering_apply_duration_ms",
        "drive_t_db_steering_apply_pwm_ms",
        "db_steering_apply_age_ms",
        "db_steering_pending_age_ms",
        "db_steering_pending",
        "drive_t_db_motion_apply_duration_ms",
        "drive_t_db_motion_apply_pwm_ms",
        "db_motion_apply_age_ms",
        "db_motion_pending_age_ms",
        "db_motion_pending",
        "db_motion_applied_mode",
        "db_motion_applied_duty",
        "drive_t_db_led_apply_duration_ms",
        "drive_t_db_led_apply_pwm_ms",
        "db_led_apply_age_ms",
        "db_led_pending_age_ms",
        "drive_t_db_actuator_stale_motion_duration_ms",
        "drive_t_db_internal_brake_publish_duration_ms",
        "drive_t_db_set_target_speed_call_ms",
        "drive_t_db_set_target_speed_lock_wait_ms",
        "drive_t_db_get_telemetry_call_ms",
        "drive_t_db_get_telemetry_lock_wait_ms",
        "drive_t_db_apply_output_duration_ms",
        "drive_t_db_apply_output_lock_wait_ms",
        "drive_t_db_apply_output_pwm_ms",
        "drive_t_db_led_duration_ms",
        "drive_t_db_led_lock_wait_ms",
        "drive_timing_hw_pwm_max_ms",
        "drive_pwm_write_count_total",
        "drive_pwm_skip_count_total",
        "drive_pwm_cache_fallback_count_total",
        "drive_pwm_cache_open_count_total",
        "drive_pwm_cache_close_count_total",
        "drive_pwm_cached_fd_count",
        "drive_pwm_write_count_servo",
        "drive_pwm_skip_count_servo",
        "drive_pwm_write_count_drive_reverse",
        "drive_pwm_skip_count_drive_reverse",
        "drive_pwm_write_count_drive_forward",
        "drive_pwm_skip_count_drive_forward",
        "db_internal_loop_ms",
        "db_internal_loop_overrun_ms",
    ]
    print("  timing fields p95/max:")
    for key in fields:
        vals = [f(r, key) for r in rows]
        finite = [v for v in vals if math.isfinite(v)]
        if not finite:
            continue
        print(f"    {key:42s} p95={fmt(pct(finite,0.95))} max={fmt(max(finite))}")

    pwm_channels = ("servo", "front_led", "drive_reverse", "drive_forward")
    pwm_parts = []
    for channel in pwm_channels:
        writes = [f(r, f"drive_pwm_write_count_{channel}") for r in rows]
        skips = [f(r, f"drive_pwm_skip_count_{channel}") for r in rows]
        fallbacks = [f(r, f"drive_pwm_cache_fallback_count_{channel}") for r in rows]
        write_vals = [v for v in writes if math.isfinite(v)]
        skip_vals = [v for v in skips if math.isfinite(v)]
        fallback_vals = [v for v in fallbacks if math.isfinite(v)]
        if write_vals or skip_vals or fallback_vals:
            write_delta = max(write_vals) - min(write_vals) if write_vals else 0.0
            skip_delta = max(skip_vals) - min(skip_vals) if skip_vals else 0.0
            fallback_delta = max(fallback_vals) - min(fallback_vals) if fallback_vals else 0.0
            pwm_parts.append(f"{channel}:w={write_delta:.0f},skip={skip_delta:.0f},fallback={fallback_delta:.0f}")
    if pwm_parts:
        print("  pwm duty counter deltas: " + "; ".join(pwm_parts))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, nargs="?", default=Path("logs/latest"), help="Run directory or drive_debug.csv")
    parser.add_argument("--top", type=int, default=15, help="Top slow rows to print")
    args = parser.parse_args()

    csv_path = find_csv(args.path)
    with csv_path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    print(f"CSV: {csv_path}")
    print(f"Rows: {len(rows)}")
    summarize(rows, "all")

    nav_rows = [r for r in rows if r.get("actuation_allowed") == "1" and abs(f(r, "cmd_target_speed_mps", 0.0)) > 0.02]
    summarize(nav_rows, "actuation moving")

    by_state = {}
    for r in rows:
        key = r.get("navigation_state") or r.get("cmd_reason") or "unknown"
        by_state.setdefault(key, []).append(r)
    for key, state_rows in sorted(by_state.items(), key=lambda kv: len(kv[1]), reverse=True)[:8]:
        summarize(state_rows, f"state {key}")

    print("\nTop slow drive rows:")
    ranked = sorted(rows, key=lambda r: f(r, "loop_ms", 0.0), reverse=True)[: max(0, args.top)]
    for r in ranked:
        print(
            f"  t={r.get('t','')} loop={r.get('loop_ms','')} over={r.get('drive_loop_overrun_ms','')} "
            f"cmd={r.get('cmd_target_speed_mps','')} steer={r.get('cmd_steering_norm','')} "
            f"motion={r.get('drive_t_motion_command_ms','')} set_steer={r.get('drive_t_set_steering_ms','')} "
            f"db_steer_wait={r.get('drive_t_db_set_steering_lock_wait_ms','')} db_steer_pwm={r.get('drive_t_db_set_steering_pwm_ms','')} "
            f"steer_apply={r.get('drive_t_db_steering_apply_duration_ms','')} steer_age={r.get('db_steering_apply_age_ms','')} "
            f"steer_pending_age={r.get('db_steering_pending_age_ms','')} "
            f"motion_apply={r.get('drive_t_db_motion_apply_duration_ms','')} motion_age={r.get('db_motion_apply_age_ms','')} "
            f"motion_pending_age={r.get('db_motion_pending_age_ms','')} mode={r.get('db_motion_applied_mode','')} "
            f"led_apply={r.get('drive_t_db_led_apply_duration_ms','')} led_age={r.get('db_led_apply_age_ms','')} "
            f"db_get={r.get('drive_t_db_get_telemetry_call_ms','')} "
            f"db_get_wait={r.get('drive_t_db_get_telemetry_lock_wait_ms','')} hw={r.get('drive_timing_hw_pwm_max_ms','')} "
            f"pwm_w={r.get('drive_pwm_write_count_total','')} pwm_skip={r.get('drive_pwm_skip_count_total','')} "
            f"pwm_fd={r.get('drive_pwm_cached_fd_count','')} pwm_fallback={r.get('drive_pwm_cache_fallback_count_total','')} "
            f"events={r.get('drive_timing_event_summary','')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
