#!/usr/bin/env python3
"""Analyze turn-coupled fused-vs-OTOS pose error from drive_debug.csv.

This tool is read-only. It is meant for diagnosing the current FE26 pose issue
where fused pose and OTOS-only pose agree on straights, but a repeatable offset
appears during or immediately after corners.

Examples:
    python3 src/tests/analyze_turn_coupled_pose_error.py logs/latest
    python3 src/tests/analyze_turn_coupled_pose_error.py logs/2026-06-08_22-52-04 --csv /tmp/turn_impulses.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


SECTION_INNER_AXIS = {
    "bottom": (0.0, 1.0),
    "right": (-1.0, 0.0),
    "top": (0.0, -1.0),
    "left": (1.0, 0.0),
}

SECTION_ALONG_AXIS = {
    "bottom": (1.0, 0.0),
    "right": (0.0, 1.0),
    "top": (-1.0, 0.0),
    "left": (0.0, -1.0),
}


@dataclass
class Row:
    index: int
    t: float
    section: str
    phase: str
    x: float | None
    y: float | None
    h: float | None
    ox: float | None
    oy: float | None
    oh: float | None
    dx: float | None
    dy: float | None
    dh: float | None
    dist: float | None
    step_x: float
    step_y: float
    step_h: float
    yaw_dps: float | None
    yaw_authority: float | None
    lidar_confidence: float | None
    nav_lateral_fused: float | None
    nav_lateral_otos: float | None
    nav_lateral_delta: float | None
    gate_pass_count: float | None


@dataclass
class Summary:
    n: int
    t0: float | None
    t1: float | None
    dx: float | None
    dy: float | None
    dh: float | None
    dist: float | None
    inner: float | None
    along: float | None
    robot_forward: float | None
    robot_left: float | None
    lateral_delta: float | None
    yaw_abs_median: float | None
    correction_x: float
    correction_y: float
    correction_h: float
    correction_inner: float | None
    correction_along: float | None
    correction_count: int


def resolve_drive_log(path: Path) -> Path:
    if path.is_dir():
        candidate = path / "drive_debug.csv"
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"no drive_debug.csv found in {path}")
    return path


def as_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def finite(values: Iterable[float | None]) -> list[float]:
    return [float(v) for v in values if v is not None and math.isfinite(float(v))]


def median(values: Iterable[float | None]) -> float | None:
    vals = finite(values)
    return statistics.median(vals) if vals else None


def mean(values: Iterable[float | None]) -> float | None:
    vals = finite(values)
    return statistics.fmean(vals) if vals else None


def p95(values: Iterable[float | None]) -> float | None:
    vals = sorted(finite(values))
    if not vals:
        return None
    return vals[min(len(vals) - 1, int(math.ceil(0.95 * len(vals))) - 1)]


def fmt(value: float | None, unit: str = "", digits: int = 3) -> str:
    if value is None:
        return "--"
    return f"{value:.{digits}f}{unit}"


def read_rows(path: Path) -> list[Row]:
    out: list[Row] = []
    with path.open(newline="") as fp:
        reader = csv.DictReader(fp)
        for index, row in enumerate(reader):
            t = as_float(row, "t")
            if t is None:
                continue
            out.append(
                Row(
                    index=index,
                    t=t,
                    section=str(row.get("nav_section", "")),
                    phase=str(row.get("race_phase", "")),
                    x=as_float(row, "otos_display_x_m"),
                    y=as_float(row, "otos_display_y_m"),
                    h=as_float(row, "otos_display_h_deg"),
                    ox=as_float(row, "otos_only_x_m"),
                    oy=as_float(row, "otos_only_y_m"),
                    oh=as_float(row, "otos_only_h_deg"),
                    dx=as_float(row, "fused_minus_otos_x_m"),
                    dy=as_float(row, "fused_minus_otos_y_m"),
                    dh=as_float(row, "fused_minus_otos_h_deg"),
                    dist=as_float(row, "fused_minus_otos_distance_m"),
                    step_x=as_float(row, "lidar_correction_axis_step_x") or 0.0,
                    step_y=as_float(row, "lidar_correction_axis_step_y") or 0.0,
                    step_h=as_float(row, "lidar_correction_axis_step_h") or 0.0,
                    yaw_dps=as_float(row, "lidar_correction_yaw_rate_dps"),
                    yaw_authority=as_float(row, "lidar_correction_yaw_authority"),
                    lidar_confidence=as_float(row, "lidar_confidence"),
                    nav_lateral_fused=as_float(row, "nav_lateral_fused_m"),
                    nav_lateral_otos=as_float(row, "nav_lateral_otos_only_m"),
                    nav_lateral_delta=as_float(row, "nav_lateral_fused_minus_otos_m"),
                    gate_pass_count=as_float(row, "race_full_gate_pass_count"),
                )
            )
    return out


def navigation_rows(rows: list[Row], min_speed_rows: bool = False) -> list[Row]:
    nav = [row for row in rows if row.phase == "navigation"]
    if not min_speed_rows:
        return nav
    return [row for row in nav if row.dist is not None]


def dot(x: float, y: float, axis: tuple[float, float]) -> float:
    return x * axis[0] + y * axis[1]


def section_inner_along(section: str, dx: float | None, dy: float | None) -> tuple[float | None, float | None]:
    if dx is None or dy is None:
        return None, None
    inner_axis = SECTION_INNER_AXIS.get(section)
    along_axis = SECTION_ALONG_AXIS.get(section)
    if inner_axis is None or along_axis is None:
        return None, None
    return dot(dx, dy, inner_axis), dot(dx, dy, along_axis)


def robot_local(h_deg: float | None, dx: float | None, dy: float | None) -> tuple[float | None, float | None]:
    if h_deg is None or dx is None or dy is None:
        return None, None
    h = math.radians(h_deg)
    c = math.cos(h)
    s = math.sin(h)
    forward = dx * c + dy * s
    left = -dx * s + dy * c
    return forward, left


def summarize(rows: list[Row], section: str | None = None) -> Summary:
    if not rows:
        return Summary(0, None, None, None, None, None, None, None, None, None, None, None, None, 0.0, 0.0, 0.0, None, None, 0)
    dx = median(row.dx for row in rows)
    dy = median(row.dy for row in rows)
    dh = median(row.dh for row in rows)
    dist = median(row.dist for row in rows)
    if section is None:
        sections = [row.section for row in rows if row.section]
        section = max(set(sections), key=sections.count) if sections else ""
    inner, along = section_inner_along(section, dx, dy)
    h = median(row.h for row in rows)
    robot_forward, robot_left = robot_local(h, dx, dy)
    sx = sum(row.step_x for row in rows)
    sy = sum(row.step_y for row in rows)
    sh = sum(row.step_h for row in rows)
    cinner, calong = section_inner_along(section, sx, sy)
    correction_count = sum(1 for row in rows if abs(row.step_x) + abs(row.step_y) + abs(row.step_h) > 1e-8)
    return Summary(
        n=len(rows),
        t0=rows[0].t,
        t1=rows[-1].t,
        dx=dx,
        dy=dy,
        dh=dh,
        dist=dist,
        inner=inner,
        along=along,
        robot_forward=robot_forward,
        robot_left=robot_left,
        lateral_delta=median(row.nav_lateral_delta for row in rows),
        yaw_abs_median=median(abs(row.yaw_dps) for row in rows if row.yaw_dps is not None),
        correction_x=sx,
        correction_y=sy,
        correction_h=sh,
        correction_inner=cinner,
        correction_along=calong,
        correction_count=correction_count,
    )


def rows_between(rows: list[Row], start_t: float, end_t: float) -> list[Row]:
    return [row for row in rows if start_t <= row.t <= end_t]


def section_transitions(rows: list[Row]) -> list[tuple[int, str, str]]:
    transitions: list[tuple[int, str, str]] = []
    prev = ""
    for idx, row in enumerate(rows):
        section = row.section
        if not section:
            continue
        if prev and section != prev:
            transitions.append((idx, prev, section))
        prev = section
    return transitions


def yaw_windows(rows: list[Row], threshold_dps: float, min_duration_s: float) -> list[tuple[int, int]]:
    windows: list[tuple[int, int]] = []
    start: int | None = None
    for idx, row in enumerate(rows):
        yaw = abs(row.yaw_dps or 0.0)
        if yaw >= threshold_dps and start is None:
            start = idx
        elif start is not None and yaw < threshold_dps:
            end = idx - 1
            if rows[end].t - rows[start].t >= min_duration_s:
                windows.append((start, end))
            start = None
    if start is not None and rows[-1].t - rows[start].t >= min_duration_s:
        windows.append((start, len(rows) - 1))
    return windows


def diff_summary(after: Summary, before: Summary, section: str) -> dict[str, float | None]:
    dx = None if after.dx is None or before.dx is None else after.dx - before.dx
    dy = None if after.dy is None or before.dy is None else after.dy - before.dy
    dh = None if after.dh is None or before.dh is None else after.dh - before.dh
    inner, along = section_inner_along(section, dx, dy)
    return {
        "dx": dx,
        "dy": dy,
        "dh": dh,
        "inner": inner,
        "along": along,
        "dist": None if dx is None or dy is None else math.hypot(dx, dy),
    }


def print_summary(label: str, summary: Summary) -> None:
    print(
        f"{label:<10} n={summary.n:4d} "
        f"dxy={fmt(summary.dist,'m')} dx={fmt(summary.dx,'m')} dy={fmt(summary.dy,'m')} dh={fmt(summary.dh,'deg',2)} "
        f"inner={fmt(summary.inner,'m')} along={fmt(summary.along,'m')} "
        f"robot_fwd={fmt(summary.robot_forward,'m')} robot_left={fmt(summary.robot_left,'m')} "
        f"lat_delta={fmt(summary.lateral_delta,'m')} yaw_med={fmt(summary.yaw_abs_median,'dps',1)} "
        f"corr=({summary.correction_x:+.3f},{summary.correction_y:+.3f},{summary.correction_h:+.2f}deg n={summary.correction_count})"
    )


def analyze_transitions(rows: list[Row], args) -> list[dict[str, object]]:
    events: list[dict[str, object]] = []
    transitions = section_transitions(rows)
    print("\nSection-transition impulse analysis")
    print(
        "  Convention: fused-minus-OTOS projected into the target section. "
        "positive inner means fused pose is closer to field center than OTOS-only."
    )
    if not transitions:
        print("  no section transitions found")
        return events
    for event_index, (idx, prev_section, section) in enumerate(transitions, start=1):
        row = rows[idx]
        t = row.t
        pre = summarize(rows_between(rows, t - args.pre_window_s, t - args.pre_gap_s), prev_section)
        post = summarize(rows_between(rows, t + args.post_gap_s, t + args.post_window_s), section)
        settled = summarize(rows_between(rows, t + args.settle_start_s, t + args.settle_end_s), section)
        corner = summarize(rows_between(rows, t - args.corner_context_before_s, t + args.corner_context_after_s), section)
        impulse = diff_summary(post, pre, section)
        cleanup = diff_summary(settled, post, section)
        events.append(
            {
                "type": "section_transition",
                "event": event_index,
                "t": t,
                "prev_section": prev_section,
                "section": section,
                "heading_deg": row.h,
                "gate_pass_count": row.gate_pass_count,
                "pre": pre,
                "post": post,
                "settled": settled,
                "corner": corner,
                "impulse": impulse,
                "cleanup": cleanup,
            }
        )
        print()
        print(
            f"{event_index:02d} {prev_section}->{section} "
            f"t={t:.3f}s h={fmt(row.h,'deg',1)} gate={fmt(row.gate_pass_count,'',0)}"
        )
        print_summary("pre", pre)
        print_summary("post", post)
        print_summary("settled", settled)
        print(
            "impulse   "
            f"inner={fmt(impulse['inner'],'m')} along={fmt(impulse['along'],'m')} "
            f"field=({fmt(impulse['dx'],'m')},{fmt(impulse['dy'],'m')}) "
            f"dh={fmt(impulse['dh'],'deg',2)} dist={fmt(impulse['dist'],'m')}"
        )
        print(
            "cleanup   "
            f"inner={fmt(cleanup['inner'],'m')} along={fmt(cleanup['along'],'m')} "
            f"field=({fmt(cleanup['dx'],'m')},{fmt(cleanup['dy'],'m')}) "
            f"dh={fmt(cleanup['dh'],'deg',2)} dist={fmt(cleanup['dist'],'m')}"
        )
    return events


def analyze_yaw_windows(rows: list[Row], args) -> list[dict[str, object]]:
    windows = yaw_windows(rows, args.yaw_threshold_dps, args.yaw_min_duration_s)
    events: list[dict[str, object]] = []
    print("\nYaw-window analysis")
    print(f"  threshold={args.yaw_threshold_dps:.1f} dps min_duration={args.yaw_min_duration_s:.2f}s")
    if not windows:
        print("  no yaw windows found")
        return events
    for event_index, (start, end) in enumerate(windows, start=1):
        first = rows[start]
        last = rows[end]
        section = last.section or first.section
        pre = summarize(rows_between(rows, first.t - args.yaw_pre_s, first.t - args.pre_gap_s), first.section)
        turn = summarize(rows[start : end + 1], section)
        post = summarize(rows_between(rows, last.t + args.post_gap_s, last.t + args.yaw_post_s), section)
        impulse = diff_summary(post, pre, section)
        h0 = first.h
        h1 = last.h
        heading_delta = None if h0 is None or h1 is None else (h1 - h0 + 180.0) % 360.0 - 180.0
        events.append(
            {
                "type": "yaw_window",
                "event": event_index,
                "t_start": first.t,
                "t_end": last.t,
                "section_start": first.section,
                "section_end": last.section,
                "heading_start_deg": h0,
                "heading_end_deg": h1,
                "heading_delta_deg": heading_delta,
                "pre": pre,
                "turn": turn,
                "post": post,
                "impulse": impulse,
            }
        )
        print()
        print(
            f"{event_index:02d} {first.section}->{last.section} "
            f"t={first.t:.3f}-{last.t:.3f}s dur={last.t - first.t:.2f}s "
            f"h={fmt(h0,'deg',1)}->{fmt(h1,'deg',1)} dh={fmt(heading_delta,'deg',1)}"
        )
        print_summary("pre", pre)
        print_summary("turn", turn)
        print_summary("post", post)
        print(
            "impulse   "
            f"inner={fmt(impulse['inner'],'m')} along={fmt(impulse['along'],'m')} "
            f"field=({fmt(impulse['dx'],'m')},{fmt(impulse['dy'],'m')}) "
            f"dh={fmt(impulse['dh'],'deg',2)} dist={fmt(impulse['dist'],'m')}"
        )
    return events


def aggregate_transition_events(events: list[dict[str, object]]) -> None:
    groups: dict[str, list[dict[str, object]]] = {}
    for event in events:
        key = f"{event['prev_section']}->{event['section']}"
        groups.setdefault(key, []).append(event)
    print("\nRepeated transition aggregate")
    if not groups:
        print("  no transition groups")
        return
    for key in sorted(groups):
        group = groups[key]
        impulses = [event["impulse"] for event in group]
        cleanups = [event["cleanup"] for event in group]
        print(
            f"  {key:<13} n={len(group)} "
            f"impulse_inner med={fmt(median(item['inner'] for item in impulses),'m')} "
            f"mean={fmt(mean(item['inner'] for item in impulses),'m')} "
            f"p95={fmt(p95(abs(item['inner']) if item['inner'] is not None else None for item in impulses),'m')} "
            f"impulse_along med={fmt(median(item['along'] for item in impulses),'m')} "
            f"cleanup_inner med={fmt(median(item['inner'] for item in cleanups),'m')} "
            f"cleanup_along med={fmt(median(item['along'] for item in cleanups),'m')}"
        )


def write_csv(path: Path, events: list[dict[str, object]]) -> None:
    fieldnames = [
        "type",
        "event",
        "t",
        "t_start",
        "t_end",
        "prev_section",
        "section",
        "section_start",
        "section_end",
        "heading_deg",
        "heading_start_deg",
        "heading_end_deg",
        "heading_delta_deg",
        "gate_pass_count",
        "pre_dx_m",
        "pre_dy_m",
        "pre_dh_deg",
        "pre_inner_m",
        "pre_along_m",
        "post_dx_m",
        "post_dy_m",
        "post_dh_deg",
        "post_inner_m",
        "post_along_m",
        "settled_dx_m",
        "settled_dy_m",
        "settled_dh_deg",
        "settled_inner_m",
        "settled_along_m",
        "impulse_dx_m",
        "impulse_dy_m",
        "impulse_dh_deg",
        "impulse_inner_m",
        "impulse_along_m",
        "cleanup_dx_m",
        "cleanup_dy_m",
        "cleanup_dh_deg",
        "cleanup_inner_m",
        "cleanup_along_m",
        "turn_correction_x_m",
        "turn_correction_y_m",
        "turn_correction_h_deg",
        "turn_correction_count",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for event in events:
            row: dict[str, object] = {name: "" for name in fieldnames}
            for key in (
                "type",
                "event",
                "t",
                "t_start",
                "t_end",
                "prev_section",
                "section",
                "section_start",
                "section_end",
                "heading_deg",
                "heading_start_deg",
                "heading_end_deg",
                "heading_delta_deg",
                "gate_pass_count",
            ):
                if key in event:
                    row[key] = event[key]
            for prefix in ("pre", "post", "settled"):
                summary = event.get(prefix)
                if isinstance(summary, Summary):
                    row[f"{prefix}_dx_m"] = summary.dx if summary.dx is not None else ""
                    row[f"{prefix}_dy_m"] = summary.dy if summary.dy is not None else ""
                    row[f"{prefix}_dh_deg"] = summary.dh if summary.dh is not None else ""
                    row[f"{prefix}_inner_m"] = summary.inner if summary.inner is not None else ""
                    row[f"{prefix}_along_m"] = summary.along if summary.along is not None else ""
                    if prefix == "turn":
                        row["turn_correction_x_m"] = summary.correction_x
                        row["turn_correction_y_m"] = summary.correction_y
                        row["turn_correction_h_deg"] = summary.correction_h
                        row["turn_correction_count"] = summary.correction_count
            turn = event.get("turn")
            if isinstance(turn, Summary):
                row["turn_correction_x_m"] = turn.correction_x
                row["turn_correction_y_m"] = turn.correction_y
                row["turn_correction_h_deg"] = turn.correction_h
                row["turn_correction_count"] = turn.correction_count
            for prefix in ("impulse", "cleanup"):
                values = event.get(prefix)
                if isinstance(values, dict):
                    row[f"{prefix}_dx_m"] = values.get("dx", "")
                    row[f"{prefix}_dy_m"] = values.get("dy", "")
                    row[f"{prefix}_dh_deg"] = values.get("dh", "")
                    row[f"{prefix}_inner_m"] = values.get("inner", "")
                    row[f"{prefix}_along_m"] = values.get("along", "")
            writer.writerow(row)
    print(f"\nWrote CSV: {path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze turn-coupled pose error from FE26 drive logs")
    parser.add_argument("path", type=Path, help="Run directory or drive_debug.csv")
    parser.add_argument("--pre-window-s", type=float, default=2.0, help="Pre-transition summary window length")
    parser.add_argument("--pre-gap-s", type=float, default=0.2, help="Gap before event excluded from pre summary")
    parser.add_argument("--post-window-s", type=float, default=2.0, help="Immediate post-transition summary window length")
    parser.add_argument("--post-gap-s", type=float, default=0.2, help="Gap after event excluded from post summary")
    parser.add_argument("--settle-start-s", type=float, default=2.0, help="Settled summary start after transition")
    parser.add_argument("--settle-end-s", type=float, default=7.0, help="Settled summary end after transition")
    parser.add_argument("--corner-context-before-s", type=float, default=2.0, help="Correction context before transition")
    parser.add_argument("--corner-context-after-s", type=float, default=2.0, help="Correction context after transition")
    parser.add_argument("--yaw-threshold-dps", type=float, default=8.0, help="Yaw-rate threshold for turn windows")
    parser.add_argument("--yaw-min-duration-s", type=float, default=1.0, help="Minimum yaw window duration")
    parser.add_argument("--yaw-pre-s", type=float, default=1.0, help="Pre-yaw summary window length")
    parser.add_argument("--yaw-post-s", type=float, default=2.0, help="Post-yaw summary window length")
    parser.add_argument("--csv", type=Path, default=None, help="Optional output CSV for event-level metrics")
    args = parser.parse_args(argv)

    drive_log = resolve_drive_log(args.path)
    rows = read_rows(drive_log)
    nav = navigation_rows(rows)
    print(f"Drive log: {drive_log}")
    print(f"rows: {len(rows)} navigation rows: {len(nav)}")
    if nav:
        print(f"navigation duration: {nav[-1].t - nav[0].t:.3f}s")
        total = summarize(nav)
        print_summary("all nav", total)

    transition_events = analyze_transitions(nav, args)
    aggregate_transition_events(transition_events)
    yaw_events = analyze_yaw_windows(nav, args)

    if args.csv is not None:
        write_csv(args.csv, transition_events + yaw_events)

    print("\nInterpretation hints:")
    print("  - Stable same-sign impulse_inner by transition means the error is repeatable and corner/section tied.")
    print("  - Large impulse during yaw windows but cleanup on straights means LiDAR is correcting a turn-created OTOS error.")
    print("  - If impulse appears only with high scan age or high yaw, inspect deskew/timing before adding model compensation.")
    print("  - If fused-minus-OTOS grows while physical pose follows fused, OTOS-only is wrong; if physical follows OTOS, LiDAR correction is wrong.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
