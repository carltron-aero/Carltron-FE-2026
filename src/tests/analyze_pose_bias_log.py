#!/usr/bin/env python3
"""Summarize fused-vs-OTOS pose bias and LiDAR correction behavior from drive logs.

Examples:
    python3 src/tests/analyze_pose_bias_log.py logs/latest
    python3 src/tests/analyze_pose_bias_log.py logs/2026-06-04_15-30-23/drive_debug.csv

The script is read-only. It uses fields added for LiDAR bias diagnosis and falls
back gracefully when older logs do not contain all columns.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Iterable

SECTIONS = ("bottom", "left", "top", "right")
CORRECTION_SOURCES = {"lidar_axis_correction", "lidar_drift_correction", "lidar_correction"}


def _resolve_drive_log(path: Path) -> Path:
    if path.is_dir():
        for name in ("drive_debug.csv", "fe26_drive_debug_latest.csv"):
            candidate = path / name
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f"no drive_debug.csv found in {path}")
    return path


def _float(row: dict[str, str], key: str) -> float | None:
    try:
        value = row.get(key, "")
        if value in (None, ""):
            return None
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _int(row: dict[str, str], key: str) -> int | None:
    try:
        value = row.get(key, "")
        if value in (None, ""):
            return None
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _boolish(row: dict[str, str], key: str) -> bool:
    value = str(row.get(key, "")).strip().lower()
    return value in {"1", "true", "yes", "on"}


def _median(values: Iterable[float]) -> float | None:
    data = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return statistics.median(data) if data else None


def _mean(values: Iterable[float]) -> float | None:
    data = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return statistics.fmean(data) if data else None


def _p95(values: Iterable[float]) -> float | None:
    data = sorted(float(v) for v in values if v is not None and math.isfinite(float(v)))
    if not data:
        return None
    return data[min(len(data) - 1, int(math.ceil(0.95 * len(data))) - 1)]


def _fmt(value: float | None, unit: str = "", digits: int = 3) -> str:
    if value is None:
        return "--"
    return f"{value:.{digits}f}{unit}"


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as fp:
        return list(csv.DictReader(fp))


def _moving_navigation_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    out = []
    for row in rows:
        if not _boolish(row, "nav_enabled"):
            continue
        speed = _float(row, "otos_forward_mps")
        if speed is None or abs(speed) < 0.05:
            continue
        out.append(row)
    return out


def _unique_anchor_correction_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    selected = []
    last_version = None
    for row in rows:
        version = row.get("anchor_version", "")
        source = row.get("anchor_source", "")
        if version == "" or version == last_version:
            continue
        last_version = version
        if source in CORRECTION_SOURCES or source.startswith("lidar_"):
            selected.append(row)
    return selected


def _section_rows(rows: list[dict[str, str]], section: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("nav_section", "") == section]


def _section_summary(rows: list[dict[str, str]], section: str) -> str:
    group = _section_rows(rows, section)
    if not group:
        return f"{section:>6}: no moving navigation rows"
    fused = [_float(row, "nav_lateral_fused_m") for row in group]
    otos = [_float(row, "nav_lateral_otos_only_m") for row in group]
    lateral_delta = [_float(row, "nav_lateral_fused_minus_otos_m") for row in group]
    pose_dist = [_float(row, "fused_minus_otos_distance_m") for row in group]
    pose_dx = [_float(row, "fused_minus_otos_x_m") for row in group]
    pose_dy = [_float(row, "fused_minus_otos_y_m") for row in group]
    pose_dh = [_float(row, "fused_minus_otos_h_deg") for row in group]
    conf = [_float(row, "nav_pose_confidence") for row in group]
    return (
        f"{section:>6}: n={len(group):5d} "
        f"lat fused med={_fmt(_median(fused),'m')} otos med={_fmt(_median(otos),'m')} "
        f"fused-otos lat med={_fmt(_median(lateral_delta),'m')} mean={_fmt(_mean(lateral_delta),'m')} "
        f"pose delta med={_fmt(_median(pose_dist),'m')} p95={_fmt(_p95(pose_dist),'m')} "
        f"dx med={_fmt(_median(pose_dx),'m')} dy med={_fmt(_median(pose_dy),'m')} dh med={_fmt(_median(pose_dh),'deg',2)} "
        f"conf med={_fmt(_median(conf),'',2)}"
    )


def _correction_summary(rows: list[dict[str, str]]) -> list[str]:
    anchors = _unique_anchor_correction_rows(rows)
    lines = [f"unique LiDAR anchor updates: {len(anchors)}"]
    if not anchors:
        return lines
    step_x = [_float(row, "lidar_correction_axis_step_x") for row in anchors]
    step_y = [_float(row, "lidar_correction_axis_step_y") for row in anchors]
    step_h = [_float(row, "lidar_correction_axis_step_h") for row in anchors]
    raw_x = [_float(row, "lidar_correction_axis_x_weighted_residual") for row in anchors]
    raw_y = [_float(row, "lidar_correction_axis_y_weighted_residual") for row in anchors]
    lines.append(
        "  total steps: "
        f"dx={_fmt(sum(v for v in step_x if v is not None),'m')} "
        f"dy={_fmt(sum(v for v in step_y if v is not None),'m')} "
        f"dh={_fmt(sum(v for v in step_h if v is not None),'deg',2)}"
    )
    lines.append(
        "  selected residuals: "
        f"x med={_fmt(_median(raw_x),'m')} y med={_fmt(_median(raw_y),'m')} "
        f"x mean={_fmt(_mean(raw_x),'m')} y mean={_fmt(_mean(raw_y),'m')}"
    )
    for section in SECTIONS:
        group = _section_rows(anchors, section)
        if not group:
            continue
        lines.append(
            f"  {section:>6}: updates={len(group):4d} "
            f"sum_dx={_fmt(sum(v for v in (_float(r,'lidar_correction_axis_step_x') for r in group) if v is not None),'m')} "
            f"sum_dy={_fmt(sum(v for v in (_float(r,'lidar_correction_axis_step_y') for r in group) if v is not None),'m')} "
            f"sum_dh={_fmt(sum(v for v in (_float(r,'lidar_correction_axis_step_h') for r in group) if v is not None),'deg',2)}"
        )
    return lines


def _first_confidence_collapse(rows: list[dict[str, str]], threshold: float) -> str:
    for row in rows:
        conf = _float(row, "nav_pose_confidence")
        if conf is not None and conf < threshold:
            return (
                f"first nav_pose_confidence < {threshold:.2f}: "
                f"t={row.get('t','--')} section={row.get('nav_section','--')} conf={conf:.3f} "
                f"reason={row.get('nav_reason','')} lateral={row.get('nav_lateral_m','')} "
                f"fused-otos={row.get('fused_minus_otos_distance_m','')}m"
            )
    return f"first nav_pose_confidence < {threshold:.2f}: none"


def _print_deskew_companion(drive_log: Path, include_deskew: bool) -> None:
    if not include_deskew:
        return
    run_dir = drive_log.parent
    index = run_dir / "lidar_deskew_debug_index.json"
    if not index.exists():
        return
    analyzer = Path(__file__).resolve().parent / "analyze_lidar_deskew_debug.py"
    if not analyzer.exists():
        print(f"\nDeskew diagnostic index: {index}")
        return
    print("\nDeskew timing-model comparison:")
    print(f"  command: python3 {analyzer} {index} --worst 8")
    try:
        subprocess.run([sys.executable, str(analyzer), str(index), "--worst", "8"], check=False)
    except Exception as exc:
        print(f"  skipped embedded deskew analyzer: {exc}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze pose bias and LiDAR correction behavior from drive_debug.csv")
    parser.add_argument("path", type=Path, help="Run directory or drive_debug.csv")
    parser.add_argument("--low-confidence", type=float, default=0.5, help="Confidence threshold for collapse marker")
    parser.add_argument("--no-deskew", dest="include_deskew", action="store_false", help="Do not invoke deskew analyzer when a diagnostic index exists")
    args = parser.parse_args(argv)

    drive_log = _resolve_drive_log(args.path)
    rows = _rows(drive_log)
    moving = _moving_navigation_rows(rows)
    print(f"Drive log: {drive_log}")
    print(f"rows: {len(rows)}  moving navigation rows: {len(moving)}")
    if rows:
        t0 = _float(rows[0], "t")
        t1 = _float(rows[-1], "t")
        if t0 is not None and t1 is not None:
            print(f"duration: {t1 - t0:.3f}s")

    print("\nSection fused-vs-OTOS lateral summary:")
    for section in SECTIONS:
        print("  " + _section_summary(moving, section))

    print("\nLiDAR anchor/correction summary:")
    for line in _correction_summary(rows):
        print("  " + line)

    print("\nConfidence:")
    print("  " + _first_confidence_collapse(rows, args.low_confidence))

    print("\nUseful next columns to inspect when bias appears:")
    print("  fused_minus_otos_x_m, fused_minus_otos_y_m, fused_minus_otos_h_deg")
    print("  nav_lateral_fused_m, nav_lateral_otos_only_m, nav_lateral_fused_minus_otos_m")
    print("  lidar_correction_axis_x_selected_residuals, lidar_correction_axis_y_selected_residuals")

    _print_deskew_companion(drive_log, args.include_deskew)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
