#!/usr/bin/env python3
"""Analyze OTOS straight-line scale from stationary LiDAR endpoint captures.

Workflow:
1. Run drivebase_speed_calibration_suite.py with --lidar-endpoint-capture.
2. Open the produced *_lidar_endpoint_index.json in lidar_deskew_viewer.py.
3. Manually align each endpoint frame and save the viewer annotations.
4. Run this script with the calibration CSV and endpoint index.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from analyze_lidar_deskew_debug import _load_frames


def _float(value: Any, default: float = float("nan")) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _wrap_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _pose_from_tuple(value: Any) -> tuple[float, float, float] | None:
    if value is None:
        return None
    vals = list(value)
    if len(vals) < 4:
        return None
    x = _float(vals[1])
    y = _float(vals[2])
    h = _float(vals[3])
    if not all(math.isfinite(v) for v in (x, y, h)):
        return None
    return x, y, h


def _frame_base_pose(frame: dict[str, Any]) -> tuple[float, float, float] | None:
    for key in ("otos_only_pose", "fused_pose", "reference_pose", "current_pose"):
        if key in frame:
            pose = _pose_from_tuple(frame.get(key))
            if pose is not None:
                return pose
    return None


def _load_annotations(path: Path) -> dict[int, dict[str, float]]:
    with path.open("r", encoding="utf-8") as fp:
        data = json.load(fp)
    frames = data.get("frames", {}) if isinstance(data, dict) else {}
    out: dict[int, dict[str, float]] = {}
    iterable = frames.items() if isinstance(frames, dict) else []
    for key, value in iterable:
        if not isinstance(value, dict):
            continue
        try:
            idx = int(key)
        except (TypeError, ValueError):
            continue
        out[idx] = {
            "dx_m": _float(value.get("dx_m"), 0.0),
            "dy_m": _float(value.get("dy_m"), 0.0),
            "dh_deg": _float(value.get("dh_deg"), 0.0),
        }
    return out


def _aligned_pose(frame: dict[str, Any], annotation: dict[str, float]) -> tuple[float, float, float] | None:
    base = _frame_base_pose(frame)
    if base is None:
        return None
    return (
        base[0] + float(annotation.get("dx_m", 0.0)),
        base[1] + float(annotation.get("dy_m", 0.0)),
        _wrap_deg(base[2] + float(annotation.get("dh_deg", 0.0))),
    )


def _mean_pose(items: list[tuple[float, float, float]]) -> tuple[float, float, float] | None:
    if not items:
        return None
    xs = [p[0] for p in items]
    ys = [p[1] for p in items]
    sin_h = sum(math.sin(math.radians(p[2])) for p in items)
    cos_h = sum(math.cos(math.radians(p[2])) for p in items)
    return float(statistics.mean(xs)), float(statistics.mean(ys)), math.degrees(math.atan2(sin_h, cos_h))


def _read_csv_stats(path: Path) -> dict[str, dict[str, Any]]:
    by_segment: dict[str, dict[str, Any]] = {}
    with path.open("r", newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            segment = str(row.get("segment", ""))
            if not segment:
                continue
            info = by_segment.setdefault(segment, {"rows": [], "samples": []})
            info["rows"].append(row)
            if row.get("event") in ("sample", "end"):
                speed = abs(_float(row.get("otos_forward_mps")))
                if math.isfinite(speed):
                    info["samples"].append(speed)
            if row.get("event") == "start":
                info["start_row"] = row
            if row.get("event") == "end":
                info["end_row"] = row
    for info in by_segment.values():
        samples = info.get("samples", [])
        if samples:
            arr = np.asarray(samples, dtype=float)
            info["speed_median_mps"] = float(np.median(arr))
            info["speed_mean_mps"] = float(np.mean(arr))
            info["speed_p95_mps"] = float(np.percentile(arr, 95))
            info["speed_max_mps"] = float(np.max(arr))
        else:
            info["speed_median_mps"] = float("nan")
            info["speed_mean_mps"] = float("nan")
            info["speed_p95_mps"] = float("nan")
            info["speed_max_mps"] = float("nan")
    return by_segment


def _endpoint_groups(frames: list[dict[str, Any]], annotations: dict[int, dict[str, float]]) -> dict[str, dict[str, dict[str, Any]]]:
    grouped: dict[str, dict[str, dict[str, Any]]] = {}
    for idx, frame in enumerate(frames):
        cal = frame.get("calibration") if isinstance(frame.get("calibration"), dict) else {}
        if cal.get("kind") != "drivebase_speed_lidar_endpoint":
            continue
        segment = str(cal.get("segment", ""))
        phase = str(cal.get("phase", ""))
        if not segment or phase not in ("measurement_start", "measurement_end"):
            continue
        annotation = annotations.get(idx)
        if annotation is None:
            continue
        pose = _aligned_pose(frame, annotation)
        if pose is None:
            continue
        entry = grouped.setdefault(segment, {}).setdefault(phase, {"poses": [], "frames": [], "calibration": cal})
        entry["poses"].append(pose)
        entry["frames"].append(idx)
    for phases in grouped.values():
        for entry in phases.values():
            entry["pose"] = _mean_pose(entry["poses"])
    return grouped


def _analyze(csv_path: Path, index_path: Path, annotation_path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    with index_path.open("r", encoding="utf-8") as fp:
        index = json.load(fp)
    frames = _load_frames(index_path)
    annotations = _load_annotations(annotation_path)
    csv_stats = _read_csv_stats(csv_path)
    endpoints = _endpoint_groups(frames, annotations)
    current_scalar = _float(index.get("linear_scalar"), 1.0)
    rows: list[dict[str, Any]] = []
    for segment, phases in sorted(endpoints.items()):
        start = phases.get("measurement_start")
        end = phases.get("measurement_end")
        if not start or not end or not start.get("pose") or not end.get("pose"):
            continue
        start_pose = start["pose"]
        end_pose = end["pose"]
        start_cal = start.get("calibration", {})
        end_cal = end.get("calibration", {})
        csv_info = csv_stats.get(segment, {})
        start_lane = _float(start_cal.get("lane_coord_m"))
        end_lane = _float(end_cal.get("lane_coord_m"))
        otos_delta = end_lane - start_lane
        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        axis_h = math.radians(start_pose[2])
        true_along = dx * math.cos(axis_h) + dy * math.sin(axis_h)
        true_distance = math.hypot(dx, dy)
        ratio_abs = abs(true_along) / abs(otos_delta) if abs(otos_delta) > 1e-6 else float("nan")
        ratio_signed = true_along / otos_delta if abs(otos_delta) > 1e-6 else float("nan")
        recommended_scalar = current_scalar * ratio_abs if math.isfinite(ratio_abs) else float("nan")
        command_value = _float(start_cal.get("command_value"))
        row = {
            "segment": segment,
            "control": str(start_cal.get("control", "")),
            "planned_direction": str(start_cal.get("planned_direction", "")),
            "repeat": start_cal.get("repeat", ""),
            "command_value": command_value,
            "speed_median_mps": csv_info.get("speed_median_mps", float("nan")),
            "speed_mean_mps": csv_info.get("speed_mean_mps", float("nan")),
            "speed_p95_mps": csv_info.get("speed_p95_mps", float("nan")),
            "speed_max_mps": csv_info.get("speed_max_mps", float("nan")),
            "otos_delta_m": otos_delta,
            "true_along_m": true_along,
            "true_distance_m": true_distance,
            "ratio_abs": ratio_abs,
            "ratio_signed": ratio_signed,
            "linear_scalar_input": current_scalar,
            "recommended_linear_scalar": recommended_scalar,
            "start_frame_indices": ";".join(str(v) for v in start.get("frames", [])),
            "end_frame_indices": ";".join(str(v) for v in end.get("frames", [])),
            "start_x_m": start_pose[0],
            "start_y_m": start_pose[1],
            "start_h_deg": start_pose[2],
            "end_x_m": end_pose[0],
            "end_y_m": end_pose[1],
            "end_h_deg": end_pose[2],
            "lane_axis_from_start_h_deg": start_pose[2],
        }
        rows.append(row)
    summary = _summarize(rows)
    summary.update({
        "csv_path": str(csv_path),
        "index_path": str(index_path),
        "annotation_path": str(annotation_path),
        "frame_count": len(frames),
        "annotation_count": len(annotations),
        "linear_scalar_input": current_scalar,
        "analyzed_legs": len(rows),
    })
    return rows, summary


def _summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [r for r in rows if math.isfinite(_float(r.get("ratio_abs")))]
    if not valid:
        return {"overall": {}, "speed_bins": []}
    ratios = np.asarray([float(r["ratio_abs"]) for r in valid], dtype=float)
    scalars = np.asarray([float(r["recommended_linear_scalar"]) for r in valid], dtype=float)
    overall = {
        "ratio_median": float(np.median(ratios)),
        "ratio_mean": float(np.mean(ratios)),
        "ratio_p10": float(np.percentile(ratios, 10)),
        "ratio_p90": float(np.percentile(ratios, 90)),
        "recommended_scalar_median": float(np.median(scalars)),
        "recommended_scalar_mean": float(np.mean(scalars)),
    }
    bins: dict[float, list[dict[str, Any]]] = {}
    for row in valid:
        speed = _float(row.get("speed_median_mps"), _float(row.get("command_value")))
        if not math.isfinite(speed):
            continue
        bucket = round(speed / 0.1) * 0.1
        bins.setdefault(bucket, []).append(row)
    speed_bins = []
    for bucket, items in sorted(bins.items()):
        vals = np.asarray([float(item["recommended_linear_scalar"]) for item in items], dtype=float)
        ratios_b = np.asarray([float(item["ratio_abs"]) for item in items], dtype=float)
        speed_bins.append({
            "speed_bin_mps": float(bucket),
            "count": len(items),
            "speed_median_mps": float(np.median([float(item["speed_median_mps"]) for item in items if math.isfinite(_float(item.get("speed_median_mps")))] or [bucket])),
            "ratio_median": float(np.median(ratios_b)),
            "recommended_scalar_median": float(np.median(vals)),
            "recommended_scalar_min": float(np.min(vals)),
            "recommended_scalar_max": float(np.max(vals)),
        })
    return {"overall": overall, "speed_bins": speed_bins}


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_plot(path: Path, rows: list[dict[str, Any]], summary: dict[str, Any]) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False
    valid = [r for r in rows if math.isfinite(_float(r.get("recommended_linear_scalar"))) and math.isfinite(_float(r.get("speed_median_mps")))]
    if not valid:
        return False
    speeds = [float(r["speed_median_mps"]) for r in valid]
    scalars = [float(r["recommended_linear_scalar"]) for r in valid]
    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.scatter(speeds, scalars, color="#2060b8", label="aligned legs")
    bins = summary.get("speed_bins", [])
    if bins:
        bx = [float(b["speed_median_mps"]) for b in bins]
        by = [float(b["recommended_scalar_median"]) for b in bins]
        ax.plot(bx, by, color="#d64a2f", marker="o", label="0.1 m/s bin median")
    ax.set_xlabel("OTOS measured median speed (m/s)")
    ax.set_ylabel("Recommended linear scalar")
    ax.set_title("Speed-dependent OTOS linear scalar from stationary LiDAR endpoints")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def _default_annotation_path(index_path: Path) -> Path:
    return index_path.with_name(f"{index_path.stem}_manual_alignment.json")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze speed-dependent OTOS scale from LiDAR endpoint captures")
    parser.add_argument("csv", help="CSV from drivebase_speed_calibration_suite.py")
    parser.add_argument("endpoint_index", help="*_lidar_endpoint_index.json produced by --lidar-endpoint-capture")
    parser.add_argument("--annotations", default="", help="Manual alignment JSON; default is next to endpoint index")
    parser.add_argument("--output-prefix", default="", help="Output prefix; default is next to endpoint index")
    parser.add_argument("--no-plot", dest="plot", action="store_false", help="Do not write PNG plot")
    parser.set_defaults(plot=True)
    args = parser.parse_args(argv)

    csv_path = Path(args.csv).expanduser()
    index_path = Path(args.endpoint_index).expanduser()
    annotation_path = Path(args.annotations).expanduser() if args.annotations else _default_annotation_path(index_path)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")
    if not index_path.exists():
        raise SystemExit(f"endpoint index not found: {index_path}")
    if not annotation_path.exists():
        raise SystemExit(f"manual annotations not found: {annotation_path}")

    rows, summary = _analyze(csv_path, index_path, annotation_path)
    prefix = Path(args.output_prefix).expanduser() if args.output_prefix else index_path.with_name(index_path.stem.replace("_index", "") + "_scale_profile")
    rows_path = prefix.with_suffix(".csv")
    json_path = prefix.with_suffix(".json")
    png_path = prefix.with_suffix(".png")
    _write_rows(rows_path, rows)
    with json_path.open("w", encoding="utf-8") as fp:
        json.dump(summary, fp, indent=2, sort_keys=True)
        fp.write("\n")
    plot_written = _write_plot(png_path, rows, summary) if args.plot else False

    print(f"Analyzed legs: {len(rows)}")
    overall = summary.get("overall", {})
    if overall:
        print(f"Recommended scalar median: {overall.get('recommended_scalar_median', float('nan')):.6f}")
        print(f"Scale ratio median: {overall.get('ratio_median', float('nan')):.6f}")
    print(f"Rows: {rows_path}")
    print(f"Summary: {json_path}")
    if plot_written:
        print(f"Plot: {png_path}")
    else:
        print("Plot: not written")
    if summary.get("speed_bins"):
        print("\nSpeed bins:")
        for item in summary["speed_bins"]:
            print(
                f"  {item['speed_bin_mps']:.1f} m/s n={item['count']}: "
                f"scalar={item['recommended_scalar_median']:.6f} ratio={item['ratio_median']:.5f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
