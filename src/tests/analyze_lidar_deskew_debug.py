#!/usr/bin/env python3
"""Analyze raw/deskewed LiDAR diagnostic captures.

Input can be either the *_index.json file produced by
--lidar-deskew-diagnostic-log or a single NPZ chunk from that capture.
The script compares the stored deskewed scan against alternate beam-time
models and separates raw seam range evidence from same-wall field/line seam
evidence so geometry and occlusions do not look like timestamp bugs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

DEFAULT_LIDAR_OFFSET = (0.0975, 0.0, 0.0)
FIELD_HALF_M = 1.5


def _as_float_array(value, shape_cols: int | None = None) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if shape_cols is not None:
        if arr.size == 0:
            return np.empty((0, shape_cols), dtype=float)
        arr = arr.reshape((-1, shape_cols))
    return arr


def _pose_tuple(value) -> tuple[float, float, float, float, float, float, float]:
    if value is None:
        vals = []
    else:
        vals = np.asarray(value, dtype=float).reshape(-1).tolist()
    while len(vals) < 7:
        vals.append(0.0)
    return tuple(float(v) for v in vals[:7])


def _poses_at(times: np.ndarray, history: np.ndarray, current_pose) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = np.asarray(times, dtype=float)
    if times.size == 0:
        return np.empty(0), np.empty(0), np.empty(0)
    current = _pose_tuple(current_pose)
    history = _as_float_array(history, 7)
    if history.size == 0:
        return (
            np.full(times.shape, current[1], dtype=float),
            np.full(times.shape, current[2], dtype=float),
            np.full(times.shape, current[3], dtype=float),
        )
    order = np.argsort(history[:, 0])
    history = history[order]
    hist_t = history[:, 0]
    hist_x = history[:, 1]
    hist_y = history[:, 2]
    hist_h = history[:, 3]
    if hist_t.size == 1:
        return (
            np.full(times.shape, hist_x[0], dtype=float),
            np.full(times.shape, hist_y[0], dtype=float),
            np.full(times.shape, hist_h[0], dtype=float),
        )
    idx_hi = np.searchsorted(hist_t, times, side="right")
    idx_hi = np.clip(idx_hi, 1, len(hist_t) - 1)
    idx_lo = idx_hi - 1
    t0 = hist_t[idx_lo]
    t1 = hist_t[idx_hi]
    u = np.clip((times - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
    x = hist_x[idx_lo] + (hist_x[idx_hi] - hist_x[idx_lo]) * u
    y = hist_y[idx_lo] + (hist_y[idx_hi] - hist_y[idx_lo]) * u
    dh = ((hist_h[idx_hi] - hist_h[idx_lo] + 180.0) % 360.0) - 180.0
    h = ((hist_h[idx_lo] + dh * u + 180.0) % 360.0) - 180.0
    before = times <= hist_t[0]
    after = times >= hist_t[-1]
    if np.any(before):
        x[before] = hist_x[0]
        y[before] = hist_y[0]
        h[before] = hist_h[0]
    if np.any(after):
        x[after] = hist_x[-1]
        y[after] = hist_y[-1]
        h[after] = hist_h[-1]
    return x, y, h


def _local_to_display_arrays(local_x: np.ndarray, local_y: np.ndarray, local_h: np.ndarray, anchor) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a = _pose_tuple(anchor)
    ah = math.radians(a[3])
    ca, sa = math.cos(ah), math.sin(ah)
    x = a[1] + ca * local_x - sa * local_y
    y = a[2] + sa * local_x + ca * local_y
    h = ((local_h + a[3] + 180.0) % 360.0) - 180.0
    return x, y, h


def _transform_lidar_to_field(points_lidar: np.ndarray, pose_x: np.ndarray, pose_y: np.ndarray, pose_h: np.ndarray, lidar_offset) -> np.ndarray:
    points_lidar = _as_float_array(points_lidar, 2)
    if points_lidar.size == 0:
        return np.empty((0, 2), dtype=float)
    lx_off, ly_off, lh_off = (float(v) for v in (lidar_offset or DEFAULT_LIDAR_OFFSET))
    lh = math.radians(lh_off)
    cl, sl = math.cos(lh), math.sin(lh)
    lx = points_lidar[:, 0]
    ly = points_lidar[:, 1]
    bx = lx_off + cl * lx - sl * ly
    by = ly_off + sl * lx + cl * ly
    rh = np.radians(pose_h)
    cr = np.cos(rh)
    sr = np.sin(rh)
    wx = pose_x + cr * bx - sr * by
    wy = pose_y + sr * bx + cr * by
    return np.column_stack((wx, wy))


def _frame_beam_time_order(frame: dict) -> str:
    order = str(frame.get("beam_time_order", "forward") or "forward").strip().lower()
    return "reverse" if order == "reverse" else "forward"


def _beam_point_times(start_t: float, ref_t: float, keep: np.ndarray, range_count: int, inc: float, order: str) -> np.ndarray:
    keep = np.asarray(keep, dtype=float)
    if float(inc or 0.0) <= 0.0:
        return np.full(keep.shape, float(ref_t), dtype=float)
    if str(order or "forward").strip().lower() == "reverse":
        return float(start_t) + (max(int(range_count) - 1, 0) - keep) * float(inc)
    return float(start_t) + keep * float(inc)


def _model_points(frame: dict, model: str) -> np.ndarray:
    points_lidar = _as_float_array(frame.get("points_lidar", np.empty((0, 2))), 2)
    if model == "stored":
        return _as_float_array(frame.get("points_field", np.empty((0, 2))), 2)
    if points_lidar.size == 0:
        return np.empty((0, 2), dtype=float)
    keep = np.asarray(frame.get("keep_indices", ()), dtype=float)
    if keep.size != points_lidar.shape[0]:
        keep = np.arange(points_lidar.shape[0], dtype=float)
    n = int(frame.get("range_count", 0) or len(frame.get("ranges", ())) or int(np.nanmax(keep) + 1 if keep.size else 0))
    inc = float(frame.get("time_increment", 0.0) or 0.0)
    start_t = float(frame.get("scan_start_t", 0.0) or 0.0)
    end_t = float(frame.get("scan_end_t", 0.0) or 0.0)
    ref_t = float(frame.get("reference_t", frame.get("scan_t", 0.0)) or 0.0)
    duration = inc * max(n - 1, 0) if inc > 0.0 else max(0.0, end_t - start_t)
    configured_order = _frame_beam_time_order(frame)
    if model == "no_deskew" or inc <= 0.0:
        point_times = np.full(points_lidar.shape[0], ref_t, dtype=float)
    elif model == "forward_beam_order":
        point_times = _beam_point_times(start_t, ref_t, keep, n, inc, "forward")
    elif model in ("reverse_beam_order", "reversed_beam_order"):
        point_times = _beam_point_times(start_t, ref_t, keep, n, inc, "reverse")
    elif model == "header_as_end":
        point_times = _beam_point_times(start_t - duration, ref_t, keep, n, inc, configured_order)
    elif model == "header_as_mid":
        point_times = _beam_point_times(start_t - 0.5 * duration, ref_t, keep, n, inc, configured_order)
    else:
        point_times = _beam_point_times(start_t, ref_t, keep, n, inc, configured_order)
    lx, ly, lh = _poses_at(point_times, frame.get("pose_history", ()), frame.get("current_pose", ()))
    px, py, ph = _local_to_display_arrays(lx, ly, lh, frame.get("anchor", ()))
    return _transform_lidar_to_field(points_lidar, px, py, ph, frame.get("lidar_offset", DEFAULT_LIDAR_OFFSET))


def _seam_window_points(frame: dict, points_field: np.ndarray, window_deg: float | None) -> tuple[np.ndarray, np.ndarray, int]:
    points_field = _as_float_array(points_field, 2)
    keep = np.asarray(frame.get("keep_indices", ()), dtype=int)
    n = int(frame.get("range_count", 0) or len(frame.get("ranges", ())))
    if points_field.shape[0] == 0 or keep.size != points_field.shape[0] or n <= 0:
        return np.empty((0, 2)), np.empty((0, 2)), 0
    if window_deg is None:
        seam = frame.get("seam_metrics", {}) if isinstance(frame.get("seam_metrics", {}), dict) else {}
        win = int(seam.get("seam_window_points", 0) or 0)
    else:
        inc = abs(float(frame.get("angle_increment", 0.0) or 0.0))
        win = int(round(math.radians(window_deg) / inc)) if inc > 0.0 else 0
    win = max(1, min(win or 8, n))
    return points_field[keep < win], points_field[keep >= max(0, n - win)], win


def _raw_range_seam_metric(frame: dict, window_deg: float | None, min_points: int) -> dict:
    ranges = np.asarray(frame.get("ranges", ()), dtype=float)
    n = int(frame.get("range_count", 0) or ranges.size)
    if ranges.size == 0 or n <= 0:
        return {"valid": False, "reason": "no_ranges"}
    ranges = ranges[:n]
    if window_deg is None:
        seam = frame.get("seam_metrics", {}) if isinstance(frame.get("seam_metrics", {}), dict) else {}
        win = int(seam.get("seam_window_points", 0) or 0)
    else:
        inc = abs(float(frame.get("angle_increment", 0.0) or 0.0))
        win = int(round(math.radians(window_deg) / inc)) if inc > 0.0 else 0
    win = max(1, min(win or 8, n))
    first = ranges[:win]
    last = ranges[-win:]
    lo = float(frame.get("range_min", 0.0) or 0.0)
    hi = float(frame.get("range_max", float("inf")) or float("inf"))
    first = first[np.isfinite(first) & (first >= lo) & (first <= hi)]
    last = last[np.isfinite(last) & (last >= lo) & (last <= hi)]
    if first.size < min_points or last.size < min_points:
        return {"valid": False, "reason": f"too_few_ranges:{first.size}/{last.size}", "first_count": int(first.size), "last_count": int(last.size)}
    signed = float(np.median(first) - np.median(last))
    return {"valid": True, "abs_delta_m": abs(signed), "signed_delta_m": signed, "first_count": int(first.size), "last_count": int(last.size), "window_points": int(win)}


def _nearest_outer_wall(points: np.ndarray, gate_m: float) -> tuple[str, str, float, float] | None:
    if points.shape[0] == 0:
        return None
    med_x = float(np.median(points[:, 0]))
    med_y = float(np.median(points[:, 1]))
    candidates = [
        ("outer_left", "x", -FIELD_HALF_M, abs(med_x + FIELD_HALF_M)),
        ("outer_right", "x", FIELD_HALF_M, abs(med_x - FIELD_HALF_M)),
        ("outer_bottom", "y", -FIELD_HALF_M, abs(med_y + FIELD_HALF_M)),
        ("outer_top", "y", FIELD_HALF_M, abs(med_y - FIELD_HALF_M)),
    ]
    name, axis, line, dist = min(candidates, key=lambda item: item[3])
    if dist > gate_m:
        return None
    return name, axis, line, dist


def _same_wall_field_metric(frame: dict, points_field: np.ndarray, window_deg: float | None, min_points: int, wall_gate_m: float) -> dict:
    first, last, win = _seam_window_points(frame, points_field, window_deg)
    if first.shape[0] < min_points or last.shape[0] < min_points:
        return {"valid": False, "reason": f"too_few_points:{first.shape[0]}/{last.shape[0]}", "first_count": int(first.shape[0]), "last_count": int(last.shape[0]), "window_points": int(win)}
    first_wall = _nearest_outer_wall(first, wall_gate_m)
    last_wall = _nearest_outer_wall(last, wall_gate_m)
    if first_wall is None or last_wall is None:
        return {"valid": False, "reason": "no_wall_match", "first_count": int(first.shape[0]), "last_count": int(last.shape[0]), "window_points": int(win)}
    if first_wall[0] != last_wall[0]:
        return {"valid": False, "reason": f"different_walls:{first_wall[0]}/{last_wall[0]}", "first_count": int(first.shape[0]), "last_count": int(last.shape[0]), "window_points": int(win)}
    _name, axis, line, _dist = first_wall
    idx = 0 if axis == "x" else 1
    first_res = first[:, idx] - line
    last_res = last[:, idx] - line
    signed = float(np.median(first_res) - np.median(last_res))
    combined_res = np.concatenate((first_res, last_res))
    mad = float(np.median(np.abs(combined_res - np.median(combined_res))))
    return {
        "valid": True,
        "wall": first_wall[0],
        "axis": axis,
        "abs_delta_m": abs(signed),
        "signed_delta_m": signed,
        "wall_mad_m": mad,
        "first_count": int(first.shape[0]),
        "last_count": int(last.shape[0]),
        "window_points": int(win),
    }


def _same_wall_line_metric(field_metric: dict, frame: dict, points_field: np.ndarray, window_deg: float | None, min_points: int, max_line_mad_m: float) -> dict:
    if not field_metric.get("valid"):
        return {"valid": False, "reason": field_metric.get("reason", "field_invalid")}
    first, last, _win = _seam_window_points(frame, points_field, window_deg)
    if first.shape[0] < min_points or last.shape[0] < min_points:
        return {"valid": False, "reason": f"too_few_points:{first.shape[0]}/{last.shape[0]}"}
    combined = np.vstack((first, last))
    center = np.mean(combined, axis=0)
    centered = combined - center
    try:
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return {"valid": False, "reason": "svd_failed"}
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]], dtype=float)
    res_first = (first - center) @ normal
    res_last = (last - center) @ normal
    res_combined = (combined - center) @ normal
    signed = float(np.median(res_first) - np.median(res_last))
    mad = float(np.median(np.abs(res_combined - np.median(res_combined))))
    if mad > max_line_mad_m:
        return {"valid": False, "reason": f"line_mad:{mad:.4f}>{max_line_mad_m:.4f}", "line_mad_m": mad}
    return {"valid": True, "abs_delta_m": abs(signed), "signed_delta_m": signed, "line_mad_m": mad, "wall": field_metric.get("wall", "")}


def _load_frames(path: Path) -> list[dict]:
    if path.suffix.lower() == ".json":
        with path.open("r", encoding="utf-8") as fp:
            index = json.load(fp)
        frames: list[dict] = []
        base = path.parent
        for chunk in index.get("chunks", []):
            chunk_path = Path(chunk.get("path", ""))
            if not chunk_path.is_absolute():
                chunk_path = base / chunk_path
            frames.extend(_load_frames(chunk_path))
        return frames
    data = np.load(path, allow_pickle=True)
    frames = data.get("frames")
    if frames is None:
        return []
    return [dict(item) for item in frames.tolist()]


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=float), pct))


def _frame_motion(frame: dict) -> tuple[float, float, str]:
    pose = _pose_tuple(frame.get("current_pose", ()))
    speed = math.hypot(pose[4], pose[5])
    yaw = abs(pose[6])
    if speed < 0.05 and yaw < 5.0:
        bucket = "stationary"
    elif yaw < 15.0:
        bucket = "straight"
    elif yaw < 60.0:
        bucket = "moderate_turn"
    else:
        bucket = "hard_turn"
    return speed, yaw, bucket


def analyze(frames: Iterable[dict], args) -> tuple[dict[str, list[dict]], list[dict]]:
    models = ["stored", "current_recomputed", "forward_beam_order", "reverse_beam_order", "header_as_end", "header_as_mid", "no_deskew"]
    by_model: dict[str, list[dict]] = {name: [] for name in models}
    raw_rows: list[dict] = []
    for frame_index, frame in enumerate(frames):
        speed, yaw, bucket = _frame_motion(frame)
        raw_metric = _raw_range_seam_metric(frame, args.seam_window_deg, args.min_window_points)
        raw_metric = dict(raw_metric)
        raw_metric.update(frame_index=frame_index, scan_t=float(frame.get("scan_t", 0.0) or 0.0), speed_mps=speed, yaw_dps=yaw, motion_bucket=bucket, source=str(frame.get("source", "")))
        raw_rows.append(raw_metric)
        for model in models:
            model_name = "current_recomputed" if model == "current_recomputed" else model
            points = _model_points(frame, "current" if model == "current_recomputed" else model)
            field_metric = _same_wall_field_metric(frame, points, args.seam_window_deg, args.min_window_points, args.same_wall_gate_m)
            line_metric = _same_wall_line_metric(field_metric, frame, points, args.seam_window_deg, args.min_window_points, args.max_line_mad_m)
            row = {
                "frame_index": frame_index,
                "scan_t": float(frame.get("scan_t", 0.0) or 0.0),
                "source": str(frame.get("source", "")),
                "speed_mps": speed,
                "yaw_dps": yaw,
                "motion_bucket": bucket,
                "field_valid": bool(field_metric.get("valid", False)),
                "field_reason": str(field_metric.get("reason", "")),
                "field_wall": str(field_metric.get("wall", "")),
                "field_abs_delta_m": float(field_metric.get("abs_delta_m", float("nan"))),
                "field_signed_delta_m": float(field_metric.get("signed_delta_m", float("nan"))),
                "field_wall_mad_m": float(field_metric.get("wall_mad_m", float("nan"))),
                "line_valid": bool(line_metric.get("valid", False)),
                "line_reason": str(line_metric.get("reason", "")),
                "line_abs_delta_m": float(line_metric.get("abs_delta_m", float("nan"))),
                "line_signed_delta_m": float(line_metric.get("signed_delta_m", float("nan"))),
                "line_mad_m": float(line_metric.get("line_mad_m", float("nan"))),
            }
            by_model[model_name].append(row)
    return by_model, raw_rows


def _valid_values(rows: list[dict], valid_key: str, value_key: str) -> list[float]:
    return [float(row[value_key]) for row in rows if row.get(valid_key) and math.isfinite(float(row.get(value_key, float("nan"))))]


def _print_metric_summary(label: str, rows: list[dict], valid_key: str, value_key: str) -> None:
    vals = _valid_values(rows, valid_key, value_key)
    print(f"{label}: valid={len(vals)}/{len(rows)} median={_percentile(vals, 50):.5f} p95={_percentile(vals, 95):.5f} max={max(vals) if vals else float('nan'):.5f}")


def print_summary(by_model: dict[str, list[dict]], raw_rows: list[dict], worst: int) -> None:
    print("raw_range_seam")
    _print_metric_summary("all", raw_rows, "valid", "abs_delta_m")
    for bucket in ("stationary", "straight", "moderate_turn", "hard_turn"):
        rows = [row for row in raw_rows if row.get("motion_bucket") == bucket]
        if rows:
            _print_metric_summary(bucket, rows, "valid", "abs_delta_m")

    print("\nmodel, frames, field_valid, field_median_abs_m, field_p95_abs_m, line_valid, line_median_abs_m, line_p95_abs_m, line_median_mad_m")
    for model, rows in by_model.items():
        field_vals = _valid_values(rows, "field_valid", "field_abs_delta_m")
        line_vals = _valid_values(rows, "line_valid", "line_abs_delta_m")
        line_mads = _valid_values(rows, "line_valid", "line_mad_m")
        print(
            f"{model}, {len(rows)}, {len(field_vals)}, {_percentile(field_vals, 50):.5f}, {_percentile(field_vals, 95):.5f}, "
            f"{len(line_vals)}, {_percentile(line_vals, 50):.5f}, {_percentile(line_vals, 95):.5f}, {_percentile(line_mads, 50):.5f}"
        )

    print("\nstored same-wall line seam by motion")
    stored = by_model.get("stored", [])
    for bucket in ("stationary", "straight", "moderate_turn", "hard_turn"):
        rows = [row for row in stored if row.get("motion_bucket") == bucket]
        if rows:
            _print_metric_summary(bucket, rows, "line_valid", "line_abs_delta_m")

    print("\nmodel wins on valid same-wall line frames")
    wins: dict[str, int] = {model: 0 for model in by_model}
    frame_ids = sorted({row["frame_index"] for rows in by_model.values() for row in rows})
    comparable = 0
    for frame_id in frame_ids:
        candidates = []
        for model, rows in by_model.items():
            row = next((item for item in rows if item["frame_index"] == frame_id), None)
            if row and row.get("line_valid"):
                candidates.append((float(row["line_abs_delta_m"]), model))
        if len(candidates) >= 2:
            comparable += 1
            _value, model = min(candidates)
            wins[model] += 1
    print(f"comparable_frames={comparable} wins={wins}")

    if worst > 0:
        valid_stored = sorted([row for row in stored if row.get("line_valid")], key=lambda row: float(row["line_abs_delta_m"]), reverse=True)
        print("\nworst valid stored same-wall line frames:")
        for row in valid_stored[:worst]:
            print(
                f"frame={row['frame_index']} scan_t={row['scan_t']:.3f} bucket={row['motion_bucket']} "
                f"speed={row['speed_mps']:.3f} yaw={row['yaw_dps']:.1f} wall={row['field_wall']} "
                f"line_abs={row['line_abs_delta_m']:.5f} field_abs={row['field_abs_delta_m']:.5f} "
                f"line_mad={row['line_mad_m']:.5f} source={row['source']}"
            )
        valid_raw = sorted([row for row in raw_rows if row.get("valid")], key=lambda row: float(row["abs_delta_m"]), reverse=True)
        print("\nworst raw range seam frames:")
        for row in valid_raw[:worst]:
            print(
                f"frame={row['frame_index']} scan_t={row['scan_t']:.3f} bucket={row['motion_bucket']} "
                f"speed={row['speed_mps']:.3f} yaw={row['yaw_dps']:.1f} "
                f"abs={row['abs_delta_m']:.5f} signed={row['signed_delta_m']:.5f} "
                f"counts={row.get('first_count')}/{row.get('last_count')} source={row['source']}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare LiDAR deskew seam behavior across beam-time models")
    parser.add_argument("path", help="Diagnostic *_index.json or NPZ chunk path")
    parser.add_argument("--seam-window-deg", type=float, default=None, help="Override seam window width; defaults to captured value")
    parser.add_argument("--worst", type=int, default=8, help="Print N worst valid seam frames")
    parser.add_argument("--min-window-points", type=int, default=3, help="Minimum valid points required on each side of the seam")
    parser.add_argument("--same-wall-gate-m", type=float, default=0.25, help="Maximum median distance from a seam window to an outer wall for same-wall analysis")
    parser.add_argument("--max-line-mad-m", type=float, default=0.04, help="Maximum combined line MAD for valid same-wall line seam frames")
    args = parser.parse_args()
    path = Path(args.path)
    if not path.exists():
        raise SystemExit(f"not found: {path}")
    frames = _load_frames(path)
    print(f"loaded_frames={len(frames)} path={path}")
    if not frames:
        return 1
    by_model, raw_rows = analyze(frames, args)
    print_summary(by_model, raw_rows, worst=max(0, int(args.worst)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
