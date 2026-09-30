#!/usr/bin/env python3
"""DriveBase speed calibration suite using OTOS distance as stop authority.

The suite bypasses perception, navigation, and the web UI. It treats the clear
lane as two fixed endpoints: home at 0 m and far at +/- --distance-m,
depending on --start-direction. Forward measurements always start at home;
reverse measurements always start at far.
Reposition legs use a fixed duty command and are logged separately from the
actual calibration samples.

Run only in a safe open area. The default settings are conservative; raise
command values deliberately.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = REPO_ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))

import qwiic_otos


SAFETY_STOP_REASONS = {"otos_missing", "otos_stale"}
DEFAULT_LIDAR_OFFSET_X = 0.0975
DEFAULT_LIDAR_OFFSET_Y = 0.0
DEFAULT_LIDAR_OFFSET_H = 0.0


def parse_float_list(text: str) -> list[float]:
    values: list[float] = []
    for part in str(text).split(","):
        part = part.strip()
        if part:
            values.append(float(part))
    if not values:
        raise argparse.ArgumentTypeError("list must contain at least one number")
    if any(v <= 0.0 for v in values):
        raise argparse.ArgumentTypeError("values must be positive; direction is handled by the suite")
    return values


def make_pose(x: float, y: float, h: float):
    return qwiic_otos.Pose2D(x, y, h)


def unit_constant(name: str, class_name: str):
    if hasattr(qwiic_otos, name):
        return getattr(qwiic_otos, name)
    return getattr(qwiic_otos.QwiicOTOS, class_name)


def require_not_false(result, label: str) -> None:
    if result is False:
        raise RuntimeError(f"{label} failed")


def wrap_delta_deg(curr: float, prev: float) -> float:
    return (float(curr) - float(prev) + 180.0) % 360.0 - 180.0


@dataclass
class OTOSEntry:
    t: float
    x: float
    y: float
    h: float
    vx: float
    vy: float
    vh: float


class OTOSTelemetry:
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._latest: Optional[OTOSEntry] = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._error: Optional[BaseException] = None

    def start(self, timeout_s: float = 20.0) -> None:
        self._thread.start()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while not self._ready.is_set():
            if self._error is not None:
                raise RuntimeError(f"OTOS setup/read failed: {self._error}") from self._error
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for OTOS setup/calibration")
            time.sleep(0.01)
        if self._error is not None:
            raise RuntimeError(f"OTOS setup/read failed: {self._error}") from self._error

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def latest(self) -> Optional[OTOSEntry]:
        with self._lock:
            return self._latest

    def wait_first(self, timeout_s: float) -> OTOSEntry:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            entry = self.latest()
            if entry is not None:
                return entry
            if self._error is not None:
                raise RuntimeError(f"OTOS setup/read failed: {self._error}") from self._error
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for first OTOS sample")

    def _setup(self):
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
        require_not_false(
            otos.setOffset(make_pose(self.args.otos_offset_x, self.args.otos_offset_y, self.args.otos_offset_h)),
            "setOffset()",
        )
        require_not_false(otos.setLinearScalar(self.args.linear_scalar), "setLinearScalar()")
        require_not_false(otos.setAngularScalar(self.args.angular_scalar), "setAngularScalar()")
        if self.args.calibrate_imu:
            attempts = max(1, int(self.args.otos_imu_calibration_retries))
            max_drift = float(self.args.otos_max_stationary_yaw_drift_dps)
            check_s = float(self.args.otos_stationary_drift_check_s)
            warmup_s = float(self.args.otos_stationary_drift_warmup_s)
            accepted = False
            last_drift = None
            for attempt in range(1, attempts + 1):
                print(f"Keep robot perfectly still: calibrating OTOS IMU ({attempt}/{attempts})...")
                require_not_false(otos.calibrateImu(), "calibrateImu()")
                if check_s > 0.0:
                    require_not_false(otos.resetTracking(), "resetTracking()")
                    require_not_false(otos.setPosition(make_pose(0.0, 0.0, 0.0)), "setPosition()")
                    drift = self._measure_stationary_yaw_drift(otos, check_s, warmup_s)
                    last_drift = drift
                    print(
                        "OTOS stationary yaw check: "
                        f"drift={drift['yaw_drift_dps']:+.4f}deg/s total={drift['yaw_delta_deg']:+.3f}deg "
                        f"xy_delta={drift['xy_delta_m']:.5f}m vh_med={drift['vh_median_dps']:+.4f}deg/s"
                    )
                    if abs(drift["yaw_drift_dps"]) <= max_drift:
                        accepted = True
                        break
                else:
                    accepted = True
                    break
            if not accepted and not self.args.otos_allow_stationary_yaw_drift:
                detail = ""
                if last_drift is not None:
                    detail = f" measured {last_drift['yaw_drift_dps']:+.4f}deg/s"
                raise RuntimeError(
                    f"OTOS stationary yaw drift above limit {max_drift:.4f}deg/s after {attempts} calibration attempts;"
                    f"{detail}. Keep robot still, restart, or use --otos-allow-stationary-yaw-drift only for diagnostics."
                )
        if self.args.reset_tracking:
            require_not_false(otos.resetTracking(), "resetTracking()")
            require_not_false(otos.setPosition(make_pose(0.0, 0.0, 0.0)), "setPosition()")
        return otos

    def _measure_stationary_yaw_drift(self, otos, duration_s: float, warmup_s: float) -> dict:
        warmup_deadline = time.monotonic() + max(0.0, float(warmup_s))
        while time.monotonic() < warmup_deadline:
            self._read(otos)
            time.sleep(0.01)
        first = self._read(otos)
        prev = first
        yaw_delta = 0.0
        vhs = [first.vh]
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            time.sleep(0.01)
            sample = self._read(otos)
            yaw_delta += wrap_delta_deg(sample.h, prev.h)
            vhs.append(sample.vh)
            prev = sample
        elapsed = max(prev.t - first.t, 1e-6)
        return {
            "duration_s": elapsed,
            "yaw_delta_deg": yaw_delta,
            "yaw_drift_dps": yaw_delta / elapsed,
            "xy_delta_m": math.hypot(prev.x - first.x, prev.y - first.y),
            "vh_median_dps": float(statistics.median(vhs)) if vhs else 0.0,
        }

    def _read(self, otos) -> OTOSEntry:
        try:
            pos, vel, _acc = otos.getPosVelAcc()
        except Exception:
            pos = otos.getPosition()
            vel = otos.getVelocity()
        return OTOSEntry(
            t=time.monotonic(),
            x=float(pos.x),
            y=float(pos.y),
            h=float(pos.h),
            vx=float(vel.x),
            vy=float(vel.y),
            vh=float(vel.h),
        )

    def _run(self) -> None:
        try:
            otos = self._setup()
            print("OTOS telemetry started")
            self._ready.set()
        except BaseException as exc:
            self._error = exc
            self._ready.set()
            return

        period = 1.0 / max(float(self.args.otos_rate), 1e-6)
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                entry = self._read(otos)
                with self._lock:
                    self._latest = entry
            except BaseException as exc:
                self._error = exc
                time.sleep(0.1)
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()



@dataclass
class LaserScanSnapshot:
    receive_t: float
    header_stamp_s: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    scan_time: float
    time_increment: float
    ranges: tuple[float, ...]


class LidarEndpointRecorder:
    """Optional stationary LiDAR endpoint capture for speed-scale calibration."""

    def __init__(self, args, csv_path: Path):
        self.args = args
        self.enabled = bool(getattr(args, "lidar_endpoint_capture", False))
        self.csv_path = Path(csv_path)
        self.base_path = self._base_path()
        self.index_path = self._index_path()
        self.chunk_size = max(1, int(getattr(args, "lidar_endpoint_chunk_size", 60) or 60))
        self._lock = threading.Lock()
        self._buffer = deque(maxlen=512)
        self._pending_frames: list[dict] = []
        self._chunks: list[dict] = []
        self._chunk_index = 0
        self.written_frames = 0
        self.dropped_frames = 0
        self.context = None
        self.node = None
        self.executor = None
        self.thread = None
        self.stop_event = threading.Event()
        self._rclpy = None

    def _base_path(self) -> Path:
        configured = str(getattr(self.args, "lidar_endpoint_output", "") or "").strip()
        if configured:
            path = Path(configured).expanduser()
        else:
            path = self.csv_path.with_name(f"{self.csv_path.stem}_lidar_endpoint.npz")
        return path.resolve()

    def _index_path(self) -> Path:
        root = self.base_path.with_suffix("")
        return root.with_name(f"{root.name}_index.json")

    def start(self) -> None:
        if not self.enabled:
            return
        self.base_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import LaserScan
        except Exception as exc:
            raise RuntimeError("LiDAR endpoint capture requires rclpy and sensor_msgs") from exc

        recorder = self
        topic = str(getattr(self.args, "lidar_endpoint_topic", "/scan") or "/scan")

        class EndpointScanNode(Node):
            def __init__(self, context):
                super().__init__("drivebase_speed_calibration_lidar_endpoint", context=context)
                scan_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                )
                self.create_subscription(LaserScan, topic, self.scan_cb, scan_qos)

            def scan_cb(self, scan):
                recorder._scan_cb(scan)

        def run() -> None:
            self.context = rclpy.context.Context()
            rclpy.init(context=self.context)
            self.node = EndpointScanNode(self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            try:
                while rclpy.ok(context=self.context) and not self.stop_event.is_set():
                    self.executor.spin_once(timeout_sec=0.1)
            finally:
                if self.executor is not None:
                    try:
                        self.executor.remove_node(self.node)
                    except Exception:
                        pass
                    self.executor.shutdown(timeout_sec=0.5)
                if self.node is not None:
                    self.node.destroy_node()
                if rclpy.ok(context=self.context):
                    rclpy.shutdown(context=self.context)

        self._rclpy = rclpy
        self.thread = threading.Thread(target=run, name="lidar-endpoint-capture", daemon=True)
        self.thread.start()
        print(f"LiDAR endpoint capture enabled: topic={topic} index={self.index_path}")

    def stop(self) -> None:
        if not self.enabled:
            return
        self.stop_event.set()
        if self.executor is not None:
            try:
                self.executor.wake()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self._flush_frames()
        self._write_index()

    def wait_first(self, timeout_s: float) -> bool:
        if not self.enabled:
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self._lock:
                if self._buffer:
                    return True
            time.sleep(0.02)
        return False

    def _scan_cb(self, scan) -> None:
        header_stamp_s = 0.0
        try:
            if scan.header.stamp.sec or scan.header.stamp.nanosec:
                header_stamp_s = float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1e-9
        except Exception:
            header_stamp_s = 0.0
        snapshot = LaserScanSnapshot(
            receive_t=time.monotonic(),
            header_stamp_s=header_stamp_s,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            scan_time=float(scan.scan_time),
            time_increment=float(scan.time_increment),
            ranges=tuple(float(v) for v in scan.ranges),
        )
        with self._lock:
            self._buffer.append(snapshot)

    def _recent_scans_since(self, start_t: float) -> list[LaserScanSnapshot]:
        with self._lock:
            return [snap for snap in self._buffer if snap.receive_t >= start_t]

    def capture_endpoint(
        self,
        segment: dict,
        phase: str,
        otos_entry: OTOSEntry,
        lane_origin: OTOSEntry,
        lane_axis_h_deg: float,
        extra: Optional[dict] = None,
    ) -> int:
        if not self.enabled or not int(bool(segment.get("is_measurement", 0))):
            return 0
        settle_s = max(0.0, float(getattr(self.args, "lidar_endpoint_settle_s", 0.30) or 0.0))
        if settle_s > 0.0:
            time.sleep(settle_s)
        start_t = time.monotonic()
        capture_s = max(0.0, float(getattr(self.args, "lidar_endpoint_capture_s", 0.60) or 0.0))
        min_scans = max(1, int(getattr(self.args, "lidar_endpoint_min_scans", 4) or 4))
        max_wait_s = max(capture_s, float(getattr(self.args, "lidar_endpoint_max_wait_s", 2.0) or 2.0))
        deadline = start_t + max_wait_s
        capture_deadline = start_t + capture_s
        scans: list[LaserScanSnapshot] = []
        while time.monotonic() < deadline:
            scans = self._recent_scans_since(start_t)
            if time.monotonic() >= capture_deadline and len(scans) >= min_scans:
                break
            time.sleep(0.02)
        scans = self._recent_scans_since(start_t)
        if len(scans) < min_scans:
            raise RuntimeError(f"LiDAR endpoint capture for {segment.get('name')} {phase} only received {len(scans)} scans")
        frame = self._frame_from_stationary_scans(scans, segment, phase, otos_entry, lane_origin, lane_axis_h_deg, extra or {})
        self._pending_frames.append(frame)
        if len(self._pending_frames) >= self.chunk_size:
            self._flush_frames()
        self._write_index()
        print(f"LiDAR endpoint captured: {segment.get('name')} {phase} scans={len(scans)}")
        return 1

    def _frame_from_stationary_scans(
        self,
        scans: list[LaserScanSnapshot],
        segment: dict,
        phase: str,
        otos_entry: OTOSEntry,
        lane_origin: OTOSEntry,
        lane_axis_h_deg: float,
        extra: dict,
    ) -> dict:
        reference = scans[-1]
        n = len(reference.ranges)
        compatible = [snap for snap in scans if len(snap.ranges) == n and abs(snap.angle_min - reference.angle_min) < 1e-9 and abs(snap.angle_increment - reference.angle_increment) < 1e-12]
        if not compatible:
            compatible = [reference]
        stack = []
        for snap in compatible:
            arr = np.asarray(snap.ranges, dtype=float)
            lo = float(snap.range_min if math.isfinite(snap.range_min) else 0.0)
            hi = float(snap.range_max if math.isfinite(snap.range_max) and snap.range_max > lo else 30.0)
            valid = np.isfinite(arr) & (arr >= lo) & (arr <= hi)
            arr = arr.astype(float, copy=True)
            arr[~valid] = np.nan
            stack.append(arr)
        with np.errstate(all="ignore"):
            ranges = np.nanmedian(np.vstack(stack), axis=0)
        indices = np.arange(n, dtype=float)
        angles = float(reference.angle_min) + indices * float(reference.angle_increment)
        lo = float(reference.range_min if math.isfinite(reference.range_min) else 0.0)
        hi = float(reference.range_max if math.isfinite(reference.range_max) and reference.range_max > lo else 30.0)
        valid = np.isfinite(ranges) & (ranges >= lo) & (ranges <= hi)
        keep_indices = indices[valid].astype(int)
        kept_ranges = ranges[valid]
        points_lidar = np.column_stack((kept_ranges * np.cos(angles[valid]), kept_ranges * np.sin(angles[valid]))) if kept_ranges.size else np.empty((0, 2), dtype=float)
        current_pose = np.asarray((otos_entry.t, otos_entry.x, otos_entry.y, otos_entry.h, otos_entry.vx, otos_entry.vy, otos_entry.vh), dtype=float)
        anchor = np.asarray((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), dtype=float)
        points_field = transform_lidar_points_to_field(points_lidar, otos_entry, self.args)
        scan_t = float(statistics.median(s.receive_t for s in compatible))
        scan_time = float(reference.scan_time or 0.0)
        if scan_time <= 0.0 and float(reference.time_increment or 0.0) > 0.0:
            scan_time = float(reference.time_increment) * max(n - 1, 0)
        lane_coord, lane_lateral, lane_disp = lane_coordinate(lane_origin, lane_axis_h_deg, otos_entry)
        endpoint_role = segment.get("start_role", "") if phase.endswith("start") else segment.get("target_role", "")
        calibration = {
            "kind": "drivebase_speed_lidar_endpoint",
            "segment": str(segment.get("name", "")),
            "phase": str(phase),
            "endpoint_role": str(endpoint_role),
            "control": str(segment.get("control", "")),
            "command_value": float(segment.get("value", 0.0) or 0.0),
            "command_signed": float(segment.get("command", 0.0) or 0.0),
            "planned_direction": str(segment.get("planned_direction", "")),
            "direction": str(segment.get("direction", "")),
            "repeat": segment.get("repeat", ""),
            "start_role": str(segment.get("start_role", "")),
            "target_role": str(segment.get("target_role", "")),
            "lane_origin_x": float(lane_origin.x),
            "lane_origin_y": float(lane_origin.y),
            "lane_origin_h_deg": float(lane_origin.h),
            "lane_axis_h_deg": float(lane_axis_h_deg),
            "lane_coord_m": float(lane_coord),
            "lane_lateral_m": float(lane_lateral),
            "lane_displacement_from_origin_m": float(lane_disp),
            "target_coord_m": float(segment.get("target_coord_m", float("nan"))),
            "start_lane_coord_m": float(segment.get("start_lane_coord_m", float("nan"))),
            "raw_scan_count": int(len(scans)),
            "compatible_scan_count": int(len(compatible)),
            "median_scan_receive_t": float(scan_t),
        }
        calibration.update(extra)
        return {
            "source": "drivebase_speed_lidar_endpoint",
            "race_phase": str(phase),
            "race_mode": "calibration",
            "scan_t": scan_t,
            "scan_start_t": scan_t - max(0.0, scan_time),
            "scan_end_t": scan_t,
            "reference_t": scan_t,
            "timestamp_source": "stationary_receive_median",
            "beam_time_order": str(getattr(self.args, "lidar_beam_time_order", "reverse") or "reverse"),
            "deskew_mode": "stationary_endpoint",
            "range_count": int(n),
            "ranges": ranges.astype(float),
            "angle_min": float(reference.angle_min),
            "angle_increment": float(reference.angle_increment),
            "range_min": lo,
            "range_max": hi,
            "scan_time": scan_time,
            "time_increment": float(reference.time_increment or (scan_time / max(n - 1, 1) if n > 1 else 0.0)),
            "keep_indices": keep_indices,
            "points_lidar": points_lidar,
            "points_field": points_field,
            "pose_history": np.asarray([current_pose], dtype=float),
            "current_pose": current_pose,
            "reference_pose": current_pose,
            "reference_local_pose": current_pose,
            "otos_only_pose": current_pose,
            "fused_pose": current_pose,
            "otos_base_anchor": anchor,
            "anchor": anchor,
            "lidar_offset": (float(getattr(self.args, "lidar_offset_x", DEFAULT_LIDAR_OFFSET_X)), float(getattr(self.args, "lidar_offset_y", DEFAULT_LIDAR_OFFSET_Y)), float(getattr(self.args, "lidar_offset_h", DEFAULT_LIDAR_OFFSET_H))),
            "speed_mps": abs(otos_forward_speed(otos_entry)),
            "yaw_dps": abs(float(otos_entry.vh)),
            "calibration": calibration,
        }

    def _chunk_path(self, index: int) -> Path:
        root = self.base_path.with_suffix("")
        return root.with_name(f"{root.name}_{index:06d}.npz")

    def _flush_frames(self) -> None:
        if not self._pending_frames:
            return
        self._chunk_index += 1
        chunk_path = self._chunk_path(self._chunk_index)
        frames = self._pending_frames
        self._pending_frames = []
        try:
            np.savez_compressed(chunk_path, frames=np.asarray(frames, dtype=object))
            self.written_frames += len(frames)
            self._chunks.append({"path": str(chunk_path), "frames": len(frames)})
        except Exception as exc:
            self.dropped_frames += len(frames)
            self._chunks.append({"path": str(chunk_path), "frames": 0, "error": repr(exc)})

    def _write_index(self) -> None:
        if not self.enabled:
            return
        data = {
            "version": 1,
            "kind": "drivebase_speed_lidar_endpoint",
            "base_path": str(self.base_path),
            "csv_path": str(self.csv_path),
            "linear_scalar": float(getattr(self.args, "linear_scalar", 1.0)),
            "angular_scalar": float(getattr(self.args, "angular_scalar", 1.0)),
            "otos_offset": [float(getattr(self.args, "otos_offset_x", 0.0)), float(getattr(self.args, "otos_offset_y", 0.0)), float(getattr(self.args, "otos_offset_h", 0.0))],
            "lidar_offset": [float(getattr(self.args, "lidar_offset_x", DEFAULT_LIDAR_OFFSET_X)), float(getattr(self.args, "lidar_offset_y", DEFAULT_LIDAR_OFFSET_Y)), float(getattr(self.args, "lidar_offset_h", DEFAULT_LIDAR_OFFSET_H))],
            "written_frames": int(self.written_frames + len(self._pending_frames)),
            "dropped_frames": int(self.dropped_frames),
            "chunk_size": int(self.chunk_size),
            "chunks": list(self._chunks),
        }
        tmp = self.index_path.with_suffix(self.index_path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2, sort_keys=True)
            fp.write("\n")
        tmp.replace(self.index_path)


def transform_lidar_points_to_field(points_lidar: np.ndarray, pose: OTOSEntry, args) -> np.ndarray:
    points = np.asarray(points_lidar, dtype=float).reshape((-1, 2)) if np.asarray(points_lidar).size else np.empty((0, 2), dtype=float)
    if points.size == 0:
        return np.empty((0, 2), dtype=float)
    lx_off = float(getattr(args, "lidar_offset_x", DEFAULT_LIDAR_OFFSET_X))
    ly_off = float(getattr(args, "lidar_offset_y", DEFAULT_LIDAR_OFFSET_Y))
    lh_off = math.radians(float(getattr(args, "lidar_offset_h", DEFAULT_LIDAR_OFFSET_H)))
    cl, sl = math.cos(lh_off), math.sin(lh_off)
    bx = lx_off + cl * points[:, 0] - sl * points[:, 1]
    by = ly_off + sl * points[:, 0] + cl * points[:, 1]
    h = math.radians(float(pose.h))
    ch, sh = math.cos(h), math.sin(h)
    wx = float(pose.x) + ch * bx - sh * by
    wy = float(pose.y) + sh * bx + ch * by
    return np.column_stack((wx, wy))

def averaged_stationary_origin(otos: OTOSTelemetry, settle_s: float, average_s: float) -> OTOSEntry:
    if settle_s > 0.0:
        print(f"OTOS origin settle: {settle_s:.2f}s stationary...")
        time.sleep(float(settle_s))

    first = otos.wait_first(1.0)
    if average_s <= 0.0:
        return first

    print(f"OTOS origin average: {average_s:.2f}s stationary...")
    samples: list[OTOSEntry] = []
    deadline = time.monotonic() + float(average_s)
    while time.monotonic() < deadline:
        entry = otos.latest()
        if entry is not None:
            samples.append(entry)
        time.sleep(0.01)
    if not samples:
        return first

    xs = [e.x for e in samples]
    ys = [e.y for e in samples]
    sin_h = sum(math.sin(math.radians(e.h)) for e in samples)
    cos_h = sum(math.cos(math.radians(e.h)) for e in samples)
    heading = math.degrees(math.atan2(sin_h, cos_h)) if samples else first.h
    vxs = [e.vx for e in samples]
    vys = [e.vy for e in samples]
    vhs = [e.vh for e in samples]
    latest_t = samples[-1].t
    origin = OTOSEntry(
        t=latest_t,
        x=float(statistics.median(xs)),
        y=float(statistics.median(ys)),
        h=float(heading),
        vx=float(statistics.median(vxs)),
        vy=float(statistics.median(vys)),
        vh=float(statistics.median(vhs)),
    )
    dx = samples[-1].x - samples[0].x
    dy = samples[-1].y - samples[0].y
    print(
        f"OTOS origin fixed: x={origin.x:+.4f}m y={origin.y:+.4f}m h={origin.h:+.2f}deg "
        f"window_delta=({dx:+.5f},{dy:+.5f})m n={len(samples)}"
    )
    return origin


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Automatic DriveBase speed calibration with fixed endpoint starts and OTOS stops.",
    )
    parser.add_argument("mode", choices=("duty-sweep", "speed-sweep", "both"), help="Calibration mode to run")
    parser.add_argument("--directions", choices=("forward", "reverse", "both"), default="both", help="Measurement direction set")
    parser.add_argument("--distance-m", type=float, default=2.00, help="Lane endpoint distance from home to far")
    parser.add_argument("--duties", type=parse_float_list, default=parse_float_list("0.20,0.30,0.40,0.50,0.60,0.80,1.00"), help="Comma-separated direct duty magnitudes for duty-sweep")
    parser.add_argument("--speeds", type=parse_float_list, default=parse_float_list("0.20,0.30,0.40,0.50,0.60,0.70,0.80"), help="Comma-separated speed targets for speed-sweep")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat each measurement this many times")
    parser.add_argument("--reposition-duty", type=float, default=0.70, help="Fixed direct duty used for non-measurement absolute endpoint positioning legs")
    parser.add_argument("--reposition-max-attempts", type=int, default=3, help="Attempts to reach a required absolute endpoint before skipping measurement")
    parser.add_argument("--max-reposition-time-s", type=float, default=8.0, help="Timeout per reposition attempt")
    parser.add_argument("--reset-after-measurement", dest="reset_after_measurement", action="store_true", default=True, help="Return to the measurement start endpoint after each measured leg")
    parser.add_argument("--no-reset-after-measurement", dest="reset_after_measurement", action="store_false", help="Only reposition before the next measurement")
    parser.add_argument("--finish-at", choices=("home", "far", "current"), default="home", help="Endpoint to return to after the suite")
    parser.add_argument("--endpoint-tolerance-m", type=float, default=0.03, help="Distance tolerance for home/far endpoint stops")
    parser.add_argument("--max-endpoint-overshoot-m", type=float, default=0.30, help="Stop a leg if OTOS lane coordinate passes home/far by this margin; the suite then repositions and continues")
    parser.add_argument("--start-direction", choices=("forward", "reverse"), default="forward", help="OTOS coordinate direction for a positive drive command from home to far; default forward matches the current speed-scale calibration lane setup")
    parser.add_argument("--steering", type=float, default=0.0, help="Normalized steering command during all legs")
    parser.add_argument("--heading-hold", dest="heading_hold", action="store_true", default=True, help="Use simple OTOS heading hold during moving legs")
    parser.add_argument("--no-heading-hold", dest="heading_hold", action="store_false", help="Disable heading hold and use fixed --steering only")
    parser.add_argument("--heading-hold-kp", type=float, default=0.010, help="Normalized steering correction per degree of heading error")
    parser.add_argument("--heading-hold-steer-max", type=float, default=0.10, help="Maximum absolute heading-hold steering correction")
    parser.add_argument("--heading-hold-arm-progress-m", type=float, default=0.04, help="Distance that must be physically traveled before OTOS heading hold arms for a leg")
    parser.add_argument("--heading-hold-sign", choices=("normal", "inverted"), default="normal", help="Use inverted if the heading hold steers away from the target")
    parser.add_argument("--lateral-hold", dest="lateral_hold", action="store_true", default=True, help="Add a gentle OTOS lateral lane-centering correction")
    parser.add_argument("--no-lateral-hold", dest="lateral_hold", action="store_false", help="Disable lateral lane-centering correction")
    parser.add_argument("--lateral-hold-kp", type=float, default=0.25, help="Normalized steering correction per meter of lateral lane error")
    parser.add_argument("--lateral-hold-steer-max", type=float, default=0.08, help="Maximum absolute lateral lane-centering steering correction")
    parser.add_argument("--lateral-hold-sign", choices=("normal", "inverted"), default="normal", help="Use inverted if lateral hold steers away from the lane center")
    parser.add_argument("--sample-period-s", type=float, default=0.02, help="CSV/log sample period")
    parser.add_argument("--pause-s", type=float, default=0.80, help="Pause after braking between legs")
    parser.add_argument("--brake-s", type=float, default=0.35, help="Brake duration after each leg")
    parser.add_argument("--brake-level", type=float, default=1.0, help="Brake level after each leg")
    parser.add_argument("--countdown-s", type=float, default=3.0, help="Countdown before the first leg")
    parser.add_argument("--max-leg-time-s", type=float, default=14.0, help="Timeout per measured leg")
    parser.add_argument("--measurement-no-motion-timeout-s", type=float, default=3.0, help="End a measured leg early if commanded duty/speed produces no real OTOS progress by this time")
    parser.add_argument("--measurement-no-motion-progress-m", type=float, default=0.015, help="Minimum lane/path progress required before --measurement-no-motion-timeout-s")
    parser.add_argument("--max-lateral-m", type=float, default=0.30, help="Abort a leg if OTOS lateral displacement from lane axis exceeds this")
    parser.add_argument("--max-heading-error-deg", type=float, default=35.0, help="Abort a leg if heading differs this much from lane start")
    parser.add_argument("--max-otos-age-ms", type=float, default=120.0, help="Abort if the latest OTOS sample is older than this")
    parser.add_argument("--min-distance-m", type=float, default=0.05, help="Minimum allowed distance")
    parser.add_argument("--output", default="", help="CSV output path; default is /tmp/drivebase_speed_cal_<timestamp>.csv")
    parser.add_argument("--lidar-endpoint-capture", action="store_true", help="Capture stationary LiDAR scans at the start/end of each measured leg for OTOS scale analysis")
    parser.add_argument("--lidar-endpoint-topic", default="/scan", help="LaserScan topic used for optional endpoint capture")
    parser.add_argument("--lidar-endpoint-output", default="", help="Endpoint LiDAR NPZ base path; default is derived from --output")
    parser.add_argument("--lidar-endpoint-settle-s", type=float, default=0.30, help="Stationary settle time before endpoint LiDAR capture")
    parser.add_argument("--lidar-endpoint-capture-s", type=float, default=0.60, help="Endpoint LiDAR capture window after settling")
    parser.add_argument("--lidar-endpoint-min-scans", type=int, default=4, help="Minimum scans required for a valid stationary endpoint capture")
    parser.add_argument("--lidar-endpoint-max-wait-s", type=float, default=2.0, help="Maximum wait for endpoint scans before failing the run")
    parser.add_argument("--lidar-endpoint-chunk-size", type=int, default=60, help="Endpoint frames per compressed NPZ chunk")
    parser.add_argument("--lidar-beam-time-order", choices=("reverse", "forward"), default="reverse", help="Beam timing order metadata stored with endpoint captures")
    parser.add_argument("--lidar-offset-x", type=float, default=DEFAULT_LIDAR_OFFSET_X)
    parser.add_argument("--lidar-offset-y", type=float, default=DEFAULT_LIDAR_OFFSET_Y)
    parser.add_argument("--lidar-offset-h", type=float, default=DEFAULT_LIDAR_OFFSET_H)
    parser.add_argument("--yes", action="store_true", help="Skip the initial safety prompt")

    parser.add_argument("--otos-rate", type=float, default=150.0)
    parser.add_argument("--otos-offset-x", type=float, default=0.0)
    parser.add_argument("--otos-offset-y", type=float, default=0.0)
    parser.add_argument("--otos-offset-h", type=float, default=0.0)
    parser.add_argument("--linear-scalar", type=float, default=1.0)
    parser.add_argument("--angular-scalar", type=float, default=1.0)
    parser.add_argument("--calibrate-imu", dest="calibrate_imu", action="store_true", default=True, help="Calibrate OTOS IMU at startup while the robot is stationary")
    parser.add_argument("--no-calibrate-imu", dest="calibrate_imu", action="store_false", help="Skip startup OTOS IMU calibration")
    parser.add_argument("--otos-imu-calibration-retries", type=int, default=3, help="Retry OTOS IMU calibration if stationary yaw drift remains too high")
    parser.add_argument("--otos-stationary-drift-check-s", type=float, default=2.0, help="Stationary yaw-drift verification window after each OTOS IMU calibration")
    parser.add_argument("--otos-stationary-drift-warmup-s", type=float, default=0.35, help="Post-reset samples to discard before measuring stationary yaw drift")
    parser.add_argument("--otos-max-stationary-yaw-drift-dps", type=float, default=0.06, help="Maximum accepted stationary OTOS yaw drift after IMU calibration")
    parser.add_argument("--otos-allow-stationary-yaw-drift", action="store_true", help="Continue even if the post-calibration stationary yaw drift check fails")
    parser.add_argument("--otos-setup-timeout-s", type=float, default=20.0, help="Maximum time to wait for OTOS setup, calibration, and drift checks before origin averaging")
    parser.add_argument("--otos-origin-settle-s", type=float, default=0.75, help="Stationary settle time after OTOS startup/reset before defining lane origin")
    parser.add_argument("--otos-origin-average-s", type=float, default=0.50, help="Stationary averaging window used to define absolute lane origin")
    parser.add_argument("--reset-tracking", dest="reset_tracking", action="store_true", default=True)
    parser.add_argument("--no-reset-tracking", dest="reset_tracking", action="store_false")

    parser.add_argument("--drivebase-debug", action="store_true", help="Enable DriveBase debug prints")
    parser.add_argument("--drivebase-kp", type=float, default=0.7)
    parser.add_argument("--drivebase-ki", type=float, default=1.6)
    parser.add_argument("--drivebase-u-vel-ff", type=float, default=1.15)
    parser.add_argument("--drivebase-u-static", type=float, default=0.10)
    parser.add_argument("--drivebase-speed-ff-profile", dest="drivebase_speed_ff_profile", action="store_true", default=True)
    parser.add_argument("--no-drivebase-speed-ff-profile", dest="drivebase_speed_ff_profile", action="store_false")
    parser.add_argument("--drivebase-speed-ff-correction-clamp", type=float, default=0.22)
    parser.add_argument("--drivebase-v-slew-mps2", type=float, default=0.35)
    parser.add_argument("--drivebase-v-decel-slew-mps2", type=float, default=1.40)
    parser.add_argument("--drivebase-decel-ff-disable-margin-mps", type=float, default=0.03)
    parser.add_argument("--drivebase-velocity-duty-slew-per-s", type=float, default=3.00)
    parser.add_argument("--drivebase-creep-duty-slew-per-s", type=float, default=1.20)
    parser.add_argument("--drivebase-steering-center-trim", type=float, default=-0.017)
    parser.add_argument("--drivebase-pwm-readback-hz", type=float, default=20.0)
    return parser


def validate_args(args) -> None:
    if args.distance_m < args.min_distance_m:
        raise SystemExit("--distance-m is too small")
    if args.sample_period_s <= 0.0 or args.otos_rate <= 0.0:
        raise SystemExit("--sample-period-s and --otos-rate must be > 0")
    if args.otos_origin_settle_s < 0.0 or args.otos_origin_average_s < 0.0:
        raise SystemExit("--otos-origin-settle-s and --otos-origin-average-s must be >= 0")
    if args.lidar_endpoint_capture:
        if args.lidar_endpoint_settle_s < 0.0 or args.lidar_endpoint_capture_s < 0.0 or args.lidar_endpoint_max_wait_s <= 0.0:
            raise SystemExit("--lidar-endpoint-settle-s/capture-s must be >= 0 and --lidar-endpoint-max-wait-s must be > 0")
        if args.lidar_endpoint_min_scans < 1 or args.lidar_endpoint_chunk_size < 1:
            raise SystemExit("--lidar-endpoint-min-scans and --lidar-endpoint-chunk-size must be >= 1")
    if args.otos_imu_calibration_retries < 1:
        raise SystemExit("--otos-imu-calibration-retries must be >= 1")
    if args.otos_stationary_drift_check_s < 0.0 or args.otos_stationary_drift_warmup_s < 0.0 or args.otos_max_stationary_yaw_drift_dps < 0.0:
        raise SystemExit("--otos-stationary-drift-check-s, --otos-stationary-drift-warmup-s, and --otos-max-stationary-yaw-drift-dps must be >= 0")
    if args.otos_setup_timeout_s <= 0.0:
        raise SystemExit("--otos-setup-timeout-s must be > 0")
    if args.pause_s < 0.0 or args.brake_s < 0.0 or args.countdown_s < 0.0:
        raise SystemExit("--pause-s, --brake-s, and --countdown-s must be >= 0")
    if args.max_leg_time_s <= 0.0 or args.max_reposition_time_s <= 0.0:
        raise SystemExit("--max-leg-time-s and --max-reposition-time-s must be > 0")
    if args.measurement_no_motion_timeout_s < 0.0 or args.measurement_no_motion_progress_m < 0.0:
        raise SystemExit("--measurement-no-motion-timeout-s and --measurement-no-motion-progress-m must be >= 0")
    if args.repeat < 1 or args.reposition_max_attempts < 1:
        raise SystemExit("--repeat and --reposition-max-attempts must be >= 1")
    if args.endpoint_tolerance_m < 0.0 or args.max_endpoint_overshoot_m < 0.0:
        raise SystemExit("--endpoint-tolerance-m and --max-endpoint-overshoot-m must be >= 0")
    if not -1.0 <= args.steering <= 1.0:
        raise SystemExit("--steering must be in [-1, 1]")
    if args.heading_hold_kp < 0.0 or args.heading_hold_steer_max < 0.0 or args.heading_hold_arm_progress_m < 0.0:
        raise SystemExit("--heading-hold-kp, --heading-hold-steer-max, and --heading-hold-arm-progress-m must be >= 0")
    if args.heading_hold_steer_max > 1.0:
        raise SystemExit("--heading-hold-steer-max must be <= 1")
    if args.lateral_hold_kp < 0.0 or args.lateral_hold_steer_max < 0.0:
        raise SystemExit("--lateral-hold-kp and --lateral-hold-steer-max must be >= 0")
    if args.lateral_hold_steer_max > 1.0:
        raise SystemExit("--lateral-hold-steer-max must be <= 1")
    if not 0.0 <= args.brake_level <= 1.0:
        raise SystemExit("--brake-level must be in [0, 1]")
    if not 0.0 < args.reposition_duty <= 1.0:
        raise SystemExit("--reposition-duty must be in (0, 1]")
    if any(v > 1.0 for v in args.duties):
        raise SystemExit("--duties must be <= 1.0")
    if min(args.drivebase_kp, args.drivebase_ki, args.drivebase_u_vel_ff, args.drivebase_u_static) < 0.0:
        raise SystemExit("DriveBase gains/feedforward values must be >= 0")
    if args.drivebase_speed_ff_correction_clamp < 0.0:
        raise SystemExit("--drivebase-speed-ff-correction-clamp must be >= 0")
    if min(args.drivebase_v_slew_mps2, args.drivebase_v_decel_slew_mps2, args.drivebase_velocity_duty_slew_per_s, args.drivebase_creep_duty_slew_per_s) < 0.0:
        raise SystemExit("DriveBase slew values must be >= 0")


def output_path(args) -> Path:
    if args.output:
        return Path(args.output).expanduser()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("/tmp") / f"drivebase_speed_cal_{stamp}.csv"


def direction_order(args) -> list[str]:
    if args.directions == "both":
        return ["forward", "reverse"]
    return [args.directions]


def command_values(args) -> list[tuple[str, int, float]]:
    values: list[tuple[str, int, float]] = []
    if args.mode in ("duty-sweep", "both"):
        values.extend(("duty", idx, duty) for idx, duty in enumerate(args.duties, start=1))
    if args.mode in ("speed-sweep", "both"):
        values.extend(("speed", idx, speed) for idx, speed in enumerate(args.speeds, start=1))
    return values


def measurement_plan(args) -> list[dict]:
    plan: list[dict] = []
    for repeat_index in range(1, int(args.repeat) + 1):
        for direction in direction_order(args):
            sign = 1.0 if direction == "forward" else -1.0
            start_role = "home" if direction == "forward" else "far"
            target_role = "far" if direction == "forward" else "home"
            for kind, idx, value in command_values(args):
                plan.append(
                    {
                        "name": f"r{repeat_index:02d}_{kind}_{idx:02d}_{direction}_{value:.3f}",
                        "kind": kind,
                        "control": "duty" if kind == "duty" else "speed",
                        "is_measurement": 1,
                        "repeat": repeat_index,
                        "start_role": start_role,
                        "target_role": target_role,
                        "planned_direction": direction,
                        "value": float(value),
                        "sign": sign,
                        "direction": direction,
                        "command": sign * float(value),
                    }
                )
    return plan


def countdown(seconds: float) -> None:
    end_t = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = end_t - time.monotonic()
        if remaining <= 0.0:
            break
        print(f"  starting in {remaining:0.1f}s", end="\r", flush=True)
        time.sleep(min(0.2, remaining))
    print("  starting now        ")


def angular_error_deg(current: float, reference: float) -> float:
    return (float(current) - float(reference) + 180.0) % 360.0 - 180.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def lane_hold_steering(args, heading_error_deg: float, lateral_error_m: float, command_signed: float) -> tuple[float, float, float]:
    heading_correction = 0.0
    if bool(getattr(args, "heading_hold", True)):
        drive_sign = 1.0 if float(command_signed) >= 0.0 else -1.0

        # DriveBase steering convention on this robot is opposite to mathematical
        # bicycle steering: negative normalized steering turns the car left while
        # driving forward. Therefore a positive heading error needs positive
        # DriveBase steering in forward motion, and the opposite while reversing.
        response_sign = drive_sign
        if str(getattr(args, "heading_hold_sign", "normal")) == "inverted":
            response_sign *= -1.0
        heading_correction = response_sign * float(args.heading_hold_kp) * float(heading_error_deg)
        heading_correction = clamp(
            heading_correction,
            -float(args.heading_hold_steer_max),
            float(args.heading_hold_steer_max),
        )

    lateral_correction = 0.0
    if bool(getattr(args, "lateral_hold", True)):
        # Positive lane-lateral means the robot is left of the stored lane axis.
        # Positive DriveBase steering turns right in forward motion and creates
        # the matching corrective heading while reversing, so this term is
        # independent of drive direction.
        lateral_sign = 1.0
        if str(getattr(args, "lateral_hold_sign", "normal")) == "inverted":
            lateral_sign = -1.0
        lateral_correction = lateral_sign * float(args.lateral_hold_kp) * float(lateral_error_m)
        lateral_correction = clamp(
            lateral_correction,
            -float(args.lateral_hold_steer_max),
            float(args.lateral_hold_steer_max),
        )

    steering = clamp(float(args.steering) + heading_correction + lateral_correction, -1.0, 1.0)
    return steering, heading_correction, lateral_correction


def heading_hold_steering(args, heading_error_deg: float, command_signed: float) -> float:
    steering, _heading_correction, _lateral_correction = lane_hold_steering(args, heading_error_deg, 0.0, command_signed)
    return steering


def project_along_heading(origin: OTOSEntry, current: OTOSEntry, heading_deg: float) -> tuple[float, float, float]:
    heading = math.radians(float(heading_deg))
    dx = current.x - origin.x
    dy = current.y - origin.y
    along = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    return along, lateral, math.hypot(dx, dy)


def lane_coordinate(origin: OTOSEntry, axis_h_deg: float, current: OTOSEntry) -> tuple[float, float, float]:
    return project_along_heading(origin, current, axis_h_deg)


def endpoint_coord(args, role: str) -> float:
    if role == "home":
        return 0.0
    far = float(args.distance_m) if args.start_direction == "forward" else -float(args.distance_m)
    if role == "far":
        return far
    raise ValueError(f"unknown endpoint role: {role}")


def drive_command_for_lane_remaining(args, remaining_m: float, duty_magnitude: float) -> float:
    if abs(float(remaining_m)) <= 1e-9:
        return 0.0
    lane_target_sign = 1.0 if float(remaining_m) >= 0.0 else -1.0
    far_sign = 1.0 if endpoint_coord(args, "far") >= 0.0 else -1.0
    return lane_target_sign * far_sign * abs(float(duty_magnitude))


def otos_forward_speed(entry: OTOSEntry) -> float:
    heading = math.radians(entry.h)
    return entry.vx * math.cos(heading) + entry.vy * math.sin(heading)


def row_from_state(segment: dict, db, otos: Optional[OTOSEntry], leg_start: Optional[OTOSEntry], t0: float, leg_start_t: float, event: str, stop_reason: str = "") -> dict:
    telem = db.get_telemetry()
    leg_progress = leg_lateral = leg_displacement = leg_heading_error = float("nan")
    heading_hold_steer_norm = float(segment.get("heading_hold_steer_norm", float("nan")))
    heading_hold_heading_correction_norm = float(segment.get("heading_hold_heading_correction_norm", float("nan")))
    heading_hold_lateral_correction_norm = float(segment.get("heading_hold_lateral_correction_norm", float("nan")))
    lane_coord_m = lane_lateral_m = lane_remaining_m = lane_target_m = float("nan")
    lane_origin = None
    if "lane_origin_x" in segment:
        lane_origin = OTOSEntry(
            t=0.0,
            x=float(segment.get("lane_origin_x", 0.0)),
            y=float(segment.get("lane_origin_y", 0.0)),
            h=float(segment.get("lane_axis_h_deg", 0.0)),
            vx=0.0,
            vy=0.0,
            vh=0.0,
        )
    if otos is not None and leg_start is not None and lane_origin is not None:
        along_start, leg_lateral, leg_displacement = project_along_heading(leg_start, otos, float(segment.get("lane_axis_h_deg", 0.0)))
        leg_progress = float(segment.get("sign", 1.0)) * along_start
        leg_heading_error = angular_error_deg(otos.h, float(segment.get("lane_axis_h_deg", 0.0)))
    if otos is not None and lane_origin is not None:
        lane_coord_m, lane_lateral_m, _lane_disp = lane_coordinate(lane_origin, float(segment.get("lane_axis_h_deg", 0.0)), otos)
        lane_target_m = float(segment.get("target_coord_m", float("nan")))
        if math.isfinite(lane_target_m):
            lane_remaining_m = lane_target_m - lane_coord_m
    return {
        "t_monotonic": f"{time.monotonic():.6f}",
        "run_elapsed_s": f"{time.monotonic() - t0:.4f}",
        "segment": segment.get("name", ""),
        "event": event,
        "stop_reason": stop_reason,
        "kind": segment.get("kind", ""),
        "control": segment.get("control", ""),
        "is_measurement": int(bool(segment.get("is_measurement", 0))),
        "repeat": segment.get("repeat", ""),
        "start_role": segment.get("start_role", ""),
        "target_role": segment.get("target_role", ""),
        "planned_direction": segment.get("planned_direction", ""),
        "direction": segment.get("direction", ""),
        "command_value": f"{float(segment.get('value', 0.0)):.4f}",
        "command_signed": f"{float(segment.get('command', 0.0)):.4f}",
        "leg_elapsed_s": f"{time.monotonic() - leg_start_t:.4f}" if leg_start_t else "",
        "otos_t": "" if otos is None else f"{otos.t:.6f}",
        "otos_age_ms": "" if otos is None else f"{(time.monotonic() - otos.t) * 1000.0:.3f}",
        "otos_x_m": "" if otos is None else f"{otos.x:.6f}",
        "otos_y_m": "" if otos is None else f"{otos.y:.6f}",
        "otos_h_deg": "" if otos is None else f"{otos.h:.3f}",
        "otos_vx_mps": "" if otos is None else f"{otos.vx:.6f}",
        "otos_vy_mps": "" if otos is None else f"{otos.vy:.6f}",
        "otos_vh_dps": "" if otos is None else f"{otos.vh:.6f}",
        "otos_forward_mps": "" if otos is None else f"{otos_forward_speed(otos):.6f}",
        "leg_progress_m": "" if not math.isfinite(leg_progress) else f"{leg_progress:.6f}",
        "leg_lateral_m": "" if not math.isfinite(leg_lateral) else f"{leg_lateral:.6f}",
        "leg_displacement_m": "" if not math.isfinite(leg_displacement) else f"{leg_displacement:.6f}",
        "leg_heading_error_deg": "" if not math.isfinite(leg_heading_error) else f"{leg_heading_error:.3f}",
        "heading_hold_enabled": int(bool(segment.get("heading_hold_enabled", False))),
        "heading_hold_armed": int(bool(segment.get("heading_hold_armed", False))),
        "lateral_hold_enabled": int(bool(segment.get("lateral_hold_enabled", False))),
        "heading_hold_target_deg": segment.get("heading_hold_target_deg", ""),
        "heading_hold_steer_norm": "" if not math.isfinite(heading_hold_steer_norm) else f"{heading_hold_steer_norm:.5f}",
        "heading_hold_heading_correction_norm": "" if not math.isfinite(heading_hold_heading_correction_norm) else f"{heading_hold_heading_correction_norm:.5f}",
        "heading_hold_lateral_correction_norm": "" if not math.isfinite(heading_hold_lateral_correction_norm) else f"{heading_hold_lateral_correction_norm:.5f}",
        "lane_coord_m": "" if not math.isfinite(lane_coord_m) else f"{lane_coord_m:.6f}",
        "lane_target_m": "" if not math.isfinite(lane_target_m) else f"{lane_target_m:.6f}",
        "lane_remaining_m": "" if not math.isfinite(lane_remaining_m) else f"{lane_remaining_m:.6f}",
        "lane_lateral_m": "" if not math.isfinite(lane_lateral_m) else f"{lane_lateral_m:.6f}",
        "leg_target_distance_m": segment.get("leg_target_distance_m", ""),
        "db_mode": telem.get("mode", ""),
        "db_submode": telem.get("submode", ""),
        "db_target_user_mps": telem.get("target_user_mps", ""),
        "db_target_ramped_mps": telem.get("target_ramped_mps", ""),
        "db_measured_mps": telem.get("measured_mps", ""),
        "db_rate_mps": telem.get("rate_mps", ""),
        "db_position_m": telem.get("position_m", ""),
        "db_count": telem.get("count", ""),
        "db_last_duty": "" if telem.get("last_duty") is None else telem.get("last_duty"),
        "db_raw_duty_before_direction_guard": telem.get("raw_duty_before_direction_guard", ""),
        "db_actuating": int(bool(telem.get("actuating", False))),
        "drive_pwm_mode": telem.get("drive_pwm_mode", ""),
        "drive_pwm_throttle": telem.get("drive_pwm_throttle", ""),
        "drive_pwm_forward": telem.get("drive_pwm_forward", ""),
        "drive_pwm_reverse": telem.get("drive_pwm_reverse", ""),
        "drive_pwm_brake_level": telem.get("drive_pwm_brake_level", ""),
    }


def print_sample(row: dict) -> None:
    tag = "MEAS" if int(row.get("is_measurement") or 0) else "POS "
    print(
        f"  {tag} {row['segment']} t={float(row['leg_elapsed_s'] or 0.0):5.2f}s "
        f"lane={float(row['lane_coord_m'] or 0.0):+0.3f}->{float(row['lane_target_m'] or 0.0):+0.3f}m "
        f"prog={float(row['leg_progress_m'] or 0.0):+0.3f}m "
        f"lat={float(row['lane_lateral_m'] or row['leg_lateral_m'] or 0.0):+0.3f}m "
        f"steer={float(row['heading_hold_steer_norm'] or 0.0):+0.3f} "
        f"otos_v={float(row['otos_forward_mps'] or 0.0):+0.3f} "
        f"db_v={float(row['db_measured_mps'] or 0.0):+0.3f} "
        f"u={row['db_last_duty'] if row['db_last_duty'] != '' else '--'}"
    )


def stop_drive(db, args) -> None:
    try:
        db.brake(float(args.brake_level))
        time.sleep(max(0.0, float(args.brake_s)))
        db.neutral()
    except Exception:
        try:
            db.coast()
        except Exception:
            pass


def prepare_segment_for_lane(args, segment: dict, lane_origin: OTOSEntry, lane_axis_h_deg: float, current: OTOSEntry) -> dict:
    prepared = dict(segment)
    target_coord = endpoint_coord(args, str(prepared.get("target_role", "far")))
    current_coord, current_lateral, _disp = lane_coordinate(lane_origin, lane_axis_h_deg, current)
    remaining = target_coord - current_coord
    lane_target_sign = 1.0 if remaining >= 0.0 else -1.0

    if int(bool(prepared.get("is_measurement", 0))):
        command = float(prepared.get("command", 0.0))
    else:
        # Repositioning is absolute: every tick recomputes the command from the
        # current lane coordinate to the fixed home/far endpoint. This initial
        # value is only used for startup/logging before the first loop update.
        command = drive_command_for_lane_remaining(args, remaining, float(args.reposition_duty))

    actual_direction = "forward" if command >= 0.0 else "reverse"
    prepared.update(
        {
            "lane_origin_x": lane_origin.x,
            "lane_origin_y": lane_origin.y,
            "lane_axis_h_deg": lane_axis_h_deg,
            "lane_far_coord_m": endpoint_coord(args, "far"),
            "target_coord_m": target_coord,
            "heading_hold_enabled": int(bool(getattr(args, "heading_hold", True))),
            "heading_hold_armed": 0,
            "lateral_hold_enabled": int(bool(getattr(args, "lateral_hold", True))),
            "heading_hold_target_deg": f"{float(lane_axis_h_deg):.3f}",
            "start_lane_coord_m": current_coord,
            "start_lane_lateral_m": current_lateral,
            "leg_target_distance_m": f"{abs(remaining):.6f}",
            "sign": lane_target_sign,
            "direction": actual_direction,
            "command": command,
        }
    )
    return prepared


def run_segment(args, db, otos: OTOSTelemetry, writer: csv.DictWriter, segment: dict, run_t0: float, lane_origin: OTOSEntry, lane_axis_h_deg: float, lidar_recorder: Optional[LidarEndpointRecorder] = None) -> str:
    start = otos.wait_first(1.0)
    segment = prepare_segment_for_lane(args, segment, lane_origin, lane_axis_h_deg, start)
    leg_start_t = time.monotonic()
    next_sample_t = leg_start_t
    path_accum_m = 0.0
    last_otos = start
    stop_reason = ""
    timeout_s = float(segment.get("timeout_s", args.max_leg_time_s))

    target_distance = abs(float(segment["target_coord_m"]) - float(segment["start_lane_coord_m"]))
    print("\n" + "=" * 80)
    print(
        f"{segment['name']}: {segment['kind']} {segment['control']} command={segment['command']:+.3f}, "
        f"lane {segment['start_lane_coord_m']:+.3f}m -> {segment['target_coord_m']:+.3f}m "
        f"remaining={target_distance:.3f}m"
    )

    if target_distance <= float(args.endpoint_tolerance_m):
        stop_reason = "already_at_endpoint"
        db.neutral()
        row = row_from_state(segment, db, start, start, run_t0, leg_start_t, "skip", stop_reason)
        row["leg_path_m"] = "0.000000"
        writer.writerow(row)
        print_sample(row)
        print(f"Segment skipped: already within {args.endpoint_tolerance_m:.3f}m of endpoint")
        time.sleep(max(0.0, float(args.pause_s)))
        return stop_reason

    lane_min = min(0.0, float(segment["lane_far_coord_m"])) - float(args.max_endpoint_overshoot_m)
    lane_max = max(0.0, float(segment["lane_far_coord_m"])) + float(args.max_endpoint_overshoot_m)
    start_target_error = abs(float(segment["target_coord_m"]) - float(segment["start_lane_coord_m"]))
    was_inside_endpoint_bounds = lane_min <= float(segment["start_lane_coord_m"]) <= lane_max

    initial_lateral_error = float(segment.get("start_lane_lateral_m", 0.0) or 0.0)
    current_steering, heading_corr, lateral_corr = lane_hold_steering(
        args,
        0.0,
        initial_lateral_error,
        float(segment["command"]),
    )
    segment["heading_hold_steer_norm"] = current_steering
    segment["heading_hold_heading_correction_norm"] = heading_corr
    segment["heading_hold_lateral_correction_norm"] = lateral_corr
    if lidar_recorder is not None:
        lidar_recorder.capture_endpoint(segment, "measurement_start", start, lane_origin, lane_axis_h_deg)

    db.set_steering(current_steering)
    if segment.get("control") == "duty":
        db.set_drive_power(float(segment["command"]))
    else:
        db.set_target_speed(float(segment["command"]))

    writer.writerow(row_from_state(segment, db, start, start, run_t0, leg_start_t, "start"))
    try:
        while True:
            now = time.monotonic()
            current = otos.latest()
            if current is None:
                stop_reason = "otos_missing"
                break
            age_ms = (now - current.t) * 1000.0
            if age_ms > float(args.max_otos_age_ms):
                stop_reason = "otos_stale"
                break

            path_accum_m += math.hypot(current.x - last_otos.x, current.y - last_otos.y)
            last_otos = current

            lane_coord_m, lane_lateral_m, _lane_disp = lane_coordinate(lane_origin, lane_axis_h_deg, current)
            target_remaining = float(segment["target_coord_m"]) - lane_coord_m
            if float(segment["sign"]) * target_remaining <= float(args.endpoint_tolerance_m):
                stop_reason = "endpoint_reached"
                break
            if not int(bool(segment.get("is_measurement", 0))):
                segment["command"] = drive_command_for_lane_remaining(args, target_remaining, float(args.reposition_duty))
                db.set_drive_power(float(segment["command"]))
            moved_from_leg_start = abs(lane_coord_m - float(segment.get("start_lane_coord_m", lane_coord_m)))
            if (
                bool(getattr(args, "heading_hold", True))
                and not bool(segment.get("heading_hold_armed", False))
                and (path_accum_m >= float(args.heading_hold_arm_progress_m) or moved_from_leg_start >= float(args.heading_hold_arm_progress_m))
            ):
                segment["heading_hold_armed"] = 1

            if bool(segment.get("heading_hold_armed", False)):
                heading_target_deg = float(segment.get("heading_hold_target_deg", lane_axis_h_deg) or lane_axis_h_deg)
                heading_error = angular_error_deg(current.h, heading_target_deg)
            else:
                heading_error = 0.0
            current_steering, heading_corr, lateral_corr = lane_hold_steering(
                args,
                heading_error,
                lane_lateral_m,
                float(segment["command"]),
            )
            segment["heading_hold_steer_norm"] = current_steering
            segment["heading_hold_heading_correction_norm"] = heading_corr
            segment["heading_hold_lateral_correction_norm"] = lateral_corr
            db.set_steering(current_steering)
            elapsed = now - leg_start_t

            inside_endpoint_bounds = lane_min <= lane_coord_m <= lane_max
            if inside_endpoint_bounds:
                was_inside_endpoint_bounds = True
            elif was_inside_endpoint_bounds or abs(target_remaining) > start_target_error + 0.02:
                stop_reason = "endpoint_overshoot"
                break
            if (
                int(bool(segment.get("is_measurement", 0)))
                and float(args.measurement_no_motion_timeout_s) > 0.0
                and elapsed >= float(args.measurement_no_motion_timeout_s)
                and max(path_accum_m, moved_from_leg_start) < float(args.measurement_no_motion_progress_m)
            ):
                stop_reason = "no_motion_timeout"
                break
            if elapsed >= timeout_s:
                stop_reason = "timeout"
                break
            if abs(lane_lateral_m) > float(args.max_lateral_m):
                stop_reason = "lateral_abort"
                break
            if bool(segment.get("heading_hold_armed", False)) and abs(heading_error) > float(args.max_heading_error_deg):
                stop_reason = "heading_abort"
                break

            if now >= next_sample_t:
                row = row_from_state(segment, db, current, start, run_t0, leg_start_t, "sample")
                row["leg_path_m"] = f"{path_accum_m:.6f}"
                writer.writerow(row)
                print_sample(row)
                next_sample_t += max(0.005, float(args.sample_period_s))
            time.sleep(0.003)
    finally:
        stop_drive(db, args)

    end = otos.latest()
    end_row = row_from_state(segment, db, end, start, run_t0, leg_start_t, "end", stop_reason)
    end_row["leg_path_m"] = f"{path_accum_m:.6f}"
    writer.writerow(end_row)
    print_sample(end_row)
    if lidar_recorder is not None and end is not None:
        lidar_recorder.capture_endpoint(
            segment,
            "measurement_end",
            end,
            lane_origin,
            lane_axis_h_deg,
            {"stop_reason": str(stop_reason), "leg_path_m": float(path_accum_m)},
        )
    print(f"Segment ended: {stop_reason}. Pausing {args.pause_s:.1f}s")
    time.sleep(max(0.0, float(args.pause_s)))
    return stop_reason


def reposition_segment(args, role: str, reason: str, attempt: int) -> dict:
    return {
        "name": f"position_{role}_{reason}_a{attempt}",
        "kind": "reposition",
        "control": "duty",
        "is_measurement": 0,
        "repeat": "",
        "start_role": "",
        "target_role": role,
        "planned_direction": "",
        "value": float(args.reposition_duty),
        "sign": 1.0,
        "direction": "",
        "command": float(args.reposition_duty),
        "timeout_s": float(args.max_reposition_time_s),
    }


def endpoint_error(args, otos_entry: OTOSEntry, lane_origin: OTOSEntry, lane_axis_h_deg: float, role: str) -> tuple[float, float, float]:
    coord, lateral, _disp = lane_coordinate(lane_origin, lane_axis_h_deg, otos_entry)
    return endpoint_coord(args, role) - coord, coord, lateral


def ensure_endpoint(args, db, otos: OTOSTelemetry, writer: csv.DictWriter, run_t0: float, lane_origin: OTOSEntry, lane_axis_h_deg: float, role: str, reason: str) -> tuple[bool, str]:
    for attempt in range(1, int(args.reposition_max_attempts) + 1):
        current = otos.wait_first(1.0)
        error, coord, lateral = endpoint_error(args, current, lane_origin, lane_axis_h_deg, role)
        if abs(error) <= float(args.endpoint_tolerance_m):
            print(f"Endpoint {role} ready for {reason}: lane={coord:+.3f}m lateral={lateral:+.3f}m")
            return True, "already_at_endpoint"
        print(f"Reposition for {reason}: target={role} error={error:+.3f}m attempt={attempt}")
        seg = reposition_segment(args, role, reason, attempt)
        result = run_segment(args, db, otos, writer, seg, run_t0, lane_origin, lane_axis_h_deg, None)
        if result in SAFETY_STOP_REASONS:
            return False, result
        current = otos.wait_first(1.0)
        error, coord, lateral = endpoint_error(args, current, lane_origin, lane_axis_h_deg, role)
        if abs(error) <= float(args.endpoint_tolerance_m):
            print(f"Endpoint {role} reached for {reason}: lane={coord:+.3f}m lateral={lateral:+.3f}m")
            return True, result
    return False, "endpoint_not_reached"


def write_measurement_skip(
    args,
    writer: csv.DictWriter,
    db,
    otos: OTOSTelemetry,
    measurement: dict,
    run_t0: float,
    lane_origin: OTOSEntry,
    lane_axis_h_deg: float,
    reason: str,
) -> None:
    now = time.monotonic()
    current = otos.latest()
    segment = measurement
    if current is not None:
        segment = prepare_segment_for_lane(args, measurement, lane_origin, lane_axis_h_deg, current)
    row = row_from_state(segment, db, current, current, run_t0, now, "measurement_skipped", reason)
    row["leg_path_m"] = "0.000000"
    writer.writerow(row)
    print(f"Skipping measurement {measurement['name']}: {reason}")


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args)

    from robot_core.drivebase import DriveBase, DriveConfig

    cfg = DriveConfig(
        debug=bool(args.drivebase_debug),
        kp=float(args.drivebase_kp),
        ki=float(args.drivebase_ki),
        u_vel_ff=float(args.drivebase_u_vel_ff),
        u_static=float(args.drivebase_u_static),
        speed_ff_profile_enable=bool(args.drivebase_speed_ff_profile),
        speed_ff_correction_clamp=float(args.drivebase_speed_ff_correction_clamp),
        v_slew_mps2=float(args.drivebase_v_slew_mps2),
        v_decel_slew_mps2=float(args.drivebase_v_decel_slew_mps2),
        decel_feedforward_disable_margin_mps=float(args.drivebase_decel_ff_disable_margin_mps),
        velocity_duty_slew_per_s=float(args.drivebase_velocity_duty_slew_per_s),
        creep_duty_slew_per_s=float(args.drivebase_creep_duty_slew_per_s),
        steering_center_trim=float(args.drivebase_steering_center_trim),
        pwm_readback_hz=float(args.drivebase_pwm_readback_hz),
    )
    measurements = measurement_plan(args)
    csv_path = output_path(args)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    print("DriveBase speed calibration suite")
    print(f"Mode: {args.mode}")
    print(f"Directions: {args.directions}")
    print(f"Measurements: {len(measurements)}")
    print(f"Lane distance: {args.distance_m:.2f}m")
    print(f"Reposition duty: {args.reposition_duty:.2f}")
    print(f"CSV: {csv_path}")
    if not args.yes:
        input("This script commands hardware automatically. Place robot at home in the clear lane, then press Enter...")
    countdown(args.countdown_s)

    otos = OTOSTelemetry(args)
    lidar_recorder = LidarEndpointRecorder(args, csv_path)
    db = DriveBase(cfg)
    header_segment = {
        "name": "header",
        "kind": "",
        "control": "",
        "is_measurement": 0,
        "repeat": "",
        "start_role": "",
        "target_role": "",
        "planned_direction": "",
        "value": 0.0,
        "sign": 1.0,
        "direction": "",
        "command": 0.0,
    }
    fieldnames = list(row_from_state(header_segment, db, None, None, time.monotonic(), 0.0, "header").keys())
    if "leg_path_m" not in fieldnames:
        fieldnames.append("leg_path_m")

    try:
        lidar_recorder.start()
        if bool(getattr(args, "lidar_endpoint_capture", False)) and not lidar_recorder.wait_first(float(args.lidar_endpoint_max_wait_s)):
            raise RuntimeError(f"Timed out waiting for LiDAR scans on {args.lidar_endpoint_topic}")
        otos.start(float(args.otos_setup_timeout_s))
        lane_origin = averaged_stationary_origin(
            otos,
            float(args.otos_origin_settle_s),
            float(args.otos_origin_average_s),
        )
        lane_axis_h_deg = float(lane_origin.h)
        print(f"Lane origin: x={lane_origin.x:+.3f}m y={lane_origin.y:+.3f}m h={lane_axis_h_deg:+.2f}deg")
        print(
            f"Lane endpoints: home=0.000m far={endpoint_coord(args, 'far'):+.3f}m "
            f"tolerance={args.endpoint_tolerance_m:.3f}m"
        )
        db.start()
        time.sleep(0.2)
        run_t0 = time.monotonic()
        abort = False
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for measurement in measurements:
                ok, reason = ensure_endpoint(args, db, otos, writer, run_t0, lane_origin, lane_axis_h_deg, str(measurement["start_role"]), "measurement_start")
                f.flush()
                if reason in SAFETY_STOP_REASONS:
                    print(f"Aborting remaining measurements after safety stop while positioning: {reason}")
                    abort = True
                    break
                if not ok:
                    write_measurement_skip(args, writer, db, otos, measurement, run_t0, lane_origin, lane_axis_h_deg, reason)
                    f.flush()
                    continue

                result = run_segment(args, db, otos, writer, measurement, run_t0, lane_origin, lane_axis_h_deg, lidar_recorder)
                f.flush()
                if result in SAFETY_STOP_REASONS:
                    print(f"Aborting remaining measurements after safety stop: {result}")
                    abort = True
                    break
                if bool(args.reset_after_measurement):
                    ok, reason = ensure_endpoint(args, db, otos, writer, run_t0, lane_origin, lane_axis_h_deg, str(measurement["start_role"]), "measurement_reset")
                    f.flush()
                    if reason in SAFETY_STOP_REASONS:
                        print(f"Aborting remaining measurements after safety stop during reset: {reason}")
                        abort = True
                        break
            if not abort and args.finish_at != "current":
                ok, reason = ensure_endpoint(args, db, otos, writer, run_t0, lane_origin, lane_axis_h_deg, str(args.finish_at), "finish")
                f.flush()
                if reason in SAFETY_STOP_REASONS:
                    print(f"Finish reposition stopped by safety reason: {reason}")
        print("\nCalibration complete.")
        print(f"Log written to {csv_path}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping drive and shutting down.")
        return 130
    finally:
        try:
            stop_drive(db, args)
            db.set_steering(0.0)
            time.sleep(0.1)
            db.shutdown()
        except Exception:
            pass
        try:
            lidar_recorder.stop()
        except Exception:
            pass
        otos.stop()


if __name__ == "__main__":
    raise SystemExit(main())
