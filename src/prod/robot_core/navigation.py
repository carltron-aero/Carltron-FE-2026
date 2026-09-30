"""Pose-based navigation debug planner for the WRO FE field.

This module is intentionally command-only/debug-oriented for now. It converts
the current fused pose and obstacle memory into a short-horizon route preview,
but does not touch DriveBase or hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


FIELD_HALF_M = 1.5
ORDER_CLOCK = ["bottom", "left", "top", "right"]
ORDER_COUNTER = ["bottom", "right", "top", "left"]


@dataclass(frozen=True)
class NavFrame:
    section: str
    direction: str
    along_m: float
    lateral_m: float
    heading_error_deg: float


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _direction_from_pose(pose, args) -> str:
    requested = getattr(args, "navigation_direction", "auto")
    if requested in ("clock", "counter"):
        return requested.upper()
    # On the bottom start section, heading 180 means clockwise, heading 0 means
    # counterclockwise. This remains consistent around the rectangular loop.
    return "CLOCK" if math.cos(math.radians(pose.h)) < 0.0 else "COUNTER"


def _section_for_pose(pose) -> str:
    x, y = float(pose.x), float(pose.y)
    if abs(y) >= abs(x):
        return "top" if y >= 0.0 else "bottom"
    return "right" if x >= 0.0 else "left"


def _section_axes(section: str, direction: str) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Return tangent, outward-normal, and nominal heading for a side section."""
    clockwise = direction == "CLOCK"
    if section == "bottom":
        tangent = (-1.0, 0.0) if clockwise else (1.0, 0.0)
        outward = (0.0, -1.0)
        heading = 180.0 if clockwise else 0.0
    elif section == "left":
        tangent = (0.0, 1.0) if clockwise else (0.0, -1.0)
        outward = (-1.0, 0.0)
        heading = 90.0 if clockwise else -90.0
    elif section == "top":
        tangent = (1.0, 0.0) if clockwise else (-1.0, 0.0)
        outward = (0.0, 1.0)
        heading = 0.0 if clockwise else 180.0
    else:
        tangent = (0.0, -1.0) if clockwise else (0.0, 1.0)
        outward = (1.0, 0.0)
        heading = -90.0 if clockwise else 90.0
    return tangent, outward, heading


def _section_progress(section: str, direction: str, x: float, y: float) -> float:
    clockwise = direction == "CLOCK"
    if section == "bottom":
        return FIELD_HALF_M - x if clockwise else x + FIELD_HALF_M
    if section == "left":
        return y + FIELD_HALF_M if clockwise else FIELD_HALF_M - y
    if section == "top":
        return x + FIELD_HALF_M if clockwise else FIELD_HALF_M - x
    return FIELD_HALF_M - y if clockwise else y + FIELD_HALF_M


def _section_lateral(section: str, x: float, y: float) -> float:
    if section == "bottom":
        return y + FIELD_HALF_M
    if section == "left":
        return x + FIELD_HALF_M
    if section == "top":
        return FIELD_HALF_M - y
    return FIELD_HALF_M - x


def _point_on_section(section: str, direction: str, along: float, lateral: float) -> tuple[float, float]:
    clockwise = direction == "CLOCK"
    along = along % 3.0
    if section == "bottom":
        x = FIELD_HALF_M - along if clockwise else -FIELD_HALF_M + along
        y = -FIELD_HALF_M + lateral
    elif section == "left":
        x = -FIELD_HALF_M + lateral
        y = -FIELD_HALF_M + along if clockwise else FIELD_HALF_M - along
    elif section == "top":
        x = -FIELD_HALF_M + along if clockwise else FIELD_HALF_M - along
        y = FIELD_HALF_M - lateral
    else:
        x = FIELD_HALF_M - lateral
        y = FIELD_HALF_M - along if clockwise else -FIELD_HALF_M + along
    return x, y


def _advance_section(section: str, direction: str, step: int = 1) -> str:
    order = ORDER_CLOCK if direction == "CLOCK" else ORDER_COUNTER
    return order[(order.index(section) + step) % len(order)]


def _section_delta(from_section: str, to_section: str, direction: str) -> int:
    order = ORDER_CLOCK if direction == "CLOCK" else ORDER_COUNTER
    return (order.index(to_section) - order.index(from_section)) % len(order)


def _slot_along_lateral(slot: dict, section: str, direction: str) -> tuple[float, float]:
    return (
        _section_progress(section, direction, float(slot["x"]), float(slot["y"])),
        _section_lateral(section, float(slot["x"]), float(slot["y"])),
    )


def navigation_frame(pose, args) -> NavFrame:
    direction = _direction_from_pose(pose, args)
    section = _section_for_pose(pose)
    _tangent, _outward, nominal_heading = _section_axes(section, direction)
    return NavFrame(
        section=section,
        direction=direction,
        along_m=_section_progress(section, direction, float(pose.x), float(pose.y)),
        lateral_m=_section_lateral(section, float(pose.x), float(pose.y)),
        heading_error_deg=_wrap_deg(float(pose.h) - nominal_heading),
    )


def _desired_lateral_for_obstacle(slot: dict, direction: str, args) -> tuple[float, str]:
    color = slot.get("known_color") or slot.get("current_color") or slot.get("color") or ""
    row = float(slot.get("row_m", 0.5))
    if color == "RED":
        # Matches the old combinedfollower behavior: red causes the robot to
        # choose the outer/far lane when driving clockwise, reversed for counter.
        desired, reason = (args.navigation_far_lane_m if direction == "CLOCK" else args.navigation_close_lane_m), "red"
    elif color == "GREEN":
        desired, reason = (args.navigation_close_lane_m if direction == "CLOCK" else args.navigation_far_lane_m), "green"
    elif row <= 0.5:
        desired, reason = args.navigation_far_lane_m, "unknown_near"
    else:
        desired, reason = args.navigation_close_lane_m, "unknown_far"

    clearance = float(args.navigation_obstacle_clearance_m)
    if desired >= row:
        desired = max(desired, row + clearance)
    else:
        desired = min(desired, row - clearance)
    return (
        _clamp(desired, args.navigation_min_lane_m, args.navigation_max_lane_m),
        reason,
    )


def _navigation_obstacles(obstacles: list[dict], frame: NavFrame, args) -> list[dict]:
    decisions = []
    for slot in obstacles:
        if not (slot.get("known") or slot.get("current")):
            continue
        slot_section = slot.get("section")
        if slot_section not in ORDER_CLOCK:
            continue
        along, lateral = _slot_along_lateral(slot, slot_section, frame.direction)
        section_delta = _section_delta(frame.section, slot_section, frame.direction)
        if section_delta == 0:
            delta = along - frame.along_m
        else:
            delta = 3.0 * section_delta + along - frame.along_m
        if delta < -args.navigation_obstacle_behind_m or delta > args.navigation_horizon_m:
            continue
        desired, reason = _desired_lateral_for_obstacle(slot, frame.direction, args)
        confidence = max(
            float(slot.get("known_color_confidence", 0.0) or 0.0),
            float(slot.get("raw_color_confidence", 0.0) or 0.0),
        )
        decisions.append(
            {
                "id": slot.get("id", ""),
                "section": slot_section,
                "x": float(slot["x"]),
                "y": float(slot["y"]),
                "along_m": along,
                "delta_m": delta,
                "row_m": lateral,
                "clearance_m": abs(desired - lateral),
                "color": slot.get("known_color") or slot.get("current_color") or slot.get("color") or "",
                "color_confidence": confidence,
                "target_lateral_m": desired,
                "reason": reason,
                "current": bool(slot.get("current")),
                "known": bool(slot.get("known")),
            }
        )
    decisions.sort(key=lambda item: item["delta_m"])
    return decisions


def _lateral_at(along_delta: float, decisions: list[dict], args) -> float:
    lateral = float(args.navigation_base_lane_m)
    for obs in decisions:
        center = float(obs["delta_m"])
        half = float(args.navigation_obstacle_influence_m)
        start = center - half
        end = center + half
        if along_delta < start or along_delta > end:
            continue
        u = (along_delta - start) / max(2.0 * half, 1e-6)
        weight = 0.5 - 0.5 * math.cos(2.0 * math.pi * u)
        lateral += (float(obs["target_lateral_m"]) - lateral) * weight
    return _clamp(lateral, args.navigation_min_lane_m, args.navigation_max_lane_m)


def _sample_path(frame: NavFrame, decisions: list[dict], args) -> list[dict]:
    points = []
    count = max(2, int(args.navigation_path_points))
    step = float(args.navigation_horizon_m) / (count - 1)
    for i in range(count):
        dist = i * step
        section_adv = int((frame.along_m + dist) // 3.0)
        section = _advance_section(frame.section, frame.direction, section_adv)
        along = (frame.along_m + dist) % 3.0
        lateral = _lateral_at(dist, decisions, args)
        x, y = _point_on_section(section, frame.direction, along, lateral)
        points.append({"x": x, "y": y, "section": section, "lateral_m": lateral, "distance_m": dist})
    return points


def _lookahead_point(path: list[dict], args) -> dict:
    target = float(args.navigation_lookahead_m)
    if not path:
        return {}
    best = path[-1]
    for point in path:
        if float(point["distance_m"]) >= target:
            best = point
            break
    return dict(best)


def _curvature_to_point(pose, target: dict) -> float:
    if not target:
        return 0.0
    dx = float(target["x"]) - float(pose.x)
    dy = float(target["y"]) - float(pose.y)
    h = math.radians(float(pose.h))
    local_y = -math.sin(h) * dx + math.cos(h) * dy
    dist2 = dx * dx + dy * dy
    if dist2 <= 1e-6:
        return 0.0
    return 2.0 * local_y / dist2


def build_navigation_debug(pose, lidar_estimate, obstacles: list[dict], args) -> dict:
    if not getattr(args, "navigation_debug", True):
        return {"enabled": False}
    frame = navigation_frame(pose, args)
    decisions = _navigation_obstacles(obstacles, frame, args)
    path = _sample_path(frame, decisions, args)
    lookahead = _lookahead_point(path, args)
    curvature = _curvature_to_point(pose, lookahead)
    lidar_conf = float(getattr(lidar_estimate, "confidence", 0.0) or 0.0)
    pose_ok = bool(getattr(lidar_estimate, "valid", False)) and lidar_conf >= float(args.navigation_min_pose_confidence)
    speed_cap = float(args.navigation_cruise_speed_mps)
    speed_reason = "cruise"
    if not pose_ok:
        speed_cap = min(speed_cap, float(args.navigation_low_confidence_speed_mps))
        speed_reason = "low_pose_confidence"
    if abs(curvature) > float(args.navigation_curvature_slow_threshold):
        speed_cap = min(speed_cap, float(args.navigation_corner_speed_mps))
        speed_reason = "curvature"
    if decisions and decisions[0]["delta_m"] < float(args.navigation_obstacle_slow_distance_m):
        speed_cap = min(speed_cap, float(args.navigation_obstacle_speed_mps))
        speed_reason = "obstacle"
    return {
        "enabled": True,
        "mode": "debug_path",
        "direction": frame.direction,
        "section": frame.section,
        "along_m": frame.along_m,
        "lateral_m": frame.lateral_m,
        "heading_error_deg": frame.heading_error_deg,
        "pose_confidence_ok": pose_ok,
        "path": path,
        "lookahead": lookahead,
        "curvature": curvature,
        "speed_cap_mps": speed_cap,
        "speed_reason": speed_reason,
        "obstacle_decisions": decisions,
    }
