"""Ackermann state-lattice navigation debug planner for the fixed WRO FE field.

This module is hardware-free. It turns the current fused pose and obstacle
memory into a vehicle-feasible route preview. Unlike the earlier lane preview,
the planner expands actual robot states from the current pose/heading with
bounded steering primitives and only uses the rounded reference loop as a
progress guide, not as the path shape.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace


OUTER_HALF_M = 1.5
INNER_HALF_M = 0.5
TRACK_WIDTH_M = OUTER_HALF_M - INNER_HALF_M
REF_LANE_M = 0.5
REF_ARC_RADIUS_M = TRACK_WIDTH_M - REF_LANE_M
STRAIGHT_LEN_M = 2.0 * INNER_HALF_M
ARC_ANGLE_RAD = math.pi / 2.0
REF_ARC_LEN_M = REF_ARC_RADIUS_M * ARC_ANGLE_RAD
REF_LAP_LENGTH_M = 4.0 * (STRAIGHT_LEN_M + REF_ARC_LEN_M)
ORDER_CLOCK = ("bottom", "left", "top", "right")
ORDER_COUNTER = ("bottom", "right", "top", "left")
OBSTACLE_RADIUS_M = 0.05
DEFAULT_ROBOT_WHEELBASE_M = 0.104
DEFAULT_ROBOT_MAX_STEER_DEG = 40.0
LANE_OUTER_TIGHT_M = 0.20
LANE_OUTER_PASS_M = 0.40
LANE_CENTER_M = 0.50
LANE_INNER_PASS_M = 0.60
LANE_INNER_TIGHT_M = 0.80
LANE_VALUES_M = {
    "outer_tight": LANE_OUTER_TIGHT_M,
    "outer_pass": LANE_OUTER_PASS_M,
    "center": LANE_CENTER_M,
    "inner_pass": LANE_INNER_PASS_M,
    "inner_tight": LANE_INNER_TIGHT_M,
}
GATE_ALONGS_M = (-0.5, 0.0, 0.5)
CORNER_LANE_NAMES = ("outer_tight", "outer_pass", "center", "inner_pass", "inner_tight")
FREE_RACE_CORNER_LANE_NAMES = ("outer_pass", "inner_tight")
FREE_RACE_CORNER_TRANSITIONS = tuple(f"{a}_to_{b}" for a in FREE_RACE_CORNER_LANE_NAMES for b in FREE_RACE_CORNER_LANE_NAMES)
DEFAULT_FREE_RACE_CORNER_RADIUS_M = 0.28
DEFAULT_FREE_RACE_CORNER_CENTER_OFFSET_M = 0.35
DEFAULT_CORNER_WAYPOINT_CONFIG = Path(__file__).with_name("corner_waypoints.json")
_CORNER_CONFIG_CACHE: dict = {"path": "", "mtime": None, "data": None, "error": ""}


@dataclass(frozen=True)
class Projection:
    direction: str
    section: str
    primitive: str
    s_m: float
    local_s_m: float
    lateral_m: float
    heading_error_deg: float
    distance_error_m: float
    nominal_heading_deg: float


@dataclass(frozen=True)
class Primitive:
    name: str
    section: str
    kind: str
    length_m: float
    start_s_m: float
    start_xy: tuple[float, float] | None = None
    end_xy: tuple[float, float] | None = None
    center_xy: tuple[float, float] | None = None
    start_angle_rad: float = 0.0
    turn_sign: float = 1.0
    heading_deg: float = 0.0


@dataclass(frozen=True)
class Node:
    x: float
    y: float
    h: float
    s_m: float
    progress_m: float
    distance_m: float
    steer_deg: float
    curvature: float
    cost: float
    parent: int


@dataclass(frozen=True)
class Waypoint:
    id: str
    x: float
    y: float
    heading_deg: float
    s_m: float
    lane_m: float
    reason: str
    kind: str = "gate"
    obstacle_id: str = ""
    speed_hint: str = "cruise"


@dataclass
class NavigationPlannerState:
    signature: tuple = field(default_factory=tuple)
    direction: str = ""
    waypoints: list[Waypoint] = field(default_factory=list)
    active_waypoint_id: str = ""
    active_waypoint_min_distance_m: float = float("inf")
    active_waypoint_last_signed_forward_m: float = 0.0
    active_waypoint_has_sample: bool = False
    active_waypoint_seen_approach: bool = False
    active_waypoint_line_armed: bool = False
    active_waypoint_selected_at_route_s_m: float = 0.0
    active_waypoint_distance_m: float = 0.0
    active_waypoint_signed_forward_m: float = 0.0
    active_waypoint_passed_reason: str = ""
    active_waypoint_capture_votes: deque = field(default_factory=deque)
    active_waypoint_arm_votes: deque = field(default_factory=deque)
    active_waypoint_pass_line_votes: deque = field(default_factory=deque)
    active_waypoint_route_votes: deque = field(default_factory=deque)
    active_waypoint_reset_reason: str = ""
    active_waypoint_reset_previous_id: str = ""
    replan_count: int = 0
    corner_observation_until: float = 0.0
    corner_observation_section: str = ""
    corner_observation_direction: str = ""
    corner_observed_sections: set[str] = field(default_factory=set)
    corner_observation_handoff_section: str = ""
    corner_observation_handoff_direction: str = ""
    corner_observation_handoff_until_t: float = 0.0
    color_confirmation_until: float = 0.0
    color_confirmation_slot: str = ""
    color_confirmation_section: str = ""
    color_confirmation_proposal: str = ""
    color_confirmation_mode: str = ""
    color_confirmation_direction: str = ""
    color_confirmation_committed_start: str = ""
    color_confirmation_start_t: float = 0.0
    color_confirmation_timeout_t: float = 0.0
    color_confirmation_samples: list[dict] = field(default_factory=list)
    color_confirmation_all_samples: list[dict] = field(default_factory=list)
    color_confirmation_last_sample_key: str = ""
    color_confirmation_seen_measurement_ids: set[str] = field(default_factory=set)
    color_confirmation_decision: dict = field(default_factory=dict)
    color_confirmation_hold_counts: dict[str, int] = field(default_factory=dict)
    color_confirmation_deadline_holds: set[str] = field(default_factory=set)
    color_confirmation_trigger_source: str = ""
    color_confirmation_deadline_key: str = ""
    color_confirmation_deadline_section: str = ""
    color_confirmation_deadline_gate_idx: int | None = None
    color_confirmation_obstacle_gate_idx: int | None = None
    color_confirmation_deadline_delta_m: float = 0.0
    color_confirmation_deadline_trigger_delta_m: float = 0.0
    color_confirmation_deadline_pre_gate_m: float = 0.0
    color_confirmation_deadline_trigger_margin_m: float = 0.0
    color_confirmation_deadline_bypass_used: bool = False
    color_confirmation_blocked_slot: str = ""
    color_confirmation_blocked_reason: str = ""
    color_confirmation_recovery_target_m: float = 0.0
    color_confirmation_recovery_trigger_distance_m: float = 0.0
    color_confirmation_recovery_reason: str = ""
    color_confirmation_attempt: int = 0
    color_confirmation_gain: float = 0.0
    color_confirmation_base_gain: float = 0.0
    color_confirmation_settle_until_t: float = 0.0
    color_confirmation_exposure_reason: str = ""
    color_confirmation_measurement_debug: dict = field(default_factory=dict)
    post_unpark_start_confirmation_direction: str = ""
    post_unpark_start_confirmation_seen_unpark: bool = False
    post_unpark_start_confirmation_consumed: bool = False
    post_unpark_start_confirmation_last_reason: str = ""
    absence_confirmation_until: float = 0.0
    absence_confirmation_slot: str = ""
    absence_confirmation_section: str = ""
    absence_confirmation_mode: str = ""
    absence_confirmation_direction: str = ""
    absence_confirmation_hold_counts: dict[str, int] = field(default_factory=dict)
    absence_confirmation_blocked_slot: str = ""
    absence_confirmation_blocked_reason: str = ""
    locked_gate_choices: dict[tuple[str, int], dict] = field(default_factory=dict)
    pending_unknown_gate_choices: dict[tuple[str, int], dict] = field(default_factory=dict)
    pending_unknown_gate_last_action: str = ""
    pending_unknown_gate_last_key: str = ""
    pending_unknown_gate_last_reason: str = ""
    fixed_route_projection_error_m: float = 0.0
    fixed_route_projected_s_m: float = 0.0
    fixed_route_projected_heading_deg: float = 0.0
    blocked_route_fallback_started_t: float = 0.0
    blocked_route_fallback_active: bool = False
    blocked_route_fallback_reason: str = ""
    adaptive_corridor_lookahead_m: float = 0.0
    adaptive_corridor_t: float = 0.0
    adaptive_yaw_rate_samples: deque = field(default_factory=lambda: deque(maxlen=3))
    route_start_s_m: float = 0.0
    route_prev_s_m: float = 0.0
    route_last_s_m: float = 0.0
    route_step_delta_m: float = 0.0
    route_prev_progress_m: float = 0.0
    route_progress_m: float = 0.0
    route_progress_initialized: bool = False
    route_progress_mutations_allowed: bool = True
    route_progress_baseline_synced: bool = False
    route_progress_suppressed_reason: str = ""
    route_progress_suppressed_delta_m: float = 0.0
    gate_lock_last_event: str = ""
    gate_lock_last_section: str = ""
    gate_lock_last_gate_idx: int = -1
    gate_lock_last_lane_name: str = ""
    gate_lock_last_reason: str = ""
    gate_lock_last_decision_reason: str = ""
    gate_lock_last_obstacle_ids: str = ""
    gate_lock_last_route_progress_m: float = 0.0
    gate_lock_last_route_step_delta_m: float = 0.0
    gate_lock_last_active_waypoint_id: str = ""
    gate_lock_last_startup_status: str = ""
    gate_lock_last_startup_state: str = ""
    gate_lock_last_anchor_version: int = -1
    gate_lock_last_anchor_source: str = ""
    gate_lock_last_anchor_jump_m: float = 0.0
    gate_lock_last_anchor_jump_h_deg: float = 0.0
    gate_lock_last_initial_pose_locked: bool = True
    gate_lock_event_count: int = 0
    gate_lock_recent_events: deque = field(default_factory=lambda: deque(maxlen=12))
    gate_lock_suppressed_reason: str = ""
    gate_lock_context_startup_status: str = ""
    gate_lock_context_startup_state: str = ""
    gate_lock_context_anchor_version: int = -1
    gate_lock_context_anchor_source: str = ""
    gate_lock_context_anchor_jump_m: float = 0.0
    gate_lock_context_anchor_jump_h_deg: float = 0.0
    gate_lock_context_initial_pose_locked: bool = True
    route_section: str = ""
    route_primitive: str = ""
    race_lap_direction: str = ""
    race_lap_completed_count: int = 0
    race_lap_prev_first_gate_passed: bool = False
    race_lap_left_start_section: bool = False
    race_lap_last_count_progress_m: float = 0.0
    free_race_lane_plan_direction: str = ""
    free_race_lane_plan_mode_key: str = ""
    free_race_lane_plan_applied: dict = field(default_factory=dict)
    free_race_lane_plan_pending: dict = field(default_factory=dict)
    free_race_lane_plan_frozen: dict = field(default_factory=dict)
    free_race_lane_plan_locked_seen: bool = False
    free_race_lane_plan_generation: int = 0
    free_race_lane_plan_last_reason: str = ""


_HYBRID_STATE = NavigationPlannerState()


def _all_gates_locked() -> bool:
    return all(
        (section, gate_idx) in _HYBRID_STATE.locked_gate_choices
        for section in ORDER_CLOCK
        for gate_idx in range(len(GATE_ALONGS_M))
    )


def _free_race_enabled(args) -> bool:
    return bool(getattr(args, "free_race", False) or getattr(args, "navigation_free_race", False))


def _blocked_route_fallback_eligible(args) -> tuple[bool, str]:
    if not bool(getattr(args, "navigation_blocked_route_fallback", True)):
        return False, "disabled"
    race_phase = str(getattr(args, "navigation_race_phase", "") or "")
    if race_phase != "navigation":
        return False, f"race_phase_{race_phase or 'empty'}"
    if hasattr(args, "navigation_deadman_active") and not bool(getattr(args, "navigation_deadman_active", False)):
        return False, "deadman_inactive"
    if bool(getattr(args, "navigation_unpark_pending", False)):
        return False, "unpark_pending"
    unpark_state = str(getattr(args, "navigation_unpark_state", "") or "")
    if unpark_state not in ("", "idle", "done"):
        return False, f"unpark_{unpark_state}"
    smart_state = str(getattr(args, "navigation_smart_parking_state", "") or "")
    if smart_state not in ("", "idle", "done", "free_race_done"):
        return False, f"smart_parking_{smart_state}"
    return True, "eligible"


def _reset_blocked_route_fallback(reason: str = "") -> None:
    _HYBRID_STATE.blocked_route_fallback_started_t = 0.0
    _HYBRID_STATE.blocked_route_fallback_active = False
    _HYBRID_STATE.blocked_route_fallback_reason = str(reason or "reset")


def _update_blocked_route_fallback_state(args) -> dict:
    enabled = bool(getattr(args, "navigation_blocked_route_fallback", True))
    delay_s = max(0.0, float(getattr(args, "navigation_blocked_route_fallback_delay_s", 1.0)))
    eligible, reason = _blocked_route_fallback_eligible(args)
    if not eligible:
        _reset_blocked_route_fallback(reason)
        return {
            "enabled": enabled,
            "eligible": False,
            "active": False,
            "age_s": 0.0,
            "delay_s": delay_s,
            "reason": reason,
        }
    now = time.monotonic()
    if _HYBRID_STATE.blocked_route_fallback_started_t <= 0.0:
        _HYBRID_STATE.blocked_route_fallback_started_t = now
    age_s = max(0.0, now - _HYBRID_STATE.blocked_route_fallback_started_t)
    active = age_s >= delay_s
    state_reason = "delay_elapsed" if active else "waiting_for_delay"
    _HYBRID_STATE.blocked_route_fallback_active = active
    _HYBRID_STATE.blocked_route_fallback_reason = state_reason
    return {
        "enabled": enabled,
        "eligible": True,
        "active": active,
        "age_s": age_s,
        "delay_s": delay_s,
        "reason": state_reason,
    }


def _free_race_adaptive_enabled(args) -> bool:
    return _free_race_enabled(args) and bool(getattr(args, "free_race_adaptive_inner_route", True))


def _free_race_inner_config(args) -> dict:
    config = getattr(args, "free_race_inner_config", {}) if args is not None else {}
    return config if isinstance(config, dict) else {}


def _free_race_lane_value(name: str, args) -> float:
    if name == "inner_pass":
        value = float(getattr(args, "free_race_inner_lane_m", LANE_INNER_PASS_M) or LANE_INNER_PASS_M)
    elif name == "outer_pass":
        value = float(getattr(args, "free_race_outer_lane_m", LANE_OUTER_PASS_M) or LANE_OUTER_PASS_M)
    else:
        value = LANE_VALUES_M.get(name, LANE_CENTER_M)
    return _clamp(value, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))


def _free_race_lane_entry(lane_name: str, args, reason: str, classification: str = "unknown") -> dict:
    lane_name = str(lane_name or "outer_pass")
    return {
        "lane_name": lane_name,
        "lane_m": _free_race_lane_value(lane_name, args),
        "reason": str(reason or "free_race_scan_lap_outer"),
        "classification": str(classification or "unknown"),
    }


def _free_race_outer_entry(args, reason: str = "free_race_scan_lap_outer", classification: str = "unknown") -> dict:
    return _free_race_lane_entry("outer_pass", args, reason, classification)


def _free_race_section_classification(sections: dict, section: str) -> str:
    section_data = sections.get(section, {}) if isinstance(sections, dict) else {}
    if not isinstance(section_data, dict):
        return "unknown"
    classification = str(section_data.get("classification", "unknown") or "unknown")
    return classification if classification in ("far", "close") else "unknown"


def _free_race_desired_entry(section: str, sections: dict, args) -> dict:
    classification = _free_race_section_classification(sections, section)
    if classification == "far":
        return _free_race_lane_entry("inner_tight", args, "free_race_far_inner_tight", classification)
    return _free_race_outer_entry(args, f"free_race_{classification}_outer", classification)


def _free_race_lane_plan_summary(plan: dict) -> str:
    return " ".join(
        f"{section}:{(plan.get(section, {}) if isinstance(plan, dict) else {}).get('lane_name', 'outer_pass')}"
        for section in ORDER_CLOCK
    )


def _free_race_lane_plan_signature(plan: dict) -> tuple:
    return tuple(
        (
            section,
            str((plan.get(section, {}) if isinstance(plan, dict) else {}).get("lane_name", "outer_pass")),
        )
        for section in ORDER_CLOCK
    )


def _free_race_reset_lane_plan(direction: str, args, mode_key: str, reason: str) -> None:
    applied = {section: _free_race_outer_entry(args) for section in ORDER_CLOCK}
    _HYBRID_STATE.free_race_lane_plan_direction = str(direction or "")
    _HYBRID_STATE.free_race_lane_plan_mode_key = str(mode_key or "")
    _HYBRID_STATE.free_race_lane_plan_applied = {section: dict(entry) for section, entry in applied.items()}
    _HYBRID_STATE.free_race_lane_plan_pending = {section: dict(entry) for section, entry in applied.items()}
    _HYBRID_STATE.free_race_lane_plan_frozen = {}
    _HYBRID_STATE.free_race_lane_plan_locked_seen = False
    _HYBRID_STATE.free_race_lane_plan_generation += 1
    _HYBRID_STATE.free_race_lane_plan_last_reason = str(reason or "reset")


def _free_race_previous_section(section: str, direction: str) -> str:
    sections = _order_sections(direction)
    if section not in sections:
        return ""
    return sections[(sections.index(section) - 1) % len(sections)]


def _free_race_apply_lane_plan_section(section: str, entry: dict, reason: str) -> bool:
    current = _HYBRID_STATE.free_race_lane_plan_applied.get(section, {})
    if (
        str(current.get("lane_name", "")) == str(entry.get("lane_name", ""))
        and abs(float(current.get("lane_m", 0.0) or 0.0) - float(entry.get("lane_m", 0.0) or 0.0)) < 1e-6
        and str(current.get("classification", "")) == str(entry.get("classification", ""))
    ):
        return False
    updated = dict(entry)
    updated["apply_reason"] = str(reason or "apply")
    _HYBRID_STATE.free_race_lane_plan_applied[section] = updated
    _HYBRID_STATE.free_race_lane_plan_generation += 1
    _HYBRID_STATE.free_race_lane_plan_last_reason = f"{section}:{reason}"
    return True


def _free_race_lap1_start_section_active(direction: str, completed_laps: int, enabled: bool, locked: bool, sections: dict) -> bool:
    # The global adaptive lock intentionally happens only at lap-1 completion.
    # For the already-known start section, allow an earlier lane switch based on
    # that section's own classification so the lap-1 return corner can transition
    # into the shortened route.
    del locked
    if not enabled or completed_laps >= 1:
        return False
    if _free_race_section_classification(sections, "bottom") != "far":
        return False
    if direction not in ("CLOCK", "COUNTER"):
        direction = str(getattr(_HYBRID_STATE, "direction", "") or getattr(_HYBRID_STATE, "race_lap_direction", "") or "CLOCK")
    if direction not in ("CLOCK", "COUNTER"):
        return False
    final_section = _order_sections(direction)[-1]
    final_starts = [
        float(prim.start_s_m)
        for prim in _primitives(direction)
        if prim.kind == "straight" and prim.section == final_section
    ]
    if not final_starts or not bool(getattr(_HYBRID_STATE, "route_progress_initialized", False)):
        return False
    route_start_s = float(getattr(_HYBRID_STATE, "route_start_s_m", 0.0) or 0.0)
    final_entry_progress = min(_forward_delta(route_start_s, final_start_s) for final_start_s in final_starts)
    progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    return progress >= max(0.0, final_entry_progress - 0.02)


def _free_race_update_lane_plan(args, direction: str = "") -> dict:
    if direction not in ("CLOCK", "COUNTER"):
        direction = str(getattr(_HYBRID_STATE, "direction", "") or getattr(_HYBRID_STATE, "race_lap_direction", "") or "CLOCK")
    config = _free_race_inner_config(args)
    completed_laps = int(getattr(_HYBRID_STATE, "race_lap_completed_count", 0) or 0)
    enabled = _free_race_adaptive_enabled(args)
    locked = bool(config.get("locked", False))
    sections = config.get("sections", {}) if isinstance(config.get("sections", {}), dict) else {}
    mode_key = str(config.get("mode_key", "") or f"free:{direction}")
    if not enabled:
        if (
            _HYBRID_STATE.free_race_lane_plan_direction != direction
            or _HYBRID_STATE.free_race_lane_plan_mode_key != mode_key
            or not _HYBRID_STATE.free_race_lane_plan_applied
            or _HYBRID_STATE.free_race_lane_plan_last_reason != "disabled"
        ):
            _free_race_reset_lane_plan(direction, args, mode_key, "disabled")
        applied = _HYBRID_STATE.free_race_lane_plan_applied
        pending = _HYBRID_STATE.free_race_lane_plan_pending
        return {
            "enabled": False,
            "locked": locked,
            "active": False,
            "start_section_active": False,
            "completed_laps": completed_laps,
            "sections": sections,
            "summary": str(config.get("summary", "")),
            "applied_active": False,
            "applied_sections": {section: dict(applied.get(section, _free_race_outer_entry(args))) for section in ORDER_CLOCK},
            "pending_sections": {section: dict(pending.get(section, _free_race_outer_entry(args))) for section in ORDER_CLOCK},
            "applied_summary": _free_race_lane_plan_summary(applied),
            "pending_summary": _free_race_lane_plan_summary(pending),
            "apply_generation": int(_HYBRID_STATE.free_race_lane_plan_generation),
            "apply_reason": str(_HYBRID_STATE.free_race_lane_plan_last_reason),
        }
    if (
        _HYBRID_STATE.free_race_lane_plan_direction != direction
        or _HYBRID_STATE.free_race_lane_plan_mode_key != mode_key
        or not _HYBRID_STATE.free_race_lane_plan_applied
    ):
        _free_race_reset_lane_plan(direction, args, mode_key, "init")

    live_pending = {section: _free_race_desired_entry(section, sections, args) for section in ORDER_CLOCK}
    if enabled and locked and completed_laps >= 1:
        if not _HYBRID_STATE.free_race_lane_plan_locked_seen:
            _HYBRID_STATE.free_race_lane_plan_locked_seen = True
            _HYBRID_STATE.free_race_lane_plan_last_reason = "lap1_lock"
    elif not locked:
        _HYBRID_STATE.free_race_lane_plan_locked_seen = False
    pending = live_pending
    _HYBRID_STATE.free_race_lane_plan_pending = {section: dict(entry) for section, entry in pending.items()}

    start_section_active = _free_race_lap1_start_section_active(direction, completed_laps, enabled, locked, sections)
    current_section = str(getattr(_HYBRID_STATE, "route_section", "") or "")
    if enabled and completed_laps < 1:
        bottom_pending = pending.get("bottom", _free_race_outer_entry(args))
        if start_section_active:
            _free_race_apply_lane_plan_section("bottom", bottom_pending, "lap1_return_corner")
    elif enabled and locked and completed_laps >= 1:
        for section in ORDER_CLOCK:
            if section == current_section:
                continue
            current = _HYBRID_STATE.free_race_lane_plan_applied.get(section, {})
            desired = pending.get(section, _free_race_outer_entry(args))
            if str(current.get("lane_name", "")) == str(desired.get("lane_name", "")):
                continue
            if _free_race_previous_section(section, direction) == current_section:
                _free_race_apply_lane_plan_section(section, desired, f"corner_entry_{current_section}_to_{section}")

    applied = _HYBRID_STATE.free_race_lane_plan_applied
    applied_active = any(str((applied.get(section, {}) or {}).get("lane_name", "outer_pass")) != "outer_pass" for section in ORDER_CLOCK)
    return {
        "enabled": enabled,
        "locked": locked,
        "active": bool(enabled and locked and completed_laps >= 1),
        "start_section_active": start_section_active,
        "completed_laps": completed_laps,
        "sections": sections,
        "summary": str(config.get("summary", "")),
        "applied_active": applied_active,
        "applied_sections": {section: dict(applied.get(section, _free_race_outer_entry(args))) for section in ORDER_CLOCK},
        "pending_sections": {section: dict(_HYBRID_STATE.free_race_lane_plan_pending.get(section, _free_race_outer_entry(args))) for section in ORDER_CLOCK},
        "applied_summary": _free_race_lane_plan_summary(applied),
        "pending_summary": _free_race_lane_plan_summary(_HYBRID_STATE.free_race_lane_plan_pending),
        "apply_generation": int(_HYBRID_STATE.free_race_lane_plan_generation),
        "apply_reason": str(_HYBRID_STATE.free_race_lane_plan_last_reason),
    }


def _free_race_adaptive_status(args, direction: str = "") -> dict:
    return _free_race_update_lane_plan(args, direction)


def _free_race_section_lane(section: str, args, direction: str = "") -> tuple[str, float, str]:
    status = _free_race_adaptive_status(args, direction)
    applied_sections = status.get("applied_sections", {})
    entry = applied_sections.get(section, {}) if isinstance(applied_sections, dict) else {}
    if not isinstance(entry, dict):
        entry = {}
    lane_name = str(entry.get("lane_name", "outer_pass") or "outer_pass")
    lane_m = float(entry.get("lane_m", _free_race_lane_value(lane_name, args)) or _free_race_lane_value(lane_name, args))
    reason = str(entry.get("reason", "free_race_scan_lap_outer") or "free_race_scan_lap_outer")
    return lane_name, lane_m, reason


def _free_race_inner_signature(args, direction: str = "") -> tuple:
    status = _free_race_adaptive_status(args, direction)
    outer_lane = round(_free_race_lane_value("outer_pass", args), 4)
    inner_lane = round(_free_race_lane_value("inner_tight", args), 4)
    return (
        bool(status["enabled"]),
        bool(status.get("active", False)),
        bool(status["locked"]),
        bool(status.get("start_section_active", False)),
        int(status.get("apply_generation", 0)),
        outer_lane,
        inner_lane,
        _free_race_lane_plan_signature(status.get("applied_sections", {})),
    )


def _parking_center_x_from_args(args) -> float:
    gap = float(getattr(args, "parking_gap_m", 0.0) or 0.0)
    if gap <= 0.0:
        gap = 1.5 * float(getattr(args, "robot_length_m", 0.1925) or 0.1925)
    center = getattr(args, "parking_center_x", None)
    if center is not None:
        return float(center)
    return float(getattr(args, "parking_right_x", 0.5) or 0.5) - gap / 2.0


def _smart_parking_trigger_status(route: Projection, args, freeze: bool = False) -> dict:
    laps = max(0.0, float(getattr(args, "smart_parking_laps", 1.0)))
    margin = max(0.0, float(getattr(args, "smart_parking_gate_pass_margin_m", 0.10)))
    progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    target_progress = laps * REF_LAP_LENGTH_M
    first_gate_along = 0.5 if route.direction == "CLOCK" else -0.5
    trigger_margin = margin
    if route.direction == "CLOCK":
        turn_radius = max(0.0, float(getattr(args, "smart_parking_turn_radius_m", 0.12) or 0.12))
        stage_extra = float(getattr(args, "smart_parking_stage_offset_m", 0.0) or 0.0)
        stage_x = _parking_center_x_from_args(args) - max(0.0, turn_radius + stage_extra)
        stage_delta = max(0.0, first_gate_along - stage_x)
        trigger_margin = min(margin, max(0.0, stage_delta * 0.5))
    enabled = bool(getattr(args, "smart_parking", True)) and not _free_race_enabled(args)
    gate_status = _update_bottom_gate_lap_counter(route, trigger_margin, enabled, freeze=freeze)
    completed_laps = int(gate_status.get("completed_laps", 0) or 0)
    lap_complete = completed_laps >= laps
    return {
        "enabled": enabled,
        "laps": laps,
        "lap_length_m": REF_LAP_LENGTH_M,
        "route_progress_m": progress,
        "target_progress_m": target_progress,
        "route_progress_lap_complete": progress >= target_progress,
        "lap_complete": lap_complete,
        **gate_status,
        "trigger": (not freeze) and enabled and lap_complete and bool(gate_status.get("first_gate_passed", False)),
    }


def _bottom_first_gate_status(route: Projection, margin: float) -> dict:
    first_gate_along = 0.5 if route.direction == "CLOCK" else -0.5
    first_gate_s = _section_route_s("bottom", route.direction, first_gate_along)
    gate_delta = _forward_delta(first_gate_s, route.s_m)
    in_start_section = route.section == "bottom"
    gate_passed = in_start_section and margin <= gate_delta <= STRAIGHT_LEN_M + margin
    return {
        "section": route.section,
        "direction": route.direction,
        "first_gate_along_m": first_gate_along,
        "first_gate_s_m": first_gate_s,
        "first_gate_delta_m": gate_delta,
        "trigger_margin_m": margin,
        "first_gate_passed": gate_passed,
    }


def _update_bottom_gate_lap_counter(route: Projection, margin: float, enabled: bool, freeze: bool = False) -> dict:
    status = _bottom_first_gate_status(route, margin)
    if freeze or not enabled:
        status.update(
            {
                "completed_laps": _HYBRID_STATE.race_lap_completed_count,
                "counted_now": False,
                "left_start_section": _HYBRID_STATE.race_lap_left_start_section,
                "last_count_progress_m": _HYBRID_STATE.race_lap_last_count_progress_m,
                "count_progress_gap_ok": False,
                "count_progress_min_gap_m": REF_LAP_LENGTH_M * 0.5,
                "frozen": bool(freeze),
            }
        )
        return status
    if _HYBRID_STATE.race_lap_direction != route.direction:
        _HYBRID_STATE.race_lap_direction = route.direction
        _HYBRID_STATE.race_lap_completed_count = 0
        _HYBRID_STATE.race_lap_prev_first_gate_passed = False
        _HYBRID_STATE.race_lap_left_start_section = False
        _HYBRID_STATE.race_lap_last_count_progress_m = 0.0
    if route.section != "bottom":
        _HYBRID_STATE.race_lap_left_start_section = True

    gate_passed = bool(status.get("first_gate_passed", False))
    progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    last_progress = float(getattr(_HYBRID_STATE, "race_lap_last_count_progress_m", 0.0) or 0.0)
    min_gap = REF_LAP_LENGTH_M * 0.5
    progress_gap_ok = (
        progress >= min_gap
        if _HYBRID_STATE.race_lap_completed_count <= 0
        else progress - last_progress >= min_gap
    )
    counted_now = False
    if (
        _HYBRID_STATE.race_lap_left_start_section
        and gate_passed
        and not _HYBRID_STATE.race_lap_prev_first_gate_passed
        and progress_gap_ok
    ):
        _HYBRID_STATE.race_lap_completed_count += 1
        _HYBRID_STATE.race_lap_last_count_progress_m = progress
        counted_now = True
    _HYBRID_STATE.race_lap_prev_first_gate_passed = gate_passed
    status.update(
        {
            "completed_laps": _HYBRID_STATE.race_lap_completed_count,
            "counted_now": counted_now,
            "left_start_section": _HYBRID_STATE.race_lap_left_start_section,
            "last_count_progress_m": _HYBRID_STATE.race_lap_last_count_progress_m,
            "count_progress_gap_ok": progress_gap_ok,
            "count_progress_min_gap_m": min_gap,
        }
    )
    return status


def _free_race_finish_status(route: Projection, args, freeze: bool = False) -> dict:
    laps = max(0.0, float(getattr(args, "smart_parking_laps", 3.0)))
    margin = max(0.0, float(getattr(args, "smart_parking_gate_pass_margin_m", 0.10)))
    progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    target_progress = laps * REF_LAP_LENGTH_M
    enabled = _free_race_enabled(args)
    gate_status = _update_bottom_gate_lap_counter(route, margin, enabled, freeze=freeze)
    completed_laps = int(gate_status.get("completed_laps", 0) or 0)
    lap_complete = completed_laps >= laps
    return {
        "enabled": enabled,
        "laps": laps,
        "lap_length_m": REF_LAP_LENGTH_M,
        "route_progress_m": progress,
        "target_progress_m": target_progress,
        "route_progress_lap_complete": progress >= target_progress,
        "lap_complete": lap_complete,
        **gate_status,
        "trigger": (not freeze) and enabled and lap_complete and bool(gate_status.get("first_gate_passed", False)),
    }


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _wrap_rad(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _clear_color_confirmation_state() -> None:
    _HYBRID_STATE.color_confirmation_until = 0.0
    _HYBRID_STATE.color_confirmation_slot = ""
    _HYBRID_STATE.color_confirmation_section = ""
    _HYBRID_STATE.color_confirmation_proposal = ""
    _HYBRID_STATE.color_confirmation_mode = ""
    _HYBRID_STATE.color_confirmation_committed_start = ""
    _HYBRID_STATE.color_confirmation_start_t = 0.0
    _HYBRID_STATE.color_confirmation_timeout_t = 0.0
    _HYBRID_STATE.color_confirmation_samples.clear()
    _HYBRID_STATE.color_confirmation_all_samples.clear()
    _HYBRID_STATE.color_confirmation_last_sample_key = ""
    _HYBRID_STATE.color_confirmation_seen_measurement_ids.clear()
    _HYBRID_STATE.color_confirmation_decision.clear()
    _HYBRID_STATE.color_confirmation_trigger_source = ""
    _HYBRID_STATE.color_confirmation_deadline_key = ""
    _HYBRID_STATE.color_confirmation_deadline_section = ""
    _HYBRID_STATE.color_confirmation_deadline_gate_idx = None
    _HYBRID_STATE.color_confirmation_obstacle_gate_idx = None
    _HYBRID_STATE.color_confirmation_deadline_delta_m = 0.0
    _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m = 0.0
    _HYBRID_STATE.color_confirmation_deadline_pre_gate_m = 0.0
    _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m = 0.0
    _HYBRID_STATE.color_confirmation_deadline_bypass_used = False
    _HYBRID_STATE.color_confirmation_blocked_slot = ""
    _HYBRID_STATE.color_confirmation_blocked_reason = ""
    _HYBRID_STATE.color_confirmation_recovery_target_m = 0.0
    _HYBRID_STATE.color_confirmation_recovery_trigger_distance_m = 0.0
    _HYBRID_STATE.color_confirmation_recovery_reason = ""
    _HYBRID_STATE.color_confirmation_attempt = 0
    _HYBRID_STATE.color_confirmation_gain = 0.0
    _HYBRID_STATE.color_confirmation_base_gain = 0.0
    _HYBRID_STATE.color_confirmation_settle_until_t = 0.0
    _HYBRID_STATE.color_confirmation_exposure_reason = ""
    _HYBRID_STATE.color_confirmation_measurement_debug.clear()


def clear_color_confirmation_block() -> None:
    _clear_color_confirmation_state()


def _color_confirmation_recovery_distance(distance_m: float, args) -> float:
    near_d = max(0.0, float(getattr(args, "navigation_close_color_recovery_near_distance_m", 0.15)))
    mid_d = max(near_d, float(getattr(args, "navigation_close_color_recovery_mid_distance_m", 0.33)))
    far_d = max(mid_d, float(getattr(args, "navigation_close_color_recovery_far_distance_m", 0.50)))
    near_r = max(0.0, float(getattr(args, "navigation_close_color_recovery_near_reverse_m", 0.33)))
    mid_r = max(0.0, float(getattr(args, "navigation_close_color_recovery_mid_reverse_m", 0.22)))
    far_r = max(0.0, float(getattr(args, "navigation_close_color_recovery_far_reverse_m", 0.0)))
    d = max(0.0, float(distance_m))
    if d <= near_d:
        return near_r
    if d <= mid_d:
        frac = (d - near_d) / max(mid_d - near_d, 1e-6)
        return near_r + (mid_r - near_r) * frac
    if d <= far_d:
        frac = (d - mid_d) / max(far_d - mid_d, 1e-6)
        return mid_r + (far_r - mid_r) * frac
    return 0.0


def _color_confirmation_route_side_changed(obs: dict, start_color: str, decided_color: str, route: Projection, args) -> tuple[bool, dict]:
    if decided_color not in ("RED", "GREEN"):
        return False, {"start_color": start_color, "decided_color": decided_color}
    start_obs = dict(obs)
    decided_obs = dict(obs)
    start_obs["color"] = start_color if start_color in ("RED", "GREEN") else ""
    decided_obs["color"] = decided_color
    start_lane, start_reason = _desired_obstacle_lane(start_obs, route.direction, args)
    decided_lane, decided_reason = _desired_obstacle_lane(decided_obs, route.direction, args)
    start_side, _ = _pass_side_by_lane(start_lane, start_obs, route.direction)
    decided_side, _ = _pass_side_by_lane(decided_lane, decided_obs, route.direction)
    lane_delta = abs(float(decided_lane) - float(start_lane))
    if start_color in ("RED", "GREEN"):
        changed = start_side != decided_side or lane_delta > 0.02
    else:
        changed = lane_delta > 0.02
    return changed, {
        "start_color": start_color,
        "decided_color": decided_color,
        "start_lane_m": start_lane,
        "decided_lane_m": decided_lane,
        "start_lane_reason": start_reason,
        "decided_lane_reason": decided_reason,
        "start_pass_side": start_side,
        "decided_pass_side": decided_side,
        "lane_delta_m": lane_delta,
    }


def _clear_absence_confirmation_state() -> None:
    _HYBRID_STATE.absence_confirmation_until = 0.0
    _HYBRID_STATE.absence_confirmation_slot = ""
    _HYBRID_STATE.absence_confirmation_section = ""
    _HYBRID_STATE.absence_confirmation_mode = ""
    _HYBRID_STATE.absence_confirmation_blocked_slot = ""
    _HYBRID_STATE.absence_confirmation_blocked_reason = ""


def _navigation_confirmations_active(args) -> bool:
    return bool(getattr(args, "navigation_confirmations_active", True))


def _navigation_confirmations_active_reason(args) -> str:
    reason = str(getattr(args, "navigation_confirmations_active_reason", "") or "")
    if reason:
        return reason
    return "active" if _navigation_confirmations_active(args) else "inactive_unknown"


def _navigation_confirmations_inactive_expected(args) -> bool:
    reason = _navigation_confirmations_active_reason(args)
    return reason in {
        "competition_wait",
        "init_not_locked",
        "free_race",
        "unpark_pending",
        "unpark_active",
        "smart_parking_active",
    }


def _unpark_navigation_state_active(args) -> bool:
    unpark_state = str(getattr(args, "navigation_unpark_state", "") or "")
    return bool(getattr(args, "navigation_unpark_pending", False)) or unpark_state not in ("", "idle", "done")


def _post_unpark_start_confirmation_status(
    route: Projection,
    obstacles: list[dict],
    args,
    source: str = "relevant",
    relevant_obstacles: list[dict] | None = None,
) -> dict:
    enabled = bool(getattr(args, "navigation_post_unpark_start_confirmation", True))
    direction = str(route.direction or "")
    obstacle_count = len(obstacles)
    relevant_count = len(relevant_obstacles) if relevant_obstacles is not None else obstacle_count

    def status(extra: dict | None = None) -> dict:
        result = {
            "enabled": enabled,
            "triggered": False,
            "source": source,
            "obstacle_count": obstacle_count,
            "relevant_obstacle_count": relevant_count,
        }
        if extra:
            result.update(extra)
        return result

    if _HYBRID_STATE.post_unpark_start_confirmation_direction != direction:
        _HYBRID_STATE.post_unpark_start_confirmation_direction = direction
        _HYBRID_STATE.post_unpark_start_confirmation_seen_unpark = False
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = False
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "direction_reset"

    unpark_state = str(getattr(args, "navigation_unpark_state", "") or "")
    race_phase = str(getattr(args, "navigation_race_phase", "") or "")
    progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    window_m = max(0.0, float(getattr(args, "navigation_post_unpark_start_confirmation_progress_m", 0.75)))
    active_unpark = _unpark_navigation_state_active(args)
    if active_unpark:
        _HYBRID_STATE.post_unpark_start_confirmation_seen_unpark = True
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = False
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = f"unpark_{unpark_state or 'pending'}"
        return status({"reason": _HYBRID_STATE.post_unpark_start_confirmation_last_reason, "window_m": window_m, "progress_m": progress})

    if bool(getattr(args, "navigation_party_mode", False)):
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "party_mode"
        return status({"enabled": False, "reason": "party_mode", "party_mode": True, "consumed": True, "window_m": window_m, "progress_m": progress})
    if not enabled:
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "disabled"
        return status({"enabled": False, "reason": "disabled", "window_m": window_m, "progress_m": progress})
    if _free_race_enabled(args):
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "free_race"
        return status({"reason": "free_race", "window_m": window_m, "progress_m": progress})
    if _all_gates_locked():
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "full_gate_lock"
        return status({"reason": "full_gate_lock", "window_m": window_m, "progress_m": progress})
    if race_phase != "navigation" or unpark_state != "done":
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = f"waiting_{race_phase or 'no_phase'}_{unpark_state or 'no_unpark'}"
        return status({"reason": _HYBRID_STATE.post_unpark_start_confirmation_last_reason, "window_m": window_m, "progress_m": progress})
    if _HYBRID_STATE.post_unpark_start_confirmation_consumed:
        return status({"reason": _HYBRID_STATE.post_unpark_start_confirmation_last_reason or "consumed", "consumed": True, "window_m": window_m, "progress_m": progress})
    if progress > window_m:
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "window_elapsed"
        return status({"reason": "window_elapsed", "consumed": True, "window_m": window_m, "progress_m": progress})
    if route.section != "bottom":
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = f"section_{route.section or 'empty'}"
        return status({"reason": _HYBRID_STATE.post_unpark_start_confirmation_last_reason, "window_m": window_m, "progress_m": progress})
    if _section_has_known_color_obstacle("bottom", obstacles):
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "already_confirmed"
        return status({"reason": "already_confirmed", "consumed": True, "window_m": window_m, "progress_m": progress})

    candidates: list[dict] = []
    for obs in obstacles:
        if str(obs.get("section", "") or "") != "bottom":
            continue
        if not (obs.get("known") or obs.get("current")):
            continue
        color = str(obs.get("color", "") or obs.get("known_color", "") or obs.get("current_color", "") or "")
        if color in ("RED", "GREEN"):
            continue
        gate_idx = _gate_index_from_along(_slot_along_m(obs))
        if ("bottom", gate_idx) in _HYBRID_STATE.locked_gate_choices:
            continue
        candidates.append(obs)
    if not candidates:
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "no_start_section_candidate"
        return status({"reason": "no_start_section_candidate", "window_m": window_m, "progress_m": progress, "candidate_count": 0})

    def candidate_delta(item: dict) -> float:
        try:
            if "s_m" in item:
                slot_s = float(item.get("s_m", 0.0) or 0.0)
            else:
                slot_s = float(_project_slot(item, route.direction).get("s_m", 0.0) or 0.0)
            delta = _forward_delta(route.s_m, slot_s)
            if delta > float(getattr(args, "navigation_horizon_m", 1.4)):
                delta = -float(REF_LAP_LENGTH_M - delta)
            return delta
        except Exception:
            return float("inf")

    obs = min(candidates, key=lambda item: max(0.0, candidate_delta(item)))
    slot_id = str(obs.get("id", ""))
    relevant_ids = {str(item.get("id", "")) for item in (relevant_obstacles or [])}
    slot_in_relevant = slot_id in relevant_ids if relevant_obstacles is not None else source == "relevant"
    return status({
        "enabled": True,
        "triggered": True,
        "reason": "needs_start_section_color",
        "slot_id": slot_id,
        "section": "bottom",
        "window_m": window_m,
        "progress_m": progress,
        "seen_unpark": bool(_HYBRID_STATE.post_unpark_start_confirmation_seen_unpark),
        "candidate_count": len(candidates),
        "slot_in_relevant": slot_in_relevant,
        "slot_missing_from_relevant": relevant_obstacles is not None and not slot_in_relevant,
    })

def _gate_decision_reason(decision: dict) -> str:
    return str(decision.get("reason", "") or "").lower()


def _gate_decision_is_provisional_unknown(decision: dict) -> bool:
    reason = _gate_decision_reason(decision)
    if "unknown" not in reason:
        return False
    if bool(decision.get("locked", False)):
        return False
    # Committed historical unknown locks are allowed to influence later laps.
    return "locked_" not in reason


def _unknown_gate_lock_should_defer(decision: dict, args) -> bool:
    _ = args
    return _gate_decision_is_provisional_unknown(decision)


def _color_confirmation_stationary(pose, args) -> tuple[bool, float, float]:
    speed = math.hypot(float(getattr(pose, "vx", 0.0) or 0.0), float(getattr(pose, "vy", 0.0) or 0.0))
    yaw_rate = abs(float(getattr(pose, "vh", 0.0) or 0.0))
    stationary = (
        speed <= float(getattr(args, "navigation_color_confirmation_stationary_max_speed_mps", 0.03))
        and yaw_rate <= float(getattr(args, "navigation_color_confirmation_stationary_max_yaw_rate_dps", 8.0))
    )
    return stationary, speed, yaw_rate


def _measurement_float(measurement: dict, key: str, default: float = 0.0) -> float:
    try:
        value = float(measurement.get(key, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _color_confirmation_measurement_sample(measurement: dict, now: float, args) -> tuple[dict | None, str]:
    measurement_id = str(measurement.get("measurement_id", "") or "")
    if not measurement_id:
        return None, "missing_id"
    if measurement_id in _HYBRID_STATE.color_confirmation_seen_measurement_ids:
        return None, "duplicate"
    if not bool(measurement.get("real_measurement", False)):
        return None, "not_real"
    request_t = _measurement_float(measurement, "request_t", 0.0)
    result_t = _measurement_float(measurement, "result_t", 0.0)
    start_t = float(_HYBRID_STATE.color_confirmation_start_t or 0.0)
    settle_t = float(_HYBRID_STATE.color_confirmation_settle_until_t or start_t)
    if request_t > 0.0 and request_t < start_t:
        return None, "before_hold"
    if result_t > 0.0 and result_t < start_t:
        return None, "before_hold"
    if request_t > 0.0 and request_t < settle_t:
        return None, "gain_settle"
    max_age_ms = max(0.0, float(getattr(args, "navigation_color_confirmation_max_measurement_age_ms", 150.0)))
    age_ms = _measurement_float(measurement, "age_ms", float("nan"))
    if max_age_ms > 0.0 and (not math.isfinite(age_ms) or abs(age_ms) > max_age_ms):
        return None, "stale_frame"
    label = str(measurement.get("label", "") or "")
    sample = {
        "t": now,
        "key": measurement_id,
        "measurement_id": measurement_id,
        "label": label if label in ("RED", "GREEN") else "OTHER",
        "confidence": float(measurement.get("confidence", 0.0) or 0.0),
        "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
        "camera_gain": float(measurement.get("camera_gain", _HYBRID_STATE.color_confirmation_gain or getattr(args, "camera_analog_gain", 14.0)) or 0.0),
        "request_t": request_t,
        "result_t": result_t,
        "age_ms": age_ms,
    }
    for key in (
        "v",
        "y_med",
        "valid_ratio",
        "dark_invalid_ratio",
        "bright_invalid_ratio",
        "n_valid",
        "n_total",
    ):
        if key in measurement:
            sample[key] = measurement.get(key)
    return sample, "added"


def _color_confirmation_counts(samples: list[dict]) -> dict[str, int]:
    return {
        "RED": sum(1 for sample in samples if sample.get("label") == "RED"),
        "GREEN": sum(1 for sample in samples if sample.get("label") == "GREEN"),
        "OTHER": sum(1 for sample in samples if sample.get("label") not in ("RED", "GREEN")),
    }


def _color_confirmation_gain_request(args, reason: str = "") -> dict:
    base_gain = float(getattr(args, "camera_analog_gain", 14.0))
    gain = float(_HYBRID_STATE.color_confirmation_gain or base_gain)
    return {
        "active": bool(_HYBRID_STATE.color_confirmation_slot),
        "gain": gain,
        "base_gain": base_gain,
        "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
        "reason": reason or _HYBRID_STATE.color_confirmation_exposure_reason,
        "restore": False,
    }


def _color_confirmation_restore_gain_request(args, reason: str = "restore") -> dict:
    return {
        "active": False,
        "gain": float(getattr(args, "camera_analog_gain", 14.0)),
        "base_gain": float(getattr(args, "camera_analog_gain", 14.0)),
        "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
        "reason": reason,
        "restore": True,
    }


def _sample_float(sample: dict, key: str) -> float:
    try:
        return float(sample.get(key, float("nan")))
    except (TypeError, ValueError):
        return float("nan")


def _color_confirmation_exposure_retry(samples: list[dict], args) -> dict:
    current_gain = float(_HYBRID_STATE.color_confirmation_gain or getattr(args, "camera_analog_gain", 14.0))
    gain_min = float(getattr(args, "camera_analog_gain_min", 1.0))
    gain_max = float(getattr(args, "camera_analog_gain_max", 16.0))
    sample_count = len(samples)
    counts = _color_confirmation_counts(samples)
    color_votes = counts["RED"] + counts["GREEN"]
    color_ratio = color_votes / max(sample_count, 1)
    timeout_min_color_votes = max(0, int(getattr(args, "navigation_color_confirmation_timeout_min_color_votes", 4)))
    min_ratio = float(getattr(args, "navigation_color_confirmation_exposure_min_color_ratio", 0.10))
    min_samples = max(1, int(getattr(args, "navigation_color_confirmation_exposure_min_samples", 6)))
    bright_ratio_threshold = float(getattr(args, "navigation_color_confirmation_exposure_bright_invalid_ratio", 0.45))
    dark_ratio_threshold = float(getattr(args, "navigation_color_confirmation_exposure_dark_invalid_ratio", 0.45))
    bright_v_threshold = float(getattr(args, "navigation_color_confirmation_exposure_bright_v", 245.0))
    dark_v_threshold = float(getattr(args, "navigation_color_confirmation_exposure_dark_v", 45.0))
    low_valid_threshold = float(getattr(args, "navigation_color_confirmation_exposure_low_valid_ratio", 0.10))
    required_votes = max(1, int(math.floor(0.5 * sample_count)) + 1)
    details = {
        "retry": False,
        "reason": "",
        "next_gain": current_gain,
        "current_gain": current_gain,
        "sample_count": sample_count,
        "min_samples": min_samples,
        "counts": counts,
        "color_vote_count": color_votes,
        "color_ratio": color_ratio,
        "min_color_ratio": min_ratio,
        "timeout_min_color_votes": timeout_min_color_votes,
        "required_vote_count": required_votes,
        "bright_vote_count": 0,
        "dark_vote_count": 0,
        "low_valid_count": 0,
        "bright_invalid_ratio_threshold": bright_ratio_threshold,
        "dark_invalid_ratio_threshold": dark_ratio_threshold,
        "bright_v_threshold": bright_v_threshold,
        "dark_v_threshold": dark_v_threshold,
        "low_valid_ratio_threshold": low_valid_threshold,
    }
    if not samples:
        details["reason"] = "no_samples"
        return details
    if sample_count < min_samples:
        details["reason"] = "insufficient_samples"
        return details
    if color_ratio >= min_ratio and color_votes >= timeout_min_color_votes:
        details["reason"] = "enough_color_votes"
        return details

    bright_votes = 0
    dark_votes = 0
    low_valid_votes = 0
    for sample in samples:
        v_med = _sample_float(sample, "v")
        y_med = _sample_float(sample, "y_med")
        bright_ratio = _sample_float(sample, "bright_invalid_ratio")
        dark_ratio = _sample_float(sample, "dark_invalid_ratio")
        valid_ratio = _sample_float(sample, "valid_ratio")
        if math.isfinite(valid_ratio) and valid_ratio < low_valid_threshold:
            low_valid_votes += 1
        if (
            (math.isfinite(v_med) and v_med >= bright_v_threshold)
            or (math.isfinite(y_med) and y_med >= bright_v_threshold)
            or (math.isfinite(bright_ratio) and bright_ratio >= bright_ratio_threshold)
        ):
            bright_votes += 1
        if (
            (math.isfinite(v_med) and v_med <= dark_v_threshold)
            or (math.isfinite(y_med) and y_med <= dark_v_threshold)
            or (math.isfinite(dark_ratio) and dark_ratio >= dark_ratio_threshold)
        ):
            dark_votes += 1
    details.update(
        {
            "bright_vote_count": bright_votes,
            "dark_vote_count": dark_votes,
            "low_valid_count": low_valid_votes,
        }
    )
    if bright_votes >= required_votes and bright_votes > dark_votes:
        factor = float(getattr(args, "navigation_color_confirmation_gain_down_factor", 0.50))
        details["retry"] = True
        details["reason"] = "overexposed"
        details["next_gain"] = _clamp(current_gain * factor, gain_min, gain_max)
        return details
    if dark_votes >= required_votes and dark_votes > bright_votes:
        factor = float(getattr(args, "navigation_color_confirmation_gain_up_factor", 1.50))
        details["retry"] = True
        details["reason"] = "underexposed"
        details["next_gain"] = _clamp(current_gain * factor, gain_min, gain_max)
        return details
    if bright_votes >= required_votes and dark_votes >= required_votes:
        details["reason"] = "exposure_ambiguous"
    else:
        details["reason"] = "not_exposure_limited"
    return details

def _color_confirmation_decision(samples: list[dict], args, timed_out: bool) -> dict:
    initial_samples = max(1, int(getattr(args, "navigation_color_confirmation_initial_samples", 10)))
    rolling_window = max(1, int(getattr(args, "navigation_color_confirmation_rolling_window", 20)))
    rolling_required = max(1, int(getattr(args, "navigation_color_confirmation_rolling_required", 12)))
    if len(samples) >= initial_samples:
        first = samples[:initial_samples]
        counts = _color_confirmation_counts(first)
        initial_required = max(1, int(math.floor(initial_samples * 0.5)) + 1)
        if counts["RED"] >= initial_required and counts["RED"] > counts["GREEN"]:
            return {"color": "RED", "source": "initial_window", "confidence": counts["RED"] / max(initial_samples, 1), "counts": counts}
        if counts["GREEN"] >= initial_required and counts["GREEN"] > counts["RED"]:
            return {"color": "GREEN", "source": "initial_window", "confidence": counts["GREEN"] / max(initial_samples, 1), "counts": counts}
    if len(samples) >= rolling_window:
        recent = samples[-rolling_window:]
        counts = _color_confirmation_counts(recent)
        if counts["RED"] >= rolling_required and counts["RED"] > counts["GREEN"]:
            return {"color": "RED", "source": "rolling_window", "confidence": counts["RED"] / max(rolling_window, 1), "counts": counts}
        if counts["GREEN"] >= rolling_required and counts["GREEN"] > counts["RED"]:
            return {"color": "GREEN", "source": "rolling_window", "confidence": counts["GREEN"] / max(rolling_window, 1), "counts": counts}
    if timed_out:
        counts = _color_confirmation_counts(samples)
        color_votes = counts["RED"] + counts["GREEN"]
        timeout_min_color_votes = max(0, int(getattr(args, "navigation_color_confirmation_timeout_min_color_votes", 4)))
        if color_votes >= timeout_min_color_votes and color_votes > 0 and counts["RED"] > counts["GREEN"]:
            return {"color": "RED", "source": "timeout_majority", "confidence": counts["RED"] / max(color_votes, 1), "counts": counts}
        if color_votes >= timeout_min_color_votes and color_votes > 0 and counts["GREEN"] > counts["RED"]:
            return {"color": "GREEN", "source": "timeout_majority", "confidence": counts["GREEN"] / max(color_votes, 1), "counts": counts}
        return {"color": "GREEN", "source": "timeout_fallback_green", "confidence": 0.10, "counts": counts}
    return {}


def _direction_from_pose(pose, args) -> str:
    requested = getattr(args, "navigation_direction", "auto")
    if requested == "clock":
        return "CLOCK"
    if requested == "counter":
        return "COUNTER"
    locked = getattr(args, "navigation_locked_direction", "")
    if locked == "CLOCK":
        return "CLOCK"
    if locked == "COUNTER":
        return "COUNTER"
    return "CLOCK" if math.cos(math.radians(float(pose.h))) < 0.0 else "COUNTER"


def _forward_delta(from_s: float, to_s: float) -> float:
    return (to_s - from_s) % REF_LAP_LENGTH_M


def _angle_norm(rad: float) -> float:
    return rad % (2.0 * math.pi)


def _angle_delta_signed(start: float, angle: float, sign: float) -> float:
    if sign >= 0.0:
        return _angle_norm(angle - start)
    return _angle_norm(start - angle)


def _arc_heading_deg(angle_rad: float, sign: float) -> float:
    if sign >= 0.0:
        return math.degrees(angle_rad + math.pi / 2.0)
    return math.degrees(angle_rad - math.pi / 2.0)


def _with_starts(specs: tuple[Primitive, ...]) -> list[Primitive]:
    out: list[Primitive] = []
    start_s = 0.0
    for prim in specs:
        out.append(
            Primitive(
                name=prim.name,
                section=prim.section,
                kind=prim.kind,
                length_m=prim.length_m,
                start_s_m=start_s,
                start_xy=prim.start_xy,
                end_xy=prim.end_xy,
                center_xy=prim.center_xy,
                start_angle_rad=prim.start_angle_rad,
                turn_sign=prim.turn_sign,
                heading_deg=prim.heading_deg,
            )
        )
        start_s += prim.length_m
    return out


_CLOCK_PRIMITIVES = _with_starts(
    (
        Primitive("bottom_straight", "bottom", "straight", STRAIGHT_LEN_M, 0.0, (INNER_HALF_M, -OUTER_HALF_M), (-INNER_HALF_M, -OUTER_HALF_M), heading_deg=180.0),
        Primitive("bottom_left_arc", "bottom", "arc", REF_ARC_LEN_M, 0.0, center_xy=(-INNER_HALF_M, -INNER_HALF_M), start_angle_rad=-math.pi / 2.0, turn_sign=-1.0),
        Primitive("left_straight", "left", "straight", STRAIGHT_LEN_M, 0.0, (-OUTER_HALF_M, -INNER_HALF_M), (-OUTER_HALF_M, INNER_HALF_M), heading_deg=90.0),
        Primitive("top_left_arc", "left", "arc", REF_ARC_LEN_M, 0.0, center_xy=(-INNER_HALF_M, INNER_HALF_M), start_angle_rad=math.pi, turn_sign=-1.0),
        Primitive("top_straight", "top", "straight", STRAIGHT_LEN_M, 0.0, (-INNER_HALF_M, OUTER_HALF_M), (INNER_HALF_M, OUTER_HALF_M), heading_deg=0.0),
        Primitive("top_right_arc", "top", "arc", REF_ARC_LEN_M, 0.0, center_xy=(INNER_HALF_M, INNER_HALF_M), start_angle_rad=math.pi / 2.0, turn_sign=-1.0),
        Primitive("right_straight", "right", "straight", STRAIGHT_LEN_M, 0.0, (OUTER_HALF_M, INNER_HALF_M), (OUTER_HALF_M, -INNER_HALF_M), heading_deg=-90.0),
        Primitive("bottom_right_arc", "right", "arc", REF_ARC_LEN_M, 0.0, center_xy=(INNER_HALF_M, -INNER_HALF_M), start_angle_rad=0.0, turn_sign=-1.0),
    )
)
_COUNTER_PRIMITIVES = _with_starts(
    (
        Primitive("bottom_straight", "bottom", "straight", STRAIGHT_LEN_M, 0.0, (-INNER_HALF_M, -OUTER_HALF_M), (INNER_HALF_M, -OUTER_HALF_M), heading_deg=0.0),
        Primitive("bottom_right_arc", "bottom", "arc", REF_ARC_LEN_M, 0.0, center_xy=(INNER_HALF_M, -INNER_HALF_M), start_angle_rad=-math.pi / 2.0, turn_sign=1.0),
        Primitive("right_straight", "right", "straight", STRAIGHT_LEN_M, 0.0, (OUTER_HALF_M, -INNER_HALF_M), (OUTER_HALF_M, INNER_HALF_M), heading_deg=90.0),
        Primitive("top_right_arc", "right", "arc", REF_ARC_LEN_M, 0.0, center_xy=(INNER_HALF_M, INNER_HALF_M), start_angle_rad=0.0, turn_sign=1.0),
        Primitive("top_straight", "top", "straight", STRAIGHT_LEN_M, 0.0, (INNER_HALF_M, OUTER_HALF_M), (-INNER_HALF_M, OUTER_HALF_M), heading_deg=180.0),
        Primitive("top_left_arc", "top", "arc", REF_ARC_LEN_M, 0.0, center_xy=(-INNER_HALF_M, INNER_HALF_M), start_angle_rad=math.pi / 2.0, turn_sign=1.0),
        Primitive("left_straight", "left", "straight", STRAIGHT_LEN_M, 0.0, (-OUTER_HALF_M, INNER_HALF_M), (-OUTER_HALF_M, -INNER_HALF_M), heading_deg=-90.0),
        Primitive("bottom_left_arc", "left", "arc", REF_ARC_LEN_M, 0.0, center_xy=(-INNER_HALF_M, -INNER_HALF_M), start_angle_rad=math.pi, turn_sign=1.0),
    )
)


def _primitives(direction: str) -> list[Primitive]:
    return _CLOCK_PRIMITIVES if direction == "CLOCK" else _COUNTER_PRIMITIVES


def _primitive_by_name(direction: str, name: str) -> Primitive | None:
    for prim in _primitives(direction):
        if prim.name == name:
            return prim
    return None


def _point_on_primitive(prim: Primitive, local_s: float, lane_m: float) -> tuple[float, float, float]:
    local_s = _clamp(local_s, 0.0, prim.length_m)
    lane_m = _clamp(lane_m, 0.0, TRACK_WIDTH_M)
    if prim.kind == "straight":
        sx, sy = prim.start_xy or (0.0, 0.0)
        ex, ey = prim.end_xy or (0.0, 0.0)
        u = local_s / max(prim.length_m, 1e-9)
        ox, oy = sx + (ex - sx) * u, sy + (ey - sy) * u
        if prim.section == "bottom":
            return ox, -OUTER_HALF_M + lane_m, prim.heading_deg
        if prim.section == "top":
            return ox, OUTER_HALF_M - lane_m, prim.heading_deg
        if prim.section == "left":
            return -OUTER_HALF_M + lane_m, oy, prim.heading_deg
        return OUTER_HALF_M - lane_m, oy, prim.heading_deg
    cx, cy = prim.center_xy or (0.0, 0.0)
    u = local_s / max(prim.length_m, 1e-9)
    angle = prim.start_angle_rad + prim.turn_sign * ARC_ANGLE_RAD * u
    radius = TRACK_WIDTH_M - lane_m
    x = cx + radius * math.cos(angle)
    y = cy + radius * math.sin(angle)
    return x, y, _arc_heading_deg(angle, prim.turn_sign)


def _point_on_route_s(s_m: float, lane_m: float, direction: str) -> tuple[float, float, str, str, float, float]:
    s_m %= REF_LAP_LENGTH_M
    prims = _primitives(direction)
    for idx, prim in enumerate(prims):
        end_s = prim.start_s_m + prim.length_m
        if s_m <= end_s or idx == len(prims) - 1:
            local_s = _clamp(s_m - prim.start_s_m, 0.0, prim.length_m)
            x, y, heading = _point_on_primitive(prim, local_s, lane_m)
            return x, y, prim.section, prim.name, local_s, heading
    prim = prims[-1]
    x, y, heading = _point_on_primitive(prim, prim.length_m, lane_m)
    return x, y, prim.section, prim.name, prim.length_m, heading


def _project_to_direction(x: float, y: float, heading_deg: float, direction: str) -> Projection:
    best: Projection | None = None
    best_score = float("inf")
    for prim in _primitives(direction):
        if prim.kind == "straight":
            sx, sy = prim.start_xy or (0.0, 0.0)
            ex, ey = prim.end_xy or (0.0, 0.0)
            dx, dy = ex - sx, ey - sy
            length2 = max(dx * dx + dy * dy, 1e-9)
            u = _clamp(((x - sx) * dx + (y - sy) * dy) / length2, 0.0, 1.0)
            local_s = prim.length_m * u
            if prim.section == "bottom":
                lane = y + OUTER_HALF_M
                px, py = sx + dx * u, -OUTER_HALF_M + lane
            elif prim.section == "top":
                lane = OUTER_HALF_M - y
                px, py = sx + dx * u, OUTER_HALF_M - lane
            elif prim.section == "left":
                lane = x + OUTER_HALF_M
                px, py = -OUTER_HALF_M + lane, sy + dy * u
            else:
                lane = OUTER_HALF_M - x
                px, py = OUTER_HALF_M - lane, sy + dy * u
            nominal_heading = prim.heading_deg
        else:
            cx, cy = prim.center_xy or (0.0, 0.0)
            vx, vy = x - cx, y - cy
            radius = math.hypot(vx, vy)
            angle = math.atan2(vy, vx)
            angle_delta = _clamp(_angle_delta_signed(prim.start_angle_rad, angle, prim.turn_sign), 0.0, ARC_ANGLE_RAD)
            local_s = (angle_delta / ARC_ANGLE_RAD) * prim.length_m
            fit_angle = prim.start_angle_rad + prim.turn_sign * angle_delta
            lane = TRACK_WIDTH_M - radius
            px, py = cx + radius * math.cos(fit_angle), cy + radius * math.sin(fit_angle)
            nominal_heading = _arc_heading_deg(fit_angle, prim.turn_sign)
        dist = math.hypot(x - px, y - py)
        lane_penalty = 0.0 if -0.25 <= lane <= 1.25 else abs(lane - _clamp(lane, -0.25, 1.25))
        score = dist + 0.35 * lane_penalty
        projection = Projection(
            direction=direction,
            section=prim.section,
            primitive=prim.name,
            s_m=(prim.start_s_m + local_s) % REF_LAP_LENGTH_M,
            local_s_m=local_s,
            lateral_m=lane,
            heading_error_deg=_wrap_deg(heading_deg - nominal_heading),
            distance_error_m=score,
            nominal_heading_deg=nominal_heading,
        )
        if score < best_score:
            best_score = score
            best = projection
    assert best is not None
    return best


def _project_pose(pose, args) -> Projection:
    direction = _direction_from_pose(pose, args)
    return _project_to_direction(float(pose.x), float(pose.y), float(pose.h), direction)


def project_pose_for_navigation(pose, args) -> Projection:
    return _project_pose(pose, args)


def _field_clearance(x: float, y: float) -> tuple[bool, float, float]:
    outer_clearance = OUTER_HALF_M - max(abs(x), abs(y))
    dx_inner = abs(x) - INNER_HALF_M
    dy_inner = abs(y) - INNER_HALF_M
    if dx_inner <= 0.0 and dy_inner <= 0.0:
        inner_clearance = -min(-dx_inner, -dy_inner)
    elif dx_inner <= 0.0:
        inner_clearance = dy_inner
    elif dy_inner <= 0.0:
        inner_clearance = dx_inner
    else:
        inner_clearance = math.hypot(dx_inner, dy_inner)
    return outer_clearance >= 0.0 and inner_clearance >= 0.0, outer_clearance, inner_clearance


def _field_cost(x: float, y: float, args) -> tuple[float, bool]:
    inside_track, outer_clearance, inner_clearance = _field_clearance(x, y)
    required = float(args.navigation_wall_clearance_m)
    if not inside_track or outer_clearance < required or inner_clearance < required:
        return 1_000_000.0, False
    min_clearance = min(outer_clearance, inner_clearance)
    soft = 0.0
    if min_clearance < required + 0.10:
        soft += 12.0 * (required + 0.10 - min_clearance) ** 2
    return soft, True


def _waypoint_debug_dict(wp: Waypoint | None) -> dict:
    if wp is None:
        return {}
    return {
        "id": wp.id,
        "reason": wp.reason,
        "kind": wp.kind,
        "obstacle_id": wp.obstacle_id,
        "x": wp.x,
        "y": wp.y,
        "h": wp.heading_deg,
        "s_m": wp.s_m,
        "lane_m": wp.lane_m,
    }


def _project_slot(slot: dict, direction: str) -> dict:
    x, y = float(slot["x"]), float(slot["y"])
    proj = _project_to_direction(x, y, 0.0, direction)
    color = slot.get("known_color") or slot.get("current_color") or slot.get("color") or ""
    proposed_color = slot.get("proposed_color") or slot.get("color_debug", {}).get("proposed_color", "") or ""
    color_measurements = slot.get("color_measurements", [])
    if not isinstance(color_measurements, list):
        color_measurements = []
    return {
        "id": slot.get("id", ""),
        "section": slot.get("section", proj.section),
        "x": x,
        "y": y,
        "s_m": proj.s_m,
        "delta_m": 0.0,
        "row_m": proj.lateral_m,
        "primitive": proj.primitive,
        "color": color,
        "known_color": slot.get("known_color", ""),
        "current_color": slot.get("current_color", ""),
        "proposed_color": proposed_color,
        "color_confidence": float(slot.get("known_color_confidence", 0.0) or 0.0),
        "confidence": float(slot.get("confidence", 0.0) or 0.0),
        "recent_votes": int(slot.get("recent_votes", 0) or 0),
        "raw_color": slot.get("raw_color", ""),
        "raw_color_confidence": float(slot.get("raw_color_confidence", 0.0) or 0.0),
        "known_color_confidence": float(slot.get("known_color_confidence", 0.0) or 0.0),
        "color_trusted": bool(slot.get("color_trusted", False)),
        "color_trust_reason": slot.get("color_trust_reason", ""),
        "color_debug": dict(slot.get("color_debug", {}) or {}),
        "color_measurements": [dict(item) for item in color_measurements if isinstance(item, dict)],
        "current": bool(slot.get("current")),
        "known": bool(slot.get("known")),
        "absence_clear_pending": bool(slot.get("absence_clear_pending")),
        "absence_votes": int(slot.get("absence_votes", 0) or 0),
    }


def _relevant_obstacles(obstacles: list[dict], route: Projection, args) -> list[dict]:
    if _all_gates_locked():
        return []
    relevant = []
    horizon = float(args.navigation_horizon_m)
    behind = float(args.navigation_obstacle_behind_m)
    for slot in obstacles:
        if not (slot.get("known") or slot.get("current")):
            continue
        if slot.get("section") not in ORDER_CLOCK:
            continue
        if not _start_section_obstacle_slot_allowed(slot, route.direction, args):
            continue
        obs = _project_slot(slot, route.direction)
        delta = _forward_delta(route.s_m, obs["s_m"])
        if delta > horizon and (REF_LAP_LENGTH_M - delta) > behind:
            continue
        if delta > horizon:
            delta = -float(REF_LAP_LENGTH_M - delta)
        obs["delta_m"] = delta
        relevant.append(obs)
    relevant.sort(key=lambda item: item["delta_m"])
    return relevant


def _pass_side_by_lane(lane_m: float, obstacle: dict, direction: str) -> tuple[str, float]:
    lateral_gap = lane_m - float(obstacle["row_m"])
    color = obstacle.get("color", "")
    if color == "RED":
        # Red must be passed on the robot's right side. With this lane frame,
        # right is inner for clockwise and outer for counterclockwise driving.
        if direction == "CLOCK":
            return "right_inner", lateral_gap
        return "right_outer", -lateral_gap
    if color == "GREEN":
        # Green must be passed on the robot's left side: outer for clockwise,
        # inner for counterclockwise.
        if direction == "CLOCK":
            return "left_outer", -lateral_gap
        return "left_inner", lateral_gap
    return "unknown", abs(lateral_gap)


def _desired_obstacle_lane(obstacle: dict, direction: str, args) -> tuple[float, str]:
    color = obstacle.get("color", "")
    row = float(obstacle["row_m"])
    clearance = float(args.navigation_obstacle_clearance_m)
    if color == "RED":
        # Red: pass on the right side of the obstacle. Right-of-travel is
        # inner for clockwise and outer for counterclockwise.
        lane = row + clearance if direction == "CLOCK" else row - clearance
        reason = "red"
    elif color == "GREEN":
        # Green: pass on the left side of the obstacle.
        lane = row - clearance if direction == "CLOCK" else row + clearance
        reason = "green"
    elif row <= float(args.navigation_base_lane_m):
        lane = float(args.navigation_far_lane_m)
        reason = "unknown_near"
    else:
        lane = float(args.navigation_close_lane_m)
        reason = "unknown_far"
    lane = _clamp(lane, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))
    return lane, reason


def _obstacle_cost(
    x: float,
    y: float,
    progress_m: float,
    lane_m: float,
    obstacles: list[dict],
    direction: str,
    args,
    debug_details: list[dict] | None = None,
) -> tuple[float, bool, list[str]]:
    cost = 0.0
    warnings: list[str] = []
    clearance = float(args.navigation_obstacle_clearance_m)
    influence = float(args.navigation_obstacle_influence_m)
    side_gate = max(0.08, clearance * 0.75)
    hard_clearance = clearance
    for obs in obstacles:
        ds = abs(float(obs["delta_m"]) - progress_m)
        if ds > influence + 0.25:
            continue
        dist = math.hypot(x - float(obs["x"]), y - float(obs["y"]))
        if dist < hard_clearance:
            warnings.append(f"{obs['id']}:clearance")
            if debug_details is not None:
                side, side_gap = _pass_side_by_lane(lane_m, obs, direction)
                debug_details.append(
                    {
                        "reason": "clearance",
                        "obstacle_id": str(obs.get("id", "")),
                        "distance_m": dist,
                        "required_clearance_m": hard_clearance,
                        "progress_delta_m": ds,
                        "obstacle_delta_m": float(obs.get("delta_m", 0.0) or 0.0),
                        "point_progress_m": progress_m,
                        "point_lane_m": lane_m,
                        "obstacle_lane_m": float(obs.get("row_m", 0.0) or 0.0),
                        "side": side,
                        "side_gap_m": side_gap,
                    }
                )
            return 1_000_000.0, False, warnings
        if ds < influence:
            along_weight = 0.5 + 0.5 * math.cos(math.pi * ds / max(influence, 1e-6))
            cost += 2.8 * along_weight / max(dist - OBSTACLE_RADIUS_M, 0.03)
            side, side_gap = _pass_side_by_lane(lane_m, obs, direction)
            if side != "unknown" and side_gap < clearance:
                if ds <= side_gate:
                    warnings.append(f"{obs['id']}:wrong_side")
                    if debug_details is not None:
                        debug_details.append(
                            {
                                "reason": "wrong_side",
                                "obstacle_id": str(obs.get("id", "")),
                                "distance_m": dist,
                                "required_clearance_m": clearance,
                                "progress_delta_m": ds,
                                "side_gate_m": side_gate,
                                "obstacle_delta_m": float(obs.get("delta_m", 0.0) or 0.0),
                                "point_progress_m": progress_m,
                                "point_lane_m": lane_m,
                                "obstacle_lane_m": float(obs.get("row_m", 0.0) or 0.0),
                                "side": side,
                                "side_gap_m": side_gap,
                            }
                        )
                    return 1_000_000.0, False, warnings
                cost += 45.0 * along_weight * (clearance - side_gap) ** 2
                warnings.append(f"{obs['id']}:pass_side")
            elif side == "unknown":
                cost += 2.0 * along_weight
    return cost, True, warnings


def _obstacle_signature(obstacles: list[dict], direction: str, args) -> tuple:
    locked_sections = {
        section
        for section in ORDER_CLOCK
        if all((section, gate_idx) in _HYBRID_STATE.locked_gate_choices for gate_idx in range(len(GATE_ALONGS_M)))
    }
    items = []
    for slot in obstacles:
        if not (slot.get("known") or slot.get("current")):
            continue
        section = str(slot.get("section", ""))
        if section not in ORDER_CLOCK or section in locked_sections:
            continue
        if not _start_section_obstacle_slot_allowed(slot, direction, args):
            continue
        color = slot.get("known_color") or slot.get("current_color") or slot.get("color") or ""
        items.append(
            (
                str(slot.get("id", "")),
                section,
                round(float(slot.get("x", 0.0)), 3),
                round(float(slot.get("y", 0.0)), 3),
                color,
                bool(slot.get("known")),
                bool(slot.get("current")),
            )
        )
    return (direction, tuple(sorted(items)))


def _locked_gate_signature() -> tuple:
    return tuple(
        sorted(
            (
                section,
                gate_idx,
                str(data.get("lane_name", "")),
                round(float(data.get("lane_m", 0.0) or 0.0), 4),
                str(data.get("obstacle_id", "")),
            )
            for (section, gate_idx), data in _HYBRID_STATE.locked_gate_choices.items()
        )
    )


def _waypoint_dict(wp: Waypoint) -> dict:
    return {
        "id": wp.id,
        "x": wp.x,
        "y": wp.y,
        "heading_deg": wp.heading_deg,
        "s_m": wp.s_m,
        "lane_m": wp.lane_m,
        "reason": wp.reason,
        "kind": wp.kind,
        "obstacle_id": wp.obstacle_id,
        "speed_hint": wp.speed_hint,
    }


def _make_waypoint(
    s_m: float,
    lane_m: float,
    direction: str,
    reason: str,
    obstacle_id: str = "",
    speed_hint: str = "cruise",
    waypoint_id: str = "",
    kind: str = "gate",
) -> Waypoint:
    x, y, _section, _primitive, _local_s, heading = _point_on_route_s(s_m, lane_m, direction)
    return Waypoint(
        id=waypoint_id or f"{reason}_{s_m % REF_LAP_LENGTH_M:.3f}_{lane_m:.2f}",
        x=x,
        y=y,
        heading_deg=_wrap_deg(heading),
        s_m=s_m % REF_LAP_LENGTH_M,
        lane_m=lane_m,
        reason=reason,
        kind=kind,
        obstacle_id=obstacle_id,
        speed_hint=speed_hint,
    )


def _make_explicit_waypoint(
    x: float,
    y: float,
    heading_deg: float,
    s_m: float,
    lane_m: float,
    reason: str,
    waypoint_id: str,
    kind: str,
    obstacle_id: str = "",
    speed_hint: str = "cruise",
) -> Waypoint:
    return Waypoint(
        id=waypoint_id,
        x=x,
        y=y,
        heading_deg=_wrap_deg(heading_deg),
        s_m=s_m % REF_LAP_LENGTH_M,
        lane_m=lane_m,
        reason=reason,
        kind=kind,
        obstacle_id=obstacle_id,
        speed_hint=speed_hint,
    )


def _bearing_deg(a: tuple[float, float], b: tuple[float, float], fallback_deg: float = 0.0) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if abs(dx) + abs(dy) <= 1e-9:
        return _wrap_deg(fallback_deg)
    return _wrap_deg(math.degrees(math.atan2(dy, dx)))


def _clamp_heading_between(start_deg: float, end_deg: float, heading_deg: float) -> float:
    turn = _wrap_deg(end_deg - start_deg)
    rel = _wrap_deg(heading_deg - start_deg)
    if turn >= 0.0:
        rel = _clamp(rel, 0.0, turn)
    else:
        rel = -_clamp(-rel, 0.0, -turn)
    return _wrap_deg(start_deg + rel)


def _section_route_s(section: str, direction: str, along_m: float) -> float:
    prims = _primitives(direction)
    for prim in prims:
        if prim.kind != "straight" or prim.section != section:
            continue
        sx, sy = prim.start_xy or (0.0, 0.0)
        ex, ey = prim.end_xy or (0.0, 0.0)
        dx, dy = ex - sx, ey - sy
        length2 = max(dx * dx + dy * dy, 1e-9)
        if section in ("bottom", "top"):
            px, py = along_m, sy
        else:
            px, py = sx, along_m
        u = _clamp(((px - sx) * dx + (py - sy) * dy) / length2, 0.0, 1.0)
        return (prim.start_s_m + prim.length_m * u) % REF_LAP_LENGTH_M
    return 0.0


def _slot_along_m(slot: dict) -> float:
    section = str(slot.get("section", ""))
    if section in ("bottom", "top"):
        return float(slot["x"])
    return float(slot["y"])


def _canonical_lane_name(lane_m: float) -> str:
    return min(LANE_VALUES_M, key=lambda name: abs(LANE_VALUES_M[name] - lane_m))


def _lane_value(name: str, args) -> float:
    value = LANE_VALUES_M.get(name, LANE_CENTER_M)
    return _clamp(value, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))


def _normalize_start_section_lane(section: str, lane_name: str, lane_m: float) -> tuple[str, float, bool]:
    lane_name = str(lane_name or "center")
    try:
        lane = float(lane_m)
    except (TypeError, ValueError):
        lane = LANE_VALUES_M.get(lane_name, LANE_CENTER_M)
    if str(section or "") != "bottom":
        return lane_name, lane, False
    min_start_lane = LANE_OUTER_PASS_M
    if lane_name == "outer_tight" or lane < min_start_lane - 1e-6:
        return "outer_pass", min_start_lane, True
    return lane_name, lane, False


def _append_gate_warning(decision: dict, warning: str) -> None:
    current = str(decision.get("warning", "") or "")
    if not current:
        decision["warning"] = warning
    elif warning not in current.split("|"):
        decision["warning"] = f"{current}|{warning}"


def _normalize_start_section_decision(section: str, decision: dict) -> None:
    lane_name, lane_m, changed = _normalize_start_section_lane(
        section,
        str(decision.get("lane_name", "center") or "center"),
        float(decision.get("lane_m", LANE_CENTER_M) or LANE_CENTER_M),
    )
    if not changed:
        return
    decision["lane_name"] = lane_name
    decision["lane_m"] = lane_m
    _append_gate_warning(decision, "bottom_start_outer_tight_clamped")


def _normalize_start_section_gate_decisions(decisions: dict[tuple[str, int], dict]) -> None:
    for (section, _gate_idx), decision in decisions.items():
        _normalize_start_section_decision(section, decision)


def _gate_index_from_along(along_m: float) -> int:
    return min(range(len(GATE_ALONGS_M)), key=lambda idx: abs(GATE_ALONGS_M[idx] - along_m))


def _start_section_obstacle_slot_rule_reason(slot: dict, direction: str, args) -> str:
    if str(slot.get("section", "") or "") != "bottom":
        return ""
    try:
        row_m = float(slot.get("row_m", 0.0) or 0.0)
        row_near = float(getattr(args, "obstacle_row_near_m", 0.40) or 0.40)
        row_far = float(getattr(args, "obstacle_row_far_m", 0.60) or 0.60)
    except (TypeError, ValueError):
        return "bottom_start_invalid_row"
    outer_row = min(row_near, row_far)
    inner_row = max(row_near, row_far)
    if inner_row > outer_row + 1e-6 and row_m < 0.5 * (outer_row + inner_row):
        return "bottom_start_outer_row"
    if direction not in ("CLOCK", "COUNTER"):
        return ""
    gate_idx = _gate_index_from_along(_slot_along_m(slot))
    direct_ahead_gate_idx = 1 if direction == "CLOCK" else 2
    if gate_idx == direct_ahead_gate_idx:
        return "bottom_start_direct_ahead_gate"
    return ""


def _start_section_obstacle_slot_allowed(slot: dict, direction: str, args) -> bool:
    if _free_race_enabled(args):
        return True
    return _start_section_obstacle_slot_rule_reason(slot, direction, args) == ""


def _navigation_obstacle_slots(obstacles: list[dict], direction: str, args) -> list[dict]:
    if _free_race_enabled(args):
        return list(obstacles)
    return [slot for slot in obstacles if _start_section_obstacle_slot_allowed(slot, direction, args)]


def _section_gate_id(section: str, gate_idx: int) -> str:
    return f"{section}_gate_{gate_idx}"


def _gate_order_for_section(section: str, direction: str) -> list[int]:
    return sorted(range(len(GATE_ALONGS_M)), key=lambda idx: _section_route_s(section, direction, GATE_ALONGS_M[idx]))


def _post_unpark_bottom_handoff_active(args) -> bool:
    if _free_race_enabled(args) or _all_gates_locked():
        return False
    if not _HYBRID_STATE.route_progress_initialized:
        return False
    if _HYBRID_STATE.route_section != "bottom":
        return False
    if ("bottom", 1) in _HYBRID_STATE.locked_gate_choices:
        return False
    max_progress = max(0.0, float(getattr(args, "navigation_unpark_handoff_progress_m", 0.75)))
    return float(_HYBRID_STATE.route_progress_m) <= max_progress


def _keep_empty_middle_gate_for_unpark_handoff(section: str, gate_idx: int, args) -> bool:
    return section == "bottom" and gate_idx == 1 and _post_unpark_bottom_handoff_active(args)


def _navigation_state_mutations_allowed(args) -> bool:
    return bool(getattr(args, "navigation_state_mutations_allowed", True)) and bool(getattr(args, "navigation_initial_pose_locked", True))


def _navigation_state_mutation_suppressed_reason(args) -> str:
    if not bool(getattr(args, "navigation_initial_pose_locked", True)):
        return "initial_pose_not_locked"
    if not bool(getattr(args, "navigation_state_mutations_allowed", True)):
        return "state_mutations_disabled"
    return ""


def _update_route_progress(route: Projection, mutations_allowed: bool = True, suppressed_reason: str = "") -> None:
    _HYBRID_STATE.route_section = str(route.section or "")
    _HYBRID_STATE.route_primitive = str(route.primitive or "")
    _HYBRID_STATE.route_progress_mutations_allowed = bool(mutations_allowed)
    _HYBRID_STATE.route_progress_baseline_synced = False
    _HYBRID_STATE.route_progress_suppressed_reason = ""
    _HYBRID_STATE.route_progress_suppressed_delta_m = 0.0
    if _HYBRID_STATE.direction != route.direction or not _HYBRID_STATE.route_progress_initialized:
        _HYBRID_STATE.route_start_s_m = route.s_m
        _HYBRID_STATE.route_prev_s_m = route.s_m
        _HYBRID_STATE.route_last_s_m = route.s_m
        _HYBRID_STATE.route_step_delta_m = 0.0
        _HYBRID_STATE.route_prev_progress_m = 0.0
        _HYBRID_STATE.route_progress_m = 0.0
        _HYBRID_STATE.route_progress_initialized = True
        if not mutations_allowed:
            _HYBRID_STATE.route_progress_baseline_synced = True
            _HYBRID_STATE.route_progress_suppressed_reason = suppressed_reason or "state_mutations_disabled"
        return
    delta = _forward_delta(_HYBRID_STATE.route_last_s_m, route.s_m)
    if delta > REF_LAP_LENGTH_M * 0.5:
        delta = 0.0
    if not mutations_allowed:
        _HYBRID_STATE.route_start_s_m = route.s_m
        _HYBRID_STATE.route_prev_s_m = route.s_m
        _HYBRID_STATE.route_last_s_m = route.s_m
        _HYBRID_STATE.route_step_delta_m = 0.0
        _HYBRID_STATE.route_prev_progress_m = 0.0
        _HYBRID_STATE.route_progress_m = 0.0
        _HYBRID_STATE.route_progress_baseline_synced = True
        _HYBRID_STATE.route_progress_suppressed_reason = suppressed_reason or "state_mutations_disabled"
        _HYBRID_STATE.route_progress_suppressed_delta_m = max(0.0, delta)
        return
    _HYBRID_STATE.route_prev_s_m = _HYBRID_STATE.route_last_s_m
    _HYBRID_STATE.route_step_delta_m = max(0.0, delta)
    _HYBRID_STATE.route_prev_progress_m = _HYBRID_STATE.route_progress_m
    _HYBRID_STATE.route_progress_m += max(0.0, delta)
    _HYBRID_STATE.route_last_s_m = route.s_m


def _lane_name_for_obstacle(obstacle: dict, direction: str, args) -> tuple[str, str]:
    color = obstacle.get("color", "")
    row = float(obstacle["row_m"])
    outer_pass = False
    inner_pass = False
    if color == "RED":
        outer_pass = direction == "COUNTER"
        inner_pass = not outer_pass
        reason = "red"
    elif color == "GREEN":
        inner_pass = direction == "COUNTER"
        outer_pass = not inner_pass
        reason = "green"
    else:
        # Unknown obstacles are handled by the row-based conservative choice
        # that leaves the larger gap open.
        outer_pass = row >= 0.5
        inner_pass = not outer_pass
        reason = "unknown"

    if outer_pass:
        return ("outer_tight" if row <= 0.5 else "outer_pass"), reason
    if inner_pass:
        return ("inner_pass" if row <= 0.5 else "inner_tight"), reason
    return "center", reason


def _trailing_lane_after_obstacle(source_position: str, source_lane_name: str) -> tuple[str, str]:
    lane = str(source_lane_name or "center")
    if source_position == "middle":
        return lane, "trail_middle_obstacle_same_lane"
    if source_position == "first":
        first_gate_map = {
            "inner_tight": "inner_pass",
            "inner_pass": "inner_pass",
            "outer_pass": "inner_pass",
            "outer_tight": "outer_pass",
            "center": "center",
        }
        return first_gate_map.get(lane, "center"), "trail_first_obstacle_step_lane"
    return "center", "trail_obstacle_unknown"


def _apply_trailing_gate_lane_rules(direction: str, decisions: dict[tuple[str, int], dict], args) -> None:
    for section in _order_sections(direction):
        gate_order = _gate_order_for_section(section, direction)
        if len(gate_order) < 3:
            continue
        first_gate_idx = gate_order[0]
        middle_gate_idx = gate_order[1]
        last_gate_idx = gate_order[-1]
        last_decision = decisions[(section, last_gate_idx)]
        if last_decision["obstacle_ids"] or last_decision.get("locked"):
            continue

        source_position = ""
        source_decision = None
        middle_decision = decisions[(section, middle_gate_idx)]
        first_decision = decisions[(section, first_gate_idx)]
        if middle_decision["obstacle_ids"]:
            source_position = "middle"
            source_decision = middle_decision
        elif first_decision["obstacle_ids"]:
            source_position = "first"
            source_decision = first_decision
        if source_decision is None:
            continue

        lane_name, reason = _trailing_lane_after_obstacle(source_position, str(source_decision.get("lane_name", "center")))
        last_decision["lane_name"] = lane_name
        last_decision["lane_m"] = _lane_value(lane_name, args)
        last_decision["reason"] = reason
        last_decision["warning"] = f"lane_after_{source_position}_obstacle"


def _gate_decisions(direction: str, obstacles: list[dict], args) -> dict[tuple[str, int], dict]:
    decisions: dict[tuple[str, int], dict] = {}
    free_race = _free_race_enabled(args)
    for section in ORDER_CLOCK:
        if free_race:
            default_lane_name, default_lane_m, default_reason = _free_race_section_lane(section, args, direction)
        else:
            default_lane_name, default_lane_m, default_reason = "center", _lane_value("center", args), "clear"
        for idx in range(len(GATE_ALONGS_M)):
            decisions[(section, idx)] = {
                "section": section,
                "gate_idx": idx,
                "along_m": GATE_ALONGS_M[idx],
                "lane_name": default_lane_name,
                "lane_m": default_lane_m,
                "reason": default_reason,
                "obstacle_ids": [],
                "warning": "",
            }

    if free_race:
        return decisions

    lane_rank = {name: idx for idx, name in enumerate(("outer_tight", "outer_pass", "center", "inner_pass", "inner_tight"))}
    for slot in obstacles:
        if not (slot.get("known") or slot.get("current")):
            continue
        section = str(slot.get("section", ""))
        if section not in ORDER_CLOCK:
            continue
        if not _start_section_obstacle_slot_allowed(slot, direction, args):
            continue
        obs = _project_slot(slot, direction)
        gate_idx = _gate_index_from_along(_slot_along_m(slot))
        lane_name, reason = _lane_name_for_obstacle(obs, direction, args)
        decision = decisions[(section, gate_idx)]
        decision["obstacle_ids"].append(str(obs.get("id", "")))
        decision["reason"] = reason
        if decision["lane_name"] != "center" and decision["lane_name"] != lane_name:
            current_rank = lane_rank[decision["lane_name"]]
            new_rank = lane_rank[lane_name]
            # Pick the lane farther from center if two same-gate obstacles force
            # different options; also expose this rare constraint in debug.
            lane_name = decision["lane_name"] if abs(current_rank - 2) >= abs(new_rank - 2) else lane_name
            decision["warning"] = "conflicting_gate_obstacles"
        decision["lane_name"] = lane_name
        decision["lane_m"] = _lane_value(lane_name, args)

    for key, locked in _HYBRID_STATE.locked_gate_choices.items():
        if key not in decisions:
            continue
        decision = decisions[key]
        decision["lane_name"] = str(locked.get("lane_name", decision["lane_name"]))
        decision["lane_m"] = float(locked.get("lane_m", decision["lane_m"]))
        decision["reason"] = str(locked.get("reason", "locked"))
        obstacle_id = str(locked.get("obstacle_id", ""))
        decision["obstacle_ids"] = [obstacle_id] if obstacle_id else list(decision["obstacle_ids"])
        decision["warning"] = "locked_passed_gate"
        decision["locked"] = True

    for section in _order_sections(direction):
        next_obstacle_decision = None
        for gate_idx in reversed(_gate_order_for_section(section, direction)):
            decision = decisions[(section, gate_idx)]
            if decision["obstacle_ids"] or decision.get("locked"):
                next_obstacle_decision = decision
                continue
            if next_obstacle_decision is None:
                continue
            decision["lane_name"] = next_obstacle_decision["lane_name"]
            decision["lane_m"] = next_obstacle_decision["lane_m"]
            decision["reason"] = f"lead_{next_obstacle_decision['reason']}"
            decision["warning"] = "lane_from_next_obstacle"
    _apply_trailing_gate_lane_rules(direction, decisions, args)
    _normalize_start_section_gate_decisions(decisions)
    if _navigation_state_mutations_allowed(args):
        _commit_pending_unknown_matches(decisions)
        _lock_passed_gate_decisions(direction, decisions, args)
    else:
        _set_gate_lock_suppressed(_navigation_state_mutation_suppressed_reason(args))
    return decisions


def _gate_decision_payload(decisions: dict[tuple[str, int], dict]) -> list[dict]:
    payload = []
    for (_section, _idx), decision in sorted(decisions.items(), key=lambda item: (item[0][0], item[0][1])):
        payload.append(
            {
                "section": decision["section"],
                "gate_idx": decision["gate_idx"],
                "along_m": decision["along_m"],
                "lane_name": decision["lane_name"],
                "lane_m": decision["lane_m"],
                "reason": decision["reason"],
                "obstacle_ids": list(decision["obstacle_ids"]),
                "warning": decision["warning"],
                "locked": bool(decision.get("locked", False)),
                "pending_unknown": (_section, _idx) in _HYBRID_STATE.pending_unknown_gate_choices,
            }
        )
    return payload


def _gate_lock_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _update_gate_lock_context(args) -> None:
    competition = getattr(args, "competition_start", {}) if args is not None else {}
    if not isinstance(competition, dict):
        competition = {}
    _HYBRID_STATE.gate_lock_context_startup_status = str(getattr(args, "navigation_initial_status", "") or "")
    _HYBRID_STATE.gate_lock_context_startup_state = str(competition.get("state", "") or "")
    _HYBRID_STATE.gate_lock_context_anchor_version = int(_gate_lock_float(getattr(args, "navigation_anchor_version", -1), -1.0))
    _HYBRID_STATE.gate_lock_context_anchor_source = str(getattr(args, "navigation_anchor_source", "") or "")
    _HYBRID_STATE.gate_lock_context_anchor_jump_m = _gate_lock_float(getattr(args, "navigation_anchor_last_jump_m", 0.0), 0.0)
    _HYBRID_STATE.gate_lock_context_anchor_jump_h_deg = _gate_lock_float(getattr(args, "navigation_anchor_last_jump_h_deg", 0.0), 0.0)
    _HYBRID_STATE.gate_lock_context_initial_pose_locked = bool(getattr(args, "navigation_initial_pose_locked", True))


def _set_gate_lock_event(action: str, section: str, gate_idx: int, decision: dict, reason: str, lane_name: str = "") -> None:
    obstacle_ids = _obstacle_ids_from_decision(decision)
    lane = str(lane_name or decision.get("lane_name", ""))
    route_progress = float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0)
    route_step = float(getattr(_HYBRID_STATE, "route_step_delta_m", 0.0) or 0.0)
    _HYBRID_STATE.gate_lock_last_event = str(action or "")
    _HYBRID_STATE.gate_lock_last_section = str(section or "")
    _HYBRID_STATE.gate_lock_last_gate_idx = int(gate_idx)
    _HYBRID_STATE.gate_lock_last_lane_name = lane
    _HYBRID_STATE.gate_lock_last_reason = str(reason or "")
    _HYBRID_STATE.gate_lock_last_decision_reason = str(decision.get("reason", "") or "")
    _HYBRID_STATE.gate_lock_last_obstacle_ids = ",".join(str(item) for item in obstacle_ids)
    _HYBRID_STATE.gate_lock_last_route_progress_m = route_progress
    _HYBRID_STATE.gate_lock_last_route_step_delta_m = route_step
    _HYBRID_STATE.gate_lock_last_active_waypoint_id = str(getattr(_HYBRID_STATE, "active_waypoint_id", "") or "")
    _HYBRID_STATE.gate_lock_last_startup_status = _HYBRID_STATE.gate_lock_context_startup_status
    _HYBRID_STATE.gate_lock_last_startup_state = _HYBRID_STATE.gate_lock_context_startup_state
    _HYBRID_STATE.gate_lock_last_anchor_version = _HYBRID_STATE.gate_lock_context_anchor_version
    _HYBRID_STATE.gate_lock_last_anchor_source = _HYBRID_STATE.gate_lock_context_anchor_source
    _HYBRID_STATE.gate_lock_last_anchor_jump_m = _HYBRID_STATE.gate_lock_context_anchor_jump_m
    _HYBRID_STATE.gate_lock_last_anchor_jump_h_deg = _HYBRID_STATE.gate_lock_context_anchor_jump_h_deg
    _HYBRID_STATE.gate_lock_last_initial_pose_locked = _HYBRID_STATE.gate_lock_context_initial_pose_locked
    _HYBRID_STATE.gate_lock_event_count += 1
    event = (
        f"{_HYBRID_STATE.gate_lock_event_count}:{_HYBRID_STATE.gate_lock_last_event}"
        f":{_HYBRID_STATE.gate_lock_last_section}:{_HYBRID_STATE.gate_lock_last_gate_idx}"
        f":lane={lane}:reason={_HYBRID_STATE.gate_lock_last_reason}"
        f":decision={_HYBRID_STATE.gate_lock_last_decision_reason}"
        f":obs={_HYBRID_STATE.gate_lock_last_obstacle_ids}"
        f":rp={route_progress:.3f}:step={route_step:.3f}"
        f":wp={_HYBRID_STATE.gate_lock_last_active_waypoint_id}"
        f":init={_HYBRID_STATE.gate_lock_last_startup_status}"
        f":startup={_HYBRID_STATE.gate_lock_last_startup_state}"
        f":locked={int(_HYBRID_STATE.gate_lock_last_initial_pose_locked)}"
        f":anchor={_HYBRID_STATE.gate_lock_last_anchor_version}/{_HYBRID_STATE.gate_lock_last_anchor_source}"
        f":jump={_HYBRID_STATE.gate_lock_last_anchor_jump_m:.3f}/{_HYBRID_STATE.gate_lock_last_anchor_jump_h_deg:.2f}"
    )
    _HYBRID_STATE.gate_lock_recent_events.append(event)
    _HYBRID_STATE.gate_lock_suppressed_reason = ""


def _set_gate_lock_suppressed(reason: str) -> None:
    _HYBRID_STATE.gate_lock_suppressed_reason = str(reason or "")


def _lock_gate_decision(section: str, gate_idx: int, decision: dict, reason: str) -> None:
    raw_lane_name = str(decision.get("lane_name", "center"))
    raw_lane_m = float(decision.get("lane_m", LANE_VALUES_M.get(raw_lane_name, LANE_CENTER_M)))
    lane_name, lane_m, clamped = _normalize_start_section_lane(section, raw_lane_name, raw_lane_m)
    obstacle_ids = list(decision.get("obstacle_ids", []))
    waypoint_id = f"{_section_gate_id(section, gate_idx)}_{lane_name}"
    locked = {
        "lane_name": lane_name,
        "lane_m": lane_m,
        "reason": f"locked_{decision.get('reason', reason)}",
        "obstacle_id": ",".join(str(item) for item in obstacle_ids),
        "waypoint_id": waypoint_id,
        "lock_reason": reason,
    }
    if clamped:
        locked["start_section_lane_clamped"] = True
    _HYBRID_STATE.locked_gate_choices[(section, gate_idx)] = locked
    _set_gate_lock_event("locked", section, gate_idx, decision, reason, lane_name)


def _gate_key_label(key: tuple[str, int]) -> str:
    return f"{key[0]}:{key[1]}"


def _set_pending_unknown_gate_event(action: str, key: tuple[str, int], reason: str) -> None:
    _HYBRID_STATE.pending_unknown_gate_last_action = str(action or "")
    _HYBRID_STATE.pending_unknown_gate_last_key = _gate_key_label(key)
    _HYBRID_STATE.pending_unknown_gate_last_reason = str(reason or "")


def _obstacle_ids_from_decision(decision: dict) -> list[str]:
    obstacle_ids = decision.get("obstacle_ids", []) or []
    if isinstance(obstacle_ids, str):
        return [item for item in obstacle_ids.split(",") if item]
    return [str(item) for item in obstacle_ids if str(item)]


def _remember_pending_unknown_gate(section: str, gate_idx: int, decision: dict, reason: str) -> None:
    key = (section, gate_idx)
    if key in _HYBRID_STATE.locked_gate_choices or key in _HYBRID_STATE.pending_unknown_gate_choices:
        return
    raw_lane_name = str(decision.get("lane_name", "center"))
    raw_lane_m = float(decision.get("lane_m", LANE_VALUES_M.get(raw_lane_name, LANE_CENTER_M)))
    lane_name, lane_m, clamped = _normalize_start_section_lane(section, raw_lane_name, raw_lane_m)
    obstacle_ids = _obstacle_ids_from_decision(decision)
    pending = {
        "lane_name": lane_name,
        "lane_m": lane_m,
        "reason": str(decision.get("reason", reason) or reason),
        "obstacle_id": ",".join(obstacle_ids),
        "waypoint_id": f"{_section_gate_id(section, gate_idx)}_{lane_name}",
        "lock_reason": reason,
        "passed_s_m": float(getattr(_HYBRID_STATE, "route_last_s_m", 0.0) or 0.0),
        "passed_route_progress_m": float(getattr(_HYBRID_STATE, "route_progress_m", 0.0) or 0.0),
        "created_t": time.monotonic(),
    }
    if clamped:
        pending["start_section_lane_clamped"] = True
    _HYBRID_STATE.pending_unknown_gate_choices[key] = pending
    _set_pending_unknown_gate_event("pending_unknown_stored", key, reason)
    _set_gate_lock_event("pending_unknown_stored", section, gate_idx, decision, reason, lane_name)


def _commit_pending_unknown_gate(key: tuple[str, int], reason: str) -> None:
    data = _HYBRID_STATE.pending_unknown_gate_choices.pop(key, None)
    if data is None:
        return
    if key in _HYBRID_STATE.locked_gate_choices:
        _set_pending_unknown_gate_event("pending_unknown_dropped_already_locked", key, reason)
        drop_decision = {
            "lane_name": str(data.get("lane_name", "")),
            "lane_m": data.get("lane_m", 0.0),
            "reason": str(data.get("reason", reason) or reason),
            "obstacle_ids": [item for item in str(data.get("obstacle_id", "") or "").split(",") if item],
        }
        _set_gate_lock_event("pending_unknown_dropped_already_locked", key[0], key[1], drop_decision, reason, str(data.get("lane_name", "")))
        return
    obstacle_id = str(data.get("obstacle_id", "") or "")
    decision = {
        "lane_name": str(data.get("lane_name", "center")),
        "lane_m": float(data.get("lane_m", LANE_CENTER_M) or LANE_CENTER_M),
        "reason": str(data.get("reason", reason) or reason),
        "obstacle_ids": [item for item in obstacle_id.split(",") if item],
    }
    _lock_gate_decision(key[0], key[1], decision, reason)
    locked = _HYBRID_STATE.locked_gate_choices.get(key)
    if locked is not None:
        locked["deferred_unknown"] = True
        locked["deferred_unknown_reason"] = reason
        locked["pending_lock_reason"] = str(data.get("lock_reason", "") or "")
    _set_pending_unknown_gate_event("pending_unknown_committed", key, reason)
    _set_gate_lock_event("pending_unknown_committed", key[0], key[1], decision, reason, str(decision.get("lane_name", "")))


def _gate_lane_matches_pending(decision: dict, pending: dict) -> bool:
    lane_name = str(decision.get("lane_name", ""))
    pending_lane_name = str(pending.get("lane_name", ""))
    if lane_name and pending_lane_name and lane_name == pending_lane_name:
        return True
    try:
        lane_m = float(decision.get("lane_m", float("nan")))
        pending_lane_m = float(pending.get("lane_m", float("nan")))
    except (TypeError, ValueError):
        return False
    return math.isfinite(lane_m) and math.isfinite(pending_lane_m) and abs(lane_m - pending_lane_m) <= 1e-4


def _commit_pending_unknown_matches(decisions: dict[tuple[str, int], dict]) -> None:
    for key, pending in list(_HYBRID_STATE.pending_unknown_gate_choices.items()):
        if key in _HYBRID_STATE.locked_gate_choices:
            _HYBRID_STATE.pending_unknown_gate_choices.pop(key, None)
            _set_pending_unknown_gate_event("pending_unknown_dropped_already_locked", key, "retrospective_match")
            drop_decision = {
                "lane_name": str(pending.get("lane_name", "")),
                "lane_m": pending.get("lane_m", 0.0),
                "reason": str(pending.get("reason", "retrospective_match") or "retrospective_match"),
                "obstacle_ids": [item for item in str(pending.get("obstacle_id", "") or "").split(",") if item],
            }
            _set_gate_lock_event("pending_unknown_dropped_already_locked", key[0], key[1], drop_decision, "retrospective_match", str(pending.get("lane_name", "")))
            continue
        decision = decisions.get(key)
        if decision is None or _gate_decision_is_provisional_unknown(decision):
            continue
        if _gate_lane_matches_pending(decision, pending):
            _commit_pending_unknown_gate(key, "retrospective_known_match")
        else:
            mismatch_reason = str(decision.get("reason", "") or "")
            _set_pending_unknown_gate_event("pending_unknown_known_mismatch", key, mismatch_reason)
            _set_gate_lock_event("pending_unknown_known_mismatch", key[0], key[1], decision, mismatch_reason, str(decision.get("lane_name", "")))


def _gate_counted_passed(section: str, gate_idx: int) -> bool:
    key = (section, gate_idx)
    return key in _HYBRID_STATE.locked_gate_choices or key in _HYBRID_STATE.pending_unknown_gate_choices


def _commit_completed_pending_unknown_sections(direction: str) -> None:
    for section in _order_sections(direction):
        pending_keys = [key for key in _HYBRID_STATE.pending_unknown_gate_choices if key[0] == section]
        if not pending_keys:
            continue
        if all(_gate_counted_passed(section, gate_idx) for gate_idx in range(len(GATE_ALONGS_M))):
            for key in sorted(pending_keys, key=lambda item: item[1]):
                _commit_pending_unknown_gate(key, "deferred_unknown_section_complete")


def _lock_passed_gate_decisions(direction: str, decisions: dict[tuple[str, int], dict], args) -> None:
    if not _navigation_state_mutations_allowed(args):
        _set_gate_lock_suppressed(_navigation_state_mutation_suppressed_reason(args))
        return
    if not _HYBRID_STATE.route_progress_initialized:
        return
    margin = float(getattr(args, "navigation_gate_lock_pass_margin_m", 0.04))
    step_delta = float(getattr(_HYBRID_STATE, "route_step_delta_m", 0.0) or 0.0)
    max_step = max(0.05, float(getattr(args, "navigation_gate_lock_max_step_m", 0.25)))
    if step_delta <= 0.0 or step_delta > max_step:
        return
    prev_s = float(getattr(_HYBRID_STATE, "route_prev_s_m", _HYBRID_STATE.route_last_s_m))
    for section in _order_sections(direction):
        for gate_idx in range(len(GATE_ALONGS_M)):
            key = (section, gate_idx)
            if key in _HYBRID_STATE.locked_gate_choices or key in _HYBRID_STATE.pending_unknown_gate_choices:
                continue
            gate_s = _section_route_s(section, direction, GATE_ALONGS_M[gate_idx])
            crossed_delta = _forward_delta(prev_s, gate_s)
            if crossed_delta <= step_delta + margin:
                decision = decisions[key]
                if _unknown_gate_lock_should_defer(decision, args):
                    _remember_pending_unknown_gate(section, gate_idx, decision, "route_passed_gate")
                    decision["warning"] = "unknown_gate_lock_pending"
                    continue
                _lock_gate_decision(section, gate_idx, decision, "route_passed_gate")
    _commit_completed_pending_unknown_sections(direction)


def _corner_config_path(args) -> Path:
    configured = str(getattr(args, "navigation_corner_config", "") or "")
    return Path(configured).expanduser() if configured else DEFAULT_CORNER_WAYPOINT_CONFIG


def _corner_config_mtime(args) -> float | None:
    path = _corner_config_path(args)
    try:
        return path.stat().st_mtime
    except OSError:
        return None


def _load_corner_config(args) -> dict | None:
    path = _corner_config_path(args)
    path_key = str(path)
    try:
        stat = path.stat()
    except OSError:
        _CORNER_CONFIG_CACHE.update({"path": path_key, "mtime": None, "data": None, "error": ""})
        return None
    if _CORNER_CONFIG_CACHE["path"] == path_key and _CORNER_CONFIG_CACHE["mtime"] == stat.st_mtime:
        return _CORNER_CONFIG_CACHE["data"]
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        _CORNER_CONFIG_CACHE.update({"path": path_key, "mtime": stat.st_mtime, "data": None, "error": str(exc)})
        return None
    _CORNER_CONFIG_CACHE.update({"path": path_key, "mtime": stat.st_mtime, "data": data, "error": ""})
    return data


def _corner_profile(in_lane: float, out_lane: float, args) -> tuple[tuple[str, float, float], ...]:
    entry_frac = _clamp(float(getattr(args, "navigation_corner_entry_frac", 0.26)), 0.05, 0.45)
    exit_frac = _clamp(float(getattr(args, "navigation_corner_exit_frac", 0.74)), 0.55, 0.95)
    lane_delta = out_lane - in_lane
    if abs(lane_delta) < 0.035:
        return (
            ("entry", entry_frac, in_lane),
            ("blend", 0.50, in_lane + 0.5 * lane_delta),
            ("exit", exit_frac, out_lane),
        )
    if lane_delta > 0.0:
        return (
            ("entry", 0.26, in_lane),
            ("blend", 0.54, in_lane + 0.45 * lane_delta),
            ("exit", 0.74, out_lane),
        )
    late_exit_frac = 0.75 if 0.35 <= out_lane <= 0.65 else 0.86
    return (
        ("entry", 0.26, in_lane),
        ("blend", 0.62, in_lane + 0.35 * lane_delta),
        ("exit", late_exit_frac, out_lane),
    )


def _corner_basis(in_heading: float, out_heading: float) -> tuple[tuple[float, float], tuple[float, float]]:
    ih = math.radians(in_heading)
    oh = math.radians(out_heading)
    return (math.cos(ih), math.sin(ih)), (math.cos(oh), math.sin(oh))


def _corner_generated_points(
    direction: str,
    arc: Primitive,
    in_lane: float,
    out_lane: float,
    in_heading: float,
    out_heading: float,
    args,
    start_xy: tuple[float, float] | None = None,
    end_xy: tuple[float, float] | None = None,
) -> list[dict]:
    raw_points = []
    for label, local_frac, lane in _corner_profile(in_lane, out_lane, args):
        s_m = arc.start_s_m + arc.length_m * local_frac
        lane = _clamp(lane, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))
        x, y, _section_name, _primitive, _local_s, nominal_heading = _point_on_route_s(s_m, lane, direction)
        raw_points.append({"label": label, "s_m": s_m, "lane_m": lane, "x": x, "y": y, "nominal_heading": nominal_heading})
    sequence = [(item["x"], item["y"]) for item in raw_points]
    if start_xy is not None:
        sequence = [start_xy] + sequence
    if end_xy is not None:
        sequence = sequence + [end_xy]
    return _corner_with_path_headings(raw_points, sequence, in_heading, out_heading)


def _corner_with_path_headings(raw_points: list[dict], sequence: list[tuple[float, float]], in_heading: float, out_heading: float) -> list[dict]:
    headings = []
    for point_idx, item in enumerate(raw_points):
        offset = 1 if len(sequence) == len(raw_points) + 2 else 0
        current_xy = sequence[point_idx + offset]
        next_xy = sequence[point_idx + offset + 1] if point_idx + offset + 1 < len(sequence) else current_xy
        headings.append(_clamp_heading_between(in_heading, out_heading, _bearing_deg(current_xy, next_xy, float(item.get("nominal_heading", in_heading)))))
    turn = _wrap_deg(out_heading - in_heading)
    last_rel = 0.0
    out = []
    for item, heading in zip(raw_points, headings):
        rel = _wrap_deg(heading - in_heading)
        if turn >= 0.0:
            rel = _clamp(max(rel, last_rel), 0.0, turn)
            last_rel = rel
        else:
            rel_abs = _clamp(max(-rel, last_rel), 0.0, -turn)
            last_rel = rel_abs
            rel = -rel_abs
        fixed = dict(item)
        fixed["heading_deg"] = _wrap_deg(in_heading + rel)
        out.append(fixed)
    return out


def _free_race_corner_points(
    direction: str,
    arc: Primitive,
    in_heading: float,
    out_heading: float,
    args,
) -> list[dict]:
    radius = max(0.05, float(getattr(args, "free_race_corner_radius_m", DEFAULT_FREE_RACE_CORNER_RADIUS_M)))
    offset = max(0.0, float(getattr(args, "free_race_corner_center_offset_m", DEFAULT_FREE_RACE_CORNER_CENTER_OFFSET_M)))
    cx, cy = arc.center_xy or (0.0, 0.0)
    in_vec, out_vec = _corner_basis(in_heading, out_heading)
    center_x = cx + offset * (in_vec[0] - out_vec[0])
    center_y = cy + offset * (in_vec[1] - out_vec[1])
    mid_len = math.hypot(in_vec[0] - out_vec[0], in_vec[1] - out_vec[1])
    mid_vec = (in_vec[0], in_vec[1]) if mid_len <= 1e-9 else ((in_vec[0] - out_vec[0]) / mid_len, (in_vec[1] - out_vec[1]) / mid_len)
    turn = _wrap_deg(out_heading - in_heading)
    raw_points = (
        ("entry", center_x - out_vec[0] * radius, center_y - out_vec[1] * radius, 0.0),
        ("blend", center_x + mid_vec[0] * radius, center_y + mid_vec[1] * radius, 0.5),
        ("exit", center_x + in_vec[0] * radius, center_y + in_vec[1] * radius, 1.0),
    )
    points = []
    for label, x, y, frac in raw_points:
        heading = _wrap_deg(in_heading + turn * frac)
        proj = _project_to_direction(x, y, heading, direction)
        points.append(
            {
                "label": label,
                "s_m": proj.s_m,
                "lane_m": _clamp(proj.lateral_m, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m)),
                "x": x,
                "y": y,
                "heading_deg": heading,
                "nominal_heading": proj.nominal_heading_deg,
            }
        )
    return points


def _corner_points_from_config_item(
    item: dict,
    direction: str,
    arc: Primitive,
    in_heading: float,
    out_heading: float,
    args,
) -> list[dict] | None:
    points = item.get("points") if isinstance(item, dict) else None
    if not isinstance(points, list) or len(points) < 2:
        return None
    cx, cy = arc.center_xy or (0.0, 0.0)
    in_vec, out_vec = _corner_basis(in_heading, out_heading)
    out = []
    for point in points:
        if not isinstance(point, dict):
            return None
        try:
            label = str(point["label"])
            u_m = float(point["u_m"])
            v_m = float(point["v_m"])
            rel_heading = float(point["heading_deg"])
        except (KeyError, TypeError, ValueError):
            return None
        x = cx + u_m * in_vec[0] + v_m * out_vec[0]
        y = cy + u_m * in_vec[1] + v_m * out_vec[1]
        heading = _wrap_deg(in_heading + rel_heading)
        proj = _project_to_direction(x, y, heading, direction)
        out.append(
            {
                "label": label,
                "s_m": proj.s_m,
                "lane_m": _clamp(proj.lateral_m, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m)),
                "x": x,
                "y": y,
                "heading_deg": heading,
                "nominal_heading": proj.nominal_heading_deg,
            }
        )
    return out


def _corner_config_points(
    direction: str,
    transition_key: str,
    arc: Primitive,
    in_heading: float,
    out_heading: float,
    args,
) -> list[dict] | None:
    config = _load_corner_config(args)
    if not config or not bool(config.get("enabled", True)):
        return None
    direction_key = direction.lower()
    transitions = config.get(direction_key, {})
    item = transitions.get(transition_key, {}) if isinstance(transitions, dict) else {}
    return _corner_points_from_config_item(item, direction, arc, in_heading, out_heading, args)


def _corner_generation_args(args=None):
    return args or SimpleNamespace(
        navigation_min_lane_m=0.20,
        navigation_max_lane_m=0.82,
        navigation_corner_entry_frac=0.26,
        navigation_corner_exit_frac=0.74,
        free_race_corner_radius_m=DEFAULT_FREE_RACE_CORNER_RADIUS_M,
        free_race_corner_center_offset_m=DEFAULT_FREE_RACE_CORNER_CENTER_OFFSET_M,
    )


def _corner_canonical_context(direction: str) -> tuple[Primitive, float, float, float, float, tuple[float, float], tuple[float, float]]:
    direction = direction.upper()
    prims = _primitives(direction)
    # Use the first corner after the bottom straight as canonical for the
    # editor; runtime rotates this local shape into each actual corner.
    arc = next(prim for prim in prims if prim.kind == "arc" and prim.name in ("bottom_right_arc", "bottom_left_arc"))
    if direction == "COUNTER":
        in_heading, out_heading = 0.0, 90.0
    else:
        in_heading, out_heading = 180.0, 90.0
    cx, cy = arc.center_xy or (0.0, 0.0)
    in_vec, out_vec = _corner_basis(in_heading, out_heading)
    return arc, in_heading, out_heading, cx, cy, in_vec, out_vec


def _corner_local_item_from_points(
    points: list[dict],
    cx: float,
    cy: float,
    in_vec: tuple[float, float],
    out_vec: tuple[float, float],
    in_heading: float,
) -> dict:
    local_points = []
    for point in points:
        dx, dy = point["x"] - cx, point["y"] - cy
        local_points.append(
            {
                "label": point["label"],
                "u_m": dx * in_vec[0] + dy * in_vec[1],
                "v_m": dx * out_vec[0] + dy * out_vec[1],
                "heading_deg": _wrap_deg(point["heading_deg"] - in_heading),
            }
        )
    return {"points": local_points}


def _free_race_default_transition_item(direction: str, transition_key: str, args=None) -> dict:
    args = _corner_generation_args(args)
    direction = direction.upper()
    if transition_key not in FREE_RACE_CORNER_TRANSITIONS:
        transition_key = "outer_pass_to_outer_pass"
    in_lane_name, out_lane_name = transition_key.split("_to_", 1)
    arc, in_heading, out_heading, cx, cy, in_vec, out_vec = _corner_canonical_context(direction)
    if transition_key == "outer_pass_to_outer_pass":
        # Preserve the pre-adaptive free-race corner formula exactly as the
        # editable default for the free-race outer lane.
        points = _free_race_corner_points(direction, arc, in_heading, out_heading, args)
    else:
        points = _corner_generated_points(
            direction,
            arc,
            LANE_VALUES_M[in_lane_name],
            LANE_VALUES_M[out_lane_name],
            in_heading,
            out_heading,
            args,
        )
    return _corner_local_item_from_points(points, cx, cy, in_vec, out_vec, in_heading)


def _free_race_corner_config_points(
    direction: str,
    transition_key: str,
    arc: Primitive,
    in_heading: float,
    out_heading: float,
    args,
) -> list[dict]:
    config = _load_corner_config(args)
    item = {}
    if config and bool(config.get("enabled", True)):
        free_race = config.get("free_race", {})
        direction_table = free_race.get(direction.lower(), {}) if isinstance(free_race, dict) else {}
        item = direction_table.get(transition_key, {}) if isinstance(direction_table, dict) else {}
    points = _corner_points_from_config_item(item, direction, arc, in_heading, out_heading, args)
    if points is not None:
        return points
    default_item = _free_race_default_transition_item(direction, transition_key, args)
    points = _corner_points_from_config_item(default_item, direction, arc, in_heading, out_heading, args)
    if points is not None:
        return points
    return _free_race_corner_points(direction, arc, in_heading, out_heading, args)


def generate_corner_waypoint_config(args=None) -> dict:
    """Return editable local-frame defaults for normal and free-race corners.

    The stored coordinates are local to a corner: u_m follows the incoming
    straight heading from the corner center, v_m follows the outgoing straight
    heading, and heading_deg is relative to the incoming heading.
    """
    args = _corner_generation_args(args)
    config = {
        "version": 2,
        "enabled": True,
        "lanes": list(CORNER_LANE_NAMES),
        "free_race_lanes": list(FREE_RACE_CORNER_LANE_NAMES),
        "counter": {},
        "clock": {},
        "free_race": {"counter": {}, "clock": {}},
    }
    for direction in ("COUNTER", "CLOCK"):
        arc, in_heading, out_heading, cx, cy, in_vec, out_vec = _corner_canonical_context(direction)
        direction_key = direction.lower()
        for in_lane_name in CORNER_LANE_NAMES:
            for out_lane_name in CORNER_LANE_NAMES:
                in_lane = LANE_VALUES_M[in_lane_name]
                out_lane = LANE_VALUES_M[out_lane_name]
                points = _corner_generated_points(direction, arc, in_lane, out_lane, in_heading, out_heading, args)
                config[direction_key][f"{in_lane_name}_to_{out_lane_name}"] = _corner_local_item_from_points(points, cx, cy, in_vec, out_vec, in_heading)
        for transition_key in FREE_RACE_CORNER_TRANSITIONS:
            config["free_race"][direction_key][transition_key] = _free_race_default_transition_item(direction, transition_key, args)
    return config


def complete_corner_waypoint_config(data: dict, args=None) -> dict:
    defaults = generate_corner_waypoint_config(args)
    if not isinstance(data, dict):
        return defaults
    out = dict(data)
    try:
        version = int(out.get("version", 0) or 0)
    except (TypeError, ValueError):
        version = 0
    out["version"] = max(version, int(defaults["version"]))
    out.setdefault("enabled", True)
    out["lanes"] = list(out.get("lanes") or defaults["lanes"])
    out["free_race_lanes"] = list(out.get("free_race_lanes") or defaults["free_race_lanes"])
    for direction_key in ("counter", "clock"):
        table = out.get(direction_key)
        if not isinstance(table, dict):
            table = {}
        for key, item in defaults[direction_key].items():
            table.setdefault(key, item)
        out[direction_key] = table
    free_race = out.get("free_race")
    if not isinstance(free_race, dict):
        free_race = {}
    for direction_key in ("counter", "clock"):
        table = free_race.get(direction_key)
        if not isinstance(table, dict):
            table = {}
        for key in FREE_RACE_CORNER_TRANSITIONS:
            if key == "outer_pass_to_outer_pass":
                default_item = defaults["free_race"][direction_key][key]
            else:
                default_item = out.get(direction_key, {}).get(key, defaults["free_race"][direction_key][key])
            table.setdefault(key, default_item)
        free_race[direction_key] = table
    out["free_race"] = free_race
    return out


def _base_template_waypoints(direction: str, args) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    for section in _order_sections(direction):
        for gate_idx, along in enumerate(GATE_ALONGS_M):
            s_m = _section_route_s(section, direction, along)
            wp_id = f"{_section_gate_id(section, gate_idx)}_center"
            waypoints.append(_make_waypoint(s_m, _lane_value("center", args), direction, wp_id, waypoint_id=wp_id, kind="gate"))
    return waypoints


def _order_sections(direction: str) -> tuple[str, ...]:
    return ORDER_CLOCK if direction == "CLOCK" else ORDER_COUNTER


def _next_section(section: str, direction: str) -> str:
    sections = _order_sections(direction)
    if section not in sections:
        return ""
    return sections[(sections.index(section) + 1) % len(sections)]


def _previous_section(section: str, direction: str) -> str:
    sections = _order_sections(direction)
    if section not in sections:
        return ""
    return sections[(sections.index(section) - 1) % len(sections)]


def _straight_primitive_for_section(section: str, direction: str) -> Primitive | None:
    for prim in _primitives(direction):
        if prim.kind == "straight" and prim.section == section:
            return prim
    return None


def _clear_corner_observation_handoff() -> None:
    _HYBRID_STATE.corner_observation_handoff_section = ""
    _HYBRID_STATE.corner_observation_handoff_direction = ""
    _HYBRID_STATE.corner_observation_handoff_until_t = 0.0


def _arm_corner_observation_handoff(section: str, direction: str, now: float, args) -> None:
    _HYBRID_STATE.corner_observation_handoff_section = str(section or "")
    _HYBRID_STATE.corner_observation_handoff_direction = str(direction or "")
    _HYBRID_STATE.corner_observation_handoff_until_t = now + max(0.1, float(getattr(args, "navigation_color_confirmation_corner_handoff_s", 0.75)))


def _corner_observation_handoff_active(route: Projection, now: float) -> str:
    section = str(_HYBRID_STATE.corner_observation_handoff_section or "")
    if not section:
        return ""
    if str(_HYBRID_STATE.corner_observation_handoff_direction or "") != route.direction:
        _clear_corner_observation_handoff()
        return ""
    if now > float(_HYBRID_STATE.corner_observation_handoff_until_t or 0.0):
        _clear_corner_observation_handoff()
        return ""
    return section


def _color_confirmation_same_section_pre_gate_m(args) -> float:
    return max(0.0, float(getattr(args, "navigation_color_confirmation_deadline_pre_gate_m", 0.10)))


def _color_confirmation_deadline_trigger_margin_m(args) -> float:
    return max(0.0, float(getattr(args, "navigation_color_confirmation_deadline_trigger_margin_m", 0.02)))


def _color_confirmation_deadline_key(direction: str, section: str, gate_idx: int) -> str:
    return f"{direction}:{section}:{gate_idx}"


def _color_confirmation_deadline_status(route: Projection, obs: dict, args) -> dict:
    section = str(obs.get("section", "") or "")
    if section not in ORDER_CLOCK or route.direction not in ("CLOCK", "COUNTER"):
        return {"triggered": False}
    gate_idx = _gate_index_from_along(_slot_along_m(obs))
    gate_order = _gate_order_for_section(section, route.direction)
    if gate_idx not in gate_order:
        return {"triggered": False}
    order_pos = gate_order.index(gate_idx)
    if order_pos <= 0:
        deadline_section = _previous_section(section, route.direction)
        prim = _straight_primitive_for_section(deadline_section, route.direction)
        deadline_s = (prim.start_s_m + prim.length_m) % REF_LAP_LENGTH_M if prim is not None else 0.0
        return {
            "triggered": False,
            "trigger_source": "deadline_after_corner",
            "deadline_key": _color_confirmation_deadline_key(route.direction, section, gate_idx),
            "deadline_section": deadline_section,
            "deadline_gate_idx": None,
            "obstacle_section": section,
            "obstacle_gate_idx": gate_idx,
            "deadline_delta_m": _forward_delta(route.s_m, deadline_s) if prim is not None else 0.0,
            "deadline_trigger_delta_m": 0.0,
            "deadline_pre_gate_m": 0.0,
            "deadline_trigger_margin_m": 0.0,
            "deadline_crossed_recently": False,
            "deadline_requires_corner_handoff": True,
        }

    deadline_gate_idx = gate_order[order_pos - 1]
    deadline_section = section
    gate_s = _section_route_s(section, route.direction, GATE_ALONGS_M[deadline_gate_idx])
    pre_gate_m = _color_confirmation_same_section_pre_gate_m(args)
    trigger_margin_m = _color_confirmation_deadline_trigger_margin_m(args)
    trigger_s = (gate_s - pre_gate_m) % REF_LAP_LENGTH_M
    gate_delta_m = _forward_delta(route.s_m, gate_s)
    trigger_delta_m = _forward_delta(route.s_m, trigger_s)
    prev_s = float(getattr(_HYBRID_STATE, "route_prev_s_m", route.s_m) or route.s_m)
    step_delta = max(0.0, float(getattr(_HYBRID_STATE, "route_step_delta_m", 0.0) or 0.0))
    max_step = max(0.05, float(getattr(args, "navigation_gate_lock_max_step_m", 0.25)))
    crossed_recently = False
    if 0.0 < step_delta <= max_step:
        prev_trigger_delta = _forward_delta(prev_s, trigger_s)
        crossed_recently = prev_trigger_delta <= step_delta + trigger_margin_m
    triggered = gate_delta_m <= pre_gate_m + trigger_margin_m or crossed_recently
    return {
        "triggered": triggered,
        "trigger_source": "deadline_gate",
        "deadline_key": _color_confirmation_deadline_key(route.direction, section, gate_idx),
        "deadline_section": deadline_section,
        "deadline_gate_idx": deadline_gate_idx,
        "obstacle_section": section,
        "obstacle_gate_idx": gate_idx,
        "deadline_delta_m": gate_delta_m,
        "deadline_trigger_delta_m": trigger_delta_m,
        "deadline_pre_gate_m": pre_gate_m,
        "deadline_trigger_margin_m": trigger_margin_m,
        "deadline_crossed_recently": crossed_recently,
        "deadline_requires_corner_handoff": False,
    }


def _gate_waypoints(direction: str, decisions: dict[tuple[str, int], dict], args) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    free_race = _free_race_enabled(args)
    for section in _order_sections(direction):
        for gate_idx, along in enumerate(GATE_ALONGS_M):
            decision = decisions[(section, gate_idx)]
            if (
                gate_idx == 1
                and not free_race
                and not decision["obstacle_ids"]
                and not _keep_empty_middle_gate_for_unpark_handoff(section, gate_idx, args)
            ):
                continue
            lane_name = str(decision["lane_name"])
            lane_m = float(decision["lane_m"])
            s_m = _section_route_s(section, direction, along)
            wp_id = f"{_section_gate_id(section, gate_idx)}_{lane_name}"
            obstacle_id = ",".join(decision["obstacle_ids"])
            reason = f"{section}_gate_{gate_idx}_{decision['reason']}_{lane_name}"
            speed_hint = "obstacle" if obstacle_id else "cruise"
            waypoints.append(_make_waypoint(s_m, lane_m, direction, reason, obstacle_id, speed_hint, wp_id, "gate"))
    return waypoints


def _corner_waypoints(direction: str, decisions: dict[tuple[str, int], dict], args) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    sections = _order_sections(direction)
    for idx, section in enumerate(sections):
        next_section = sections[(idx + 1) % len(sections)]
        current_gate_order = _gate_order_for_section(section, direction)
        next_gate_order = _gate_order_for_section(next_section, direction)
        incoming = decisions[(section, current_gate_order[-1])]
        outgoing = decisions[(next_section, next_gate_order[0])]
        in_lane_name = str(incoming["lane_name"])
        out_lane_name = str(outgoing["lane_name"])
        in_lane = float(incoming["lane_m"])
        out_lane = float(outgoing["lane_m"])
        # Find the arc between the two straight primitives in route order.
        prims = _primitives(direction)
        straight_indices = [i for i, prim in enumerate(prims) if prim.kind == "straight" and prim.section == section]
        if not straight_indices:
            continue
        arc = prims[(straight_indices[0] + 1) % len(prims)]
        if arc.kind != "arc":
            continue
        in_s = _section_route_s(section, direction, float(incoming["along_m"]))
        out_s = _section_route_s(next_section, direction, float(outgoing["along_m"]))
        in_x, in_y, _in_section, _in_primitive, _in_local_s, in_heading = _point_on_route_s(in_s, in_lane, direction)
        out_x, out_y, _out_section, _out_primitive, _out_local_s, out_heading = _point_on_route_s(out_s, out_lane, direction)
        transition_key = f"{in_lane_name}_to_{out_lane_name}"
        transition_id = f"{section}_to_{next_section}_{transition_key}"
        if _free_race_enabled(args):
            points = _free_race_corner_config_points(direction, transition_key, arc, in_heading, out_heading, args)
        else:
            points = _corner_config_points(direction, transition_key, arc, in_heading, out_heading, args)
            if points is None:
                points = _corner_generated_points(direction, arc, in_lane, out_lane, in_heading, out_heading, args, (in_x, in_y), (out_x, out_y))
        for point in points:
            label = str(point["label"])
            s_m = float(point["s_m"])
            lane = float(point["lane_m"])
            x = float(point["x"])
            y = float(point["y"])
            heading = float(point["heading_deg"])
            wp_id = f"{transition_id}_{label}"
            reason = f"corner_{transition_id}_{label}"
            waypoints.append(_make_explicit_waypoint(x, y, heading, s_m, lane, reason, wp_id, "corner"))
    return waypoints


def _build_global_waypoints(direction: str, obstacles: list[dict], args) -> list[Waypoint]:
    planner_obstacles = [] if _free_race_enabled(args) else _navigation_obstacle_slots(obstacles, direction, args)
    decisions = _gate_decisions(direction, planner_obstacles, args)
    waypoints = _gate_waypoints(direction, decisions, args)
    waypoints.extend(_corner_waypoints(direction, decisions, args))
    waypoints.sort(key=lambda wp: wp.s_m)
    return waypoints


def _get_cached_waypoints(direction: str, obstacles: list[dict], args) -> tuple[list[Waypoint], bool]:
    free_race = _free_race_enabled(args)
    obstacle_sig = ("free_race",) if free_race else _obstacle_signature(obstacles, direction, args)
    locked_sig = tuple() if free_race else _locked_gate_signature()
    signature = (
        obstacle_sig,
        locked_sig,
        _corner_config_path(args).as_posix(),
        _corner_config_mtime(args),
        bool(free_race),
        _post_unpark_bottom_handoff_active(args),
        _free_race_inner_signature(args, direction),
    )
    if _HYBRID_STATE.signature != signature or _HYBRID_STATE.direction != direction or not _HYBRID_STATE.waypoints:
        _HYBRID_STATE.signature = signature
        _HYBRID_STATE.direction = direction
        _HYBRID_STATE.waypoints = _build_global_waypoints(direction, obstacles, args)
        _HYBRID_STATE.active_waypoint_id = ""
        _reset_waypoint_progress("", 0.0)
        _HYBRID_STATE.replan_count += 1
        return _HYBRID_STATE.waypoints, True
    return _HYBRID_STATE.waypoints, False


def _ahead_delta(route_s: float, target_s: float) -> float:
    return _forward_delta(route_s, target_s)


def _waypoint_by_id(waypoints: list[Waypoint], waypoint_id: str) -> Waypoint | None:
    for wp in waypoints:
        if wp.id == waypoint_id:
            return wp
    return None


def _reset_waypoint_progress(waypoint_id: str, route_s: float) -> None:
    _HYBRID_STATE.active_waypoint_id = waypoint_id
    _HYBRID_STATE.active_waypoint_min_distance_m = float("inf")
    _HYBRID_STATE.active_waypoint_last_signed_forward_m = 0.0
    _HYBRID_STATE.active_waypoint_has_sample = False
    _HYBRID_STATE.active_waypoint_seen_approach = False
    _HYBRID_STATE.active_waypoint_line_armed = False
    _HYBRID_STATE.active_waypoint_selected_at_route_s_m = route_s
    _HYBRID_STATE.active_waypoint_distance_m = 0.0
    _HYBRID_STATE.active_waypoint_signed_forward_m = 0.0
    _HYBRID_STATE.active_waypoint_passed_reason = ""
    _HYBRID_STATE.active_waypoint_capture_votes = deque()
    _HYBRID_STATE.active_waypoint_arm_votes = deque()
    _HYBRID_STATE.active_waypoint_pass_line_votes = deque()
    _HYBRID_STATE.active_waypoint_route_votes = deque()


def reset_active_waypoint_after_close_color_recovery(reason: str = "close_color_recovery_done") -> dict:
    previous_id = _HYBRID_STATE.active_waypoint_id
    _reset_waypoint_progress("", _HYBRID_STATE.route_last_s_m)
    _HYBRID_STATE.active_waypoint_reset_reason = str(reason or "close_color_recovery_done")
    _HYBRID_STATE.active_waypoint_reset_previous_id = previous_id
    return {
        "previous_id": previous_id,
        "reason": _HYBRID_STATE.active_waypoint_reset_reason,
    }


def _set_active_waypoint(waypoint: Waypoint, route_s: float) -> None:
    if _HYBRID_STATE.active_waypoint_id != waypoint.id:
        _reset_waypoint_progress(waypoint.id, route_s)


def _lock_passed_gate_waypoint(waypoint: Waypoint, direction: str, args=None) -> None:
    if args is not None and not _navigation_state_mutations_allowed(args):
        _set_gate_lock_suppressed(_navigation_state_mutation_suppressed_reason(args))
        return
    if waypoint.kind != "gate":
        return
    parts = waypoint.id.split("_")
    if len(parts) < 4 or parts[1] != "gate":
        return
    try:
        gate_idx = int(parts[2])
    except ValueError:
        return
    section = parts[0]
    if section not in ORDER_CLOCK:
        return
    key = (section, gate_idx)
    if key in _HYBRID_STATE.locked_gate_choices:
        return
    lane_name = "_".join(parts[3:])
    decision = {
        "lane_name": lane_name,
        "lane_m": waypoint.lane_m,
        "reason": waypoint.reason,
        "obstacle_ids": [waypoint.obstacle_id] if waypoint.obstacle_id else [],
    }
    pending = _HYBRID_STATE.pending_unknown_gate_choices.get(key)
    if pending is not None:
        if not _gate_decision_is_provisional_unknown(decision) and _gate_lane_matches_pending(decision, pending):
            _commit_pending_unknown_gate(key, "retrospective_waypoint_known_match")
            _commit_completed_pending_unknown_sections(direction)
        return
    if _gate_decision_is_provisional_unknown(decision):
        _remember_pending_unknown_gate(section, gate_idx, decision, "waypoint_passed")
        _commit_completed_pending_unknown_sections(direction)
        return
    _lock_gate_decision(section, gate_idx, decision, "waypoint_passed")
    _commit_completed_pending_unknown_sections(direction)


def _waypoint_signed_forward(pose, waypoint: Waypoint) -> float:
    heading = math.radians(float(waypoint.heading_deg))
    return (float(pose.x) - waypoint.x) * math.cos(heading) + (float(pose.y) - waypoint.y) * math.sin(heading)


def _vote(track: deque, value: bool, args) -> tuple[deque, int]:
    window = max(1, int(getattr(args, "navigation_waypoint_consensus_window", 3)))
    if track.maxlen != window:
        existing = list(track)[-window:]
        track = deque(existing, maxlen=window)
    track.append(bool(value))
    return track, sum(1 for item in track if item)


def _vote_required(args) -> int:
    window = max(1, int(getattr(args, "navigation_waypoint_consensus_window", 3)))
    return _clamp(int(getattr(args, "navigation_waypoint_consensus_required", 2)), 1, window)


def _update_waypoint_progress(pose, route: Projection, waypoint: Waypoint, args) -> tuple[bool, str]:
    distance = math.hypot(float(pose.x) - waypoint.x, float(pose.y) - waypoint.y)
    signed_forward = _waypoint_signed_forward(pose, waypoint)
    _HYBRID_STATE.active_waypoint_distance_m = distance
    _HYBRID_STATE.active_waypoint_signed_forward_m = signed_forward
    _HYBRID_STATE.active_waypoint_min_distance_m = min(_HYBRID_STATE.active_waypoint_min_distance_m, distance)
    required = _vote_required(args)
    route_pass_margin = float(getattr(args, "navigation_waypoint_pass_margin_m", 0.12))
    active_delta = _ahead_delta(route.s_m, waypoint.s_m)

    if (
        _post_unpark_bottom_handoff_active(args)
        and waypoint.kind == "gate"
        and waypoint.id.startswith("bottom_gate_")
        and active_delta > REF_LAP_LENGTH_M - route_pass_margin
    ):
        _HYBRID_STATE.active_waypoint_passed_reason = "unpark_handoff_route_progress"
        _HYBRID_STATE.active_waypoint_last_signed_forward_m = signed_forward
        _HYBRID_STATE.active_waypoint_has_sample = True
        return True, "unpark_handoff_route_progress"

    capture_radius = float(getattr(args, "navigation_waypoint_capture_radius_m", 0.08))
    _HYBRID_STATE.active_waypoint_capture_votes, capture_votes = _vote(_HYBRID_STATE.active_waypoint_capture_votes, distance <= capture_radius, args)
    if capture_votes >= required:
        _HYBRID_STATE.active_waypoint_passed_reason = "capture_radius"
        _HYBRID_STATE.active_waypoint_last_signed_forward_m = signed_forward
        _HYBRID_STATE.active_waypoint_has_sample = True
        return True, "capture_radius"

    arm_margin = float(getattr(args, "navigation_waypoint_line_arm_margin_m", 0.05))
    arm_radius = float(getattr(args, "navigation_waypoint_line_arm_radius_m", 0.16))
    if signed_forward <= -arm_margin:
        _HYBRID_STATE.active_waypoint_seen_approach = True
    arm_candidate = _HYBRID_STATE.active_waypoint_seen_approach or distance <= arm_radius
    _HYBRID_STATE.active_waypoint_arm_votes, arm_votes = _vote(_HYBRID_STATE.active_waypoint_arm_votes, arm_candidate, args)
    if arm_votes >= required:
        _HYBRID_STATE.active_waypoint_line_armed = True

    pass_line_margin = float(getattr(args, "navigation_waypoint_pass_line_margin_m", 0.04))
    pass_line_candidate = (
        _HYBRID_STATE.active_waypoint_line_armed
        and signed_forward >= -pass_line_margin
        and (_HYBRID_STATE.active_waypoint_seen_approach or _HYBRID_STATE.active_waypoint_min_distance_m <= arm_radius)
    )
    _HYBRID_STATE.active_waypoint_pass_line_votes, pass_line_votes = _vote(_HYBRID_STATE.active_waypoint_pass_line_votes, pass_line_candidate, args)
    if pass_line_votes >= required:
        _HYBRID_STATE.active_waypoint_passed_reason = "armed_pass_line"
        _HYBRID_STATE.active_waypoint_last_signed_forward_m = signed_forward
        _HYBRID_STATE.active_waypoint_has_sample = True
        return True, "armed_pass_line"

    _HYBRID_STATE.active_waypoint_route_votes, route_votes = _vote(_HYBRID_STATE.active_waypoint_route_votes, active_delta > REF_LAP_LENGTH_M - route_pass_margin, args)
    if route_votes >= required:
        _HYBRID_STATE.active_waypoint_passed_reason = "route_progress"
        _HYBRID_STATE.active_waypoint_last_signed_forward_m = signed_forward
        _HYBRID_STATE.active_waypoint_has_sample = True
        return True, "route_progress"

    _HYBRID_STATE.active_waypoint_passed_reason = ""
    _HYBRID_STATE.active_waypoint_last_signed_forward_m = signed_forward
    _HYBRID_STATE.active_waypoint_has_sample = True
    return False, ""


def _next_waypoint_after(route_s: float, waypoints: list[Waypoint], min_delta: float) -> Waypoint | None:
    ahead = [(wp, _ahead_delta(route_s, wp.s_m)) for wp in waypoints]
    ahead = [(wp, delta) for wp, delta in ahead if delta >= min_delta]
    if not ahead:
        return min(waypoints, key=lambda wp: _ahead_delta(route_s, wp.s_m)) if waypoints else None
    ahead.sort(key=lambda item: item[1])
    return ahead[0][0]


def _ordered_targets_from_active(waypoints: list[Waypoint], active: Waypoint, count: int = 3) -> list[Waypoint]:
    if not waypoints:
        return []
    ordered = sorted(waypoints, key=lambda wp: wp.s_m)
    active_idx = next((idx for idx, wp in enumerate(ordered) if wp.id == active.id), 0)
    targets = []
    for i in range(count):
        wp = ordered[(active_idx + i) % len(ordered)]
        targets.append(wp)
        if wp.kind == "smart_parking" and wp.id.startswith("smart_parking_extension_"):
            break
    return targets


def _next_waypoint_in_order(waypoints: list[Waypoint], active: Waypoint) -> Waypoint | None:
    if not waypoints:
        return None
    ordered = sorted(waypoints, key=lambda wp: wp.s_m)
    active_idx = next((idx for idx, wp in enumerate(ordered) if wp.id == active.id), None)
    if active_idx is None:
        return None
    return ordered[(active_idx + 1) % len(ordered)]


def _local_target_window(route_s: float, waypoints: list[Waypoint], active: Waypoint, args) -> list[Waypoint]:
    if not waypoints:
        return []
    min_count = max(2, int(getattr(args, "navigation_local_path_min_waypoints", 2)))
    max_count = max(min_count, int(getattr(args, "navigation_local_path_max_waypoints", 10)))
    range_m = float(getattr(args, "navigation_lookahead_m", 0.42)) + float(getattr(args, "navigation_local_path_extra_m", 1.5))
    ordered = _ordered_targets_from_active(waypoints, active, max_count)
    selected: list[Waypoint] = []
    for wp in ordered:
        delta = _ahead_delta(route_s, wp.s_m)
        if len(selected) >= min_count and delta > range_m:
            break
        selected.append(wp)
    return selected or ordered[:min_count]


def _select_local_targets(pose, route: Projection, waypoints: list[Waypoint], args) -> list[Waypoint]:
    if not waypoints:
        return []
    active = _waypoint_by_id(waypoints, _HYBRID_STATE.active_waypoint_id)
    if active is None:
        initial_min_ahead = float(getattr(args, "navigation_initial_waypoint_min_ahead_m", 0.10))
        active = _next_waypoint_after(route.s_m, waypoints, initial_min_ahead)
    if active is None:
        active = _make_waypoint(route.s_m + float(getattr(args, "navigation_connector_target_m", 0.90)), float(args.navigation_base_lane_m), route.direction, "generated", waypoint_id="generated")
    _set_active_waypoint(active, route.s_m)
    for _ in range(len(waypoints)):
        passed, _reason = _update_waypoint_progress(pose, route, active, args)
        if not passed:
            break
        _lock_passed_gate_waypoint(active, route.direction, args)
        next_active = _next_waypoint_in_order(waypoints, active)
        if next_active is None or next_active.id == active.id:
            break
        active = next_active
        _reset_waypoint_progress(active.id, route.s_m)
    return _local_target_window(route.s_m, waypoints, active, args)


def _shift_waypoint_lane(wp: Waypoint, shift_m: float, direction: str, args) -> Waypoint:
    if abs(shift_m) <= 1e-6:
        return wp
    lane = _clamp(wp.lane_m + shift_m, float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))
    return _make_waypoint(wp.s_m, lane, direction, wp.reason + ("_shift" if abs(shift_m) > 1e-6 else ""), wp.obstacle_id, wp.speed_hint)


def _steering_samples(args) -> list[float]:
    max_steer = float(getattr(args, "navigation_max_steer_deg", DEFAULT_ROBOT_MAX_STEER_DEG))
    count = max(3, int(getattr(args, "navigation_steering_samples", 7)))
    if count % 2 == 0:
        count += 1
    return [-max_steer + 2.0 * max_steer * i / (count - 1) for i in range(count)]


def _integrate_ackermann(x: float, y: float, h: float, steer_deg: float, distance: float, wheelbase: float) -> tuple[float, float, float, float]:
    steer_rad = math.radians(steer_deg)
    curvature = math.tan(steer_rad) / max(wheelbase, 1e-6)
    if abs(curvature) < 1e-6:
        nx = x + distance * math.cos(h)
        ny = y + distance * math.sin(h)
        nh = h
    else:
        dh = curvature * distance
        radius = 1.0 / curvature
        nx = x + radius * (math.sin(h + dh) - math.sin(h))
        ny = y - radius * (math.cos(h + dh) - math.cos(h))
        nh = _wrap_rad(h + dh)
    return nx, ny, nh, curvature


def _hermite_point(p0: tuple[float, float], p1: tuple[float, float], m0: tuple[float, float], m1: tuple[float, float], t: float) -> tuple[float, float, float, float]:
    t2 = t * t
    t3 = t2 * t
    h00 = 2.0 * t3 - 3.0 * t2 + 1.0
    h10 = t3 - 2.0 * t2 + t
    h01 = -2.0 * t3 + 3.0 * t2
    h11 = t3 - t2
    x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
    y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
    dh00 = 6.0 * t2 - 6.0 * t
    dh10 = 3.0 * t2 - 4.0 * t + 1.0
    dh01 = -6.0 * t2 + 6.0 * t
    dh11 = 3.0 * t2 - 2.0 * t
    dx = dh00 * p0[0] + dh10 * m0[0] + dh01 * p1[0] + dh11 * m1[0]
    dy = dh00 * p0[1] + dh10 * m0[1] + dh01 * p1[1] + dh11 * m1[1]
    return x, y, dx, dy


def _sample_waypoint_connector(start: Waypoint, end: Waypoint, samples: int, tangent_scale: float) -> list[dict]:
    p0 = (start.x, start.y)
    p1 = (end.x, end.y)
    chord = max(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 0.05)
    tangent_len = chord * tangent_scale
    start_h = math.radians(start.heading_deg)
    end_h = math.radians(end.heading_deg)
    m0 = (math.cos(start_h) * tangent_len, math.sin(start_h) * tangent_len)
    m1 = (math.cos(end_h) * tangent_len, math.sin(end_h) * tangent_len)
    points = []
    for i in range(max(2, samples)):
        t = i / max(samples - 1, 1)
        x, y, dx, dy = _hermite_point(p0, p1, m0, m1, t)
        heading = math.degrees(math.atan2(dy, dx)) if abs(dx) + abs(dy) > 1e-9 else start.heading_deg
        points.append(
            {
                "x": x,
                "y": y,
                "heading_deg": _wrap_deg(heading),
                "from_id": start.id,
                "to_id": end.id,
            }
        )
    return points


def _preview_path_from_targets(targets: list[Waypoint], args) -> list[dict]:
    if len(targets) < 2:
        return []
    max_waypoints = max(2, int(getattr(args, "navigation_preview_waypoints", 10)))
    samples = max(3, int(getattr(args, "navigation_preview_segment_samples", 10)))
    tangent_scale = float(getattr(args, "navigation_connector_tangent_scale", 0.65))
    selected = targets[:max_waypoints]
    preview = []
    for idx in range(len(selected) - 1):
        segment = _sample_waypoint_connector(selected[idx], selected[idx + 1], samples, tangent_scale)
        if idx > 0:
            segment = segment[1:]
        preview.extend(segment)
    return preview


def _last_reached_waypoint_id(path: list[dict], targets: list[Waypoint], threshold_m: float = 0.035) -> str:
    if not path or not targets:
        return ""
    reached = ""
    for target in targets:
        best = min(math.hypot(float(point["x"]) - target.x, float(point["y"]) - target.y) for point in path)
        if best <= threshold_m:
            reached = target.id
        else:
            break
    return reached


def _targets_from_waypoint(waypoints: list[Waypoint], waypoint_id: str, count: int) -> list[Waypoint]:
    if not waypoints or not waypoint_id:
        return []
    ordered = sorted(waypoints, key=lambda wp: wp.s_m)
    idx = next((i for i, wp in enumerate(ordered) if wp.id == waypoint_id), None)
    if idx is None:
        return []
    targets = []
    for offset in range(max(0, count)):
        wp = ordered[(idx + offset) % len(ordered)]
        targets.append(wp)
        if wp.kind == "smart_parking" and wp.id.startswith("smart_parking_extension_"):
            break
    return targets


def _previous_waypoint(waypoints: list[Waypoint], active: Waypoint) -> Waypoint | None:
    if not waypoints:
        return None
    ordered = sorted(waypoints, key=lambda wp: wp.s_m)
    active_idx = next((idx for idx, wp in enumerate(ordered) if wp.id == active.id), None)
    if active_idx is None:
        return None
    return ordered[(active_idx - 1) % len(ordered)]


def _sample_fixed_waypoint_route(
    waypoints: list[Waypoint],
    route: Projection,
    obstacles: list[dict],
    args,
) -> tuple[list[dict], bool, float, list[str]]:
    if len(waypoints) < 2:
        return [], False, 1_000_000.0, ["fixed_route_too_few_waypoints"]
    sample_step = max(0.015, float(getattr(args, "navigation_route_sample_step_m", 0.035)))
    tangent_scale = float(getattr(args, "navigation_local_segment_tangent_scale", 1.1))
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    points: list[dict] = []
    distance = 0.0
    cost = 0.0
    warnings: list[str] = []
    last_x = float(waypoints[0].x)
    last_y = float(waypoints[0].y)
    last_h = math.radians(float(waypoints[0].heading_deg))

    for idx in range(len(waypoints) - 1):
        start = waypoints[idx]
        end = waypoints[idx + 1]
        chord = max(math.hypot(end.x - start.x, end.y - start.y), sample_step)
        samples = max(3, int(math.ceil(chord / sample_step)) + 1)
        segment = _sample_waypoint_connector(start, end, samples, tangent_scale)
        if idx > 0:
            segment = segment[1:]
        for point_idx, point in enumerate(segment):
            x = float(point["x"])
            y = float(point["y"])
            heading = math.radians(float(point["heading_deg"]))
            if points or point_idx > 0:
                distance += math.hypot(x - last_x, y - last_y)
            step_dist = max(math.hypot(x - last_x, y - last_y), 1e-6)
            curvature = _wrap_rad(heading - last_h) / step_dist if points else 0.0
            steer_deg = math.degrees(math.atan(curvature * wheelbase))
            proj = _project_to_direction(x, y, math.degrees(heading), route.direction)
            progress = _forward_delta(route.s_m, proj.s_m)
            if progress > REF_LAP_LENGTH_M * 0.5:
                progress -= REF_LAP_LENGTH_M
            field_cost, field_ok = _field_cost(x, y, args)
            if not field_ok:
                warnings.append("fixed_route_field_clearance")
                return points, False, 1_000_000.0, sorted(set(warnings))[:20]
            obs_cost = 0.0
            if progress >= -0.05:
                obs_cost, obs_ok, obs_warnings = _obstacle_cost(x, y, progress, proj.lateral_m, obstacles, route.direction, args)
                warnings.extend(obs_warnings)
                if not obs_ok:
                    warnings.append("fixed_route_obstacle")
                    return points, False, 1_000_000.0, sorted(set(warnings))[:20]
            cost += field_cost + obs_cost + 0.04 * curvature * curvature
            points.append(
                {
                    "x": x,
                    "y": y,
                    "s_m": proj.s_m,
                    "distance_m": distance,
                    "progress_m": progress,
                    "lateral_m": proj.lateral_m,
                    "section": proj.section,
                    "primitive": proj.primitive,
                    "local_s_m": proj.local_s_m,
                    "heading_deg": math.degrees(heading),
                    "steer_deg": steer_deg,
                    "curvature": curvature,
                    "cost": cost,
                    "target_reason": end.reason,
                    "from_id": start.id,
                    "to_id": end.id,
                }
            )
            last_x, last_y, last_h = x, y, heading
    return points, len(points) >= 2, cost, sorted(set(warnings))[:20]


def _interpolate_route_point(a: dict, b: dict, u: float, wheelbase: float) -> dict:
    u = _clamp(u, 0.0, 1.0)
    x = float(a["x"]) + (float(b["x"]) - float(a["x"])) * u
    y = float(a["y"]) + (float(b["y"]) - float(a["y"])) * u
    heading = _wrap_deg(float(a.get("heading_deg", 0.0)) + _wrap_deg(float(b.get("heading_deg", 0.0)) - float(a.get("heading_deg", 0.0))) * u)
    distance = float(a.get("distance_m", 0.0)) + (float(b.get("distance_m", 0.0)) - float(a.get("distance_m", 0.0))) * u
    progress = float(a.get("progress_m", 0.0)) + (float(b.get("progress_m", 0.0)) - float(a.get("progress_m", 0.0))) * u
    proj = dict(a)
    proj.update(
        {
            "x": x,
            "y": y,
            "heading_deg": heading,
            "distance_m": distance,
            "progress_m": progress,
            "steer_deg": 0.0,
            "curvature": 0.0,
            "cost": float(a.get("cost", 0.0)) + (float(b.get("cost", 0.0)) - float(a.get("cost", 0.0))) * u,
        }
    )
    return proj


def _project_pose_to_fixed_route(pose, route_points: list[dict], args) -> tuple[int, dict, float]:
    if len(route_points) < 2:
        return 0, dict(route_points[0]) if route_points else {}, float("inf")
    window = max(0.05, float(getattr(args, "navigation_route_projection_window_m", 0.80)))
    px, py = float(pose.x), float(pose.y)
    candidates: list[tuple[float, int, dict]] = []
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    for idx in range(1, len(route_points)):
        a = route_points[idx - 1]
        b = route_points[idx]
        mid_progress = 0.5 * (float(a.get("progress_m", 0.0)) + float(b.get("progress_m", 0.0)))
        if abs(mid_progress) > window:
            continue
        ax, ay = float(a["x"]), float(a["y"])
        bx, by = float(b["x"]), float(b["y"])
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        if length2 <= 1e-9:
            u = 0.0
        else:
            u = _clamp(((px - ax) * vx + (py - ay) * vy) / length2, 0.0, 1.0)
        x = ax + vx * u
        y = ay + vy * u
        dist = math.hypot(px - x, py - y)
        candidates.append((dist, idx, _interpolate_route_point(a, b, u, wheelbase)))
    if not candidates:
        for idx, point in enumerate(route_points):
            candidates.append((math.hypot(px - float(point["x"]), py - float(point["y"])), idx, dict(point)))
    dist, idx, point = min(candidates, key=lambda item: item[0])
    return idx, point, dist


def _fixed_route_path_from_projection(pose, route_points: list[dict], args) -> tuple[list[dict], float]:
    if not route_points:
        return [], float("inf")
    start_idx, projected, error = _project_pose_to_fixed_route(pose, route_points, args)
    max_len = max(float(getattr(args, "navigation_lookahead_m", 0.22)) + float(getattr(args, "navigation_local_path_extra_m", 1.5)) + 0.50, 0.50)
    path = [dict(projected)]
    start_distance = float(projected.get("distance_m", 0.0))
    for point in route_points[start_idx:]:
        if float(point.get("distance_m", 0.0)) <= start_distance + 1e-6:
            continue
        next_point = dict(point)
        next_point["distance_m"] = float(point.get("distance_m", 0.0)) - start_distance
        path.append(next_point)
        if next_point["distance_m"] >= max_len:
            break
    path[0]["distance_m"] = 0.0
    if len(path) >= 2:
        wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
        last_h = math.radians(float(path[0].get("heading_deg", pose.h)))
        last_x = float(path[0]["x"])
        last_y = float(path[0]["y"])
        path[0]["curvature"] = 0.0
        path[0]["steer_deg"] = 0.0
        for idx in range(1, len(path)):
            point = path[idx]
            x = float(point["x"])
            y = float(point["y"])
            h = math.radians(float(point.get("heading_deg", math.degrees(last_h))))
            step = max(math.hypot(x - last_x, y - last_y), 1e-6)
            curvature = _wrap_rad(h - last_h) / step
            point["curvature"] = curvature
            point["steer_deg"] = math.degrees(math.atan(curvature * wheelbase))
            last_x, last_y, last_h = x, y, h
    _HYBRID_STATE.fixed_route_projection_error_m = error
    _HYBRID_STATE.fixed_route_projected_s_m = float(projected.get("s_m", 0.0))
    _HYBRID_STATE.fixed_route_projected_heading_deg = float(projected.get("heading_deg", 0.0))
    return path, error


def _fixed_route_follow_path(
    pose,
    route: Projection,
    waypoints: list[Waypoint],
    active: Waypoint,
    targets: list[Waypoint],
    obstacles: list[dict],
    args,
    allow_projection_error: bool = False,
) -> tuple[list[dict], bool, float, list[str]]:
    previous = _previous_waypoint(waypoints, active)
    if previous is None:
        return [], False, 1_000_000.0, ["fixed_route_no_previous_waypoint"]
    ordered = [previous]
    seen = {previous.id}
    for wp in targets:
        if wp.id in seen:
            continue
        ordered.append(wp)
        seen.add(wp.id)
    if len(ordered) < 2:
        return [], False, 1_000_000.0, ["fixed_route_no_targets"]
    route_points, route_ok, route_cost, warnings = _sample_fixed_waypoint_route(ordered, route, obstacles, args)
    if not route_ok:
        return route_points, False, route_cost, warnings
    path, projection_error = _fixed_route_path_from_projection(pose, route_points, args)
    max_error = max(0.02, float(getattr(args, "navigation_route_follow_max_error_m", 0.18)))
    if projection_error > max_error:
        warnings.append("fixed_route_projection_error")
        if not allow_projection_error:
            return path, False, route_cost + 1_000_000.0, sorted(set(warnings))[:20]
        warnings.append("fixed_route_projection_error_bypassed")
    return path, len(path) >= 2, route_cost, sorted(set(warnings))[:20]


def _connect_to_waypoint(pose, route: Projection, target: Waypoint, obstacles: list[dict], args) -> tuple[list[dict], bool, float, list[str], dict]:
    p0 = (float(pose.x), float(pose.y))
    p1 = (target.x, target.y)
    start_h = math.radians(float(pose.h))
    end_h = math.radians(target.heading_deg)
    chord = max(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 0.05)
    target_delta = _forward_delta(route.s_m, target.s_m)
    if target_delta > REF_LAP_LENGTH_M * 0.5:
        target_delta -= REF_LAP_LENGTH_M
    debug = {
        "target": _waypoint_debug_dict(target),
        "target_id": target.id,
        "target_reason": target.reason,
        "start_x_m": p0[0],
        "start_y_m": p0[1],
        "start_h_deg": float(pose.h),
        "start_route_s_m": route.s_m,
        "start_section": route.section,
        "start_lateral_m": route.lateral_m,
        "start_heading_error_deg": route.heading_error_deg,
        "target_route_delta_m": target_delta,
        "target_heading_deg": target.heading_deg,
        "heading_delta_deg": _wrap_deg(target.heading_deg - float(pose.h)),
        "chord_m": chord,
        "attempts": [],
    }
    base_tangent_scale = float(getattr(args, "navigation_connector_tangent_scale", 0.65))
    scale_factors = [1.0, 0.75, 0.5, 0.35, 1.25, 1.5]
    tangent_scales = []
    for factor in scale_factors:
        scale = max(0.05, base_tangent_scale * factor)
        if all(abs(scale - existing) > 1e-6 for existing in tangent_scales):
            tangent_scales.append(scale)
    sample_step = max(0.025, float(getattr(args, "navigation_connector_sample_step_m", 0.055)))
    count = max(8, int(math.ceil(chord / sample_step)) + 1)
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    max_curvature = math.tan(math.radians(float(getattr(args, "navigation_max_steer_deg", DEFAULT_ROBOT_MAX_STEER_DEG)))) / max(wheelbase, 1e-6)
    check_curvature = bool(getattr(args, "navigation_connector_check_curvature", False))
    best_path: list[dict] = []
    best_cost = float("inf")
    best_warnings: list[str] = []
    best_attempt: dict | None = None
    all_warnings: list[str] = []
    for tangent_scale in tangent_scales:
        tangent_len = chord * tangent_scale
        m0 = (math.cos(start_h) * tangent_len, math.sin(start_h) * tangent_len)
        m1 = (math.cos(end_h) * tangent_len, math.sin(end_h) * tangent_len)
        path = []
        warnings: list[str] = []
        attempt = {
            "tangent_scale": tangent_scale,
            "valid": False,
            "reject_reason": "",
            "sample_index": "",
            "sample_count": count,
        }
        cost = 0.0
        last_x = p0[0]
        last_y = p0[1]
        last_h = start_h
        distance = 0.0
        last_progress = 0.0
        valid = True
        max_seen_curvature = 0.0
        for i in range(count):
            t = i / (count - 1)
            if i == 0:
                x, y = p0
                heading = start_h
            else:
                x, y, dx, dy = _hermite_point(p0, p1, m0, m1, t)
                heading = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else last_h
                distance += math.hypot(x - last_x, y - last_y)
            proj = _project_to_direction(x, y, math.degrees(heading), route.direction)
            progress = _forward_delta(route.s_m, proj.s_m)
            if progress > REF_LAP_LENGTH_M * 0.5:
                progress -= REF_LAP_LENGTH_M
            if i > 0 and progress < last_progress - 0.08:
                warnings.append("connector_backtrack")
                attempt.update(
                    {
                        "reject_reason": "connector_backtrack",
                        "sample_index": i,
                        "sample_x_m": x,
                        "sample_y_m": y,
                        "sample_h_deg": math.degrees(heading),
                        "sample_progress_m": progress,
                        "previous_progress_m": last_progress,
                        "backtrack_m": progress - last_progress,
                        "sample_lateral_m": proj.lateral_m,
                        "sample_section": proj.section,
                    }
                )
                valid = False
                break
            field_cost, field_ok = _field_cost(x, y, args)
            if not field_ok:
                inside_track, outer_clearance, inner_clearance = _field_clearance(x, y)
                warnings.append("connector_field_clearance")
                attempt.update(
                    {
                        "reject_reason": "connector_field_clearance",
                        "sample_index": i,
                        "sample_x_m": x,
                        "sample_y_m": y,
                        "sample_h_deg": math.degrees(heading),
                        "sample_progress_m": progress,
                        "sample_lateral_m": proj.lateral_m,
                        "sample_section": proj.section,
                        "inside_track": inside_track,
                        "outer_clearance_m": outer_clearance,
                        "inner_clearance_m": inner_clearance,
                        "required_clearance_m": float(args.navigation_wall_clearance_m),
                    }
                )
                valid = False
                break
            obs_details: list[dict] = []
            obs_cost, obs_ok, obs_warnings = _obstacle_cost(x, y, progress, proj.lateral_m, obstacles, route.direction, args, obs_details)
            warnings.extend(obs_warnings)
            if not obs_ok:
                detail = dict(obs_details[-1]) if obs_details else {}
                reject_reason = obs_warnings[-1] if obs_warnings else "connector_obstacle"
                attempt.update(
                    {
                        "reject_reason": reject_reason,
                        "sample_index": i,
                        "sample_x_m": x,
                        "sample_y_m": y,
                        "sample_h_deg": math.degrees(heading),
                        "sample_progress_m": progress,
                        "sample_lateral_m": proj.lateral_m,
                        "sample_section": proj.section,
                        "obstacle_detail": detail,
                    }
                )
                valid = False
                break
            step_dist = max(math.hypot(x - last_x, y - last_y), 1e-6)
            curvature = _wrap_rad(heading - last_h) / step_dist if i > 0 else 0.0
            max_seen_curvature = max(max_seen_curvature, abs(curvature))
            if check_curvature and abs(curvature) > max_curvature * 1.35:
                warnings.append("connector_curvature")
                attempt.update(
                    {
                        "reject_reason": "connector_curvature",
                        "sample_index": i,
                        "sample_x_m": x,
                        "sample_y_m": y,
                        "sample_h_deg": math.degrees(heading),
                        "sample_progress_m": progress,
                        "sample_lateral_m": proj.lateral_m,
                        "sample_section": proj.section,
                        "curvature": curvature,
                        "max_allowed_curvature": max_curvature * 1.35,
                    }
                )
                valid = False
                break
            steer_deg = math.degrees(math.atan(curvature * wheelbase))
            cost += field_cost + obs_cost + 0.03 * abs(_wrap_deg(math.degrees(heading) - proj.nominal_heading_deg)) + 0.12 * curvature * curvature
            path.append(
                {
                    "x": x,
                    "y": y,
                    "s_m": proj.s_m,
                    "distance_m": distance,
                    "progress_m": progress,
                    "lateral_m": proj.lateral_m,
                    "section": proj.section,
                    "primitive": proj.primitive,
                    "local_s_m": proj.local_s_m,
                    "heading_deg": math.degrees(heading),
                    "steer_deg": steer_deg,
                    "curvature": curvature,
                    "cost": cost,
                    "target_reason": target.reason,
                    "connector_tangent_scale": tangent_scale,
                }
            )
            last_x, last_y, last_h, last_progress = x, y, heading, progress
        all_warnings.extend(warnings)
        if valid and len(path) >= 2:
            total_cost = cost + 0.05 * max_seen_curvature + 0.01 * abs(tangent_scale - base_tangent_scale)
            attempt.update(
                {
                    "valid": True,
                    "cost": total_cost,
                    "path_points": len(path),
                    "max_curvature": max_seen_curvature,
                    "warnings": sorted(set(warnings))[:10],
                }
            )
            if total_cost < best_cost:
                best_path = path
                best_cost = total_cost
                best_warnings = warnings
                best_attempt = dict(attempt)
        elif valid:
            attempt.update({"reject_reason": "connector_too_few_points", "path_points": len(path)})
        debug["attempts"].append(attempt)
    failure_reasons = sorted({str(item.get("reject_reason", "")) for item in debug["attempts"] if item.get("reject_reason")})
    debug["attempt_count"] = len(debug["attempts"])
    debug["attempt_reject_reasons"] = failure_reasons
    if best_path:
        debug["success"] = True
        debug["failure_reasons"] = []
        debug["best_attempt"] = best_attempt or {}
        return best_path, True, best_cost, sorted(set(best_warnings))[:20], debug
    debug["success"] = False
    debug["failure_reasons"] = failure_reasons
    return best_path, False, 1_000_000.0, sorted(set(all_warnings or ["connector_failed"]))[:20], debug

def _waypoint_preview_from_pose(pose, route: Projection, targets: list[Waypoint], args) -> list[dict]:
    if not targets:
        return []
    samples = max(8, int(math.ceil(float(getattr(args, "navigation_connector_target_m", 0.90)) / max(float(getattr(args, "navigation_connector_sample_step_m", 0.055)), 1e-6))))
    local_samples = max(3, int(getattr(args, "navigation_local_segment_samples", 8)))
    tangent_scale = float(getattr(args, "navigation_connector_tangent_scale", 0.65))
    local_tangent_scale = float(getattr(args, "navigation_local_segment_tangent_scale", 1.1))
    selected = targets
    first = selected[0]
    start = Waypoint(
        id="current_pose",
        x=float(pose.x),
        y=float(pose.y),
        heading_deg=float(pose.h),
        s_m=route.s_m,
        lane_m=route.lateral_m,
        reason="current_pose",
        kind="pose",
    )
    points = _sample_waypoint_connector(start, first, samples, tangent_scale)
    for idx in range(len(selected) - 1):
        segment = _sample_waypoint_connector(selected[idx], selected[idx + 1], local_samples, local_tangent_scale)
        points.extend(segment[1:])
    reason_by_id = {wp.id: wp.reason for wp in selected}
    path = []
    distance = 0.0
    last_x = float(pose.x)
    last_y = float(pose.y)
    last_h = math.radians(float(pose.h))
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    for idx, point in enumerate(points):
        x = float(point["x"])
        y = float(point["y"])
        heading = math.radians(float(point["heading_deg"]))
        if idx > 0:
            distance += math.hypot(x - last_x, y - last_y)
        step_dist = max(math.hypot(x - last_x, y - last_y), 1e-6)
        curvature = _wrap_rad(heading - last_h) / step_dist if idx > 0 else 0.0
        steer_deg = math.degrees(math.atan(curvature * wheelbase))
        proj = _project_to_direction(x, y, math.degrees(heading), route.direction)
        progress = _forward_delta(route.s_m, proj.s_m)
        if progress > REF_LAP_LENGTH_M * 0.5:
            progress -= REF_LAP_LENGTH_M
        path.append(
            {
                "x": x,
                "y": y,
                "s_m": proj.s_m,
                "distance_m": distance,
                "progress_m": progress,
                "lateral_m": proj.lateral_m,
                "section": proj.section,
                "primitive": proj.primitive,
                "local_s_m": proj.local_s_m,
                "heading_deg": math.degrees(heading),
                "steer_deg": steer_deg,
                "curvature": curvature,
                "cost": 0.0,
                "target_reason": reason_by_id.get(str(point.get("to_id", "")), first.reason),
            }
        )
        last_x, last_y, last_h = x, y, heading
    return path


def _extend_path_through_waypoints(
    path: list[dict],
    route: Projection,
    targets: list[Waypoint],
    obstacles: list[dict],
    args,
) -> tuple[list[dict], float, list[str]]:
    if len(path) < 2 or len(targets) < 2:
        return path, 0.0, []
    samples = max(3, int(getattr(args, "navigation_local_segment_samples", 8)))
    tangent_scale = float(getattr(args, "navigation_local_segment_tangent_scale", 1.1))
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    max_curvature = math.tan(math.radians(float(getattr(args, "navigation_max_steer_deg", DEFAULT_ROBOT_MAX_STEER_DEG)))) / max(wheelbase, 1e-6)
    extended = list(path)
    selected = targets
    last = dict(extended[-1])
    last_x = float(last["x"])
    last_y = float(last["y"])
    last_h = math.radians(float(last.get("heading_deg", selected[0].heading_deg)))
    distance = float(last.get("distance_m", 0.0))
    cost = 0.0
    warnings: list[str] = []
    for idx in range(len(selected) - 1):
        start = selected[idx]
        end = selected[idx + 1]
        segment = _sample_waypoint_connector(start, end, samples, tangent_scale)
        committed_len = len(extended)
        segment_cost = 0.0
        segment_points = []
        seg_last_x, seg_last_y, seg_last_h = last_x, last_y, last_h
        seg_distance = distance
        for point in segment[1:]:
            x = float(point["x"])
            y = float(point["y"])
            heading = math.radians(float(point["heading_deg"]))
            step_dist = max(math.hypot(x - seg_last_x, y - seg_last_y), 1e-6)
            seg_distance += step_dist
            proj = _project_to_direction(x, y, math.degrees(heading), route.direction)
            progress = _forward_delta(route.s_m, proj.s_m)
            if progress > REF_LAP_LENGTH_M * 0.5:
                progress -= REF_LAP_LENGTH_M
            field_cost, field_ok = _field_cost(x, y, args)
            if not field_ok:
                warnings.append("local_extension_field_clearance")
                return extended[:committed_len], cost + 1_000_000.0, sorted(set(warnings))[:20]
            obs_cost, obs_ok, obs_warnings = _obstacle_cost(x, y, progress, proj.lateral_m, obstacles, route.direction, args)
            warnings.extend(obs_warnings)
            if not obs_ok:
                warnings.append("local_extension_obstacle")
                return extended[:committed_len], cost + 1_000_000.0, sorted(set(warnings))[:20]
            curvature = _wrap_rad(heading - seg_last_h) / step_dist
            if abs(curvature) > max_curvature * 1.35:
                warnings.append("local_extension_curvature")
                return extended[:committed_len], cost + 1_000_000.0, sorted(set(warnings))[:20]
            steer_deg = math.degrees(math.atan(curvature * wheelbase))
            segment_cost += field_cost + obs_cost + 0.08 * curvature * curvature
            segment_points.append(
                {
                    "x": x,
                    "y": y,
                    "s_m": proj.s_m,
                    "distance_m": seg_distance,
                    "progress_m": progress,
                    "lateral_m": proj.lateral_m,
                    "section": proj.section,
                    "primitive": proj.primitive,
                    "local_s_m": proj.local_s_m,
                    "heading_deg": math.degrees(heading),
                    "steer_deg": steer_deg,
                    "curvature": curvature,
                    "cost": cost + segment_cost,
                    "target_reason": end.reason,
                }
            )
            seg_last_x, seg_last_y, seg_last_h = x, y, heading
        extended.extend(segment_points)
        cost += segment_cost
        last_x, last_y, last_h = seg_last_x, seg_last_y, seg_last_h
        distance = seg_distance
    return extended, cost, sorted(set(warnings))[:20]


def _fallback_local_lattice(pose, route: Projection, obstacles: list[dict], args) -> tuple[list[dict], bool, float, list[str]]:
    fallback_args = SimpleNamespace(**vars(args))
    fallback_args.navigation_horizon_m = min(float(args.navigation_horizon_m), float(getattr(args, "navigation_fallback_lattice_horizon_m", 1.2)))
    fallback_args.navigation_max_states = min(int(getattr(args, "navigation_max_states", 3600)), 600)
    fallback_args.navigation_steering_samples = 3
    fallback_args.navigation_keep_per_bucket = 1
    nodes, best_idx, blocked, warnings = _plan_state_lattice(pose, route, obstacles, fallback_args)
    path = _reconstruct_path(nodes, best_idx, route, args)
    cost = nodes[best_idx].cost if nodes else 0.0
    return path, not blocked and len(path) >= 2, cost, warnings + ["fallback_lattice"]


def _plan_hybrid_waypoint(pose, route: Projection, all_obstacles: list[dict], relevant_obstacles: list[dict], args) -> tuple[list[dict], bool, float, list[str], list[Waypoint], bool, str, dict]:
    projected_obstacles = []
    if not _free_race_enabled(args) and not _all_gates_locked():
        for slot in all_obstacles:
            if not (slot.get("known") or slot.get("current")) or slot.get("section") not in ORDER_CLOCK:
                continue
            if not _start_section_obstacle_slot_allowed(slot, route.direction, args):
                continue
            obs = _project_slot(slot, route.direction)
            delta = _forward_delta(route.s_m, obs["s_m"])
            if delta > REF_LAP_LENGTH_M * 0.5:
                delta -= REF_LAP_LENGTH_M
            obs["delta_m"] = delta
            projected_obstacles.append(obs)
    planner_slots = _navigation_obstacle_slots(all_obstacles, route.direction, args)
    waypoints, replanned = _get_cached_waypoints(route.direction, planner_slots, args)
    waypoints, parking_route_active = _smart_parking_route_waypoints(route, waypoints, all_obstacles, args)
    targets = _select_local_targets(pose, route, waypoints, args)
    active = _waypoint_by_id(waypoints, _HYBRID_STATE.active_waypoint_id) or (targets[0] if targets else None)
    planner_debug = {
        "planner": "hybrid-waypoint",
        "active_waypoint": _waypoint_debug_dict(active),
        "targets": [_waypoint_debug_dict(wp) for wp in targets[:5]],
        "target_ids": [wp.id for wp in targets[:5]],
        "target_reasons": [wp.reason for wp in targets[:5]],
        "projected_obstacle_count": len(projected_obstacles),
        "fixed_route_attempted": False,
        "fixed_route_failed": False,
        "fixed_route_warnings": [],
        "connector": {},
        "extension_warnings": [],
        "blocked_route_fallback": {},
    }
    warnings: list[str] = []
    if parking_route_active:
        warnings.append("smart_parking_route_geometry")
    if targets:
        if bool(getattr(args, "navigation_fixed_route_follow", True)):
            planner_debug["fixed_route_attempted"] = True
            path, ok, cost, fixed_warnings = _fixed_route_follow_path(pose, route, waypoints, active or targets[0], targets, projected_obstacles, args)
            planner_debug["fixed_route_warnings"] = sorted(set(fixed_warnings))[:20]
            planner_debug["fixed_route_cost"] = cost
            planner_debug["fixed_route_path_points"] = len(path)
            warnings.extend(fixed_warnings)
            if ok:
                _reset_blocked_route_fallback("fixed_route")
                planner_debug["result"] = "fixed_route"
                return path, False, cost, sorted(set(warnings))[:20], waypoints, replanned, "fixed_route", planner_debug
            planner_debug["fixed_route_failed"] = True
            if not bool(getattr(args, "navigation_fixed_route_recovery_connector", True)):
                planner_debug["result"] = "fixed_route_blocked"
                return path, True, cost, sorted(set(warnings))[:20], waypoints, replanned, "fixed_route_blocked", planner_debug
        target = targets[0]
        path, ok, cost, target_warnings, connector_debug = _connect_to_waypoint(pose, route, target, projected_obstacles, args)
        planner_debug["connector"] = connector_debug
        warnings.extend(target_warnings)
        if ok:
            _reset_blocked_route_fallback("recovery_connector")
            path, extension_cost, extension_warnings = _extend_path_through_waypoints(path, route, targets, projected_obstacles, args)
            planner_debug["extension_warnings"] = sorted(set(extension_warnings))[:20]
            planner_debug["extension_cost"] = extension_cost
            warnings.extend(extension_warnings)
            planner_debug["result"] = "recovery_connector"
            return path, False, cost + extension_cost, sorted(set(warnings))[:20], waypoints, replanned, "recovery_connector", planner_debug
        if bool(getattr(args, "navigation_fixed_route_follow", True)):
            fallback_status = _update_blocked_route_fallback_state(args)
            planner_debug["blocked_route_fallback"] = fallback_status
            if bool(fallback_status.get("active", False)):
                fallback_path, fallback_ok, fallback_cost, fallback_warnings = _fixed_route_follow_path(
                    pose,
                    route,
                    waypoints,
                    active or targets[0],
                    targets,
                    projected_obstacles,
                    args,
                    allow_projection_error=True,
                )
                fallback_status.update(
                    {
                        "path_points": len(fallback_path),
                        "projection_error_m": _HYBRID_STATE.fixed_route_projection_error_m,
                        "warnings": sorted(set(fallback_warnings))[:20],
                    }
                )
                warnings.extend(fallback_warnings)
                if fallback_ok:
                    warnings.append("blocked_route_fallback")
                    planner_debug["result"] = "blocked_route_fallback"
                    planner_debug["blocked"] = False
                    return fallback_path, False, fallback_cost, sorted(set(warnings))[:20], waypoints, replanned, "blocked_route_fallback", planner_debug
                fallback_status["reason"] = "fallback_path_invalid"
                _HYBRID_STATE.blocked_route_fallback_reason = "fallback_path_invalid"
        else:
            _reset_blocked_route_fallback("fixed_route_follow_disabled")
            planner_debug["blocked_route_fallback"] = {
                "enabled": bool(getattr(args, "navigation_blocked_route_fallback", True)),
                "eligible": False,
                "active": False,
                "age_s": 0.0,
                "delay_s": max(0.0, float(getattr(args, "navigation_blocked_route_fallback_delay_s", 1.0))),
                "reason": "fixed_route_follow_disabled",
            }
    else:
        _reset_blocked_route_fallback("no_targets")
        planner_debug["blocked_route_fallback"] = {
            "enabled": bool(getattr(args, "navigation_blocked_route_fallback", True)),
            "eligible": False,
            "active": False,
            "age_s": 0.0,
            "delay_s": max(0.0, float(getattr(args, "navigation_blocked_route_fallback_delay_s", 1.0))),
            "reason": "no_targets",
        }
    preview_path = _waypoint_preview_from_pose(pose, route, targets, args)
    warnings.append("waypoint_connector_blocked")
    planner_debug["result"] = "blocked_waypoint_preview"
    planner_debug["blocked"] = True
    planner_debug["preview_path_points"] = len(preview_path)
    return preview_path, True, 1_000_000.0, sorted(set(warnings))[:20], waypoints, replanned, "blocked_waypoint_preview", planner_debug

def _section_has_known_obstacle(section: str, obstacles: list[dict]) -> bool:
    for slot in obstacles:
        if slot.get("section") == section and (slot.get("known") or slot.get("current")):
            return True
    return False


def _section_has_known_color_obstacle(section: str, obstacles: list[dict]) -> bool:
    for slot in obstacles:
        if slot.get("section") != section or not (slot.get("known") or slot.get("current")):
            continue
        color = str(slot.get("color") or slot.get("known_color") or slot.get("current_color") or "")
        if color in ("RED", "GREEN"):
            return True
    return False


def _corner_observation_hold(route: Projection, obstacles: list[dict], args, full_gate_lock: bool) -> tuple[bool, str, str, float, bool]:
    if full_gate_lock or _free_race_enabled(args) or bool(getattr(args, "navigation_party_mode", False)):
        _HYBRID_STATE.corner_observation_until = 0.0
        _HYBRID_STATE.corner_observation_section = ""
        _clear_corner_observation_handoff()
        return False, "", "", 0.0, False
    if not bool(getattr(args, "navigation_corner_observation_stop", True)):
        _clear_corner_observation_handoff()
        return False, "", "", 0.0, False
    now = time.monotonic()
    if _HYBRID_STATE.corner_observation_direction != route.direction:
        _HYBRID_STATE.corner_observation_until = 0.0
        _HYBRID_STATE.corner_observation_section = ""
        _HYBRID_STATE.corner_observation_direction = route.direction
        _HYBRID_STATE.corner_observed_sections.clear()
        _clear_corner_observation_handoff()

    active_section = _HYBRID_STATE.corner_observation_section
    if active_section:
        if _section_has_known_color_obstacle(active_section, obstacles) or now >= _HYBRID_STATE.corner_observation_until:
            _HYBRID_STATE.corner_observed_sections.add(f"{route.direction}:{active_section}")
            _arm_corner_observation_handoff(active_section, route.direction, now, args)
            _HYBRID_STATE.corner_observation_until = 0.0
            _HYBRID_STATE.corner_observation_section = ""
            return False, "corner_observation_done", active_section, 0.0, False
        return True, "corner_observation", active_section, max(0.0, _HYBRID_STATE.corner_observation_until - now), False

    if route.primitive.endswith("_straight"):
        target_section = _next_section(route.section, route.direction)
        key = f"{route.direction}:{target_section}"
        prim = _primitive_by_name(route.direction, route.primitive)
        remaining = max(0.0, (prim.length_m if prim is not None else STRAIGHT_LEN_M) - route.local_s_m)
        trigger = max(0.0, float(getattr(args, "navigation_corner_observation_lidar_forward_m", 0.0975))) + max(0.0, float(getattr(args, "navigation_corner_observation_trigger_margin_m", 0.02)))
        approaching = remaining <= trigger + max(0.0, float(getattr(args, "navigation_corner_observation_approach_m", 0.35)))
        should_observe = target_section and key not in _HYBRID_STATE.corner_observed_sections and not _section_has_known_color_obstacle(target_section, obstacles)
        if should_observe and remaining <= trigger:
            lidar_rate = max(1.0, float(getattr(args, "navigation_corner_observation_lidar_hz", 15.0)))
            scans = max(1, int(getattr(args, "navigation_corner_observation_scans", 10)))
            _HYBRID_STATE.corner_observation_section = target_section
            _HYBRID_STATE.corner_observation_until = now + scans / lidar_rate
            return True, "corner_observation", target_section, scans / lidar_rate, False
        if should_observe and approaching:
            return False, "corner_observation_approach", target_section, 0.0, True

    return False, "", "", 0.0, False


def _color_confirmation_hold(
    route: Projection,
    obstacles: list[dict],
    pose,
    args,
    full_gate_lock: bool,
    corner_hold_active: bool = False,
    corner_hold_section: str = "",
    corner_hold_reason: str = "",
    measurement_obstacles: list[dict] | None = None,
    post_unpark_obstacles: list[dict] | None = None,
) -> tuple[bool, str, str, float, bool, bool, dict]:
    now = time.monotonic()
    post_unpark_source_obstacles = post_unpark_obstacles if post_unpark_obstacles is not None else obstacles
    post_unpark_source = "planner_obstacles" if post_unpark_obstacles is not None else "relevant"
    post_unpark_status = _post_unpark_start_confirmation_status(
        route,
        post_unpark_source_obstacles,
        args,
        source=post_unpark_source,
        relevant_obstacles=obstacles,
    )
    if full_gate_lock or not bool(getattr(args, "navigation_color_confirmation_stop", True)):
        if _HYBRID_STATE.color_confirmation_slot:
            _clear_color_confirmation_state()
        return False, "", "", 0.0, False, False, {"post_unpark_start_confirmation": post_unpark_status}
    if not _navigation_confirmations_active(args):
        _clear_color_confirmation_state()
        return False, "", "", 0.0, False, False, {"suppressed": True, "reason": _navigation_confirmations_active_reason(args), "post_unpark_start_confirmation": post_unpark_status}
    status: dict = {}
    if _HYBRID_STATE.color_confirmation_direction != route.direction:
        _HYBRID_STATE.color_confirmation_until = 0.0
        _HYBRID_STATE.color_confirmation_slot = ""
        _HYBRID_STATE.color_confirmation_section = ""
        _HYBRID_STATE.color_confirmation_proposal = ""
        _HYBRID_STATE.color_confirmation_mode = ""
        _HYBRID_STATE.color_confirmation_direction = route.direction
        _HYBRID_STATE.color_confirmation_committed_start = ""
        _HYBRID_STATE.color_confirmation_start_t = 0.0
        _HYBRID_STATE.color_confirmation_timeout_t = 0.0
        _HYBRID_STATE.color_confirmation_samples.clear()
        _HYBRID_STATE.color_confirmation_all_samples.clear()
        _HYBRID_STATE.color_confirmation_last_sample_key = ""
        _HYBRID_STATE.color_confirmation_seen_measurement_ids.clear()
        _HYBRID_STATE.color_confirmation_decision.clear()
        _HYBRID_STATE.color_confirmation_hold_counts.clear()
        _HYBRID_STATE.color_confirmation_deadline_holds.clear()
        _HYBRID_STATE.color_confirmation_trigger_source = ""
        _HYBRID_STATE.color_confirmation_deadline_key = ""
        _HYBRID_STATE.color_confirmation_deadline_section = ""
        _HYBRID_STATE.color_confirmation_deadline_gate_idx = None
        _HYBRID_STATE.color_confirmation_obstacle_gate_idx = None
        _HYBRID_STATE.color_confirmation_deadline_delta_m = 0.0
        _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m = 0.0
        _HYBRID_STATE.color_confirmation_deadline_pre_gate_m = 0.0
        _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m = 0.0
        _HYBRID_STATE.color_confirmation_deadline_bypass_used = False
        _HYBRID_STATE.color_confirmation_blocked_slot = ""
        _HYBRID_STATE.color_confirmation_blocked_reason = ""
        _HYBRID_STATE.color_confirmation_recovery_target_m = 0.0
        _HYBRID_STATE.color_confirmation_recovery_trigger_distance_m = 0.0
        _HYBRID_STATE.color_confirmation_recovery_reason = ""
        _HYBRID_STATE.color_confirmation_attempt = 0
        _HYBRID_STATE.color_confirmation_gain = 0.0
        _HYBRID_STATE.color_confirmation_base_gain = 0.0
        _HYBRID_STATE.color_confirmation_settle_until_t = 0.0
        _HYBRID_STATE.color_confirmation_exposure_reason = ""
        _HYBRID_STATE.color_confirmation_measurement_debug.clear()

    if _HYBRID_STATE.color_confirmation_blocked_slot:
        return (
            False,
            _HYBRID_STATE.color_confirmation_blocked_reason or "color_conflict_close",
            _HYBRID_STATE.color_confirmation_section,
            0.0,
            False,
            True,
            {
                "slot_id": _HYBRID_STATE.color_confirmation_blocked_slot,
                "proposal": _HYBRID_STATE.color_confirmation_proposal,
                "mode": _HYBRID_STATE.color_confirmation_mode,
                "trigger_source": _HYBRID_STATE.color_confirmation_trigger_source,
                "deadline_key": _HYBRID_STATE.color_confirmation_deadline_key,
                "deadline_section": _HYBRID_STATE.color_confirmation_deadline_section,
                "deadline_gate_idx": _HYBRID_STATE.color_confirmation_deadline_gate_idx,
                "obstacle_gate_idx": _HYBRID_STATE.color_confirmation_obstacle_gate_idx,
                "deadline_delta_m": _HYBRID_STATE.color_confirmation_deadline_delta_m,
                "deadline_trigger_delta_m": _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m,
                "deadline_pre_gate_m": _HYBRID_STATE.color_confirmation_deadline_pre_gate_m,
                "deadline_trigger_margin_m": _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m,
                "deadline_bypass_used": _HYBRID_STATE.color_confirmation_deadline_bypass_used,
                "blocked": True,
                "recovery_target_m": _HYBRID_STATE.color_confirmation_recovery_target_m,
                "recovery_trigger_distance_m": _HYBRID_STATE.color_confirmation_recovery_trigger_distance_m,
                "recovery_reason": _HYBRID_STATE.color_confirmation_recovery_reason,
                "recovery_decision_reason": "armed_confirmed_side_change",
                "camera_gain_request": _color_confirmation_restore_gain_request(args, "blocked"),
            },
        )

    active_slot = _HYBRID_STATE.color_confirmation_slot
    if active_slot:
        active_obs = next((obs for obs in obstacles if str(obs.get("id", "")) == active_slot), None)
        measurement_source_obs = active_obs
        measurement_source = "relevant" if active_obs is not None else "none"
        relevant_measurements = list(active_obs.get("color_measurements", []) or []) if active_obs is not None else []
        full_obs = None
        full_measurements: list[dict] = []
        if measurement_obstacles is not None:
            full_obs = next((obs for obs in measurement_obstacles if str(obs.get("id", "")) == active_slot), None)
            if full_obs is not None:
                full_measurements = list(full_obs.get("color_measurements", []) or [])
        decision_obs = active_obs if active_obs is not None else full_obs
        decision_obstacle_source = "relevant" if active_obs is not None else "planner_obstacles" if full_obs is not None else "none"
        committed = str(decision_obs.get("color", "")) if decision_obs is not None else ""
        if (active_obs is None or not relevant_measurements) and full_obs is not None:
            measurement_source_obs = full_obs
            measurement_source = "planner_obstacles"
        proposal = _HYBRID_STATE.color_confirmation_proposal
        mode = _HYBRID_STATE.color_confirmation_mode
        stationary, speed_mps, yaw_rate_dps = _color_confirmation_stationary(pose, args)
        sample_added = False
        sample_added_count = 0
        measurement_debug = {
            "active_obstacle_in_relevant": active_obs is not None,
            "active_obstacle_in_measurement_source": measurement_source_obs is not None,
            "active_obstacle_in_decision_source": decision_obs is not None,
            "decision_obstacle_source": decision_obstacle_source,
            "measurement_source": measurement_source,
            "measurement_blocked_reason": "",
            "measurement_relevant_events_available": len(relevant_measurements),
            "measurement_full_events_available": len(full_measurements),
            "measurement_events_available": 0,
            "measurement_events_added": 0,
            "measurement_events_duplicate": 0,
            "measurement_events_not_real": 0,
            "measurement_events_stale": 0,
            "measurement_events_before_hold": 0,
            "measurement_events_gain_settle": 0,
            "measurement_events_missing_id": 0,
            "last_measurement_id": "",
            "last_sample_label": "",
            "last_skip_reason": "",
        }
        if measurement_source_obs is None:
            measurement_debug["measurement_blocked_reason"] = "active_slot_missing"
        elif not stationary:
            measurement_debug["measurement_blocked_reason"] = "not_stationary"
            measurement_debug["measurement_events_available"] = len(
                full_measurements if measurement_source == "planner_obstacles" else relevant_measurements
            )
        else:
            measurements = list(measurement_source_obs.get("color_measurements", []) or [])
            measurements.sort(key=lambda item: (_measurement_float(item, "result_t", 0.0), _measurement_float(item, "request_t", 0.0), str(item.get("measurement_id", ""))))
            measurement_debug["measurement_events_available"] = len(measurements)
            for measurement in measurements:
                sample, reason = _color_confirmation_measurement_sample(dict(measurement), now, args)
                if sample is None:
                    if reason == "duplicate":
                        measurement_debug["measurement_events_duplicate"] += 1
                    elif reason == "not_real":
                        measurement_debug["measurement_events_not_real"] += 1
                    elif reason == "stale_frame":
                        measurement_debug["measurement_events_stale"] += 1
                    elif reason == "before_hold":
                        measurement_debug["measurement_events_before_hold"] += 1
                    elif reason == "gain_settle":
                        measurement_debug["measurement_events_gain_settle"] += 1
                    elif reason == "missing_id":
                        measurement_debug["measurement_events_missing_id"] += 1
                    measurement_debug["last_skip_reason"] = reason
                    continue
                _HYBRID_STATE.color_confirmation_seen_measurement_ids.add(sample["measurement_id"])
                _HYBRID_STATE.color_confirmation_last_sample_key = sample["measurement_id"]
                _HYBRID_STATE.color_confirmation_samples.append(sample)
                _HYBRID_STATE.color_confirmation_all_samples.append(dict(sample))
                measurement_debug["last_measurement_id"] = sample["measurement_id"]
                measurement_debug["last_sample_label"] = sample["label"]
                sample_added_count += 1
            sample_added = sample_added_count > 0
            measurement_debug["measurement_events_added"] = sample_added_count
        _HYBRID_STATE.color_confirmation_measurement_debug = dict(measurement_debug)
        timed_out = now >= _HYBRID_STATE.color_confirmation_timeout_t > 0.0
        decision = _color_confirmation_decision(_HYBRID_STATE.color_confirmation_samples, args, timed_out)
        counts = _color_confirmation_counts(_HYBRID_STATE.color_confirmation_samples)
        all_counts = _color_confirmation_counts(_HYBRID_STATE.color_confirmation_all_samples)
        remaining = max(0.0, _HYBRID_STATE.color_confirmation_timeout_t - now) if _HYBRID_STATE.color_confirmation_timeout_t else 0.0
        status = {
            "slot_id": active_slot,
            "proposal": proposal,
            "committed": committed,
            "committed_start": _HYBRID_STATE.color_confirmation_committed_start,
            "mode": mode,
            "trigger_source": _HYBRID_STATE.color_confirmation_trigger_source,
            "deadline_key": _HYBRID_STATE.color_confirmation_deadline_key,
            "deadline_section": _HYBRID_STATE.color_confirmation_deadline_section,
            "deadline_gate_idx": _HYBRID_STATE.color_confirmation_deadline_gate_idx,
            "obstacle_gate_idx": _HYBRID_STATE.color_confirmation_obstacle_gate_idx,
            "deadline_delta_m": _HYBRID_STATE.color_confirmation_deadline_delta_m,
            "deadline_trigger_delta_m": _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m,
            "deadline_pre_gate_m": _HYBRID_STATE.color_confirmation_deadline_pre_gate_m,
            "deadline_trigger_margin_m": _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m,
            "deadline_bypass_used": _HYBRID_STATE.color_confirmation_deadline_bypass_used,
            "blocked": False,
            "sample_count": len(_HYBRID_STATE.color_confirmation_samples),
            "all_sample_count": len(_HYBRID_STATE.color_confirmation_all_samples),
            "counts": counts,
            "all_counts": all_counts,
            "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
            "camera_gain_request": _color_confirmation_gain_request(args),
            "gain_settle_remaining_s": max(0.0, float(_HYBRID_STATE.color_confirmation_settle_until_t or 0.0) - now),
            "exposure_retry_reason": _HYBRID_STATE.color_confirmation_exposure_reason,
            "stationary": stationary,
            "stationary_speed_mps": speed_mps,
            "stationary_yaw_rate_dps": yaw_rate_dps,
            "sample_added": sample_added,
            "sample_added_count": sample_added_count,
            **measurement_debug,
            "timeout_remaining_s": remaining,
            "post_unpark_start_confirmation": post_unpark_status,
            "exposure_retry": False,
            "exposure_retry_candidate": False,
            "exposure_retry_eval_reason": "",
            "exposure_retry_sample_count": "",
            "exposure_retry_min_samples": "",
            "exposure_retry_color_vote_count": "",
            "exposure_retry_color_ratio": "",
            "exposure_retry_required_vote_count": "",
            "exposure_retry_bright_vote_count": "",
            "exposure_retry_dark_vote_count": "",
            "exposure_retry_low_valid_count": "",
            "exposure_retry_current_gain": "",
            "exposure_retry_next_gain": "",
        }
        if timed_out and decision and str(decision.get("source", "")) == "timeout_fallback_green":
            max_retries = max(0, int(getattr(args, "navigation_color_confirmation_exposure_retries", 2)))
            exposure_status = _color_confirmation_exposure_retry(_HYBRID_STATE.color_confirmation_samples, args)
            exposure_retry_candidate = bool(exposure_status.get("retry", False))
            exposure_reason = str(exposure_status.get("reason", "") or "")
            current_gain = float(exposure_status.get("current_gain", _HYBRID_STATE.color_confirmation_gain or getattr(args, "camera_analog_gain", 14.0)) or 0.0)
            next_gain = float(exposure_status.get("next_gain", current_gain) or current_gain)
            status.update(
                {
                    "exposure_retry_candidate": exposure_retry_candidate,
                    "exposure_retry_eval_reason": exposure_reason,
                    "exposure_retry_reason": exposure_reason,
                    "exposure_retry_sample_count": exposure_status.get("sample_count", ""),
                    "exposure_retry_min_samples": exposure_status.get("min_samples", ""),
                    "exposure_retry_color_vote_count": exposure_status.get("color_vote_count", ""),
                    "exposure_retry_color_ratio": exposure_status.get("color_ratio", ""),
                    "exposure_retry_required_vote_count": exposure_status.get("required_vote_count", ""),
                    "exposure_retry_bright_vote_count": exposure_status.get("bright_vote_count", ""),
                    "exposure_retry_dark_vote_count": exposure_status.get("dark_vote_count", ""),
                    "exposure_retry_low_valid_count": exposure_status.get("low_valid_count", ""),
                    "exposure_retry_current_gain": current_gain,
                    "exposure_retry_next_gain": next_gain,
                }
            )
            if int(_HYBRID_STATE.color_confirmation_attempt) < max_retries:
                if exposure_retry_candidate and abs(next_gain - current_gain) > 1e-6:
                    _HYBRID_STATE.color_confirmation_attempt += 1
                    _HYBRID_STATE.color_confirmation_gain = next_gain
                    _HYBRID_STATE.color_confirmation_exposure_reason = exposure_reason
                    _HYBRID_STATE.color_confirmation_samples.clear()
                    _HYBRID_STATE.color_confirmation_last_sample_key = ""
                    _HYBRID_STATE.color_confirmation_timeout_t = now + max(0.0, float(getattr(args, "navigation_color_confirmation_timeout_s", 4.0)))
                    _HYBRID_STATE.color_confirmation_until = _HYBRID_STATE.color_confirmation_timeout_t
                    _HYBRID_STATE.color_confirmation_settle_until_t = now + max(0.0, float(getattr(args, "navigation_color_confirmation_gain_settle_s", 0.25)))
                    status.update(
                        {
                            "decision": {},
                            "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
                            "exposure_retry": True,
                            "counts": {"RED": 0, "GREEN": 0, "OTHER": 0},
                            "sample_count": 0,
                            "timeout_remaining_s": max(0.0, _HYBRID_STATE.color_confirmation_timeout_t - now),
                            "camera_gain_request": _color_confirmation_gain_request(args, exposure_reason),
                            "gain_settle_remaining_s": max(0.0, _HYBRID_STATE.color_confirmation_settle_until_t - now),
                        }
                    )
                    return True, "color_confirmation", _HYBRID_STATE.color_confirmation_section, status["timeout_remaining_s"], False, False, status
            decision = _color_confirmation_decision(_HYBRID_STATE.color_confirmation_all_samples, args, True)
            status["decision_from_all_attempts"] = True
        if not decision and not timed_out:
            return True, "color_confirmation", _HYBRID_STATE.color_confirmation_section, remaining, False, False, {
                **status,
            }
        if decision:
            _HYBRID_STATE.color_confirmation_decision = dict(decision)
            decided_color = str(decision.get("color", ""))
            source = str(decision.get("source", ""))
            confidence = float(decision.get("confidence", 0.0) or 0.0)
            if (
                str(_HYBRID_STATE.color_confirmation_trigger_source or "") == "post_unpark_start_section"
                and source == "timeout_fallback_green"
            ):
                section = _HYBRID_STATE.color_confirmation_section
                status.update(
                    {
                        "decision": decision,
                        "decision_rejected": True,
                        "decision_reject_reason": "post_unpark_fallback_disallowed",
                        "decision_color": decided_color,
                        "decision_source": source,
                        "decision_confidence": confidence,
                        "camera_gain_request": _color_confirmation_restore_gain_request(args, "post_unpark_unresolved"),
                    }
                )
                _clear_color_confirmation_state()
                return False, "color_confirmation_unresolved", section, 0.0, False, False, status
            commit = {
                "slot_id": active_slot,
                "color": decided_color,
                "confidence": confidence,
                "source": source,
                "mode": mode,
                "counts": dict(decision.get("counts", counts)),
            }
            status.update({"decision": decision, "commit_color": commit, "committed": decided_color})
            status["camera_gain_request"] = _color_confirmation_restore_gain_request(args, "confirmation_done")
            recovery_target_m = 0.0
            recovery_route_delta_m = float("inf")
            recovery_los_distance_m = float("inf")
            recovery_side_changed = False
            recovery_side_status: dict = {}
            recovery_gate_locked = False
            recovery_gate_idx = None
            recovery_section_key = ""
            recovery_decision_reason = "invalid_decision_color"
            post_unpark_reverse_suppressed = False
            if decision_obs is not None and decided_color in ("RED", "GREEN"):
                recovery_route_delta_m = max(0.0, float(decision_obs.get("delta_m", 0.0) or 0.0))
                obs_x = float(decision_obs["x"]) if decision_obs.get("x") is not None else float(pose.x)
                obs_y = float(decision_obs["y"]) if decision_obs.get("y") is not None else float(pose.y)
                recovery_los_distance_m = math.hypot(
                    obs_x - float(pose.x),
                    obs_y - float(pose.y),
                )
                start_color = str(_HYBRID_STATE.color_confirmation_committed_start or "")
                recovery_side_changed, recovery_side_status = _color_confirmation_route_side_changed(
                    decision_obs,
                    start_color,
                    decided_color,
                    route,
                    args,
                )
                recovery_gate_idx = _gate_index_from_along(_slot_along_m(decision_obs))
                recovery_section_key = str(decision_obs.get("section", _HYBRID_STATE.color_confirmation_section) or "")
                recovery_gate_locked = (recovery_section_key, recovery_gate_idx) in _HYBRID_STATE.locked_gate_choices
                if recovery_side_changed and not recovery_gate_locked:
                    if str(_HYBRID_STATE.color_confirmation_trigger_source or "") == "post_unpark_start_section":
                        recovery_target_m = 0.0
                        recovery_decision_reason = "post_unpark_reverse_suppressed"
                        post_unpark_reverse_suppressed = True
                    else:
                        recovery_target_m = _color_confirmation_recovery_distance(recovery_los_distance_m, args)
                        recovery_decision_reason = "armed_confirmed_side_change" if recovery_target_m > 0.0 else "zero_target_distance_curve"
                elif recovery_gate_locked:
                    recovery_decision_reason = "gate_locked"
                else:
                    recovery_decision_reason = "no_side_change"
            elif decision_obs is None:
                recovery_decision_reason = "missing_active_obstacle"
            status.update(
                {
                    "decision_color": decided_color,
                    "decision_source": source,
                    "decision_confidence": confidence,
                    "committed_at_decision": committed,
                    "delta_m_at_decision": recovery_route_delta_m if math.isfinite(recovery_route_delta_m) else None,
                    "los_distance_m_at_decision": recovery_los_distance_m if math.isfinite(recovery_los_distance_m) else None,
                    "section": recovery_section_key or _HYBRID_STATE.color_confirmation_section,
                    "gate_idx": recovery_gate_idx,
                    "recovery_side_changed": recovery_side_changed,
                    "recovery_gate_locked": recovery_gate_locked,
                    "post_unpark_reverse_suppressed": post_unpark_reverse_suppressed,
                    "recovery_trigger_distance_m": recovery_los_distance_m if math.isfinite(recovery_los_distance_m) else None,
                    "recovery_target_m": recovery_target_m,
                    "recovery_reason": "confirmed_side_change" if recovery_target_m > 0.0 else "",
                    "recovery_decision_reason": recovery_decision_reason,
                    "recovery_side": recovery_side_status,
                    "recovery_side_from": recovery_side_status.get("start_pass_side", ""),
                    "recovery_side_to": recovery_side_status.get("decided_pass_side", ""),
                    "recovery_start_lane_m": recovery_side_status.get("start_lane_m", ""),
                    "recovery_decided_lane_m": recovery_side_status.get("decided_lane_m", ""),
                    "recovery_lane_delta_m": recovery_side_status.get("lane_delta_m", ""),
                }
            )
            if recovery_target_m > 0.0:
                _HYBRID_STATE.color_confirmation_blocked_slot = active_slot
                _HYBRID_STATE.color_confirmation_blocked_reason = "color_conflict_close"
                _HYBRID_STATE.color_confirmation_recovery_target_m = recovery_target_m
                _HYBRID_STATE.color_confirmation_recovery_trigger_distance_m = recovery_los_distance_m
                _HYBRID_STATE.color_confirmation_recovery_reason = "confirmed_side_change"
                _HYBRID_STATE.color_confirmation_until = 0.0
                _HYBRID_STATE.color_confirmation_slot = ""
                status["blocked"] = True
                return False, "color_conflict_close", _HYBRID_STATE.color_confirmation_section, 0.0, False, True, status
            section = _HYBRID_STATE.color_confirmation_section
            _clear_color_confirmation_state()
            return False, "color_confirmation_done", section, 0.0, False, False, status

        _HYBRID_STATE.color_confirmation_until = 0.0
        _HYBRID_STATE.color_confirmation_slot = ""
        status["camera_gain_request"] = _color_confirmation_restore_gain_request(args, "confirmation_done")
        return False, "color_confirmation_done", _HYBRID_STATE.color_confirmation_section, 0.0, False, False, status

    min_dist = float(getattr(args, "navigation_color_confirmation_min_distance_m", 0.33))
    close_dist = float(getattr(args, "navigation_color_confirmation_close_distance_m", 0.12))
    max_dist = float(getattr(args, "navigation_color_confirmation_max_distance_m", 0.90))
    unknown_dist = float(getattr(args, "navigation_color_confirmation_unknown_distance_m", 0.55))
    max_holds = max(0, int(getattr(args, "navigation_color_confirmation_max_holds_per_section", 3)))
    if max_holds <= 0:
        return False, "", "", 0.0, False, False, {"post_unpark_start_confirmation": post_unpark_status}
    if corner_hold_active:
        return False, "", "", 0.0, False, False, {
            "suppressed": True,
            "reason": "corner_observation_active",
            "corner_hold_section": corner_hold_section,
            "post_unpark_start_confirmation": post_unpark_status,
        }

    handoff_section = _corner_observation_handoff_active(route, now)
    candidates = []
    post_unpark_slot_id = str(post_unpark_status.get("slot_id", "") or "")
    post_unpark_candidate_added = False

    def projected_confirmation_obstacle(source_obs: dict) -> dict:
        obs = _project_slot(source_obs, route.direction)
        delta = _forward_delta(route.s_m, float(obs.get("s_m", 0.0) or 0.0))
        if delta > float(getattr(args, "navigation_horizon_m", 1.4)):
            delta = -float(REF_LAP_LENGTH_M - delta)
        obs["delta_m"] = delta
        return obs
    for obs in obstacles:
        delta = float(obs.get("delta_m", 0.0) or 0.0)
        if not math.isfinite(delta):
            continue
        committed = str(obs.get("color", "") or "")
        proposal = str(obs.get("proposed_color", "") or "")
        needs_unknown = committed not in ("RED", "GREEN")
        needs_conflict = proposal in ("RED", "GREEN") and proposal != committed
        if not needs_unknown and not needs_conflict:
            continue
        section = str(obs.get("section", "") or "")
        if section not in ORDER_CLOCK:
            continue
        gate_idx = _gate_index_from_along(_slot_along_m(obs))
        if (section, gate_idx) in _HYBRID_STATE.locked_gate_choices:
            continue

        hold_count = _HYBRID_STATE.color_confirmation_hold_counts.get(section, 0)
        hold_limit_reached = hold_count >= max_holds
        deadline_status = _color_confirmation_deadline_status(route, obs, args)
        deadline_triggered = bool(deadline_status.get("triggered", False))
        deadline_key = str(deadline_status.get("deadline_key", "") or "")
        gate_order = _gate_order_for_section(section, route.direction)
        first_gate_in_section = gate_idx in gate_order and gate_order.index(gate_idx) == 0
        handoff_triggered = bool(handoff_section and section == handoff_section and first_gate_in_section)
        deadline_already_used = bool(deadline_key and deadline_key in _HYBRID_STATE.color_confirmation_deadline_holds)
        mandatory_triggered = (deadline_triggered or handoff_triggered) and bool(deadline_key) and not deadline_already_used
        deadline_bypass_used = mandatory_triggered and hold_limit_reached
        if hold_limit_reached and not deadline_bypass_used:
            continue

        post_unpark_triggered = (
            bool(post_unpark_status.get("triggered", False))
            and str(post_unpark_status.get("slot_id", "") or "") == str(obs.get("id", "") or "")
        )
        if post_unpark_triggered:
            mode = "unknown" if needs_unknown else "close" if delta < min_dist else "normal"
            post_status = dict(deadline_status)
            post_status.update(
                {
                    "triggered": True,
                    "trigger_source": "post_unpark_start_section",
                    "deadline_key": "",
                    "deadline_section": "",
                    "deadline_gate_idx": None,
                    "post_unpark_start_confirmation": post_unpark_status,
                }
            )
            candidates.append((-1, max(0.0, delta), obs, mode, "post_unpark_start_section", post_status, False))
            post_unpark_candidate_added = True
            continue

        if handoff_triggered and not deadline_already_used:
            mode = "unknown" if needs_unknown else "close" if delta < min_dist else "normal"
            handoff_status = dict(deadline_status)
            handoff_status.update(
                {
                    "triggered": True,
                    "trigger_source": "corner_handoff",
                    "corner_hold_reason": corner_hold_reason,
                    "corner_hold_section": corner_hold_section,
                }
            )
            candidates.append((0, max(0.0, delta), obs, mode, "corner_handoff", handoff_status, deadline_bypass_used))
            continue

        if deadline_triggered and not deadline_already_used:
            mode = "unknown" if needs_unknown else "close" if delta < min_dist else "normal"
            candidates.append((0, max(0.0, delta), obs, mode, str(deadline_status.get("trigger_source", "deadline_gate")), deadline_status, deadline_bypass_used))
            continue

        distance_mode = ""
        distance_source = ""
        if close_dist <= delta <= max_dist:
            if needs_unknown and delta <= unknown_dist:
                distance_mode = "unknown"
                distance_source = "distance_unknown"
            elif needs_conflict:
                distance_mode = "close" if delta < min_dist else "normal"
                distance_source = "distance_conflict"
        if distance_mode:
            candidates.append((1, max(0.0, delta), obs, distance_mode, distance_source, deadline_status, False))
    if bool(post_unpark_status.get("triggered", False)) and post_unpark_slot_id and not post_unpark_candidate_added:
        post_source_obs = next((obs for obs in post_unpark_source_obstacles if str(obs.get("id", "") or "") == post_unpark_slot_id), None)
        if post_source_obs is not None:
            obs = projected_confirmation_obstacle(post_source_obs)
            delta = float(obs.get("delta_m", 0.0) or 0.0)
            committed = str(obs.get("color", "") or "")
            needs_unknown = committed not in ("RED", "GREEN")
            mode = "unknown" if needs_unknown else "close" if delta < min_dist else "normal"
            post_status = {
                "triggered": True,
                "trigger_source": "post_unpark_start_section",
                "deadline_key": "",
                "deadline_section": "",
                "deadline_gate_idx": None,
                "obstacle_gate_idx": _gate_index_from_along(_slot_along_m(obs)),
                "post_unpark_start_confirmation": {
                    **post_unpark_status,
                    "slot_missing_from_relevant": True,
                    "injected_candidate": True,
                },
            }
            post_unpark_status = dict(post_status["post_unpark_start_confirmation"])
            candidates.append((-1, max(0.0, delta), obs, mode, "post_unpark_start_section", post_status, False))
        else:
            post_unpark_status = {**post_unpark_status, "injected_candidate": False, "injected_candidate_reject_reason": "slot_missing_from_post_unpark_source"}
    if not candidates:
        return False, "", "", 0.0, False, False, {"post_unpark_start_confirmation": post_unpark_status}
    _priority, delta, obs, mode, trigger_source, deadline_status, deadline_bypass_used = min(candidates, key=lambda item: (item[0], item[1]))
    lidar_rate = max(1.0, float(getattr(args, "navigation_color_confirmation_lidar_hz", 15.0)))
    scans = max(1, int(getattr(args, "navigation_color_confirmation_scans", 10)))
    section = str(obs.get("section", ""))
    deadline_key = str(deadline_status.get("deadline_key", "") or "")
    deadline_section = str(deadline_status.get("deadline_section", "") or "")
    deadline_gate_idx = deadline_status.get("deadline_gate_idx")
    obstacle_gate_idx = deadline_status.get("obstacle_gate_idx")
    deadline_delta_m = float(deadline_status.get("deadline_delta_m", 0.0) or 0.0)
    deadline_trigger_delta_m = float(deadline_status.get("deadline_trigger_delta_m", 0.0) or 0.0)
    deadline_pre_gate_m = float(deadline_status.get("deadline_pre_gate_m", 0.0) or 0.0)
    deadline_trigger_margin_m = float(deadline_status.get("deadline_trigger_margin_m", 0.0) or 0.0)
    if (str(trigger_source).startswith("deadline") or str(trigger_source) == "corner_handoff") and deadline_key:
        _HYBRID_STATE.color_confirmation_deadline_holds.add(deadline_key)
    if str(trigger_source) == "corner_handoff":
        _clear_corner_observation_handoff()
    if str(trigger_source) == "post_unpark_start_section":
        _HYBRID_STATE.post_unpark_start_confirmation_consumed = True
        _HYBRID_STATE.post_unpark_start_confirmation_last_reason = "started_hold"
    _HYBRID_STATE.color_confirmation_hold_counts[section] = _HYBRID_STATE.color_confirmation_hold_counts.get(section, 0) + 1
    _HYBRID_STATE.color_confirmation_trigger_source = str(trigger_source or "distance")
    _HYBRID_STATE.color_confirmation_deadline_key = deadline_key
    _HYBRID_STATE.color_confirmation_deadline_section = deadline_section
    _HYBRID_STATE.color_confirmation_deadline_gate_idx = int(deadline_gate_idx) if deadline_gate_idx is not None else None
    _HYBRID_STATE.color_confirmation_obstacle_gate_idx = int(obstacle_gate_idx) if obstacle_gate_idx is not None else None
    _HYBRID_STATE.color_confirmation_deadline_delta_m = deadline_delta_m
    _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m = deadline_trigger_delta_m
    _HYBRID_STATE.color_confirmation_deadline_pre_gate_m = deadline_pre_gate_m
    _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m = deadline_trigger_margin_m
    _HYBRID_STATE.color_confirmation_deadline_bypass_used = bool(deadline_bypass_used)
    _HYBRID_STATE.color_confirmation_slot = str(obs.get("id", ""))
    _HYBRID_STATE.color_confirmation_section = section
    _HYBRID_STATE.color_confirmation_proposal = str(obs.get("proposed_color", "")) if mode != "unknown" else "UNKNOWN"
    _HYBRID_STATE.color_confirmation_mode = mode
    _HYBRID_STATE.color_confirmation_committed_start = str(obs.get("color", ""))
    _HYBRID_STATE.color_confirmation_start_t = now
    _HYBRID_STATE.color_confirmation_samples.clear()
    _HYBRID_STATE.color_confirmation_all_samples.clear()
    _HYBRID_STATE.color_confirmation_last_sample_key = ""
    _HYBRID_STATE.color_confirmation_seen_measurement_ids.clear()
    _HYBRID_STATE.color_confirmation_decision.clear()
    _HYBRID_STATE.color_confirmation_attempt = 0
    _HYBRID_STATE.color_confirmation_base_gain = float(getattr(args, "camera_analog_gain", 14.0))
    _HYBRID_STATE.color_confirmation_gain = _clamp(
        _HYBRID_STATE.color_confirmation_base_gain,
        float(getattr(args, "camera_analog_gain_min", 1.0)),
        float(getattr(args, "camera_analog_gain_max", 16.0)),
    )
    _HYBRID_STATE.color_confirmation_settle_until_t = now
    _HYBRID_STATE.color_confirmation_exposure_reason = "base"
    _HYBRID_STATE.color_confirmation_measurement_debug.clear()
    _HYBRID_STATE.color_confirmation_until = now + scans / lidar_rate
    _HYBRID_STATE.color_confirmation_timeout_t = now + max(0.0, float(getattr(args, "navigation_color_confirmation_timeout_s", 4.0)))
    return True, "color_confirmation", section, max(0.0, _HYBRID_STATE.color_confirmation_timeout_t - now), False, False, {
        "slot_id": _HYBRID_STATE.color_confirmation_slot,
        "proposal": _HYBRID_STATE.color_confirmation_proposal,
        "committed": str(obs.get("color", "")),
        "committed_start": str(obs.get("color", "")),
        "mode": mode,
        "trigger_source": _HYBRID_STATE.color_confirmation_trigger_source,
        "deadline_key": _HYBRID_STATE.color_confirmation_deadline_key,
        "deadline_section": _HYBRID_STATE.color_confirmation_deadline_section,
        "deadline_gate_idx": _HYBRID_STATE.color_confirmation_deadline_gate_idx,
        "obstacle_gate_idx": _HYBRID_STATE.color_confirmation_obstacle_gate_idx,
        "deadline_delta_m": _HYBRID_STATE.color_confirmation_deadline_delta_m,
        "deadline_trigger_delta_m": _HYBRID_STATE.color_confirmation_deadline_trigger_delta_m,
        "deadline_pre_gate_m": _HYBRID_STATE.color_confirmation_deadline_pre_gate_m,
        "deadline_trigger_margin_m": _HYBRID_STATE.color_confirmation_deadline_trigger_margin_m,
        "deadline_bypass_used": _HYBRID_STATE.color_confirmation_deadline_bypass_used,
        "delta_m": delta,
        "section_hold_count": _HYBRID_STATE.color_confirmation_hold_counts[section],
        "blocked": False,
        "sample_count": 0,
        "all_sample_count": 0,
        "counts": {"RED": 0, "GREEN": 0, "OTHER": 0},
        "all_counts": {"RED": 0, "GREEN": 0, "OTHER": 0},
        "attempt": int(_HYBRID_STATE.color_confirmation_attempt),
        "camera_gain_request": _color_confirmation_gain_request(args, "base"),
        "gain_settle_remaining_s": 0.0,
        "timeout_remaining_s": max(0.0, _HYBRID_STATE.color_confirmation_timeout_t - now),
        "post_unpark_start_confirmation": post_unpark_status,
        "sample_added_count": 0,
        "measurement_events_available": 0,
        "measurement_events_added": 0,
        "measurement_events_duplicate": 0,
        "measurement_events_not_real": 0,
        "measurement_events_stale": 0,
        "measurement_events_before_hold": 0,
        "measurement_events_gain_settle": 0,
        "measurement_events_missing_id": 0,
        "last_measurement_id": "",
        "last_sample_label": "",
        "last_skip_reason": "",
    }


def _absence_confirmation_hold(route: Projection, obstacles: list[dict], args, full_gate_lock: bool) -> tuple[bool, str, str, float, bool, bool, dict]:
    if full_gate_lock or not bool(getattr(args, "navigation_obstacle_absence_confirmation_stop", True)):
        return False, "", "", 0.0, False, False, {}
    if not _navigation_confirmations_active(args):
        _clear_absence_confirmation_state()
        return False, "", "", 0.0, False, False, {"suppressed": True, "reason": _navigation_confirmations_active_reason(args)}
    now = time.monotonic()
    if _HYBRID_STATE.absence_confirmation_direction != route.direction:
        _HYBRID_STATE.absence_confirmation_until = 0.0
        _HYBRID_STATE.absence_confirmation_slot = ""
        _HYBRID_STATE.absence_confirmation_section = ""
        _HYBRID_STATE.absence_confirmation_mode = ""
        _HYBRID_STATE.absence_confirmation_direction = route.direction
        _HYBRID_STATE.absence_confirmation_hold_counts.clear()
        _HYBRID_STATE.absence_confirmation_blocked_slot = ""
        _HYBRID_STATE.absence_confirmation_blocked_reason = ""

    if _HYBRID_STATE.absence_confirmation_blocked_slot:
        return (
            False,
            _HYBRID_STATE.absence_confirmation_blocked_reason or "obstacle_absence_close",
            _HYBRID_STATE.absence_confirmation_section,
            0.0,
            False,
            True,
            {
                "slot_id": _HYBRID_STATE.absence_confirmation_blocked_slot,
                "mode": _HYBRID_STATE.absence_confirmation_mode,
                "blocked": True,
            },
        )

    active_slot = _HYBRID_STATE.absence_confirmation_slot
    if active_slot:
        active_obs = next((obs for obs in obstacles if str(obs.get("id", "")) == active_slot), None)
        still_pending = active_obs is not None and bool(active_obs.get("absence_clear_pending", False))
        mode = _HYBRID_STATE.absence_confirmation_mode
        if not still_pending:
            _HYBRID_STATE.absence_confirmation_until = 0.0
            _HYBRID_STATE.absence_confirmation_slot = ""
            return False, "obstacle_absence_confirmation_done", _HYBRID_STATE.absence_confirmation_section, 0.0, False, False, {
                "slot_id": active_slot,
                "mode": mode,
                "still_pending": still_pending,
                "blocked": False,
                "released_early": True,
            }
        if now < _HYBRID_STATE.absence_confirmation_until:
            return True, "obstacle_absence_confirmation", _HYBRID_STATE.absence_confirmation_section, max(0.0, _HYBRID_STATE.absence_confirmation_until - now), False, False, {
                "slot_id": active_slot,
                "mode": mode,
                "still_pending": still_pending,
                "blocked": False,
            }
        _HYBRID_STATE.absence_confirmation_until = 0.0
        _HYBRID_STATE.absence_confirmation_slot = ""
        if mode == "close" and still_pending:
            return False, "obstacle_absence_confirmation_close_done", _HYBRID_STATE.absence_confirmation_section, 0.0, False, False, {
                "slot_id": active_slot,
                "mode": mode,
                "still_pending": still_pending,
                "blocked": False,
                "continued_after_close_pending": True,
            }
        return False, "obstacle_absence_confirmation_done", _HYBRID_STATE.absence_confirmation_section, 0.0, False, False, {
            "slot_id": active_slot,
            "mode": mode,
            "still_pending": still_pending,
            "blocked": False,
        }

    close_dist = float(getattr(args, "navigation_color_confirmation_close_distance_m", 0.12))
    min_dist = float(getattr(args, "navigation_color_confirmation_min_distance_m", 0.33))
    max_dist = float(getattr(args, "navigation_color_confirmation_max_distance_m", 0.90))
    max_holds = max(0, int(getattr(args, "navigation_color_confirmation_max_holds_per_section", 3)))
    if max_holds <= 0:
        return False, "", "", 0.0, False, False, {}
    candidates = [
        obs for obs in obstacles
        if bool(obs.get("absence_clear_pending", False))
        and close_dist <= float(obs.get("delta_m", 0.0) or 0.0) <= max_dist
    ]
    candidates.sort(key=lambda obs: float(obs.get("delta_m", 0.0) or 0.0))
    for obs in candidates:
        section = str(obs.get("section", ""))
        gate_idx = _gate_index_from_along(_slot_along_m(obs))
        if (section, gate_idx) in _HYBRID_STATE.locked_gate_choices:
            continue
        if _HYBRID_STATE.absence_confirmation_hold_counts.get(section, 0) >= max_holds:
            continue
        delta = float(obs.get("delta_m", 0.0) or 0.0)
        mode = "close" if delta < min_dist else "normal"
        lidar_rate = max(1.0, float(getattr(args, "navigation_color_confirmation_lidar_hz", 15.0)))
        scans = max(1, int(getattr(args, "navigation_color_confirmation_scans", 10)))
        _HYBRID_STATE.absence_confirmation_hold_counts[section] = _HYBRID_STATE.absence_confirmation_hold_counts.get(section, 0) + 1
        _HYBRID_STATE.absence_confirmation_slot = str(obs.get("id", ""))
        _HYBRID_STATE.absence_confirmation_section = section
        _HYBRID_STATE.absence_confirmation_mode = mode
        _HYBRID_STATE.absence_confirmation_until = now + scans / lidar_rate
        return True, "obstacle_absence_confirmation", section, scans / lidar_rate, False, False, {
            "slot_id": _HYBRID_STATE.absence_confirmation_slot,
            "mode": mode,
            "delta_m": delta,
            "section_hold_count": _HYBRID_STATE.absence_confirmation_hold_counts[section],
            "blocked": False,
        }
    return False, "", "", 0.0, False, False, {}


def _state_cost(node: Node, nx: float, ny: float, nh: float, steer_deg: float, prev_steer_deg: float, curvature: float, route: Projection, obstacles: list[dict], args) -> tuple[float, bool, float, Projection, list[str]]:
    proj = _project_to_direction(nx, ny, math.degrees(nh), route.direction)
    raw_delta = _forward_delta(route.s_m, proj.s_m)
    progress = raw_delta if raw_delta <= REF_LAP_LENGTH_M * 0.5 else raw_delta - REF_LAP_LENGTH_M
    if progress < node.progress_m - 0.03:
        return 1_000_000.0, False, progress, proj, ["backtrack"]
    field_cost, field_ok = _field_cost(nx, ny, args)
    if not field_ok:
        return 1_000_000.0, False, progress, proj, ["field_clearance"]
    obs_cost, obs_ok, warnings = _obstacle_cost(nx, ny, progress, proj.lateral_m, obstacles, route.direction, args)
    if not obs_ok:
        return 1_000_000.0, False, progress, proj, warnings
    base_lane = float(args.navigation_base_lane_m)
    lane_cost = float(args.navigation_centerline_weight) * (proj.lateral_m - base_lane) ** 2
    heading_cost = 0.018 * abs(_wrap_deg(math.degrees(nh) - proj.nominal_heading_deg))
    steer_cost = 0.0025 * abs(steer_deg) + 0.006 * abs(steer_deg - prev_steer_deg)
    curvature_cost = float(args.navigation_curvature_weight) * 0.004 * curvature * curvature
    progress_reward = 2.2 * max(0.0, progress - node.progress_m)
    total = field_cost + obs_cost + lane_cost + heading_cost + steer_cost + curvature_cost - progress_reward
    return total, True, progress, proj, warnings


def _bucket_key(proj: Projection, progress: float, heading_deg: float, args) -> tuple[int, int, int]:
    progress_bin = int(progress / max(float(args.navigation_lattice_step_m), 0.05))
    lane_bin = int(proj.lateral_m / 0.08)
    heading_bin = int((_wrap_deg(heading_deg - proj.nominal_heading_deg) + 180.0) / 18.0)
    return progress_bin, lane_bin, heading_bin


def _plan_state_lattice(pose, route: Projection, obstacles: list[dict], args) -> tuple[list[Node], int, bool, list[str]]:
    step = float(getattr(args, "navigation_primitive_step_m", args.navigation_lattice_step_m))
    horizon = float(args.navigation_horizon_m)
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    max_states = max(200, int(getattr(args, "navigation_max_states", 2400)))
    keep_per_bucket = max(1, int(getattr(args, "navigation_keep_per_bucket", 2)))
    steer_values = _steering_samples(args)
    start = Node(float(pose.x), float(pose.y), math.radians(float(pose.h)), route.s_m, 0.0, 0.0, 0.0, 0.0, 0.0, -1)
    nodes: list[Node] = [start]
    frontier = [0]
    best_idx = 0
    warnings: list[str] = []
    levels = max(1, int(math.ceil(horizon / max(step, 1e-6))))
    for _level in range(levels):
        candidates: dict[tuple[int, int, int], list[int]] = {}
        for idx in frontier:
            node = nodes[idx]
            for steer in steer_values:
                nx, ny, nh, curvature = _integrate_ackermann(node.x, node.y, node.h, steer, step, wheelbase)
                inc_cost, ok, progress, proj, state_warnings = _state_cost(node, nx, ny, nh, steer, node.steer_deg, curvature, route, obstacles, args)
                warnings.extend(state_warnings)
                if not ok:
                    continue
                new_node = Node(nx, ny, nh, proj.s_m, progress, node.distance_m + step, steer, curvature, node.cost + inc_cost, idx)
                if len(nodes) >= max_states:
                    break
                nodes.append(new_node)
                new_idx = len(nodes) - 1
                key = _bucket_key(proj, progress, math.degrees(nh), args)
                candidates.setdefault(key, []).append(new_idx)
                if (new_node.progress_m, -new_node.cost) > (nodes[best_idx].progress_m, -nodes[best_idx].cost):
                    best_idx = new_idx
            if len(nodes) >= max_states:
                break
        if not candidates:
            break
        next_frontier: list[int] = []
        for idxs in candidates.values():
            idxs.sort(key=lambda i: nodes[i].cost)
            next_frontier.extend(idxs[:keep_per_bucket])
        next_frontier.sort(key=lambda i: (nodes[i].cost - 5.0 * nodes[i].progress_m))
        beam_width = max(24, min(80, max_states // 45))
        frontier = next_frontier[:beam_width]
        for idx in frontier:
            if nodes[idx].progress_m > nodes[best_idx].progress_m + 0.15 or (abs(nodes[idx].progress_m - nodes[best_idx].progress_m) < 0.15 and nodes[idx].cost < nodes[best_idx].cost):
                best_idx = idx
        if nodes[best_idx].progress_m >= horizon - 0.5 * step:
            break
    blocked = nodes[best_idx].progress_m < min(0.35, horizon * 0.20)
    return nodes, best_idx, blocked, sorted(set(warnings))[:30]


def _reconstruct_path(nodes: list[Node], best_idx: int, route: Projection, args) -> list[dict]:
    chain: list[Node] = []
    idx = best_idx
    while idx >= 0:
        chain.append(nodes[idx])
        idx = nodes[idx].parent
    chain.reverse()
    max_points = max(2, int(args.navigation_path_points))
    if len(chain) > max_points:
        stride = max(1, math.ceil((len(chain) - 1) / (max_points - 1)))
        sampled = [chain[0]] + chain[1:-1:stride] + [chain[-1]]
    else:
        sampled = chain
    path = []
    for node in sampled[:max_points]:
        proj = _project_to_direction(node.x, node.y, math.degrees(node.h), route.direction)
        path.append(
            {
                "x": node.x,
                "y": node.y,
                "s_m": node.s_m,
                "distance_m": node.distance_m,
                "progress_m": node.progress_m,
                "lateral_m": proj.lateral_m,
                "section": proj.section,
                "primitive": proj.primitive,
                "local_s_m": proj.local_s_m,
                "heading_deg": math.degrees(node.h),
                "steer_deg": node.steer_deg,
                "curvature": node.curvature,
                "cost": node.cost,
            }
        )
    return path


def _lookahead_point(path: list[dict], args, distance_m: float | None = None) -> dict:
    if not path:
        return {}
    target = float(args.navigation_lookahead_m) if distance_m is None else float(distance_m)
    for point in path:
        if float(point["distance_m"]) >= target:
            return dict(point)
    return dict(path[-1])


def _path_straightness(path: list[dict], pose, args) -> tuple[float, dict]:
    if len(path) < 2:
        return 0.0, {"angle_error_deg": 180.0, "max_heading_change_deg": 180.0, "near_curvature": 999.0}
    probe_dist = float(getattr(args, "navigation_adaptive_lookahead_probe_m", 1.0))
    base = float(getattr(args, "navigation_lookahead_m", 0.22))
    probe = _lookahead_point(path, args, max(base, probe_dist))
    dx = float(probe.get("x", pose.x)) - float(pose.x)
    dy = float(probe.get("y", pose.y)) - float(pose.y)
    forward = math.radians(float(pose.h))
    bearing = math.atan2(dy, dx)
    angle_error = abs(_wrap_deg(math.degrees(bearing - forward)))
    heading0 = float(path[0].get("heading_deg", pose.h))
    max_heading_change = 0.0
    max_curvature = 0.0
    for point in path:
        if float(point.get("distance_m", 0.0) or 0.0) > probe_dist:
            break
        max_heading_change = max(max_heading_change, abs(_wrap_deg(float(point.get("heading_deg", heading0)) - heading0)))
        max_curvature = max(max_curvature, abs(float(point.get("curvature", 0.0) or 0.0)))
    angle_score = 1.0 - _clamp(angle_error / max(float(getattr(args, "navigation_adaptive_lookahead_angle_deg", 18.0)), 1e-6), 0.0, 1.0)
    heading_score = 1.0 - _clamp(max_heading_change / max(float(getattr(args, "navigation_adaptive_lookahead_heading_deg", 22.0)), 1e-6), 0.0, 1.0)
    curvature_score = 1.0 - _clamp(max_curvature / max(float(getattr(args, "navigation_adaptive_lookahead_curvature", 2.0)), 1e-6), 0.0, 1.0)
    straightness = _clamp(0.50 * angle_score + 0.30 * heading_score + 0.20 * curvature_score, 0.0, 1.0)
    return straightness, {
        "angle_error_deg": angle_error,
        "max_heading_change_deg": max_heading_change,
        "near_curvature": max_curvature,
    }


def _adaptive_lookahead_distance(path: list[dict], pose, args) -> tuple[float, float, dict]:
    base = float(getattr(args, "navigation_lookahead_m", 0.22))
    free_lh = float(getattr(args, "free_race_lookahead_m", 0.0) or 0.0)
    if _free_race_enabled(args) and free_lh > 0.0:
        return free_lh, 1.0, {
            "mode": "free_race_constant",
            "corridor_min_lookahead_m": free_lh,
            "corridor_max_lookahead_m": free_lh,
            "corridor_valid_lookahead_m": free_lh,
            "corridor_raw_lookahead_m": free_lh,
            "corridor_applied_lookahead_m": free_lh,
            "corridor_speed_scale": 1.0,
            "corridor_rate_limited": False,
        }
    if (
        not _free_race_enabled(args)
        and bool(getattr(args, "navigation_obstacle_speed_lookahead", True))
        and not bool(getattr(args, "_smart_parking_alignment_active", False))
    ):
        min_lh = max(0.05, float(getattr(args, "navigation_obstacle_speed_lookahead_min_m", 0.20)))
        max_lh = max(min_lh, float(getattr(args, "navigation_obstacle_speed_lookahead_max_m", 0.50)))
        max_speed = max(0.01, float(getattr(args, "navigation_obstacle_speed_lookahead_max_speed_mps", 1.0)))
        heading = math.radians(float(getattr(pose, "h", 0.0) or 0.0))
        vx = float(getattr(pose, "vx", 0.0) or 0.0)
        vy = float(getattr(pose, "vy", 0.0) or 0.0)
        forward_speed = vx * math.cos(heading) + vy * math.sin(heading)
        scale = _clamp(abs(forward_speed) / max_speed, 0.0, 1.0)
        distance = min_lh + (max_lh - min_lh) * scale
        return distance, scale, {
            "mode": "obstacle_speed",
            "obstacle_speed_lookahead_m": distance,
            "obstacle_speed_lookahead_min_m": min_lh,
            "obstacle_speed_lookahead_max_m": max_lh,
            "obstacle_speed_lookahead_max_speed_mps": max_speed,
            "obstacle_speed_lookahead_forward_mps": forward_speed,
            "obstacle_speed_lookahead_scale": scale,
            "corridor_min_lookahead_m": min_lh,
            "corridor_max_lookahead_m": max_lh,
            "corridor_valid_lookahead_m": distance,
            "corridor_raw_lookahead_m": distance,
            "corridor_applied_lookahead_m": distance,
            "corridor_speed_scale": scale,
            "corridor_rate_limited": False,
        }
    if bool(getattr(args, "navigation_adaptive_corridor", True)):
        min_lh = max(0.05, float(getattr(args, "navigation_adaptive_corridor_min_lookahead_m", 0.22)))
        max_lh = max(min_lh, float(getattr(args, "navigation_adaptive_corridor_max_lookahead_m", 0.32)))
        half_width = max(0.01, float(getattr(args, "navigation_adaptive_corridor_half_width_m", 0.07)))
        heading_limit = max(0.1, float(getattr(args, "navigation_adaptive_corridor_heading_deg", 10.0)))
        curvature_limit = max(0.01, float(getattr(args, "navigation_adaptive_corridor_curvature", 1.2)))
        max_samples = max(2, int(getattr(args, "navigation_adaptive_corridor_samples", 40)))
        consensus_window = max(1, int(getattr(args, "navigation_adaptive_corridor_consensus_window", 4)))
        consensus_required = max(1, min(consensus_window, int(getattr(args, "navigation_adaptive_corridor_consensus_required", 3))))
        continuity_start = max(0.0, float(getattr(args, "navigation_adaptive_corridor_continuity_start_m", 0.16)))
        h = math.radians(float(pose.h))
        ch, sh = math.cos(h), math.sin(h)
        farthest_inside = 0.0
        first_outside = None
        best_candidate = 0.0
        selected = None
        checked = 0
        inside_votes: deque = deque(maxlen=consensus_window)
        inside_count = 0
        max_heading_error_seen = 0.0
        max_curvature_seen = 0.0
        for point in path:
            distance = float(point.get("distance_m", 0.0) or 0.0)
            if distance <= 1e-6:
                continue
            if distance > max_lh:
                break
            dx = float(point.get("x", pose.x)) - float(pose.x)
            dy = float(point.get("y", pose.y)) - float(pose.y)
            local_x = ch * dx + sh * dy
            local_y = -sh * dx + ch * dy
            heading_error = abs(_wrap_deg(float(point.get("heading_deg", pose.h)) - float(pose.h)))
            point_curvature = abs(float(point.get("curvature", 0.0) or 0.0))
            max_heading_error_seen = max(max_heading_error_seen, heading_error)
            max_curvature_seen = max(max_curvature_seen, point_curvature)
            inside = (
                local_x > 0.0
                and abs(local_y) <= half_width
                and heading_error <= heading_limit
                and point_curvature <= curvature_limit
            )
            checked += 1
            if distance < continuity_start:
                if checked >= max_samples:
                    break
                continue
            inside_votes.append(bool(inside))
            if inside:
                inside_count += 1
            elif first_outside is None:
                first_outside = distance
            votes = sum(1 for value in inside_votes if value)
            locally_consistent = len(inside_votes) < consensus_window or votes >= consensus_required
            if not locally_consistent:
                break
            if distance >= min_lh and inside and locally_consistent:
                farthest_inside = distance
                best_candidate = distance
                selected = point
            if checked >= max_samples:
                break
        valid = selected is not None and farthest_inside >= min_lh
        raw_distance = _clamp(farthest_inside, min_lh, max_lh) if valid else base
        applied_distance = raw_distance
        scale = _clamp((applied_distance - min_lh) / max(max_lh - min_lh, 1e-6), 0.0, 1.0) if valid else 0.0
        ox = float(pose.x)
        oy = float(pose.y)
        left_x = -sh * half_width
        left_y = ch * half_width
        front_x = ch * max_lh
        front_y = sh * max_lh
        metrics = {
            "mode": "corridor",
            "valid": valid,
            "corridor_half_angle_deg": None,
            "corridor_half_width_m": half_width,
            "corridor_heading_limit_deg": heading_limit,
            "corridor_curvature_limit": curvature_limit,
            "corridor_heading_error_deg": max_heading_error_seen,
            "corridor_max_curvature_seen": max_curvature_seen,
            "corridor_min_lookahead_m": min_lh,
            "corridor_max_lookahead_m": max_lh,
            "corridor_valid_lookahead_m": farthest_inside if selected is not None else 0.0,
            "corridor_raw_lookahead_m": raw_distance,
            "corridor_applied_lookahead_m": applied_distance,
            "corridor_speed_scale": scale,
            "corridor_exit_distance_m": first_outside,
            "corridor_first_outside_m": first_outside,
            "corridor_best_candidate_m": best_candidate,
            "corridor_inside_count": inside_count,
            "corridor_consensus_window": consensus_window,
            "corridor_consensus_required": consensus_required,
            "corridor_continuity_start_m": continuity_start,
            "corridor_checked_points": checked,
            "corridor_increase_rate_mps": 0.0,
            "corridor_rate_limited": False,
            "corridor_points": {
                "origin": {"x": ox, "y": oy},
                "left": {"x": ox + left_x, "y": oy + left_y},
                "right": {"x": ox - left_x, "y": oy - left_y},
                "front_left": {"x": ox + front_x + left_x, "y": oy + front_y + left_y},
                "front_right": {"x": ox + front_x - left_x, "y": oy + front_y - left_y},
            },
        }
        return applied_distance, scale, metrics
    if not bool(getattr(args, "navigation_adaptive_lookahead", True)):
        return base, 0.0, {}
    min_lh = max(0.05, float(getattr(args, "navigation_adaptive_lookahead_min_m", base)))
    max_lh = max(min_lh, float(getattr(args, "navigation_adaptive_lookahead_max_m", 0.45)))
    straightness, metrics = _path_straightness(path, pose, args)
    distance = min_lh + (max_lh - min_lh) * straightness
    return distance, straightness, metrics


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


def _first_high_curvature_distance(path: list[dict], threshold: float, max_distance: float) -> float | None:
    if threshold <= 0.0 or max_distance <= 0.0:
        return None
    for point in path:
        distance = float(point.get("distance_m", 0.0) or 0.0)
        if distance <= 1e-6:
            continue
        if distance > max_distance:
            break
        if abs(float(point.get("curvature", 0.0) or 0.0)) > threshold:
            return distance
    return None


def _curvature_preview_speed_cap(path: list[dict], args) -> tuple[float | None, dict]:
    threshold = float(getattr(args, "navigation_curvature_slow_threshold", 2.2))
    preview_m = float(getattr(args, "navigation_curvature_preview_m", 0.95))
    distance = _first_high_curvature_distance(path, threshold, preview_m)
    if distance is None:
        return None, {
            "distance_m": None,
            "speed_cap_mps": None,
            "threshold": threshold,
            "preview_m": preview_m,
        }
    corner_speed = max(0.0, float(getattr(args, "navigation_corner_speed_mps", 0.36)))
    decel = max(0.01, float(getattr(args, "drive_command_decel_mps2", 1.40)))
    margin = max(0.0, float(getattr(args, "navigation_curvature_preview_margin_m", 0.10)))
    usable_distance = max(0.0, distance - margin)
    cap = math.sqrt(max(0.0, corner_speed * corner_speed + 2.0 * decel * usable_distance))
    return cap, {
        "distance_m": distance,
        "speed_cap_mps": cap,
        "threshold": threshold,
        "preview_m": preview_m,
        "margin_m": margin,
        "corner_speed_mps": corner_speed,
        "decel_mps2": decel,
    }


def _adaptive_yaw_rate_factor(pose, args) -> tuple[float, dict]:
    window = max(1, int(getattr(args, "navigation_adaptive_yaw_rate_window", 3)))
    if _HYBRID_STATE.adaptive_yaw_rate_samples.maxlen != window:
        _HYBRID_STATE.adaptive_yaw_rate_samples = deque(
            list(_HYBRID_STATE.adaptive_yaw_rate_samples)[-window:],
            maxlen=window,
        )
    yaw_rate = abs(float(getattr(pose, "vh", 0.0) or 0.0))
    _HYBRID_STATE.adaptive_yaw_rate_samples.append(yaw_rate)
    avg_yaw_rate = sum(_HYBRID_STATE.adaptive_yaw_rate_samples) / max(len(_HYBRID_STATE.adaptive_yaw_rate_samples), 1)
    full = max(0.0, float(getattr(args, "navigation_adaptive_yaw_rate_full_dps", 15.0)))
    zero = max(full + 1e-6, float(getattr(args, "navigation_adaptive_yaw_rate_zero_dps", 55.0)))
    if avg_yaw_rate <= full:
        factor = 1.0
    elif avg_yaw_rate >= zero:
        factor = 0.0
    else:
        factor = 1.0 - (avg_yaw_rate - full) / (zero - full)
    return _clamp(factor, 0.0, 1.0), {
        "yaw_rate_dps": yaw_rate,
        "yaw_rate_avg_dps": avg_yaw_rate,
        "yaw_rate_factor": _clamp(factor, 0.0, 1.0),
        "yaw_rate_window": window,
        "yaw_rate_full_dps": full,
        "yaw_rate_zero_dps": zero,
    }


def _obstacle_decisions(obstacles: list[dict], route: Projection, args) -> list[dict]:
    decisions = []
    base_lane = float(args.navigation_base_lane_m)
    for obs in obstacles:
        side, gap = _pass_side_by_lane(base_lane, obs, route.direction)
        reason = (obs.get("color") or "unknown").lower()
        decisions.append(
            {
                **obs,
                "pass_side": side,
                "clearance_m": abs(float(obs["row_m"]) - base_lane),
                "target_lateral_m": float(obs["row_m"]),
                "reason": f"{reason}_{side}",
                "pass_gap_m": gap,
            }
        )
    return decisions


def _obstacle_decision_confidence(obs: dict, args) -> float:
    position_conf = float(obs.get("confidence", 0.0) or 0.0)
    votes_conf = _clamp(float(obs.get("recent_votes", 0) or 0.0) / max(float(getattr(args, "obstacle_confirm_required", 3)), 1.0), 0.0, 1.0)
    color = str(obs.get("color", ""))
    color_conf = float(obs.get("color_confidence", 0.0) or 0.0) if color in ("RED", "GREEN") else 0.0
    if obs.get("known") and not obs.get("current"):
        position_conf = max(position_conf, 0.55)
    return _clamp(0.35 * position_conf + 0.25 * votes_conf + 0.40 * color_conf, 0.0, 1.0)


def _uncertain_obstacle_speed_cap(pose, obstacles: list[dict], args) -> tuple[float | None, str, dict]:
    enabled = bool(getattr(args, "navigation_uncertain_obstacle_slowdown", True))
    if not enabled:
        return None, "", {}
    max_distance = float(getattr(args, "navigation_uncertain_obstacle_distance_m", 0.95))
    candidates = [obs for obs in obstacles if 0.0 <= float(obs.get("delta_m", 0.0) or 0.0) <= max_distance]
    if not candidates:
        return None, "", {}
    obs = min(candidates, key=lambda item: float(item.get("delta_m", 0.0) or 0.0))
    delta = float(obs.get("delta_m", 0.0) or 0.0)
    confidence = _obstacle_decision_confidence(obs, args)
    threshold = float(getattr(args, "navigation_uncertain_obstacle_confidence_threshold", 0.72))
    if confidence >= threshold:
        return None, "", {"id": obs.get("id", ""), "confidence": confidence, "delta_m": delta}
    min_speed = float(getattr(args, "navigation_uncertain_obstacle_min_speed_mps", 0.10))
    normal_speed = float(getattr(args, "navigation_obstacle_speed_mps", 0.42))
    uncertainty = _clamp((threshold - confidence) / max(threshold, 1e-6), 0.0, 1.0)
    closeness = _clamp(1.0 - delta / max(max_distance, 1e-6), 0.0, 1.0)
    motion = math.hypot(float(getattr(pose, "vx", 0.0) or 0.0), float(getattr(pose, "vy", 0.0) or 0.0))
    turn_rate = abs(float(getattr(pose, "vh", 0.0) or 0.0))
    motion_factor = _clamp(motion / max(float(getattr(args, "drive_command_max_speed_mps", 0.3)), 0.05), 0.0, 1.0)
    turn_factor = _clamp(turn_rate / max(float(getattr(args, "navigation_uncertain_obstacle_turn_rate_dps", 70.0)), 1e-6), 0.0, 1.0)
    severity = _clamp(0.45 * uncertainty + 0.35 * closeness + 0.10 * motion_factor + 0.10 * turn_factor, 0.0, 1.0)
    cap = normal_speed + (min_speed - normal_speed) * severity
    stop_distance = float(getattr(args, "navigation_uncertain_obstacle_stop_distance_m", 0.28))
    stop_conf = float(getattr(args, "navigation_uncertain_obstacle_stop_confidence", 0.45))
    if delta <= stop_distance and confidence < stop_conf:
        cap = 0.0
    return max(0.0, cap), "uncertain_obstacle", {"id": obs.get("id", ""), "confidence": confidence, "delta_m": delta}


def _unknown_color_speed_cap(obstacles: list[dict], args) -> tuple[float | None, str, dict]:
    candidates = [
        obs for obs in obstacles
        if float(obs.get("delta_m", 0.0) or 0.0) >= 0.0
    ]
    if not candidates:
        return None, "", {}
    obs = min(candidates, key=lambda item: float(item.get("delta_m", 0.0) or 0.0))
    color = str(obs.get("color", "") or "")
    if color in ("RED", "GREEN"):
        return None, "", {}
    return (
        max(0.0, float(getattr(args, "navigation_unknown_color_speed_mps", 0.20))),
        "unknown_color",
        {"id": obs.get("id", ""), "delta_m": float(obs.get("delta_m", 0.0) or 0.0)},
    )


def _obstacle_gate_locked(obs: dict) -> bool:
    section = str(obs.get("section", "") or "")
    if section not in ORDER_CLOCK:
        return False
    try:
        gate_idx = _gate_index_from_along(_slot_along_m(obs))
    except (KeyError, TypeError, ValueError):
        return False
    return (section, gate_idx) in _HYBRID_STATE.locked_gate_choices


def _section_obstacle_color_status(section: str, obstacles: list[dict]) -> dict:
    status = {
        "section": section,
        "has_obstacle": False,
        "ready": False,
        "obstacle_id": "",
        "color": "",
    }
    if section not in ORDER_CLOCK:
        return status
    first_candidate: dict | None = None
    for slot in obstacles:
        if slot.get("section") != section or not (slot.get("known") or slot.get("current")):
            continue
        if first_candidate is None:
            first_candidate = slot
        color = str(slot.get("color") or slot.get("known_color") or slot.get("current_color") or "")
        if color in ("RED", "GREEN"):
            return {
                "section": section,
                "has_obstacle": True,
                "ready": True,
                "obstacle_id": str(slot.get("id", "")),
                "color": color,
            }
    if first_candidate is not None:
        status.update(
            {
                "has_obstacle": True,
                "obstacle_id": str(first_candidate.get("id", "")),
                "color": str(first_candidate.get("color") or first_candidate.get("known_color") or first_candidate.get("current_color") or ""),
            }
        )
    return status


def _tracking_error_speed_factor(route: Projection, args) -> tuple[float, str, dict]:
    if not bool(getattr(args, "navigation_tracking_error_speed_cap", True)):
        return 1.0, "", {"enabled": False, "factor": 1.0}
    lat_soft = max(0.0, float(getattr(args, "navigation_tracking_lateral_soft_m", 0.05)))
    lat_hard = max(lat_soft + 1e-6, float(getattr(args, "navigation_tracking_lateral_hard_m", 0.14)))
    h_soft = max(0.0, float(getattr(args, "navigation_tracking_heading_soft_deg", 8.0)))
    h_hard = max(h_soft + 1e-6, float(getattr(args, "navigation_tracking_heading_hard_deg", 24.0)))
    min_factor = _clamp(float(getattr(args, "navigation_tracking_error_min_factor", 0.45)), 0.0, 1.0)
    lat_error = abs(float(route.distance_error_m))
    heading_error = abs(float(route.heading_error_deg))
    lat_t = _clamp((lat_error - lat_soft) / max(lat_hard - lat_soft, 1e-6), 0.0, 1.0)
    heading_t = _clamp((heading_error - h_soft) / max(h_hard - h_soft, 1e-6), 0.0, 1.0)
    severity = max(lat_t, heading_t)
    factor = 1.0 + (min_factor - 1.0) * severity
    reason = ""
    if severity > 0.0:
        reason = "tracking_heading" if heading_t >= lat_t else "tracking_lateral"
    return factor, reason, {
        "enabled": True,
        "factor": factor,
        "reason": reason,
        "lateral_error_m": lat_error,
        "heading_error_deg": heading_error,
        "lateral_t": lat_t,
        "heading_t": heading_t,
        "min_factor": min_factor,
    }


def _corner_transition_speed_keys(args) -> list[str]:
    raw = str(
        getattr(
            args,
            "navigation_corner_transition_speed_transition",
            "inner_tight_to_outer_tight,inner_tight_to_outer_pass",
        )
        or ""
    )
    keys = []
    for part in raw.replace(";", ",").replace("|", ",").split(","):
        key = part.strip()
        if key:
            keys.append(key)
    return keys


def _clock_bottom_right_outer_pass_transition(reason: str) -> str:
    prefix = "corner_right_to_bottom_"
    if not reason.startswith(prefix):
        return ""
    tail = reason[len(prefix):]
    for in_lane_name in CORNER_LANE_NAMES:
        transition = f"{in_lane_name}_to_outer_pass"
        if tail.startswith(f"{transition}_") or tail == transition:
            return transition
    return ""


def _obstacle_corner_transition_speed_cap(path: list[dict], args, direction: str = "") -> tuple[float | None, str, dict]:
    if not bool(getattr(args, "navigation_corner_transition_speed_cap", True)):
        return None, "", {"enabled": False, "active": False, "factor": 1.0}
    transitions = _corner_transition_speed_keys(args)
    if not transitions:
        return None, "", {"enabled": True, "active": False, "factor": 1.0, "reason": ""}
    lookahead_m = max(0.0, float(getattr(args, "navigation_corner_transition_speed_distance_m", 0.15)))
    max_mps = max(0.0, float(getattr(args, "navigation_corner_transition_speed_max_mps", 0.72)))
    for point in path:
        distance = float(point.get("distance_m", 0.0) or 0.0)
        if distance < 0.0:
            continue
        if distance > lookahead_m:
            break
        reason = str(point.get("target_reason", "") or "")
        special_transition = (
            _clock_bottom_right_outer_pass_transition(reason)
            if str(direction).upper() == "CLOCK"
            else ""
        )
        if special_transition:
            return max_mps, "corner_transition_bottom_right_clock_outer_pass", {
                "enabled": True,
                "active": True,
                "factor": 1.0,
                "reason": "corner_transition_bottom_right_clock_outer_pass",
                "rule": "bottom_right_clock_outer_pass",
                "corner": "right_to_bottom",
                "out_lane": "outer_pass",
                "transition": special_transition,
                "transitions": transitions,
                "distance_m": distance,
                "max_mps": max_mps,
                "target_reason": reason,
            }
        matched_transition = next((transition for transition in transitions if transition in reason), "")
        if reason.startswith("corner_") and matched_transition:
            return max_mps, "corner_transition", {
                "enabled": True,
                "active": True,
                "factor": 1.0,
                "reason": "corner_transition",
                "transition": matched_transition,
                "transitions": transitions,
                "distance_m": distance,
                "max_mps": max_mps,
                "target_reason": reason,
            }
    return None, "", {
        "enabled": True,
        "active": False,
        "factor": 1.0,
        "reason": "",
        "transition": ",".join(transitions),
        "transitions": transitions,
        "distance_m": None,
        "max_mps": max_mps,
    }


def _obstacle_mode_speed_policy(
    route: Projection,
    obstacles: list[dict],
    relevant: list[dict],
    uncertainty_cap: float | None,
    uncertainty_status: dict,
    unknown_color_cap: float | None,
    args,
) -> tuple[float, str, dict]:
    del uncertainty_cap, uncertainty_status, unknown_color_cap
    cruise = max(0.0, float(getattr(args, "navigation_first_lap_cruise_speed_mps", 0.60)))
    caution = max(0.0, float(getattr(args, "navigation_caution_speed_mps", 0.32)))
    current_status = _section_obstacle_color_status(route.section, obstacles)
    next_section = _next_section(route.section, route.direction)
    next_status = _section_obstacle_color_status(next_section, obstacles)
    status: dict = {
        "policy": "obstacle_first_lap",
        "base_speed_mps": caution,
        "cap_factor": 1.0,
        "cap_reason": "",
        "current_section": route.section,
        "current_section_ready": bool(current_status.get("ready", False)),
        "current_section_has_obstacle": bool(current_status.get("has_obstacle", False)),
        "current_section_obstacle_id": str(current_status.get("obstacle_id", "")),
        "current_section_obstacle_color": str(current_status.get("color", "")),
        "next_section": next_section,
        "next_section_ready": bool(next_status.get("ready", False)),
        "next_section_has_obstacle": bool(next_status.get("has_obstacle", False)),
        "next_section_obstacle_id": str(next_status.get("obstacle_id", "")),
        "next_section_obstacle_color": str(next_status.get("color", "")),
        "next_obstacle_id": "",
        "next_obstacle_delta_m": None,
        "next_obstacle_color": "",
        "next_obstacle_gate_locked": False,
    }
    candidates = [obs for obs in relevant if float(obs.get("delta_m", 0.0) or 0.0) >= 0.0]
    if candidates:
        obs = min(candidates, key=lambda item: float(item.get("delta_m", 0.0) or 0.0))
        status.update(
            {
                "next_obstacle_id": str(obs.get("id", "")),
                "next_obstacle_delta_m": float(obs.get("delta_m", 0.0) or 0.0),
                "next_obstacle_color": str(obs.get("color", "") or ""),
                "next_obstacle_gate_locked": _obstacle_gate_locked(obs),
            }
        )
    if not bool(current_status.get("ready", False)):
        status["base_speed_mps"] = caution
        status["cap_reason"] = "current_section_unknown"
        return caution, "caution_current_section_unknown", status
    if not bool(next_status.get("ready", False)):
        status["base_speed_mps"] = caution
        status["cap_reason"] = "next_section_unknown"
        return caution, "caution_next_section_unknown", status
    status["base_speed_mps"] = cruise
    status["cap_reason"] = "sections_ready"
    return cruise, "first_lap_cruise_sections_ready", status


def _reference_lane_fallback(pose, route: Projection, obstacles: list[dict], args) -> tuple[list[dict], bool, float, list[str]]:
    # Minimal safe fallback if the lattice cannot find a path: project a short
    # straight-ahead preview from the robot pose and mark blocked when it exits
    # the legal corridor. This keeps the web payload alive without inventing a
    # non-vehicle-feasible route.
    path = []
    h = math.radians(float(pose.h))
    step = float(getattr(args, "navigation_primitive_step_m", args.navigation_lattice_step_m))
    horizon = min(float(args.navigation_horizon_m), 1.0)
    blocked = False
    for i in range(max(2, int(horizon / max(step, 1e-6)) + 1)):
        d = i * step
        x = float(pose.x) + d * math.cos(h)
        y = float(pose.y) + d * math.sin(h)
        proj = _project_to_direction(x, y, float(pose.h), route.direction)
        _field_cost_value, ok = _field_cost(x, y, args)
        if not ok:
            blocked = True
            break
        path.append({"x": x, "y": y, "s_m": proj.s_m, "distance_m": d, "progress_m": _forward_delta(route.s_m, proj.s_m), "lateral_m": proj.lateral_m, "section": proj.section, "primitive": proj.primitive, "local_s_m": proj.local_s_m, "heading_deg": float(pose.h), "steer_deg": 0.0, "curvature": 0.0, "cost": 0.0})
    return path, True, 0.0, ["fallback"] if blocked else []


def _reference_lane_preview(pose, route: Projection, args) -> tuple[list[dict], bool, float, list[str]]:
    path = [
        {
            "x": float(pose.x),
            "y": float(pose.y),
            "s_m": route.s_m,
            "distance_m": 0.0,
            "progress_m": 0.0,
            "lateral_m": route.lateral_m,
            "section": route.section,
            "primitive": route.primitive,
            "local_s_m": route.local_s_m,
            "heading_deg": float(pose.h),
            "steer_deg": 0.0,
            "curvature": 0.0,
            "cost": 0.0,
        }
    ]
    horizon = float(args.navigation_horizon_m)
    count = max(2, int(args.navigation_path_points))
    lane = _clamp(float(args.navigation_base_lane_m), float(args.navigation_min_lane_m), float(args.navigation_max_lane_m))
    for i in range(1, count):
        distance = horizon * i / (count - 1)
        x, y, section, primitive, local_s, heading = _point_on_route_s(route.s_m + distance, lane, route.direction)
        path.append(
            {
                "x": x,
                "y": y,
                "s_m": (route.s_m + distance) % REF_LAP_LENGTH_M,
                "distance_m": distance,
                "progress_m": distance,
                "lateral_m": lane,
                "section": section,
                "primitive": primitive,
                "local_s_m": local_s,
                "heading_deg": heading,
                "steer_deg": 0.0,
                "curvature": 0.0,
                "cost": 0.0,
            }
        )
    return path, False, 0.0, ["reference_lane"]


def _bottom_stage_s(direction: str, lane_m: float, args) -> float:
    center_x = _parking_center_x_from_args(args)
    turn_radius = max(0.0, float(getattr(args, "smart_parking_turn_radius_m", 0.12) or 0.12))
    stage_extra = float(getattr(args, "smart_parking_stage_offset_m", 0.0) or 0.0)
    stage_offset = max(0.0, turn_radius + stage_extra)
    stage_x = center_x - stage_offset if direction == "CLOCK" else center_x + stage_offset
    heading = 180.0 if direction == "CLOCK" else 0.0
    y = -OUTER_HALF_M + lane_m
    return _project_to_direction(stage_x, y, heading, direction).s_m


def _smart_parking_middle_gate_meaningful(decision: dict) -> bool:
    if decision.get("obstacle_ids"):
        return True
    locked = _HYBRID_STATE.locked_gate_choices.get(("bottom", 1), {})
    return str(locked.get("lock_reason", "")) == "waypoint_passed"


def _smart_parking_alignment_lane(direction: str, obstacles: list[dict], args) -> tuple[float, str, dict]:
    decisions = _gate_decisions(direction, obstacles, args)
    lane_plan = []
    if direction == "CLOCK":
        source_idx = 2
        middle_meaningful = False
        lane_plan.append((source_idx, decisions[("bottom", source_idx)]))
    else:
        middle = decisions[("bottom", 1)]
        middle_meaningful = _smart_parking_middle_gate_meaningful(middle)
        lane_plan.append((0, decisions[("bottom", 0)]))
        if middle_meaningful:
            lane_plan.append((1, middle))
        source_idx = lane_plan[-1][0]
    decision = decisions[("bottom", source_idx)]
    lane_m = float(decision.get("lane_m", _lane_value("center", args)))
    source = f"bottom_gate_{source_idx}_{decision.get('lane_name', 'center')}"
    debug = {
        "source_gate_idx": source_idx,
        "source_lane_name": str(decision.get("lane_name", "center")),
        "source_lane_m": lane_m,
        "source_reason": str(decision.get("reason", "")),
        "source_locked": bool(decision.get("locked", False)),
        "middle_considered": direction == "COUNTER",
        "middle_chosen": direction == "COUNTER" and source_idx == 1,
        "middle_has_obstacle": bool(decisions[("bottom", 1)].get("obstacle_ids")),
        "middle_hard_locked": str(_HYBRID_STATE.locked_gate_choices.get(("bottom", 1), {}).get("lock_reason", "")) == "waypoint_passed",
        "middle_meaningful": middle_meaningful,
        "lane_plan": [
            {
                "gate_idx": idx,
                "lane_name": str(item.get("lane_name", "center")),
                "lane_m": float(item.get("lane_m", _lane_value("center", args))),
                "reason": str(item.get("reason", "")),
                "locked": bool(item.get("locked", False)),
            }
            for idx, item in lane_plan
        ],
    }
    return lane_m, source, debug


def _smart_parking_lane_for_s(s_m: float, route_s: float, direction: str, lane_m: float, lane_debug: dict) -> float:
    lane_plan = lane_debug.get("lane_plan", [])
    if direction != "COUNTER" or len(lane_plan) < 2:
        return lane_m
    gate0_s = _section_route_s("bottom", direction, GATE_ALONGS_M[0])
    gate1_s = _section_route_s("bottom", direction, GATE_ALONGS_M[1])
    lane0 = float(lane_plan[0].get("lane_m", lane_m))
    lane1 = float(lane_plan[1].get("lane_m", lane_m))
    gate0_delta = _forward_delta(route_s, gate0_s)
    gate1_delta = _forward_delta(route_s, gate1_s)
    point_delta = _forward_delta(route_s, s_m)
    if gate1_delta > REF_LAP_LENGTH_M * 0.5:
        return lane1
    if gate0_delta > REF_LAP_LENGTH_M * 0.5:
        if point_delta >= gate1_delta:
            return lane1
        u = point_delta / max(gate1_delta, 1e-6)
        return lane0 + (lane1 - lane0) * _clamp(u, 0.0, 1.0)
    if point_delta <= gate0_delta:
        return lane0
    if point_delta >= gate1_delta:
        return lane1
    u = (point_delta - gate0_delta) / max(gate1_delta - gate0_delta, 1e-6)
    return lane0 + (lane1 - lane0) * _clamp(u, 0.0, 1.0)


def _smart_parking_alignment_point(route: Projection, dist: float, lane_m: float, lane_debug: dict) -> tuple[float, float, str, str, float, float, float]:
    point_s = (route.s_m + dist) % REF_LAP_LENGTH_M
    point_lane_m = _smart_parking_lane_for_s(point_s, route.s_m, route.direction, lane_m, lane_debug)
    bottom = _primitive_by_name(route.direction, "bottom_straight")
    if bottom is None:
        x, y, section, primitive, local_s, heading = _point_on_route_s(point_s, point_lane_m, route.direction)
        return x, y, section, primitive, local_s, heading, point_lane_m

    bottom_end_s = (bottom.start_s_m + bottom.length_m) % REF_LAP_LENGTH_M
    bottom_end_delta = _forward_delta(route.s_m, bottom_end_s)
    if bottom_end_delta > REF_LAP_LENGTH_M * 0.5 or dist <= bottom_end_delta:
        x, y, section, primitive, local_s, heading = _point_on_route_s(point_s, point_lane_m, route.direction)
        return x, y, section, primitive, local_s, heading, point_lane_m

    extra = dist - bottom_end_delta
    end_lane_m = _smart_parking_lane_for_s(bottom_end_s, route.s_m, route.direction, lane_m, lane_debug)
    end_x, end_y, _section, _primitive, _local_s, heading = _point_on_route_s(bottom_end_s, end_lane_m, route.direction)
    h = math.radians(heading)
    x = end_x + math.cos(h) * extra
    y = end_y + math.sin(h) * extra
    return x, y, "bottom", "bottom_straight_extended", bottom.length_m + extra, heading, end_lane_m


def _smart_parking_handoff(route: Projection, lane_debug: dict) -> tuple[float, str, int]:
    gate_idx = int(lane_debug.get("source_gate_idx", 2 if route.direction == "CLOCK" else 0))
    gate_idx = max(0, min(len(GATE_ALONGS_M) - 1, gate_idx))
    handoff_s = _section_route_s("bottom", route.direction, GATE_ALONGS_M[gate_idx])
    return handoff_s, f"bottom_gate_{gate_idx}", gate_idx


def _smart_parking_route_waypoints(route: Projection, waypoints: list[Waypoint], obstacles: list[dict], args) -> tuple[list[Waypoint], bool]:
    if not waypoints or route.direction not in ("CLOCK", "COUNTER") or _free_race_enabled(args):
        return waypoints, False
    if not bool(getattr(args, "smart_parking_alignment_enabled", True)) or not bool(getattr(args, "smart_parking", True)):
        return waypoints, False

    status = _smart_parking_trigger_status(route, args)
    preview_m = max(0.0, float(getattr(args, "smart_parking_alignment_preview_m", 1.75)))
    remaining_to_lap_target = float(status.get("target_progress_m", 0.0) or 0.0) - float(status.get("route_progress_m", 0.0) or 0.0)
    if remaining_to_lap_target > preview_m:
        return waypoints, False

    lane_m, source, lane_debug = _smart_parking_alignment_lane(route.direction, obstacles, args)
    handoff_s, handoff_gate, handoff_gate_idx = _smart_parking_handoff(route, lane_debug)
    stage_s = _bottom_stage_s(route.direction, lane_m, args)
    extension = max(0.0, float(getattr(args, "smart_parking_alignment_extension_m", 0.70)))
    end_s = stage_s + extension
    stage_delta = _forward_delta(route.s_m, stage_s)
    end_delta = _forward_delta(route.s_m, end_s)
    handoff_delta = _forward_delta(route.s_m, handoff_s)
    preview_active = (
        stage_delta <= preview_m
        or end_delta <= preview_m
        or handoff_delta <= preview_m
        or bool(status.get("trigger", False))
    )
    if not preview_active:
        return waypoints, False

    stage_x, stage_y, _stage_section, _stage_primitive, _stage_local_s, stage_heading, stage_lane = _smart_parking_alignment_point(route, stage_delta, lane_m, lane_debug)
    end_x, end_y, _end_section, _end_primitive, _end_local_s, end_heading, end_lane = _smart_parking_alignment_point(route, end_delta, lane_m, lane_debug)
    virtual = [
        _make_explicit_waypoint(
            stage_x,
            stage_y,
            stage_heading,
            stage_s % REF_LAP_LENGTH_M,
            stage_lane,
            f"smart_parking_stage_from_{source}",
            f"smart_parking_stage_{handoff_gate}",
            "smart_parking",
        ),
        _make_explicit_waypoint(
            end_x,
            end_y,
            end_heading,
            end_s % REF_LAP_LENGTH_M,
            end_lane,
            f"smart_parking_extension_from_{source}",
            f"smart_parking_extension_{handoff_gate}",
            "smart_parking",
        ),
    ]

    filtered: list[Waypoint] = []
    for wp in waypoints:
        delta = _forward_delta(handoff_s, wp.s_m)
        if delta <= 1e-6:
            filtered.append(wp)
            continue
        if wp.kind in ("gate", "corner") and delta <= REF_LAP_LENGTH_M * 0.35:
            continue
        filtered.append(wp)
    filtered.extend(virtual)
    filtered.sort(key=lambda wp: wp.s_m)
    return filtered, True


def _smart_parking_alignment_path(route: Projection, obstacles: list[dict], args) -> tuple[list[dict], dict]:
    status = _smart_parking_trigger_status(route, args)
    if route.direction in ("CLOCK", "COUNTER"):
        lane_m, source, lane_debug = _smart_parking_alignment_lane(route.direction, obstacles, args)
    else:
        lane_m, source, lane_debug = _lane_value("center", args), "none", {}
    extension = max(0.0, float(getattr(args, "smart_parking_alignment_extension_m", 0.70)))
    preview_m = max(0.0, float(getattr(args, "smart_parking_alignment_preview_m", 1.75)))
    stage_s = _bottom_stage_s(route.direction, lane_m, args) if route.direction in ("CLOCK", "COUNTER") else route.s_m
    end_s = stage_s + extension
    handoff_s, handoff_gate, handoff_gate_idx = _smart_parking_handoff(route, lane_debug) if route.direction in ("CLOCK", "COUNTER") else (route.s_m, "none", -1)
    handoff_delta = _forward_delta(route.s_m, handoff_s)
    handoff_already_passed = handoff_delta > REF_LAP_LENGTH_M * 0.5
    if handoff_already_passed:
        handoff_delta = 0.0
    stage_delta = _forward_delta(route.s_m, stage_s)
    end_delta = _forward_delta(route.s_m, end_s)
    remaining_to_lap_target = float(status.get("target_progress_m", 0.0) or 0.0) - float(status.get("route_progress_m", 0.0) or 0.0)
    final_section_active = remaining_to_lap_target <= preview_m
    preview_active = final_section_active and (
        stage_delta <= preview_m
        or end_delta <= preview_m
        or bool(status.get("trigger", False))
    )
    enabled = (
        bool(getattr(args, "smart_parking_alignment_enabled", True))
        and preview_active
        and route.direction in ("CLOCK", "COUNTER")
        and not _free_race_enabled(args)
    )
    if not enabled:
        return [], {"active": False, "splice_active": False, "preview_active": preview_active, "final_section_active": final_section_active, "remaining_to_lap_target_m": remaining_to_lap_target, "handoff_gate": handoff_gate, "handoff_gate_idx": handoff_gate_idx, "handoff_s_m": handoff_s, "handoff_delta_m": handoff_delta, "remaining_to_handoff_m": handoff_delta, "handoff_already_passed": handoff_already_passed, "stage_delta_m": stage_delta, "remaining_to_stage_m": stage_delta, "end_delta_m": end_delta, "preview_m": preview_m, "source_gate": source, "lane_m": lane_m, **lane_debug, **status}
    target_delta = _forward_delta(route.s_m, end_s)
    max_len = max(
        target_delta,
        float(getattr(args, "navigation_lookahead_m", 0.36)) + float(getattr(args, "navigation_local_path_extra_m", 1.5)),
        0.50,
    )
    count = max(8, int(math.ceil(max_len / max(float(getattr(args, "navigation_route_sample_step_m", 0.035)), 0.02))) + 1)
    wheelbase = float(getattr(args, "navigation_wheelbase_m", DEFAULT_ROBOT_WHEELBASE_M))
    path = []
    last_x = last_y = None
    last_h = None
    distance = 0.0
    for idx in range(count):
        dist = max_len * idx / max(count - 1, 1)
        point_s = (route.s_m + dist) % REF_LAP_LENGTH_M
        x, y, section, primitive, local_s, heading, point_lane_m = _smart_parking_alignment_point(route, dist, lane_m, lane_debug)
        h_rad = math.radians(heading)
        if last_x is not None and last_y is not None:
            step = max(math.hypot(x - last_x, y - last_y), 1e-6)
            distance += step
            curvature = _wrap_rad(h_rad - last_h) / step
        else:
            curvature = 0.0
        path.append(
            {
                "x": x,
                "y": y,
                "s_m": point_s,
                "distance_m": distance,
                "progress_m": dist,
                "lateral_m": point_lane_m,
                "section": section,
                "primitive": primitive,
                "local_s_m": local_s,
                "heading_deg": heading,
                "steer_deg": math.degrees(math.atan(curvature * wheelbase)),
                "curvature": curvature,
                "cost": 0.0,
                "target_reason": "smart_parking_alignment",
            }
        )
        last_x, last_y, last_h = x, y, h_rad
    debug = {
        "active": True,
        "splice_active": False,
        "lane_m": lane_m,
        "source_gate": source,
        **lane_debug,
        "handoff_gate": handoff_gate,
        "handoff_gate_idx": handoff_gate_idx,
        "handoff_s_m": handoff_s,
        "handoff_delta_m": handoff_delta,
        "remaining_to_handoff_m": handoff_delta,
        "handoff_already_passed": handoff_already_passed,
        "stage_s_m": stage_s,
        "target_delta_m": target_delta,
        "stage_delta_m": stage_delta,
        "remaining_to_stage_m": stage_delta,
        "end_delta_m": end_delta,
        "preview_m": preview_m,
        "preview_active": preview_active,
        "final_section_active": final_section_active,
        "remaining_to_lap_target_m": remaining_to_lap_target,
        "extension_m": extension,
        **status,
    }
    return path, debug



def _localization_hold_status(args) -> dict:
    status = getattr(args, "localization_hold", {}) if args is not None else {}
    if not isinstance(status, dict):
        status = {}
    return {
        "active": bool(status.get("active", False)),
        "reason": str(status.get("reason", "") or ""),
        "release_reason": str(status.get("release_reason", "") or ""),
        "remaining_s": float(status.get("remaining_s", 0.0) or 0.0),
        "min_remaining_s": float(status.get("min_remaining_s", 0.0) or 0.0),
        "timeout_remaining_s": float(status.get("timeout_remaining_s", 0.0) or 0.0),
        "stationary": bool(status.get("stationary", False)),
        "wide_reacquire": bool(status.get("wide_reacquire", False)),
        "effective_axis_max_error_m": status.get("effective_axis_max_error_m"),
        "missing_xy_count": status.get("missing_xy_count", 0),
        "valid_xy_count": status.get("valid_xy_count", 0),
        "settled_count": status.get("settled_count", 0),
        "trigger_source": str(status.get("trigger_source", "") or ""),
        "cooldown_remaining_s": float(status.get("cooldown_remaining_s", 0.0) or 0.0),
        "no_current_trigger_enabled": bool(status.get("no_current_trigger_enabled", False)),
        "no_current_triggered": bool(status.get("no_current_triggered", False)),
        "no_current_trigger_axis": str(status.get("no_current_trigger_axis", "") or ""),
        "no_current_suppressed_reason": str(status.get("no_current_suppressed_reason", "") or ""),
        "no_current_window_count": int(status.get("no_current_window_count", 0) or 0),
        "no_current_window_size": int(status.get("no_current_window_size", 0) or 0),
        "no_current_required": int(status.get("no_current_required", 0) or 0),
        "no_current_x_count": int(status.get("no_current_x_count", 0) or 0),
        "no_current_y_count": int(status.get("no_current_y_count", 0) or 0),
        "no_current_x_missing": bool(status.get("no_current_x_missing", False)),
        "no_current_y_missing": bool(status.get("no_current_y_missing", False)),
        "no_current_section_visit_key": str(status.get("no_current_section_visit_key", "") or ""),
        "no_current_section_visit_count": int(status.get("no_current_section_visit_count", 0) or 0),
        "no_current_section_visit_limit": int(status.get("no_current_section_visit_limit", 0) or 0),
        "no_current_event_count": int(status.get("no_current_event_count", 0) or 0),
        "low_conf_trigger_enabled": bool(status.get("low_conf_trigger_enabled", False)),
        "low_conf_triggered": bool(status.get("low_conf_triggered", False)),
        "low_conf_suppressed_reason": str(status.get("low_conf_suppressed_reason", "") or ""),
        "low_conf_window_count": int(status.get("low_conf_window_count", 0) or 0),
        "low_conf_window_size": int(status.get("low_conf_window_size", 0) or 0),
        "low_conf_required": int(status.get("low_conf_required", 0) or 0),
        "low_conf_low_count": int(status.get("low_conf_low_count", 0) or 0),
        "low_conf_last_confidence": status.get("low_conf_last_confidence"),
        "low_conf_last_valid": bool(status.get("low_conf_last_valid", False)),
        "low_conf_last_reason": str(status.get("low_conf_last_reason", "") or ""),
        "low_conf_section_visit_key": str(status.get("low_conf_section_visit_key", "") or ""),
        "low_conf_section_visit_count": int(status.get("low_conf_section_visit_count", 0) or 0),
        "low_conf_section_visit_limit": int(status.get("low_conf_section_visit_limit", 0) or 0),
        "low_conf_event_count": int(status.get("low_conf_event_count", 0) or 0),
        "low_conf_startup_grace_remaining_s": float(status.get("low_conf_startup_grace_remaining_s", 0.0) or 0.0),
        "low_conf_progress_allowed": bool(status.get("low_conf_progress_allowed", False)),
        "low_conf_progress_reason": str(status.get("low_conf_progress_reason", "") or ""),
        "low_conf_progress_started_t": float(status.get("low_conf_progress_started_t", 0.0) or 0.0),
        "low_conf_progress_trigger_source": str(status.get("low_conf_progress_trigger_source", "") or ""),
        "timeout_progress_allowed": bool(status.get("timeout_progress_allowed", False)),
        "timeout_progress_reason": str(status.get("timeout_progress_reason", "") or ""),
        "timeout_progress_started_t": float(status.get("timeout_progress_started_t", 0.0) or 0.0),
        "timeout_progress_trigger_source": str(status.get("timeout_progress_trigger_source", "") or ""),
        "timeout_progress_clear_reason": str(status.get("timeout_progress_clear_reason", "") or ""),
        "timeout_progress_clear_window_count": int(status.get("timeout_progress_clear_window_count", 0) or 0),
        "timeout_progress_clear_window_size": int(status.get("timeout_progress_clear_window_size", 0) or 0),
        "timeout_progress_clear_required": int(status.get("timeout_progress_clear_required", 0) or 0),
        "timeout_progress_clear_settled_count": int(status.get("timeout_progress_clear_settled_count", 0) or 0),
    }

def build_navigation_debug(pose, lidar_estimate, obstacles: list[dict], args) -> dict:
    if not getattr(args, "navigation_debug", True):
        return {"enabled": False}
    route = _project_pose(pose, args)
    localization_hold_status = _localization_hold_status(args)
    localization_hold_active = bool(localization_hold_status.get("active", False))
    state_mutations_allowed = _navigation_state_mutations_allowed(args)
    state_mutation_suppressed_reason = _navigation_state_mutation_suppressed_reason(args)
    _update_gate_lock_context(args)
    if not localization_hold_active:
        _update_route_progress(route, state_mutations_allowed, state_mutation_suppressed_reason)
    full_gate_lock = _all_gates_locked()
    free_race = _free_race_enabled(args)
    planner_obstacles = [] if free_race else _navigation_obstacle_slots(obstacles, route.direction, args)
    relevant = [] if full_gate_lock or free_race else _relevant_obstacles(planner_obstacles, route, args)
    planner_mode = getattr(args, "navigation_planner", "hybrid-waypoint")
    waypoints: list[Waypoint] = []
    global_replanned = False
    local_connector_mode = ""
    planner_debug: dict = {}
    if localization_hold_active:
        path, blocked, planner_cost, warnings = _reference_lane_preview(pose, route, args)
        mode = "localization_hold_debug"
        local_connector_mode = "localization_hold_freeze"
        planner_debug = {"localization_hold_freeze": True}
    elif planner_mode == "reference-lane":
        path, blocked, planner_cost, warnings = _reference_lane_preview(pose, route, args)
        mode = "reference_lane_preview_debug"
        local_connector_mode = "reference_lane"
    elif planner_mode == "state-lattice":
        nodes, best_idx, blocked, warnings = _plan_state_lattice(pose, route, relevant, args)
        path = _reconstruct_path(nodes, best_idx, route, args)
        planner_cost = nodes[best_idx].cost if nodes else 0.0
        if blocked and len(path) < 2:
            path, _fb_blocked, _fb_cost, fb_warnings = _reference_lane_fallback(pose, route, relevant, args)
            warnings = sorted(set(warnings + fb_warnings))[:30]
        mode = "state_lattice_debug"
        local_connector_mode = "state_lattice"
    else:
        path, blocked, planner_cost, warnings, waypoints, global_replanned, local_connector_mode, planner_debug = _plan_hybrid_waypoint(pose, route, planner_obstacles, relevant, args)
        mode = "hybrid_waypoint_debug"
    smart_parking_alignment_path, smart_parking_alignment = _smart_parking_alignment_path(route, planner_obstacles, args)
    if smart_parking_alignment_path and any(warning == "smart_parking_route_geometry" for warning in warnings):
        smart_parking_alignment = dict(smart_parking_alignment)
        smart_parking_alignment["route_geometry_active"] = True
    if localization_hold_active:
        corner_hold = color_hold = absence_hold = False
        corner_hold_reason = color_hold_reason = absence_hold_reason = ""
        corner_hold_section = color_hold_section = absence_hold_section = route.section
        corner_hold_remaining_s = color_hold_remaining_s = absence_hold_remaining_s = 0.0
        observation_approach = False
        color_blocked = False
        absence_blocked = False
        color_confirmation_status = {"enabled": _navigation_confirmations_active(args), "blocked_by_localization_hold": True}
        absence_confirmation_status = {"enabled": _navigation_confirmations_active(args), "blocked_by_localization_hold": True}
    else:
        corner_hold, corner_hold_reason, corner_hold_section, corner_hold_remaining_s, observation_approach = _corner_observation_hold(route, planner_obstacles, args, full_gate_lock)
        color_hold, color_hold_reason, color_hold_section, color_hold_remaining_s, _color_approach, color_blocked, color_confirmation_status = _color_confirmation_hold(route, relevant, pose, args, full_gate_lock, corner_hold, corner_hold_section, corner_hold_reason, measurement_obstacles=planner_obstacles, post_unpark_obstacles=planner_obstacles)
        absence_hold, absence_hold_reason, absence_hold_section, absence_hold_remaining_s, _absence_approach, absence_blocked, absence_confirmation_status = _absence_confirmation_hold(route, relevant, args, full_gate_lock)
    hold = localization_hold_active or corner_hold or color_hold or absence_hold
    hold_reason = "localization_hold" if localization_hold_active else corner_hold_reason if corner_hold else color_hold_reason if color_hold else absence_hold_reason
    hold_section = route.section if localization_hold_active else corner_hold_section if corner_hold else color_hold_section if color_hold else absence_hold_section
    hold_remaining_s = localization_hold_status.get("remaining_s", 0.0) if localization_hold_active else corner_hold_remaining_s if corner_hold else color_hold_remaining_s if color_hold else absence_hold_remaining_s
    lookahead_args = args
    if not free_race and bool(smart_parking_alignment.get("active", False)):
        lookahead_args = SimpleNamespace(**vars(args), _smart_parking_alignment_active=True)
    lookahead_distance_m, straightness_score, straightness_metrics = _adaptive_lookahead_distance(path, pose, lookahead_args)
    lookahead = _lookahead_point(path, args, lookahead_distance_m)
    curvature = _curvature_to_point(pose, lookahead)
    corridor_exit_distance = straightness_metrics.get("corridor_exit_distance_m")
    curvature_preview_cap, curvature_preview_status = _curvature_preview_speed_cap(path, args)
    yaw_rate_factor, yaw_rate_status = _adaptive_yaw_rate_factor(pose, args)
    lidar_conf = float(getattr(lidar_estimate, "confidence", 0.0) or 0.0)
    pose_ok = bool(getattr(lidar_estimate, "valid", False)) and lidar_conf >= float(args.navigation_min_pose_confidence)
    pose_confidence_progress_allowed = bool(localization_hold_status.get("low_conf_progress_allowed", False))
    pose_confidence_progress_reason = str(localization_hold_status.get("low_conf_progress_reason", "") or "")
    localization_timeout_progress_allowed = bool(localization_hold_status.get("timeout_progress_allowed", False))
    localization_timeout_progress_reason = str(localization_hold_status.get("timeout_progress_reason", "") or "")
    uncertainty_cap, uncertainty_reason, uncertainty_status = _uncertain_obstacle_speed_cap(pose, relevant, args)
    unknown_color_cap, unknown_color_reason, unknown_color_status = _unknown_color_speed_cap(relevant, args)
    tracking_factor, tracking_reason, tracking_status = _tracking_error_speed_factor(route, args)
    corner_transition_cap_mps, corner_transition_reason, corner_transition_status = (
        _obstacle_corner_transition_speed_cap(path, args, route.direction)
        if not free_race
        else (None, "", {"enabled": False, "active": False, "factor": 1.0})
    )
    speed_policy_status: dict = {
        "policy": "",
        "base_speed_mps": None,
        "cap_factor": 1.0,
        "cap_reason": "",
    }
    speed_cap = float(args.navigation_cruise_speed_mps)
    speed_reason = "cruise"
    if hold:
        speed_cap = 0.0
        speed_reason = hold_reason
    elif observation_approach:
        speed_cap = min(speed_cap, float(getattr(args, "navigation_corner_observation_approach_speed_mps", 0.14)))
        speed_reason = hold_reason
    elif color_blocked or absence_blocked:
        speed_cap = 0.0
        speed_reason = "color_conflict_close" if color_blocked else "obstacle_absence_close"
    elif blocked:
        speed_cap = min(speed_cap, float(args.navigation_low_confidence_speed_mps))
        speed_reason = "blocked"
    elif not pose_ok:
        speed_cap = min(speed_cap, float(args.navigation_low_confidence_speed_mps))
        speed_reason = "low_pose_confidence_timeout_progress" if pose_confidence_progress_allowed else "low_pose_confidence"
    elif localization_timeout_progress_allowed:
        speed_cap = min(speed_cap, float(args.navigation_low_confidence_speed_mps))
        speed_reason = "localization_timeout_progress"
    elif free_race:
        speed_cap = max(0.0, float(getattr(args, "free_race_speed_mps", 0.70)))
        speed_reason = "free_race_constant"
        speed_policy_status.update({"policy": "free_race_constant", "base_speed_mps": speed_cap})
    elif full_gate_lock and bool(getattr(args, "navigation_full_lock_speed", True)):
        speed_cap = max(0.0, float(getattr(args, "navigation_full_lock_speed_mps", 0.70)))
        speed_reason = "full_gate_lock"
        speed_policy_status.update({"policy": "full_gate_lock", "base_speed_mps": speed_cap})
    else:
        speed_cap, speed_reason, speed_policy_status = _obstacle_mode_speed_policy(
            route,
            planner_obstacles,
            relevant,
            uncertainty_cap,
            uncertainty_status,
            unknown_color_cap,
            args,
        )
    connector_caution_modes = {"recovery_connector", "reference_lane", "state_lattice", "blocked_route_fallback"}
    if (
        local_connector_mode in connector_caution_modes
        and speed_cap > 0.0
        and speed_reason not in ("color_conflict_close", "obstacle_absence_close", "blocked", "low_pose_confidence", "low_pose_confidence_timeout_progress", "localization_timeout_progress")
        and not hold
        and not color_blocked
        and not absence_blocked
    ):
        if local_connector_mode == "blocked_route_fallback":
            fallback_speed = float(getattr(args, "navigation_blocked_route_fallback_speed_mps", -1.0) or -1.0)
            connector_caution = max(0.0, fallback_speed) if fallback_speed >= 0.0 else max(0.0, float(getattr(args, "navigation_caution_speed_mps", 0.32)))
        else:
            connector_caution = max(0.0, float(getattr(args, "navigation_caution_speed_mps", 0.32)))
        if speed_cap > connector_caution:
            connector_original_speed_cap = speed_cap
            speed_cap = connector_caution
            existing_cap_reason = str(speed_policy_status.get("cap_reason", "") or "")
            speed_policy_status["cap_factor"] = (
                float(speed_policy_status.get("cap_factor", 1.0) or 1.0)
                * (speed_cap / connector_original_speed_cap if connector_original_speed_cap > 0.0 else 1.0)
            )
            speed_policy_status["cap_reason"] = "+".join(
                reason for reason in (existing_cap_reason, "connector_caution") if reason
            )
            speed_reason = f"{speed_reason}_connector_caution"
    if (
        not free_race
        and speed_cap > 0.0
        and speed_reason not in ("color_conflict_close", "obstacle_absence_close", "blocked", "low_pose_confidence", "low_pose_confidence_timeout_progress", "localization_timeout_progress")
        and not hold
        and not color_blocked
        and not absence_blocked
    ):
        original_speed_cap = speed_cap
        cap_reasons: list[str] = []
        existing_cap_factor = float(speed_policy_status.get("cap_factor", 1.0) or 1.0)
        existing_cap_reasons = [
            reason
            for reason in str(speed_policy_status.get("cap_reason", "") or "").split("+")
            if reason
        ]
        if tracking_reason and tracking_factor < 1.0:
            speed_cap *= tracking_factor
            cap_reasons.append(tracking_reason)
        corner_transition_effective_factor = 1.0
        if corner_transition_reason and corner_transition_cap_mps is not None and speed_cap > corner_transition_cap_mps:
            before_corner_cap = speed_cap
            speed_cap = corner_transition_cap_mps
            corner_transition_effective_factor = speed_cap / before_corner_cap if before_corner_cap > 0.0 else 1.0
            cap_reasons.append(corner_transition_reason)
        if speed_cap < original_speed_cap:
            combined_factor = speed_cap / original_speed_cap if original_speed_cap > 0.0 else 1.0
            speed_policy_status["cap_factor"] = existing_cap_factor * combined_factor
            speed_policy_status["cap_reason"] = "+".join(existing_cap_reasons + cap_reasons)
            speed_reason = f"{speed_reason}_{'+'.join(cap_reasons)}" if cap_reasons else speed_reason
        corner_transition_status["factor"] = corner_transition_effective_factor
        corner_transition_status["cap_mps"] = corner_transition_cap_mps
    obstacle_mode_max_speed_mps = None
    if not free_race:
        obstacle_mode_max_speed_mps = max(0.0, float(getattr(args, "navigation_obstacle_mode_max_speed_mps", 0.70)))
        speed_cap = min(speed_cap, obstacle_mode_max_speed_mps)
    active_wp = _waypoint_by_id(waypoints, _HYBRID_STATE.active_waypoint_id) if planner_mode == "hybrid-waypoint" else None
    preview_targets = _ordered_targets_from_active(waypoints, active_wp, max(2, int(getattr(args, "navigation_preview_waypoints", 10)))) if active_wp is not None else []
    target_waypoint_ids = [wp.id for wp in preview_targets[:3]]
    target_waypoint_reasons = [wp.reason for wp in preview_targets[:3]]
    connector_debug = planner_debug.get("connector", {}) if isinstance(planner_debug, dict) else {}
    if not isinstance(connector_debug, dict):
        connector_debug = {}
    preview_path = []
    if planner_mode == "hybrid-waypoint" and preview_targets:
        reached_id = _last_reached_waypoint_id(path, preview_targets)
        dotted_count = max(2, int(getattr(args, "navigation_preview_waypoints", 10)))
        dotted_targets = _targets_from_waypoint(waypoints, reached_id, dotted_count) if reached_id else preview_targets
        preview_path = _preview_path_from_targets(dotted_targets, args)
    free_race_finish = _free_race_finish_status(route, args, freeze=localization_hold_active)
    free_race_inner_route = _free_race_adaptive_status(args, route.direction)
    return {
        "enabled": True,
        "mode": mode,
        "direction": route.direction,
        "direction_source": "manual" if getattr(args, "navigation_direction", "auto") in ("clock", "counter") else "initial_lock" if getattr(args, "navigation_locked_direction", "") in ("CLOCK", "COUNTER") else "pose_auto",
        "section": route.section,
        "primitive": route.primitive,
        "route_s_m": route.s_m,
        "route_progress_m": _HYBRID_STATE.route_progress_m,
        "route_step_delta_m": _HYBRID_STATE.route_step_delta_m,
        "route_progress_mutations_allowed": _HYBRID_STATE.route_progress_mutations_allowed,
        "route_progress_baseline_synced": _HYBRID_STATE.route_progress_baseline_synced,
        "route_progress_suppressed_reason": _HYBRID_STATE.route_progress_suppressed_reason,
        "route_progress_suppressed_delta_m": _HYBRID_STATE.route_progress_suppressed_delta_m,
        "smart_parking_trigger": _smart_parking_trigger_status(route, args, freeze=localization_hold_active),
        "smart_parking_alignment": smart_parking_alignment,
        "free_race_finish_trigger": free_race_finish,
        "free_race_inner_route": free_race_inner_route,
        "along_m": route.local_s_m,
        "lateral_m": route.lateral_m,
        "heading_error_deg": route.heading_error_deg,
        "projection_error_m": route.distance_error_m,
        "horizon_m": float(args.navigation_horizon_m),
        "pose_confidence_ok": pose_ok,
        "pose_confidence_progress_allowed": pose_confidence_progress_allowed,
        "pose_confidence_progress_reason": pose_confidence_progress_reason,
        "localization_timeout_progress_allowed": localization_timeout_progress_allowed,
        "localization_timeout_progress_reason": localization_timeout_progress_reason,
        "pose_confidence": lidar_conf,
        "lidar_valid": bool(getattr(lidar_estimate, "valid", False)),
        "lidar_reason": str(getattr(lidar_estimate, "reason", "") or ""),
        "lidar_residual_m": float(getattr(lidar_estimate, "residual_m", 0.0) or 0.0),
        "lidar_dx_m": float(getattr(lidar_estimate, "dx", 0.0) or 0.0),
        "lidar_dy_m": float(getattr(lidar_estimate, "dy", 0.0) or 0.0),
        "lidar_dh_deg": float(getattr(lidar_estimate, "dh", 0.0) or 0.0),
        "lidar_matched_points": int(getattr(lidar_estimate, "matched_points", 0) or 0),
        "lidar_rejected_points": int(getattr(lidar_estimate, "rejected_points", 0) or 0),
        "lidar_visible_walls": list(getattr(lidar_estimate, "visible_walls", ()) or ()),
        "path": path,
        "preview_path": preview_path,
        "lookahead": lookahead,
        "lookahead_distance_m": lookahead_distance_m,
        "straightness_score": straightness_score,
        "adaptive_speed_score": straightness_score * yaw_rate_factor,
        "adaptive_yaw_rate": yaw_rate_status,
        "adaptive_yaw_rate_factor": yaw_rate_factor,
        "adaptive_yaw_rate_dps": yaw_rate_status.get("yaw_rate_dps"),
        "adaptive_yaw_rate_avg_dps": yaw_rate_status.get("yaw_rate_avg_dps"),
        "straightness_metrics": straightness_metrics,
        "adaptive_mode": straightness_metrics.get("mode", "legacy" if bool(getattr(args, "navigation_adaptive_lookahead", False)) else "off"),
        "speed_policy": speed_policy_status.get("policy", ""),
        "speed_policy_base_mps": speed_policy_status.get("base_speed_mps"),
        "speed_policy_cap_factor": speed_policy_status.get("cap_factor", 1.0),
        "speed_policy_cap_reason": speed_policy_status.get("cap_reason", ""),
        "speed_policy_next_obstacle_id": speed_policy_status.get("next_obstacle_id", ""),
        "speed_policy_next_obstacle_delta_m": speed_policy_status.get("next_obstacle_delta_m"),
        "speed_policy_next_obstacle_color": speed_policy_status.get("next_obstacle_color", ""),
        "speed_policy_next_obstacle_gate_locked": speed_policy_status.get("next_obstacle_gate_locked", False),
        "speed_policy_current_section": speed_policy_status.get("current_section", ""),
        "speed_policy_current_section_ready": speed_policy_status.get("current_section_ready", False),
        "speed_policy_current_section_has_obstacle": speed_policy_status.get("current_section_has_obstacle", False),
        "speed_policy_current_section_obstacle_id": speed_policy_status.get("current_section_obstacle_id", ""),
        "speed_policy_current_section_obstacle_color": speed_policy_status.get("current_section_obstacle_color", ""),
        "speed_policy_next_section": speed_policy_status.get("next_section", ""),
        "speed_policy_next_section_ready": speed_policy_status.get("next_section_ready", False),
        "speed_policy_next_section_has_obstacle": speed_policy_status.get("next_section_has_obstacle", False),
        "speed_policy_next_section_obstacle_id": speed_policy_status.get("next_section_obstacle_id", ""),
        "speed_policy_next_section_obstacle_color": speed_policy_status.get("next_section_obstacle_color", ""),
        "obstacle_speed_lookahead_m": straightness_metrics.get("obstacle_speed_lookahead_m"),
        "obstacle_speed_lookahead_forward_mps": straightness_metrics.get("obstacle_speed_lookahead_forward_mps"),
        "obstacle_speed_lookahead_scale": straightness_metrics.get("obstacle_speed_lookahead_scale"),
        "tracking_error_speed_factor": tracking_status.get("factor", 1.0),
        "tracking_error_speed_reason": tracking_status.get("reason", ""),
        "tracking_error_speed": tracking_status,
        "corner_transition_speed_factor": corner_transition_status.get("factor", 1.0),
        "corner_transition_speed_reason": corner_transition_status.get("reason", ""),
        "corner_transition_speed_cap_mps": corner_transition_status.get("cap_mps", corner_transition_status.get("max_mps")),
        "corner_transition_speed_active": corner_transition_status.get("active", False),
        "corner_transition_speed": corner_transition_status,
        "corridor_half_angle_deg": straightness_metrics.get("corridor_half_angle_deg"),
        "corridor_half_width_m": straightness_metrics.get("corridor_half_width_m"),
        "corridor_heading_limit_deg": straightness_metrics.get("corridor_heading_limit_deg"),
        "corridor_curvature_limit": straightness_metrics.get("corridor_curvature_limit"),
        "corridor_heading_error_deg": straightness_metrics.get("corridor_heading_error_deg"),
        "corridor_max_curvature_seen": straightness_metrics.get("corridor_max_curvature_seen"),
        "corridor_min_lookahead_m": straightness_metrics.get("corridor_min_lookahead_m"),
        "corridor_max_lookahead_m": straightness_metrics.get("corridor_max_lookahead_m"),
        "corridor_valid_lookahead_m": straightness_metrics.get("corridor_valid_lookahead_m"),
        "corridor_raw_lookahead_m": straightness_metrics.get("corridor_raw_lookahead_m"),
        "corridor_applied_lookahead_m": straightness_metrics.get("corridor_applied_lookahead_m"),
        "corridor_speed_scale": straightness_metrics.get("corridor_speed_scale"),
        "corridor_exit_distance_m": straightness_metrics.get("corridor_exit_distance_m"),
        "corridor_first_outside_m": straightness_metrics.get("corridor_first_outside_m"),
        "corridor_best_candidate_m": straightness_metrics.get("corridor_best_candidate_m"),
        "corridor_inside_count": straightness_metrics.get("corridor_inside_count"),
        "corridor_consensus_window": straightness_metrics.get("corridor_consensus_window"),
        "corridor_consensus_required": straightness_metrics.get("corridor_consensus_required"),
        "corridor_continuity_start_m": straightness_metrics.get("corridor_continuity_start_m"),
        "corridor_high_curvature_distance_m": None,
        "corridor_corner_distance_m": corridor_exit_distance,
        "curvature_preview": curvature_preview_status,
        "curvature_preview_distance_m": curvature_preview_status.get("distance_m"),
        "curvature_preview_speed_cap_mps": curvature_preview_status.get("speed_cap_mps"),
        "corridor_brake_distance_m": None,
        "corridor_brake_speed_cap_mps": None,
        "corridor_corner_speed_cap_mps": None,
        "corridor_rate_limited": straightness_metrics.get("corridor_rate_limited"),
        "corridor_increase_rate_mps": straightness_metrics.get("corridor_increase_rate_mps"),
        "corridor_points": straightness_metrics.get("corridor_points"),
        "curvature": curvature,
        "speed_cap_mps": speed_cap,
        "speed_reason": speed_reason,
        "obstacle_mode_max_speed_mps": obstacle_mode_max_speed_mps,
        "uncertain_obstacle": uncertainty_status,
        "unknown_color_obstacle": unknown_color_status,
        "hold": hold,
        "hold_reason": hold_reason,
        "hold_section": hold_section,
        "hold_remaining_s": hold_remaining_s,
        "localization_hold": localization_hold_status,
        "confirmations_active": _navigation_confirmations_active(args),
        "confirmations_active_reason": _navigation_confirmations_active_reason(args),
        "timing_warning": str(getattr(args, "navigation_timing_warning", "") or ""),
        "party_mode": bool(getattr(args, "navigation_party_mode", False)),
        "color_confirmation": color_confirmation_status,
        "absence_confirmation": absence_confirmation_status,
        "observation_approach": observation_approach,
        "planner_cost": planner_cost,
        "blocked": bool(blocked or color_blocked or absence_blocked),
        "blocked_reason": "color_conflict_close" if color_blocked else "obstacle_absence_close" if absence_blocked else ("blocked" if blocked else ""),
        "warnings": warnings,
        "global_replanned": global_replanned,
        "global_replan_count": _HYBRID_STATE.replan_count,
        "full_gate_lock": full_gate_lock,
        "locked_gate_count": len(_HYBRID_STATE.locked_gate_choices),
        "pending_unknown_gate_count": len(_HYBRID_STATE.pending_unknown_gate_choices),
        "pending_unknown_gate_last_action": _HYBRID_STATE.pending_unknown_gate_last_action,
        "pending_unknown_gate_last_key": _HYBRID_STATE.pending_unknown_gate_last_key,
        "pending_unknown_gate_last_reason": _HYBRID_STATE.pending_unknown_gate_last_reason,
        "gate_lock_last_event": _HYBRID_STATE.gate_lock_last_event,
        "gate_lock_last_section": _HYBRID_STATE.gate_lock_last_section,
        "gate_lock_last_gate_idx": _HYBRID_STATE.gate_lock_last_gate_idx,
        "gate_lock_last_lane_name": _HYBRID_STATE.gate_lock_last_lane_name,
        "gate_lock_last_reason": _HYBRID_STATE.gate_lock_last_reason,
        "gate_lock_last_decision_reason": _HYBRID_STATE.gate_lock_last_decision_reason,
        "gate_lock_last_obstacle_ids": _HYBRID_STATE.gate_lock_last_obstacle_ids,
        "gate_lock_last_route_progress_m": _HYBRID_STATE.gate_lock_last_route_progress_m,
        "gate_lock_last_route_step_delta_m": _HYBRID_STATE.gate_lock_last_route_step_delta_m,
        "gate_lock_last_active_waypoint_id": _HYBRID_STATE.gate_lock_last_active_waypoint_id,
        "gate_lock_last_startup_status": _HYBRID_STATE.gate_lock_last_startup_status,
        "gate_lock_last_startup_state": _HYBRID_STATE.gate_lock_last_startup_state,
        "gate_lock_last_anchor_version": _HYBRID_STATE.gate_lock_last_anchor_version,
        "gate_lock_last_anchor_source": _HYBRID_STATE.gate_lock_last_anchor_source,
        "gate_lock_last_anchor_jump_m": _HYBRID_STATE.gate_lock_last_anchor_jump_m,
        "gate_lock_last_anchor_jump_h_deg": _HYBRID_STATE.gate_lock_last_anchor_jump_h_deg,
        "gate_lock_last_initial_pose_locked": _HYBRID_STATE.gate_lock_last_initial_pose_locked,
        "gate_lock_event_count": _HYBRID_STATE.gate_lock_event_count,
        "gate_lock_recent_events": "|".join(str(item) for item in _HYBRID_STATE.gate_lock_recent_events),
        "gate_lock_suppressed_reason": _HYBRID_STATE.gate_lock_suppressed_reason,
        "local_connector_mode": local_connector_mode,
        "blocked_route_fallback_active": bool(_HYBRID_STATE.blocked_route_fallback_active) if planner_mode == "hybrid-waypoint" else False,
        "blocked_route_fallback_age_s": (
            max(0.0, time.monotonic() - _HYBRID_STATE.blocked_route_fallback_started_t)
            if planner_mode == "hybrid-waypoint" and _HYBRID_STATE.blocked_route_fallback_started_t > 0.0
            else 0.0
        ),
        "blocked_route_fallback_reason": _HYBRID_STATE.blocked_route_fallback_reason if planner_mode == "hybrid-waypoint" else "",
        "blocked_route_fallback_projection_error_m": _HYBRID_STATE.fixed_route_projection_error_m if planner_mode == "hybrid-waypoint" else 0.0,
        "fixed_route_projection_error_m": _HYBRID_STATE.fixed_route_projection_error_m if planner_mode == "hybrid-waypoint" else 0.0,
        "fixed_route_projected_s_m": _HYBRID_STATE.fixed_route_projected_s_m if planner_mode == "hybrid-waypoint" else 0.0,
        "fixed_route_projected_heading_deg": _HYBRID_STATE.fixed_route_projected_heading_deg if planner_mode == "hybrid-waypoint" else 0.0,
        "active_waypoint_id": _HYBRID_STATE.active_waypoint_id if planner_mode == "hybrid-waypoint" else "",
        "active_waypoint_reason": active_wp.reason if active_wp is not None else "",
        "active_waypoint_distance_m": _HYBRID_STATE.active_waypoint_distance_m if planner_mode == "hybrid-waypoint" else 0.0,
        "active_waypoint_signed_forward_m": _HYBRID_STATE.active_waypoint_signed_forward_m if planner_mode == "hybrid-waypoint" else 0.0,
        "active_waypoint_line_armed": _HYBRID_STATE.active_waypoint_line_armed if planner_mode == "hybrid-waypoint" else False,
        "active_waypoint_passed_reason": _HYBRID_STATE.active_waypoint_passed_reason if planner_mode == "hybrid-waypoint" else "",
        "active_waypoint_reset_reason": _HYBRID_STATE.active_waypoint_reset_reason if planner_mode == "hybrid-waypoint" else "",
        "active_waypoint_reset_previous_id": _HYBRID_STATE.active_waypoint_reset_previous_id if planner_mode == "hybrid-waypoint" else "",
        "active_waypoint_consensus": {
            "required": _vote_required(args),
            "capture": sum(1 for item in _HYBRID_STATE.active_waypoint_capture_votes if item),
            "arm": sum(1 for item in _HYBRID_STATE.active_waypoint_arm_votes if item),
            "pass_line": sum(1 for item in _HYBRID_STATE.active_waypoint_pass_line_votes if item),
            "route": sum(1 for item in _HYBRID_STATE.active_waypoint_route_votes if item),
        } if planner_mode == "hybrid-waypoint" else {},
        "target_waypoint_ids": target_waypoint_ids,
        "target_waypoint_reasons": target_waypoint_reasons,
        "planner_diagnostics": planner_debug,
        "connector_target_id": connector_debug.get("target_id", ""),
        "connector_target_reason": connector_debug.get("target_reason", ""),
        "connector_chord_m": connector_debug.get("chord_m", ""),
        "connector_heading_delta_deg": connector_debug.get("heading_delta_deg", ""),
        "connector_target_route_delta_m": connector_debug.get("target_route_delta_m", ""),
        "connector_start_lateral_m": connector_debug.get("start_lateral_m", ""),
        "connector_start_heading_error_deg": connector_debug.get("start_heading_error_deg", ""),
        "connector_attempt_count": connector_debug.get("attempt_count", ""),
        "connector_failure_reasons": connector_debug.get("failure_reasons", []),
        "connector_attempts": connector_debug.get("attempts", []),
        "fixed_route_failed": planner_debug.get("fixed_route_failed", False) if isinstance(planner_debug, dict) else False,
        "fixed_route_warnings": planner_debug.get("fixed_route_warnings", []) if isinstance(planner_debug, dict) else [],
        "extension_warnings": planner_debug.get("extension_warnings", []) if isinstance(planner_debug, dict) else [],
        "gate_decisions": _gate_decision_payload(_gate_decisions(route.direction, obstacles, args)) if planner_mode == "hybrid-waypoint" else [],
        "locked_gate_choices": [
            {"section": section, "gate_idx": gate_idx, **data}
            for (section, gate_idx), data in sorted(_HYBRID_STATE.locked_gate_choices.items())
        ],
        "pending_unknown_gate_choices": [
            {"section": section, "gate_idx": gate_idx, **data}
            for (section, gate_idx), data in sorted(_HYBRID_STATE.pending_unknown_gate_choices.items())
        ],
        "waypoints": [_waypoint_dict(wp) for wp in waypoints[:80]],
        "constraint_warnings": warnings[:20],
        "obstacle_decisions": _obstacle_decisions(relevant, route, args),
    }
