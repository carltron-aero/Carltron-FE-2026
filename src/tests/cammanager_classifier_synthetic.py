#!/usr/bin/env python3
"""Synthetic checks for the CamManager robust color classifier."""

from __future__ import annotations

import argparse
import sys
import types

import cv2
import numpy as np


if "libcamera" not in sys.modules:
    sys.modules["libcamera"] = types.SimpleNamespace()

from src.prod.robot_core.cammanager import _classify_bgr_vectors  # noqa: E402


def bgr_from_hsv(h_values, s=220, v=180):
    hsv = np.array([[[int(h), int(s), int(v)]] for h in h_values], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).reshape(-1, 3)
    return bgr[:, 0], bgr[:, 1], bgr[:, 2]


def run_bgr_case(name: str, b, g, r, expected: str, y_value: int = 120) -> bool:
    y = np.full_like(b, y_value)
    label, confidence, stats = _classify_bgr_vectors(y, b, g, r)
    ok = label == expected
    print(
        f"{name}: label={label} expected={expected} conf={confidence:.3f} "
        f"R={stats.get('red_ratio', 0.0):.2f} G={stats.get('green_ratio', 0.0):.2f} "
        f"rN={stats.get('r_norm_med', 0.0):.2f} labA={stats.get('lab_a_med', 0.0):.1f} "
        f"xG={stats.get('excess_green_med', 0.0):.2f} xR={stats.get('excess_red_med', 0.0):.2f} "
        f"rawR={stats.get('raw_red_candidate', 0)} strongR={stats.get('strong_red_active', 0)} "
        f"{'OK' if ok else 'FAIL'}"
    )
    return ok


def run_case(name: str, h_values, expected: str, s=220, v=180, y_value: int = 120) -> bool:
    b, g, r = bgr_from_hsv(h_values, s=s, v=v)
    return run_bgr_case(name, b, g, r, expected, y_value=y_value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    green_b, green_g, green_r = bgr_from_hsv([65] * 48, s=210, v=85)
    red_b, red_g, red_r = bgr_from_hsv([2] * 12, s=220, v=170)
    black_b = np.full(60, 8, dtype=np.uint8)
    black_g = np.full(60, 8, dtype=np.uint8)
    black_r = np.full(60, 8, dtype=np.uint8)
    gray_b = np.full(60, 60, dtype=np.uint8)
    gray_g = np.full(60, 60, dtype=np.uint8)
    gray_r = np.full(60, 60, dtype=np.uint8)
    checks = [
        run_case("red_wrap", [2] * 30 + [178] * 30, "RED"),
        run_case("moderate_red", [174] * 60, "RED", s=110, v=140, y_value=110),
        run_case("log_like_real_red_lower_bound", [174] * 60, "RED", s=145, v=95, y_value=90),
        run_case("weak_red_tinted_wall", [174] * 60, "OTHER", s=65, v=145, y_value=120),
        run_case("log_like_false_red_green_slot_a", [174] * 60, "OTHER", s=89, v=55, y_value=55),
        run_case("log_like_false_red_green_slot_b", [12] * 60, "OTHER", s=144, v=42, y_value=42),
        run_case("green", [65] * 60, "GREEN"),
        run_case("dark_green", [65] * 60, "GREEN", s=220, v=80, y_value=70),
        run_case("bright_green", [65] * 60, "GREEN", s=230, v=230, y_value=180),
        run_case("mixed_red_green", [2] * 25 + [65] * 25, "OTHER"),
        run_bgr_case(
            "green_contaminated_red",
            np.concatenate([green_b, red_b]),
            np.concatenate([green_g, red_g]),
            np.concatenate([green_r, red_r]),
            "GREEN",
            y_value=95,
        ),
        run_bgr_case("black_wall", black_b, black_g, black_r, "OTHER", y_value=12),
        run_bgr_case("gray_wall", gray_b, gray_g, gray_r, "OTHER", y_value=60),
        run_case("saturated_blue", [115] * 60, "OTHER"),
    ]
    return 0 if all(checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
