#!/usr/bin/env python3
"""Summarize FE26 camera color classifier CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict


def as_float(value, default=float("nan")) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def finite_values(rows, key: str) -> list[float]:
    vals = [as_float(row.get(key)) for row in rows]
    return [v for v in vals if math.isfinite(v)]


def fmt_stat(values: list[float], digits: int = 3) -> str:
    if not values:
        return "--"
    return (
        f"mean={statistics.mean(values):.{digits}f} "
        f"med={statistics.median(values):.{digits}f} "
        f"min={min(values):.{digits}f} "
        f"max={max(values):.{digits}f}"
    )


def label_flips(rows: list[dict]) -> tuple[int, int]:
    prev = ""
    flips = 0
    samples = 0
    for row in sorted(rows, key=lambda item: as_float(item.get("t"), 0.0)):
        label = str(row.get("label", ""))
        if label not in ("RED", "GREEN"):
            continue
        samples += 1
        if prev and label != prev:
            flips += 1
        prev = label
    return flips, samples


def first_stable_sample(rows: list[dict], required: int) -> str:
    streak_label = ""
    streak = 0
    for idx, row in enumerate(sorted(rows, key=lambda item: as_float(item.get("t"), 0.0)), start=1):
        label = str(row.get("label", ""))
        if label not in ("RED", "GREEN"):
            streak_label = ""
            streak = 0
            continue
        if label == streak_label:
            streak += 1
        else:
            streak_label = label
            streak = 1
        if streak >= required:
            return f"{idx} ({label})"
    return "--"


def summarize(path: str, stable_required: int) -> str:
    with open(path, newline="") as fp:
        rows = list(csv.DictReader(fp))

    out: list[str] = []
    out.append(f"Camera color log: {path}")
    out.append(f"rows: {len(rows)}")
    if not rows:
        return "\n".join(out)

    labels = Counter(str(row.get("label", "")) for row in rows)
    classifier_labels = Counter(str(row.get("classifier_label", "")) for row in rows if row.get("classifier_label", ""))
    accepted = sum(1 for row in rows if str(row.get("accepted", "")) in ("1", "True", "true"))
    out.append("labels: " + ", ".join(f"{key}={labels[key]}" for key in sorted(labels)))
    if classifier_labels:
        out.append("classifier_labels: " + ", ".join(f"{key}={classifier_labels[key]}" for key in sorted(classifier_labels)))
        out.append(f"accepted_red_green: {accepted}")
    out.append(f"confidence: {fmt_stat(finite_values(rows, 'confidence'))}")
    out.append(f"valid_ratio: {fmt_stat(finite_values(rows, 'valid_ratio'))}")
    out.append(f"dominance_margin: {fmt_stat(finite_values(rows, 'dominance_margin'))}")
    if any(row.get("excess_green_med", "") for row in rows):
        out.append(f"excess_green_med: {fmt_stat(finite_values(rows, 'excess_green_med'))}")
        out.append(f"excess_red_med: {fmt_stat(finite_values(rows, 'excess_red_med'))}")
        out.append(f"g_norm_med: {fmt_stat(finite_values(rows, 'g_norm_med'))}")
        out.append(f"r_norm_med: {fmt_stat(finite_values(rows, 'r_norm_med'))}")
        out.append(f"green_rescue_active: {sum(1 for row in rows if str(row.get('green_rescue_active', '')) in ('1', 'True', 'true'))}")
        out.append(f"red_suppressed: {sum(1 for row in rows if str(row.get('red_suppressed', '')) in ('1', 'True', 'true'))}")
        out.append(f"weak_red_rejected: {sum(1 for row in rows if str(row.get('weak_red_rejected', '')) in ('1', 'True', 'true'))}")
        out.append(f"raw_red_candidate: {sum(1 for row in rows if str(row.get('raw_red_candidate', '')) in ('1', 'True', 'true'))}")
        out.append(f"strong_red_active: {sum(1 for row in rows if str(row.get('strong_red_active', '')) in ('1', 'True', 'true'))}")
        out.append(f"red_strength_rejected: {sum(1 for row in rows if str(row.get('red_strength_rejected', '')) in ('1', 'True', 'true'))}")
    out.append(f"frame_age_ms: {fmt_stat(finite_values(rows, 'age_ms'), digits=2)}")
    out.append(f"async_queue_age_ms: {fmt_stat(finite_values(rows, 'async_queue_age_ms'), digits=2)}")
    out.append(f"async_total_age_ms: {fmt_stat(finite_values(rows, 'async_total_age_ms'), digits=2)}")

    slot_rows: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        slot_rows[str(row.get("slot_id", ""))].append(row)

    out.append("")
    out.append("Per slot:")
    for slot_id in sorted(slot_rows):
        group = slot_rows[slot_id]
        if not slot_id:
            slot_id = "--"
        counts = Counter(str(row.get("label", "")) for row in group)
        flips, rg_samples = label_flips(group)
        other_rate = counts.get("OTHER", 0) / max(len(group), 1)
        out.append(
            f"  {slot_id}: n={len(group)} "
            f"RED={counts.get('RED', 0)} GREEN={counts.get('GREEN', 0)} OTHER={counts.get('OTHER', 0)} "
            f"other_rate={other_rate:.3f} flips={flips}/{rg_samples} "
            f"first_stable={first_stable_sample(group, stable_required)}"
        )
        out.append(f"    conf {fmt_stat(finite_values(group, 'confidence'))}")
        out.append(f"    red_ratio {fmt_stat(finite_values(group, 'red_ratio'))}")
        out.append(f"    green_ratio {fmt_stat(finite_values(group, 'green_ratio'))}")
        out.append(f"    valid_ratio {fmt_stat(finite_values(group, 'valid_ratio'))}")
        if any(row.get("excess_green_med", "") for row in group):
            out.append(f"    excess_green {fmt_stat(finite_values(group, 'excess_green_med'))}")
            out.append(f"    excess_red {fmt_stat(finite_values(group, 'excess_red_med'))}")
            out.append(f"    g_norm {fmt_stat(finite_values(group, 'g_norm_med'))}")
            out.append(f"    r_norm {fmt_stat(finite_values(group, 'r_norm_med'))}")
            rescue = sum(1 for row in group if str(row.get("green_rescue_active", "")) in ("1", "True", "true"))
            suppressed = sum(1 for row in group if str(row.get("red_suppressed", "")) in ("1", "True", "true"))
            weak_red = sum(1 for row in group if str(row.get("weak_red_rejected", "")) in ("1", "True", "true"))
            raw_red = sum(1 for row in group if str(row.get("raw_red_candidate", "")) in ("1", "True", "true"))
            strong_red = sum(1 for row in group if str(row.get("strong_red_active", "")) in ("1", "True", "true"))
            strength_rejected = sum(1 for row in group if str(row.get("red_strength_rejected", "")) in ("1", "True", "true"))
            out.append(
                f"    green_rescue={rescue} red_suppressed={suppressed} weak_red_rejected={weak_red} "
                f"raw_red={raw_red} strong_red={strong_red} red_strength_rejected={strength_rejected}"
            )

    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help="Camera color debug CSV files")
    parser.add_argument("--stable-required", type=int, default=3, help="Consecutive RED/GREEN samples counted as first stable color")
    args = parser.parse_args()

    for idx, path in enumerate(args.logs):
        if idx:
            print()
            print("=" * 72)
        print(summarize(path, max(1, args.stable_required)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
