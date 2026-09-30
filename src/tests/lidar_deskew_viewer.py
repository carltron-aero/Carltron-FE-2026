#!/usr/bin/env python3
"""Offline viewer for LiDAR deskew diagnostic captures.

Run with a diagnostic index produced by --lidar-deskew-diagnostic-log:

    python3 src/tests/lidar_deskew_viewer.py logs/latest/lidar_deskew_debug_index.json

The viewer is intentionally offline-only. It does not start ROS, access robot
hardware, or change runtime correction behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import socketserver
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from analyze_lidar_deskew_debug import (
    FIELD_HALF_M,
    _frame_beam_time_order,
    _frame_motion,
    _load_frames,
    _model_points,
    _raw_range_seam_metric,
    _same_wall_field_metric,
    _same_wall_line_metric,
    _seam_window_points,
)


MODELS = [
    ("stored", "Stored", "#56c2ff"),
    ("current_recomputed", "Configured", "#79e07f"),
    ("forward_beam_order", "Forward Time", "#9bb2ff"),
    ("reverse_beam_order", "Reverse Time", "#ffb64d"),
    ("no_deskew", "No Deskew", "#f26868"),
    ("header_as_end", "Header End", "#b68cff"),
    ("header_as_mid", "Header Mid", "#d1c7ff"),
]

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>LiDAR Deskew Viewer</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #101418;
    --panel: #171d22;
    --panel2: #1f272e;
    --text: #e8edf2;
    --muted: #94a4b4;
    --line: #2d3943;
    --accent: #ffb64d;
    font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--text); height: 100vh; overflow: hidden; }
  .app { display: grid; grid-template-columns: minmax(520px, 1fr) 390px; height: 100vh; }
  main { min-width: 0; padding: 14px; display: flex; flex-direction: column; gap: 10px; }
  aside { border-left: 1px solid var(--line); background: var(--panel); overflow: auto; padding: 14px; }
  canvas { width: 100%; flex: 1; min-height: 420px; background: #0b0f13; border: 1px solid var(--line); }
  .toolbar, .section { background: var(--panel); border: 1px solid var(--line); padding: 10px; }
  .toolbar { display: grid; grid-template-columns: auto auto 1fr auto; gap: 8px; align-items: center; }
  button, select, input[type="number"] {
    background: var(--panel2); color: var(--text); border: 1px solid var(--line); min-height: 32px; padding: 0 10px;
  }
  button { cursor: pointer; }
  input[type="range"] { width: 100%; }
  h1 { margin: 0 0 10px; font-size: 18px; font-weight: 650; }
  h2 { margin: 14px 0 8px; font-size: 13px; color: #c9d6e0; text-transform: uppercase; letter-spacing: 0; }
  .kv { display: grid; grid-template-columns: minmax(120px, 1fr) minmax(110px, auto); gap: 6px 12px; font-size: 13px; }
  .kv div:nth-child(odd) { color: var(--muted); }
  .models { display: grid; gap: 6px; }
  label.model { display: flex; align-items: center; gap: 8px; font-size: 13px; }
  .swatch { width: 14px; height: 14px; border: 1px solid #0008; }
  .small { color: var(--muted); font-size: 12px; }
  .metric-table { width: 100%; border-collapse: collapse; font-size: 12px; }
  .metric-table th, .metric-table td { border-bottom: 1px solid var(--line); padding: 5px 4px; text-align: right; }
  .metric-table th:first-child, .metric-table td:first-child { text-align: left; }
  .winner { color: var(--accent); font-weight: 700; }
  .bad { color: #ff8a8a; }
  .good { color: #9af29a; }
  .control { display: grid; grid-template-columns: 64px 1fr 78px; gap: 8px; align-items: center; margin: 7px 0; font-size: 13px; }
  .control input[type="number"] { width: 78px; padding: 0 6px; text-align: right; }
  .actions { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 7px; margin-top: 10px; }
  .pill { display: inline-block; border: 1px solid var(--line); background: var(--panel2); padding: 2px 7px; font-size: 12px; color: var(--muted); }
  .fit-row { display: grid; grid-template-columns: 1fr auto; gap: 8px; align-items: center; margin: 8px 0; }
  .fit-row select { width: 100%; }
  .tune-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin: 8px 0; }
  .tune-controls { display: grid; gap: 7px; margin-top: 8px; }
  .tune-control { display: grid; grid-template-columns: 112px 1fr 82px 34px; gap: 7px; align-items: center; font-size: 12px; }
  .tune-control label { color: var(--muted); }
  .tune-control input[type="number"] { width: 82px; padding: 0 6px; text-align: right; }
  pre.params { white-space: pre-wrap; color: #cdd8e2; background: #11161b; border: 1px solid var(--line); padding: 8px; font-size: 11px; max-height: 180px; overflow: auto; }
  @media (max-width: 900px) {
    body { overflow: auto; }
    .app { display: block; height: auto; }
    main { height: 72vh; }
    aside { border-left: 0; border-top: 1px solid var(--line); }
  }
</style>
</head>
<body>
<div class="app">
  <main>
    <div class="toolbar">
      <button id="prev">Prev</button>
      <button id="next">Next</button>
      <input id="frame" type="range" min="0" value="0" />
      <select id="jump">
        <option value="">Jump...</option>
      </select>
    </div>
    <canvas id="map" width="1100" height="900"></canvas>
  </main>
  <aside>
    <h1>LiDAR Deskew Viewer</h1>
    <div class="section">
      <div class="kv" id="meta"></div>
    </div>
    <h2>Models</h2>
    <div class="section models" id="models"></div>
    <h2>Display</h2>
    <div class="section">
      <label class="model"><input id="showSeam" type="checkbox" checked /> Highlight seam windows</label>
      <label class="model"><input id="showFit" type="checkbox" checked /> Draw same-wall seam fit</label>
      <label class="model"><input id="showRobot" type="checkbox" checked /> Draw poses</label>
      <label class="model"><input id="showLabels" type="checkbox" checked /> Draw labels</label>
      <label class="model"><input id="showManual" type="checkbox" checked /> Draw manual aligned scan</label>
      <label class="model"><input id="showPaths" type="checkbox" checked /> Draw OTOS/fused/corrected paths</label>
      <label class="model"><input id="showModelFit" type="checkbox" checked /> Draw fitted OTOS model</label>
      <label class="model"><input id="showField" type="checkbox" checked /> Draw full field background</label>
    </div>
    <h2>OTOS Model Fits</h2>
    <div class="section">
      <div class="small">Offline fits compare OTOS-only pose models against fused pose during moving navigation frames. Most XY fits keep heading unchanged; the offset-h simulator also shifts reported heading.</div>
      <div class="fit-row"><select id="modelFitSelect"></select><button id="exportModelFit">Export</button></div>
      <div class="small" id="modelFitStatus"></div>
      <table class="metric-table" id="modelFitMetrics"></table>
      <div class="tune-actions"><button id="resetModelFit">Reset Sliders</button><button id="fitModelToCurrent">Load Fitted Defaults</button></div>
      <div class="small" id="modelFitTuneStatus"></div>
      <div class="tune-controls" id="modelFitTuneControls"></div>
      <pre class="params" id="modelFitParams"></pre>
    </div>
    <h2>Manual Alignment</h2>
    <div class="section">
      <div class="small">Adjust scans until the wall points line up with the fixed field. Edits are staged immediately; Save writes the complete run alignment file.</div>
      <div class="control"><span>X</span><input id="manualX" type="range" min="-2.000" max="2.000" step="0.001" value="0"><input id="manualXNum" type="number" step="0.001" value="0"></div>
      <div class="control"><span>Y</span><input id="manualY" type="range" min="-2.000" max="2.000" step="0.001" value="0"><input id="manualYNum" type="number" step="0.001" value="0"></div>
      <div class="control"><span>Heading</span><input id="manualH" type="range" min="-12.0" max="12.0" step="0.05" value="0"><input id="manualHNum" type="number" step="0.05" value="0"></div>
      <div class="actions"><button id="saveManual">Save All</button><button id="deleteManual">Delete Frame</button><button id="zeroManual">Zero Frame</button></div>
      <div class="small" id="manualStatus"></div>
    </div>
    <h2>Metrics</h2>
    <div class="section">
      <table class="metric-table" id="metrics"></table>
    </div>
  </aside>
</div>
<script>
const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const frameSlider = document.getElementById("frame");
const jump = document.getElementById("jump");
const meta = document.getElementById("meta");
const metricsEl = document.getElementById("metrics");
const modelsEl = document.getElementById("models");
const colors = {};
const enabled = {};
let state = null;
let current = 0;
let currentFrame = null;
let manual = {dx_m: 0, dy_m: 0, dh_deg: 0};
let manualDirty = false;
let annotations = {};
let annotationsDirty = false;
let sampleByIndex = {};
let selectedFit = "";
let modelFitEdits = {};
let modelFitTouched = {};
const FIELD_HALF_M = 1.5;
const INNER_FAR_HALF_M = 0.5;
const INNER_CLOSE_HALF_M = 0.9;
const GATE_ALONGS_M = [-0.5, 0.0, 0.5];
const OBSTACLE_ROWS_M = [0.40, 0.60];
const OBSTACLE_SLOT_RADIUS_M = 0.05;
const PARKING_GAP_M = 0.285;
const PARKING_RIGHT_X_M = 0.5;
const PARKING_CENTER_X_M = PARKING_RIGHT_X_M - PARKING_GAP_M / 2;
const PARKING_WALL_LENGTH_M = 0.20;

function fmt(v, digits=3) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "--";
  return Number(v).toFixed(digits);
}
function fmtPose(pose) {
  if (!pose || pose.x === null || pose.y === null || pose.h === null) return "--";
  return `${fmt(pose.x, 3)}, ${fmt(pose.y, 3)}, ${fmt(pose.h, 1)}deg`;
}
function fmtLidarPose(pose) {
  if (!pose) return "--";
  if (!pose.valid) return pose.reason || "invalid";
  return `${fmtPose(pose)} c=${fmt(pose.confidence, 2)}`;
}
function fmtPoseDelta(delta) {
  if (!delta) return "--";
  return `dx=${fmt(delta.dx_m * 100, 1)}cm dy=${fmt(delta.dy_m * 100, 1)}cm |d|=${fmt(delta.distance_m * 100, 1)}cm dh=${fmt(delta.dh_deg, 2)}deg`;
}
function worldToCanvas(x, y) {
  const pad = 60;
  const field = 3.3;
  const scale = Math.min((canvas.width - 2 * pad) / field, (canvas.height - 2 * pad) / field);
  const cx = canvas.width / 2 + x * scale;
  const cy = canvas.height / 2 - y * scale;
  return [cx, cy];
}
function rectWorld(x1, y1, x2, y2) {
  const [cx1, cy1] = worldToCanvas(x1, y1);
  const [cx2, cy2] = worldToCanvas(x2, y2);
  return [Math.min(cx1, cx2), Math.min(cy1, cy2), Math.abs(cx2 - cx1), Math.abs(cy2 - cy1)];
}
function drawLineWorld(x1, y1, x2, y2, color, width=2, alpha=1) {
  const [a, b] = worldToCanvas(x1, y1);
  const [c, d] = worldToCanvas(x2, y2);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a, b);
  ctx.lineTo(c, d);
  ctx.stroke();
  ctx.restore();
}
function drawRectWorld(x1, y1, x2, y2, color, width=2, alpha=1, fill=null, dash=null) {
  const [x, y, w, h] = rectWorld(x1, y1, x2, y2);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dash) ctx.setLineDash(dash);
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fillRect(x, y, w, h);
  }
  if (width > 0) ctx.strokeRect(x, y, w, h);
  ctx.restore();
}
function drawCircleWorld(x, y, radiusM, stroke, width=1.5, alpha=1, fill=null) {
  const [cx, cy] = worldToCanvas(x, y);
  const [r0x] = worldToCanvas(0, 0);
  const [r1x] = worldToCanvas(radiusM, 0);
  const r = Math.abs(r1x - r0x);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = stroke;
  ctx.fillStyle = fill || "transparent";
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  if (fill) ctx.fill();
  ctx.stroke();
  ctx.restore();
}
function drawTextWorld(text, x, y, color="#9fb1c1", alpha=0.82) {
  const [cx, cy] = worldToCanvas(x, y);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.font = "11px ui-sans-serif, system-ui";
  ctx.fillText(text, cx + 5, cy - 5);
  ctx.restore();
}
function drawGridBackground() {
  drawRectWorld(-1.55, -1.55, 1.55, 1.55, "rgba(0,0,0,0)", 0, 1, "#111820");
  for (let i = -15; i <= 15; i++) {
    const v = i / 10;
    const major = i % 5 === 0;
    drawLineWorld(v, -FIELD_HALF_M, v, FIELD_HALF_M, major ? "#34424f" : "#24303b", major ? 1.2 : 0.8, major ? 0.65 : 0.46);
    drawLineWorld(-FIELD_HALF_M, v, FIELD_HALF_M, v, major ? "#34424f" : "#24303b", major ? 1.2 : 0.8, major ? 0.65 : 0.46);
  }
}
function drawStaticGates() {
  for (const along of GATE_ALONGS_M) {
    drawLineWorld(along, -FIELD_HALF_M, along, -INNER_FAR_HALF_M, "#738190", 1.5, 0.52);
    drawLineWorld(along, INNER_FAR_HALF_M, along, FIELD_HALF_M, "#738190", 1.5, 0.52);
    drawLineWorld(-FIELD_HALF_M, along, -INNER_FAR_HALF_M, along, "#738190", 1.5, 0.52);
    drawLineWorld(INNER_FAR_HALF_M, along, FIELD_HALF_M, along, "#738190", 1.5, 0.52);
  }
  drawTextWorld("gates", -0.46, -0.72, "#738190", 0.75);
}
function drawStaticObstacleSlots() {
  for (const row of OBSTACLE_ROWS_M) {
    const rowAlpha = row < 0.5 ? 0.32 : 0.24;
    for (const along of GATE_ALONGS_M) {
      drawCircleWorld(along, -FIELD_HALF_M + row, OBSTACLE_SLOT_RADIUS_M, "#ffdf5d", 1.2, rowAlpha, "rgba(255, 223, 93, 0.035)");
      drawCircleWorld(along, FIELD_HALF_M - row, OBSTACLE_SLOT_RADIUS_M, "#ffdf5d", 1.2, rowAlpha, "rgba(255, 223, 93, 0.035)");
      drawCircleWorld(-FIELD_HALF_M + row, along, OBSTACLE_SLOT_RADIUS_M, "#ffdf5d", 1.2, rowAlpha, "rgba(255, 223, 93, 0.035)");
      drawCircleWorld(FIELD_HALF_M - row, along, OBSTACLE_SLOT_RADIUS_M, "#ffdf5d", 1.2, rowAlpha, "rgba(255, 223, 93, 0.035)");
    }
  }
}
function drawParkingWalls() {
  const x0 = PARKING_CENTER_X_M - PARKING_GAP_M / 2;
  const x1 = PARKING_CENTER_X_M + PARKING_GAP_M / 2;
  const y0 = -FIELD_HALF_M;
  const y1 = -FIELD_HALF_M + PARKING_WALL_LENGTH_M;
  drawLineWorld(x0, y0, x0, y1, "#ff4fc3", 4, 0.65);
  drawLineWorld(x1, y0, x1, y1, "#ff4fc3", 4, 0.65);
}
function drawFieldBackground() {
  drawGridBackground();
  drawRectWorld(-FIELD_HALF_M, -FIELD_HALF_M, FIELD_HALF_M, FIELD_HALF_M, "#d84fc7", 8, 0.78);
  drawRectWorld(-INNER_FAR_HALF_M, -INNER_FAR_HALF_M, INNER_FAR_HALF_M, INNER_FAR_HALF_M, "#d84fc7", 6, 0.62);
  drawRectWorld(-INNER_CLOSE_HALF_M, -INNER_CLOSE_HALF_M, INNER_CLOSE_HALF_M, INNER_CLOSE_HALF_M, "#58d6ff", 2, 0.23, null, [8, 8]);
  drawStaticGates();
  drawStaticObstacleSlots();
  drawParkingWalls();
  ctx.save();
  ctx.fillStyle = "rgba(128, 224, 167, 0.10)";
  const [sx, sy, sw, sh] = rectWorld(-0.25, -1.15, 0.25, -0.85);
  ctx.fillRect(sx, sy, sw, sh);
  ctx.strokeStyle = "rgba(128, 224, 167, 0.55)";
  ctx.lineWidth = 2;
  ctx.strokeRect(sx, sy, sw, sh);
  ctx.restore();
}
function drawPointCloud(points, color, size=2.2, alpha=0.75) {
  if (!points) return;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  for (const p of points) {
    const [x, y] = worldToCanvas(p[0], p[1]);
    ctx.fillRect(x - size / 2, y - size / 2, size, size);
  }
  ctx.restore();
}
function drawPoseDelta(a, b) {
  if (!a || !b || a.x === null || a.y === null || b.x === null || b.y === null) return;
  drawLineWorld(a.x, a.y, b.x, b.y, "#fff178", 2, 0.78);
  const [x, y] = worldToCanvas(b.x, b.y);
  ctx.save();
  ctx.fillStyle = "#fff178";
  ctx.beginPath();
  ctx.arc(x, y, 4, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
}
function drawRobot(pose, stroke="#ffb64d", fill="rgba(255,182,77,0.16)", label="pose") {
  if (!pose || pose.x === null || pose.y === null || pose.h === null) return;
  const [x, y] = worldToCanvas(pose.x, pose.y);
  const h = -pose.h * Math.PI / 180;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(h);
  ctx.strokeStyle = stroke;
  ctx.fillStyle = fill;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect(-18, -14, 36, 28);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(18, 0);
  ctx.lineTo(31, 0);
  ctx.stroke();
  ctx.restore();
  if (document.getElementById("showLabels").checked) {
    ctx.save();
    ctx.fillStyle = stroke;
    ctx.font = "12px ui-sans-serif, system-ui";
    ctx.fillText(label, x + 8, y + 16);
    ctx.restore();
  }
}
function finiteOr(v, fallback=0) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}
function manualClean(m) {
  return {
    dx_m: finiteOr(m && m.dx_m, 0),
    dy_m: finiteOr(m && m.dy_m, 0),
    dh_deg: finiteOr(m && m.dh_deg, 0),
  };
}
function annotationKeys(source=annotations) {
  return Object.keys(source).map(k => Number(k)).filter(k => Number.isInteger(k)).sort((a, b) => a - b);
}
function sameCorrection(a, b) {
  const ca = manualClean(a);
  const cb = manualClean(b);
  return Math.abs(ca.dx_m - cb.dx_m) < 1e-9 && Math.abs(ca.dy_m - cb.dy_m) < 1e-9 && Math.abs(ca.dh_deg - cb.dh_deg) < 1e-9;
}
function frameScanTime(index) {
  const sample = sampleByIndex[index];
  if (sample && Number.isFinite(sample.scan_t)) return sample.scan_t;
  if (currentFrame && currentFrame.index === index && Number.isFinite(currentFrame.scan_t)) return currentFrame.scan_t;
  return Number(index);
}
function interpolatedAnnotation(index, source=annotations) {
  const keys = annotationKeys(source);
  if (!keys.length) return {dx_m: 0, dy_m: 0, dh_deg: 0};
  if (Object.prototype.hasOwnProperty.call(source, String(index))) return manualClean(source[String(index)]);
  const t = frameScanTime(index);
  let lo = null;
  let hi = null;
  for (const key of keys) {
    const kt = frameScanTime(key);
    if (kt <= t) lo = key;
    if (hi === null && kt >= t) hi = key;
  }
  if (lo === null) return manualClean(source[String(keys[0])]);
  if (hi === null) return manualClean(source[String(keys[keys.length - 1])]);
  if (lo === hi) return manualClean(source[String(lo)]);
  const t0 = frameScanTime(lo);
  const t1 = frameScanTime(hi);
  const u = t1 <= t0 ? 0 : Math.max(0, Math.min(1, (t - t0) / (t1 - t0)));
  const a = manualClean(source[String(lo)]);
  const b = manualClean(source[String(hi)]);
  const dh = a.dh_deg + ((((b.dh_deg - a.dh_deg) + 180) % 360) - 180) * u;
  return {
    dx_m: a.dx_m + (b.dx_m - a.dx_m) * u,
    dy_m: a.dy_m + (b.dy_m - a.dy_m) * u,
    dh_deg: ((dh + 180) % 360) - 180,
  };
}
function stageAnnotation(index, corr) {
  annotations[String(index)] = manualClean(corr);
  annotationsDirty = true;
}
function deleteStagedAnnotation(index) {
  delete annotations[String(index)];
  annotationsDirty = true;
}
function transformPointCloud(points, pose, corr) {
  if (!points || !pose) return [];
  const c = manualClean(corr);
  const cx = finiteOr(pose.x, 0);
  const cy = finiteOr(pose.y, 0);
  const a = c.dh_deg * Math.PI / 180;
  const ca = Math.cos(a);
  const sa = Math.sin(a);
  return points.map(p => {
    const x = finiteOr(p[0], 0) - cx;
    const y = finiteOr(p[1], 0) - cy;
    return [cx + ca * x - sa * y + c.dx_m, cy + sa * x + ca * y + c.dy_m];
  });
}
function transformedPose(pose, corr) {
  if (!pose) return null;
  const c = manualClean(corr);
  return {x: finiteOr(pose.x, 0) + c.dx_m, y: finiteOr(pose.y, 0) + c.dy_m, h: finiteOr(pose.h, 0) + c.dh_deg};
}
function drawPath(points, color, width=2, alpha=0.75) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = alpha;
  ctx.lineWidth = width;
  ctx.beginPath();
  let started = false;
  for (const p of points) {
    if (!Number.isFinite(p[0]) || !Number.isFinite(p[1])) continue;
    const [x, y] = worldToCanvas(p[0], p[1]);
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();
}
function drawAnnotationMarkers(markers) {
  if (!markers) return;
  ctx.save();
  ctx.fillStyle = '#ff5df2';
  ctx.strokeStyle = '#111';
  ctx.lineWidth = 1;
  for (const p of markers) {
    const [x, y] = worldToCanvas(p[0], p[1]);
    ctx.beginPath();
    ctx.arc(x, y, 4.5, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}
function setManualControls(corr) {
  const c = manualClean(corr);
  manual = c;
  manualDirty = false;
  const pairs = [['manualX', c.dx_m], ['manualY', c.dy_m], ['manualH', c.dh_deg]];
  for (const [id, val] of pairs) {
    const slider = document.getElementById(id);
    const number = document.getElementById(id + 'Num');
    slider.value = String(val);
    number.value = String(val);
  }
  updateManualStatus();
}
function updateManualStatus(text) {
  const el = document.getElementById('manualStatus');
  if (text) { el.textContent = text; return; }
  if (!currentFrame) { el.textContent = ''; return; }
  const exact = Object.prototype.hasOwnProperty.call(annotations, String(current));
  const savedExact = currentFrame.manual_saved && exact && sameCorrection(currentFrame.manual, annotations[String(current)]);
  const status = exact ? (savedExact ? 'saved keyframe' : 'staged keyframe') : 'interpolated';
  const dirty = annotationsDirty ? ' unsaved' : '';
  const count = annotationKeys().length;
  el.innerHTML = `<span class="pill">${status}${dirty}</span> dx=${fmt(manual.dx_m * 100, 1)}cm dy=${fmt(manual.dy_m * 100, 1)}cm dh=${fmt(manual.dh_deg, 2)}deg keyframes=${count}`;
}
function updateManualFromInputs(changedId, rawValue) {
  const val = Number(rawValue);
  if (!Number.isFinite(val)) return;
  if (changedId.startsWith('manualX')) manual.dx_m = val;
  if (changedId.startsWith('manualY')) manual.dy_m = val;
  if (changedId.startsWith('manualH')) manual.dh_deg = val;
  for (const id of ['manualX', 'manualY', 'manualH']) {
    const key = id === 'manualX' ? 'dx_m' : (id === 'manualY' ? 'dy_m' : 'dh_deg');
    document.getElementById(id).value = String(manual[key]);
    document.getElementById(id + 'Num').value = String(manual[key]);
  }
  manualDirty = true;
  stageAnnotation(current, manual);
  updateManualStatus();
  if (currentFrame) draw(currentFrame);
  if (currentFrame) updatePanels(currentFrame);
}
function storedModel(frame) {
  return (frame.models || []).find(m => m.name === 'stored') || (frame.models || [])[0];
}
function modelFitSummary() {
  return state && state.model_fit ? state.model_fit : {models: []};
}
function selectedModelFit() {
  const fits = modelFitSummary();
  const models = fits.models || [];
  if (!models.length) return null;
  let fit = models.find(m => m.name === selectedFit);
  if (!fit) fit = models.find(m => m.name === fits.recommended) || models[0];
  selectedFit = fit.name;
  return fit;
}
function cloneJson(value) {
  return JSON.parse(JSON.stringify(value || {}));
}
function getPathValue(obj, path, fallback=0) {
  let cur = obj;
  for (const key of path) {
    if (cur === null || cur === undefined) return fallback;
    cur = cur[key];
  }
  const n = Number(cur);
  return Number.isFinite(n) ? n : fallback;
}
function setPathValue(obj, path, value) {
  let cur = obj;
  for (let i = 0; i < path.length - 1; i++) {
    const key = path[i];
    if (cur[key] === undefined || cur[key] === null) cur[key] = typeof path[i + 1] === "number" ? [] : {};
    cur = cur[key];
  }
  cur[path[path.length - 1]] = value;
}
function ensureArray(value, fallback) {
  return Array.isArray(value) ? value : cloneJson(fallback);
}
function deriveSimilarityFields(params) {
  params.translation_m = ensureArray(params.translation_m, [0, 0]);
  params.transform_origin_m = modelFitOrigin(params.transform_origin_m);
  params.matrix = ensureArray(params.matrix, [[1, 0], [0, 1]]);
  if (!Number.isFinite(Number(params.scale)) || !Number.isFinite(Number(params.rotation_deg))) {
    const m = params.matrix;
    const m00 = Number(m[0] && m[0][0]);
    const m10 = Number(m[1] && m[1][0]);
    params.scale = Number.isFinite(Number(params.scale)) ? Number(params.scale) : Math.hypot(m00 || 1, m10 || 0);
    params.rotation_deg = Number.isFinite(Number(params.rotation_deg)) ? Number(params.rotation_deg) : Math.atan2(m10 || 0, m00 || 1) * 180 / Math.PI;
  }
  syncSimilarityMatrix(params);
}
function syncSimilarityMatrix(params) {
  const scale = Number(params.scale);
  const rot = Number(params.rotation_deg) * Math.PI / 180;
  const c = Math.cos(rot) * (Number.isFinite(scale) ? scale : 1);
  const s = Math.sin(rot) * (Number.isFinite(scale) ? scale : 1);
  params.matrix = [[c, -s], [s, c]];
  params.translation_m = ensureArray(params.translation_m, [0, 0]);
  params.transform_origin_m = modelFitOrigin(params.transform_origin_m);
}
function ensureModelParams(fit, params) {
  if (!fit || !params) return params || {};
  if (fit.type === "none" || fit.type === "raw_otos" || fit.type === "raw" || fit.type === "constant") {
    params.dx_m = Number.isFinite(Number(params.dx_m)) ? Number(params.dx_m) : 0;
    params.dy_m = Number.isFinite(Number(params.dy_m)) ? Number(params.dy_m) : 0;
  } else if (fit.type === "similarity") {
    deriveSimilarityFields(params);
  } else if (fit.type === "affine") {
    params.translation_m = ensureArray(params.translation_m, [0, 0]);
    params.transform_origin_m = modelFitOrigin(params.transform_origin_m);
    params.matrix = ensureArray(params.matrix, [[1, 0], [0, 1]]);
  } else if (fit.type === "section_offsets") {
    params.offsets_m = params.offsets_m && typeof params.offsets_m === "object" ? params.offsets_m : {};
    for (const section of ["bottom", "left", "right", "top", "default"]) {
      params.offsets_m[section] = ensureArray(params.offsets_m[section], [0, 0]);
    }
  } else if (fit.type === "similarity_periodic") {
    params.base = params.base && typeof params.base === "object" ? params.base : {};
    deriveSimilarityFields(params.base);
    params.period_m = Number.isFinite(Number(params.period_m)) ? Number(params.period_m) : 7;
    params.x_coeffs_m = ensureArray(params.x_coeffs_m, [0, 0, 0, 0]);
    params.y_coeffs_m = ensureArray(params.y_coeffs_m, [0, 0, 0, 0]);
  } else if (fit.type === "otos_offset_h_sim" || fit.type === "otos_offset_h_frame_sim" || fit.type === "otos_offset_h_rigid_sim") {
    params.base_offset_h_deg = Number.isFinite(Number(params.base_offset_h_deg)) ? Number(params.base_offset_h_deg) : 183.3;
    params.target_offset_h_deg = Number.isFinite(Number(params.target_offset_h_deg)) ? Number(params.target_offset_h_deg) : 180.9;
    params.offset_x_m = Number.isFinite(Number(params.offset_x_m)) ? Number(params.offset_x_m) : 0.1261;
    params.offset_y_m = Number.isFinite(Number(params.offset_y_m)) ? Number(params.offset_y_m) : -0.0161;
    params.dx_m = Number.isFinite(Number(params.dx_m)) ? Number(params.dx_m) : 0;
    params.dy_m = Number.isFinite(Number(params.dy_m)) ? Number(params.dy_m) : 0;
    params.heading_effect_sign = Number.isFinite(Number(params.heading_effect_sign)) ? Number(params.heading_effect_sign) : (Number.isFinite(Number(params.effect_sign)) ? Number(params.effect_sign) : -1);
    params.frame_effect_sign = Number.isFinite(Number(params.frame_effect_sign)) ? Number(params.frame_effect_sign) : params.heading_effect_sign;
    params.heading_output_delta_deg = Number.isFinite(Number(params.heading_output_delta_deg)) ? Number(params.heading_output_delta_deg) : 0;
    params.max_continuous_step_m = Number.isFinite(Number(params.max_continuous_step_m)) ? Number(params.max_continuous_step_m) : 0.35;
  } else if (fit.type === "reference_offset_sim") {
    params.translation_m = ensureArray(params.translation_m, [0, 0]);
    params.local_offset_m = ensureArray(params.local_offset_m, [0, 0]);
    params.anchor_heading_deg = Number.isFinite(Number(params.anchor_heading_deg)) ? Number(params.anchor_heading_deg) : null;
    params.heading_output_delta_deg = Number.isFinite(Number(params.heading_output_delta_deg)) ? Number(params.heading_output_delta_deg) : 0;
  }
  return params;
}
function effectiveModelFitParams(fit) {
  if (!fit) return {};
  if (!modelFitEdits[fit.name]) modelFitEdits[fit.name] = cloneJson(fit.params || {});
  return ensureModelParams(fit, modelFitEdits[fit.name]);
}
function sampleSection(sample) {
  if (sample && sample.section) return sample.section;
  const pose = sample && (sample.fused || sample.otos || sample.base);
  const x = pose && Number(pose.x);
  const y = pose && Number(pose.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return "default";
  return Math.abs(x) >= Math.abs(y) ? (x >= 0 ? "right" : "left") : (y >= 0 ? "top" : "bottom");
}
function sampleBasePose(sample) {
  return sample && (sample.base || sample.otos || sample.fused);
}
function firstManualAlignedOrigin() {
  for (const key of annotationKeys()) {
    const sample = sampleByIndex[Number(key)];
    const pose = sampleBasePose(sample);
    if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) continue;
    const corr = manualClean(annotations[String(key)]);
    return [Number(pose.x) + corr.dx_m, Number(pose.y) + corr.dy_m];
  }
  return null;
}
function modelFitOrigin(fallback=null) {
  const manualOrigin = firstManualAlignedOrigin();
  if (manualOrigin) return manualOrigin;
  if (Array.isArray(fallback)) return fallback;
  const fits = modelFitSummary();
  if (fits && Array.isArray(fits.transform_origin_m)) return fits.transform_origin_m;
  const samples = state && Array.isArray(state.path_samples) ? state.path_samples : [];
  const first = samples.find(s => s.otos && Number.isFinite(Number(s.otos.x)) && Number.isFinite(Number(s.otos.y)));
  return first ? [Number(first.otos.x), Number(first.otos.y)] : [0, 0];
}
function offsetHeadingBaseDeltaDeg(params) {
  return Number(params.target_offset_h_deg || 0) - Number(params.base_offset_h_deg || 0);
}
function offsetHeadingOutputDeltaDeg(params) {
  const sign = Number.isFinite(Number(params.heading_effect_sign)) ? Number(params.heading_effect_sign) : -1;
  return offsetHeadingBaseDeltaDeg(params) * sign + Number(params.heading_output_delta_deg || 0);
}
function offsetHeadingFrameDeltaDeg(params) {
  const sign = Number.isFinite(Number(params.frame_effect_sign)) ? Number(params.frame_effect_sign) : -1;
  return offsetHeadingBaseDeltaDeg(params) * sign;
}
function rotateOffset(x, y, hRad) {
  const c = Math.cos(hRad);
  const s = Math.sin(hRad);
  return [c * x - s * y, s * x + c * y];
}
function offsetHeadingRigidPathItems(fit) {
  const params = effectiveModelFitParams(fit);
  const samples = (state && state.path_samples ? state.path_samples : []).filter(s => s.otos && Number.isFinite(Number(s.otos.x)) && Number.isFinite(Number(s.otos.y)));
  const out = [];
  const dxBias = Number(params.dx_m || 0);
  const dyBias = Number(params.dy_m || 0);
  const offsetX = Number(params.offset_x_m || 0);
  const offsetY = Number(params.offset_y_m || 0);
  const headingDeltaDeg = offsetHeadingOutputDeltaDeg(params);
  for (const sample of samples) {
    const ox = Number(sample.otos.x);
    const oy = Number(sample.otos.y);
    const oh = Number(sample.otos.h || 0);
    const [oldLeverX, oldLeverY] = rotateOffset(offsetX, offsetY, oh * Math.PI / 180);
    const sensorX = ox + oldLeverX;
    const sensorY = oy + oldLeverY;
    const targetH = oh + headingDeltaDeg;
    const [targetLeverX, targetLeverY] = rotateOffset(offsetX, offsetY, targetH * Math.PI / 180);
    out.push({index: Number(sample.index), x: sensorX - targetLeverX + dxBias, y: sensorY - targetLeverY + dyBias, h: targetH});
  }
  return out;
}
function offsetHeadingFramePathItems(fit) {
  const params = effectiveModelFitParams(fit);
  const samples = (state && state.path_samples ? state.path_samples : []).filter(s => s.otos && Number.isFinite(Number(s.otos.x)) && Number.isFinite(Number(s.otos.y)));
  const out = [];
  let prev = null;
  let targetSensorX = null;
  let targetSensorY = null;
  const headingDeltaDeg = offsetHeadingOutputDeltaDeg(params);
  const frameDelta = offsetHeadingFrameDeltaDeg(params) * Math.PI / 180;
  const dxBias = Number(params.dx_m || 0);
  const dyBias = Number(params.dy_m || 0);
  const offsetX = Number(params.offset_x_m || 0);
  const offsetY = Number(params.offset_y_m || 0);
  const maxStep = Math.max(0.02, Number(params.max_continuous_step_m || 0.35));
  for (const sample of samples) {
    const ox = Number(sample.otos.x);
    const oy = Number(sample.otos.y);
    const oh = Number(sample.otos.h || 0);
    const [oldLeverX, oldLeverY] = rotateOffset(offsetX, offsetY, oh * Math.PI / 180);
    const oldSensorX = ox + oldLeverX;
    const oldSensorY = oy + oldLeverY;
    const targetH = oh + headingDeltaDeg;
    const [targetLeverX, targetLeverY] = rotateOffset(offsetX, offsetY, targetH * Math.PI / 180);
    if (!prev || targetSensorX === null || targetSensorY === null) {
      targetSensorX = oldSensorX;
      targetSensorY = oldSensorY;
    } else {
      const prevOh = Number(prev.otos.h || oh);
      const prevOldHRad = prevOh * Math.PI / 180;
      const prevTargetHRad = (prevOh + headingDeltaDeg) * Math.PI / 180;
      const [prevOldLeverX, prevOldLeverY] = rotateOffset(offsetX, offsetY, prevOldHRad);
      const prevSensorOldX = Number(prev.otos.x) + prevOldLeverX;
      const prevSensorOldY = Number(prev.otos.y) + prevOldLeverY;
      const pdx = oldSensorX - prevSensorOldX;
      const pdy = oldSensorY - prevSensorOldY;
      const step = Math.hypot(pdx, pdy);
      if (!Number.isFinite(step) || step > maxStep) {
        targetSensorX = oldSensorX;
        targetSensorY = oldSensorY;
      } else {
        const oldC = Math.cos(prevOldHRad);
        const oldS = Math.sin(prevOldHRad);
        const localForward = pdx * oldC + pdy * oldS;
        const localLateral = -pdx * oldS + pdy * oldC;
        const frameC = Math.cos(frameDelta);
        const frameS = Math.sin(frameDelta);
        const targetForward = frameC * localForward - frameS * localLateral;
        const targetLateral = frameS * localForward + frameC * localLateral;
        const targetC = Math.cos(prevTargetHRad);
        const targetS = Math.sin(prevTargetHRad);
        targetSensorX += targetForward * targetC - targetLateral * targetS;
        targetSensorY += targetForward * targetS + targetLateral * targetC;
      }
    }
    out.push({index: Number(sample.index), x: targetSensorX - targetLeverX + dxBias, y: targetSensorY - targetLeverY + dyBias, h: targetH});
    prev = sample;
  }
  return out;
}
function referenceOffsetAnchorHeading(params, samples) {
  if (Number.isFinite(Number(params.anchor_heading_deg))) return Number(params.anchor_heading_deg);
  const first = samples.find(s => s.otos && Number.isFinite(Number(s.otos.h)));
  return first ? Number(first.otos.h) : 0;
}
function referenceOffsetSimPathItems(fit) {
  const params = effectiveModelFitParams(fit);
  const samples = (state && state.path_samples ? state.path_samples : []).filter(s => s.otos && Number.isFinite(Number(s.otos.x)) && Number.isFinite(Number(s.otos.y)));
  const anchor = referenceOffsetAnchorHeading(params, samples) * Math.PI / 180;
  const ca = Math.cos(anchor);
  const sa = Math.sin(anchor);
  const tx = Number((params.translation_m || [0, 0])[0] || 0);
  const ty = Number((params.translation_m || [0, 0])[1] || 0);
  const rx = Number((params.local_offset_m || [0, 0])[0] || 0);
  const ry = Number((params.local_offset_m || [0, 0])[1] || 0);
  const hBias = Number(params.heading_output_delta_deg || 0);
  const out = [];
  for (const sample of samples) {
    const h = Number(sample.otos.h || 0) * Math.PI / 180;
    const c = Math.cos(h);
    const s = Math.sin(h);
    const dx = tx + (c - ca) * rx - (s - sa) * ry;
    const dy = ty + (s - sa) * rx + (c - ca) * ry;
    out.push({index: Number(sample.index), x: Number(sample.otos.x) + dx, y: Number(sample.otos.y) + dy, h: Number(sample.otos.h || 0) + hBias});
  }
  return out;
}
function applyModelFitParams(type, params, x, y, section, progress) {
  x = Number(x); y = Number(y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return [null, null];
  if (type === "none" || type === "raw_otos" || type === "raw" || type === "constant") return [x + Number(params.dx_m || 0), y + Number(params.dy_m || 0)];
  if (type === "similarity" || type === "affine") {
    const m = params.matrix || [[1, 0], [0, 1]];
    const t = params.translation_m || [0, 0];
    const origin = modelFitOrigin(params.transform_origin_m);
    const ox = Number(origin[0] || 0);
    const oy = Number(origin[1] || 0);
    const rx = x - ox;
    const ry = y - oy;
    return [
      ox + Number(m[0][0]) * rx + Number(m[0][1]) * ry + Number(t[0] || 0),
      oy + Number(m[1][0]) * rx + Number(m[1][1]) * ry + Number(t[1] || 0),
    ];
  }
  if (type === "section_offsets") {
    const offsets = params.offsets_m || {};
    const off = offsets[section] || offsets.default || [0, 0];
    return [x + Number(off[0] || 0), y + Number(off[1] || 0)];
  }
  if (type === "similarity_periodic") {
    const base = params.base || {};
    const [bx, by] = applyModelFitParams("similarity", base, x, y, section, progress);
    const period = Number(params.period_m || 0);
    if (!Number.isFinite(period) || period <= 1e-6) return [bx, by];
    const phi = 2 * Math.PI * ((Number(progress || 0) % period) / period);
    const f = [Math.sin(phi), Math.cos(phi), Math.sin(2 * phi), Math.cos(2 * phi)];
    const xc = params.x_coeffs_m || [0, 0, 0, 0];
    const yc = params.y_coeffs_m || [0, 0, 0, 0];
    const dx = f.reduce((acc, v, i) => acc + Number(xc[i] || 0) * v, 0);
    const dy = f.reduce((acc, v, i) => acc + Number(yc[i] || 0) * v, 0);
    return [bx + dx, by + dy];
  }
  return [x, y];
}
function modelFitPathItems(fit) {
  if (!fit) return [];
  if (fit.type === "otos_offset_h_sim" || fit.type === "otos_offset_h_frame_sim") return offsetHeadingFramePathItems(fit);
  if (fit.type === "otos_offset_h_rigid_sim") return offsetHeadingRigidPathItems(fit);
  if (fit.type === "reference_offset_sim") return referenceOffsetSimPathItems(fit);
  const params = effectiveModelFitParams(fit);
  const samples = state && state.path_samples ? state.path_samples : [];
  if (!samples.length) return fit.path || [];
  const out = [];
  for (const sample of samples) {
    if (!sample.otos || !Number.isFinite(Number(sample.otos.x)) || !Number.isFinite(Number(sample.otos.y))) continue;
    const [x, y] = applyModelFitParams(fit.type, params, sample.otos.x, sample.otos.y, sampleSection(sample), Number(sample.progress_m || 0));
    if (x === null || y === null) continue;
    out.push({index: Number(sample.index), x, y, h: Number(sample.otos.h || 0)});
  }
  return out;
}
function metricsFromErrors(errors) {
  const arr = errors.filter(v => Number.isFinite(v)).sort((a, b) => a - b);
  if (!arr.length) return {count: 0, median_m: null, p95_m: null, mean_m: null, max_m: null};
  const pct = (p) => arr[Math.max(0, Math.min(arr.length - 1, Math.round((arr.length - 1) * p)))];
  return {count: arr.length, median_m: pct(0.5), p95_m: pct(0.95), mean_m: arr.reduce((a, b) => a + b, 0) / arr.length, max_m: arr[arr.length - 1]};
}
function currentModelFitMetrics(fit) {
  if (!fit) return {metrics: metricsFromErrors([]), per_section: {}};
  const predictionByIndex = new Map(modelFitPathItems(fit).map(p => [Number(p.index), p]));
  const errors = [];
  const bySection = {};
  for (const sample of state.path_samples || []) {
    if (!sample.fit_sample || !sample.otos || !sample.fused) continue;
    if (!Number.isFinite(Number(sample.fused.x)) || !Number.isFinite(Number(sample.fused.y))) continue;
    const pred = predictionByIndex.get(Number(sample.index));
    if (!pred || !Number.isFinite(Number(pred.x)) || !Number.isFinite(Number(pred.y))) continue;
    const section = sampleSection(sample);
    const err = Math.hypot(Number(pred.x) - Number(sample.fused.x), Number(pred.y) - Number(sample.fused.y));
    errors.push(err);
    if (!bySection[section]) bySection[section] = [];
    bySection[section].push(err);
  }
  const perSection = {};
  for (const section of Object.keys(bySection).sort()) perSection[section] = metricsFromErrors(bySection[section]);
  return {metrics: metricsFromErrors(errors), per_section: perSection};
}
function selectedModelFitPose() {
  const fit = selectedModelFit();
  const item = modelFitPathItems(fit).find(p => Number(p.index) === Number(current));
  if (!item) return null;
  return {x: item.x, y: item.y, h: item.h};
}
function buildModelFitSelect() {
  const select = document.getElementById("modelFitSelect");
  if (!select) return;
  const fits = modelFitSummary();
  const models = fits.models || [];
  if (!models.length) {
    select.innerHTML = `<option value="">No fit data</option>`;
    document.getElementById("exportModelFit").disabled = true;
    renderModelFitTuneControls();
    updateModelFitPanel();
    return;
  }
  if (!selectedFit || !models.some(m => m.name === selectedFit)) selectedFit = fits.recommended || models[0].name;
  select.innerHTML = models.map(m => `<option value="${m.name}">${m.label || m.name}</option>`).join("");
  select.value = selectedFit;
  document.getElementById("exportModelFit").disabled = false;
  renderModelFitTuneControls();
  updateModelFitPanel();
}
function metricValue(v, digits=3, scale=1) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "--";
  return (Number(v) * scale).toFixed(digits);
}
function updateModelFitPanel(text) {
  const status = document.getElementById("modelFitStatus");
  const table = document.getElementById("modelFitMetrics");
  const paramsEl = document.getElementById("modelFitParams");
  if (!status || !table || !paramsEl) return;
  const fits = modelFitSummary();
  const fit = selectedModelFit();
  if (!fit) {
    status.textContent = "No OTOS/fused navigation samples available for model fitting.";
    table.innerHTML = "";
    paramsEl.textContent = "";
    return;
  }
  const effectiveParams = effectiveModelFitParams(fit);
  const live = currentModelFitMetrics(fit);
  const m = live.metrics || {};
  const sections = live.per_section || {};
  const rows = [
    ["all", m.count, m.median_m, m.p95_m, m.mean_m, m.max_m],
    ...Object.keys(sections).sort().map(k => [k, sections[k].count, sections[k].median_m, sections[k].p95_m, sections[k].mean_m, sections[k].max_m]),
  ];
  table.innerHTML = `<tr><th>Set</th><th>N</th><th>Med cm</th><th>P95 cm</th><th>Mean cm</th><th>Max cm</th></tr>` + rows.map(r => `<tr><td>${r[0]}</td><td>${r[1] || 0}</td><td>${metricValue(r[2], 1, 100)}</td><td>${metricValue(r[3], 1, 100)}</td><td>${metricValue(r[4], 1, 100)}</td><td>${metricValue(r[5], 1, 100)}</td></tr>`).join("");
  const edited = modelFitTouched[fit.name] ? "tuned" : "fit";
  status.textContent = text || `${fit.label || fit.name}: ${edited}, ${fits.fit_sample_count || 0} samples, export path ${fits.model_fit_path || "--"}`;
  paramsEl.textContent = JSON.stringify(effectiveParams || {}, null, 2);
}
function fitTuneChanged(path, rawDisplayValue, scale) {
  const fit = selectedModelFit();
  if (!fit) return;
  const params = effectiveModelFitParams(fit);
  const displayValue = Number(rawDisplayValue);
  if (!Number.isFinite(displayValue)) return;
  setPathValue(params, path, displayValue / scale);
  modelFitTouched[fit.name] = true;
  ensureModelParams(fit, params);
  updateModelFitPanel();
  if (currentFrame) { draw(currentFrame); updatePanels(currentFrame); }
}
function renderModelFitTuneControls(rebuild=true) {
  const el = document.getElementById("modelFitTuneControls");
  const status = document.getElementById("modelFitTuneStatus");
  if (!el || !status) return;
  const fit = selectedModelFit();
  if (!fit) {
    el.innerHTML = "";
    status.textContent = "";
    return;
  }
  const params = effectiveModelFitParams(fit);
  const controls = [];
  const add = (label, path, min, max, step, scale=1, unit="") => {
    controls.push({label, path, min, max, step, scale, unit});
  };
  if (fit.type === "none" || fit.type === "raw_otos" || fit.type === "raw" || fit.type === "constant") {
    add("X offset", ["dx_m"], -30, 30, 0.1, 100, "cm");
    add("Y offset", ["dy_m"], -30, 30, 0.1, 100, "cm");
  } else if (fit.type === "similarity") {
    add("X translate", ["translation_m", 0], -30, 30, 0.1, 100, "cm");
    add("Y translate", ["translation_m", 1], -30, 30, 0.1, 100, "cm");
    add("Rotation", ["rotation_deg"], -8, 8, 0.02, 1, "deg");
    add("Scale", ["scale"], 0.94, 1.06, 0.0005, 1, "x");
  } else if (fit.type === "affine") {
    add("X translate", ["translation_m", 0], -30, 30, 0.1, 100, "cm");
    add("Y translate", ["translation_m", 1], -30, 30, 0.1, 100, "cm");
    add("m00", ["matrix", 0, 0], 0.85, 1.15, 0.0005, 1, "");
    add("m01", ["matrix", 0, 1], -0.15, 0.15, 0.0005, 1, "");
    add("m10", ["matrix", 1, 0], -0.15, 0.15, 0.0005, 1, "");
    add("m11", ["matrix", 1, 1], 0.85, 1.15, 0.0005, 1, "");
  } else if (fit.type === "section_offsets") {
    for (const section of ["bottom", "left", "right", "top"]) {
      add(`${section} X`, ["offsets_m", section, 0], -30, 30, 0.1, 100, "cm");
      add(`${section} Y`, ["offsets_m", section, 1], -30, 30, 0.1, 100, "cm");
    }
  } else if (fit.type === "otos_offset_h_sim" || fit.type === "otos_offset_h_frame_sim" || fit.type === "otos_offset_h_rigid_sim") {
    add("Base offset-h", ["base_offset_h_deg"], 175, 190, 0.05, 1, "deg");
    add("Target offset-h", ["target_offset_h_deg"], 175, 190, 0.05, 1, "deg");
    add("Offset X", ["offset_x_m"], -25, 25, 0.1, 100, "cm");
    add("Offset Y", ["offset_y_m"], -25, 25, 0.1, 100, "cm");
    add("Start X", ["dx_m"], -30, 30, 0.1, 100, "cm");
    add("Start Y", ["dy_m"], -30, 30, 0.1, 100, "cm");
    add("Heading sign", ["heading_effect_sign"], -1, 1, 2, 1, "");
    add("Frame sign", ["frame_effect_sign"], -1, 1, 2, 1, "");
    add("Heading add", ["heading_output_delta_deg"], -5, 5, 0.05, 1, "deg");
    add("Gap reset", ["max_continuous_step_m"], 5, 80, 1, 100, "cm");
  } else if (fit.type === "reference_offset_sim") {
    add("Field X bias", ["translation_m", 0], -30, 30, 0.1, 100, "cm");
    add("Field Y bias", ["translation_m", 1], -30, 30, 0.1, 100, "cm");
    add("Local X offset", ["local_offset_m", 0], -30, 30, 0.1, 100, "cm");
    add("Local Y offset", ["local_offset_m", 1], -30, 30, 0.1, 100, "cm");
    add("Anchor heading", ["anchor_heading_deg"], -180, 180, 0.1, 1, "deg");
    add("Heading add", ["heading_output_delta_deg"], -5, 5, 0.05, 1, "deg");
  } else if (fit.type === "similarity_periodic") {
    add("Base X", ["base", "translation_m", 0], -30, 30, 0.1, 100, "cm");
    add("Base Y", ["base", "translation_m", 1], -30, 30, 0.1, 100, "cm");
    add("Base rot", ["base", "rotation_deg"], -8, 8, 0.02, 1, "deg");
    add("Base scale", ["base", "scale"], 0.94, 1.06, 0.0005, 1, "x");
    add("Period", ["period_m"], 4, 10, 0.01, 1, "m");
    ["sin1", "cos1", "sin2", "cos2"].forEach((name, i) => {
      add(`X ${name}`, ["x_coeffs_m", i], -15, 15, 0.1, 100, "cm");
      add(`Y ${name}`, ["y_coeffs_m", i], -15, 15, 0.1, 100, "cm");
    });
  }
  if (!controls.length) {
    el.innerHTML = `<div class="small">This model has no tunable parameters.</div>`;
    status.textContent = "Select another fit to expose sliders.";
    return;
  }
  status.textContent = "Slider changes update the path and metrics immediately. Export saves the tuned parameter set.";
  el.innerHTML = controls.map((c, i) => {
    const value = getPathValue(params, c.path, 0) * c.scale;
    const shown = Number.isFinite(value) ? value : 0;
    return `<div class="tune-control"><label>${c.label}</label><input id="fitTune${i}" type="range" min="${c.min}" max="${c.max}" step="${c.step}" value="${shown}"><input id="fitTune${i}Num" type="number" min="${c.min}" max="${c.max}" step="${c.step}" value="${shown}"><span>${c.unit}</span></div>`;
  }).join("");
  controls.forEach((c, i) => {
    const slider = document.getElementById(`fitTune${i}`);
    const number = document.getElementById(`fitTune${i}Num`);
    const update = (raw) => {
      slider.value = String(raw);
      number.value = String(raw);
      fitTuneChanged(c.path, raw, c.scale);
    };
    slider.addEventListener("input", ev => update(ev.target.value));
    number.addEventListener("input", ev => update(ev.target.value));
  });
}
async function exportSelectedModelFit() {
  const fit = selectedModelFit();
  if (!fit) return;
  const params = cloneJson(effectiveModelFitParams(fit));
  const live = currentModelFitMetrics(fit);
  const tuned = {
    name: `${fit.name}_tuned`,
    label: `Tuned ${fit.label || fit.name}`,
    type: fit.type,
    source_model: fit.name,
    params,
    metrics: live.metrics,
    per_section: live.per_section,
  };
  updateModelFitPanel("Exporting tuned model fit...");
  const res = await fetch("/api/model_fit/export", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({name: tuned.name, model: tuned}),
  });
  if (!res.ok) throw new Error(await res.text());
  const payload = await res.json();
  updateModelFitPanel(`Exported ${payload.name} to ${payload.path}`);
}
function workingPaths() {
  const samples = state && state.path_samples ? state.path_samples : [];
  const otos = [];
  const fused = [];
  const corrected = [];
  const markers = [];
  const modelFitPath = [];
  const exact = new Set(annotationKeys().map(k => String(k)));
  for (const sample of samples) {
    if (sample.otos && Number.isFinite(sample.otos.x) && Number.isFinite(sample.otos.y)) {
      otos.push([sample.otos.x, sample.otos.y]);
    }
    if (sample.fused && Number.isFinite(sample.fused.x) && Number.isFinite(sample.fused.y)) {
      fused.push([sample.fused.x, sample.fused.y]);
    }
    const base = sample.base || sample.otos || sample.fused;
    if (base && Number.isFinite(base.x) && Number.isFinite(base.y)) {
      const corr = interpolatedAnnotation(Number(sample.index));
      const p = [base.x + corr.dx_m, base.y + corr.dy_m];
      corrected.push(p);
      if (exact.has(String(sample.index))) markers.push(p);
    }
  }
  const fit = selectedModelFit();
  if (fit && document.getElementById("showModelFit") && document.getElementById("showModelFit").checked) {
    for (const p of modelFitPathItems(fit)) {
      if (Number.isFinite(Number(p.x)) && Number.isFinite(Number(p.y))) modelFitPath.push([Number(p.x), Number(p.y)]);
    }
  }
  return {otos_only: otos, fused, manual_corrected: corrected, annotation_markers: markers, model_fit: modelFitPath};
}
function drawFit(metric, color) {
  if (!metric || !metric.field_valid || !metric.field_wall) return;
  const w = metric.field_wall;
  if (w === "outer_left") drawLineWorld(-1.5, -1.5, -1.5, 1.5, color, 5, 0.32);
  if (w === "outer_right") drawLineWorld(1.5, -1.5, 1.5, 1.5, color, 5, 0.32);
  if (w === "outer_bottom") drawLineWorld(-1.5, -1.5, 1.5, -1.5, color, 5, 0.32);
  if (w === "outer_top") drawLineWorld(-1.5, 1.5, 1.5, 1.5, color, 5, 0.32);
}
function draw(frame) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (document.getElementById("showField").checked) {
    drawFieldBackground();
  } else {
    drawLineWorld(-1.5, -1.5, 1.5, -1.5, "#53616d", 2);
    drawLineWorld(1.5, -1.5, 1.5, 1.5, "#53616d", 2);
    drawLineWorld(1.5, 1.5, -1.5, 1.5, "#53616d", 2);
    drawLineWorld(-1.5, 1.5, -1.5, -1.5, "#53616d", 2);
  }
  if (document.getElementById("showPaths").checked) {
    const paths = workingPaths();
    drawPath(paths.otos_only, "#ffb64d", 2.2, 0.55);
    drawPath(paths.fused, "#80e0a7", 1.7, 0.45);
    drawPath(paths.model_fit, "#58d6ff", 2.6, 0.82);
    drawPath(paths.manual_corrected, "#ff5df2", 2.5, 0.75);
    drawAnnotationMarkers(paths.annotation_markers);
  }
  for (const m of frame.models) {
    if (!enabled[m.name]) continue;
    if (document.getElementById("showFit").checked) drawFit(m.metrics, colors[m.name]);
    drawPointCloud(m.points, colors[m.name], m.name === "stored" ? 2.6 : 2.0, m.name === "stored" ? 0.88 : 0.58);
    if (document.getElementById("showSeam").checked) {
      drawPointCloud(m.seam_first, "#ffffff", 4.0, 0.85);
      drawPointCloud(m.seam_last, "#ff5df2", 4.0, 0.85);
    }
  }
  if (document.getElementById("showManual").checked) {
    const base = storedModel(frame);
    const centerPose = frame.pose_otos_only || frame.pose_fused || frame.pose;
    if (base && centerPose) {
      const correctedPoints = transformPointCloud(base.points, centerPose, manual);
      drawPointCloud(correctedPoints, "#ff5df2", 3.1, 0.92);
      drawRobot(transformedPose(centerPose, manual), "#ff5df2", "rgba(255,93,242,0.12)", "manual");
    }
  }
  if (document.getElementById("showRobot").checked) {
    drawPoseDelta(frame.pose_otos_only, frame.pose_fused || frame.pose);
    drawRobot(frame.pose_otos_only, "#ffb64d", "rgba(255,182,77,0.16)", "OTOS-only");
    drawRobot(frame.pose_fused || frame.pose, "#80e0a7", "rgba(128,224,167,0.14)", "fused");
    if (document.getElementById("showModelFit") && document.getElementById("showModelFit").checked) {
      drawRobot(selectedModelFitPose(), "#58d6ff", "rgba(88,214,255,0.13)", "model fit");
    }
    if (frame.pose_lidar && frame.pose_lidar.valid) {
      drawRobot(frame.pose_lidar, "#61c8ff", "rgba(97,200,255,0.14)", "LiDAR-only");
    }
  }
  if (document.getElementById("showLabels").checked) {
    ctx.fillStyle = "#d9e4ee";
    ctx.font = "14px ui-sans-serif, system-ui";
    ctx.fillText(`Frame ${frame.index + 1}/${state.frame_count}`, 18, 28);
    ctx.fillText(`${frame.motion_bucket}  speed=${fmt(frame.speed_mps, 3)} m/s  yaw=${fmt(frame.yaw_dps, 1)} dps`, 18, 48);
    ctx.fillStyle = "#ffb64d";
    ctx.fillText("OTOS-only path", 18, 70);
    ctx.fillStyle = "#80e0a7";
    ctx.fillText("fused path", 142, 70);
    ctx.fillStyle = "#58d6ff";
    ctx.fillText("fitted OTOS", 232, 70);
    ctx.fillStyle = "#ff5df2";
    ctx.fillText("manual corrected path", 332, 70);
  }
}
function updatePanels(frame) {
  meta.innerHTML = [
    ["Run", state.run_id || "--"],
    ["Frame", `${frame.index + 1} / ${state.frame_count}`],
    ["Scan t", fmt(frame.scan_t, 3)],
    ["Annotations", `${annotationKeys().length}${annotationsDirty ? " unsaved" : ""} -> ${state.annotation_path || "--"}`],
    ["Manual profile", `dx=${fmt(interpolatedAnnotation(frame.index).dx_m * 100, 1)}cm dy=${fmt(interpolatedAnnotation(frame.index).dy_m * 100, 1)}cm dh=${fmt(interpolatedAnnotation(frame.index).dh_deg, 2)}deg`],
    ["Phase", `${frame.race_phase || "--"} / ${frame.race_mode || "--"}`],
    ["Motion", `${frame.motion_bucket}`],
    ["Speed", `${fmt(frame.speed_mps, 3)} m/s`],
    ["Yaw rate", `${fmt(frame.yaw_dps, 1)} deg/s`],
    ["Timestamp", frame.timestamp_source || "--"],
    ["Beam order", frame.beam_time_order || "--"],
    ["Deskew", frame.deskew_mode || "--"],
    ["Raw seam", frame.raw_metric.valid ? `${fmt(frame.raw_metric.abs_delta_m, 4)} m` : frame.raw_metric.reason],
    ["Fused pose", fmtPose(frame.pose_fused || frame.pose)],
    ["OTOS-only pose", fmtPose(frame.pose_otos_only)],
    ["Fused - OTOS", fmtPoseDelta(frame.pose_delta)],
    ["LiDAR-only pose", fmtLidarPose(frame.pose_lidar)],
    ["OTOS model fit", selectedModelFit() ? `${selectedModelFit().label || selectedModelFit().name} p95=${metricValue((selectedModelFit().metrics || {}).p95_m, 1, 100)}cm` : "--"],
  ].map(([a,b]) => `<div>${a}</div><div>${b}</div>`).join("");
  updateModelFitPanel();
  const validModels = frame.models.filter(m => m.metrics.line_valid);
  const best = validModels.length ? validModels.reduce((a,b) => a.metrics.line_abs_delta_m <= b.metrics.line_abs_delta_m ? a : b).name : "";
  metricsEl.innerHTML = `<tr><th>Model</th><th>Field</th><th>Line</th><th>MAD</th><th>Wall</th></tr>` + frame.models.map(m => {
    const mm = m.metrics;
    const cls = m.name === best ? "winner" : "";
    const field = mm.field_valid ? fmt(mm.field_abs_delta_m, 4) : `<span class="bad">${mm.field_reason || "--"}</span>`;
    const line = mm.line_valid ? fmt(mm.line_abs_delta_m, 4) : `<span class="bad">${mm.line_reason || "--"}</span>`;
    return `<tr class="${cls}"><td>${m.label}</td><td>${field}</td><td>${line}</td><td>${fmt(mm.line_mad_m, 4)}</td><td>${mm.field_wall || "--"}</td></tr>`;
  }).join("");
}
async function refreshSummary() {
  const res = await fetch("/api/summary");
  state = await res.json();
  frameSlider.max = Math.max(0, state.frame_count - 1);
  sampleByIndex = {};
  for (const sample of state.path_samples || []) sampleByIndex[Number(sample.index)] = sample;
  const fitModels = (state.model_fit && state.model_fit.models) ? state.model_fit.models : [];
  if (fitModels.length && (!selectedFit || !fitModels.some(m => m.name === selectedFit))) selectedFit = state.model_fit.recommended || fitModels[0].name;
  annotations = {};
  for (const [key, value] of Object.entries(state.annotations || {})) annotations[String(Number(key))] = manualClean(value);
  annotationsDirty = false;
}
async function loadFrame(idx) {
  current = Math.max(0, Math.min(state.frame_count - 1, idx));
  frameSlider.value = current;
  const res = await fetch(`/api/frame?index=${current}`);
  const frame = await res.json();
  currentFrame = frame;
  setManualControls(interpolatedAnnotation(current));
  draw(frame);
  updatePanels(frame);
}
async function saveManual() {
  if (!currentFrame) return;
  updateManualStatus("Saving all staged keyframes...");
  const res = await fetch("/api/annotations", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({frames: annotations}),
  });
  if (!res.ok) throw new Error(await res.text());
  await refreshSummary();
  await loadFrame(current);
  updateManualStatus("Saved all.");
}
function deleteManual() {
  if (!currentFrame) return;
  deleteStagedAnnotation(current);
  setManualControls(interpolatedAnnotation(current));
  draw(currentFrame);
  updatePanels(currentFrame);
}
function buildModelToggles() {
  modelsEl.innerHTML = state.models.map(m => {
    colors[m.name] = m.color;
    enabled[m.name] = m.name === "stored" || m.name === "current_recomputed" || m.name === "forward_beam_order" || m.name === "reverse_beam_order";
    return `<label class="model"><input data-model="${m.name}" type="checkbox" ${enabled[m.name] ? "checked" : ""} /><span class="swatch" style="background:${m.color}"></span>${m.label}</label>`;
  }).join("");
  modelsEl.querySelectorAll("input").forEach(input => {
    input.addEventListener("change", () => {
      enabled[input.dataset.model] = input.checked;
      loadFrame(current);
    });
  });
}
async function init() {
  await refreshSummary();
  buildModelToggles();
  buildModelFitSelect();
  jump.innerHTML = `<option value="">Jump...</option>` + state.jump_frames.map(item => `<option value="${item.index}">${item.label}</option>`).join("");
  frameSlider.addEventListener("input", () => loadFrame(Number(frameSlider.value)));
  document.getElementById("prev").addEventListener("click", () => loadFrame(current - 1));
  document.getElementById("next").addEventListener("click", () => loadFrame(current + 1));
  jump.addEventListener("change", () => { if (jump.value !== "") loadFrame(Number(jump.value)); jump.value = ""; });
  ["showSeam", "showFit", "showRobot", "showLabels", "showManual", "showPaths", "showModelFit", "showField"].forEach(id => document.getElementById(id).addEventListener("change", () => { if (currentFrame) { draw(currentFrame); updatePanels(currentFrame); } }));
  document.getElementById("modelFitSelect").addEventListener("change", (ev) => {
    selectedFit = ev.target.value;
    renderModelFitTuneControls();
    updateModelFitPanel();
    if (currentFrame) { draw(currentFrame); updatePanels(currentFrame); }
  });
  document.getElementById("resetModelFit").addEventListener("click", () => {
    const fit = selectedModelFit();
    if (fit) {
      delete modelFitEdits[fit.name];
      delete modelFitTouched[fit.name];
    }
    renderModelFitTuneControls();
    updateModelFitPanel("Reset selected model sliders to fitted defaults.");
    if (currentFrame) { draw(currentFrame); updatePanels(currentFrame); }
  });
  document.getElementById("fitModelToCurrent").addEventListener("click", () => {
    const fit = selectedModelFit();
    if (fit) {
      delete modelFitEdits[fit.name];
      delete modelFitTouched[fit.name];
    }
    renderModelFitTuneControls();
    updateModelFitPanel("Loaded fitted defaults for the selected model.");
    if (currentFrame) { draw(currentFrame); updatePanels(currentFrame); }
  });
  document.getElementById("exportModelFit").addEventListener("click", () => exportSelectedModelFit().catch(err => updateModelFitPanel(`Export failed: ${err.message || err}`)));
  for (const id of ["manualX", "manualY", "manualH", "manualXNum", "manualYNum", "manualHNum"]) {
    document.getElementById(id).addEventListener("input", (ev) => updateManualFromInputs(id, ev.target.value));
  }
  document.getElementById("saveManual").addEventListener("click", () => saveManual().catch(err => updateManualStatus(`Save failed: ${err.message || err}`)));
  document.getElementById("deleteManual").addEventListener("click", () => deleteManual());
  document.getElementById("zeroManual").addEventListener("click", () => {
    manual = {dx_m: 0, dy_m: 0, dh_deg: 0};
    for (const [id, val] of [["manualX", 0], ["manualY", 0], ["manualH", 0]]) {
      document.getElementById(id).value = String(val);
      document.getElementById(id + "Num").value = String(val);
    }
    manualDirty = true;
    stageAnnotation(current, manual);
    updateManualStatus();
    if (currentFrame) draw(currentFrame);
    if (currentFrame) updatePanels(currentFrame);
  });
  document.addEventListener("keydown", (ev) => {
    const tag = ev.target && ev.target.tagName ? ev.target.tagName.toUpperCase() : "";
    if (["INPUT", "SELECT", "TEXTAREA", "BUTTON"].includes(tag)) return;
    if (ev.key === "ArrowLeft") { ev.preventDefault(); loadFrame(current - 1); }
    if (ev.key === "ArrowRight") { ev.preventDefault(); loadFrame(current + 1); }
  });
  await loadFrame(0);
}
init().catch(err => {
  document.body.innerHTML = `<pre>${err.stack || err}</pre>`;
});
</script>
</body>
</html>
"""


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class DeskewViewer:
    def __init__(
        self,
        index_path: Path,
        seam_window_deg: float | None,
        min_window_points: int,
        same_wall_gate_m: float,
        max_line_mad_m: float,
        max_points: int,
        annotation_path: Path | None = None,
        model_fit_path: Path | None = None,
    ):
        self.index_path = index_path
        self.resolved_index_path = index_path.resolve()
        self.frames = _load_frames(index_path)
        self.seam_window_deg = seam_window_deg
        self.min_window_points = min_window_points
        self.same_wall_gate_m = same_wall_gate_m
        self.max_line_mad_m = max_line_mad_m
        self.max_points = max(50, int(max_points))
        self.run_id = self.resolved_index_path.parent.name
        self.annotation_path = annotation_path or self.resolved_index_path.with_name(f"{self.resolved_index_path.stem}_manual_alignment.json")
        self.model_fit_path = model_fit_path or self.resolved_index_path.with_name(f"{self.resolved_index_path.stem}_otos_model_fit.json")
        self._cache: dict[int, dict[str, Any]] = {}
        self._model_fit_cache: dict[str, Any] | None = None
        self._lock = threading.Lock()
        self.annotations: dict[int, dict[str, float]] = self._load_annotations()
        self._jump_frames = self._compute_jump_frames()

    def _model_metric(self, frame: dict, model: str, points: np.ndarray) -> dict:
        field = _same_wall_field_metric(frame, points, self.seam_window_deg, self.min_window_points, self.same_wall_gate_m)
        line = _same_wall_line_metric(field, frame, points, self.seam_window_deg, self.min_window_points, self.max_line_mad_m)
        return {
            "field_valid": bool(field.get("valid", False)),
            "field_reason": str(field.get("reason", "")),
            "field_wall": str(field.get("wall", "")),
            "field_abs_delta_m": _json_number(field.get("abs_delta_m")),
            "field_signed_delta_m": _json_number(field.get("signed_delta_m")),
            "field_wall_mad_m": _json_number(field.get("wall_mad_m")),
            "line_valid": bool(line.get("valid", False)),
            "line_reason": str(line.get("reason", "")),
            "line_abs_delta_m": _json_number(line.get("abs_delta_m")),
            "line_signed_delta_m": _json_number(line.get("signed_delta_m")),
            "line_mad_m": _json_number(line.get("line_mad_m")),
        }

    def _compute_jump_frames(self) -> list[dict[str, Any]]:
        scored: list[tuple[float, int, str]] = []
        for i, frame in enumerate(self.frames):
            points = _model_points(frame, "stored")
            metric = self._model_metric(frame, "stored", points)
            line = metric.get("line_abs_delta_m")
            if isinstance(line, (int, float)) and math.isfinite(float(line)):
                scored.append((float(line), i, "stored line"))
            raw = _raw_range_seam_metric(frame, self.seam_window_deg, self.min_window_points)
            val = raw.get("abs_delta_m")
            if raw.get("valid") and isinstance(val, (int, float)) and math.isfinite(float(val)):
                scored.append((float(val), i, "raw range"))
        result = []
        seen = set()
        for value, idx, reason in sorted(scored, reverse=True):
            if idx in seen:
                continue
            seen.add(idx)
            speed, yaw, bucket = _frame_motion(self.frames[idx])
            result.append({"index": idx, "label": f"{idx + 1}: {reason} {value:.3f}m {bucket} {speed:.2f}m/s {yaw:.0f}dps"})
            if len(result) >= 24:
                break
        return sorted(result, key=lambda item: item["index"])

    def summary(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "path": str(self.index_path),
            "resolved_path": str(self.resolved_index_path),
            "annotation_path": str(self.annotation_path),
            "annotation_count": len(self.annotations),
            "annotations": self._serialized_annotations(),
            "path_samples": self._path_pose_samples(),
            "model_fit": self._model_fit_summary(),
            "frame_count": len(self.frames),
            "models": [{"name": name, "label": label, "color": color} for name, label, color in MODELS],
            "jump_frames": self._jump_frames,
        }

    def _serialized_annotations(self) -> dict[str, dict[str, float]]:
        return {str(idx): dict(corr) for idx, corr in sorted(self.annotations.items())}

    def _path_pose_samples(self) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        obs_by_index = {int(obs["index"]): obs for obs in _model_fit_observations(self.frames, fit_only=False)}
        for i, frame in enumerate(self.frames):
            otos_pose = _otos_only_pose_json(frame)
            fused_pose = _pose_json_from_value(frame.get("fused_pose", frame.get("reference_pose", frame.get("current_pose", ()))))
            base_pose = otos_pose or fused_pose
            obs = obs_by_index.get(i, {})
            samples.append(
                {
                    "index": i,
                    "scan_t": _json_number(frame.get("scan_t")),
                    "otos": otos_pose,
                    "fused": fused_pose,
                    "base": base_pose,
                    "progress_m": _json_number(obs.get("progress_m")),
                    "section": str(obs.get("section", "")),
                    "phase": str(obs.get("phase", frame.get("race_phase", ""))),
                    "speed_mps": _json_number(obs.get("speed_mps")),
                    "fit_sample": bool(obs.get("fit_sample", False)),
                }
            )
        return samples


    def _model_fit_summary(self) -> dict[str, Any]:
        if self._model_fit_cache is not None:
            return self._model_fit_cache
        all_obs = _model_fit_observations(self.frames, fit_only=False)
        fit_obs = _model_fit_observations(self.frames, fit_only=True)
        models: list[dict[str, Any]] = []
        if fit_obs:
            raw = _make_otos_fit_model("raw_otos", "Raw OTOS", "none", {}, fit_obs, all_obs)
            models.append(raw)
            offset_h_frame = _make_otos_fit_model(
                "offset_h_frame_1809",
                "OTOS offset-h frame sim: 180.9deg",
                "otos_offset_h_frame_sim",
                {
                    "base_offset_h_deg": 183.4,
                    "target_offset_h_deg": 180.9,
                    "offset_x_m": 0.1261,
                    "offset_y_m": -0.0161,
                    "dx_m": 0.0,
                    "dy_m": 0.0,
                    "heading_effect_sign": -1.0,
                    "frame_effect_sign": -1.0,
                    "heading_output_delta_deg": 0.0,
                    "max_continuous_step_m": 0.35,
                },
                fit_obs,
                all_obs,
            )
            models.append(offset_h_frame)
            offset_h_rigid = _make_otos_fit_model(
                "offset_h_rigid_1809",
                "OTOS offset-h rigid transform: 180.9deg",
                "otos_offset_h_rigid_sim",
                {
                    "base_offset_h_deg": 183.4,
                    "target_offset_h_deg": 180.9,
                    "offset_x_m": 0.1261,
                    "offset_y_m": -0.0161,
                    "dx_m": 0.0,
                    "dy_m": 0.0,
                    "heading_effect_sign": -1.0,
                    "frame_effect_sign": -1.0,
                    "heading_output_delta_deg": 0.0,
                    "max_continuous_step_m": 0.35,
                },
                fit_obs,
                all_obs,
            )
            models.append(offset_h_rigid)
            reference_offset = _fit_reference_offset_otos_model(fit_obs, all_obs)
            if reference_offset:
                models.append(reference_offset)
            constant = _fit_constant_otos_model(fit_obs, all_obs)
            if constant:
                models.append(constant)
            origin_xy = _model_fit_origin(all_obs or fit_obs, self.annotations)
            similarity = _fit_similarity_otos_model(fit_obs, all_obs, origin_xy)
            if similarity:
                models.append(similarity)
            affine = _fit_affine_otos_model(fit_obs, all_obs, origin_xy)
            if affine:
                models.append(affine)
            section = _fit_section_offset_otos_model(fit_obs, all_obs)
            if section:
                models.append(section)
            periodic = _fit_similarity_periodic_otos_model(fit_obs, all_obs, similarity)
            if periodic:
                models.append(periodic)
        imported = self._load_external_model_fit(all_obs, fit_obs)
        if imported:
            models.append(imported)
        recommended = "similarity" if any(m.get("name") == "similarity" for m in models) else (models[0]["name"] if models else "")
        ox, oy = _model_fit_origin(all_obs or fit_obs, self.annotations)
        payload = {
            "fit_sample_count": len(fit_obs),
            "path_sample_count": len(all_obs),
            "fit_filter": {"race_phase": "navigation", "min_abs_speed_mps": 0.03},
            "model_fit_path": str(self.model_fit_path),
            "transform_origin_m": [round(float(ox), 6), round(float(oy), 6)],
            "recommended": recommended,
            "models": models,
        }
        self._model_fit_cache = _json_safe(payload)
        return self._model_fit_cache

    def _load_external_model_fit(self, all_obs: list[dict[str, Any]], fit_obs: list[dict[str, Any]]) -> dict[str, Any] | None:
        path = self.model_fit_path
        if not path.exists():
            return None
        try:
            with path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception:
            return None
        model = data.get("model") if isinstance(data, dict) else None
        if not isinstance(model, dict):
            return None
        name = str(model.get("name") or model.get("type") or "imported")
        params = model.get("params", {}) if isinstance(model.get("params"), dict) else {}
        model_type = str(model.get("type") or name)
        imported_name = f"imported_{name}"
        return _make_otos_fit_model(imported_name, f"Imported: {model.get('label') or name}", model_type, params, fit_obs, all_obs)

    def export_model_fit(self, name: str, model_override: dict[str, Any] | None = None) -> dict[str, Any]:
        if isinstance(model_override, dict):
            compact = {k: v for k, v in model_override.items() if k != "path"}
            compact.setdefault("name", name or "tuned_model")
            compact.setdefault("label", compact.get("name", "tuned_model"))
        else:
            fit = next((m for m in self._model_fit_summary().get("models", []) if m.get("name") == name), None)
            if fit is None:
                raise ValueError(f"unknown model fit: {name}")
            compact = {k: v for k, v in fit.items() if k != "path"}
        payload = {
            "version": 1,
            "index_path": str(self.resolved_index_path),
            "run_id": self.run_id,
            "updated_t": time.time(),
            "model": compact,
        }
        self.model_fit_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.model_fit_path.with_suffix(self.model_fit_path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as fp:
            json.dump(_json_safe(payload), fp, indent=2, sort_keys=True)
            fp.write("\n")
        tmp.replace(self.model_fit_path)
        return {"ok": True, "name": str(compact.get("name", name)), "path": str(self.model_fit_path)}

    def _load_annotations(self) -> dict[int, dict[str, float]]:
        path = self.annotation_path
        if not path.exists():
            return {}
        try:
            with path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception:
            return {}
        frames = data.get("frames", {}) if isinstance(data, dict) else {}
        out: dict[int, dict[str, float]] = {}
        if isinstance(frames, dict):
            iterable = frames.items()
        elif isinstance(frames, list):
            iterable = ((item.get("index"), item) for item in frames if isinstance(item, dict))
        else:
            iterable = ()
        for key, value in iterable:
            if not isinstance(value, dict):
                continue
            try:
                idx = int(key)
            except (TypeError, ValueError):
                continue
            if idx < 0 or idx >= len(self.frames):
                continue
            out[idx] = {
                "dx_m": float(value.get("dx_m", 0.0) or 0.0),
                "dy_m": float(value.get("dy_m", 0.0) or 0.0),
                "dh_deg": float(value.get("dh_deg", 0.0) or 0.0),
            }
        return out

    def _save_annotations(self) -> None:
        self.annotation_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "index_path": str(self.resolved_index_path),
            "updated_t": time.time(),
            "frames": {
                str(idx): {
                    "index": idx,
                    "scan_t": _json_number(self.frames[idx].get("scan_t")) if 0 <= idx < len(self.frames) else None,
                    "dx_m": corr["dx_m"],
                    "dy_m": corr["dy_m"],
                    "dh_deg": corr["dh_deg"],
                }
                for idx, corr in sorted(self.annotations.items())
            },
        }
        tmp = self.annotation_path.with_suffix(self.annotation_path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as fp:
            json.dump(payload, fp, indent=2, sort_keys=True)
            fp.write("\n")
        tmp.replace(self.annotation_path)

    def replace_annotations(self, frames: dict) -> dict[str, Any]:
        out: dict[int, dict[str, float]] = {}
        if not isinstance(frames, dict):
            raise ValueError("frames must be an object keyed by frame index")
        for key, value in frames.items():
            if not isinstance(value, dict):
                continue
            idx = int(key)
            if idx < 0 or idx >= len(self.frames):
                continue
            corr = {
                "dx_m": float(value.get("dx_m", 0.0) or 0.0),
                "dy_m": float(value.get("dy_m", 0.0) or 0.0),
                "dh_deg": float(value.get("dh_deg", 0.0) or 0.0),
            }
            out[idx] = corr
        with self._lock:
            self.annotations = out
            self._cache.clear()
            self._model_fit_cache = None
            self._save_annotations()
        return {"ok": True, "annotation_count": len(self.annotations), "annotation_path": str(self.annotation_path), "annotations": self._serialized_annotations()}

    def set_annotation(self, index: int, dx_m: float, dy_m: float, dh_deg: float) -> dict[str, Any]:
        index = max(0, min(len(self.frames) - 1, int(index)))
        corr = {"dx_m": float(dx_m), "dy_m": float(dy_m), "dh_deg": float(dh_deg)}
        with self._lock:
            self.annotations[index] = corr
            self._cache.clear()
            self._model_fit_cache = None
            self._save_annotations()
        return {"ok": True, "index": index, "annotation": corr, "annotation_count": len(self.annotations), "annotation_path": str(self.annotation_path)}

    def delete_annotation(self, index: int) -> dict[str, Any]:
        index = max(0, min(len(self.frames) - 1, int(index)))
        with self._lock:
            self.annotations.pop(index, None)
            self._cache.clear()
            self._model_fit_cache = None
            self._save_annotations()
        return {"ok": True, "index": index, "annotation_count": len(self.annotations), "annotation_path": str(self.annotation_path)}

    def _frame_scan_t(self, index: int) -> float:
        if not self.frames:
            return float(index)
        frame = self.frames[max(0, min(len(self.frames) - 1, int(index)))]
        try:
            value = float(frame.get("scan_t", index) or index)
        except (TypeError, ValueError):
            value = float(index)
        return value if math.isfinite(value) else float(index)

    def _annotation_for_index(self, index: int) -> dict[str, float] | None:
        corr = self.annotations.get(int(index))
        return dict(corr) if corr is not None else None

    def _interpolated_annotation(self, index: int) -> dict[str, float]:
        if not self.annotations:
            return {"dx_m": 0.0, "dy_m": 0.0, "dh_deg": 0.0}
        index = max(0, min(len(self.frames) - 1, int(index)))
        if index in self.annotations:
            return dict(self.annotations[index])
        keys = sorted(self.annotations)
        t = self._frame_scan_t(index)
        before = [k for k in keys if self._frame_scan_t(k) <= t]
        after = [k for k in keys if self._frame_scan_t(k) >= t]
        if not before:
            return dict(self.annotations[keys[0]])
        if not after:
            return dict(self.annotations[keys[-1]])
        lo = before[-1]
        hi = after[0]
        if lo == hi:
            return dict(self.annotations[lo])
        t0 = self._frame_scan_t(lo)
        t1 = self._frame_scan_t(hi)
        u = 0.0 if t1 <= t0 else max(0.0, min(1.0, (t - t0) / (t1 - t0)))
        a = self.annotations[lo]
        b = self.annotations[hi]
        return {
            "dx_m": float(a["dx_m"] + (b["dx_m"] - a["dx_m"]) * u),
            "dy_m": float(a["dy_m"] + (b["dy_m"] - a["dy_m"]) * u),
            "dh_deg": float(_wrap_deg(a["dh_deg"] + _wrap_deg(b["dh_deg"] - a["dh_deg"]) * u)),
        }

    def _paths_payload(self) -> dict[str, list[list[float]]]:
        otos: list[list[float]] = []
        fused: list[list[float]] = []
        corrected: list[list[float]] = []
        markers: list[list[float]] = []
        if not self.frames:
            return {"otos_only": [], "fused": [], "manual_corrected": [], "annotation_markers": []}
        step = max(1, int(math.ceil(len(self.frames) / max(50, min(1200, self.max_points)))))
        selected = set(range(0, len(self.frames), step)) | set(self.annotations.keys()) | {0, len(self.frames) - 1}
        for i in sorted(idx for idx in selected if 0 <= idx < len(self.frames)):
            frame = self.frames[i]
            otos_pose = _otos_only_pose_json(frame)
            fused_pose = _pose_json_from_value(frame.get("fused_pose", frame.get("reference_pose", frame.get("current_pose", ()))))
            base_pose = otos_pose or fused_pose
            if otos_pose and otos_pose.get("x") is not None and otos_pose.get("y") is not None:
                otos.append([round(float(otos_pose["x"]), 4), round(float(otos_pose["y"]), 4)])
            if fused_pose and fused_pose.get("x") is not None and fused_pose.get("y") is not None:
                fused.append([round(float(fused_pose["x"]), 4), round(float(fused_pose["y"]), 4)])
            if base_pose and base_pose.get("x") is not None and base_pose.get("y") is not None:
                corr = self._interpolated_annotation(i)
                x = float(base_pose["x"]) + float(corr.get("dx_m", 0.0))
                y = float(base_pose["y"]) + float(corr.get("dy_m", 0.0))
                corrected.append([round(x, 4), round(y, 4)])
                if i in self.annotations:
                    markers.append([round(x, 4), round(y, 4)])
        return {"otos_only": otos, "fused": fused, "manual_corrected": corrected, "annotation_markers": markers}

    def frame_payload(self, index: int) -> dict[str, Any]:
        if not self.frames:
            raise IndexError("no diagnostic frames loaded")
        index = max(0, min(len(self.frames) - 1, int(index)))
        with self._lock:
            cached = self._cache.get(index)
        if cached is not None:
            return cached
        frame = self.frames[index]
        speed, yaw, bucket = _frame_motion(frame)
        raw_metric = _raw_range_seam_metric(frame, self.seam_window_deg, self.min_window_points)
        models = []
        for name, label, color in MODELS:
            model_name = "current" if name == "current_recomputed" else name
            points = _model_points(frame, model_name)
            seam_first, seam_last, _win = _seam_window_points(frame, points, self.seam_window_deg)
            models.append(
                {
                    "name": name,
                    "label": label,
                    "color": color,
                    "points": _points_json(points, self.max_points),
                    "seam_first": _points_json(seam_first, self.max_points),
                    "seam_last": _points_json(seam_last, self.max_points),
                    "metrics": self._model_metric(frame, name, points),
                }
            )
        fused_pose = _pose_json_from_value(frame.get("fused_pose", frame.get("reference_pose", frame.get("current_pose", ()))))
        otos_only_pose = _otos_only_pose_json(frame)
        lidar_pose = _lidar_estimate_pose_json(frame.get("lidar_estimate"))
        pose_delta = _pose_delta_json(fused_pose, otos_only_pose)
        saved_manual = self._annotation_for_index(index)
        manual_profile = self._interpolated_annotation(index)
        payload = {
            "index": index,
            "scan_t": _json_number(frame.get("scan_t")),
            "race_phase": str(frame.get("race_phase", "")),
            "race_mode": str(frame.get("race_mode", "")),
            "source": str(frame.get("source", "")),
            "timestamp_source": str(frame.get("timestamp_source", "")),
            "beam_time_order": _frame_beam_time_order(frame),
            "deskew_mode": str(frame.get("deskew_mode", "")),
            "motion_bucket": bucket,
            "speed_mps": _json_number(speed),
            "yaw_dps": _json_number(yaw),
            "raw_metric": _json_safe(raw_metric),
            "pose": fused_pose,
            "pose_fused": fused_pose,
            "pose_otos_only": otos_only_pose,
            "pose_lidar": lidar_pose,
            "pose_delta": pose_delta,
            "manual_saved": saved_manual is not None,
            "manual": saved_manual if saved_manual is not None else manual_profile,
            "manual_profile": manual_profile,
            "paths": self._paths_payload(),
            "models": models,
        }
        with self._lock:
            if len(self._cache) > 64:
                self._cache.clear()
            self._cache[index] = payload
        return payload


def _pose_tuple(value) -> tuple[float, float, float, float, float, float, float]:
    if value is None:
        vals = []
    else:
        vals = np.asarray(value, dtype=float).reshape(-1).tolist()
    while len(vals) < 7:
        vals.append(0.0)
    return tuple(float(v) for v in vals[:7])


def _pose_json_from_value(value) -> dict[str, float | None] | None:
    if value is None:
        return None
    current_pose = _pose_tuple(value)
    return {"x": _json_number(current_pose[1]), "y": _json_number(current_pose[2]), "h": _json_number(current_pose[3])}


def _local_pose_to_display_json(
    local_pose: dict[str, float | None] | None,
    anchor_pose: dict[str, float | None] | None,
) -> dict[str, float | None] | None:
    if not local_pose or not anchor_pose:
        return None
    try:
        local_x = float(local_pose["x"])
        local_y = float(local_pose["y"])
        local_h = float(local_pose["h"])
        anchor_x = float(anchor_pose["x"])
        anchor_y = float(anchor_pose["y"])
        anchor_h = float(anchor_pose["h"])
    except (TypeError, ValueError, KeyError):
        return None
    if not all(math.isfinite(v) for v in (local_x, local_y, local_h, anchor_x, anchor_y, anchor_h)):
        return None
    ca = math.cos(math.radians(anchor_h))
    sa = math.sin(math.radians(anchor_h))
    return {
        "x": _json_number(anchor_x + ca * local_x - sa * local_y),
        "y": _json_number(anchor_y + sa * local_x + ca * local_y),
        "h": _json_number(_wrap_deg(anchor_h + local_h)),
    }


def _otos_only_pose_json(frame: dict[str, Any]) -> dict[str, float | None] | None:
    pose = _pose_json_from_value(frame.get("otos_only_pose")) if "otos_only_pose" in frame else None
    if pose and pose.get("x") is not None and pose.get("y") is not None and pose.get("h") is not None:
        return pose
    local_pose = _pose_json_from_value(frame.get("reference_local_pose", frame.get("current_pose", ())))
    otos_base_anchor = _pose_json_from_value(frame.get("otos_base_anchor"))
    return _local_pose_to_display_json(local_pose, otos_base_anchor)


def _wrap_deg(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def _pose_delta_json(fused: dict[str, float | None] | None, otos_only: dict[str, float | None] | None) -> dict[str, float | None] | None:
    if not fused or not otos_only:
        return None
    try:
        dx = float(fused["x"]) - float(otos_only["x"])
        dy = float(fused["y"]) - float(otos_only["y"])
        dh = _wrap_deg(float(fused["h"]) - float(otos_only["h"]))
    except (TypeError, ValueError, KeyError):
        return None
    return {"dx_m": _json_number(dx), "dy_m": _json_number(dy), "dh_deg": _json_number(dh), "distance_m": _json_number(math.hypot(dx, dy))}


def _pose_xyh(pose: dict[str, float | None] | None) -> tuple[float, float, float] | None:
    if not pose:
        return None
    try:
        x = float(pose["x"])
        y = float(pose["y"])
        h = float(pose["h"])
    except (TypeError, ValueError, KeyError):
        return None
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(h)):
        return None
    return x, y, h


def _section_from_xy(x: float, y: float) -> str:
    if abs(float(x)) >= abs(float(y)):
        return "right" if x >= 0.0 else "left"
    return "top" if y >= 0.0 else "bottom"


def _model_fit_observations(frames: list[dict[str, Any]], fit_only: bool) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    progress = 0.0
    last_fused: tuple[float, float] | None = None
    for idx, frame in enumerate(frames):
        otos_pose = _pose_xyh(_otos_only_pose_json(frame))
        fused_pose = _pose_xyh(_pose_json_from_value(frame.get("fused_pose", frame.get("reference_pose", frame.get("current_pose", ())))))
        if fused_pose is not None:
            if last_fused is not None:
                step = math.hypot(fused_pose[0] - last_fused[0], fused_pose[1] - last_fused[1])
                if math.isfinite(step) and step < 0.25:
                    progress += step
            last_fused = (fused_pose[0], fused_pose[1])
        if otos_pose is None:
            continue
        speed, yaw, bucket = _frame_motion(frame)
        try:
            abs_speed = abs(float(speed))
        except (TypeError, ValueError):
            abs_speed = 0.0
        phase = str(frame.get("race_phase", ""))
        reference_xy = fused_pose if fused_pose is not None else otos_pose
        section = _section_from_xy(reference_xy[0], reference_xy[1])
        include = fused_pose is not None and phase == "navigation" and abs_speed >= 0.03
        if fit_only and not include:
            continue
        item = {
            "index": idx,
            "scan_t": _json_number(frame.get("scan_t")),
            "progress_m": float(progress),
            "section": section,
            "phase": phase,
            "speed_mps": _json_number(speed),
            "yaw_dps": _json_number(yaw),
            "motion_bucket": bucket,
            "fit_sample": bool(include),
            "otos_x": float(otos_pose[0]),
            "otos_y": float(otos_pose[1]),
            "otos_h": float(otos_pose[2]),
            "fused_x": float(fused_pose[0]) if fused_pose is not None else None,
            "fused_y": float(fused_pose[1]) if fused_pose is not None else None,
            "fused_h": float(fused_pose[2]) if fused_pose is not None else None,
        }
        out.append(item)
    return out


def _model_fit_origin(obs_list: list[dict[str, Any]], annotations: dict[int, dict[str, float]] | None = None) -> tuple[float, float]:
    ordered = sorted(obs_list, key=lambda item: int(item["index"]))
    if annotations and ordered:
        by_index = {int(obs["index"]): obs for obs in ordered}
        for idx, corr in sorted(annotations.items()):
            obs = by_index.get(int(idx))
            if obs is None:
                obs = min(ordered, key=lambda item: abs(int(item["index"]) - int(idx)))
            try:
                x = float(obs["otos_x"]) + float(corr.get("dx_m", 0.0) or 0.0)
                y = float(obs["otos_y"]) + float(corr.get("dy_m", 0.0) or 0.0)
            except (TypeError, ValueError, KeyError, AttributeError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                return x, y
    for obs in ordered:
        try:
            x = float(obs["otos_x"])
            y = float(obs["otos_y"])
        except (TypeError, ValueError, KeyError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    return 0.0, 0.0


def _xy_metrics(errors: np.ndarray) -> dict[str, Any]:
    arr = np.asarray(errors, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"count": 0, "median_m": None, "p95_m": None, "mean_m": None, "max_m": None}
    return {
        "count": int(arr.size),
        "median_m": float(np.median(arr)),
        "p95_m": float(np.percentile(arr, 95)),
        "mean_m": float(np.mean(arr)),
        "max_m": float(np.max(arr)),
    }


def _apply_otos_xy_model(model_type: str, params: dict[str, Any], x: float, y: float, section: str, progress_m: float) -> tuple[float, float]:
    if model_type in ("none", "raw_otos", "raw", "constant"):
        return float(x) + float(params.get("dx_m", 0.0) or 0.0), float(y) + float(params.get("dy_m", 0.0) or 0.0)
    if model_type in ("similarity", "affine"):
        matrix = params.get("matrix", [[1.0, 0.0], [0.0, 1.0]])
        trans = params.get("translation_m", [0.0, 0.0])
        origin = params.get("transform_origin_m", [0.0, 0.0])
        m00, m01 = float(matrix[0][0]), float(matrix[0][1])
        m10, m11 = float(matrix[1][0]), float(matrix[1][1])
        tx, ty = float(trans[0]), float(trans[1])
        ox, oy = float(origin[0]), float(origin[1])
        rx, ry = float(x) - ox, float(y) - oy
        return ox + m00 * rx + m01 * ry + tx, oy + m10 * rx + m11 * ry + ty
    if model_type == "section_offsets":
        offsets = params.get("offsets_m", {})
        off = offsets.get(section, offsets.get("default", [0.0, 0.0])) if isinstance(offsets, dict) else [0.0, 0.0]
        return float(x) + float(off[0]), float(y) + float(off[1])
    if model_type == "similarity_periodic":
        base = params.get("base", {}) if isinstance(params.get("base"), dict) else {}
        bx, by = _apply_otos_xy_model("similarity", base, x, y, section, progress_m)
        period = float(params.get("period_m", 0.0) or 0.0)
        if period <= 1e-6:
            return bx, by
        phi = 2.0 * math.pi * (float(progress_m) % period) / period
        features = [math.sin(phi), math.cos(phi), math.sin(2.0 * phi), math.cos(2.0 * phi)]
        xc = params.get("x_coeffs_m", [0.0, 0.0, 0.0, 0.0])
        yc = params.get("y_coeffs_m", [0.0, 0.0, 0.0, 0.0])
        dx = sum(float(a) * b for a, b in zip(xc, features))
        dy = sum(float(a) * b for a, b in zip(yc, features))
        return bx + dx, by + dy
    return float(x), float(y)


def _offset_heading_base_delta_deg(params: dict[str, Any]) -> float:
    return float(params.get("target_offset_h_deg", 180.9) or 180.9) - float(params.get("base_offset_h_deg", 183.4) or 183.4)


def _offset_heading_output_delta_deg(params: dict[str, Any]) -> float:
    sign = float(params.get("heading_effect_sign", params.get("effect_sign", -1.0)) if params.get("heading_effect_sign", params.get("effect_sign", -1.0)) is not None else -1.0)
    return _offset_heading_base_delta_deg(params) * sign + float(params.get("heading_output_delta_deg", 0.0) or 0.0)


def _offset_heading_frame_delta_deg(params: dict[str, Any]) -> float:
    sign = float(params.get("frame_effect_sign", params.get("heading_effect_sign", params.get("effect_sign", -1.0))) if params.get("frame_effect_sign", params.get("heading_effect_sign", params.get("effect_sign", -1.0))) is not None else -1.0)
    return _offset_heading_base_delta_deg(params) * sign


def _rotate_offset(x: float, y: float, h_rad: float) -> tuple[float, float]:
    c = math.cos(h_rad)
    s = math.sin(h_rad)
    return c * x - s * y, s * x + c * y


def _simulate_otos_offset_h_rigid_predictions(obs_list: list[dict[str, Any]], params: dict[str, Any]) -> list[dict[str, float]]:
    predictions: list[dict[str, float]] = []
    ordered = sorted(obs_list, key=lambda item: int(item["index"]))
    dx_bias = float(params.get("dx_m", 0.0) or 0.0)
    dy_bias = float(params.get("dy_m", 0.0) or 0.0)
    offset_x = float(params.get("offset_x_m", 0.1261) if params.get("offset_x_m") is not None else 0.1261)
    offset_y = float(params.get("offset_y_m", -0.0161) if params.get("offset_y_m") is not None else -0.0161)
    heading_delta_deg = _offset_heading_output_delta_deg(params)
    for obs in ordered:
        ox = float(obs["otos_x"])
        oy = float(obs["otos_y"])
        oh = float(obs["otos_h"])
        old_lever_x, old_lever_y = _rotate_offset(offset_x, offset_y, math.radians(oh))
        sensor_x = ox + old_lever_x
        sensor_y = oy + old_lever_y
        target_h = oh + heading_delta_deg
        target_lever_x, target_lever_y = _rotate_offset(offset_x, offset_y, math.radians(target_h))
        predictions.append({"index": int(obs["index"]), "x": float(sensor_x - target_lever_x + dx_bias), "y": float(sensor_y - target_lever_y + dy_bias), "h": float(target_h)})
    return predictions


def _simulate_otos_offset_h_frame_predictions(obs_list: list[dict[str, Any]], params: dict[str, Any]) -> list[dict[str, float]]:
    predictions: list[dict[str, float]] = []
    ordered = sorted(obs_list, key=lambda item: int(item["index"]))
    heading_delta_deg = _offset_heading_output_delta_deg(params)
    frame_delta = math.radians(_offset_heading_frame_delta_deg(params))
    dx_bias = float(params.get("dx_m", 0.0) or 0.0)
    dy_bias = float(params.get("dy_m", 0.0) or 0.0)
    offset_x = float(params.get("offset_x_m", 0.1261) if params.get("offset_x_m") is not None else 0.1261)
    offset_y = float(params.get("offset_y_m", -0.0161) if params.get("offset_y_m") is not None else -0.0161)
    max_step = max(0.02, float(params.get("max_continuous_step_m", 0.35) or 0.35))
    target_sensor_x: float | None = None
    target_sensor_y: float | None = None
    prev: dict[str, Any] | None = None
    for obs in ordered:
        ox = float(obs["otos_x"])
        oy = float(obs["otos_y"])
        oh = float(obs["otos_h"])
        old_lever_x, old_lever_y = _rotate_offset(offset_x, offset_y, math.radians(oh))
        old_sensor_x = ox + old_lever_x
        old_sensor_y = oy + old_lever_y
        target_h = oh + heading_delta_deg
        target_lever_x, target_lever_y = _rotate_offset(offset_x, offset_y, math.radians(target_h))
        if prev is None or target_sensor_x is None or target_sensor_y is None:
            target_sensor_x = old_sensor_x
            target_sensor_y = old_sensor_y
        else:
            prev_oh = float(prev.get("otos_h", oh) or oh)
            prev_old_h = math.radians(prev_oh)
            prev_target_h = math.radians(prev_oh + heading_delta_deg)
            prev_old_lever_x, prev_old_lever_y = _rotate_offset(offset_x, offset_y, prev_old_h)
            prev_sensor_old_x = float(prev["otos_x"]) + prev_old_lever_x
            prev_sensor_old_y = float(prev["otos_y"]) + prev_old_lever_y
            raw_dx = old_sensor_x - prev_sensor_old_x
            raw_dy = old_sensor_y - prev_sensor_old_y
            step = math.hypot(raw_dx, raw_dy)
            if not math.isfinite(step) or step > max_step:
                target_sensor_x = old_sensor_x
                target_sensor_y = old_sensor_y
            else:
                old_c = math.cos(prev_old_h)
                old_s = math.sin(prev_old_h)
                local_forward = raw_dx * old_c + raw_dy * old_s
                local_lateral = -raw_dx * old_s + raw_dy * old_c
                frame_c = math.cos(frame_delta)
                frame_s = math.sin(frame_delta)
                target_forward = frame_c * local_forward - frame_s * local_lateral
                target_lateral = frame_s * local_forward + frame_c * local_lateral
                target_c = math.cos(prev_target_h)
                target_s = math.sin(prev_target_h)
                target_sensor_x += target_forward * target_c - target_lateral * target_s
                target_sensor_y += target_forward * target_s + target_lateral * target_c
        predictions.append({"index": int(obs["index"]), "x": float(target_sensor_x - target_lever_x + dx_bias), "y": float(target_sensor_y - target_lever_y + dy_bias), "h": float(target_h)})
        prev = obs
    return predictions


def _simulate_otos_offset_h_predictions(obs_list: list[dict[str, Any]], params: dict[str, Any]) -> list[dict[str, float]]:
    return _simulate_otos_offset_h_frame_predictions(obs_list, params)

def _simulate_reference_offset_predictions(obs_list: list[dict[str, Any]], params: dict[str, Any]) -> list[dict[str, float]]:
    predictions: list[dict[str, float]] = []
    ordered = sorted(obs_list, key=lambda item: int(item["index"]))
    if params.get("anchor_heading_deg") is None and ordered:
        anchor_heading_deg = float(ordered[0].get("otos_h", 0.0) or 0.0)
    else:
        anchor_heading_deg = float(params.get("anchor_heading_deg", 0.0) or 0.0)
    anchor = math.radians(anchor_heading_deg)
    ca = math.cos(anchor)
    sa = math.sin(anchor)
    translation = params.get("translation_m", [0.0, 0.0])
    local_offset = params.get("local_offset_m", [0.0, 0.0])
    tx = float(translation[0] if len(translation) > 0 else 0.0)
    ty = float(translation[1] if len(translation) > 1 else 0.0)
    rx = float(local_offset[0] if len(local_offset) > 0 else 0.0)
    ry = float(local_offset[1] if len(local_offset) > 1 else 0.0)
    h_bias = float(params.get("heading_output_delta_deg", 0.0) or 0.0)
    for obs in ordered:
        h = math.radians(float(obs.get("otos_h", 0.0) or 0.0))
        c = math.cos(h)
        s = math.sin(h)
        dx = tx + (c - ca) * rx - (s - sa) * ry
        dy = ty + (s - sa) * rx + (c - ca) * ry
        predictions.append({"index": int(obs["index"]), "x": float(obs["otos_x"] + dx), "y": float(obs["otos_y"] + dy), "h": float(obs["otos_h"] + h_bias)})
    return predictions


def _otos_model_predictions(model_type: str, params: dict[str, Any], obs_list: list[dict[str, Any]]) -> list[dict[str, float]]:
    if model_type in ("otos_offset_h_sim", "otos_offset_h_frame_sim"):
        return _simulate_otos_offset_h_frame_predictions(obs_list, params)
    if model_type == "otos_offset_h_rigid_sim":
        return _simulate_otos_offset_h_rigid_predictions(obs_list, params)
    if model_type == "reference_offset_sim":
        return _simulate_reference_offset_predictions(obs_list, params)
    predictions: list[dict[str, float]] = []
    for obs in obs_list:
        px, py = _apply_otos_xy_model(model_type, params, obs["otos_x"], obs["otos_y"], obs["section"], obs["progress_m"])
        predictions.append({"index": int(obs["index"]), "x": float(px), "y": float(py), "h": float(obs["otos_h"])})
    return predictions


def _make_otos_fit_model(name: str, label: str, model_type: str, params: dict[str, Any], fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]]) -> dict[str, Any]:
    errors: list[float] = []
    section_errors: dict[str, list[float]] = {}
    predictions = _otos_model_predictions(model_type, params, all_obs)
    pred_by_index = {int(pred["index"]): pred for pred in predictions}
    for obs in fit_obs:
        pred = pred_by_index.get(int(obs["index"]))
        if pred is None or obs.get("fused_x") is None or obs.get("fused_y") is None:
            continue
        err = math.hypot(float(pred["x"]) - float(obs["fused_x"]), float(pred["y"]) - float(obs["fused_y"]))
        errors.append(err)
        section_errors.setdefault(str(obs["section"]), []).append(err)
    path = [
        {"index": int(pred["index"]), "x": round(float(pred["x"]), 4), "y": round(float(pred["y"]), 4), "h": round(float(pred["h"]), 3)}
        for pred in predictions
    ]
    return {
        "name": name,
        "label": label,
        "type": model_type,
        "params": params,
        "metrics": _xy_metrics(np.asarray(errors, dtype=float)),
        "per_section": {section: _xy_metrics(np.asarray(vals, dtype=float)) for section, vals in sorted(section_errors.items())},
        "path": path,
    }


def _fit_constant_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not fit_obs:
        return None
    offsets = np.asarray([[float(o["fused_x"]) - o["otos_x"], float(o["fused_y"]) - o["otos_y"]] for o in fit_obs], dtype=float)
    if offsets.size == 0:
        return None
    dx, dy = np.mean(offsets, axis=0)
    return _make_otos_fit_model("constant", "Constant XY offset", "constant", {"dx_m": float(dx), "dy_m": float(dy)}, fit_obs, all_obs)


def _fit_reference_offset_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]]) -> dict[str, Any] | None:
    if len(fit_obs) < 4:
        return None
    anchor_heading_deg = float(fit_obs[0].get("otos_h", 0.0) or 0.0)
    anchor = math.radians(anchor_heading_deg)
    ca = math.cos(anchor)
    sa = math.sin(anchor)
    rows: list[list[float]] = []
    values: list[float] = []
    for obs in fit_obs:
        h = math.radians(float(obs.get("otos_h", 0.0) or 0.0))
        c = math.cos(h)
        s = math.sin(h)
        ex = float(obs["fused_x"]) - float(obs["otos_x"])
        ey = float(obs["fused_y"]) - float(obs["otos_y"])
        rows.append([1.0, 0.0, c - ca, -(s - sa)])
        values.append(ex)
        rows.append([0.0, 1.0, s - sa, c - ca])
        values.append(ey)
    if len(rows) < 8:
        return None
    coef, *_ = np.linalg.lstsq(np.asarray(rows, dtype=float), np.asarray(values, dtype=float), rcond=None)
    tx, ty, rx, ry = (float(v) for v in coef)
    params = {
        "translation_m": [tx, ty],
        "local_offset_m": [rx, ry],
        "anchor_heading_deg": anchor_heading_deg,
        "heading_output_delta_deg": 0.0,
    }
    return _make_otos_fit_model("reference_offset", "Reference offset: heading-dependent XY", "reference_offset_sim", params, fit_obs, all_obs)


def _fit_similarity_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]], origin_xy: tuple[float, float] | None = None) -> dict[str, Any] | None:
    if len(fit_obs) < 2:
        return None
    ox, oy = origin_xy if origin_xy is not None else _model_fit_origin(all_obs or fit_obs)
    origin = complex(ox, oy)
    z = np.asarray([complex(o["otos_x"], o["otos_y"]) - origin for o in fit_obs], dtype=np.complex128)
    w = np.asarray([complex(float(o["fused_x"]), float(o["fused_y"])) - origin for o in fit_obs], dtype=np.complex128)
    zc = z - np.mean(z)
    wc = w - np.mean(w)
    denom = np.sum(np.conj(zc) * zc).real
    if denom <= 1e-12:
        return None
    a = np.sum(np.conj(zc) * wc) / denom
    b = np.mean(w) - a * np.mean(z)
    ar = float(a.real)
    ai = float(a.imag)
    params = {
        "matrix": [[ar, -ai], [ai, ar]],
        "translation_m": [float(b.real), float(b.imag)],
        "transform_origin_m": [float(ox), float(oy)],
        "scale": float(abs(a)),
        "rotation_deg": float(math.degrees(math.atan2(ai, ar))),
    }
    return _make_otos_fit_model("similarity", "Similarity: start-origin rotate + scale", "similarity", params, fit_obs, all_obs)


def _fit_affine_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]], origin_xy: tuple[float, float] | None = None) -> dict[str, Any] | None:
    if len(fit_obs) < 3:
        return None
    ox, oy = origin_xy if origin_xy is not None else _model_fit_origin(all_obs or fit_obs)
    x = np.asarray([[o["otos_x"] - ox, o["otos_y"] - oy, 1.0] for o in fit_obs], dtype=float)
    y = np.asarray([[float(o["fused_x"]) - ox, float(o["fused_y"]) - oy] for o in fit_obs], dtype=float)
    coef, *_ = np.linalg.lstsq(x, y, rcond=None)
    params = {
        "matrix": [[float(coef[0, 0]), float(coef[1, 0])], [float(coef[0, 1]), float(coef[1, 1])]],
        "translation_m": [float(coef[2, 0]), float(coef[2, 1])],
        "transform_origin_m": [float(ox), float(oy)],
    }
    return _make_otos_fit_model("affine", "Affine XY around start", "affine", params, fit_obs, all_obs)


def _fit_section_offset_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not fit_obs:
        return None
    offsets: dict[str, list[list[float]]] = {}
    for obs in fit_obs:
        offsets.setdefault(str(obs["section"]), []).append([float(obs["fused_x"]) - obs["otos_x"], float(obs["fused_y"]) - obs["otos_y"]])
    params = {"offsets_m": {section: [float(v) for v in np.mean(np.asarray(vals, dtype=float), axis=0)] for section, vals in sorted(offsets.items())}}
    params["offsets_m"].setdefault("default", [0.0, 0.0])
    return _make_otos_fit_model("section_offsets", "Section-specific XY offsets", "section_offsets", params, fit_obs, all_obs)


def _fit_similarity_periodic_otos_model(fit_obs: list[dict[str, Any]], all_obs: list[dict[str, Any]], base_model: dict[str, Any] | None) -> dict[str, Any] | None:
    if base_model is None or len(fit_obs) < 12:
        return None
    progress = np.asarray([float(o["progress_m"]) for o in fit_obs], dtype=float)
    span = float(np.max(progress) - np.min(progress)) if progress.size else 0.0
    if span <= 1e-6:
        return None
    period = span / 3.0 if span > 12.0 else span
    if period <= 1e-6:
        return None
    base_params = base_model.get("params", {})
    residuals = []
    features = []
    for obs in fit_obs:
        bx, by = _apply_otos_xy_model("similarity", base_params, obs["otos_x"], obs["otos_y"], obs["section"], obs["progress_m"])
        residuals.append([float(obs["fused_x"]) - bx, float(obs["fused_y"]) - by])
        phi = 2.0 * math.pi * (float(obs["progress_m"]) % period) / period
        features.append([math.sin(phi), math.cos(phi), math.sin(2.0 * phi), math.cos(2.0 * phi)])
    f = np.asarray(features, dtype=float)
    r = np.asarray(residuals, dtype=float)
    coef, *_ = np.linalg.lstsq(f, r, rcond=None)
    params = {
        "base": base_params,
        "period_m": float(period),
        "x_coeffs_m": [float(v) for v in coef[:, 0]],
        "y_coeffs_m": [float(v) for v in coef[:, 1]],
    }
    return _make_otos_fit_model("similarity_periodic", "Similarity + periodic residual", "similarity_periodic", params, fit_obs, all_obs)


def _lidar_estimate_pose_json(value) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    x = _json_number(value.get("x"))
    y = _json_number(value.get("y"))
    h = _json_number(value.get("h"))
    valid = bool(value.get("valid", False)) and x is not None and y is not None and h is not None
    return {
        "valid": valid,
        "x": x,
        "y": y,
        "h": h,
        "confidence": _json_number(value.get("confidence")),
        "residual_m": _json_number(value.get("residual_m")),
        "matched_points": _json_number(value.get("matched_points")),
        "rejected_points": _json_number(value.get("rejected_points")),
        "reason": str(value.get("reason", "")),
        "wall_mode": str(value.get("wall_mode", "")),
    }


def _json_number(value) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _json_safe(value):
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, (float, int)):
        return _json_number(value)
    if isinstance(value, (str, bool)) or value is None:
        return value
    return str(value)


def _points_json(points: np.ndarray, max_points: int) -> list[list[float]]:
    arr = np.asarray(points, dtype=float)
    if arr.size == 0:
        return []
    arr = arr.reshape((-1, 2))
    if arr.shape[0] > max_points:
        step = max(1, int(math.ceil(arr.shape[0] / max_points)))
        arr = arr[::step]
    arr = arr[np.isfinite(arr).all(axis=1)]
    return [[round(float(x), 4), round(float(y), 4)] for x, y in arr]


def make_handler(viewer: DeskewViewer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _send_json(self, payload: dict[str, Any]) -> None:
            data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send_html(self) -> None:
            data = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            try:
                n = int(self.headers.get("Content-Length", "0") or 0)
            except ValueError:
                n = 0
            data = self.rfile.read(max(0, n)) if n > 0 else b"{}"
            try:
                payload = json.loads(data.decode("utf-8") or "{}")
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid json: {exc}") from exc
            if not isinstance(payload, dict):
                raise ValueError("json body must be an object")
            return payload

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path in ("", "/"):
                self._send_html()
                return
            if parsed.path == "/api/summary":
                self._send_json(viewer.summary())
                return
            if parsed.path == "/api/frame":
                qs = urllib.parse.parse_qs(parsed.query)
                idx = int(qs.get("index", ["0"])[0] or 0)
                self._send_json(viewer.frame_payload(idx))
                return
            self.send_error(404)

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            try:
                payload = self._read_json()
                idx = int(payload.get("index", 0) or 0)
                if parsed.path == "/api/model_fit/export":
                    override = payload.get("model") if isinstance(payload.get("model"), dict) else None
                    self._send_json(viewer.export_model_fit(str(payload.get("name", "") or ""), override))
                    return
                if parsed.path == "/api/annotations":
                    self._send_json(viewer.replace_annotations(payload.get("frames", {})))
                    return
                if parsed.path == "/api/annotation":
                    self._send_json(
                        viewer.set_annotation(
                            idx,
                            float(payload.get("dx_m", 0.0) or 0.0),
                            float(payload.get("dy_m", 0.0) or 0.0),
                            float(payload.get("dh_deg", 0.0) or 0.0),
                        )
                    )
                    return
                if parsed.path == "/api/annotation/delete":
                    self._send_json(viewer.delete_annotation(idx))
                    return
            except Exception as exc:
                self.send_error(400, str(exc))
                return
            self.send_error(404)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve an offline LiDAR deskew comparison viewer")
    parser.add_argument("path", help="Diagnostic *_index.json or NPZ chunk path")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8090, help="HTTP bind port")
    parser.add_argument("--seam-window-deg", type=float, default=None, help="Override seam window width; defaults to captured value")
    parser.add_argument("--min-window-points", type=int, default=3, help="Minimum valid seam points per side")
    parser.add_argument("--same-wall-gate-m", type=float, default=0.25, help="Maximum median distance from seam window to an outer wall")
    parser.add_argument("--max-line-mad-m", type=float, default=0.04, help="Maximum same-wall line MAD for a valid line seam")
    parser.add_argument("--max-display-points", type=int, default=1200, help="Maximum points sent per model per frame for drawing")
    parser.add_argument("--annotations", default=None, help="Manual alignment JSON path; defaults next to the diagnostic index")
    parser.add_argument("--model-fit-json", default=None, help="OTOS model fit JSON import/export path; defaults next to the diagnostic index")
    args = parser.parse_args()
    index_path = Path(args.path)
    if not index_path.exists():
        raise SystemExit(f"not found: {index_path}")
    annotation_path = Path(args.annotations) if args.annotations else None
    model_fit_path = Path(args.model_fit_json) if args.model_fit_json else None
    viewer = DeskewViewer(index_path, args.seam_window_deg, args.min_window_points, args.same_wall_gate_m, args.max_line_mad_m, args.max_display_points, annotation_path, model_fit_path)
    if not viewer.frames:
        raise SystemExit("no diagnostic frames loaded")
    handler = make_handler(viewer)
    with ReusableThreadingHTTPServer((args.host, args.port), handler) as server:
        url = f"http://{args.host}:{args.port}/"
        print(f"LiDAR deskew viewer: {url}")
        print(f"Loaded {len(viewer.frames)} frames from {index_path}")
        print(f"Manual annotations: {viewer.annotation_path}")
        print(f"OTOS model fits: {viewer.model_fit_path}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nviewer stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
