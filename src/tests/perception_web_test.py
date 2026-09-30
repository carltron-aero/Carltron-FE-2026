#!/usr/bin/env python3
"""OTOS + LiDAR + camera debug map for the perception core.

This test adapter starts hardware sensors and the web UI, but delegates all
pose estimation, correction, obstacle detection, and color classification to
src/prod/robot_core. DriveBase control is enabled by default and gated by the
configured deadman source.
"""

from __future__ import annotations

import json
import math
import csv
import queue
import sys
import threading
import time
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import numpy as np
import qwiic_otos
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))

from robot_core import LidarScanInput, Pose, RobotPerception, parse_args
from robot_core.perception import local_to_display_pose


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WRO FE Telemetry Map</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101418;
      color: #e7edf3;
    }
    html {
      height: 100%;
      overflow: hidden;
    }
    body {
      margin: 0;
      height: 100%;
      min-height: 0;
      display: grid;
      grid-template-rows: auto minmax(0, 1fr);
      overflow: hidden;
    }
    header {
      display: flex;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      padding: 10px 14px;
      background: #182029;
      border-bottom: 1px solid #2b3641;
      font-size: 14px;
    }
    .title-row {
      display: flex;
      gap: 12px;
      align-items: center;
      min-width: 0;
    }
    .view-controls {
      display: inline-flex;
      gap: 6px;
      align-items: center;
      flex: 0 0 auto;
    }
    .view-controls button {
      border: 1px solid #344555;
      background: #101820;
      color: #c9d5df;
      padding: 4px 8px;
      border-radius: 5px;
      cursor: pointer;
      font: inherit;
    }
    .view-controls button.active {
      border-color: #80e0a7;
      color: #ffffff;
      background: #1d382c;
    }
    .status {
      color: #9fb1c1;
    }
    .top-summary {
      display: flex;
      gap: 8px;
      align-items: center;
      min-width: 0;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    .summary-pill {
      min-width: 0;
      max-width: 620px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      padding: 5px 8px;
      background: #0d1319;
      border: 1px solid #202b35;
      border-radius: 5px;
      color: #9fb1c1;
    }
    .debug-toggle,
    .debug-close {
      border: 1px solid #344555;
      background: #101820;
      color: #c9d5df;
      padding: 5px 9px;
      border-radius: 5px;
      cursor: pointer;
      font: inherit;
    }
    .debug-toggle.active {
      border-color: #80e0a7;
      color: #ffffff;
      background: #1d382c;
    }
    .app-shell {
      min-height: 0;
      height: 100%;
      overflow: hidden;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 380px;
    }
    .app-shell.debug-collapsed {
      grid-template-columns: minmax(0, 1fr) 0;
    }
    .debug-panel {
      min-width: 0;
      min-height: 0;
      height: 100%;
      overflow-y: auto;
      overflow-x: hidden;
      background: #101820;
      border-left: 1px solid #26323d;
      font-size: 13px;
      transition: opacity 120ms ease;
    }
    .app-shell.debug-collapsed .debug-panel {
      opacity: 0;
      pointer-events: none;
      overflow: hidden;
    }
    .debug-panel-header {
      position: sticky;
      top: 0;
      z-index: 2;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
      padding: 10px 12px;
      background: #182029;
      border-bottom: 1px solid #2b3641;
    }
    .debug-content {
      display: grid;
      gap: 10px;
      padding: 10px;
    }
    .debug-section {
      border: 1px solid #202b35;
      background: #0d1319;
      border-radius: 6px;
      overflow: hidden;
    }
    .debug-section h2 {
      margin: 0;
      padding: 7px 9px;
      background: #141d26;
      border-bottom: 1px solid #202b35;
      color: #d9e4ed;
      font-size: 12px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }
    .kv-grid {
      display: grid;
      gap: 1px;
      background: #17212b;
    }
    .kv-row {
      display: grid;
      grid-template-columns: minmax(104px, 0.85fr) minmax(0, 1.15fr);
      gap: 8px;
      padding: 5px 8px;
      background: #0d1319;
    }
    .kv-label {
      color: #7f91a0;
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .kv-value {
      color: #d9e4ed;
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .kv-row.ok .kv-value {
      color: #80e0a7;
    }
    .kv-row.warn .kv-value {
      color: #ffca75;
    }
    .speed-meter {
      padding: 8px;
      background: #0d1319;
      border-bottom: 1px solid #202b35;
    }
    .speed-meter-header {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      margin-bottom: 6px;
      color: #d9e4ed;
      font-size: 12px;
      font-weight: 700;
    }
    .speed-meter-value {
      color: #80e0a7;
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }
    .speed-meter-track {
      height: 12px;
      overflow: hidden;
      border: 1px solid #26323d;
      border-radius: 6px;
      background: #17212b;
    }
    .speed-meter-fill {
      width: 0%;
      height: 100%;
      background: linear-gradient(90deg, #80e0a7 0%, #ffca75 70%, #ff6b6b 100%);
      transition: width 80ms linear;
    }
    .speed-meter-scale {
      display: flex;
      justify-content: space-between;
      margin-top: 4px;
      color: #7f91a0;
      font-size: 10px;
      font-variant-numeric: tabular-nums;
    }
    .timing-table {
      display: grid;
      gap: 1px;
      background: #17212b;
    }
    .timing-row {
      min-width: 0;
      display: grid;
      grid-template-columns: 14px minmax(0, 1fr) auto;
      gap: 7px;
      align-items: center;
      padding: 6px 8px;
      background: #0d1319;
    }
    .timing-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #475766;
    }
    .timing-name {
      color: #d9e4ed;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .timing-time {
      color: #d9e4ed;
      white-space: nowrap;
    }
    .timing-row.done {
      background: #102018;
    }
    .timing-row.done .timing-dot {
      background: #80e0a7;
    }
    .timing-row.current {
      background: #2a2110;
    }
    .timing-row.current .timing-dot {
      background: #ffca75;
    }
    .timing-row.pending .timing-time,
    .timing-row.pending .timing-name {
      color: #7f91a0;
    }
    .timing-row.ranking {
      border-top: 1px solid #80e0a7;
    }
    .timing-row.ranking .timing-name,
    .timing-row.ranking .timing-time {
      font-weight: 700;
    }
    .ok {
      color: #80e0a7;
    }
    .warn {
      color: #ffca75;
    }
    main {
      min-height: 0;
      height: 100%;
      overflow: hidden;
      display: grid;
      grid-template-columns: minmax(0, 1fr);
      gap: 12px;
      align-items: center;
      justify-items: center;
      padding: 12px;
    }
    main.both {
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    }
    .panel {
      min-width: 0;
      min-height: 0;
      width: 100%;
      height: 100%;
      display: grid;
      place-items: center;
    }
    .hidden {
      display: none !important;
    }
    canvas {
      width: min(100%, 100%);
      max-width: calc(100vh - 76px);
      max-height: calc(100vh - 76px);
      aspect-ratio: 1 / 1;
      height: auto;
      background: #0c1014;
      border: 1px solid #2b3641;
    }
    main.both canvas {
      width: min(100%, 100%);
      height: auto;
    }
    #cameraFrame {
      width: min(100%, 100%);
      max-width: calc(100vh - 76px);
      max-height: calc(100vh - 76px);
      object-fit: contain;
      background: #0c1014;
      border: 1px solid #2b3641;
    }
    main.both #cameraFrame {
      width: min(100%, 100%);
    }
    #cameraPanel .placeholder {
      color: #9fb1c1;
      border: 1px solid #2b3641;
      background: #0c1014;
      width: min(100%, 100%);
      max-width: calc(100vh - 76px);
      max-height: calc(100vh - 76px);
      aspect-ratio: 1 / 1;
      display: grid;
      place-items: center;
      text-align: center;
      padding: 16px;
      box-sizing: border-box;
    }
    @media (max-width: 980px) {
      header {
        align-items: flex-start;
        flex-direction: column;
      }
      .top-summary {
        justify-content: flex-start;
      }
      .summary-pill {
        max-width: min(96vw, 620px);
      }
      .app-shell {
        grid-template-columns: minmax(0, 1fr);
        grid-template-rows: minmax(0, 1fr) minmax(160px, 42vh);
      }
      .app-shell.debug-collapsed {
        grid-template-rows: minmax(0, 1fr) 0;
      }
      .debug-panel {
        border-left: 0;
        border-top: 1px solid #26323d;
      }
      main.both {
        grid-template-columns: minmax(0, 1fr);
      }
      main.both canvas,
      main.both #cameraFrame {
        width: min(100%, 72vh);
        height: auto;
      }
    }
    @media (max-width: 620px) {
      .kv-row {
        grid-template-columns: minmax(92px, 0.75fr) minmax(0, 1.25fr);
      }
    }
  </style>
</head>
<body>
  <header>
    <div class="title-row">
      <strong>WRO FE Telemetry</strong>
      <span id="conn" class="status">connecting</span>
    </div>
    <span class="view-controls">
      <button id="viewMap" type="button">Map</button>
      <button id="viewCamera" type="button">Camera</button>
      <button id="viewBoth" type="button">Both</button>
      <button id="cameraOverview" type="button">Cam All</button>
      <button id="cameraExact" type="button">Cam Exact</button>
    </span>
    <span class="top-summary">
      <span id="wKey" class="summary-pill">W: false</span>
      <span id="driveCommand" class="summary-pill">cmd: --</span>
      <button id="debugToggle" class="debug-toggle active" type="button">Debug</button>
    </span>
  </header>
  <div id="appShell" class="app-shell">
    <main id="mainView">
      <section id="mapPanel" class="panel">
        <canvas id="map" width="900" height="900"></canvas>
      </section>
      <section id="cameraPanel" class="panel hidden">
        <img id="cameraFrame" class="hidden" alt="Camera debug frame">
        <div id="cameraPlaceholder" class="placeholder hidden">camera web debug disabled</div>
      </section>
    </main>
    <aside id="debugPanel" class="debug-panel">
      <div class="debug-panel-header">
        <strong>Live Debug</strong>
        <button id="debugClose" class="debug-close" type="button">Close</button>
      </div>
      <div class="debug-content">
        <section class="debug-section">
          <h2>Race Timing</h2>
          <div id="raceTiming" class="timing-table"></div>
        </section>
        <section class="debug-section">
          <h2>Drive</h2>
          <div class="speed-meter" aria-label="OTOS measured forward speed from 0 to 1 meters per second">
            <div class="speed-meter-header">
              <span>OTOS forward speed</span>
              <span id="physicalSpeedValue" class="speed-meter-value">0.000 m/s</span>
            </div>
            <div class="speed-meter-track">
              <div id="physicalSpeedFill" class="speed-meter-fill"></div>
            </div>
            <div class="speed-meter-scale">
              <span>0</span>
              <span>0.5</span>
              <span>1.0 m/s</span>
            </div>
          </div>
          <div id="driveDetails" class="kv-grid"></div>
        </section>
        <section class="debug-section"><h2>Pose</h2><div id="poseDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>LiDAR / Correction</h2><div id="lidarDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Navigation</h2><div id="navigationDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Obstacles / Color</h2><div id="obstacleDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Startup</h2><div id="startupDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Unpark</h2><div id="unparkDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Parking</h2><div id="parkingDetails" class="kv-grid"></div></section>
        <section class="debug-section"><h2>Runtime</h2><div id="runtimeDetails" class="kv-grid"></div></section>
      </div>
    </aside>
  </div>
  <script>
    const canvas = document.getElementById("map");
    const ctx = canvas.getContext("2d");
    const mainView = document.getElementById("mainView");
    const mapPanel = document.getElementById("mapPanel");
    const cameraPanel = document.getElementById("cameraPanel");
    const cameraFrame = document.getElementById("cameraFrame");
    const cameraPlaceholder = document.getElementById("cameraPlaceholder");
    const viewButtons = {
      map: document.getElementById("viewMap"),
      camera: document.getElementById("viewCamera"),
      both: document.getElementById("viewBoth"),
    };
    const cameraModeButtons = {
      overview: document.getElementById("cameraOverview"),
      exact: document.getElementById("cameraExact"),
    };
    const conn = document.getElementById("conn");
    const wKeyEl = document.getElementById("wKey");
    const driveCommandEl = document.getElementById("driveCommand");
    const debugToggle = document.getElementById("debugToggle");
    const debugClose = document.getElementById("debugClose");
    const appShell = document.getElementById("appShell");
    const raceTimingEl = document.getElementById("raceTiming");
    const physicalSpeedValueEl = document.getElementById("physicalSpeedValue");
    const physicalSpeedFillEl = document.getElementById("physicalSpeedFill");
    const driveDetailsEl = document.getElementById("driveDetails");
    const poseDetailsEl = document.getElementById("poseDetails");
    const lidarDetailsEl = document.getElementById("lidarDetails");
    const navigationDetailsEl = document.getElementById("navigationDetails");
    const obstacleDetailsEl = document.getElementById("obstacleDetails");
    const startupDetailsEl = document.getElementById("startupDetails");
    const unparkDetailsEl = document.getElementById("unparkDetails");
    const parkingDetailsEl = document.getElementById("parkingDetails");
    const runtimeDetailsEl = document.getElementById("runtimeDetails");

    let latest = null;
    let trail = [];
    let anchorVersion = null;
    let viewMode = "map";
    let cameraMode = "overview";
    let cameraDebugEnabled = false;
    let cameraRefreshTimer = null;
    let debugCollapsed = false;
    const deadmanClientId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
    const deadmanKeysDown = new Set();
    let deadmanLastSent = "";
    let deadmanLastPostT = 0;
    let deadmanPostInFlight = false;

    function isDeadmanKey(event) {
      const key = event.key ? event.key.toLowerCase() : "";
      return event.code === "KeyW" || key === "w";
    }

    function deadmanKeyId(event) {
      return event.code || (event.key ? event.key.toLowerCase() : "");
    }

    function deadmanActive() {
      return deadmanKeysDown.size > 0;
    }

    function updateWKeyFlag() {
      const active = deadmanActive();
      wKeyEl.textContent = `W: ${active ? "true" : "false"}`;
      wKeyEl.className = `summary-pill ${active ? "ok" : "warn"}`;
    }

    function setDebugCollapsed(collapsed) {
      debugCollapsed = collapsed;
      appShell.classList.toggle("debug-collapsed", collapsed);
      debugToggle.classList.toggle("active", !collapsed);
      debugToggle.textContent = collapsed ? "Show Debug" : "Hide Debug";
    }

    debugToggle.addEventListener("click", () => setDebugCollapsed(!debugCollapsed));
    debugClose.addEventListener("click", () => setDebugCollapsed(true));

    function escapeHtml(value) {
      return String(value)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;");
    }

    function fmt(value, digits = 2, suffix = "") {
      return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
    }

    function fmtMs(value) {
      return Number.isFinite(value) ? `${value.toFixed(0)}ms` : "--";
    }

    function fmtM(value, digits = 3) {
      return Number.isFinite(value) ? `${value.toFixed(digits)}m` : "--";
    }

    function fmtBool(value) {
      return value ? "true" : "false";
    }

    function rowValue(value) {
      if (value === null || value === undefined || value === "") return "--";
      return value;
    }

    function setRows(el, rows) {
      el.innerHTML = rows.map(([label, value, cls]) => {
        const rowClass = cls ? ` ${cls}` : "";
        return `<div class="kv-row${rowClass}"><span class="kv-label">${escapeHtml(label)}</span><span class="kv-value">${escapeHtml(rowValue(value))}</span></div>`;
      }).join("");
    }

    function updatePhysicalSpeedMeter(otos) {
      const headingRad = Number.isFinite(otos.h) ? otos.h * Math.PI / 180 : NaN;
      const forwardMps = Number.isFinite(otos.vx) && Number.isFinite(otos.vy) && Number.isFinite(headingRad)
        ? otos.vx * Math.cos(headingRad) + otos.vy * Math.sin(headingRad)
        : NaN;
      const absMps = Number.isFinite(forwardMps) ? Math.abs(forwardMps) : 0;
      const fraction = Math.max(0, Math.min(1, absMps));
      if (physicalSpeedValueEl) {
        physicalSpeedValueEl.textContent = Number.isFinite(forwardMps) ? `${forwardMps.toFixed(3)} m/s` : "-- m/s";
      }
      if (physicalSpeedFillEl) {
        physicalSpeedFillEl.style.width = `${(fraction * 100).toFixed(1)}%`;
      }
    }

    function deadmanPayload(forceInactive = false) {
      return {
        client_id: deadmanClientId,
        active: forceInactive ? false : deadmanActive(),
        visible: !document.hidden,
        focused: document.hasFocus(),
        keys: Array.from(deadmanKeysDown),
      };
    }

    function postDeadman(force = false, forceInactive = false) {
      const payload = deadmanPayload(forceInactive);
      const encoded = JSON.stringify(payload);
      const now = performance.now();
      if (!force && !payload.active && encoded === deadmanLastSent && now - deadmanLastPostT < 70) return;
      if (deadmanPostInFlight && !forceInactive) return;
      deadmanLastSent = encoded;
      deadmanLastPostT = now;
      deadmanPostInFlight = true;
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 250);
      fetch("/deadman", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: encoded,
        signal: controller.signal,
      }).catch(() => {}).finally(() => {
        clearTimeout(timeout);
        deadmanPostInFlight = false;
      });
    }

    function beaconDeadmanInactive() {
      const body = new Blob([JSON.stringify(deadmanPayload(true))], {type: "application/json"});
      if (navigator.sendBeacon) {
        navigator.sendBeacon("/deadman", body);
      } else {
        postDeadman(true, true);
      }
    }

    window.addEventListener("keydown", (event) => {
      if (isDeadmanKey(event)) {
        event.preventDefault();
        deadmanKeysDown.add(deadmanKeyId(event));
        updateWKeyFlag();
        postDeadman(true);
      }
    });
    window.addEventListener("keyup", (event) => {
      if (isDeadmanKey(event)) {
        event.preventDefault();
        deadmanKeysDown.delete(deadmanKeyId(event));
        updateWKeyFlag();
        postDeadman(true, deadmanKeysDown.size === 0);
      }
    });
    window.addEventListener("blur", () => {
      deadmanKeysDown.clear();
      updateWKeyFlag();
      postDeadman(true, true);
    });
    window.addEventListener("focus", () => {
      postDeadman(true);
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) deadmanKeysDown.clear();
      updateWKeyFlag();
      postDeadman(true, document.hidden);
    });
    window.addEventListener("pagehide", beaconDeadmanInactive);
    window.addEventListener("beforeunload", beaconDeadmanInactive);
    setInterval(() => {
      updateWKeyFlag();
      postDeadman(deadmanActive());
    }, 50);
    let cameraRefreshBusy = false;
    let parkingConfig = {
      enabled: true,
      centerX: 0.0,
      gap: 1.5 * 0.28,
      length: 0.20,
    };

    function setViewMode(mode) {
      viewMode = mode;
      for (const [key, button] of Object.entries(viewButtons)) {
        button.classList.toggle("active", key === mode);
      }
      mainView.classList.toggle("both", mode === "both");
      mapPanel.classList.toggle("hidden", mode === "camera");
      cameraPanel.classList.toggle("hidden", mode === "map");
      updateCameraRefresh();
    }

    function setCameraMode(mode) {
      cameraMode = mode === "exact" ? "exact" : "overview";
      for (const [key, button] of Object.entries(cameraModeButtons)) {
        button.classList.toggle("active", key === cameraMode);
      }
      cameraRefreshBusy = false;
      cameraFrame.src = "";
      updateCameraRefresh();
    }

    function updateCameraRefresh() {
      const shouldRun = cameraDebugEnabled && viewMode !== "map";
      cameraFrame.classList.toggle("hidden", !cameraDebugEnabled);
      cameraPlaceholder.classList.toggle("hidden", cameraDebugEnabled);
      if (!shouldRun) {
        if (cameraRefreshTimer !== null) {
          clearTimeout(cameraRefreshTimer);
          cameraRefreshTimer = null;
        }
        cameraRefreshBusy = false;
        return;
      }
      const fps = latest && latest.config && latest.config.camera_web_debug_fps ? latest.config.camera_web_debug_fps : 4;
      const intervalMs = Math.max(120, Math.round(1000 / fps));
      if (cameraRefreshTimer !== null) return;
      const schedule = () => {
        if (!cameraDebugEnabled || viewMode === "map") {
          cameraRefreshTimer = null;
          cameraRefreshBusy = false;
          return;
        }
        cameraRefreshTimer = setTimeout(refresh, intervalMs);
      };
      const refresh = () => {
        cameraRefreshTimer = null;
        if (!cameraDebugEnabled || viewMode === "map" || cameraRefreshBusy) {
          schedule();
          return;
        }
        cameraRefreshBusy = true;
        cameraFrame.src = `/camera.jpg?mode=${cameraMode}&t=${Date.now()}`;
      };
      cameraFrame.onload = () => {
        cameraRefreshBusy = false;
        schedule();
      };
      cameraFrame.onerror = () => {
        cameraRefreshBusy = false;
        schedule();
      };
      refresh();
    }

    viewButtons.map.addEventListener("click", () => setViewMode("map"));
    viewButtons.camera.addEventListener("click", () => setViewMode("camera"));
    viewButtons.both.addEventListener("click", () => setViewMode("both"));
    cameraModeButtons.overview.addEventListener("click", () => setCameraMode("overview"));
    cameraModeButtons.exact.addEventListener("click", () => setCameraMode("exact"));
    setViewMode("map");
    setCameraMode("overview");

    function worldToCanvas(x, y) {
      const pad = 70;
      const field = 3.0;
      const scale = (canvas.width - 2 * pad) / field;
      const cx = pad + (x + field / 2) * scale;
      const cy = canvas.height - (pad + (y + field / 2) * scale);
      return [cx, cy];
    }

    function rectWorld(x1, y1, x2, y2) {
      const [cx1, cy1] = worldToCanvas(x1, y1);
      const [cx2, cy2] = worldToCanvas(x2, y2);
      return [Math.min(cx1, cx2), Math.min(cy1, cy2), Math.abs(cx2 - cx1), Math.abs(cy2 - cy1)];
    }

    function drawGrid() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#111820";
      const [bgx, bgy, bgw, bgh] = rectWorld(-1.55, -1.55, 1.55, 1.55);
      ctx.fillRect(bgx, bgy, bgw, bgh);

      ctx.lineWidth = 1;
      ctx.strokeStyle = "#24303b";
      ctx.fillStyle = "#9fb1c1";
      ctx.font = "12px system-ui";
      for (let i = -15; i <= 15; i++) {
        const v = i / 10;
        const [x1, y1] = worldToCanvas(v, -1.5);
        const [x2, y2] = worldToCanvas(v, 1.5);
        const [x3, y3] = worldToCanvas(-1.5, v);
        const [x4, y4] = worldToCanvas(1.5, v);
        ctx.beginPath();
        ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
        ctx.moveTo(x3, y3); ctx.lineTo(x4, y4);
        ctx.stroke();
      }
      const [fx1, fy1] = worldToCanvas(-1.5, -1.5);
      const [fx2, fy2] = worldToCanvas(1.5, 1.5);
      ctx.strokeStyle = "#d84fc7";
      ctx.lineWidth = 10;
      ctx.strokeRect(fx1, fy2, fx2 - fx1, fy1 - fy2);

      const [ix, iy, iw, ih] = rectWorld(-0.5, -0.5, 0.5, 0.5);
      ctx.strokeStyle = "#d84fc7";
      ctx.lineWidth = 8;
      ctx.strokeRect(ix, iy, iw, ih);

      ctx.strokeStyle = "#475766";
      ctx.lineWidth = 2;
      for (const v of [-0.5, 0.5]) {
        let [a1x, a1y] = worldToCanvas(v, -1.5);
        let [a2x, a2y] = worldToCanvas(v, -0.5);
        ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(v, 0.5);
        [a2x, a2y] = worldToCanvas(v, 1.5);
        ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(-1.5, v);
        [a2x, a2y] = worldToCanvas(-0.5, v);
        ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(0.5, v);
        [a2x, a2y] = worldToCanvas(1.5, v);
        ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
      }

      ctx.fillStyle = "rgba(128, 224, 167, 0.13)";
      const [sx, sy, sw, sh] = rectWorld(-0.25, -1.15, 0.25, -0.85);
      ctx.fillRect(sx, sy, sw, sh);
      ctx.strokeStyle = "#80e0a7";
      ctx.lineWidth = 2;
      ctx.strokeRect(sx, sy, sw, sh);

      if (parkingConfig.enabled) {
        const py0 = -1.5;
        const py1 = -1.5 + parkingConfig.length;
        ctx.strokeStyle = "#ff4fc3";
        ctx.lineWidth = 6;
        for (const px of [parkingConfig.centerX - parkingConfig.gap / 2, parkingConfig.centerX + parkingConfig.gap / 2]) {
          const [p1x, p1y] = worldToCanvas(px, py0);
          const [p2x, p2y] = worldToCanvas(px, py1);
          ctx.beginPath();
          ctx.moveTo(p1x, p1y);
          ctx.lineTo(p2x, p2y);
          ctx.stroke();
        }
      }

      const [ox, oy] = worldToCanvas(0, 0);
      ctx.fillStyle = "#9fb1c1";
      ctx.fillText("field center", ox + 8, oy - 8);
      const [stx, sty] = worldToCanvas(0, -1.0);
      ctx.fillText("default start", stx + 8, sty - 8);
    }

    function drawPoints(points, color, size) {
      if (!points) return;
      ctx.fillStyle = color;
      for (const p of points) {
        const [cx, cy] = worldToCanvas(p[0], p[1]);
        ctx.fillRect(cx - size / 2, cy - size / 2, size, size);
      }
    }

    function drawLidarWallFit(wallFit) {
      if (!wallFit || !Array.isArray(wallFit.lines)) return;
      ctx.save();
      ctx.font = "10px system-ui";
      for (const line of wallFit.lines) {
        const x1w = Number(line.x1);
        const y1w = Number(line.y1);
        const x2w = Number(line.x2);
        const y2w = Number(line.y2);
        if (!Number.isFinite(x1w) || !Number.isFinite(y1w) || !Number.isFinite(x2w) || !Number.isFinite(y2w)) continue;
        const [x1, y1] = worldToCanvas(x1w, y1w);
        const [x2, y2] = worldToCanvas(x2w, y2w);
        const used = !!line.used_for_xy;
        const heading = !!line.used_for_heading;
        const authorityRaw = Number(line.distance_authority);
        const authority = Number.isFinite(authorityRaw) ? Math.max(0, Math.min(1, authorityRaw)) : 1;
        const alpha = used ? (0.28 + 0.62 * authority) : 0.35;
        ctx.strokeStyle = heading ? `rgba(255, 178, 72, ${alpha})` : `rgba(255, 132, 60, ${alpha})`;
        ctx.lineWidth = used ? (2 + 2 * authority) : 2;
        ctx.setLineDash(used ? [] : [7, 6]);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        if (used) {
          const mx = 0.5 * (x1 + x2);
          const my = 0.5 * (y1 + y2);
          const residualValue = Number(line.residual_m);
          const headingValue = Number(line.heading_error_deg);
          const distanceValue = Number(line.perpendicular_distance_m);
          const residual = Number.isFinite(residualValue) ? `${residualValue.toFixed(3)}m` : "--";
          const dh = Number.isFinite(headingValue) ? ` ${headingValue.toFixed(2)}deg` : "";
          const dist = Number.isFinite(distanceValue) ? ` d=${distanceValue.toFixed(2)}m` : "";
          const auth = Number.isFinite(authorityRaw) ? ` a=${authority.toFixed(2)}` : "";
          const label = `${line.name} ${residual}${dh}${dist}${auth}`;
          ctx.setLineDash([]);
          ctx.strokeStyle = "rgba(10, 14, 20, 0.88)";
          ctx.fillStyle = "rgba(255, 194, 92, 0.96)";
          ctx.lineWidth = 3;
          ctx.strokeText(label, mx + 5, my - 5);
          ctx.fillText(label, mx + 5, my - 5);
        }
      }
      ctx.restore();
    }

    function drawObstacleSlots(obstacles) {
      if (!obstacles || !obstacles.slots) return;
      const radius = latest.config && latest.config.obstacle_roi_radius_m ? latest.config.obstacle_roi_radius_m : 0.06;
      const [r0x, r0y] = worldToCanvas(0, 0);
      const [r1x, r1y] = worldToCanvas(radius, 0);
      const pr = Math.abs(r1x - r0x);
      const colorFor = (slot, alpha) => {
        const color = slot.current_color || slot.known_color || slot.color || "";
        if (color === "RED") return `rgba(255, 82, 82, ${alpha})`;
        if (color === "GREEN") return `rgba(70, 220, 120, ${alpha})`;
        return `rgba(255, 210, 80, ${alpha})`;
      };
      for (const slot of obstacles.slots) {
        const [cx, cy] = worldToCanvas(slot.x, slot.y);
        ctx.beginPath();
        ctx.arc(cx, cy, pr, 0, Math.PI * 2);
        if (slot.current) {
          ctx.fillStyle = colorFor(slot, 0.48);
          ctx.strokeStyle = colorFor(slot, 0.95);
          ctx.lineWidth = 3;
        } else if (slot.known) {
          ctx.fillStyle = colorFor(slot, 0.16);
          ctx.strokeStyle = colorFor(slot, 0.50);
          ctx.lineWidth = 2;
        } else {
          ctx.fillStyle = "rgba(210, 220, 230, 0.035)";
          ctx.strokeStyle = "rgba(210, 220, 230, 0.16)";
          ctx.lineWidth = 1;
        }
        ctx.fill();
        ctx.stroke();
        if (slot.current || slot.known) {
          const posConf = Number.isFinite(slot.confidence) ? slot.confidence : 0;
          const colorConf = Number.isFinite(slot.known_color_confidence)
            ? slot.known_color_confidence
            : Number.isFinite(slot.raw_color_confidence) ? slot.raw_color_confidence : 0;
          const votes = Number.isFinite(slot.recent_votes) ? slot.recent_votes : 0;
          const dbg = slot.color_debug || {};
          const prop = slot.proposed_color || dbg.proposed_color || "";
          const trusted = slot.color_trusted || dbg.trusted || false;
          const reason = slot.color_trust_reason || dbg.trust_reason || "";
          const label = `p${posConf.toFixed(2)} c${colorConf.toFixed(2)} v${votes}`;
          const colorLabel = prop ? `prop:${prop.slice(0, 1)} ${trusted ? "T" : "U"} ${reason}` : `${trusted ? "T" : "U"} ${reason}`;
          ctx.save();
          ctx.font = "11px system-ui";
          ctx.fillStyle = slot.current ? "rgba(235, 245, 255, 0.96)" : "rgba(185, 198, 210, 0.82)";
          ctx.strokeStyle = "rgba(12, 18, 24, 0.85)";
          ctx.lineWidth = 3;
          ctx.strokeText(label, cx + pr + 4, cy + 4);
          ctx.fillText(label, cx + pr + 4, cy + 4);
          if (reason || prop) {
            ctx.strokeText(colorLabel, cx + pr + 4, cy + 18);
            ctx.fillText(colorLabel, cx + pr + 4, cy + 18);
          }
          ctx.restore();
        }
      }
    }

    function drawNavigation(nav) {
      if (!nav || !nav.enabled) return;
      if (nav.waypoints && nav.waypoints.length > 1) {
        const lockedIds = new Set((nav.locked_gate_choices || []).map((item) => item.waypoint_id).filter(Boolean));
        for (const wp of nav.waypoints) {
          const [wx, wy] = worldToCanvas(wp.x, wp.y);
          const heading = Number.isFinite(wp.heading_deg) ? wp.heading_deg * Math.PI / 180 : 0;
          const isObstacleWp = !!wp.obstacle_id;
          const isActive = wp.id && wp.id === nav.active_waypoint_id;
          const isLocked = wp.id && lockedIds.has(wp.id);
          const headingLen = isActive ? 18 : isObstacleWp ? 14 : wp.kind === "corner" ? 12 : 9;
          ctx.fillStyle = isLocked ? "rgba(255, 136, 0, 0.95)" : isActive ? "rgba(255, 235, 130, 0.95)" : isObstacleWp ? "rgba(135, 175, 255, 0.72)" : "rgba(135, 175, 255, 0.28)";
          ctx.strokeStyle = isLocked ? "rgba(255, 185, 65, 1.0)" : isActive ? "rgba(255, 235, 130, 1.0)" : isObstacleWp ? "rgba(135, 175, 255, 0.95)" : "rgba(135, 175, 255, 0.42)";
          ctx.lineWidth = isActive ? 3 : isObstacleWp ? 2 : 1;
          ctx.beginPath();
          ctx.arc(wx, wy, isLocked ? 7 : isActive ? 6 : isObstacleWp ? 4 : 2.5, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(wx, wy);
          ctx.lineTo(wx + Math.cos(heading) * headingLen, wy - Math.sin(heading) * headingLen);
          ctx.stroke();
        }
      }
      if (nav.preview_path && nav.preview_path.length > 1) {
        ctx.save();
        ctx.strokeStyle = "rgba(190, 200, 215, 0.38)";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 7]);
        ctx.beginPath();
        for (let i = 0; i < nav.preview_path.length; i++) {
          const [px, py] = worldToCanvas(nav.preview_path[i].x, nav.preview_path[i].y);
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.restore();
      }
      if (nav.corridor_points && nav.corridor_points.origin && nav.corridor_points.left && nav.corridor_points.right) {
        const [lx, ly] = worldToCanvas(nav.corridor_points.left.x, nav.corridor_points.left.y);
        const [rx, ry] = worldToCanvas(nav.corridor_points.right.x, nav.corridor_points.right.y);
        ctx.save();
        ctx.strokeStyle = "rgba(94, 211, 255, 0.34)";
        ctx.fillStyle = "rgba(94, 211, 255, 0.07)";
        ctx.lineWidth = 2;
        if (nav.corridor_points.front_left && nav.corridor_points.front_right) {
          const [flx, fly] = worldToCanvas(nav.corridor_points.front_left.x, nav.corridor_points.front_left.y);
          const [frx, fry] = worldToCanvas(nav.corridor_points.front_right.x, nav.corridor_points.front_right.y);
          ctx.beginPath();
          ctx.moveTo(lx, ly);
          ctx.lineTo(flx, fly);
          ctx.lineTo(frx, fry);
          ctx.lineTo(rx, ry);
          ctx.closePath();
          ctx.fill();
          ctx.stroke();
        } else {
          const [ox, oy] = worldToCanvas(nav.corridor_points.origin.x, nav.corridor_points.origin.y);
          ctx.beginPath();
          ctx.moveTo(ox, oy);
          ctx.lineTo(lx, ly);
          ctx.lineTo(rx, ry);
          ctx.closePath();
          ctx.fill();
          ctx.beginPath();
          ctx.moveTo(ox, oy);
          ctx.lineTo(lx, ly);
          ctx.moveTo(ox, oy);
          ctx.lineTo(rx, ry);
          ctx.stroke();
        }
        ctx.restore();
      }
      if (nav.path && nav.path.length > 1) {
        ctx.strokeStyle = nav.pose_confidence_ok ? "rgba(180, 135, 255, 0.95)" : "rgba(255, 202, 117, 0.90)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        for (let i = 0; i < nav.path.length; i++) {
          const [cx, cy] = worldToCanvas(nav.path[i].x, nav.path[i].y);
          if (i === 0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        }
        ctx.stroke();
      }
      if (nav.lookahead && Number.isFinite(nav.lookahead.x) && Number.isFinite(nav.lookahead.y)) {
        const [lx, ly] = worldToCanvas(nav.lookahead.x, nav.lookahead.y);
        ctx.fillStyle = "#d7bcff";
        ctx.strokeStyle = "#5b3b95";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(lx, ly, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      if (nav.obstacle_decisions) {
        ctx.font = "12px system-ui";
        for (const decision of nav.obstacle_decisions) {
          const [sx, sy] = worldToCanvas(decision.x, decision.y);
          const color = decision.color === "RED" ? "#ff6b6b" : decision.color === "GREEN" ? "#61e58d" : "#ffdf7a";
          ctx.strokeStyle = color;
          ctx.fillStyle = color;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(sx, sy, 10, 0, Math.PI * 2);
          ctx.stroke();
          ctx.fillText(`${decision.reason} ${(decision.delta_m).toFixed(2)}m`, sx + 10, sy - 10);
        }
      }
      const connector = nav.local_connector_mode ? ` ${nav.local_connector_mode}` : "";
      const warnings = Array.isArray(nav.warnings) && nav.warnings.length ? ` ${nav.warnings.slice(0, 3).join(",")}` : "";
      const blocked = nav.blocked ? " BLOCKED" : "";
      const hold = nav.hold ? ` HOLD ${nav.hold_section || ""} ${Number.isFinite(nav.hold_remaining_s) ? nav.hold_remaining_s.toFixed(1) + "s" : ""}` : "";
      const adaptiveScore = nav.adaptive_mode === "corridor" ? nav.corridor_speed_scale : nav.straightness_score;
      const adaptiveLabel = nav.adaptive_mode === "corridor" ? "cs" : "st";
      const limited = nav.corridor_rate_limited ? "*" : "";
      const curveDistance = Number.isFinite(nav.curvature_preview_distance_m) ? nav.curvature_preview_distance_m : nav.corridor_corner_distance_m;
      const corner = (nav.speed_reason === "curvature_preview" || nav.speed_reason === "corridor_corner") && Number.isFinite(curveDistance) ? ` curve=${curveDistance.toFixed(2)}` : "";
      const adaptive = Number.isFinite(nav.lookahead_distance_m) ? ` lh=${nav.lookahead_distance_m.toFixed(2)}${limited} ${adaptiveLabel}=${Number.isFinite(adaptiveScore) ? adaptiveScore.toFixed(2) : "--"}${corner}` : "";
      const label = `${nav.direction} ${nav.section} lane=${nav.lateral_m.toFixed(2)} v<=${nav.speed_cap_mps.toFixed(2)} ${nav.speed_reason}${adaptive}${connector}${blocked}${hold}${warnings}`;
      ctx.fillStyle = nav.blocked ? "#ff7b7b" : nav.hold ? "#ffdf7a" : nav.pose_confidence_ok ? "#d7bcff" : "#ffca75";
      ctx.fillText(label, 16, canvas.height - 18);
    }

    function drawTrail() {
      if (trail.length < 2) return;
      ctx.strokeStyle = "#f2b56b";
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < trail.length; i++) {
        const [cx, cy] = worldToCanvas(trail[i].x, trail[i].y);
        if (i === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      }
      ctx.stroke();
    }

    function drawRobot(pose, stroke, fill, label) {
      const [cx, cy] = worldToCanvas(pose.x, pose.y);
      const h = pose.h * Math.PI / 180;
      const len = latest && latest.config && latest.config.robot_length_m ? latest.config.robot_length_m : 0.1925;
      const wid = latest && latest.config && latest.config.robot_width_m ? latest.config.robot_width_m : 0.128;
      const rear = Math.min(Math.max(
        latest && latest.config && latest.config.robot_rear_overhang_m ? latest.config.robot_rear_overhang_m : 0.0335,
        0.0
      ), len);
      const front = len - rear;
      const corners = [
        [ front,  wid / 2],
        [ front, -wid / 2],
        [-rear, -wid / 2],
        [-rear,  wid / 2],
      ].map(([x, y]) => {
        const wx = pose.x + Math.cos(h) * x - Math.sin(h) * y;
        const wy = pose.y + Math.sin(h) * x + Math.cos(h) * y;
        return worldToCanvas(wx, wy);
      });
      ctx.fillStyle = fill;
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(corners[0][0], corners[0][1]);
      for (let i = 1; i < corners.length; i++) ctx.lineTo(corners[i][0], corners[i][1]);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      const noseX = pose.x + Math.cos(h) * front;
      const noseY = pose.y + Math.sin(h) * front;
      const [nx, ny] = worldToCanvas(noseX, noseY);
      ctx.strokeStyle = stroke;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(nx, ny);
      ctx.stroke();
      ctx.fillStyle = stroke;
      ctx.fillText(label, cx + 8, cy + 14);
    }

    function formatRaceSeconds(value) {
      return Number.isFinite(value) ? `${value.toFixed(2)}s` : "--";
    }

    function updateRaceTiming() {
      const timing = latest && latest.race_timing ? latest.race_timing : {};
      const mode = timing.mode || "--";
      const phase = timing.phase || "--";
      const free = mode === "free";
      const lap3ToStop = free ? timing.free_lap3_to_stationary_s : timing.obstacle_lap3_to_stop_s;
      const total = free ? timing.total_to_free_stationary_s : timing.total_to_parked_s;
      const rows = [
        ["Mode", free ? "Free" : (mode === "obstacle" ? "Obstacle" : "--"), mode === "free" || mode === "obstacle"],
        ["Initialization", timing.initialization_time_s, Number.isFinite(timing.initialization_time_s), false, phase === "start_delay" || phase === "initializing"],
        ["Unpark", free ? null : timing.unpark_time_s, !free && Number.isFinite(timing.unpark_time_s), false, !free && phase === "unpark"],
        ["Lap 1", timing.lap1_time_s, Number.isFinite(timing.lap1_time_s), false, phase === "navigation" && !Number.isFinite(timing.lap1_time_s)],
        ["Lap 2", timing.lap2_time_s, Number.isFinite(timing.lap2_time_s), false, phase === "navigation" && Number.isFinite(timing.lap1_time_s) && !Number.isFinite(timing.lap2_time_s)],
        ["Lap 3 to stop", lap3ToStop, Number.isFinite(lap3ToStop), false, phase === "navigation" && Number.isFinite(timing.lap2_time_s) && !Number.isFinite(lap3ToStop)],
        ["Stop", free ? null : timing.parking_stop_time_s, !free && Number.isFinite(timing.parking_stop_time_s), false, !free && phase === "parking_stop"],
        ["Parking", free ? null : timing.parking_duration_s, !free && Number.isFinite(timing.parking_duration_s), false, !free && phase === "parking"],
        ["Ranking", timing.ranking_time_s, Number.isFinite(timing.ranking_time_s), true],
        ["Total", total, Number.isFinite(total)],
      ];
      raceTimingEl.innerHTML = rows.map(([label, value, done, rankingRow, current]) => {
        const text = typeof value === "string" ? value : formatRaceSeconds(value);
        const state = done ? "done" : current ? "current" : "pending";
        return `<div class="timing-row ${state}${rankingRow ? " ranking" : ""}"><span class="timing-dot"></span><span class="timing-name">${escapeHtml(label)}</span><span class="timing-time">${escapeHtml(text)}</span></div>`;
      }).join("");
    }

    function draw() {
      try {
        drawGrid();
        if (latest) {
          const scan = latest.scan || {};
          drawPoints(scan.rejected_points || scan.points || [], "#405262", 2);
          drawPoints(scan.matched_points || [], "#7fd5ff", 3);
          drawLidarWallFit(latest.lidar_wall_fit);
          drawNavigation(latest.navigation);
          drawObstacleSlots(latest.obstacles);
          if (latest.obstacles) drawPoints(latest.obstacles.current_points || [], "#ffdf5d", 4);
          drawTrail();
          if (latest.otos) drawRobot(latest.otos, "#80e0a7", "rgba(128, 224, 167, 0.25)", "OTOS");
          if (latest.lidar_pose && latest.lidar_pose.valid) {
            drawRobot(latest.lidar_pose, "#61c8ff", "rgba(97, 200, 255, 0.18)", "LiDAR");
          }
        }
      } catch (err) {
        console.error("map draw failed", err);
        ctx.save();
        ctx.setLineDash([]);
        ctx.fillStyle = "#ffca75";
        ctx.fillText("map draw error", 16, 24);
        ctx.restore();
      } finally {
        requestAnimationFrame(draw);
      }
    }
    draw();

    const events = new EventSource("/events");
    events.onopen = () => {
      conn.textContent = "connected";
      conn.className = "ok";
    };
    events.onerror = () => {
      conn.textContent = "disconnected";
      conn.className = "warn";
    };
    events.onmessage = (event) => {
      latest = JSON.parse(event.data);
      if (latest.config) {
        parkingConfig = {
          enabled: !!latest.config.parking_enabled,
          centerX: latest.config.parking_center_x,
          gap: latest.config.parking_gap_m,
          length: latest.config.parking_wall_length_m,
        };
        cameraDebugEnabled = !!latest.config.camera_web_debug;
        updateCameraRefresh();
      }
      if (latest.anchor && latest.anchor.version !== anchorVersion) {
        const previousVersion = anchorVersion;
        anchorVersion = latest.anchor.version;
        const correctionAnchor = latest.anchor.source === "lidar_drift_correction" || latest.anchor.source === "lidar_axis_correction";
        if (previousVersion === null || !correctionAnchor) {
          trail = [];
        }
      }
      const o = latest.otos;
      updatePhysicalSpeedMeter(o);
      trail.push({x: o.x, y: o.y});
      if (trail.length > 1200) trail.shift();
      updateRaceTiming();
      const lp = latest.lidar_pose;
      const init = latest.initial_localization || {};
      const corr = latest.lidar_correction || {};
      const cloud = latest.lidar_cloud_wall_metrics || {};
      const cmd = latest.drive_command || {};
      const dm = cmd.deadman || {};
      const dmAge = Number.isFinite(dm.age_ms) ? `${dm.age_ms.toFixed(0)}ms` : "--";
      const dmState = `dm=${dm.active ? "true" : "false"} fresh=${dm.fresh ? "true" : "false"} clients=${dm.active_clients || 0}/${dm.client_count || 0} last=${dm.last_post_active ? "true" : "false"}`;
      if (cmd.enabled && cmd.reason === "ok" && cmd.deadman_ok) {
        driveCommandEl.textContent = `cmd: ready v=${cmd.target_speed_mps.toFixed(3)}m/s steer=${cmd.steering_deg.toFixed(1)}deg age=${dmAge} ${dmState}`;
        driveCommandEl.className = "summary-pill ok";
      } else if (cmd.enabled) {
        const v = Number.isFinite(cmd.target_speed_mps) ? cmd.target_speed_mps.toFixed(3) : "0.000";
        const st = Number.isFinite(cmd.steering_deg) ? cmd.steering_deg.toFixed(1) : "0.0";
        const brake = cmd.brake ? " brake" : "";
        driveCommandEl.textContent = `cmd: hold v=${v}m/s steer=${st}deg ${cmd.reason || "--"} age=${dmAge} ${dmState}${brake}`;
        driveCommandEl.className = "summary-pill warn";
      } else {
        driveCommandEl.textContent = "cmd: disabled";
        driveCommandEl.className = "summary-pill";
      }
      const nav = latest.navigation || {};
      const targetV = Number.isFinite(cmd.target_speed_mps) ? cmd.target_speed_mps.toFixed(3) : "0.000";
      const rawV = Number.isFinite(cmd.raw_target_speed_mps) ? cmd.raw_target_speed_mps.toFixed(3) : "0.000";
      const ctrlSt = Number.isFinite(cmd.controller_steering_deg) ? cmd.controller_steering_deg.toFixed(1) : "--";
      const dbSt = Number.isFinite(cmd.steering_deg) ? cmd.steering_deg.toFixed(1) : "--";
      const rawSt = Number.isFinite(cmd.raw_steering_deg) ? cmd.raw_steering_deg.toFixed(1) : "--";
      const scale = Number.isFinite(cmd.steering_output_scale) ? cmd.steering_output_scale.toFixed(2) : "--";
      const db = latest.drivebase || {};
      const act = latest.drive_actuation || {};
      const dbV = Number.isFinite(db.measured_mps) ? db.measured_mps.toFixed(3) : "--";
      const dbRamp = Number.isFinite(db.target_ramped_mps) ? db.target_ramped_mps.toFixed(3) : "--";
      const dbDuty = Number.isFinite(db.last_duty) ? db.last_duty.toFixed(3) : "--";
      const dbAge = Number.isFinite(db.age_ms) ? `${db.age_ms.toFixed(0)}ms` : "--";
      const lookahead = Number.isFinite(nav.lookahead_distance_m) ? nav.lookahead_distance_m.toFixed(2) : "--";
      const adaptiveScore = nav.adaptive_mode === "corridor" ? nav.corridor_speed_scale : nav.straightness_score;
      const straight = Number.isFinite(adaptiveScore) ? adaptiveScore.toFixed(2) : "--";
      const yawFactor = Number.isFinite(nav.adaptive_yaw_rate_factor) ? nav.adaptive_yaw_rate_factor.toFixed(2) : "--";
      const yawAvg = Number.isFinite(nav.adaptive_yaw_rate_avg_dps) ? nav.adaptive_yaw_rate_avg_dps.toFixed(0) : "--";
      const straightLabel = nav.adaptive_mode === "corridor" ? "cs" : "st";
      const lhLimited = nav.corridor_rate_limited ? "*" : "";
      const routeErr = Number.isFinite(nav.fixed_route_projection_error_m) ? nav.fixed_route_projection_error_m.toFixed(3) : "--";
      const actFlag = act.allowed ? "act=on" : `act=off:${act.reason || "--"}`;
      const otosAge = o.age_ms === null ? "--" : `${o.age_ms.toFixed(0)}ms`;
      const scanAge = latest.scan.age_ms === null ? "--" : `${latest.scan.age_ms.toFixed(0)}ms`;
      const timing = latest.runtime && latest.runtime.timing ? latest.runtime.timing : {};
      const fmtHz = (key) => timing[key] && Number.isFinite(timing[key].rate_hz) ? timing[key].rate_hz.toFixed(1) : "--";
      const fmtTimingMs = (key) => timing[key] && Number.isFinite(timing[key].duration_ms) ? `${timing[key].duration_ms.toFixed(1)}ms` : "--";
      const perceptionAge = latest.runtime && Number.isFinite(latest.runtime.perception_age_ms) ? `${latest.runtime.perception_age_ms.toFixed(0)}ms` : "--";
      const ts = timing.lidar_timestamp || {};
      const midAge = Number.isFinite(ts.scan_mid_age_ms) ? `${ts.scan_mid_age_ms.toFixed(0)}ms` : "--";
      const colorPending = latest.obstacles && Number.isFinite(latest.obstacles.color_async_pending) ? latest.obstacles.color_async_pending : 0;
      const colorResults = latest.obstacles && Number.isFinite(latest.obstacles.color_async_results) ? latest.obstacles.color_async_results : 0;
      const bottom = (lp && lp.wall_observations || []).find((w) => w.name === "outer_bottom");
      const obs = latest.obstacles || {};
      const navWarnings = Array.isArray(nav.warnings) && nav.warnings.length ? ` warn=${nav.warnings.slice(0, 3).join(",")}` : "";
      const navBlocked = nav.blocked ? " BLOCKED" : "";
      const navHold = nav.hold ? ` HOLD=${nav.hold_section || "--"}:${Number.isFinite(nav.hold_remaining_s) ? nav.hold_remaining_s.toFixed(1) : "--"}s` : "";
      const colorConf = nav.color_confirmation || {};
      const colorConfText = colorConf.slot_id ? ` colorConf=${colorConf.slot_id}:${colorConf.proposal || "--"}:${colorConf.mode || "--"}` : "";
      const absConf = nav.absence_confirmation || {};
      const absConfText = absConf.slot_id ? ` absConf=${absConf.slot_id}:${absConf.mode || "--"}` : "";
      const unc = nav.uncertain_obstacle || {};
      const uncText = unc.id ? ` unc=${unc.id}:${Number.isFinite(unc.confidence) ? unc.confidence.toFixed(2) : "--"}` : "";
      const locks = Array.isArray(nav.locked_gate_choices) ? nav.locked_gate_choices.length : 0;
      const lockText = nav.full_gate_lock ? ` locks=${locks}/12 FULL` : locks ? ` locks=${locks}/12` : "";
      const startup = latest.initial_localization && latest.initial_localization.start_classifier ? latest.initial_localization.start_classifier : {};
      const startupCloseFlags = `${startup.left_close ? "L" : ""}${startup.right_close ? "R" : ""}${startup.front_close ? "F" : ""}` || "-";
      const unpark = latest.unpark || {};
      const unparkClearance = Number.isFinite(unpark.clearance_m) ? `${(unpark.clearance_m * 1000).toFixed(0)}mm` : "--";
      const unparkRearClearance = Number.isFinite(unpark.rear_clearance_m) ? `${(unpark.rear_clearance_m * 1000).toFixed(0)}mm` : "--";
      const unparkHeading = Number.isFinite(unpark.heading_progress_deg) ? `${unpark.heading_progress_deg.toFixed(0)}deg` : "--";
      const unparkPassiveColor = unpark.passive_color_confirmation || {};
      const unparkPassiveCounts = Number.isFinite(unparkPassiveColor.sample_count)
        ? `R${unparkPassiveColor.red_count || 0}/G${unparkPassiveColor.green_count || 0}/O${unparkPassiveColor.other_count || 0}`
        : "--";
      const unparkPassiveEpoch = unparkPassiveColor.stationary_epoch_grace_active ? " grace" : (unparkPassiveColor.stationary_epoch_active ? " epoch" : "");
      const unparkPassiveDecision = unparkPassiveColor.committed_color
        ? `${unparkPassiveColor.committed_slot || "--"}=${unparkPassiveColor.committed_color} conf=${fmt(unparkPassiveColor.committed_confidence, 2)}`
        : `${unparkPassiveColor.reason || "--"}${unparkPassiveEpoch} ${unparkPassiveColor.last_slot || ""} ${unparkPassiveCounts}`.trim();
      const parking = latest.smart_parking || {};
      const parkHeading = Number.isFinite(parking.heading_error_deg) ? `${parking.heading_error_deg.toFixed(0)}deg` : "--";
      const parkDist = Number.isFinite(parking.distance_to_target_m) ? `${parking.distance_to_target_m.toFixed(2)}m` : "--";
      const parkProgress = Number.isFinite(parking.route_progress_m) ? `${parking.route_progress_m.toFixed(2)}m` : "--";
      const navAdaptiveScore = Number.isFinite(nav.adaptive_speed_score)
        ? nav.adaptive_speed_score
        : (nav.adaptive_mode === "corridor" ? nav.corridor_speed_scale : nav.straightness_score);
      const navAdaptiveLabel = nav.adaptive_mode === "corridor" ? "cs" : "st";
      const navLimited = nav.corridor_rate_limited ? "*" : "";
      const adaptiveText = Number.isFinite(nav.lookahead_distance_m) ? ` lh=${nav.lookahead_distance_m.toFixed(2)}${navLimited} ${navAdaptiveLabel}=${Number.isFinite(navAdaptiveScore) ? navAdaptiveScore.toFixed(2) : "--"}` : "";
      const navCurveDistance = Number.isFinite(nav.curvature_preview_distance_m) ? ` curve=${nav.curvature_preview_distance_m.toFixed(2)}` : "";
      const sectionPolicy = nav.speed_policy_current_section
        ? ` cur=${nav.speed_policy_current_section}:${nav.speed_policy_current_section_ready ? "ready" : "wait"}/${nav.speed_policy_current_section_obstacle_color || "--"} next=${nav.speed_policy_next_section || "--"}:${nav.speed_policy_next_section_ready ? "ready" : "wait"}/${nav.speed_policy_next_section_obstacle_color || "--"}`
        : "";
      const speedPolicy = nav.speed_policy ? `${nav.speed_policy} base=${fmt(nav.speed_policy_base_mps, 2)} factor=${fmt(nav.speed_policy_cap_factor, 2)} ${nav.speed_policy_cap_reason || ""}${sectionPolicy}`.trim() : "--";
      const obstacleLookahead = Number.isFinite(nav.obstacle_speed_lookahead_m)
        ? `${fmt(nav.obstacle_speed_lookahead_m, 2)}m v=${fmt(nav.obstacle_speed_lookahead_forward_mps, 2)} scale=${fmt(nav.obstacle_speed_lookahead_scale, 2)}`
        : "--";
      const trackingCap = Number.isFinite(nav.tracking_error_speed_factor)
        ? `${fmt(nav.tracking_error_speed_factor, 2)} ${nav.tracking_error_speed_reason || ""}`.trim()
        : "--";
      const cornerTransitionCap = Number.isFinite(nav.corner_transition_speed_cap_mps)
        ? `${nav.corner_transition_speed_active ? "active " : ""}max=${fmt(nav.corner_transition_speed_cap_mps, 2)}m/s ${nav.corner_transition_speed_reason || ""}`.trim()
        : (Number.isFinite(nav.corner_transition_speed_factor) ? `${fmt(nav.corner_transition_speed_factor, 2)} ${nav.corner_transition_speed_reason || ""}`.trim() : "--");

      setRows(driveDetailsEl, [
        ["Status", cmd.enabled ? (cmd.reason || "--") : "disabled", cmd.enabled && cmd.reason === "ok" ? "ok" : "warn"],
        ["Target / raw", `${targetV} / ${rawV} m/s`],
        ["DriveBase speed", `${dbV} / ${dbRamp} m/s`],
        ["Duty", dbDuty],
        ["Mode", `${db.mode || "--"} / ${db.submode || "--"}`],
        ["Actuation", actFlag],
        ["Brake", cmd.brake ? `${fmt(cmd.brake_level, 2)}` : "false"],
        ["Neutral stop", fmtBool(cmd.neutral_stop)],
        ["Steering", `${dbSt} deg db, ${ctrlSt} deg ctrl, ${rawSt} deg raw`],
        ["Steer scale", scale],
        ["Deadman", `${dmState}, age=${dmAge}`],
        ["Source", cmd.source || "--"],
      ]);
      setRows(poseDetailsEl, [
        ["OTOS", `x=${fmt(o.x, 3)} y=${fmt(o.y, 3)} h=${fmt(o.h, 1)} deg`],
        ["OTOS age", otosAge],
        ["OTOS velocity", `vx=${fmt(o.vx, 3)} vy=${fmt(o.vy, 3)} vh=${fmt(o.vh, 1)} dps`],
        ["Anchor", latest.anchor ? `${latest.anchor.source || "--"} v${latest.anchor.version || "--"}` : "--"],
        ["Init status", init.status || "--"],
        ["Init confidence", fmt(init.confidence, 2)],
        ["Init solve", fmtMs(init.solve_ms)],
      ]);
      setRows(lidarDetailsEl, [
        ["Pose", lp && lp.valid ? `x=${fmt(lp.x, 3)} y=${fmt(lp.y, 3)} h=${fmt(lp.h, 1)} deg` : (lp && lp.reason ? lp.reason : "invalid")],
        ["Confidence", lp && lp.valid ? `${fmt(lp.confidence, 2)} residual=${fmt((lp.residual_m || 0) * 100, 1)}cm` : "--"],
        ["Scan age", scanAge],
        ["Scan mid age", midAge],
        ["Matched / rejected", `${latest.scan.matched_points.length} / ${latest.scan.rejected_points.length}`],
        ["Cloud wall error", cloud.valid ? `med=${fmt(cloud.median_abs_m * 100, 1)}cm p80=${fmt(cloud.p80_abs_m * 100, 1)}cm p95=${fmt(cloud.p95_abs_m * 100, 1)}cm` : (cloud.reason || "--")],
        ["Cloud wall detail", cloud.valid ? `${cloud.dominant_wall || "--"} med=${fmt(cloud.dominant_residual_m * 100, 1)}cm worst=${cloud.worst_wall || "--"}/${fmt(cloud.worst_wall_p80_abs_m * 100, 1)}cm` : "--"],
        ["Cloud ref delta", Number.isFinite(cloud.live_minus_reference_distance_m) ? `d=${fmt(cloud.live_minus_reference_distance_m * 100, 1)}cm dx=${fmt(cloud.live_minus_reference_dx_m * 100, 1)} dy=${fmt(cloud.live_minus_reference_dy_m * 100, 1)} dh=${fmt(cloud.live_minus_reference_dh_deg, 2)}deg` : "--"],
        ["Cloud lag", Number.isFinite(cloud.scan_mid_age_ms) ? `mid=${fmt(cloud.scan_mid_age_ms, 0)}ms end=${fmt(cloud.scan_age_ms, 0)}ms v*mid=${fmt(cloud.speed_x_scan_age_m * 100, 1)}cm` : "--"],
        ["Correction", corr.enabled === false ? "disabled" : `${corr.status || "--"} samples=${corr.samples || 0}`],
        ["Correction step", `dx=${fmt((corr.dx_median || corr.axis_step_x) * 1000, 0)}mm dy=${fmt((corr.dy_median || corr.axis_step_y) * 1000, 0)}mm dh=${fmt(corr.dh_median || corr.axis_step_h, 2)}deg`],
        ["Bottom wall", bottom ? `${bottom.fresh ? "fresh" : "hold"} pts=${bottom.point_count} span=${fmt(bottom.span_m, 2)} frag=${bottom.fragment_count} bridge=${fmt(bottom.bridged_span_m, 2)} mad=${fmt(bottom.mad_m * 1000, 0)}mm conf=${fmt(bottom.confidence, 2)}` : "--"],
      ]);
      setRows(navigationDetailsEl, [
        ["Enabled", fmtBool(nav.enabled)],
        ["Direction / section", `${nav.direction || "--"} / ${nav.section || "--"}`],
        ["Speed cap", `${fmt(nav.speed_cap_mps, 2)} m/s ${nav.speed_reason || ""}`],
        ["Speed policy", speedPolicy],
        ["Speed lookahead", obstacleLookahead],
        ["Tracking cap", trackingCap],
        ["Corner cap", cornerTransitionCap],
        ["Lookahead", `${lookahead}${lhLimited} m ${navAdaptiveLabel}=${straight} yaw=${yawFactor}/${yawAvg}`],
        ["Adaptive", `${adaptiveText}${navCurveDistance}`.trim() || "--"],
        ["Route error", `${routeErr} m`],
        ["Connector", nav.local_connector_mode || "--"],
        ["Active waypoint", `${nav.active_waypoint_id || "--"} ${nav.active_waypoint_reason || ""}`],
        ["Hold", nav.hold ? `${nav.hold_section || "--"} ${fmt(nav.hold_remaining_s, 1)}s` : "false"],
        ["Blocked", nav.blocked ? (nav.blocked_reason || "true") : "false"],
        ["Locks", nav.full_gate_lock ? `${locks}/12 FULL` : `${locks}/12`],
        ["Warnings", navWarnings || "--"],
        ["Confirmations", `${colorConfText || "--"} ${absConfText || ""} ${uncText || ""}`.trim()],
      ]);
      setRows(obstacleDetailsEl, [
        ["Detection", obs.detection_enabled === false ? "off" : "on"],
        ["Race mode", obs.effective_race_mode || "--"],
        ["Current / known", `${obs.current_count || 0} / ${obs.known_count || 0}`],
        ["Raw current", Number.isFinite(obs.raw_current_count) ? obs.raw_current_count : "--"],
        ["Color", obs.color_enabled ? "on" : "off"],
        ["Color queue", `${colorPending} pending / ${colorResults} results`],
      ]);
      setRows(startupDetailsEl, [
        ["Status", init.status || "--"],
        ["Source", init.source || startup.source || "--"],
        ["Reason", init.reason || startup.reason || "--"],
        ["Mode / direction", `${startup.race_mode || init.race_mode || "--"} / ${startup.direction || init.direction || "--"}`],
        ["Side L/R", `${fmt(startup.left_m, 2)} / ${fmt(startup.right_m, 2)} m`],
        ["Front", `${fmt(startup.front_m, 2)} m lateral=${fmt(startup.front_lateral_m, 2)} m`],
        ["Close flags", startupCloseFlags],
        ["Parking ROI", `${Number.isFinite(startup.parking_roi_votes) ? startup.parking_roi_votes : "--"} / ${Number.isFinite(startup.parking_roi_required) ? startup.parking_roi_required : "--"}`],
      ]);
      setRows(unparkDetailsEl, [
        ["State", unpark.state || "--"],
        ["Start mode", unpark.start_mode || "--"],
        ["Direction", unpark.direction || "--"],
        ["Reason", unpark.reason || "--"],
        ["Heading progress", unparkHeading],
        ["Retry", unpark.retry_count || 0],
        ["Front clearance", unparkClearance],
        ["Rear clearance", unparkRearClearance],
        ["Reverse progress", Number.isFinite(unpark.reverse_progress_m) && Number.isFinite(unpark.reverse_target_m) ? `${fmt(unpark.reverse_progress_m * 1000, 0)} / ${fmt(unpark.reverse_target_m * 1000, 0)} mm` : "--"],
        ["Reverse first", Number.isFinite(unpark.reverse_first_progress_m) && Number.isFinite(unpark.reverse_first_target_m) ? `${unpark.reverse_first_active ? "active " : ""}${fmt(unpark.reverse_first_progress_m * 1000, 0)} / ${fmt(unpark.reverse_first_target_m * 1000, 0)} mm ${unpark.reverse_first_stop_reason || ""}`.trim() : "--"],
        ["Reverse commit", Number.isFinite(unpark.reverse_commit_remaining_s) ? `${fmt(unpark.reverse_commit_remaining_s, 1)}s` : "--"],
        ["Direct duty", fmt(unpark.reverse_direct_duty, 2)],
        ["Arm", Number.isFinite(unpark.reverse_arm_speed_mps) ? `${fmt(unpark.reverse_arm_speed_mps * 100, 1)}cm/s stable=${fmt(unpark.reverse_arm_stable_s, 2)} ready=${fmtBool(unpark.reverse_arm_ready)}` : "--"],
        ["Reverse steering", unpark.reverse_steering_mode || "--"],
        ["Passive color", unparkPassiveColor.enabled === false ? "off" : `${unparkPassiveColor.active ? "active " : ""}${unparkPassiveDecision}`.trim()],
        ["Passive samples", `${unparkPassiveCounts} add=${unparkPassiveColor.measurement_events_added || 0} dup=${unparkPassiveColor.measurement_events_duplicate || 0} old=${unparkPassiveColor.measurement_events_before_window || 0} out=${unparkPassiveColor.measurement_events_outside_stationary || 0} stale=${unparkPassiveColor.measurement_events_stale || 0}`],
      ]);
      setRows(parkingDetailsEl, [
        ["State", parking.state || "--"],
        ["Direction", parking.direction || "--"],
        ["Reason", parking.reason || "--"],
        ["Progress", parkProgress],
        ["Heading error", parkHeading],
        ["Target distance", parkDist],
        ["Stage x", fmtM(parking.stage_x, 3)],
        ["Slot center x", fmtM(parking.slot_center_x, 3)],
        ["Final rear y", fmtM(parking.final_rear_y, 3)],
        ["Final push", `${fmt(parking.final_push_elapsed_s, 2)}s duty=${fmt(parking.final_push_direct_duty, 2)} active=${fmtBool(parking.final_push_actuated)}`],
      ]);
      setRows(runtimeDetailsEl, [
        ["OTOS rate", `${fmtHz("otos_update")} Hz`],
        ["LiDAR scan rate", `${fmtHz("lidar_scan_arrival")} Hz`],
        ["LiDAR process rate", `${fmtHz("lidar_process")} Hz`],
        ["Control rate", `${fmtHz("control_update")} Hz`],
        ["Actuation rate", `${fmtHz("drive_actuation")} Hz`],
        ["Perception age", perceptionAge],
        ["LiDAR CPU", fmtTimingMs("lidar_process")],
        ["Pose CPU", fmtTimingMs("lidar_stage_pose_estimate")],
        ["Obstacle CPU", fmtTimingMs("lidar_stage_obstacles")],
        ["Camera queue", `${colorPending} / ${colorResults}`],
      ]);
    };
  </script>
</body>
</html>
"""


LITE_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WRO FE Lite Telemetry</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #101418; color: #e7edf3; }
    html, body { height: 100%; margin: 0; overflow: hidden; }
    body { display: grid; grid-template-rows: auto minmax(0, 1fr); }
    header { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 9px 12px; background: #182029; border-bottom: 1px solid #2b3641; font-size: 14px; }
    .title { display: flex; align-items: center; gap: 10px; min-width: 0; }
    h1 { margin: 0; font-size: 16px; white-space: nowrap; }
    .pill { min-width: 0; max-width: 760px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; padding: 5px 8px; background: #0d1319; border: 1px solid #202b35; border-radius: 5px; color: #9fb1c1; }
    .ok { color: #80e0a7; } .warn { color: #ffca75; } .bad { color: #ff8585; }
    button { border: 1px solid #344555; background: #101820; color: #c9d5df; padding: 5px 9px; border-radius: 5px; cursor: pointer; font: inherit; }
    button.warn { border-color: #785438; color: #ffd8aa; }
    .shell { min-height: 0; height: 100%; display: grid; grid-template-columns: minmax(0, 1fr) 360px; overflow: hidden; }
    .map-pane { min-width: 0; min-height: 0; overflow: hidden; display: grid; place-items: center; padding: 8px; box-sizing: border-box; }
    canvas { display: block; background: #0f151b; border: 1px solid #2b3641; box-sizing: border-box; }
    aside { min-width: 0; overflow-y: auto; background: #101820; border-left: 1px solid #26323d; padding: 9px; font-size: 13px; }
    .meter { height: 10px; border-radius: 5px; background: #26313b; overflow: hidden; margin: 6px 0 10px; }
    .meter > div { height: 100%; width: 0%; background: linear-gradient(90deg, #80e0a7, #ffdf7a); }
    .group { margin-bottom: 10px; border: 1px solid #202b35; border-radius: 5px; overflow: hidden; }
    .group h2 { margin: 0; padding: 6px 8px; background: #17212b; color: #c8d6e2; font-size: 13px; }
    .row { display: grid; grid-template-columns: minmax(115px, 0.7fr) minmax(0, 1.3fr); gap: 8px; padding: 6px 8px; border-top: 1px solid #17212b; }
    .label { color: #7f91a0; }
    .value { color: #d9e4ed; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .timing-table { display: grid; gap: 1px; background: #17212b; }
    .timing-row { min-width: 0; display: grid; grid-template-columns: 14px minmax(0, 1fr) auto; gap: 7px; align-items: center; padding: 6px 8px; background: #0d1319; }
    .timing-dot { width: 8px; height: 8px; border-radius: 50%; background: #475766; }
    .timing-name { color: #d9e4ed; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .timing-time { color: #d9e4ed; white-space: nowrap; }
    .timing-row.done { background: #102018; }
    .timing-row.done .timing-dot { background: #80e0a7; }
    .timing-row.current { background: #2a2110; }
    .timing-row.current .timing-dot { background: #ffca75; }
    .timing-row.pending .timing-time, .timing-row.pending .timing-name { color: #7f91a0; }
    .timing-row.ranking { border-top: 1px solid #80e0a7; }
    .timing-row.ranking .timing-name, .timing-row.ranking .timing-time { font-weight: 700; }
  </style>
</head>
<body>
  <header>
    <div class="title"><h1>Lite Telemetry</h1><span id="conn" class="pill warn">connecting</span></div>
    <div class="title"><span id="summary" class="pill">waiting</span><button id="stopLite" class="warn" type="button">Stop Lite</button></div>
  </header>
  <div class="shell">
    <main id="mapPane" class="map-pane"><canvas id="map" width="900" height="900"></canvas></main>
    <aside>
      <div class="group"><h2>Physical Speed</h2><div class="row"><span class="label">OTOS forward</span><span id="speedText" class="value">--</span></div><div class="meter"><div id="speedFill"></div></div></div>
      <div class="group"><h2>Race Timing</h2><div id="raceRows" class="timing-table"></div></div>
      <div class="group"><h2>Drive</h2><div id="driveRows"></div></div>
      <div class="group"><h2>Pose</h2><div id="poseRows"></div></div>
      <div class="group"><h2>Navigation</h2><div id="navRows"></div></div>
      <div class="group"><h2>LiDAR</h2><div id="lidarRows"></div></div>
      <div class="group"><h2>Obstacles</h2><div id="obstacleRows"></div></div>
      <div class="group"><h2>Timing</h2><div id="timingRows"></div></div>
    </aside>
  </div>
  <script>
    const canvas = document.getElementById("map");
    const ctx = canvas.getContext("2d");
    const mapPane = document.getElementById("mapPane");
    const conn = document.getElementById("conn");
    const summary = document.getElementById("summary");
    const speedFill = document.getElementById("speedFill");
    const speedText = document.getElementById("speedText");
    const groups = {
      race: document.getElementById("raceRows"),
      drive: document.getElementById("driveRows"),
      pose: document.getElementById("poseRows"),
      nav: document.getElementById("navRows"),
      lidar: document.getElementById("lidarRows"),
      obstacles: document.getElementById("obstacleRows"),
      timing: document.getElementById("timingRows"),
    };
    let latest = null;
    let trail = [];
    let anchorVersion = null;
    let parkingConfig = {enabled: true, centerX: 0, gap: 0.42, length: 0.20};
    const deadmanKeys = new Set();
    let deadmanLast = "";
    let deadmanLastPostT = 0;
    let deadmanBusy = false;
    const deadmanId = `lite-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;

    function escapeHtml(value) { return String(value ?? "--").replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;").replaceAll('"', "&quot;"); }
    function fmt(value, digits = 2) { const n = Number(value); return Number.isFinite(n) ? n.toFixed(digits) : "--"; }
    function fmtBool(value) { return value ? "true" : "false"; }
    function fmtMs(value) { return Number.isFinite(Number(value)) ? `${Number(value).toFixed(0)}ms` : "--"; }
    function fmtHz(key) { const t = latest && latest.runtime && latest.runtime.timing ? latest.runtime.timing[key] : null; return t && Number.isFinite(t.rate_hz) ? `${t.rate_hz.toFixed(1)}Hz` : "--"; }
    function fmtTiming(key) { const t = latest && latest.runtime && latest.runtime.timing ? latest.runtime.timing[key] : null; return t && Number.isFinite(t.duration_ms) ? `${t.duration_ms.toFixed(1)}ms` : "--"; }
    function fmtSeconds(value) { return Number.isFinite(Number(value)) ? `${Number(value).toFixed(2)}s` : "--"; }
    function row(label, value, cls = "") { return `<div class="row"><span class="label">${escapeHtml(label)}</span><span class="value ${cls}">${escapeHtml(value)}</span></div>`; }
    function setRows(el, rows) { el.innerHTML = rows.map(([label, value, cls]) => row(label, value, cls || "")).join(""); }
    function finiteValue(value) { return value !== null && value !== undefined && value !== "" && Number.isFinite(Number(value)); }
    function raceSeconds(value) { return finiteValue(value) ? `${Number(value).toFixed(2)}s` : "--"; }
    function setRaceTimingRows(race) {
      const mode = race.mode || "--";
      const phase = race.phase || "--";
      const free = mode === "free";
      const lap3ToStop = free ? race.free_lap3_to_stationary_s : race.obstacle_lap3_to_stop_s;
      const total = free ? race.total_to_free_stationary_s : race.total_to_parked_s;
      const rows = [
        ["Mode", free ? "Free" : (mode === "obstacle" ? "Obstacle" : "--"), mode === "free" || mode === "obstacle"],
        ["Initialization", race.initialization_time_s, finiteValue(race.initialization_time_s), false, phase === "start_delay" || phase === "initializing"],
        ["Unpark", free ? null : race.unpark_time_s, !free && finiteValue(race.unpark_time_s), false, !free && phase === "unpark"],
        ["Lap 1", race.lap1_time_s, finiteValue(race.lap1_time_s), false, phase === "navigation" && !finiteValue(race.lap1_time_s)],
        ["Lap 2", race.lap2_time_s, finiteValue(race.lap2_time_s), false, phase === "navigation" && finiteValue(race.lap1_time_s) && !finiteValue(race.lap2_time_s)],
        ["Lap 3 to stop", lap3ToStop, finiteValue(lap3ToStop), false, phase === "navigation" && finiteValue(race.lap2_time_s) && !finiteValue(lap3ToStop)],
        ["Stop", free ? null : race.parking_stop_time_s, !free && finiteValue(race.parking_stop_time_s), false, !free && phase === "parking_stop"],
        ["Parking", free ? null : race.parking_duration_s, !free && finiteValue(race.parking_duration_s), false, !free && phase === "parking"],
        ["Ranking", race.ranking_time_s, finiteValue(race.ranking_time_s), true],
        ["Total", total, finiteValue(total)],
      ];
      groups.race.innerHTML = rows.map(([label, value, done, rankingRow, current]) => {
        const text = typeof value === "string" ? value : raceSeconds(value);
        const state = done ? "done" : current ? "current" : "pending";
        return `<div class="timing-row ${state}${rankingRow ? " ranking" : ""}"><span class="timing-dot"></span><span class="timing-name">${escapeHtml(label)}</span><span class="timing-time">${escapeHtml(text)}</span></div>`;
      }).join("");
    }

    function resizeCanvas() {
      const rect = mapPane.getBoundingClientRect();
      const style = getComputedStyle(mapPane);
      const padX = parseFloat(style.paddingLeft || "0") + parseFloat(style.paddingRight || "0");
      const padY = parseFloat(style.paddingTop || "0") + parseFloat(style.paddingBottom || "0");
      const side = Math.max(1, Math.floor(Math.min(rect.width - padX, rect.height - padY)));
      canvas.style.width = `${side}px`;
      canvas.style.height = `${side}px`;
      const dpr = Math.max(1, window.devicePixelRatio || 1);
      const nextW = Math.max(1, Math.round(side * dpr));
      const nextH = Math.max(1, Math.round(side * dpr));
      if (canvas.width !== nextW || canvas.height !== nextH) {
        canvas.width = nextW;
        canvas.height = nextH;
      }
    }
    window.addEventListener("resize", resizeCanvas);

    function worldToCanvas(x, y) {
      const pad = 62;
      const field = 3.0;
      const scale = Math.min(canvas.width, canvas.height) - 2 * pad;
      const s = scale / field;
      const cx = (canvas.width - scale) / 2 + (x + field / 2) * s;
      const cy = canvas.height - ((canvas.height - scale) / 2 + (y + field / 2) * s);
      return [cx, cy];
    }
    function rectWorld(x1, y1, x2, y2) {
      const [cx1, cy1] = worldToCanvas(x1, y1); const [cx2, cy2] = worldToCanvas(x2, y2);
      return [Math.min(cx1, cx2), Math.min(cy1, cy2), Math.abs(cx2 - cx1), Math.abs(cy2 - cy1)];
    }
    function drawGrid() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#111820"; ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.lineWidth = 1; ctx.strokeStyle = "#24303b"; ctx.fillStyle = "#9fb1c1"; ctx.font = "12px system-ui";
      for (let i = -15; i <= 15; i++) {
        const v = i / 10;
        let [x1, y1] = worldToCanvas(v, -1.5); let [x2, y2] = worldToCanvas(v, 1.5);
        let [x3, y3] = worldToCanvas(-1.5, v); let [x4, y4] = worldToCanvas(1.5, v);
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.moveTo(x3, y3); ctx.lineTo(x4, y4); ctx.stroke();
      }
      const [fx1, fy1] = worldToCanvas(-1.5, -1.5); const [fx2, fy2] = worldToCanvas(1.5, 1.5);
      ctx.strokeStyle = "#d84fc7"; ctx.lineWidth = 10; ctx.strokeRect(fx1, fy2, fx2 - fx1, fy1 - fy2);
      const [ix, iy, iw, ih] = rectWorld(-0.5, -0.5, 0.5, 0.5); ctx.lineWidth = 8; ctx.strokeRect(ix, iy, iw, ih);
      ctx.strokeStyle = "#475766"; ctx.lineWidth = 2;
      for (const v of [-0.5, 0.5]) {
        let [a1x, a1y] = worldToCanvas(v, -1.5); let [a2x, a2y] = worldToCanvas(v, -0.5); ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(v, 0.5); [a2x, a2y] = worldToCanvas(v, 1.5); ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(-1.5, v); [a2x, a2y] = worldToCanvas(-0.5, v); ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
        [a1x, a1y] = worldToCanvas(0.5, v); [a2x, a2y] = worldToCanvas(1.5, v); ctx.beginPath(); ctx.moveTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.stroke();
      }
      const [sx, sy, sw, sh] = rectWorld(-0.25, -1.15, 0.25, -0.85); ctx.fillStyle = "rgba(128, 224, 167, 0.13)"; ctx.fillRect(sx, sy, sw, sh); ctx.strokeStyle = "#80e0a7"; ctx.lineWidth = 2; ctx.strokeRect(sx, sy, sw, sh);
      if (parkingConfig.enabled) {
        ctx.strokeStyle = "#ff4fc3"; ctx.lineWidth = 6;
        for (const px of [parkingConfig.centerX - parkingConfig.gap / 2, parkingConfig.centerX + parkingConfig.gap / 2]) {
          const [p1x, p1y] = worldToCanvas(px, -1.5); const [p2x, p2y] = worldToCanvas(px, -1.5 + parkingConfig.length);
          ctx.beginPath(); ctx.moveTo(p1x, p1y); ctx.lineTo(p2x, p2y); ctx.stroke();
        }
      }
    }
    function drawPolyline(points, color, width, dash = []) {
      if (!Array.isArray(points) || points.length < 2) return;
      ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.beginPath();
      for (let i = 0; i < points.length; i++) { const [x, y] = worldToCanvas(points[i].x, points[i].y); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }
      ctx.stroke(); ctx.restore();
    }
    function drawWallFit(wallFit) {
      if (!wallFit || !Array.isArray(wallFit.lines)) return;
      ctx.save();
      for (const line of wallFit.lines) {
        const [x1, y1] = worldToCanvas(Number(line.x1), Number(line.y1)); const [x2, y2] = worldToCanvas(Number(line.x2), Number(line.y2));
        const used = !!line.used_for_xy; const heading = !!line.used_for_heading; const a = Number.isFinite(Number(line.distance_authority)) ? Math.max(0.2, Math.min(1, Number(line.distance_authority))) : 0.7;
        ctx.strokeStyle = heading ? `rgba(255, 190, 72, ${a})` : `rgba(255, 132, 60, ${a})`; ctx.lineWidth = used ? 3 : 2; ctx.setLineDash(used ? [] : [7, 6]); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      }
      ctx.restore();
    }
    function drawObstacleSlots(obs) {
      if (!obs || !Array.isArray(obs.slots)) return;
      const radius = latest.config && latest.config.obstacle_roi_radius_m ? latest.config.obstacle_roi_radius_m : 0.06;
      const [r0x] = worldToCanvas(0, 0); const [r1x] = worldToCanvas(radius, 0); const pr = Math.abs(r1x - r0x);
      for (const slot of obs.slots) {
        if (!Number.isFinite(Number(slot.x)) || !Number.isFinite(Number(slot.y))) continue;
        const [cx, cy] = worldToCanvas(Number(slot.x), Number(slot.y));
        const color = slot.current_color || slot.known_color || slot.raw_color || "";
        const fill = color === "RED" ? "rgba(255,82,82,0.42)" : color === "GREEN" ? "rgba(70,220,120,0.42)" : "rgba(255,210,80,0.22)";
        const stroke = color === "RED" ? "#ff6b6b" : color === "GREEN" ? "#61e58d" : "#ffdf7a";
        ctx.beginPath(); ctx.arc(cx, cy, pr, 0, Math.PI * 2); ctx.fillStyle = slot.current || slot.known ? fill : "rgba(210,220,230,0.035)"; ctx.strokeStyle = slot.current || slot.known ? stroke : "rgba(210,220,230,0.16)"; ctx.lineWidth = slot.current ? 3 : slot.known ? 2 : 1; ctx.fill(); ctx.stroke();
      }
    }
    function drawRobot(pose, stroke, fill, label) {
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return;
      const h = Number(pose.h) * Math.PI / 180; const len = latest.config && latest.config.robot_length_m ? latest.config.robot_length_m : 0.1925; const wid = latest.config && latest.config.robot_width_m ? latest.config.robot_width_m : 0.128; const rear = latest.config && latest.config.robot_rear_overhang_m ? latest.config.robot_rear_overhang_m : 0.0335; const front = len - rear;
      const corners = [[front, wid/2], [front, -wid/2], [-rear, -wid/2], [-rear, wid/2]].map(([x, y]) => worldToCanvas(pose.x + Math.cos(h)*x - Math.sin(h)*y, pose.y + Math.sin(h)*x + Math.cos(h)*y));
      const [cx, cy] = worldToCanvas(pose.x, pose.y); const [nx, ny] = worldToCanvas(pose.x + Math.cos(h)*front, pose.y + Math.sin(h)*front);
      ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(corners[0][0], corners[0][1]); for (let i = 1; i < corners.length; i++) ctx.lineTo(corners[i][0], corners[i][1]); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(nx, ny); ctx.stroke(); ctx.fillStyle = stroke; ctx.fillText(label, cx + 8, cy + 14);
    }
    function drawNavigation(nav) {
      if (!nav || !nav.enabled) return;
      drawPolyline(nav.preview_path, "rgba(190,200,215,0.38)", 2, [6, 7]);
      drawPolyline(nav.path, nav.pose_confidence_ok ? "rgba(180,135,255,0.95)" : "rgba(255,202,117,0.90)", 4);
      if (Array.isArray(nav.waypoints)) {
        const locked = new Set((nav.locked_gate_choices || []).map(x => x.waypoint_id).filter(Boolean));
        for (const wp of nav.waypoints) {
          if (!Number.isFinite(Number(wp.x)) || !Number.isFinite(Number(wp.y))) continue;
          const [x, y] = worldToCanvas(Number(wp.x), Number(wp.y)); const active = wp.id && wp.id === nav.active_waypoint_id; const isLocked = wp.id && locked.has(wp.id);
          ctx.fillStyle = isLocked ? "#ff9f35" : active ? "#ffeb82" : wp.obstacle_id ? "rgba(135,175,255,0.72)" : "rgba(135,175,255,0.30)"; ctx.strokeStyle = isLocked ? "#ffc264" : active ? "#ffeb82" : "rgba(135,175,255,0.80)"; ctx.lineWidth = active ? 3 : 1; ctx.beginPath(); ctx.arc(x, y, isLocked ? 6 : active ? 6 : wp.obstacle_id ? 4 : 2.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        }
      }
      if (nav.lookahead && Number.isFinite(Number(nav.lookahead.x)) && Number.isFinite(Number(nav.lookahead.y))) { const [x, y] = worldToCanvas(nav.lookahead.x, nav.lookahead.y); ctx.fillStyle = "#d7bcff"; ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill(); }
    }
    function drawTrail() { drawPolyline(trail, "#f2b56b", 2); }
    function draw() {
      resizeCanvas();
      drawGrid();
      if (latest) {
        drawWallFit(latest.lidar_wall_fit);
        drawNavigation(latest.navigation);
        drawObstacleSlots(latest.obstacles);
        drawTrail();
        drawRobot(latest.otos_only, "#f2b56b", "rgba(242,181,107,0.18)", "OTOS-only");
        drawRobot(latest.otos, "#80e0a7", "rgba(128,224,167,0.25)", "fused");
        if (latest.lidar_pose && latest.lidar_pose.valid) drawRobot(latest.lidar_pose, "#61c8ff", "rgba(97,200,255,0.18)", "LiDAR");
      }
      requestAnimationFrame(draw);
    }
    function updateTables() {
      const o = latest.otos || {}; const oo = latest.otos_only || {}; const lp = latest.lidar_pose || {}; const cmd = latest.drive_command || {}; const db = latest.drivebase || {}; const nav = latest.navigation || {}; const race = latest.race_timing || {}; const obs = latest.obstacles || {}; const scan = latest.scan || {}; const corr = latest.lidar_correction || {}; const delta = latest.pose_delta || {}; const act = latest.drive_actuation || {};
      const forward = Number.isFinite(Number(o.forward_mps)) ? Number(o.forward_mps) : NaN; const speed = Number.isFinite(forward) ? Math.abs(forward) : 0; speedText.textContent = Number.isFinite(forward) ? `${fmt(forward, 3)} m/s` : "-- m/s"; speedFill.style.width = `${Math.max(0, Math.min(100, speed * 100))}%`;
      summary.textContent = `${race.mode || "--"}/${race.phase || "--"} v=${fmt(cmd.target_speed_mps, 2)} ${nav.section || "--"} ${nav.blocked ? "BLOCKED" : nav.hold ? "HOLD" : ""}`;
      setRaceTimingRows(race);
      setRows(groups.drive, [["Target / raw", `${fmt(cmd.target_speed_mps, 3)} / ${fmt(cmd.raw_target_speed_mps, 3)} m/s`], ["OTOS forward", Number.isFinite(forward) ? `${fmt(forward, 3)} m/s` : "--"], ["DriveBase speed", `${fmt(db.measured_mps, 3)} / ${fmt(db.target_ramped_mps, 3)} m/s`], ["Duty", fmt(db.last_duty, 3)], ["Steering", `${fmt(cmd.steering_deg, 1)} deg db / ${fmt(cmd.controller_steering_deg, 1)} deg ctrl`], ["Actuation", act.allowed ? "on" : `off:${act.reason || "--"}`, act.allowed ? "ok" : "warn"], ["Command age", fmtMs(act.command_age_ms)]]);
      setRows(groups.pose, [["Fused", `x=${fmt(o.x, 3)} y=${fmt(o.y, 3)} h=${fmt(o.h, 1)}`], ["OTOS-only", `x=${fmt(oo.x, 3)} y=${fmt(oo.y, 3)} h=${fmt(oo.h, 1)}`], ["Delta", `d=${fmt(delta.fused_minus_otos_distance_m * 100, 1)}cm dh=${fmt(delta.fused_minus_otos_h_deg, 2)}deg`], ["Anchor", latest.anchor ? `${latest.anchor.source || "--"} v${latest.anchor.version || "--"}` : "--"]]);
      setRows(groups.nav, [["Enabled", fmtBool(nav.enabled)], ["Direction / section", `${nav.direction || "--"} / ${nav.section || "--"}`], ["Speed", `${fmt(nav.speed_cap_mps, 2)} ${nav.speed_reason || ""}`], ["Policy", nav.speed_policy || "--"], ["Lookahead", `${fmt(nav.lookahead_distance_m, 2)}m`], ["Connector", nav.local_connector_mode || "--"], ["Hold", nav.hold ? `${nav.hold_section || "--"} ${fmt(nav.hold_remaining_s, 1)}s` : "false"], ["Blocked", nav.blocked ? (nav.blocked_reason || "true") : "false", nav.blocked ? "bad" : "ok"]]);
      setRows(groups.lidar, [["Pose", lp.valid ? `x=${fmt(lp.x, 3)} y=${fmt(lp.y, 3)} h=${fmt(lp.h, 1)}` : (lp.reason || "invalid")], ["Confidence", lp.valid ? `${fmt(lp.confidence, 2)} residual=${fmt(lp.residual_m * 100, 1)}cm` : "--"], ["Scan age", fmtMs(scan.age_ms)], ["Correction", corr.enabled === false ? "disabled" : `${corr.status || "--"} samples=${corr.samples || 0}`], ["Step", `dx=${fmt((corr.dx_median || corr.axis_step_x) * 1000, 0)}mm dy=${fmt((corr.dy_median || corr.axis_step_y) * 1000, 0)}mm dh=${fmt(corr.dh_median || corr.axis_step_h, 2)}deg`]]);
      setRows(groups.obstacles, [["Detection", obs.detection_enabled === false ? "off" : "on"], ["Mode", obs.effective_race_mode || "--"], ["Current / known", `${obs.current_count || 0} / ${obs.known_count || 0}`], ["Raw / cleared", `${obs.raw_current_count || 0} / ${obs.cleared_count || 0}`]]);
      setRows(groups.timing, [["OTOS rate", fmtHz("otos_update")], ["LiDAR scan", fmtHz("lidar_scan_arrival")], ["LiDAR process", fmtHz("lidar_process")], ["Control", fmtHz("control_update")], ["Actuation", fmtHz("drive_actuation")], ["Lite payload", fmtTiming("lite_payload")], ["Lite JSON", fmtTiming("lite_payload_json")], ["Result age", fmtTiming("lidar_result_scan_age")]]);
    }
    function onPayload(data) {
      latest = data;
      if (latest.config) parkingConfig = {enabled: !!latest.config.parking_enabled, centerX: latest.config.parking_center_x, gap: latest.config.parking_gap_m, length: latest.config.parking_wall_length_m};
      if (latest.anchor && latest.anchor.version !== anchorVersion) { if (anchorVersion === null || (latest.anchor.source !== "lidar_drift_correction" && latest.anchor.source !== "lidar_axis_correction")) trail = []; anchorVersion = latest.anchor.version; }
      if (latest.otos) { trail.push({x: latest.otos.x, y: latest.otos.y}); if (trail.length > 900) trail.shift(); }
      updateTables();
    }
    function postDeadman(forceInactive = false) {
      const payload = {client_id: deadmanId, active: forceInactive ? false : deadmanKeys.size > 0, visible: !document.hidden, focused: document.hasFocus(), keys: Array.from(deadmanKeys)};
      const encoded = JSON.stringify(payload);
      const now = performance.now();
      if (!forceInactive && !payload.active && encoded === deadmanLast && now - deadmanLastPostT < 500) return;
      if (!forceInactive && deadmanBusy) return;
      deadmanLast = encoded; deadmanLastPostT = now; deadmanBusy = true;
      fetch("/deadman", {method: "POST", headers: {"Content-Type": "application/json"}, body: encoded}).catch(() => {}).finally(() => { deadmanBusy = false; });
    }
    window.addEventListener("keydown", (event) => { if (event.key === "w" || event.key === "W" || event.code === "Space") { event.preventDefault(); deadmanKeys.add(event.code || event.key); postDeadman(); } });
    window.addEventListener("keyup", (event) => { if (event.key === "w" || event.key === "W" || event.code === "Space") { event.preventDefault(); deadmanKeys.delete(event.code || event.key); postDeadman(deadmanKeys.size === 0); } });
    window.addEventListener("blur", () => { deadmanKeys.clear(); postDeadman(true); });
    window.addEventListener("pagehide", () => postDeadman(true));
    setInterval(() => postDeadman(), 100);
    document.getElementById("stopLite").addEventListener("click", async () => { await fetch("/debug-state", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({lite_web_debug_enabled: false})}).catch(() => {}); window.location.href = "/"; });
    const events = new EventSource("/lite-events");
    events.onopen = () => { conn.textContent = "connected"; conn.className = "pill ok"; };
    events.onerror = () => { conn.textContent = "disconnected"; conn.className = "pill warn"; };
    events.onmessage = (event) => onPayload(JSON.parse(event.data));
    draw();
  </script>
</body>
</html>
"""


PORTAL_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WRO FE Debug Portal</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101418;
      color: #e7edf3;
    }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: #101418;
    }
    main {
      width: min(760px, calc(100vw - 28px));
      display: grid;
      gap: 12px;
    }
    header, section {
      border: 1px solid #26323d;
      background: #101820;
      border-radius: 6px;
      padding: 14px;
    }
    h1 {
      margin: 0 0 6px;
      font-size: 20px;
    }
    h2 {
      margin: 0 0 10px;
      color: #d9e4ed;
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }
    p {
      margin: 0;
      color: #9fb1c1;
    }
    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
    }
    button, input {
      border: 1px solid #344555;
      background: #0d1319;
      color: #e7edf3;
      border-radius: 5px;
      padding: 8px 10px;
      font: inherit;
    }
    button {
      cursor: pointer;
    }
    button.primary {
      border-color: #80e0a7;
      background: #183427;
    }
    button.warn {
      border-color: #ffca75;
      background: #352710;
    }
    input {
      width: 82px;
    }
    .grid {
      display: grid;
      gap: 1px;
      background: #17212b;
      border: 1px solid #17212b;
      border-radius: 5px;
      overflow: hidden;
    }
    .row {
      display: grid;
      grid-template-columns: minmax(145px, 0.8fr) minmax(0, 1.2fr);
      gap: 10px;
      padding: 7px 9px;
      background: #0d1319;
    }
    .label {
      color: #7f91a0;
    }
    .value {
      color: #d9e4ed;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .ok {
      color: #80e0a7;
    }
    .warn-text {
      color: #ffca75;
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>WRO FE Robot Portal</h1>
      <p>Lightweight status page. Full telemetry and camera rendering stay off until enabled.</p>
    </header>
    <section>
      <h2>Debug Controls</h2>
      <div class="actions">
        <button id="startLite" class="primary" type="button">Start Lite WebUI</button>
        <button id="stopLite" class="warn" type="button">Stop Lite WebUI</button>
        <button id="startDebug" type="button">Start Full WebUI</button>
        <button id="stopDebug" class="warn" type="button">Stop Full WebUI</button>
        <button id="cameraToggle" type="button">Camera Preview</button>
        <label>FPS <input id="cameraFps" type="number" min="0.1" step="0.5" value="4"></label>
        <button id="applyCamera" type="button">Apply Camera</button>
      </div>
    </section>
    <section>
      <h2>Status</h2>
      <div id="statusGrid" class="grid"></div>
    </section>
  </main>
  <script>
    const statusGrid = document.getElementById("statusGrid");
    const startLite = document.getElementById("startLite");
    const stopLite = document.getElementById("stopLite");
    const startDebug = document.getElementById("startDebug");
    const stopDebug = document.getElementById("stopDebug");
    const cameraToggle = document.getElementById("cameraToggle");
    const cameraFps = document.getElementById("cameraFps");
    const applyCamera = document.getElementById("applyCamera");
    let latest = {};

    function escapeHtml(value) {
      return String(value ?? "--")
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;");
    }

    function fmt(value, digits = 2) {
      const n = Number(value);
      return Number.isFinite(n) ? n.toFixed(digits) : "--";
    }

    function row(label, value, cls = "") {
      return `<div class="row"><span class="label">${escapeHtml(label)}</span><span class="value ${cls}">${escapeHtml(value)}</span></div>`;
    }

    function render(data) {
      latest = data || {};
      if (document.activeElement !== cameraFps) {
        cameraFps.value = fmt(latest.camera_web_debug_fps ?? 4, 1);
      }
      cameraToggle.textContent = latest.camera_web_debug_enabled ? "Camera Preview: On" : "Camera Preview: Off";
      cameraToggle.className = latest.camera_web_debug_enabled ? "primary" : "";
      startLite.className = latest.lite_web_debug_enabled ? "primary" : "primary";
      startDebug.className = latest.full_web_debug_enabled ? "primary" : "";
      statusGrid.innerHTML = [
        row("Lite WebUI", latest.lite_web_debug_enabled ? "enabled" : "disabled", latest.lite_web_debug_enabled ? "ok" : "warn-text"),
        row("Full WebUI", latest.full_web_debug_enabled ? "enabled" : "disabled", latest.full_web_debug_enabled ? "ok" : "warn-text"),
        row("Camera Preview", latest.camera_web_debug_enabled ? `enabled @ ${fmt(latest.camera_web_debug_fps, 1)} fps` : "disabled", latest.camera_web_debug_enabled ? "ok" : "warn-text"),
        row("Race", `${latest.race_mode || "--"} / ${latest.race_phase || "--"}`),
        row("Initial Pose", latest.initial_status || "--"),
        row("Drive", `${latest.drive_source || "--"} / ${latest.drive_reason || "--"} / v=${fmt(latest.target_speed_mps, 3)}m/s`),
        row("Deadman", `${latest.deadman_source || "none"} active=${!!latest.deadman_active} fresh=${!!latest.deadman_fresh} gpio=${latest.deadman_gpio_level ?? ""}`, latest.deadman_active && latest.deadman_fresh ? "ok" : "warn-text"),
        row("Navigation", `${latest.navigation_section || "--"} fullLock=${!!latest.full_gate_lock}`),
        row("Obstacles", `detect=${!!latest.obstacle_detection_enabled} known=${latest.known_obstacles || 0}`),
      ].join("");
    }

    async function getStatus() {
      const res = await fetch("/debug-state", {cache: "no-store"});
      render(await res.json());
    }

    async function setState(patch) {
      const res = await fetch("/debug-state", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(patch),
      });
      render(await res.json());
    }

    startLite.addEventListener("click", async () => {
      await setState({lite_web_debug_enabled: true, full_web_debug_enabled: false});
      window.location.href = "/lite";
    });
    stopLite.addEventListener("click", () => setState({lite_web_debug_enabled: false}));
    startDebug.addEventListener("click", async () => {
      await setState({full_web_debug_enabled: true, lite_web_debug_enabled: false});
      window.location.href = "/debug";
    });
    stopDebug.addEventListener("click", () => setState({full_web_debug_enabled: false}));
    cameraToggle.addEventListener("click", () => setState({camera_web_debug_enabled: !latest.camera_web_debug_enabled}));
    applyCamera.addEventListener("click", () => setState({
      camera_web_debug_enabled: latest.camera_web_debug_enabled,
      camera_web_debug_fps: Number(cameraFps.value),
    }));
    getStatus().catch(() => {});
    setInterval(() => getStatus().catch(() => {}), 1000);
  </script>
</body>
</html>
"""



def make_pose(x: float, y: float, h: float):
    return qwiic_otos.Pose2D(x, y, h)


def unit_constant(name: str, class_name: str):
    if hasattr(qwiic_otos, name):
        return getattr(qwiic_otos, name)
    return getattr(qwiic_otos.QwiicOTOS, class_name)


def require_not_false(result, label: str) -> None:
    if result is False:
        raise RuntimeError(f"{label} failed")


class OTOSThread:
    def __init__(self, args, perception: RobotPerception, drive_feedback=None):
        self.args = args
        self.perception = perception
        self.drive_feedback = drive_feedback
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

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
            print("Keep robot still: calibrating OTOS IMU...")
            require_not_false(otos.calibrateImu(), "calibrateImu()")
        if self.args.reset_tracking:
            require_not_false(otos.resetTracking(), "resetTracking()")
            require_not_false(otos.setPosition(make_pose(0.0, 0.0, 0.0)), "setPosition()")
        return otos

    def _read(self, otos) -> Pose:
        try:
            pos, vel, _acc = otos.getPosVelAcc()
        except Exception:
            pos = otos.getPosition()
            vel = otos.getVelocity()
        return Pose(
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
        except Exception as e:
            print(f"OTOS setup failed: {e}", file=sys.stderr)
            return

        period = 1.0 / self.args.otos_rate
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            try:
                pose = self._read(otos)
                self.perception.update_otos(pose)
                feedback = self.drive_feedback
                if feedback is not None:
                    update = getattr(feedback, "update_otos_feedback", None)
                    if callable(update):
                        update(pose)
            except Exception as e:
                print(f"OTOS read failed: {e}", file=sys.stderr)
                time.sleep(0.25)
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()




class ScanNode(Node):
    def __init__(self, args, perception: RobotPerception, topic: str, context):
        super().__init__("telemetry_map_server", context=context)
        self.args = args
        self.perception = perception
        self.last_scan_error_t = 0.0
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(LaserScan, topic, self.scan_cb, scan_qos)

    def scan_cb(self, scan: LaserScan) -> None:
        try:
            receive_t = time.monotonic()
            ros_now_s = self.get_clock().now().nanoseconds * 1e-9
            header_stamp_s = 0.0
            if scan.header.stamp.sec or scan.header.stamp.nanosec:
                header_stamp_s = float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1e-9
            self.perception.process_lidar_scan(
                LidarScanInput(
                    ranges=tuple(float(v) for v in scan.ranges),
                    angle_min=float(scan.angle_min),
                    angle_increment=float(scan.angle_increment),
                    range_min=float(scan.range_min),
                    range_max=float(scan.range_max),
                    scan_time=float(scan.scan_time),
                    time_increment=float(scan.time_increment),
                    header_stamp_s=header_stamp_s,
                    ros_now_s=ros_now_s,
                    receive_t=receive_t,
                )
            )
        except Exception:
            now = time.monotonic()
            if now - self.last_scan_error_t > 1.0:
                self.last_scan_error_t = now
                traceback.print_exc()


class RosThread:
    def __init__(self, args, perception: RobotPerception):
        self.args = args
        self.perception = perception
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.context = None
        self.node = None
        self.executor = None

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.executor is not None:
            try:
                self.executor.wake()
            except Exception:
                pass
        if self.context is not None:
            try:
                rclpy.shutdown(context=self.context)
            except Exception:
                pass
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = ScanNode(self.args, self.perception, self.args.scan_topic, self.context)
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


class PhysicalDeadmanThread:
    def __init__(self, args, perception: RobotPerception):
        self.args = args
        self.perception = perception
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.req = None
        self.last_error_t = 0.0

    def start(self) -> None:
        if str(getattr(self.args, "deadman_source", "physical")) not in ("physical", "both"):
            return
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._post(False, reason="physical_deadman_stop")
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self._post(False, reason="physical_deadman_stop")
        self._close_request()

    @staticmethod
    def _coerce_level(value) -> int:
        if isinstance(value, bool):
            return 1 if value else 0
        if isinstance(value, int):
            return int(value)
        name = str(value).upper()
        if "ACTIVE" in name and "INACTIVE" not in name:
            return 1
        if "INACTIVE" in name:
            return 0
        return 1 if bool(value) else 0

    def _open_request(self) -> None:
        import gpiod

        bias = gpiod.line.Bias.PULL_UP if bool(getattr(self.args, "deadman_gpio_pull_up", True)) else gpiod.line.Bias.AS_IS
        settings = gpiod.LineSettings(
            direction=gpiod.line.Direction.INPUT,
            bias=bias,
        )
        self.req = gpiod.request_lines(
            str(getattr(self.args, "deadman_gpiochip", "/dev/gpiochip4")),
            consumer="fe26-deadman",
            config={int(getattr(self.args, "deadman_gpio_line", 4)): settings},
        )

    def _close_request(self) -> None:
        if self.req is None:
            return
        try:
            self.req.release()
        except Exception:
            pass
        self.req = None

    def _post(self, active: bool, *, reason: str = "ok", gpio_level=None) -> None:
        status = {
            "client_id": f"physical_gpio{int(getattr(self.args, 'deadman_gpio_line', 4))}",
            "source": "physical",
            "active": bool(active),
            "visible": True,
            "focused": True,
            "reason": str(reason),
        }
        if gpio_level is not None:
            status["gpio_level"] = int(gpio_level)
        try:
            self.perception.update_drive_deadman(status)
        except Exception:
            pass

    def _read_active(self) -> tuple[bool, int]:
        line = int(getattr(self.args, "deadman_gpio_line", 4))
        values = self.req.get_values([line])
        level = self._coerce_level(values[0])
        active_low = bool(getattr(self.args, "deadman_gpio_active_low", True))
        active = level == 0 if active_low else level != 0
        return active, level

    def _run(self) -> None:
        period = 1.0 / max(float(getattr(self.args, "deadman_gpio_poll_hz", 50.0)), 1e-6)
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            try:
                if self.req is None:
                    self._open_request()
                    print(
                        "Physical deadman started: "
                        f"{getattr(self.args, 'deadman_gpiochip', '/dev/gpiochip4')} "
                        f"line {int(getattr(self.args, 'deadman_gpio_line', 4))} "
                        f"active {'LOW' if bool(getattr(self.args, 'deadman_gpio_active_low', True)) else 'HIGH'}"
                    )
                active, level = self._read_active()
                self._post(active, gpio_level=level)
            except Exception as exc:
                self._close_request()
                self._post(False, reason="gpio_error")
                now = time.monotonic()
                if now - self.last_error_t >= 1.0:
                    self.last_error_t = now
                    print(f"Physical deadman unavailable: {exc}", file=sys.stderr)
                time.sleep(min(0.5, max(period, 0.02)))
                next_t = time.monotonic()
                continue

            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()


class LEDStatusThread:
    def __init__(self, args, perception: RobotPerception, db):
        self.args = args
        self.perception = perception
        self.db = db
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.last_front = None
        self.last_rear = None
        self.started_t = time.monotonic()

    def start(self) -> None:
        if not bool(getattr(self.args, "led_status", True)):
            return
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self._write_leds(0.0, 0.0, force=True)

    @staticmethod
    def _clamp_level(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    @staticmethod
    def _triangle01(phase: float) -> float:
        phase = phase % 1.0
        if phase < 0.5:
            return phase * 2.0
        return (1.0 - phase) * 2.0

    def _payload_snapshot(self) -> dict:
        try:
            payload = self.perception.shared.cached_payload()
            return payload if isinstance(payload, dict) else {}
        except Exception:
            return {}

    def _lidar_scan_recent(self, now: float) -> bool:
        try:
            timings = self.perception.shared.timing_snapshot()
            lidar_arrival = timings.get("lidar_scan_arrival", {}) if isinstance(timings, dict) else {}
            last_t = float(lidar_arrival.get("last_t", 0.0) or 0.0) if isinstance(lidar_arrival, dict) else 0.0
            timeout_s = max(0.001, float(getattr(self.args, "led_wait_lidar_timeout_s", 0.50)))
            return last_t > 0.0 and 0.0 <= now - last_t <= timeout_s
        except Exception:
            return False

    def _front_confirmation_level(self, color_conf: dict, front_base: float) -> float:
        confirm_level = self._clamp_level(getattr(self.args, "led_front_confirmation", 0.80))
        attempt = int(color_conf.get("attempt", 0) or 0)
        gain_request = color_conf.get("camera_gain_request", {})
        gain_reason = str(gain_request.get("reason", "") if isinstance(gain_request, dict) else "")
        exposure_reason = str(color_conf.get("exposure_retry_reason", "") or gain_reason)
        if attempt > 0 and exposure_reason == "overexposed":
            return front_base
        return confirm_level

    def _desired_levels(self, now: float) -> tuple[float, float]:
        front_base = self._clamp_level(getattr(self.args, "led_front_base", 0.03))
        rear_base = self._clamp_level(getattr(self.args, "led_rear_base", 0.10))
        rear_pulse = self._clamp_level(getattr(self.args, "led_rear_pulse", 1.0))
        payload = self._payload_snapshot()
        competition = payload.get("competition_start", {}) if isinstance(payload, dict) else {}
        competition_state = str(competition.get("state", "") if isinstance(competition, dict) else "")

        if competition_state in ("", "waiting_for_start"):
            if self._lidar_scan_recent(now):
                period = max(0.05, float(getattr(self.args, "led_wait_breathe_period_s", 2.2)))
                phase = self._triangle01((now - self.started_t) / period)
                front_max = self._clamp_level(getattr(self.args, "led_wait_front_breathe_max", 0.30))
                rear_max = self._clamp_level(getattr(self.args, "led_wait_rear_breathe_max", 1.00))
                return phase * front_max, phase * rear_max
            return front_base, rear_base

        nav = payload.get("navigation", {}) if isinstance(payload, dict) else {}
        color_conf = nav.get("color_confirmation", {}) if isinstance(nav, dict) else {}
        hold_reason = str(nav.get("hold_reason", "") if isinstance(nav, dict) else "")
        color_confirm_active = (
            isinstance(color_conf, dict)
            and bool(color_conf.get("slot_id", ""))
            and hold_reason == "color_confirmation"
            and not bool(color_conf.get("commit_color"))
        )
        front = self._front_confirmation_level(color_conf, front_base) if color_confirm_active else front_base

        blink_period = max(0.05, float(getattr(self.args, "led_rear_blink_period_s", 2.0)))
        blink_on = max(0.001, min(blink_period, float(getattr(self.args, "led_rear_blink_on_s", 0.10))))
        rear = rear_pulse if ((now - self.started_t) % blink_period) < blink_on else rear_base
        return front, rear

    def _write_leds(self, front: float, rear: float, *, force: bool = False) -> None:
        front = self._clamp_level(front)
        rear = self._clamp_level(rear)
        if force or self.last_front is None or abs(front - float(self.last_front)) >= 0.01:
            try:
                self.db.set_front_led(front)
                self.last_front = front
            except Exception:
                pass
        if force or self.last_rear is None or abs(rear - float(self.last_rear)) >= 0.01:
            try:
                self.db.set_rear_led(rear)
                self.last_rear = rear
            except Exception:
                pass

    def _run(self) -> None:
        period = 0.02
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            front, rear = self._desired_levels(now)
            self._write_leds(front, rear)
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()


class DriveControlThread:
    def __init__(self, args, perception: RobotPerception):
        self.args = args
        self.perception = perception
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.db = None
        self.last_drive_allowed_t = 0.0
        self.drive_log_file = None
        self.drive_log_writer = None
        self.drive_log_queue = None
        self.drive_log_thread = None
        self.drive_log_stop = threading.Event()
        self.drive_log_dropped_rows = 0
        self.drive_log_last_flush_t = 0.0
        self.last_output_kind = "neutral"
        self.applied_brake_level = 0.0
        self.last_brake_update_t = 0.0
        self.brake_release_active = False
        self.last_drive_writerow_ms = ""
        self.last_drive_flush_ms = ""
        self.last_drive_flush_requested = False
        self.last_drive_sleep_ms = ""
        self.last_commanded_steering = None
        self.last_steering_write_t = 0.0
        self.last_runtime_duty_cap = None
        self._drive_io_timing = None
        self.led_thread = None

    def _record_drive_io_timing(self, name: str, value) -> None:
        timing = getattr(self, "_drive_io_timing", None)
        if isinstance(timing, dict):
            timing[name] = value

    def _timed_db_call(self, name: str, func, *args, **kwargs):
        start = time.monotonic()
        try:
            return func(*args, **kwargs)
        finally:
            self._record_drive_io_timing(name, f"{(time.monotonic() - start) * 1000.0:.3f}")

    def _summarize_drive_timing_events(self, events: list) -> dict:
        if not events:
            return {}

        def as_float(value, default=0.0):
            try:
                return float(value)
            except (TypeError, ValueError):
                return default

        def set_max(out: dict, key: str, value) -> None:
            val = as_float(value, 0.0)
            if key not in out or val > as_float(out.get(key), 0.0):
                out[key] = f"{val:.3f}"

        out = {
            "timing_event_count": len(events),
            "timing_event_slow_count": 0,
        }
        threshold = max(0.0, float(getattr(self.args, "drive_timing_slow_threshold_ms", 10.0)))
        slow_items = []
        hw_items = []
        op_prefix = {
            "set_steering": "db_set_steering",
            "steering_apply": "db_steering_apply",
            "motion_apply": "db_motion_apply",
            "led_apply": "db_led_apply",
            "actuator_stale_motion": "db_actuator_stale_motion",
            "set_target_speed": "db_set_target_speed",
            "get_telemetry": "db_get_telemetry",
            "_apply_output": "db_apply_output",
            "set_front_led": "db_led",
            "set_rear_led": "db_led",
            "set_mirror_leds_enabled": "db_led",
            "set_external_velocity_feedback": "db_external_feedback",
            "select_velocity_feedback_read": "db_feedback_select",
            "select_velocity_feedback_write": "db_feedback_select",
            "brake": "db_brake",
            "neutral": "db_neutral",
            "coast": "db_coast",
            "set_runtime_max_command_duty": "db_runtime_duty_cap",
            "internal_brake_pwm": "db_internal_brake",
            "internal_brake_publish": "db_internal_brake_publish",
            "internal_loop": "db_internal_loop",
        }

        for ev in events:
            if not isinstance(ev, dict):
                continue
            category = str(ev.get("category", ""))
            duration = as_float(ev.get("duration_ms"), 0.0)
            lock_wait = as_float(ev.get("lock_wait_ms"), 0.0)
            lock_inside = as_float(ev.get("lock_inside_ms"), 0.0)
            pwm_write = as_float(ev.get("pwm_write_ms"), 0.0)
            slow = bool(ev.get("slow", False)) or max(duration, lock_wait, lock_inside, pwm_write) >= threshold
            if slow:
                out["timing_event_slow_count"] = int(out.get("timing_event_slow_count", 0)) + 1
            set_max(out, "timing_event_max_ms", duration)
            set_max(out, "timing_lock_wait_max_ms", lock_wait)
            set_max(out, "timing_lock_inside_max_ms", lock_inside)
            set_max(out, "timing_pwm_write_max_ms", pwm_write)

            if category == "hw_pwm":
                set_max(out, "timing_hw_pwm_max_ms", duration)
                if slow:
                    out["timing_hw_pwm_slow_count"] = int(out.get("timing_hw_pwm_slow_count", 0)) + 1
                if duration > 0.0:
                    hw_items.append((duration, str(ev.get("path_kind", ev.get("path", "")))))
                continue

            op = str(ev.get("operation", ""))
            prefix = op_prefix.get(op)
            if prefix:
                set_max(out, f"{prefix}_duration_ms", duration)
                set_max(out, f"{prefix}_lock_wait_ms", lock_wait)
                set_max(out, f"{prefix}_lock_inside_ms", lock_inside)
                set_max(out, f"{prefix}_pwm_write_ms", pwm_write)
                if op == "internal_loop":
                    set_max(out, "db_internal_loop_overrun_ms", ev.get("overrun_ms", 0.0))
            if duration > 0.0:
                slow_items.append((duration, category, op or str(ev.get("path_kind", "")), str(ev.get("thread", ""))))

        slow_items.sort(reverse=True, key=lambda item: item[0])
        hw_items.sort(reverse=True, key=lambda item: item[0])
        out["timing_event_summary"] = ";".join(
            f"{cat}:{name}:{dur:.1f}ms:{thread}" for dur, cat, name, thread in slow_items[:6]
        )
        out["timing_hw_pwm_summary"] = ";".join(f"{name}:{dur:.1f}ms" for dur, name in hw_items[:6])
        return out

    @staticmethod
    def _otos_forward_speed(pose: Pose) -> float:
        h_rad = math.radians(float(pose.h))
        return float(pose.vx) * math.cos(h_rad) + float(pose.vy) * math.sin(h_rad)

    def update_otos_feedback(self, pose: Pose) -> None:
        if not bool(getattr(self.args, "drivebase_external_velocity_feedback", True)):
            return
        db = self.db
        if db is None:
            return
        setter = getattr(db, "set_external_velocity_feedback", None)
        if not callable(setter):
            return
        try:
            setter(self._otos_forward_speed(pose), sample_t=float(pose.t), source="otos_forward")
        except Exception:
            pass

    def _set_steering_if_needed(self, steer: float, now: float) -> None:
        deadband = max(0.0, float(getattr(self.args, "drive_steering_write_deadband_norm", 0.004)))
        keepalive_s = max(0.0, float(getattr(self.args, "drive_steering_write_keepalive_s", 0.25)))
        previous = self.last_commanded_steering
        due = previous is None or abs(float(steer) - float(previous)) >= deadband
        if not due and keepalive_s > 0.0 and now - float(self.last_steering_write_t or 0.0) >= keepalive_s:
            due = True
        self._record_drive_io_timing("steering_write_due", int(bool(due)))
        if due:
            self._timed_db_call("db_set_steering_call_ms", self.db.set_steering, steer)
            self.last_commanded_steering = float(steer)
            self.last_steering_write_t = now

    def _set_runtime_duty_cap_if_needed(self, duty_cap) -> None:
        normalized = None
        if duty_cap not in (None, ""):
            normalized = max(0.0, min(1.0, float(duty_cap)))
        if self.last_runtime_duty_cap == normalized:
            return
        setter = getattr(self.db, "set_runtime_max_command_duty", None)
        if callable(setter):
            self._timed_db_call("db_runtime_duty_cap_call_ms", setter, normalized)
            self.last_runtime_duty_cap = normalized

    def start(self) -> None:
        if not self.args.enable_drive_control:
            return
        self.thread.start()

    def _start_led_thread(self) -> None:
        if self.db is None or not bool(getattr(self.args, "led_status", True)):
            return
        self.led_thread = LEDStatusThread(self.args, self.perception, self.db)
        self.led_thread.start()

    def _stop_led_thread(self) -> None:
        if self.led_thread is None:
            return
        try:
            self.led_thread.stop()
        except Exception:
            pass
        self.led_thread = None

    def _stop_power(self) -> None:
        if self.db is None:
            return
        try:
            self.db.brake(1.0)
        except Exception:
            pass
        try:
            self.db.set_steering(0.0, wait=True)
        except Exception:
            pass

    def _neutral_power(self) -> None:
        if self.db is None:
            return
        try:
            self._timed_db_call("db_neutral_call_ms", self.db.neutral)
        except Exception:
            try:
                self._timed_db_call("db_coast_call_ms", self.db.coast)
            except Exception:
                pass

    def _center_steering(self) -> None:
        if self.db is None:
            return
        try:
            self.db.set_steering(0.0, wait=True)
        except Exception:
            pass

    def _brake_release_enabled(self) -> bool:
        return bool(getattr(self.args, "drive_brake_release_enabled", True))

    def _brake_order_for_command(self, cmd: dict) -> str:
        order = str(cmd.get("brake_order", cmd.get("brake_release_order", "reverse_first")) or "reverse_first")
        return "forward_first" if order == "forward_first" else "reverse_first"

    def _next_brake_level(self, requested_level: float, now: float) -> float:
        requested = max(0.0, min(1.0, float(requested_level)))
        last_t = self.last_brake_update_t or now
        dt = max(0.0, now - last_t)
        self.last_brake_update_t = now
        current = max(0.0, min(1.0, float(self.applied_brake_level)))
        if requested >= current or not self._brake_release_enabled():
            applied = requested
        else:
            rate = max(0.0, float(getattr(self.args, "drive_brake_release_rate_per_s", 4.0)))
            min_level = max(0.0, min(1.0, float(getattr(self.args, "drive_brake_release_min_level", 0.0))))
            target = max(requested, min_level if current > requested else requested)
            applied = target if rate <= 0.0 else max(target, current - rate * dt)
        self.applied_brake_level = applied
        self.brake_release_active = applied > requested + 1e-4
        return applied

    def _annotate_brake_release(self, cmd: dict, requested_level: float, applied_level: float, phase: str) -> None:
        cmd["drive_requested_brake_level"] = requested_level
        cmd["drive_applied_brake_level"] = applied_level
        cmd["drive_brake_release_active"] = bool(self.brake_release_active)
        cmd["drive_brake_order"] = self._brake_order_for_command(cmd)
        if self.brake_release_active and not cmd.get("drive_handoff_phase"):
            cmd["drive_handoff_phase"] = phase

    def _apply_brake_request(self, cmd: dict, requested_level: float, now: float, *, phase: str = "brake_release") -> float:
        applied = self._next_brake_level(requested_level, now)
        self._annotate_brake_release(cmd, requested_level, applied, phase)
        brake_order = self._brake_order_for_command(cmd)
        if applied > 1e-4:
            self._timed_db_call("db_brake_call_ms", self.db.brake, applied, order=brake_order)
            self.last_output_kind = "brake"
        else:
            self._neutral_power()
            self.last_output_kind = "neutral"
        return applied

    def _release_brake_before_motion(self, cmd: dict, now: float, *, phase: str) -> bool:
        applied = self._next_brake_level(0.0, now)
        self._annotate_brake_release(cmd, 0.0, applied, phase)
        brake_order = self._brake_order_for_command(cmd)
        if applied > 1e-4:
            self._timed_db_call("db_release_brake_call_ms", self.db.brake, applied, order=brake_order)
            self.last_output_kind = "brake"
            return True
        return False

    def _open_drive_debug_log(self) -> None:
        path = str(getattr(self.args, "drive_debug_log", "") or "").strip()
        if not path:
            return
        try:
            log_path = Path(path).expanduser()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.drive_log_file = log_path.open("w", newline="", buffering=1)
            fields = [
                "t",
                "drive_loop_start_t",
                "drive_loop_after_command_t",
                "drive_telemetry_t",
                "loop_ms",
                "drive_loop_target_period_ms",
                "drive_loop_overrun_ms",
                "drive_t_cmd_fetch_ms",
                "drive_t_status_update_ms",
                "drive_t_duty_cap_ms",
                "drive_t_set_steering_ms",
                "drive_t_motion_command_ms",
                "drive_steering_write_due",
                "drive_t_db_runtime_duty_cap_call_ms",
                "drive_t_db_set_steering_call_ms",
                "drive_t_db_release_brake_call_ms",
                "drive_t_db_brake_call_ms",
                "drive_t_db_neutral_call_ms",
                "drive_t_db_coast_call_ms",
                "drive_t_db_set_drive_power_call_ms",
                "drive_t_db_set_target_speed_call_ms",
                "drive_t_db_get_telemetry_call_ms",
                "drive_t_get_telemetry_ms",
                "drive_t_shared_telemetry_ms",
                "drive_t_record_timing_ms",
                "drive_t_row_build_ms",
                "drive_t_writerow_prev_ms",
                "drive_t_flush_prev_ms",
                "drive_log_flush_requested_prev",
                "drive_log_queue_depth",
                "drive_log_dropped_rows",
                "drive_t_sleep_ms",
                "drive_t_total_instrumented_ms",
                "drive_timing_event_count",
                "drive_timing_event_slow_count",
                "drive_timing_event_max_ms",
                "drive_timing_event_summary",
                "drive_timing_lock_wait_max_ms",
                "drive_timing_lock_inside_max_ms",
                "drive_timing_pwm_write_max_ms",
                "drive_timing_hw_pwm_slow_count",
                "drive_timing_hw_pwm_max_ms",
                "drive_timing_hw_pwm_summary",
                "drive_t_db_set_steering_duration_ms",
                "drive_t_db_set_steering_lock_wait_ms",
                "drive_t_db_set_steering_lock_inside_ms",
                "drive_t_db_set_steering_pwm_ms",
                "drive_t_db_steering_apply_duration_ms",
                "drive_t_db_steering_apply_lock_wait_ms",
                "drive_t_db_steering_apply_lock_inside_ms",
                "drive_t_db_steering_apply_pwm_ms",
                "drive_t_db_motion_apply_duration_ms",
                "drive_t_db_motion_apply_lock_wait_ms",
                "drive_t_db_motion_apply_lock_inside_ms",
                "drive_t_db_motion_apply_pwm_ms",
                "drive_t_db_led_apply_duration_ms",
                "drive_t_db_led_apply_lock_wait_ms",
                "drive_t_db_led_apply_lock_inside_ms",
                "drive_t_db_led_apply_pwm_ms",
                "drive_t_db_actuator_stale_motion_duration_ms",
                "drive_t_db_internal_brake_publish_duration_ms",
                "drive_t_db_set_target_speed_duration_ms",
                "drive_t_db_set_target_speed_lock_wait_ms",
                "drive_t_db_set_target_speed_lock_inside_ms",
                "drive_t_db_set_target_speed_pwm_ms",
                "drive_t_db_get_telemetry_duration_ms",
                "drive_t_db_get_telemetry_lock_wait_ms",
                "drive_t_db_get_telemetry_lock_inside_ms",
                "drive_t_db_apply_output_duration_ms",
                "drive_t_db_apply_output_lock_wait_ms",
                "drive_t_db_apply_output_lock_inside_ms",
                "drive_t_db_apply_output_pwm_ms",
                "drive_t_db_led_duration_ms",
                "drive_t_db_led_lock_wait_ms",
                "drive_t_db_led_lock_inside_ms",
                "drive_t_db_led_pwm_ms",
                "drive_t_db_external_feedback_duration_ms",
                "drive_t_db_external_feedback_lock_wait_ms",
                "drive_t_db_external_feedback_lock_inside_ms",
                "drive_t_db_feedback_select_duration_ms",
                "drive_t_db_feedback_select_lock_wait_ms",
                "drive_t_db_feedback_select_lock_inside_ms",
                "drive_t_db_brake_duration_ms",
                "drive_t_db_brake_lock_wait_ms",
                "drive_t_db_brake_pwm_ms",
                "drive_t_db_internal_brake_duration_ms",
                "drive_t_db_internal_brake_pwm_ms",
                "drive_t_db_internal_loop_duration_ms",
                "drive_t_db_internal_loop_overrun_ms",
                "actuation_allowed",
                "cmd_source",
                "cmd_reason",
                "cmd_blocked_while_nav_unblocked",
                "cmd_brake",
                "cmd_brake_preserve_steering",
                "cmd_brake_level",
                "cmd_neutral_stop",
                "cmd_allow_reverse",
                "cmd_precision_reverse",
                "cmd_allow_breakaway",
                "cmd_target_speed_mps",
                "cmd_raw_target_speed_mps",
                "cmd_direct_drive_power",
                "drive_handoff_phase",
                "drive_requested_brake_level",
                "drive_applied_brake_level",
                "drive_brake_release_active",
                "drive_brake_order",
                "cmd_drive_duty_cap",
                "cmd_steering_norm",
                "cmd_steering_deg",
                "cmd_command_age_ms",
                "cmd_staleness_speed_cap_active",
                "cmd_staleness_speed_cap_triggered",
                "cmd_staleness_speed_cap_age_ms",
                "cmd_staleness_speed_cap_threshold_ms",
                "cmd_staleness_speed_cap_mps",
                "cmd_staleness_speed_cap_hold_s",
                "cmd_staleness_speed_cap_remaining_s",
                "cmd_staleness_speed_cap_original_target_mps",
                "cmd_staleness_speed_cap_original_raw_mps",
                "cmd_staleness_speed_cap_event_count",
                "cmd_staleness_speed_cap_last_event_age_ms",
                "cmd_low_confidence_watchdog_enabled",
                "cmd_low_confidence_watchdog_active",
                "cmd_low_confidence_watchdog_armed",
                "cmd_low_confidence_watchdog_elapsed_s",
                "cmd_low_confidence_watchdog_timeout_s",
                "cmd_low_confidence_watchdog_stationary",
                "cmd_low_confidence_watchdog_stationary_speed_mps",
                "cmd_low_confidence_watchdog_stationary_threshold_mps",
                "cmd_low_confidence_watchdog_reason",
                "collision_supervisor_enabled",
                "collision_supervisor_suppressed_reason",
                "collision_would_action",
                "collision_lidar_active_brake_enabled",
                "collision_lidar_active_brake_triggered",
                "collision_lidar_active_brake_active",
                "collision_lidar_active_brake_remaining_s",
                "collision_lidar_active_brake_hold_s",
                "collision_lidar_active_brake_event_count",
                "collision_lidar_active_brake_last_event_age_ms",
                "collision_lidar_active_brake_trigger_scan_t",
                "collision_lidar_metric_present",
                "collision_lidar_metric_source",
                "collision_lidar_raw_status",
                "collision_lidar_raw_usable",
                "collision_lidar_raw_points",
                "collision_lidar_status",
                "collision_lidar_points",
                "collision_lidar_min_points",
                "collision_lidar_hit",
                "collision_lidar_warning_points",
                "collision_lidar_warning_min_points",
                "collision_lidar_warning_shell_points",
                "collision_lidar_warning_hit",
                "collision_lidar_trigger",
                "collision_lidar_trigger_reason",
                "collision_lidar_trigger_by_strong_single",
                "collision_lidar_trigger_by_hard",
                "collision_lidar_trigger_by_warning_handoff",
                "collision_lidar_strong_single_enabled",
                "collision_lidar_strong_hard_points",
                "collision_lidar_strong_combo_hard_points",
                "collision_lidar_strong_combo_warning_points",
                "collision_lidar_window_hits",
                "collision_lidar_window_warning_hits",
                "collision_lidar_window_prior_warning_hits",
                "collision_lidar_window_usable",
                "collision_lidar_window_required",
                "collision_lidar_window_warning_required",
                "collision_lidar_window_hard_required_with_warning",
                "collision_lidar_yaw_rate_dps",
                "collision_lidar_yaw_gated",
                "collision_lidar_depth_m",
                "collision_lidar_warning_depth_m",
                "collision_lidar_nearest_x_m",
                "collision_lidar_nearest_abs_y_m",
                "collision_lidar_warning_nearest_x_m",
                "collision_lidar_warning_nearest_abs_y_m",
                "collision_stuck_candidate",
                "collision_stuck_trigger",
                "collision_stuck_reason",
                "collision_stuck_elapsed_s",
                "collision_stuck_required_s",
                "collision_stuck_target_speed_mps",
                "collision_stuck_applied_mode",
                "collision_stuck_applied_duty",
                "collision_stuck_duty_threshold",
                "collision_stuck_otos_forward_mps",
                "collision_stuck_route_progress_m",
                "collision_stuck_route_progress_delta_m",
                "collision_stuck_sample_count",
                "collision_recovery_state",
                "collision_recovery_enabled",
                "collision_recovery_reason",
                "collision_recovery_exit_reason",
                "collision_recovery_episode_elapsed_s",
                "collision_recovery_persistent_ready",
                "collision_recovery_hazard_usable",
                "collision_recovery_hazard_latest_age_ms",
                "collision_recovery_reverse_clear_elapsed_s",
                "collision_recovery_section_visit_key",
                "collision_recovery_section_visit_count",
                "collision_recovery_section_visit_limit",
                "collision_recovery_reverse_attempt_count",
                "collision_recovery_requested_target_m",
                "collision_recovery_target_m",
                "collision_recovery_safe_distance_m",
                "collision_recovery_progress_m",
                "collision_recovery_remaining_m",
                "collision_recovery_reverse_clear_since_t",
                "collision_recovery_limiting_reason",
                "collision_recovery_creep_active",
                "collision_recovery_creep_override",
                "collision_recovery_creep_speed_mps",
                "collision_recovery_creep_clear_elapsed_s",
                "deadman_active",
                "deadman_fresh",
                "deadman_source",
                "deadman_client_id",
                "deadman_reason",
                "deadman_gpio_level",
                "deadman_active_clients",
                "deadman_client_count",
                "deadman_age_ms",
                "competition_start_state",
                "competition_processing_allowed",
                "competition_drive_logic_allowed",
                "competition_trigger_t",
                "competition_measurement_start_t",
                "competition_active_t",
                "competition_delay_remaining_s",
                "competition_initial_hold_remaining_s",
                "competition_startup_lidar_correction_remaining_s",
                "competition_startup_lidar_correction_active",
                "competition_startup_lidar_correction_done",
                "otos_vx_mps",
                "otos_vy_mps",
                "otos_vh_dps",
                "otos_speed_mps",
                "otos_forward_mps",
                "otos_display_vx_mps",
                "otos_display_vy_mps",
                "otos_display_speed_mps",
                "otos_age_ms",
                "otos_jump_detected",
                "otos_jump_triggered_hold",
                "otos_jump_reason",
                "otos_jump_trigger_rules",
                "otos_jump_event_count",
                "otos_jump_dt_ms",
                "otos_jump_dx_m",
                "otos_jump_dy_m",
                "otos_jump_dist_m",
                "otos_jump_implied_speed_mps",
                "otos_jump_reported_speed_mps",
                "otos_jump_vector_mismatch_mps",
                "otos_jump_angle_velocity_deg",
                "otos_jump_angle_trend_deg",
                "otos_jump_trend_distance_m",
                "otos_jump_phase",
                "otos_jump_hold_suppressed_reason",
                "otos_display_x_m",
                "otos_display_y_m",
                "otos_display_h_deg",
                "otos_only_x_m",
                "otos_only_y_m",
                "otos_only_h_deg",
                "fused_minus_otos_x_m",
                "fused_minus_otos_y_m",
                "fused_minus_otos_h_deg",
                "fused_minus_otos_distance_m",
                "otos_local_x_m",
                "otos_local_y_m",
                "otos_local_h_deg",
                "anchor_x_m",
                "anchor_y_m",
                "anchor_h_deg",
                "anchor_version",
                "anchor_source",
                "anchor_last_source",
                "anchor_last_rejected",
                "anchor_last_jump_m",
                "anchor_last_jump_h_deg",
                "lidar_valid",
                "lidar_confidence",
                "lidar_reason",
                "lidar_residual_m",
                "lidar_dx_m",
                "lidar_dy_m",
                "lidar_dh_deg",
                "lidar_matched_points",
                "lidar_rejected_points",
                "lidar_visible_walls",
                "lidar_cloud_wall_valid",
                "lidar_cloud_wall_reason",
                "lidar_cloud_wall_matched_points",
                "lidar_cloud_wall_wall_count",
                "lidar_cloud_wall_median_abs_m",
                "lidar_cloud_wall_p80_abs_m",
                "lidar_cloud_wall_p95_abs_m",
                "lidar_cloud_wall_median_signed_m",
                "lidar_cloud_wall_worst_wall",
                "lidar_cloud_wall_worst_wall_p80_abs_m",
                "lidar_cloud_wall_dominant_wall",
                "lidar_cloud_wall_dominant_residual_m",
                "lidar_cloud_wall_live_ref_dx_m",
                "lidar_cloud_wall_live_ref_dy_m",
                "lidar_cloud_wall_live_ref_distance_m",
                "lidar_cloud_wall_live_ref_dh_deg",
                "lidar_cloud_wall_scan_age_ms",
                "lidar_cloud_wall_scan_mid_age_ms",
                "lidar_cloud_wall_speed_mps",
                "lidar_cloud_wall_yaw_rate_dps",
                "lidar_cloud_wall_speed_x_scan_age_m",
                "lidar_cloud_wall_diagnostic_status",
                "lidar_cloud_wall_diagnostic_mode",
                "lidar_cloud_wall_diagnostic_request_id",
                "lidar_cloud_wall_diagnostic_pending",
                "lidar_cloud_wall_diagnostic_submitted",
                "lidar_cloud_wall_diagnostic_completed",
                "lidar_cloud_wall_diagnostic_dropped",
                "lidar_cloud_wall_diagnostic_dropped_this_submit",
                "lidar_cloud_wall_diagnostic_errors",
                "lidar_cloud_wall_diagnostic_source_scan_t",
                "lidar_cloud_wall_diagnostic_age_ms",
                "lidar_cloud_wall_diagnostic_result_age_ms",
                "lidar_cloud_wall_diagnostic_queue_age_ms",
                "lidar_cloud_wall_diagnostic_worker_wall_ms",
                "lidar_cloud_wall_diagnostic_worker_process_cpu_ms",
                "lidar_cloud_wall_diagnostic_worker_thread_cpu_ms",
                "lidar_cloud_wall_points_ms",
                "lidar_cloud_wall_wall_loop_ms",
                "lidar_cloud_wall_stats_ms",
                "lidar_cloud_wall_meta_ms",
                "lidar_cloud_wall_summary",
                "lidar_correction_status",
                "lidar_correction_mode",
                "lidar_correction_validation_enabled",
                "lidar_correction_heading_enabled",
                "lidar_correction_axis_heading_enabled",
                "lidar_correction_samples",
                "lidar_correction_dx_median",
                "lidar_correction_dy_median",
                "lidar_correction_dh_median",
                "lidar_correction_dx_std",
                "lidar_correction_dy_std",
                "lidar_correction_dh_std",
                "lidar_correction_last_step_x",
                "lidar_correction_last_step_y",
                "lidar_correction_last_step_h",
                "lidar_correction_axis_x_raw",
                "lidar_correction_axis_y_raw",
                "lidar_correction_axis_h_raw",
                "lidar_correction_axis_x_weighted_residual",
                "lidar_correction_axis_y_weighted_residual",
                "lidar_correction_axis_x_selected_residuals",
                "lidar_correction_axis_y_selected_residuals",
                "lidar_correction_estimate_confidence",
                "lidar_correction_estimate_dx_m",
                "lidar_correction_estimate_dy_m",
                "lidar_correction_estimate_dh_deg",
                "lidar_correction_rolling_decision",
                "lidar_correction_rolling_reject_reason",
                "lidar_correction_rolling_consistent_xy",
                "lidar_correction_rolling_consistent_h",
                "lidar_correction_rolling_large_enough",
                "lidar_correction_rolling_confidence_median",
                "lidar_correction_rolling_step_x_unclamped",
                "lidar_correction_rolling_step_y_unclamped",
                "lidar_correction_rolling_step_h_unclamped",
                "lidar_correction_rolling_step_h_limited",
                "lidar_correction_rolling_candidate_step_x",
                "lidar_correction_rolling_candidate_step_y",
                "lidar_correction_rolling_candidate_step_h",
                "lidar_correction_axis_x_filter_pre_step",
                "lidar_correction_axis_y_filter_pre_step",
                "lidar_correction_axis_h_filter_pre_step",
                "lidar_correction_axis_x_filter",
                "lidar_correction_axis_y_filter",
                "lidar_correction_axis_h_filter",
                "lidar_correction_axis_step_x",
                "lidar_correction_axis_step_y",
                "lidar_correction_axis_step_h",
                "lidar_correction_axis_step_x_unclamped",
                "lidar_correction_axis_step_y_unclamped",
                "lidar_correction_axis_step_h_unclamped",
                "lidar_correction_axis_step_h_limited",
                "lidar_correction_axis_step_zero_reason",
                "lidar_correction_axis_candidate_step_x",
                "lidar_correction_axis_candidate_step_y",
                "lidar_correction_axis_candidate_step_h",
                "lidar_correction_axis_motion_scale",
                "lidar_correction_yaw_rate_dps",
                "lidar_correction_yaw_authority",
                "lidar_correction_yaw_limited",
                "lidar_correction_rolling_yaw_authority_median",
                "lidar_correction_rolling_yaw_authority_min",
                "lidar_correction_rolling_distance_authority_median",
                "lidar_correction_rolling_distance_authority_min",
                "lidar_correction_rolling_combined_authority_median",
                "lidar_correction_rolling_combined_authority_min",
                "lidar_correction_axis_x_distance_authority",
                "lidar_correction_axis_y_distance_authority",
                "lidar_correction_axis_h_distance_authority",
                "lidar_correction_axis_distance_authority_min",
                "lidar_correction_axis_distance_authority_median",
                "lidar_correction_axis_effective_max_error_m",
                "lidar_correction_axis_used_walls",
                "lidar_correction_axis_used_wall_count",
                "lidar_correction_axis_x_wall_count",
                "lidar_correction_axis_y_wall_count",
                "lidar_correction_axis_rejected",
                "lidar_correction_axis_rejected_count",
                "lidar_correction_axis_heading_walls",
                "lidar_correction_axis_heading_wall_count",
                "lidar_correction_axis_heading_rejected",
                "lidar_correction_axis_heading_rejected_count",
                "lidar_correction_axis_heading_block_reason",
                "lidar_correction_axis_heading_measurement_candidate",
                "lidar_correction_axis_heading_measurement_source",
                "lidar_correction_axis_heading_would_filter",
                "lidar_correction_axis_heading_would_step_h_unclamped",
                "lidar_correction_axis_heading_would_step_h",
                "lidar_correction_axis_heading_would_step_h_limited",
                "lidar_correction_axis_heading_would_step_zero_reason",
                "lidar_correction_axis_heading_raw_weighted_mean",
                "lidar_correction_axis_heading_clamped_mean",
                "lidar_correction_axis_heading_effective_weight_sum",
                "lidar_correction_axis_heading_close_wall_count",
                "lidar_correction_axis_heading_far_wall_count",
                "lidar_correction_axis_heading_close_contribution",
                "lidar_correction_axis_heading_far_contribution",
                "lidar_correction_axis_heading_contributions",
                "lidar_correction_axis_wall_observations",
                "lidar_correction_axis_trend_x_decision",
                "lidar_correction_axis_trend_x_reject_reason",
                "lidar_correction_axis_trend_x_count",
                "lidar_correction_axis_trend_x_active_count",
                "lidar_correction_axis_trend_x_sign_ratio",
                "lidar_correction_axis_trend_x_median",
                "lidar_correction_axis_trend_x_std",
                "lidar_correction_axis_trend_x_age_ms",
                "lidar_correction_axis_trend_x_authority_median",
                "lidar_correction_axis_trend_x_wall_count_median",
                "lidar_correction_axis_trend_x_step_unclamped",
                "lidar_correction_axis_trend_x_step",
                "lidar_correction_axis_trend_y_decision",
                "lidar_correction_axis_trend_y_reject_reason",
                "lidar_correction_axis_trend_y_count",
                "lidar_correction_axis_trend_y_active_count",
                "lidar_correction_axis_trend_y_sign_ratio",
                "lidar_correction_axis_trend_y_median",
                "lidar_correction_axis_trend_y_std",
                "lidar_correction_axis_trend_y_age_ms",
                "lidar_correction_axis_trend_y_authority_median",
                "lidar_correction_axis_trend_y_wall_count_median",
                "lidar_correction_axis_trend_y_step_unclamped",
                "lidar_correction_axis_trend_y_step",
                "lidar_correction_axis_trend_h_decision",
                "lidar_correction_axis_trend_h_reject_reason",
                "lidar_correction_axis_trend_h_count",
                "lidar_correction_axis_trend_h_active_count",
                "lidar_correction_axis_trend_h_sign_ratio",
                "lidar_correction_axis_trend_h_median",
                "lidar_correction_axis_trend_h_std",
                "lidar_correction_axis_trend_h_age_ms",
                "lidar_correction_axis_trend_h_authority_median",
                "lidar_correction_axis_trend_h_wall_count_median",
                "lidar_correction_axis_trend_h_step_unclamped",
                "lidar_correction_axis_trend_h_step",
                "lidar_correction_validation_decision",
                "lidar_correction_validation_reject_reason",
                "lidar_correction_validation_component_decision",
                "lidar_correction_validation_component_reject_reason",
                "lidar_correction_validation_pre_xy_m",
                "lidar_correction_validation_post_xy_m",
                "lidar_correction_validation_improvement_xy_m",
                "lidar_correction_validation_pre_h_deg",
                "lidar_correction_validation_post_h_deg",
                "lidar_correction_validation_improvement_h_deg",
                "lidar_correction_validation_post_confidence",
                "lidar_correction_validation_post_valid",
                "lidar_correction_validation_points",
                "lidar_correction_validation_applied_step_x",
                "lidar_correction_validation_applied_step_y",
                "lidar_correction_validation_applied_step_h",
                "lidar_correction_validation_async_enabled",
                "lidar_correction_validation_async_pending",
                "lidar_correction_validation_async_request_id",
                "lidar_correction_validation_async_applied_id",
                "lidar_correction_validation_async_generation",
                "lidar_correction_validation_async_submit_status",
                "lidar_correction_validation_async_source",
                "lidar_correction_validation_async_submitted_request_id",
                "lidar_correction_validation_async_anchor_version",
                "lidar_correction_validation_async_submit_scan_t",
                "lidar_correction_validation_async_submit_scan_age_ms",
                "lidar_correction_validation_async_result_scan_t",
                "lidar_correction_validation_async_result_anchor_version",
                "lidar_correction_validation_async_current_anchor_version",
                "lidar_correction_validation_async_anchor_version_delta",
                "lidar_correction_validation_async_dropped_this_submit",
                "lidar_correction_validation_async_dropped_requests",
                "lidar_correction_validation_async_stale_results",
                "lidar_correction_validation_async_errors",
                "lidar_correction_validation_async_wall_ms",
                "lidar_correction_validation_async_process_cpu_ms",
                "lidar_correction_validation_async_thread_cpu_ms",
                "lidar_correction_validation_async_age_ms",
                "lidar_correction_validation_async_last_wall_ms",
                "lidar_correction_validation_async_last_age_ms",
                "lidar_correction_validation_async_last_decision",
                "lidar_correction_validation_async_last_source",
                "lidar_correction_intended_step_h",
                "lidar_correction_actual_anchor_delta_h",
                "lidar_correction_unintended_anchor_delta_h",
                "lidar_correction_reference_local_h",
                "lidar_correction_apply_local_h",
                "lidar_correction_reference_pose_h",
                "lidar_correction_anchor_h_before",
                "lidar_correction_anchor_h_after",
                "lidar_correction_reference_local_source",
                "lidar_correction_diagnostic_status_mode",
                "lidar_correction_diagnostic_status_configured_hz",
                "lidar_correction_diagnostic_status_important",
                "lidar_correction_diagnostic_status_deferred_since_full",
                "lidar_correction_diagnostic_status_full_count",
                "lidar_correction_diagnostic_status_light_count",
                "lidar_correction_diagnostic_status_last_full_age_ms",
                "lidar_scan_arrival_rate_hz",
                "lidar_scan_arrival_rate_1s_hz",
                "lidar_scan_arrival_rate_3s_hz",
                "lidar_scan_arrival_period_ms",
                "lidar_process_ms",
                "lidar_process_rate_hz",
                "lidar_process_rate_1s_hz",
                "lidar_process_rate_3s_hz",
                "lidar_process_max_1s_ms",
                "lidar_process_max_3s_ms",
                "lidar_stage_filter_ms",
                "lidar_stage_deskew_ms",
                "lidar_stage_snapshot_ms",
                "lidar_stage_localization_correction_ms",
                "lidar_stage_pose_estimate_ms",
                "lidar_stage_obstacles_ms",
                "lidar_stage_color_enqueue_ms",
                "lidar_stage_color_tracks_ms",
                "lidar_stage_runtime_update_ms",
                "obstacle_known_before_ms",
                "obstacle_known_before_process_cpu_ms",
                "obstacle_known_before_thread_cpu_ms",
                "obstacle_detection_allowed_ms",
                "obstacle_detection_allowed_process_cpu_ms",
                "obstacle_detection_allowed_thread_cpu_ms",
                "obstacle_detect_slots_ms",
                "obstacle_detect_slots_process_cpu_ms",
                "obstacle_detect_slots_thread_cpu_ms",
                "obstacle_track_update_ms",
                "obstacle_track_update_process_cpu_ms",
                "obstacle_track_update_thread_cpu_ms",
                "obstacle_expected_visible_ms",
                "obstacle_expected_visible_process_cpu_ms",
                "obstacle_expected_visible_thread_cpu_ms",
                "obstacle_absence_update_ms",
                "obstacle_absence_update_process_cpu_ms",
                "obstacle_absence_update_thread_cpu_ms",
                "obstacle_memory_refresh_ms",
                "obstacle_finalize_results_ms",
                "obstacle_finalize_results_process_cpu_ms",
                "obstacle_finalize_results_thread_cpu_ms",
                "runtime_perception_publish_ms",
                "runtime_perception_publish_process_cpu_ms",
                "runtime_perception_publish_thread_cpu_ms",
                "runtime_cloud_wall_ms",
                "runtime_cloud_wall_process_cpu_ms",
                "runtime_cloud_wall_thread_cpu_ms",
                "runtime_free_race_inner_ms",
                "runtime_free_race_inner_process_cpu_ms",
                "runtime_free_race_inner_thread_cpu_ms",
                "runtime_shared_publish_ms",
                "runtime_shared_publish_process_cpu_ms",
                "runtime_shared_publish_thread_cpu_ms",
                "runtime_publish_obstacle_copy_ms",
                "runtime_publish_scan_points_copy_ms",
                "runtime_publish_matched_points_copy_ms",
                "runtime_publish_rejected_points_copy_ms",
                "runtime_publish_cloud_wall_copy_ms",
                "runtime_publish_free_race_copy_ms",
                "runtime_publish_lock_assign_ms",
                "obstacle_detect_slot_generation_ms",
                "obstacle_detect_pre_arrays_ms",
                "obstacle_detect_distance_matrix_ms",
                "obstacle_detect_distance_index_group_ms",
                "obstacle_detect_distance_hit_count",
                "obstacle_detect_loop_ms",
                "obstacle_detect_distance_ms",
                "obstacle_detect_cluster_ms",
                "obstacle_detect_result_ms",
                "obstacle_detect_empty_result_ms",
                "obstacle_detect_input_points",
                "obstacle_detect_slot_count",
                "obstacle_detect_nonempty_slots",
                "obstacle_detect_current_slots",
                "obstacle_expected_visible_filter_ms",
                "obstacle_expected_visible_barrier_occlusion_ms",
                "obstacle_expected_visible_obstacle_occlusion_ms",
                "obstacle_expected_visible_count",
                "obstacle_expected_visible_barrier_checks",
                "obstacle_expected_visible_obstacle_checks",
                "obstacle_expected_visible_range_rejected",
                "obstacle_expected_visible_rear_rejected",
                "obstacle_expected_visible_barrier_rejected",
                "obstacle_expected_visible_obstacle_rejected",
                "absence_pending_obstacles",
                "color_points_prepare_ms",
                "color_debug_meta_ms",
                "color_drain_results_ms",
                "color_enqueue_clusters_ms",
                "color_pending_count_ms",
                "color_observation_map_ms",
                "color_measurement_buffer_ms",
                "color_enqueue_request_count",
                "color_sync_analysis_count",
                "color_event_slot_count",
                "lidar_stage_payload_build_ms",
                "web_payload_refresh_ms",
                "web_payload_refresh_rate_1s_hz",
                "web_payload_refresh_rate_3s_hz",
                "web_payload_cached_ms",
                "web_payload_cached_rate_1s_hz",
                "lite_payload_ms",
                "lite_payload_rate_1s_hz",
                "lite_payload_rate_3s_hz",
                "lite_payload_json_ms",
                "lite_payload_json_rate_1s_hz",
                "lite_payload_json_rate_3s_hz",
                "web_payload_refresh_control_mode",
                "web_payload_refresh_control_configured_hz",
                "web_payload_refresh_control_cached_count",
                "web_payload_refresh_control_last_full_age_ms",
                "lidar_runtime_payload_ms",
                "control_payload_refresh_mode",
                "control_payload_refresh_configured_hz",
                "control_payload_refresh_skipped_since_full",
                "control_payload_refresh_last_full_age_ms",
                "control_payload_light_ms",
                "control_payload_light_rate_1s_hz",
                "control_payload_light_rate_3s_hz",
                "control_payload_full_mode",
                "control_payload_full_updates",
                "control_payload_light_updates",
                "control_payload_skipped_updates",
                "lidar_correction_status_update_ms",
                "lidar_correction_status_update_rate_1s_hz",
                "lidar_correction_status_publish_mode",
                "lidar_correction_status_publish_configured_hz",
                "lidar_correction_status_publish_deferred_since_full",
                "lidar_correction_status_publish_important",
                "lidar_status_mode",
                "lidar_status_reason",
                "lidar_status_total_ms",
                "lidar_status_update_lock_wait_ms",
                "lidar_status_update_lock_inside_ms",
                "lidar_status_record_timing_ms",
                "lidar_status_record_metric_ms",
                "scheduler_probe_enabled",
                "scheduler_probe_late_ms",
                "scheduler_probe_late_p50_1s_ms",
                "scheduler_probe_late_p95_1s_ms",
                "scheduler_probe_late_max_1s_ms",
                "scheduler_probe_late_p50_3s_ms",
                "scheduler_probe_late_p95_3s_ms",
                "scheduler_probe_late_max_3s_ms",
                "scheduler_probe_rate_hz",
                "lidar_points_raw",
                "lidar_points_after_range_filter",
                "lidar_points_kept",
                "lidar_points_dropped_by_max_points",
                "lidar_timestamp_scan_start_age_ms",
                "lidar_timestamp_scan_mid_age_ms",
                "lidar_timestamp_scan_end_age_ms",
                "lidar_timestamp_scan_duration_ms",
                "lidar_timestamp_source",
                "lidar_beam_time_order",
                "nav_pose_confidence_ok",
                "nav_pose_confidence_progress_allowed",
                "nav_pose_confidence_progress_reason",
                "nav_pose_confidence",
                "nav_projection_error_m",
                "nav_lateral_m",
                "nav_lateral_fused_m",
                "nav_lateral_otos_only_m",
                "nav_lateral_fused_minus_otos_m",
                "nav_heading_error_deg",
                "nav_enabled",
                "nav_confirmations_active",
                "nav_confirmations_active_reason",
                "nav_timing_warning",
                "nav_full_gate_lock",
                "nav_locked_gate_count",
                "nav_local_connector_mode",
                "nav_blocked_route_fallback_active",
                "nav_blocked_route_fallback_age_s",
                "nav_blocked_route_fallback_reason",
                "nav_blocked_route_fallback_projection_error_m",
                "nav_warnings",
                "nav_constraint_warnings",
                "nav_target_waypoint_ids",
                "nav_target_waypoint_reasons",
                "nav_active_waypoint_id",
                "nav_active_waypoint_reason",
                "nav_active_waypoint_distance_m",
                "nav_active_waypoint_signed_forward_m",
                "nav_connector_target_id",
                "nav_connector_target_reason",
                "nav_connector_chord_m",
                "nav_connector_heading_delta_deg",
                "nav_connector_target_route_delta_m",
                "nav_connector_start_lateral_m",
                "nav_connector_start_heading_error_deg",
                "nav_connector_attempt_count",
                "nav_connector_failure_reasons",
                "nav_connector_attempts",
                "nav_fixed_route_failed",
                "nav_fixed_route_warnings",
                "nav_extension_warnings",
                "nav_planner_diagnostics",
                "nav_speed_reason",
                "nav_speed_cap_mps",
                "nav_speed_policy",
                "nav_speed_policy_base_mps",
                "nav_speed_policy_cap_factor",
                "nav_speed_policy_cap_reason",
                "nav_speed_policy_next_obstacle_id",
                "nav_speed_policy_next_obstacle_delta_m",
                "nav_speed_policy_next_obstacle_color",
                "nav_speed_policy_next_obstacle_gate_locked",
                "nav_speed_policy_current_section",
                "nav_speed_policy_current_section_ready",
                "nav_speed_policy_current_section_has_obstacle",
                "nav_speed_policy_current_section_obstacle_id",
                "nav_speed_policy_current_section_obstacle_color",
                "nav_speed_policy_next_section",
                "nav_speed_policy_next_section_ready",
                "nav_speed_policy_next_section_has_obstacle",
                "nav_speed_policy_next_section_obstacle_id",
                "nav_speed_policy_next_section_obstacle_color",
                "nav_obstacle_speed_lookahead_m",
                "nav_obstacle_speed_lookahead_forward_mps",
                "nav_obstacle_speed_lookahead_scale",
                "nav_tracking_error_speed_factor",
                "nav_tracking_error_speed_reason",
                "nav_corner_transition_speed_factor",
                "nav_corner_transition_speed_cap_mps",
                "nav_corner_transition_speed_active",
                "nav_corner_transition_speed_reason",
                "nav_hold",
                "nav_hold_reason",
                "nav_hold_section",
                "nav_hold_remaining_s",
                "localization_hold_active",
                "localization_hold_reason",
                "localization_hold_release_reason",
                "localization_hold_remaining_s",
                "localization_hold_min_remaining_s",
                "localization_hold_timeout_remaining_s",
                "localization_hold_stationary",
                "localization_hold_stationary_speed_mps",
                "localization_hold_wide_reacquire",
                "localization_hold_effective_axis_max_error_m",
                "localization_hold_base_axis_max_error_m",
                "localization_hold_missing_xy_count",
                "localization_hold_valid_xy_count",
                "localization_hold_settled_count",
                "localization_hold_x_measured",
                "localization_hold_y_measured",
                "localization_hold_x_residual_m",
                "localization_hold_y_residual_m",
                "localization_hold_x_step_m",
                "localization_hold_y_step_m",
                "localization_hold_trigger_source",
                "localization_hold_cooldown_remaining_s",
                "localization_hold_no_current_trigger_enabled",
                "localization_hold_no_current_triggered",
                "localization_hold_no_current_trigger_axis",
                "localization_hold_no_current_suppressed_reason",
                "localization_hold_no_current_window_count",
                "localization_hold_no_current_window_size",
                "localization_hold_no_current_required",
                "localization_hold_no_current_x_count",
                "localization_hold_no_current_y_count",
                "localization_hold_no_current_x_missing",
                "localization_hold_no_current_y_missing",
                "localization_hold_no_current_section_visit_key",
                "localization_hold_no_current_section_visit_count",
                "localization_hold_no_current_section_visit_limit",
                "localization_hold_no_current_event_count",
                "localization_hold_low_conf_trigger_enabled",
                "localization_hold_low_conf_triggered",
                "localization_hold_low_conf_suppressed_reason",
                "localization_hold_low_conf_window_count",
                "localization_hold_low_conf_window_size",
                "localization_hold_low_conf_required",
                "localization_hold_low_conf_low_count",
                "localization_hold_low_conf_last_confidence",
                "localization_hold_low_conf_last_valid",
                "localization_hold_low_conf_last_reason",
                "localization_hold_low_conf_section_visit_key",
                "localization_hold_low_conf_section_visit_count",
                "localization_hold_low_conf_section_visit_limit",
                "localization_hold_low_conf_event_count",
                "localization_hold_low_conf_startup_grace_remaining_s",
                "localization_hold_low_conf_progress_allowed",
                "localization_hold_low_conf_progress_reason",
                "localization_hold_low_conf_progress_started_t",
                "localization_hold_low_conf_progress_trigger_source",
                "localization_hold_timeout_progress_allowed",
                "localization_hold_timeout_progress_reason",
                "localization_hold_timeout_progress_started_t",
                "localization_hold_timeout_progress_trigger_source",
                "localization_hold_timeout_progress_clear_reason",
                "localization_hold_timeout_progress_clear_window_count",
                "localization_hold_timeout_progress_clear_window_size",
                "localization_hold_timeout_progress_clear_required",
                "localization_hold_timeout_progress_clear_settled_count",
                "nav_blocked",
                "nav_blocked_reason",
                "nav_observation_approach",
                "nav_detect_enabled",
                "obstacle_detection_yaw_gate_enabled",
                "obstacle_detection_yaw_rate_dps",
                "obstacle_detection_yaw_rate_limit_dps",
                "obstacle_detection_yaw_rate_gated",
                "obstacle_detection_yaw_gate_reason",
                "nav_direction",
                "nav_section",
                "free_race_inner_enabled",
                "free_race_inner_locked",
                "free_race_inner_route_active",
                "free_race_inner_start_section_active",
                "free_race_inner_applied_active",
                "free_race_inner_applied_summary",
                "free_race_inner_pending_summary",
                "free_race_inner_apply_generation",
                "free_race_inner_apply_reason",
                "free_race_inner_bottom_applied_lane",
                "free_race_inner_left_applied_lane",
                "free_race_inner_top_applied_lane",
                "free_race_inner_right_applied_lane",
                "free_race_inner_bottom_pending_lane",
                "free_race_inner_left_pending_lane",
                "free_race_inner_top_pending_lane",
                "free_race_inner_right_pending_lane",
                "free_race_inner_last_sample_reason",
                "free_race_inner_last_section",
                "free_race_inner_last_gap_m",
                "free_race_inner_last_left_m",
                "free_race_inner_last_right_m",
                "free_race_inner_last_left_count",
                "free_race_inner_last_right_count",
                "free_race_inner_last_scan_t",
                "free_race_inner_last_probe_mode",
                "free_race_inner_last_sample_source",
                "free_race_inner_summary",
                "free_race_inner_bottom_class",
                "free_race_inner_bottom_samples",
                "free_race_inner_bottom_far_count",
                "free_race_inner_bottom_close_count",
                "free_race_inner_bottom_median_gap_m",
                "free_race_inner_left_class",
                "free_race_inner_left_samples",
                "free_race_inner_left_far_count",
                "free_race_inner_left_close_count",
                "free_race_inner_left_median_gap_m",
                "free_race_inner_top_class",
                "free_race_inner_top_samples",
                "free_race_inner_top_far_count",
                "free_race_inner_top_close_count",
                "free_race_inner_top_median_gap_m",
                "free_race_inner_right_class",
                "free_race_inner_right_samples",
                "free_race_inner_right_far_count",
                "free_race_inner_right_close_count",
                "free_race_inner_right_median_gap_m",
                "nav_route_progress_m",
                "nav_route_step_delta_m",
                "nav_route_progress_mutations_allowed",
                "nav_route_progress_baseline_synced",
                "nav_route_progress_suppressed_reason",
                "nav_route_progress_suppressed_delta_m",
                "nav_gate_lock_last_event",
                "nav_gate_lock_last_section",
                "nav_gate_lock_last_gate_idx",
                "nav_gate_lock_last_lane_name",
                "nav_gate_lock_last_reason",
                "nav_gate_lock_last_decision_reason",
                "nav_gate_lock_last_obstacle_ids",
                "nav_gate_lock_last_route_progress_m",
                "nav_gate_lock_last_route_step_delta_m",
                "nav_gate_lock_last_active_waypoint_id",
                "nav_gate_lock_last_startup_status",
                "nav_gate_lock_last_startup_state",
                "nav_gate_lock_last_anchor_version",
                "nav_gate_lock_last_anchor_source",
                "nav_gate_lock_last_anchor_jump_m",
                "nav_gate_lock_last_anchor_jump_h_deg",
                "nav_gate_lock_last_initial_pose_locked",
                "nav_gate_lock_event_count",
                "nav_gate_lock_recent_events",
                "nav_gate_lock_suppressed_reason",
                "nav_party_mode",
                "nav_post_unpark_start_confirmation_reason",
                "nav_post_unpark_start_confirmation_triggered",
                "nav_post_unpark_start_confirmation_party_mode",
                "nav_post_unpark_start_confirmation_source",
                "nav_post_unpark_start_confirmation_obstacle_count",
                "nav_post_unpark_start_confirmation_relevant_count",
                "nav_post_unpark_start_confirmation_candidate_count",
                "nav_post_unpark_start_confirmation_slot",
                "nav_post_unpark_start_confirmation_slot_in_relevant",
                "nav_post_unpark_start_confirmation_slot_missing_from_relevant",
                "nav_post_unpark_start_confirmation_injected_candidate",
                "nav_post_unpark_start_confirmation_injected_candidate_reject_reason",
                "nav_active_waypoint_id",
                "nav_active_waypoint_reason",
                "nav_active_waypoint_passed_reason",
                "nav_active_waypoint_reset_reason",
                "nav_active_waypoint_reset_previous_id",
                "nav_color_confirmation_slot",
                "nav_color_confirmation_proposal",
                "nav_color_confirmation_mode",
                "nav_color_confirmation_trigger_source",
                "nav_color_confirmation_committed_start",
                "nav_color_confirmation_committed_at_decision",
                "nav_color_confirmation_decision_color",
                "nav_color_confirmation_decision_source",
                "nav_color_confirmation_decision_confidence",
                "nav_color_confirmation_decision_rejected",
                "nav_color_confirmation_decision_reject_reason",
                "nav_color_confirmation_sample_count",
                "nav_color_confirmation_all_sample_count",
                "nav_color_confirmation_red_count",
                "nav_color_confirmation_green_count",
                "nav_color_confirmation_other_count",
                "nav_color_confirmation_all_red_count",
                "nav_color_confirmation_all_green_count",
                "nav_color_confirmation_all_other_count",
                "nav_color_confirmation_attempt",
                "nav_color_confirmation_exposure_retry",
                "nav_color_confirmation_exposure_retry_candidate",
                "nav_color_confirmation_exposure_retry_reason",
                "nav_color_confirmation_exposure_retry_eval_reason",
                "nav_color_confirmation_exposure_retry_sample_count",
                "nav_color_confirmation_exposure_retry_min_samples",
                "nav_color_confirmation_exposure_retry_color_vote_count",
                "nav_color_confirmation_exposure_retry_color_ratio",
                "nav_color_confirmation_exposure_retry_required_vote_count",
                "nav_color_confirmation_exposure_retry_bright_vote_count",
                "nav_color_confirmation_exposure_retry_dark_vote_count",
                "nav_color_confirmation_exposure_retry_low_valid_count",
                "nav_color_confirmation_exposure_retry_current_gain",
                "nav_color_confirmation_exposure_retry_next_gain",
                "nav_color_confirmation_stationary",
                "nav_color_confirmation_stationary_speed_mps",
                "nav_color_confirmation_stationary_yaw_rate_dps",
                "nav_color_confirmation_sample_added",
                "nav_color_confirmation_sample_added_count",
                "nav_color_confirmation_active_obstacle_in_relevant",
                "nav_color_confirmation_active_obstacle_in_measurement_source",
                "nav_color_confirmation_active_obstacle_in_decision_source",
                "nav_color_confirmation_decision_obstacle_source",
                "nav_color_confirmation_measurement_source",
                "nav_color_confirmation_measurement_blocked_reason",
                "nav_color_confirmation_measurement_relevant_events_available",
                "nav_color_confirmation_measurement_full_events_available",
                "nav_color_confirmation_measurement_events_available",
                "nav_color_confirmation_measurement_events_added",
                "nav_color_confirmation_measurement_events_duplicate",
                "nav_color_confirmation_measurement_events_not_real",
                "nav_color_confirmation_measurement_events_stale",
                "nav_color_confirmation_measurement_events_before_hold",
                "nav_color_confirmation_measurement_events_gain_settle",
                "nav_color_confirmation_measurement_events_missing_id",
                "nav_color_confirmation_last_measurement_id",
                "nav_color_confirmation_last_sample_label",
                "nav_color_confirmation_last_skip_reason",
                "nav_color_confirmation_delta_m_at_decision",
                "nav_color_confirmation_los_distance_m_at_decision",
                "nav_color_confirmation_section",
                "nav_color_confirmation_gate_idx",
                "nav_color_confirmation_recovery_side_changed",
                "nav_color_confirmation_recovery_gate_locked",
                "nav_color_confirmation_post_unpark_reverse_suppressed",
                "nav_color_confirmation_recovery_decision_reason",
                "nav_color_confirmation_recovery_side_from",
                "nav_color_confirmation_recovery_side_to",
                "nav_color_confirmation_recovery_start_lane_m",
                "nav_color_confirmation_recovery_decided_lane_m",
                "nav_color_confirmation_recovery_lane_delta_m",
                "nav_color_confirmation_recovery_trigger_distance_m",
                "nav_color_confirmation_recovery_target_m",
                "nav_color_confirmation_recovery_reason",
                "close_color_recovery_state",
                "close_color_recovery_slot",
                "close_color_recovery_trigger_distance_m",
                "close_color_recovery_target_m",
                "close_color_recovery_requested_target_m",
                "close_color_recovery_safe_distance_m",
                "close_color_recovery_applied_target_m",
                "close_color_recovery_safety_clamped",
                "close_color_recovery_safety_suppressed",
                "close_color_recovery_safety_limiting_reason",
                "close_color_recovery_safety_limiting_distance_m",
                "close_color_recovery_safety_obstacle_count",
                "close_color_recovery_progress_m",
                "close_color_recovery_remaining_m",
                "close_color_recovery_slowdown_m",
                "close_color_recovery_stop_tolerance_m",
                "close_color_recovery_reason",
                "close_color_recovery_handoff_reset_reason",
                "close_color_recovery_handoff_reset_previous_id",
                "close_color_recovery_start_allowed",
                "close_color_recovery_start_suppressed_reason",
                "close_color_recovery_nav_blocked_reason",
                "close_color_recovery_enabled",
                "close_color_recovery_deadman_active",
                "close_color_recovery_deadman_fresh",
                "close_color_recovery_target_speed_mps",
                "planner_blocked_recovery_state",
                "planner_blocked_recovery_case_id",
                "planner_blocked_recovery_case_key",
                "planner_blocked_recovery_blocked_age_s",
                "planner_blocked_recovery_eligible",
                "planner_blocked_recovery_start_allowed",
                "planner_blocked_recovery_start_suppressed_reason",
                "planner_blocked_recovery_nav_blocked_reason",
                "planner_blocked_recovery_safe_distance_m",
                "planner_blocked_recovery_straight_safe_distance_m",
                "planner_blocked_recovery_target_m",
                "planner_blocked_recovery_progress_m",
                "planner_blocked_recovery_remaining_m",
                "planner_blocked_recovery_heading_offset_deg",
                "planner_blocked_recovery_path_h",
                "planner_blocked_recovery_limiting_reason",
                "planner_blocked_recovery_limiting_distance_m",
                "planner_blocked_recovery_selected_candidate",
                "planner_blocked_recovery_obstacle_count",
                "planner_blocked_recovery_candidate_count",
                "planner_blocked_recovery_target_speed_mps",
                "planner_blocked_recovery_deadman_active",
                "planner_blocked_recovery_deadman_fresh",
                "planner_blocked_recovery_parking_exclusion",
                "planner_blocked_recovery_post_unpark_elapsed_s",
                "planner_blocked_recovery_post_unpark_progress_m",
                "planner_blocked_recovery_handoff_reset_reason",
                "planner_blocked_recovery_handoff_reset_previous_id",
                "planner_blocked_recovery_exit_reason",
                "nav_smart_parking_alignment_active",
                "nav_smart_parking_alignment_splice_active",
                "nav_smart_parking_route_geometry_active",
                "nav_smart_parking_alignment_handoff_gate",
                "nav_smart_parking_alignment_source_gate",
                "nav_smart_parking_alignment_remaining_to_handoff_m",
                "nav_smart_parking_alignment_remaining_to_stage_m",
                "startup_status",
                "startup_reason",
                "startup_source",
                "startup_left_m",
                "startup_right_m",
                "startup_front_m",
                "startup_front_lateral_m",
                "startup_left_close",
                "startup_right_close",
                "startup_front_close",
                "startup_parking_roi_votes",
                "startup_parking_roi_required",
                "startup_race_mode",
                "startup_direction",
                "race_mode",
                "race_phase",
                "race_warning",
                "race_run_start_t",
                "race_initialization_complete_t",
                "race_unpark_start_t",
                "race_navigation_start_t",
                "race_initialization_time_s",
                "race_unpark_time_s",
                "race_lap1_time_s",
                "race_lap2_time_s",
                "race_obstacle_lap3_to_stop_s",
                "race_free_lap3_to_stationary_s",
                "race_parking_stop_time_s",
                "race_parking_duration_s",
                "race_total_to_parked_s",
                "race_total_to_free_stationary_s",
                "race_ranking_time_s",
                "race_full_gate_pass_count",
                "unpark_state",
                "unpark_reason",
                "unpark_start_mode",
                "unpark_reverse_first_active",
                "unpark_reverse_first_target_m",
                "unpark_reverse_first_progress_m",
                "unpark_reverse_first_stop_reason",
                "unpark_anchor_phase",
                "unpark_anchor_cause",
                "unpark_anchor_arm_timed_out",
                "unpark_heading_progress_deg",
                "unpark_exit_heading_reached",
                "unpark_rear_clearance_m",
                "unpark_reverse_room_m",
                "unpark_reverse_stop_clearance_m",
                "unpark_reverse_progress_m",
                "unpark_reverse_target_m",
                "unpark_reverse_commit_active",
                "unpark_reverse_commit_remaining_s",
                "unpark_reverse_stop_reason",
                "unpark_forward_direct_duty",
                "unpark_forward_direct_age_s",
                "unpark_anchor_release_brake_level",
                "unpark_anchor_release_age_s",
                "unpark_reverse_direct_duty",
                "unpark_reverse_arm_speed_mps",
                "unpark_reverse_arm_stable_s",
                "unpark_reverse_arm_ready",
                "unpark_reverse_arm_last_count",
                "unpark_retry_count",
                "unpark_passive_color_enabled",
                "unpark_passive_color_active",
                "unpark_passive_color_reason",
                "unpark_passive_color_state",
                "unpark_passive_color_stationary",
                "unpark_passive_color_speed_mps",
                "unpark_passive_color_yaw_rate_dps",
                "unpark_passive_color_candidate_count",
                "unpark_passive_color_committed_slot",
                "unpark_passive_color_committed_color",
                "unpark_passive_color_committed_confidence",
                "unpark_passive_color_last_slot",
                "unpark_passive_color_last_measurement_id",
                "unpark_passive_color_last_sample_label",
                "unpark_passive_color_last_skip_reason",
                "unpark_passive_color_sample_count",
                "unpark_passive_color_red_count",
                "unpark_passive_color_green_count",
                "unpark_passive_color_other_count",
                "unpark_passive_color_measurement_events_available",
                "unpark_passive_color_measurement_events_added",
                "unpark_passive_color_measurement_events_duplicate",
                "unpark_passive_color_measurement_events_not_real",
                "unpark_passive_color_measurement_events_stale",
                "unpark_passive_color_measurement_events_before_window",
                "unpark_passive_color_measurement_events_outside_stationary",
                "unpark_passive_color_measurement_events_missing_id",
                "unpark_passive_color_stationary_window_start_t",
                "unpark_passive_color_stationary_epoch_count",
                "unpark_passive_color_stationary_epoch_active",
                "unpark_passive_color_stationary_epoch_last_start_t",
                "unpark_passive_color_stationary_epoch_last_end_t",
                "unpark_passive_color_stationary_epoch_grace_active",
                "smart_parking_state",
                "smart_parking_reason",
                "smart_parking_direction",
                "smart_parking_heading_error_deg",
                "smart_parking_distance_to_target_m",
                "smart_parking_lap_stop_route_steering_active",
                "smart_parking_lap_stop_measured_speed_mps",
                "smart_parking_lap_stop_yaw_rate_dps",
                "smart_parking_lap_stop_route_steering_norm",
                "smart_parking_lap_stop_steering_stationary_latched",
                "smart_parking_lap_stop_steering_rearm_count",
                "smart_parking_lap_stop_steering_rearm_required",
                "smart_parking_lap_stop_steering_rearm_speed_mps",
                "smart_parking_lap_stop_steering_rearm_yaw_rate_dps",
                "smart_parking_stage_x",
                "smart_parking_slot_center_x",
                "smart_parking_final_rear_y",
                "smart_parking_final_push_elapsed_s",
                "smart_parking_final_push_direct_duty",
                "smart_parking_final_push_actuated",
                "db_mode",
                "db_submode",
                "db_target_user_mps",
                "db_target_ramped_mps",
                "db_measured_mps",
                "db_encoder_mps",
                "db_feedback_mps",
                "db_feedback_source",
                "db_feedback_age_ms",
                "db_external_feedback_mps",
                "db_external_feedback_filtered_mps",
                "db_external_feedback_age_ms",
                "db_rate_mps",
                "db_position_m",
                "db_count",
                "db_last_duty",
                "db_actuating",
                "db_creep_breakaway_enabled",
                "db_breakaway_allowed",
                "db_breakaway_enabled",
                "db_breakaway_active",
                "db_breakaway_max_speed_mps",
                "db_creep_threshold_mps",
                "db_target_direction",
                "db_direction_guard_clamped",
                "db_raw_duty_before_direction_guard",
                "db_runtime_max_command_duty",
                "db_speed_ff_target_duty_abs",
                "db_speed_ff_correction_clamp",
                "db_speed_ff_duty",
                "db_speed_pi_trim_duty",
                "db_speed_error_mps",
                "db_speed_ff_trim_kp",
                "db_speed_ff_trim_ki",
                "db_velocity_duty_slew_per_s",
                "db_internal_loop_ms",
                "db_internal_loop_overrun_ms",
                "db_internal_apply_output_ms",
                "db_internal_loop_t",
                "db_steering_desired_norm",
                "db_steering_applied_norm",
                "db_steering_desired_seq",
                "db_steering_applied_seq",
                "db_steering_pending",
                "db_steering_pending_age_ms",
                "db_steering_apply_age_ms",
                "db_steering_apply_duration_ms",
                "db_steering_apply_pwm_ms",
                "db_steering_last_error",
                "db_motion_applied_mode",
                "db_motion_applied_duty",
                "db_motion_applied_brake_level",
                "db_motion_desired_seq",
                "db_motion_applied_seq",
                "db_motion_pending",
                "db_motion_pending_age_ms",
                "db_motion_apply_age_ms",
                "db_motion_apply_duration_ms",
                "db_motion_apply_pwm_ms",
                "db_motion_last_error",
                "db_led_pending",
                "db_led_pending_age_ms",
                "db_led_apply_age_ms",
                "db_led_apply_duration_ms",
                "db_led_apply_pwm_ms",
                "db_led_last_error",
                "drive_pwm_mode",
                "drive_pwm_reverse",
                "drive_pwm_forward",
                "drive_pwm_req_reverse",
                "drive_pwm_req_forward",
                "drive_pwm_read_reverse",
                "drive_pwm_read_forward",
                "drive_pwm_readback_enabled",
                "drive_pwm_throttle",
                "drive_pwm_brake_level",
                "drive_pwm_brake_order",
                "drive_pwm_command_t",
                "drive_pwm_read_t",
                "drive_pwm_command_age_ms",
                "drive_pwm_read_age_ms",
                "drive_pwm_age_ms",
                "drive_pwm_exact_dedupe_enabled",
                "drive_pwm_cache_fds_enabled",
                "drive_pwm_cached_fd_count",
                "drive_pwm_write_count_total",
                "drive_pwm_skip_count_total",
                "drive_pwm_cache_fallback_count_total",
                "drive_pwm_cache_open_count_total",
                "drive_pwm_cache_close_count_total",
                "drive_pwm_write_count_servo",
                "drive_pwm_skip_count_servo",
                "drive_pwm_cache_fallback_count_servo",
                "drive_pwm_write_count_front_led",
                "drive_pwm_skip_count_front_led",
                "drive_pwm_cache_fallback_count_front_led",
                "drive_pwm_write_count_drive_reverse",
                "drive_pwm_skip_count_drive_reverse",
                "drive_pwm_cache_fallback_count_drive_reverse",
                "drive_pwm_write_count_drive_forward",
                "drive_pwm_skip_count_drive_forward",
                "drive_pwm_cache_fallback_count_drive_forward",
            ]
            self.drive_log_writer = csv.DictWriter(self.drive_log_file, fieldnames=fields, extrasaction="ignore")
            self.drive_log_writer.writeheader()
            self.drive_log_last_flush_t = time.monotonic()
            self.drive_log_queue = queue.Queue(maxsize=max(100, int(float(getattr(self.args, "drive_control_rate", 75.0)) * 10.0)))
            self.drive_log_stop.clear()
            self.drive_log_thread = threading.Thread(target=self._drive_debug_log_worker, daemon=True)
            self.drive_log_thread.start()
            print(f"Drive debug log: {log_path}")
        except Exception:
            traceback.print_exc()
            self.drive_log_file = None
            self.drive_log_writer = None
            self.drive_log_queue = None
            self.drive_log_thread = None

    def _drive_debug_log_worker(self) -> None:
        while True:
            try:
                item = self.drive_log_queue.get(timeout=0.1) if self.drive_log_queue is not None else None
            except queue.Empty:
                if self.drive_log_stop.is_set():
                    break
                continue
            if item is None:
                break
            writer = self.drive_log_writer
            if writer is None:
                continue
            try:
                writerow_start = time.monotonic()
                writer.writerow(item)
                self.last_drive_writerow_ms = f"{(time.monotonic() - writerow_start) * 1000.0:.3f}"
                now = time.monotonic()
                if now - self.drive_log_last_flush_t >= 0.25 and self.drive_log_file is not None:
                    flush_start = time.monotonic()
                    self.drive_log_file.flush()
                    self.last_drive_flush_ms = f"{(time.monotonic() - flush_start) * 1000.0:.3f}"
                    self.last_drive_flush_requested = True
                    self.drive_log_last_flush_t = now
                else:
                    self.last_drive_flush_ms = ""
                    self.last_drive_flush_requested = False
            except Exception:
                traceback.print_exc()
        try:
            if self.drive_log_file is not None:
                self.drive_log_file.flush()
        except Exception:
            pass

    def _close_drive_debug_log(self) -> None:
        if self.drive_log_file is None:
            return
        try:
            self.drive_log_stop.set()
            if self.drive_log_queue is not None:
                try:
                    self.drive_log_queue.put_nowait(None)
                except queue.Full:
                    pass
            if self.drive_log_thread is not None:
                self.drive_log_thread.join(timeout=1.0)
            self.drive_log_file.flush()
            self.drive_log_file.close()
        except Exception:
            pass
        self.drive_log_file = None
        self.drive_log_writer = None
        self.drive_log_queue = None
        self.drive_log_thread = None

    def _write_drive_debug_log(self, loop_start: float, cmd: dict, telemetry: dict, after_command_t: Optional[float] = None) -> None:
        if self.drive_log_writer is None or self.drive_log_queue is None:
            return
        try:
            log_build_start = time.monotonic()
            log_t = time.monotonic()
            after_command_t = loop_start if after_command_t is None else float(after_command_t)
            unpark = cmd.get("unpark")
            if not isinstance(unpark, dict):
                unpark = {}
            passive_unpark_color = unpark.get("passive_color_confirmation", {})
            if not isinstance(passive_unpark_color, dict):
                passive_unpark_color = {}
            smart_parking = cmd.get("smart_parking")
            if not isinstance(smart_parking, dict):
                smart_parking = {}
            deadman = cmd.get("deadman")
            if not isinstance(deadman, dict):
                deadman = {}
            race_timing = {}
            nav = {}
            obstacles = {}
            startup = {}
            competition_start = {}
            payload = {}
            timings = {}
            try:
                race_timing = self.perception.shared.race_timing_snapshot()
                payload = self.perception.shared.cached_payload()
                timings = self.perception.shared.timing_snapshot()
                if isinstance(payload, dict):
                    maybe_nav = payload.get("navigation", {})
                    if isinstance(maybe_nav, dict):
                        nav = maybe_nav
                    maybe_obstacles = payload.get("obstacles", {})
                    if isinstance(maybe_obstacles, dict):
                        obstacles = maybe_obstacles
                    maybe_competition = payload.get("competition_start", {})
                    if isinstance(maybe_competition, dict):
                        competition_start = maybe_competition
                    init = payload.get("initial_localization", {})
                    if isinstance(init, dict):
                        classifier = init.get("start_classifier", {})
                        startup = dict(classifier) if isinstance(classifier, dict) else {}
                        startup["status"] = init.get("status", "")
                        startup["source"] = init.get("source", "")
                        startup["reason"] = init.get("reason", startup.get("reason", ""))
                        startup["race_mode"] = init.get("race_mode", startup.get("race_mode", ""))
            except Exception:
                payload = {}
                nav = {}
                obstacles = {}
                startup = {}
                competition_start = {}
                race_timing = {}
                timings = {}
            if not isinstance(payload, dict):
                payload = {}
            if not isinstance(timings, dict):
                timings = {}
            try:
                pose, _scan, anchor, anchor_version, anchor_source, _init, correction_status = self.perception.snapshot()
                display_pose = local_to_display_pose(pose, anchor)
                try:
                    with self.perception.shared.lock:
                        otos_base_anchor = self.perception.shared.otos_base_anchor
                    otos_only_pose = local_to_display_pose(pose, otos_base_anchor)
                    fused_minus_otos_x = float(display_pose.x) - float(otos_only_pose.x)
                    fused_minus_otos_y = float(display_pose.y) - float(otos_only_pose.y)
                    fused_minus_otos_h = ((float(display_pose.h) - float(otos_only_pose.h) + 180.0) % 360.0) - 180.0
                    fused_minus_otos_distance = math.hypot(fused_minus_otos_x, fused_minus_otos_y)
                except Exception:
                    otos_only_pose = None
                    fused_minus_otos_x = ""
                    fused_minus_otos_y = ""
                    fused_minus_otos_h = ""
                    fused_minus_otos_distance = ""
                otos_speed = math.hypot(float(pose.vx), float(pose.vy))
                otos_forward_mps = self._otos_forward_speed(pose)
                display_speed = math.hypot(float(display_pose.vx), float(display_pose.vy))
                otos_age_ms = (loop_start - float(pose.t)) * 1000.0 if getattr(pose, "t", 0.0) else ""
            except Exception:
                pose = None
                display_pose = None
                otos_only_pose = None
                fused_minus_otos_x = ""
                fused_minus_otos_y = ""
                fused_minus_otos_h = ""
                fused_minus_otos_distance = ""
                anchor = None
                anchor_version = ""
                anchor_source = ""
                correction_status = {}
                otos_speed = ""
                otos_forward_mps = ""
                display_speed = ""
                otos_age_ms = ""
            anchor_payload = payload.get("anchor", {}) if isinstance(payload, dict) else {}
            if not isinstance(anchor_payload, dict):
                anchor_payload = {}
            anchor_last = anchor_payload.get("last_update", {})
            if not isinstance(anchor_last, dict):
                anchor_last = {}
            otos_jump = {}
            localization_hold = {}
            try:
                with self.perception.shared.lock:
                    anchor_last = dict(self.perception.shared.anchor_diagnostics)
                    otos_jump = dict(getattr(self.perception.shared, "otos_jump_status", {}) or {})
                    localization_hold = dict(getattr(self.perception.shared, "localization_hold", {}) or {})
            except Exception:
                pass
            lidar_pose = payload.get("lidar_pose", {}) if isinstance(payload, dict) else {}
            if not isinstance(lidar_pose, dict):
                lidar_pose = {}
            lidar_correction = payload.get("lidar_correction", {}) if isinstance(payload, dict) else {}
            if not isinstance(lidar_correction, dict):
                lidar_correction = {}
            if isinstance(correction_status, dict):
                lidar_correction = dict(correction_status)
            lidar_cloud_wall = payload.get("lidar_cloud_wall_metrics", {}) if isinstance(payload, dict) else {}
            if not isinstance(lidar_cloud_wall, dict):
                lidar_cloud_wall = {}

            def csv_seq(value):
                if value is None:
                    return ""
                if isinstance(value, (list, tuple, set)):
                    return " ".join(str(item) for item in value)
                return str(value)

            free_finish = nav.get("free_race_finish_trigger", {}) if isinstance(nav, dict) else {}
            if not isinstance(free_finish, dict):
                free_finish = {}
            smart_trigger = nav.get("smart_parking_trigger", {}) if isinstance(nav, dict) else {}
            if not isinstance(smart_trigger, dict):
                smart_trigger = {}
            free_race_inner = payload.get("free_race_inner_config", {}) if isinstance(payload, dict) else {}
            if not isinstance(free_race_inner, dict):
                free_race_inner = {}
            free_race_inner_route = nav.get("free_race_inner_route", {}) if isinstance(nav, dict) else {}
            if not isinstance(free_race_inner_route, dict):
                free_race_inner_route = {}
            free_race_inner_sections = free_race_inner.get("sections", {})
            if not isinstance(free_race_inner_sections, dict):
                free_race_inner_sections = {}
            free_race_inner_applied_sections = free_race_inner_route.get("applied_sections", {})
            if not isinstance(free_race_inner_applied_sections, dict):
                free_race_inner_applied_sections = {}
            free_race_inner_pending_sections = free_race_inner_route.get("pending_sections", {})
            if not isinstance(free_race_inner_pending_sections, dict):
                free_race_inner_pending_sections = {}

            def free_inner_section(section, key):
                item = free_race_inner_sections.get(section, {})
                return item.get(key, "") if isinstance(item, dict) else ""

            def free_inner_route_section(plan, section, key):
                item = plan.get(section, {}) if isinstance(plan, dict) else {}
                return item.get(key, "") if isinstance(item, dict) else ""

            close_color_recovery = cmd.get("close_color_recovery", {}) if isinstance(cmd, dict) else {}
            if not isinstance(close_color_recovery, dict):
                close_color_recovery = {}
            if not close_color_recovery:
                try:
                    close_color_recovery = self.perception.shared.close_color_recovery_snapshot()
                except Exception:
                    close_color_recovery = {}
            if not isinstance(close_color_recovery, dict):
                close_color_recovery = {}
            planner_blocked_recovery = cmd.get("planner_blocked_recovery", {}) if isinstance(cmd, dict) else {}
            if not isinstance(planner_blocked_recovery, dict):
                planner_blocked_recovery = {}
            if not planner_blocked_recovery:
                try:
                    planner_blocked_recovery = self.perception.shared.planner_blocked_recovery_snapshot()
                except Exception:
                    planner_blocked_recovery = {}
            if not isinstance(planner_blocked_recovery, dict):
                planner_blocked_recovery = {}
            color_confirmation = nav.get("color_confirmation", {}) if isinstance(nav, dict) else {}
            if not isinstance(color_confirmation, dict):
                color_confirmation = {}
            color_counts = color_confirmation.get("counts", {})
            if not isinstance(color_counts, dict):
                color_counts = {}
            color_all_counts = color_confirmation.get("all_counts", {})
            if not isinstance(color_all_counts, dict):
                color_all_counts = {}
            post_unpark_start_confirmation = color_confirmation.get("post_unpark_start_confirmation", {})
            if not isinstance(post_unpark_start_confirmation, dict):
                post_unpark_start_confirmation = {}
            pwm_t = telemetry.get("drive_pwm_t", "")
            pwm_command_t = telemetry.get("drive_pwm_command_t", pwm_t)
            pwm_read_t = telemetry.get("drive_pwm_read_t", "")
            try:
                pwm_age_ms = (loop_start - float(pwm_t)) * 1000.0 if pwm_t != "" else ""
            except (TypeError, ValueError):
                pwm_age_ms = ""
            try:
                pwm_command_age_ms = (log_t - float(pwm_command_t)) * 1000.0 if pwm_command_t != "" else ""
            except (TypeError, ValueError):
                pwm_command_age_ms = ""
            try:
                pwm_read_age_ms = (log_t - float(pwm_read_t)) * 1000.0 if pwm_read_t != "" else ""
            except (TypeError, ValueError):
                pwm_read_age_ms = ""
            timing_debug = cmd.get("drive_timing", {}) if bool(getattr(self.args, "drive_timing_debug", True)) else {}
            if not isinstance(timing_debug, dict):
                timing_debug = {}
            target_period_ms = 1000.0 / max(float(getattr(self.args, "drive_control_rate", 1.0)), 1e-6)
            def timing_value(name: str, key: str = "duration_ms"):
                values = timings.get(name, {})
                if not isinstance(values, dict):
                    return ""
                return values.get(key, "")

            def timing_metric(name: str, key: str):
                values = timings.get(name, {})
                if not isinstance(values, dict):
                    return ""
                return values.get(key, "")
            if not bool(getattr(self.args, "lidar_timing_debug", True)):
                timings = {}
            lidar_points = timings.get("lidar_points", {})
            if not isinstance(lidar_points, dict):
                lidar_points = {}
            lidar_timestamp = timings.get("lidar_timestamp", {})
            if not isinstance(lidar_timestamp, dict):
                lidar_timestamp = {}
            runtime_metric = timings.get("lidar_runtime_update", {})
            if not isinstance(runtime_metric, dict):
                runtime_metric = {}

            def straight_lateral_m(pose_obj, section_name: str):
                if pose_obj is None:
                    return ""
                try:
                    x = float(getattr(pose_obj, "x"))
                    y = float(getattr(pose_obj, "y"))
                except (TypeError, ValueError):
                    return ""
                section_name = str(section_name or "")
                if section_name == "bottom":
                    return y + 1.5
                if section_name == "top":
                    return 1.5 - y
                if section_name == "left":
                    return x + 1.5
                if section_name == "right":
                    return 1.5 - x
                return ""

            nav_lateral_fused_m = nav.get("lateral_m", "")
            nav_lateral_otos_only_m = straight_lateral_m(otos_only_pose, str(nav.get("section", "")))
            try:
                nav_lateral_fused_minus_otos_m = float(nav_lateral_fused_m) - float(nav_lateral_otos_only_m)
            except (TypeError, ValueError):
                nav_lateral_fused_minus_otos_m = ""

            def compact_json(value) -> str:
                if value is None or (isinstance(value, str) and value == ""):
                    return ""
                try:
                    return json.dumps(json_safe(value), separators=(",", ":"), sort_keys=True)
                except (TypeError, ValueError):
                    return str(value)

            def compact_list(value) -> str:
                if isinstance(value, (list, tuple)):
                    return "|".join(str(item) for item in value)
                if value is None or (isinstance(value, str) and value == ""):
                    return ""
                return str(value)

            low_conf_watchdog = cmd.get("low_confidence_watchdog", {}) if isinstance(cmd.get("low_confidence_watchdog", {}), dict) else {}
            collision_supervisor = cmd.get("collision_supervisor", {}) if isinstance(cmd.get("collision_supervisor", {}), dict) else {}
            if not collision_supervisor:
                try:
                    collision_supervisor = self.perception.shared.collision_supervisor_snapshot()
                except Exception:
                    collision_supervisor = {}
            collision_lidar = collision_supervisor.get("lidar", {}) if isinstance(collision_supervisor.get("lidar", {}), dict) else {}
            collision_stuck = collision_supervisor.get("stuck", {}) if isinstance(collision_supervisor.get("stuck", {}), dict) else {}
            collision_recovery = cmd.get("collision_recovery", {}) if isinstance(cmd.get("collision_recovery", {}), dict) else {}
            if not collision_recovery:
                collision_recovery = collision_supervisor.get("recovery", {}) if isinstance(collision_supervisor.get("recovery", {}), dict) else {}

            row = {
                "t": f"{log_t:.6f}",
                "drive_loop_start_t": f"{loop_start:.6f}",
                "drive_loop_after_command_t": f"{after_command_t:.6f}",
                "drive_telemetry_t": f"{log_t:.6f}",
                "loop_ms": f"{(log_t - loop_start) * 1000.0:.3f}",
                "drive_loop_target_period_ms": f"{target_period_ms:.3f}",
                "drive_loop_overrun_ms": f"{max(0.0, (log_t - loop_start) * 1000.0 - target_period_ms):.3f}",
                "drive_t_cmd_fetch_ms": timing_debug.get("cmd_fetch_ms", ""),
                "drive_t_status_update_ms": timing_debug.get("status_update_ms", ""),
                "drive_t_duty_cap_ms": timing_debug.get("duty_cap_ms", ""),
                "drive_t_set_steering_ms": timing_debug.get("set_steering_ms", ""),
                "drive_t_motion_command_ms": timing_debug.get("motion_command_ms", ""),
                "drive_steering_write_due": timing_debug.get("steering_write_due", ""),
                "drive_t_db_runtime_duty_cap_call_ms": timing_debug.get("db_runtime_duty_cap_call_ms", ""),
                "drive_t_db_set_steering_call_ms": timing_debug.get("db_set_steering_call_ms", ""),
                "drive_t_db_release_brake_call_ms": timing_debug.get("db_release_brake_call_ms", ""),
                "drive_t_db_brake_call_ms": timing_debug.get("db_brake_call_ms", ""),
                "drive_t_db_neutral_call_ms": timing_debug.get("db_neutral_call_ms", ""),
                "drive_t_db_coast_call_ms": timing_debug.get("db_coast_call_ms", ""),
                "drive_t_db_set_drive_power_call_ms": timing_debug.get("db_set_drive_power_call_ms", ""),
                "drive_t_db_set_target_speed_call_ms": timing_debug.get("db_set_target_speed_call_ms", ""),
                "drive_t_db_get_telemetry_call_ms": timing_debug.get("db_get_telemetry_call_ms", telemetry.get("drive_t_get_telemetry_ms", "")),
                "drive_t_get_telemetry_ms": telemetry.get("drive_t_get_telemetry_ms", ""),
                "drive_t_shared_telemetry_ms": telemetry.get("drive_t_shared_telemetry_ms", ""),
                "drive_t_record_timing_ms": telemetry.get("drive_t_record_timing_ms", ""),
                "drive_t_row_build_ms": "",
                "drive_t_writerow_prev_ms": self.last_drive_writerow_ms,
                "drive_t_flush_prev_ms": self.last_drive_flush_ms,
                "drive_log_flush_requested_prev": int(bool(self.last_drive_flush_requested)),
                "drive_log_queue_depth": self.drive_log_queue.qsize() if self.drive_log_queue is not None else "",
                "drive_log_dropped_rows": self.drive_log_dropped_rows,
                "drive_t_sleep_ms": self.last_drive_sleep_ms,
                "drive_t_total_instrumented_ms": timing_debug.get("total_instrumented_ms", ""),
                "drive_timing_event_count": timing_debug.get("timing_event_count", ""),
                "drive_timing_event_slow_count": timing_debug.get("timing_event_slow_count", ""),
                "drive_timing_event_max_ms": timing_debug.get("timing_event_max_ms", ""),
                "drive_timing_event_summary": timing_debug.get("timing_event_summary", ""),
                "drive_timing_lock_wait_max_ms": timing_debug.get("timing_lock_wait_max_ms", ""),
                "drive_timing_lock_inside_max_ms": timing_debug.get("timing_lock_inside_max_ms", ""),
                "drive_timing_pwm_write_max_ms": timing_debug.get("timing_pwm_write_max_ms", ""),
                "drive_timing_hw_pwm_slow_count": timing_debug.get("timing_hw_pwm_slow_count", ""),
                "drive_timing_hw_pwm_max_ms": timing_debug.get("timing_hw_pwm_max_ms", ""),
                "drive_timing_hw_pwm_summary": timing_debug.get("timing_hw_pwm_summary", ""),
                "drive_t_db_set_steering_duration_ms": timing_debug.get("db_set_steering_duration_ms", ""),
                "drive_t_db_set_steering_lock_wait_ms": timing_debug.get("db_set_steering_lock_wait_ms", ""),
                "drive_t_db_set_steering_lock_inside_ms": timing_debug.get("db_set_steering_lock_inside_ms", ""),
                "drive_t_db_set_steering_pwm_ms": timing_debug.get("db_set_steering_pwm_write_ms", ""),
                "drive_t_db_steering_apply_duration_ms": timing_debug.get("db_steering_apply_duration_ms", ""),
                "drive_t_db_steering_apply_lock_wait_ms": timing_debug.get("db_steering_apply_lock_wait_ms", ""),
                "drive_t_db_steering_apply_lock_inside_ms": timing_debug.get("db_steering_apply_lock_inside_ms", ""),
                "drive_t_db_steering_apply_pwm_ms": timing_debug.get("db_steering_apply_pwm_write_ms", ""),
                "drive_t_db_motion_apply_duration_ms": timing_debug.get("db_motion_apply_duration_ms", ""),
                "drive_t_db_motion_apply_lock_wait_ms": timing_debug.get("db_motion_apply_lock_wait_ms", ""),
                "drive_t_db_motion_apply_lock_inside_ms": timing_debug.get("db_motion_apply_lock_inside_ms", ""),
                "drive_t_db_motion_apply_pwm_ms": timing_debug.get("db_motion_apply_pwm_write_ms", ""),
                "drive_t_db_led_apply_duration_ms": timing_debug.get("db_led_apply_duration_ms", ""),
                "drive_t_db_led_apply_lock_wait_ms": timing_debug.get("db_led_apply_lock_wait_ms", ""),
                "drive_t_db_led_apply_lock_inside_ms": timing_debug.get("db_led_apply_lock_inside_ms", ""),
                "drive_t_db_led_apply_pwm_ms": timing_debug.get("db_led_apply_pwm_write_ms", ""),
                "drive_t_db_actuator_stale_motion_duration_ms": timing_debug.get("db_actuator_stale_motion_duration_ms", ""),
                "drive_t_db_internal_brake_publish_duration_ms": timing_debug.get("db_internal_brake_publish_duration_ms", ""),
                "drive_t_db_set_target_speed_duration_ms": timing_debug.get("db_set_target_speed_duration_ms", ""),
                "drive_t_db_set_target_speed_lock_wait_ms": timing_debug.get("db_set_target_speed_lock_wait_ms", ""),
                "drive_t_db_set_target_speed_lock_inside_ms": timing_debug.get("db_set_target_speed_lock_inside_ms", ""),
                "drive_t_db_set_target_speed_pwm_ms": timing_debug.get("db_set_target_speed_pwm_write_ms", ""),
                "drive_t_db_get_telemetry_duration_ms": timing_debug.get("db_get_telemetry_duration_ms", ""),
                "drive_t_db_get_telemetry_lock_wait_ms": timing_debug.get("db_get_telemetry_lock_wait_ms", ""),
                "drive_t_db_get_telemetry_lock_inside_ms": timing_debug.get("db_get_telemetry_lock_inside_ms", ""),
                "drive_t_db_apply_output_duration_ms": timing_debug.get("db_apply_output_duration_ms", ""),
                "drive_t_db_apply_output_lock_wait_ms": timing_debug.get("db_apply_output_lock_wait_ms", ""),
                "drive_t_db_apply_output_lock_inside_ms": timing_debug.get("db_apply_output_lock_inside_ms", ""),
                "drive_t_db_apply_output_pwm_ms": timing_debug.get("db_apply_output_pwm_write_ms", ""),
                "drive_t_db_led_duration_ms": timing_debug.get("db_led_duration_ms", ""),
                "drive_t_db_led_lock_wait_ms": timing_debug.get("db_led_lock_wait_ms", ""),
                "drive_t_db_led_lock_inside_ms": timing_debug.get("db_led_lock_inside_ms", ""),
                "drive_t_db_led_pwm_ms": timing_debug.get("db_led_pwm_write_ms", ""),
                "drive_t_db_external_feedback_duration_ms": timing_debug.get("db_external_feedback_duration_ms", ""),
                "drive_t_db_external_feedback_lock_wait_ms": timing_debug.get("db_external_feedback_lock_wait_ms", ""),
                "drive_t_db_external_feedback_lock_inside_ms": timing_debug.get("db_external_feedback_lock_inside_ms", ""),
                "drive_t_db_feedback_select_duration_ms": timing_debug.get("db_feedback_select_duration_ms", ""),
                "drive_t_db_feedback_select_lock_wait_ms": timing_debug.get("db_feedback_select_lock_wait_ms", ""),
                "drive_t_db_feedback_select_lock_inside_ms": timing_debug.get("db_feedback_select_lock_inside_ms", ""),
                "drive_t_db_brake_duration_ms": timing_debug.get("db_brake_duration_ms", ""),
                "drive_t_db_brake_lock_wait_ms": timing_debug.get("db_brake_lock_wait_ms", ""),
                "drive_t_db_brake_pwm_ms": timing_debug.get("db_brake_pwm_write_ms", ""),
                "drive_t_db_internal_brake_duration_ms": timing_debug.get("db_internal_brake_duration_ms", ""),
                "drive_t_db_internal_brake_pwm_ms": timing_debug.get("db_internal_brake_pwm_write_ms", ""),
                "drive_t_db_internal_loop_duration_ms": timing_debug.get("db_internal_loop_duration_ms", ""),
                "drive_t_db_internal_loop_overrun_ms": timing_debug.get("db_internal_loop_overrun_ms", ""),
                "actuation_allowed": int(bool(cmd.get("actuation_allowed", False))),
                "cmd_source": str(cmd.get("source", "")),
                "cmd_reason": str(cmd.get("reason", "")),
                "cmd_blocked_while_nav_unblocked": int(str(cmd.get("reason", "")) == "blocked" and not bool(nav.get("blocked", False))),
                "cmd_brake": int(bool(cmd.get("brake", False))),
                "cmd_brake_preserve_steering": int(bool(cmd.get("brake_preserve_steering", False))),
                "cmd_brake_level": cmd.get("brake_level", ""),
                "cmd_neutral_stop": int(bool(cmd.get("neutral_stop", False))),
                "cmd_allow_reverse": int(bool(cmd.get("allow_reverse", False))),
                "cmd_precision_reverse": int(bool(cmd.get("precision_reverse", False))),
                "cmd_allow_breakaway": int(bool(cmd.get("allow_breakaway", not bool(cmd.get("precision_reverse", False))))),
                "cmd_target_speed_mps": cmd.get("target_speed_mps", ""),
                "cmd_raw_target_speed_mps": cmd.get("raw_target_speed_mps", ""),
                "cmd_direct_drive_power": cmd.get("direct_drive_power", ""),
                "drive_handoff_phase": cmd.get("drive_handoff_phase", ""),
                "drive_requested_brake_level": cmd.get("drive_requested_brake_level", ""),
                "drive_applied_brake_level": cmd.get("drive_applied_brake_level", ""),
                "drive_brake_release_active": int(bool(cmd.get("drive_brake_release_active", False))),
                "drive_brake_order": cmd.get("drive_brake_order", cmd.get("brake_order", "")),
                "cmd_drive_duty_cap": cmd.get("drive_duty_cap", ""),
                "cmd_steering_norm": cmd.get("steering_norm", ""),
                "cmd_steering_deg": cmd.get("steering_deg", ""),
                "cmd_command_age_ms": cmd.get("command_age_ms", ""),
                "cmd_staleness_speed_cap_active": int(bool(cmd.get("staleness_speed_cap_active", False))),
                "cmd_staleness_speed_cap_triggered": int(bool(cmd.get("staleness_speed_cap_triggered", False))),
                "cmd_staleness_speed_cap_age_ms": cmd.get("staleness_speed_cap_age_ms", ""),
                "cmd_staleness_speed_cap_threshold_ms": cmd.get("staleness_speed_cap_threshold_ms", ""),
                "cmd_staleness_speed_cap_mps": cmd.get("staleness_speed_cap_mps", ""),
                "cmd_staleness_speed_cap_hold_s": cmd.get("staleness_speed_cap_hold_s", ""),
                "cmd_staleness_speed_cap_remaining_s": cmd.get("staleness_speed_cap_remaining_s", ""),
                "cmd_staleness_speed_cap_original_target_mps": cmd.get("staleness_speed_cap_original_target_mps", ""),
                "cmd_staleness_speed_cap_original_raw_mps": cmd.get("staleness_speed_cap_original_raw_mps", ""),
                "cmd_staleness_speed_cap_event_count": cmd.get("staleness_speed_cap_event_count", ""),
                "cmd_staleness_speed_cap_last_event_age_ms": cmd.get("staleness_speed_cap_last_event_age_ms", ""),
                "cmd_low_confidence_watchdog_enabled": int(bool(low_conf_watchdog.get("enabled", False))),
                "cmd_low_confidence_watchdog_active": int(bool(low_conf_watchdog.get("active", False))),
                "cmd_low_confidence_watchdog_armed": int(bool(low_conf_watchdog.get("armed", False))),
                "cmd_low_confidence_watchdog_elapsed_s": low_conf_watchdog.get("elapsed_s", ""),
                "cmd_low_confidence_watchdog_timeout_s": low_conf_watchdog.get("timeout_s", ""),
                "cmd_low_confidence_watchdog_stationary": int(bool(low_conf_watchdog.get("stationary", False))),
                "cmd_low_confidence_watchdog_stationary_speed_mps": low_conf_watchdog.get("stationary_speed_mps", ""),
                "cmd_low_confidence_watchdog_stationary_threshold_mps": low_conf_watchdog.get("stationary_threshold_mps", ""),
                "cmd_low_confidence_watchdog_reason": str(low_conf_watchdog.get("reason", "")),
                "collision_supervisor_enabled": int(bool(collision_supervisor.get("enabled", False))),
                "collision_supervisor_suppressed_reason": str(collision_supervisor.get("suppressed_reason", "")),
                "collision_would_action": str(collision_supervisor.get("would_action", "")),
                "collision_lidar_active_brake_enabled": int(bool(cmd.get("collision_lidar_active_brake_enabled", False))),
                "collision_lidar_active_brake_triggered": int(bool(cmd.get("collision_lidar_active_brake_triggered", False))),
                "collision_lidar_active_brake_active": int(bool(cmd.get("collision_lidar_active_brake_active", False))),
                "collision_lidar_active_brake_remaining_s": cmd.get("collision_lidar_active_brake_remaining_s", ""),
                "collision_lidar_active_brake_hold_s": cmd.get("collision_lidar_active_brake_hold_s", ""),
                "collision_lidar_active_brake_event_count": cmd.get("collision_lidar_active_brake_event_count", ""),
                "collision_lidar_active_brake_last_event_age_ms": cmd.get("collision_lidar_active_brake_last_event_age_ms", ""),
                "collision_lidar_active_brake_trigger_scan_t": cmd.get("collision_lidar_active_brake_trigger_scan_t", ""),
                "collision_lidar_metric_present": int(bool(collision_lidar.get("metric_present", False))),
                "collision_lidar_metric_source": str(collision_lidar.get("metric_source", "")),
                "collision_lidar_raw_status": str(collision_lidar.get("raw_status", "")),
                "collision_lidar_raw_usable": int(bool(collision_lidar.get("raw_usable", False))),
                "collision_lidar_raw_points": collision_lidar.get("raw_points", ""),
                "collision_lidar_status": str(collision_lidar.get("status", "")),
                "collision_lidar_points": collision_lidar.get("point_count", ""),
                "collision_lidar_min_points": collision_lidar.get("min_points", ""),
                "collision_lidar_hit": int(bool(collision_lidar.get("hit", False))),
                "collision_lidar_warning_points": collision_lidar.get("warning_point_count", ""),
                "collision_lidar_warning_min_points": collision_lidar.get("warning_min_points", ""),
                "collision_lidar_warning_shell_points": collision_lidar.get("warning_shell_points", ""),
                "collision_lidar_warning_hit": int(bool(collision_lidar.get("warning_hit", False))),
                "collision_lidar_trigger": int(bool(collision_lidar.get("trigger", False))),
                "collision_lidar_trigger_reason": str(collision_lidar.get("trigger_reason", "")),
                "collision_lidar_trigger_by_strong_single": int(bool(collision_lidar.get("trigger_by_strong_single", False))),
                "collision_lidar_trigger_by_hard": int(bool(collision_lidar.get("trigger_by_hard", False))),
                "collision_lidar_trigger_by_warning_handoff": int(bool(collision_lidar.get("trigger_by_warning_handoff", False))),
                "collision_lidar_strong_single_enabled": int(bool(collision_lidar.get("strong_single_enabled", False))),
                "collision_lidar_strong_hard_points": collision_lidar.get("strong_hard_points", ""),
                "collision_lidar_strong_combo_hard_points": collision_lidar.get("strong_combo_hard_points", ""),
                "collision_lidar_strong_combo_warning_points": collision_lidar.get("strong_combo_warning_points", ""),
                "collision_lidar_window_hits": collision_lidar.get("window_hits", ""),
                "collision_lidar_window_warning_hits": collision_lidar.get("window_warning_hits", ""),
                "collision_lidar_window_prior_warning_hits": collision_lidar.get("window_prior_warning_hits", ""),
                "collision_lidar_window_usable": collision_lidar.get("window_usable", ""),
                "collision_lidar_window_required": collision_lidar.get("window_required", ""),
                "collision_lidar_window_warning_required": collision_lidar.get("window_warning_required", ""),
                "collision_lidar_window_hard_required_with_warning": collision_lidar.get("window_hard_required_with_warning", ""),
                "collision_lidar_yaw_rate_dps": collision_lidar.get("yaw_rate_dps", ""),
                "collision_lidar_yaw_gated": int(bool(collision_lidar.get("yaw_gated", False))),
                "collision_lidar_depth_m": collision_lidar.get("depth_m", ""),
                "collision_lidar_warning_depth_m": collision_lidar.get("warning_depth_m", ""),
                "collision_lidar_nearest_x_m": collision_lidar.get("nearest_x_m", ""),
                "collision_lidar_nearest_abs_y_m": collision_lidar.get("nearest_abs_y_m", ""),
                "collision_lidar_warning_nearest_x_m": collision_lidar.get("warning_nearest_x_m", ""),
                "collision_lidar_warning_nearest_abs_y_m": collision_lidar.get("warning_nearest_abs_y_m", ""),
                "collision_stuck_candidate": int(bool(collision_stuck.get("candidate", False))),
                "collision_stuck_trigger": int(bool(collision_stuck.get("trigger", False))),
                "collision_stuck_reason": str(collision_stuck.get("reason", "")),
                "collision_stuck_elapsed_s": collision_stuck.get("elapsed_s", ""),
                "collision_stuck_required_s": collision_stuck.get("required_s", ""),
                "collision_stuck_target_speed_mps": collision_stuck.get("target_speed_mps", ""),
                "collision_stuck_applied_mode": collision_stuck.get("applied_mode", ""),
                "collision_stuck_applied_duty": collision_stuck.get("applied_duty", ""),
                "collision_stuck_duty_threshold": collision_stuck.get("duty_threshold", ""),
                "collision_stuck_otos_forward_mps": collision_stuck.get("otos_forward_mps", ""),
                "collision_stuck_route_progress_m": collision_stuck.get("route_progress_m", ""),
                "collision_stuck_route_progress_delta_m": collision_stuck.get("route_progress_delta_m", ""),
                "collision_stuck_sample_count": collision_stuck.get("sample_count", ""),
                "collision_recovery_state": str(collision_recovery.get("state", "")),
                "collision_recovery_enabled": int(bool(collision_recovery.get("enabled", False))),
                "collision_recovery_reason": str(collision_recovery.get("reason", "")),
                "collision_recovery_exit_reason": str(collision_recovery.get("exit_reason", "")),
                "collision_recovery_episode_elapsed_s": collision_recovery.get("episode_elapsed_s", ""),
                "collision_recovery_persistent_ready": int(bool(collision_recovery.get("persistent_ready", False))),
                "collision_recovery_hazard_usable": int(bool(collision_recovery.get("hazard_usable", False))),
                "collision_recovery_hazard_latest_age_ms": collision_recovery.get("hazard_latest_age_ms", ""),
                "collision_recovery_reverse_clear_elapsed_s": collision_recovery.get("reverse_clear_elapsed_s", ""),
                "collision_recovery_section_visit_key": str(collision_recovery.get("section_visit_key", "")),
                "collision_recovery_section_visit_count": collision_recovery.get("section_visit_count", ""),
                "collision_recovery_section_visit_limit": collision_recovery.get("section_visit_limit", ""),
                "collision_recovery_reverse_attempt_count": collision_recovery.get("reverse_attempt_count", ""),
                "collision_recovery_requested_target_m": collision_recovery.get("requested_target_m", ""),
                "collision_recovery_target_m": collision_recovery.get("target_m", ""),
                "collision_recovery_safe_distance_m": collision_recovery.get("safe_distance_m", ""),
                "collision_recovery_progress_m": collision_recovery.get("progress_m", ""),
                "collision_recovery_remaining_m": collision_recovery.get("remaining_m", ""),
                "collision_recovery_reverse_clear_since_t": collision_recovery.get("reverse_clear_since_t", ""),
                "collision_recovery_limiting_reason": str(collision_recovery.get("limiting_reason", "")),
                "collision_recovery_creep_active": int(bool(collision_recovery.get("creep_active", False))),
                "collision_recovery_creep_override": int(bool(collision_recovery.get("creep_override_active", False) or cmd.get("collision_recovery_creep_override", False))),
                "collision_recovery_creep_speed_mps": collision_recovery.get("creep_speed_mps", ""),
                "collision_recovery_creep_clear_elapsed_s": collision_recovery.get("creep_clear_elapsed_s", ""),
                "deadman_active": int(bool(deadman.get("active", False))),
                "deadman_fresh": int(bool(deadman.get("fresh", False))),
                "deadman_source": str(deadman.get("source", "")),
                "deadman_client_id": str(deadman.get("client_id", "")),
                "deadman_reason": str(deadman.get("reason", "")),
                "deadman_gpio_level": deadman.get("gpio_level", ""),
                "deadman_active_clients": deadman.get("active_clients", ""),
                "deadman_client_count": deadman.get("client_count", ""),
                "deadman_age_ms": deadman.get("age_ms", ""),
                "competition_start_state": str(competition_start.get("state", "")),
                "competition_processing_allowed": int(bool(competition_start.get("processing_allowed", False))),
                "competition_drive_logic_allowed": int(bool(competition_start.get("drive_logic_allowed", False))),
                "competition_trigger_t": competition_start.get("trigger_t", ""),
                "competition_measurement_start_t": competition_start.get("measurement_start_t", ""),
                "competition_active_t": competition_start.get("active_t", ""),
                "competition_delay_remaining_s": competition_start.get("delay_remaining_s", ""),
                "competition_initial_hold_remaining_s": competition_start.get("initial_hold_remaining_s", ""),
                "competition_startup_lidar_correction_remaining_s": competition_start.get("startup_lidar_correction_remaining_s", ""),
                "competition_startup_lidar_correction_active": int(bool(competition_start.get("startup_lidar_correction_active", False))),
                "competition_startup_lidar_correction_done": int(bool(competition_start.get("startup_lidar_correction_done", False))),
                "otos_vx_mps": "" if pose is None else getattr(pose, "vx", ""),
                "otos_vy_mps": "" if pose is None else getattr(pose, "vy", ""),
                "otos_vh_dps": "" if pose is None else getattr(pose, "vh", ""),
                "otos_speed_mps": otos_speed,
                "otos_forward_mps": otos_forward_mps,
                "otos_display_vx_mps": "" if display_pose is None else getattr(display_pose, "vx", ""),
                "otos_display_vy_mps": "" if display_pose is None else getattr(display_pose, "vy", ""),
                "otos_display_speed_mps": display_speed,
                "otos_age_ms": otos_age_ms,
                "otos_jump_detected": int(bool(otos_jump.get("detected", False))),
                "otos_jump_triggered_hold": int(bool(otos_jump.get("triggered_hold", False))),
                "otos_jump_reason": str(otos_jump.get("reason", "")),
                "otos_jump_trigger_rules": str(otos_jump.get("trigger_rules", "")),
                "otos_jump_event_count": otos_jump.get("event_count", ""),
                "otos_jump_dt_ms": otos_jump.get("dt_ms", ""),
                "otos_jump_dx_m": otos_jump.get("dx_m", ""),
                "otos_jump_dy_m": otos_jump.get("dy_m", ""),
                "otos_jump_dist_m": otos_jump.get("dist_m", ""),
                "otos_jump_implied_speed_mps": otos_jump.get("implied_speed_mps", ""),
                "otos_jump_reported_speed_mps": otos_jump.get("reported_speed_mps", ""),
                "otos_jump_vector_mismatch_mps": otos_jump.get("vector_mismatch_mps", ""),
                "otos_jump_angle_velocity_deg": otos_jump.get("angle_velocity_deg", ""),
                "otos_jump_angle_trend_deg": otos_jump.get("angle_trend_deg", ""),
                "otos_jump_trend_distance_m": otos_jump.get("trend_distance_m", ""),
                "otos_jump_phase": str(otos_jump.get("phase", "")),
                "otos_jump_hold_suppressed_reason": str(otos_jump.get("hold_suppressed_reason", "")),
                "otos_display_x_m": "" if display_pose is None else getattr(display_pose, "x", ""),
                "otos_display_y_m": "" if display_pose is None else getattr(display_pose, "y", ""),
                "otos_display_h_deg": "" if display_pose is None else getattr(display_pose, "h", ""),
                "otos_only_x_m": "" if otos_only_pose is None else getattr(otos_only_pose, "x", ""),
                "otos_only_y_m": "" if otos_only_pose is None else getattr(otos_only_pose, "y", ""),
                "otos_only_h_deg": "" if otos_only_pose is None else getattr(otos_only_pose, "h", ""),
                "fused_minus_otos_x_m": fused_minus_otos_x,
                "fused_minus_otos_y_m": fused_minus_otos_y,
                "fused_minus_otos_h_deg": fused_minus_otos_h,
                "fused_minus_otos_distance_m": fused_minus_otos_distance,
                "otos_local_x_m": "" if pose is None else getattr(pose, "x", ""),
                "otos_local_y_m": "" if pose is None else getattr(pose, "y", ""),
                "otos_local_h_deg": "" if pose is None else getattr(pose, "h", ""),
                "anchor_x_m": "" if anchor is None else getattr(anchor, "x", ""),
                "anchor_y_m": "" if anchor is None else getattr(anchor, "y", ""),
                "anchor_h_deg": "" if anchor is None else getattr(anchor, "h", ""),
                "anchor_version": anchor_version,
                "anchor_source": anchor_source,
                "anchor_last_source": anchor_last.get("source", ""),
                "anchor_last_rejected": int(bool(anchor_last.get("rejected", False))),
                "anchor_last_jump_m": anchor_last.get("jump_m", ""),
                "anchor_last_jump_h_deg": anchor_last.get("jump_h_deg", ""),
                "lidar_valid": int(bool(lidar_pose.get("valid", False))),
                "lidar_confidence": lidar_pose.get("confidence", ""),
                "lidar_reason": str(lidar_pose.get("reason", "")),
                "lidar_residual_m": lidar_pose.get("residual_m", ""),
                "lidar_dx_m": lidar_pose.get("dx", ""),
                "lidar_dy_m": lidar_pose.get("dy", ""),
                "lidar_dh_deg": lidar_pose.get("dh", ""),
                "lidar_matched_points": lidar_pose.get("matched_points", ""),
                "lidar_rejected_points": lidar_pose.get("rejected_points", ""),
                "lidar_visible_walls": " ".join(str(w) for w in (lidar_pose.get("visible_walls", []) or [])),
                "lidar_cloud_wall_valid": int(bool(lidar_cloud_wall.get("valid", False))),
                "lidar_cloud_wall_reason": str(lidar_cloud_wall.get("reason", "")),
                "lidar_cloud_wall_matched_points": lidar_cloud_wall.get("matched_points", ""),
                "lidar_cloud_wall_wall_count": lidar_cloud_wall.get("wall_count", ""),
                "lidar_cloud_wall_median_abs_m": lidar_cloud_wall.get("median_abs_m", ""),
                "lidar_cloud_wall_p80_abs_m": lidar_cloud_wall.get("p80_abs_m", ""),
                "lidar_cloud_wall_p95_abs_m": lidar_cloud_wall.get("p95_abs_m", ""),
                "lidar_cloud_wall_median_signed_m": lidar_cloud_wall.get("median_signed_m", ""),
                "lidar_cloud_wall_worst_wall": str(lidar_cloud_wall.get("worst_wall", "")),
                "lidar_cloud_wall_worst_wall_p80_abs_m": lidar_cloud_wall.get("worst_wall_p80_abs_m", ""),
                "lidar_cloud_wall_dominant_wall": str(lidar_cloud_wall.get("dominant_wall", "")),
                "lidar_cloud_wall_dominant_residual_m": lidar_cloud_wall.get("dominant_residual_m", ""),
                "lidar_cloud_wall_live_ref_dx_m": lidar_cloud_wall.get("live_minus_reference_dx_m", ""),
                "lidar_cloud_wall_live_ref_dy_m": lidar_cloud_wall.get("live_minus_reference_dy_m", ""),
                "lidar_cloud_wall_live_ref_distance_m": lidar_cloud_wall.get("live_minus_reference_distance_m", ""),
                "lidar_cloud_wall_live_ref_dh_deg": lidar_cloud_wall.get("live_minus_reference_dh_deg", ""),
                "lidar_cloud_wall_scan_age_ms": lidar_cloud_wall.get("scan_age_ms", ""),
                "lidar_cloud_wall_scan_mid_age_ms": lidar_cloud_wall.get("scan_mid_age_ms", ""),
                "lidar_cloud_wall_speed_mps": lidar_cloud_wall.get("speed_mps", ""),
                "lidar_cloud_wall_yaw_rate_dps": lidar_cloud_wall.get("yaw_rate_dps", ""),
                "lidar_cloud_wall_speed_x_scan_age_m": lidar_cloud_wall.get("speed_x_scan_age_m", ""),
                "lidar_cloud_wall_diagnostic_status": str(lidar_cloud_wall.get("diagnostic_status", "")),
                "lidar_cloud_wall_diagnostic_mode": str(lidar_cloud_wall.get("diagnostic_mode", "")),
                "lidar_cloud_wall_diagnostic_request_id": lidar_cloud_wall.get("diagnostic_request_id", ""),
                "lidar_cloud_wall_diagnostic_pending": lidar_cloud_wall.get("diagnostic_pending", ""),
                "lidar_cloud_wall_diagnostic_submitted": lidar_cloud_wall.get("diagnostic_submitted", ""),
                "lidar_cloud_wall_diagnostic_completed": lidar_cloud_wall.get("diagnostic_completed", ""),
                "lidar_cloud_wall_diagnostic_dropped": lidar_cloud_wall.get("diagnostic_dropped", ""),
                "lidar_cloud_wall_diagnostic_dropped_this_submit": lidar_cloud_wall.get("diagnostic_dropped_this_submit", ""),
                "lidar_cloud_wall_diagnostic_errors": lidar_cloud_wall.get("diagnostic_errors", ""),
                "lidar_cloud_wall_diagnostic_source_scan_t": lidar_cloud_wall.get("diagnostic_source_scan_t", ""),
                "lidar_cloud_wall_diagnostic_age_ms": lidar_cloud_wall.get("diagnostic_age_ms", ""),
                "lidar_cloud_wall_diagnostic_result_age_ms": lidar_cloud_wall.get("diagnostic_result_age_ms", ""),
                "lidar_cloud_wall_diagnostic_queue_age_ms": lidar_cloud_wall.get("diagnostic_queue_age_ms", ""),
                "lidar_cloud_wall_diagnostic_worker_wall_ms": lidar_cloud_wall.get("diagnostic_worker_wall_ms", ""),
                "lidar_cloud_wall_diagnostic_worker_process_cpu_ms": lidar_cloud_wall.get("diagnostic_worker_process_cpu_ms", ""),
                "lidar_cloud_wall_diagnostic_worker_thread_cpu_ms": lidar_cloud_wall.get("diagnostic_worker_thread_cpu_ms", ""),
                "lidar_cloud_wall_points_ms": lidar_cloud_wall.get("points_ms", ""),
                "lidar_cloud_wall_wall_loop_ms": lidar_cloud_wall.get("wall_loop_ms", ""),
                "lidar_cloud_wall_stats_ms": lidar_cloud_wall.get("stats_ms", ""),
                "lidar_cloud_wall_meta_ms": lidar_cloud_wall.get("meta_ms", ""),
                "lidar_cloud_wall_summary": str(lidar_cloud_wall.get("summary", "")),
                "lidar_correction_status": str(lidar_correction.get("status", "")),
                "lidar_correction_mode": str(lidar_correction.get("mode", "")),
                "lidar_correction_validation_enabled": lidar_correction.get("validation_enabled", ""),
                "lidar_correction_heading_enabled": lidar_correction.get("heading_enabled", ""),
                "lidar_correction_axis_heading_enabled": lidar_correction.get("axis_heading_enabled", ""),
                "lidar_correction_samples": lidar_correction.get("samples", ""),
                "lidar_correction_dx_median": lidar_correction.get("dx_median", ""),
                "lidar_correction_dy_median": lidar_correction.get("dy_median", ""),
                "lidar_correction_dh_median": lidar_correction.get("dh_median", ""),
                "lidar_correction_dx_std": lidar_correction.get("dx_std", ""),
                "lidar_correction_dy_std": lidar_correction.get("dy_std", ""),
                "lidar_correction_dh_std": lidar_correction.get("dh_std", ""),
                "lidar_correction_last_step_x": lidar_correction.get("last_step_x", ""),
                "lidar_correction_last_step_y": lidar_correction.get("last_step_y", ""),
                "lidar_correction_last_step_h": lidar_correction.get("last_step_h", ""),
                "lidar_correction_axis_x_raw": lidar_correction.get("axis_x_raw", ""),
                "lidar_correction_axis_y_raw": lidar_correction.get("axis_y_raw", ""),
                "lidar_correction_axis_h_raw": lidar_correction.get("axis_h_raw", ""),
                "lidar_correction_axis_x_weighted_residual": lidar_correction.get("axis_x_weighted_residual", ""),
                "lidar_correction_axis_y_weighted_residual": lidar_correction.get("axis_y_weighted_residual", ""),
                "lidar_correction_axis_x_selected_residuals": str(lidar_correction.get("axis_x_selected_residuals", "")),
                "lidar_correction_axis_y_selected_residuals": str(lidar_correction.get("axis_y_selected_residuals", "")),
                "lidar_correction_estimate_confidence": lidar_correction.get("estimate_confidence", ""),
                "lidar_correction_estimate_dx_m": lidar_correction.get("estimate_dx", ""),
                "lidar_correction_estimate_dy_m": lidar_correction.get("estimate_dy", ""),
                "lidar_correction_estimate_dh_deg": lidar_correction.get("estimate_dh", ""),
                "lidar_correction_rolling_decision": str(lidar_correction.get("rolling_decision", "")),
                "lidar_correction_rolling_reject_reason": str(lidar_correction.get("rolling_reject_reason", "")),
                "lidar_correction_rolling_consistent_xy": lidar_correction.get("rolling_consistent_xy", ""),
                "lidar_correction_rolling_consistent_h": lidar_correction.get("rolling_consistent_h", ""),
                "lidar_correction_rolling_large_enough": lidar_correction.get("rolling_large_enough", ""),
                "lidar_correction_rolling_confidence_median": lidar_correction.get("rolling_confidence_median", ""),
                "lidar_correction_rolling_step_x_unclamped": lidar_correction.get("rolling_step_x_unclamped", ""),
                "lidar_correction_rolling_step_y_unclamped": lidar_correction.get("rolling_step_y_unclamped", ""),
                "lidar_correction_rolling_step_h_unclamped": lidar_correction.get("rolling_step_h_unclamped", ""),
                "lidar_correction_rolling_step_h_limited": lidar_correction.get("rolling_step_h_limited", ""),
                "lidar_correction_rolling_candidate_step_x": lidar_correction.get("rolling_candidate_step_x", ""),
                "lidar_correction_rolling_candidate_step_y": lidar_correction.get("rolling_candidate_step_y", ""),
                "lidar_correction_rolling_candidate_step_h": lidar_correction.get("rolling_candidate_step_h", ""),
                "lidar_correction_axis_x_filter_pre_step": lidar_correction.get("axis_x_filter_pre_step", ""),
                "lidar_correction_axis_y_filter_pre_step": lidar_correction.get("axis_y_filter_pre_step", ""),
                "lidar_correction_axis_h_filter_pre_step": lidar_correction.get("axis_h_filter_pre_step", ""),
                "lidar_correction_axis_x_filter": lidar_correction.get("axis_x_filter", ""),
                "lidar_correction_axis_y_filter": lidar_correction.get("axis_y_filter", ""),
                "lidar_correction_axis_h_filter": lidar_correction.get("axis_h_filter", ""),
                "lidar_correction_axis_step_x": lidar_correction.get("axis_step_x", ""),
                "lidar_correction_axis_step_y": lidar_correction.get("axis_step_y", ""),
                "lidar_correction_axis_step_h": lidar_correction.get("axis_step_h", ""),
                "lidar_correction_axis_step_x_unclamped": lidar_correction.get("axis_step_x_unclamped", ""),
                "lidar_correction_axis_step_y_unclamped": lidar_correction.get("axis_step_y_unclamped", ""),
                "lidar_correction_axis_step_h_unclamped": lidar_correction.get("axis_step_h_unclamped", ""),
                "lidar_correction_axis_step_h_limited": lidar_correction.get("axis_step_h_limited", ""),
                "lidar_correction_axis_step_zero_reason": str(lidar_correction.get("axis_step_zero_reason", "")),
                "lidar_correction_axis_candidate_step_x": lidar_correction.get("axis_candidate_step_x", ""),
                "lidar_correction_axis_candidate_step_y": lidar_correction.get("axis_candidate_step_y", ""),
                "lidar_correction_axis_candidate_step_h": lidar_correction.get("axis_candidate_step_h", ""),
                "lidar_correction_axis_motion_scale": lidar_correction.get("axis_motion_scale", ""),
                "lidar_correction_yaw_rate_dps": lidar_correction.get("yaw_rate_dps", ""),
                "lidar_correction_yaw_authority": lidar_correction.get("yaw_authority", ""),
                "lidar_correction_yaw_limited": lidar_correction.get("yaw_limited", ""),
                "lidar_correction_rolling_yaw_authority_median": lidar_correction.get("rolling_yaw_authority_median", ""),
                "lidar_correction_rolling_yaw_authority_min": lidar_correction.get("rolling_yaw_authority_min", ""),
                "lidar_correction_rolling_distance_authority_median": lidar_correction.get("rolling_distance_authority_median", ""),
                "lidar_correction_rolling_distance_authority_min": lidar_correction.get("rolling_distance_authority_min", ""),
                "lidar_correction_rolling_combined_authority_median": lidar_correction.get("rolling_combined_authority_median", ""),
                "lidar_correction_rolling_combined_authority_min": lidar_correction.get("rolling_combined_authority_min", ""),
                "lidar_correction_axis_x_distance_authority": lidar_correction.get("axis_x_distance_authority", ""),
                "lidar_correction_axis_y_distance_authority": lidar_correction.get("axis_y_distance_authority", ""),
                "lidar_correction_axis_h_distance_authority": lidar_correction.get("axis_h_distance_authority", ""),
                "lidar_correction_axis_distance_authority_min": lidar_correction.get("axis_distance_authority_min", ""),
                "lidar_correction_axis_distance_authority_median": lidar_correction.get("axis_distance_authority_median", ""),
                "lidar_correction_axis_effective_max_error_m": lidar_correction.get("axis_effective_max_error_m", ""),
                "lidar_correction_axis_used_walls": csv_seq(lidar_correction.get("axis_used_walls", "")),
                "lidar_correction_axis_used_wall_count": lidar_correction.get("axis_used_wall_count", ""),
                "lidar_correction_axis_x_wall_count": lidar_correction.get("axis_x_wall_count", ""),
                "lidar_correction_axis_y_wall_count": lidar_correction.get("axis_y_wall_count", ""),
                "lidar_correction_axis_rejected": csv_seq(lidar_correction.get("axis_rejected", "")),
                "lidar_correction_axis_rejected_count": lidar_correction.get("axis_rejected_count", ""),
                "lidar_correction_axis_heading_walls": csv_seq(lidar_correction.get("axis_heading_walls", "")),
                "lidar_correction_axis_heading_wall_count": lidar_correction.get("axis_heading_wall_count", ""),
                "lidar_correction_axis_heading_rejected": csv_seq(lidar_correction.get("axis_heading_rejected", "")),
                "lidar_correction_axis_heading_rejected_count": lidar_correction.get("axis_heading_rejected_count", ""),
                "lidar_correction_axis_heading_block_reason": str(lidar_correction.get("axis_heading_block_reason", "")),
                "lidar_correction_axis_heading_measurement_candidate": lidar_correction.get("axis_heading_measurement_candidate", ""),
                "lidar_correction_axis_heading_measurement_source": lidar_correction.get("axis_heading_measurement_source", ""),
                "lidar_correction_axis_heading_would_filter": lidar_correction.get("axis_heading_would_filter", ""),
                "lidar_correction_axis_heading_would_step_h_unclamped": lidar_correction.get("axis_heading_would_step_h_unclamped", ""),
                "lidar_correction_axis_heading_would_step_h": lidar_correction.get("axis_heading_would_step_h", ""),
                "lidar_correction_axis_heading_would_step_h_limited": lidar_correction.get("axis_heading_would_step_h_limited", ""),
                "lidar_correction_axis_heading_would_step_zero_reason": str(lidar_correction.get("axis_heading_would_step_zero_reason", "")),
                "lidar_correction_axis_heading_raw_weighted_mean": lidar_correction.get("axis_heading_raw_weighted_mean", ""),
                "lidar_correction_axis_heading_clamped_mean": lidar_correction.get("axis_heading_clamped_mean", ""),
                "lidar_correction_axis_heading_effective_weight_sum": lidar_correction.get("axis_heading_effective_weight_sum", ""),
                "lidar_correction_axis_heading_close_wall_count": lidar_correction.get("axis_heading_close_wall_count", ""),
                "lidar_correction_axis_heading_far_wall_count": lidar_correction.get("axis_heading_far_wall_count", ""),
                "lidar_correction_axis_heading_close_contribution": lidar_correction.get("axis_heading_close_contribution", ""),
                "lidar_correction_axis_heading_far_contribution": lidar_correction.get("axis_heading_far_contribution", ""),
                "lidar_correction_axis_heading_contributions": str(lidar_correction.get("axis_heading_contributions", "")),
                "lidar_correction_axis_wall_observations": str(lidar_correction.get("axis_wall_observations", "")),
                "lidar_correction_axis_trend_x_decision": lidar_correction.get("axis_trend_x_decision", ""),
                "lidar_correction_axis_trend_x_reject_reason": lidar_correction.get("axis_trend_x_reject_reason", ""),
                "lidar_correction_axis_trend_x_count": lidar_correction.get("axis_trend_x_count", ""),
                "lidar_correction_axis_trend_x_active_count": lidar_correction.get("axis_trend_x_active_count", ""),
                "lidar_correction_axis_trend_x_sign_ratio": lidar_correction.get("axis_trend_x_sign_ratio", ""),
                "lidar_correction_axis_trend_x_median": lidar_correction.get("axis_trend_x_median", ""),
                "lidar_correction_axis_trend_x_std": lidar_correction.get("axis_trend_x_std", ""),
                "lidar_correction_axis_trend_x_age_ms": lidar_correction.get("axis_trend_x_age_ms", ""),
                "lidar_correction_axis_trend_x_authority_median": lidar_correction.get("axis_trend_x_authority_median", ""),
                "lidar_correction_axis_trend_x_wall_count_median": lidar_correction.get("axis_trend_x_wall_count_median", ""),
                "lidar_correction_axis_trend_x_step_unclamped": lidar_correction.get("axis_trend_x_step_unclamped", ""),
                "lidar_correction_axis_trend_x_step": lidar_correction.get("axis_trend_x_step", ""),
                "lidar_correction_axis_trend_y_decision": lidar_correction.get("axis_trend_y_decision", ""),
                "lidar_correction_axis_trend_y_reject_reason": lidar_correction.get("axis_trend_y_reject_reason", ""),
                "lidar_correction_axis_trend_y_count": lidar_correction.get("axis_trend_y_count", ""),
                "lidar_correction_axis_trend_y_active_count": lidar_correction.get("axis_trend_y_active_count", ""),
                "lidar_correction_axis_trend_y_sign_ratio": lidar_correction.get("axis_trend_y_sign_ratio", ""),
                "lidar_correction_axis_trend_y_median": lidar_correction.get("axis_trend_y_median", ""),
                "lidar_correction_axis_trend_y_std": lidar_correction.get("axis_trend_y_std", ""),
                "lidar_correction_axis_trend_y_age_ms": lidar_correction.get("axis_trend_y_age_ms", ""),
                "lidar_correction_axis_trend_y_authority_median": lidar_correction.get("axis_trend_y_authority_median", ""),
                "lidar_correction_axis_trend_y_wall_count_median": lidar_correction.get("axis_trend_y_wall_count_median", ""),
                "lidar_correction_axis_trend_y_step_unclamped": lidar_correction.get("axis_trend_y_step_unclamped", ""),
                "lidar_correction_axis_trend_y_step": lidar_correction.get("axis_trend_y_step", ""),
                "lidar_correction_axis_trend_h_decision": lidar_correction.get("axis_trend_h_decision", ""),
                "lidar_correction_axis_trend_h_reject_reason": lidar_correction.get("axis_trend_h_reject_reason", ""),
                "lidar_correction_axis_trend_h_count": lidar_correction.get("axis_trend_h_count", ""),
                "lidar_correction_axis_trend_h_active_count": lidar_correction.get("axis_trend_h_active_count", ""),
                "lidar_correction_axis_trend_h_sign_ratio": lidar_correction.get("axis_trend_h_sign_ratio", ""),
                "lidar_correction_axis_trend_h_median": lidar_correction.get("axis_trend_h_median", ""),
                "lidar_correction_axis_trend_h_std": lidar_correction.get("axis_trend_h_std", ""),
                "lidar_correction_axis_trend_h_age_ms": lidar_correction.get("axis_trend_h_age_ms", ""),
                "lidar_correction_axis_trend_h_authority_median": lidar_correction.get("axis_trend_h_authority_median", ""),
                "lidar_correction_axis_trend_h_wall_count_median": lidar_correction.get("axis_trend_h_wall_count_median", ""),
                "lidar_correction_axis_trend_h_step_unclamped": lidar_correction.get("axis_trend_h_step_unclamped", ""),
                "lidar_correction_axis_trend_h_step": lidar_correction.get("axis_trend_h_step", ""),
                "lidar_correction_validation_decision": str(lidar_correction.get("validation_decision", "")),
                "lidar_correction_validation_reject_reason": str(lidar_correction.get("validation_reject_reason", "")),
                "lidar_correction_validation_component_decision": str(lidar_correction.get("validation_component_decision", "")),
                "lidar_correction_validation_component_reject_reason": str(lidar_correction.get("validation_component_reject_reason", "")),
                "lidar_correction_validation_pre_xy_m": lidar_correction.get("validation_pre_xy_m", ""),
                "lidar_correction_validation_post_xy_m": lidar_correction.get("validation_post_xy_m", ""),
                "lidar_correction_validation_improvement_xy_m": lidar_correction.get("validation_improvement_xy_m", ""),
                "lidar_correction_validation_pre_h_deg": lidar_correction.get("validation_pre_h_deg", ""),
                "lidar_correction_validation_post_h_deg": lidar_correction.get("validation_post_h_deg", ""),
                "lidar_correction_validation_improvement_h_deg": lidar_correction.get("validation_improvement_h_deg", ""),
                "lidar_correction_validation_post_confidence": lidar_correction.get("validation_post_confidence", ""),
                "lidar_correction_validation_post_valid": lidar_correction.get("validation_post_valid", ""),
                "lidar_correction_validation_points": lidar_correction.get("validation_points", ""),
                "lidar_correction_validation_applied_step_x": lidar_correction.get("validation_applied_step_x", ""),
                "lidar_correction_validation_applied_step_y": lidar_correction.get("validation_applied_step_y", ""),
                "lidar_correction_validation_applied_step_h": lidar_correction.get("validation_applied_step_h", ""),
                "lidar_correction_validation_async_enabled": lidar_correction.get("validation_async_enabled", ""),
                "lidar_correction_validation_async_pending": lidar_correction.get("validation_async_pending", ""),
                "lidar_correction_validation_async_request_id": lidar_correction.get("validation_async_request_id", ""),
                "lidar_correction_validation_async_applied_id": lidar_correction.get("validation_async_applied_id", ""),
                "lidar_correction_validation_async_generation": lidar_correction.get("validation_async_generation", ""),
                "lidar_correction_validation_async_submit_status": lidar_correction.get("validation_async_submit_status", ""),
                "lidar_correction_validation_async_source": lidar_correction.get("validation_async_source", ""),
                "lidar_correction_validation_async_submitted_request_id": lidar_correction.get("validation_async_submitted_request_id", ""),
                "lidar_correction_validation_async_anchor_version": lidar_correction.get("validation_async_anchor_version", ""),
                "lidar_correction_validation_async_submit_scan_t": lidar_correction.get("validation_async_submit_scan_t", ""),
                "lidar_correction_validation_async_submit_scan_age_ms": lidar_correction.get("validation_async_submit_scan_age_ms", ""),
                "lidar_correction_validation_async_result_scan_t": lidar_correction.get("validation_async_result_scan_t", ""),
                "lidar_correction_validation_async_result_anchor_version": lidar_correction.get("validation_async_result_anchor_version", ""),
                "lidar_correction_validation_async_current_anchor_version": lidar_correction.get("validation_async_current_anchor_version", ""),
                "lidar_correction_validation_async_anchor_version_delta": lidar_correction.get("validation_async_anchor_version_delta", ""),
                "lidar_correction_validation_async_dropped_this_submit": lidar_correction.get("validation_async_dropped_this_submit", ""),
                "lidar_correction_validation_async_dropped_requests": lidar_correction.get("validation_async_dropped_requests", ""),
                "lidar_correction_validation_async_stale_results": lidar_correction.get("validation_async_stale_results", ""),
                "lidar_correction_validation_async_errors": lidar_correction.get("validation_async_errors", ""),
                "lidar_correction_validation_async_wall_ms": lidar_correction.get("validation_async_wall_ms", ""),
                "lidar_correction_validation_async_process_cpu_ms": lidar_correction.get("validation_async_process_cpu_ms", ""),
                "lidar_correction_validation_async_thread_cpu_ms": lidar_correction.get("validation_async_thread_cpu_ms", ""),
                "lidar_correction_validation_async_age_ms": lidar_correction.get("validation_async_age_ms", ""),
                "lidar_correction_validation_async_last_wall_ms": lidar_correction.get("validation_async_last_wall_ms", ""),
                "lidar_correction_validation_async_last_age_ms": lidar_correction.get("validation_async_last_age_ms", ""),
                "lidar_correction_validation_async_last_decision": lidar_correction.get("validation_async_last_decision", ""),
                "lidar_correction_validation_async_last_source": lidar_correction.get("validation_async_last_source", ""),
                "lidar_correction_intended_step_h": lidar_correction.get("correction_intended_step_h", ""),
                "lidar_correction_actual_anchor_delta_h": lidar_correction.get("correction_actual_anchor_delta_h", ""),
                "lidar_correction_unintended_anchor_delta_h": lidar_correction.get("correction_unintended_anchor_delta_h", ""),
                "lidar_correction_reference_local_h": lidar_correction.get("correction_reference_local_h", ""),
                "lidar_correction_apply_local_h": lidar_correction.get("correction_apply_local_h", ""),
                "lidar_correction_reference_pose_h": lidar_correction.get("correction_reference_pose_h", ""),
                "lidar_correction_anchor_h_before": lidar_correction.get("correction_anchor_h_before", ""),
                "lidar_correction_anchor_h_after": lidar_correction.get("correction_anchor_h_after", ""),
                "lidar_correction_reference_local_source": str(lidar_correction.get("correction_reference_local_source", "")),
                "lidar_correction_diagnostic_status_mode": str(lidar_correction.get("diagnostic_status_mode", "")),
                "lidar_correction_diagnostic_status_configured_hz": lidar_correction.get("diagnostic_status_configured_hz", ""),
                "lidar_correction_diagnostic_status_important": lidar_correction.get("diagnostic_status_important", ""),
                "lidar_correction_diagnostic_status_deferred_since_full": lidar_correction.get("diagnostic_status_deferred_since_full", ""),
                "lidar_correction_diagnostic_status_full_count": lidar_correction.get("diagnostic_status_full_count", ""),
                "lidar_correction_diagnostic_status_light_count": lidar_correction.get("diagnostic_status_light_count", ""),
                "lidar_correction_diagnostic_status_last_full_age_ms": lidar_correction.get("diagnostic_status_last_full_age_ms", ""),
                "lidar_scan_arrival_rate_hz": timing_value("lidar_scan_arrival", "rate_hz"),
                "lidar_scan_arrival_rate_1s_hz": timing_value("lidar_scan_arrival", "rate_1s_hz"),
                "lidar_scan_arrival_rate_3s_hz": timing_value("lidar_scan_arrival", "rate_3s_hz"),
                "lidar_scan_arrival_period_ms": timing_value("lidar_scan_arrival", "period_ms"),
                "lidar_process_ms": timing_value("lidar_process"),
                "lidar_process_rate_hz": timing_value("lidar_process", "rate_hz"),
                "lidar_process_rate_1s_hz": timing_value("lidar_process", "rate_1s_hz"),
                "lidar_process_rate_3s_hz": timing_value("lidar_process", "rate_3s_hz"),
                "lidar_process_max_1s_ms": timing_value("lidar_process", "max_1s_ms"),
                "lidar_process_max_3s_ms": timing_value("lidar_process", "max_3s_ms"),
                "lidar_stage_filter_ms": timing_value("lidar_stage_filter"),
                "lidar_stage_deskew_ms": timing_value("lidar_stage_deskew"),
                "lidar_stage_snapshot_ms": timing_value("lidar_stage_snapshot"),
                "lidar_stage_localization_correction_ms": timing_value("lidar_stage_localization_correction"),
                "lidar_stage_pose_estimate_ms": timing_value("lidar_stage_pose_estimate"),
                "lidar_stage_obstacles_ms": timing_value("lidar_stage_obstacles"),
                "lidar_stage_color_enqueue_ms": timing_value("lidar_stage_color_enqueue"),
                "lidar_stage_color_tracks_ms": timing_value("lidar_stage_color_tracks"),
                "lidar_stage_runtime_update_ms": timing_value("lidar_stage_runtime_update"),
                "obstacle_known_before_ms": runtime_metric.get("obstacle_known_before_ms", ""),
                "obstacle_known_before_process_cpu_ms": runtime_metric.get("obstacle_known_before_process_cpu_ms", ""),
                "obstacle_known_before_thread_cpu_ms": runtime_metric.get("obstacle_known_before_thread_cpu_ms", ""),
                "obstacle_detection_allowed_ms": runtime_metric.get("obstacle_detection_allowed_ms", ""),
                "obstacle_detection_allowed_process_cpu_ms": runtime_metric.get("obstacle_detection_allowed_process_cpu_ms", ""),
                "obstacle_detection_allowed_thread_cpu_ms": runtime_metric.get("obstacle_detection_allowed_thread_cpu_ms", ""),
                "obstacle_detect_slots_ms": runtime_metric.get("obstacle_detect_slots_ms", ""),
                "obstacle_detect_slots_process_cpu_ms": runtime_metric.get("obstacle_detect_slots_process_cpu_ms", ""),
                "obstacle_detect_slots_thread_cpu_ms": runtime_metric.get("obstacle_detect_slots_thread_cpu_ms", ""),
                "obstacle_track_update_ms": runtime_metric.get("obstacle_track_update_ms", ""),
                "obstacle_track_update_process_cpu_ms": runtime_metric.get("obstacle_track_update_process_cpu_ms", ""),
                "obstacle_track_update_thread_cpu_ms": runtime_metric.get("obstacle_track_update_thread_cpu_ms", ""),
                "obstacle_expected_visible_ms": runtime_metric.get("obstacle_expected_visible_ms", ""),
                "obstacle_expected_visible_process_cpu_ms": runtime_metric.get("obstacle_expected_visible_process_cpu_ms", ""),
                "obstacle_expected_visible_thread_cpu_ms": runtime_metric.get("obstacle_expected_visible_thread_cpu_ms", ""),
                "obstacle_absence_update_ms": runtime_metric.get("obstacle_absence_update_ms", ""),
                "obstacle_absence_update_process_cpu_ms": runtime_metric.get("obstacle_absence_update_process_cpu_ms", ""),
                "obstacle_absence_update_thread_cpu_ms": runtime_metric.get("obstacle_absence_update_thread_cpu_ms", ""),
                "obstacle_memory_refresh_ms": runtime_metric.get("obstacle_memory_refresh_ms", ""),
                "obstacle_finalize_results_ms": runtime_metric.get("obstacle_finalize_results_ms", ""),
                "obstacle_finalize_results_process_cpu_ms": runtime_metric.get("obstacle_finalize_results_process_cpu_ms", ""),
                "obstacle_finalize_results_thread_cpu_ms": runtime_metric.get("obstacle_finalize_results_thread_cpu_ms", ""),
                "runtime_perception_publish_ms": runtime_metric.get("runtime_perception_publish_ms", ""),
                "runtime_perception_publish_process_cpu_ms": runtime_metric.get("runtime_perception_publish_process_cpu_ms", ""),
                "runtime_perception_publish_thread_cpu_ms": runtime_metric.get("runtime_perception_publish_thread_cpu_ms", ""),
                "runtime_cloud_wall_ms": runtime_metric.get("runtime_cloud_wall_ms", ""),
                "runtime_cloud_wall_process_cpu_ms": runtime_metric.get("runtime_cloud_wall_process_cpu_ms", ""),
                "runtime_cloud_wall_thread_cpu_ms": runtime_metric.get("runtime_cloud_wall_thread_cpu_ms", ""),
                "runtime_free_race_inner_ms": runtime_metric.get("runtime_free_race_inner_ms", ""),
                "runtime_free_race_inner_process_cpu_ms": runtime_metric.get("runtime_free_race_inner_process_cpu_ms", ""),
                "runtime_free_race_inner_thread_cpu_ms": runtime_metric.get("runtime_free_race_inner_thread_cpu_ms", ""),
                "runtime_shared_publish_ms": runtime_metric.get("runtime_shared_publish_ms", ""),
                "runtime_shared_publish_process_cpu_ms": runtime_metric.get("runtime_shared_publish_process_cpu_ms", ""),
                "runtime_shared_publish_thread_cpu_ms": runtime_metric.get("runtime_shared_publish_thread_cpu_ms", ""),
                "runtime_publish_obstacle_copy_ms": runtime_metric.get("runtime_publish_obstacle_copy_ms", ""),
                "runtime_publish_scan_points_copy_ms": runtime_metric.get("runtime_publish_scan_points_copy_ms", ""),
                "runtime_publish_matched_points_copy_ms": runtime_metric.get("runtime_publish_matched_points_copy_ms", ""),
                "runtime_publish_rejected_points_copy_ms": runtime_metric.get("runtime_publish_rejected_points_copy_ms", ""),
                "runtime_publish_cloud_wall_copy_ms": runtime_metric.get("runtime_publish_cloud_wall_copy_ms", ""),
                "runtime_publish_free_race_copy_ms": runtime_metric.get("runtime_publish_free_race_copy_ms", ""),
                "runtime_publish_lock_assign_ms": runtime_metric.get("runtime_publish_lock_assign_ms", ""),
                "obstacle_detect_slot_generation_ms": runtime_metric.get("obstacle_detect_slot_generation_ms", ""),
                "obstacle_detect_pre_arrays_ms": runtime_metric.get("obstacle_detect_pre_arrays_ms", ""),
                "obstacle_detect_distance_matrix_ms": runtime_metric.get("obstacle_detect_distance_matrix_ms", ""),
                "obstacle_detect_distance_index_group_ms": runtime_metric.get("obstacle_detect_distance_index_group_ms", ""),
                "obstacle_detect_distance_hit_count": runtime_metric.get("obstacle_detect_distance_hit_count", ""),
                "obstacle_detect_loop_ms": runtime_metric.get("obstacle_detect_loop_ms", ""),
                "obstacle_detect_distance_ms": runtime_metric.get("obstacle_detect_distance_ms", ""),
                "obstacle_detect_cluster_ms": runtime_metric.get("obstacle_detect_cluster_ms", ""),
                "obstacle_detect_result_ms": runtime_metric.get("obstacle_detect_result_ms", ""),
                "obstacle_detect_empty_result_ms": runtime_metric.get("obstacle_detect_empty_result_ms", ""),
                "obstacle_detect_input_points": runtime_metric.get("obstacle_detect_input_points", ""),
                "obstacle_detect_slot_count": runtime_metric.get("obstacle_detect_slot_count", ""),
                "obstacle_detect_nonempty_slots": runtime_metric.get("obstacle_detect_nonempty_slots", ""),
                "obstacle_detect_current_slots": runtime_metric.get("obstacle_detect_current_slots", ""),
                "obstacle_expected_visible_filter_ms": runtime_metric.get("obstacle_expected_visible_filter_ms", ""),
                "obstacle_expected_visible_barrier_occlusion_ms": runtime_metric.get("obstacle_expected_visible_barrier_occlusion_ms", ""),
                "obstacle_expected_visible_obstacle_occlusion_ms": runtime_metric.get("obstacle_expected_visible_obstacle_occlusion_ms", ""),
                "obstacle_expected_visible_count": runtime_metric.get("obstacle_expected_visible_count", ""),
                "obstacle_expected_visible_barrier_checks": runtime_metric.get("obstacle_expected_visible_barrier_checks", ""),
                "obstacle_expected_visible_obstacle_checks": runtime_metric.get("obstacle_expected_visible_obstacle_checks", ""),
                "obstacle_expected_visible_range_rejected": runtime_metric.get("obstacle_expected_visible_range_rejected", ""),
                "obstacle_expected_visible_rear_rejected": runtime_metric.get("obstacle_expected_visible_rear_rejected", ""),
                "obstacle_expected_visible_barrier_rejected": runtime_metric.get("obstacle_expected_visible_barrier_rejected", ""),
                "obstacle_expected_visible_obstacle_rejected": runtime_metric.get("obstacle_expected_visible_obstacle_rejected", ""),
                "absence_pending_obstacles": runtime_metric.get("absence_pending_obstacles", ""),
                "color_points_prepare_ms": runtime_metric.get("color_points_prepare_ms", ""),
                "color_debug_meta_ms": runtime_metric.get("color_debug_meta_ms", ""),
                "color_drain_results_ms": runtime_metric.get("color_drain_results_ms", ""),
                "color_enqueue_clusters_ms": runtime_metric.get("color_enqueue_clusters_ms", ""),
                "color_pending_count_ms": runtime_metric.get("color_pending_count_ms", ""),
                "color_observation_map_ms": runtime_metric.get("color_observation_map_ms", ""),
                "color_measurement_buffer_ms": runtime_metric.get("color_measurement_buffer_ms", ""),
                "color_enqueue_request_count": runtime_metric.get("color_enqueue_request_count", ""),
                "color_sync_analysis_count": runtime_metric.get("color_sync_analysis_count", ""),
                "color_event_slot_count": runtime_metric.get("color_event_slot_count", ""),
                "lidar_stage_payload_build_ms": timing_value("lidar_stage_payload_build"),
                "web_payload_refresh_ms": timing_value("web_payload_refresh"),
                "web_payload_refresh_rate_1s_hz": timing_value("web_payload_refresh", "rate_1s_hz"),
                "web_payload_refresh_rate_3s_hz": timing_value("web_payload_refresh", "rate_3s_hz"),
                "web_payload_cached_ms": timing_value("web_payload_cached"),
                "web_payload_cached_rate_1s_hz": timing_value("web_payload_cached", "rate_1s_hz"),
                "lite_payload_ms": timing_value("lite_payload"),
                "lite_payload_rate_1s_hz": timing_value("lite_payload", "rate_1s_hz"),
                "lite_payload_rate_3s_hz": timing_value("lite_payload", "rate_3s_hz"),
                "lite_payload_json_ms": timing_value("lite_payload_json"),
                "lite_payload_json_rate_1s_hz": timing_value("lite_payload_json", "rate_1s_hz"),
                "lite_payload_json_rate_3s_hz": timing_value("lite_payload_json", "rate_3s_hz"),
                "web_payload_refresh_control_mode": str(timing_metric("web_payload_refresh_control", "mode")),
                "web_payload_refresh_control_configured_hz": timing_metric("web_payload_refresh_control", "configured_hz"),
                "web_payload_refresh_control_cached_count": timing_metric("web_payload_refresh_control", "cached_count"),
                "web_payload_refresh_control_last_full_age_ms": timing_metric("web_payload_refresh_control", "last_full_age_ms"),
                "lidar_runtime_payload_ms": timing_value("runtime_payload"),
                "control_payload_refresh_mode": str(timing_metric("control_payload_refresh", "mode")),
                "control_payload_refresh_configured_hz": timing_metric("control_payload_refresh", "configured_hz"),
                "control_payload_refresh_skipped_since_full": timing_metric("control_payload_refresh", "skipped_since_full"),
                "control_payload_refresh_last_full_age_ms": timing_metric("control_payload_refresh", "last_full_age_ms"),
                "control_payload_light_ms": timing_value("control_payload_light"),
                "control_payload_light_rate_1s_hz": timing_value("control_payload_light", "rate_1s_hz"),
                "control_payload_light_rate_3s_hz": timing_value("control_payload_light", "rate_3s_hz"),
                "control_payload_full_mode": str((payload.get("runtime", {}) if isinstance(payload.get("runtime", {}), dict) else {}).get("payload_refresh_mode", "")),
                "control_payload_full_updates": (payload.get("runtime", {}) if isinstance(payload.get("runtime", {}), dict) else {}).get("payload_control_full_updates", ""),
                "control_payload_light_updates": (payload.get("runtime", {}) if isinstance(payload.get("runtime", {}), dict) else {}).get("payload_control_light_updates", ""),
                "control_payload_skipped_updates": (payload.get("runtime", {}) if isinstance(payload.get("runtime", {}), dict) else {}).get("payload_control_skipped_updates", ""),
                "lidar_correction_status_update_ms": timing_value("lidar_correction_status_update"),
                "lidar_correction_status_update_rate_1s_hz": timing_value("lidar_correction_status_update", "rate_1s_hz"),
                "lidar_correction_status_publish_mode": str(timing_metric("lidar_correction_status_publish", "mode")),
                "lidar_correction_status_publish_configured_hz": timing_metric("lidar_correction_status_publish", "configured_hz"),
                "lidar_correction_status_publish_deferred_since_full": timing_metric("lidar_correction_status_publish", "deferred_since_full"),
                "lidar_correction_status_publish_important": timing_metric("lidar_correction_status_publish", "important"),
                "lidar_status_mode": timing_metric("lidar_correction_status_detail", "lidar_status_mode"),
                "lidar_status_reason": timing_metric("lidar_correction_status_detail", "lidar_status_reason"),
                "lidar_status_total_ms": timing_metric("lidar_correction_status_detail", "lidar_status_total_ms"),
                "lidar_status_update_lock_wait_ms": timing_metric("lidar_correction_status_detail", "lidar_status_update_lock_wait_ms"),
                "lidar_status_update_lock_inside_ms": timing_metric("lidar_correction_status_detail", "lidar_status_update_lock_inside_ms"),
                "lidar_status_record_timing_ms": timing_metric("lidar_correction_status_detail", "lidar_status_record_timing_ms"),
                "lidar_status_record_metric_ms": timing_metric("lidar_correction_status_detail", "lidar_status_record_metric_ms"),
                "scheduler_probe_enabled": timing_metric("scheduler_probe", "enabled"),
                "scheduler_probe_late_ms": timing_metric("scheduler_probe", "late_ms"),
                "scheduler_probe_late_p50_1s_ms": timing_metric("scheduler_probe", "late_p50_1s_ms"),
                "scheduler_probe_late_p95_1s_ms": timing_metric("scheduler_probe", "late_p95_1s_ms"),
                "scheduler_probe_late_max_1s_ms": timing_metric("scheduler_probe", "late_max_1s_ms"),
                "scheduler_probe_late_p50_3s_ms": timing_metric("scheduler_probe", "late_p50_3s_ms"),
                "scheduler_probe_late_p95_3s_ms": timing_metric("scheduler_probe", "late_p95_3s_ms"),
                "scheduler_probe_late_max_3s_ms": timing_metric("scheduler_probe", "late_max_3s_ms"),
                "scheduler_probe_rate_hz": timing_metric("scheduler_probe", "rate_hz"),
                "lidar_points_raw": lidar_points.get("raw", ""),
                "lidar_points_after_range_filter": lidar_points.get("after_range_filter", ""),
                "lidar_points_kept": lidar_points.get("kept", ""),
                "lidar_points_dropped_by_max_points": lidar_points.get("dropped_by_max_points", ""),
                "lidar_timestamp_scan_start_age_ms": lidar_timestamp.get("scan_start_age_ms", ""),
                "lidar_timestamp_scan_mid_age_ms": lidar_timestamp.get("scan_mid_age_ms", ""),
                "lidar_timestamp_scan_end_age_ms": lidar_timestamp.get("scan_end_age_ms", ""),
                "lidar_timestamp_scan_duration_ms": lidar_timestamp.get("scan_duration_ms", ""),
                "lidar_timestamp_source": lidar_timestamp.get("timestamp_source", ""),
                "lidar_beam_time_order": lidar_timestamp.get("beam_time_order", ""),
                "nav_pose_confidence_ok": int(bool(nav.get("pose_confidence_ok", False))),
                "nav_pose_confidence_progress_allowed": int(bool(nav.get("pose_confidence_progress_allowed", False))),
                "nav_pose_confidence_progress_reason": str(nav.get("pose_confidence_progress_reason", "")),
                "nav_pose_confidence": nav.get("pose_confidence", ""),
                "nav_projection_error_m": nav.get("projection_error_m", ""),
                "nav_lateral_m": nav.get("lateral_m", ""),
                "nav_lateral_fused_m": nav_lateral_fused_m,
                "nav_lateral_otos_only_m": nav_lateral_otos_only_m,
                "nav_lateral_fused_minus_otos_m": nav_lateral_fused_minus_otos_m,
                "nav_heading_error_deg": nav.get("heading_error_deg", ""),
                "nav_enabled": int(bool(nav.get("enabled", False))),
                "nav_confirmations_active": int(bool(nav.get("confirmations_active", False))),
                "nav_confirmations_active_reason": str(nav.get("confirmations_active_reason", "")),
                "nav_timing_warning": str(nav.get("timing_warning", "")),
                "nav_full_gate_lock": int(bool(nav.get("full_gate_lock", False))),
                "nav_locked_gate_count": nav.get("locked_gate_count", len(nav.get("locked_gate_choices", []) or [])),
                "nav_local_connector_mode": str(nav.get("local_connector_mode", "")),
                "nav_blocked_route_fallback_active": int(bool(nav.get("blocked_route_fallback_active", False))),
                "nav_blocked_route_fallback_age_s": nav.get("blocked_route_fallback_age_s", ""),
                "nav_blocked_route_fallback_reason": str(nav.get("blocked_route_fallback_reason", "")),
                "nav_blocked_route_fallback_projection_error_m": nav.get("blocked_route_fallback_projection_error_m", ""),
                "nav_warnings": compact_list(nav.get("warnings", [])),
                "nav_constraint_warnings": compact_list(nav.get("constraint_warnings", [])),
                "nav_target_waypoint_ids": compact_list(nav.get("target_waypoint_ids", [])),
                "nav_target_waypoint_reasons": compact_list(nav.get("target_waypoint_reasons", [])),
                "nav_active_waypoint_id": str(nav.get("active_waypoint_id", "")),
                "nav_active_waypoint_reason": str(nav.get("active_waypoint_reason", "")),
                "nav_active_waypoint_distance_m": nav.get("active_waypoint_distance_m", ""),
                "nav_active_waypoint_signed_forward_m": nav.get("active_waypoint_signed_forward_m", ""),
                "nav_connector_target_id": str(nav.get("connector_target_id", "")),
                "nav_connector_target_reason": str(nav.get("connector_target_reason", "")),
                "nav_connector_chord_m": nav.get("connector_chord_m", ""),
                "nav_connector_heading_delta_deg": nav.get("connector_heading_delta_deg", ""),
                "nav_connector_target_route_delta_m": nav.get("connector_target_route_delta_m", ""),
                "nav_connector_start_lateral_m": nav.get("connector_start_lateral_m", ""),
                "nav_connector_start_heading_error_deg": nav.get("connector_start_heading_error_deg", ""),
                "nav_connector_attempt_count": nav.get("connector_attempt_count", ""),
                "nav_connector_failure_reasons": compact_list(nav.get("connector_failure_reasons", [])),
                "nav_connector_attempts": compact_json(nav.get("connector_attempts", [])),
                "nav_fixed_route_failed": int(bool(nav.get("fixed_route_failed", False))),
                "nav_fixed_route_warnings": compact_list(nav.get("fixed_route_warnings", [])),
                "nav_extension_warnings": compact_list(nav.get("extension_warnings", [])),
                "nav_planner_diagnostics": compact_json(nav.get("planner_diagnostics", {})),
                "nav_speed_reason": str(nav.get("speed_reason", "")),
                "nav_speed_cap_mps": nav.get("speed_cap_mps", ""),
                "nav_speed_policy": str(nav.get("speed_policy", "")),
                "nav_speed_policy_base_mps": nav.get("speed_policy_base_mps", ""),
                "nav_speed_policy_cap_factor": nav.get("speed_policy_cap_factor", ""),
                "nav_speed_policy_cap_reason": str(nav.get("speed_policy_cap_reason", "")),
                "nav_speed_policy_next_obstacle_id": str(nav.get("speed_policy_next_obstacle_id", "")),
                "nav_speed_policy_next_obstacle_delta_m": nav.get("speed_policy_next_obstacle_delta_m", ""),
                "nav_speed_policy_next_obstacle_color": str(nav.get("speed_policy_next_obstacle_color", "")),
                "nav_speed_policy_next_obstacle_gate_locked": int(bool(nav.get("speed_policy_next_obstacle_gate_locked", False))),
                "nav_speed_policy_current_section": str(nav.get("speed_policy_current_section", "")),
                "nav_speed_policy_current_section_ready": int(bool(nav.get("speed_policy_current_section_ready", False))),
                "nav_speed_policy_current_section_has_obstacle": int(bool(nav.get("speed_policy_current_section_has_obstacle", False))),
                "nav_speed_policy_current_section_obstacle_id": str(nav.get("speed_policy_current_section_obstacle_id", "")),
                "nav_speed_policy_current_section_obstacle_color": str(nav.get("speed_policy_current_section_obstacle_color", "")),
                "nav_speed_policy_next_section": str(nav.get("speed_policy_next_section", "")),
                "nav_speed_policy_next_section_ready": int(bool(nav.get("speed_policy_next_section_ready", False))),
                "nav_speed_policy_next_section_has_obstacle": int(bool(nav.get("speed_policy_next_section_has_obstacle", False))),
                "nav_speed_policy_next_section_obstacle_id": str(nav.get("speed_policy_next_section_obstacle_id", "")),
                "nav_speed_policy_next_section_obstacle_color": str(nav.get("speed_policy_next_section_obstacle_color", "")),
                "nav_obstacle_speed_lookahead_m": nav.get("obstacle_speed_lookahead_m", ""),
                "nav_obstacle_speed_lookahead_forward_mps": nav.get("obstacle_speed_lookahead_forward_mps", ""),
                "nav_obstacle_speed_lookahead_scale": nav.get("obstacle_speed_lookahead_scale", ""),
                "nav_tracking_error_speed_factor": nav.get("tracking_error_speed_factor", ""),
                "nav_tracking_error_speed_reason": str(nav.get("tracking_error_speed_reason", "")),
                "nav_corner_transition_speed_factor": nav.get("corner_transition_speed_factor", ""),
                "nav_corner_transition_speed_cap_mps": nav.get("corner_transition_speed_cap_mps", ""),
                "nav_corner_transition_speed_active": int(bool(nav.get("corner_transition_speed_active", False))),
                "nav_corner_transition_speed_reason": str(nav.get("corner_transition_speed_reason", "")),
                "nav_hold": int(bool(nav.get("hold", False))),
                "nav_hold_reason": str(nav.get("hold_reason", "")),
                "nav_hold_section": str(nav.get("hold_section", "")),
                "nav_hold_remaining_s": nav.get("hold_remaining_s", ""),
                "localization_hold_active": int(bool(localization_hold.get("active", False))),
                "localization_hold_reason": str(localization_hold.get("reason", "")),
                "localization_hold_release_reason": str(localization_hold.get("release_reason", "")),
                "localization_hold_remaining_s": localization_hold.get("remaining_s", ""),
                "localization_hold_min_remaining_s": localization_hold.get("min_remaining_s", ""),
                "localization_hold_timeout_remaining_s": localization_hold.get("timeout_remaining_s", ""),
                "localization_hold_stationary": int(bool(localization_hold.get("stationary", False))),
                "localization_hold_stationary_speed_mps": localization_hold.get("stationary_speed_mps", ""),
                "localization_hold_wide_reacquire": int(bool(localization_hold.get("wide_reacquire", False))),
                "localization_hold_effective_axis_max_error_m": localization_hold.get("effective_axis_max_error_m", ""),
                "localization_hold_base_axis_max_error_m": localization_hold.get("base_axis_max_error_m", ""),
                "localization_hold_missing_xy_count": localization_hold.get("missing_xy_count", ""),
                "localization_hold_valid_xy_count": localization_hold.get("valid_xy_count", ""),
                "localization_hold_settled_count": localization_hold.get("settled_count", ""),
                "localization_hold_x_measured": int(bool(localization_hold.get("x_measured", False))),
                "localization_hold_y_measured": int(bool(localization_hold.get("y_measured", False))),
                "localization_hold_x_residual_m": localization_hold.get("x_residual_m", ""),
                "localization_hold_y_residual_m": localization_hold.get("y_residual_m", ""),
                "localization_hold_x_step_m": localization_hold.get("x_step_m", ""),
                "localization_hold_y_step_m": localization_hold.get("y_step_m", ""),
                "localization_hold_trigger_source": str(localization_hold.get("trigger_source", "")),
                "localization_hold_cooldown_remaining_s": localization_hold.get("cooldown_remaining_s", ""),
                "localization_hold_no_current_trigger_enabled": int(bool(localization_hold.get("no_current_trigger_enabled", False))),
                "localization_hold_no_current_triggered": int(bool(localization_hold.get("no_current_triggered", False))),
                "localization_hold_no_current_trigger_axis": str(localization_hold.get("no_current_trigger_axis", "")),
                "localization_hold_no_current_suppressed_reason": str(localization_hold.get("no_current_suppressed_reason", "")),
                "localization_hold_no_current_window_count": localization_hold.get("no_current_window_count", ""),
                "localization_hold_no_current_window_size": localization_hold.get("no_current_window_size", ""),
                "localization_hold_no_current_required": localization_hold.get("no_current_required", ""),
                "localization_hold_no_current_x_count": localization_hold.get("no_current_x_count", ""),
                "localization_hold_no_current_y_count": localization_hold.get("no_current_y_count", ""),
                "localization_hold_no_current_x_missing": int(bool(localization_hold.get("no_current_x_missing", False))),
                "localization_hold_no_current_y_missing": int(bool(localization_hold.get("no_current_y_missing", False))),
                "localization_hold_no_current_section_visit_key": str(localization_hold.get("no_current_section_visit_key", "")),
                "localization_hold_no_current_section_visit_count": localization_hold.get("no_current_section_visit_count", ""),
                "localization_hold_no_current_section_visit_limit": localization_hold.get("no_current_section_visit_limit", ""),
                "localization_hold_no_current_event_count": localization_hold.get("no_current_event_count", ""),
                "localization_hold_low_conf_trigger_enabled": int(bool(localization_hold.get("low_conf_trigger_enabled", False))),
                "localization_hold_low_conf_triggered": int(bool(localization_hold.get("low_conf_triggered", False))),
                "localization_hold_low_conf_suppressed_reason": str(localization_hold.get("low_conf_suppressed_reason", "")),
                "localization_hold_low_conf_window_count": localization_hold.get("low_conf_window_count", ""),
                "localization_hold_low_conf_window_size": localization_hold.get("low_conf_window_size", ""),
                "localization_hold_low_conf_required": localization_hold.get("low_conf_required", ""),
                "localization_hold_low_conf_low_count": localization_hold.get("low_conf_low_count", ""),
                "localization_hold_low_conf_last_confidence": localization_hold.get("low_conf_last_confidence", ""),
                "localization_hold_low_conf_last_valid": int(bool(localization_hold.get("low_conf_last_valid", False))),
                "localization_hold_low_conf_last_reason": str(localization_hold.get("low_conf_last_reason", "")),
                "localization_hold_low_conf_section_visit_key": str(localization_hold.get("low_conf_section_visit_key", "")),
                "localization_hold_low_conf_section_visit_count": localization_hold.get("low_conf_section_visit_count", ""),
                "localization_hold_low_conf_section_visit_limit": localization_hold.get("low_conf_section_visit_limit", ""),
                "localization_hold_low_conf_event_count": localization_hold.get("low_conf_event_count", ""),
                "localization_hold_low_conf_startup_grace_remaining_s": localization_hold.get("low_conf_startup_grace_remaining_s", ""),
                "localization_hold_low_conf_progress_allowed": int(bool(localization_hold.get("low_conf_progress_allowed", False))),
                "localization_hold_low_conf_progress_reason": str(localization_hold.get("low_conf_progress_reason", "")),
                "localization_hold_low_conf_progress_started_t": localization_hold.get("low_conf_progress_started_t", ""),
                "localization_hold_low_conf_progress_trigger_source": str(localization_hold.get("low_conf_progress_trigger_source", "")),
                "localization_hold_timeout_progress_allowed": int(bool(localization_hold.get("timeout_progress_allowed", False))),
                "localization_hold_timeout_progress_reason": str(localization_hold.get("timeout_progress_reason", "")),
                "localization_hold_timeout_progress_started_t": localization_hold.get("timeout_progress_started_t", ""),
                "localization_hold_timeout_progress_trigger_source": str(localization_hold.get("timeout_progress_trigger_source", "")),
                "localization_hold_timeout_progress_clear_reason": str(localization_hold.get("timeout_progress_clear_reason", "")),
                "localization_hold_timeout_progress_clear_window_count": localization_hold.get("timeout_progress_clear_window_count", ""),
                "localization_hold_timeout_progress_clear_window_size": localization_hold.get("timeout_progress_clear_window_size", ""),
                "localization_hold_timeout_progress_clear_required": localization_hold.get("timeout_progress_clear_required", ""),
                "localization_hold_timeout_progress_clear_settled_count": localization_hold.get("timeout_progress_clear_settled_count", ""),
                "nav_blocked": int(bool(nav.get("blocked", False))),
                "nav_blocked_reason": str(nav.get("blocked_reason", "")),
                "nav_observation_approach": int(bool(nav.get("observation_approach", False))),
                "nav_detect_enabled": int(bool(obstacles.get("detection_enabled", False))),
                "obstacle_detection_yaw_gate_enabled": int(bool((obstacles.get("detection_yaw_gate", {}) if isinstance(obstacles.get("detection_yaw_gate", {}), dict) else {}).get("enabled", False))),
                "obstacle_detection_yaw_rate_dps": (obstacles.get("detection_yaw_gate", {}) if isinstance(obstacles.get("detection_yaw_gate", {}), dict) else {}).get("yaw_rate_dps", ""),
                "obstacle_detection_yaw_rate_limit_dps": (obstacles.get("detection_yaw_gate", {}) if isinstance(obstacles.get("detection_yaw_gate", {}), dict) else {}).get("max_yaw_rate_dps", ""),
                "obstacle_detection_yaw_rate_gated": int(bool((obstacles.get("detection_yaw_gate", {}) if isinstance(obstacles.get("detection_yaw_gate", {}), dict) else {}).get("gated", False))),
                "obstacle_detection_yaw_gate_reason": str((obstacles.get("detection_yaw_gate", {}) if isinstance(obstacles.get("detection_yaw_gate", {}), dict) else {}).get("reason", "")),
                "nav_direction": str(nav.get("direction", "")),
                "nav_section": str(nav.get("section", "")),
                "free_race_inner_enabled": int(bool(free_race_inner.get("enabled", False))),
                "free_race_inner_locked": int(bool(free_race_inner.get("locked", False))),
                "free_race_inner_route_active": int(bool(free_race_inner_route.get("active", False))),
                "free_race_inner_start_section_active": int(bool(free_race_inner_route.get("start_section_active", False))),
                "free_race_inner_applied_active": int(bool(free_race_inner_route.get("applied_active", False))),
                "free_race_inner_applied_summary": str(free_race_inner_route.get("applied_summary", "")),
                "free_race_inner_pending_summary": str(free_race_inner_route.get("pending_summary", "")),
                "free_race_inner_apply_generation": free_race_inner_route.get("apply_generation", ""),
                "free_race_inner_apply_reason": str(free_race_inner_route.get("apply_reason", "")),
                "free_race_inner_bottom_applied_lane": str(free_inner_route_section(free_race_inner_applied_sections, "bottom", "lane_name")),
                "free_race_inner_left_applied_lane": str(free_inner_route_section(free_race_inner_applied_sections, "left", "lane_name")),
                "free_race_inner_top_applied_lane": str(free_inner_route_section(free_race_inner_applied_sections, "top", "lane_name")),
                "free_race_inner_right_applied_lane": str(free_inner_route_section(free_race_inner_applied_sections, "right", "lane_name")),
                "free_race_inner_bottom_pending_lane": str(free_inner_route_section(free_race_inner_pending_sections, "bottom", "lane_name")),
                "free_race_inner_left_pending_lane": str(free_inner_route_section(free_race_inner_pending_sections, "left", "lane_name")),
                "free_race_inner_top_pending_lane": str(free_inner_route_section(free_race_inner_pending_sections, "top", "lane_name")),
                "free_race_inner_right_pending_lane": str(free_inner_route_section(free_race_inner_pending_sections, "right", "lane_name")),
                "free_race_inner_last_sample_reason": str(free_race_inner.get("last_sample_reason", "")),
                "free_race_inner_last_section": str(free_race_inner.get("last_sample_section", "")),
                "free_race_inner_last_gap_m": free_race_inner.get("last_gap_m", ""),
                "free_race_inner_last_left_m": free_race_inner.get("last_left_m", ""),
                "free_race_inner_last_right_m": free_race_inner.get("last_right_m", ""),
                "free_race_inner_last_left_count": free_race_inner.get("last_left_count", ""),
                "free_race_inner_last_right_count": free_race_inner.get("last_right_count", ""),
                "free_race_inner_last_scan_t": free_race_inner.get("last_scan_t", ""),
                "free_race_inner_last_probe_mode": str(free_race_inner.get("last_probe_mode", "")),
                "free_race_inner_last_sample_source": str(free_race_inner.get("last_sample_source", "")),
                "free_race_inner_summary": str(free_race_inner.get("summary", "")),
                "free_race_inner_bottom_class": str(free_inner_section("bottom", "classification")),
                "free_race_inner_bottom_samples": free_inner_section("bottom", "samples"),
                "free_race_inner_bottom_far_count": free_inner_section("bottom", "far_count"),
                "free_race_inner_bottom_close_count": free_inner_section("bottom", "close_count"),
                "free_race_inner_bottom_median_gap_m": free_inner_section("bottom", "median_gap_m"),
                "free_race_inner_left_class": str(free_inner_section("left", "classification")),
                "free_race_inner_left_samples": free_inner_section("left", "samples"),
                "free_race_inner_left_far_count": free_inner_section("left", "far_count"),
                "free_race_inner_left_close_count": free_inner_section("left", "close_count"),
                "free_race_inner_left_median_gap_m": free_inner_section("left", "median_gap_m"),
                "free_race_inner_top_class": str(free_inner_section("top", "classification")),
                "free_race_inner_top_samples": free_inner_section("top", "samples"),
                "free_race_inner_top_far_count": free_inner_section("top", "far_count"),
                "free_race_inner_top_close_count": free_inner_section("top", "close_count"),
                "free_race_inner_top_median_gap_m": free_inner_section("top", "median_gap_m"),
                "free_race_inner_right_class": str(free_inner_section("right", "classification")),
                "free_race_inner_right_samples": free_inner_section("right", "samples"),
                "free_race_inner_right_far_count": free_inner_section("right", "far_count"),
                "free_race_inner_right_close_count": free_inner_section("right", "close_count"),
                "free_race_inner_right_median_gap_m": free_inner_section("right", "median_gap_m"),
                "nav_route_progress_m": nav.get("route_progress_m", ""),
                "nav_route_step_delta_m": nav.get("route_step_delta_m", ""),
                "nav_route_progress_mutations_allowed": int(bool(nav.get("route_progress_mutations_allowed", True))),
                "nav_route_progress_baseline_synced": int(bool(nav.get("route_progress_baseline_synced", False))),
                "nav_route_progress_suppressed_reason": nav.get("route_progress_suppressed_reason", ""),
                "nav_route_progress_suppressed_delta_m": nav.get("route_progress_suppressed_delta_m", ""),
                "nav_gate_lock_last_event": nav.get("gate_lock_last_event", ""),
                "nav_gate_lock_last_section": nav.get("gate_lock_last_section", ""),
                "nav_gate_lock_last_gate_idx": nav.get("gate_lock_last_gate_idx", ""),
                "nav_gate_lock_last_lane_name": nav.get("gate_lock_last_lane_name", ""),
                "nav_gate_lock_last_reason": nav.get("gate_lock_last_reason", ""),
                "nav_gate_lock_last_decision_reason": nav.get("gate_lock_last_decision_reason", ""),
                "nav_gate_lock_last_obstacle_ids": nav.get("gate_lock_last_obstacle_ids", ""),
                "nav_gate_lock_last_route_progress_m": nav.get("gate_lock_last_route_progress_m", ""),
                "nav_gate_lock_last_route_step_delta_m": nav.get("gate_lock_last_route_step_delta_m", ""),
                "nav_gate_lock_last_active_waypoint_id": nav.get("gate_lock_last_active_waypoint_id", ""),
                "nav_gate_lock_last_startup_status": nav.get("gate_lock_last_startup_status", ""),
                "nav_gate_lock_last_startup_state": nav.get("gate_lock_last_startup_state", ""),
                "nav_gate_lock_last_anchor_version": nav.get("gate_lock_last_anchor_version", ""),
                "nav_gate_lock_last_anchor_source": nav.get("gate_lock_last_anchor_source", ""),
                "nav_gate_lock_last_anchor_jump_m": nav.get("gate_lock_last_anchor_jump_m", ""),
                "nav_gate_lock_last_anchor_jump_h_deg": nav.get("gate_lock_last_anchor_jump_h_deg", ""),
                "nav_gate_lock_last_initial_pose_locked": int(bool(nav.get("gate_lock_last_initial_pose_locked", True))),
                "nav_gate_lock_event_count": nav.get("gate_lock_event_count", ""),
                "nav_gate_lock_recent_events": nav.get("gate_lock_recent_events", ""),
                "nav_gate_lock_suppressed_reason": nav.get("gate_lock_suppressed_reason", ""),
                "nav_obstacle_mode_max_speed_mps": nav.get("obstacle_mode_max_speed_mps", ""),
                "nav_smart_parking_completed_laps": smart_trigger.get("completed_laps", ""),
                "nav_smart_parking_lap_complete": int(bool(smart_trigger.get("lap_complete", False))),
                "nav_smart_parking_trigger": int(bool(smart_trigger.get("trigger", False))),
                "nav_smart_parking_first_gate_passed": int(bool(smart_trigger.get("first_gate_passed", False))),
                "nav_smart_parking_counted_now": int(bool(smart_trigger.get("counted_now", False))),
                "nav_smart_parking_left_start_section": int(bool(smart_trigger.get("left_start_section", False))),
                "nav_smart_parking_gate_delta_m": smart_trigger.get("first_gate_delta_m", ""),
                "nav_smart_parking_trigger_margin_m": smart_trigger.get("trigger_margin_m", ""),
                "nav_smart_parking_route_progress_complete": int(bool(smart_trigger.get("route_progress_lap_complete", False))),
                "nav_smart_parking_target_progress_m": smart_trigger.get("target_progress_m", ""),
                "nav_free_finish_completed_laps": free_finish.get("completed_laps", ""),
                "nav_free_finish_lap_complete": int(bool(free_finish.get("lap_complete", False))),
                "nav_free_finish_trigger": int(bool(free_finish.get("trigger", False))),
                "nav_free_finish_first_gate_passed": int(bool(free_finish.get("first_gate_passed", False))),
                "nav_free_finish_counted_now": int(bool(free_finish.get("counted_now", False))),
                "nav_free_finish_left_start_section": int(bool(free_finish.get("left_start_section", False))),
                "nav_free_finish_gate_delta_m": free_finish.get("first_gate_delta_m", ""),
                "nav_free_finish_trigger_margin_m": free_finish.get("trigger_margin_m", ""),
                "nav_free_finish_route_progress_complete": int(bool(free_finish.get("route_progress_lap_complete", False))),
                "nav_free_finish_target_progress_m": free_finish.get("target_progress_m", ""),
                "nav_party_mode": int(bool(nav.get("party_mode", False))),
                "nav_post_unpark_start_confirmation_reason": str(post_unpark_start_confirmation.get("reason", "")),
                "nav_post_unpark_start_confirmation_triggered": int(bool(post_unpark_start_confirmation.get("triggered", False))),
                "nav_post_unpark_start_confirmation_party_mode": int(bool(post_unpark_start_confirmation.get("party_mode", False))),
                "nav_post_unpark_start_confirmation_source": str(post_unpark_start_confirmation.get("source", "")),
                "nav_post_unpark_start_confirmation_obstacle_count": post_unpark_start_confirmation.get("obstacle_count", ""),
                "nav_post_unpark_start_confirmation_relevant_count": post_unpark_start_confirmation.get("relevant_obstacle_count", ""),
                "nav_post_unpark_start_confirmation_candidate_count": post_unpark_start_confirmation.get("candidate_count", ""),
                "nav_post_unpark_start_confirmation_slot": str(post_unpark_start_confirmation.get("slot_id", "")),
                "nav_post_unpark_start_confirmation_slot_in_relevant": int(bool(post_unpark_start_confirmation.get("slot_in_relevant", False))),
                "nav_post_unpark_start_confirmation_slot_missing_from_relevant": int(bool(post_unpark_start_confirmation.get("slot_missing_from_relevant", False))),
                "nav_post_unpark_start_confirmation_injected_candidate": int(bool(post_unpark_start_confirmation.get("injected_candidate", False))),
                "nav_post_unpark_start_confirmation_injected_candidate_reject_reason": str(post_unpark_start_confirmation.get("injected_candidate_reject_reason", "")),
                "nav_active_waypoint_id": str(nav.get("active_waypoint_id", "")),
                "nav_active_waypoint_reason": str(nav.get("active_waypoint_reason", "")),
                "nav_active_waypoint_passed_reason": str(nav.get("active_waypoint_passed_reason", "")),
                "nav_active_waypoint_reset_reason": str(nav.get("active_waypoint_reset_reason", "")),
                "nav_active_waypoint_reset_previous_id": str(nav.get("active_waypoint_reset_previous_id", "")),
                "nav_color_confirmation_slot": str(color_confirmation.get("slot_id", "")),
                "nav_color_confirmation_proposal": str(color_confirmation.get("proposal", "")),
                "nav_color_confirmation_mode": str(color_confirmation.get("mode", "")),
                "nav_color_confirmation_trigger_source": str(color_confirmation.get("trigger_source", "")),
                "nav_color_confirmation_committed_start": str(color_confirmation.get("committed_start", "")),
                "nav_color_confirmation_committed_at_decision": str(color_confirmation.get("committed_at_decision", "")),
                "nav_color_confirmation_decision_color": str(color_confirmation.get("decision_color", "")),
                "nav_color_confirmation_decision_source": str(color_confirmation.get("decision_source", "")),
                "nav_color_confirmation_decision_confidence": color_confirmation.get("decision_confidence", ""),
                "nav_color_confirmation_decision_rejected": int(bool(color_confirmation.get("decision_rejected", False))),
                "nav_color_confirmation_decision_reject_reason": str(color_confirmation.get("decision_reject_reason", "")),
                "nav_color_confirmation_sample_count": color_confirmation.get("sample_count", ""),
                "nav_color_confirmation_all_sample_count": color_confirmation.get("all_sample_count", ""),
                "nav_color_confirmation_red_count": color_counts.get("RED", ""),
                "nav_color_confirmation_green_count": color_counts.get("GREEN", ""),
                "nav_color_confirmation_other_count": color_counts.get("OTHER", ""),
                "nav_color_confirmation_all_red_count": color_all_counts.get("RED", ""),
                "nav_color_confirmation_all_green_count": color_all_counts.get("GREEN", ""),
                "nav_color_confirmation_all_other_count": color_all_counts.get("OTHER", ""),
                "nav_color_confirmation_attempt": color_confirmation.get("attempt", ""),
                "nav_color_confirmation_exposure_retry": int(bool(color_confirmation.get("exposure_retry", False))),
                "nav_color_confirmation_exposure_retry_candidate": int(bool(color_confirmation.get("exposure_retry_candidate", False))),
                "nav_color_confirmation_exposure_retry_reason": str(color_confirmation.get("exposure_retry_reason", "")),
                "nav_color_confirmation_exposure_retry_eval_reason": str(color_confirmation.get("exposure_retry_eval_reason", "")),
                "nav_color_confirmation_exposure_retry_sample_count": color_confirmation.get("exposure_retry_sample_count", ""),
                "nav_color_confirmation_exposure_retry_min_samples": color_confirmation.get("exposure_retry_min_samples", ""),
                "nav_color_confirmation_exposure_retry_color_vote_count": color_confirmation.get("exposure_retry_color_vote_count", ""),
                "nav_color_confirmation_exposure_retry_color_ratio": color_confirmation.get("exposure_retry_color_ratio", ""),
                "nav_color_confirmation_exposure_retry_required_vote_count": color_confirmation.get("exposure_retry_required_vote_count", ""),
                "nav_color_confirmation_exposure_retry_bright_vote_count": color_confirmation.get("exposure_retry_bright_vote_count", ""),
                "nav_color_confirmation_exposure_retry_dark_vote_count": color_confirmation.get("exposure_retry_dark_vote_count", ""),
                "nav_color_confirmation_exposure_retry_low_valid_count": color_confirmation.get("exposure_retry_low_valid_count", ""),
                "nav_color_confirmation_exposure_retry_current_gain": color_confirmation.get("exposure_retry_current_gain", ""),
                "nav_color_confirmation_exposure_retry_next_gain": color_confirmation.get("exposure_retry_next_gain", ""),
                "nav_color_confirmation_stationary": int(bool(color_confirmation.get("stationary", False))),
                "nav_color_confirmation_stationary_speed_mps": color_confirmation.get("stationary_speed_mps", ""),
                "nav_color_confirmation_stationary_yaw_rate_dps": color_confirmation.get("stationary_yaw_rate_dps", ""),
                "nav_color_confirmation_sample_added": int(bool(color_confirmation.get("sample_added", False))),
                "nav_color_confirmation_sample_added_count": color_confirmation.get("sample_added_count", ""),
                "nav_color_confirmation_active_obstacle_in_relevant": int(bool(color_confirmation.get("active_obstacle_in_relevant", False))),
                "nav_color_confirmation_active_obstacle_in_measurement_source": int(bool(color_confirmation.get("active_obstacle_in_measurement_source", False))),
                "nav_color_confirmation_active_obstacle_in_decision_source": int(bool(color_confirmation.get("active_obstacle_in_decision_source", False))),
                "nav_color_confirmation_decision_obstacle_source": str(color_confirmation.get("decision_obstacle_source", "")),
                "nav_color_confirmation_measurement_source": str(color_confirmation.get("measurement_source", "")),
                "nav_color_confirmation_measurement_blocked_reason": str(color_confirmation.get("measurement_blocked_reason", "")),
                "nav_color_confirmation_measurement_relevant_events_available": color_confirmation.get("measurement_relevant_events_available", ""),
                "nav_color_confirmation_measurement_full_events_available": color_confirmation.get("measurement_full_events_available", ""),
                "nav_color_confirmation_measurement_events_available": color_confirmation.get("measurement_events_available", ""),
                "nav_color_confirmation_measurement_events_added": color_confirmation.get("measurement_events_added", ""),
                "nav_color_confirmation_measurement_events_duplicate": color_confirmation.get("measurement_events_duplicate", ""),
                "nav_color_confirmation_measurement_events_not_real": color_confirmation.get("measurement_events_not_real", ""),
                "nav_color_confirmation_measurement_events_stale": color_confirmation.get("measurement_events_stale", ""),
                "nav_color_confirmation_measurement_events_before_hold": color_confirmation.get("measurement_events_before_hold", ""),
                "nav_color_confirmation_measurement_events_gain_settle": color_confirmation.get("measurement_events_gain_settle", ""),
                "nav_color_confirmation_measurement_events_missing_id": color_confirmation.get("measurement_events_missing_id", ""),
                "nav_color_confirmation_last_measurement_id": color_confirmation.get("last_measurement_id", ""),
                "nav_color_confirmation_last_sample_label": color_confirmation.get("last_sample_label", ""),
                "nav_color_confirmation_last_skip_reason": color_confirmation.get("last_skip_reason", ""),
                "nav_color_confirmation_delta_m_at_decision": color_confirmation.get("delta_m_at_decision", ""),
                "nav_color_confirmation_los_distance_m_at_decision": color_confirmation.get("los_distance_m_at_decision", ""),
                "nav_color_confirmation_section": str(color_confirmation.get("section", "")),
                "nav_color_confirmation_gate_idx": color_confirmation.get("gate_idx", ""),
                "nav_color_confirmation_recovery_side_changed": int(bool(color_confirmation.get("recovery_side_changed", False))),
                "nav_color_confirmation_recovery_gate_locked": int(bool(color_confirmation.get("recovery_gate_locked", False))),
                "nav_color_confirmation_post_unpark_reverse_suppressed": int(bool(color_confirmation.get("post_unpark_reverse_suppressed", False))),
                "nav_color_confirmation_recovery_decision_reason": str(color_confirmation.get("recovery_decision_reason", "")),
                "nav_color_confirmation_recovery_side_from": str(color_confirmation.get("recovery_side_from", "")),
                "nav_color_confirmation_recovery_side_to": str(color_confirmation.get("recovery_side_to", "")),
                "nav_color_confirmation_recovery_start_lane_m": color_confirmation.get("recovery_start_lane_m", ""),
                "nav_color_confirmation_recovery_decided_lane_m": color_confirmation.get("recovery_decided_lane_m", ""),
                "nav_color_confirmation_recovery_lane_delta_m": color_confirmation.get("recovery_lane_delta_m", ""),
                "nav_color_confirmation_recovery_trigger_distance_m": color_confirmation.get("recovery_trigger_distance_m", ""),
                "nav_color_confirmation_recovery_target_m": color_confirmation.get("recovery_target_m", ""),
                "nav_color_confirmation_recovery_reason": str(color_confirmation.get("recovery_reason", "")),
                "close_color_recovery_state": str(close_color_recovery.get("state", "")),
                "close_color_recovery_slot": str(close_color_recovery.get("slot_id", "")),
                "close_color_recovery_trigger_distance_m": close_color_recovery.get("trigger_distance_m", ""),
                "close_color_recovery_target_m": close_color_recovery.get("target_m", ""),
                "close_color_recovery_requested_target_m": close_color_recovery.get("requested_target_m", ""),
                "close_color_recovery_safe_distance_m": close_color_recovery.get("safe_distance_m", ""),
                "close_color_recovery_applied_target_m": close_color_recovery.get("applied_target_m", ""),
                "close_color_recovery_safety_clamped": int(bool(close_color_recovery.get("safety_clamped", False))),
                "close_color_recovery_safety_suppressed": int(bool(close_color_recovery.get("safety_suppressed", False))),
                "close_color_recovery_safety_limiting_reason": str(close_color_recovery.get("safety_limiting_reason", "")),
                "close_color_recovery_safety_limiting_distance_m": close_color_recovery.get("safety_limiting_distance_m", ""),
                "close_color_recovery_safety_obstacle_count": close_color_recovery.get("safety_obstacle_count", ""),
                "close_color_recovery_progress_m": close_color_recovery.get("progress_m", ""),
                "close_color_recovery_remaining_m": close_color_recovery.get("remaining_m", ""),
                "close_color_recovery_slowdown_m": close_color_recovery.get("slowdown_m", ""),
                "close_color_recovery_stop_tolerance_m": close_color_recovery.get("stop_tolerance_m", ""),
                "close_color_recovery_reason": str(close_color_recovery.get("reason", "")),
                "close_color_recovery_handoff_reset_reason": str(close_color_recovery.get("handoff_reset_reason", "")),
                "close_color_recovery_handoff_reset_previous_id": str(close_color_recovery.get("handoff_reset_previous_waypoint_id", "")),
                "close_color_recovery_start_allowed": int(bool(close_color_recovery.get("start_allowed", False))),
                "close_color_recovery_start_suppressed_reason": str(close_color_recovery.get("start_suppressed_reason", "")),
                "close_color_recovery_nav_blocked_reason": str(close_color_recovery.get("nav_blocked_reason", "")),
                "close_color_recovery_enabled": int(bool(close_color_recovery.get("enabled", False))),
                "close_color_recovery_deadman_active": int(bool(close_color_recovery.get("deadman_active", False))),
                "close_color_recovery_deadman_fresh": int(bool(close_color_recovery.get("deadman_fresh", False))),
                "close_color_recovery_target_speed_mps": close_color_recovery.get("target_speed_mps", ""),
                "planner_blocked_recovery_state": str(planner_blocked_recovery.get("state", "")),
                "planner_blocked_recovery_case_id": planner_blocked_recovery.get("case_id", ""),
                "planner_blocked_recovery_case_key": str(planner_blocked_recovery.get("case_key", "")),
                "planner_blocked_recovery_blocked_age_s": planner_blocked_recovery.get("blocked_age_s", ""),
                "planner_blocked_recovery_eligible": int(bool(planner_blocked_recovery.get("eligible", False))),
                "planner_blocked_recovery_start_allowed": int(bool(planner_blocked_recovery.get("start_allowed", False))),
                "planner_blocked_recovery_start_suppressed_reason": str(planner_blocked_recovery.get("start_suppressed_reason", "")),
                "planner_blocked_recovery_nav_blocked_reason": str(planner_blocked_recovery.get("nav_blocked_reason", "")),
                "planner_blocked_recovery_safe_distance_m": planner_blocked_recovery.get("safe_distance_m", ""),
                "planner_blocked_recovery_straight_safe_distance_m": planner_blocked_recovery.get("straight_safe_distance_m", ""),
                "planner_blocked_recovery_target_m": planner_blocked_recovery.get("target_m", ""),
                "planner_blocked_recovery_progress_m": planner_blocked_recovery.get("progress_m", ""),
                "planner_blocked_recovery_remaining_m": planner_blocked_recovery.get("remaining_m", ""),
                "planner_blocked_recovery_heading_offset_deg": planner_blocked_recovery.get("heading_offset_deg", ""),
                "planner_blocked_recovery_path_h": planner_blocked_recovery.get("path_h", ""),
                "planner_blocked_recovery_limiting_reason": str(planner_blocked_recovery.get("limiting_reason", "")),
                "planner_blocked_recovery_limiting_distance_m": planner_blocked_recovery.get("limiting_distance_m", ""),
                "planner_blocked_recovery_selected_candidate": str(planner_blocked_recovery.get("selected_candidate", "")),
                "planner_blocked_recovery_obstacle_count": planner_blocked_recovery.get("obstacle_count", ""),
                "planner_blocked_recovery_candidate_count": planner_blocked_recovery.get("candidate_count", ""),
                "planner_blocked_recovery_target_speed_mps": planner_blocked_recovery.get("target_speed_mps", ""),
                "planner_blocked_recovery_deadman_active": int(bool(planner_blocked_recovery.get("deadman_active", False))),
                "planner_blocked_recovery_deadman_fresh": int(bool(planner_blocked_recovery.get("deadman_fresh", False))),
                "planner_blocked_recovery_parking_exclusion": int(bool(planner_blocked_recovery.get("parking_exclusion", False))),
                "planner_blocked_recovery_post_unpark_elapsed_s": planner_blocked_recovery.get("post_unpark_elapsed_s", ""),
                "planner_blocked_recovery_post_unpark_progress_m": planner_blocked_recovery.get("post_unpark_progress_m", ""),
                "planner_blocked_recovery_handoff_reset_reason": str(planner_blocked_recovery.get("handoff_reset_reason", "")),
                "planner_blocked_recovery_handoff_reset_previous_id": str(planner_blocked_recovery.get("handoff_reset_previous_waypoint_id", "")),
                "planner_blocked_recovery_exit_reason": str(planner_blocked_recovery.get("exit_reason", "")),
                "nav_smart_parking_alignment_active": int(bool((nav.get("smart_parking_alignment") or {}).get("active", False))),
                "nav_smart_parking_alignment_splice_active": int(bool((nav.get("smart_parking_alignment") or {}).get("splice_active", False))),
                "nav_smart_parking_route_geometry_active": int(bool((nav.get("smart_parking_alignment") or {}).get("route_geometry_active", False))),
                "nav_smart_parking_alignment_handoff_gate": str((nav.get("smart_parking_alignment") or {}).get("handoff_gate", "")),
                "nav_smart_parking_alignment_source_gate": str((nav.get("smart_parking_alignment") or {}).get("source_gate", "")),
                "nav_smart_parking_alignment_remaining_to_handoff_m": (nav.get("smart_parking_alignment") or {}).get("remaining_to_handoff_m", ""),
                "nav_smart_parking_alignment_remaining_to_stage_m": (nav.get("smart_parking_alignment") or {}).get("remaining_to_stage_m", ""),
                "startup_status": str(startup.get("status", "")),
                "startup_reason": str(startup.get("reason", "")),
                "startup_source": str(startup.get("source", "")),
                "startup_left_m": startup.get("left_m", ""),
                "startup_right_m": startup.get("right_m", ""),
                "startup_front_m": startup.get("front_m", ""),
                "startup_front_lateral_m": startup.get("front_lateral_m", ""),
                "startup_left_close": int(bool(startup.get("left_close", False))),
                "startup_right_close": int(bool(startup.get("right_close", False))),
                "startup_front_close": int(bool(startup.get("front_close", False))),
                "startup_parking_roi_votes": startup.get("parking_roi_votes", ""),
                "startup_parking_roi_required": startup.get("parking_roi_required", ""),
                "startup_race_mode": str(startup.get("race_mode", "")),
                "startup_direction": str(startup.get("direction", "")),
                "race_mode": str(race_timing.get("mode", "")),
                "race_mode_source": str(race_timing.get("mode_source", "")),
                "race_phase": str(race_timing.get("phase", "")),
                "race_warning": str(race_timing.get("warning", "")),
                "race_run_start_t": race_timing.get("run_start_t", ""),
                "race_initialization_complete_t": race_timing.get("initialization_complete_t", ""),
                "race_unpark_start_t": race_timing.get("unpark_start_t", ""),
                "race_navigation_start_t": race_timing.get("navigation_start_t", ""),
                "race_initialization_time_s": race_timing.get("initialization_time_s", ""),
                "race_unpark_time_s": race_timing.get("unpark_time_s", ""),
                "race_lap1_time_s": race_timing.get("lap1_time_s", ""),
                "race_lap2_time_s": race_timing.get("lap2_time_s", ""),
                "race_obstacle_lap3_to_stop_s": race_timing.get("obstacle_lap3_to_stop_s", ""),
                "race_free_lap3_to_stationary_s": race_timing.get("free_lap3_to_stationary_s", ""),
                "race_parking_stop_time_s": race_timing.get("parking_stop_time_s", ""),
                "race_parking_duration_s": race_timing.get("parking_duration_s", ""),
                "race_total_to_parked_s": race_timing.get("total_to_parked_s", ""),
                "race_total_to_free_stationary_s": race_timing.get("total_to_free_stationary_s", ""),
                "race_ranking_time_s": race_timing.get("ranking_time_s", ""),
                "race_full_gate_pass_count": race_timing.get("full_gate_pass_count", ""),
                "unpark_state": str(unpark.get("state", "")),
                "unpark_reason": str(unpark.get("reason", "")),
                "unpark_start_mode": str(unpark.get("start_mode", "")),
                "unpark_reverse_first_active": int(bool(unpark.get("reverse_first_active", False))),
                "unpark_reverse_first_target_m": unpark.get("reverse_first_target_m", ""),
                "unpark_reverse_first_progress_m": unpark.get("reverse_first_progress_m", ""),
                "unpark_reverse_first_stop_reason": unpark.get("reverse_first_stop_reason", ""),
                "unpark_anchor_phase": unpark.get("anchor_phase", ""),
                "unpark_anchor_cause": unpark.get("anchor_cause", ""),
                "unpark_anchor_arm_timed_out": int(bool(unpark.get("anchor_arm_timed_out", False))),
                "unpark_heading_progress_deg": unpark.get("heading_progress_deg", ""),
                "unpark_exit_heading_reached": int(bool(unpark.get("exit_heading_reached", False))),
                "unpark_rear_clearance_m": unpark.get("rear_clearance_m", ""),
                "unpark_reverse_room_m": unpark.get("reverse_room_m", ""),
                "unpark_reverse_stop_clearance_m": unpark.get("reverse_stop_clearance_m", ""),
                "unpark_reverse_progress_m": unpark.get("reverse_progress_m", ""),
                "unpark_reverse_target_m": unpark.get("reverse_target_m", ""),
                "unpark_reverse_commit_active": int(bool(unpark.get("reverse_commit_active", False))),
                "unpark_reverse_commit_remaining_s": unpark.get("reverse_commit_remaining_s", ""),
                "unpark_reverse_stop_reason": unpark.get("reverse_stop_reason", ""),
                "unpark_forward_direct_duty": unpark.get("forward_direct_duty", ""),
                "unpark_forward_direct_age_s": unpark.get("forward_direct_age_s", ""),
                "unpark_anchor_release_brake_level": unpark.get("anchor_release_brake_level", ""),
                "unpark_anchor_release_age_s": unpark.get("anchor_release_age_s", ""),
                "unpark_reverse_direct_duty": unpark.get("reverse_direct_duty", ""),
                "unpark_reverse_arm_speed_mps": unpark.get("reverse_arm_speed_mps", ""),
                "unpark_reverse_arm_stable_s": unpark.get("reverse_arm_stable_s", ""),
                "unpark_reverse_arm_ready": int(bool(unpark.get("reverse_arm_ready", False))),
                "unpark_reverse_arm_last_count": unpark.get("reverse_arm_last_count", ""),
                "unpark_retry_count": unpark.get("retry_count", ""),
                "unpark_passive_color_enabled": int(bool(passive_unpark_color.get("enabled", False))),
                "unpark_passive_color_active": int(bool(passive_unpark_color.get("active", False))),
                "unpark_passive_color_reason": str(passive_unpark_color.get("reason", "")),
                "unpark_passive_color_state": str(passive_unpark_color.get("state", "")),
                "unpark_passive_color_stationary": int(bool(passive_unpark_color.get("stationary", False))),
                "unpark_passive_color_speed_mps": passive_unpark_color.get("stationary_speed_mps", ""),
                "unpark_passive_color_yaw_rate_dps": passive_unpark_color.get("stationary_yaw_rate_dps", ""),
                "unpark_passive_color_candidate_count": passive_unpark_color.get("candidate_count", ""),
                "unpark_passive_color_committed_slot": str(passive_unpark_color.get("committed_slot", "")),
                "unpark_passive_color_committed_color": str(passive_unpark_color.get("committed_color", "")),
                "unpark_passive_color_committed_confidence": passive_unpark_color.get("committed_confidence", ""),
                "unpark_passive_color_last_slot": str(passive_unpark_color.get("last_slot", "")),
                "unpark_passive_color_last_measurement_id": str(passive_unpark_color.get("last_measurement_id", "")),
                "unpark_passive_color_last_sample_label": str(passive_unpark_color.get("last_sample_label", "")),
                "unpark_passive_color_last_skip_reason": str(passive_unpark_color.get("last_skip_reason", "")),
                "unpark_passive_color_sample_count": passive_unpark_color.get("sample_count", ""),
                "unpark_passive_color_red_count": passive_unpark_color.get("red_count", ""),
                "unpark_passive_color_green_count": passive_unpark_color.get("green_count", ""),
                "unpark_passive_color_other_count": passive_unpark_color.get("other_count", ""),
                "unpark_passive_color_measurement_events_available": passive_unpark_color.get("measurement_events_available", ""),
                "unpark_passive_color_measurement_events_added": passive_unpark_color.get("measurement_events_added", ""),
                "unpark_passive_color_measurement_events_duplicate": passive_unpark_color.get("measurement_events_duplicate", ""),
                "unpark_passive_color_measurement_events_not_real": passive_unpark_color.get("measurement_events_not_real", ""),
                "unpark_passive_color_measurement_events_stale": passive_unpark_color.get("measurement_events_stale", ""),
                "unpark_passive_color_measurement_events_before_window": passive_unpark_color.get("measurement_events_before_window", ""),
                "unpark_passive_color_measurement_events_outside_stationary": passive_unpark_color.get("measurement_events_outside_stationary", ""),
                "unpark_passive_color_measurement_events_missing_id": passive_unpark_color.get("measurement_events_missing_id", ""),
                "unpark_passive_color_stationary_window_start_t": passive_unpark_color.get("stationary_window_start_t", ""),
                "unpark_passive_color_stationary_epoch_count": passive_unpark_color.get("stationary_epoch_count", ""),
                "unpark_passive_color_stationary_epoch_active": int(bool(passive_unpark_color.get("stationary_epoch_active", False))),
                "unpark_passive_color_stationary_epoch_last_start_t": passive_unpark_color.get("stationary_epoch_last_start_t", ""),
                "unpark_passive_color_stationary_epoch_last_end_t": passive_unpark_color.get("stationary_epoch_last_end_t", ""),
                "unpark_passive_color_stationary_epoch_grace_active": int(bool(passive_unpark_color.get("stationary_epoch_grace_active", False))),
                "smart_parking_state": str(smart_parking.get("state", "")),
                "smart_parking_reason": str(smart_parking.get("reason", "")),
                "smart_parking_direction": str(smart_parking.get("direction", "")),
                "smart_parking_heading_error_deg": smart_parking.get("heading_error_deg", ""),
                "smart_parking_distance_to_target_m": smart_parking.get("distance_to_target_m", ""),
                "smart_parking_lap_stop_route_steering_active": int(bool(smart_parking.get("lap_stop_route_steering_active", False))),
                "smart_parking_lap_stop_measured_speed_mps": smart_parking.get("lap_stop_measured_speed_mps", ""),
                "smart_parking_lap_stop_yaw_rate_dps": smart_parking.get("lap_stop_yaw_rate_dps", ""),
                "smart_parking_lap_stop_route_steering_norm": smart_parking.get("lap_stop_route_steering_norm", ""),
                "smart_parking_lap_stop_steering_stationary_latched": int(bool(smart_parking.get("lap_stop_steering_stationary_latched", False))),
                "smart_parking_lap_stop_steering_rearm_count": smart_parking.get("lap_stop_steering_rearm_count", ""),
                "smart_parking_lap_stop_steering_rearm_required": smart_parking.get("lap_stop_steering_rearm_required", ""),
                "smart_parking_lap_stop_steering_rearm_speed_mps": smart_parking.get("lap_stop_steering_rearm_speed_mps", ""),
                "smart_parking_lap_stop_steering_rearm_yaw_rate_dps": smart_parking.get("lap_stop_steering_rearm_yaw_rate_dps", ""),
                "smart_parking_stage_x": smart_parking.get("stage_x", ""),
                "smart_parking_slot_center_x": smart_parking.get("slot_center_x", ""),
                "smart_parking_final_rear_y": smart_parking.get("final_rear_y", ""),
                "smart_parking_final_push_elapsed_s": smart_parking.get("final_push_elapsed_s", ""),
                "smart_parking_final_push_direct_duty": smart_parking.get("final_push_direct_duty", ""),
                "smart_parking_final_push_actuated": int(bool(smart_parking.get("final_push_actuated", False))),
                "db_mode": str(telemetry.get("mode", "")),
                "db_submode": str(telemetry.get("submode", "")),
                "db_target_user_mps": telemetry.get("target_user_mps", ""),
                "db_target_ramped_mps": telemetry.get("target_ramped_mps", ""),
                "db_measured_mps": telemetry.get("measured_mps", ""),
                "db_encoder_mps": telemetry.get("encoder_mps", telemetry.get("measured_mps", "")),
                "db_feedback_mps": telemetry.get("feedback_mps", ""),
                "db_feedback_source": telemetry.get("feedback_source", ""),
                "db_feedback_age_ms": telemetry.get("feedback_age_ms", ""),
                "db_external_feedback_mps": telemetry.get("external_feedback_mps", ""),
                "db_external_feedback_filtered_mps": telemetry.get("external_feedback_filtered_mps", ""),
                "db_external_feedback_age_ms": telemetry.get("external_feedback_age_ms", ""),
                "db_rate_mps": telemetry.get("rate_mps", ""),
                "db_position_m": telemetry.get("position_m", ""),
                "db_count": telemetry.get("count", ""),
                "db_last_duty": telemetry.get("last_duty", ""),
                "db_actuating": int(bool(telemetry.get("actuating", False))),
                "db_creep_breakaway_enabled": int(bool(telemetry.get("creep_breakaway_enabled", False))),
                "db_breakaway_allowed": int(bool(telemetry.get("breakaway_allowed", False))),
                "db_breakaway_enabled": int(bool(telemetry.get("breakaway_enabled", False))),
                "db_breakaway_active": int(bool(telemetry.get("breakaway_active", False))),
                "db_breakaway_max_speed_mps": telemetry.get("breakaway_max_speed_mps", ""),
                "db_creep_threshold_mps": telemetry.get("creep_threshold_mps", ""),
                "db_target_direction": telemetry.get("target_direction", ""),
                "db_direction_guard_clamped": int(bool(telemetry.get("direction_guard_clamped", False))),
                "db_raw_duty_before_direction_guard": telemetry.get("raw_duty_before_direction_guard", ""),
                "db_runtime_max_command_duty": telemetry.get("runtime_max_command_duty", ""),
                "db_speed_ff_target_duty_abs": telemetry.get("speed_ff_target_duty_abs", ""),
                "db_speed_ff_correction_clamp": telemetry.get("speed_ff_correction_clamp", ""),
                "db_speed_ff_duty": telemetry.get("speed_ff_duty", ""),
                "db_speed_pi_trim_duty": telemetry.get("speed_pi_trim_duty", ""),
                "db_speed_error_mps": telemetry.get("speed_error_mps", ""),
                "db_speed_ff_trim_kp": telemetry.get("speed_ff_trim_kp", ""),
                "db_speed_ff_trim_ki": telemetry.get("speed_ff_trim_ki", ""),
                "db_velocity_duty_slew_per_s": telemetry.get("velocity_duty_slew_per_s", ""),
                "db_internal_loop_ms": telemetry.get("internal_loop_ms", ""),
                "db_internal_loop_overrun_ms": telemetry.get("internal_loop_overrun_ms", ""),
                "db_internal_apply_output_ms": telemetry.get("internal_apply_output_ms", ""),
                "db_internal_loop_t": telemetry.get("internal_loop_t", ""),
                "db_steering_desired_norm": telemetry.get("steering_desired_norm", ""),
                "db_steering_applied_norm": telemetry.get("steering_applied_norm", ""),
                "db_steering_desired_seq": telemetry.get("steering_desired_seq", ""),
                "db_steering_applied_seq": telemetry.get("steering_applied_seq", ""),
                "db_steering_pending": int(bool(telemetry.get("steering_pending", False))),
                "db_steering_pending_age_ms": telemetry.get("steering_pending_age_ms", ""),
                "db_steering_apply_age_ms": telemetry.get("steering_apply_age_ms", ""),
                "db_steering_apply_duration_ms": telemetry.get("steering_apply_duration_ms", ""),
                "db_steering_apply_pwm_ms": telemetry.get("steering_apply_pwm_ms", ""),
                "db_steering_last_error": telemetry.get("steering_last_error", ""),
                "db_motion_applied_mode": telemetry.get("motion_applied_mode", ""),
                "db_motion_applied_duty": telemetry.get("motion_applied_duty", ""),
                "db_motion_applied_brake_level": telemetry.get("motion_applied_brake_level", ""),
                "db_motion_desired_seq": telemetry.get("motion_desired_seq", ""),
                "db_motion_applied_seq": telemetry.get("motion_applied_seq", ""),
                "db_motion_pending": int(bool(telemetry.get("motion_pending", False))),
                "db_motion_pending_age_ms": telemetry.get("motion_pending_age_ms", ""),
                "db_motion_apply_age_ms": telemetry.get("motion_apply_age_ms", ""),
                "db_motion_apply_duration_ms": telemetry.get("motion_apply_duration_ms", ""),
                "db_motion_apply_pwm_ms": telemetry.get("motion_apply_pwm_ms", ""),
                "db_motion_last_error": telemetry.get("motion_last_error", ""),
                "db_led_pending": int(bool(telemetry.get("led_pending", False))),
                "db_led_pending_age_ms": telemetry.get("led_pending_age_ms", ""),
                "db_led_apply_age_ms": telemetry.get("led_apply_age_ms", ""),
                "db_led_apply_duration_ms": telemetry.get("led_apply_duration_ms", ""),
                "db_led_apply_pwm_ms": telemetry.get("led_apply_pwm_ms", ""),
                "db_led_last_error": telemetry.get("led_last_error", ""),
                "drive_pwm_mode": telemetry.get("drive_pwm_mode", ""),
                "drive_pwm_reverse": telemetry.get("drive_pwm_reverse", ""),
                "drive_pwm_forward": telemetry.get("drive_pwm_forward", ""),
                "drive_pwm_req_reverse": telemetry.get("drive_pwm_req_reverse", ""),
                "drive_pwm_req_forward": telemetry.get("drive_pwm_req_forward", ""),
                "drive_pwm_read_reverse": telemetry.get("drive_pwm_read_reverse", ""),
                "drive_pwm_read_forward": telemetry.get("drive_pwm_read_forward", ""),
                "drive_pwm_readback_enabled": telemetry.get("drive_pwm_readback_enabled", ""),
                "drive_pwm_throttle": telemetry.get("drive_pwm_throttle", ""),
                "drive_pwm_brake_level": telemetry.get("drive_pwm_brake_level", ""),
                "drive_pwm_brake_order": telemetry.get("drive_pwm_brake_order", ""),
                "drive_pwm_command_t": pwm_command_t,
                "drive_pwm_read_t": pwm_read_t,
                "drive_pwm_command_age_ms": pwm_command_age_ms,
                "drive_pwm_read_age_ms": pwm_read_age_ms,
                "drive_pwm_age_ms": pwm_age_ms,
                "drive_pwm_exact_dedupe_enabled": int(bool(telemetry.get("drive_pwm_exact_dedupe_enabled", False))),
                "drive_pwm_cache_fds_enabled": int(bool(telemetry.get("drive_pwm_cache_fds_enabled", False))),
                "drive_pwm_cached_fd_count": telemetry.get("drive_pwm_cached_fd_count", ""),
                "drive_pwm_write_count_total": telemetry.get("drive_pwm_write_count_total", ""),
                "drive_pwm_skip_count_total": telemetry.get("drive_pwm_skip_count_total", ""),
                "drive_pwm_cache_fallback_count_total": telemetry.get("drive_pwm_cache_fallback_count_total", ""),
                "drive_pwm_cache_open_count_total": telemetry.get("drive_pwm_cache_open_count_total", ""),
                "drive_pwm_cache_close_count_total": telemetry.get("drive_pwm_cache_close_count_total", ""),
                "drive_pwm_write_count_servo": telemetry.get("drive_pwm_write_count_servo", ""),
                "drive_pwm_skip_count_servo": telemetry.get("drive_pwm_skip_count_servo", ""),
                "drive_pwm_cache_fallback_count_servo": telemetry.get("drive_pwm_cache_fallback_count_servo", ""),
                "drive_pwm_write_count_front_led": telemetry.get("drive_pwm_write_count_front_led", ""),
                "drive_pwm_skip_count_front_led": telemetry.get("drive_pwm_skip_count_front_led", ""),
                "drive_pwm_cache_fallback_count_front_led": telemetry.get("drive_pwm_cache_fallback_count_front_led", ""),
                "drive_pwm_write_count_drive_reverse": telemetry.get("drive_pwm_write_count_drive_reverse", ""),
                "drive_pwm_skip_count_drive_reverse": telemetry.get("drive_pwm_skip_count_drive_reverse", ""),
                "drive_pwm_cache_fallback_count_drive_reverse": telemetry.get("drive_pwm_cache_fallback_count_drive_reverse", ""),
                "drive_pwm_write_count_drive_forward": telemetry.get("drive_pwm_write_count_drive_forward", ""),
                "drive_pwm_skip_count_drive_forward": telemetry.get("drive_pwm_skip_count_drive_forward", ""),
                "drive_pwm_cache_fallback_count_drive_forward": telemetry.get("drive_pwm_cache_fallback_count_drive_forward", ""),
            }
            row["drive_t_row_build_ms"] = f"{(time.monotonic() - log_build_start) * 1000.0:.3f}"
            try:
                self.drive_log_queue.put_nowait(row)
            except queue.Full:
                self.drive_log_dropped_rows += 1
        except Exception:
            traceback.print_exc()

    def _record_drivebase_telemetry(self, loop_start: float, after_command_t: Optional[float] = None) -> dict:
        telemetry = {}
        try:
            get_start = time.monotonic()
            telemetry = self._timed_db_call("db_get_telemetry_call_ms", self.db.get_telemetry)
            get_end = time.monotonic()
            telemetry["drive_loop_start_t"] = loop_start
            telemetry["drive_loop_after_command_t"] = after_command_t
            telemetry["drive_telemetry_t"] = time.monotonic()
            telemetry["drive_t_get_telemetry_ms"] = f"{(get_end - get_start) * 1000.0:.3f}"
            shared_start = time.monotonic()
            self.perception.update_drivebase_telemetry(telemetry)
            telemetry["drive_t_shared_telemetry_ms"] = f"{(time.monotonic() - shared_start) * 1000.0:.3f}"
        except Exception:
            pass
        try:
            timing_start = time.monotonic()
            self.perception.shared.record_timing("drive_actuation", loop_start)
            telemetry["drive_t_record_timing_ms"] = f"{(time.monotonic() - timing_start) * 1000.0:.3f}"
        except Exception:
            pass
        return telemetry

    def stop(self) -> None:
        print("Drive shutdown: neutral requested")
        self.stop_event.set()
        self._stop_led_thread()
        self._neutral_power()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        print("Drive shutdown: drive thread stopped")
        if self.db is not None:
            try:
                self._center_steering()
                for _ in range(4):
                    self._neutral_power()
                    time.sleep(0.04)
                self._center_steering()
            except Exception:
                pass
            try:
                self.db.shutdown(disable_drive_pwm=bool(self.args.drivebase_disable_drive_pwm_on_shutdown))
                print("Drive shutdown: drivebase shutdown neutral")
            except Exception:
                traceback.print_exc()
            self.db = None

    def _run(self) -> None:
        try:
            from robot_core.drivebase import DriveBase, DriveConfig
            cfg = DriveConfig(
                debug=bool(self.args.drivebase_debug),
                kp=float(self.args.drivebase_kp),
                ki=float(self.args.drivebase_ki),
                u_vel_ff=float(self.args.drivebase_u_vel_ff),
                u_static=float(self.args.drivebase_u_static),
                speed_ff_profile_enable=bool(getattr(self.args, "drivebase_speed_ff_profile", True)),
                speed_ff_correction_clamp=float(getattr(self.args, "drivebase_speed_ff_correction_clamp", 0.08)),
                speed_ff_trim_kp=float(getattr(self.args, "drivebase_speed_ff_trim_kp", 0.18)),
                speed_ff_trim_ki=float(getattr(self.args, "drivebase_speed_ff_trim_ki", 0.35)),
                speed_ff_trim_integrator_limit=float(getattr(self.args, "drivebase_speed_ff_trim_integrator_limit", 0.05)),
                external_velocity_feedback_enable=bool(getattr(self.args, "drivebase_external_velocity_feedback", True)),
                external_velocity_feedback_min_speed_mps=float(getattr(self.args, "drivebase_external_velocity_min_speed_mps", 0.20)),
                external_velocity_feedback_max_age_s=float(getattr(self.args, "drivebase_external_velocity_timeout_s", 0.08)),
                external_velocity_feedback_filter_alpha=float(getattr(self.args, "drivebase_external_velocity_filter_alpha", 0.35)),
                v_slew_mps2=float(self.args.drivebase_v_slew_mps2),
                v_decel_slew_mps2=float(self.args.drivebase_v_decel_slew_mps2),
                decel_feedforward_disable_margin_mps=float(self.args.drivebase_decel_ff_disable_margin_mps),
                velocity_duty_slew_per_s=float(self.args.drivebase_velocity_duty_slew_per_s),
                steering_center_trim=float(self.args.drivebase_steering_center_trim),
                creep_breakaway_enable=bool(getattr(self.args, "drivebase_creep_breakaway_enable", False)),
                creep_threshold_mps=float(getattr(self.args, "drivebase_creep_threshold_mps", 0.05)),
                breakaway_enable=bool(getattr(self.args, "drivebase_breakaway_enable", True)),
                breakaway_max_speed_mps=float(getattr(self.args, "drivebase_breakaway_max_speed_mps", 0.05)),
                breakaway_duty_start=float(getattr(self.args, "drivebase_breakaway_duty_start", 0.25)),
                breakaway_duty_max=float(getattr(self.args, "drivebase_breakaway_duty_max", 0.90)),
                breakaway_duty_ramp_per_s=float(getattr(self.args, "drivebase_breakaway_duty_ramp_per_s", 1.20)),
                creep_duty_slew_per_s=float(self.args.drivebase_creep_duty_slew_per_s),
                pwm_readback_hz=float(getattr(self.args, "drivebase_pwm_readback_hz", 5.0)),
                pwm_exact_dedupe_enable=bool(getattr(self.args, "drivebase_pwm_exact_dedupe", True)),
                pwm_cache_fds=bool(getattr(self.args, "drivebase_pwm_cache_fds", True)),
                timing_detail_enable=bool(getattr(self.args, "drive_timing_debug", True)),
                timing_slow_threshold_ms=float(getattr(self.args, "drive_timing_slow_threshold_ms", 10.0)),
                timing_ring_size=int(getattr(self.args, "drive_timing_ring_size", 160)),
            )
            self.db = DriveBase(cfg)
            self.db.start()
            self._start_led_thread()
            self._open_drive_debug_log()
            print("Drive control started")
        except BaseException:
            traceback.print_exc()
            self.db = None
            return

        period = 1.0 / self.args.drive_control_rate
        next_t = time.monotonic()
        failed = False
        try:
            while not self.stop_event.is_set():
                loop_start = time.monotonic()
                self._drive_io_timing = {}
                drive_timing = {}
                segment_start = time.monotonic()
                cmd = self.perception.drive_command_for_actuation()
                drive_timing["cmd_fetch_ms"] = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                try:
                    segment_start = time.monotonic()
                    self.perception.update_drive_actuation_status(cmd)
                    drive_timing["status_update_ms"] = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                except Exception:
                    drive_timing["status_update_ms"] = "error"
                    pass
                if self.stop_event.is_set():
                    break
                motion_start = time.monotonic()
                duty_cap_ms = ""
                set_steering_ms = ""
                steer = max(-1.0, min(1.0, float(cmd.get("steering_norm", 0.0) or 0.0)))
                if cmd.get("actuation_allowed"):
                    self.last_drive_allowed_t = loop_start
                    duty_cap = cmd.get("drive_duty_cap")
                    try:
                        segment_start = time.monotonic()
                        self._set_runtime_duty_cap_if_needed(duty_cap)
                        duty_cap_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                    except Exception:
                        duty_cap_ms = "error"
                        pass
                    direct_power = cmd.get("direct_drive_power")
                    if direct_power is not None:
                        power = max(-1.0, min(1.0, float(direct_power)))
                        if power < 0.0 and not bool(cmd.get("allow_reverse", False)):
                            power = 0.0
                        segment_start = time.monotonic()
                        self._set_steering_if_needed(steer, loop_start)
                        set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                        if abs(power) > 1e-6 and self._release_brake_before_motion(cmd, loop_start, phase="brake_release_before_direct"):
                            pass
                        elif abs(power) <= 1e-6:
                            self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_to_zero_direct")
                        else:
                            self._annotate_brake_release(cmd, 0.0, self.applied_brake_level, "")
                            self._timed_db_call("db_set_drive_power_call_ms", self.db.set_drive_power, power)
                            self.last_output_kind = "direct" if abs(power) > 1e-6 else "neutral"
                    else:
                        raw_speed = float(cmd.get("target_speed_mps", 0.0) or 0.0)
                        max_speed = float(self.args.drive_command_max_speed_mps)
                        if bool(cmd.get("allow_reverse", False)):
                            speed = max(-max_speed, min(max_speed, raw_speed))
                        else:
                            speed = max(0.0, min(max_speed, raw_speed))
                        segment_start = time.monotonic()
                        self._set_steering_if_needed(steer, loop_start)
                        set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                        if abs(speed) > 1e-6 and self._release_brake_before_motion(cmd, loop_start, phase="brake_release_before_velocity"):
                            pass
                        elif abs(speed) <= 1e-6:
                            self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_to_zero_speed")
                        else:
                            self._annotate_brake_release(cmd, 0.0, self.applied_brake_level, "")
                            allow_breakaway = bool(cmd.get("allow_breakaway", not bool(cmd.get("precision_reverse", False))))
                            self._timed_db_call("db_set_target_speed_call_ms", self.db.set_target_speed, speed, allow_breakaway=allow_breakaway)
                            self.last_output_kind = "velocity"
                else:
                    if bool(cmd.get("terminal_idle", False)):
                        self.last_drive_allowed_t = 0.0
                        segment_start = time.monotonic()
                        self._set_steering_if_needed(0.0, loop_start)
                        set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                        self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_to_terminal_idle")
                    else:
                        reason = str(cmd.get("reason", ""))
                        if bool(cmd.get("neutral_stop", False)):
                            segment_start = time.monotonic()
                            self._set_steering_if_needed(0.0, loop_start)
                            set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                            self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_to_neutral_stop")
                        elif bool(cmd.get("brake", False)) and reason not in ("deadman", "stale_command", "not_driveable"):
                            segment_start = time.monotonic()
                            brake_steer = steer if bool(cmd.get("brake_preserve_steering", False)) else 0.0
                            self._set_steering_if_needed(brake_steer, loop_start)
                            set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                            self._apply_brake_request(cmd, float(cmd.get("brake_level", 1.0) or 1.0), loop_start, phase="brake_release")
                        elif reason == "deadman":
                            self.last_drive_allowed_t = 0.0
                            segment_start = time.monotonic()
                            self._set_steering_if_needed(0.0, loop_start)
                            set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                            self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_deadman_neutral")
                        else:
                            segment_start = time.monotonic()
                            self._set_steering_if_needed(0.0, loop_start)
                            set_steering_ms = f"{(time.monotonic() - segment_start) * 1000.0:.3f}"
                            self._apply_brake_request(cmd, 0.0, loop_start, phase="brake_release_to_neutral")

                after_command_t = time.monotonic()
                drive_timing["duty_cap_ms"] = duty_cap_ms
                drive_timing["set_steering_ms"] = set_steering_ms
                drive_timing["motion_command_ms"] = f"{(after_command_t - motion_start) * 1000.0:.3f}"
                if isinstance(self._drive_io_timing, dict):
                    drive_timing.update(self._drive_io_timing)
                drive_timing["total_instrumented_ms"] = f"{(after_command_t - loop_start) * 1000.0:.3f}"
                cmd["drive_timing"] = drive_timing
                telemetry = self._record_drivebase_telemetry(loop_start, after_command_t)
                if isinstance(self._drive_io_timing, dict):
                    drive_timing.update(self._drive_io_timing)
                if bool(getattr(self.args, "drive_timing_debug", True)) and self.db is not None:
                    try:
                        drain_events = getattr(self.db, "drain_timing_events", None)
                        if callable(drain_events):
                            drive_timing.update(self._summarize_drive_timing_events(drain_events()))
                    except Exception:
                        drive_timing["timing_event_summary"] = "error"
                self._write_drive_debug_log(loop_start, cmd, telemetry, after_command_t)

                next_t += period
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    self.last_drive_sleep_ms = f"{sleep_s * 1000.0:.3f}"
                    time.sleep(sleep_s)
                else:
                    self.last_drive_sleep_ms = "0.000"
                    next_t = time.monotonic()
        except BaseException:
            failed = True
            traceback.print_exc()
            self._stop_power()
        finally:
            if failed:
                self._stop_power()
            else:
                self._neutral_power()
            self._stop_led_thread()
            self._close_drive_debug_log()
            print("Drive control stopped")


class ControlPreviewThread:
    def __init__(self, args, perception: RobotPerception):
        self.args = args
        self.perception = perception
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _run(self) -> None:
        period = 1.0 / self.args.drive_control_rate
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.perception.update_control_preview()
            except Exception:
                traceback.print_exc()
                time.sleep(0.10)
                next_t = time.monotonic()
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()


def make_handler(perception: RobotPerception, args):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *values):
            if args.http_log:
                super().log_message(fmt, *values)

        def _send_json(self, data: dict, status: HTTPStatus = HTTPStatus.OK):
            payload = json.dumps(json_safe(data), separators=(",", ":"), allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _read_json_body(self) -> dict:
            length = int(self.headers.get("Content-Length", "0") or "0")
            body = self.rfile.read(min(length, 8192)) if length > 0 else b"{}"
            return json.loads(body.decode("utf-8")) if body else {}

        def do_POST(self):
            if self.path.startswith("/debug-state"):
                try:
                    status = self._read_json_body()
                    if "lite_web_debug_enabled" in status:
                        perception.set_runtime_lite_web_debug_enabled(bool(status.get("lite_web_debug_enabled", False)))
                    if "full_web_debug_enabled" in status:
                        perception.set_runtime_web_debug_enabled(bool(status.get("full_web_debug_enabled", False)))
                    camera_enabled = status.get("camera_web_debug_enabled") if "camera_web_debug_enabled" in status else None
                    camera_fps = status.get("camera_web_debug_fps") if "camera_web_debug_fps" in status else None
                    if camera_enabled is not None or camera_fps is not None:
                        perception.set_runtime_camera_debug(camera_enabled, camera_fps)
                    self._send_json(perception.runtime_debug_status())
                except Exception:
                    self.send_error(HTTPStatus.BAD_REQUEST, "invalid debug state payload")
                return
            if self.path.startswith("/deadman"):
                try:
                    status = self._read_json_body()
                    if str(getattr(args, "deadman_source", "physical")) in ("web", "both"):
                        status["source"] = "web"
                        perception.update_drive_deadman(status)
                        payload = b'{"ok":true}'
                    else:
                        payload = b'{"ok":true,"ignored":true,"reason":"web_deadman_disabled"}'
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store, max-age=0")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                except Exception:
                    self.send_error(HTTPStatus.BAD_REQUEST, "invalid deadman payload")
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                body = PORTAL_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/debug":
                body = HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/lite":
                body = LITE_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/debug-state":
                self._send_json(perception.runtime_debug_status())
                return
            if path == "/lite-events":
                if not perception.runtime_lite_web_debug_enabled():
                    self._send_json({"ok": False, "reason": "lite_web_debug_disabled"}, HTTPStatus.FORBIDDEN)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                period = 1.0 / args.stream_rate
                try:
                    while True:
                        if not perception.runtime_lite_web_debug_enabled():
                            return
                        json_start = time.monotonic()
                        payload = json.dumps(json_safe(perception.lite_payload()), separators=(",", ":"), allow_nan=False)
                        perception.shared.record_timing("lite_payload_json", json_start)
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    return
            if path == "/events":
                if not perception.runtime_full_web_debug_enabled():
                    self._send_json({"ok": False, "reason": "full_web_debug_disabled"}, HTTPStatus.FORBIDDEN)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                period = 1.0 / args.stream_rate
                try:
                    while True:
                        if not perception.runtime_full_web_debug_enabled():
                            return
                        payload = json.dumps(json_safe(perception.payload()), separators=(",", ":"), allow_nan=False)
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    return
            if path == "/camera.jpg":
                if not perception.runtime_debug_status().get("camera_web_debug_enabled", False):
                    self.send_error(HTTPStatus.NOT_FOUND, "camera debug disabled")
                    return
                jpeg = None
                mode = "exact" if "mode=exact" in self.path else "overview"
                analyzer = getattr(perception, "color_analyzer", None)
                if analyzer is not None:
                    get_jpeg = getattr(analyzer, "camera_debug_jpeg", None)
                    if callable(get_jpeg):
                        jpeg = get_jpeg(mode=mode)
                if jpeg is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "camera debug frame unavailable")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                try:
                    self.wfile.write(jpeg)
                except (BrokenPipeError, ConnectionResetError):
                    return
                return
            self.send_error(HTTPStatus.NOT_FOUND)

    return Handler


def main(argv=None) -> int:
    args = parse_args(argv)
    perception = RobotPerception(args)
    if perception.color_analyzer.enabled:
        print("Camera color analyzer started")
    elif not args.no_camera_color:
        print(f"Camera color analyzer disabled: {perception.color_analyzer.reason}", file=sys.stderr)
    drive_thread = DriveControlThread(args, perception)
    otos_thread = OTOSThread(args, perception, drive_thread)
    ros_thread = RosThread(args, perception)
    physical_deadman_thread = PhysicalDeadmanThread(args, perception)
    control_preview_thread = ControlPreviewThread(args, perception)
    server = None
    otos_thread.start()
    ros_thread.start()
    physical_deadman_thread.start()
    control_preview_thread.start()
    if args.web_portal or args.web_debug:
        server = ThreadingHTTPServer((args.host, args.port), make_handler(perception, args))
    drive_thread.start()
    if server is not None:
        if args.web_debug:
            print(f"Telemetry map: http://{args.host}:{args.port}/debug")
        else:
            print(f"Debug portal: http://{args.host}:{args.port}/")
    else:
        print("Web portal disabled. Remove --no-web-portal to start the lightweight debug portal.")
    if args.enable_drive_control:
        deadman_source = str(getattr(args, "deadman_source", "physical"))
        if deadman_source == "physical":
            print(
                "Drive control ENABLED. Physical deadman: "
                f"{args.deadman_gpiochip} line {int(args.deadman_gpio_line)} "
                f"active {'LOW' if bool(args.deadman_gpio_active_low) else 'HIGH'}."
            )
        elif deadman_source == "both":
            print(
                "Drive control ENABLED. Physical deadman or browser W in /debug can allow motion: "
                f"{args.deadman_gpiochip} line {int(args.deadman_gpio_line)} "
                f"active {'LOW' if bool(args.deadman_gpio_active_low) else 'HIGH'}."
            )
        elif server is not None:
            print("Drive control ENABLED. Hold W in /debug to allow browser deadman motion.")
        else:
            print("Drive control ENABLED, but web deadman source is selected and no web portal is running; motion remains blocked.")
    else:
        print("Drive control disabled. Add --enable-drive-control to command DriveBase.")
    print("Press Ctrl+C to stop.")
    try:
        if server is not None:
            server.serve_forever()
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        drive_thread.stop()
        if server is not None:
            server.shutdown()
        control_preview_thread.stop()
        physical_deadman_thread.stop()
        ros_thread.stop()
        otos_thread.stop()
        perception.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
