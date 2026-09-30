#!/usr/bin/env python3
"""Standalone manual editor for navigation corner waypoint templates.

This app is intentionally separate from the telemetry map. It does not start
ROS, OTOS, LiDAR, camera, or motor code; it only edits the JSON consumed by the
navigation planner's corner override layer.
"""

from __future__ import annotations

import argparse
import json
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PROD_SRC = ROOT / "src" / "prod"
if str(PROD_SRC) not in sys.path:
    sys.path.insert(0, str(PROD_SRC))

from robot_core.navigation_global import DEFAULT_CORNER_WAYPOINT_CONFIG, complete_corner_waypoint_config, generate_corner_waypoint_config


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Corner Waypoint Editor</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #11161b; color: #e8eef4; }
    body { margin: 0; min-height: 100vh; display: grid; grid-template-columns: 380px 1fr; }
    aside { padding: 14px; background: #182029; border-right: 1px solid #2a3641; overflow: auto; }
    main { display: grid; place-items: center; padding: 16px; }
    h1 { font-size: 18px; margin: 0 0 14px; font-weight: 650; }
    label { display: grid; gap: 5px; font-size: 12px; color: #9eb0bf; margin: 10px 0; }
    select, input, button { border: 1px solid #344656; border-radius: 6px; background: #101820; color: #e8eef4; padding: 7px 8px; font: inherit; }
    button { cursor: pointer; }
    button.primary { background: #1f6feb; border-color: #388bfd; }
    button.secondary { background: #142232; border-color: #37536d; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .point { border: 1px solid #2d3a45; border-radius: 8px; padding: 8px; margin: 8px 0; background: #121a22; }
    .point h2 { font-size: 13px; margin: 0 0 6px; color: #d5e2ed; }
    .status { min-height: 20px; margin-top: 10px; font-size: 12px; color: #9eb0bf; }
    canvas { width: min(900px, calc(100vw - 420px)); height: min(900px, calc(100vh - 32px)); max-width: 900px; max-height: 900px; background: #0d1217; border: 1px solid #2b3844; border-radius: 8px; }
    .hint { color: #9eb0bf; font-size: 12px; line-height: 1.4; }
  </style>
</head>
<body>
  <aside>
    <h1>Corner Waypoint Editor</h1>
    <p class="hint">Templates are local to a corner. Drag points in the canvas or edit values directly. Headings are relative to the incoming straight.</p>
    <label>Mode
      <select id="mode"><option value="normal">normal corners</option><option value="free_race">free race corners</option></select>
    </label>
    <label>Direction
      <select id="direction"><option value="counter">counter</option><option value="clock">clock</option></select>
    </label>
    <label>Transition
      <select id="transition"></select>
    </label>
    <div class="row">
      <button id="mirrorCurrent" class="secondary">Mirror Current</button>
      <button id="mirrorAll" class="secondary">Mirror All</button>
    </div>
    <div id="points"></div>
    <button id="save" class="primary">Save JSON</button>
    <button id="reload">Reload</button>
    <div id="status" class="status"></div>
  </aside>
  <main><canvas id="canvas" width="900" height="900"></canvas></main>
<script>
const lanes = ["outer_tight", "outer_pass", "center", "inner_pass", "inner_tight"];
const freeRaceLanes = ["outer_pass", "inner_tight"];
const laneValue = {outer_tight:0.20, outer_pass:0.40, center:0.50, inner_pass:0.60, inner_tight:0.80};
const colors = {entry:"#68d391", blend:"#f6c177", exit:"#7aa2f7"};
let config = null;
let dragging = -1;

const modeEl = document.getElementById("mode");
const directionEl = document.getElementById("direction");
const transitionEl = document.getElementById("transition");
const pointsEl = document.getElementById("points");
const statusEl = document.getElementById("status");
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

function transitionKey(inLane, outLane) { return `${inLane}_to_${outLane}`; }
function selectedKey() { return transitionEl.value; }
function selectedRoot() { return modeEl.value === "free_race" ? config.free_race : config; }
function editableLanes() { return modeEl.value === "free_race" ? freeRaceLanes : lanes; }
function selectedTable(direction = directionEl.value) {
  const root = selectedRoot();
  if (!root[direction]) root[direction] = {};
  return root[direction];
}
function selectedItem() { return selectedTable()[selectedKey()]; }
function selectedLanes() {
  const key = selectedKey();
  for (const a of editableLanes()) for (const b of editableLanes()) if (key === transitionKey(a,b)) return [a,b];
  return modeEl.value === "free_race" ? ["outer_pass", "outer_pass"] : ["center", "center"];
}
function canonicalBasis() {
  if (directionEl.value === "counter") return {cx:0.5, cy:-0.5, ih:0, oh:90, inx:1, iny:0, outx:0, outy:1};
  return {cx:-0.5, cy:-0.5, ih:180, oh:90, inx:-1, iny:0, outx:0, outy:1};
}
function localToWorld(p) {
  const b = canonicalBasis();
  return {x:b.cx + p.u_m*b.inx + p.v_m*b.outx, y:b.cy + p.u_m*b.iny + p.v_m*b.outy, h:b.ih + p.heading_deg};
}
function viewportCenter() {
  return localToWorld({u_m:0.5, v_m:-0.5, heading_deg:0});
}
function worldToLocal(x, y) {
  const b = canonicalBasis();
  const dx = x - b.cx, dy = y - b.cy;
  return {u_m: dx*b.inx + dy*b.iny, v_m: dx*b.outx + dy*b.outy};
}
function toCanvas(x, y) {
  const center = viewportCenter();
  const scale = 780;
  return [canvas.width/2 + (x - center.x)*scale, canvas.height/2 - (y - center.y)*scale];
}
function fromCanvas(px, py) {
  const center = viewportCenter();
  const scale = 780;
  return {x:center.x + (px - canvas.width/2)/scale, y:center.y + (canvas.height/2 - py)/scale};
}
function lineWorld(a, b, color, width=2, dash=[]) {
  const p0 = toCanvas(a.x, a.y), p1 = toCanvas(b.x, b.y);
  ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(p0[0], p0[1]); ctx.lineTo(p1[0], p1[1]); ctx.stroke(); ctx.restore();
}
function hermitePoint(p0, p1, m0, m1, t) {
  const t2 = t*t, t3 = t2*t;
  const h00 = 2*t3 - 3*t2 + 1;
  const h10 = t3 - 2*t2 + t;
  const h01 = -2*t3 + 3*t2;
  const h11 = t3 - t2;
  return {x:h00*p0.x + h10*m0.x + h01*p1.x + h11*m1.x, y:h00*p0.y + h10*m0.y + h01*p1.y + h11*m1.y};
}
function drawHermite(points, color, width=3, dash=[]) {
  if (points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  for (let idx=0; idx<points.length-1; idx++) {
    const a = points[idx], b = points[idx+1];
    const chord = Math.max(Math.hypot(b.x-a.x, b.y-a.y), 0.05);
    const tangent = chord * 0.72;
    const ah = a.h * Math.PI / 180;
    const bh = b.h * Math.PI / 180;
    const m0 = {x:Math.cos(ah)*tangent, y:Math.sin(ah)*tangent};
    const m1 = {x:Math.cos(bh)*tangent, y:Math.sin(bh)*tangent};
    for (let i=0; i<28; i++) {
      const t = i / 27;
      const p = hermitePoint(a, b, m0, m1, t);
      const c = toCanvas(p.x, p.y);
      if (idx === 0 && i === 0) ctx.moveTo(c[0], c[1]); else ctx.lineTo(c[0], c[1]);
    }
  }
  ctx.stroke();
  ctx.restore();
}
function drawTrack() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  const b = canonicalBasis();
  const incomingOuter = localToWorld({u_m:-0.35, v_m:-1.0, heading_deg:0});
  const incomingInner = localToWorld({u_m:1.12, v_m:0.0, heading_deg:0});
  const outgoingOuter = localToWorld({u_m:1.0, v_m:-0.35, heading_deg:0});
  const outgoingInner = localToWorld({u_m:0.0, v_m:1.12, heading_deg:0});
  lineWorld(incomingOuter, localToWorld({u_m:1.12, v_m:-1.0, heading_deg:0}), "#4a5967",4);
  lineWorld(outgoingOuter, localToWorld({u_m:1.0, v_m:1.12, heading_deg:0}), "#4a5967",4);
  lineWorld(localToWorld({u_m:0.0, v_m:-0.35, heading_deg:0}), outgoingInner, "#4a5967",4);
  lineWorld(localToWorld({u_m:-0.35, v_m:0.0, heading_deg:0}), incomingInner, "#4a5967",4);
  lineWorld(localToWorld({u_m:0.0, v_m:0.0, heading_deg:0}), localToWorld({u_m:1.0, v_m:-1.0, heading_deg:0}), "#26323d",1,[5,7]);
  ctx.fillStyle = "#8293a3";
  ctx.font = "13px system-ui";
  const [cx, cy] = toCanvas(b.cx, b.cy);
  ctx.fillText("corner center", cx + 10, cy - 10);
}
function drawPoint(p, idx) {
  const w = localToWorld(p);
  const [cx, cy] = toCanvas(w.x, w.y);
  ctx.fillStyle = colors[p.label] || "#e8eef4";
  ctx.strokeStyle = "#071018";
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, 9, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  const hr = (w.h * Math.PI) / 180;
  lineWorld(w, {x:w.x + Math.cos(hr)*0.13, y:w.y + Math.sin(hr)*0.13}, colors[p.label] || "#e8eef4", 3);
  ctx.fillStyle = "#e8eef4"; ctx.font = "12px system-ui"; ctx.fillText(p.label, cx + 12, cy - 10);
}
function drawGates() {
  const [inLane, outLane] = selectedLanes();
  const inV = laneValue[inLane] - 1.0;
  const outU = 1.0 - laneValue[outLane];
  const inP = localToWorld({u_m:0, v_m:inV, heading_deg:0});
  const outP = localToWorld({u_m:outU, v_m:0, heading_deg: directionEl.value === "counter" ? 90 : -90});
  lineWorld(inP, outP, "#334150", 1, [8, 8]);
  for (const lane of lanes) {
    lineWorld(localToWorld({u_m:-0.18, v_m:laneValue[lane]-1.0, heading_deg:0}), localToWorld({u_m:0.22, v_m:laneValue[lane]-1.0, heading_deg:0}), "#26323d", 1);
    lineWorld(localToWorld({u_m:1.0-laneValue[lane], v_m:-0.18, heading_deg:0}), localToWorld({u_m:1.0-laneValue[lane], v_m:0.22, heading_deg:0}), "#26323d", 1);
  }
  drawPoint({label:"in", u_m:0, v_m:inV, heading_deg:0}, -1);
  drawPoint({label:"out", u_m:outU, v_m:0, heading_deg: directionEl.value === "counter" ? 90 : -90}, -1);
  return [inP, outP];
}
function draw() {
  if (!config) return;
  drawTrack();
  const gates = drawGates();
  const pts = selectedItem().points;
  const route = [gates[0], ...pts.map(localToWorld), gates[1]];
  drawHermite(route, "#8b5cf6", 3);
  for (let i=0;i<pts.length-1;i++) lineWorld(localToWorld(pts[i]), localToWorld(pts[i+1]), "#b48ead", 2, [7, 7]);
  pts.forEach(drawPoint);
}
function populateTransitions() {
  const previous = transitionEl.value;
  transitionEl.innerHTML = "";
  for (const a of editableLanes()) for (const b of editableLanes()) {
    const opt = document.createElement("option");
    opt.value = transitionKey(a,b); opt.textContent = `${a} -> ${b}`;
    transitionEl.appendChild(opt);
  }
  if ([...transitionEl.options].some(opt => opt.value === previous)) transitionEl.value = previous;
}
function populateInputs() {
  const pts = selectedItem().points;
  pointsEl.innerHTML = "";
  pts.forEach((p, idx) => {
    const div = document.createElement("div");
    div.className = "point";
    div.innerHTML = `<h2>${p.label}</h2>
      <div class="row">
        <label>u_m <input data-i="${idx}" data-k="u_m" type="number" step="0.005" value="${p.u_m}"></label>
        <label>v_m <input data-i="${idx}" data-k="v_m" type="number" step="0.005" value="${p.v_m}"></label>
      </div>
      <label>heading_deg <input data-i="${idx}" data-k="heading_deg" type="number" step="1" value="${p.heading_deg}"></label>
      <input data-i="${idx}" data-k="heading_deg" type="range" min="-100" max="100" step="1" value="${p.heading_deg}">`;
    pointsEl.appendChild(div);
  });
  pointsEl.querySelectorAll("input").forEach(input => input.addEventListener("input", ev => {
    const i = Number(ev.target.dataset.i), k = ev.target.dataset.k;
    selectedItem().points[i][k] = Number(ev.target.value);
    if (ev.target.type === "range") {
      const number = pointsEl.querySelector(`input[type="number"][data-i="${i}"][data-k="${k}"]`);
      if (number) number.value = ev.target.value;
    } else if (k === "heading_deg") {
      const range = pointsEl.querySelector(`input[type="range"][data-i="${i}"][data-k="${k}"]`);
      if (range) range.value = ev.target.value;
    }
    draw();
  }));
}
function oppositeDirection(direction) { return direction === "counter" ? "clock" : "counter"; }
function mirrorPoint(point) {
  return {
    label: point.label,
    u_m: Number(point.u_m),
    v_m: Number(point.v_m),
    heading_deg: Math.round(-Number(point.heading_deg) * 100) / 100,
  };
}
function mirrorCurrent() {
  const srcDirection = directionEl.value;
  const dstDirection = oppositeDirection(srcDirection);
  const key = selectedKey();
  selectedTable(dstDirection)[key] = {points: selectedItem().points.map(mirrorPoint)};
  statusEl.textContent = `mirrored ${srcDirection} ${key} to ${dstDirection}`;
}
function mirrorAll() {
  const srcDirection = directionEl.value;
  const dstDirection = oppositeDirection(srcDirection);
  for (const key of Object.keys(selectedTable(srcDirection))) {
    selectedTable(dstDirection)[key] = {points: selectedTable(srcDirection)[key].points.map(mirrorPoint)};
  }
  statusEl.textContent = `mirrored all ${srcDirection} transitions to ${dstDirection}`;
}
async function load() {
  const res = await fetch("/config");
  config = await res.json();
  if (!config.free_race) config.free_race = {counter:{}, clock:{}};
  if (!config.free_race.counter) config.free_race.counter = {};
  if (!config.free_race.clock) config.free_race.clock = {};
  populateTransitions();
  populateInputs();
  draw();
  statusEl.textContent = `loaded ${config.path || ""}`;
}
async function save() {
  const res = await fetch("/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(config, null, 2)});
  const data = await res.json();
  statusEl.textContent = data.ok ? `saved ${data.path}` : `save failed: ${data.error}`;
}
canvas.addEventListener("mousedown", ev => {
  const rect = canvas.getBoundingClientRect();
  const mx = (ev.clientX - rect.left) * canvas.width / rect.width;
  const my = (ev.clientY - rect.top) * canvas.height / rect.height;
  let best = -1, bestD = 18;
  selectedItem().points.forEach((p, idx) => {
    const w = localToWorld(p), c = toCanvas(w.x, w.y);
    const d = Math.hypot(mx-c[0], my-c[1]);
    if (d < bestD) { bestD = d; best = idx; }
  });
  dragging = best;
});
canvas.addEventListener("mousemove", ev => {
  if (dragging < 0) return;
  const rect = canvas.getBoundingClientRect();
  const mx = (ev.clientX - rect.left) * canvas.width / rect.width;
  const my = (ev.clientY - rect.top) * canvas.height / rect.height;
  const w = fromCanvas(mx, my);
  const local = worldToLocal(w.x, w.y);
  const p = selectedItem().points[dragging];
  p.u_m = Math.round(local.u_m * 10000) / 10000;
  p.v_m = Math.round(local.v_m * 10000) / 10000;
  populateInputs();
  draw();
});
window.addEventListener("mouseup", () => dragging = -1);
modeEl.addEventListener("change", () => { populateTransitions(); populateInputs(); draw(); });
directionEl.addEventListener("change", () => { populateInputs(); draw(); });
transitionEl.addEventListener("change", () => { populateInputs(); draw(); });
document.getElementById("save").addEventListener("click", save);
document.getElementById("reload").addEventListener("click", load);
document.getElementById("mirrorCurrent").addEventListener("click", mirrorCurrent);
document.getElementById("mirrorAll").addEventListener("click", mirrorAll);
load();
</script>
</body>
</html>
"""


def ensure_config(path: Path) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    data = generate_corner_waypoint_config()
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_handler(config_path: Path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/?"):
                body = HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/config":
                ensure_config(config_path)
                payload = complete_corner_waypoint_config(json.loads(config_path.read_text(encoding="utf-8")))
                payload["path"] = str(config_path)
                self._send_json(payload)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            if self.path != "/config":
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                payload.pop("path", None)
                payload = complete_corner_waypoint_config(payload)
                for key in ("counter", "clock"):
                    if key not in payload or not isinstance(payload[key], dict):
                        raise ValueError(f"missing {key} transition table")
                free_race = payload.get("free_race", {})
                if not isinstance(free_race, dict):
                    raise ValueError("missing free_race transition table")
                for key in ("counter", "clock"):
                    if key not in free_race or not isinstance(free_race[key], dict):
                        raise ValueError(f"missing free_race.{key} transition table")
                config_path.parent.mkdir(parents=True, exist_ok=True)
                config_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._send_json({"ok": True, "path": str(config_path)})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone corner waypoint config editor")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--config", default=str(DEFAULT_CORNER_WAYPOINT_CONFIG))
    args = parser.parse_args()
    config_path = Path(args.config).expanduser()
    ensure_config(config_path)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(config_path))
    print(f"Corner waypoint editor: http://{args.host}:{args.port}/")
    print(f"Config: {config_path}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
