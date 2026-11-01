"""Read-only, localhost replay of saved paired camera frames and robot logs.

No camera imports, device access, robot connections or writes to run evidence.
"""
from __future__ import annotations
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RUN = Path(__file__).resolve().parent.parent/'examples/first-run'
ASSETS = Path(__file__).resolve().parent


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def number(value):
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


class Replay:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        if (self.directory/'replay.json').is_file():
            self.data = load(self.directory/'replay.json')
            self.frame_indexes = {c['id']: {r['index']: r for r in c['frames']} for c in self.data['cameras']}
            self.json_bytes = json.dumps(self.data, separators=(',', ':'), allow_nan=False).encode()
            return
        timing = load(self.directory/'observed-led-timing.json')
        self.offsets = {v['physical_camera']: v['estimated_robot_t_minus_camera_elapsed_s'] for v in timing['views']}
        self.receipts = {c: load(self.directory/f'camera-{c}-receipts.json') for c in [1, 2]}
        self.frame_indexes = {c: {r['client_frame_index']: r for r in rows} for c, rows in self.receipts.items()}
        origin = min(rows[0]['host_receipt_ns'] for rows in self.receipts.values())
        observations = {c: load(self.directory/f'camera-{c}-moving-observations.json') for c in [1, 2]}
        frames = {}
        for c, rows in observations.items():
            frames[c] = []
            for row in rows:
                r = self.frame_indexes[c][row['client_frame_index']]
                elapsed = (r['host_receipt_ns']-origin)*1e-9
                if abs(elapsed-row['preview_receipt_elapsed_s']) > 1e-6:
                    raise ValueError('Detection and recording timestamps disagree.')
                pose = row.get('pose')
                frames[c].append({'index': row['client_frame_index'], 'robot_t': elapsed+self.offsets[c],
                                  'receipt_s': elapsed, 'quad': row.get('corners_px'),
                                  'xy': pose['rear_axle_xy_mm'] if pose else None,
                                  'marker': pose['marker_centre_xy_mm'] if pose else None,
                                  'heading': pose['body_heading_degrees'] if pose else None})
            if not frames[c]:
                raise ValueError('The run has no analyzed camera frames.')
        start = max(rows[0]['robot_t'] for rows in frames.values())
        end = min(rows[-1]['robot_t'] for rows in frames.values())
        if not start < end:
            raise ValueError('Camera recording ranges do not overlap.')
        robot = []
        for row in load(self.directory/'robot-logs/selected-drive-telemetry.json'):
            t = number(row['t'])
            if t is None or not start-.5 <= t <= end+.5:
                continue
            entry = {'time': t-start, 'robot_t': t, 'phase': row.get('race_phase', ''),
                     'speed': number(row.get('otos_speed_mps')), 'command': number(row.get('cmd_target_speed_mps'))}
            for label, prefix in [('fused', 'otos_display'), ('otos', 'otos_only')]:
                x, y = number(row.get(prefix+'_x_m')), number(row.get(prefix+'_y_m'))
                entry[label] = None if x is None or y is None else [x*1000+1500, y*1000+1500]
                entry[label+'_heading'] = number(row.get(prefix+'_h_deg'))
            robot.append(entry)
        if len(robot) < 2:
            raise ValueError('Too few robot log rows in the replay window.')
        for rows in frames.values():
            for row in rows:
                row['time'] = row['robot_t']-start
        manifest = load(self.directory/'capture-manifest.json')
        identities = {r['physical_camera']: r.get('identity', {}).get('parent_id', '').split('\\')[-1]
                      for r in manifest.get('records', [])}
        self.data = {'format': 'carltron-local-run-replay-v1', 'run_name': self.directory.name, 'duration': end-start, 'start_robot_t': start,
                     'robot': robot, 'cameras': [{'id': c, 'identity': identities.get(c, ''),
                         'offset': self.offsets[c], 'frames': frames[c]} for c in [1, 2]],
                     'reference': 'Rear axle midpoint; marker centre + 100.72 mm forward',
                     'field_frame': 'Lower-left field corner is (0,0); X right, Y up. Log map shifted by +1500 mm on each axis.',
                     'timing_note': 'LED-estimated clock alignment. Preview receipts are not exposure timestamps; individual LED matches scatter by tens of milliseconds.',
                     'mapping_note': 'Camera views independently registered from fixed tags. Lens, height and reference-placement uncertainties remain. No XY fit to robot logs.',
                     'interpolation_note': 'Robot log positions are linearly interpolated between rows. Camera positions are genuine detections at the displayed frame; absent detections stay absent.'}
        self.json_bytes = json.dumps(self.data, separators=(',', ':'), allow_nan=False).encode()

    def frame(self, camera, index):
        if camera not in self.frame_indexes or index not in self.frame_indexes[camera]:
            raise KeyError('Unknown camera or frame.')
        row = self.frame_indexes[camera][index]
        if 'example_kind' in self.data:
            body = (self.directory/'frames'/f'camera-{camera}-{index}.jpg').read_bytes()
            if not body.startswith(b'\xff\xd8') or not body.endswith(b'\xff\xd9'):
                raise ValueError('Example frame is not a complete JPEG.')
            return body
        path = self.directory/f'camera-{camera}.mjpeg'
        offset, length = row['byte_offset'], row['byte_length']
        if offset < 0 or length <= 0 or offset+length > path.stat().st_size:
            raise ValueError('Invalid recording frame bounds.')
        with path.open('rb') as source:
            source.seek(offset)
            body = source.read(length)
        if not body.startswith(b'\xff\xd8') or not body.endswith(b'\xff\xd9'):
            raise ValueError('Saved frame is not a complete JPEG.')
        return body


def handler_for(replay):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, body, mime, cache='no-store', code=200):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            host = self.headers.get('Host', '').split(':')[0].lower()
            if host not in ('127.0.0.1', 'localhost'):
                self.send_error(403)
                return
            url = urlsplit(self.path)
            try:
                if url.path == '/api/replay':
                    self.respond(replay.json_bytes, 'application/json; charset=utf-8')
                elif url.path == '/api/frame':
                    query = parse_qs(url.query)
                    camera, index = int(query['camera'][0]), int(query['index'][0])
                    self.respond(replay.frame(camera, index), 'image/jpeg', 'private, max-age=3600')
                elif url.path in ('/', '/index.html', '/math.js', '/app.js', '/style.css', '/field-artwork.json'):
                    asset = ASSETS/('index.html' if url.path == '/' else url.path[1:])
                    mime = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8'}[asset.suffix]
                    self.respond(asset.read_bytes(), mime)
                elif url.path == '/favicon.ico':
                    self.respond(b'', 'image/x-icon', code=204)
                else:
                    self.send_error(404)
            except (ValueError, KeyError, IndexError):
                self.send_error(400, 'Invalid replay request')
            except OSError:
                self.send_error(500, 'Could not read the saved recording')
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=DEFAULT_RUN)
    parser.add_argument('--port', type=int, default=8770)
    args = parser.parse_args()
    replay = Replay(args.run)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(replay))
    print(f'Read-only run replay ready: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
