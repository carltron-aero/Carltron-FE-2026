"""Loopback-only camera control UI, with a serialized Windows COM bridge."""
from __future__ import annotations

import argparse
import datetime as dt
import json
import queue
import secrets
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from preview import NativePreview

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUTPUT = ROOT / 'output' / 'tracking' / 'camera-control'
PRESETS = OUTPUT / 'presets'
PRESET_FORMAT = 'carltron-camera-preset-v1'
VERSION = '2026.10.04.2'


class Controller:
    def __init__(self):
        self.lock = threading.Lock()
        self._start()

    def _start(self):
        self.lines = queue.Queue()
        self.process = subprocess.Popen(
            ['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
             '-File', str(HERE / 'controller.ps1')], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW)
        # Readers belong to one process generation; an old EOF cannot poison a new queue.
        threading.Thread(target=self._read, args=(self.process, self.lines), daemon=True).start()
        threading.Thread(target=self._errors, args=(self.process,), daemon=True).start()

    @staticmethod
    def _read(process, lines):
        try:
            for line in process.stdout:
                lines.put(line)
        finally:
            lines.put(None)

    @staticmethod
    def _errors(process):
        for line in process.stderr:
            print('Camera bridge:', line.rstrip(), flush=True)

    def call(self, request):
        with self.lock:
            if self.process.poll() is not None:
                raise ConnectionError('Camera driver connection has stopped.')
            try:
                self.process.stdin.write(json.dumps(request) + '\n')
                self.process.stdin.flush()
            except (OSError, ValueError) as error:
                raise ConnectionError('Camera driver connection was lost.') from error
            try:
                line = self.lines.get(timeout=8)
            except queue.Empty:
                self.process.kill()
                self.process.wait(timeout=3)
                raise ConnectionError('Camera driver timed out. Waiting to reconnect.')
            if line is None:
                raise ConnectionError('Camera driver connection has stopped.')
            reply = json.loads(line)
            if not reply.get('ok'):
                raise ValueError(reply.get('error', 'Camera request failed.'))
            return reply['result']

    def _close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.close()
            except OSError:
                pass
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)

    def restart(self):
        with self.lock:
            self._close()
            self._start()

    def close(self):
        with self.lock:
            self._close()


class App:
    def __init__(self, port):
        self.port = port
        self.origin = f'http://localhost:{port}'
        self.key = secrets.token_hex(24)
        self.controller = Controller()
        self.selected = None
        self.preferred_path = None
        self.state = None
        self.devices = []
        self.connection_error = None
        self.connection_revision = 0
        self.last_recovery = float('-inf')
        self.transaction = threading.RLock()
        self.preview = None
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.get_state()
        if self.state:
            self.save_snapshot('startup', self.state)

    def connection(self):
        return {'state': self.state, 'devices': self.devices, 'selected': self.selected,
                'preferred_path': self.preferred_path, 'connection_error': self.connection_error,
                'connection_revision': self.connection_revision, 'key': self.key, 'version': VERSION,
                'preview': self.preview.info() if getattr(self, 'preview', None) else None}

    def stop_preview(self, token=None):
        preview = getattr(self, 'preview', None)
        if preview and (token is None or token == preview.token):
            self.preview = None
            preview.close()

    def require_device(self, data):
        if not self.state or data.get('device_path') != self.state.get('Device', {}).get('Path'):
            raise ValueError('Camera selection changed or is unverified. Read settings and try again; no command was sent.')

    def disconnected(self, message):
        self.stop_preview()
        if self.state is not None:
            self.connection_revision += 1
        self.state = None
        self.selected = None
        self.connection_error = message

    @staticmethod
    def usable(state):
        return bool(state and any(c.get('Supported') for c in state.get('Controls', [])))

    def _read_connection(self, allow_single=False):
        self.devices = []
        self.devices = self.controller.call({'action': 'devices'})
        target = next((d for d in self.devices if d['Path'] == self.preferred_path), None)
        if (self.preferred_path is None and self.devices) or (allow_single and target is None and len(self.devices) == 1):
            target = self.devices[0]
            self.preferred_path = target['Path']
        if target is None:
            self.disconnected('Selected camera is disconnected. Waiting for it to return.'
                              if self.preferred_path else 'Connect a PTZ Pro 2 camera.')
            return
        if self.selected != target['Index'] or self.state is None:
            if getattr(self, 'preview', None) and self.preview.path != target['Path']:
                self.stop_preview()
            state = self.controller.call({'action': 'select', 'index': target['Index']})
            self.connection_revision += 1
        else:
            state = self.controller.call({'action': 'state'})
        if state.get('Device', {}).get('Path') != target['Path']:
            raise ConnectionError('Camera inventory changed while binding the driver. Rediscovering without sending commands.')
        # COM can keep a stale device object after unplugging, returning only failed controls.
        if not self.usable(state):
            raise ConnectionError('Camera is detected, but its controls are not responding.')
        self.selected = target['Index']
        self.state = state
        self.connection_error = None

    def get_state(self, force=False):
        with self.transaction:
            now = time.monotonic()
            if force:
                self.disconnected('Rediscovering camera controls…')
                self.last_recovery = now
                try:
                    self.controller.restart()
                except Exception as error:
                    self.disconnected(str(error))
                    return self.connection()
            try:
                self._read_connection(allow_single=force)
            except Exception as error:
                self.disconnected(str(error))
                if not force and now - self.last_recovery >= 3:
                    self.last_recovery = now
                    try:
                        self.controller.restart()
                        self._read_connection()
                    except Exception as retry_error:
                        self.disconnected(str(retry_error))
            return self.connection()

    def save_snapshot(self, label, state):
        stamp = dt.datetime.now().astimezone().strftime('%Y%m%dT%H%M%S-%f')
        safe = ''.join(ch for ch in label if ch.isalnum() or ch in '-_')[:40] or 'snapshot'
        path = OUTPUT / f'{stamp}-{safe}.json'
        path.write_text(json.dumps({'saved_at': dt.datetime.now().astimezone().isoformat(),
                                   'label': label, 'state': state}, indent=2), encoding='utf-8')
        return str(path)

    def log(self, request, result):
        row = {'at': dt.datetime.now().astimezone().isoformat(), 'device': self.selected,
               'request': request, 'result': result}
        with (OUTPUT / 'actions.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row) + '\n')

    @staticmethod
    def preset_path(identifier):
        if not isinstance(identifier, str) or len(identifier) != 16 or any(ch not in '0123456789abcdef' for ch in identifier):
            raise ValueError('Invalid preset identifier.')
        return PRESETS / f'{identifier}.json'

    def read_preset(self, identifier):
        path = self.preset_path(identifier)
        if not path.is_file():
            raise ValueError('Saved view was not found.')
        saved = json.loads(path.read_text(encoding='utf-8'))
        if (saved.get('format') != PRESET_FORMAT or saved.get('id') != identifier
                or not isinstance(saved.get('name'), str) or not isinstance(saved.get('state'), dict)):
            raise ValueError('Saved view is not a valid preset.')
        return saved

    def list_presets(self):
        items, errors = [], []
        if PRESETS.is_dir():
            for path in PRESETS.glob('*.json'):
                try:
                    items.append(self.read_preset(path.stem))
                except (ValueError, OSError) as error:
                    errors.append(f'{path.name}: {error}')
        def saved_time(item):
            try:
                return dt.datetime.fromisoformat(item['saved_at']).timestamp()
            except (KeyError, TypeError, ValueError):
                return 0
        return {'presets': sorted(items, key=saved_time, reverse=True),
                'errors': errors}

    def save_preset(self, name):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80 or any(ord(ch) < 32 for ch in name):
            raise ValueError('Give the view a name of 1–80 characters.')
        self.get_state()
        if self.state is None:
            raise ConnectionError(self.connection_error)
        PRESETS.mkdir(parents=True, exist_ok=True)
        identifier = secrets.token_hex(8)
        saved = {'format': PRESET_FORMAT, 'id': identifier, 'name': name.strip(),
                 'saved_at': dt.datetime.now().astimezone().isoformat(),
                 'state': self.state, 'position_note': 'Logical pan/tilt feedback, not measured physical position. Recheck field references after restoring.'}
        # Saving is read-only for the camera. Never overwrite an earlier view implicitly.
        path = self.preset_path(identifier)
        with path.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(saved, indent=2))
        return {'preset': saved, 'path': str(path)}

    @staticmethod
    def validate_value(value, minimum, maximum, step, label):
        if (type(value) is not int or not -2147483648 <= value <= 2147483647
                or any(type(item) is not int for item in [minimum, maximum, step])
                or step <= 0 or not minimum <= value <= maximum or (value - minimum) % step):
            raise ValueError(f'{label} does not match the connected camera range and step.')

    def preset_plan(self, saved):
        source = saved['state']
        live = self.state
        if source.get('Device', {}).get('Name') != live['Device']['Name']:
            raise ValueError('This view belongs to a different camera model.')
        pose, current_pose = source.get('Absolute', {}), live.get('Absolute', {})
        if (not pose.get('Readable') or not current_pose.get('Readable') or not current_pose.get('Writable')
                or pose.get('Units') != 'arcseconds' or current_pose.get('Units') != 'arcseconds'):
            raise ValueError('Restoring a full view requires readable saved position and verified live absolute controls.')
        for axis in ['Pan', 'Tilt']:
            self.validate_value(pose.get(axis), current_pose.get(f'{axis}Min'), current_pose.get(f'{axis}Max'),
                                current_pose.get(f'{axis}Step'), axis)
        commands, found = [], set()
        auto_exposure = any(c.get('Group') == 'camera' and c.get('Id') == 4 and c.get('Mode') == 'auto'
                            for c in source.get('Controls', []))
        # Relative motors and unknown vendor bytes must never be replayed from snapshots.
        allowed = {'camera': {3, 4, 6}, 'video': {0, 1, 3, 4, 7, 8, 9, 13}}
        for control in source.get('Controls', []):
            group, identifier = control.get('Group'), control.get('Id')
            if not control.get('Supported') or group not in allowed or identifier not in allowed[group]:
                continue
            # Observed on these cameras: auto exposure adjusts gain even though its
            # Windows row reports "manual". Restoring that transient read-back would
            # fight the automatic pipeline and can fail immediate verification.
            if auto_exposure and (group, identifier) == ('video', 9):
                continue
            pair = (group, identifier)
            if pair in found:
                raise ValueError('Duplicate control in saved view.')
            found.add(pair)
            current = next((c for c in live['Controls'] if (c.get('Group'), c.get('Id')) == pair), None)
            if not current or not current.get('Supported'):
                raise ValueError(f'{control.get("Name", pair)} is unavailable on this camera.')
            auto = control.get('Mode') == 'auto'
            scalar = pair == ('video', 13)
            if (not scalar and control.get('Mode') not in {'auto', 'manual'}) or not current.get('Auto' if auto else 'Manual'):
                raise ValueError('A saved control mode is unavailable or unrecognised.')
            value = control.get('Value')
            self.validate_value(value, current.get('Min'), current.get('Max'), current.get('Step'), control.get('Name', str(pair)))
            same_mode = scalar or current.get('Mode') == control.get('Mode')
            if not same_mode or (not auto and current.get('Value') != value):
                commands.append({'action': 'set', 'group': group, 'id': identifier, 'value': value, 'auto': auto})
        if not {('camera', 3), ('camera', 6)} <= found:
            raise ValueError('A full view must include saved zoom and focus.')
        priority = next((c for c in source.get('Extended', []) if c.get('Name') == 'AutoExposurePriority' and c.get('Readable')), None)
        if priority:
            current = next((c for c in live.get('Extended', []) if c.get('Name') == 'AutoExposurePriority'), {})
            if type(priority.get('Value')) is not int or priority['Value'] not in {0, 1} or not current.get('Writable'):
                raise ValueError('Saved low-light compensation cannot be restored.')
            if priority['Value'] != current.get('Value'):
                commands.append({'action': 'extended', 'id': 19, 'value': priority['Value']})
        if any(pose[axis] != current_pose.get(axis) for axis in ['Pan', 'Tilt']):
            commands.append({'action': 'absolute', 'pan': pose['Pan'], 'tilt': pose['Tilt'], 'confirmed': True})
        return commands

    def apply_preset(self, data):
        if data.get('confirmed') is not True:
            raise ValueError('Restoring a view requires explicit confirmation; it can move the camera.')
        saved = self.read_preset(data.get('id'))
        self.get_state()
        if self.state is None:
            raise ConnectionError(self.connection_error)
        if data.get('device_path') != self.state['Device']['Path']:
            raise ValueError('Camera selection changed. Read the settings and confirm again.')
        if (saved['state'].get('Device', {}).get('Path') != self.state['Device']['Path']
                and data.get('allow_different_device') is not True):
            raise ValueError('This view was saved with a different camera identifier. Confirm the intended camera first.')
        commands = self.preset_plan(saved)  # Validate the entire view before the first camera write.
        snapshot = self.save_snapshot('before-view-restore', self.state)
        completed = []
        for command in commands:
            try:
                result = self.controller.call(command)  # Never replay a failed or timed-out write.
                if command['action'] == 'set':
                    scalar = command['group'] == 'video' and command['id'] == 13
                    if (not result.get('Supported') or (not scalar and result.get('Mode') != ('auto' if command['auto'] else 'manual'))
                            or (not command['auto'] and result.get('Value') != command['value'])):
                        raise ValueError('Setting read-back does not match the saved view.')
                elif command['action'] == 'extended' and result.get('Value') != command['value']:
                    raise ValueError('Low-light setting read-back does not match.')
                elif command['action'] == 'absolute' and (not result.get('Readable') or result.get('Pan') != command['pan'] or result.get('Tilt') != command['tilt']):
                    raise ValueError('Logical position read-back does not match the requested view.')
                completed.append(command)
                self.log(command, result)
            except Exception as error:
                message = f'Restore stopped after {len(completed)} confirmed commands: {error}. No command was repeated or rolled back. Recheck the camera.'
                self.log(data, {'error': message, 'completed': completed, 'failed': command, 'snapshot': snapshot})
                self.disconnected(message)
                raise ConnectionError(message) from error
        self.get_state()
        result = {'state': self.state, 'applied_count': len(completed), 'snapshot': snapshot,
                  'movement_requested': any(c['action'] == 'absolute' for c in completed),
                  'note': 'Position read-back is logical only. Wait for settling and recheck field references.'}
        self.log(data, result)
        return result

    def action(self, data):
        with self.transaction:
            action = data.get('action')
            integer_keys = {'select': ['index'], 'set': ['id', 'value'], 'jog': ['pan', 'tilt', 'duration', 'speed'], 'extended': ['id', 'value'], 'absolute': ['pan', 'tilt']}.get(action, [])
            for name in integer_keys:
                if type(data.get(name)) is not int or not -2147483648 <= data[name] <= 2147483647:
                    raise ValueError(f'{name} must be a signed 32-bit integer.')
            if action == 'absolute' and data.get('confirmed') is not True:
                raise ValueError('Absolute movement requires explicit confirmation.')
            if action == 'set' and (type(data.get('auto')) is not bool or data.get('group') not in {'camera', 'video'}):
                raise ValueError('A valid control group and boolean auto mode are required.')
            if action == 'save-preset':
                self.get_state()
                self.require_device(data)
                result = self.save_preset(data.get('name'))
                self.log(data, {'id': result['preset']['id'], 'path': result['path']})
                return result
            if action == 'apply-preset':
                return self.apply_preset(data)
            if action == 'preview-stop':
                if not isinstance(data.get('token'), str):
                    raise ValueError('A preview token is required.')
                self.stop_preview(data['token'])
                return {'stopped': True}
            if action == 'preview-start':
                self.get_state()
                self.require_device(data)
                if getattr(self, 'preview', None) and not self.preview.closed and not self.preview.info()['error']:
                    return self.preview.info()
                self.stop_preview()
                preview = NativePreview(self.state['Device']['Path'])
                try:
                    preview.next_frame()
                except Exception:
                    preview.close()
                    raise
                self.preview = preview
                return preview.info()
            if action == 'rediscover':
                result = self.get_state(force=True)
                self.log(data, {'selected': self.selected, 'connection_error': self.connection_error})
                return result
            if action == 'snapshot':
                if not isinstance(data.get('label', 'snapshot'), str):
                    raise ValueError('Snapshot label must be text.')
                self.get_state()
                if self.state is None:
                    raise ConnectionError(self.connection_error)
                self.require_device(data)
                return {'path': self.save_snapshot(data.get('label', 'snapshot'), self.state), 'state': self.state}
            if action == 'shutdown':
                self.stop_preview()
                if self.selected is not None:
                    try:
                        self.controller.call({'action': 'stop'})
                    except Exception:
                        pass
                self.log(data, {'shutdown': True})
                return {'shutdown': True}
            if action not in {'select', 'set', 'jog', 'stop', 'extended', 'absolute'}:
                raise ValueError('Unknown action.')
            if action == 'select':
                self.get_state()
                target = next((d for d in self.devices if d['Index'] == data['index']), None)
                if target is None or data.get('path') != target['Path']:
                    raise ValueError('Camera list changed. Rediscover cameras and select again.')
                self.preferred_path = target['Path']
                self.disconnected('Selecting camera…')
                self.get_state(force=True)
                if self.state is None:
                    raise ConnectionError(self.connection_error)
                result = self.state
                self.save_snapshot('camera-selected', result)
            else:
                self.get_state()
                if self.state is None:
                    raise ConnectionError(self.connection_error)
                self.require_device(data)
                try:
                    # Never retry a write: a timed-out movement may already have executed.
                    result = self.controller.call(data)
                except ConnectionError as error:
                    self.disconnected(str(error))
                    raise
            self.log(data, result)
            return result


def handler_for(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data, mime='application/json; charset=utf-8'):
            body = json.dumps(data).encode('utf-8') if mime.startswith('application/json') else data
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; media-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def host_ok(self):
            return self.headers.get('Host') in {f'localhost:{app.port}', f'127.0.0.1:{app.port}'}

        def do_GET(self):
            if not self.host_ok():
                self.reply(403, {'error': 'Localhost only.'}); return
            try:
                if urlparse(self.path).path == '/api/preview':
                    self.stream_preview()
                elif self.path == '/api/session':
                    with app.transaction:
                        result = {'app': 'Carltron Camera Control', **app.connection(), 'output': str(OUTPUT)}
                    self.reply(200, result)
                elif self.path == '/api/state':
                    self.reply(200, app.get_state())
                elif self.path == '/api/presets':
                    with app.transaction:
                        self.reply(200, app.list_presets())
                elif self.path in {'/', '/index.html', '/app.js', '/style.css'}:
                    name = 'index.html' if self.path == '/' else self.path[1:]
                    mime = {'index.html': 'text/html; charset=utf-8', 'app.js': 'text/javascript; charset=utf-8', 'style.css': 'text/css; charset=utf-8'}[name]
                    self.reply(200, (HERE / name).read_bytes(), mime)
                else:
                    self.reply(404, {'error': 'Not found.'})
            except Exception as error:
                self.reply(500, {'error': str(error)})

        def stream_preview(self):
            token = parse_qs(urlparse(self.path).query).get('token', [''])[0]
            with app.transaction:
                preview = getattr(app, 'preview', None)
                if not preview or not secrets.compare_digest(token, preview.token) or preview.closed:
                    self.reply(404, {'error': 'Preview is not active.'})
                    return
                if not app.state or preview.path != app.state['Device']['Path']:
                    self.reply(409, {'error': 'Camera selection changed.'})
                    return
                with preview.condition:
                    preview.clients += 1
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.connection.settimeout(3)
            sequence = 0
            try:
                while not preview.closed:
                    sequence, frame = preview.next_frame(sequence)
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            except (OSError, ConnectionError):
                pass
            finally:
                preview.client_left()

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (not self.host_ok() or self.path != '/api/action'
                    or self.headers.get('X-Carltron-Key') != app.key
                    or origin not in {None, app.origin, f'http://127.0.0.1:{app.port}'}
                    or self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
                self.reply(403, {'error': 'Unauthorized local request.'}); return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size < 2 or size > 16384:
                    raise ValueError('Invalid request size.')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('Expected an object.')
                self.reply(200, {'result': app.action(data)})
                if data.get('action') == 'shutdown':
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
            except (ValueError, KeyError) as error:
                self.reply(400, {'error': str(error)})
            except Exception as error:
                self.reply(500, {'error': str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    app = App(args.port)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(app))
    try:
        print(f'Carltron Camera Control ready: {app.origin}', flush=True)
        print('Opening the app does not change settings or move the camera.', flush=True)
        server.serve_forever()
    finally:
        server.server_close()
        app.stop_preview()
        app.controller.close()


if __name__ == '__main__':
    main()
