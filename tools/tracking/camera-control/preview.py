"""Read-only native preview, bound to a DirectShow physical device moniker."""
from __future__ import annotations

from collections import deque
from pathlib import Path
import secrets
import subprocess
import threading
import time
import os
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[3]
FFMPEG = Path(os.environ.get('CARLTRON_FFMPEG') or imageio_ffmpeg.get_ffmpeg_exe())


def dshow_name(path):
    # FFmpeg's enumerated alternative name replaces the moniker's colons with underscores.
    # No friendly-name or browser/DirectShow ordinal matching is permitted.
    if not isinstance(path, str) or not path.startswith('@device:pnp:') or '\\usb#' not in path.lower():
        raise ValueError('A verified physical USB camera identifier is required.')
    return path.replace(':', '_')


class NativePreview:
    width, height, fps = 1920, 1080, 15

    def __init__(self, path, output_fps=15):
        if output_fps not in (15, 30):
            raise ValueError('Preview output rate must be 15 or 30 fps.')
        self.fps = output_fps
        self.path, self.token = path, secrets.token_hex(24)
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.error = None
        self.closed = False
        self.clients = 0
        self.last_frame_at = time.monotonic()
        self.errors = deque(maxlen=8)
        if not FFMPEG.is_file():
            raise FileNotFoundError('The local camera preview runtime is missing.')
        self.command = [str(FFMPEG), '-hide_banner', '-loglevel', 'error', '-nostdin',
                        '-f', 'dshow', '-rtbufsize', '64M', '-framerate', '30',
                        '-video_size', '1920x1080', '-pixel_format', 'yuyv422',
                        '-i', f'video={dshow_name(path)}', '-an', '-vf', f'fps={self.fps}',
                        '-threads', '2', '-c:v', 'mjpeg', '-q:v', '3', '-f', 'image2pipe', 'pipe:1']
        self.process = subprocess.Popen(self.command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0, creationflags=subprocess.CREATE_NO_WINDOW)
        threading.Thread(target=self._errors, daemon=True).start()
        threading.Thread(target=self._frames, daemon=True).start()

    def _errors(self):
        for line in self.process.stderr:
            self.errors.append(line.decode('utf-8', errors='replace').strip())

    def _frames(self):
        pending = bytearray()
        try:
            while not self.closed:
                part = self.process.stdout.read(65536)
                if not part:
                    break
                pending.extend(part)
                while True:
                    start = pending.find(b'\xff\xd8')
                    end = pending.find(b'\xff\xd9', start + 2) if start >= 0 else -1
                    if end < 0:
                        if start > 0:
                            del pending[:start]
                        break
                    frame = bytes(pending[start:end + 2])
                    del pending[:end + 2]
                    with self.condition:
                        self.frame = frame
                        self.sequence += 1
                        self.last_frame_at = time.monotonic()
                        self.condition.notify_all()
                if len(pending) > 8_000_000:
                    raise ValueError('Native preview returned an invalid oversized frame.')
        except Exception as error:
            self.error = str(error)
        finally:
            with self.condition:
                if not self.closed:
                    self.error = self.error or '\n'.join(self.errors) or 'Camera capture ended. Stop other camera captures and try Start preview again.'
                self.condition.notify_all()

    def next_frame(self, after=0, timeout=8):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence > after or self.error or self.closed, timeout)
            if self.error or self.closed:
                raise ConnectionError(self.error or 'Preview stopped.')
            if self.sequence <= after:
                raise ConnectionError('No camera frame arrived. Stop other camera captures and try Start preview again.')
            return self.sequence, self.frame

    def info(self):
        return {'device_path': self.path, 'token': self.token, 'width': self.width, 'height': self.height,
                'fps': self.fps, 'frames': self.sequence,
                'error': self.error or ('No fresh camera frames.' if time.monotonic() - self.last_frame_at > 5 else None)}

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        for handle in [self.process.stdout, self.process.stderr]:
            handle.close()

    def client_left(self):
        with self.condition:
            self.clients -= 1
        # A switched/disconnected tab must not leave its USB camera captured forever.
        def release_unused():
            with self.condition:
                unused = self.clients == 0
            if unused and not self.closed:
                self.close()
        timer = threading.Timer(1, release_unused)
        timer.daemon = True
        timer.start()
