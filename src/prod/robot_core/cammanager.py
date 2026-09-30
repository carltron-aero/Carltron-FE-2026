# cam_manager.py
# CamManager: background capture thread with 1.5 s buffer (Y, U, V only!),
# ROI ring + angular patch, manual exposure/gain + FPS lock, preview toggle,
# analyze_patch_at_time() that converts only the patch pixels to HSV, optional
# headless debug previews saved to disk with a per-manager cap, and clean shutdown.
#
# PERFORMANCE: get_patch_mask() uses a cached ring angle lookup computed only on
# ring pixels. This avoids recomputing angles on the full HxW grid and reduces
# per call time from about 60 to 70 ms down to under 1 ms on a Pi.
"""
Overview
--------
This module provides a single class CamManager that opens a libcamera viewfinder
stream, keeps a rolling buffer of the last 1.5 seconds of frames, and exposes a
patch analysis method for a circular ROI. The analysis converts only the pixels
inside a narrow angular patch on the ring ROI from YUV to HSV for lightweight
color classification. A background thread handles capture and buffering. Live
desktop preview can be enabled, and on demand headless debug frames can be
saved to disk from analyze_patch_at_time.

Key points
- Input pixel format is YUV420 (I420 layout). The capture thread stores only
  square cropped planes Y, U, V to avoid converting the full frame to BGR.
- The ROI is a circle centered near the image center with a configurable
  radius and thickness. The patch is an angular sector on that ring.
- Angle convention: 0 degrees at the bottom of the frame, increasing clockwise.
  Left is 90, top is 180, right is 270.
- Performance: only ring pixels are used to compute angles. A cache stores
  ring indices and their angles relative to the ROI center. Subsequent calls to
  build a patch mask filter this 1D angle array and scatter into an HxW mask.
- Headless debug: analyze_patch_at_time can save at most N preview images per
  CamManager. This is off by default and controlled per call via the preview
  flag. The save location and cap are constructor parameters.

API sketch
- CamManager(preview_on=False, patch_angle_deg=210.0, patch_width_deg=2.0,
             debug_save_dir="/tmp/cammanager_debug", debug_max_previews=100)
    Starts capture in a second thread and begins buffering frames.

- analyze_patch_at_time(query_ts_ns, angle_deg, width_deg, preview=False)
    Looks up the buffered frame with timestamp closest to query_ts_ns and
    classifies the selected patch. Returns
    (label, used_ts_ns, signed_age_ms, h_mean, s_mean, v_mean).
    If preview is True and the manager has not yet reached its save cap, a PNG
    with full grey ROI ring, patch overlay and simple text is written to disk.

- close()
    Stops capture, releases resources, and closes mmaps.

"""

import os, select, mmap, time, collections, math, threading
import numpy as np
import cv2
import libcamera as lc

# ---------- tuned knobs ----------
SIZE = (1100, 1100)            # requested (w,h); libcamera will pick nearest VF
PIXEL_FMT = "YUV420"

TARGET_FPS     = 30
FRAME_USEC     = int(1_000_000 / TARGET_FPS)
EXPOSURE_USEC  = max(FRAME_USEC - 1, 33000)   # must be <= FRAME_USEC
ANALOG_GAIN    = 14.0                         # ~ISO ≈ gain*100

# ROI (square-space)
ROI_RADIUS      = 502
ROI_THICKNESS   = 4
ROI_OFFSET_X    = 12
ROI_OFFSET_Y    = -6                          # 1 px up is negative
ROI_COLOR       = (255, 0, 255)               # BGR magenta, kept in case you want it again
ROI_GREY_COLOR  = (128, 128, 128)             # BGR grey, full ROI ring

# Patch
PATCH_COLOR       = (255, 255, 0)             # BGR
SHOW_PATCH_CENTER = True

# Buffer (~1.5 s). Using a small safety margin above theoretical frame count.
BUFFER_SEC = 1.5
BUFFER_MAX = int(max(60, TARGET_FPS * BUFFER_SEC * 1.2))
PRINT_PERF = os.environ.get("CAMMANAGER_PRINT_PERF", "").lower() in ("1", "true", "yes", "on")

# ---------- helpers ----------
def page_size():
    """System page size used for proper mmap alignment of libcamera planes."""
    return os.sysconf("SC_PAGESIZE")

def mmap_plane(fd: int, length: int, offset: int) -> memoryview:
    """Map a single libcamera plane into user space as a read-only memoryview.

    The return value is a slice that compensates for page alignment, so callers
    can treat it as a tight plane buffer starting at the given offset.
    """
    ps = page_size()
    page_off = offset % ps
    map_off = offset - page_off
    map_len = length + page_off
    fd_dup = os.dup(fd)
    mm = mmap.mmap(fd_dup, map_len, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ, offset=map_off)
    os.close(fd_dup)
    return memoryview(mm)[page_off:page_off + length]

def now_boottime_ns() -> int:
    """Nanoseconds from CLOCK_BOOTTIME if available, else monotonic."""
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()

# -------- ROI and patch utilities (cached coords and masks) --------
class _MaskCache:
    """Cache for ring geometry and per ring pixel angles.

    The cache removes per call HxW angle computations. For a given geometry
    defined by (h, w, radius, thickness, dx, dy) it stores:
    - ring_bool: HxW boolean mask of the ring
    - ring_ys, ring_xs: indices of ring pixels
    - theta_ring: per ring pixel angle in degrees with 0 at bottom and positive
      clockwise
    """

    def __init__(self):
        self.last_ring_key  = None   # (h,w,radius,thickness,dx,dy)
        self.ring_bool      = None   # (H,W) bool
        self.ring_ys        = None   # (N,)
        self.ring_xs        = None   # (N,)
        self.theta_ring     = None   # (N,) float32, 0 bottom, +CW

    def _build_ring_cache(self, h: int, w: int, radius: int, thickness: int, dx: int, dy: int):
        """Build ring mask and per ring pixel angles, then store in the cache."""
        yy, xx = np.mgrid[0:h, 0:w]
        cx, cy = (w // 2 + dx), (h // 2 + dy)
        r = np.sqrt((xx - cx)**2 + (yy - cy)**2)
        ring_bool = (np.abs(r - radius) <= (thickness / 2.0))

        ys, xs = np.nonzero(ring_bool)
        # Angle only for ring pixels. atan2 returns angle from +x axis CCW.
        dxp = xs - cx
        dyp = ys - cy
        ang = np.degrees(np.arctan2(dyp, dxp))
        theta = (ang - 90.0) % 360.0  # map to 0 at bottom, positive clockwise

        # Cache fields
        self.ring_bool  = ring_bool
        self.ring_ys    = ys.astype(np.int32)
        self.ring_xs    = xs.astype(np.int32)
        self.theta_ring = theta.astype(np.float32)

    def get_ring_mask(self, h: int, w: int, radius: int, thickness: int, dx: int, dy: int):
        """Return the HxW boolean ring mask, rebuilding cache when geometry changes."""
        key = (h, w, radius, thickness, dx, dy)
        if key != self.last_ring_key:
            self._build_ring_cache(h, w, radius, thickness, dx, dy)
            self.last_ring_key = key
        return self.ring_bool

    def get_patch_mask(self, h: int, w: int, ring_mask: np.ndarray,
                       dx: int, dy: int,
                       center_deg: float, width_deg: float) -> np.ndarray:
        """Return an HxW boolean mask for a narrow angular sector on the ring.

        This reuses cached ring indices and precomputed angles, then scatters the
        1D selection back into an HxW mask. Sector inclusion handles wraparound.
        """
        # Ensure cache corresponds to the provided geometry
        _ = self.get_ring_mask(h, w, ROI_RADIUS, ROI_THICKNESS, dx, dy)

        half = width_deg / 2.0
        a0 = (center_deg - half) % 360.0
        a1 = (center_deg + half) % 360.0
        if a0 <= a1:
            on_sector = (self.theta_ring >= a0) & (self.theta_ring <= a1)
        else:
            on_sector = (self.theta_ring >= a0) | (self.theta_ring <= a1)

        mask = np.zeros((h, w), dtype=bool)
        if np.any(on_sector):
            mask[self.ring_ys[on_sector], self.ring_xs[on_sector]] = True
        return mask

# -------- YUV helpers (no full frame conversion) --------
def _y_plane_view(y_plane: memoryview, W: int, H: int):
    """Return a 2D view (H, y_stride) over the Y plane. Caller crops columns to W."""
    y_len = len(y_plane)
    y_stride = y_len // H
    y_src = np.frombuffer(y_plane, dtype=np.uint8).reshape(H, y_stride)
    return y_src, y_stride

def _uv_plane_view(uv_plane: memoryview, W: int, H: int):
    """Return a 2D view (H/2, u_stride) over a chroma plane. Caller crops to W/2."""
    uv_len = len(uv_plane)
    uv_h = H // 2
    u_stride = uv_len // uv_h
    u_src = np.frombuffer(uv_plane, dtype=np.uint8).reshape(uv_h, u_stride)
    return u_src, u_stride

def _square_crop_from_yuv_planes(W_full, H_full, x0, side,
                                 y_plane, u_plane, v_plane):
    """Build square cropped Y, U, V from viewfinder planes only by slicing.

    Returns
    - Ysq: shape (side, side)
    - Usq: shape (side/2, side/2)
    - Vsq: shape (side/2, side/2)
    """
    # Luma
    y_src, y_stride = _y_plane_view(y_plane, W_full, H_full)
    Ysq = y_src[:, x0:x0+side].copy()

    # Chroma (subsampled by 2)
    u_src, u_stride = _uv_plane_view(u_plane, W_full, H_full)
    v_src, v_stride = _uv_plane_view(v_plane, W_full, H_full)
    u_x0 = x0 // 2
    side_uv = side // 2
    Usq = u_src[:, u_x0:u_x0+side_uv].copy()
    Vsq = v_src[:, u_x0:u_x0+side_uv].copy()

    return Ysq, Usq, Vsq

def _patch_yuv_vectors(Ysq, Usq, Vsq, patch_mask):
    """Extract Y, U, V vectors for all True positions in the patch mask.

    U and V are sampled using 2x2 subsampling at indices (y//2, x//2).
    Returns three 1D arrays or (None, None, None) if the mask is empty.
    """
    ys, xs = np.nonzero(patch_mask)
    if ys.size == 0:
        return None, None, None
    Yv = Ysq[ys, xs]
    Uv = Usq[ys // 2, xs // 2]
    Vv = Vsq[ys // 2, xs // 2]
    return Yv, Uv, Vv

def _yuv_to_bgr_vec(Yv, Uv, Vv):
    """Convert YUV vectors to B, G, R vectors using BT.601 integer math."""
    C = Yv.astype(np.int32) - 16
    D = Uv.astype(np.int32) - 128
    E = Vv.astype(np.int32) - 128

    R = (298*C + 409*E + 128) >> 8
    G = (298*C - 100*D - 208*E + 128) >> 8
    B = (298*C + 516*D + 128) >> 8

    R = np.clip(R, 0, 255).astype(np.uint8)
    G = np.clip(G, 0, 255).astype(np.uint8)
    B = np.clip(B, 0, 255).astype(np.uint8)
    return B, G, R

def _empty_color_stats() -> dict:
    return {
        "h_med": float("nan"),
        "s_med": float("nan"),
        "v_med": float("nan"),
        "y_med": float("nan"),
        "red_ratio": 0.0,
        "green_ratio": 0.0,
        "valid_ratio": 0.0,
        "dark_invalid_ratio": 0.0,
        "bright_invalid_ratio": 0.0,
        "dominance_margin": 0.0,
        "rg_opp_med": float("nan"),
        "r_norm_med": float("nan"),
        "g_norm_med": float("nan"),
        "b_norm_med": float("nan"),
        "excess_red_med": float("nan"),
        "excess_green_med": float("nan"),
        "lab_a_med": float("nan"),
        "red_score": 0,
        "green_score": 0,
        "green_rescue_active": 0,
        "red_suppressed": 0,
        "weak_red_rejected": 0,
        "raw_red_candidate": 0,
        "strong_red_active": 0,
        "red_strength_rejected": 0,
        "n_valid": 0,
        "n_total": 0,
    }


def _classify_bgr_vectors(Y, B, G, R):
    """Classify patch color from per-pixel HSV/Lab votes.

    Red wraps around the OpenCV hue boundary, so arithmetic mean hue is not
    used for the decision. The returned HSV values are medians for compatibility
    with existing debug display.
    """
    if B is None or B.size == 0:
        stats = _empty_color_stats()
        return ("none", 0.0, stats)
    pix = np.stack([B, G, R], axis=1).reshape(-1, 1, 3)
    hsv = cv2.cvtColor(pix, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    lab = cv2.cvtColor(pix, cv2.COLOR_BGR2LAB).reshape(-1, 3)
    h = hsv[:, 0].astype(np.float32)
    s = hsv[:, 1].astype(np.float32)
    v = hsv[:, 2].astype(np.float32)
    y = Y.astype(np.float32)
    b = B.astype(np.float32)
    g = G.astype(np.float32)
    r = R.astype(np.float32)

    total_pixels = int(B.size)
    valid = (
        (y > 35.0) &
        (y < 250.0) &
        (s > 45.0) &
        (v > 35.0) &
        (v < 250.0)
    )
    n_valid = int(np.count_nonzero(valid))
    y_dark = y <= 35.0
    y_bright = y >= 250.0
    v_dark = v <= 35.0
    v_bright = v >= 250.0
    dark_invalid_ratio = float(np.count_nonzero(y_dark | v_dark) / max(total_pixels, 1))
    bright_invalid_ratio = float(np.count_nonzero(y_bright | v_bright) / max(total_pixels, 1))
    min_valid = max(4, min(25, int(math.ceil(max(total_pixels, 1) * 0.25))))
    if n_valid < min_valid:
        stats = _empty_color_stats()
        stats.update({
            "h_med": float(np.median(h)) if h.size else float("nan"),
            "s_med": float(np.median(s)) if s.size else float("nan"),
            "v_med": float(np.median(v)) if v.size else float("nan"),
            "y_med": float(np.median(y)) if y.size else float("nan"),
            "valid_ratio": n_valid / max(total_pixels, 1),
            "dark_invalid_ratio": dark_invalid_ratio,
            "bright_invalid_ratio": bright_invalid_ratio,
            "n_valid": n_valid,
            "n_total": total_pixels,
        })
        return ("OTHER", 0.0, stats)

    hv = h[valid]
    sv = s[valid]
    vv = v[valid]
    rv = r[valid]
    gv = g[valid]
    bv = b[valid]
    lab_valid = lab[valid]
    lab_a = lab_valid[:, 1].astype(np.float32)

    red_pixels = int(np.count_nonzero((hv <= 12.0) | (hv >= 168.0)))
    green_pixels = int(np.count_nonzero((hv >= 35.0) & (hv <= 95.0)))
    red_ratio = red_pixels / max(n_valid, 1)
    green_ratio = green_pixels / max(n_valid, 1)

    total_rgb = rv + gv + bv + 1e-6
    r_norm = rv / total_rgb
    g_norm = gv / total_rgb
    b_norm = bv / total_rgb
    excess_red = (2.0 * rv - gv - bv) / total_rgb
    excess_green = (2.0 * gv - rv - bv) / total_rgb
    rg_opp = (rv - gv) / total_rgb
    rg_opp_med = float(np.median(rg_opp))
    r_norm_med = float(np.median(r_norm))
    g_norm_med = float(np.median(g_norm))
    b_norm_med = float(np.median(b_norm))
    excess_red_med = float(np.median(excess_red))
    excess_green_med = float(np.median(excess_green))
    lab_a_med = float(np.median(lab_a))

    red_score = 0
    green_score = 0
    if red_ratio > 0.35:
        red_score += 2
    if green_ratio > 0.35:
        green_score += 2
    if rg_opp_med > 0.08:
        red_score += 1
    elif rg_opp_med < -0.06:
        green_score += 1
    if excess_red_med > 0.14 and r_norm_med > 0.38:
        red_score += 1
    elif excess_green_med > 0.14 and g_norm_med > 0.38:
        green_score += 1
    if lab_a_med > 140.0:
        red_score += 1
    elif lab_a_med <= 118.0:
        green_score += 1
    elif lab_a_med <= 122.0 and green_ratio >= 0.35 and (rg_opp_med <= -0.045 or excess_green_med >= 0.12):
        green_score += 1

    h_med = float(np.median(hv))
    s_med = float(np.median(sv))
    v_med = float(np.median(vv))
    y_med = float(np.median(y[valid]))
    winning_ratio = max(red_ratio, green_ratio)
    dominance_margin = abs(red_ratio - green_ratio)
    valid_ratio = n_valid / max(total_pixels, 1)
    confidence = max(0.0, min(1.0, 0.45 * dominance_margin + 0.35 * winning_ratio + 0.20 * valid_ratio))
    green_rescue_active = int(
        valid_ratio >= 0.25
        and green_ratio >= 0.45
        and dominance_margin >= 0.25
        and rg_opp_med <= -0.055
        and excess_green_med >= 0.10
        and g_norm_med >= 0.37
    )
    red_suppressed = int(
        red_score >= 3
        and green_ratio >= 0.20
        and (green_ratio >= red_ratio * 0.45 or excess_green_med >= 0.05 or lab_a_med <= 126.0)
    )
    weak_red_rejected = int(
        red_score >= 3
        and s_med < 80.0
        and not (
            r_norm_med >= 0.50
            and excess_red_med >= 0.45
            and lab_a_med >= 155.0
        )
    )
    raw_red_candidate = int(red_score >= 3 and red_score > green_score and not red_suppressed and not weak_red_rejected)
    strong_red_active = int(
        raw_red_candidate
        and red_ratio >= 0.55
        and lab_a_med >= 145.0
        and r_norm_med >= 0.43
        and excess_red_med >= 0.28
        and v_med >= 55.0
    )
    red_strength_rejected = int(raw_red_candidate and not strong_red_active)

    stats = {
        "h_med": h_med,
        "s_med": s_med,
        "v_med": v_med,
        "y_med": y_med,
        "red_ratio": float(red_ratio),
        "green_ratio": float(green_ratio),
        "valid_ratio": float(valid_ratio),
        "dark_invalid_ratio": dark_invalid_ratio,
        "bright_invalid_ratio": bright_invalid_ratio,
        "dominance_margin": float(dominance_margin),
        "rg_opp_med": float(rg_opp_med),
        "r_norm_med": float(r_norm_med),
        "g_norm_med": float(g_norm_med),
        "b_norm_med": float(b_norm_med),
        "excess_red_med": float(excess_red_med),
        "excess_green_med": float(excess_green_med),
        "lab_a_med": float(lab_a_med),
        "red_score": int(red_score),
        "green_score": int(green_score),
        "green_rescue_active": int(green_rescue_active),
        "red_suppressed": int(red_suppressed),
        "weak_red_rejected": int(weak_red_rejected),
        "raw_red_candidate": int(raw_red_candidate),
        "strong_red_active": int(strong_red_active),
        "red_strength_rejected": int(red_strength_rejected),
        "n_valid": int(n_valid),
        "n_total": int(total_pixels),
    }

    if strong_red_active:
        return ("RED", float(confidence), stats)
    if green_score >= 3 and green_score > red_score:
        return ("GREEN", float(confidence), stats)
    if green_rescue_active and green_score >= red_score:
        return ("GREEN", float(confidence), stats)
    return ("OTHER", float(confidence), stats)

def _square_bgr_from_yuv(Ysq, Usq, Vsq):
    """Reconstruct a square BGR image from Y, U, V planes for preview overlays."""
    H = Ysq.shape[0]
    W = H
    uv_h = H // 2
    i420 = np.concatenate([Ysq.reshape(-1), Usq.reshape(-1), Vsq.reshape(-1)])
    i420_mat = i420.reshape((H + uv_h, W))
    bgr = cv2.cvtColor(i420_mat, cv2.COLOR_YUV2BGR_I420)
    return bgr

# ==========================================================
#                      CamManager
# ==========================================================
class CamManager:
    """Camera manager with background capture, ROI patch analysis and debug saves."""

    def __init__(self,
                 preview_on: bool = False,
                 patch_angle_deg: float = 210.0,
                 patch_width_deg: float = 2.0,
                 analog_gain: float = ANALOG_GAIN,
                 debug_save_dir: str = "/tmp/cammanager_debug",
                 debug_max_previews: int = 100):
        """Create and start a camera manager.

        preview_on controls a desktop live preview window from the capture
        thread. For headless systems this is usually False.

        patch_angle_deg and patch_width_deg set the default live preview patch.

        debug_save_dir is where analyze_patch_at_time(preview=True) writes PNGs.
        debug_max_previews limits how many debug frames are saved per manager.
        """
        self.preview_on = preview_on
        self.patch_angle_deg = patch_angle_deg
        self.patch_width_deg = patch_width_deg

        # Debug preview saving configuration
        self.debug_save_dir = debug_save_dir
        self.debug_max_previews = int(debug_max_previews)
        self._debug_saved = 0
        try:
            os.makedirs(self.debug_save_dir, exist_ok=True)
        except Exception:
            # If creation fails the save feature is disabled silently
            self.debug_save_dir = None
            self.debug_max_previews = 0

        # Frame buffer stores tuples (sensor_ts_ns, Ysq, Usq, Vsq)
        self.frame_buffer = collections.deque(maxlen=BUFFER_MAX)
        self.ts_hist = collections.deque(maxlen=90)  # for simple FPS display
        self._buf_lock = threading.Lock()

        self._stop = threading.Event()
        self._mask_cache = _MaskCache()
        self._control_lock = threading.Lock()
        self.base_analog_gain = float(analog_gain)
        self.active_analog_gain = float(analog_gain)

        # --- libcamera setup matching the confirmed working control set ---
        self.cm = lc.CameraManager.singleton()
        self.cam = self.cm.cameras[0]
        self.cam.acquire()

        self.cfg = self.cam.generate_configuration([lc.StreamRole.Viewfinder])
        sc = self.cfg.at(0)
        sc.pixel_format = lc.PixelFormat(PIXEL_FMT)
        sc.size = lc.Size(*SIZE)
        self.cam.configure(self.cfg)

        actual = self.cfg.at(0).size
        self.W_full, self.H_full = actual.width, actual.height

        self.alloc = lc.FrameBufferAllocator(self.cam)
        self.stream = self.cfg.at(0).stream
        if self.alloc.allocate(self.stream) < 0:
            raise RuntimeError("Buffer allocation failed")

        # Map planes for all frame buffers once
        self.fb_maps = {}
        for fb in self.alloc.buffers(self.stream):
            self.fb_maps[fb] = [mmap_plane(p.fd, p.length, p.offset) for p in fb.planes]

        self.reqs = []
        for fb in self.alloc.buffers(self.stream):
            r = self.cam.create_request()
            r.add_buffer(self.stream, fb)
            r.set_control(lc.controls.FrameDurationLimits, (FRAME_USEC, FRAME_USEC))
            r.set_control(lc.controls.AeEnable, False)
            r.set_control(lc.controls.AnalogueGainMode, int(1))
            r.set_control(lc.controls.ExposureTime, int(EXPOSURE_USEC))
            r.set_control(lc.controls.AnalogueGain, float(self.active_analog_gain))
            self.reqs.append(r)

        self.cam.start()
        for r in self.reqs:
            self.cam.queue_request(r)

        self.poller = select.poll()
        self.poller.register(self.cm.event_fd, select.POLLIN)

        # Background capture thread
        self._thread = threading.Thread(target=self._run, name="CamCaptureThread", daemon=True)
        self._thread.start()

    # ---------- public API ----------
    def close(self):
        """Signal stop and release camera resources and mmaps."""
        try:
            self._stop.set()
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
        finally:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            try:
                self.cam.stop()
            except Exception:
                pass
            try:
                self.cam.release()
            except Exception:
                pass
            # Close mapped planes
            for mmap_list in self.fb_maps.values():
                for mv in mmap_list:
                    try:
                        mm = mv.obj; mv.release()
                        if hasattr(mm, "close"): mm.close()
                    except Exception:
                        pass

    def set_analog_gain(self, gain: float) -> float:
        """Set the analog gain applied to subsequently requeued requests."""
        gain = float(gain)
        if not math.isfinite(gain):
            gain = self.base_analog_gain
        with self._control_lock:
            self.active_analog_gain = gain
            return self.active_analog_gain

    def reset_analog_gain(self) -> float:
        return self.set_analog_gain(self.base_analog_gain)

    def analog_gain_status(self) -> dict:
        with self._control_lock:
            return {
                "base": float(self.base_analog_gain),
                "active": float(self.active_analog_gain),
            }

    def analyze_patch_at_time(self, query_ts_ns: int,
                              angle_deg: float,
                              width_deg: float,
                              preview: bool = False):
        """
        Return (label, used_ts_ns, signed_age_ms, h_mean, s_mean, v_mean).

        If preview=True and debug saving is enabled, a patch debug image is saved
        to disk until the per-manager cap is reached. No GUI windows are opened here.

        The saved debug image now also draws the full grey ROI ring, so you can see
        exactly where the complete ring lies, not only the small patch.

        Notes on timing prints:
          [perf] mask_build shows the cost of building the angular patch mask only.
          total shows the cost of the entire function call.
        """
        # t0 starts total runtime measurement for this call
        t0 = time.monotonic_ns()

        # 1) Pick the buffered frame closest in time to the requested timestamp.
        #    The buffer holds tuples of (sensor_ts_ns, Ysq, Usq, Vsq).
        #    The lock guards against the capture thread mutating the buffer while we read.
        with self._buf_lock:
            if not self.frame_buffer:
                # No frames yet. Return a neutral result with NaNs for metrics.
                return ("none", query_ts_ns, float("nan"), float("nan"), float("nan"), float("nan"))
            idx_best = min(
                range(len(self.frame_buffer)),
                key=lambda i: abs(self.frame_buffer[i][0] - query_ts_ns)
            )
            used_ts, Ysq, Usq, Vsq = self.frame_buffer[idx_best]

        # 2) Work in square-space dimensions. H == W == side of the center crop.
        H = Ysq.shape[0]
        W = H  # by construction

        # 3) Get the binary ring mask for the configured ROI geometry.
        #    This call is cache-backed and only recomputes when geometry changes.
        ring = self._mask_cache.get_ring_mask(
            H, W, ROI_RADIUS, ROI_THICKNESS, ROI_OFFSET_X, ROI_OFFSET_Y
        )

        # 4) Build the angular "patch" mask on that ring for the requested angle and width.
        #    The mask cache precomputes angles only on ring pixels and scatters them,
        #    which is why this step is fast. t1..t2 isolates just the patch construction time.
        t1 = time.monotonic_ns()
        patch = self._mask_cache.get_patch_mask(
            H, W, ring, ROI_OFFSET_X, ROI_OFFSET_Y, angle_deg, width_deg
        )
        t2 = time.monotonic_ns()

        # 5) Convert only the pixels inside the patch from YUV to BGR vectors.
        #    This avoids converting the whole frame. U and V are sampled at 2x2 subsampling.
        Yv, Uv, Vv = _patch_yuv_vectors(Ysq, Usq, Vsq, patch)

        # Signed age in milliseconds of the actual frame we used relative to the query time.
        # Positive if the used frame is newer, negative if it is older.
        age_ms = (used_ts - query_ts_ns) / 1e6

        # Shared center of the ROI ring in square-space.
        cx, cy = W // 2 + ROI_OFFSET_X, H // 2 + ROI_OFFSET_Y

        # 6) Handle the case where the patch contains no pixels.
        #    This can happen if the ROI parameters or angle are out of bounds.
        if Yv is None:
            # Optional one-off preview save for debugging geometry, if enabled and under cap.
            if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
                view = _square_bgr_from_yuv(Ysq, Usq, Vsq)

                # Draw full grey ROI ring for orientation.
                cv2.circle(view, (cx, cy), int(ROI_RADIUS), ROI_GREY_COLOR,
                           thickness=int(ROI_THICKNESS), lineType=cv2.LINE_AA)

                # Tint patch area to make it visible in the saved image.
                view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))

                self._save_debug_frame(
                    view, used_ts, angle_deg, width_deg, label="none", hsv=None, age_ms=age_ms
                )
            # Print performance numbers and return a neutral classification.
            t3 = time.monotonic_ns()
            #print(f"[perf] mask_build={(t2 - t1)/1e6:.3f} ms | total={(t3 - t0)/1e6:.3f} ms")
            return ("none", used_ts, age_ms, float("nan"), float("nan"), float("nan"))

        # 7) Convert the YUV vectors to BGR vectors and compute robust color stats.
        B, G, R = _yuv_to_bgr_vec(Yv, Uv, Vv)
        label, confidence, stats = _classify_bgr_vectors(Yv, B, G, R)
        h_med = float(stats.get("h_med", float("nan")))
        s_med = float(stats.get("s_med", float("nan")))
        v_med = float(stats.get("v_med", float("nan")))

        # 8) Optional debug image save. This reconstructs a square BGR frame only for saving,
        #    draws the full grey ROI ring, tints the patch, draws the center line, and annotates
        #    HSV and timing. Saving stops automatically after the per-manager cap is reached.
        if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
            view = _square_bgr_from_yuv(Ysq, Usq, Vsq)

            # Draw full grey ROI ring for orientation.
            cv2.circle(view, (cx, cy), int(ROI_RADIUS), ROI_GREY_COLOR,
                       thickness=int(ROI_THICKNESS), lineType=cv2.LINE_AA)

            # Tint patch area to make it visible in the saved image.
            view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))

            # Draw center line for the requested angle.
            theta_std = math.radians(90.0 - angle_deg)  # 0 deg at bottom, clockwise positive
            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
            cv2.line(view, (cx, cy), (x_end, y_end), (0, 255, 255), 2, cv2.LINE_AA)

            # Annotate label and robust color stats.
            cv2.putText(view, f"Label: {label}  conf:{confidence:.2f}", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(view, f"H:{h_med:5.1f}  S:{s_med:5.1f}  V:{v_med:5.1f}",
                        (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(view, f"Δ to query: {age_ms:0.2f} ms", (12, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(view, f"R:{stats.get('red_ratio', 0.0):.2f} G:{stats.get('green_ratio', 0.0):.2f} valid:{stats.get('valid_ratio', 0.0):.2f}",
                        (12, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            self._save_debug_frame(
                view, used_ts, angle_deg, width_deg, label, (h_med, s_med, v_med), age_ms
            )

        # 9) Print performance numbers for quick profiling and return the result.
        t3 = time.monotonic_ns()
        if PRINT_PERF:
            print(f"[perf] mask_build={(t2 - t1)/1e6:.3f} ms | total={(t3 - t0)/1e6:.3f} ms")
        return (label, used_ts, age_ms, h_med, s_med, v_med, confidence, stats)

    def _encode_debug_view(self, view: np.ndarray, quality: int):
        quality = int(max(35, min(95, quality)))
        ok, encoded = cv2.imencode(".jpg", view, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return None
        return encoded.tobytes()

    def _draw_debug_annotations(self, view: np.ndarray, annotations, mirror_angles: bool = False):
        H = view.shape[0]
        W = H
        cx, cy = W // 2 + ROI_OFFSET_X, H // 2 + ROI_OFFSET_Y
        ring = self._mask_cache.get_ring_mask(
            H, W, ROI_RADIUS, ROI_THICKNESS, ROI_OFFSET_X, ROI_OFFSET_Y
        )

        cv2.circle(view, (cx, cy), int(ROI_RADIUS), ROI_GREY_COLOR,
                   thickness=int(ROI_THICKNESS), lineType=cv2.LINE_AA)

        label_colors = {
            "RED": np.array((40, 40, 255), dtype=np.uint8),
            "GREEN": np.array((40, 220, 80), dtype=np.uint8),
            "OTHER": np.array((80, 220, 255), dtype=np.uint8),
            "UNKNOWN": np.array((180, 180, 180), dtype=np.uint8),
            "none": np.array((180, 180, 180), dtype=np.uint8),
        }
        line_colors = {
            "RED": (40, 40, 255),
            "GREEN": (40, 220, 80),
            "OTHER": (80, 220, 255),
            "UNKNOWN": (180, 180, 180),
            "none": (180, 180, 180),
        }

        for idx, item in enumerate(annotations or ()):
            try:
                angle_deg = float(item["camera_angle_deg"])
                width_deg = float(item["patch_width_deg"])
            except (KeyError, TypeError, ValueError):
                continue
            if mirror_angles:
                angle_deg = (-angle_deg) % 360.0
            label = str(item.get("label", "UNKNOWN"))
            color = label_colors.get(label, label_colors["UNKNOWN"])
            patch = self._mask_cache.get_patch_mask(
                H, W, ring, ROI_OFFSET_X, ROI_OFFSET_Y, angle_deg, width_deg
            )
            if np.any(patch):
                blended = (0.45 * view[patch, :].astype(np.float32) + 0.55 * color.astype(np.float32)).astype(np.uint8)
                view[patch, :] = blended

            theta_std = math.radians(90.0 - angle_deg)
            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
            cv2.line(view, (cx, cy), (x_end, y_end),
                     line_colors.get(label, line_colors["UNKNOWN"]), 2, cv2.LINE_AA)

            slot_id = str(item.get("slot_id", "?"))
            confidence = item.get("confidence", 0.0)
            text = f"{slot_id} {label} {float(confidence):.2f}"
            tx = int(cx + (ROI_RADIUS + 16) * math.cos(theta_std))
            ty = int(cy + (ROI_RADIUS + 16) * math.sin(theta_std))
            tx = max(8, min(W - 260, tx))
            ty = max(54, min(H - 10, ty + (idx % 3) * 16))
            cv2.putText(view, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        line_colors.get(label, line_colors["UNKNOWN"]), 1, cv2.LINE_AA)

    def build_debug_jpeg(self, annotations=(), scale: float = 0.55, quality: int = 72,
                         mirror_angles: bool = False):
        """Return a JPEG overview of the newest frame with ROI query overlays.

        This is intended for the web debug UI only. It converts one latest frame
        once per requested overview, unlike analyze_patch_at_time(preview=True)
        which can save one image per color query.
        """
        with self._buf_lock:
            if not self.frame_buffer:
                return None, 0
            used_ts, Ysq, Usq, Vsq = self.frame_buffer[-1]

        view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
        latest_by_slot = {}
        unkeyed = []
        for item in annotations or ():
            slot_id = str(item.get("slot_id", ""))
            if slot_id:
                if slot_id not in latest_by_slot or item.get("t", 0.0) > latest_by_slot[slot_id].get("t", 0.0):
                    latest_by_slot[slot_id] = item
            else:
                unkeyed.append(item)
        recent = list(latest_by_slot.values()) + unkeyed[-8:]
        recent.sort(key=lambda item: item.get("t", 0.0))
        if recent:
            cv2.putText(view, f"camera queries: {len(recent)}", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(view, "camera queries: none", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)

        self._draw_debug_annotations(view, recent[-24:], mirror_angles=mirror_angles)

        if scale > 0.0 and scale < 1.0:
            out_w = max(1, int(view.shape[1] * scale))
            out_h = max(1, int(view.shape[0] * scale))
            view = cv2.resize(view, (out_w, out_h), interpolation=cv2.INTER_AREA)

        return self._encode_debug_view(view, quality), used_ts

    def build_exact_debug_jpeg(self, annotation, scale: float = 0.55, quality: int = 72,
                               mirror_angles: bool = False):
        """Return the actual analyzed frame for one query with its exact patch overlay."""
        if not annotation:
            return None, 0
        try:
            target_ts = int(annotation.get("used_ts_ns") or annotation.get("query_ts_ns"))
        except (TypeError, ValueError):
            return None, 0
        with self._buf_lock:
            if not self.frame_buffer:
                return None, 0
            idx_best = min(
                range(len(self.frame_buffer)),
                key=lambda i: abs(self.frame_buffer[i][0] - target_ts)
            )
            used_ts, Ysq, Usq, Vsq = self.frame_buffer[idx_best]

        view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
        self._draw_debug_annotations(view, [annotation], mirror_angles=mirror_angles)

        slot_id = str(annotation.get("slot_id", "?"))
        label = str(annotation.get("label", "UNKNOWN"))
        confidence = float(annotation.get("confidence", 0.0) or 0.0)
        distance = annotation.get("distance_m")
        age_ms = annotation.get("age_ms")
        frame_delta_ms = (used_ts - target_ts) / 1e6
        cv2.putText(view, "exact analyzed frame", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(view, f"{slot_id} {label} conf={confidence:.2f}", (12, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if distance is not None:
            cv2.putText(view, f"range={float(distance):.2f}m", (12, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if age_ms is not None:
            cv2.putText(view, f"query frame age={float(age_ms):+.1f}ms  display delta={frame_delta_ms:+.1f}ms", (12, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        if scale > 0.0 and scale < 1.0:
            out_w = max(1, int(view.shape[1] * scale))
            out_h = max(1, int(view.shape[0] * scale))
            view = cv2.resize(view, (out_w, out_h), interpolation=cv2.INTER_AREA)

        return self._encode_debug_view(view, quality), used_ts

    def _save_debug_frame(self, img_bgr: np.ndarray, ts_ns: int, angle_deg: float, width_deg: float,
                          label: str, hsv, age_ms: float):
        """Save a single PNG into debug_save_dir with a name that encodes parameters.

        Stops saving automatically after reaching debug_max_previews for this manager.
        """
        try:
            if self._debug_saved >= self.debug_max_previews or not self.debug_save_dir:
                return
            fname = f"patch_{ts_ns}_ang{angle_deg:.1f}_w{width_deg:.1f}_{label}"
            if hsv is not None:
                h, s, v = hsv
                fname += f"_H{h:.1f}_S{s:.1f}_V{v:.1f}"
            fname += f"_d{age_ms:+.1f}ms.png"
            path = os.path.join(self.debug_save_dir, fname)
            cv2.imwrite(path, img_bgr)
            self._debug_saved += 1
        except Exception:
            # Saving errors are ignored to keep analysis real time
            pass

    # ---------- background thread ------------
    def _run(self):
        """Capture loop that fills the rolling buffer and handles optional live preview."""
        try:
            while not self._stop.is_set():
                # Wait up to 1 s for camera events
                self.poller.poll(1000)

                for req in self.cm.get_ready_requests():
                    if self._stop.is_set():
                        break

                    md = req.metadata
                    ts  = md.get(lc.controls.SensorTimestamp)   # ns

                    fb = req.buffers[self.stream]
                    planes = self.fb_maps[fb]
                    y_plane, u_plane, v_plane = planes[0], planes[1], planes[2]

                    # Center square crop indices for the viewfinder frame
                    side = self.H_full
                    x0 = max(0, (self.W_full - side) // 2)

                    # Build only square Y, U, V. No full frame BGR conversion here.
                    Ysq, Usq, Vsq = _square_crop_from_yuv_planes(
                        self.W_full, self.H_full, x0, side, y_plane, u_plane, v_plane
                    )

                    # Buffer the planes with the sensor timestamp
                    if ts is not None:
                        with self._buf_lock:
                            self.frame_buffer.append((ts, Ysq, Usq, Vsq))
                            self.ts_hist.append(ts)

                    # Optional desktop preview window
                    if self.preview_on:
                        overlay = _square_bgr_from_yuv(Ysq, Usq, Vsq)

                        # Draw full grey ring and current live patch
                        ring = self._mask_cache.get_ring_mask(side, side, ROI_RADIUS, ROI_THICKNESS,
                                                              ROI_OFFSET_X, ROI_OFFSET_Y)
                        patch_live = self._mask_cache.get_patch_mask(side, side, ring,
                                                                     ROI_OFFSET_X, ROI_OFFSET_Y,
                                                                     self.patch_angle_deg, self.patch_width_deg)
                        cx, cy = side // 2 + ROI_OFFSET_X, side // 2 + ROI_OFFSET_Y

                        # Full ROI ring, now grey.
                        cv2.circle(overlay, (cx, cy), int(ROI_RADIUS), ROI_GREY_COLOR,
                                   thickness=int(ROI_THICKNESS), lineType=cv2.LINE_AA)

                        # Patch overlay.
                        overlay[patch_live, :] = np.maximum(
                            overlay[patch_live, :], np.array(PATCH_COLOR, dtype=np.uint8)
                        )

                        if SHOW_PATCH_CENTER:
                            theta_std = math.radians(90.0 - self.patch_angle_deg)
                            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
                            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
                            cv2.line(overlay, (cx, cy), (x_end, y_end),
                                     (0, 255, 255), 2, cv2.LINE_AA)

                        # Minimal HUD: FPS
                        fps_txt = "FPS: n/a"
                        with self._buf_lock:
                            if len(self.ts_hist) >= 2:
                                dt = (self.ts_hist[-1] - self.ts_hist[0]) / 1e9
                                if dt > 0:
                                    fps = (len(self.ts_hist) - 1) / dt
                                    fps_txt = f"FPS: {fps:5.2f}"
                        cv2.putText(overlay, fps_txt, (12, 26),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,200,0), 2)

                        cv2.imshow("CamManager preview", overlay)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self._stop.set()
                            break

                    # Requeue the request to continue streaming
                    req.reuse()
                    with self._control_lock:
                        active_gain = float(self.active_analog_gain)
                    req.set_control(lc.controls.AnalogueGain, active_gain)
                    self.cam.queue_request(req)

        except Exception:
            # Avoid the thread dying silently in production. Logging could be added.
            pass

# ==========================================================
#                        Test Main
# ==========================================================

def main():
    """Simple harness that repeatedly analyzes a patch slightly in the past.

    The loop prints the classification and basic timing. Stop with Ctrl+C.
    """
    PREVIEW_LIVE = False

    ANGLE_DEG = 180.0
    WIDTH_DEG = 10.0

    mgr = CamManager(preview_on=PREVIEW_LIVE,
                     patch_angle_deg=ANGLE_DEG,
                     patch_width_deg=WIDTH_DEG,
                     debug_save_dir="/home/carl/tmp/cammanager_debug",
                     debug_max_previews=10)
    try:
        while True:
            # Analyze a frame about 200 ms old relative to now
            target_ts = now_boottime_ns() - int(200 * 1e6)
            label, used_ts, age_ms, h, s, v, confidence, stats = mgr.analyze_patch_at_time(
                target_ts, ANGLE_DEG, WIDTH_DEG, preview=True
            )
            print(f"Patch -> {label:>5s} conf={confidence:.2f} | H:{h:5.1f} S:{s:5.1f} V:{v:5.1f} | "
                  f"R:{stats.get('red_ratio', 0.0):.2f} G:{stats.get('green_ratio', 0.0):.2f} | "
                  f"closest Δ={age_ms:0.2f} ms | saved={mgr._debug_saved}/{mgr.debug_max_previews}")
            time.sleep(0.06)
    except KeyboardInterrupt:
        pass
    finally:
        mgr.close()

if __name__ == "__main__":
    main()
