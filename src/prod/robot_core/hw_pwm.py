import os
import time
import threading
from collections import deque

# === HW PWM CONFIGURATION ===
PWM_CHIP = 0  # pwmchip0

# Channel mapping:
# 0: Steering servo (50 Hz RC pulse)
# 1: LED (0..1)
# 2: Drive reverse  (0..1)
# 3: Drive forward  (0..1)

# Frequencies
SERVO_FREQ_HZ = 50
LED_FREQ_HZ   = 1000
DRIVE_FREQ_HZ = 20000

# Periods
SERVO_PERIOD_NS = (1_000_000 // SERVO_FREQ_HZ) * 1000
LED_PERIOD_NS   = (1_000_000 // LED_FREQ_HZ)   * 1000
DRIVE_PERIOD_NS = (1_000_000 // DRIVE_FREQ_HZ) * 1000

# Servo pulse range (in microseconds)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
MIN_PULSE_NS = MIN_PULSE_US * 1000
MAX_PULSE_NS = MAX_PULSE_US * 1000

# === Paths ===
BASE_PATH   = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
PWM0_PATH   = f"{BASE_PATH}/pwm0"  # Servo
PWM1_PATH   = f"{BASE_PATH}/pwm1"  # LED
PWM2_PATH   = f"{BASE_PATH}/pwm2"  # Drive reverse
PWM3_PATH   = f"{BASE_PATH}/pwm3"  # Drive forward

_drive_state = {
    "mode": "unknown",
    "reverse": 0.0,
    "forward": 0.0,
    "req_reverse": 0.0,
    "req_forward": 0.0,
    "read_reverse": "",
    "read_forward": "",
    "throttle": 0.0,
    "brake_level": 0.0,
    "brake_order": "reverse_first",
    "t": 0.0,
    "command_t": 0.0,
    "read_t": 0.0,
    "readback_enabled": False,
}
_drive_state_lock = threading.Lock()

_timing_lock = threading.Lock()
_timing_enabled = False
_timing_slow_threshold_ms = 10.0
_timing_events = deque(maxlen=80)

_write_opt_lock = threading.Lock()
_fd_lock = threading.Lock()
_exact_dedupe_enabled = True
_cache_fds_enabled = False
_last_duty_values = {}
_cached_fds = {}
_cached_fd_count_snapshot = 0
_write_counts = {}
_skip_counts = {}
_cache_fallback_counts = {}
_cache_open_counts = {}
_cache_close_counts = {}


def _counter_key(path):
    text = str(path)
    if text.startswith(PWM0_PATH):
        return "servo"
    if text.startswith(PWM1_PATH):
        return "front_led"
    if text.startswith(PWM2_PATH):
        return "drive_reverse"
    if text.startswith(PWM3_PATH):
        return "drive_forward"
    return "other"


def _is_duty_path(path):
    return str(path).endswith("/duty_cycle")


def _coerce_duty_value(value):
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


def _bump_counter(counts, key):
    counts[key] = int(counts.get(key, 0)) + 1
    counts["total"] = int(counts.get("total", 0)) + 1


def _record_cached_fd_count(count):
    global _cached_fd_count_snapshot
    with _write_opt_lock:
        _cached_fd_count_snapshot = int(max(0, count))


def _record_cache_open(key, count):
    global _cached_fd_count_snapshot
    with _write_opt_lock:
        _bump_counter(_cache_open_counts, key)
        _cached_fd_count_snapshot = int(max(0, count))


def _record_cache_closes(keys, count):
    global _cached_fd_count_snapshot
    with _write_opt_lock:
        for key in keys:
            _bump_counter(_cache_close_counts, key)
        _cached_fd_count_snapshot = int(max(0, count))


def _close_cached_fds_locked():
    closed_keys = []
    for path_text, fd in list(_cached_fds.items()):
        try:
            os.close(fd)
        except OSError:
            pass
        closed_keys.append(_counter_key(path_text))
    _cached_fds.clear()
    return closed_keys


def configure_write_optimization(exact_dedupe=True, cache_fds=False):
    global _exact_dedupe_enabled, _cache_fds_enabled
    close_fds = False
    with _write_opt_lock:
        _exact_dedupe_enabled = bool(exact_dedupe)
        cache_fds = bool(cache_fds)
        close_fds = bool(_cache_fds_enabled and not cache_fds)
        _cache_fds_enabled = cache_fds
        _last_duty_values.clear()
    if close_fds:
        with _fd_lock:
            closed_keys = _close_cached_fds_locked()
        _record_cache_closes(closed_keys, 0)


def close_cached_fds():
    with _fd_lock:
        closed_keys = _close_cached_fds_locked()
    _record_cache_closes(closed_keys, 0)


def get_write_stats():
    with _write_opt_lock:
        keys = ("servo", "front_led", "drive_reverse", "drive_forward", "other", "total")
        stats = {
            "pwm_exact_dedupe_enabled": bool(_exact_dedupe_enabled),
            "pwm_cache_fds_enabled": bool(_cache_fds_enabled),
            "pwm_cached_fd_count": int(_cached_fd_count_snapshot),
        }
        for key in keys:
            stats[f"pwm_write_count_{key}"] = int(_write_counts.get(key, 0))
            stats[f"pwm_skip_count_{key}"] = int(_skip_counts.get(key, 0))
            stats[f"pwm_cache_fallback_count_{key}"] = int(_cache_fallback_counts.get(key, 0))
            stats[f"pwm_cache_open_count_{key}"] = int(_cache_open_counts.get(key, 0))
            stats[f"pwm_cache_close_count_{key}"] = int(_cache_close_counts.get(key, 0))
    return stats


def configure_timing(enabled=False, slow_threshold_ms=10.0, ring_size=80):
    global _timing_enabled, _timing_slow_threshold_ms, _timing_events
    with _timing_lock:
        _timing_enabled = bool(enabled)
        try:
            _timing_slow_threshold_ms = max(0.0, float(slow_threshold_ms))
        except (TypeError, ValueError):
            _timing_slow_threshold_ms = 10.0
        try:
            maxlen = max(1, int(ring_size))
        except (TypeError, ValueError):
            maxlen = 80
        if _timing_events.maxlen != maxlen:
            _timing_events = deque(maxlen=maxlen)


def _classify_path(path):
    text = str(path)
    if text.startswith(PWM0_PATH):
        return "servo" + text[len(PWM0_PATH):]
    if text.startswith(PWM1_PATH):
        return "front_led" + text[len(PWM1_PATH):]
    if text.startswith(PWM2_PATH):
        return "drive_reverse" + text[len(PWM2_PATH):]
    if text.startswith(PWM3_PATH):
        return "drive_forward" + text[len(PWM3_PATH):]
    if text.startswith(BASE_PATH):
        return "pwmchip" + text[len(BASE_PATH):]
    return text


def _record_timing_event(path, value, duration_ms):
    with _timing_lock:
        enabled = _timing_enabled
        threshold = _timing_slow_threshold_ms
    slow = float(duration_ms) >= threshold
    if not enabled and not slow:
        return
    event = {
        "t": time.monotonic(),
        "category": "hw_pwm",
        "thread": threading.current_thread().name,
        "path": str(path),
        "path_kind": _classify_path(path),
        "value": str(value)[:48],
        "duration_ms": float(duration_ms),
        "slow": bool(slow),
    }
    with _timing_lock:
        _timing_events.append(event)


def drain_timing_events():
    with _timing_lock:
        events = list(_timing_events)
        _timing_events.clear()
    return events

def _write_text(path, value_text, use_cached_fd=False):
    if not use_cached_fd:
        with open(path, "w") as f:
            f.write(value_text)
        return

    path_text = str(path)
    data = value_text.encode("ascii")
    opened_key = None
    cached_fd_count = None
    try:
        with _fd_lock:
            fd = _cached_fds.get(path_text)
            if fd is None:
                fd = os.open(path_text, os.O_WRONLY)
                _cached_fds[path_text] = fd
                opened_key = _counter_key(path_text)
                cached_fd_count = len(_cached_fds)
            os.lseek(fd, 0, os.SEEK_SET)
            written = os.write(fd, data)
            if written != len(data):
                raise OSError(f"partial PWM sysfs write: {written}/{len(data)} bytes")
        if opened_key is not None:
            _record_cache_open(opened_key, cached_fd_count)
        return
    except OSError:
        key = _counter_key(path_text)
        with _write_opt_lock:
            _bump_counter(_cache_fallback_counts, key)
        closed_key = None
        cached_fd_count = None
        with _fd_lock:
            fd = _cached_fds.pop(path_text, None)
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
                closed_key = key
            cached_fd_count = len(_cached_fds)
        if closed_key is not None:
            _record_cache_closes([closed_key], cached_fd_count)
        elif cached_fd_count is not None:
            _record_cached_fd_count(cached_fd_count)
        with open(path_text, "w") as f:
            f.write(value_text)


def write(path, value):
    path_text = str(path)
    value_text = str(value)
    duty_value = _coerce_duty_value(value) if _is_duty_path(path_text) else None
    key = _counter_key(path_text)
    if duty_value is not None:
        with _write_opt_lock:
            if _exact_dedupe_enabled and _last_duty_values.get(path_text) == duty_value:
                _bump_counter(_skip_counts, key)
                return False

    start = time.monotonic()
    try:
        _write_text(path_text, value_text, use_cached_fd=(duty_value is not None and _cache_fds_enabled))
        if duty_value is not None:
            with _write_opt_lock:
                _last_duty_values[path_text] = duty_value
                _bump_counter(_write_counts, key)
        return True
    finally:
        _record_timing_event(path_text, value_text, (time.monotonic() - start) * 1000.0)

def read(path):
    start = time.monotonic()
    try:
        with open(path) as f:
            return f.read().strip()
    finally:
        _record_timing_event(path, "read", (time.monotonic() - start) * 1000.0)

def ensure_exported(pwm_path, channel_index):
    if not os.path.exists(pwm_path):
        write(f"{BASE_PATH}/export", channel_index)
        time.sleep(0.05)

def setup_channel(pwm_path, period_ns, initial_duty_ns=0):
    ensure_exported(pwm_path, int(pwm_path[-1]))  # uses last char (0..3)
    try: write(f"{pwm_path}/enable", 0)
    except: pass
    write(f"{pwm_path}/period", period_ns)
    write(f"{pwm_path}/duty_cycle", initial_duty_ns)
    write(f"{pwm_path}/enable", 1)

def set_fraction(pwm_path, period_ns, frac):
    x = max(0.0, min(1.0, float(frac)))
    duty_ns = int(period_ns * x)
    write(f"{pwm_path}/duty_cycle", duty_ns)

def _read_fraction(pwm_path, period_ns):
    try:
        return max(0.0, min(1.0, float(read(f"{pwm_path}/duty_cycle")) / float(period_ns)))
    except Exception:
        return ""

def _record_drive_state(mode, reverse, forward, throttle=0.0, brake_level=0.0, brake_order="reverse_first", command_t=None):
    command_time = time.monotonic() if command_t is None else float(command_t)
    req_reverse = max(0.0, min(1.0, float(reverse)))
    req_forward = max(0.0, min(1.0, float(forward)))
    order = "forward_first" if str(brake_order) == "forward_first" else "reverse_first"
    with _drive_state_lock:
        _drive_state.update(
            {
                "mode": str(mode),
                "reverse": req_reverse,
                "forward": req_forward,
                "req_reverse": req_reverse,
                "req_forward": req_forward,
                "throttle": max(-1.0, min(1.0, float(throttle))),
                "brake_level": max(0.0, min(1.0, float(brake_level))),
                "brake_order": order,
                "t": command_time,
                "command_t": command_time,
            }
        )

def refresh_drive_readback():
    read_reverse = _read_fraction(PWM2_PATH, DRIVE_PERIOD_NS)
    read_forward = _read_fraction(PWM3_PATH, DRIVE_PERIOD_NS)
    read_time = time.monotonic()
    with _drive_state_lock:
        _drive_state.update(
            {
                "read_reverse": read_reverse,
                "read_forward": read_forward,
                "read_t": read_time,
                "readback_enabled": True,
            }
        )

def get_drive_state():
    with _drive_state_lock:
        return dict(_drive_state)

# === Servo (channel 0) ===
def angle_to_duty_ns(angle):
    angle = max(0, min(180, angle))
    return int(MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * angle // 180)

def setup_pwm_servo():
    setup_channel(PWM0_PATH, SERVO_PERIOD_NS, 1_500_000)  # center

def cleanup_pwm_servo():
    write(f"{PWM0_PATH}/enable", 0)

def set_steering_angle(normalized):
    x = max(-1.0, min(1.0, normalized))
    angle = 45.0 * x + 90.0
    angle = min(max(45, angle), 135)
    duty_ns = angle_to_duty_ns(angle)
    write(f"{PWM0_PATH}/duty_cycle", duty_ns)

# === LED (channel 1) ===
def setup_pwm_led():
    setup_channel(PWM1_PATH, LED_PERIOD_NS, 0)

def set_led(level):
    set_fraction(PWM1_PATH, LED_PERIOD_NS, level)

# === Drive (channels 2 & 3) ===
def setup_pwm_drive():
    setup_channel(PWM2_PATH, DRIVE_PERIOD_NS, 0)
    setup_channel(PWM3_PATH, DRIVE_PERIOD_NS, 0)
    coast()

def set_drive(throttle):
    #print("Drive Power: " + str(throttle))
    t = max(-1.0, min(1.0, float(throttle)))
    command_t = time.monotonic()
    if t > 0:
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)  # reverse off
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, t)    # forward on
        _record_drive_state("drive", 0.0, t, throttle=t, command_t=command_t)
    elif t < 0:
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)  # forward off
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, -t)   # reverse on
        _record_drive_state("drive", -t, 0.0, throttle=t, command_t=command_t)
    else:
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)
        _record_drive_state("drive_zero", 0.0, 0.0, throttle=0.0, command_t=command_t)

def brake(level, order="reverse_first"):
    lvl = max(0.0, min(1.0, float(level)))
    brake_order = "forward_first" if str(order) == "forward_first" else "reverse_first"
    command_t = time.monotonic()
    if brake_order == "forward_first":
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, lvl)
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, lvl)
    else:
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, lvl)
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, lvl)
    _record_drive_state("brake", lvl, lvl, brake_level=lvl, brake_order=brake_order, command_t=command_t)

def coast():
    command_t = time.monotonic()
    set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)
    set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)
    _record_drive_state("coast", 0.0, 0.0, command_t=command_t)

# === Logging helper for tests ===
_t0 = time.monotonic()
def log(msg):
    dt = time.monotonic() - _t0
    print(f"[{dt:6.3f}s] {msg}")

# === Demo / Tests ===
if __name__ == "__main__":
    try:
        print(f"Starting: pwmchip{PWM_CHIP}")
        setup_pwm_servo()
        setup_pwm_led()
        setup_pwm_drive()

        # --- LED fade test ---
        log("LED fade: 0% → 100%")
        for i in range(0, 101):
            set_led(i/100.0)
            time.sleep(0.005)
        log("LED fade: 100% → 0%")
        for i in range(100, -1, -1):
            set_led(i/100.0)
            time.sleep(0.005)
        set_led(0.2)
        log("LED set to 20%")

        # --- Drive ramp test ---
        log("Drive ramp: reverse -1.0 → 0.0")
        for i in range(0, 101):
            set_drive(-i/100.0)
            if i % 20 == 0:
                log(f"Drive throttle = {-i/100.0:+.2f}")
            time.sleep(0.01)

        log("Drive ramp: 0.0 → +1.0 forward")
        for i in range(0, 101):
            set_drive(i/100.0)
            if i % 20 == 0:
                log(f"Drive throttle = {i/100.0:+.2f}")
            time.sleep(0.01)

        # --- Comparison test: Full forward → BRAKE vs → COAST ---
        # A) Forward → BRAKE(100%)
        log("Comparison A: FULL FORWARD (+1.0) for 1.5s")
        set_drive(1.0)
        time.sleep(1.5)
        log("Apply BRAKE 100% for 1.5s")
        brake(1.0)
        time.sleep(1.5)
        log("Coast for 0.5s (after braking)")
        coast()
        time.sleep(0.5)

        # Cool-down pause
        log("Pause 1.0s before Comparison B")
        time.sleep(1.0)

        # B) Forward → COAST
        log("Comparison B: FULL FORWARD (+1.0) for 1.5s")
        set_drive(1.0)
        time.sleep(1.5)
        log("COAST for 1.5s")
        coast()
        time.sleep(1.5)

        # Restore neutral
        set_drive(0.0)
        coast()
        log("Drive neutral")

        # --- Brake strength sweep (optional extra insight) ---
        log("Brake strength sweep: 0% → 100% → 0%")
        for i in range(0, 101, 5):
            brake(i/100.0)
            time.sleep(0.05)
        for i in range(100, -1, -5):
            brake(i/100.0)
            time.sleep(0.05)
        coast()
        log("Brake released; coasting")

        # --- Steering smooth sweep loop ---
        log("Steering sweep: -1.0 ↔ +1.0 (loop)")
        steps = 200
        dwell = 0.01
        while True:
            for i in range(steps + 1):
                x = -1.0 + 2.0 * i / steps
                set_steering_angle(x)
                if i % 50 == 0:
                    log(f"Steering normalized = {x:+.2f}")
                time.sleep(dwell)
            for i in range(steps + 1):
                x = 1.0 - 2.0 * i / steps
                set_steering_angle(x)
                if i % 50 == 0:
                    log(f"Steering normalized = {x:+.2f}")
                time.sleep(dwell)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Safe shutdown states
        log("Shutdown: center steering, LED off, motor coast")
        set_steering_angle(0.0)
        set_led(0.0)
        coast()
        time.sleep(0.4)
        cleanup_pwm_servo()
        for p in (PWM1_PATH, PWM2_PATH, PWM3_PATH):
            try: write(f"{p}/enable", 0)
            except: pass
        print("PWM disabled.")
