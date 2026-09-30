#!/usr/bin/env python3
"""
DriveBase v2.7 - robust creep test for Pi 5 + Ubuntu 24.04

Goal:
- Make low-speed creep testing reliable enough that bogus velocity spikes cannot poison control.
- Keep the good parts from dbtst4:
  * breakaway supervisor
  * hold-duty capture
  * stall detection from encoder count progress
- Replace the fragile low-speed estimator/control interaction with:
  * count-truth-first motion logic
  * median-window rate estimate
  * optional period estimate used only when credible
  * spike rejection / freeze-on-bad-sample
  * no overspeed->coast at very low creep commands
  * stricter stall detection that ignores ±1 count jitter
  * forced fresh breakaway on low-speed target changes when effectively stuck

Hardware assumptions:
- Raspberry Pi 5 on Ubuntu 24.04
- hw_pwm.py in same folder
- Quadrature encoder on GPIO20 / GPIO21 via /dev/gpiochip4
- Motor encoder unchanged, drivetrain gear ratio now 21:60 motor:axle -> 60/21 motor rev per wheel rev

Usage:
- Wheels off ground first.
- Run encoder sanity test first if unsure.
- Then run creep test.
"""

from __future__ import annotations

import math
import sys
import time
import threading
import statistics
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Optional, Tuple, List

try:
    from . import hw_pwm as hpwm
except ImportError:
    import hw_pwm as hpwm

try:
    import gpiod
except ImportError as e:
    raise SystemExit(
        "Missing dependency: python gpiod module not found.\n"
        "Install inside your venv or system Python, e.g.:\n"
        "  pip install gpiod\n"
        "or apt package if available on your distro.\n"
    ) from e

try:
    from gpiozero import PWMLED
except ImportError:
    PWMLED = None


def monotonic() -> float:
    return time.monotonic()


REAR_LED_PIN = 11
RED_LED_PIN = 5
GREEN_LED_PIN = 6
LED_PWM_FREQ = 1000


# =========================
# Configuration
# =========================

# Full-battery-normalized profile measured after the 21:60 drivetrain update.
# Values are stored as (physical speed in m/s, steady duty). Reverse currently
# mirrors this profile because the calibration data showed similar behavior.
CALIBRATED_SPEED_DUTY_PROFILE: Tuple[Tuple[float, float], ...] = (
    (0.052776, 0.57),
    (0.071311, 0.58),
    (0.092461, 0.59),
    (0.111355, 0.60),
    (0.127400, 0.61),
    (0.150287, 0.62),
    (0.169326, 0.63),
    (0.190792, 0.64),
    (0.213026, 0.65),
    (0.236285, 0.66),
    (0.262405, 0.67),
    (0.284422, 0.68),
    (0.306154, 0.69),
    (0.323497, 0.70),
    (0.337189, 0.71),
    (0.353030, 0.72),
    (0.371083, 0.73),
    (0.392646, 0.74),
    (0.428524, 0.75),
    (0.445361, 0.76),
    (0.462799, 0.77),
    (0.482956, 0.78),
    (0.503387, 0.79),
    (0.524604, 0.80),
    (0.545477, 0.81),
    (0.571638, 0.82),
    (0.591996, 0.83),
    (0.609591, 0.84),
    (0.625487, 0.85),
    (0.654655, 0.86),
    (0.672600, 0.87),
    (0.691075, 0.88),
    (0.707110, 0.89),
    (0.723203, 0.90),
    (0.740242, 0.91),
    (0.755527, 0.92),
    (0.770237, 0.93),
    (0.784998, 0.94),
    (0.809141, 0.95),
    (0.823173, 0.96),
    (0.855414, 0.97),
    (0.885235, 0.98),
    (0.914402, 0.99),
    (0.937667, 1.00),
)

@dataclass(frozen=True)
class DriveConfig:
    # Encoder
    enc_gpiochip: str = "/dev/gpiochip4"
    enc_pin_a: int = 20
    enc_pin_b: int = 21
    use_pullup_bias: bool = True

    # Calibration / geometry
    counts_per_enc_rev_x4: float = 48.0 * 8.7025 * 1.02528
    reduction_motor_rev_per_wheel_rev: float = 60.0 / 21.0
    wheel_diameter_m: float = 0.0432

    # Control loop
    control_hz: float = 100.0

    # Steering calibration
    steering_center_trim: float = -0.017

    # PI controller (small integrator for creep)
    kp: float = 0.7
    ki: float = 1.6
    integrator_limit: float = 0.45

    # Feedforward / friction
    u_static: float = 0.10
    u_vel_ff: float = 1.15
    v_static_enable: float = 0.025
    speed_ff_profile_enable: bool = True
    speed_ff_profile: Tuple[Tuple[float, float], ...] = CALIBRATED_SPEED_DUTY_PROFILE
    speed_ff_correction_clamp: float = 0.08
    speed_ff_trim_kp: float = 0.18
    speed_ff_trim_ki: float = 0.35
    speed_ff_trim_integrator_limit: float = 0.05
    external_velocity_feedback_enable: bool = True
    external_velocity_feedback_min_speed_mps: float = 0.20
    external_velocity_feedback_max_age_s: float = 0.08
    external_velocity_feedback_filter_alpha: float = 0.35

    # Output deadband/hysteresis
    duty_update_deadband: float = 0.002
    u_min_actuate: float = 0.04
    u_min_hyst: float = 0.01

    # Velocity command slew
    v_slew_mps2: float = 0.35
    v_decel_slew_mps2: float = 1.40
    decel_feedforward_disable_margin_mps: float = 0.03
    velocity_duty_slew_per_s: float = 3.00

    # Limits
    max_abs_speed_mps_physical: float = 1.2
    max_command_duty: float = 1.0

    # Creep / breakaway supervisor. Disabled by default because the calibrated
    # speed profile is now the primary controller for all commanded speeds.
    creep_breakaway_enable: bool = False
    creep_threshold_mps: float = 0.05
    breakaway_enable: bool = True
    breakaway_max_speed_mps: float = 0.05
    breakaway_detect_counts: int = 12
    breakaway_min_time_s: float = 0.06
    breakaway_timeout_s: float = 0.80
    breakaway_duty_start: float = 0.25
    breakaway_duty_max: float = 0.90
    breakaway_duty_ramp_per_s: float = 1.2

    # Stall detection based on meaningful forward count progress in REG mode
    stall_timeout_s: float = 0.22

    # Hold-duty behavior
    hold_decay_per_s: float = 0.015
    hold_decay_overspeed_margin_mps: float = 0.015
    hold_min: float = 0.42
    hold_capture_factor: float = 0.88

    # Creep correction around hold duty
    creep_corr_clamp: float = 0.16
    creep_duty_slew_per_s: float = 1.20

    # Estimator robustness
    no_edge_timeout_s: float = 0.08
    v_rate_window: int = 5             # median of recent rate estimates
    v_blend_small_counts: bool = True
    spike_abs_mps: float = 0.25        # impossible for creep on this robot
    spike_factor_vs_cmd: float = 3.0   # reject if too large relative to cmd
    bad_sample_freeze_s: float = 0.10  # hold last credible speed briefly

    # Debug
    debug: bool = True
    debug_hz: float = 10.0
    pwm_readback_hz: float = 5.0
    pwm_exact_dedupe_enable: bool = True
    pwm_cache_fds: bool = True
    timing_detail_enable: bool = False
    timing_slow_threshold_ms: float = 10.0
    timing_ring_size: int = 160


# =========================
# Quadrature decoder
# =========================

class QuadratureDecoder:
    _TRANSITION_DELTA = {
        (0b00, 0b01): +1,
        (0b01, 0b11): +1,
        (0b11, 0b10): +1,
        (0b10, 0b00): +1,
        (0b00, 0b10): -1,
        (0b10, 0b11): -1,
        (0b11, 0b01): -1,
        (0b01, 0b00): -1,
    }

    def __init__(self):
        self.count = 0
        self._last_state: Optional[int] = None
        self._last_edge_t: Optional[float] = None
        self.last_step_dt: Optional[float] = None
        self.lock = threading.Lock()

    def reset(self) -> None:
        with self.lock:
            self.count = 0
            self._last_state = None
            self._last_edge_t = None
            self.last_step_dt = None

    def update_state(self, a: int, b: int, t_now: float) -> None:
        new_state = ((a & 1) << 1) | (b & 1)
        with self.lock:
            if self._last_state is None:
                self._last_state = new_state
                self._last_edge_t = t_now
                self.last_step_dt = None
                return

            old_state = self._last_state
            if new_state == old_state:
                return

            delta = self._TRANSITION_DELTA.get((old_state, new_state), 0)
            if delta != 0:
                self.count += delta
                if self._last_edge_t is not None:
                    self.last_step_dt = max(1e-6, t_now - self._last_edge_t)
                self._last_edge_t = t_now

            self._last_state = new_state

    def get_count_and_step_dt(self) -> Tuple[int, Optional[float], Optional[float]]:
        with self.lock:
            return self.count, self.last_step_dt, self._last_edge_t


# =========================
# GPIO reader (libgpiod v2)
# =========================

class GPIODQuadratureReader:
    def __init__(self, cfg: DriveConfig, decoder: QuadratureDecoder):
        self.cfg = cfg
        self.decoder = decoder
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._pwm_readback_running = False
        self._pwm_readback_thread: Optional[threading.Thread] = None
        self._req = None
        self._a_level = 0
        self._b_level = 0

    def _coerce_value(self, v) -> int:
        """Handle libgpiod Python bindings that return either ints/bools or Value enums."""
        try:
            return int(v)
        except Exception:
            pass

        name = getattr(v, "name", None)
        if isinstance(name, str):
            up = name.upper()
            if up.endswith("ACTIVE"):
                return 1
            if up.endswith("INACTIVE"):
                return 0

        val = getattr(v, "value", None)
        if isinstance(val, (int, bool)):
            return int(val)

        s = str(v).upper()
        if "ACTIVE" in s:
            return 1
        if "INACTIVE" in s:
            return 0

        raise TypeError(f"Unsupported gpiod line value type: {type(v)!r}, value={v!r}")

    def start(self) -> None:
        if self._running:
            return

        bias = gpiod.line.Bias.PULL_UP if self.cfg.use_pullup_bias else gpiod.line.Bias.AS_IS
        settings = gpiod.LineSettings(
            direction=gpiod.line.Direction.INPUT,
            edge_detection=gpiod.line.Edge.BOTH,
            bias=bias,
        )
        self._req = gpiod.request_lines(
            self.cfg.enc_gpiochip,
            consumer="dbtst6-quad",
            config={
                self.cfg.enc_pin_a: settings,
                self.cfg.enc_pin_b: settings,
            },
        )

        vals = self._req.get_values([self.cfg.enc_pin_a, self.cfg.enc_pin_b])
        self._a_level = self._coerce_value(vals[0])
        self._b_level = self._coerce_value(vals[1])
        self.decoder.update_state(self._a_level, self._b_level, monotonic())

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._req is not None:
            try:
                self._req.release()
            except Exception:
                pass
            self._req = None

    def _loop(self) -> None:
        pin_a = self.cfg.enc_pin_a
        pin_b = self.cfg.enc_pin_b

        while self._running:
            try:
                ret = self._req.wait_edge_events(timeout=0.5)
                if ret is False:
                    continue

                events = self._req.read_edge_events(64)
                if not events:
                    continue

                for ev in events:
                    off = ev.line_offset
                    et = getattr(ev, "event_type", None)
                    et_s = str(et)

                    if et_s.endswith("RISING_EDGE") or et_s.endswith("RISING"):
                        new_level = 1
                    elif et_s.endswith("FALLING_EDGE") or et_s.endswith("FALLING"):
                        new_level = 0
                    else:
                        continue

                    if off == pin_a:
                        self._a_level = new_level
                    elif off == pin_b:
                        self._b_level = new_level
                    else:
                        continue

                    self.decoder.update_state(self._a_level, self._b_level, monotonic())

            except Exception:
                time.sleep(0.01)


# =========================
# Robust low-speed estimator
# =========================

class RobustVelocityEstimator:
    """
    Count-truth-first estimator.

    Key idea:
    - Position and motion/stall truth come from quadrature counts.
    - Velocity is only used for control if it looks credible.
    - When low-speed samples are bad, keep the last good value briefly instead of
      injecting nonsense into the controller.
    """

    def __init__(self, cfg: DriveConfig, decoder: QuadratureDecoder):
        self.cfg = cfg
        self.decoder = decoder

        self.wheel_circ_m = math.pi * cfg.wheel_diameter_m
        self.counts_per_wheel_rev = float(cfg.counts_per_enc_rev_x4) * float(cfg.reduction_motor_rev_per_wheel_rev)
        self.m_per_count = self.wheel_circ_m / self.counts_per_wheel_rev

        self._last_count = 0
        self._last_t = monotonic()
        self._last_good_v = 0.0
        self._last_good_t = self._last_t
        self._rate_hist: List[float] = []

        self.position_m = 0.0
        self.velocity_mps = 0.0
        self.v_rate_mps = 0.0
        self.last_count = 0
        self.last_dcount = 0

    def reset(self) -> None:
        self.decoder.reset()
        self._last_count = 0
        self._last_t = monotonic()
        self._last_good_v = 0.0
        self._last_good_t = self._last_t
        self._rate_hist = []
        self.position_m = 0.0
        self.velocity_mps = 0.0
        self.v_rate_mps = 0.0
        self.last_count = 0
        self.last_dcount = 0

    def _median_rate(self) -> float:
        if not self._rate_hist:
            return 0.0
        return float(statistics.median(self._rate_hist))

    def update(self, v_cmd_abs: float = 0.0) -> Tuple[float, float, float, int, int, bool]:
        t = monotonic()
        dt = max(1e-6, t - self._last_t)
        self._last_t = t

        count, step_dt, last_edge_t = self.decoder.get_count_and_step_dt()
        dcount = count - self._last_count
        self._last_count = count

        self.last_count = count
        self.last_dcount = dcount
        self.position_m = count * self.m_per_count

        # Truthy medium/high-speed estimate from counts over loop dt.
        v_rate_inst = (dcount * self.m_per_count) / dt
        self._rate_hist.append(v_rate_inst)
        if len(self._rate_hist) > max(3, self.cfg.v_rate_window):
            self._rate_hist.pop(0)
        v_rate_med = self._median_rate()
        self.v_rate_mps = v_rate_med

        # Optional period estimate only if recent edge exists.
        v_period = None
        if step_dt is not None and step_dt > 0 and last_edge_t is not None:
            if (t - last_edge_t) <= self.cfg.no_edge_timeout_s:
                sign = 1.0 if (dcount > 0 or self.velocity_mps >= 0) else -1.0
                v_period = sign * (self.m_per_count / step_dt)

        # Build a candidate estimate conservatively.
        if abs(dcount) >= 2:
            v_cand = v_rate_med
        elif abs(dcount) == 1:
            if v_period is not None and self.cfg.v_blend_small_counts:
                v_cand = 0.7 * v_rate_med + 0.3 * v_period
            else:
                v_cand = v_rate_med
        else:
            # no count progress this tick -> either zero, or keep short freeze if very recent last good
            if (t - self._last_good_t) <= self.cfg.bad_sample_freeze_s:
                v_cand = self._last_good_v
            else:
                v_cand = 0.0

        # Reject impossible spikes.
        bad = False
        if abs(v_cand) > self.cfg.max_abs_speed_mps_physical:
            bad = True
        if abs(v_cand) > self.cfg.spike_abs_mps and v_cmd_abs <= self.cfg.creep_threshold_mps:
            bad = True
        if v_cmd_abs > 1e-4 and abs(v_cand) > self.cfg.spike_factor_vs_cmd * max(0.02, v_cmd_abs) and abs(dcount) <= 1:
            bad = True
        if abs(dcount) == 0 and v_period is None and abs(v_cand) > 1e-6:
            bad = True

        if bad:
            # Freeze briefly on last good value rather than injecting noise.
            if (t - self._last_good_t) <= self.cfg.bad_sample_freeze_s:
                v_est = self._last_good_v
            else:
                v_est = 0.0
        else:
            v_est = v_cand
            self._last_good_v = v_est
            self._last_good_t = t

        # Light smoothing only on credible estimate.
        alpha = 0.25
        self.velocity_mps = (1.0 - alpha) * self.velocity_mps + alpha * v_est
        return self.position_m, self.velocity_mps, self.v_rate_mps, count, dcount, bad


# =========================
# Velocity controller
# =========================

class VelocityController:
    def __init__(self, cfg: DriveConfig):
        self.cfg = cfg
        self.integrator = 0.0
        self.last_error_mps = 0.0
        self.last_ff_duty = 0.0
        self.last_pi_trim_duty = 0.0
        self._profile = tuple(sorted(
            (float(speed), float(duty))
            for speed, duty in getattr(cfg, "speed_ff_profile", ())
            if float(speed) > 0.0 and float(duty) > 0.0
        ))

    def reset(self) -> None:
        self.integrator = 0.0
        self.last_error_mps = 0.0
        self.last_ff_duty = 0.0
        self.last_pi_trim_duty = 0.0

    def profile_enabled(self) -> bool:
        return bool(getattr(self.cfg, "speed_ff_profile_enable", False)) and bool(self._profile)

    def feedforward_abs(self, speed_abs: float) -> float:
        speed_abs = abs(float(speed_abs))
        if speed_abs <= 1e-4:
            return 0.0

        if not self.profile_enabled():
            return min(float(self.cfg.max_command_duty), float(self.cfg.u_vel_ff) * speed_abs)

        profile = self._profile
        if not profile:
            return 0.0

        first_s, first_d = profile[0]
        if speed_abs <= first_s:
            return min(float(self.cfg.max_command_duty), first_d)

        prev_s, prev_d = first_s, first_d
        for cur_s, cur_d in profile[1:]:
            if speed_abs <= cur_s:
                span = max(1e-6, cur_s - prev_s)
                frac = (speed_abs - prev_s) / span
                duty = prev_d + frac * (cur_d - prev_d)
                return min(float(self.cfg.max_command_duty), max(0.0, duty))
            prev_s, prev_d = cur_s, cur_d

        return min(float(self.cfg.max_command_duty), profile[-1][1])

    def feedforward(self, *, v_cmd: float, v_meas: float, allow_static: bool) -> float:
        if abs(v_cmd) <= 1e-4:
            return 0.0

        sign = 1.0 if v_cmd > 0.0 else -1.0
        if self.profile_enabled():
            return sign * self.feedforward_abs(abs(v_cmd))

        same_direction = (v_cmd >= 0.0 and v_meas >= 0.0) or (v_cmd <= 0.0 and v_meas <= 0.0)
        overspeed_same_direction = same_direction and abs(v_meas) > abs(v_cmd) + self.cfg.decel_feedforward_disable_margin_mps
        if overspeed_same_direction:
            return 0.0

        u_ff = self.cfg.u_vel_ff * v_cmd
        if allow_static and abs(v_meas) < 0.08:
            u_ff += self.cfg.u_static * sign
        return u_ff

    def compute(self, *, v_cmd: float, v_meas: float, dt: float,
                allow_static: bool, allow_integrate: bool, include_feedforward: bool = True) -> float:
        err = v_cmd - v_meas
        profile_trim = bool(include_feedforward and self.profile_enabled())
        kp = float(getattr(self.cfg, "speed_ff_trim_kp", self.cfg.kp)) if profile_trim else float(self.cfg.kp)
        ki = float(getattr(self.cfg, "speed_ff_trim_ki", self.cfg.ki)) if profile_trim else float(self.cfg.ki)
        integrator_limit = (
            float(getattr(self.cfg, "speed_ff_trim_integrator_limit", self.cfg.integrator_limit))
            if profile_trim else float(self.cfg.integrator_limit)
        )

        if allow_integrate:
            self.integrator += err * dt * ki
            self.integrator = max(-integrator_limit, min(integrator_limit, self.integrator))

        u_pi = kp * err + self.integrator
        if profile_trim:
            corr_clamp = max(0.0, float(getattr(self.cfg, "speed_ff_correction_clamp", 0.0)))
            u_pi = max(-corr_clamp, min(corr_clamp, u_pi))

        u_ff = self.feedforward(v_cmd=v_cmd, v_meas=v_meas, allow_static=allow_static) if include_feedforward else 0.0
        u = u_pi + u_ff
        self.last_error_mps = float(err)
        self.last_ff_duty = float(u_ff)
        self.last_pi_trim_duty = float(u_pi)
        return max(-self.cfg.max_command_duty, min(self.cfg.max_command_duty, u))


# =========================
# DriveBase
# =========================

class DriveBaseV2:
    def __init__(self, cfg: Optional[DriveConfig] = None):
        self.cfg = cfg if cfg is not None else DriveConfig()

        self._timing_lock = threading.Lock()
        self._timing_events = deque(maxlen=max(1, int(getattr(self.cfg, "timing_ring_size", 160))))
        self._timing_slow_threshold_ms = max(0.0, float(getattr(self.cfg, "timing_slow_threshold_ms", 10.0)))
        self._internal_loop_ms = ""
        self._internal_loop_overrun_ms = ""
        self._internal_apply_output_ms = ""
        self._internal_loop_t = 0.0
        configure_pwm_timing = getattr(hpwm, "configure_timing", None)
        if callable(configure_pwm_timing):
            configure_pwm_timing(
                enabled=bool(getattr(self.cfg, "timing_detail_enable", False)),
                slow_threshold_ms=self._timing_slow_threshold_ms,
                ring_size=max(1, int(getattr(self.cfg, "timing_ring_size", 160))),
            )
        configure_pwm_writes = getattr(hpwm, "configure_write_optimization", None)
        if callable(configure_pwm_writes):
            configure_pwm_writes(
                exact_dedupe=bool(getattr(self.cfg, "pwm_exact_dedupe_enable", True)),
                cache_fds=bool(getattr(self.cfg, "pwm_cache_fds", False)),
            )

        hpwm.setup_pwm_servo()
        hpwm.setup_pwm_led()
        hpwm.setup_pwm_drive()
        hpwm.coast()

        self._rear_led = None
        self._red_led = None
        self._green_led = None
        self._mirror_leds_enabled = True
        self._led_hw_warning_printed = False
        if PWMLED is not None:
            try:
                self._rear_led = PWMLED(REAR_LED_PIN, frequency=LED_PWM_FREQ)
                self._red_led = PWMLED(RED_LED_PIN, frequency=LED_PWM_FREQ)
                self._green_led = PWMLED(GREEN_LED_PIN, frequency=LED_PWM_FREQ)
            except Exception as exc:
                self._rear_led = None
                self._red_led = None
                self._green_led = None
                self._led_hw_warning_printed = True
                print(f"DriveBase LED GPIO setup failed; front hpwm LED only: {exc}", file=sys.stderr)
        else:
            self._led_hw_warning_printed = True
            print("DriveBase LED GPIO setup unavailable; gpiozero is not installed", file=sys.stderr)

        self.decoder = QuadratureDecoder()
        self.reader = GPIODQuadratureReader(self.cfg, self.decoder)
        self.estimator = RobustVelocityEstimator(self.cfg, self.decoder)
        self.ctrl = VelocityController(self.cfg)

        self._cmd_lock = threading.RLock()
        self._feedback_lock = threading.Lock()
        self._actuator_lock = threading.Lock()
        self._actuator_cv = threading.Condition(self._actuator_lock)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._actuator_thread: Optional[threading.Thread] = None
        self._actuator_writer_running = False
        self._actuator_command_stale_s = 0.10

        self._mode = "IDLE"
        self._v_cmd_user = 0.0
        self._v_cmd_ramped = 0.0
        self._brake_level = 0.0
        self._brake_order = "reverse_first"

        self._last_u_sent: Optional[float] = None
        self._last_brake_sent: Optional[float] = None
        self._last_brake_order_sent: Optional[str] = None
        self._last_output_t = monotonic()
        self._actuating: bool = False

        self._motion_desired_mode = "coast"
        self._motion_desired_duty = 0.0
        self._motion_desired_brake_level = 0.0
        self._motion_desired_brake_order = "reverse_first"
        self._motion_desired_seq = 0
        self._motion_desired_t = 0.0
        self._motion_applied_mode = "coast"
        self._motion_applied_duty = 0.0
        self._motion_applied_brake_level = 0.0
        self._motion_applied_brake_order = "reverse_first"
        self._motion_applied_seq = 0
        self._motion_apply_start_t = 0.0
        self._motion_apply_t = 0.0
        self._motion_apply_duration_ms = ""
        self._motion_apply_pwm_ms = ""
        self._motion_apply_age_ms = ""
        self._motion_last_error = ""

        self._steering_desired_norm = 0.0
        self._steering_desired_seq = 0
        self._steering_desired_t = 0.0
        self._steering_applied_norm: Optional[float] = None
        self._steering_applied_seq = 0
        self._steering_apply_start_t = 0.0
        self._steering_apply_t = 0.0
        self._steering_apply_duration_ms = ""
        self._steering_apply_pwm_ms = ""
        self._steering_apply_age_ms = ""
        self._steering_last_error = ""

        self._front_led_desired_level = 0.0
        self._rear_led_desired_level = 0.0
        self._front_led_desired_seq = 0
        self._rear_led_desired_seq = 0
        self._front_led_desired_t = 0.0
        self._rear_led_desired_t = 0.0
        self._front_led_applied_level = 0.0
        self._rear_led_applied_level = 0.0
        self._front_led_applied_seq = 0
        self._rear_led_applied_seq = 0
        self._led_apply_start_t = 0.0
        self._led_apply_t = 0.0
        self._led_apply_duration_ms = ""
        self._led_apply_pwm_ms = ""
        self._led_apply_age_ms = ""
        self._led_last_error = ""

        self.pos_m = 0.0
        self.v_mps = 0.0
        self.v_rate_mps = 0.0
        self.count = 0
        self._external_velocity_mps: Optional[float] = None
        self._external_velocity_filtered_mps: Optional[float] = None
        self._external_velocity_t = 0.0
        self._external_velocity_source = ""
        self._feedback_mps = 0.0
        self._feedback_source = "encoder"
        self._feedback_age_ms: Optional[float] = None

        self._dbg_last_t = 0.0
        self._dbg_period = 1.0 / max(1.0, self.cfg.debug_hz)

        # breakaway / regulate supervisor
        self._submode = "NONE"   # NONE | BREAK | REG
        self._break_t0 = 0.0
        self._break_count0 = 0
        self._break_u = 0.0
        self._u_hold = 0.0
        self._last_progress_t = monotonic()
        self._last_count_change_t = self._last_progress_t
        self._last_real_motion_t = self._last_progress_t
        self._target_changed = False
        self._breakaway_allowed = True
        self._direction_guard_clamped = False
        self._raw_duty_before_direction_guard: Optional[float] = None
        self._runtime_max_command_duty = float(self.cfg.max_command_duty)

    def _timing_enabled(self) -> bool:
        return bool(getattr(self.cfg, "timing_detail_enable", False))

    def _record_timing_event(self, operation: str, *, duration_ms: Optional[float] = None,
                             lock_wait_ms: float = 0.0, lock_inside_ms: float = 0.0,
                             pwm_write_ms: float = 0.0, extra: Optional[dict] = None) -> None:
        duration = float(duration_ms if duration_ms is not None else lock_wait_ms + lock_inside_ms)
        threshold = self._timing_slow_threshold_ms
        slow = max(duration, float(lock_wait_ms), float(lock_inside_ms), float(pwm_write_ms)) >= threshold
        if not self._timing_enabled() and not slow:
            return
        event = {
            "t": monotonic(),
            "category": "drivebase",
            "thread": threading.current_thread().name,
            "operation": str(operation),
            "duration_ms": duration,
            "lock_wait_ms": float(lock_wait_ms),
            "lock_inside_ms": float(lock_inside_ms),
            "pwm_write_ms": float(pwm_write_ms),
            "slow": bool(slow),
        }
        if extra:
            event.update(extra)
        with self._timing_lock:
            self._timing_events.append(event)
        if str(operation) == "_apply_output":
            self._internal_apply_output_ms = f"{duration:.3f}"

    @contextmanager
    def _timed_cmd_lock(self, operation: str, *, extra: Optional[dict] = None):
        wait_start = monotonic()
        self._cmd_lock.acquire()
        acquired = monotonic()
        timing = {"pwm_write_ms": 0.0, "extra": dict(extra or {})}
        try:
            yield timing
        finally:
            inside_end = monotonic()
            self._cmd_lock.release()
            event_extra = timing.get("extra") if isinstance(timing, dict) else extra
            self._record_timing_event(
                operation,
                lock_wait_ms=(acquired - wait_start) * 1000.0,
                lock_inside_ms=(inside_end - acquired) * 1000.0,
                pwm_write_ms=float(timing.get("pwm_write_ms", 0.0) or 0.0),
                extra=event_extra if isinstance(event_extra, dict) else None,
            )

    @contextmanager
    def _timed_feedback_lock(self, operation: str, *, extra: Optional[dict] = None):
        wait_start = monotonic()
        self._feedback_lock.acquire()
        acquired = monotonic()
        try:
            yield
        finally:
            inside_end = monotonic()
            self._feedback_lock.release()
            self._record_timing_event(
                operation,
                lock_wait_ms=(acquired - wait_start) * 1000.0,
                lock_inside_ms=(inside_end - acquired) * 1000.0,
                extra=extra,
            )

    @staticmethod
    def _add_pwm_timing(timing: dict, start_t: float) -> None:
        if isinstance(timing, dict):
            timing["pwm_write_ms"] = float(timing.get("pwm_write_ms", 0.0) or 0.0) + (monotonic() - start_t) * 1000.0

    def _time_pwm_call(self, timing: dict, func, *args, **kwargs):
        start_t = monotonic()
        try:
            return func(*args, **kwargs)
        finally:
            self._add_pwm_timing(timing, start_t)

    def _record_internal_loop_timing(self, loop_start: float) -> None:
        loop_ms = (monotonic() - loop_start) * 1000.0
        target_ms = 1000.0 / max(1e-6, float(self.cfg.control_hz))
        overrun_ms = max(0.0, loop_ms - target_ms)
        self._internal_loop_ms = f"{loop_ms:.3f}"
        self._internal_loop_overrun_ms = f"{overrun_ms:.3f}"
        self._internal_loop_t = monotonic()
        if self._timing_enabled() or loop_ms >= self._timing_slow_threshold_ms or overrun_ms > 0.0:
            self._record_timing_event(
                "internal_loop",
                duration_ms=loop_ms,
                extra={"overrun_ms": overrun_ms, "target_period_ms": target_ms},
            )

    def _actuator_writer_active_locked(self) -> bool:
        thread = self._actuator_thread
        return bool(self._actuator_writer_running and thread is not None and thread.is_alive())

    def _start_actuator_writer(self) -> None:
        with self._actuator_cv:
            if self._actuator_writer_running:
                return
            self._actuator_writer_running = True
            self._motion_last_error = ""
            self._steering_last_error = ""
            self._led_last_error = ""
            thread = threading.Thread(target=self._actuator_writer_loop, name="DriveBaseActuatorWriter", daemon=True)
            self._actuator_thread = thread
        thread.start()

    def _stop_actuator_writer(self, timeout_s: float = 0.25) -> None:
        with self._actuator_cv:
            thread = self._actuator_thread
            self._actuator_writer_running = False
            self._actuator_cv.notify_all()
        if thread is not None:
            thread.join(timeout=max(0.0, float(timeout_s)))
        with self._actuator_cv:
            if self._actuator_thread is thread and (thread is None or not thread.is_alive()):
                self._actuator_thread = None

    def _publish_motion(self, mode: str, *, duty: float = 0.0, brake_level: float = 0.0, brake_order: str = "reverse_first") -> int:
        safe_mode = str(mode)
        if safe_mode not in ("drive", "brake", "coast"):
            safe_mode = "coast"
        safe_order = "forward_first" if str(brake_order) == "forward_first" else "reverse_first"
        safe_duty = max(-float(self.cfg.max_command_duty), min(float(self.cfg.max_command_duty), float(duty)))
        safe_brake = max(0.0, min(1.0, float(brake_level)))
        now = monotonic()
        with self._actuator_cv:
            self._motion_desired_mode = safe_mode
            self._motion_desired_duty = safe_duty
            self._motion_desired_brake_level = safe_brake
            self._motion_desired_brake_order = safe_order
            self._motion_desired_t = now
            self._motion_desired_seq += 1
            seq = int(self._motion_desired_seq)
            self._actuator_cv.notify_all()
            return seq

    def _publish_steering(self, target: float) -> int:
        now = monotonic()
        with self._actuator_cv:
            self._steering_desired_norm = float(target)
            self._steering_desired_t = now
            self._steering_desired_seq += 1
            seq = int(self._steering_desired_seq)
            self._actuator_cv.notify_all()
            return seq

    def _publish_front_led(self, level: float) -> int:
        now = monotonic()
        with self._actuator_cv:
            self._front_led_desired_level = max(0.0, min(1.0, float(level)))
            self._front_led_desired_t = now
            self._front_led_desired_seq += 1
            seq = int(self._front_led_desired_seq)
            self._actuator_cv.notify_all()
            return seq

    def _publish_rear_led(self, level: float) -> int:
        now = monotonic()
        with self._actuator_cv:
            self._rear_led_desired_level = max(0.0, min(1.0, float(level)))
            self._rear_led_desired_t = now
            self._rear_led_desired_seq += 1
            seq = int(self._rear_led_desired_seq)
            self._actuator_cv.notify_all()
            return seq

    def _publish_led_mirror_state(self, enabled: bool) -> None:
        now = monotonic()
        with self._actuator_cv:
            self._mirror_leds_enabled = bool(enabled)
            self._front_led_desired_t = now
            self._rear_led_desired_t = now
            self._front_led_desired_seq += 1
            self._rear_led_desired_seq += 1
            self._actuator_cv.notify_all()

    def _wait_for_motion_seq(self, seq: int, timeout_s: float = 0.10) -> None:
        deadline = monotonic() + max(0.0, float(timeout_s))
        with self._actuator_cv:
            while self._motion_applied_seq < int(seq) and self._actuator_writer_running:
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    break
                self._actuator_cv.wait(timeout=remaining)

    def _wait_for_steering_seq(self, seq: int, timeout_s: float = 0.08) -> None:
        deadline = monotonic() + max(0.0, float(timeout_s))
        with self._actuator_cv:
            while self._steering_applied_seq < int(seq) and self._actuator_writer_running:
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    break
                self._actuator_cv.wait(timeout=remaining)

    @staticmethod
    def _apply_motion_to_hardware(mode: str, duty: float, brake_level: float, brake_order: str) -> None:
        if mode == "drive":
            hpwm.set_drive(float(duty))
        elif mode == "brake":
            hpwm.brake(float(brake_level), order=brake_order)
        else:
            hpwm.coast()

    def _apply_led_to_hardware(self, front_pending: bool, front_level: float, rear_pending: bool, rear_level: float, mirror_enabled: bool) -> None:
        if front_pending:
            hpwm.set_led(front_level)
            self._set_gpio_led(self._green_led, front_level if mirror_enabled else 0.0)
        if rear_pending:
            self._set_gpio_led(self._rear_led, rear_level)
            self._set_gpio_led(self._red_led, rear_level if mirror_enabled else 0.0)

    def _actuator_writer_loop(self) -> None:
        last_job = ""
        while True:
            stale_event = None
            with self._actuator_cv:
                while self._actuator_writer_running:
                    motion_pending = self._motion_desired_seq > self._motion_applied_seq
                    steering_pending = self._steering_desired_seq > self._steering_applied_seq
                    led_pending = (
                        self._front_led_desired_seq > self._front_led_applied_seq
                        or self._rear_led_desired_seq > self._rear_led_applied_seq
                    )
                    if motion_pending and not (last_job == "motion" and steering_pending and self._motion_desired_mode == "drive"):
                        now = monotonic()
                        mode = str(self._motion_desired_mode)
                        if (
                            mode == "drive"
                            and self._motion_desired_t > 0.0
                            and now - float(self._motion_desired_t) > self._actuator_command_stale_s
                        ):
                            stale_event = {
                                "stale_age_ms": (now - float(self._motion_desired_t)) * 1000.0,
                                "stale_mode": mode,
                                "stale_duty": float(self._motion_desired_duty),
                            }
                            mode = "brake"
                            self._motion_desired_mode = "brake"
                            self._motion_desired_duty = 0.0
                            self._motion_desired_brake_level = 1.0
                            self._motion_desired_t = now
                            self._motion_desired_seq += 1
                        job = (
                            "motion",
                            int(self._motion_desired_seq),
                            mode,
                            float(self._motion_desired_duty),
                            float(self._motion_desired_brake_level),
                            str(self._motion_desired_brake_order),
                            float(self._motion_desired_t or 0.0),
                        )
                        break
                    if steering_pending:
                        job = (
                            "steering",
                            int(self._steering_desired_seq),
                            float(self._steering_desired_norm),
                            float(self._steering_desired_t or 0.0),
                        )
                        break
                    if motion_pending:
                        job = (
                            "motion",
                            int(self._motion_desired_seq),
                            str(self._motion_desired_mode),
                            float(self._motion_desired_duty),
                            float(self._motion_desired_brake_level),
                            str(self._motion_desired_brake_order),
                            float(self._motion_desired_t or 0.0),
                        )
                        break
                    if led_pending:
                        front_pending = self._front_led_desired_seq > self._front_led_applied_seq
                        rear_pending = self._rear_led_desired_seq > self._rear_led_applied_seq
                        command_t_values = []
                        if front_pending and self._front_led_desired_t > 0.0:
                            command_t_values.append(float(self._front_led_desired_t))
                        if rear_pending and self._rear_led_desired_t > 0.0:
                            command_t_values.append(float(self._rear_led_desired_t))
                        command_t = min(command_t_values) if command_t_values else 0.0
                        job = (
                            "led",
                            bool(front_pending),
                            int(self._front_led_desired_seq),
                            float(self._front_led_desired_level),
                            bool(rear_pending),
                            int(self._rear_led_desired_seq),
                            float(self._rear_led_desired_level),
                            bool(self._mirror_leds_enabled),
                            command_t,
                        )
                        break
                    self._actuator_cv.wait(timeout=0.05)
                else:
                    continue
                if not self._actuator_writer_running:
                    break

            if stale_event is not None:
                self._record_timing_event(
                    "actuator_stale_motion",
                    duration_ms=float(stale_event.get("stale_age_ms", 0.0) or 0.0),
                    extra=stale_event,
                )

            kind = job[0]
            if kind == "motion":
                _, seq, mode, duty, brake_level, brake_order, command_t = job
                apply_start = monotonic()
                error = ""
                try:
                    self._apply_motion_to_hardware(mode, duty, brake_level, brake_order)
                except Exception as exc:
                    error = repr(exc)
                apply_end = monotonic()
                duration_ms = (apply_end - apply_start) * 1000.0
                age_start_ms = max(0.0, (apply_start - command_t) * 1000.0) if command_t > 0.0 else 0.0
                age_end_ms = max(0.0, (apply_end - command_t) * 1000.0) if command_t > 0.0 else 0.0
                with self._actuator_cv:
                    if seq >= self._motion_applied_seq:
                        self._motion_applied_seq = seq
                        if not error:
                            self._motion_applied_mode = mode
                            self._motion_applied_duty = duty if mode == "drive" else 0.0
                            self._motion_applied_brake_level = brake_level if mode == "brake" else 0.0
                            self._motion_applied_brake_order = brake_order
                    self._motion_apply_start_t = apply_start
                    self._motion_apply_t = apply_end
                    self._motion_apply_duration_ms = f"{duration_ms:.3f}"
                    self._motion_apply_pwm_ms = f"{duration_ms:.3f}"
                    self._motion_apply_age_ms = f"{age_end_ms:.3f}"
                    self._motion_last_error = error
                    self._actuator_cv.notify_all()
                extra = {
                    "motion_async": True,
                    "motion_mode": mode,
                    "motion_duty": duty,
                    "motion_brake_level": brake_level,
                    "motion_seq": seq,
                    "motion_command_age_start_ms": age_start_ms,
                    "motion_command_age_end_ms": age_end_ms,
                }
                if error:
                    extra["error"] = error[:120]
                self._record_timing_event("motion_apply", duration_ms=duration_ms, pwm_write_ms=duration_ms, extra=extra)
                last_job = "motion"
                continue

            if kind == "steering":
                _, seq, target, command_t = job
                apply_start = monotonic()
                error = ""
                try:
                    hpwm.set_steering_angle(target)
                except Exception as exc:
                    error = repr(exc)
                apply_end = monotonic()
                duration_ms = (apply_end - apply_start) * 1000.0
                age_start_ms = max(0.0, (apply_start - command_t) * 1000.0) if command_t > 0.0 else 0.0
                age_end_ms = max(0.0, (apply_end - command_t) * 1000.0) if command_t > 0.0 else 0.0
                with self._actuator_cv:
                    if seq >= self._steering_applied_seq:
                        self._steering_applied_seq = seq
                        if not error:
                            self._steering_applied_norm = target
                    self._steering_apply_start_t = apply_start
                    self._steering_apply_t = apply_end
                    self._steering_apply_duration_ms = f"{duration_ms:.3f}"
                    self._steering_apply_pwm_ms = f"{duration_ms:.3f}"
                    self._steering_apply_age_ms = f"{age_end_ms:.3f}"
                    self._steering_last_error = error
                    self._actuator_cv.notify_all()
                extra = {
                    "steering_async": True,
                    "steering_target_norm": target,
                    "steering_seq": seq,
                    "steering_command_age_start_ms": age_start_ms,
                    "steering_command_age_end_ms": age_end_ms,
                }
                if error:
                    extra["error"] = error[:120]
                self._record_timing_event("steering_apply", duration_ms=duration_ms, pwm_write_ms=duration_ms, extra=extra)
                last_job = "steering"
                continue

            _, front_pending, front_seq, front_level, rear_pending, rear_seq, rear_level, mirror_enabled, command_t = job
            apply_start = monotonic()
            error = ""
            try:
                self._apply_led_to_hardware(front_pending, front_level, rear_pending, rear_level, mirror_enabled)
            except Exception as exc:
                error = repr(exc)
            apply_end = monotonic()
            duration_ms = (apply_end - apply_start) * 1000.0
            age_start_ms = max(0.0, (apply_start - command_t) * 1000.0) if command_t > 0.0 else 0.0
            age_end_ms = max(0.0, (apply_end - command_t) * 1000.0) if command_t > 0.0 else 0.0
            with self._actuator_cv:
                if front_pending and front_seq >= self._front_led_applied_seq:
                    self._front_led_applied_seq = front_seq
                    if not error:
                        self._front_led_applied_level = front_level
                if rear_pending and rear_seq >= self._rear_led_applied_seq:
                    self._rear_led_applied_seq = rear_seq
                    if not error:
                        self._rear_led_applied_level = rear_level
                self._led_apply_start_t = apply_start
                self._led_apply_t = apply_end
                self._led_apply_duration_ms = f"{duration_ms:.3f}"
                self._led_apply_pwm_ms = f"{duration_ms:.3f}"
                self._led_apply_age_ms = f"{age_end_ms:.3f}"
                self._led_last_error = error
                self._actuator_cv.notify_all()
            extra = {
                "led_async": True,
                "front_pending": bool(front_pending),
                "rear_pending": bool(rear_pending),
                "front_level": front_level,
                "rear_level": rear_level,
                "led_command_age_start_ms": age_start_ms,
                "led_command_age_end_ms": age_end_ms,
            }
            if error:
                extra["error"] = error[:120]
            self._record_timing_event("led_apply", duration_ms=duration_ms, pwm_write_ms=duration_ms, extra=extra)
            last_job = "led"

    def drain_timing_events(self) -> list:
        with self._timing_lock:
            events = list(self._timing_events)
            self._timing_events.clear()
        drain_pwm = getattr(hpwm, "drain_timing_events", None)
        if callable(drain_pwm):
            try:
                events.extend(drain_pwm())
            except Exception:
                pass
        return events

    def _reset_velocity_state_locked(self, *, mode: Optional[str] = None, brake_level: Optional[float] = None) -> None:
        if mode is not None:
            self._mode = mode
        self._v_cmd_user = 0.0
        self._v_cmd_ramped = 0.0
        if brake_level is not None:
            self._brake_level = max(0.0, min(1.0, float(brake_level)))
        self.ctrl.reset()
        self._submode = "NONE"
        self._break_t0 = 0.0
        self._break_count0 = self.count
        self._break_u = 0.0
        self._u_hold = 0.0
        self._last_progress_t = monotonic()
        self._last_count_change_t = self._last_progress_t
        self._last_real_motion_t = self._last_progress_t
        self._target_changed = False
        self._last_u_sent = None
        self._last_brake_sent = None
        self._actuating = False
        self._breakaway_allowed = True
        self._direction_guard_clamped = False
        self._raw_duty_before_direction_guard = None
        with self._feedback_lock:
            self._feedback_mps = float(self.v_mps)
            self._feedback_source = "encoder"
            self._feedback_age_ms = None

    def set_external_velocity_feedback(self, v_mps: Optional[float], sample_t: Optional[float] = None, source: str = "external") -> None:
        if v_mps is None:
            with self._timed_feedback_lock("set_external_velocity_feedback"):
                self._external_velocity_mps = None
                self._external_velocity_filtered_mps = None
                self._external_velocity_t = 0.0
                self._external_velocity_source = ""
            return

        try:
            v = float(v_mps)
        except (TypeError, ValueError):
            return
        if not math.isfinite(v):
            return

        now = monotonic()
        try:
            t = float(sample_t) if sample_t is not None else now
        except (TypeError, ValueError):
            t = now
        if not math.isfinite(t) or t <= 0.0:
            t = now

        alpha = max(0.0, min(1.0, float(getattr(self.cfg, "external_velocity_feedback_filter_alpha", 0.35))))
        max_age_s = max(0.0, float(getattr(self.cfg, "external_velocity_feedback_max_age_s", 0.08)))
        with self._timed_feedback_lock("set_external_velocity_feedback"):
            prev = self._external_velocity_filtered_mps
            prev_t = self._external_velocity_t
            if prev is None or prev_t <= 0.0 or (now - prev_t) > max(0.2, 2.0 * max_age_s):
                filtered = v
            else:
                filtered = (1.0 - alpha) * float(prev) + alpha * v
            self._external_velocity_mps = v
            self._external_velocity_filtered_mps = filtered
            self._external_velocity_t = t
            self._external_velocity_source = str(source or "external")

    def _select_velocity_feedback(self, *, v_cmd: float, creep: bool, now: float) -> Tuple[float, str, Optional[float]]:
        feedback = float(self.v_mps)
        source = "encoder"
        age_ms: Optional[float] = None
        if (
            bool(getattr(self.cfg, "external_velocity_feedback_enable", True))
            and (not creep)
            and abs(float(v_cmd)) >= max(0.0, float(getattr(self.cfg, "external_velocity_feedback_min_speed_mps", 0.20)))
        ):
            with self._timed_feedback_lock("select_velocity_feedback_read"):
                ext = self._external_velocity_filtered_mps
                ext_t = float(self._external_velocity_t)
                ext_source = self._external_velocity_source
            max_age_s = max(0.0, float(getattr(self.cfg, "external_velocity_feedback_max_age_s", 0.08)))
            if ext is not None and ext_t > 0.0:
                age_s = max(0.0, now - ext_t)
                if age_s <= max_age_s:
                    feedback = float(ext)
                    source = ext_source or "external"
                    age_ms = age_s * 1000.0

        with self._timed_feedback_lock("select_velocity_feedback_write"):
            self._feedback_mps = feedback
            self._feedback_source = source
            self._feedback_age_ms = age_ms
        return feedback, source, age_ms

    # ---------- lifecycle ----------
    def start(self) -> None:
        with self._cmd_lock:
            if self._running:
                return
            self._running = True
            self._pwm_readback_running = True
            self.reader.start()
            self.estimator.reset()
            self._reset_velocity_state_locked(mode="IDLE", brake_level=0.0)
            self._last_output_t = monotonic()
            hpwm.coast()
        self._start_actuator_writer()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        if float(getattr(self.cfg, "pwm_readback_hz", 5.0)) > 0.0:
            self._pwm_readback_thread = threading.Thread(target=self._pwm_readback_loop, daemon=True)
            self._pwm_readback_thread.start()

    def stop(self, brake: bool = True) -> None:
        seq = 0
        with self._cmd_lock:
            if brake:
                self._reset_velocity_state_locked(mode="BRAKE", brake_level=1.0)
                seq = self._publish_motion("brake", brake_level=self._brake_level, brake_order=self._brake_order)
                self._last_brake_sent = self._brake_level
                self._last_brake_order_sent = self._brake_order
            else:
                self._reset_velocity_state_locked(mode="IDLE", brake_level=0.0)
                seq = self._publish_motion("coast")
                self._last_u_sent = None
                self._last_brake_sent = None
                self._last_brake_order_sent = None
        if seq:
            self._wait_for_motion_seq(seq, timeout_s=0.15)
        self._running = False
        self._pwm_readback_running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._pwm_readback_thread:
            self._pwm_readback_thread.join(timeout=1.0)
            self._pwm_readback_thread = None
        self.reader.stop()
        self._stop_actuator_writer()
        with self._cmd_lock:
            if brake:
                hpwm.brake(1.0)
            else:
                hpwm.coast()

    def shutdown(self, disable_drive_pwm: bool = False) -> None:
        try:
            self.stop(brake=False)
        finally:
            try:
                for _ in range(5):
                    hpwm.coast()
                    time.sleep(0.04)
                hpwm.set_led(0.0)
                self._set_gpio_led(self._rear_led, 0.0)
                self._set_gpio_led(self._red_led, 0.0)
                self._set_gpio_led(self._green_led, 0.0)
                self._close_gpio_led(self._rear_led)
                self._close_gpio_led(self._red_led)
                self._close_gpio_led(self._green_led)
                self._rear_led = None
                self._red_led = None
                self._green_led = None
                hpwm.cleanup_pwm_servo()
                paths = (hpwm.PWM1_PATH,)
                if disable_drive_pwm:
                    paths = (hpwm.PWM1_PATH, hpwm.PWM2_PATH, hpwm.PWM3_PATH)
                for path in paths:
                    try:
                        hpwm.write(f"{path}/enable", 0)
                    except Exception:
                        pass
                close_pwm_fds = getattr(hpwm, "close_cached_fds", None)
                if callable(close_pwm_fds):
                    close_pwm_fds()
            except Exception:
                pass

    # ---------- commands ----------
    def set_steering(self, normalized: float, wait: bool = False, timeout_s: float = 0.08) -> None:
        submit_start = monotonic()
        target = max(-1.0, min(1.0, float(normalized) + float(self.cfg.steering_center_trim)))
        with self._actuator_cv:
            use_writer = self._actuator_writer_active_locked()
        if use_writer:
            seq = self._publish_steering(target)
            if wait:
                self._wait_for_steering_seq(seq, timeout_s=timeout_s)
            self._record_timing_event(
                "set_steering",
                duration_ms=(monotonic() - submit_start) * 1000.0,
                extra={"steering_async": True, "steering_wait": bool(wait), "steering_target_norm": target, "steering_seq": seq},
            )
            return

        with self._timed_cmd_lock("set_steering") as timing:
            self._time_pwm_call(timing, hpwm.set_steering_angle, target)
        apply_end = monotonic()
        duration_ms = (apply_end - submit_start) * 1000.0
        pwm_write_ms = float(timing.get("pwm_write_ms", 0.0) or 0.0)
        with self._actuator_cv:
            self._steering_desired_norm = target
            self._steering_desired_t = submit_start
            self._steering_desired_seq += 1
            seq = int(self._steering_desired_seq)
            self._steering_applied_seq = seq
            self._steering_applied_norm = target
            self._steering_apply_start_t = submit_start
            self._steering_apply_t = apply_end
            self._steering_apply_duration_ms = f"{duration_ms:.3f}"
            self._steering_apply_pwm_ms = f"{pwm_write_ms:.3f}"
            self._steering_apply_age_ms = f"{duration_ms:.3f}"
            self._steering_last_error = ""
            self._actuator_cv.notify_all()

    def set_drive_power(self, throttle: float) -> None:
        with self._timed_cmd_lock("set_drive_power"):
            self._reset_velocity_state_locked(mode="IDLE", brake_level=0.0)
            throttle = max(-float(self.cfg.max_command_duty), min(float(self.cfg.max_command_duty), float(throttle)))
            self._publish_motion("drive" if abs(throttle) > 1e-6 else "coast", duty=throttle)
            self._last_u_sent = throttle if abs(throttle) > 1e-6 else None
            self._last_brake_sent = None
            self._last_brake_order_sent = None
            self._actuating = abs(throttle) > 1e-6

    def brake(self, level: float, order: str = "reverse_first") -> None:
        brake_order = "forward_first" if str(order) == "forward_first" else "reverse_first"
        with self._timed_cmd_lock("brake", extra={"brake_order": brake_order}):
            self._reset_velocity_state_locked(mode="BRAKE", brake_level=level)
            self._brake_order = brake_order
            self._publish_motion("brake", brake_level=self._brake_level, brake_order=brake_order)
            self._last_u_sent = None
            self._last_brake_sent = self._brake_level
            self._last_brake_order_sent = brake_order
            self._actuating = False

    def neutral(self) -> None:
        with self._timed_cmd_lock("neutral"):
            self._reset_velocity_state_locked(mode="IDLE", brake_level=0.0)
            self._publish_motion("coast")
            self._last_u_sent = None
            self._last_brake_sent = None
            self._last_brake_order_sent = None
            self._actuating = False

    def set_front_led(self, level: float) -> None:
        with self._timed_cmd_lock("set_front_led"):
            self._publish_front_led(level)

    def set_rear_led(self, level: float) -> None:
        with self._timed_cmd_lock("set_rear_led"):
            self._publish_rear_led(level)

    def set_mirror_leds_enabled(self, enabled: bool) -> None:
        with self._timed_cmd_lock("set_mirror_leds_enabled"):
            self._publish_led_mirror_state(bool(enabled))

    @staticmethod
    def _set_gpio_led(led, level: float) -> None:
        if led is None:
            return
        try:
            led.value = max(0.0, min(1.0, float(level)))
        except Exception:
            pass

    @staticmethod
    def _close_gpio_led(led) -> None:
        if led is None:
            return
        try:
            led.close()
        except Exception:
            pass

    def set_target_speed(self, v_mps: float, allow_breakaway: bool = True) -> None:
        with self._timed_cmd_lock("set_target_speed"):
            prev = self._v_cmd_user
            if abs(float(v_mps)) < 0.005:
                self._reset_velocity_state_locked(mode="BRAKE", brake_level=1.0)
                self._publish_motion("brake", brake_level=self._brake_level, brake_order=self._brake_order)
                self._last_u_sent = None
                self._last_brake_sent = self._brake_level
                self._last_brake_order_sent = self._brake_order
                self._actuating = False
                return
            prev_mode = self._mode
            prev_submode = self._submode
            if prev_mode != "VEL":
                self.ctrl.reset()
                self._submode = "NONE"
                self._break_u = 0.0
                self._u_hold = 0.0
                self._last_u_sent = None
                self._last_brake_sent = None
                self._last_brake_order_sent = None
                self._actuating = False
            self._mode = "VEL"
            self._v_cmd_user = float(v_mps)
            self._breakaway_allowed = bool(allow_breakaway)
            self._target_changed = abs(self._v_cmd_user - prev) > 1e-6

            if self._breakaway_active_for_speed(self._v_cmd_user):
                same_direction = prev * self._v_cmd_user > 0.0
                already_handling_creep = (
                    prev_mode == "VEL"
                    and prev_submode in ("BREAK", "REG")
                    and same_direction
                    and not self._target_changed
                )
                if abs(self.v_mps) < 0.02 and not already_handling_creep:
                    self._enter_breakaway()

    def _creep_active_for_speed(self, v_mps: float) -> bool:
        if not bool(self.cfg.creep_breakaway_enable):
            return False
        creep_limit = min(float(self.cfg.creep_threshold_mps), float(self.cfg.breakaway_max_speed_mps))
        return abs(float(v_mps)) <= max(0.0, creep_limit)

    def _breakaway_active_for_speed(self, v_mps: float) -> bool:
        return (
            bool(self.cfg.creep_breakaway_enable)
            and bool(self._breakaway_allowed)
            and bool(self.cfg.breakaway_enable)
            and abs(float(v_mps)) <= max(0.0, float(self.cfg.breakaway_max_speed_mps))
        )

    def set_runtime_max_command_duty(self, duty: Optional[float]) -> None:
        with self._timed_cmd_lock("set_runtime_max_command_duty"):
            if duty is None:
                self._runtime_max_command_duty = float(self.cfg.max_command_duty)
            else:
                self._runtime_max_command_duty = max(0.0, min(float(self.cfg.max_command_duty), float(duty)))

    def coast(self) -> None:
        with self._timed_cmd_lock("coast"):
            self._reset_velocity_state_locked(mode="IDLE", brake_level=0.0)
            self._publish_motion("coast")
            self._last_u_sent = None
            self._last_brake_sent = None
            self._last_brake_order_sent = None
            self._actuating = False

    # ---------- telemetry ----------
    def get_speed(self) -> float:
        return float(self.v_mps)

    def get_distance(self) -> float:
        return float(self.pos_m)

    def get_count(self) -> int:
        return int(self.count)

    # ---------- internals ----------
    @staticmethod
    def _slew_limit(target: float, current: float, dt: float, rate: float) -> float:
        max_step = rate * dt
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    @staticmethod
    def _slew_limit_asym(target: float, current: float, dt: float, accel_rate: float, decel_rate: float) -> float:
        reducing_magnitude = abs(target) < abs(current) and (target == 0.0 or current == 0.0 or target * current >= 0.0)
        rate = decel_rate if reducing_magnitude else accel_rate
        return DriveBaseV2._slew_limit(target, current, dt, rate)

    def _apply_output(self, u: float, slew_per_s: Optional[float] = None) -> float:
        with self._timed_cmd_lock("_apply_output"):
            if self._mode != "VEL":
                return 0.0
            now = monotonic()
            self._raw_duty_before_direction_guard = float(u)
            self._direction_guard_clamped = False
            duty_limit = max(0.0, min(float(self.cfg.max_command_duty), float(self._runtime_max_command_duty)))
            u = max(-duty_limit, min(duty_limit, u))
            target = self._v_cmd_user
            if target < -0.005 and u > 0.0:
                u = 0.0
                self._direction_guard_clamped = True
            elif target > 0.005 and u < 0.0:
                u = 0.0
                self._direction_guard_clamped = True
            if (
                slew_per_s is not None
                and slew_per_s > 0.0
                and self._last_u_sent is not None
                and abs(u) > 1e-6
                and self._last_u_sent * u > 0.0
            ):
                dt = max(0.0, min(0.10, now - self._last_output_t))
                max_step = slew_per_s * dt
                if max_step > 0.0:
                    if u > self._last_u_sent + max_step:
                        u = self._last_u_sent + max_step
                    elif u < self._last_u_sent - max_step:
                        u = self._last_u_sent - max_step
            u_abs = abs(u)

            if self._actuating:
                if u_abs < (self.cfg.u_min_actuate - self.cfg.u_min_hyst):
                    self._actuating = False
            else:
                if u_abs > (self.cfg.u_min_actuate + self.cfg.u_min_hyst):
                    self._actuating = True

            if not self._actuating:
                if self._last_u_sent is not None:
                    self._publish_motion("coast")
                self._last_u_sent = None
                self._last_brake_sent = None
                self._last_brake_order_sent = None
                self._last_output_t = now
                return 0.0

            if self._last_u_sent is not None and abs(u - self._last_u_sent) < self.cfg.duty_update_deadband:
                return float(self._last_u_sent)

            self._publish_motion("drive", duty=u)
            self._last_u_sent = u
            self._last_brake_sent = None
            self._last_brake_order_sent = None
            self._last_output_t = now
            return float(u)

    def get_telemetry(self) -> dict:
        t0 = monotonic()
        try:
            with self._feedback_lock:
                feedback_mps = self._feedback_mps
                feedback_source = self._feedback_source
                feedback_age_ms = self._feedback_age_ms
                external_velocity_mps = self._external_velocity_mps
                external_velocity_filtered_mps = self._external_velocity_filtered_mps
                external_velocity_t = self._external_velocity_t
            now = monotonic()
            with self._actuator_cv:
                motion_desired_seq = self._motion_desired_seq
                motion_applied_seq = self._motion_applied_seq
                motion_pending = motion_desired_seq > motion_applied_seq
                motion_pending_age_ms = (
                    max(0.0, (now - float(self._motion_desired_t)) * 1000.0)
                    if motion_pending and float(self._motion_desired_t or 0.0) > 0.0
                    else ""
                )
                motion_applied_mode = self._motion_applied_mode
                motion_applied_duty = self._motion_applied_duty
                motion_applied_brake_level = self._motion_applied_brake_level
                motion_apply_age_ms = self._motion_apply_age_ms
                motion_apply_duration_ms = self._motion_apply_duration_ms
                motion_apply_pwm_ms = self._motion_apply_pwm_ms
                motion_last_error = self._motion_last_error

                steering_desired_norm = self._steering_desired_norm
                steering_applied_norm = self._steering_applied_norm
                steering_desired_seq = self._steering_desired_seq
                steering_applied_seq = self._steering_applied_seq
                steering_pending = steering_desired_seq > steering_applied_seq
                steering_pending_age_ms = (
                    max(0.0, (now - float(self._steering_desired_t)) * 1000.0)
                    if steering_pending and float(self._steering_desired_t or 0.0) > 0.0
                    else ""
                )
                steering_apply_age_ms = self._steering_apply_age_ms
                steering_apply_duration_ms = self._steering_apply_duration_ms
                steering_apply_pwm_ms = self._steering_apply_pwm_ms
                steering_last_error = self._steering_last_error

                led_pending = (
                    self._front_led_desired_seq > self._front_led_applied_seq
                    or self._rear_led_desired_seq > self._rear_led_applied_seq
                )
                led_times = []
                if self._front_led_desired_seq > self._front_led_applied_seq and self._front_led_desired_t > 0.0:
                    led_times.append(float(self._front_led_desired_t))
                if self._rear_led_desired_seq > self._rear_led_applied_seq and self._rear_led_desired_t > 0.0:
                    led_times.append(float(self._rear_led_desired_t))
                led_pending_age_ms = max(0.0, (now - min(led_times)) * 1000.0) if led_times else ""
                led_apply_age_ms = self._led_apply_age_ms
                led_apply_duration_ms = self._led_apply_duration_ms
                led_apply_pwm_ms = self._led_apply_pwm_ms
                led_last_error = self._led_last_error
            pwm = hpwm.get_drive_state() if hasattr(hpwm, "get_drive_state") else {}
            pwm_stats = hpwm.get_write_stats() if hasattr(hpwm, "get_write_stats") else {}
            return {
                "mode": self._mode,
                "submode": self._submode,
                "brake_level": float(self._brake_level),
                "target_user_mps": float(self._v_cmd_user),
                "target_ramped_mps": float(self._v_cmd_ramped),
                "measured_mps": float(self.v_mps),
                "encoder_mps": float(self.v_mps),
                "feedback_mps": float(feedback_mps),
                "feedback_source": str(feedback_source),
                "feedback_age_ms": "" if feedback_age_ms is None else float(feedback_age_ms),
                "external_feedback_mps": "" if external_velocity_mps is None else float(external_velocity_mps),
                "external_feedback_filtered_mps": "" if external_velocity_filtered_mps is None else float(external_velocity_filtered_mps),
                "external_feedback_age_ms": (
                    "" if external_velocity_t <= 0.0 else max(0.0, (now - float(external_velocity_t)) * 1000.0)
                ),
                "rate_mps": float(self.v_rate_mps),
                "position_m": float(self.pos_m),
                "count": int(self.count),
                "motion_applied_mode": str(motion_applied_mode),
                "motion_applied_duty": float(motion_applied_duty),
                "motion_applied_brake_level": float(motion_applied_brake_level),
                "motion_desired_seq": int(motion_desired_seq),
                "motion_applied_seq": int(motion_applied_seq),
                "motion_pending": bool(motion_pending),
                "motion_pending_age_ms": motion_pending_age_ms,
                "motion_apply_age_ms": motion_apply_age_ms,
                "motion_apply_duration_ms": motion_apply_duration_ms,
                "motion_apply_pwm_ms": motion_apply_pwm_ms,
                "motion_last_error": motion_last_error,
                "steering_desired_norm": float(steering_desired_norm),
                "steering_applied_norm": "" if steering_applied_norm is None else float(steering_applied_norm),
                "steering_desired_seq": int(steering_desired_seq),
                "steering_applied_seq": int(steering_applied_seq),
                "steering_pending": bool(steering_pending),
                "steering_pending_age_ms": steering_pending_age_ms,
                "steering_apply_age_ms": steering_apply_age_ms,
                "steering_apply_duration_ms": steering_apply_duration_ms,
                "steering_apply_pwm_ms": steering_apply_pwm_ms,
                "steering_last_error": steering_last_error,
                "led_pending": bool(led_pending),
                "led_pending_age_ms": led_pending_age_ms,
                "led_apply_age_ms": led_apply_age_ms,
                "led_apply_duration_ms": led_apply_duration_ms,
                "led_apply_pwm_ms": led_apply_pwm_ms,
                "led_last_error": led_last_error,
                "last_duty": None if self._last_u_sent is None else float(self._last_u_sent),
                "actuating": bool(self._actuating),
                "target_direction": -1 if self._v_cmd_user < -0.005 else (1 if self._v_cmd_user > 0.005 else 0),
                "direction_guard_clamped": bool(self._direction_guard_clamped),
                "raw_duty_before_direction_guard": self._raw_duty_before_direction_guard,
                "runtime_max_command_duty": float(self._runtime_max_command_duty),
                "speed_ff_profile_enable": bool(getattr(self.cfg, "speed_ff_profile_enable", False)),
                "speed_ff_target_duty_abs": float(self.ctrl.feedforward_abs(abs(self._v_cmd_ramped))),
                "speed_ff_correction_clamp": float(getattr(self.cfg, "speed_ff_correction_clamp", 0.0)),
                "speed_ff_duty": float(getattr(self.ctrl, "last_ff_duty", 0.0)),
                "speed_pi_trim_duty": float(getattr(self.ctrl, "last_pi_trim_duty", 0.0)),
                "speed_error_mps": float(getattr(self.ctrl, "last_error_mps", 0.0)),
                "speed_ff_trim_kp": float(getattr(self.cfg, "speed_ff_trim_kp", 0.0)),
                "speed_ff_trim_ki": float(getattr(self.cfg, "speed_ff_trim_ki", 0.0)),
                "v_slew_mps2": float(self.cfg.v_slew_mps2),
                "v_decel_slew_mps2": float(self.cfg.v_decel_slew_mps2),
                "decel_ff_margin_mps": float(self.cfg.decel_feedforward_disable_margin_mps),
                "velocity_duty_slew_per_s": float(self.cfg.velocity_duty_slew_per_s),
                "creep_duty_slew_per_s": float(self.cfg.creep_duty_slew_per_s),
                "creep_breakaway_enabled": bool(self.cfg.creep_breakaway_enable),
                "breakaway_allowed": bool(self._breakaway_allowed),
                "breakaway_enabled": bool(self.cfg.breakaway_enable),
                "breakaway_active": bool(self._submode == "BREAK"),
                "breakaway_max_speed_mps": float(self.cfg.breakaway_max_speed_mps),
                "creep_threshold_mps": float(self.cfg.creep_threshold_mps),
                "internal_loop_ms": self._internal_loop_ms,
                "internal_loop_overrun_ms": self._internal_loop_overrun_ms,
                "internal_apply_output_ms": self._internal_apply_output_ms,
                "internal_loop_t": self._internal_loop_t,
                "drive_pwm_mode": str(pwm.get("mode", "")),
                "drive_pwm_reverse": pwm.get("reverse", ""),
                "drive_pwm_forward": pwm.get("forward", ""),
                "drive_pwm_req_reverse": pwm.get("req_reverse", pwm.get("reverse", "")),
                "drive_pwm_req_forward": pwm.get("req_forward", pwm.get("forward", "")),
                "drive_pwm_read_reverse": pwm.get("read_reverse", ""),
                "drive_pwm_read_forward": pwm.get("read_forward", ""),
                "drive_pwm_readback_enabled": pwm.get("readback_enabled", ""),
                "drive_pwm_throttle": pwm.get("throttle", ""),
                "drive_pwm_brake_level": pwm.get("brake_level", ""),
                "drive_pwm_brake_order": pwm.get("brake_order", ""),
                "drive_pwm_t": pwm.get("t", ""),
                "drive_pwm_command_t": pwm.get("command_t", pwm.get("t", "")),
                "drive_pwm_read_t": pwm.get("read_t", ""),
                "drive_pwm_exact_dedupe_enabled": pwm_stats.get("pwm_exact_dedupe_enabled", ""),
                "drive_pwm_cache_fds_enabled": pwm_stats.get("pwm_cache_fds_enabled", ""),
                "drive_pwm_cached_fd_count": pwm_stats.get("pwm_cached_fd_count", ""),
                "drive_pwm_write_count_total": pwm_stats.get("pwm_write_count_total", ""),
                "drive_pwm_skip_count_total": pwm_stats.get("pwm_skip_count_total", ""),
                "drive_pwm_cache_fallback_count_total": pwm_stats.get("pwm_cache_fallback_count_total", ""),
                "drive_pwm_cache_open_count_total": pwm_stats.get("pwm_cache_open_count_total", ""),
                "drive_pwm_cache_close_count_total": pwm_stats.get("pwm_cache_close_count_total", ""),
                "drive_pwm_write_count_servo": pwm_stats.get("pwm_write_count_servo", ""),
                "drive_pwm_skip_count_servo": pwm_stats.get("pwm_skip_count_servo", ""),
                "drive_pwm_cache_fallback_count_servo": pwm_stats.get("pwm_cache_fallback_count_servo", ""),
                "drive_pwm_write_count_front_led": pwm_stats.get("pwm_write_count_front_led", ""),
                "drive_pwm_skip_count_front_led": pwm_stats.get("pwm_skip_count_front_led", ""),
                "drive_pwm_cache_fallback_count_front_led": pwm_stats.get("pwm_cache_fallback_count_front_led", ""),
                "drive_pwm_write_count_drive_reverse": pwm_stats.get("pwm_write_count_drive_reverse", ""),
                "drive_pwm_skip_count_drive_reverse": pwm_stats.get("pwm_skip_count_drive_reverse", ""),
                "drive_pwm_cache_fallback_count_drive_reverse": pwm_stats.get("pwm_cache_fallback_count_drive_reverse", ""),
                "drive_pwm_write_count_drive_forward": pwm_stats.get("pwm_write_count_drive_forward", ""),
                "drive_pwm_skip_count_drive_forward": pwm_stats.get("pwm_skip_count_drive_forward", ""),
                "drive_pwm_cache_fallback_count_drive_forward": pwm_stats.get("pwm_cache_fallback_count_drive_forward", ""),
            }
        finally:
            self._record_timing_event("get_telemetry", duration_ms=(monotonic() - t0) * 1000.0)

    def _enter_breakaway(self) -> None:
        if self._mode != "VEL" or not self._breakaway_allowed:
            return
        self._submode = "BREAK"
        self._break_t0 = monotonic()
        self._break_count0 = self.count
        self._break_u = self.cfg.breakaway_duty_start
        self.ctrl.reset()

    def _break_motion(self) -> bool:
        return abs(self.count - self._break_count0) >= self.cfg.breakaway_detect_counts

    def _debug_print(self, msg: str) -> None:
        if not self.cfg.debug:
            return
        now = monotonic()
        if (now - self._dbg_last_t) >= self._dbg_period:
            self._dbg_last_t = now
            print(msg)

    def _pwm_readback_loop(self) -> None:
        hz = float(getattr(self.cfg, "pwm_readback_hz", 5.0))
        if hz <= 0.0:
            return
        period = 1.0 / max(0.5, hz)
        while self._pwm_readback_running:
            t0 = monotonic()
            try:
                refresh = getattr(hpwm, "refresh_drive_readback", None)
                if callable(refresh):
                    refresh()
            except Exception:
                pass
            remaining = period - (monotonic() - t0)
            if remaining > 0.0:
                time.sleep(min(remaining, period))

    # ---------- control ----------
    def _control_loop(self) -> None:
        dt_nom = 1.0 / self.cfg.control_hz
        next_t = monotonic()

        while self._running:
            loop_start = monotonic()
            pos, vel, v_rate, cnt, dcount, bad_vel = self.estimator.update(abs(self._v_cmd_ramped))
            self.pos_m = pos
            self.v_mps = vel
            self.v_rate_mps = v_rate
            self.count = cnt

            now = monotonic()
            if (not bad_vel) and abs(dcount) >= 2:
                self._last_progress_t = now
                self._last_real_motion_t = now
            if dcount != 0:
                self._last_count_change_t = now

            if self._mode == "IDLE":
                self._v_cmd_ramped = 0.0
                self.ctrl.reset()
                self._submode = "NONE"
                u_app = self._apply_output(0.0)
                self._debug_print(f"[dbg] IDLE v={self.v_mps:+.3f} u={u_app:+.3f}")

            elif self._mode == "BRAKE":
                self._v_cmd_ramped = 0.0
                self.ctrl.reset()
                self._submode = "NONE"
                if (
                    self._last_brake_sent is None
                    or abs(self._brake_level - self._last_brake_sent) >= self.cfg.duty_update_deadband
                    or self._last_brake_order_sent != self._brake_order
                ):
                    publish_t0 = monotonic()
                    self._publish_motion("brake", brake_level=self._brake_level, brake_order=self._brake_order)
                    publish_ms = (monotonic() - publish_t0) * 1000.0
                    self._record_timing_event("internal_brake_publish", duration_ms=publish_ms, extra={"brake_order": self._brake_order})
                    self._last_brake_sent = self._brake_level
                    self._last_brake_order_sent = self._brake_order
                self._last_u_sent = None
                self._actuating = False
                self._debug_print(f"[dbg] BRAKE v={self.v_mps:+.3f} level={self._brake_level:.2f}")

            elif self._mode == "VEL":
                self._v_cmd_ramped = self._slew_limit_asym(
                    self._v_cmd_user,
                    self._v_cmd_ramped,
                    dt_nom,
                    self.cfg.v_slew_mps2,
                    self.cfg.v_decel_slew_mps2,
                )
                v_cmd = self._v_cmd_ramped

                if abs(v_cmd) < 0.005:
                    self.ctrl.reset()
                    self._submode = "NONE"
                    u_app = self._apply_output(0.0)
                    self._debug_print(f"[dbg] VEL near-zero coast v={self.v_mps:+.3f} u={u_app:+.3f}")
                else:
                    direction = 1.0 if v_cmd > 0 else -1.0
                    creep = self._creep_active_for_speed(v_cmd)
                    feedback_mps, feedback_source, _feedback_age_ms = self._select_velocity_feedback(
                        v_cmd=v_cmd,
                        creep=creep,
                        now=now,
                    )

                    # ---- BREAKAWAY ----
                    if self._breakaway_active_for_speed(v_cmd) and self._submode in ("NONE", "BREAK"):
                        if self._submode == "NONE":
                            self._enter_breakaway()

                        t_in = now - self._break_t0
                        self._break_u += self.cfg.breakaway_duty_ramp_per_s * dt_nom
                        self._break_u = min(self._break_u, self.cfg.breakaway_duty_max)

                        u_raw = direction * self._break_u
                        u_raw = max(-self.cfg.max_command_duty, min(self.cfg.max_command_duty, u_raw))
                        u_app = self._apply_output(u_raw)

                        dc = self.count - self._break_count0
                        moved = self._break_motion()

                        if moved and t_in >= self.cfg.breakaway_min_time_s:
                            target_hold = self.ctrl.feedforward_abs(abs(v_cmd)) if self.ctrl.profile_enabled() else 0.0
                            self._u_hold = max(self.cfg.hold_min, self._break_u * self.cfg.hold_capture_factor, target_hold)
                            self._submode = "REG"
                            self.ctrl.reset()

                        if t_in > self.cfg.breakaway_timeout_s:
                            self._break_u = self.cfg.breakaway_duty_max

                        self._debug_print(
                            f"[dbg] VEL BREAK v={self.v_mps:+.3f} v_rate={self.v_rate_mps:+.3f} cmd={v_cmd:+.3f} "
                            f"u={u_app:+.3f} dc={dc} t={t_in:.2f} bad={bad_vel}"
                        )

                    else:
                        # ---- REGULATE ----
                        self._submode = "REG"

                        # Stall truth is based on meaningful forward encoder progress, not tiny jitter.
                        effectively_stalled = False
                        if creep:
                            stall_age = now - self._last_progress_t
                            count_stall_age = now - self._last_count_change_t
                            no_progress_too_long = stall_age > self.cfg.stall_timeout_s
                            count_frozen_too_long = count_stall_age > self.cfg.stall_timeout_s
                            too_slow_vs_cmd = abs(self.v_mps) < max(0.012, 0.55 * abs(v_cmd))
                            tiny_or_wrong_counts = abs(dcount) <= 1

                            if (no_progress_too_long and (too_slow_vs_cmd or tiny_or_wrong_counts)) or count_frozen_too_long:
                                effectively_stalled = True

                            # On a fresh low-speed target change, do not stay trapped in REG if we are barely moving.
                            if self._target_changed:
                                if abs(self.v_mps) < max(0.015, 0.65 * abs(v_cmd)) and abs(dcount) <= 1:
                                    effectively_stalled = True
                                self._target_changed = False

                        if effectively_stalled:
                            if self._breakaway_active_for_speed(v_cmd):
                                self._enter_breakaway()
                                next_t += dt_nom
                                sleep_s = next_t - monotonic()
                                if sleep_s > 0:
                                    time.sleep(sleep_s)
                                else:
                                    next_t = monotonic()
                                continue
                            effectively_stalled = False

                        # For creep, do NOT coast on small overspeed. Only treat truly absurd rate as bad.
                        absurd_rate = abs(self.v_rate_mps) > max(0.20, 4.0 * max(0.02, abs(v_cmd)))
                        if creep and absurd_rate and abs(dcount) <= 1:
                            # freeze correction instead of coasting violently
                            self.ctrl.reset()
                            u_corr = 0.0
                        else:
                            allow_static = abs(v_cmd) >= self.cfg.v_static_enable
                            if (not creep) and self.ctrl.profile_enabled() and feedback_source != "encoder":
                                allow_integrate = self._actuating
                            else:
                                allow_integrate = self._actuating and (abs(dcount) >= 2) and (not bad_vel)
                            u_corr = self.ctrl.compute(
                                v_cmd=v_cmd,
                                v_meas=feedback_mps,
                                dt=dt_nom,
                                allow_static=allow_static,
                                allow_integrate=allow_integrate,
                                include_feedforward=not creep,
                            )

                        if creep:
                            corr_clamp = self.cfg.creep_corr_clamp
                            u_corr = max(-corr_clamp, min(corr_clamp, u_corr))

                            target_hold = self.ctrl.feedforward_abs(abs(v_cmd)) if self.ctrl.profile_enabled() else self.cfg.hold_min
                            self._u_hold = max(self.cfg.hold_min, self._u_hold, target_hold)

                            overspeeding = abs(self.v_mps) > abs(v_cmd) + self.cfg.hold_decay_overspeed_margin_mps
                            # Decay hold only when we are really overspeeding. Under steering load,
                            # decaying while below target is what lets the robot settle into a stall.
                            if overspeeding and (not bad_vel) and abs(dcount) >= 2:
                                self._u_hold = max(self.cfg.hold_min, target_hold, self._u_hold - self.cfg.hold_decay_per_s * dt_nom)

                            u_raw = direction * self._u_hold + u_corr
                        else:
                            u_raw = u_corr

                        u_raw = max(-self.cfg.max_command_duty, min(self.cfg.max_command_duty, u_raw))
                        duty_slew = self.cfg.creep_duty_slew_per_s if creep else self.cfg.velocity_duty_slew_per_s
                        u_app = self._apply_output(u_raw, duty_slew)

                        self._debug_print(
                            f"[dbg] VEL REG v={self.v_mps:+.3f} v_rate={self.v_rate_mps:+.3f} cmd={v_cmd:+.3f} "
                            f"fb={feedback_source}:{feedback_mps:+.3f} u={u_app:+.3f} hold={self._u_hold:.3f} I={self.ctrl.integrator:+.3f} "
                            f"creep={creep} bad={bad_vel} dcount={dcount} stall_age={now - self._last_progress_t:.2f}"
                        )

            else:
                self._mode = "IDLE"
                self.ctrl.reset()
                self._submode = "NONE"
                u_app = self._apply_output(0.0)
                self._debug_print(f"[dbg] UNKNOWN->IDLE v={self.v_mps:+.3f} u={u_app:+.3f}")

            self._record_internal_loop_timing(loop_start)
            next_t += dt_nom
            sleep_s = next_t - monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = monotonic()


# =========================
# Tests
# =========================

def _print_header(title: str) -> None:
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def encoder_sanity_test(db: DriveBaseV2, duration_s: float = 3.0) -> None:
    _print_header("ENCODER SANITY TEST (spin wheel/motor by hand)")
    t_end = monotonic() + duration_s
    while monotonic() < t_end:
        print(f"pos={db.get_distance():+.4f} m | v={db.get_speed():+.3f} m/s | count={db.get_count():d}")
        time.sleep(0.05)


def creep_test(db: DriveBaseV2, v_list: Optional[List[float]] = None, dwell_s: float = 3.0) -> None:
    _print_header("SLOW CREEP (ROBUST ESTIMATOR + BREAKAWAY/HOLD)")
    if v_list is None:
        v_list = [0.06, 0.05, 0.04, 0.03]

    print("Observe continuity, not perfection. Remove 0.02 until 0.03 is clean.")
    for v in v_list:
        print(f"\nCommanding v={v:.3f} m/s for {dwell_s:.1f}s")
        db.set_target_speed(v)
        t_end = monotonic() + dwell_s
        while monotonic() < t_end:
            print(f"tgt={v:+.3f} | v={db.get_speed():+.3f} | pos={db.get_distance():+.3f} | count={db.get_count()}")
            time.sleep(0.05)

    db.coast()


def main():
    cfg = DriveConfig()

    db = DriveBaseV2(cfg)
    try:
        db.start()
        time.sleep(0.5)

        # Uncomment for initial bring-up:
        # encoder_sanity_test(db, duration_s=3.0)

        creep_test(db, v_list=[0.05, 0.05, 0.05, 0.05], dwell_s=6.0)

        _print_header("ALL TESTS COMPLETE")
        print("If 0.03 still collapses:")
        print("  - raise hold_min slightly (e.g. 0.36 -> 0.40)")
        print("  - lower kp a little (e.g. 0.9 -> 0.8)")
        print("  - reduce hold_decay_per_s (e.g. 0.04 -> 0.02)")
        print("If 0.06/0.05 run too fast:")
        print("  - reduce hold_min slightly")
        print("  - reduce breakaway capture factor a bit")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        db.shutdown()


if __name__ == "__main__":
    main()


DriveBase = DriveBaseV2
