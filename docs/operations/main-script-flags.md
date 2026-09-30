# Main Script Flags

Main program:

```bash
python3 src/tests/perception_web_test.py
```

This file lists the flags most likely to matter at competition. It is not a full replacement for `--help`; it is a practical tuning map.

Current baseline: with no flags, the program uses the tuned competition defaults listed here.

## Command Pattern

Use defaults:

```bash
python3 src/tests/perception_web_test.py
```

Override only what you are actively testing:

```bash
python3 src/tests/perception_web_test.py \
  --navigation-first-lap-cruise-speed-mps 0.70 \
  --navigation-caution-speed-mps 0.35
```

For systemd autostart, edit `/usr/local/bin/run_midcmbfol.sh` only if you need the service to use non-default flags. Otherwise leave it as the simple default command.

## Startup, Mode, And Deadman

| Flag | Default | Use |
| --- | ---: | --- |
| `--race-mode {auto,obstacle,free}` | `auto` | Startup classifies race mode. Force only for controlled tests. |
| `--free-race` | off | Shortcut for `--race-mode free`. |
| `--no-free-race` | off | Shortcut for obstacle mode. |
| `--competition-start-gate` | on | Wait for physical start/deadman before planning. Keep on for competition. |
| `--no-competition-start-gate` | off | Lab behavior. Do not use for normal competition. |
| `--competition-start-delay-s` | `1.2` | First start delay before sensor data is used. |
| `--competition-initial-stationary-s` | `1.2` | Stationary scan collection after start delay. |
| `--startup-lidar-correction-s` | `0.0` | Optional startup-only LiDAR correction hold. |
| `--deadman-source {physical,web,both}` | `physical` | Start/actuation source. Keep `physical` for competition. |

If the robot starts moving before initial pose lock, that is a serious bug. The current code is intended to prevent irreversible route progress and driving until startup pose is locked.

## Web And Debug UI

| Flag | Default | Use |
| --- | ---: | --- |
| `--port` | `8080` | Web server port. |
| `--web-debug` | off | Start with Full WebUI enabled. Avoid for ranking. |
| `--no-web-portal` | off | Disable the portal. Usually do not use. |
| `--stream-rate` | `25.0` | SSE stream rate for Full/Lite UI. Lower for demo recording. |
| `--web-payload-refresh-hz` | `10.0` | Full payload rebuild rate cap. Lower for Full UI demo recording. |
| `--camera-web-debug` | off | Start with web camera preview enabled. Usually toggle from portal instead. |
| `--camera-web-debug-fps` | `4.0` | Camera JPEG generation rate. Lower if UI causes stale commands. |
| `--camera-web-debug-scale` | `0.55` | JPEG scale. Lower reduces payload/work. |
| `--camera-web-debug-quality` | `72` | JPEG quality. Lower reduces payload size. |

Known behavior: Lite WebUI is much lighter than Full WebUI. Full WebUI can trigger stale-command speed caps during fast runs. Camera preview is useful for demos, but avoid it for ranking.

## OTOS Calibration Values

Coordinate convention for OTOS offset:

- `+x` is forward from the robot reference pose.
- `+y` is left from the robot reference pose.
- `+h` is counterclockwise mounting angle in degrees.

Current defaults:

| Flag | Default | Use |
| --- | ---: | --- |
| `--linear-scalar` | `1.028` | OTOS linear scale. Higher means OTOS reports more distance for the same physical motion. |
| `--angular-scalar` | `0.99582` | OTOS angular scale. |
| `--otos-offset-x` | `0.1261` | OTOS sensor x offset from robot pose. |
| `--otos-offset-y` | `-0.0161` | OTOS sensor y offset from robot pose. Negative means right of pose. |
| `--otos-offset-h` | `181.0` | OTOS mounting heading offset. |

Only change these after a deliberate calibration or A/B test.

## LiDAR Correction

Normal default is LiDAR correction active:

```text
--lidar-correction-mode apply
```

| Flag | Default | Use |
| --- | ---: | --- |
| `--lidar-correction-mode {apply,observe}` | `apply` | `apply` changes the fused anchor. `observe` logs correction without applying it. |
| `--no-lidar-correction` | off | Disable correction entirely. Only for diagnostics. |
| `--lidar-axis-correction-heading` | on | Allow fast heading correction. |
| `--no-lidar-axis-correction-heading` | off | Disable fast heading correction if heading noise is suspected. |
| `--lidar-axis-correction-max-step-m` | `0.014` | Max XY correction per accepted scan. Higher corrects faster but risks overreacting. |
| `--lidar-axis-correction-max-step-deg` | `0.08` | Max heading correction per accepted scan. |
| `--lidar-axis-correction-max-error-m` | `0.12` | Reject one-axis wall residuals above this. Lower triggers missing-measurement holds earlier; higher tolerates bigger pose error. |
| `--lidar-axis-trend-window` | `5` | Recent scans in fast trend window. |
| `--lidar-axis-trend-min-samples` | `3` | Minimum samples needed to accept an axis trend. |
| `--lidar-axis-trend-sign-ratio` | `0.65` | Same-sign majority required. |

Position wall-distance weighting:

| Flag | Default | Meaning |
| --- | ---: | --- |
| `--lidar-wall-distance-min-m` | `0.33` | Distance where position authority reaches minimum. |
| `--lidar-wall-distance-full-m` | `0.44` | Distance where position authority is full. |
| `--lidar-wall-distance-min-authority` | `0.0` | Minimum close-wall position authority. |

Heading wall-distance weighting:

| Flag | Default | Meaning |
| --- | ---: | --- |
| `--lidar-heading-wall-distance-min-m` | `0.90` | Distance where heading authority reaches minimum. |
| `--lidar-heading-wall-distance-full-m` | `1.10` | Distance where heading authority is full. |
| `--lidar-heading-wall-distance-min-authority` | `0.0` | Minimum close-wall heading authority. |

Other correction flags:

| Flag | Default | Use |
| --- | ---: | --- |
| `--lidar-correction-during-unpark` | off | Apply LiDAR correction during unpark. Default off. |
| `--lidar-correction-during-smart-parking` | on | Apply LiDAR correction during smart parking. |
| `--lidar-correction-status-detail` | off | More verbose correction logs. Useful for analysis, not needed for ranking. |
| `--lidar-correction-diagnostic-status-hz` | `10.0` | Verbose status publish rate. |
| `--lidar-processing-worker-process` | on | Full LiDAR worker process. Keep on. |
| `--no-lidar-processing-worker-process` | off | Main-process LiDAR processing. Only for diagnostics. |

## Localization Holds And Low Pose Confidence

These are recovery systems for pose jumps or missing/low-quality LiDAR correction measurements.

| Flag | Default | Use |
| --- | ---: | --- |
| `--otos-pose-jump-detection` | on | Detect physically implausible OTOS pose jumps. |
| `--localization-hold-min-s` | `2.0` | Minimum localization hold duration. |
| `--localization-hold-timeout-s` | `6.0` | Maximum hold after latest trigger. |
| `--localization-hold-no-current-measurement-trigger` | on | Trigger hold when X/Y current measurements are repeatedly missing. |
| `--localization-hold-no-current-window` | `5` | Window size for no-current trigger. |
| `--localization-hold-no-current-required` | `2` | Missing measurements required in the window. |
| `--localization-hold-low-confidence-trigger` | on | Trigger hold for repeated low LiDAR pose confidence. |
| `--localization-hold-low-confidence-window` | `5` | Window size. |
| `--localization-hold-low-confidence-required` | `3` | Low-confidence readings required. |
| `--navigation-low-confidence-speed-mps` | `0.22` | Speed while progress is allowed despite low pose confidence. |
| `--navigation-low-confidence-progress-watchdog` | on | Prevent terminal stuck state when low confidence is the only blocker. |

If localization holds trigger too often, first inspect logs. Do not blindly disable them, because they protect against real OTOS jumps.

## Free Race

| Flag | Default | Use |
| --- | ---: | --- |
| `--free-race-speed-mps` | `1.4` | Free race target speed cap. |
| `--free-race-lookahead-m` | `0.38` | Free race lookahead. |
| `--free-race-adaptive-inner-route` | on | Scan lap 1 and use shorter far-wall route when safe. |
| `--no-free-race-adaptive-inner-route` | off | Disable far/close inner wall route adaptation. |
| `--free-race-inner-gap-threshold-m` | `0.80` | Gap threshold: above is far, below is close. |
| `--free-race-inner-gap-min-samples` | `3` | Confirmed samples per section. |
| `--free-race-inner-gap-side-probe-deg` | `6.0` | Side ROI width. |
| `--free-race-outer-lane-m` | `0.40` | Conservative scan/unknown lane from outer wall. |
| `--free-race-inner-lane-m` | `0.60` | Deprecated compatibility flag; adaptive far sections use predefined inner-tight route. |

## Obstacle Mode Speeds And Lookahead

| Flag | Default | Use |
| --- | ---: | --- |
| `--drive-command-max-speed-mps` | `1.2` | Global maximum command speed. |
| `--navigation-obstacle-mode-max-speed-mps` | `1.1` | Obstacle-mode ceiling below global max. |
| `--navigation-full-lock-speed-mps` | `1.1` | Speed after all obstacle gates are locked. |
| `--navigation-first-lap-cruise-speed-mps` | `0.75` | First lap cruise when current and next section are color-ready. |
| `--navigation-caution-speed-mps` | `0.38` | Conservative speed for uncertain cases. |
| `--navigation-obstacle-speed-lookahead-min-m` | `0.16` | Lookahead at zero/low speed. |
| `--navigation-obstacle-speed-lookahead-max-m` | `0.44` | Lookahead at full configured speed scale. |
| `--navigation-obstacle-speed-lookahead-max-speed-mps` | `1.0` | Speed that maps to max lookahead. |
| `--navigation-tracking-error-speed-cap` | off | Tracking-error speed cap is disabled by default. |

Adaptive corridor:

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-adaptive-corridor` | on | Corridor-based lookahead/speed shaping. |
| `--no-navigation-adaptive-corridor` | off | Disable corridor adaptive logic. |
| `--navigation-adaptive-corridor-min-lookahead-m` | `0.22` | Minimum corridor lookahead. |
| `--navigation-adaptive-corridor-max-lookahead-m` | `0.32` | Maximum corridor lookahead. |

Corner transition speed cap:

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-corner-transition-speed-cap` | on | Applies special cap to selected risky transitions. |
| `--navigation-corner-transition-speed-distance-m` | `0.32` | Route distance window where cap applies. |
| `--navigation-corner-transition-speed-max-mps` | `0.72` | Absolute cap during those transitions. |
| `--navigation-corner-transition-speed-transition` | `inner_tight_to_outer_tight,inner_tight_to_outer_pass` | Transition keys receiving the cap. |

Stale command safety cap:

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-command-staleness-speed-cap` | on | Cap speed if drive command is stale. |
| `--navigation-command-staleness-threshold-ms` | `80.0` | Command age threshold. |
| `--navigation-command-staleness-cap-mps` | `0.70` | Temporary cap after stale event. |
| `--navigation-command-staleness-hold-s` | `0.20` | Hold time after each stale event. |

Do not disable this for competition. If Full WebUI causes this cap, use Lite WebUI or reduce web/camera rates.

## Color Confirmation And Party Mode

Normal confirmation behavior:

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-color-confirmation-stop` | on | Stationary confirmation holds for route-relevant color. |
| `--navigation-post-unpark-start-confirmation` | on | One-shot start-section confirmation right after unpark. |
| `--navigation-color-confirmation-timeout-s` | `4.0` | Max confirmation hold duration. |
| `--navigation-color-confirmation-timeout-min-color-votes` | `4` | Minimum color samples for timeout majority. |
| `--navigation-color-confirmation-exposure-retries` | `3` | Gain-adjusted retry holds after clipped/dark timeouts. |
| `--navigation-color-confirmation-max-holds-per-section` | `3` | Hold limit per section. |
| `--navigation-unknown-color-speed-mps` | `0.26` | Cap for next relevant obstacle with unknown color. |

Party Mode:

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-party-mode` | off | Experimental fast obstacle mode. Skips optional corner/start holds and locks strong moving color windows. |
| `--navigation-party-color-window` | `5` | Moving color event window. |
| `--navigation-party-color-required` | `4` | Matching moving samples required. |
| `--navigation-party-color-max-measurement-age-ms` | `150.0` | Timing age limit for moving color lock. |

Party Mode is intentionally opt-in. Do not use it for normal competition unless it has been tested in the exact lighting/layout conditions.

Camera color tuning:

| Flag | Default | Use |
| --- | ---: | --- |
| `--camera-analog-gain` | `14.0` | Normal operation camera gain. |
| `--camera-analog-gain-min` | `1.0` | Min gain for exposure retries. |
| `--camera-analog-gain-max` | `38.0` | Max gain for exposure retries. |
| `--camera-color-delay-ms` | `30.0` | Camera query timestamp offset. |
| `--camera-color-angle-offset-deg` | `178.4` | Camera ROI angle calibration. |
| `--camera-color-min-confidence` | `0.45` | Minimum classifier confidence. |

## Obstacle Slot Detection

Usually leave these alone unless false positives/negatives appear.

| Flag | Default | Use |
| --- | ---: | --- |
| `--obstacle-color-confirm-window` | `5` | Rolling color confirmation window. |
| `--obstacle-color-confirm-required` | `3` | Confirmed labels needed. |
| `--obstacle-color-proposal-window` | `6` | Raw proposal window. |
| `--obstacle-color-proposal-required` | `4` | Raw proposal count before confirmation requested. |
| `--obstacle-distance-point-threshold` | on | Distance-dependent slot point threshold. Keep on. |
| `--obstacle-detection-yaw-rate-gate` | on | Ignore obstacle scans during hard turns. Keep on. |

## Collision Supervisor

The LiDAR collision supervisor is active by default.

| Flag | Default | Use |
| --- | ---: | --- |
| `--collision-lidar-active-hard-brake` | on | Actually hard-brake on LiDAR near-field trigger. |
| `--collision-persistent-recovery` | on | Try bounded reverse recovery after persistent hazard. |
| `--collision-lidar-window` | `5` | Recent usable scans considered. |
| `--collision-lidar-required` | `2` | Hard-zone scans required unless strong-single trigger fires. |
| `--collision-lidar-depth-max-m` | `0.22` | Hard-brake zone depth. |
| `--collision-lidar-warning-depth-max-m` | `0.30` | Warning/pretrigger zone depth. |
| `--collision-lidar-min-points` | `18` | Points required in hard zone. |
| `--collision-lidar-strong-single-hard-trigger` | on | Immediate brake for very strong single hard scan. |
| `--collision-lidar-yaw-max-dps` | `60.0` | Ignore collision scan above this yaw rate. |

If it brakes too late, increase warning/hard depth or reduce required scan evidence. If it false-triggers, increase point thresholds or required count.

## Unpark

| Flag | Default | Effect |
| --- | ---: | --- |
| `--unpark-forward-duty-max` | `0.58` | Max direct forward duty in unpark. |
| `--unpark-reverse-duty-max` | `0.58` | Max direct reverse duty in unpark. |
| `--unpark-front-anchor-clearance-m` | `0.018` | Planned front clearance stop before reverse setup. |
| `--unpark-front-wall-min-clearance-m` | `0.018` | Minimum front parking barrier clearance for recovery. |
| `--unpark-rear-wall-min-clearance-m` | `0.022` | Minimum rear clearance. |
| `--unpark-exit-heading-deg` | `57.0` | Heading threshold for exit. |
| `--unpark-steering-norm` | `0.76` | Fixed normalized steering. |

Raise duty only carefully. Unpark uses direct duty, not the normal speed profile.

## Smart Parking

| Flag | Default | Effect |
| --- | ---: | --- |
| `--smart-parking` | on | Enable smart parking. |
| `--smart-parking-turn-radius-m` | `0.12` | Measured reverse turn radius. |
| `--smart-parking-heading-target-deg` | `88.3` | Target final parking heading. |
| `--smart-parking-reverse-turn-speed-mps` | `0.12` | Reverse speed during fixed-steer parking turn. |
| `--smart-parking-straight-reverse-speed-mps` | `0.14` | Straight reverse speed into slot. |

If final heading is consistently wrong, first tune `--smart-parking-heading-target-deg` and then `--smart-parking-turn-radius-m`.

## Drivebase Actuation

| Flag | Default | Use |
| --- | ---: | --- |
| `--drive-command-max-speed-mps` | `1.2` | Global speed ceiling. |
| `--drive-command-accel-mps2` | `1.0` | Drive command ramp limit. |
| `--drivebase-v-slew-mps2` | `0.75` | Drivebase speed controller slew. |
| `--drivebase-pwm-cache-fds` | on | Keep PWM sysfs descriptors cached. Keep on for timing. |
| `--drive-command-steering-speed-min-mps` | `1.25` | Speed used by steering command limits. |

Avoid turning off PWM FD caching unless diagnosing low-level PWM behavior.

## Planner Blocked Recovery

| Flag | Default | Use |
| --- | ---: | --- |
| `--navigation-planner-blocked-recovery` | on | Bounded reverse escape after persistent planner block. |
| `--navigation-planner-blocked-recovery-delay-s` | `3.0` | Block duration before arming. |
| `--navigation-planner-blocked-recovery-max-m` | `0.30` | Max single reverse distance. |
| `--navigation-planner-blocked-recovery-speed-mps` | `0.12` | Reverse speed. |

This is a last-resort stuck prevention system. If it triggers often, inspect planner logs instead of increasing reverse distance blindly.

## Mostly Diagnostic Or Usually Do Not Touch

- `--lidar-correction-validation`: expensive old validation path. Normally off.
- `--lidar-correction-worker-process`: older correction-only worker. The full LiDAR processing worker is the important one now.
- `--navigation-adaptive-lookahead`: older adaptive lookahead path. Current default uses adaptive corridor instead.
- `--navigation-tracking-error-speed-cap`: currently disabled by default.
- `--free-race-inner-lane-m`: deprecated compatibility flag.
- `--line-correction-candidate-walls`: candidate wall selection is off by default; full wall information is used.
