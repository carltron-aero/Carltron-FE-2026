# Calibration And Test Suites

These scripts are for diagnostics and calibration. They can command hardware. Stop the main service before running any drivebase calibration.

```bash
sudo systemctl stop midcmbfol.service
```

Restart normal operation afterward:

```bash
sudo systemctl start midcmbfol.service
```

## Current Main OTOS Values

Use these when a calibration script needs to run with the same OTOS model as the main program:

```bash
--linear-scalar 1.028 \
--angular-scalar 0.99582 \
--otos-offset-x 0.1261 \
--otos-offset-y -0.0161 \
--otos-offset-h 181
```

The OTOS-only calibration suite uses different offset flag names:

```bash
--linear-scalar 1.028 \
--angular-scalar 0.99582 \
--offset-x 0.1261 \
--offset-y -0.0161 \
--offset-h 181
```

## OTOS Calibration Suite

Script:

```bash
python3 src/tests/otos_calibration_suite.py
```

This does not command motors. It only reads/configures the OTOS.

Live readout:

```bash
python3 src/tests/otos_calibration_suite.py live \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --offset-x 0.1261 \
  --offset-y -0.0161 \
  --offset-h 181
```

Angular scalar test:

```bash
python3 src/tests/otos_calibration_suite.py angular \
  --actual-deg 360 \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --offset-x 0.1261 \
  --offset-y -0.0161 \
  --offset-h 181 \
  --csv /tmp/otos_angular.csv \
  --out /tmp/otos_angular.json
```

Use a signed angle. For the rotation test around a known point, make the physical rotation as clean as possible and use the actual rotation angle.

Linear scalar test:

```bash
python3 src/tests/otos_calibration_suite.py linear \
  --actual-m 1.0 \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --offset-x 0.1261 \
  --offset-y -0.0161 \
  --offset-h 181 \
  --csv /tmp/otos_linear.csv \
  --out /tmp/otos_linear.json
```

Normally leave `--use-path-distance` off for straight-line tests. It measures integrated path length and can hide lateral/heading issues.

Useful options:

| Option | Use |
| --- | --- |
| `--rate` | OTOS sample rate. |
| `--calibrate-imu` | Calibrate OTOS IMU at start while robot is perfectly still. |
| `--no-reset` | Do not reset tracking. Useful only for specific diagnostics. |
| `--csv` | Save raw samples. |
| `--out` | Save computed scalar/result JSON. |

## Drivebase Speed Calibration Suite

Script:

```bash
python3 src/tests/drivebase_speed_calibration_suite.py
```

Modes:

- `duty-sweep`: direct duty tests.
- `speed-sweep`: closed-loop speed target tests.
- `both`: duty and speed sweep.

Important: this suite defaults to neutral OTOS values (`linear=1`, `angular=1`, offsets `0`). Pass the current main OTOS values explicitly when you want comparable data.

Basic speed sweep:

```bash
python3 src/tests/drivebase_speed_calibration_suite.py speed-sweep \
  --directions both \
  --distance-m 2.0 \
  --speeds 0.2,0.4,0.6,0.8,1.0 \
  --repeat 1 \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --otos-offset-x 0.1261 \
  --otos-offset-y -0.0161 \
  --otos-offset-h 181 \
  --yes
```

Duty sweep:

```bash
python3 src/tests/drivebase_speed_calibration_suite.py duty-sweep \
  --directions both \
  --distance-m 2.0 \
  --duties 0.2,0.3,0.4,0.5,0.6,0.8,1.0 \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --otos-offset-x 0.1261 \
  --otos-offset-y -0.0161 \
  --otos-offset-h 181 \
  --yes
```

Speed-dependent OTOS scale capture with stationary LiDAR endpoints:

```bash
python3 src/tests/drivebase_speed_calibration_suite.py speed-sweep \
  --directions both \
  --distance-m 2.0 \
  --speeds 0.2,0.3,0.4,0.5,0.6,0.8,1.0 \
  --linear-scalar 1.028 \
  --angular-scalar 0.99582 \
  --otos-offset-x 0.1261 \
  --otos-offset-y -0.0161 \
  --otos-offset-h 181 \
  --lidar-endpoint-capture \
  --yes
```

Analyze endpoint capture:

```bash
python3 src/tests/analyze_otos_speed_scale_calibration.py \
  /tmp/drivebase_speed_cal_<timestamp>.csv \
  /tmp/drivebase_speed_cal_<timestamp>_lidar_endpoint_index.json
```

If you manually aligned endpoint scans in the deskew viewer, pass the annotations:

```bash
python3 src/tests/analyze_otos_speed_scale_calibration.py \
  /tmp/drivebase_speed_cal_<timestamp>.csv \
  /tmp/drivebase_speed_cal_<timestamp>_lidar_endpoint_index.json \
  --annotations /tmp/drivebase_speed_cal_<timestamp>_annotations.json
```

Key drivebase speed suite defaults:

| Option | Default | Use |
| --- | ---: | --- |
| `--directions` | `both` | Forward/reverse/both measurements. |
| `--distance-m` | `2.0` | Endpoint distance. |
| `--repeat` | `1` | Repetitions per value. |
| `--reposition-duty` | `0.7` | Direct duty for non-measurement reposition legs. |
| `--heading-hold` | on | Keeps initial heading during legs. |
| `--lateral-hold` | on | Gentle lane centering. |
| `--sample-period-s` | `0.02` | CSV/log sample period. |
| `--max-lateral-m` | `0.3` | Abort if lateral drift exceeds this. |
| `--max-heading-error-deg` | `35.0` | Abort if heading error exceeds this. |
| `--lidar-endpoint-capture` | off | Capture stationary LiDAR scans at endpoints. |

If a calibration run keeps drifting laterally, reduce speed/duty first and verify heading/lateral hold signs before assuming the drivebase is bad.

## Steering Center Calibration

Script:

```bash
python3 src/tests/drivebase_steering_center_suite.py
```

Default candidate trims:

```text
-0.04,-0.02,0.0,0.02,0.04
```

Example:

```bash
python3 src/tests/drivebase_steering_center_suite.py \
  --trims -0.04,-0.02,0.0,0.02,0.04 \
  --speed 0.12 \
  --duration-s 2.5 \
  --repeat 1
```

Use this when the robot consistently drives slightly left/right with steering centered.

## Drivebase Creep Suite

Script:

```bash
python3 src/tests/drivebase_creep_suite.py
```

Modes:

- `straight`
- `circle`
- `reverse`
- `hold`

Example straight low-speed check:

```bash
python3 src/tests/drivebase_creep_suite.py straight \
  --speeds 0.05,0.08,0.10,0.12 \
  --duration-s 2.0 \
  --pause-s 1.0
```

Example repeated hold:

```bash
python3 src/tests/drivebase_creep_suite.py hold \
  --speed 0.08 \
  --hold-repeats 5
```

Use this for low-speed drive behavior, breakaway/creep diagnostics, and checking whether the speed controller still moves cleanly at very low speeds.

## Deskew Viewer And Endpoint Alignment

Deskew viewer:

```bash
python3 src/tests/lidar_deskew_viewer.py \
  /tmp/drivebase_speed_cal_<timestamp>_lidar_endpoint_index.json \
  --host 0.0.0.0 \
  --port 8765
```

Use it to manually align LiDAR endpoint captures and inspect OTOS/fused pose models. Export annotations, then feed them into the analyzer above.

## Logs From Calibration

Typical outputs:

- `/tmp/drivebase_speed_cal_<timestamp>.csv`
- `/tmp/drivebase_speed_cal_<timestamp>_lidar_endpoint_index.json`
- `/tmp/drivebase_steering_center_<timestamp>.csv`
- `/tmp/drivebase_creep_<timestamp>.csv`

Keep the CSV and endpoint index together. The analyzer needs both.
