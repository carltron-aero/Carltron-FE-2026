# Software architecture

The current hardware runner is `src/tests/perception_web_test.py`. It connects ROS LiDAR input, OTOS polling, the physical start/deadman input, DriveBase, logging, and development telemetry to the production package in `src/prod/robot_core`.

## Module responsibilities

| Module | Responsibility |
| --- | --- |
| `perception.py` | Pose history, LiDAR processing and correction, field and obstacle state, color confirmation, race progression, recovery logic, and command preparation. |
| `navigation_global.py` | Field-related routes, gate waypoints, lookahead targets, speed caps, and geometric safety checks. |
| `navigation.py` | Supporting section-frame and local route-preview geometry. |
| `cammanager.py` | libcamera acquisition, timestamped frame buffering, ring-region sampling, and red/green classification. |
| `drivebase.py` | Encoder feedback, speed-duty feedforward, speed control, steering calibration, and actuator supervision. |
| `hw_pwm.py` | Hardware PWM output through Linux sysfs, write caching, and timing diagnostics. |
| `corner_waypoints.json` | Current corner-route configuration loaded by the navigation module. |

## Measurement to actuation

OTOS updates are stored with timestamps. LiDAR scans are aligned with this motion history so their beams can be transformed into a consistent field frame. Wall observations correct the odometry anchor. Obstacle detections become field-related slots; the camera samples the corresponding view directions and supplies color evidence. Navigation uses the resulting pose and obstacle state to construct a route and desired motion. DriveBase translates the command into motor and servo outputs.

The time assigned to a measurement is important. A LiDAR scan covers a finite interval, the camera may supply a buffered frame, and the motion controller must reject or limit commands that are too old. Updating at a nominal rate does not establish an end-to-end latency bound. Logs should distinguish sensor time, processing time, command preparation, and command application.

## Challenge sequence

The robot waits for the configured physical start signal, observes the start delay and stationary initialization period, and establishes the initial field pose before allowing navigation to progress. A parking-area start can enter the departure sequence. Normal navigation can enter observation or color-confirmation holds when information is insufficient.

In the Obstacle Challenge, the first lap establishes obstacle and gate decisions. Later laps reuse confirmed route information. After the configured lap count, the baseline enters its stop and parking sequence. In the Open Challenge, it ends after the required laps in the finishing section. These are baseline behaviors; the international parking replacement and complete rule-specific acceptance tests are still required.

## Configuration and diagnostics

`parse_args()` in `perception.py` defines the effective command-line configuration. `DriveConfig` in `drivebase.py` defines actuator-specific parameters. Changing mechanics requires checking both the perception geometry and actuator calibration, not only a top-speed setting.

The web server and CSV output support development. They must not become required external inputs for autonomous driving. The radio-disable procedure, start button, process ownership, safe motor neutral, and shutdown behavior need physical checks on the final assembled robot.
