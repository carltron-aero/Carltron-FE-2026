# Software architecture

We use OTOS to follow the robot's movement, LiDAR wall measurements to correct drift and camera images to identify obstacle colours. Navigation uses the calculated position and obstacle map to request speed and steering. DriveBase then controls the motor and servo.

The entry point is `src/tests/perception_web_test.py`, despite its historical test filename. It connects ROS LiDAR input, OTOS polling, the physical start/deadman input, DriveBase, logging and development telemetry to the production package in `src/prod/robot_core`.

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

## Sensor processing and control

We store OTOS updates with timestamps so that each LiDAR beam can be placed using the robot's pose when it was measured, rather than its pose after processing finishes. Wall observations then correct the relationship between local odometry and field coordinates. This lets us retain frequent relative updates between absolute corrections.

Obstacle detections are stored in field-related slots. Their view directions select camera patches for color classification, and repeated observations confirm the passing decisions. Navigation uses this accumulated state instead of rebuilding its route from isolated scans. DriveBase translates the resulting speed and steering commands into motor and servo outputs.

We need to keep track of measurement times as well as processing times. One LiDAR scan covers an interval, and a camera image may spend time in a buffer before we use it. The controller also limits speed if commands become too old. Measuring the update rate alone wouldn't tell us how old the data reaching the motor actually is.

## Challenge sequence

The robot waits for the configured physical start signal, observes the start delay and stationary initialization period, and establishes the initial field pose before allowing navigation to progress. A parking-area start can enter the departure sequence. Normal navigation can enter observation or color-confirmation holds when information is insufficient.

In the Obstacle Challenge, the first lap establishes obstacle and gate decisions. Later laps reuse confirmed route information. After the configured lap count, the baseline enters its stop and parking sequence. In the Open Challenge, it ends after the required laps in the finishing section. The parking sequence is the German-final version, not an international parallel-parking replacement.

## Configuration and diagnostics

`parse_args()` in `perception.py` defines the effective command-line configuration. `DriveConfig` in `drivebase.py` defines actuator-specific parameters. Changing mechanics requires checking both the perception geometry and actuator calibration, not only a top-speed setting.

We use the web server and CSV output during development, but the robot can navigate without a connected browser. The program uses a physical start/deadman input by default. The [runbook](operations/runbook.md) describes service ownership, shutdown and radio-mode operation on the installed robot.

## Test programs

We developed the newer [test programs](testing/measurement-system.md) separately from the competition program. They record the original sensor data and drive outputs, start and stop the tracking recordings on the PC, and handle the return between measurements. Through the position API they can check the stationary start position and supervise a drive. The competition program doesn't depend on this external setup.

The [experimental fusion](testing/localisation-development.md) uses OTOS position increments and angular motion together with delayed LiDAR wall measurements. We disabled encoder acquisition, BNO085 and onboard-camera processing for its curve tests. Moving Optical Motion Tracking positions are used for evaluation and safety checks, while navigation uses the onboard estimate. We are still testing this version and haven't replaced `perception.py` with it.
