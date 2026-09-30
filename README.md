# Carltron Future Engineers 2026

This repository contains Team Carltron's autonomous vehicle software and engineering documentation for the World Robot Olympiad Future Engineers category in 2026. The starting point is the robot used at the German final, where the team achieved second place and qualified for the International Final in Puerto Rico. The vehicle uses front-wheel steering, rear-wheel drive, optical odometry, a scanning LiDAR, and an upward-facing wide-angle color camera.

The first repository version preserves the current production software and its calibration and diagnostic tools. It is an engineering baseline for international preparation, not a claim that all international requirements have already been implemented or verified. In particular, the German-final parking maneuver must be replaced and tested against the international parallel-parking requirements. Hardware models, wiring drawings, photographs, and demonstration videos will be added as the corresponding material is prepared and checked.

## Repository contents

| Location | Contents |
| --- | --- |
| [src/prod/robot_core](src/prod/robot_core/) | Production perception, localization, navigation, camera, motor, steering, and hardware PWM code. |
| [src/tests](src/tests/) | Hardware entry point, calibration suites, synthetic color-classifier checks, log analysis, and waypoint tools. |
| [docs/architecture.md](docs/architecture.md) | Sensor data flow and responsibilities of the software modules. |
| [docs/setup.md](docs/setup.md) | Software environment, hardware prerequisites, and startup procedure. |
| [docs/hardware.md](docs/hardware.md) | Baseline hardware and configuration that must be checked on an assembled vehicle. |
| [docs/operations](docs/operations/) | Existing operating notes, command-line flags, calibration procedures, and troubleshooting. |
| [docs/international-preparation.md](docs/international-preparation.md) | Work required for the international competition and the planned modeling project. |
| [docs/publication-checklist.md](docs/publication-checklist.md) | Documentation and evidence still needed before the competition submission. |
| [models](models/) | Location for mechanical models and manufacturing files. |
| [schemes](schemes/) | Location for circuit diagrams and board documentation. |
| [t-photos](t-photos/) and [v-photos](v-photos/) | Locations for team and vehicle photographs. |
| [video](video/) | Location for links and descriptions of autonomous driving demonstrations. |

The production entry point retains its existing filename, `src/tests/perception_web_test.py`, even though it is the normal hardware runner rather than a unit test. Historical controller experiments and duplicate driver copies are not included in this baseline. The runtime imports the production package from `src/prod`, keeping the software layout consistent with the existing robot installation.

## Vehicle and sensor architecture

A Raspberry Pi 5 runs the vehicle software on Ubuntu 24.04 with Python 3.12. ROS 2 Jazzy provides the transport for LiDAR scans as `sensor_msgs/LaserScan` messages on `/scan`. The rest of the control stack operates directly in Python rather than routing every internal state through ROS topics. The custom main board integrates the power distribution and actuator interfaces; a DRV8871 motor-driver module controls the brushed drive motor.

The SparkFun Qwiic Optical Tracking Odometry Sensor, or OTOS, measures short-term motion relative to the floor. It supplies frequent position, heading, and velocity updates. The RPLIDAR S3 observes the field walls and provides absolute corrections to the accumulated optical odometry. The drive encoder supplies additional motion feedback to the motor controller. These sources have different error characteristics: rapid relative motion is useful for control and scan alignment, while wall geometry helps bound longer-term drift.

The camera is a Waveshare IMX219-200 module fitted with an Arducam LN010 M12 fisheye lens. It points upward and uses a circular viewing region to observe the surrounding obstacles. The current camera pipeline classifies obstacle colors; it is not yet an independently validated camera-based absolute localization system. LiDAR detections select angular regions of interest, and the camera evaluates the corresponding image patches.

The sensor combination is intended to separate geometry from color. LiDAR provides obstacle locations and field structure, while the camera determines whether a detected obstacle should be passed on the red or green side. Keeping the camera viewpoint below the field walls is an optical requirement of this arrangement. The final camera mounting, calibration, and illumination behavior need to be documented with the assembled international vehicle.

## Localization and navigation

The perception core represents the robot pose as position and heading in a field coordinate system. Positions use meters and headings use degrees. A history of timestamped OTOS poses supports interpolation at the time of each LiDAR beam. This matters because a rotating LiDAR scan is collected over a period of motion rather than at a single instant.

LiDAR processing transforms scan points into the field frame and compensates for robot movement during the scan. Wall observations are compared with the field model to estimate absolute pose corrections. Corrections update the relationship between local odometry and the field frame, allowing frequent OTOS motion updates to remain useful between LiDAR scans. Confidence checks, correction limits, and measurement-age handling help prevent unsuitable observations from immediately changing the driven route.

Obstacle detections are stored in a field-related slot map rather than treated only as temporary objects in the current scan. The first obstacle lap gathers and confirms geometry and color information. Confirmed gate decisions can then be locked so that later laps follow the established route without repeatedly reconsidering the same obstacle. The navigation module builds waypoints and uses lookahead-based steering with speed caps, observation holds, and recovery states.

The Open Challenge uses the same pose and control foundations without the obstacle-passing sequence. Its adaptive inner-route logic classifies corridor geometry during the initial lap and can shorten the subsequent route. The Obstacle Challenge adds obstacle mapping, color confirmation, departure from the parking area, and a finishing parking state machine. Full international-layout coverage remains a separate validation task; support in the algorithm is not evidence that every legal start and layout has been tested.

## Motor and steering control

`drivebase.py` converts desired speed and steering into actuator outputs. Its configuration includes the wheel diameter, encoder conversion, drive reduction, steering trim, speed-duty calibration, acceleration and braking behavior, and low-speed motion handling. The speed controller uses a measured feedforward profile together with feedback and limits. Physical calibration must be repeated if the motor, gearing, wheels, battery behavior, or steering geometry changes.

`hw_pwm.py` controls the Pi's hardware PWM through the Linux sysfs interface. The configured channels handle steering, an LED output, and the two drive directions. The code also supports write timing diagnostics, duplicate-write suppression, and cached file descriptors. These are specific to the robot's board and OS configuration; the presence of this source alone does not configure the required kernel overlays or device permissions on a fresh Pi.

The software's requested speed is not a measurement of achieved vehicle speed. A configured target can exceed what the current drivetrain can deliver, and safety limits can reduce it further. Performance comparisons should therefore use logged physical motion and complete challenge time, not merely command-line speed settings.

## Software setup and startup

The program runs on the robot's Linux environment, not directly on a Windows development computer. A prepared system needs ROS 2 Jazzy, a working S3 driver publishing `/scan`, Python bindings for the camera, access to the OTOS over I2C, GPIO access, and the configured PWM channels. The Python dependency inventory is in [requirements.txt](requirements.txt). ROS and libcamera are system-level dependencies and are not installed by that file.

On the prepared robot, source the ROS environment and activate the existing robot virtual environment before starting the program from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
python3 src/tests/perception_web_test.py
```

The physical start/deadman source is used by default. The runner can command motors, and several calibration suites can also move the vehicle. Stop competing processes, check the actuator permissions and emergency-stop path, and perform initial checks with the wheels lifted or in a controlled test area. Do not start an unattended hardware test solely to inspect the software.

The installed robot uses `midcmbfol.service` for startup. Its wrapper and radio-mode helpers are described in the [runbook](docs/operations/runbook.md), but the installed service and wrapper files are not yet included here. They must be exported and reviewed before a fresh installation can reproduce the complete boot procedure. The setup guide explicitly distinguishes available source from robot-specific configuration that still needs to be supplied.

Command-line arguments are defined in `robot_core/perception.py`. The baseline configures OTOS polling at 150 Hz, command application to DriveBase at 75 Hz, and the inner DriveBase controller at 100 Hz. These are requested scheduling rates, not hard real-time guarantees. The operational flag guide is useful for tuning, but the source and the effective run manifest remain authoritative when checking the configuration used for a particular test.

## Diagnostics and testing

The web portal provides development telemetry, a map, and optional camera previews. It is a diagnostic interface, not an external controller permitted during a competition run. Live visualization and camera encoding consume processing time, so tests must record whether they were enabled. Competition operation must use the required autonomous start procedure with wireless communication disabled and verified on the final hardware.

Each run can create timestamped CSV logs and a manifest containing its effective settings. Important files include `drive_debug.csv`, `lidar_scan_debug.csv`, `camera_color_debug.csv`, and `obstacle_slot_debug.csv`. Raw run archives are deliberately kept outside this repository baseline. Analysis scripts remain available so selected datasets and reproducible results can be added later with their test conditions and software revision.

The included test tools cover OTOS calibration, speed calibration, steering-center adjustment, low-speed driving, LED checks, and color-classifier cases. The synthetic color test does not require camera hardware, but it does require NumPy and OpenCV:

```bash
python3 -m src.tests.cammanager_classifier_synthetic
```

Repository checks do not actuate hardware:

```bash
python3 tools/check_repository.py
```

These checks compile Python source without running it, parse JSON configuration, check local documentation links, and verify the README length. They are also run by the repository's validation workflow. Passing them establishes repository consistency, not driving performance, electrical safety, or competition compliance.

## International preparation

The highest-priority functional change is repeatable parallel parking with the full vehicle footprint inside the allowed area, sufficient wall parallelism, and no contact with the parking barriers. The current German-final parking sequence is retained as baseline source; it is not presented as a completed international solution. Validation must include both travel directions, different start sections, legal corridor configurations, obstacle layouts, and realistic variation in floor traction and lighting.

Further work focuses on bounded localization errors, handling OTOS jumps or contamination, calibrating S3 measurements, and establishing timing margin before increasing speed. Every improvement should be compared with the baseline using the same test conditions and clearly separated measures for score, clearance, pose error, maneuver time, and complete run time.

The planned MathWorks Modeling Award project will connect a measured vehicle and sensor model to controller tests and physical validation. No completed model or award result is claimed in this version. Field models, robot models, test scenarios, parameter identification, and simulation-to-real comparisons will be documented as they become available. Third-party models and dependencies must retain their applicable licenses and attribution.

This repository will evolve through genuine development commits and documented test milestones. Before submission, the team must complete the engineering journal, manufacturing and wiring material, vehicle and team photographs, autonomous driving video links for both challenges, and a verified setup procedure. The [publication checklist](docs/publication-checklist.md) records those gaps so that planned work is not confused with demonstrated capability.
