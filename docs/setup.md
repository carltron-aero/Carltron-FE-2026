# Software setup

This source baseline targets the existing Raspberry Pi 5 robot installation: Ubuntu 24.04, Python 3.12, and ROS 2 Jazzy. It is not a complete operating-system image or a hardware-independent desktop application. Preserve the proven robot image before changing packages or device configuration.

## System prerequisites

- ROS 2 Jazzy with `rclpy` and `sensor_msgs`, normally sourced from `/opt/ros/jazzy/setup.bash`.
- An S3 LiDAR driver publishing `sensor_msgs/LaserScan` on `/scan`, with the correct serial port, scan timing, and rotation configuration.
- A working libcamera installation with Python bindings for the installed camera.
- OTOS access over I2C and the SparkFun Qwiic Python dependency.
- libgpiod version 2 Python API and access to the robot's GPIO device.
- Hardware PWM channels matching the board wiring and `hw_pwm.py` configuration.
- Device permissions for the account running the robot process.

The exact LiDAR driver revision, kernel/overlay configuration, camera package versions, and robot environment lockfile have not yet been exported into this repository. Do not assume that installing the Python requirements alone reproduces these parts.

## Python environment

Use the existing validated robot virtual environment when running this baseline. For a separate installation, a virtual environment with system packages visible is usually needed to access the distro-provided ROS and camera modules:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

This is a preparation procedure, not a tested replacement for the current robot image. `requirements.txt` identifies the Python packages; it is not yet an exact lockfile. In particular, OpenCV and libcamera compatibility must be checked on the Pi before accepting a new environment. The OTOS package is documented by [SparkFun](https://github.com/sparkfun/Qwiic_OTOS_Py).

Record the Python and package versions from the validated installation, then check imports before connecting actuators:

```bash
python3 --version
python3 -m pip freeze
python3 -c "import numpy, cv2, qwiic_otos, rclpy, libcamera, gpiod; print('Imports OK')"
ros2 topic info /scan
```

Do not commit environment files containing credentials or device-specific secrets. Publishing a package list is not a substitute for documenting the PWM overlays, permissions, LiDAR driver, and startup files.

## Running the robot

Stop the existing robot service before launching a second manual process:

```bash
sudo systemctl stop midcmbfol.service
source /opt/ros/jazzy/setup.bash
python3 src/tests/perception_web_test.py
```

Activate the robot's validated virtual environment as well if it is not already active. The program enables DriveBase control by default and uses a physical start/deadman gate. Its calibration and control code can move the vehicle. Check the start input, motor neutral, steering limits, and process shutdown with the wheels lifted before ground testing.

The installed service uses a wrapper script that selects its environment. Export that service, wrapper, radio-mode helpers, and any permission rules before claiming a reproducible boot installation. Their current operating commands are listed in [the runbook](operations/runbook.md).

## Checks without robot hardware

Run the repository checks with Python 3.12 or newer:

```bash
python3 tools/check_repository.py
```

They do not import the robot runner or access devices. For the color-classifier test, install NumPy and OpenCV in a desktop test environment and run from the repository root:

```bash
python3 -m src.tests.cammanager_classifier_synthetic
```

The test substitutes a non-hardware libcamera module for classifier-only checks. It does not validate exposure, camera capture, frame timing, or obstacle recognition on the real field.

## Competition configuration

Use the physical start procedure and verify Wi-Fi/Bluetooth are disabled for an autonomous scoring run. The existing `competition-mode` helper is robot-installation-specific; it is described in the runbook but must still be included and tested in the reproduction material. Select competition settings before placement and do not use the web interface or remote commands during a scoring run.
