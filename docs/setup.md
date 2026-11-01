# Software setup

We run this baseline on our Raspberry Pi 5 with Ubuntu 24.04, Python 3.12 and ROS 2 Jazzy. The robot depends on system packages and device configuration as well as the Python code. These instructions cover manual startup on the prepared robot and the dependencies needed for a separate installation; the repository does not contain a complete system image. Back up the working robot image before changing packages or device configuration.

## System prerequisites

- ROS 2 Jazzy with `rclpy` and `sensor_msgs`, normally sourced from `/opt/ros/jazzy/setup.bash`.
- An S3 LiDAR driver publishing `sensor_msgs/LaserScan` on `/scan`, with the correct serial port, scan timing, and rotation configuration.
- A working libcamera installation with Python bindings for the installed camera.
- OTOS access over I2C and the SparkFun Qwiic Python dependency.
- libgpiod version 2 Python API and access to the robot's GPIO device.
- Hardware PWM channels matching the board wiring and `hw_pwm.py` configuration.
- Device permissions for the account running the robot process.

The exact LiDAR driver revision, kernel/overlay configuration, camera package versions and robot environment lockfile are not included. Installing the Python requirements alone therefore does not reproduce the complete robot environment.

## Python environment

Use the existing validated robot virtual environment when running this baseline. For a separate installation, a virtual environment with system packages visible is usually needed to access the distro-provided ROS and camera modules:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

`requirements.txt` lists the Python dependencies, but does not lock their versions. The commands above have not been tested as a replacement for the working robot image, and OpenCV/libcamera compatibility depends on the Pi's system packages. The OTOS package is documented by [SparkFun](https://github.com/sparkfun/Qwiic_OTOS_Py).

The following checks report the installed versions, test imports and inspect the LiDAR topic without starting the robot runner:

```bash
python3 --version
python3 -m pip freeze
python3 -c "import numpy, cv2, qwiic_otos, rclpy, libcamera, gpiod; print('Imports OK')"
ros2 topic info /scan
```

## Running the robot

Our installed robot normally starts through `midcmbfol.service`. For a manual test, stop the service first so that only one process owns the hardware:

```bash
sudo systemctl stop midcmbfol.service
source /opt/ros/jazzy/setup.bash
python3 src/tests/perception_web_test.py
```

Activate the robot's existing virtual environment as well if it is not already active. DriveBase control is enabled by default, with the physical start/deadman input gating motion. Check the start input, motor neutral, steering limits and process shutdown with the wheels lifted before ground testing. Calibration suites can also actuate the vehicle.

The installed service uses a wrapper script that selects its environment. The service, wrapper and radio-mode helper source files are not included, so automatic boot setup is not reproduced by the commands above. The [runbook](operations/runbook.md) describes operation on the existing installation.

## Checks without robot hardware

Run the repository checks with Python 3.12 or newer:

```bash
python3 tools/check_repository.py
```

They do not import the robot runner or access devices. For the color-classifier test, install NumPy and OpenCV in a desktop test environment and run from the repository root:

```bash
python3 -m src.tests.cammanager_classifier_synthetic
```

This tests the classifier with synthetic images and a substitute libcamera module. Exposure, capture timing and recognition on the real field still require camera tests.

## Competition configuration

Use the physical start procedure and verify Wi-Fi/Bluetooth are disabled for an autonomous scoring run. The existing `competition-mode` helper is specific to the installed robot; its behaviour and commands are described in the runbook, but its source is not included. Select competition settings before placement and do not use the web interface or remote commands during a scoring run.
