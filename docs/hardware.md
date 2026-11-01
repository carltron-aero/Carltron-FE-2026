# Hardware baseline

Our German-final robot uses front-wheel steering and rear-wheel drive. We connect power and actuator interfaces through our custom main board, which also determines where several components and cables can fit. We kept this working hardware as the starting point for international development, so we could test drivetrain changes without replacing the electronics at the same time.

| Subsystem | Baseline |
| --- | --- |
| Main computer | Raspberry Pi 5 running Ubuntu 24.04. |
| Power | Two LP-E6-format batteries in series and the existing integrated power-management arrangement. |
| Drive motor | Current 25GA-371-family brushed gearmotor with encoder. |
| Motor driver | DRV8871 module mounted on the custom main board. |
| Steering | Servo controlled through the Pi hardware PWM interface. |
| Relative motion | SparkFun Qwiic OTOS mounted facing the floor. |
| Field geometry | Slamtec RPLIDAR S3. |
| Color perception | Waveshare IMX219-200 with Arducam LN010 M12 fisheye, facing upward. |
| Wheel motion feedback | Quadrature encoder read through GPIO. |

The current `DriveConfig` uses a 60/21 motor-to-wheel reduction and a 43.2 mm wheel diameter. These are the dimensions assumed by the speed and encoder conversion code. A complete bill of materials, including the exact motor ordering variant, encoder resolution, servo model, connector pinouts and battery limits, is not included in this revision.

The current PWM mapping is channel 0 for steering, channel 1 for an LED output, channel 2 for reverse drive, and channel 3 for forward drive. `hw_pwm.py` uses `pwmchip0`. The encoder configuration uses `/dev/gpiochip4`, lines 20 and 21. These values are tied to the existing Pi/board setup and must not be copied onto differently wired hardware without verification.

We positioned the camera to see obstacle colours around the robot while keeping its viewpoint below the wall tops. OTOS needs a clear view of the floor, with height, orientation, scale calibration and dust on the window all affecting its readings. We include each sensor's mounting offset in the position calculation and use the odometry history to account for movement during a LiDAR scan.

The [models](../models/), [schemes](../schemes/) and [vehicle photograph](../v-photos/) folders don't yet contain the corresponding assets. The table above describes the German-final hardware used by the included program. We later installed the Pololu #4841 and tested a replacement OTOS at several heights, as described in the [drivetrain](testing/drivetrain.md) and [OTOS tests](testing/otos.md). The defaults in this source tree still describe the earlier configuration.
