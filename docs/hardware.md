# Hardware baseline

The robot combines front-wheel steering and rear-wheel drive with a custom main board. This repository starts from the German-final vehicle rather than a newly assembled international version.

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

The exact motor ordering variant, encoder resolution, wheel diameter, gearing, servo model, connector pinouts, and battery limits must be checked against the assembled robot and final bill of materials. The current `DriveConfig` includes a 60/21 motor-to-wheel reduction and a 43.2 mm wheel diameter; these are software configuration values, not independently certified mechanical measurements.

The current PWM mapping is channel 0 for steering, channel 1 for an LED output, channel 2 for reverse drive, and channel 3 for forward drive. `hw_pwm.py` uses `pwmchip0`. The encoder configuration uses `/dev/gpiochip4`, lines 20 and 21. These values are tied to the existing Pi/board setup and must not be copied onto differently wired hardware without verification.

The camera's circular image region is used for obstacle colors. Its mounting height must preserve useful views of the surrounding field walls and obstacles. OTOS height, optical-window cleanliness, orientation, and scale factors are calibration-sensitive. LiDAR timing and the relative positions of all sensors must be checked after any mechanical change.

Mechanical files belong in [models](../models/), electrical drawings in [schemes](../schemes/), and final vehicle views in [v-photos](../v-photos/). These files are not yet present in this first version. Motor upgrades, additional odometry sensors, and alternative cameras remain proposed changes rather than the hardware baseline documented here.
