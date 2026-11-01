# Testing and development

After the German final, we wanted to make the robot faster while keeping it reliable. We already had a working drivetrain and localisation system, but we didn't know precisely where their limits were. Before changing the motor or increasing the speed, we wanted to measure how the robot actually moved and compare that with what its sensors reported.

For this we built our [Optical Motion Tracking](../tracking/README.md) setup. It gives us a position reference separate from the sensors on the robot. We then added sensor recording and automated test sequences, so we could repeat the same measurements after a hardware or software change. Having the original recordings also means we can try another analysis afterwards without having to drive again.

These pages describe that work up to 27 October 2026. The code in `src/prod` is still the version from the German final. We have tested the replacement motor and newer controllers separately, and haven't yet integrated them into that competition version.

## Test setup

In the first tracking comparisons, the stationary measurements looked promising, but the moving trajectories differed much more. Part of the problem was timing. A camera image reaches the PC after it was exposed, and the robot records its measurements on a different clock. We therefore had to look at the measuring setup itself before using it to judge the robot's localisation.

As we extended the tests, we also started using tracking during the measurement sequence. The program can check the robot's endpoint and correct the return to its starting area. For the faster straight runs, tracking also measures how far the robot has travelled. This became important once we found that OTOS could underestimate the distance at speed.

| Topic | What we tested |
| --- | --- |
| [Test setup and recording](measurement-system.md) | Recording the sensors and cameras together, matching their timestamps and repeating tests. |
| [Motor and drivetrain tests](drivetrain.md) | Speed, starting behaviour, slow driving and braking before and after replacing the motor. |
| [OTOS testing](otos.md) | Read rate, missing movement, sensor replacement and mounting heights of 10, 14 and 20 mm. |
| [Camera localisation](camera-localisation.md) | Finding wall features and trying learned position estimates from the same images. |
| [Localisation and driving tests](localisation-development.md) | Combining OTOS with LiDAR and testing the result over several laps. |

## Results so far

One of the more useful findings was that we were dealing with two different OTOS problems. Reading the encoder put enough load on the program to reduce the OTOS read rate. Disabling encoder acquisition brought it back to about 150 Hz, but the sensor could still miss movement. We then compared the spare sensor at different heights and got substantially better straight-line measurements with its window 20 mm above the floor. We kept that mount for the following tests.

The lap tests then let us compare localisation over a longer route. The robot could follow the centreline quite well even while the OTOS-only position drifted away from the optical reference. Our experimental OTOS/LiDAR fusion reduced that difference in the recorded runs. We still need more testing before relying on it across the full range of routes and speeds, especially since timing affects the measured error and some runs ended in protective stops. Reaching 2 m/s through the field is still our goal.

We have included selected results and nine figures with these pages. The [measurement data and analysis guide](REPRODUCING.md) lists the runs, explains the evaluation methods and shows how to generate the figures. The large original recordings stay outside Git.
