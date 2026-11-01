# Measurement data and analysis

We have included the data used for the figures in [Testing and development](README.md) under [`tools/experiments/data`](../../tools/experiments/data/). This lets you generate the figures from the same selected results. The full videos and native sensor recordings remain in our separate development archive, which is also needed to repeat marker detection or run the newer experimental robot controller.

## Generating the figures

From the repository root, with Python 3.10 or later, create an isolated environment:

```bash
python -m venv .venv-plots
```

Activate it with `source .venv-plots/bin/activate` on Linux/macOS, or `.\.venv-plots\Scripts\Activate.ps1` in Windows PowerShell. Then run:

```bash
python -m pip install -r tools/experiments/requirements.txt
python tools/experiments/render_figures.py --verify-only
python tools/experiments/render_figures.py
```

The script checks the data files against [`manifest.json`](../../tools/experiments/data/manifest.json) before plotting and runs entirely offline. By default, it replaces the nine generated PNG figures in `docs/testing/images`. Use `--output PATH` to save them elsewhere. Different font or library versions can change the PNG files even when the plotted data is the same.

## Test runs

| Data | Runs and evaluation |
| --- | --- |
| [Original motor floor sweep](../../tools/experiments/data/original-motor-floor.json) | 51 forward duty observations, including 50 accepted plateaus. Fixed window: start + max(0.7 s, 40% of active duration) to last active sample −0.35 s. Robust joint XY optical fit, at least eight frames over 0.3 s. No fastest-window search or battery correction. |
| [Raised motors](../../tools/experiments/data/raised-motors.json) | Original motor `motor-functional-20261024-004655` and installed Pololu `motor-functional-20261024-024102`. Forward and reverse late-window results, with quadrature validity. These are unloaded gearbox-speed conversions. |
| [First ground breakaway](../../tools/experiments/data/initial-breakaway.json) | `breakaway-03443b8eb4b4`: 11 ascending 50–60% trials. Four no-motion timeouts remain in the result. |
| [Start/hold comparison](../../tools/experiments/data/breakaway.json) | `breakaway-607dd218c9f2`: 22 paired trials at 53–54%. `breakaway-1bfa4a86528d`: 11 lower-duty hold trials at 45–49%. Optical speed fits use fixed target-phase windows, not initial drive or brake samples. |
| [Drive/coast and drive/brake](../../tools/experiments/data/drive-coast-brake.json) | `pwm-compare-03c0875e94b6`: ten legs, one per mode at 18, 15, 10, 8 and 5 cm/s. Three sensor summaries per leg. Target band is ±max(1 cm/s, 10% of target). Adjacent samples are not independent trials. |
| [Braking](../../tools/experiments/data/braking.json) | `brake-comparison-6ed697cf4d7c`: seven trials grouped by 70/75% duty and full/pulsed profile. Group values are medians. The 75% pulsed group has only one trial. |
| [Target speed and battery](../../tools/experiments/data/target-speed.json) | Fresh-battery `target-speed-023e07ee1b45` paired with the previous battery's test using identical speed and mount profiles. Six legs with four primary windows. The two unsupported windows are not filled with post-hoc substitutes. |
| [Encoder on/off](../../tools/experiments/data/encoder-ab.json) | `encoder-ab-59ecc9a060a3`: four floor legs at 80%, ordered on/off/off/on. The last leg stopped at a tracking boundary check. All four returned. These data come from the floor drives, excluding the earlier rehearsal without actuation. |
| [Rotated original sensor](../../tools/experiments/data/rotated-mount.json) | `mounting-sweep-5535ed47de85`: interrupted 45° campaign, with the changed lever arm recorded. Orientation, position and floor track changed together. |
| [Window heights](../../tools/experiments/data/mounting-heights.json) | Replacement sensor after wheel repair: 14 mm composite, 10 mm `mounting-sweep-d3f186b73d6f`, 20 mm `mounting-sweep-cd40e88c646d`. Full analysed sets have 21/20/20 attempts. The matched comparison uses 18 per height at 55–95%. |
| [Camera modes](../../tools/experiments/data/camera-modes.json) | Separate stationary binned/full samples at the same nominal position. The samples weren't taken simultaneously and only compare image detail. |
| [Learned camera position](../../tools/experiments/data/camera-learning.json) | Same 162 held-out straight images across three validation-selected learned variants. Geometric observations accept 47. The export retains missing counts, axis errors and the all-eligible denominator. |
| [Curve pilots](../../tools/experiments/data/curve-pilots.json) | `curve-pilot-f02b4f1b3a7a`: four completed 1.0–1.3 m/s requested laps. `curve-pilot-c05282da0704`: three completed 1.1–1.3 m/s laps, partial 1.4 m/s stage. |
| [Fusion development](../../tools/experiments/data/fusion-development.json), [recovery replay](../../tools/experiments/data/recovery-replay.json), [stop diagnosis](../../tools/experiments/data/curve-stop.json) | Exploratory model comparison, replay of already inspected recordings and reconstructed stopping branches. None is an additional physical drive. |

The manifest lists SHA-256 hashes for the source reports and included data files, so the analysis versions used here can be identified. It doesn't include the raw recordings. The competition source and first-run tracking tools remain in their own directories.

## Coordinates and timing

We compare positions at the rear axle midpoint. The optical calculation applies the marker's offset in the measured heading direction and an approximate height correction. Field X/Y are in metres, with headings and sensor transforms given in the relevant profile. Along-track displacement is projected onto the initial direction of a straight leg, so it measures only one component of the planar difference.

In the height comparison, the sensor is 157.7 mm forward and 16.1 mm right of the axle midpoint. Its configured orientation is 0.9°, with unit linear and angular scale. We measure height from the sensor window to the floor. The mounting dimensions come from mechanical and CAD measurements, without a survey of the optical centres.

For the mounting analysis, we kept the empirically estimated optical delay at 120 ms and varied it by ±20 ms to check timing sensitivity. The start/hold study used a separate 60 ms estimate from its return legs. These estimates belong to different recording sessions. Clock exchanges don't measure the exposure and buffering delays. We also kept the original optical mapping and timing for the curve results, without refitting the spatial map to improve agreement with the robot.

To compare missed motion during powered driving, we use intervals ending at least 80 ms before the brake decision. A severe episode requires two or more adjacent overlapping windows spanning at least 160 ms, with less than 80% captured displacement after low-speed normalisation, under all three timing variants. The low-speed gain is the median of the first 55 and 60% stationary endpoint ratios. We only use this correction for that diagnostic, leaving the raw sensor and unscaled figure unchanged. Speed-bin summaries weight legs equally because overlapping windows aren't independent drives.

## Interpreting the results

The median and 95th percentile describe the recorded observations, rather than a guaranteed error bound for future runs. We identify failed and shortened attempts separately. Their stationary endpoints can still be useful for comparing displacement, even when the driving test stopped early. A recording that closed correctly isn't automatically a successful drive.

Our Optical Motion Tracking reference has both spatial and timing uncertainty. We fit its spatial mapping from fixed markers, while the initial position and motion-based timing estimates affect the independence of the comparisons as described on the topic pages. These tests don't establish whole-field dynamic millimetre accuracy, one universal sensor failure speed or high-speed competition reliability.
