# International preparation

This repository starts with the working German-final software. Changes for the International Final must be implemented, measured, and documented before they are described as completed capabilities.

## Functional priorities

1. Replace the existing finishing maneuver with full parallel parking. Verify footprint containment, wall parallelism, and clearance from both parking barriers.
2. Test Open and Obstacle Challenge behavior across legal layouts, starts, and travel directions. Keep controlled tests distinct from competition results.
3. Characterize localization error, measurement latency, OTOS jumps/contamination, and LiDAR bias. Validate recovery behavior instead of assuming every sensor remains healthy.
4. Establish safe performance limits for the actual drivetrain and control stack. Measure achieved speed and complete challenge time, including parking.
5. Verify the physical start procedure, radio state, dimensions, mass, and final hardware configuration.

The working reliability objective is autonomous completion across realistic conditions. A small set of successful runs does not establish a 99.9% success probability. Record test conditions, software revisions, sample counts, failures, and recovery actions so that reliability claims have a clear evidence base.

## MathWorks modeling project

The planned project uses a measured robot model to improve parallel parking and navigation robustness. Vehicle parameters, sensor error and delay, scenario design, control integration, and physical verification are intended team contributions. Any supplied field model must retain its attribution and permitted-use terms.

Begin with one complete experiment: identify the model from robot measurements, compare baseline and improved control in simulation, and check the result on the physical vehicle. Expand to randomized layouts and fault cases after that cycle works. The current Python runtime need not be replaced simply to conduct modeling and verification.

Future repository additions should include readable model files, parameter-identification scripts, selected test data, reproducible scenario settings, quantitative plots, limitations, and a simulation-to-real demonstration. The modeling project is planned; no completed award submission or validated simulation is included in this initial baseline.

## Rules and documentation

Use the [official 2026 season material](https://wro-association.org/competition/2026-season/) and [official Q&A](https://wro-association.org/competition/questions-answers/). Recheck amendments and any surprise rule before final design decisions and travel. The German-final implementation is historical evidence, not authority for international interpretations.

The first repository milestone is dated 30 September 2026. Subsequent work should use genuine commits that connect changes to their test evidence. Before the competition submission, complete the [publication checklist](publication-checklist.md) and confirm the organizer's exact deadlines and submission channels.
