# Optical Motion Tracking Tools

The [optical motion tracking guide](../../docs/tracking/README.md) explains the system design, marker geometry, field registration and initial validation results. The [setup and usage guide](../../docs/tracking/USAGE.md) contains the installation and analysis commands.

- `camera-control/` is the Windows camera controller used for exposure, focus, zoom and PTZ settings.
- `track_robot_marker_video.py` detects and reviews a saved clip, with marker-centre position and speed overlays.
- `render_marker_frame.py` outlines all genuinely decoded markers in a saved image, with a detection report and optional recorded-map coordinates.
- `plane_mapping.py` contains the empirical floor mapping and distortion functions extracted from the calibration experiments.
- `adapt_run_registration.py` re-registers recorded views using stationary field tags, not the moving robot.
- `height_geometry.py` handles raised-marker parallax and the recorded map representation.
- `run-replay/` compares camera observations with fused and OTOS-only robot logs.
- `examples/first-run/` contains complete analyzed position traces and a small set of inspected original images.
- `profiles/` contains the marker size and example field, camera and mapping geometry. These are not universal calibrations for a different setup.

All analysis is offline. The camera controller only accesses hardware when explicitly started. These tools run on the development PC and are separate from the robot's competition software.
