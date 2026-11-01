# First Driving Comparison with Optical Motion Tracking

This example comes from the paired recording on 5 October 2026. Robot log calendar dates were wrong, so the analysis used monotonic times and observed LED events rather than the folder date.

`replay.json` contains the complete analyzed position traces for the approximately 23 second driving interval. Ten original JPEG keyframes are included in `frames/`. They are a sparse selection, not a complete video. Missing detections in the observations remain missing.

[frames-manifest.json](frames-manifest.json) records the selected source frame indexes, estimated times and hashes of the original JPEG bytes. The raw recordings are retained locally, outside Git.

`results.json` records one held-out static placement and the stationary endpoint displacement comparison. It includes measurement uncertainty and limitations. Neither camera agreement nor fitted residuals establish independently verified whole-field or dynamic accuracy.

From the repository root, run `python tools/tracking/run-replay/server.py`, then open `http://127.0.0.1:8770`.
