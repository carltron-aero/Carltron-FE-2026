# Competition Runbook

This file is for field use. It covers how to start/stop the robot program, switch radio modes, choose the web UI mode, and recover enough information without internet access.

## Current Default Run

The normal competition program is:

```bash
python3 src/tests/perception_web_test.py
```

Current defaults already include the tuned values for LiDAR processing, OTOS scalars/offsets, LiDAR correction, race speeds, unpark, smart parking, collision supervisor, adaptive corridor lookahead, and PWM file descriptor caching. Long commands that only repeat those values are usually unnecessary.

Only pass flags when you intentionally want to override the current default.

## Systemd Service

The service is `midcmbfol.service`.

Status:

```bash
systemctl status midcmbfol.service --no-pager
```

Start/stop/restart:

```bash
sudo systemctl start midcmbfol.service
sudo systemctl stop midcmbfol.service
sudo systemctl restart midcmbfol.service
```

Follow logs:

```bash
journalctl -u midcmbfol.service -f
```

Enable autostart at boot:

```bash
sudo systemctl enable midcmbfol.service
```

Enable and start immediately:

```bash
sudo systemctl enable --now midcmbfol.service
```

Disable autostart:

```bash
sudo systemctl disable midcmbfol.service
```

The installed service currently uses:

```text
WorkingDirectory=/home/carl/fe-26
ExecStart=/usr/local/bin/run_midcmbfol.sh
```

The wrapper script activates the robot venv, sources ROS Jazzy, and executes:

```bash
python3 src/tests/perception_web_test.py
```

## Competition And Development Radio Modes

These are radio modes only. They do not enable or disable `midcmbfol.service`.

Competition mode:

```bash
sudo competition-mode
```

What it does:

- enables `competition-radio-off.service`
- runs `nmcli radio wifi off`
- runs `rfkill block wifi`
- runs `rfkill block bluetooth`
- prints `rfkill list`

Use this before competition if radios must stay off across reboot.

Development mode:

```bash
sudo dev-mode
```

What it does:

- disables `competition-radio-off.service`
- runs `nmcli radio wifi on`
- runs `rfkill unblock wifi`
- runs `rfkill unblock bluetooth`
- prints `rfkill list` and `nmcli radio`

Check current radio state:

```bash
rfkill list
nmcli radio
systemctl is-enabled competition-radio-off.service
```

Warning: if you are connected over Wi-Fi, `sudo competition-mode` can cut your connection.

## Web UI Modes

Default portal:

```text
http://<robot-ip>:8080/
```

Portal behavior:

- Start Lite WebUI: enables Lite stream and disables Full stream.
- Start Full WebUI: enables Full stream and disables Lite stream.
- Camera Preview: toggles `/camera.jpg` generation separately.

Recommended for real runs:

- Use Lite WebUI when you need live map/status.
- Avoid Full WebUI during ranking or high-speed testing.
- Avoid camera preview during ranking unless you intentionally accept timing overhead.

Relevant flags:

```bash
--port 8080
--stream-rate 25.0
--web-payload-refresh-hz 10.0
--camera-web-debug
--camera-web-debug-fps 4.0
--camera-web-debug-scale 0.55
--camera-web-debug-quality 72
```

For demo screen recording with less performance impact:

```bash
--stream-rate 8 \
--web-payload-refresh-hz 2 \
--camera-web-debug-fps 2 \
--camera-web-debug-scale 0.40 \
--camera-web-debug-quality 55
```

Do not use Full WebUI just because the service is running. Full WebUI has previously caused stale command speed caps during fast runs.

## Basic Field Workflow

Before a competition run:

1. Stop manual test processes.
2. Enable autostart if wanted:

   ```bash
   sudo systemctl enable midcmbfol.service
   ```

3. Switch radios off if required:

   ```bash
   sudo competition-mode
   ```

4. Reboot or start service:

   ```bash
   sudo systemctl restart midcmbfol.service
   ```

5. Check status:

   ```bash
   systemctl status midcmbfol.service --no-pager
   ```

6. Use the physical start/deadman flow. The default deadman source is `physical`.

For development after competition:

```bash
sudo dev-mode
sudo systemctl stop midcmbfol.service
```

## Logs

Current run logs are under:

```text
logs/latest/
```

Each run has a timestamped directory:

```text
logs/YYYY-MM-DD_HH-MM-SS/
```

Find the exact command/flags used:

```bash
cat logs/latest/run_manifest.json
```

Check log folder size:

```bash
du -sb logs
```

Important CSVs:

- `drive_debug.csv`: drive loop, navigation, speed caps, holds, parking, collision supervisor, LiDAR correction summary.
- `lidar_scan_debug.csv`: LiDAR processing and timing details.
- `camera_color_debug.csv`: camera color classifier details.
- `obstacle_slot_debug.csv`: obstacle slot detection evidence.

Do not delete recent logs before extracting the manifest and any run data you may need later.
