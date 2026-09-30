# Troubleshooting

Use this when something fails at the field and you need a fast local check.

## Program Is Not Running

Check:

```bash
systemctl status midcmbfol.service --no-pager
```

Start:

```bash
sudo systemctl start midcmbfol.service
```

Restart:

```bash
sudo systemctl restart midcmbfol.service
```

Follow logs:

```bash
journalctl -u midcmbfol.service -f
```

If you changed `/etc/systemd/system/midcmbfol.service`:

```bash
sudo systemctl daemon-reload
sudo systemctl restart midcmbfol.service
```

## Autostart Does Not Work

Check if enabled:

```bash
systemctl is-enabled midcmbfol.service
```

Enable:

```bash
sudo systemctl enable midcmbfol.service
```

Enable and start now:

```bash
sudo systemctl enable --now midcmbfol.service
```

Remember: `competition-mode` and `dev-mode` do not control program autostart. They control radios.

## No Wi-Fi Or Bluetooth

Check:

```bash
rfkill list
nmcli radio
systemctl is-enabled competition-radio-off.service
```

Development mode:

```bash
sudo dev-mode
```

Competition radio-off mode:

```bash
sudo competition-mode
```

If `competition-radio-off.service` is enabled, radios will be disabled again on boot.

## Web UI Not Reachable

Check program:

```bash
systemctl status midcmbfol.service --no-pager
```

Default URL:

```text
http://<robot-ip>:8080/
```

If you started a manual run with a different port:

```bash
python3 src/tests/perception_web_test.py --port 8081
```

If Wi-Fi is off because of competition mode, the web UI is only reachable over whatever wired/local network access is still available.

## Robot Is Slower With Web UI

Likely cause: stale-command speed cap. Full WebUI and camera preview can add enough scheduling pressure to make drive commands older than the default `80 ms` threshold.

Use Lite WebUI for real runs. Avoid Full WebUI and camera preview during ranking.

Demo-only reduced-rate flags:

```bash
--stream-rate 8 \
--web-payload-refresh-hz 2 \
--camera-web-debug-fps 2 \
--camera-web-debug-scale 0.40 \
--camera-web-debug-quality 55
```

Do not disable the stale-command cap for competition.

## Need The Exact Command Used For A Run

Latest:

```bash
cat logs/latest/run_manifest.json
```

Specific run:

```bash
cat logs/YYYY-MM-DD_HH-MM-SS/run_manifest.json
```

The `argv` field lists flags passed to `perception_web_test.py`.

## Logs Folder Too Large

Check size:

```bash
du -sb logs
du -sh logs
```

List newest runs:

```bash
ls -lt logs | head -30
```

Do not delete logs while investigating a failure. If you need space, first copy or note the run directories that contain useful data.

## Check Whether A Flag Is A Default

Fast practical check:

```bash
python3 src/tests/perception_web_test.py --help | less
```

For the currently important defaults, use:

- [Main script flags](main-script-flags.md)

If a flag in an old command matches the default in that file, it can be omitted.

## Sensor Or Service Conflicts During Calibration

Before running calibration scripts that command motors:

```bash
sudo systemctl stop midcmbfol.service
```

After calibration:

```bash
sudo systemctl start midcmbfol.service
```

If two programs try to access hardware at the same time, stop the service and check for extra Python processes.

## Localization Holds Or Collision Brakes Trigger

Use `drive_debug.csv` in the relevant run. Useful fields include:

- `localization_hold_active`
- `localization_hold_reason`
- `localization_hold_release_reason`
- `otos_jump_detected`
- `otos_jump_reason`
- `collision_lidar_active_brake_triggered`
- `collision_lidar_trigger_reason`
- `collision_recovery_state`
- `nav_pose_confidence`
- `lidar_correction_axis_effective_max_error_m`

The current default behavior is to hold/recover rather than driving blindly. If it happens once after a real pose jump or hazard, the system may be working as intended. If it repeats often in normal driving, inspect before tuning.

## Common Emergency Actions

Stop service:

```bash
sudo systemctl stop midcmbfol.service
```

Disable autostart:

```bash
sudo systemctl disable midcmbfol.service
```

Restore development radio state:

```bash
sudo dev-mode
```

Restart from clean service:

```bash
sudo systemctl restart midcmbfol.service
```
