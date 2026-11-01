"""Offline full-video robot-target tracking with provisional floor XY and speed.

No live camera access. Fixed references alone fit the clip-specific planar map;
the moving target never trains it. Container time is not camera exposure time.
Raised-target height compensation and calibrated 3D pose are not implemented.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
import cv2
import imageio_ffmpeg
import numpy as np

from plane_mapping import (
    fit, invert, project, PIXEL_CENTRE, PIXEL_SCALE, WORLD_CENTRE, WORLD_SCALE,
)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


def lines(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def file_hash(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def recording_context(path=None):
    """Never inherit exposure, camera identity or target height from another clip."""
    context = {'physical_camera_user_reported': None,
               'optical_settings_user_reported': {},
               'target_height_user_reported_mm': None}
    if path is not None:
        supplied = json.loads(path.read_text(encoding='utf-8'))
        forbidden = {'input_video', 'frames', 'mapping', 'profile', 'marker_id',
                     'target_height_compensated', 'all_recorded_frames_processed', 'live_camera_access'}
        if not isinstance(supplied, dict) or forbidden.intersection(supplied):
            raise ValueError('Recording context must contain metadata only, not analysis results.')
        context.update(supplied)
        context['recording_context_source'] = str(path.resolve())
    return context


def presentation_times(path, log_path, ignore_editlist=False):
    """Audit native decoded PTS without resampling, including the final gap."""
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-nostdin', '-copyts']
    if ignore_editlist:
        command += ['-ignore_editlist', '1']
    command += ['-i', str(path), '-vf', 'showinfo', '-an', '-fps_mode', 'passthrough', '-f', 'null', '-']
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                               text=True, encoding='utf-8', errors='replace')
    base, pts, durations = None, [], []
    with log_path.open('w', encoding='utf-8') as log:
        for line in process.stderr:
            log.write(line)
            found = re.search(r'config in time_base:\s*(\d+)/(\d+)', line)
            if found:
                base = [int(found[1]), int(found[2])]
            found = re.search(r'\bn:\s*(\d+)\s+pts:\s*(-?\d+).*?duration:\s*(\d+)', line)
            if found:
                if int(found[1]) != len(pts):
                    raise ValueError('Unexpected PTS frame sequence')
                pts.append(int(found[2]))
                durations.append(int(found[3]))
    if process.wait() or base is None or not pts:
        raise RuntimeError('Native presentation timestamp audit failed')
    return {'time_base': base, 'pts': pts, 'durations': durations}


def finalize_metadata(args):
    records = lines(args.output / 'detections.jsonl')
    cap = cv2.VideoCapture(str(args.video))
    width, height = int(cap.get(3)), int(cap.get(4))
    fps, expected = cap.get(cv2.CAP_PROP_FPS), int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    timing = presentation_times(args.video, args.output / 'source-pts-audit.log')
    if len(records) != expected or len(timing['pts']) != expected:
        raise ValueError('Detection/decoded PTS frame counts do not agree')
    base = timing['time_base'][0] / timing['time_base'][1]
    times = np.asarray(timing['pts']) * base
    if np.min(np.diff(times)) <= 0:
        raise ValueError('Source PTS are not strictly increasing')
    opencv_times = np.asarray([r['time_s'] for r in records])
    if np.max(abs(times-opencv_times)) > base:
        raise ValueError('OpenCV detections cannot be aligned with source PTS')
    duration = float(times[-1] + timing['durations'][-1]*base)
    changes = [r['mean_image_change_gray'] for r in records if r['mean_image_change_gray'] is not None]
    save(args.output / 'video-metadata.json', {'input_video': str(args.video.resolve()),
        'width': width, 'height': height, 'fps': fps, 'frames': expected, 'duration_s': duration,
        'source_presentation_timestamps': timing, 'decoded_presentation_times_strictly_increasing': True,
        'max_container_vs_nominal_time_difference_s': float(np.max(abs(times-np.arange(expected)/fps))),
        'max_opencv_vs_native_pts_difference_s': float(np.max(abs(times-opencv_times))),
        'all_recorded_frames_processed': True,
        'image_change_gray_percentiles': np.percentile(changes, [0, 10, 50, 90, 100]).tolist(),
        'qualification': 'Native decoded MP4 presentation timestamps preserved, including gaps. These are not physical camera exposure timestamps.'})


def timed_spans(mask, times, end_time):
    result, start = [], None
    for index, active in enumerate(list(mask)+[False]):
        if active and start is None:
            start = index
        elif not active and start is not None:
            end = times[index] if index < len(times) else end_time
            result.append({'start_s': float(times[start]), 'end_s': float(end),
                           'duration_s': float(end-times[start]), 'frames': index-start})
            start = None
    return result


def make_detector(expanded=False):
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMax = 71 if expanded else 35
    parameters.adaptiveThreshWinSizeStep = 7
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), parameters)


def decoded(detector, gray):
    corners, identifiers, _ = detector.detectMarkers(gray)
    matches = defaultdict(list)
    for identifier, quad in zip([] if identifiers is None else identifiers.flatten(), corners):
        matches[int(identifier)].append(quad.reshape(4, 2))
    # An ID that occurs more than once is ambiguous, not a measurement.
    return {key: value[0] for key, value in matches.items() if len(value) == 1}, [
        key for key, value in matches.items() if len(value) > 1]


def intersection_center(quad):
    """The intersection of diagonals, not an image-space corner average."""
    quad = np.asarray(quad, dtype=float)
    a = np.r_[quad[0], 1.]
    b = np.r_[quad[2], 1.]
    c = np.r_[quad[1], 1.]
    d = np.r_[quad[3], 1.]
    point = np.cross(np.cross(a, b), np.cross(c, d))
    if not np.isfinite(point).all() or abs(point[2]) < 1e-12:
        raise ValueError('Degenerate marker diagonals')
    return point[:2] / point[2]


def detect_video(args):
    args.output.mkdir(parents=True, exist_ok=True)
    destination = args.output / 'detections.jsonl'
    if destination.exists():
        raise FileExistsError('Saved detections already exist; use analyse/render or a new directory.')
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError('Cannot open source recording')
    width, height = int(cap.get(3)), int(cap.get(4))
    fps, expected = cap.get(cv2.CAP_PROP_FPS), int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if width / height != 16 / 9 or fps <= 0:
        raise ValueError('This clip-specific mapping expects an uncropped 16:9 camera view.')
    ordinary, expanded = make_detector(), make_detector(True)
    clahe = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))
    count, last_target, previous_gray = 0, None, None
    timestamps, small_changes = [], []
    started = time.perf_counter()
    try:
        with destination.open('x', encoding='utf-8') as output:
            while True:
                okay, frame = cap.read()
                if not okay:
                    break
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                timestamp = float(cap.get(cv2.CAP_PROP_POS_MSEC)) / 1000.
                if not math.isfinite(timestamp):
                    raise ValueError('Nonfinite container presentation time')
                timestamps.append(timestamp)
                hits, ambiguous = decoded(ordinary, gray)
                method = 'aruco-subpixel' if args.marker_id in hits else None
                if args.marker_id not in hits and args.marker_id not in ambiguous:
                    if last_target is not None and timestamp - last_target[0] < 1.:
                        quad = last_target[1]
                        lo = np.maximum(0, np.floor(quad.min(axis=0) - 260)).astype(int)
                        hi = np.minimum([width, height], np.ceil(quad.max(axis=0) + 260)).astype(int)
                        offset = lo
                        roi = gray[lo[1]:hi[1], lo[0]:hi[0]]
                    else:
                        offset, roi = np.array([0, 0]), gray
                    extra, duplicate = decoded(expanded, roi)
                    if args.marker_id not in extra and args.marker_id not in duplicate:
                        extra, duplicate = decoded(expanded, clahe.apply(roi))
                        recovery = 'aruco-local-contrast'
                    else:
                        recovery = 'aruco-expanded-threshold'
                    if args.marker_id in extra:
                        hits[args.marker_id] = extra[args.marker_id] + offset
                        method = recovery
                    elif args.marker_id in duplicate:
                        ambiguous.append(args.marker_id)
                if args.marker_id in hits:
                    last_target = timestamp, hits[args.marker_id]
                small = cv2.resize(gray, (320, 180), interpolation=cv2.INTER_AREA)
                change = None if previous_gray is None else float(np.mean(cv2.absdiff(small, previous_gray)))
                if change is not None:
                    small_changes.append(change)
                previous_gray = small
                record = {'frame': count, 'time_s': timestamp, 'nominal_frame_time_s': count / fps,
                          'ids': sorted(hits), 'ambiguous_ids': ambiguous,
                          'corners_px': {str(i): q.tolist() for i, q in hits.items()},
                          'target_method': method, 'mean_image_change_gray': change,
                          'near_unchanged_recorded_image': change is not None and change < .025}
                output.write(json.dumps(record, separators=(',', ':')) + '\n')
                count += 1
                if count % 300 == 0:
                    print(f'DETECT {count}/{expected} | video {timestamp:.1f}s | elapsed {time.perf_counter()-started:.1f}s', flush=True)
    finally:
        cap.release()
    if count != expected:
        raise RuntimeError(f'Decoded {count} frames, metadata states {expected}.')
    times = np.asarray(timestamps)
    if count > 1 and np.min(np.diff(times)) <= 0:
        raise ValueError('Container times are not strictly increasing.')
    finalize_metadata(args)


def speeds(tracks, window_s=.15, reset_gap_s=.075):
    history, last, results = deque(), None, []
    for row in tracks:
        now, current, state = row['time_s'], None, 'not-detected'
        if row['status'] == 'decoded' and not row['shape_warning']:
            if history and now - history[-1][0] > reset_gap_s:
                history.clear()
            history.append((now, row['xy_center_mm']))
            while history and now - history[0][0] > window_s + 1e-9:
                history.popleft()
            state = 'collecting-samples'
            if len(history) >= 5 and now - history[0][0] >= .066 - 1e-9:
                ts = np.asarray([value[0] for value in history])
                xy = np.asarray([value[1] for value in history])
                dt = ts - ts.mean()
                velocity = np.sum(dt[:, None]*(xy-xy.mean(axis=0)), axis=0) / np.dot(dt, dt)
                residual = float(np.sqrt(np.mean(np.sum((xy-xy.mean(axis=0)-dt[:, None]*velocity)**2, axis=1))))
                current = {'time_s': now, 'speed_m_s': float(np.linalg.norm(velocity)/1000),
                           'vx_mm_s': float(velocity[0]), 'vy_mm_s': float(velocity[1]),
                           'fit_rms_mm': residual, 'uncertain': residual > 5.,
                           'window_span_s': now-history[0][0], 'recorded_samples': len(history)}
                last, state = current.copy(), 'estimated'
        elif row['status'] == 'decoded':
            history.clear()
            state = 'shape-warning'
        results.append({'frame': row['frame'], 'time_s': now, 'current': current,
                        'current_status': state, 'last_valid': last.copy() if last else None,
                        'last_valid_age_s': now-last['time_s'] if last else None})
    return results


def analyse(args):
    out = args.output
    records, metadata = lines(out / 'detections.jsonl'), json.loads((out / 'video-metadata.json').read_text())
    timing = metadata['source_presentation_timestamps']
    source_times = np.asarray(timing['pts']) * timing['time_base'][0] / timing['time_base'][1]
    for record, timestamp in zip(records, source_times):
        record['time_s'] = float(timestamp)
    profile = json.loads(args.profile.read_text())
    marker_size = profile['black_code_dimensions_mm'][0]
    if profile['marker_id'] != args.marker_id or profile['black_code_dimensions_mm'][1] != marker_size:
        raise ValueError('Profile does not match square moving target.')
    session = json.loads(args.geometry.read_text())
    geometry = {int(i): np.asarray(q) for i, q in session['fixed_geometry_mm'].items()}
    samples = defaultdict(list)
    for record in records:
        for identifier, quad in record['corners_px'].items():
            if int(identifier) in geometry:
                samples[int(identifier)].append(quad)
    medians = {i: np.median(value, axis=0) for i, value in samples.items() if len(value) >= 30}
    if len(medians) < 4:
        raise RuntimeError('Need four fixed primary tags for this provisional radial map.')
    ids = sorted(medians)
    scale = np.array([1920/metadata['width'], 1080/metadata['height']])
    world = np.concatenate([geometry[i] for i in ids])
    pixels = np.concatenate([medians[i] * scale for i in ids])
    models = []
    for radial_terms in (0, 1, 2):
        parameters = fit(world, pixels, radial_terms)
        prediction = project((world-WORLD_CENTRE)/WORLD_SCALE, parameters)*PIXEL_SCALE+PIXEL_CENTRE
        errors = invert(pixels, parameters)-world
        omitted = {}
        for identifier in ids:
            retained = [i for i in ids if i != identifier]
            loo = fit(np.concatenate([geometry[i] for i in retained]),
                      np.concatenate([medians[i]*scale for i in retained]), radial_terms)
            displacement = invert(medians[identifier]*scale, loo)-geometry[identifier]
            omitted[str(identifier)] = {'rms_mm': float(np.sqrt(np.mean(np.sum(displacement**2, axis=1)))),
                                        'centre_offset_xy_mm': displacement.mean(axis=0).tolist()}
        models.append({'radial_terms': radial_terms, 'parameters': parameters.tolist(),
                       'training_rms_pixels_at_1920x1080': float(np.sqrt(np.mean(np.sum((prediction-pixels)**2, axis=1)))),
                       'training_rms_mm': float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
                       'leave_one_fixed_tag_out': omitted})
    # Predetermined two-radial-term model, not chosen using target-size or motion residuals.
    selected = models[2]
    parameters = np.asarray(selected['parameters'])
    tracks, drifts, sensitivity = [], [], []
    for record in records:
        values = record['corners_px']
        drift = [np.linalg.norm(np.asarray(values[str(i)])-medians[i], axis=1).max()
                 for i in ids if str(i) in values]
        reference_drift = float(np.median(drift)) if drift else None
        if reference_drift is not None:
            drifts.append(reference_drift)
        row = {'frame': record['frame'], 'time_s': record['time_s'], 'status': 'not-detected',
               'xy_center_mm': None, 'z_mm': None, 'corners_px': None,
               'shape_warning': False, 'reference_drift_px': reference_drift,
               'near_unchanged_recorded_image': record['near_unchanged_recorded_image']}
        if str(args.marker_id) in values:
            quad = np.asarray(values[str(args.marker_id)])
            floor = invert(quad*scale, parameters)
            centre = intersection_center(floor)
            edges = np.linalg.norm(np.roll(floor, -1, axis=0)-floor, axis=1)
            deviation = float(np.max(abs(edges/marker_size-1)))
            warning = deviation > .12
            row.update({'status': 'decoded', 'xy_center_mm': centre.tolist(),
                        'corners_px': quad.tolist(), 'centre_px': intersection_center(quad).tolist(),
                        'floor_projected_corners_mm': floor.tolist(),
                        'target_method': record['target_method'], 'edge_lengths_floor_mm': edges.tolist(),
                        'max_edge_scale_deviation_fraction': deviation, 'shape_warning': warning})
            alternate = intersection_center(invert(quad*scale, np.asarray(models[0]['parameters'])))
            sensitivity.append(float(np.linalg.norm(centre-alternate)))
        tracks.append(row)
    motion = speeds(tracks)
    for name, data in [('id9-track.jsonl', tracks), ('id9-speed.jsonl', motion)]:
        with (out / name).open('w', encoding='utf-8') as output:
            for row in data:
                output.write(json.dumps(row, separators=(',', ':'))+'\n')
    mask = np.asarray([row['status'] == 'decoded' for row in tracks])
    detected = np.flatnonzero(mask)
    active = np.zeros(len(mask), dtype=bool)
    if len(detected):
        active[detected[0]:detected[-1]+1] = True
    counts = Counter(identifier for record in records for identifier in record['ids'])
    reliable_speeds = [row['current'] for row in motion if row['current'] is not None and not row['current']['uncertain']]
    peak = max(reliable_speeds, key=lambda row: row['speed_m_s']) if reliable_speeds else None
    # This guessed intrinsic matrix is for the decorative cube ONLY.
    # It is never used to determine XY, height, or speed.
    camera = [[metadata['width'], 0, metadata['width']/2],
              [0, metadata['width'], metadata['height']/2], [0, 0, 1]]
    summary = {**metadata, 'input_video_sha256': file_hash(args.video),
        'marker_id': args.marker_id, 'profile': profile,
        **recording_context(getattr(args, 'recording_context', None)),
        'target_height_compensated': False,
        'coordinate_origin': 'ID0 near-left physical wall-reference datum in the saved field geometry; not image bottom-left.',
        'coordinate_axes': 'X field-right; Y field-far in the original field overview. Camera2 side view has different screen axes.',
        'reference': 'Centre of the black marker, from diagonals after planar lens correction; no paper/body reference extrapolation.',
        'mapping': {'selected_radial_terms': 2, 'selection_policy': 'Predetermined; moving ID9 excluded from every fit.',
                    'parameters': selected['parameters'], 'fixed_ids': ids, 'pixel_scale_to_1920x1080': scale.tolist(),
                    'fixed_median_corners_px': {str(i): q.tolist() for i, q in medians.items()},
                    'geometry_source': str(args.geometry.resolve()), 'geometry_sha256': hashlib.sha256(args.geometry.read_bytes()).hexdigest(),
                    'qualification': 'Clip-specific empirical floor-plane radial mapping, not physical intrinsics/3D pose or independently surveyed accuracy.',
                    'comparison_models': models},
        'decoded_frames': int(mask.sum()), 'decoded_fraction_full_recording': float(mask.mean()),
        'first_detection_s': tracks[int(detected[0])]['time_s'] if len(detected) else None,
        'last_detection_s': tracks[int(detected[-1])]['time_s'] if len(detected) else None,
        'decoded_fraction_between_first_last': float(mask[active].mean()) if active.any() else None,
        'not_decoded_intervals_between_first_last': timed_spans(active & ~mask, source_times, metadata['duration_s']),
        'not_decoded_intervals_full_recording': timed_spans(~mask, source_times, metadata['duration_s']),
        'loss_intervals_include_absence_or_occlusion': True, 'ID_frame_counts': dict(counts),
        'near_unchanged_recorded_frames': sum(record['near_unchanged_recorded_image'] for record in records),
        'shape_warning_frames': sum(row['shape_warning'] for row in tracks),
        'target_shortest_edge_px_p05_p50_p95': np.percentile([
            np.linalg.norm(np.roll(np.asarray(row['corners_px']), -1, axis=0)-row['corners_px'], axis=1).min()
            for row in tracks if row['status'] == 'decoded'], [5, 50, 95]).tolist() if len(detected) else [],
        'fixed_reference_drift_px_p50_p95_max': [float(np.median(drifts)), float(np.percentile(drifts, 95)), float(max(drifts))] if drifts else [],
        'radial_vs_uncorrected_homography_target_difference_mm_p50_p95_max': [float(np.median(sensitivity)), float(np.percentile(sensitivity, 95)), float(max(sensitivity))] if sensitivity else [],
        'speed_estimation': {'window_s': .15, 'reset_gap_s': .075, 'fit': 'Backward linear least-squares fit of recorded centre coordinates',
                             'qualification': 'Apparent planar speed on MP4 presentation time. Repeated/buffered images, unknown exposure/latency and nonplanarity can bias it. No current speed during missing detections.',
                             'max_nonflagged_fit': peak, 'nonflagged_fit_frames': len(reliable_speeds)},
        'illustrative_cube_camera_matrix': camera,
        'z_conclusion': 'Not measured. Any reported or assumed target height is not a per-frame Z estimate.',
        'accuracy_conclusion': 'Provisional position/speed estimates for reviewing tracking, not certified millimetre accuracy or independent dynamic speed truth.',
        'all_recorded_frames_processed': True, 'live_camera_access': False}
    save(out / 'analysis-summary.json', summary)
    print(json.dumps({key: summary[key] for key in ['frames', 'duration_s', 'decoded_frames',
        'first_detection_s', 'last_detection_s', 'decoded_fraction_between_first_last',
        'not_decoded_intervals_between_first_last', 'shape_warning_frames', 'speed_estimation']}, indent=2), flush=True)


def label(frame, text, point, scale=.6, color=(235, 240, 245), thickness=1):
    cv2.putText(frame, text, tuple(map(int, point)), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (12, 15, 20), thickness+2, cv2.LINE_AA)
    cv2.putText(frame, text, tuple(map(int, point)), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def annotate(frame, row, motion, summary, history):
    height, width = frame.shape[:2]
    scale = np.array(summary['mapping']['pixel_scale_to_1920x1080'])
    parameters = np.asarray(summary['mapping']['parameters'])
    if row['status'] == 'decoded':
        quad = np.asarray(row['corners_px'])
        color = (20, 190, 255) if row['shape_warning'] else (100, 245, 120)
        # The raised box is illustrative, not a measured target height/3D pose.
        camera = np.asarray(summary['illustrative_cube_camera_matrix'])
        size = summary['profile']['black_code_dimensions_mm'][0]
        half = size/2
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=float)
        okay, rvec, tvec = cv2.solvePnP(obj, quad.astype(float), camera, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if okay:
            vertices = np.vstack([obj, obj + [0, 0, size*.35]])
            points, _ = cv2.projectPoints(vertices, rvec, tvec, camera, np.zeros(5))
            points = points.reshape(-1, 2)
            if np.isfinite(points).all() and abs(points).max() < 100000:
                points = np.rint(points).astype(int)
                for k in range(4):
                    cv2.line(frame, tuple(points[k+4]), tuple(points[(k+1)%4+4]), (245, 195, 65), 2, cv2.LINE_AA)
                    cv2.line(frame, tuple(np.rint(quad[k]).astype(int)), tuple(points[k+4]), (245, 195, 65), 2, cv2.LINE_AA)
        # The actual base follows the exact decoded corners, independently of
        # whether the uncalibrated decorative cube reprojects them perfectly.
        cv2.polylines(frame, [np.rint(quad).astype(np.int32)], True, color, 2, cv2.LINE_AA)
        centre = np.rint(row['centre_px']).astype(int)
        cv2.drawMarker(frame, tuple(centre), (255, 245, 90), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        label(frame, f'ID {summary["marker_id"]}', quad[0]+[-2, -16], .65, color, 2)
        history.append((row['time_s'], centre))
    else:
        history.clear()
    while history and row['time_s']-history[0][0] > .6:
        history.popleft()
    if len(history) > 1:
        cv2.polylines(frame, [np.array([p for _, p in history], np.int32)], False, (240, 195, 70), 2, cv2.LINE_AA)
    # Keep the original camera view unobscured: append a dedicated bottom HUD.
    panel_height = 200
    panel = np.full((panel_height, width, 3), (22, 26, 32), np.uint8)
    minute, second = divmod(row['time_s'], 60)
    label(panel, f'CARLTRON | ID {summary["marker_id"]} CENTRE | {int(minute):02d}:{second:05.2f}', (30, 34), .85, thickness=2)
    if row['status'] == 'decoded':
        x, y = row['xy_center_mm']
        xy_prefix = '~ ' if summary.get('coordinate_quality_warning') else ''
        label(panel, f'X = {xy_prefix}{x:.1f} mm      Y = {xy_prefix}{y:.1f} mm', (30, 76), 1.,
              (40, 195, 255) if row['shape_warning'] else (110, 245, 135), 2)
        state = 'DECODED | height/tilt/outline warning: floor XY unreliable' if row['shape_warning'] else 'DECODED'
    else:
        label(panel, 'X = -- mm      Y = -- mm', (30, 76), 1., (40, 195, 255), 2)
        state = 'NOT DETECTED | no predicted position'
    value = motion['current'] or motion['last_valid']
    if value is None:
        label(panel, 'SPEED = -- m/s | collecting samples' if row['status']=='decoded' else 'SPEED = -- m/s', (1050, 76), .9, thickness=2)
    elif motion['current'] is not None:
        caution = ' | fit uncertain' if value['uncertain'] else ''
        label(panel, f'SPEED = {value["speed_m_s"]:.2f} m/s{caution}', (1050, 76), .9, (110, 245, 135) if not caution else (40, 195, 255), 2)
        label(panel, f'Vx = {value["vx_mm_s"]:+.0f} mm/s    Vy = {value["vy_mm_s"]:+.0f} mm/s', (1050, 111), .65)
    else:
        caution = ' | fit uncertain' if value['uncertain'] else ''
        label(panel, f'LAST SPEED = {value["speed_m_s"]:.2f} m/s | {motion["last_valid_age_s"]:.2f}s ago{caution}', (1050, 76), .8, (40, 195, 255), 2)
        label(panel, 'Historical estimate only; no current velocity during loss', (1050, 111), .6, (40, 195, 255))
    label(panel, state, (30, 112), .65, (40, 195, 255) if row['status']!='decoded' or row['shape_warning'] else (230, 240, 245))
    label(panel, 'Origin: ID0 near-left field datum | +X field-right, +Y field-far (not screen directions)', (30, 151), .6)
    footer = summary.get('coordinate_quality_warning') or 'APPROXIMATE FLOOR XY | target height not corrected | cube illustrative | speed: 150 ms fit on recording time'
    label(panel, footer, (30, 182), .6, (110, 195, 245))
    return np.vstack([frame, panel])


def render(args):
    out = args.output
    summary = json.loads((out / 'analysis-summary.json').read_text())
    tracks, motion = lines(out / 'id9-track.jsonl'), lines(out / 'id9-speed.jsonl')
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError('Cannot reopen source recording')
    destination = out / 'Carltron-ID9-Centre-Position-Speed-Full.mp4'
    width, height = summary['width'], summary['height'] + 200
    timing = summary['source_presentation_timestamps']
    # This recording has integer-frame-grid PTS with a gap on its last frame.
    # Preserve such gaps using piecewise PTS offsets without duplicating frames.
    frame_positions = np.asarray(timing['pts']) * timing['time_base'][0] / timing['time_base'][1] * summary['fps']
    rounded = np.rint(frame_positions).astype(int)
    if np.max(abs(frame_positions-rounded)) > 1e-6:
        raise ValueError('Non-frame-grid variable PTS require another renderer; do not silently resample.')
    offsets = rounded - np.arange(len(rounded))
    expression = f'N+{int(offsets[0])}'
    for index in np.flatnonzero(np.diff(offsets))+1:
        difference = int(offsets[index]-offsets[index-1])
        expression += f'+if(gte(N\\,{index})\\,{difference}\\,0)'
    pts_filter = f'settb=expr=1/{int(round(summary["fps"]))},setpts={expression}'
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-nostdin', '-y', '-f', 'rawvideo',
               '-pixel_format', 'bgr24', '-video_size', f'{width}x{height}', '-framerate', str(summary['fps']),
               '-i', 'pipe:0', '-i', str(args.video), '-map', '0:v:0', '-map', '1:a?', '-c:a', 'copy',
               '-vf', pts_filter, '-fps_mode', 'passthrough',
               '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p',
               '-threads', '8', '-movflags', '+faststart', str(destination)]
    started, history = time.perf_counter(), deque()
    sample_indices = {round(t*summary['fps']) for t in (0, 22, 26, 30, 35, 40, 45, 50, 58)}
    sample_indices.add(len(tracks)-1)
    with (out / 'overlay-encode.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
        try:
            for index, (row, speed) in enumerate(zip(tracks, motion)):
                okay, frame = cap.read()
                if not okay:
                    raise RuntimeError(f'Source decode stopped at frame {index}')
                annotated = annotate(frame, row, speed, summary, history)
                process.stdin.write(annotated.tobytes())
                if index in sample_indices:
                    cv2.imwrite(str(out / f'overlay-frame-{index:05d}.jpg'), annotated)
                if index % 300 == 0:
                    print(f'RENDER {index}/{len(tracks)} | video {row["time_s"]:.1f}s | elapsed {time.perf_counter()-started:.1f}s', flush=True)
        finally:
            cap.release()
            process.stdin.close()
        code = process.wait()
    if code:
        raise RuntimeError(f'Encoder exited with status {code}')
    destination = verify_output(args, destination, repair=True, elapsed=time.perf_counter()-started)
    print(f'COMPLETE {destination.resolve()}', flush=True)


def verify_output(args, destination, repair=False, elapsed=None):
    out = args.output
    summary = json.loads((out / 'analysis-summary.json').read_text())
    timing = summary['source_presentation_timestamps']
    width, height = summary['width'], summary['height']+200
    output_cap = cv2.VideoCapture(str(destination))
    frames, fps = int(output_cap.get(cv2.CAP_PROP_FRAME_COUNT)), output_cap.get(cv2.CAP_PROP_FPS)
    dims = [int(output_cap.get(3)), int(output_cap.get(4))]
    output_cap.release()
    if frames != summary['frames'] or dims != [width, height]:
        raise RuntimeError('Output dimensions, frame count or frame rate do not match expected values.')
    output_timing = presentation_times(destination, out / 'output-pts-audit.log')
    source_pts_seconds = np.asarray(timing['pts'])*timing['time_base'][0]/timing['time_base'][1]
    output_pts_seconds = np.asarray(output_timing['pts'])*output_timing['time_base'][0]/output_timing['time_base'][1]
    if len(output_pts_seconds) != len(source_pts_seconds):
        if not repair:
            raise RuntimeError('Output is not fully visible on normal playback.')
        raw_timing = presentation_times(destination, out / 'output-ignore-editlist-pts.log', ignore_editlist=True)
        raw_seconds = np.asarray(raw_timing['pts'])*raw_timing['time_base'][0]/raw_timing['time_base'][1]
        offset = raw_seconds[0]-source_pts_seconds[0]
        if len(raw_seconds) != len(source_pts_seconds) or np.max(abs(raw_seconds-offset-source_pts_seconds)) > .0001:
            raise RuntimeError('Encoded frame timestamps cannot be repaired by a constant edit-list offset.')
        tick_offset = int(round(offset*raw_timing['time_base'][1]/raw_timing['time_base'][0]))
        duration_ticks = int(round(raw_timing['time_base'][1]/raw_timing['time_base'][0]/summary['fps']))
        repaired = out / 'Carltron-ID9-Centre-Position-Speed-Verified.mp4'
        command = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-nostdin', '-y', '-copyts',
            '-ignore_editlist', '1', '-i', str(destination), '-i', str(args.video),
            '-map', '0:v:0', '-map', '1:a?', '-c', 'copy', '-bsf:v',
            f'setts=pts=PTS-{tick_offset}:dts=DTS-{tick_offset}:duration={duration_ticks}',
            '-avoid_negative_ts', 'disabled', '-movflags', '+faststart', str(repaired)]
        with (out / 'container-repair.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=log)
        if result.returncode:
            raise RuntimeError('Lossless container repair failed.')
        return verify_output(args, repaired, elapsed=elapsed)
    if np.max(abs(output_pts_seconds-source_pts_seconds)) > .0001:
        raise RuntimeError('Output did not preserve all source presentation timestamps')
    if file_hash(args.video) != summary['input_video_sha256']:
        raise RuntimeError('Source recording changed after analysis; output must not be treated as verified.')
    save(out / 'output-verification.json', {'output': str(destination.resolve()), 'frames': frames,
        'average_container_fps': fps, 'duration_s': summary['duration_s'], 'dimensions': dims, 'bytes': destination.stat().st_size,
        'all_source_presentation_timestamps_preserved': True,
        'max_output_pts_difference_s': float(np.max(abs(output_pts_seconds-source_pts_seconds))),
        'original_untouched': True, 'audio_mapping': 'Original audio copied when present',
        'elapsed_render_s': elapsed})
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['detect', 'finalize', 'analyse', 'render', 'verify', 'all'])
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--marker-id', type=int, default=9)
    parser.add_argument('--profile', type=Path, default=ROOT/'tools/tracking/profiles/robot-marker-85mm.json')
    parser.add_argument('--geometry', type=Path, required=True, help='Measured fixed tag coordinates JSON; the moving target must not train the map.')
    parser.add_argument('--annotated-video', type=Path)
    parser.add_argument('--recording-context', type=Path, help='Optional per-clip reported settings/assumptions JSON; omitted values stay unknown.')
    args = parser.parse_args()
    cv2.setNumThreads(4)
    if args.stage in ('detect', 'all'):
        detect_video(args)
    if args.stage == 'finalize':
        finalize_metadata(args)
    if args.stage in ('analyse', 'all'):
        analyse(args)
    if args.stage in ('render', 'all'):
        render(args)
    if args.stage == 'verify':
        if args.annotated_video is None:
            raise ValueError('Verification needs --annotated-video.')
        print(verify_output(args, args.annotated_video).resolve(), flush=True)


if __name__ == '__main__':
    main()
