"""Annotate genuine decoded markers in one saved image; no camera access."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from adapt_run_registration import pose_from_quad
from track_robot_marker_video import decoded, intersection_center, make_detector


def detect(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hits, ambiguous = decoded(make_detector(), gray)
    extra, duplicates = decoded(make_detector(True), gray)
    ambiguous = sorted(set(ambiguous + duplicates))
    for identifier, quad in extra.items():
        if identifier not in hits and identifier not in ambiguous:
            hits[identifier] = quad
    return {i: q for i, q in hits.items() if i not in ambiguous}, ambiguous


def text(image, value, point, color=(255, 255, 255), scale=.75):
    cv2.putText(image, value, point, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (12, 17, 22), 5, cv2.LINE_AA)
    cv2.putText(image, value, point, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 2, cv2.LINE_AA)


def annotate(frame, camera, mapping=None, mount=None, elapsed=None, height_mm=4.825):
    if (mapping is None) != (mount is None):
        raise ValueError('A metric overlay needs both a recorded mapping and camera mount.')
    hits, ambiguous = detect(frame)
    output = frame.copy()
    records = []
    pose = None
    for identifier, quad in sorted(hits.items()):
        role = 'robot' if identifier == 9 else 'fixed' if identifier < 8 else 'shared / not fitted'
        color = (255, 220, 60) if identifier == 9 else (90, 235, 90) if identifier < 8 else (50, 195, 255)
        points = np.round(quad).astype(np.int32)
        cv2.polylines(output, [points], True, color, 3, cv2.LINE_AA)
        for point in points:
            cv2.circle(output, tuple(point), 4, color, -1, cv2.LINE_AA)
        centre = intersection_center(quad)
        cv2.drawMarker(output, tuple(np.round(centre).astype(int)), color,
                       cv2.MARKER_CROSS, 19, 2, cv2.LINE_AA)
        anchor = np.maximum([8, 55], points.min(axis=0) - [0, 13])
        anchor[0] = min(anchor[0], frame.shape[1] - 130)
        text(output, f'ID {identifier}', tuple(anchor), color)
        records.append({'id': identifier, 'role': role, 'corners_px': quad.tolist(),
                        'projective_centre_px': centre.tolist()})
        if identifier == 9:
            # Image-plane coded axes are illustrative, not a solved 3D pose.
            u, v = quad[1] - quad[0], quad[0] - quad[3]
            for vector, label, axis_color in [(u, 'u', (70, 70, 255)), (v, 'v', (90, 235, 90))]:
                end = centre + 48 * vector / np.linalg.norm(vector)
                cv2.arrowedLine(output, tuple(np.round(centre).astype(int)),
                               tuple(np.round(end).astype(int)), axis_color, 2, cv2.LINE_AA, tipLength=.2)
                label_point = np.maximum([8, 22], np.round(end + [5, -5]).astype(int))
                text(output, label, tuple(label_point), axis_color, .6)
            if mapping is not None:
                pose = pose_from_quad(quad, mapping, mount, height_mm=height_mm)

    cv2.rectangle(output, (15, 15), (585, 157 if pose else 120), (19, 26, 33), -1)
    text(output, f'CARLTRON  |  Camera {camera}  |  Recorded frame', (30, 45), scale=.7)
    text(output, 'Fixed references   /   Robot ID 9   /   Shared tags', (30, 77), scale=.6)
    stamp = '' if elapsed is None else f'  |  LED-aligned run time {elapsed:.2f} s'
    text(output, f'{len(hits)} decoded markers{stamp}', (30, 108), scale=.6)
    if pose:
        x, y = pose['marker_centre_xy_mm']
        text(output, f'Marker centre: X {x:.1f} mm   Y {y:.1f} mm', (30, 140), (255, 220, 60), .65)
    footer = ('Decoded corners and centre crosses | u/v: image-plane code axes, not 3D pose'
              + (' | XY: recorded empirical map + approximate height correction' if pose else ''))
    cv2.rectangle(output, (0, frame.shape[0]-40), (frame.shape[1], frame.shape[0]), (19, 26, 33), -1)
    text(output, footer, (20, frame.shape[0]-14), scale=.56)
    return output, {'camera': camera, 'decoded_markers': records, 'ambiguous_ids': ambiguous,
                    'marker_pose': pose, 'run_time_s': elapsed,
                    'marker_centre_height_mm': height_mm if pose else None,
                    'qualification': 'Genuine single-image detections. XY is an estimate from this recorded view; '
                                     'not a certified dynamic accuracy result. u/v are coded image-plane directions.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera', type=int, choices=[1, 2], required=True)
    parser.add_argument('--mappings', type=Path)
    parser.add_argument('--mounts', type=Path)
    parser.add_argument('--time-seconds', type=float)
    parser.add_argument('--height-mm', type=float, default=4.825)
    args = parser.parse_args()
    if args.output.suffix.lower() not in {'.png', '.jpg', '.jpeg'}:
        parser.error('Output must be a PNG or JPEG image.')
    if args.output.exists() or args.output.with_suffix('.json').exists():
        parser.error('Output exists; choose a new filename.')
    if bool(args.mappings) != bool(args.mounts):
        parser.error('Provide both --mappings and --mounts, or neither.')
    frame = cv2.imread(str(args.image))
    if frame is None:
        raise ValueError('Cannot decode source image.')
    mapping = mount = None
    if args.mappings:
        if frame.shape[:2] != (1080, 1920):
            raise ValueError('Recorded example maps require the uncropped 1920 x 1080 view.')
        mapping = next(m for m in json.loads(args.mappings.read_text())['views'] if m['physical_camera'] == args.camera)
        mount = next(m for m in json.loads(args.mounts.read_text())['cameras'] if m['physical_camera'] == args.camera)
    output, report = annotate(frame, args.camera, mapping, mount, args.time_seconds, args.height_mm)
    report['source_image'] = args.image.name
    report['source_sha256'] = hashlib.sha256(args.image.read_bytes()).hexdigest()
    report['detector'] = 'OpenCV DICT_4X4_50, subpixel corners, normal + expanded threshold passes'
    report['opencv_version'] = cv2.__version__
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), output, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise OSError('Cannot write annotated image.')
    report['output_sha256'] = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({'image': str(args.output), 'ids': [r['id'] for r in report['decoded_markers']],
                      'marker_pose': report['marker_pose']}))


if __name__ == '__main__':
    main()
