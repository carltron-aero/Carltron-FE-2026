"""Register saved runs from fixed field tags, never the moving robot target.

Pose is refreshed for each recording. Lens coefficients remain separately
identified, with fixed-tag-only diagnostics for transfer to a changed view.
"""
from pathlib import Path
import json
import numpy as np

import cv2
from height_geometry import load, map_quad, correct_height, project_to_floor
from plane_mapping import refresh_pose, expand_corners, model_project, model_invert, fit

ROOT = Path(__file__).resolve().parents[2]


def compatible_lens(mapping):
    p = np.asarray(mapping['parameters'], dtype=float)
    if mapping['invert_function'] == 'evaluate_planar_distortion.invert':
        if len(p) != 9:
            raise ValueError('This adapter expects the recorded one-radial-term model.')
        p = np.r_[p, 0.0]
    elif mapping['invert_function'] != 'refine_field_features.model_invert':
        raise ValueError('Unrecognized lens representation.')
    return p


def register(rows, original, geometry):
    medians, counts = {}, {}
    for identifier in range(8):
        quads = [r['corners_px'][str(identifier)] for r in rows
                 if str(identifier) in r['corners_px'] and identifier not in r.get('ambiguous_ids', [])]
        if len(quads) >= 5 and str(identifier) in geometry:
            medians[identifier] = np.median(quads, axis=0)
            counts[identifier] = len(quads)
    ids = sorted(medians)
    if len(ids) < 3:
        raise ValueError('At least three separate fixed tags are needed for run registration.')
    world = {i: np.asarray(geometry[str(i)], dtype=float) for i in ids}
    pixels = {i: expand_corners(medians[i], original['corner_expansion_pixels']) for i in ids}
    ww, pp = np.concatenate(list(world.values())), np.concatenate(list(pixels.values()))
    if cv2.contourArea(cv2.convexHull(ww.astype(np.float32))) < 100_000:
        raise ValueError('Fixed references have insufficient spatial coverage.')
    parameters, training_rms = refresh_pose(ww, pp, compatible_lens(original))
    result = {**original, 'parameters': parameters.tolist(),
              'invert_function': 'refine_field_features.model_invert',
              'mapping': 'Current recording fixed-tag pose; inherited lens coefficients',
              'fixed_training_ids': ids, 'fixed_counts': counts,
              'fixed_corner_training_rms_pixels': training_rms,
              'target_used_for_registration': False, 'shared_tags_used_for_registration': False,
              'lens_coefficients_changed': False, 'pose_refreshed': True,
              'leave_one_fixed_tag_out': {}}
    for omitted in ids:
        kept = [i for i in ids if i != omitted]
        if len(kept) < 3:
            continue
        probe, _ = refresh_pose(np.concatenate([world[i] for i in kept]),
                                np.concatenate([pixels[i] for i in kept]), parameters)
        errors = model_invert(pixels[omitted], probe)-world[omitted]
        result['leave_one_fixed_tag_out'][str(omitted)] = {
            'corner_rms_mm': float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
            'mean_corner_error_xy_mm': errors.mean(axis=0).tolist()}
    # An optical-transfer challenger is fitted only to fixed tags. Retain it
    # as a diagnostic; do not select a lens using robot-trajectory agreement.
    challenger = fit(ww, pp, 1)
    result['fixed_only_radial1_challenger_parameters'] = challenger.tolist()
    result['qualification'] = ('Re-registered planar empirical mapping; fixed geometry and inherited lens '
        'remain uncertain. Changed zoom/focus is not automatically certified by small training residuals. '
        'Moving ID9, shared tags 27/28, robot logs and LED timing never fit the spatial map.')
    return result


def pose_from_quad(quad, mapping, mount, height_mm=4.825, forward_from_code_edge_deg=-90):
    world = map_quad(quad, mapping)
    world = np.asarray([correct_height(p, height_mm, mount) for p in world])
    # Intersect diagonals for centre; use all four corners for orientation.
    from track_robot_marker_video import intersection_center
    centre = intersection_center(world)
    canonical = np.array([[-1,1],[1,1],[1,-1],[-1,-1]], dtype=float)
    u, _, vt = np.linalg.svd(canonical.T@(world-world.mean(axis=0)))
    rotation = vt.T@u.T
    if np.linalg.det(rotation) < 0:
        raise ValueError('Reflected target geometry.')
    angle = np.radians(forward_from_code_edge_deg)
    forward = rotation@np.array([np.cos(angle),np.sin(angle)])
    left = np.array([-forward[1],forward[0]])
    rear_axle = centre+100.72*forward
    led_xy = centre+140*forward+56*left
    return {'marker_centre_xy_mm': centre.tolist(),
            'rear_axle_xy_mm': rear_axle.tolist(),
            'body_heading_degrees': float(np.degrees(np.arctan2(forward[1],forward[0]))),
            'forward_vector': forward.tolist(), 'left_vector': left.tolist(),
            'approximate_led_group_xy_mm': led_xy.tolist()}


def led_roi(pose, mapping, mount, image_shape, height_mm=4.825, half_size_mm=12):
    centre = np.asarray(pose['approximate_led_group_xy_mm'])
    forward, left = np.asarray(pose['forward_vector']), np.asarray(pose['left_vector'])
    corners = [centre+half_size_mm*(a*forward+b*left) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    rays = np.asarray([project_to_floor(p,height_mm+25,mount) for p in corners])
    pixels = model_project(rays, np.asarray(mapping['parameters']))
    low = np.floor(pixels.min(axis=0)-4).astype(int)
    high = np.ceil(pixels.max(axis=0)+4).astype(int)
    low = np.maximum(low,0); high = np.minimum(high,[image_shape[1],image_shape[0]])
    if np.any(high-low < 3):
        return None
    return [int(low[0]),int(low[1]),int(high[0]),int(high[1])]


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--lens-file',type=Path,required=True,help='JSON with views, inherited lens parameters and reference camera numbers.')
    parser.add_argument('--geometry',type=Path,required=True,help='JSON with measured fixed_geometry_mm.')
    args = parser.parse_args()
    frozen = load(args.lens_file)
    geometry = load(args.geometry)['fixed_geometry_mm']
    maps = [register(load(args.directory/f'camera-{m["physical_camera"]}-preliminary-sampled-detections.json'),m,geometry)
            for m in frozen['views']]
    result = {'format':'carltron-current-run-fixed-tag-registration-v1','views':maps,
              'fixed_geometry_source':str(args.geometry), 'moving_target_used':False,
              'frozen_baseline_modified':False,'live_camera_settings_modified':False,
              'optical_settings_at_recording_independently_read_back':False,
              'policy':'Use current fixed-reference registration per run. Monitor reference residuals across the recording; never align cameras by forcing moving robot positions to agree.'}
    destination = args.directory/'registered-run-mappings.json'
    if destination.exists():
        raise ValueError('Registration output already exists; refusing overwrite.')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps([{'camera':m['physical_camera'],'training_rms_px':m['fixed_corner_training_rms_pixels'],
                      'leave_one_tag_out':m['leave_one_fixed_tag_out']} for m in maps],indent=2))


if __name__ == '__main__':
    main()
