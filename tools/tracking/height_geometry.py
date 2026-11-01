"""Raised-marker parallax correction using measured camera mounting geometry."""
import numpy as np
from plane_mapping import expand_corners, model_invert, invert


def load(path):
    import json
    return json.loads(path.read_text(encoding='utf-8'))


def correct_height(floor_point, height, camera):
    h = camera['camera_height_above_field_mm']
    c = camera['estimated_ground_projection_xy_mm']
    if not 0 <= height < h:
        raise ValueError('Target height must be below the camera.')
    return [c[i]+(h-height)/h*(floor_point[i]-c[i]) for i in range(2)]


def project_to_floor(point, height, camera):
    h = camera['camera_height_above_field_mm']
    c = camera['estimated_ground_projection_xy_mm']
    if not 0 <= height < h:
        raise ValueError('Target height must be below the camera.')
    return [c[i]+h/(h-height)*(point[i]-c[i]) for i in range(2)]


def map_quad(quad, mapping):
    quad = np.asarray(quad, dtype=float)
    if quad.shape != (4, 2) or not np.isfinite(quad).all():
        raise ValueError('A finite, complete four-corner target is required.')
    quad = expand_corners(quad, mapping['corner_expansion_pixels'])
    parameters = np.asarray(mapping['parameters'])
    if mapping['invert_function'] == 'refine_field_features.model_invert':
        return model_invert(quad, parameters)
    if mapping['invert_function'] == 'evaluate_planar_distortion.invert':
        return invert(quad, parameters)
    raise ValueError('Unknown stored mapping representation.')
