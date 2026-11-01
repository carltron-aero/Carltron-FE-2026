"""Offline regression tests: projective centres and honest speed-gap handling."""
import unittest
import numpy as np

from track_robot_marker_video import intersection_center, speeds, annotate, timed_spans, recording_context


class CentreTests(unittest.TestCase):
    def test_new_clip_does_not_inherit_previous_settings(self):
        context = recording_context()
        self.assertEqual(context['optical_settings_user_reported'], {})
        self.assertIsNone(context['physical_camera_user_reported'])
        self.assertIsNone(context['target_height_user_reported_mm'])

    def test_diagonal_intersection_respects_perspective(self):
        quad = np.array([[-1., 1.], [1., 1.], [1., -1.], [-1., -1.]])
        h = np.array([[2., .3, 8.], [.2, 3., 4.], [.2, .1, 1.]])
        homogeneous = np.column_stack([quad, np.ones(4)]) @ h.T
        projected = homogeneous[:, :2] / homogeneous[:, 2:3]
        np.testing.assert_allclose(intersection_center(projected), [8., 4.], atol=1e-10)
        self.assertGreater(np.linalg.norm(projected.mean(axis=0)-[8., 4.]), .1)

    def test_hud_does_not_cover_the_camera_picture(self):
        original = np.full((1440, 2560, 3), 110, np.uint8)
        row = {'time_s': 0., 'status': 'not-detected', 'shape_warning': False}
        motion = {'current': None, 'last_valid': None}
        summary = {'marker_id': 9, 'mapping': {'pixel_scale_to_1920x1080': [.75, .75],
                    'parameters': [1., 0., 0., 0., 1., 0., 0., 0., 0., 0.]}}
        from collections import deque
        output = annotate(original, row, motion, summary, deque())
        self.assertEqual(output.shape, (1640, 2560, 3))
        np.testing.assert_array_equal(output[:1440], original)

    def test_per_recording_coordinate_warning_is_shown(self):
        from collections import deque
        from unittest.mock import patch
        row = {'time_s': 0., 'status': 'not-detected', 'shape_warning': False}
        motion = {'current': None, 'last_valid': None}
        warning = 'UNVALIDATED FLOOR XY: model disagreement; review only'
        summary = {'marker_id': 9, 'coordinate_quality_warning': warning,
                   'mapping': {'pixel_scale_to_1920x1080': [.75, .75],
                               'parameters': [1., 0., 0., 0., 1., 0., 0., 0., 0., 0.]}}
        with patch('track_robot_marker_video.label') as draw:
            annotate(np.zeros((1440, 2560, 3), np.uint8), row, motion, summary, deque())
        self.assertIn(warning, [call.args[1] for call in draw.call_args_list])


class SpeedTests(unittest.TestCase):
    @staticmethod
    def row(n, valid=True, warning=False):
        return {'frame': n, 'time_s': n/60., 'status': 'decoded' if valid else 'not-detected',
                'shape_warning': warning, 'xy_center_mm': [1000.+1000.*n/60., 1200.] if valid else None}

    def test_constant_speed_is_in_metres_per_second(self):
        result = speeds([self.row(n) for n in range(20)])
        self.assertIsNone(result[0]['current'])
        self.assertAlmostEqual(result[-1]['current']['speed_m_s'], 1.)
        self.assertAlmostEqual(result[-1]['current']['vx_mm_s'], 1000.)
        self.assertAlmostEqual(result[-1]['current']['vy_mm_s'], 0.)

    def test_dropout_has_last_speed_not_current_or_predicted(self):
        result = speeds([self.row(n) for n in range(20)]+[self.row(n, False) for n in range(20, 28)])
        self.assertIsNone(result[-1]['current'])
        self.assertAlmostEqual(result[-1]['last_valid']['speed_m_s'], 1.)
        self.assertAlmostEqual(result[-1]['last_valid_age_s'], 8/60.)

    def test_long_gap_requires_new_fit(self):
        rows = [self.row(n) for n in range(20)] + [self.row(n, False) for n in range(20, 30)] + [self.row(30)]
        self.assertIsNone(speeds(rows)[-1]['current'])
        self.assertEqual(speeds(rows)[-1]['current_status'], 'collecting-samples')

    def test_shape_warning_resets_current_velocity(self):
        result = speeds([self.row(n) for n in range(20)]+[self.row(20, warning=True), self.row(21)])
        self.assertIsNone(result[-2]['current'])
        self.assertEqual(result[-2]['current_status'], 'shape-warning')
        self.assertIsNone(result[-1]['current'])


class TimestampTests(unittest.TestCase):
    def test_gap_span_uses_pts_not_nominal_frame_count(self):
        results = timed_spans([False, True, True, False], [0., .1, .2, .4], .5)
        self.assertAlmostEqual(results[0]['duration_s'], .3)
        self.assertEqual(results[0]['frames'], 2)

    def test_final_span_includes_last_frame_duration(self):
        results = timed_spans([False, True], [0., .2], .3)
        self.assertAlmostEqual(results[0]['end_s'], .3)
        self.assertAlmostEqual(results[0]['duration_s'], .1)


if __name__ == '__main__':
    unittest.main()
