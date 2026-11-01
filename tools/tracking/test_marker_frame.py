"""Offline checks for genuine detections and honest single-frame overlays."""
import unittest

import cv2
import numpy as np

from render_marker_frame import annotate


class MarkerFrameTests(unittest.TestCase):
    def frame(self, identifiers):
        frame = np.full((500, 900, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for n, identifier in enumerate(identifiers):
            tag = cv2.aruco.generateImageMarker(dictionary, identifier, 150)
            x = 90 + n * 240
            frame[210:360, x:x+150] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
        return frame

    def test_all_decoded_markers_are_reported(self):
        _, report = annotate(self.frame([0, 9, 27]), 1)
        self.assertEqual([r['id'] for r in report['decoded_markers']], [0, 9, 27])
        self.assertIsNone(report['marker_pose'])
        self.assertTrue(all(np.asarray(r['corners_px']).shape == (4, 2) for r in report['decoded_markers']))

    def test_duplicate_id_is_not_a_measurement(self):
        _, report = annotate(self.frame([9, 9]), 2)
        self.assertEqual(report['decoded_markers'], [])
        self.assertEqual(report['ambiguous_ids'], [9])

    def test_partial_metric_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            annotate(self.frame([9]), 1, mapping={})


if __name__ == '__main__':
    unittest.main()
