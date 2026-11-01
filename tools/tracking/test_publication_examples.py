"""Offline checks for the portable example and its original image bytes."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('publication_replay', HERE/'run-replay/server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class ExampleTests(unittest.TestCase):
    def setUp(self):
        self.replay = server.Replay(server.DEFAULT_RUN)

    def test_original_keyframe_hashes(self):
        manifest = json.loads((server.DEFAULT_RUN/'frames-manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(manifest['format'], 'carltron-frame-manifest-v1')
        self.assertEqual(len(manifest['images']), 10)
        expected_fields = {'file', 'camera', 'original_frame_index', 'estimated_run_time_s',
                           'sha256_original_jpeg', 'image_transformed'}
        for row in manifest['images']:
            self.assertEqual(set(row), expected_fields)
            self.assertEqual(row['file'], f"frames/camera-{row['camera']}-{row['original_frame_index']}.jpg")
            body = self.replay.frame(row['camera'], row['original_frame_index'])
            self.assertEqual(hashlib.sha256(body).hexdigest(), row['sha256_original_jpeg'])
            self.assertFalse(row['image_transformed'])

    def test_sparse_images_are_not_presented_as_full_video(self):
        self.assertIn('Sparse', self.replay.data['example_kind'])
        for camera in self.replay.data['cameras']:
            self.assertEqual(len(camera['frames']), 5)
            self.assertGreater(len(camera['observations']), 600)
            self.assertTrue(any(r['xy'] is None for r in camera['observations']))
            for row in camera['frames']:
                self.assertIn(row, camera['observations'])

    def test_unknown_frame_and_camera_are_rejected(self):
        for camera, index in [(3, 0), (1, -1), (1, 123456)]:
            with self.assertRaises(KeyError):
                self.replay.frame(camera, index)

    def test_reference_sizes_and_results(self):
        profile = json.loads((HERE/'profiles/robot-marker-85mm.json').read_text())
        self.assertEqual(profile['black_code_dimensions_mm'], [85, 85])
        self.assertEqual(profile['plate_dimensions_mm'], [100, 100])
        results = json.loads((server.DEFAULT_RUN/'results.json').read_text())
        self.assertFalse(results['static_held_out_placement']['reference_used_to_fit_or_select_mapping'])
        self.assertAlmostEqual(results['static_held_out_placement']['intercamera_disagreement_mm'], 1.0577036162133295)

    def test_documentation_detection_frame_identity(self):
        images = HERE.parents[1] / 'docs/tracking/images'
        for camera, index in [(1, 1307), (2, 1128)]:
            report = json.loads((images / f'camera-{camera}-detections.json').read_text())
            source = self.replay.frame(camera, index)
            self.assertEqual(report['source_sha256'], hashlib.sha256(source).hexdigest())
            self.assertEqual(report['original_frame_index'], index)
            data = next(c for c in self.replay.data['cameras'] if c['id'] == camera)
            row = next(f for f in data['frames'] if f['index'] == index)
            self.assertEqual(report['run_time_s'], row['time'])
            self.assertEqual(report['marker_pose']['marker_centre_xy_mm'], row['marker'])
            self.assertEqual(len(report['decoded_markers']), 7)
            self.assertIn(9, [m['id'] for m in report['decoded_markers']])
            output = (images / f'camera-{camera}-detections.jpg').read_bytes()
            self.assertEqual(report['output_sha256'], hashlib.sha256(output).hexdigest())


if __name__ == '__main__':
    unittest.main()
