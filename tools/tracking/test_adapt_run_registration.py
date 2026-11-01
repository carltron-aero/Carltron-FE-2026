import unittest
import numpy as np
from adapt_run_registration import register, compatible_lens, pose_from_quad
from plane_mapping import model_project, model_invert, invert


class RunRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.old = np.array([.52,.01,.02,-.01,-.54,.01,.025,-.01,.035,.008])
        self.changed = self.old.copy()
        self.changed[:8] += [0.02,.03,.015,-.015,.02,-.01,.003,-.004]
        self.mapping = {'physical_camera':1,'parameters':self.old.tolist(),
            'invert_function':'refine_field_features.model_invert','corner_expansion_pixels':0}
        self.geometry = {}
        for i,centre in enumerate([(300,300),(2700,300),(2700,2700),(300,2700)]):
            self.geometry[str(i)] = (np.array([[-70,70],[70,70],[70,-70],[-70,-70]])+centre).tolist()
        points = {i:model_project(np.asarray(g),self.changed).tolist() for i,g in self.geometry.items()}
        self.rows = [{'corners_px':points,'ambiguous_ids':[]} for _ in range(8)]

    def test_refresh_tracks_changed_view_without_target(self):
        result = register(self.rows,self.mapping,self.geometry)
        self.assertFalse(result['target_used_for_registration'])
        p = np.asarray(result['parameters'])
        for g in self.geometry.values():
            pixels = model_project(np.asarray(g),self.changed)
            self.assertLess(np.max(abs(model_invert(pixels,p)-g)),1e-4)

    def test_moving_target_cannot_change_registration(self):
        plain = register(self.rows,self.mapping,self.geometry)
        contaminated = [{'corners_px':{**r['corners_px'],'9':[[1,2],[4,2],[4,5],[1,5]],
                         '27':[[0,0],[1,0],[1,1],[0,1]]},'ambiguous_ids':[]} for r in self.rows]
        result = register(contaminated,self.mapping,self.geometry)
        np.testing.assert_allclose(result['parameters'],plain['parameters'],atol=0,rtol=0)

    def test_radial_representation_conversion(self):
        original = {**self.mapping,'parameters':self.old[:9].tolist(),
                    'invert_function':'evaluate_planar_distortion.invert'}
        pixels = np.array([[200,180],[800,400],[1650,800]],float)
        np.testing.assert_allclose(model_invert(pixels,compatible_lens(original)),
                                   invert(pixels,self.old[:9]),atol=1e-7)

    def test_rear_axle_offset_rotates_with_body(self):
        canonical = np.array([[-42.5,42.5],[42.5,42.5],[42.5,-42.5],[-42.5,-42.5]])
        marker_centre = np.array([1300,1400])
        angle = np.radians(37)
        rotation = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        world = canonical@rotation.T+marker_centre
        pixels = model_project(world,self.old)
        mount = {'camera_height_above_field_mm':2000,'estimated_ground_projection_xy_mm':[1500,1500]}
        result = pose_from_quad(pixels,self.mapping,mount,height_mm=0)
        expected_angle = angle-np.pi/2
        expected = marker_centre+100.72*np.array([np.cos(expected_angle),np.sin(expected_angle)])
        np.testing.assert_allclose(result['marker_centre_xy_mm'],marker_centre,atol=1e-6)
        np.testing.assert_allclose(result['rear_axle_xy_mm'],expected,atol=1e-6)

    def test_too_few_fixed_tags_are_rejected(self):
        rows = [{'corners_px':{k:v for k,v in r['corners_px'].items() if k in ['0','1']},
                 'ambiguous_ids':[]} for r in self.rows]
        with self.assertRaises(ValueError):
            register(rows,self.mapping,self.geometry)


if __name__ == '__main__':
    unittest.main()
