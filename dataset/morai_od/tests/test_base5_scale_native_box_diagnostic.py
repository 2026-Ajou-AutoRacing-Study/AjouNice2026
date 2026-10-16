"""Hypothesis-only geometry tests; these never create replacement GT labels."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import diagnose_base5_scale_native_boxes as d


class ScaleNativeBoxDiagnosticTest(unittest.TestCase):
    def component(self, **kwargs):
        value = dict(native_center_lidar_m=[0., 0., 10.],
                     native_rotation_rpy=[0., 0., 0.])
        value.update(kwargs)
        return value

    def camera(self):
        return dict(lidar_to_optical=np.eye(4).tolist(),
                    intrinsic=[[50., 0., 50.], [0., 50., 50.], [0., 0., 1.]])

    def test_projection_containment_distinguishes_native_and_hypothetical_size(self):
        component = self.component()
        before = copy.deepcopy(component)
        mask = np.zeros((100, 100), bool)
        mask[40:61, 40:61] = True
        native = d.containment(d.project(component, [2., 2., 2.], self.camera()), mask)
        hypothetical = d.containment(d.project(component, [4., 4., 4.], self.camera()), mask)
        self.assertLess(native['instance_fraction_inside_projected_hull'], .6)
        self.assertEqual(hypothetical['instance_fraction_inside_projected_hull'], 1.)
        self.assertEqual(component, before)

    def test_nonvisible_corner_does_not_invent_a_projection(self):
        value = d.project(self.component(native_center_lidar_m=[0., 0., .5]),
                          [2., 2., 2.], self.camera())
        self.assertFalse(value['available'])

    def test_lidar_and_ground_hypotheses_keep_native_center_rotation_fixed(self):
        component = self.component(native_center_lidar_m=[0., 0., 1.])
        before = copy.deepcopy(component)
        points = np.asarray([[0., 0., 1., 1.], [1.5, 0., 1., 1.], [4., 0., 1., 1.]])
        ground = [dict(uid=8500, plane=dict(coefficients=[0., 0., 0.]))]
        native = d.diagnostic(component, [2., 2., 2.], points, {}, {}, np.eye(4), ground)
        hypothesis = d.diagnostic(component, [4., 4., 4.], points, {}, {}, np.eye(4), ground)
        self.assertEqual(native['lidar_bbox_points'], 1)
        self.assertEqual(hypothesis['lidar_bbox_points'], 2)
        self.assertEqual(native['hypothetical_ground_plane_gaps'][0][
            'minimum_annotation_corner_gap_over_same_observed_plane_m'], 0.)
        self.assertEqual(hypothesis['hypothetical_ground_plane_gaps'][0][
            'minimum_annotation_corner_gap_over_same_observed_plane_m'], -1.)
        self.assertTrue(hypothesis['hypothetical_ground_plane_gaps'][0]['not_mesh_contact'])
        self.assertEqual(component, before)

    def test_corners_use_full_rpy_not_yaw_only(self):
        component = self.component(native_rotation_rpy=[.2, .1, np.pi / 2])
        size = [2., 4., 6.]
        result = d.corners(component, size)
        self.assertEqual(result.shape, (8, 3))
        np.testing.assert_allclose(result.mean(axis=0), component['native_center_lidar_m'], atol=1e-12)
        local = (result - component['native_center_lidar_m']) @ d.rotation_matrix(component['native_rotation_rpy'])
        np.testing.assert_allclose(np.ptp(local, axis=0), size, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
