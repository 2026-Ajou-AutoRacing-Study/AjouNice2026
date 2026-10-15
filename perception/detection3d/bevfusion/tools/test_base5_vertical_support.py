"""CPU-only checks for native full-RPY Z support accounting."""
import unittest

import numpy as np

from audit_base5_vertical_support import augmented_z_counts, crop_center_depths, native_rotation, range_count, summarize, supported_points


class VerticalSupportTest(unittest.TestCase):
    def test_rotation_and_native_support(self):
        rotation = native_rotation([.2, -.3, .7])
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-14)
        component = dict(center_lidar_m=[1., 2., 3.], size_m=[2., 1., 1.], rotation_rpy=[.2, -.3, .7])
        xyz = np.array([[0., 0., 0.], [.9, .4, .4], [1.1, 0., 0.]]) @ rotation.T + [1., 2., 3.]
        self.assertEqual(len(supported_points(xyz, component)), 2)

    def test_open_boundaries_and_empty_cloud(self):
        pts = np.array([[0., 0., z] for z in (-3., -2.99, 1.99, 2., 2.5)])
        self.assertEqual(range_count(pts, (-3., 2.)), 2)
        self.assertEqual(range_count(pts, (-5., 3.)), 5)
        self.assertEqual(range_count(np.empty((0, 3)), (-3., 2.)), 0)

    def test_lost_support_not_camera_only(self):
        rows = [dict(native_xy_support=0, current_support=0, internal_support=0),
                dict(native_xy_support=3, current_support=0, internal_support=3),
                dict(native_xy_support=10, current_support=8, internal_support=9)]
        result = summarize(rows)
        self.assertEqual(result['zero_before'], 1)
        self.assertEqual(result['lost_all_support'], 1)
        self.assertEqual(result['removed_points'], 5)
        self.assertEqual(result['recovered_points_internal'], 4)
        self.assertEqual(result['affected_objects'], 2)
        self.assertIsNone(summarize([])['removed_point_fraction'])

    def test_augmentation_translation_then_scale(self):
        points = np.array([[0., 0., 1.8]])
        # (1.8 + .1) * 1.1 = 2.09, outside; .9 scales it inside.
        np.testing.assert_array_equal(augmented_z_counts(points, np.array([1.1, .9]), np.array([.1, .1])), [0, 1])
        np.testing.assert_array_equal(augmented_z_counts(np.empty((0, 3)), np.array([1., 1.]), np.zeros(2)), [0, 0])

    def test_depth_is_optical_z_not_horizontal_range(self):
        camera = dict(lidar2cam=np.eye(4).tolist(), width=1280, height=720,
                      cam2img=[[640., 0., 640.], [0., 640., 360.], [0., 0., 1.]])
        self.assertEqual(crop_center_depths([1., 0., 50.], {'front': camera}), [50.])
        self.assertEqual(crop_center_depths([0., 0., -1.], {'front': camera}), [])


if __name__ == '__main__':
    unittest.main()
