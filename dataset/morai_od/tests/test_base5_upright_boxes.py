import itertools
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_upright_boxes import upright_bbox, component_bbox, LEGACY_BOX_GEOMETRY_POLICY


def native_corners(center, size, rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
    return (signs * np.asarray(size) / 2) @ (rz @ ry @ rx).T + center


class UprightBoxesTest(unittest.TestCase):
    def test_upright_is_identical_and_keeps_heading(self):
        for yaw in (-np.pi, -.8, 0, .7, np.pi):
            self.assertEqual(upright_bbox([1, 2, 3], [4, 2, 1], [0, 0, yaw]), [1, 2, 3, 4, 2, 1, yaw])

    def test_sideways_barrel_changes_height_and_width(self):
        bbox = upright_bbox([4, 2, -1], [.5, .5, .8], [np.pi/2, 0, 1.2])
        np.testing.assert_allclose(bbox, [4, 2, -1, .5, .8, .5, 1.2], rtol=0, atol=1e-14)

    def test_all_native_corners_inside_and_extents_tight_at_fixed_yaw(self):
        rng = np.random.RandomState(261011)
        for _ in range(500):
            center = rng.uniform(-100, 100, 3); size = rng.uniform(.1, 10, 3); rpy = rng.uniform(-np.pi, np.pi, 3)
            result = np.asarray(upright_bbox(center, size, rpy))
            corners = native_corners(center, size, rpy)
            y = rpy[2]; rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
            local = (corners - result[:3]) @ rz
            self.assertTrue((np.abs(local) <= result[3:6]/2 + 1e-12).all())
            np.testing.assert_allclose(local.max(0)-local.min(0), result[3:6], rtol=0, atol=1e-12)
            np.testing.assert_array_equal(result[:3], center)
            self.assertEqual(result[6], rpy[2])

    def test_bev_rotation_scale_translation_commutes_with_conversion(self):
        center = np.array([20., -10., -1.]); size = np.array([.5, .8, 1.2]); rpy = [.5, -.8, 1.1]
        angle = .1; scale = 1.05; translation = np.array([.2, -.1, .3])
        rz = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        box = np.asarray(upright_bbox(center, size, rpy))
        transformed = upright_bbox(rz @ center * scale + translation, size * scale, [rpy[0], rpy[1], rpy[2]+angle])
        np.testing.assert_allclose(transformed, np.r_[rz @ box[:3] * scale + translation, box[3:6] * scale, box[6]+angle], atol=1e-12)

    def test_invalid_geometry_and_unknown_policy_rejected(self):
        for center,size,rpy in [([0,0], [1,1,1], [0,0,0]), ([0,0,0], [0,1,1], [0,0,0]),
                                ([0,0,0], [1,1,1], [0,float('nan'),0])]:
            with self.assertRaises(ValueError): upright_bbox(center,size,rpy)
        component = dict(center_lidar_m=[0,0,0], size_m=[1,1,2], rotation_rpy=[1,0,.5])
        self.assertEqual(component_bbox(component, LEGACY_BOX_GEOMETRY_POLICY), [0,0,0,1,1,2,.5])
        with self.assertRaises(ValueError): component_bbox(component, 'guess')


if __name__ == '__main__': unittest.main()
