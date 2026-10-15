#!/usr/bin/env python3
"""CPU depth-hint contract tests; CUDA reproducibility must be tested separately."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from perception.detection3d.bevfusion.depth_raster import (
    project_positive_camera_z, rasterize_positive_camera_z, reduce_depth_pixels,
)


class PixelTests(unittest.TestCase):
    def test_nearest_not_sum_or_average(self):
        coordinates = torch.tensor([[2, 1], [2, 1], [1, 3]])
        depth = torch.tensor([7., 2., 4.])
        for backend in ('sorted', 'scatter_reduce'):
            image = reduce_depth_pixels(coordinates, depth, (4, 4), backend=backend)
            self.assertEqual(float(image[2, 1]), 2.)
            self.assertEqual(float(image[1, 3]), 4.)
            self.assertEqual(int(torch.count_nonzero(image)), 2)

    def test_permutation_invariant_forward(self):
        coords = torch.tensor([[1, 1], [2, 2], [1, 1], [1, 1], [2, 2]])
        z = torch.tensor([4., 8., 2., 2., 7.])
        for policy in ('nearest_positive', 'farthest_positive'):
            reference = reduce_depth_pixels(coords, z, (3, 3), policy)
            for order in (torch.arange(5), torch.tensor([4, 3, 2, 1, 0]), torch.tensor([2, 0, 4, 1, 3])):
                for backend in ('sorted', 'scatter_reduce'):
                    self.assertTrue(torch.equal(reference, reduce_depth_pixels(coords[order], z[order], (3, 3), policy, backend)))

    def test_farthest_and_last_controls(self):
        coords, z = torch.tensor([[1, 1], [1, 1], [1, 1]]), torch.tensor([3., 8., 2.])
        self.assertEqual(float(reduce_depth_pixels(coords, z, (2, 2), 'farthest_positive')[1, 1]), 8.)
        self.assertEqual(float(reduce_depth_pixels(coords, z, (2, 2), 'unique_last_positive')[1, 1]), 2.)

    def test_invalid_and_zero_are_empty_not_sanitized_points(self):
        coords = torch.tensor([[1., 1.], [1, 1], [1, 1], [1, 1], [float('nan'), 1], [-1, 1], [2, 1]])
        z = torch.tensor([0., -2., float('nan'), float('inf'), 3., 4., 5.])
        for backend in ('sorted', 'scatter_reduce'):
            image = reduce_depth_pixels(coords, z, (2, 2), backend=backend)
            self.assertTrue(torch.equal(image, torch.zeros(2, 2)))

    def test_unique_pixels_exact(self):
        coords = torch.tensor([[0, 0], [0, 1], [1, 1]])
        z = torch.tensor([2., 5., 10.])
        expected = torch.tensor([[2., 5.], [0., 10.]])
        for policy in ('nearest_positive', 'farthest_positive', 'unique_last_positive'):
            self.assertTrue(torch.equal(expected, reduce_depth_pixels(coords, z, (2, 2), policy)))

    def test_empty_has_shape_dtype_and_zero_gradient(self):
        z = torch.tensor([0., -1.], dtype=torch.float64, requires_grad=True)
        image = reduce_depth_pixels(torch.tensor([[0, 0], [1, 1]]), z, (2, 3))
        self.assertEqual((tuple(image.shape), image.dtype), ((2, 3), torch.float64))
        image.sum().backward()
        self.assertTrue(torch.equal(z.grad, torch.zeros_like(z)))

    def test_gradient_distinct_minimum(self):
        for backend in ('sorted', 'scatter_reduce'):
            z = torch.tensor([5., 2., 7.], requires_grad=True)
            image = reduce_depth_pixels(torch.tensor([[1, 1]] * 3), z, (2, 2), backend=backend)
            image.sum().backward()
            self.assertTrue(torch.equal(z.grad, torch.tensor([0., 1., 0.])))

    def test_tie_gradient_contract(self):
        for backend, expected in (('sorted', [1., 0.]), ('scatter_reduce', [.5, .5])):
            z = torch.tensor([2., 2.], requires_grad=True)
            reduce_depth_pixels(torch.tensor([[1, 1]] * 2), z, (2, 2), backend=backend).sum().backward()
            self.assertTrue(torch.equal(z.grad, torch.tensor(expected)))

    def test_reject_bad_policy(self):
        with self.assertRaises(ValueError):
            reduce_depth_pixels(torch.tensor([[0, 0]]), torch.ones(1), (2, 2), 'magic')


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.matrix = torch.eye(4)[None, None]
        self.aug = torch.eye(4)[None]

    def test_axial_z_not_euclidean_range_and_row_column(self):
        points = [torch.tensor([[6., 2., 2., .5]])]
        sample = project_positive_camera_z(points, self.matrix, self.matrix, self.aug, (8, 8))[0][0]
        self.assertTrue(torch.equal(sample[0], torch.tensor([[1, 3]])))
        self.assertTrue(torch.equal(sample[1], torch.tensor([2.])))

    def test_two_depths_same_pixel_occlusion(self):
        p = [torch.tensor([[2., 4., 2., 1.], [5., 10., 5., 1.]])]
        image = rasterize_positive_camera_z(p, self.matrix, self.matrix, self.aug, (8, 8))
        self.assertEqual(tuple(image.shape), (1, 1, 1, 8, 8))
        self.assertEqual(float(image[0, 0, 0, 2, 1]), 2.)

    def test_points_are_not_modified(self):
        p = [torch.tensor([[2., 4., 2., 1.]])]
        saved = copy.deepcopy(p)
        aug = self.aug.clone()
        aug[:, :3, 3] = torch.tensor([1., 1., 0.])
        rasterize_positive_camera_z(p, self.matrix, self.matrix, aug, (8, 8))
        self.assertTrue(torch.equal(p[0], saved[0]))

    def test_behind_and_invalid_are_not_front_hints(self):
        p = [torch.tensor([[0., 0., -1., 1.], [0., 0., 0., 1.], [float('nan'), 0., 2., 1.]])]
        image = rasterize_positive_camera_z(p, self.matrix, self.matrix, self.aug, (8, 8))
        self.assertEqual(int(torch.count_nonzero(image)), 0)

    def test_projection_gradient_to_visible_z_only(self):
        points = torch.tensor([[2., 4., 2., 1.], [5., 10., 5., 1.]], requires_grad=True)
        image = rasterize_positive_camera_z([points], self.matrix, self.matrix, self.aug, (8, 8))
        image.sum().backward()
        self.assertTrue(torch.equal(points.grad, torch.tensor([[0., 0., 1., 0.], [0., 0., 0., 0.]])))


@unittest.skipUnless(importlib.util.find_spec('mmengine'), 'Full registered model requires team Docker')
class RegisteredTests(unittest.TestCase):
    def build(self):
        from mmengine.config import Config
        from mmengine.utils import import_modules_from_strings
        from mmdet3d.registry import MODELS
        from mmdet3d.utils import register_all_modules
        register_all_modules()
        cfg = Config.fromfile(str(ROOT / 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_nearest_depth_trial.py'))
        import_modules_from_strings(**cfg.custom_imports)
        return MODELS.build(cfg.model.view_transform)

    def test_state_and_geometry_unchanged(self):
        candidate = self.build()
        from perception.detection3d.bevfusion.plugin.morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform
        from mmengine.config import Config
        cfg = Config.fromfile(str(ROOT / 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py'))
        args = dict(cfg.model.view_transform)
        args.pop('type')
        original = MoraiSeparatedRectangularDepthLSSTransform(**args)
        candidate.load_state_dict(original.state_dict(), strict=True)
        self.assertEqual(list(original.state_dict()), list(candidate.state_dict()))
        self.assertEqual(candidate.image_size, original.image_size)
        self.assertTrue(torch.equal(candidate.nx, original.nx))
        self.assertTrue(torch.equal(candidate.dx, original.dx))
        self.assertTrue(torch.equal(candidate.frustum, original.frustum))

    def test_none_real_parent_delegate(self):
        candidate = self.build()
        candidate.depth_raster_policy = None
        from perception.detection3d.bevfusion.plugin.morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform
        value = object()
        with patch.object(MoraiSeparatedRectangularDepthLSSTransform, 'forward', return_value=value) as parent:
            self.assertIs(candidate(None, None, None, None, None, None, None, None), value)
            parent.assert_called_once()


if __name__ == '__main__':
    unittest.main()
