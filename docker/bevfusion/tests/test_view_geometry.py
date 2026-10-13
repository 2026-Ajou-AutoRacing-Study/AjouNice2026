import unittest
from types import SimpleNamespace
import torch
from app.view_geometry import restore_view_geometry


class ViewGeometryTest(unittest.TestCase):
    def test_same_shape_stale_checkpoint_bounds_are_reset(self):
        view = SimpleNamespace(xbound=[-60, 60, .3], ybound=[-30, 30, .3],
            zbound=[-10, 10, 20], dx=torch.nn.Parameter(torch.zeros(3), requires_grad=False),
            bx=torch.nn.Parameter(torch.zeros(3), requires_grad=False),
            nx=torch.nn.Parameter(torch.ones(3, dtype=torch.long), requires_grad=False),
            D=2, create_frustum=lambda: torch.ones(2, 4, 8, 3), frustum=None)
        restore_view_geometry(view)
        self.assertEqual(view.nx.tolist(), [400, 200, 1])
        self.assertAlmostEqual(view.bx[0].item(), -59.85, places=4)
        self.assertEqual(tuple(view.frustum.shape), (2, 4, 8, 3))
        self.assertFalse(view.frustum.requires_grad)

    def test_depth_bin_mismatch_is_rejected(self):
        view = SimpleNamespace(xbound=[-30, 30, .3], ybound=[-30, 30, .3],
            zbound=[-10, 10, 20], D=118, create_frustum=lambda: torch.zeros(2, 1, 1, 3))
        with self.assertRaisesRegex(ValueError, 'depth bin'):
            restore_view_geometry(view)
