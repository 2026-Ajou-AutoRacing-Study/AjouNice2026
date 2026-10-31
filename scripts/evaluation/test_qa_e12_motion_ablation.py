"""Independent analytic baseline checks; no ROS or runtime side effects."""
import math
import unittest

import numpy as np

from qa_e12_motion_ablation import analytic_paths, check_close


class IndependentMotionQATest(unittest.TestCase):
    def obj(self, **values):
        return dict(dict(center=[2., 3., 0.], yaw=0., yaw_rate=0.,
                         world_velocity=[4., 1.], body_velocity=[4., 1.]), **values)

    def test_cartesian_preserves_lateral_ctrv_uses_signed_forward_only(self):
        paths = analytic_paths(self.obj())
        self.assertTrue(np.allclose(paths['cv'][-1], [14., 6.]))
        self.assertTrue(np.allclose(paths['ctrv'][-1], [14., 3.]))

    def test_quarter_circle(self):
        paths = analytic_paths(self.obj(yaw_rate=math.pi/6.))
        radius = 4. / (math.pi/6.)
        self.assertTrue(np.allclose(paths['ctrv'][-1], [2.+radius, 3.+radius]))

    def test_reverse_velocity_is_not_made_positive(self):
        paths = analytic_paths(self.obj(world_velocity=[-4., 0.], body_velocity=[-4., 0.]))
        self.assertTrue(np.allclose(paths['ctrv'][-1], [-10., 3.]))

    def test_audit_rejects_nonfinite_or_shape_mismatch(self):
        for a, b in (([float('nan')], [1.]), ([1., 2.], [1.]), ([1.], [2.])):
            with self.assertRaises(ValueError):
                check_close(a, b, 'test')


if __name__ == '__main__':
    unittest.main()
