"""Synthetic offline ablation checks; no ROS master, GPU or simulator."""
import math
from types import SimpleNamespace
import unittest

import numpy as np

from analyze_e12_motion_ablation import (causal_velocity, cartesian_cv, error_metrics,
    evaluation_axes, fixed_route_prediction, lifecycle, replace_longitudinal)


class MotionAblationTest(unittest.TestCase):
    def test_velocity_never_uses_future_even_if_large_future_change(self):
        series = ([0., .05, .1, .15, .2], [[0., 0.], [.1, 0.], [.2, 0.], [100., 0.], [200., 0.]], [0, 1, 2, 3, 4])
        value, reason, span = causal_velocity(series, .1, with_span=True)
        np.testing.assert_allclose(value, [2., 0.]); self.assertIsNone(reason)
        self.assertEqual(span['stamps'], [0., .05, .1])
        self.assertIsNone(causal_velocity(series, .05)[0])

    def test_velocity_no_nonadjacent_or_gapped_native_fill(self):
        self.assertIsNone(causal_velocity(([0., .1], [[0., 0.], [.1, 0.]], [0, 2]), .1)[0])
        self.assertIsNone(causal_velocity(([0., .2], [[0., 0.], [.2, 0.]], [0, 1]), .2)[0])

    def test_gt_longitudinal_replacement_preserves_lateral_and_heading(self):
        obj = dict(center=[0., 0., 0.], yaw=math.pi/2, body_velocity=[1., 3.], world_velocity=[-3., 1.], yaw_rate=.2)
        result = replace_longitudinal(obj, [0., 4.])
        np.testing.assert_allclose(result['body_velocity'], [4., 3.])
        np.testing.assert_allclose(result['world_velocity'], [-3., 4.])
        self.assertEqual(result['yaw_rate'], .2); self.assertEqual(obj['body_velocity'], [1., 3.])

    def test_cartesian_cv_keeps_world_lateral_component(self):
        result = cartesian_cv(dict(center=[1., 2., 0.], world_velocity=[2., 3.]))
        np.testing.assert_allclose(result[-1], [7., 11.])

    def test_curved_gt_axes_follow_future_trajectory_not_fixed_current_heading(self):
        angle = np.arange(1, 31)/10.
        reference = np.column_stack([np.sin(angle), 1.-np.cos(angle)])
        axes = evaluation_axes([0., 0., 0.], reference)
        self.assertGreater(axes[14][1], .99)
        offset = np.asarray([[-v[1], v[0]] for v in axes])
        metrics = error_metrics(reference+offset, reference, axes, 3)
        self.assertAlmostEqual(metrics['lateral_signed_ADE_m'], 1.)
        self.assertAlmostEqual(metrics['longitudinal_abs_ADE_m'], 0.)
        self.assertAlmostEqual(metrics['FDE_m']**2,
            metrics['longitudinal_signed_FDE_m']**2+metrics['lateral_signed_FDE_m']**2)

    def test_static_keeps_euclidean_without_fake_heading_components(self):
        reference = np.zeros((30, 2)); axes = evaluation_axes([0., 0., 0.], reference)
        result = error_metrics(np.ones((30, 2)), reference, axes, 3)
        self.assertEqual(result['component_samples'], 0)
        self.assertAlmostEqual(result['FDE_m'], math.sqrt(2.))
        self.assertIsNone(result['lateral_abs_FDE_m'])

    def test_fixed_route_never_clamps_new_oracle_speed(self):
        network = SimpleNamespace(config=SimpleNamespace(lateral_convergence_time_sec=1.5))
        geometry = SimpleNamespace(length=4., sample=lambda d: (d, 0., 0.))
        projection = SimpleNamespace(project=lambda x, y: SimpleNamespace(arc_length=0., x=0., y=0., z=0.))
        network._route_geometry = lambda ids: (geometry, projection)
        path, reason = fixed_route_prediction(network, dict(center=[0., 0., 0.], body_velocity=[2., 0.]), [1])
        self.assertIsNone(path); self.assertIn('domain', reason)
        path, reason = fixed_route_prediction(network, dict(center=[0., 1., 0.], body_velocity=[1., 0.]), [1])
        self.assertIsNone(reason); np.testing.assert_allclose(path[14], [1.5, 0.])

    def test_reacquisition_requires_real_confirmed_past_association_continuity(self):
        def frame(t, associated, native):
            return dict(stamp=t, native_index=native, tracks=[dict(id=1, confirmed=True,
                tracker_label=1, associated=associated, birth_stamp=0., last_measurement_stamp=t if associated else 1.)])
        case = dict(frames=[frame(1., False, 0), frame(1.1, True, 1), frame(1.2, False, 2),
                            frame(1.3, True, 3), frame(1.4, False, 4), frame(1.5, True, 7)])
        rows = lifecycle(case)
        self.assertFalse(rows[(1.1, '1')]['reacquisition_event'])
        self.assertTrue(rows[(1.3, '1')]['reacquisition_event'])
        self.assertFalse(rows[(1.5, '1')]['reacquisition_event'])


if __name__ == '__main__':
    unittest.main()
