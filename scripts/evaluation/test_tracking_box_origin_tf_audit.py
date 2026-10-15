"""Actual ROS generated messages/tf2, no ROS master or GPU."""
from pathlib import Path
import unittest

import numpy as np

from audit_tracking_continuity_box_origin_tf import (
    ROOT, core_ema_fixture, map_future_fixture, same_non_z_trace, transform_fixture)
from audit_tracking_continuity_box_origin_private_ros import private_uri, typed_world_center


class BoxOriginActualTFTest(unittest.TestCase):
    def test_yaw_only_transform_does_not_shift_xy(self):
        result = transform_fixture((0., 0., 83.), 1.8)
        np.testing.assert_allclose(result['actual_map_delta_m'], [0., 0., .9], atol=1.e-10)

    def test_full_rpy_actual_tf2_changes_xy_by_rotated_half_height(self):
        result = transform_fixture((8., -5., 63.), 1.8)
        self.assertGreater(result['actual_xy_shift_m'], .1)
        np.testing.assert_allclose(result['actual_map_delta_m'], result['expected_map_delta_m'], atol=1.e-10)

    def test_origin_only_compare_does_not_hide_other_state_changes(self):
        a = [dict(tracks=[dict(center=[1., 2., 3.], score=.9)])]
        b = [dict(tracks=[dict(center=[1., 2., 4.], score=.9)])]
        self.assertTrue(same_non_z_trace(a, b))
        b[0]['tracks'][0]['score'] = .8
        self.assertFalse(same_non_z_trace(a, b))

    def test_actual_map_future_does_not_preserve_half_height(self):
        result = map_future_fixture()
        np.testing.assert_allclose([r['origin_delta_m'] for r in result['values']], [.6, .3, 0., 0.], atol=1.e-10)

    def test_original_core_fresh_reset_and_mixed_ema(self):
        binary = ROOT/'planner_logs/analysis/261014_offline_tracking_review/packet_dense_v4/core_build/original_core'
        if not binary.exists():
            self.skipTest('Ignored original-core qualification binary unavailable; run audit with an explicit binary')
        result, _, _ = core_ema_fixture(binary)
        self.assertTrue(result['fresh_reset_immediate_correct_z'])
        np.testing.assert_allclose(result['mixed_origin_ema_samples'][0]['residual_m'], .63, atol=1.e-10)

    def test_private_master_rejects_standard_operational_and_system_ports(self):
        for port in (80, 11311, 65536):
            with self.assertRaises(ValueError):
                private_uri(port)

    def test_actual_node_float32_center_oracle_is_explicit(self):
        physical = [407.59271412668244, 103.2172491651218, 26.4]
        typed = typed_world_center(physical)
        np.testing.assert_array_equal(typed, np.asarray(physical, dtype=np.float32).astype(np.float64))
        self.assertGreater(np.linalg.norm(typed-physical), 0.)


if __name__ == '__main__':
    unittest.main()
