import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_base5_primary_obstacle_range import (band, cap_for_size, component_bbox,
    physical, scope, unique_index, verify_gt_join, verify_scale_parent)


class PrimaryObstacleRangeAuditTest(unittest.TestCase):
    def setUp(self):
        self.actor = dict(unique_id=8500, data_id=123, name='Box', source_list='objectList',
                          population_pool='obstacle_box_barrel', motion='static')
        self.spec = dict(family='obstacle', mission=False)
        self.scenario = dict(objectList=[dict(UNIQUEID=8500, DataID=123,
                                             scale=dict(x=1, y=1, z=1))])

    def test_scope_uses_runtime_pool_not_nonexistent_semantic(self):
        self.assertTrue(physical(self.actor))
        self.assertFalse(physical(dict(self.actor, source_list='pedestrianList')))
        self.assertFalse(physical(dict(self.actor, population_pool='stroller')))

    def test_strict_scope_preserves_independent_avoidance_solver(self):
        self.assertTrue(scope(self.spec, self.actor))
        phased = dict(self.spec, avoidance_phase='left_pass')
        self.assertTrue(scope(phased, self.actor, strict=False))
        self.assertFalse(scope(phased, self.actor))
        self.assertFalse(scope(dict(self.spec, mission=True), self.actor))
        self.assertFalse(scope(dict(self.spec, family='shadow'), self.actor))

    def test_compact_rule_distinguishes_bollard_and_wide_barrier(self):
        self.assertEqual(cap_for_size([.37, .37, .856]), 30)
        self.assertEqual(cap_for_size([.393, 1.921, .821]), 40)
        self.assertEqual(cap_for_size([.213, .213, .752]), 20)
        self.assertEqual(cap_for_size([.37, .37, .856], compact=False), 40)
        self.assertIsNone(cap_for_size([.60, 1., 1.]))

    def test_thresholds_are_strict_and_inputs_positive_finite(self):
        self.assertEqual(cap_for_size([.25, 1., 1.]), 30)
        self.assertEqual(cap_for_size([.35, 1.5, 1.]), 40)
        self.assertEqual(cap_for_size([.40, 1., 1.]), 40)
        for invalid in ([0, 1, 1], [float('nan'), 1, 1], [True, 1, 1]):
            with self.assertRaises(ValueError):
                cap_for_size(invalid)

    def test_size_reference_requires_scale_one_static_single_component(self):
        verify_scale_parent(self.scenario, self.actor, 1)
        changed = copy.deepcopy(self.scenario)
        changed['objectList'][0]['scale']['z'] = 1.01
        for scenario, actor, count in ((changed, self.actor, 1),
                (self.scenario, dict(self.actor, motion='walking'), 1),
                (self.scenario, self.actor, 2)):
            with self.assertRaises(ValueError):
                verify_scale_parent(scenario, actor, count)

    def test_join_validates_upright_not_raw_size(self):
        component = dict(parent_uid=8500, asset='Box', center_lidar_m=[1, 2, 3],
                         size_m=[.2, .8, .4], rotation_rpy=[.1, .2, .7])
        box = component_bbox(component)
        instance = dict(parent_uid=8500, asset='Box', bbox_label_3d=2, bbox_3d=box)
        support = dict(asset='Box', cls='obstacle', center_lidar_m=[1, 2, 3])
        verify_gt_join(component, instance, support)
        with self.assertRaises(ValueError):
            verify_gt_join(component, dict(instance, bbox_3d=[1, 2, 3, .2, .8, .4, .7]), support)
        rotated = dict(component, rotation_rpy=[.1, .2, -.3])
        np.testing.assert_array_equal(component_bbox(rotated)[3:6], box[3:6])

    def test_duplicate_keys_fail_instead_of_silent_overwrite(self):
        with self.assertRaises(ValueError):
            unique_index([dict(id=1), dict(id=1)], lambda x: x['id'], 'test ID')

    def test_distance_bands_half_open(self):
        self.assertEqual(band(29.999), '20_30m')
        self.assertEqual(band(30.), '30_40m')
        self.assertEqual(band(100.), '100plus_m')
        with self.assertRaises(ValueError):
            band(-.1)


if __name__ == '__main__':
    unittest.main()
