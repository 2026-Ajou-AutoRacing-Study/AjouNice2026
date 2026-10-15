import copy
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_parked_npc_placement as placement


class ParkedNpcPlacementTests(unittest.TestCase):
    def asset(self):
        return dict(data_id=10, source_list='vehicleList')

    def ground(self, nonroad=0, distance=.1):
        corners = [dict(usable=True, confidence='roadside_estimate' if i < nonroad else 'road_strip',
                        nearest_road_distance_m=distance) for i in range(4)]
        return dict(footprint=dict(all_supported=True, samples=[dict(usable=True)] + corners))

    def test_opt_in_and_physical_long_static_scope(self):
        for policy, asset, motion, size in (
                (None, self.asset(), 'static', [12, 3, 4]),
                (placement.DEFAULT, dict(self.asset(), source_list='objectList'), 'static', [12, 3, 4]),
                (placement.DEFAULT, self.asset(), 'moving', [12, 3, 4]),
                (placement.DEFAULT, self.asset(), 'static', [5, 2, 2])):
            self.assertIsNone(placement.screen(asset, motion, size, self.ground(), policy))

    def test_map_height_availability_cannot_certify_road_support(self):
        proof = placement.screen(self.asset(), 'static', [12, 3, 4], self.ground(4, 6), placement.DEFAULT)
        self.assertFalse(proof['road_supported']); self.assertFalse(proof['contact_verified'])

    def test_path_locked_traffic_is_not_replanned_by_static_yaw_guard(self):
        for row in (dict(family='roundabout', static_yaw_policy=dict(scope='stationary_avoidance')),
                    dict(family='highway', static_yaw_policy=dict(scope='stationary_avoidance')),
                    dict(family='obstacle', static_yaw_policy=dict(scope='generic'))):
            self.assertIsNone(placement.screen(self.asset(), 'static', [12, 3, 4], self.ground(4, 6), placement.DEFAULT, row))

    def test_allow_small_map_edge_uncertainty_without_contact_claim(self):
        proof = placement.screen(self.asset(), 'static', [12, 3, 4], self.ground(1), placement.DEFAULT)
        self.assertTrue(proof['road_supported']); self.assertFalse(proof['contact_verified'])

    def test_missing_nonfinite_or_distant_corners_are_rejected(self):
        for ground in (None, self.ground(0, float('nan')), self.ground(0, 3)):
            proof = placement.screen(self.asset(), 'static', [12, 3, 4], ground, placement.DEFAULT)
            self.assertFalse(proof['road_supported'])

    def test_override_is_explicit_bound_to_actor_and_does_not_mutate_spec(self):
        row = dict(parked_npc_yaw_overrides={'0': dict(policy=placement.POLICY, data_id=10, mode='road_aligned')})
        original = copy.deepcopy(row)
        self.assertEqual(placement.yaw_override(row, self.asset(), 0, 'static', [12, 3, 4]), 'road_aligned')
        self.assertIsNone(placement.yaw_override(row, self.asset(), 1, 'static'))
        self.assertEqual(row, original)
        for asset, motion in ((dict(self.asset(), data_id=20), 'static'),
                              (dict(self.asset(), source_list='objectList'), 'static'),
                              (self.asset(), 'moving')):
            with self.assertRaises(ValueError): placement.yaw_override(row, asset, 0, motion, [12, 3, 4])
        for size in (None, [5, 2, 2]):
            with self.assertRaises(ValueError): placement.yaw_override(row, self.asset(), 0, 'static', size)


if __name__ == '__main__':
    unittest.main()
