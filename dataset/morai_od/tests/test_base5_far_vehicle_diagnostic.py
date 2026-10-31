"""Offline probe contracts, without constructing generator context or ROS."""
import copy
import sys
from collections import Counter
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_far_vehicle_diagnostic as probe


class FarDiagnosticTest(unittest.TestCase):
    def test_full_factor_coverage(self):
        rows = probe.specifications()
        self.assertEqual(len(rows), 16)
        self.assertEqual(Counter(r['distance_m'] for r in rows), {85: 6, 95: 6, 60: 4})
        self.assertEqual(Counter(r['role'] for r in rows), {'front': 8, 'rear': 8})
        self.assertEqual(Counter(r['lane'] for r in rows), {'same': 8, 'adjacent': 8})
        for asset in probe.ASSETS:
            self.assertEqual(sum(r['asset'] == asset and r['distance_m'] >= 80 for r in rows), 4)

    def test_no_training_budget_or_frozen_population_inheritance(self):
        source = dict(family='highway', frozen_population_data_ids=[1, 2],
            population_budget_bounds=[10, 14], original_role='rear', required_secondary_pool='obstacle',
            population_rng_reference={'actors': 3}, spatial_sampling_plan={'old': True})
        before = copy.deepcopy(source)
        row = probe.make_row(source, probe.specifications()[0], 30100007, 0)
        self.assertEqual(source, before)
        self.assertEqual(row['actors'], 1)
        self.assertTrue(row['vehicle_only_population'])
        self.assertEqual(row['primary_identity'], 30100007)
        self.assertEqual(row['distance_bounds'], [84., 86.])
        for key in ('frozen_population_data_ids', 'population_budget_bounds', 'spatial_sampling_plan'):
            self.assertNotIn(key, row)

    def test_lane_proof_is_not_ego_lateral_offset(self):
        meta = dict(base=dict(link_idx='a', yaw=90), runtime_actors=[dict(
            placement_yaw_deg=90, map_ground_initializer=dict(link_id='b'),
            reference_frames=dict(road_lateral_m=0, ego_lateral_m=20))])
        paths = {6: ['a', 'b'], 8: ['c'], 10: ['d']}
        proof = probe.lane_proof(meta, paths)
        self.assertEqual(proof['lane'], 'same')
        meta['runtime_actors'][0]['map_ground_initializer']['link_id'] = 'c'
        self.assertEqual(probe.lane_proof(meta, paths)['lane'], 'adjacent')
        meta['runtime_actors'][0]['map_ground_initializer']['link_id'] = 'd'
        self.assertIsNone(probe.lane_proof(meta, paths))

    def test_unknown_link_and_bend_rejected(self):
        actor = dict(placement_yaw_deg=6, map_ground_initializer=dict(link_id='a'),
                     reference_frames=dict(road_lateral_m=0, ego_lateral_m=0))
        meta = dict(base=dict(link_idx='a', yaw=0), runtime_actors=[actor])
        self.assertIsNone(probe.lane_proof(meta, {6: ['a']}))
        actor['placement_yaw_deg'] = 0
        actor['map_ground_initializer']['link_id'] = 'unknown'
        self.assertIsNone(probe.lane_proof(meta, {6: ['a']}))


if __name__ == '__main__':
    unittest.main()
