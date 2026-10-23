"""CPU probe policy tests; no live simulator or generator context."""
import copy
import sys
from collections import Counter
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_near_shape_diagnostic as probe


class NearShapeDiagnosticTest(unittest.TestCase):
    def test_deck_has_balanced_shape_range_yaw(self):
        rows = probe.specifications()
        self.assertEqual(len(rows), 16)
        self.assertEqual(Counter(r['distance_m'] for r in rows), {8: 8, 22: 8})
        self.assertEqual(Counter(r['nominal_road_yaw_deg'] for r in rows), {0: 8, 90: 8})
        self.assertEqual(Counter(r['source_anchor_index'] for r in rows), {0: 8, 1: 8})
        self.assertTrue(all(sum(r['asset'] == a for r in rows) == 4 for a in probe.ASSETS))

    def test_source_preserved_and_free_sector_is_explicit(self):
        base = dict(static_yaw_policy=dict(policy='static-free-sector-yaw-v130',
            eligible_data_ids=[1], scope='generic'), frozen_population_data_ids=[1, 2],
            population_budget_bounds=[10, 14], avoidance_phase='alongside',
            spatial_sampling_plan={'preserved': True})
        before = copy.deepcopy(base)
        row = probe.make_row(base, probe.specifications()[0], 1, 0)
        self.assertEqual(base, before)
        self.assertEqual(row['static_yaw_policy']['primary_sector'], 6)
        self.assertEqual(row['static_yaw_policy']['primary_mode'], 'uniform')
        self.assertEqual(row['spatial_sampling_plan'], {'preserved': True})
        self.assertNotIn('avoidance_phase', row)
        self.assertNotIn('frozen_population_data_ids', row)
        self.assertEqual(row['actors'], 1)

    def test_ninety_degree_sector_and_wrapping(self):
        condition = dict(asset=probe.ASSETS[0], distance_m=22, nominal_road_yaw_deg=90)
        base = dict(static_yaw_policy=dict(policy='static-free-sector-yaw-v130'))
        row = probe.make_row(base, condition, 1, 0)
        self.assertEqual(row['static_yaw_policy']['primary_sector'], 9)
        self.assertEqual(probe.yaw_error(101, 90), 11)
        self.assertEqual(probe.yaw_error(359, 0), -1)


if __name__ == '__main__':
    unittest.main()
