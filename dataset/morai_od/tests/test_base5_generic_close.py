import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import generate_base5_independent as gen


class GenericCloseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = gen.geo.load_json(gen.CONFIG)
        cls.ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)

    def test_only_near_generic_cells_change(self):
        old = copy.deepcopy(self.config);old.pop('generic_close_observation')
        before = {r['index']: r for r in gen.specs(old)}
        changed = []
        for row in gen.specs(self.config):
            previous = before[row['index']]
            if row.get('avoidance_origin') == 'generic_close':
                changed.append(row)
                self.assertFalse(row['mission'])
                self.assertLessEqual(previous['distance_bounds'][1], 15)
                self.assertEqual(row['original_distance_bounds'], previous['distance_bounds'])
                self.assertEqual(row['original_role'], previous['role'])
                for key in ('asset_index', 'seed', 'population_mode', 'population_layout'):
                    self.assertEqual(row[key], previous[key])
            else:
                self.assertEqual(row, previous)
        self.assertEqual(len(changed), 175)

    def test_left_right_balance(self):
        rows = gen.specs(self.config)
        for family in self.config['generic_close_observation']['families']:
            group = [r for r in rows if r['family'] == family and r.get('avoidance_origin') == 'generic_close']
            left = sum(r['avoidance_side'] == 'left' for r in group)
            self.assertLessEqual(abs(left - (len(group) - left)), 1)

    def test_surface_gap_and_population_are_both_preserved(self):
        rows = gen.specs(self.config)
        for family in ('obstacle', 'other_vehicle'):
            for side in ('left', 'right'):
                row = next(r for r in rows if r['family'] == family and r.get('avoidance_origin') == 'generic_close' and r['avoidance_side'] == side)
                for attempt in range(512):
                    try:
                        _, meta = gen.build(row, self.config, self.ctx, attempt)
                        break
                    except ValueError:
                        continue
                else:
                    self.fail('Unfillable close observation')
                actor = meta['runtime_actors'][0];obs = meta['avoidance_observation']
                self.assertEqual(obs['origin'], 'generic_close')
                self.assertAlmostEqual(meta['ego']['x'], meta['base']['x'])
                self.assertAlmostEqual(meta['ego']['y'], meta['base']['y'])
                self.assertTrue(obs['ego_road_anchor_preserved'])
                self.assertEqual(meta['ego_lateral_offset_m'], 0.)
                self.assertAlmostEqual(actor['planned_surface_clearance_m'], obs['requested_surface_gap_m'], places=5)
                floor = .3 if actor['geometry_reference_known'] else 1.
                self.assertGreaterEqual(actor['planned_surface_clearance_m'], floor - 1e-6)
                self.assertEqual(actor['actual_ego_role'], row['role'])
                if row['population_mode'] != 'generic_single':
                    lo, hi = row['population_budget_bounds']
                    self.assertTrue(lo <= meta['population_budget']['total_cost'] <= hi)


if __name__ == '__main__':
    unittest.main()
