import copy
import json
from pathlib import Path
import random
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_obstacle_front_support as front
import base5_obstacle_rich_floor as rich
import base5_obstacle_support_policy as support


class FrontSupportTests(unittest.TestCase):
    def item(self, slot='slot0', data_id=10, size=(.5, .5, 1.), seed=1):
        actors = [dict(data_id=1, source_list='objectList', motion='static',
                       population_pool='obstacle_box', estimated_size=[1., 1., 1.]),
                  dict(data_id=2, source_list='objectList', motion='static',
                       population_pool='obstacle_box', estimated_size=[1., 1., 1.]),
                  dict(data_id=data_id, source_list='objectList', motion='static',
                       population_pool='obstacle_box', estimated_size=list(size))]
        row = dict(family='obstacle', mission=False, population_mode='generic_regular',
                   actors=3, population_max_actors=8, population_budget_bounds=[12, 16],
                   seed=seed, role='rear', distance_bounds=[15, 30],
                   frozen_population_data_ids=[1, 2, data_id],
                   secondary_obstacle_support=dict(policy=rich.POLICY, variant='richfloor',
                       ordinals=[2], new_actor_ordinals=[2], protected_near_ordinals=[1],
                       protected_mid_ordinals=[], primary_preserved=True,
                       mission_and_moving_excluded=True, original_generation_attempt=0))
        return slot, row, actors

    def selected(self, size=(.5, .5, 1.)):
        items = [self.item('slot' + str(i), size=size, seed=i) for i in range(4)]
        rows = front.assign_plans(items)
        return next(row for row in rows.values() if row[front.PLAN_KEY]['selected'])

    def legacy(self, row, ordinal=2, attempt=0, size=(.5, .5, 1.)):
        asset = dict(data_id=row['frozen_population_data_ids'][ordinal],
                     source_list='objectList', semantic_class='obstacle')
        return asset, rich.sample(row, ordinal, asset, size, 'rear', attempt)

    def test_per_asset_selection_is_half_and_order_independent(self):
        items = [self.item(f'{asset}_{i}', data_id=asset, seed=i)
                 for asset, count in ((10, 35), (11, 36), (12, 1))
                 for i in range(count)]
        before = copy.deepcopy(items)
        a, b = front.assign_plans(items), front.assign_plans(reversed(items))
        self.assertEqual(a, b)
        self.assertEqual(items, before)
        for data_id, count in ((10, 35), (11, 36), (12, 1)):
            plans = [row[front.PLAN_KEY] for row in a.values()
                     if row[front.PLAN_KEY]['data_id'] == data_id]
            selected = sum(p['selected'] for p in plans)
            self.assertLessEqual(abs(selected - (count - selected)), 1)
            self.assertEqual({p['asset_selected_count'] for p in plans}, {selected})

    def test_plan_changes_no_other_spec_and_no_actor_list(self):
        item = self.item()
        planned = front.assign_plans([item])['slot0']
        original = copy.deepcopy(planned)
        original.pop(front.PLAN_KEY)
        self.assertEqual(original, item[1])
        self.assertEqual(planned['frozen_population_data_ids'], [1, 2, 10])

    def test_no_plan_unselected_and_original_actor_are_exact_legacy(self):
        _, row, _ = self.item()
        asset, proof = self.legacy(row)
        self.assertIs(front.apply(row, 2, asset, [.5, .5, 1.], 0, proof), proof)
        planned = front.assign_plans([self.item('s' + str(i), seed=i) for i in range(4)])
        row = next(r for r in planned.values() if not r[front.PLAN_KEY]['selected'])
        asset, proof = self.legacy(row)
        self.assertIs(front.apply(row, 2, asset, [.5, .5, 1.], 0, proof), proof)
        row = self.selected()
        asset, proof = self.legacy(row, ordinal=1, attempt=1)
        self.assertIs(front.apply(row, 1, asset, [.5, .5, 1.], 1, proof), proof)
        self.assertEqual(proof['distance_bounds_m'], [6., 18.])

    def test_selected_is_deterministic_bounded_and_keeps_legacy_flags(self):
        row = self.selected()
        snapshot = copy.deepcopy(row)
        asset, proof = self.legacy(row)
        original = copy.deepcopy(proof)
        for attempt in (0, 1, 17, 512):
            out = front.apply(row, 2, asset, [.5, .5, 1.], attempt, proof)
            self.assertEqual(out, front.apply(row, 2, asset, [.5, .5, 1.], attempt, proof))
            self.assertEqual(out['role'], 'front')
            self.assertEqual(out['distance_bounds_m'], [6., 10.])
            self.assertTrue(6. < out['target_center_distance_m'] < 10.)
            for key in ('policy', 'ordinal', 'added_actor', 'count_changed',
                        'primary_changed', 'band_fixed_across_retry'):
                self.assertEqual(out[key], proof[key])
        self.assertEqual(proof, original)
        self.assertEqual(row, snapshot)

    def test_large_geometry_uses_eight_to_fourteen(self):
        row = self.selected(size=(3.1, 1., .5))
        asset, proof = self.legacy(row, size=(3.1, 1., .5))
        self.assertEqual(front.apply(row, 2, asset, [3.1, 1., .5], 0, proof)
                         ['distance_bounds_m'], [8., 14.])
        self.assertEqual(front.distance_bounds([2., 2., 1.]), [8., 14.])

    def test_no_global_rng_changes(self):
        state = random.getstate()
        row = self.selected()
        asset, proof = self.legacy(row)
        front.apply(row, 2, asset, [.5, .5, 1.], 3, proof)
        self.assertEqual(state, random.getstate())

    def test_duplicate_slots_and_reassignment_are_rejected(self):
        with self.assertRaises(ValueError):
            front.assign_plans([self.item(), self.item()])
        item = self.item()
        planned = front.assign_plans([item])['slot0']
        with self.assertRaises(ValueError):
            front.assign_plans([('slot0', planned, item[2])])

    def test_missions_movers_wrong_container_and_primary_are_rejected(self):
        for edit in ('mission', 'moving', 'container', 'primary', 'count', 'ids'):
            slot, row, actors = self.item()
            if edit == 'mission': row['mission'] = True
            if edit == 'moving': actors[1]['motion'] = 'walking'
            if edit == 'container': actors[2]['source_list'] = 'pedestrianList'
            if edit == 'primary': row['secondary_obstacle_support']['new_actor_ordinals'] = [0]
            if edit == 'count': row['actors'] = 4
            if edit == 'ids': row['frozen_population_data_ids'][-1] = 999
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                front.assign_plans([(slot, row, actors)])

    def test_frozen_seed_source_assignment_and_identity_tampering_rejected(self):
        row = self.selected()
        mutations = [('seed', 99), ('ordinal', 0), ('data_id', 99),
                     ('selected', False), ('implementation_sha256', '0' * 64),
                     ('distance_bounds_m', [1., 100.]), ('free_yaw_preserved', False)]
        for key, value in mutations:
            changed = copy.deepcopy(row)
            if key == 'seed': changed['seed'] = value
            else: changed[front.PLAN_KEY][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                front.validate_plan(changed)

    def test_apply_rejects_wrong_asset_geometry_attempt_and_missing_proof(self):
        row = self.selected()
        asset, proof = self.legacy(row)
        for a, size, attempt, old in ((dict(asset, data_id=999), [.5, .5, 1.], 0, proof),
                                     (asset, [4., 4., 1.], 0, proof),
                                     (asset, [.5, .5, 1.], -1, proof),
                                     (asset, [.5, .5, 1.], 0, None)):
            with self.assertRaises(ValueError):
                front.apply(row, 2, a, size, attempt, old)

    def test_no_added_actor_leaves_noneligible_scenes_unchanged(self):
        slot, row, actors = self.item()
        row['secondary_obstacle_support']['new_actor_ordinals'] = []
        row['mission'] = True
        self.assertEqual(front.assign_plans([(slot, row, actors)])[slot], row)

    def test_actual_role_and_range_must_match_frozen_plan(self):
        row = self.selected()
        front.verify_actual(row, 2, 8., 'front')
        for distance, role in ((5.9, 'front'), (10., 'front'), (8., 'rear'),
                               (float('nan'), 'front')):
            with self.assertRaises(ValueError):
                front.verify_actual(row, 2, distance, role)
        front.verify_actual(row, 1, 25., 'rear')

    def test_bad_dimensions_fail_closed(self):
        for size in ((0., 1., 1.), (1., float('nan'), 1.), (True, 1., 1.), (1., 1.)):
            with self.assertRaises(ValueError):
                front.distance_bounds(size)

    def test_source_pin_is_checked_before_replacement(self):
        row = self.selected()
        with mock.patch.object(front, 'implementation_sha256', return_value='changed'):
            with self.assertRaises(ValueError):
                front.validate_plan(row)

    def test_generator_dispatch_is_exact_legacy_without_plan(self):
        _, row, _ = self.item()
        for ordinal, attempt in ((1, 0), (1, 3), (2, 0), (2, 512)):
            asset, expected = self.legacy(row, ordinal=ordinal, attempt=attempt)
            self.assertEqual(support.sample(row, ordinal, asset, [.5, .5, 1.],
                                            'rear', attempt), expected)

    def test_generator_dispatch_applies_only_selected_added_actor(self):
        rows = front.assign_plans([self.item('s' + str(i), seed=i) for i in range(4)])
        for row in rows.values():
            for ordinal, attempt in ((1, 3), (2, 0), (2, 512)):
                asset, expected = self.legacy(row, ordinal=ordinal, attempt=attempt)
                actual = support.sample(row, ordinal, asset, [.5, .5, 1.], 'rear', attempt)
                if ordinal == 2 and row[front.PLAN_KEY]['selected']:
                    self.assertEqual(actual['role'], 'front')
                    self.assertIn('front_support', actual)
                else:
                    self.assertEqual(actual, expected)

    def test_json_frozen_subset_keeps_assignment_and_retry_stream(self):
        row = self.selected()
        resumed = json.loads(json.dumps(row))
        asset, proof = self.legacy(row)
        for attempt in (0, 7, 512):
            self.assertEqual(front.apply(row, 2, asset, [.5, .5, 1.], attempt, proof),
                             front.apply(resumed, 2, asset, [.5, .5, 1.], attempt, proof))
        self.assertEqual(front.validate_plan(resumed), row[front.PLAN_KEY])


if __name__ == '__main__':
    unittest.main()
