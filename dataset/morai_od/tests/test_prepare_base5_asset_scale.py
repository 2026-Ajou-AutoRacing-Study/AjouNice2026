import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_asset_scale as scale
import prepare_base5_asset_scale as prepare


class ScalePreparationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.policy = scale.candidate_policy([0, 2, 1])

    def test_candidate_only_policy_blocks_before_any_source_read_or_output(self):
        with patch.object(prepare, 'read', return_value=self.policy) as read:
            with self.assertRaisesRegex(scale.ScaleContractError, 'No native-qualified'):
                prepare.prepare(self.root / 'source.json', self.root / 'policy.json', self.root / 'output')
        read.assert_called_once_with(self.root / 'policy.json')
        self.assertFalse((self.root / 'output').exists())

    def test_existing_output_and_unbounded_search_are_rejected(self):
        for maximum in (0, 4097):
            with self.assertRaises(scale.ScaleContractError):
                prepare.prepare(self.root / 'source.json', self.root / 'policy.json',
                                self.root / 'output', maximum)
        output = self.root / 'existing'
        output.mkdir()
        with self.assertRaises(FileExistsError):
            prepare.prepare(self.root / 'source.json', self.root / 'policy.json', output)

    def test_non5k_source_inventory_rejected(self):
        with patch.object(prepare, 'read', return_value=dict(count=1, scenes=[])):
            with self.assertRaises(scale.ScaleContractError):
                prepare.inventory(self.root / 'manifest.json')

    def test_budget_overflow_is_explicit_no_profile_or_actor_fallback(self):
        actor = dict(name='CargoBox', data_id=40100001, source_list='objectList',
                     estimated_size=[.8, .8, .5], motion='static', unique_id=8500)
        row = dict(seed=1, actors=1, population_mode='generic_regular', population_budget_bounds=[2, 2])
        before = dict(spec=row, runtime_actors=[actor], split='train')
        metadata = {'one': before}
        deck = dict(scenes=[dict(id='one')])
        policy = copy.deepcopy(self.policy)
        policy['assets']['CargoBox']['profiles']['iso125']['native_geometry'] = dict(bbox_size_m=[4., 4., 2.])
        mapping = {scale.actor_key(actor, row, 0): 'iso125'}
        snapshot = copy.deepcopy((deck, metadata, policy, mapping))
        with self.assertRaisesRegex(scale.ScaleContractError, 'exceeds existing size budget'):
            prepare.validate_allocation(deck, metadata, policy, mapping, {40100001: actor})
        self.assertEqual((deck, metadata, policy, mapping), snapshot)

    def test_scale_transition_rejects_actor_order_class_or_split_changes(self):
        actor = dict(name='CargoBox', data_id=40100001, source_list='objectList', motion='static',
                     unique_id=8500, population_pool='obstacle')
        source = dict(spec=dict(seed=1), split='train', runtime_actors=[actor])
        for changed in (dict(source, split='val'), dict(source, runtime_actors=[dict(actor, data_id=40100002)])):
            with self.assertRaises(scale.ScaleContractError):
                prepare.verify_transition(source, changed, {}, source['spec'], {}, {})

    def qualified_policy(self, mild=[2., 1., .7], large=[4., 4., 1.]):
        policy = copy.deepcopy(self.policy)
        evidence = self.root / 'synthetic_evidence.json'
        evidence.write_text('synthetic unit fixture, not a real qualification')
        qualification = dict(observations=2, semantic_visual_review='accepted', contact_review='passed',
            sensor_gt_review='passed', axis_mapping_verified=True, single_component_obstacle_verified=True,
            evidence=[dict(path=str(evidence), sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())])
        for ident, size in (('iso125', mild), ('iso150', large)):
            policy['assets']['CargoBox']['profiles'][ident].update(status='qualified',
                native_qualification=copy.deepcopy(qualification), native_geometry=dict(
                    bbox_size_m=list(size), pivot_to_bbox_center_local_m=[0., 0., size[2]/2],
                    pivot_height_above_ground_m=0.))
        return policy

    def scene(self, count=1, bounds=[2, 3]):
        asset = dict(name='CargoBox', data_id=40100001, source_list='objectList', semantic_class='obstacle')
        actors = [dict(asset, estimated_size=[.8, .8, .5], motion='static', unique_id=8500 + i)
                  for i in range(count)]
        row = dict(seed=5, actors=count, population_mode='generic_regular',
                   population_budget_bounds=list(bounds), family='obstacle', mission=False, role='front')
        metadata = {'scene': dict(spec=row, runtime_actors=actors, split='train')}
        mapping = {scale.actor_key(asset, row, i): 'iso150' for i in range(count)}
        return dict(scenes=[dict(id='scene')]), metadata, mapping, {40100001: asset}

    def test_pre_freeze_large_profile_can_choose_approved_same_asset_mild_profile(self):
        policy = self.qualified_policy()
        deck, metadata, desired, catalog = self.scene()
        snapshot = copy.deepcopy((deck, metadata, desired))
        actual, report = prepare.feasible_allocation(deck, metadata, policy, desired, catalog)
        self.assertEqual(set(actual.values()), {'iso125'})
        self.assertEqual(report['desired_variants'], 1)
        self.assertEqual(report['actual_variants'], 1)
        self.assertEqual(report['qualified_alternative_before_freeze'], 1)
        self.assertEqual(report['decisions'][0]['decision'], 'same_asset_qualified_alternative')
        self.assertTrue(report['decisions'][0]['attempts'][0]['reasons'])
        self.assertFalse(report['frozen_retry_profile_fallback'])
        self.assertEqual((deck, metadata, desired), snapshot)

    def test_pre_freeze_infeasible_profiles_keep_original_and_report_reduced_fraction(self):
        policy = self.qualified_policy()
        deck, metadata, desired, catalog = self.scene(bounds=[2, 2])
        actual, report = prepare.feasible_allocation(deck, metadata, policy, desired, catalog)
        self.assertEqual(set(actual.values()), {'original'})
        self.assertEqual(report['desired_variants'], 1)
        self.assertEqual(report['actual_variants'], 0)
        self.assertEqual(report['original_retained_before_freeze'], 1)
        self.assertEqual(report['groups'][0]['actual_fraction'], 0.)

    def test_joint_scene_budget_order_is_deterministic_and_original_slots_stay_protected(self):
        policy = self.qualified_policy()
        deck, metadata, desired, catalog = self.scene(count=2, bounds=[4, 5])
        first, report = prepare.feasible_allocation(deck, metadata, policy, desired, catalog)
        second, second_report = prepare.feasible_allocation(deck, metadata, policy,
                                                          dict(reversed(list(desired.items()))), catalog)
        self.assertEqual((first, report), (second, second_report))
        self.assertEqual(sorted(first.values()), ['iso125', 'original'])
        self.assertEqual({d['final_scene_cost'] for d in report['decisions']}, {5})
        key = next(iter(desired))
        desired[key] = 'original'
        actual, _ = prepare.feasible_allocation(deck, metadata, policy, desired, catalog)
        self.assertEqual(actual[key], 'original')

    def test_existing_front_size_tier_is_not_relaxed_for_new_scale(self):
        policy = self.qualified_policy(mild=[4., .5, .7])
        deck, metadata, desired, catalog = self.scene(bounds=[2, 8])
        with patch.object(prepare.front, 'validate_plan', return_value=dict(ordinal=0, distance_bounds_m=[6., 10.])):
            actual, report = prepare.feasible_allocation(deck, metadata, policy, desired, catalog)
        self.assertEqual(set(actual.values()), {'original'})
        self.assertTrue(all('scaled_actor_changes_frozen_front_size_tier' in a['reasons']
                            for a in report['decisions'][0]['attempts']))

    def test_already_invalid_source_budget_is_fatal_not_repaired_by_profile_choice(self):
        policy = self.qualified_policy()
        deck, metadata, desired, catalog = self.scene(bounds=[5, 8])
        with self.assertRaisesRegex(scale.ScaleContractError, 'Source population already violates'):
            prepare.feasible_allocation(deck, metadata, policy, desired, catalog)


if __name__ == '__main__':
    unittest.main()
