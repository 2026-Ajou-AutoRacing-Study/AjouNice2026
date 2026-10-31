import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_primary_obstacle_range as policy
from test_base5_front_completion_adapter import selected as front_selected


def source_meta():
    return dict(spec=dict(family='obstacle', mission=False, role='front', actors=2,
        seed=11, distance_bounds=[60, 100], distance_bin='far', population_mode='generic_regular',
        population_max_actors=8, static_yaw_policy=dict(primary_mode='uniform', primary_sector=4),
        spatial_sampling_plan=dict(split='train', intervals_m=[[1, 10]])),
        runtime_actors=[dict(data_id=10, unique_id=8500, name='Thin', source_list='objectList',
            population_pool='obstacle_barrier', motion='static', planned_center_distance_m=80.,
            actual_ego_role='front'), dict(data_id=20, unique_id=8501, name='Box', source_list='objectList',
            population_pool='obstacle_box', motion='static', planned_center_distance_m=12.,
            actual_ego_role='left')], split='train', retry=2, scene_group_id='fresh:slot')


class RangeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.reference = Path(self.directory.name) / 'sizes.json'
        self.record = dict(name='Thin', size_q50_m=[2., .2, 1.], observations=4)
        self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA, assets={'10': self.record})))
        self.config = {policy.CONFIG_KEY: policy.configuration(self.reference)}
        self.asset = dict(data_id=10, source_list='objectList', semantic_class='obstacle')

    def planned(self, before=None, asset=None):
        before = before or source_meta()
        row, decision = policy.attach(before['spec'], before['runtime_actors'], asset or self.asset,
            self.config[policy.CONFIG_KEY], 'slot', before['split'])
        actual = copy.deepcopy(before)
        actual['spec'] = row
        actual['runtime_actors'][0]['planned_center_distance_m'] = sum(row['distance_bounds']) / 2
        actual['runtime_actors'][0]['actual_ego_role'] = row['role']
        return actual, decision

    def choose(self, source, ordinal=1, known=None, build=None, novelty=None):
        def generated(row, config, ctx, retry):
            self.assertEqual(row, source['spec'])
            actual = copy.deepcopy(source)
            actual['retry'] = retry
            return dict(retry=retry), actual
        return policy.choose(source, self.config, {}, {}, ordinal, 'slot', known or set(),
            build or generated, lambda m, cal, kind: dict(score=m['retry']),
            novelty or (lambda m, history: 1.))

    def test_classifier_strict_edges_and_compact_extension(self):
        for size, cap in (([2, 1, .249], 20), ([2, 1, .25], 30), ([2, 1, .349], 30),
                          ([2, 1, .35], 40), ([1, 1, .35], 30), ([1.5, 1, .35], 40),
                          ([1, 1, .399], 30), ([1, 1, .4], 40), ([1.92, .393, 1], 40),
                          ([1, 1, .599], 40), ([1, 1, .6], None)):
            with self.subTest(size=size): self.assertEqual(policy.classify(size), cap)
        for size in ([0, 1, 1], [True, 1, 1], [float('nan'), 1, 1], [1, 1]):
            with self.assertRaises(policy.RangeContractError): policy.classify(size)

    def test_declared_bounds_preserve_lower_or_move_last_ten_metres(self):
        for original, cap, expected in (([60, 100], 20, [10, 20]), ([20, 30], 20, [10, 20]),
            ([10, 30], 20, [10, 20]), ([0, 100], 40, [0, 40]), ([3, 10], 20, [3, 10]),
            ([10, 20], 20, [10, 20]), ([30, 50], None, [30, 50])):
            self.assertEqual(policy.bounds(original, cap), expected)
        for original in ([10, 10], [-1, 20], [0, float('inf')]):
            with self.assertRaises(policy.RangeContractError): policy.bounds(original, 20)

    def test_attach_only_changes_declared_fields_and_preserves_all_actor_ids(self):
        before = source_meta()
        snapshot = copy.deepcopy(before)
        actual, decision = self.planned(before)
        self.assertTrue(decision['changed'])
        self.assertEqual(before, snapshot)
        expected = copy.deepcopy(before['spec'])
        expected.update(distance_bounds=[10, 20], distance_bin='primary_native_cap_20m_v152',
                        frozen_population_data_ids=[10, 20])
        expected[policy.PLAN_KEY] = actual['spec'][policy.PLAN_KEY]
        self.assertEqual(actual['spec'], expected)
        self.assertEqual(policy.verify_preserved(actual, actual), actual['spec'][policy.PLAN_KEY])

    def test_already_capped_uncapped_and_missing_reference_are_exact_legacy_specs(self):
        for kind in ('near', 'uncapped', 'missing'):
            before = source_meta()
            if kind == 'near': before['spec']['distance_bounds'] = [3, 10]
            elif kind == 'uncapped':
                self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA,
                    assets={'10': dict(self.record, size_q50_m=[1., 1., 1.])})))
                self.config[policy.CONFIG_KEY] = policy.configuration(self.reference)
            else:
                self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA, assets={})))
                self.config[policy.CONFIG_KEY] = policy.configuration(self.reference)
            row, decision = policy.attach(before['spec'], before['runtime_actors'], self.asset,
                self.config[policy.CONFIG_KEY], 'slot', 'train')
            self.assertFalse(decision['changed'])
            self.assertEqual(row, before['spec'])
            self.assertNotIn(policy.PLAN_KEY, row)

    def test_mission_shadow_avoidance_vehicle_pedestrian_moving_are_excluded(self):
        for kind in ('mission', 'shadow_phase', 'avoidance_phase', 'family', 'semantic', 'container', 'moving'):
            before, asset = source_meta(), copy.deepcopy(self.asset)
            if kind == 'mission': before['spec']['mission'] = True
            elif kind in ('shadow_phase', 'avoidance_phase'): before['spec'][kind] = 'approach'
            elif kind == 'family': before['spec']['family'] = 'highway'
            elif kind == 'semantic': asset['semantic_class'] = 'vehicle'
            elif kind == 'container': before['runtime_actors'][0]['source_list'] = 'pedestrianList'
            else: before['runtime_actors'][0]['motion'] = 'walking'
            row, decision = policy.attach(before['spec'], before['runtime_actors'], asset,
                self.config[policy.CONFIG_KEY], 'slot', 'train')
            self.assertEqual(row, before['spec'])
            self.assertFalse(decision['changed'])

    def test_plan_requires_explicit_matching_config(self):
        actual, _ = self.planned()
        self.assertIsNotNone(policy.validate_plan(actual['spec'], self.config))
        for config in (None, {}, {policy.CONFIG_KEY: {}},
                       {policy.CONFIG_KEY: dict(self.config[policy.CONFIG_KEY], source_sha256='bad')}):
            with self.assertRaises(policy.RangeContractError): policy.validate_plan(actual['spec'], config)
        self.assertIsNone(policy.validate_plan(source_meta()['spec']))

    def test_reference_helper_seed_policy_and_spec_mutations_are_fatal(self):
        original, _ = self.planned()
        for key in ('seed', 'role', 'spatial_sampling_plan', 'static_yaw_policy', 'bounds', 'cap', 'identity', 'front'):
            actual = copy.deepcopy(original)
            row, plan = actual['spec'], actual['spec'][policy.PLAN_KEY]
            if key in ('seed', 'role', 'spatial_sampling_plan', 'static_yaw_policy'): row[key] = 'changed'
            elif key == 'bounds': row['distance_bounds'] = [0, 20]
            elif key == 'cap': plan['cap_m'] = 40
            elif key == 'identity': plan['actor_identity'][0]['data_id'] = 99
            else: row['secondary_obstacle_front_support'] = {'changed': True}
            with self.subTest(key=key), self.assertRaises(policy.RangeContractError):
                policy.validate_plan(row, self.config)
        with patch.object(policy, 'implementation_sha256', return_value='changed'):
            with self.assertRaises(policy.RangeContractError): policy.validate_plan(original['spec'], self.config)
        self.reference.write_text('{}')
        with self.assertRaises(policy.RangeContractError): policy.validate_plan(original['spec'], self.config)

    def test_native_reference_requires_repeated_unmixed_observations(self):
        for record in (dict(self.record, observations=1), dict(self.record, scale1_observations_verified=3)):
            self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA, assets={'10': record})))
            self.config[policy.CONFIG_KEY] = policy.configuration(self.reference)
            with self.assertRaises(policy.RangeContractError): self.planned()

    def test_runtime_identity_motion_split_and_actual_range_must_remain(self):
        source, _ = self.planned()
        for key in ('data_id', 'unique_id', 'name', 'source_list', 'population_pool', 'motion', 'split', 'range', 'role'):
            actual = copy.deepcopy(source)
            if key == 'split': actual['split'] = 'val'
            elif key == 'range': actual['runtime_actors'][0]['planned_center_distance_m'] = 20.
            elif key == 'role': actual['runtime_actors'][0]['actual_ego_role'] = 'rear'
            else: actual['runtime_actors'][0][key] = 'changed'
            with self.subTest(key=key), self.assertRaises(policy.RangeContractError):
                policy.verify_preserved(source, actual)
        with self.assertRaises(policy.RangeContractError): policy.verify_preserved(source_meta(), source)
        self.assertIsNone(policy.verify_preserved(source_meta(), source_meta()))

    def test_existing_front_plan_is_preserved_without_reassignment(self):
        before = front_selected()
        primary = before['runtime_actors'][0]
        self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA,
            assets={str(primary['data_id']): dict(self.record, name=primary['name'])})))
        self.config[policy.CONFIG_KEY] = policy.configuration(self.reference)
        actual, _ = self.planned(before, dict(data_id=primary['data_id'], source_list='objectList', semantic_class='obstacle'))
        self.assertEqual(actual['spec']['secondary_obstacle_front_support'], before['spec']['secondary_obstacle_front_support'])
        self.assertEqual(actual['spec']['seed'], before['spec']['seed'])
        self.assertIsNotNone(policy.verify_preserved(actual, actual))

    def test_dual_front_adapter_cannot_swallow_a_range_contract_failure(self):
        before = front_selected()
        primary = before['runtime_actors'][0]
        self.reference.write_text(json.dumps(dict(schema=policy.REFERENCE_SCHEMA,
            assets={str(primary['data_id']): dict(self.record, name=primary['name'])})))
        self.config[policy.CONFIG_KEY] = policy.configuration(self.reference)
        source, _ = self.planned(before, dict(data_id=primary['data_id'], source_list='objectList', semantic_class='obstacle'))
        attempts = []
        def failed(row, config, ctx, retry):
            attempts.append(retry)
            raise policy.RangeContractError('Range reference/config changed')
        with self.assertRaises(policy.RangeContractError):
            policy.front_completion.choose(source, self.config, {}, {}, 1, 'slot', set(),
                failed, lambda *args: dict(score=1.), lambda *args: 1.)
        self.assertEqual(len(attempts), 1)

    def test_choose_is_seed_fixed_bounded_novel_and_resume_deterministic(self):
        source, _ = self.planned()
        a, b = self.choose(source, 2), self.choose(source, 2)
        self.assertEqual(a, b)
        self.assertEqual(a[1]['flexible_completion']['retry_block_start'], 4099)
        self.assertEqual(a[1]['flexible_completion']['valid_candidates'], 8)
        self.assertEqual(a[1]['spec'], source['spec'])
        known = hashlib.sha256((json.dumps(dict(retry=3), indent=2) + '\n').encode()).hexdigest()
        result = self.choose(source, known={known}, novelty=lambda m, h: .1 if m['retry'] == 4 else 1.)
        self.assertEqual(result[0]['retry'], 12)

    def test_geometry_exhaustion_is_finite_but_contract_failure_is_not_swallowed(self):
        source, _ = self.planned()
        attempts = []
        def failed(row, config, ctx, retry):
            attempts.append(retry)
            raise ValueError('No safe geometry')
        with self.assertRaises(policy.RangeGeometryExhausted): self.choose(source, build=failed)
        self.assertEqual(len(attempts), 512)
        attempts.clear()
        def bad(row, config, ctx, retry):
            attempts.append(retry)
            raise policy.RangeContractError('Broken reference')
        with self.assertRaises(policy.RangeContractError): self.choose(source, build=bad)
        self.assertEqual(len(attempts), 1)


if __name__ == '__main__':
    unittest.main()
