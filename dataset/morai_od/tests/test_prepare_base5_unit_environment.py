import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_unit_environment as preparation
from base5_capture_safety import atomic_json, sha256


def metadata(x=0., ident='scene', split='train', primary=1, yaw=0.):
    return dict(split=split, scene_group_id='clip:'+ident,
        ego=dict(x=x, y=0., z=0., yaw=yaw),
        spec=dict(family='obstacle', mission=False, role='front',
                  motion='static', distance_bounds=[8, 14], seed=1),
        runtime_actors=[dict(name='Box', data_id=primary, unique_id=8500,
            source_list='objectList', position=dict(x=x+10, y=0., z=0.),
            placement_yaw_deg=0., estimated_size=[1., 1., 1.], motion='static',
            planned_center_distance_m=10.)])


def record(ident, x=0., split='train', primary=1, yaw=0.):
    return dict(id=ident, entry=dict(derived_training_split=split),
        meta=metadata(x, ident, split, primary, yaw), scenario_sha256='payload:'+ident)


class UnitEnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def manifests(self):
        source = self.root/'source'/'manifest.json'
        baseline = self.root/'baseline'/'manifest.json'
        range_ref = self.root/'range.json'
        atomic_json(range_ref, dict(example=True))
        deck = dict(count=5000, scenes=[{}]*5000,
            config=dict(obstacle_support_policy=dict(variant='richfloor')))
        atomic_json(source, deck)
        reference = dict(count=5000, scenes=[{}]*5000, split_overlay=dict(path='unused'),
            config=dict(environment_tag='sunny13', native_unit_geometry_reference=dict(path='unused'),
                primary_obstacle_range_policy=dict(reference=dict(path=str(range_ref), sha256=sha256(range_ref))),
                static_mission_extent=dict(counts=dict(before=300, intersection=120, after=180))))
        atomic_json(baseline, reference)
        return source, baseline

    def plan(self, source, baseline, output, environment='sunny15', seed=261017):
        with patch.object(preparation.unit, 'load', return_value=dict(sources=[dict(path='ref', sha256='pin', assets=['Box'])])):
            return preparation.plan(source, baseline, output, environment, seed)

    def test_plan_is_read_only_and_does_not_grant_native_or_bulk(self):
        source, baseline = self.manifests()
        before = {str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        output = self.root/'new_environment'
        result = self.plan(source, baseline, output)
        self.assertFalse(output.exists())
        self.assertFalse(result['native_qualified'])
        self.assertFalse(result['bulk_ready'])
        self.assertFalse(result['simulator_access'])
        self.assertEqual(result['static_extent_counts'], dict(before=300, intersection=120, after=180))
        self.assertEqual(before, {str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()})

    def test_plan_rejects_sunny13_or_malformed_seed(self):
        source, baseline = self.manifests()
        for env, seed in (('sunny13', 1), ('rainy15', 1), ('sunny15', True), ('sunny15', -1)):
            with self.subTest(environment=env, seed=seed):
                with self.assertRaises(ValueError):
                    self.plan(source, baseline, self.root/'new', env, seed)

    def test_plan_rejects_frozen_policy_source_and_existing_output(self):
        source, baseline = self.manifests()
        for key in preparation.FORBIDDEN_SOURCE_POLICIES:
            deck = preparation.read(source)
            deck['config'][key] = {}
            atomic_json(source, deck)
            with self.subTest(policy=key), self.assertRaisesRegex(ValueError, 'pre-front'):
                self.plan(source, baseline, self.root/'new')
            deck['config'].pop(key)
            atomic_json(source, deck)
        for output in (source.parent/'nested', baseline.parent/'nested', self.root):
            with self.assertRaises(ValueError):
                self.plan(source, baseline, output)

    def test_plan_refuses_symlink_and_changed_range_reference(self):
        source, baseline = self.manifests()
        target = self.root/'target'
        target.mkdir()
        alias = self.root/'alias'
        alias.symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.plan(source, baseline, alias)
        atomic_json(self.root/'range.json', dict(changed=True))
        with self.assertRaisesRegex(ValueError, 'range reference'):
            self.plan(source, baseline, self.root/'new')

    def test_independent_geometry_with_same_logical_quota_is_allowed(self):
        left = record('sunny13_slot', 0.)
        right = record('sunny15_slot', 1., split='val')
        for row in (left, right):
            row['meta']['logical_quota_slot_id'] = 'same_quota'
        result = preparation.cross_environment_audit([left], [right])
        self.assertTrue(result['passed'])
        self.assertEqual(result['combined_layout_groups'], 2)

    def test_exact_geometry_cannot_hide_behind_different_ids_or_instance_uids(self):
        left, right = record('sunny13_slot'), record('sunny15_slot', split='val')
        right['meta']['runtime_actors'][0]['unique_id'] = 9999
        result = preparation.cross_environment_audit([left], [right])
        self.assertFalse(result['passed'])
        self.assertEqual(len(result['cross_environment_duplicate_groups']), 1)
        self.assertEqual(len(result['cross_split_duplicate_groups']), 1)
        self.assertFalse(result['baseline_modified'])

    def test_near_duplicate_is_rejected_even_in_same_split(self):
        result = preparation.cross_environment_audit([record('old')], [record('new', .02, yaw=.1)])
        self.assertFalse(result['passed'])
        self.assertEqual(len(result['near_duplicate_pairs']), 1)
        self.assertEqual(result['cross_split_duplicate_groups'], [])

    def test_same_background_is_allowed_when_actor_identity_differs(self):
        result = preparation.cross_environment_audit([record('old')], [record('new', primary=2)])
        self.assertTrue(result['passed'])

    def test_quota_ignores_auxiliary_count_seed_and_split_but_not_primary_or_phase(self):
        left, right = metadata(), metadata()
        right['split'] = 'val'
        right['spec']['seed'] = 42
        right['runtime_actors'].append(copy.deepcopy(right['runtime_actors'][0]))
        self.assertEqual(preparation.quota_key(left), preparation.quota_key(right))
        right['runtime_actors'][0]['data_id'] = 9
        self.assertNotEqual(preparation.quota_key(left), preparation.quota_key(right))
        right = copy.deepcopy(left)
        right['spec']['avoidance_phase'] = 'alongside'
        self.assertNotEqual(preparation.quota_key(left), preparation.quota_key(right))
        right = copy.deepcopy(left)
        right['spec']['distance_bounds'] = [20, 30]
        self.assertEqual(preparation.quota_key(left), preparation.quota_key(right))
        self.assertNotEqual(preparation.quota_key(left, True), preparation.quota_key(right, True))

    def test_distribution_preserves_unprepared_moving_shadow_and_refuses_old_reuse(self):
        row = record('fresh_shadow')
        row['meta']['spec']['family'] = 'shadow'
        row['meta']['runtime_actors'][0]['motion'] = 'walking'
        result = preparation.distribution([row], {'Box'})
        self.assertEqual(result['fresh_shadow_on_demand_preparation_required'], ['fresh_shadow'])
        self.assertFalse(result['old_exact_shadow_reuse_adopted'])
        row['meta']['shadow_motion_ground_reuse'] = dict(old=True)
        with self.assertRaisesRegex(ValueError, 'cannot inherit'):
            preparation.distribution([row], {'Box'})

    def test_normalize_only_new_manifest_keeps_environment_and_no_sunny13_claim(self):
        target = self.root/'new'/'manifest.json'
        atomic_json(target, dict(config=dict(environment_tag='sunny15',
            environment_suite=dict(split_policy='old repeated scenes'), new_sunny13_diagnostic_plan={},
            fresh_geometry_policy={})))
        preparation.normalize_environment(target, 'sunny15')
        result = preparation.read(target)
        self.assertNotIn('new_sunny13_diagnostic_plan', result['config'])
        self.assertIn('Independent', result['config']['environment_suite']['split_policy'])
        self.assertFalse(result['bulk_ready'])
        self.assertFalse(result['native_qualified'])

    def test_records_detect_changed_payload_and_duplicate_scene_ids(self):
        path = self.root/'manifest.json'
        scenario, meta = self.root/'scene.json', self.root/'metadata.json'
        atomic_json(scenario, {'objectList':[]})
        atomic_json(meta, metadata())
        entry = dict(id='one', scenario=str(scenario), metadata=str(meta), scenario_sha256=sha256(scenario))
        atomic_json(path, dict(scenes=[entry, entry]))
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            preparation.records(path)
        atomic_json(path, dict(scenes=[entry]))
        atomic_json(scenario, dict(changed=True))
        with self.assertRaisesRegex(ValueError, 'checksum'):
            preparation.records(path)

    def close_source(self):
        source = self.root/'close_source'/'manifest.json'
        scenario, meta = source.parent/'scene.json', source.parent/'metadata.json'
        atomic_json(scenario, dict(objectList=[]))
        value = metadata()
        value['retry'] = 0
        atomic_json(meta, value)
        entry = dict(id='only_one', scenario=str(scenario), metadata=str(meta),
                     scenario_sha256=sha256(scenario), split='train')
        config = dict(native_unit_geometry_reference=dict(path='reference', sha256='pin'))
        atomic_json(source, dict(config=config, count=1, scenes=[entry]))
        return source, value, config

    def test_close_accepts_empty_eligibility_without_context_or_hardcoded_23(self):
        source, meta, config = self.close_source()
        with patch.object(preparation.close_factory, 'inventory', return_value=(dict(records=[]), config)), \
                patch.object(preparation, 'normalized_sensor_calibration', return_value={}), \
                patch.object(preparation, 'prewarm') as prewarm:
            target = preparation.close_candidate(source, self.root/'close_output')
        self.assertEqual(preparation.read(target)['count'], 1)
        prewarm.assert_not_called()
        audit = preparation.read(target.parent/'close_primary_audit.json')
        self.assertEqual(audit['expected_eligible_scenes'], 0)
        self.assertEqual(audit['changed_scenes'], 0)
        self.assertFalse(audit['native_qualified'])

    def test_close_one_eligible_is_bounded_and_does_not_edit_source(self):
        source, meta, config = self.close_source()
        before = {str(p):p.read_bytes() for p in source.parent.iterdir() if p.is_file()}
        row = dict(meta['spec'], close_primary_observation=dict(example=True))
        proposed = copy.deepcopy(meta)
        proposed['spec'] = row
        inventory = dict(records=[dict(id='only_one')])
        with patch.object(preparation.close_factory, 'inventory', return_value=(inventory, config)), \
                patch.object(preparation, 'normalized_sensor_calibration', return_value={}), \
                patch.object(preparation, 'prewarm', return_value={}), \
                patch.object(preparation.spatial.gen, 'context', return_value={}), \
                patch.object(preparation.close, 'tagged_row', return_value=row), \
                patch.object(preparation.spatial, 'build', return_value=(dict(objectList=[]), proposed)), \
                patch.object(preparation.close, 'projection_proposal', return_value=dict(eligible=True)), \
                patch.object(preparation.unit, 'load', return_value={}), \
                patch.object(preparation.unit, 'validate_actor_identities'), \
                patch.object(preparation.close, 'validate_entry'):
            target = preparation.close_candidate(source, self.root/'close_output')
        audit = preparation.read(target.parent/'close_primary_audit.json')
        self.assertEqual(audit['changed_scenes'], 1)
        self.assertEqual(audit['full_band'], [1., 1.5])
        self.assertEqual(before, {str(p):p.read_bytes() for p in source.parent.iterdir() if p.is_file()})

    def test_close_failure_never_relaxes_or_loops_indefinitely(self):
        source, meta, config = self.close_source()
        output = self.root/'close_output'
        inventory = dict(records=[dict(id='only_one')])
        with patch.object(preparation.close_factory, 'inventory', return_value=(inventory, config)), \
                patch.object(preparation, 'normalized_sensor_calibration', return_value={}), \
                patch.object(preparation, 'prewarm', return_value={}), \
                patch.object(preparation.spatial.gen, 'context', return_value={}), \
                patch.object(preparation.close, 'tagged_row', return_value=meta['spec']), \
                patch.object(preparation.spatial, 'build', side_effect=ValueError('no legal pose')) as build:
            with self.assertRaisesRegex(ValueError, 'Bounded'):
                preparation.close_candidate(source, output)
        self.assertEqual(build.call_count, 512)
        self.assertEqual(preparation.read(output/'failure.json')['maximum_proposals'], 512)
        self.assertFalse((output/'manifest.json').exists())

    def test_frozen_secondary_lane_requirement_is_not_replaced_by_primary_only(self):
        ctx = dict(pools=dict(vehicle=[dict(data_id=1, source_list='vehicleList'),
                                      dict(data_id=4, source_list='vehicleList')],
                              obstacle=[dict(data_id=2, source_list='objectList')],
                              person=[dict(data_id=3, source_list='pedestrianList')]))
        spec = dict(family='other_vehicle', role='front', population_layout='distributed',
                    frozen_population_data_ids=[1, 2, 3, 4, 2])
        self.assertEqual(preparation.required_adjacent_sides(spec, ctx), ('right',))
        self.assertEqual(preparation.required_adjacent_sides(dict(spec, family='highway'), ctx), ())
        self.assertEqual(preparation.required_adjacent_sides(dict(spec, role='rear'), ctx), ('left',))

    def test_adjacent_screen_preserves_only_real_allowed_destination_intervals(self):
        ctx = dict(route=[], arc=[], links=dict(
            invalid=dict(can_move_right_lane=False, right_lane_change_dst_link_idx='parallel'),
            valid=dict(can_move_right_lane=True, right_lane_change_dst_link_idx='parallel'),
            missing=dict(can_move_right_lane=True, right_lane_change_dst_link_idx='absent'), parallel={}))
        def pose(route, arc, distance):
            return dict(link_idx='invalid' if distance < 2 else 'valid' if distance < 4 else 'missing')
        with patch.object(preparation.spatial.gen.geo, 'continuous_route_pose_at_arc', side_effect=pose):
            self.assertEqual(preparation.screen_adjacent_intervals(ctx, [[0., 6.]], ('right',)), [[2., 4.]])
            with self.assertRaisesRegex(ValueError, 'No primary-and-secondary'):
                preparation.screen_adjacent_intervals(ctx, [[0., 2.]], ('right',))
        self.assertEqual(preparation.screen_adjacent_intervals(ctx, [[0., 6.]], ()), [[0., 6.]])

    def test_adjacent_adapter_partitions_rosters_and_restores_original_functions(self):
        spatial = preparation.spatial
        original_assign, original_feasible = spatial.assign_plans, spatial.feasible_intervals
        items = [('one', dict(kind='one'), 'train'), ('two', dict(kind='two'), 'val')]
        def assign(values, intervals, ctx):
            for _, spec, _ in values:
                spec['spatial_sampling_plan'] = dict(example=True)
        with patch.object(preparation, 'required_adjacent_sides', side_effect=lambda spec, ctx:
                ('right',) if spec['kind'] == 'one' else ()), \
                patch.object(spatial, 'assign_plans', side_effect=assign) as calls:
            outer = spatial.assign_plans
            with self.assertRaisesRegex(ValueError, 'intentional'):
                with preparation.adjacent_feasibility_adapter():
                    spatial.assign_plans(items, {}, {})
                    raise ValueError('intentional')
            self.assertIs(spatial.assign_plans, outer)
            self.assertEqual(calls.call_count, 2)
        self.assertIs(spatial.assign_plans, original_assign)
        self.assertIs(spatial.feasible_intervals, original_feasible)
        self.assertEqual(items[0][1]['spatial_sampling_plan']['ordinary_adjacent_lane_screening']['required_sides'], ['right'])
        self.assertNotIn('ordinary_adjacent_lane_screening', items[1][1]['spatial_sampling_plan'])

    def test_capture_split_is_supported_spec_override_not_metadata_relabeling(self):
        value = metadata()
        config = {preparation.layout.CONFIG_KEY:preparation.layout.configuration()}
        original = copy.deepcopy(value)
        spec = preparation.capture_split_spec(value, config)
        self.assertEqual(value, original)
        self.assertEqual(spec, dict(value['spec'], layout_split='train'))
        probe = metadata(1000., 'different_geometry', split='train')
        probe['spec'] = spec
        self.assertEqual(preparation.layout.provisional_split(probe, config), 'train')
        with self.assertRaisesRegex(ValueError, 'already frozen'):
            preparation.capture_split_spec(dict(value, spec=spec), config)
        with self.assertRaisesRegex(ValueError, 'Known-map'):
            preparation.capture_split_spec(value, {})
        with self.assertRaisesRegex(ValueError, 'Explicit'):
            preparation.capture_split_spec(dict(value, split='test'), config)

    def test_freeze_full_source_preserves_payload_and_original_metadata_bytes(self):
        source, meta, config = self.close_source()
        config[preparation.layout.CONFIG_KEY] = preparation.layout.configuration()
        original = preparation.read(source)
        original['config'] = config
        original['scenes'] = [dict(original['scenes'][0], id='slot_'+str(i)) for i in range(5000)]
        original['count'] = 5000
        atomic_json(source, original)
        before = {str(p):p.read_bytes() for p in source.parent.iterdir() if p.is_file()}
        output = self.root/'frozen_capture_split'
        target = preparation.freeze_capture_splits(source, output)
        candidate = preparation.read(target)
        frozen = preparation.read(candidate['config']['frozen_specs_path'])
        self.assertEqual(len(candidate['scenes']), 5000)
        self.assertEqual(sha256(candidate['config']['frozen_specs_path']), candidate['config']['frozen_specs_sha256'])
        for index in (0, 4999):
            entry = candidate['scenes'][index]
            metadata = preparation.read(entry['metadata'])
            self.assertEqual(metadata['spec'], frozen[index])
            self.assertEqual(metadata['spec']['layout_split'], metadata['split'])
            self.assertEqual(entry['scenario'], original['scenes'][index]['scenario'])
            self.assertTrue(metadata['provisional_capture_split_contract']['training_split_allocation_deferred'])
        self.assertEqual(before, {str(p):p.read_bytes() for p in source.parent.iterdir() if p.is_file()})

    def test_numeric_error_summary_is_not_only_a_maximum(self):
        result = preparation.numeric_summary([1, 2, 3, 4])
        self.assertEqual(result['mean'], 2.5)
        self.assertEqual(result['p50'], 2.5)
        self.assertAlmostEqual(result['p90'], 3.7)
        self.assertAlmostEqual(result['p95'], 3.85)
        self.assertEqual(result['maximum'], 4.)
        self.assertEqual(preparation.numeric_summary([]), dict(count=0))

    def test_guarded_spread_reserves_existing_source_as_eighth_and_restores_adapter(self):
        source, target = self.root/'source.json', self.root/'spread'/'manifest.json'
        atomic_json(source, dict(example=True))
        plan = dict(logical_slot_id='slot', target_ordinal=1)
        meta = dict(spec=dict(secondary_far_vehicle=plan), runtime_actors=[{}, dict(planned_center_distance_m=11.)],
                    secondary_far_vehicle_generation=dict(attempt=99))
        old = dict(id='slot', meta=meta, entry={})
        new = copy.deepcopy(old)
        new['meta']['secondary_far_vehicle_generation']['initial_distance_spread'] = dict(
            retry=11, actual_distance_ego_xy_m=11., target_error_m=11., rejected=[])
        candidates = [(None, None, None, None, float(i)) for i in range(12)]
        def prepare(source, output):
            ranked = preparation.spread.proposal_order(plan, candidates, 0.)
            self.assertEqual(ranked[:8], [(i, i) for i in range(7)]+[(11, 11)])
            self.assertEqual(len(set(ranked)), 12)
            atomic_json(target, dict(example=True))
            return target
        original = preparation.spread.proposal_order
        with patch.object(preparation, 'records', side_effect=[({}, [old]), ({}, [new])]), \
                patch.object(preparation.spread, 'proposal_order', return_value=[(i, i) for i in range(12)]) as order, \
                patch.object(preparation.far.far, '_draw_index', return_value=11), \
                patch.object(preparation.spread, 'prepare', side_effect=prepare):
            preparation.guarded_spread(source, target.parent)
            self.assertIs(preparation.spread.proposal_order, order)
        self.assertIs(preparation.spread.proposal_order, original)
        audit = preparation.read(target.parent/'guarded_source_fallback_audit.json')
        self.assertEqual(audit['maximum_guarded_candidates_per_slot'], 8)
        self.assertEqual(audit['fallback_selected_count'], 1)
        self.assertFalse(audit['native_qa_relaxed'])
        self.assertFalse(audit['exact_uniform_distance_distribution_claimed'])

    def test_continuation_requires_an_explicit_paired_manifest_checksum(self):
        source, baseline = self.manifests()
        continuation = self.root/'far'/'manifest.json'
        atomic_json(continuation, dict(example=True))
        with patch.object(preparation.unit, 'load', return_value=dict(sources=[])):
            with self.assertRaisesRegex(ValueError, 'together'):
                preparation.plan(source, baseline, self.root/'new', 'sunny15',
                                 continue_far=continuation)
            with self.assertRaisesRegex(ValueError, 'checksum'):
                preparation.plan(source, baseline, self.root/'new', 'sunny15',
                                 continue_far=continuation, continue_far_sha='0'*64)
            request = preparation.plan(source, baseline, self.root/'new', 'sunny15',
                continue_far=continuation, continue_far_sha=sha256(continuation))
        self.assertFalse(request['continue_from_unqualified_far_stage']['native_qualified'])
        self.assertFalse((self.root/'new').exists())


if __name__ == '__main__':
    unittest.main()
