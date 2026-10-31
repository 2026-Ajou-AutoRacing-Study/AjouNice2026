import copy
import hashlib
import json
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_asset_scale as scale


class AssetScaleTests(unittest.TestCase):
    def setUp(self):
        self.policy = scale.candidate_policy([0, 2, 1])
        self.asset = dict(name='CargoBox', data_id=40100001, source_list='objectList',
                          semantic_class='obstacle', ontology_class='Obstacle', collection_role='positive')
        self.row = dict(seed=19, family='obstacle', mission=False, split='train',
                        distance_bin='10_20', role='front')
        self.actor = dict(DataID=40100001, UNIQUEID=8500,
                          scale=dict(x=9., y=9., z=9., _x='9.000', _y='9.000', _z='9.000'))
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def select(self, **kwargs):
        return scale.select(self.asset, self.row, 0, 'static', self.policy, **kwargs)

    def qualify(self, profile='iso125'):
        path = Path(self.directory.name) / 'native_evidence.json'
        path.write_text(json.dumps(dict(note='synthetic test fixture, not real qualification')))
        record = self.policy['assets']['CargoBox']['profiles'][profile]
        record.update(status='qualified', native_geometry=dict(bbox_size_m=[1.1, 1.2, .7],
                      pivot_to_bbox_center_local_m=[.01, -.02, .32], pivot_height_above_ground_m=.01),
                      native_qualification=dict(observations=2, semantic_visual_review='accepted',
                          contact_review='passed', sensor_gt_review='passed', axis_mapping_verified=True,
                          single_component_obstacle_verified=True,
                          evidence=[dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())]))
        return path

    def test_no_policy_noop_does_not_change_actor_or_geometry(self):
        self.assertIsNone(scale.select(self.asset, self.row, 0, 'static'))
        size, pivot, proof = scale.placement_geometry([1, 2, 3], [0, 0, 1], None, None)
        self.assertEqual((size, pivot, proof), ([1., 2., 3.], [0., 0., 1.], None))
        self.assertEqual(scale.apply_actor(self.actor, None, None), self.actor)
        with self.assertRaises(scale.ScaleContractError):
            scale.select(self.asset, self.row, 0, 'static', profile_id='iso125')

    def test_candidate_policy_declares_twelve_identity_pairs_and_no_default_axis(self):
        self.assertEqual(len(self.policy['assets']), 12)
        self.assertFalse(self.policy['continuous_jitter'])
        self.assertFalse(self.policy['training_gt_modified'])
        with self.assertRaises(scale.ScaleContractError):
            scale.candidate_policy(None)
        for name, record in self.policy['assets'].items():
            self.assertEqual((record['data_id'], record['group']), scale.WHITELIST[name])
            self.assertTrue(all(p['status'] == 'candidate' for p in record['profiles'].values()))

    def test_candidates_are_not_collection_defaults(self):
        for seed in range(200):
            assignment = scale.select(self.asset, dict(self.row, seed=seed), 0, 'static', self.policy)
            self.assertEqual(assignment['profile_id'], 'original')
            self.assertEqual(assignment['scale_xyz'], [1., 1., 1.])
            self.assertFalse(assignment['candidate_only'])
        with self.assertRaises(scale.ScaleContractError):
            self.select(profile_id='iso125')

    def test_probe_flag_required_and_explicit_candidate_survives_retry(self):
        assignment = self.select(profile_id='sample_2_3_2', allow_candidate=True)
        self.assertTrue(assignment['candidate_only'])
        self.assertFalse(assignment['retry_resamples_scale'])
        self.assertEqual(assignment, scale.select(self.asset, dict(self.row, retry=99), 0,
                         'static', self.policy, profile_id='sample_2_3_2', allow_candidate=True))
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_assignment(assignment, self.policy)

    def test_selection_is_independent_of_global_random_and_actor_order(self):
        expected = self.select(allow_candidate=True)
        random.seed(100)
        for _ in range(100):
            random.random()
        self.assertEqual(expected, self.select(allow_candidate=True))
        other = scale.select(self.asset, self.row, 1, 'static', self.policy, allow_candidate=True)
        self.assertNotEqual(expected['actor_key'], other['actor_key'])
        self.assertNotEqual(expected['actor_key'], scale.actor_key(self.asset, dict(self.row, seed=20), 0))

    def test_probe_sampling_preserves_expected_original_majority(self):
        assignments = [scale.select(self.asset, dict(self.row, seed=i), 0, 'static', self.policy,
                                   allow_candidate=True) for i in range(2000)]
        variants = sum(a['profile_id'] != scale.BASE for a in assignments)
        self.assertTrue(500 < variants < 700, variants)
        self.assertEqual({a['profile_id'] for a in assignments},
                         {'original', 'iso125', 'iso150', 'sample_2_3_2'})

    def test_whitelist_rejects_similar_names_vehicle_obj_person_and_thin_assets(self):
        excluded = [dict(name='Bollard01', data_id=40100058, source_list='objectList'),
                    dict(name='Lane_Separation_Bar', data_id=40100026, source_list='objectList'),
                    dict(name='OBJ_CargoBox', data_id=40109999, source_list='objectList'),
                    dict(name='Man1', data_id=100, source_list='pedestrianList'),
                    dict(name='Vehicle', data_id=101, source_list='vehicleList')]
        for asset in excluded:
            with self.subTest(asset=asset):
                self.assertIsNone(scale.select(asset, self.row, 0, 'static', self.policy, allow_candidate=True))
                with self.assertRaises(scale.ScaleContractError):
                    scale.select(asset, self.row, 0, 'static', self.policy,
                                 profile_id='iso125', allow_candidate=True)

    def test_catalog_identity_semantic_and_source_must_match(self):
        for change in (dict(name='cargoBox'), dict(data_id=40100002), dict(source_list='vehicleList'),
                       dict(semantic_class='vehicle'), dict(ontology_class='Vehicle'),
                       dict(collection_role='roadside_context')):
            with self.subTest(change=change), self.assertRaises(scale.ScaleContractError):
                scale.select(dict(self.asset, **change), self.row, 0, 'static', self.policy,
                             allow_candidate=True)
        for motion in ('walking', 'running'):
            self.assertIsNone(scale.select(self.asset, self.row, 0, motion, self.policy))
            with self.assertRaises(scale.ScaleContractError):
                scale.select(self.asset, self.row, 0, motion, self.policy,
                             profile_id='iso125', allow_candidate=True)

    def test_policy_cannot_whitelist_any_training_class_member(self):
        for change in (dict(data_id=123), dict(group='vehicle'), dict(source_list='vehicleList'),
                       dict(semantic_class='vehicle')):
            policy = copy.deepcopy(self.policy)
            policy['assets']['CargoBox'].update(change)
            with self.assertRaises(scale.ScaleContractError):
                scale.validate_policy(policy)
        policy = copy.deepcopy(self.policy)
        policy['assets']['Bollard01'] = policy['assets']['CargoBox']
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_policy(policy)

    def test_invalid_policy_scale_axes_fraction_and_jitter_fail_closed(self):
        changes = [(('continuous_jitter',), True), (('training_gt_modified',), True),
                   (('assets', 'CargoBox', 'variation_fraction'), .9),
                   (('assets', 'CargoBox', 'variation_fraction'), float('nan')),
                   (('assets', 'CargoBox', 'minimum_original'), 0),
                   (('assets', 'CargoBox', 'bbox_axis_to_scale_axis'), [0, 1, 1]),
                   (('assets', 'CargoBox', 'bbox_axis_to_scale_axis'), [True, 1, 2]),
                   (('assets', 'CargoBox', 'profiles', 'iso125', 'scale_xyz'), [1, 0, 1]),
                   (('assets', 'CargoBox', 'profiles', 'iso125', 'scale_xyz'), [1, float('inf'), 1]),
                   (('assets', 'CargoBox', 'profiles', 'iso125', 'weight'), False)]
        for path, value in changes:
            policy = copy.deepcopy(self.policy)
            target = policy
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            with self.subTest(path=path), self.assertRaises(scale.ScaleContractError):
                scale.validate_policy(policy)

    def test_drums_must_remain_isotropic(self):
        self.policy['assets']['RedBarrel']['profiles']['iso120']['scale_xyz'] = [1.2, 1.3, 1.2]
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_policy(self.policy)

    def test_unapproved_status_or_partial_qualification_cannot_be_used(self):
        self.policy['assets']['CargoBox']['profiles']['iso125']['status'] = 'qualified'
        with self.assertRaises(scale.ScaleContractError):
            self.select(profile_id='iso125')
        self.qualify()
        for key, value in (('observations', 1), ('semantic_visual_review', 'not_reviewed'),
                           ('contact_review', 'unknown'), ('axis_mapping_verified', False),
                           ('single_component_obstacle_verified', False)):
            policy = copy.deepcopy(self.policy)
            policy['assets']['CargoBox']['profiles']['iso125']['native_qualification'][key] = value
            with self.subTest(key=key), self.assertRaises(scale.ScaleContractError):
                scale.validate_policy(policy)

    def test_qualified_profile_uses_measured_geometry_without_double_scale(self):
        self.qualify()
        assignment = self.select(profile_id='iso125')
        size, pivot, proof = scale.placement_geometry([1, 1, .5], [0, 0, .25], assignment, self.policy)
        self.assertEqual(size, [1.1, 1.2, .7])
        self.assertEqual(pivot, [.01, -.02, .32])
        self.assertEqual(proof['mode'], 'qualified_native_geometry_not_rescaled')
        self.assertFalse(proof['training_gt_modified'])
        self.assertFalse(proof['scale1_height_cache_reusable'])
        self.assertFalse(assignment['candidate_only'])

    def test_evidence_changes_are_fatal_even_for_existing_assignment(self):
        path = self.qualify()
        assignment = self.select(profile_id='iso125')
        path.write_text('changed')
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_assignment(assignment, self.policy)

    def test_candidate_geometry_has_explicit_axis_mapping_and_signed_pivot(self):
        assignment = self.select(profile_id='sample_2_3_2', allow_candidate=True)
        size, pivot, proof = scale.placement_geometry([1, 2, 3], [-.1, .2, -.3], assignment,
                                                     self.policy, allow_candidate=True)
        self.assertEqual(size, [2., 4., 9.])
        for actual, expected in zip(pivot, [-.2, .4, -.9]):
            self.assertAlmostEqual(actual, expected)
        self.assertIn('not_native_certification', proof['mode'])
        self.assertFalse(proof['scale1_height_cache_reusable'])
        assignment = self.select(profile_id='original')
        self.assertTrue(scale.placement_geometry([1, 2, 3], [0, 0, 1], assignment,
                                               self.policy)[2]['scale1_height_cache_reusable'])

    def test_assignment_tampering_rejected_even_with_rehashed_payload(self):
        assignment = self.select(profile_id='iso125', allow_candidate=True)
        for key, value in (('scale_xyz', [2, 2, 2]), ('name', 'WoodBox'), ('candidate_only', False),
                           ('semantic_class', 'vehicle'), ('actor_key', 'foreign'),
                           ('bbox_axis_to_scale_axis', [0, 1, 2]), ('training_gt_modified', True)):
            changed = copy.deepcopy(assignment)
            changed[key] = value
            changed['assignment_sha256'] = scale.digest({k: v for k, v in changed.items()
                                                        if k != 'assignment_sha256'})
            with self.subTest(key=key), self.assertRaises(scale.ScaleContractError):
                scale.validate_assignment(changed, self.policy, allow_candidate=True)

    def test_policy_change_invalidates_old_assignment(self):
        assignment = self.select(profile_id='original')
        self.policy['assets']['CargoBox']['variation_fraction'] = .2
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_assignment(assignment, self.policy)

    def test_actor_is_copied_template_scale_overwritten_not_multiplied(self):
        assignment = self.select(profile_id='iso125', allow_candidate=True)
        original = copy.deepcopy(self.actor)
        actor = scale.apply_actor(self.actor, assignment, self.policy, allow_candidate=True)
        self.assertEqual(self.actor, original)
        self.assertEqual(actor['scale']['x'], 1.25)
        self.assertEqual(actor['scale']['_y'], '1.250')
        self.assertEqual(scale.verify_actor(actor, assignment, self.policy, asset=self.asset,
                         row=self.row, ordinal=0, motion='static', allow_candidate=True), assignment)
        actor['scale']['_y'] = '1.200'
        with self.assertRaises(scale.ScaleContractError):
            scale.verify_actor(actor, assignment, self.policy, allow_candidate=True)

    def test_cross_actor_and_cross_seed_resume_rejected(self):
        assignment = self.select(profile_id='iso125', allow_candidate=True)
        actor = scale.apply_actor(self.actor, assignment, self.policy, allow_candidate=True)
        with self.assertRaises(scale.ScaleContractError):
            scale.apply_actor(dict(actor, DataID=40100002), assignment, self.policy, allow_candidate=True)
        for row, ordinal in ((dict(self.row, seed=20), 0), (self.row, 1)):
            with self.assertRaises(scale.ScaleContractError):
                scale.verify_actor(actor, assignment, self.policy, asset=self.asset, row=row,
                                   ordinal=ordinal, motion='static', allow_candidate=True)
        with self.assertRaises(scale.ScaleContractError):
            scale.verify_actor(actor, assignment, self.policy, asset=self.asset, allow_candidate=True)

    def items(self, count=100):
        return [dict(asset=self.asset, row=dict(self.row, seed=i,
                     role=('front', 'left', 'rear', 'right')[i % 4],
                     family='obstacle' if i % 2 else 'pedestrian_related'),
                     ordinal=0 if i % 3 == 0 else 1, motion='static') for i in range(count)]

    def test_stratified_allocation_is_order_independent_exact_and_original_protected(self):
        items = self.items()
        before = copy.deepcopy(items)
        mapping, summary = scale.stratified_profiles(items, self.policy, allow_candidate=True)
        reversed_map, reversed_summary = scale.stratified_profiles(items[::-1], self.policy, allow_candidate=True)
        self.assertEqual((mapping, summary), (reversed_map, reversed_summary))
        self.assertEqual(items, before)
        self.assertEqual(sum(v != 'original' for v in mapping.values()), 30)
        self.assertEqual(summary['groups'][0]['variants'], 30)
        primary_original = [item for item in items if item['ordinal'] == 0
                            and mapping[scale.actor_key(self.asset, item['row'], 0)] == 'original']
        self.assertTrue(primary_original)
        for key in summary['groups'][0]['protected_original_keys']:
            self.assertEqual(mapping[key], 'original')

    def test_rare_assets_and_val_protected_separately(self):
        items = self.items(2)
        items += [dict(asset=self.asset, row=dict(self.row, seed=99, split='val'), ordinal=0, motion='static')]
        mapping, summary = scale.stratified_profiles(items, self.policy, allow_candidate=True)
        self.assertEqual(set(mapping.values()), {'original'})
        self.assertEqual({g['split'] for g in summary['groups']}, {'train', 'val'})
        self.assertTrue(all(g['variants'] == 0 for g in summary['groups']))

    def test_allocation_without_native_approval_has_zero_variants(self):
        mapping, summary = scale.stratified_profiles(self.items(), self.policy)
        self.assertEqual(set(mapping.values()), {'original'})
        self.assertEqual(summary['groups'][0]['variants'], 0)

    def test_duplicate_slots_unknown_split_rejected_and_base_metadata_consistent(self):
        items = self.items(2)
        with self.assertRaises(scale.ScaleContractError):
            scale.stratified_profiles([items[0], items[0]], self.policy)
        items[0]['row']['split'] = 'test'
        with self.assertRaises(scale.ScaleContractError):
            scale.stratified_profiles(items, self.policy)
        assignment = self.select(profile_id='original')
        self.assertEqual(scale.validate_assignment(assignment, self.policy), assignment)

    def source_actors(self):
        return [dict(data_id=self.asset['data_id'], name=self.asset['name'], source_list='objectList',
                     unique_id=8500, population_pool='obstacle', motion='static',
                     estimated_size=[1., 1., .5])]

    def test_attach_freezes_source_actor_profiles_and_retry_has_no_fallback(self):
        self.qualify()
        row = dict(self.row, actors=1)
        actors = self.source_actors()
        source = copy.deepcopy(row)
        frozen = scale.attach_plan(row, actors, self.policy, ['iso125'])
        self.assertEqual(row, source)
        plan = scale.validate_plan(frozen, {scale.CONFIG_KEY: self.policy})
        self.assertEqual(plan['source_spec'], source)
        self.assertEqual(frozen['frozen_population_data_ids'], [40100001])
        size, pivot, assignment, proof = scale.planned_geometry(self.asset, frozen, 0,
            'static', {scale.CONFIG_KEY: self.policy}, [1., 1., .5], [0., 0., .25])
        self.assertEqual(size, [1.1, 1.2, .7])
        self.assertEqual(pivot, [.01, -.02, .32])
        self.assertEqual(assignment['profile_id'], 'iso125')
        self.assertEqual(proof['mode'], 'qualified_native_geometry_not_rescaled')
        for key, value in (('seed', 20), ('role', 'rear'), ('actors', 2)):
            changed = dict(frozen, **{key: value})
            with self.subTest(key=key), self.assertRaises(scale.ScaleContractError):
                scale.validate_plan(changed, {scale.CONFIG_KEY: self.policy})
        with self.assertRaises(scale.ScaleContractError):
            scale.planned_geometry(self.asset, frozen, 0, 'walking',
                {scale.CONFIG_KEY: self.policy}, [1, 1, .5], [0, 0, .25])

    def test_original_only_attach_is_exact_legacy_and_candidate_attach_is_forbidden(self):
        row = dict(self.row, actors=1)
        self.assertEqual(scale.attach_plan(row, self.source_actors(), self.policy, ['original']), row)
        with self.assertRaises(scale.ScaleContractError):
            scale.attach_plan(row, self.source_actors(), self.policy, ['iso125'])
        self.assertEqual(scale.planned_geometry(self.asset, row, 0, 'static', {},
                        [1, 2, 3], [0, 0, 1]), ([1, 2, 3], [0, 0, 1], None, None))

    def test_range_rebinding_changes_only_new_frozen_spec_hash_and_is_auditable(self):
        self.qualify()
        row = dict(self.row, actors=1,
                   primary_obstacle_range=dict(frozen_spec_fields_sha256='old', cap_m=40,
                                               distance_bounds_m=[30, 40], classifier='retained'))
        frozen = scale.attach_plan(row, self.source_actors(), self.policy, ['iso125'])
        self.assertEqual(frozen[scale.PLAN_KEY]['source_spec']['primary_obstacle_range'],
                         row['primary_obstacle_range'])
        self.assertTrue(frozen[scale.PLAN_KEY]['range_plan_rebind'])
        expected = dict(row['primary_obstacle_range'])
        expected['frozen_spec_fields_sha256'] = scale.digest(
            {k: value for k, value in frozen.items() if k != 'primary_obstacle_range'})
        self.assertEqual(frozen['primary_obstacle_range'], expected)
        changed = copy.deepcopy(frozen)
        changed['primary_obstacle_range']['cap_m'] = 100
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_plan(changed, {scale.CONFIG_KEY: self.policy})

    def test_variant_height_never_reads_saved_scale1_height_cache(self):
        self.qualify()
        assignment = self.select(profile_id='iso125')
        import base5_map_surface
        with patch('base5_static_height.saved_height_candidate', side_effect=AssertionError('scale1 cache reused')):
            result, initializer, ground = base5_map_surface.initial_pose(
                dict(x=1., y=2., z=3., yaw=0.), self.asset, [1.1, 1.2, .7], [.01, -.02, .32],
                {}, scale_assignment=assignment, scale_policy=self.policy)
        self.assertAlmostEqual(result['z'], 3.01)
        self.assertFalse(initializer['scale1_height_cache_reused'])
        self.assertIsNone(ground)

    def test_set_pose_applies_scale_after_legacy_template_reset(self):
        import generate_base5_independent as gen
        assignment = self.select(profile_id='iso125', allow_candidate=True)
        # The production hook intentionally does not accept candidate flags.
        scene = dict(objectList=[], waypointDataList=[])
        with self.assertRaises(scale.ScaleContractError):
            gen.set_pose(dict(self.actor, pos={}, rot={}), dict(x=0., y=0., z=0., yaw=0.),
                         8500, self.asset, scene, 'static', asset_scale=assignment,
                         asset_scale_policy=self.policy)
        self.qualify()
        assignment = self.select(profile_id='iso125')
        gen.set_pose(dict(self.actor, pos={}, rot={}), dict(x=0., y=0., z=0., yaw=0.),
                     8500, self.asset, scene, 'static', asset_scale=assignment,
                     asset_scale_policy=self.policy)
        self.assertEqual(len(scene['objectList']), 1)
        self.assertEqual(scene['objectList'][0]['scale']['x'], 1.25)


if __name__ == '__main__':
    unittest.main()
