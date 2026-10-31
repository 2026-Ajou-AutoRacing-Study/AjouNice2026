import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_secondary_far_vehicle as far
import prepare_base5_secondary_far_vehicle_candidate as prepare


def actor(uid, distance):
    return dict(unique_id=uid, data_id=uid, name='car_'+str(uid), source_list='vehicleList',
        population_pool='vehicle', motion='static', position=dict(x=distance, y=0., z=0.),
        geometric_center_estimate=[distance, 0.], estimated_size=[4., 2., 1.5],
        placement_yaw_deg=0., planned_center_distance_m=distance, longitudinal_m=distance,
        lateral_m=0., actual_ego_role='front', map_ground_initializer=dict(link_id='A2256W000409'),
        stationary_vehicle_policy='park-static-snapshot-v119', unchanged_field={'keep': True})


class FarVehicleTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manifest = self.root/'manifest.json'
        self.manifest.write_text(json.dumps(dict(config=dict(environment_tag='sunny13'))))
        self.meta = dict(spec=dict(family='highway', mission=False, seed=11, actors=2,
            role='front', distance_bounds=[6, 30], population_budget_bounds=[10, 14]),
            ego=dict(x=0., y=0., z=0., yaw=0.), base=dict(link_idx='A2256W000409', yaw=0.),
            runtime_actors=[actor(8500, 10.), actor(8501, 35.)], split='train',
            split_group='A2256W000409', details=dict(topology='three_lane'),
            scene_group_id='original_group', population_budget=dict(actor_count=2),
            planned_primary_distance_m=10., retry=3, other_top_level=dict(keep=True))
        self.scene = dict(egoVehicle=dict(keep='Ego'), vehicleList=[], objectList=[], background='same')
        for a in self.meta['runtime_actors']:
            asset = dict(source_list='vehicleList', data_id=a['data_id'], name=a['name'])
            pose = dict(a['position'], yaw=0., link_idx='A2256W000409', point_ratio=.5)
            npc = dict(DataID=a['data_id'], UNIQUEID=a['unique_id'], initPosition=dict(
                pos={}, rot={}, vehicleSaveData=dict(inputData=[0]*14, gearbox={})))
            far.gen.set_pose(npc, pose, a['unique_id'], asset, self.scene, 'static',
                             stationary_vehicle_policy='park-static-snapshot-v119')
        self.entry = dict(id='slot', scenario=str(self.root/'scene.json'), metadata=str(self.root/'meta.json'))
        Path(self.entry['scenario']).write_text(json.dumps(self.scene))
        Path(self.entry['metadata']).write_text(json.dumps(self.meta))
        self.config = dict(environment_tag='sunny13', capture=dict(
            static_vehicle_policy='park-static-snapshot-v119'), placement_ground=dict(enabled=True))
        self.config[far.CONFIG_KEY] = far.configuration('sunny13', self.manifest)
        self.calibration = far.visibility.validate_policy(self.config[far.CONFIG_KEY]['proposal_visibility'])
        self.spec = far.attach(self.entry, self.meta, self.scene, 1, [60, 80], self.config[far.CONFIG_KEY])
        self.ctx = dict(pools=dict(vehicle=[dict(data_id=8501, name='car_8501', source_list='vehicleList')]))

    def built(self, attempt=0, x=70.):
        pose = dict(x=x, y=0., z=0., yaw=0., link_idx='A2256W000409', point_ratio=.7)
        candidate = (pose, [x, 0.], x, 0., x, x-5.)
        ground = dict(usable=True, confidence='near_centerline', footprint=dict(all_supported=True),
                      link_id='A2256W000409')
        with patch.object(far, '_candidates', return_value=([candidate], [0., 0., .75])), \
                patch.object(far, 'configure'), patch.object(far, 'initial_pose',
                    return_value=(pose, dict(usable=False), ground)):
            return far.build(self.spec, self.config, self.ctx, attempt)

    def choose(self, *args):
        # These original ranking tests deliberately provide distinct synthetic
        # geometries; road-point exhaustion has separate focused tests below.
        with patch.object(far, 'proposal_index', side_effect=lambda row, ctx, retry: (retry, 1000000)):
            return far.choose(*args)

    def test_plan_only_adds_declared_field_and_pins_source(self):
        self.assertEqual({k: v for k, v in self.spec.items() if k != far.PLAN_KEY}, self.meta['spec'])
        plan = far.validate_plan(self.spec, self.config)
        self.assertEqual(plan['target_uid'], 8501)
        self.assertEqual(plan['lane_relation'], 'same')
        self.assertEqual(plan['distance_bounds_m'], [60, 80])
        self.assertEqual(plan['frozen_seed'], 11)
        for bounds in ([60, 80], [80, 90], [90, 95], [95, 99]):
            spec = far.attach(self.entry, self.meta, self.scene, 1, bounds, self.config[far.CONFIG_KEY])
            self.assertEqual(far.validate_plan(spec, self.config)['distance_bounds_m'], bounds)

    def test_unchanged_legacy_validation_and_foreign_assignment(self):
        self.assertIsNone(far.validate_plan(self.meta['spec']))
        self.assertIsNone(far.verify_preserved(self.meta, self.meta))
        _, after = self.built()
        with self.assertRaises(far.FarContractError): far.verify_preserved(self.meta, after)

    def test_target_only_build_keeps_source_everything_else(self):
        snapshot = copy.deepcopy((self.meta, self.scene, self.spec))
        scene, result = self.built()
        self.assertEqual(result['ego'], self.meta['ego'])
        self.assertEqual(result['runtime_actors'][0], self.meta['runtime_actors'][0])
        self.assertEqual(scene['egoVehicle'], self.scene['egoVehicle'])
        self.assertEqual(scene['vehicleList'][0], self.scene['vehicleList'][0])
        self.assertEqual(result['runtime_actors'][1]['planned_center_distance_m'], 70.)
        self.assertEqual(result['spec']['seed'], 11)
        self.assertEqual((self.meta, self.scene, self.spec), snapshot)
        self.assertEqual(self.built(), (scene, result))

    def test_source_file_tamper_is_fatal_in_validation_and_audit(self):
        _, result = self.built()
        Path(self.entry['metadata']).write_text('{}')
        with self.assertRaises(far.FarContractError): far.validate_plan(self.spec, self.config)
        with self.assertRaises(far.FarContractError): far.verify_preserved(result, result)

    def test_helper_config_environment_seed_and_plan_tamper(self):
        for mutation in ('helper', 'environment', 'policy_environment', 'seed', 'uid', 'flag'):
            spec, config = copy.deepcopy(self.spec), copy.deepcopy(self.config)
            if mutation == 'helper': config[far.CONFIG_KEY]['source_sha256'] = 'bad'
            elif mutation == 'environment': config['environment_tag'] = 'foggy13'
            elif mutation == 'policy_environment':
                config['environment_tag'] = 'foggy13'
                config[far.CONFIG_KEY]['environment_tag'] = 'foggy13'
                spec[far.PLAN_KEY]['environment_tag'] = 'foggy13'
            elif mutation == 'seed': spec['seed'] += 1
            elif mutation == 'uid': spec[far.PLAN_KEY]['target_uid'] = 8500
            else: spec[far.PLAN_KEY]['primary_preserved'] = False
            with self.subTest(mutation=mutation), self.assertRaises(far.FarContractError):
                far.validate_plan(spec, config)

    def test_non_target_pose_and_target_identity_tamper_are_fatal(self):
        _, result = self.built()
        for mutation in ('primary', 'ego', 'count', 'asset', 'size', 'other_top_level', 'distance', 'role', 'lane'):
            after = copy.deepcopy(result)
            if mutation == 'primary': after['runtime_actors'][0]['position']['x'] += 1
            elif mutation == 'ego': after['ego']['x'] += 1
            elif mutation == 'count': after['runtime_actors'].pop()
            elif mutation == 'asset': after['runtime_actors'][1]['name'] = 'different'
            elif mutation == 'size': after['runtime_actors'][1]['estimated_size'][0] = 20.
            elif mutation == 'other_top_level': after['other_top_level']['keep'] = False
            elif mutation == 'distance': after['runtime_actors'][1]['planned_center_distance_m'] = 80.
            elif mutation == 'role': after['runtime_actors'][1]['actual_ego_role'] = 'rear'
            else: after['runtime_actors'][1]['map_ground_initializer']['link_id'] = 'A2256W000434'
            with self.subTest(mutation=mutation), self.assertRaises(far.FarContractError):
                far.verify_preserved(result, after)

    def test_target_state_speed_gear_behavior_and_other_payload_tamper(self):
        scene, result = self.built()
        plan = result['spec'][far.PLAN_KEY]
        for mutation in ('speed', 'gear', 'behavior', 'other', 'ego', 'background'):
            after = copy.deepcopy(scene)
            if mutation == 'speed': after['vehicleList'][1]['desiredVelocity'] = 1.
            elif mutation == 'gear': after['vehicleList'][1]['initPosition']['gear'] = 4
            elif mutation == 'behavior': after['vehicleList'][1]['isLaneChange'] = True
            elif mutation == 'other': after['vehicleList'][0]['initPosition']['pos']['x'] += 1
            elif mutation == 'ego': after['egoVehicle']['keep'] = 'changed'
            else: after['background'] = 'changed'
            with self.subTest(mutation=mutation), self.assertRaises(far.FarContractError):
                far.verify_payload(after, self.scene, plan)

    def test_serialized_target_link_pose_and_roll_tamper(self):
        scene, meta = self.built()
        plan = meta['spec'][far.PLAN_KEY]
        for mutation in ('link', 'x', 'z', 'yaw', 'roll', 'ratio'):
            after = copy.deepcopy(scene)
            initial = after['vehicleList'][1]['initPosition']
            if mutation == 'link': initial['initLink'] = 'FOREIGN'
            elif mutation in ('x', 'z'): initial['pos'][mutation] += 1.
            elif mutation == 'yaw': initial['rot']['yaw'] = '90.000'
            elif mutation == 'roll': initial['rot']['roll'] = '5.000'
            else: initial['initLinkRatio'] = float('nan')
            with self.subTest(mutation=mutation), self.assertRaises(far.FarContractError):
                far.verify_payload(after, self.scene, plan, meta)
        after = copy.deepcopy(scene)
        after['vehicleList'][1]['initPosition']['initLink'] = 'FOREIGN'
        with self.assertRaises(far.FarContractError): far.verify_payload(after, self.scene, plan)

    def test_original_near_primary_and_unsupported_lane_not_donors(self):
        for ordinal, bounds in ((0, [60, 80]), (1, [50, 60]), (1, [80, 99])):
            with self.assertRaises(far.FarGeometryExhausted):
                far.attach(self.entry, self.meta, self.scene, ordinal, bounds, self.config[far.CONFIG_KEY])
        self.assertEqual(far.lane_relation('A2256W000409', 'A2256W000410'), 'adjacent')
        self.assertIsNone(far.lane_relation('A2256W000409', 'A2256W000418'))

    def test_empty_geometry_is_bounded_and_not_contract_error(self):
        with patch.object(far, '_candidates', return_value=([], [0, 0, 0])):
            with self.assertRaises(far.FarGeometryExhausted): far.build(self.spec, self.config, self.ctx)
        self.assertTrue(issubclass(far.FarContractError, RuntimeError))
        self.assertFalse(issubclass(far.FarContractError, ValueError))

    def test_existing_region_gate_and_regional_proof_are_preserved(self):
        with patch('base5_region_contract.validate', side_effect=ValueError('primary nearly occluded')):
            with self.assertRaisesRegex(far.FarGeometryExhausted, 'primary nearly occluded'):
                self.built()
        with patch('base5_region_contract.validate', return_value={'actor_regions': ['other']}):
            with self.assertRaisesRegex(far.FarGeometryExhausted, 'regional placement proof'):
                self.built()

    def test_completion_uses_target_novelty_not_unchanged_primary(self):
        _, source = self.built(x=65.)
        calls = []
        def builder(row, config, ctx, retry):
            calls.append(retry)
            return self.built(retry, x=75.)
        score_calls = []
        def scorer(meta, calibration, kind):
            score_calls.append(meta['runtime_actors'][0]['unique_id'])
            return dict(score=2.)
        scene, result = self.choose(source, self.config, self.ctx, self.calibration, 2, 'slot', set(),
            builder, scorer, lambda m, h: 0.)
        self.assertEqual(len(calls), 8)
        self.assertEqual(calls[0], source['retry']+1+4096)
        self.assertEqual(set(score_calls), {8501})
        self.assertGreater(result['flexible_completion']['target_displacement_m'], 9.)
        self.assertEqual(result['runtime_actors'][0], source['runtime_actors'][0])

    def test_known_payload_and_no_target_movement_exhaust_finitely(self):
        scene, source = self.built(x=70.)
        known = {far.sha256(self.entry['scenario'])}
        import hashlib
        known.add(hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2)+'\n').encode()).hexdigest())
        count = []
        def builder(row, config, ctx, retry):
            count.append(retry)
            return self.built(retry, x=70.)
        with self.assertRaises(far.FarGeometryExhausted):
            self.choose(source, self.config, self.ctx, self.calibration, 1, 'slot', known,
                       builder, lambda *a: dict(score=1.), lambda *a: 1.)
        self.assertEqual(len(count), 512)

    def test_contract_error_from_builder_is_not_geometry_exhaustion(self):
        _, source = self.built()
        def broken(*args): raise far.FarContractError('bad pin')
        with self.assertRaisesRegex(far.FarContractError, 'bad pin'):
            self.choose(source, self.config, self.ctx, self.calibration, 1, 'slot', set(),
                       broken, lambda *a: dict(score=1.), lambda *a: 1.)

    def test_visibility_helper_reference_sensor_and_normalizer_pins_are_fatal(self):
        for key in ('implementation', 'ray_helper', 'geometry_reference', 'sensor', 'normalizer'):
            spec, config = copy.deepcopy(self.spec), copy.deepcopy(self.config)
            config[far.CONFIG_KEY]['proposal_visibility'][key]['sha256'] = 'bad'
            spec[far.PLAN_KEY]['proposal_visibility'][key]['sha256'] = 'bad'
            with self.subTest(key=key), self.assertRaises(far.FarContractError):
                far.validate_plan(spec, config)
        _, result = self.built()
        result['spec'][far.PLAN_KEY]['proposal_visibility']['geometry_reference']['sha256'] = 'bad'
        with self.assertRaises(far.FarContractError): far.verify_preserved(result, result)

    def test_supplied_calibration_cannot_override_frozen_input(self):
        _, source = self.built()
        with self.assertRaisesRegex(far.FarContractError, 'calibration'):
            self.choose(source, self.config, self.ctx, {}, 1, 'slot', set(),
                       lambda *a: self.fail('Builder should not run'), lambda *a: dict(score=1.), lambda *a: 1.)

    def test_complete_blockage_is_geometry_failure_but_partial_is_retained(self):
        with patch.object(far.visibility, 'evaluate', return_value=dict(rejected=True)):
            with self.assertRaisesRegex(far.FarGeometryExhausted, '3-D proposal blockage'):
                self.built()
        with patch.object(far.visibility, 'evaluate', return_value=dict(rejected=False, reason='partial')):
            _, result = self.built()
            self.assertEqual(result['secondary_far_vehicle_generation']['proposal_visibility']['reason'], 'partial')

    def test_saved_visibility_proof_tampering_halts(self):
        _, source = self.built()
        source['secondary_far_vehicle_generation']['proposal_visibility']['reason'] = 'forged'
        with self.assertRaisesRegex(far.FarContractError, 'visibility proof'):
            far.verify_preserved(source, source)

    def test_completion_applies_same_guard_to_callback_result(self):
        _, source = self.built(x=65.)
        count = []
        def builder(row, config, ctx, retry):
            count.append(retry)
            return self.built(retry, x=75.)
        with patch.object(far, 'score_proposal', return_value=dict(score=-math.inf)) as guard:
            with self.assertRaises(far.FarGeometryExhausted):
                self.choose(source, self.config, self.ctx, self.calibration, 1, 'slot', set(),
                           builder, lambda *a: dict(score=1.), lambda *a: 1.)
        self.assertEqual(guard.call_count, 512)
        self.assertEqual(len(count), 512)

    def test_allocation_preserves_required_totals_and_asset_fairness(self):
        self.assertEqual(sum(n for _, _, n in prepare.DEFAULT_QUOTAS), 160)
        self.assertEqual({tuple(b): sum(n for bb, r, n in prepare.DEFAULT_QUOTAS if bb == b)
                          for b, _, _ in prepare.DEFAULT_QUOTAS},
                         {(60, 80): 60, (80, 90): 50, (90, 95): 24, (95, 99): 26})
        values = [dict(id='a', data_id=1, ordinal=1, role='front'),
                  dict(id='b', data_id=2, ordinal=1, role='front')]
        ordered = prepare.order_candidates(values, {1: 2, 2: 0}, ((60, 80), 'front'), set())
        self.assertEqual(ordered[0]['data_id'], 2)
        self.assertEqual(prepare.order_candidates(values, {1: 2, 2: 0}, ((60, 80), 'front'), {'b'}), values[:1])

    def test_tail_sampling_window_is_explicit_and_frozen(self):
        values = [prepare.sampling_bounds((95, 99), i, 13) for i in range(13)]
        self.assertEqual(values.count([95, 97]), 6)
        self.assertEqual(values.count([97, 99]), 7)
        self.assertEqual(prepare.sampling_bounds((80, 90), 0, 25), [80, 90])
        with self.assertRaises(far.FarContractError): prepare.sampling_bounds((95, 99), 0, 12)
        self.spec = far.attach(self.entry, self.meta, self.scene, 1, [95, 99],
                              self.config[far.CONFIG_KEY], sampling_bounds_m=[97, 99])
        plan = far.validate_plan(self.spec, self.config)
        self.assertEqual(plan['sampling_bounds_m'], [97, 99])
        _, result = self.built(x=98.)
        result['runtime_actors'][1]['planned_center_distance_m'] = 96.
        with self.assertRaises(far.FarContractError): far.verify_preserved(result, result)

    def test_unregistered_sampling_windows_and_source_plan_tamper_are_fatal(self):
        for bounds, window in (([60, 80], [70, 80]), ([95, 99], [96, 99]), ([95, 99], [99, 101])):
            with self.subTest(bounds=bounds, window=window), self.assertRaises(far.FarContractError):
                far.attach(self.entry, self.meta, self.scene, 1, bounds,
                           self.config[far.CONFIG_KEY], sampling_bounds_m=window)
        modified = copy.deepcopy(self.spec)
        modified[far.PLAN_KEY]['sampling_bounds_m'] = [70, 80]
        with self.assertRaises(far.FarContractError): far.validate_plan(modified, self.config)

    def test_proposal_index_matches_original_choice_and_is_stateless(self):
        import random
        points = list(range(7))
        with patch.object(far, '_candidates', return_value=(points, [0., 0., 0.])):
            for attempt in range(30):
                seed = int(far.digest([far.POLICY, 11, 'slot', attempt])[:16], 16)
                expected = random.Random(seed).choice(points)
                self.assertEqual(far.proposal_index(self.spec, self.ctx, attempt), (expected, 7))
                self.assertEqual(far.proposal_index(self.spec, self.ctx, attempt), (expected, 7))

    def test_four_legal_points_are_built_once_in_first_draw_order(self):
        _, source = self.built(x=65.)
        calls = []
        first = source['retry']+1
        # Duplicate draws must neither count as eight valid layouts nor rebuild.
        indices = [0, 0, 1, 2, 1, 3]
        def point_index(row, ctx, retry):
            return indices[retry-first], 4
        def builder(row, config, ctx, retry):
            calls.append(retry)
            return self.built(retry, x=75.)
        with patch.object(far, 'proposal_index', side_effect=point_index):
            _, result = far.choose(source, self.config, self.ctx, self.calibration, 1, 'slot', set(),
                                   builder, lambda *a: dict(score=1.), lambda *a: 1.)
        self.assertEqual(calls, [first+i for i in (0, 2, 3, 5)])
        self.assertEqual(result['flexible_completion']['distinct_geometry_proposals'], 4)
        self.assertEqual(result['flexible_completion']['random_draws_considered'], 6)
        self.assertEqual(result['flexible_completion']['available_legal_road_points'], 4)

    def test_all_blocked_distinct_points_exhaust_without_512_rebuilds(self):
        _, source = self.built()
        calls = []
        def builder(row, config, ctx, retry):
            calls.append(retry)
            raise far.FarGeometryExhausted('complete blocker')
        with patch.object(far, 'proposal_index', side_effect=lambda row, ctx, retry: (retry % 4, 4)):
            with self.assertRaises(far.FarGeometryExhausted):
                far.choose(source, self.config, self.ctx, self.calibration, 1, 'slot', set(),
                           builder, lambda *a: dict(score=1.), lambda *a: 1.)
        self.assertEqual(len(calls), 4)


if __name__ == '__main__':
    unittest.main()
