"""CPU integration boundaries for the opt-in frozen far-secondary policy.

These tests isolate expensive placement and ROS children. Real helper geometry,
native evidence and frozen-payload checks have separate tests; no live process,
MORAI control, model inference or capture is launched here.
"""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import base5_capture_safety as safety
import base5_far_vehicle_native as native
import base5_secondary_far_vehicle as far
import generate_base5_independent as gen
import run_base5_capture_campaign as campaign
import run_base5_fresh_completion as fresh
import run_base5_quality_loop as quality


def payload_sha(value):
    return hashlib.sha256((json.dumps(
        value, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()


def metadata(planned=True):
    spec = dict(family='highway', mission=False, role='front',
                distance_bounds=[10, 20], seed=11)
    if planned:
        spec[far.PLAN_KEY] = dict(target_ordinal=1,
                                  distance_bounds_m=[80, 90], role='rear')
    return dict(spec=spec, split='train', scene_group_id='physical-group', retry=0,
        runtime_actors=[dict(data_id=30100001, unique_id=8500, name='primary',
                            geometric_center_estimate=[15., 0.], motion='static'),
                        dict(data_id=30100003, unique_id=8501, name='secondary',
                            geometric_center_estimate=[-85., 0.], motion='static')])


class TemporaryFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
        return path

    def fixture(self, planned=True):
        meta = metadata(planned)
        scene = dict(example='source')
        sp = self.write('source/scene.json', scene)
        mp = self.write('source/metadata.json', meta)
        entry = dict(id='slot', scenario=str(sp), metadata=str(mp),
                     scenario_sha256=safety.sha256(sp), split='train',
                     scene_group_id=meta['scene_group_id'], family='highway',
                     mission=False, role='front', distance_bounds=[10, 20],
                     motion='static', subfamily=None)
        config = dict(environment_tag='sunny13', fresh_geometry_policy='test')
        if planned:
            config[far.CONFIG_KEY] = dict(policy=far.POLICY)
        deck = dict(config=config, scenes=[entry], count=1)
        manifest = self.write('manifest.json', deck)
        return meta, scene, entry, deck, manifest


class GeneratorAndDonorIntegrationTest(unittest.TestCase):
    def test_generator_dispatches_far_spec_without_modifying_input(self):
        row = metadata()['spec']
        before = copy.deepcopy(row)
        expected = (dict(example='scene'), dict(example='metadata'))
        with patch.object(far, 'build', return_value=expected) as build:
            result = gen.build(row, {'x': 1}, {'ctx': 2}, attempt=7)
        self.assertIs(result, expected)
        build.assert_called_once_with(before, {'x': 1}, {'ctx': 2}, 7)
        self.assertEqual(row, before)
        self.assertIsNot(build.call_args.args[0], row)

    def test_other_explicit_policy_does_not_dispatch_far_builder(self):
        row = dict(primary_obstacle_range={'test': True})
        import base5_primary_obstacle_range as primary_range
        with patch.object(far, 'build') as far_build, patch.object(
                primary_range, 'validate_plan',
                side_effect=ValueError('other policy validation reached')) as validate:
            with self.assertRaisesRegex(ValueError, 'other policy validation reached'):
                gen.build(row, {}, {}, 3)
        far_build.assert_not_called()
        validate.assert_called_once()

    def test_far_completion_uses_own_source_even_below_asset_floor(self):
        source = metadata()
        donor = metadata(False)
        donor['runtime_actors'][0]['data_id'] = 999
        with patch.object(fresh, 'compatible') as compatible:
            result = fresh.eligible_floor_donors(
                source, [donor], Counter(), 4, {})
        self.assertEqual(len(result), 1)
        self.assertIs(result[0], source)
        compatible.assert_not_called()

    def test_unplanned_donor_pool_excludes_all_frozen_policy_donors(self):
        source = metadata(False)
        ordinary = copy.deepcopy(source)
        far_donor = metadata()
        front = metadata(False)
        front['spec']['secondary_obstacle_front_support'] = {'test': True}
        ranged = metadata(False)
        ranged['spec']['primary_obstacle_range'] = {'test': True}
        # Query the existing front adapter predicate instead of assuming its key.
        with patch.object(fresh.front_completion, 'has_plan',
                          side_effect=lambda m: m is front), patch.object(
                fresh, 'compatible', return_value=True):
            result = fresh.eligible_floor_donors(
                source, [far_donor, ordinary, front, ranged],
                Counter({30100001: 4}), 4, {})
        self.assertEqual(result, [ordinary])


class FreshCompletionIntegrationTest(TemporaryFiles):
    def test_prepare_calls_far_choose_and_preservation_checks(self):
        original, _, entry, deck, _ = self.fixture()
        candidate_meta = copy.deepcopy(original)
        candidate_scene = dict(example='fresh-pose')
        folder = self.root / 'prepared'
        before = copy.deepcopy(original)
        with patch.object(far, 'validate_plan') as validate, patch.object(
                far, 'choose', return_value=(candidate_scene, candidate_meta)) as choose, patch.object(
                far, 'verify_preserved') as verify, patch.object(
                fresh.front_completion, 'verify_preserved'), patch.object(
                fresh.primary_range, 'verify_preserved'), patch.object(
                fresh, 'compatible', return_value=True), patch.object(
                fresh.flex, 'candidate_delta', return_value={'bounded': True}), patch.object(
                fresh.flex, 'movers', return_value=[]):
            result = fresh.prepare(entry, original, [], deck, {}, {}, 1,
                                   folder, set(), {})
        validate.assert_called_once_with(original['spec'], deck['config'])
        choose.assert_called_once()
        self.assertIs(choose.call_args.args[0], original)
        verify.assert_called_once_with(original, candidate_meta)
        self.assertEqual(original, before)
        self.assertEqual(result['entry']['scene_group_id'], before['scene_group_id'])
        self.assertEqual(result['entry']['distance_bounds'], before['spec']['distance_bounds'])
        saved = json.loads(Path(result['entry']['metadata']).read_text())
        self.assertEqual(saved['spec'], before['spec'])
        self.assertFalse(result['moving'])

    def test_prepare_contract_error_stops_before_proposal_files(self):
        original, _, entry, deck, _ = self.fixture()
        folder = self.root / 'must_not_exist'
        with patch.object(far, 'validate_plan', side_effect=far.FarContractError(
                'source manifest changed')), patch.object(far, 'choose') as choose:
            with self.assertRaisesRegex(far.FarContractError, 'source manifest changed'):
                fresh.prepare(entry, original, [], deck, {}, {}, 1,
                              folder, set(), {})
        choose.assert_not_called()
        self.assertFalse(folder.exists())

    def accepted_attempt(self):
        original, _, entry, deck, manifest = self.fixture()
        capture = self.root / 'capture'
        self.write('capture/metadata.json', original)
        row = dict(id='slot', status='accepted_candidate', capture_root=str(capture))
        attempt = dict(moving=False, manifest=str(manifest),
            manifest_sha256=safety.sha256(manifest), entry=entry, capture=str(capture))
        return original, deck, attempt, row

    def test_verify_checks_actual_accepted_capture_with_far_guard(self):
        original, deck, attempt, row = self.accepted_attempt()
        with patch.object(fresh.adaptive.fa, 'check_placement'), patch.object(
                fresh.adaptive.fa.batch, 'inspect_batch', return_value=[row]), patch.object(
                fresh.front_completion, 'verify_preserved'), patch.object(
                fresh.primary_range, 'verify_preserved'), patch.object(
                far, 'verify_preserved') as verify, patch.object(
                fresh, 'compatible', return_value=True):
            result = fresh.verify(attempt, deck, original, {})
        self.assertEqual(result, row)
        verify.assert_called_once()
        self.assertIs(verify.call_args.args[0], original)
        self.assertEqual(verify.call_args.args[1], json.loads(
            (Path(row['capture_root']) / 'metadata.json').read_text()))

    def test_verify_far_capture_contract_error_is_not_ordinary_review(self):
        original, deck, attempt, row = self.accepted_attempt()
        with patch.object(fresh.adaptive.fa, 'check_placement'), patch.object(
                fresh.adaptive.fa.batch, 'inspect_batch', return_value=[row]), patch.object(
                fresh.front_completion, 'verify_preserved'), patch.object(
                fresh.primary_range, 'verify_preserved'), patch.object(
                far, 'verify_preserved', side_effect=far.FarContractError(
                    'captured target identity changed')):
            with self.assertRaisesRegex(far.FarContractError, 'captured target identity changed'):
                fresh.verify(attempt, deck, original, {})


class QualityReplacementIntegrationTest(TemporaryFiles):
    def test_unknown_far_value_error_is_fatal_not_geometry_exhaustion(self):
        _, _, _, _, manifest = self.fixture()
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=ValueError('unexpected source drift')) as build:
            with self.assertRaisesRegex(ValueError, 'unexpected source drift'):
                quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 1)
        self.assertFalse((self.root / 'out').exists())

    def test_far_geometry_exhaustion_is_bounded_offline_failure(self):
        _, _, _, _, manifest = self.fixture()
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=far.FarGeometryExhausted('no legal pose')) as build:
            with self.assertRaises(quality.PlacementProposalExhausted):
                quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 512)
        self.assertFalse((self.root / 'out').exists())

    def test_far_contract_runtime_error_is_fatal_after_one_proposal(self):
        _, _, _, _, manifest = self.fixture()
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=far.FarContractError('helper changed')) as build:
            with self.assertRaisesRegex(far.FarContractError, 'helper changed'):
                quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 1)

    def test_known_payload_and_too_small_target_displacement_are_skipped(self):
        original, scene, _, _, manifest = self.fixture()
        tiny = copy.deepcopy(original)
        tiny['runtime_actors'][1]['geometric_center_estimate'][0] += .249
        novel = copy.deepcopy(original)
        novel['runtime_actors'][1]['geometric_center_estimate'][0] += 3.
        proposals = [(scene, copy.deepcopy(original)),
                     (dict(example='different-bytes-tiny-pose'), tiny),
                     (dict(example='novel-pose'), novel)]
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=proposals) as build, patch.object(
                far, 'verify_preserved') as verify:
            target = quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 3)
        self.assertEqual(verify.call_count, 3)
        saved_deck = json.loads(target.read_text())
        selected = saved_deck['scenes'][0]
        self.assertEqual(json.loads(Path(selected['scenario']).read_text()),
                         dict(example='novel-pose'))
        saved_meta = json.loads(Path(selected['metadata']).read_text())
        self.assertEqual(saved_meta['spec'], original['spec'])
        self.assertEqual(saved_meta['runtime_actors'][0], original['runtime_actors'][0])

    def test_nonplanned_legacy_value_error_recovery_is_unchanged(self):
        _, _, _, _, manifest = self.fixture(planned=False)
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=ValueError('ordinary old geometry')) as build:
            with self.assertRaises(quality.PlacementProposalExhausted):
                quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 512)

    def test_far_replacement_preserves_inherited_pair_and_replacement_provenance(self):
        original, _, entry, _, manifest = self.fixture()
        link = far.gen.geo.HIGHWAY_LANE_LINK_PATHS[6][0]
        plan = original['spec'][far.PLAN_KEY]
        plan.update(lane_identity='mainline_6', route_links=[link],
            sampling_bounds_m=[80,90],
            proposal_visibility=dict(geometry_reference=dict(path='mock-reference')),
            actor_identity=[{key: actor.get(key) for key in
                ('unique_id', 'data_id', 'name', 'source_list', 'population_pool', 'motion')}
                for actor in original['runtime_actors']])
        target = original['runtime_actors'][1]
        target.update(planned_center_distance_m=85., actual_ego_role='rear',
            map_ground_initializer=dict(link_id=link),
            secondary_far_vehicle=dict(plan_sha256=far.digest(plan)))
        original['obstacle_support_pair'] = dict(policy='inherited-v150', actual_retry=17,
            ego_xy_delta_m=0., pair_precision='exact frozen Ego and primary',
            nested=dict(source='preserve verbatim'))
        original['replacement'] = dict(policy='earlier-frozen-source-replacement',
                                        source_cell_id='earlier-source-slot')
        original['secondary_far_vehicle_generation'] = dict(proposal_visibility={'rejected': False})
        self.write('source/metadata.json', original)
        source_bytes = Path(entry['metadata']).read_bytes()
        canonical = copy.deepcopy(original)
        canonical['spec'].pop(far.PLAN_KEY)
        novel = copy.deepcopy(original)
        novel['runtime_actors'][1]['geometric_center_estimate'][0] -= 3.
        novel['runtime_actors'][1]['planned_center_distance_m'] = 88.
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', return_value=(dict(example='novel-far-pose'), novel)), patch.object(
                far, '_validate_source_only', return_value=({}, canonical, plan)), patch.object(
                far, '_visibility_calibration', return_value={}), patch.object(
                far.visibility, 'evaluate', return_value={'rejected': False}):
            path = quality.replacement(manifest, self.root / 'out')
            result = json.loads(path.read_text())
            saved = json.loads(Path(result['scenes'][0]['metadata']).read_text())
            # Exercise the real frozen metadata verifier AFTER provenance is
            # appended, which is where the native recovery regression occurred.
            self.assertEqual(far.verify_preserved(original, saved), plan)
        self.assertEqual(saved['obstacle_support_pair'], original['obstacle_support_pair'])
        self.assertEqual(saved['replacement'], original['replacement'])
        self.assertEqual(saved['secondary_far_vehicle_replacement']['source_cell_id'], 'slot')
        self.assertEqual(saved['runtime_actors'][0], original['runtime_actors'][0])
        self.assertEqual(Path(entry['metadata']).read_bytes(), source_bytes)


class RuntimeHashIntegrationTest(TemporaryFiles):
    def helper_patches(self):
        # Hash scope is tested independently from full source/geometry fixtures.
        return [patch.object(native, 'validate_manifest',
                             return_value={'original-source': 'frozen-hash'})]

    def test_optin_campaign_contract_pins_helper_and_native_auditor(self):
        _, _, _, _, manifest = self.fixture()
        patches = self.helper_patches()
        for context in patches:
            context.start()
            self.addCleanup(context.stop)
        result = campaign.contract(manifest, [0], 1)
        for name in ('base5_secondary_far_vehicle.py', 'base5_far_vehicle_native.py'):
            self.assertEqual(result['code'][name], safety.sha256(SCRIPTS / name))

    def test_legacy_campaign_pins_noop_validator_but_not_far_builder(self):
        _, _, _, _, manifest = self.fixture(planned=False)
        result = campaign.contract(manifest, [0], 1)
        self.assertNotIn('base5_secondary_far_vehicle.py', result['code'])
        self.assertEqual(result['code']['base5_far_vehicle_native.py'],
                         safety.sha256(SCRIPTS / 'base5_far_vehicle_native.py'))

    def test_optin_session_contract_pins_helper_and_native_auditor(self):
        _, _, _, _, manifest = self.fixture()
        network, sensor = self.write('network.json', {}), self.write('sensor.json', {})
        patches = self.helper_patches()
        for context in patches:
            context.start()
            self.addCleanup(context.stop)
        output = self.root / 'session'
        safety.prepare_session(output, manifest, network, sensor)
        result = json.loads((output / 'session_contract.json').read_text())
        for name in ('base5_secondary_far_vehicle.py', 'base5_far_vehicle_native.py'):
            self.assertEqual(result['safety_code_sha256'][name], safety.sha256(SCRIPTS / name))

    def test_legacy_session_pins_noop_validator_but_not_far_builder(self):
        _, _, _, _, manifest = self.fixture(planned=False)
        network, sensor = self.write('network.json', {}), self.write('sensor.json', {})
        output = self.root / 'session'
        safety.prepare_session(output, manifest, network, sensor)
        result = json.loads((output / 'session_contract.json').read_text())
        self.assertNotIn('base5_secondary_far_vehicle.py', result['safety_code_sha256'])
        self.assertEqual(result['safety_code_sha256']['base5_far_vehicle_native.py'],
                         safety.sha256(SCRIPTS / 'base5_far_vehicle_native.py'))

    def test_campaign_preflight_source_drift_cannot_become_an_ordinary_deficit(self):
        _, _, _, _, manifest = self.fixture()
        with patch.object(native, 'validate_manifest',
                          side_effect=far.FarContractError('original source changed')):
            with self.assertRaisesRegex(far.FarContractError, 'original source changed'):
                campaign.contract(manifest, [0], 1)

    def test_session_resume_revalidates_original_pins_before_returning_completed(self):
        _, _, _, _, manifest = self.fixture()
        network, sensor = self.write('network.json', {}), self.write('sensor.json', {})
        output = self.root / 'session'
        with patch.object(native, 'validate_manifest', return_value={}):
            safety.prepare_session(output, manifest, network, sensor)
        with patch.object(native, 'validate_manifest',
                          side_effect=far.FarContractError('original source changed')):
            with self.assertRaisesRegex(far.FarContractError, 'original source changed'):
                safety.prepare_session(output, manifest, network, sensor)

    def test_completed_session_revalidates_saved_far_metadata(self):
        original, scene, entry, _, manifest = self.fixture()
        network, sensor = self.write('network.json', {}), self.write('sensor.json', {})
        output = self.root / 'session'
        capture = self.root / 'capture'
        with patch.object(native, 'validate_manifest', return_value={}):
            safety.prepare_session(output, manifest, network, sensor)
        for name, value in (('scenario', scene), ('metadata', original),
                            ('audit', {}), ('telemetry', {}), ('collision_history', {})):
            self.write('capture/' + name + '.json', value)
        for index in range(1, 5):
            self.write(f'capture/CAMERA_{index}/frame_Intensity.png', {'nonempty': True})
        self.write('capture/LIDAR_5/frame_Intensity.bin', {'nonempty': True})
        self.write('capture/LIDAR_5/frame_Instance.txt', {})
        row = dict(id='slot', status='accepted_candidate', capture_root=str(capture),
                   scenario_sha256=safety.sha256(capture / 'scenario.json'))
        (output / 'ledger.jsonl').write_text(json.dumps(row) + '\n')
        with patch.object(native, 'validate_manifest', return_value={}), patch.object(
                native, 'verify_capture', side_effect=far.FarContractError(
                    'saved capture metadata changed')) as verify:
            with self.assertRaisesRegex(far.FarContractError, 'saved capture metadata changed'):
                safety.prepare_session(output, manifest, network, sensor)
        verify.assert_called_once_with(entry, capture)


class SavedCaptureFarContractTest(TemporaryFiles):
    def test_nonplanned_capture_remains_noop(self):
        original, _, entry, _, _ = self.fixture(planned=False)
        self.write('capture/metadata.json', original)
        with patch.object(far, 'verify_preserved') as verify:
            native.verify_capture(entry, self.root / 'capture')
        verify.assert_not_called()

    def test_actual_saved_metadata_and_payload_are_passed_to_guards(self):
        original, scene, entry, _, _ = self.fixture()
        actual = copy.deepcopy(original)
        actual['capture_runtime_ego'] = {'observed': True}
        self.write('capture/metadata.json', actual)
        self.write('capture/scenario.json', scene)
        frozen = self.write('frozen/scenario.json', {'example': 'frozen-v152'})
        plan = dict(source=dict(scenario=dict(path=str(frozen))))
        with patch.object(far, 'verify_preserved', return_value=plan) as preserved, patch.object(
                far, 'verify_payload') as payload:
            native.verify_capture(entry, self.root / 'capture')
        preserved.assert_called_once_with(original, actual)
        payload.assert_called_once_with(scene, {'example': 'frozen-v152'}, plan, actual)

    def test_changed_saved_metadata_is_not_masked_by_unchanged_planned_entry(self):
        original, _, entry, _, _ = self.fixture()
        actual = copy.deepcopy(original)
        actual['runtime_actors'][1]['name'] = 'changed-after-capture'
        self.write('capture/metadata.json', actual)
        with patch.object(far, 'verify_preserved', side_effect=far.FarContractError(
                'captured target identity changed')) as preserved:
            with self.assertRaisesRegex(far.FarContractError, 'captured target identity changed'):
                native.verify_capture(entry, self.root / 'capture')
        preserved.assert_called_once_with(original, actual)

    def test_far_capture_without_matching_source_entry_is_rejected(self):
        self.write('capture/metadata.json', metadata())
        with self.assertRaises(far.FarContractError):
            native.verify_capture(None, self.root / 'capture')

    def test_missing_expected_far_assignment_cannot_become_noop(self):
        _, _, entry, _, _ = self.fixture()
        self.write('capture/metadata.json', metadata(planned=False))
        with patch.object(far, 'verify_preserved', side_effect=far.FarContractError(
                'captured frozen spec changed')) as preserved:
            with self.assertRaisesRegex(far.FarContractError, 'captured frozen spec changed'):
                native.verify_capture(entry, self.root / 'capture')
        preserved.assert_called_once()


class ManifestSourceSlotBindingTest(TemporaryFiles):
    def binding_fixture(self):
        original, scene, entry, deck, _ = self.fixture()
        frozen_scene = self.write('v152/frozen.json', scene)
        frozen_meta = self.write('v152/frozen_metadata.json', metadata(planned=False))
        frozen_entry = dict(id='slot', scenario=str(frozen_scene),
            metadata=str(frozen_meta), scenario_sha256=safety.sha256(frozen_scene))
        frozen_manifest = self.write('v152/manifest.json', dict(scenes=[frozen_entry]))
        plan = copy.deepcopy(original['spec'][far.PLAN_KEY])
        plan.update(logical_slot_id='slot', source={
            'scenario': dict(path=str(frozen_scene), sha256=safety.sha256(frozen_scene)),
            'metadata': dict(path=str(frozen_meta), sha256=safety.sha256(frozen_meta))})
        original['spec'][far.PLAN_KEY] = plan
        self.write('source/metadata.json', original)
        deck['config'][far.CONFIG_KEY].update(
            source_manifest=str(frozen_manifest),
            source_manifest_sha256=safety.sha256(frozen_manifest))
        return deck, plan, frozen_entry, frozen_manifest

    def test_frozen_slot_binding_keeps_external_pins_and_payload_guard(self):
        deck, plan, _, source_manifest = self.binding_fixture()
        with patch.object(far, 'validate_plan', return_value=plan), patch.object(
                far, 'verify_preserved', return_value=plan), patch.object(
                far, 'verify_payload') as payload:
            pins = native.validate_manifest(deck)
        self.assertEqual(pins[str(source_manifest)], safety.sha256(source_manifest))
        for value in plan['source'].values():
            self.assertEqual(pins[value['path']], value['sha256'])
        payload.assert_called_once()
        self.assertEqual(len(payload.call_args.args), 4)

    def test_missing_slot_cross_entry_paths_and_source_sha_are_rejected(self):
        for mutation in ('missing_slot', 'scenario_path', 'metadata_path', 'source_sha'):
            with self.subTest(mutation=mutation):
                deck, plan, source_entry, source_manifest = self.binding_fixture()
                if mutation == 'missing_slot':
                    plan['logical_slot_id'] = 'NONEXISTENT_WRONG_QUOTA_SLOT'
                elif mutation in ('scenario_path', 'metadata_path'):
                    key = mutation[:-5]
                    content = json.loads(Path(plan['source'][key]['path']).read_text())
                    foreign = self.write('foreign/' + key + '.json', content)
                    plan['source'][key] = dict(path=str(foreign), sha256=safety.sha256(foreign))
                else:
                    source_entry['scenario_sha256'] = 'wrong-frozen-entry-sha'
                    self.write('v152/manifest.json', dict(scenes=[source_entry]))
                metadata_path = deck['scenes'][0]['metadata']
                current = json.loads(Path(metadata_path).read_text())
                current['spec'][far.PLAN_KEY] = plan
                Path(metadata_path).write_text(json.dumps(current))
                with patch.object(far, 'validate_plan', return_value=plan), patch.object(
                        far, 'verify_preserved', return_value=plan), patch.object(
                        far, 'verify_payload') as payload:
                    with self.assertRaisesRegex(RuntimeError, 'frozen logical slot'):
                        native.validate_manifest(deck)
                payload.assert_not_called()


if __name__ == '__main__':
    unittest.main()
