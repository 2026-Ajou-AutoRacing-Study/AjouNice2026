"""CPU-only nonrepeat boundaries for explicitly allocated far Vehicle slots.

Native children are replaced by deterministic temporary-file fixtures. No ROS,
MORAI process, sensor capture or placement context is launched by these tests.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_secondary_far_vehicle as far
import base5_far_vehicle_native as native
import run_base5_capture_campaign as campaign
import run_base5_fresh_completion as fresh
import run_base5_quality_loop as quality


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


class NonrepeatFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(encoded(value))
        return path

    def fixture(self, planned=True, retry=0):
        spec = dict(family='highway', mission=False, role='front',
                    distance_bounds=[10, 20], seed=11)
        if planned:
            spec[far.PLAN_KEY] = dict(target_ordinal=1,
                                      distance_bounds_m=[80, 90], role='rear')
        meta = dict(spec=spec, split='train', scene_group_id='frozen-group',
            retry=retry, runtime_actors=[
                dict(data_id=30100001, unique_id=8500, name='primary',
                     geometric_center_estimate=[15., 0.], motion='static'),
                dict(data_id=30100003, unique_id=8501, name='secondary',
                     geometric_center_estimate=[-85., 0.], motion='static')])
        scene = dict(example='source')
        sp = self.write('source/scene.json', scene)
        mp = self.write('source/metadata.json', meta)
        entry = dict(id='slot', scenario=str(sp), metadata=str(mp),
            scenario_sha256=digest(scene), split=meta['split'],
            scene_group_id=meta['scene_group_id'], family='highway', mission=False,
            role='front', distance_bounds=[10, 20], motion='static', subfamily=None)
        config = dict(environment_tag='sunny13', fresh_geometry_policy='test')
        if planned:
            config[far.CONFIG_KEY] = dict(policy=far.POLICY)
        deck = dict(config=config, scenes=[entry], count=1)
        manifest = self.write('manifest.json', deck)
        return meta, scene, deck, manifest

    def proposal(self, original, name, shift=3.):
        meta = copy.deepcopy(original)
        meta['runtime_actors'][1]['geometric_center_estimate'][0] += shift
        return dict(example=name), meta

    def session(self, name, payloads):
        """Retain both accepted and reviewed raw scenes in one native ledger."""
        root = self.root / name
        root.mkdir(parents=True, exist_ok=True)
        rows = []
        for index, (scene, status) in enumerate(payloads):
            raw = root / 'captures' / str(index)
            raw.mkdir(parents=True)
            (raw / 'scenario.json').write_bytes(encoded(scene))
            rows.append(dict(id='native-' + str(index), capture_root=str(raw),
                             scenario_sha256=digest(scene), status=status))
        (root / 'ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
        return root, rows


class ReplacementNonrepeatTest(NonrepeatFiles):
    def test_inherited_failed_payload_is_skipped_without_mutating_known_set(self):
        original, _, _, manifest = self.fixture()
        repeated = self.proposal(original, 'previous-failed', 4.)
        novel = self.proposal(original, 'novel', 5.)
        known = {digest(repeated[0])}
        before = set(known)
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=[repeated, novel]) as build, patch.object(
                far, 'verify_preserved'):
            selected = quality.replacement(manifest, self.root / 'replacement',
                                           far_known_payloads=known)
        self.assertEqual(build.call_count, 2)
        self.assertEqual(known, before)
        entry = json.loads(selected.read_text())['scenes'][0]
        self.assertEqual(json.loads(Path(entry['scenario']).read_text()), novel[0])
        provenance = json.loads(Path(entry['metadata']).read_text())[
            'secondary_far_vehicle_replacement']
        self.assertEqual(set(provenance['avoided_payload_sha256']),
                         known | {digest(dict(example='source'))})
        self.assertEqual(provenance['offline_proposals'], 2)

    def test_retry_blocks_use_disjoint_bounded_intervals(self):
        original, _, _, manifest = self.fixture(retry=7)
        for block in (0, 1, 15):
            with self.subTest(block=block), patch.object(
                    quality.gen, 'context', return_value={}), patch.object(
                    quality.gen, 'build', side_effect=far.FarGeometryExhausted(
                        'no legal novel pose')) as build:
                with self.assertRaises(quality.PlacementProposalExhausted):
                    quality.replacement(manifest, self.root / ('out-' + str(block)),
                                        far_retry_block=block)
            retries = [call.args[3] for call in build.call_args_list]
            self.assertEqual(retries, list(range(8 + 4096 * block,
                                                 8 + 4096 * block + 512)))
        self.assertEqual(original['retry'], 7)

    def test_invalid_retry_block_fails_before_context_or_writes(self):
        _, _, _, manifest = self.fixture()
        for block in (True, False, 1., -1, 16, '1', None):
            with self.subTest(block=block), patch.object(quality.gen, 'context') as ctx:
                with self.assertRaisesRegex(ValueError, 'integer in 0..15'):
                    quality.replacement(manifest, self.root / 'out', far_retry_block=block)
                ctx.assert_not_called()
        self.assertFalse((self.root / 'out').exists())

    def test_all_known_candidates_exhaust_without_output_or_source_mutation(self):
        original, _, _, manifest = self.fixture()
        repeated = self.proposal(original, 'previous-failed', 4.)
        source_before = {path: path.read_bytes() for path in (self.root / 'source').iterdir()}
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', return_value=repeated) as build, patch.object(
                far, 'verify_preserved'):
            with self.assertRaises(quality.PlacementProposalExhausted):
                quality.replacement(manifest, self.root / 'out',
                                    far_known_payloads={digest(repeated[0])})
        self.assertEqual(build.call_count, 512)
        self.assertFalse((self.root / 'out').exists())
        self.assertEqual({path: path.read_bytes() for path in source_before}, source_before)

    def test_novel_bytes_still_require_quarter_meter_target_displacement(self):
        original, _, _, manifest = self.fixture()
        tiny = self.proposal(original, 'novel-bytes-tiny-pose', .249)
        novel = self.proposal(original, 'novel-pose', .25)
        with patch.object(quality.gen, 'context', return_value={}), patch.object(
                quality.gen, 'build', side_effect=[tiny, novel]) as build, patch.object(
                far, 'verify_preserved'):
            result = quality.replacement(manifest, self.root / 'out')
        self.assertEqual(build.call_count, 2)
        self.assertEqual(json.loads(Path(json.loads(result.read_text())['scenes'][0][
            'scenario']).read_text()), novel[0])

    def test_unplanned_legacy_deck_rejects_nonempty_far_options(self):
        _, _, _, manifest = self.fixture(planned=False)
        for kwargs in (dict(far_known_payloads={'old'}), dict(far_retry_block=1)):
            with self.subTest(kwargs=kwargs), patch.object(quality.gen, 'context') as ctx:
                with self.assertRaisesRegex(ValueError, 'explicit far plan'):
                    quality.replacement(manifest, self.root / 'out', **kwargs)
                ctx.assert_not_called()


class CapturedPayloadsTest(NonrepeatFiles):
    def test_reads_accepted_and_rejected_sessions_once_without_changing_raw(self):
        first, rows = self.session('first', [(dict(pose='a'), 'accepted_candidate'),
                                             (dict(pose='b'), 'review')])
        second, other = self.session('second', [(dict(pose='b'), 'review')])
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        result = quality.captured_payloads([dict(capture_root=str(first)),
                                           dict(capture_root=str(second))])
        self.assertEqual(result, {row['scenario_sha256'] for row in rows + other})
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()})

    def test_non_native_reaudit_and_no_capture_attempts_are_ignored(self):
        self.assertEqual(quality.captured_payloads([
            dict(kind='static_rigid_raw_reaudit', capture_root=str(self.root / 'absent')),
            dict(kind='ground_preparation_unavailable', error='no scene loaded')]), set())

    def test_missing_authoritative_ledger_is_fatal(self):
        with self.assertRaisesRegex(ValueError, 'Native attempt ledger missing'):
            quality.captured_payloads([dict(capture_root=str(self.root / 'absent'))])

    def test_changed_reviewed_raw_payload_is_fatal(self):
        session, rows = self.session('failed', [(dict(pose='old'), 'review')])
        Path(rows[0]['capture_root'], 'scenario.json').write_bytes(encoded(dict(pose='changed')))
        with self.assertRaisesRegex(ValueError, 'Native attempted payload changed'):
            quality.captured_payloads([dict(capture_root=str(session))])

    def test_torn_or_corrupt_ledger_is_rejected_without_repair_or_backup(self):
        session = self.root / 'session'
        session.mkdir()
        ledger = session / 'ledger.jsonl'
        for payload in (b'{"scenario_sha256":"old"}',
                        b'{"scenario_sha256":"old"}\ntruncated\n', b'\n'):
            with self.subTest(payload=payload):
                ledger.write_bytes(payload)
                before = {p: p.read_bytes() for p in session.rglob('*') if p.is_file()}
                with self.assertRaises(ValueError):
                    quality.captured_payloads([dict(capture_root=str(session))])
                self.assertEqual(before, {p: p.read_bytes() for p in session.rglob('*')
                                          if p.is_file()})


class QualityLoopNonrepeatTest(NonrepeatFiles):
    def fake_capture(self, scene, status='review'):
        """Mock Popen creates a valid local save; never starts a child process."""
        def run(command, **unused):
            root = Path(command[command.index('--output') + 1])
            root.mkdir(parents=True, exist_ok=True)
            raw = root / 'captures' / 'native'
            raw.mkdir(parents=True)
            (raw / 'scenario.json').write_bytes(encoded(scene))
            row = dict(id='slot', status=status, capture_root=str(raw),
                scenario_sha256=digest(scene), audit=dict(review_reasons=[
                    'far_vehicle_no_current_input_sensor_evidence'] if status == 'review' else []))
            (root / 'ledger.jsonl').write_text(json.dumps(row) + '\n')
            (root / 'shutdown_safety.json').write_text(json.dumps(dict(park_verified=True)))
            return Mock(returncode=0, poll=Mock(return_value=0))
        return run

    def test_inner_replacement_merges_parent_known_with_current_native_attempt(self):
        _, scene, _, manifest = self.fixture()
        inherited = {'previous-parent-failure'}
        with patch.object(quality.subprocess, 'Popen', side_effect=self.fake_capture(scene)) as child, patch.object(
                quality, 'lightweight_static', return_value=True), patch.object(
                quality.motion_ground, 'moving_actors', return_value=[]), patch.object(
                quality, 'replacement', side_effect=quality.PlacementProposalExhausted(
                    '512 bounded candidates unavailable')) as replacement:
            code = quality.execute(manifest, [0], self.root / 'quality',
                                   far_known_payloads=inherited)
        self.assertEqual(code, 0)
        self.assertEqual(child.call_count, 1)
        self.assertEqual(replacement.call_args.kwargs['far_known_payloads'],
                         inherited | {digest(scene)})
        result = json.loads((self.root / 'quality/report.json').read_text())
        self.assertEqual(result['stage'], 'complete')
        self.assertEqual(result['selected'], [])
        self.assertEqual(result['unresolved'][0]['static_quality_budget_reason'],
                         'far_secondary_offline_proposals_exhausted')
        self.assertEqual(inherited, {'previous-parent-failure'})

    def test_unplanned_inner_replacement_does_not_receive_far_kwargs(self):
        _, scene, _, manifest = self.fixture(planned=False)
        with patch.object(quality.subprocess, 'Popen', side_effect=self.fake_capture(scene)), patch.object(
                quality, 'lightweight_static', return_value=True), patch.object(
                quality.motion_ground, 'moving_actors', return_value=[]), patch.object(
                quality, 'replacement', side_effect=ValueError('ordinary test stop')) as replacement:
            code = quality.execute(manifest, [0], self.root / 'quality')
        self.assertEqual(code, 1)
        self.assertEqual(replacement.call_args.kwargs, {'require_ground_observable': False})


class CampaignNonrepeatTest(NonrepeatFiles):
    def run_campaign(self, planned=True, resume_round=False):
        _, _, deck, source = self.fixture(planned=planned)
        output = self.root / 'campaign'
        captured = []
        source_scene = dict(example='failed-round0')
        session, _ = self.session('native-round0', [(source_scene, 'review')])
        rejected = dict(stage='complete', selected=[],
            attempts=[dict(kind='initial', capture_root=str(session),
                           shutdown=dict(park_verified=True))],
            unresolved=[dict(reasons=['far_vehicle_no_current_input_sensor_evidence'])])
        accepted = dict(stage='complete', attempts=[], unresolved=[], selected=[
            dict(id='accepted', status='accepted_candidate', audit=dict(review_reasons=[]))])

        def fake_execute(manifest, indices, target, **kwargs):
            captured.append(dict(target=target.name, kwargs={key: set(value)
                if key == 'far_known_payloads' else value for key, value in kwargs.items()}))
            target.mkdir(parents=True, exist_ok=True)
            value = rejected if target.name == 'round_0' else accepted
            (target / 'report.json').write_bytes(encoded(value))
            return 0

        def fake_replacement(manifest, folder, **kwargs):
            captured.append(dict(replacement=True, kwargs={key: set(value)
                if key == 'far_known_payloads' else value for key, value in kwargs.items()}))
            folder.mkdir(parents=True)
            path = folder / 'manifest.json'
            path.write_bytes(encoded(deck))
            return path

        if resume_round:
            self.write('campaign/slot/round_0/report.json', rejected)
            self.write('campaign/status.json', dict(stage='paused', elapsed_sec=1.))
        with patch.object(campaign, 'contract', return_value={'test': 'fixed'}), patch.object(
                campaign, 'validate_selected'), patch.object(
                campaign.loop, 'execute', side_effect=fake_execute), patch.object(
                campaign.loop, 'replacement', side_effect=fake_replacement):
            code = campaign.execute(source, [0], output, resume=resume_round, rounds=2)
        return code, captured, digest(source_scene)

    def test_outer_round_inherits_all_previous_native_payloads_and_distinct_retry_block(self):
        code, calls, previous = self.run_campaign()
        self.assertEqual(code, 0)
        self.assertEqual(calls[0]['kwargs']['far_known_payloads'], set())
        self.assertEqual(calls[1]['kwargs'], dict(far_known_payloads={previous}, far_retry_block=1))
        self.assertEqual(calls[2]['kwargs']['far_known_payloads'], {previous})

    def test_resumed_round_rebuilds_known_set_without_repeating_native_capture(self):
        code, calls, previous = self.run_campaign(resume_round=True)
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), 2)
        self.assertTrue(calls[0]['replacement'])
        self.assertEqual(calls[0]['kwargs'], dict(far_known_payloads={previous}, far_retry_block=1))
        self.assertEqual(calls[1]['target'], 'round_1')
        self.assertEqual(calls[1]['kwargs']['far_known_payloads'], {previous})

    def test_legacy_campaign_passes_no_new_far_options(self):
        code, calls, _ = self.run_campaign(planned=False)
        self.assertEqual(code, 0)
        self.assertEqual(calls[1]['kwargs'], {})
        for call in calls:
            self.assertNotIn('far_known_payloads', call['kwargs'])
            self.assertNotIn('far_retry_block', call['kwargs'])

    def test_outer_offline_exhaustion_preserves_deficit_without_a_second_capture(self):
        _, _, _, source = self.fixture()
        output = self.root / 'campaign'
        session, _ = self.session('native', [(dict(pose='failed'), 'review')])
        rejected = dict(stage='complete', selected=[],
            attempts=[dict(kind='initial', capture_root=str(session),
                           shutdown=dict(park_verified=True))],
            unresolved=[dict(reasons=['far_vehicle_no_current_input_sensor_evidence'])])

        def fake_execute(manifest, indices, target, **kwargs):
            target.mkdir(parents=True)
            (target / 'report.json').write_bytes(encoded(rejected))
            return 0

        with patch.object(campaign, 'contract', return_value={'test': 'fixed'}), patch.object(
                campaign, 'validate_selected'), patch.object(
                campaign.loop, 'execute', side_effect=fake_execute) as execute, patch.object(
                campaign.loop, 'replacement', side_effect=quality.PlacementProposalExhausted(
                    'bounded far proposals unavailable')):
            code = campaign.execute(source, [0], output, rounds=2)
        self.assertEqual(code, 0)
        self.assertEqual(execute.call_count, 1)
        self.assertEqual(json.loads((output / 'quota.json').read_text())['deficit'], 1)
        self.assertEqual(json.loads((output / 'status.json').read_text())['stage'],
                         'complete_with_deficit')
        proof = json.loads((output / 'slot/round_1/report.json').read_text())
        self.assertEqual(proof['attempts'], rejected['attempts'])
        self.assertTrue(proof['offline_proposal_failure']['no_new_capture'])


class FreshParentNonrepeatTest(NonrepeatFiles):
    def parent_fixture(self):
        deck = dict(config={far.CONFIG_KEY: dict(policy=far.POLICY)},
                    scenes=[dict(id='a'), dict(id='b')])
        parent = self.root / 'parent'
        self.write('parent/batch_contract.json', dict(indices=[0, 1], chunk_size=1))
        initial0, _ = self.session('parent/chunk_0000/capture', [(dict(pose='initial-a'), 'review')])
        initial1, _ = self.session('parent/chunk_0001/capture', [(dict(pose='initial-b'), 'accepted_candidate')])
        recovery, _ = self.session('parent/chunk_0000/retained-recovery', [(dict(pose='recovery-a'), 'review')])
        self.write('parent/chunk_0000/recovery/a/a/round_0/report.json', dict(
            stage='complete', selected=[], attempts=[dict(capture_root=str(recovery))]))
        self.session('parent/orphan', [(dict(pose='must-not-count'), 'review')])
        return deck, parent, {digest(dict(pose=name)) for name in
                              ('initial-a', 'initial-b', 'recovery-a')}

    def test_registered_parent_initial_and_recovery_payloads_are_retained_not_orphans(self):
        deck, parent, expected = self.parent_fixture()
        before = {p: p.read_bytes() for p in parent.rglob('*') if p.is_file()}
        self.assertEqual(fresh.far_parent_payloads(deck, parent), expected)
        self.assertEqual(before, {p: p.read_bytes() for p in parent.rglob('*') if p.is_file()})

    def test_legacy_deck_does_not_scan_parent(self):
        self.assertEqual(fresh.far_parent_payloads(dict(config={}), self.root / 'absent'), set())

    def test_imported_parent_is_not_accepted_as_native_failure_proof(self):
        deck, parent, _ = self.parent_fixture()
        self.write('parent/imported_prefix.json', {})
        with self.assertRaisesRegex(ValueError, 'non-imported fresh parent'):
            fresh.far_parent_payloads(deck, parent)

    def test_incomplete_registered_recovery_is_fatal(self):
        deck, parent, _ = self.parent_fixture()
        path = parent / 'chunk_0000/recovery/a/a/round_0/report.json'
        row = json.loads(path.read_text())
        row['stage'] = 'halted'
        path.write_bytes(encoded(row))
        with self.assertRaisesRegex(ValueError, 'recovery report incomplete'):
            fresh.far_parent_payloads(deck, parent)

    def test_changed_failed_parent_raw_is_fatal(self):
        deck, parent, _ = self.parent_fixture()
        path = parent / 'chunk_0000/capture/captures/0/scenario.json'
        path.write_bytes(encoded(dict(pose='tampered')))
        with self.assertRaisesRegex(ValueError, 'Native attempted payload changed'):
            fresh.far_parent_payloads(deck, parent)


class SavedFarCaptureBindingTest(unittest.TestCase):
    def setUp(self):
        # Reuse the real source/serializer fixture, not a mocked preservation
        # verifier. The fixture isolates only map pose candidates/terrain lookup.
        import importlib.util
        path = Path(__file__).with_name('test_base5_secondary_far_vehicle.py')
        spec = importlib.util.spec_from_file_location('_far_source_fixture', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.fixture = module.FarVehicleTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def saved_capture(self, expected_x=70., actual_x=70.):
        expected_scene, expected_meta = self.fixture.built(x=expected_x)
        actual_scene, actual_meta = self.fixture.built(x=actual_x)
        root = self.fixture.root
        scene_path, meta_path = root / 'planned.json', root / 'planned_meta.json'
        scene_path.write_bytes(encoded(expected_scene))
        meta_path.write_bytes(encoded(expected_meta))
        capture = root / 'captured'
        capture.mkdir()
        (capture / 'scenario.json').write_bytes(encoded(actual_scene))
        # Saved metadata may include legitimate live diagnostic additions.
        actual_meta['capture_runtime_ego'] = dict(observed=True)
        (capture / 'metadata.json').write_bytes(encoded(actual_meta))
        entry = dict(id='slot', scenario=str(scene_path), metadata=str(meta_path),
                     scenario_sha256=digest(expected_scene))
        return entry, capture

    def test_exact_saved_payload_with_live_metadata_passes_real_source_guards(self):
        entry, capture = self.saved_capture()
        native.verify_capture(entry, capture)

    def test_coherent_other_legal_pose_cannot_replace_the_pinned_captured_payload(self):
        entry, capture = self.saved_capture(expected_x=70., actual_x=75.)
        # The alternate pose satisfies the same source/plan identity and legal
        # band. Its internal validity must not substitute for entry binding.
        actual = json.loads((capture / 'metadata.json').read_text())
        expected = json.loads(Path(entry['metadata']).read_text())
        plan = far.verify_preserved(expected, actual)
        far.verify_payload(json.loads((capture / 'scenario.json').read_text()),
                           self.fixture.scene, plan, actual)
        with self.assertRaisesRegex(RuntimeError, 'differs from its pinned source entry'):
            native.verify_capture(entry, capture)

    def test_far_capture_requires_a_pinned_entry_including_its_payload_hash(self):
        entry, capture = self.saved_capture()
        for supplied in (None, {key: value for key, value in entry.items()
                                if key != 'scenario_sha256'}):
            with self.subTest(supplied=supplied):
                with self.assertRaisesRegex(RuntimeError, 'lacks a pinned source entry'):
                    native.verify_capture(supplied, capture)


if __name__ == '__main__':
    unittest.main()
