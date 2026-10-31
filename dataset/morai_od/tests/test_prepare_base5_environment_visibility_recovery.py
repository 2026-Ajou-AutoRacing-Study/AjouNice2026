import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_environment_visibility_recovery as recovery
from base5_capture_safety import atomic_json, sha256
from prepare_base5_layout_split import prepare as prepare_layout


def meta(index=0):
    return dict(spec=dict(family='obstacle', mission=False, role='front', motion='static',
            seed=42, distance_bounds=[10, 30], distance_bin=1, actors=2, yaw_limit=10,
            topology='route_general'), retry=0, split='train',
        scene_group_id='clip:'+str(index),
        ego=dict(x=float(index), y=0., z=0., yaw=0., link_idx='same-road'),
        population_budget=dict(actor_count=2, required_visible_actors=2, total_cost=6),
        runtime_actors=[dict(data_id=1+i, unique_id=8500+i, name='Box'+str(i),
            source_list='objectList', population_pool='obstacle', motion='static',
            position=dict(x=float(index)+12+i*4, y=0., z=0.), placement_yaw_deg=0.,
            estimated_size=[1., 1., 1.], planned_center_distance_m=12.+i*4,
            actual_ego_role='front') for i in range(2)])


def proxy(score=10., visible=2, primary_occlusion=0., extent=20., beams=2):
    return dict(score=score, predicted_visible_parents=visible, required_visible_parents=2,
        primary=dict(lidar_beam_proxy=beams, camera_min_extent_proxy_px=extent,
                     angular_occlusion_proxy=primary_occlusion), parents=[])


class EnvironmentVisibilityRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def builder(self, before, mutate=None):
        def build(spec, config, ctx, retry):
            after = copy.deepcopy(before)
            after['retry'] = retry
            after['ego']['x'] += retry*.01
            if mutate:
                mutate(after, retry)
            return dict(retry=retry), after
        return build

    def test_complete_identity_contract_rejects_roster_scale_split_range_phase_and_budget(self):
        before = meta()
        changes = [lambda m:m['spec'].update(seed=999), lambda m:m.update(split='val'),
            lambda m:m['runtime_actors'][1].update(data_id=999),
            lambda m:m['runtime_actors'][1].update(motion='walking'),
            lambda m:m['runtime_actors'][1].update(asset_scale=dict(profile_id='enlarged')),
            lambda m:m['runtime_actors'][0].update(planned_center_distance_m=30.),
            lambda m:m['runtime_actors'][0].update(actual_ego_role='rear'),
            lambda m:m['population_budget'].update(total_cost=7),
            lambda m:m['spec'].update(avoidance_phase='entry')]
        for change in changes:
            after = copy.deepcopy(before);after['retry'] = 1;change(after)
            with self.subTest(change=change), self.assertRaises(recovery.RecoveryContractError):
                recovery.verify_transition(before, after)

    def test_search_is_finite_and_selects_metadata_score_not_retry_order(self):
        before = meta();attempts = []
        def score(after, calibration, kind):
            self.assertEqual(kind, 'all_actor_visibility')
            attempts.append(after['retry'])
            return proxy(100 if after['retry'] == 17 else 1.)
        ranked, audit = recovery.search(before, {}, {}, {}, set(), 32,
            build=self.builder(before), scorer=score)
        self.assertEqual(attempts, list(range(1, 33)))
        self.assertEqual(len(audit['attempts']), 32)
        self.assertEqual(ranked[0][4]['retry'], 17)
        self.assertFalse(audit['selection_uses_new_native_outcomes'])
        self.assertTrue(audit['native_qa_unchanged'])

    def test_known_native_payload_and_changed_auxiliary_asset_are_not_ranked(self):
        before = meta()
        known = {hashlib.sha256(recovery.encoded_scene(dict(retry=2))).hexdigest()}
        def mutate(after, retry):
            if retry == 1:after['runtime_actors'][1]['data_id'] = 55
        ranked, audit = recovery.search(before, {}, {}, {}, known, 32,
            build=self.builder(before, mutate), scorer=lambda *a:proxy())
        self.assertEqual(audit['attempts'][0]['status'], 'contract_candidate_rejected')
        self.assertEqual(audit['attempts'][1]['status'], 'observed_or_source_payload_rejected')
        self.assertEqual(ranked[0][4]['retry'], 3)

    def test_unpromising_all_actor_or_primary_proxy_cannot_be_selected(self):
        before = meta()
        for bad in (proxy(visible=1), proxy(primary_occlusion=.9), proxy(extent=1., beams=0)):
            with self.subTest(bad=bad), self.assertRaisesRegex(recovery.RecoveryContractError, 'no same-contract plausible'):
                recovery.search(before, {}, {}, {}, set(), 32,
                    build=self.builder(before), scorer=lambda *a:bad)

    def test_only_32_or_64_attempts_and_nonfinite_score_fails_closed(self):
        before = meta()
        for count in (0, 1, 31, 65, 512, True):
            with self.subTest(count=count), self.assertRaises(recovery.RecoveryContractError):
                recovery.search(before, {}, {}, {}, set(), count)
        with self.assertRaisesRegex(recovery.RecoveryContractError, 'Nonfinite'):
            recovery.search(before, {}, {}, {}, set(), 32,
                build=self.builder(before), scorer=lambda *a:proxy(float('nan')))

    def test_generation_failures_spend_attempt_budget_and_do_not_trigger_extra_search(self):
        attempts = []
        def build(spec, config, ctx, retry):
            attempts.append(retry)
            raise ValueError('No road support')
        with self.assertRaises(recovery.RecoveryContractError):
            recovery.search(meta(), {}, {}, {}, set(), 64, build=build)
        self.assertEqual(attempts, list(range(1, 65)))

    def source(self):
        entries = []
        for index in range(30):
            ident = 'scene_'+str(index).zfill(4)
            scenario, metadata = self.root/'source'/(ident+'.json'), self.root/'source'/(ident+'_metadata.json')
            atomic_json(scenario, dict(index=index));atomic_json(metadata, meta(index))
            entries.append(dict(id=ident, split='train', scenario=str(scenario), metadata=str(metadata),
                scenario_sha256=sha256(scenario)))
        source = self.root/'source'/'manifest.json'
        atomic_json(source, dict(count=30, scenes=entries, config={}, provenance={}))
        result = prepare_layout(source, self.root/'layout')
        path = Path(result['manifest'])
        return path, recovery.read(path)

    def binding(self, source, deck, index, output='proposals'):
        entry = deck['scenes'][index];before = recovery.read(entry['metadata'])
        after = copy.deepcopy(before);after['retry'] = 1;after['ego']['x'] += .2
        scene = dict(retry=1, index=index)
        proof = dict(retry=1, scenario_sha256=hashlib.sha256(recovery.encoded_scene(scene)).hexdigest(), proxy=proxy())
        context = recovery.quality.replacement_contract(deck, entry, before)
        return recovery.bind_candidate(entry, before, [(1., -1, scene, after, proof)], deck,
            self.root/output/entry['id'], dict(reasons=['primary_no_sensor_evidence']), source, context, source)

    def test_actual_existing_overlay_binding_preserves_parent_and_never_claims_native(self):
        source, deck = self.source()
        watched = {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        selected = self.binding(source, deck, 5)
        candidate = recovery.read(selected['manifest']);after = recovery.read(selected['entry']['metadata'])
        self.assertEqual(candidate['count'], 1)
        self.assertFalse(candidate['native_qualified'])
        self.assertFalse(candidate['bulk_ready'])
        self.assertEqual(candidate['scenes'][0]['source_cell_id'], candidate['scenes'][0]['id'])
        self.assertEqual(after['fresh_completion_source']['source_cell_id'], deck['scenes'][5]['id'])
        self.assertEqual(after['quality_replacement_contract']['ordinal'], 1)
        base = recovery.layout.overlay_for_metadata(recovery.read(deck['scenes'][5]['metadata']))
        actual = recovery.layout.resolve(selected['overlay'], selected['entry']['id'], selected['entry'], after)
        expected = base['assignments'][deck['scenes'][5]['id']]
        self.assertEqual((actual['split'], actual['group_id']), (expected['split'], expected['group_id']))
        self.assertEqual({p:p.read_bytes() for p in watched}, watched)

    def test_two_independent_overlay_forks_receive_joint_virtual_not_native_audit(self):
        source, deck = self.source()
        selections = {deck['scenes'][i]['id']:self.binding(source, deck, i) for i in (5, 15)}
        _, rows = recovery.environment.records(source)
        base = recovery.layout.overlay_for_metadata(rows[0]['meta'])
        virtual, audit = recovery.joint_audit(rows, selections, base)
        self.assertEqual(len(virtual), 30)
        self.assertEqual(audit['changed_cells'], 2)
        self.assertEqual(audit['unchanged_cells'], 28)
        self.assertFalse(audit['common_capture_overlay_created'])
        self.assertFalse(audit['native_selected_ledger_created'])
        self.assertEqual(audit['errors'], [])
        broken = copy.deepcopy(base);broken['audit']['train_floors']['not-observed-feature'] = 1
        with self.assertRaisesRegex(recovery.RecoveryContractError, 'coverage floor failed'):
            recovery.joint_audit(rows, selections, broken)

    def test_binding_rejects_more_than_eight_layout_forks(self):
        source, deck = self.source();entry = deck['scenes'][5];before = recovery.read(entry['metadata'])
        after = copy.deepcopy(before);after['retry'] = 1;after['ego']['x'] += .2
        scene = dict(retry=1)
        proof = dict(retry=1, scenario_sha256=hashlib.sha256(recovery.encoded_scene(scene)).hexdigest(), proxy=proxy())
        context = recovery.quality.replacement_contract(deck, entry, before)
        with patch.object(recovery.quality, 'bind_replacement_contract',
                side_effect=recovery.quality.ReplacementGeometryRejected('layout geometry collision')) as bind:
            with self.assertRaisesRegex(recovery.RecoveryContractError, 'eight'):
                recovery.bind_candidate(entry, before, [(1., -1, scene, after, proof)]*10, deck,
                    self.root/'proposals', {}, source, context, source)
        self.assertEqual(bind.call_count, 8)

    def test_original_review_evidence_is_hashed_and_wrong_reasons_cannot_recover(self):
        source, deck = self.source();entries = {};metadata = {};targets = {};rows = []
        chunk = self.root/'native'/'chunk_1';capture = chunk/'captures'
        for index, reason in ((5, 'population_visible_actor_deficit'), (15, 'primary_no_sensor_evidence')):
            entry = deck['scenes'][index];ident = entry['id'];value = recovery.read(entry['metadata'])
            raw = capture/ident;raw.mkdir(parents=True)
            (raw/'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
            atomic_json(raw/'metadata.json', value)
            audit = dict(raw_complete=True, accepted_candidate=False, review_reasons=[reason],
                missing_uids=[], runtime_missing_uids=[], capture_safety=dict(
                    collision=dict(observed=True, no_collision_verified=True, non_ego_uids=[]),
                    support=dict(stationary_support_observed=True)))
            atomic_json(raw/'audit.json', audit)
            rows.append(dict(id=ident, status='review', scenario_sha256=entry['scenario_sha256'],
                capture_root=str(raw), audit=audit))
            entries[ident], metadata[ident] = entry, value
            targets[ident] = dict(scenario_sha256=entry['scenario_sha256'], reasons=[reason])
        ledger = chunk/'ledger.jsonl'
        ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        atomic_json(chunk/'shutdown_safety.json', dict(park_verified=True))
        # An unrelated safe ground review must remain outside this recovery,
        # rather than either blocking it or being silently declared approved.
        other = deck['scenes'][7];raw = capture/other['id'];raw.mkdir(parents=True)
        (raw/'scenario.json').write_bytes(Path(other['scenario']).read_bytes())
        other_audit = copy.deepcopy(rows[0]['audit'])
        other_audit['review_reasons'] = ['static_ground_severe_below']
        atomic_json(raw/'audit.json', other_audit)
        rows.append(dict(id=other['id'], status='review', scenario_sha256=other['scenario_sha256'],
            capture_root=str(raw), audit=other_audit))
        entries[other['id']] = other
        ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        with patch.object(recovery, 'TARGETS', targets), patch.object(recovery, 'EXPECTED_LEDGER_SHA', sha256(ledger)):
            failures, evidence = recovery.inspect_reviews(ledger, entries, metadata)
            self.assertEqual(set(failures), set(targets))
            self.assertIn(str(ledger), evidence)
            known, native_pins = recovery.observed_payloads(chunk.parent)
            self.assertEqual(known, {e['scenario_sha256'] for e in entries.values()})
            self.assertIn(str(ledger), native_pins)
            targets[rows[0]['id']]['reasons'] = ['ground_failure']
            with self.assertRaisesRegex(recovery.RecoveryContractError, 'visibility review differs'):
                recovery.inspect_reviews(ledger, entries, metadata)
        with patch.object(recovery, 'EXPECTED_LEDGER_SHA', '0'*64):
            with self.assertRaisesRegex(recovery.RecoveryContractError, 'review ledger changed'):
                recovery.inspect_reviews(ledger, entries, metadata)


if __name__ == '__main__':
    unittest.main()
