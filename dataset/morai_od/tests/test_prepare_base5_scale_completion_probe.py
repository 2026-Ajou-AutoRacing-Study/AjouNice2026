"""Counterfactual probe contracts, CPU-only; no native failure ledger invented."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_scale_completion_probe as probe


def actor(name, data_id, ordinal, scaled=True):
    value = dict(name=name, data_id=data_id, motion='static')
    if scaled:
        value['asset_scale'] = dict(name=name, data_id=data_id, ordinal=ordinal,
            profile_id='iso125', profile_status='qualified', candidate_only=False)
    return value


def metadata(name, data_id, primary=True):
    actors = [actor(name, data_id, 0 if primary else 1)]
    if not primary:
        actors.insert(0, actor('Woman1', 3001, 0, scaled=False))
    row = dict(family='obstacle', mission=primary, seed=data_id,
               asset_scale_plan={'profile_ids': ['iso125'] if primary else ['original', 'iso125']})
    if primary:
        row['static_mission_extent'] = dict(stratum='before')
    return dict(spec=row, runtime_actors=actors, split='train', retry=2,
                scene_group_id='source_group', logical_quota_slot_id='source_slot')


class CompletionProbeTests(unittest.TestCase):
    def setUp(self):
        self.metadata = dict(primary=metadata('CargoBox', 1),
            auxiliary=metadata('WoodBox', 2, primary=False),
            alternate=metadata('PE_Drum', 3, primary=False))

    def test_fixed_two_slots_use_different_assets_without_metadata_mutation(self):
        before = copy.deepcopy(self.metadata)
        rows = probe.select(self.metadata)
        self.assertEqual(tuple(row['slot'] for row in rows), probe.SLOTS)
        self.assertEqual(rows[0]['ordinal'], 0)
        self.assertGreater(rows[1]['ordinal'], 0)
        self.assertNotEqual(rows[0]['data_id'], rows[1]['data_id'])
        self.assertEqual(self.metadata, before)

    def test_order_and_native_outcome_annotations_do_not_select_sources(self):
        expected = probe.select(self.metadata)
        reversed_source = dict(reversed(list(self.metadata.items())))
        for item in reversed_source.values():
            item['unrelated_native_outcome'] = 'review'
        self.assertEqual(probe.select(reversed_source), expected)

    def test_missing_primary_or_distinct_auxiliary_is_fatal(self):
        with self.assertRaisesRegex(probe.ProbeContractError, 'primary'):
            probe.select({key: value for key, value in self.metadata.items() if key != 'primary'})
        source = dict(primary=self.metadata['primary'], auxiliary=metadata('CargoBox', 1, primary=False))
        with self.assertRaisesRegex(probe.ProbeContractError, 'distinct-asset'):
            probe.select(source)

    def test_moving_and_shadow_sources_cannot_substitute_auxiliary_multi(self):
        for change in ('moving', 'shadow', 'mission'):
            source = dict(primary=self.metadata['primary'], auxiliary=copy.deepcopy(self.metadata['auxiliary']))
            if change == 'moving':
                source['auxiliary']['runtime_actors'][0]['motion'] = 'walking'
            elif change == 'shadow':
                source['auxiliary']['spec']['family'] = 'shadow'
            else:
                source['auxiliary']['spec']['mission'] = True
            with self.subTest(change=change), self.assertRaises(probe.ProbeContractError):
                probe.select(source)

    def test_candidate_profile_or_foreign_scale_identity_is_not_a_probe(self):
        for field, value in (('candidate_only', True), ('profile_status', 'unqualified'),
                             ('ordinal', 2), ('name', 'Foreign'), ('data_id', 99)):
            source = copy.deepcopy(self.metadata)
            source['primary']['runtime_actors'][0]['asset_scale'][field] = value
            with self.subTest(field=field), self.assertRaises(probe.ProbeContractError):
                probe.select(source)

    def candidate(self):
        scene = dict(new_geometry=True)
        actual = copy.deepcopy(self.metadata['primary'])
        actual['retry'] = 4
        actual['flexible_completion'] = dict(attempt=1, maximum_proposals=512,
            ranked_candidate_cap=8, proposals_tried=10, valid_candidates=8,
            actor_population_changed=False, scale_profile_fallback=False)
        return scene, actual

    def test_proposal_uses_real_adapter_dependencies_fixed_ordinal_and_explicit_nonfailure(self):
        entry = dict(id='primary', scenario_sha256='old')
        source = self.metadata['primary']
        original = copy.deepcopy(source)
        config, ctx, calibration, known = {}, {}, {}, {'old'}
        scene, actual = self.candidate()
        slot = probe.select(self.metadata)[0]
        with patch.object(probe.adapter, 'choose', return_value=(scene, actual)) as choose, \
                patch.object(probe.adapter, 'verify_preserved') as verify:
            _, result = probe.propose(entry, source, config, ctx, calibration, known, slot)
        args = choose.call_args.args
        self.assertEqual(args[:7], (source, config, ctx, calibration, 1, 'primary', known))
        self.assertIs(args[7], probe.spatial.build)
        self.assertIs(args[8], probe.fresh.frozen_proposal_score)
        self.assertIs(args[9], probe.fresh.adaptive.fa.novelty)
        verify.assert_called_once_with(source, result, config)
        self.assertEqual(source, original)
        marker = result['scale_completion_probe']
        self.assertFalse(marker['source_native_failure'])
        self.assertTrue(marker['counterfactual_recovery_probe_only'])
        self.assertFalse(marker['native_outcomes_used_for_selection'])
        self.assertTrue(marker['native_qa_unchanged'])
        self.assertEqual(marker['stored_completion_ordinal'], 1)
        self.assertEqual(result['scene_group_id'], source['scene_group_id'])

    def test_frozen_contract_errors_propagate_without_donor_or_retry_fallback(self):
        with patch.object(probe.adapter, 'choose', side_effect=probe.scale.ScaleContractError('pin changed')) as choose:
            with self.assertRaisesRegex(probe.scale.ScaleContractError, 'pin changed'):
                probe.propose(dict(id='primary'), self.metadata['primary'], {}, {}, {}, set(), {})
            self.assertEqual(choose.call_count, 1)

    def test_repeated_payload_or_relaxed_adapter_budget_is_rejected(self):
        for change in ('known', 'ordinal', 'budget', 'rank', 'actors', 'profile'):
            scene, actual = self.candidate()
            proof = actual['flexible_completion']
            known = set()
            if change == 'known':
                known.add(probe.payload_sha(scene))
            elif change == 'ordinal':
                proof['attempt'] = 2
            elif change == 'budget':
                proof['maximum_proposals'] = 1024
            elif change == 'rank':
                proof['ranked_candidate_cap'] = 16
            elif change == 'actors':
                proof['actor_population_changed'] = True
            else:
                proof['scale_profile_fallback'] = True
            with self.subTest(change=change), patch.object(probe.adapter, 'choose', return_value=(scene, actual)), \
                    patch.object(probe.adapter, 'verify_preserved'):
                with self.assertRaises(probe.ProbeContractError):
                    probe.propose(dict(id='primary', scenario_sha256='old'), self.metadata['primary'],
                                  {}, {}, {}, known, probe.select(self.metadata)[0])

    def test_source_evidence_and_policy_are_checked_at_each_explicit_boundary(self):
        mappings = ({'source': 'a'}, {'audit': 'b'}, {'runtime': 'c'})
        policy = object()
        with patch.object(probe.pins, 'verify_hashes') as hashes, \
                patch.object(probe.scale, 'verify_frozen_policy') as verify:
            probe.verify_boundaries(*mappings, policy)
        self.assertEqual([call.args[0] for call in hashes.call_args_list], list(mappings))
        verify.assert_called_once_with(policy)

    def test_preexisting_and_source_nested_outputs_are_rejected_before_input_reads(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaises(FileExistsError):
                probe.prepare(root / 'missing.json', root)
            with self.assertRaisesRegex(probe.ProbeContractError, 'source or MORAI'):
                probe.prepare(root / 'missing.json', root / 'nested')


if __name__ == '__main__':
    unittest.main()
