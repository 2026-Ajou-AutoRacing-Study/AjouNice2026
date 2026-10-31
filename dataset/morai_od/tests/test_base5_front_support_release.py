from argparse import Namespace
from contextlib import ExitStack
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json, sha256
import finalize_base5_front_support_release as release


class FrontReleaseTests(unittest.TestCase):
    def test_existing_release_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):
                release.freeze(Namespace(output=Path(temporary)))

    def completion_fixture(self, missing=True):
        source = [dict(id='s%02d' % i, metadata='/metadata/s%02d' % i) for i in range(64)]
        rows = [dict(source_cell_id=row['id'], status='accepted_candidate', audit={}) for row in source]
        baseline = copy.deepcopy(rows[:-1] if missing else rows)
        attempt = dict(id='s63', result=copy.deepcopy(rows[-1]))
        state = dict(stage='complete', baseline_selected=baseline, deficits=['s63'] if missing else [],
                     attempts=[attempt] if missing else [], contract=dict(source_sha256='manifest',
                         capture_contract={'current': True}, code_hashes={'script.py': 'pin'}))
        status = dict(stage='complete', accepted_candidate=64, replenished=int(missing), review=0, remaining_deficit=0)
        return dict(scenes=source), baseline, rows, state, status

    def verify_completion(self, fixture, official_result=None):
        deck, baseline, rows, state, status = fixture
        with ExitStack() as stack:
            stack.enter_context(patch.object(release, 'verify_native', return_value=status))
            stack.enter_context(patch.object(release, 'read', side_effect=lambda p: state if str(p).endswith('state.json') else {'original': str(p)}))
            stack.enter_context(patch.object(release, 'sha256', return_value='manifest'))
            stack.enter_context(patch.object(release.inputs, 'verify_hashes'))
            stack.enter_context(patch.object(release.completion.adaptive.fa.batch.campaign, 'contract', return_value={'current': True}))
            official = stack.enter_context(patch.object(release.completion, 'verify', side_effect=lambda attempt, **kwargs: attempt['result'] if official_result is None else official_result))
            missing = sorted({row['id'] for row in deck['scenes']} - {row['source_cell_id'] for row in rows})
            stack.enter_context(patch.object(release.completion, 'merged', return_value=(rows, missing)))
            stack.enter_context(patch.object(release, 'load_ledger', return_value=rows))
            result = release.verify_completion(Path('/completion'), Path('/manifest'), deck, baseline, {'pools': True})
            if len(state['attempts']) == 1:
                official.assert_called_once_with(state['attempts'][0], deck=deck,
                    source={'original': '/metadata/s63'}, pools={'pools': True})
            return result

    def test_real_parent_deficit_and_native_completion_qualifies(self):
        rows, state, qualifies = self.verify_completion(self.completion_fixture())
        self.assertEqual(len(rows), 64)
        self.assertTrue(qualifies)

    def test_zero_deficit_is_honest_nonqualification_not_fabricated_failure(self):
        _, state, qualifies = self.verify_completion(self.completion_fixture(missing=False))
        self.assertFalse(qualifies)
        self.assertEqual(state['attempts'], [])
        self.assertEqual(state['deficits'], [])

    def test_genuine_terminal63_retains_missing_slot_without_waiving_reviews(self):
        deck, _, full_rows, state, status = self.completion_fixture()
        rows, baseline = full_rows[:-1], full_rows[:-2]
        state.update(stage='complete_with_deficit', baseline_selected=baseline, deficits=['s62', 's63'],
                     attempts=[dict(id='s62', result=rows[-1])] + [
                         dict(id='s63', ordinal=i, result=dict(status='review', reasons=['primary_no_sensor_evidence']))
                         for i in range(1, 4)])
        status.update(stage='complete_with_deficit', accepted_candidate=63, replenished=1, review=1, remaining_deficit=1)
        observed, _, qualified = self.verify_completion((deck, baseline, rows, state, status))
        self.assertEqual(len(observed), 63)
        self.assertTrue(qualified)

    def test_completion_official_changed_result_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Official native'):
            self.verify_completion(self.completion_fixture(), {'status': 'review'})

    def test_parent_selection_and_replenished_count_cannot_be_faked(self):
        fixture = self.completion_fixture()
        fixture[3]['baseline_selected'] = []
        with self.assertRaisesRegex(ValueError, 'parent/completion identity'):
            self.verify_completion(fixture)
        fixture = self.completion_fixture()
        fixture[4]['replenished'] = 2
        with self.assertRaisesRegex(ValueError, 'replenishment count'):
            self.verify_completion(fixture)

    def test_different_completion_runtime_contract_is_rejected(self):
        fixture = self.completion_fixture()
        fixture[3]['contract']['capture_contract'] = {'old': True}
        with self.assertRaisesRegex(ValueError, 'another source/runtime'):
            self.verify_completion(fixture)

    def test_registered_front_rows_use_preservation_verifier(self):
        registration = dict(static=[dict(id='s%d' % i,
            qualification_role=dict(kind='front_selected' if i % 2 else 'front_legacy')) for i in range(38)])
        metadata = {row['id']: {'source': row['id']} for row in registration['static']}
        rows = [dict(source_cell_id=row['id'], capture_root='/raw/' + row['id']) for row in registration['static']]
        with patch.object(release, 'read', side_effect=lambda p: {'captured': str(p)}), \
                patch.object(release.adapter, 'verify_preserved', return_value=dict(data_id=10, selected=True)) as verifier:
            observed = release.verify_front_rows(registration, rows, metadata)
            self.assertEqual(len(observed), 38)
            self.assertEqual(verifier.call_count, 38)
            verifier.assert_any_call({'source': 's0'}, {'captured': '/raw/s0/metadata.json'})
            with self.assertRaisesRegex(ValueError, 'lacks native'):
                release.verify_front_rows(registration, rows[:-1], metadata)
            self.assertEqual(len(release.verify_front_rows(registration, rows[:-1], metadata, allowed_missing=['s37'])), 37)

    def probe_fixture(self, folder):
        root = Path(folder)
        manifest = root / 'manifest.json'
        atomic_json(manifest, {})
        strata, metadata, source, attempts = [], {}, [], []
        for selected, large in ((False, False), (False, True), (True, False), (True, True)):
            ident = 's%d' % len(source)
            mp = root / (ident + '_metadata.json')
            atomic_json(mp, {'source': ident})
            plan = dict(selected=selected, large_footprint=large, data_id=len(source))
            metadata[ident] = dict(spec=plan)
            stratum = dict(source_id=ident, selected=selected, large_footprint=large,
                           data_id=len(source), source_scenario_sha256=ident,
                           source_metadata_sha256=sha256(mp))
            strata.append(stratum)
            source.append(dict(id=ident, scenario_sha256=ident, metadata=str(mp)))
            attempts.append(dict(id=ident, source_id=ident, stratum=stratum,
                                 result=dict(status='accepted_candidate', capture_root=str(root / ident))))
        counts = dict(planned=672, selected=338, legacy=334, assets=19)
        inventory = dict(files={'/raw': 'pin'}, captured_rows=4, ledger_sha256map={'/ledger': 'pin'})
        state = dict(stage='complete', contract=dict(source_manifest_sha256=sha256(manifest),
                     runtime_hashes={'runtime.py': 'pin'}, strata=strata), attempts=attempts,
                     terminal_raw_inventory=inventory)
        state_path = root / 'state.json'
        atomic_json(state_path, state)
        proof = dict(schema='base5_front_recovery_probe_v1', passed=True,
            source_manifest=str(manifest), source_manifest_sha256=sha256(manifest),
            state_path=str(state_path), state_sha256=sha256(state_path), prepared_count=4, accepted_count=4,
            planned_probe=True, fabricated_parent_deficit=False, training_data=False, bulk_started=False,
            additional_completed_resume_raw=0, strata=strata, ledger_sha256map={'/ledger': 'pin'},
            source_inventory_sha256='inventory', full_deck_plan_counts=counts)
        path = root / 'proof.json'
        atomic_json(path, proof)
        return manifest, dict(scenes=source), metadata, state, proof, path

    def verify_probes(self, fixture, result_changed=False, contract_changed=False,
                      prepared_error=False, inventory_changed=False):
        manifest, deck, metadata, state, proof, path = fixture
        expected_contract = copy.deepcopy(state['contract'])
        if contract_changed:
            expected_contract['source_inventory_sha256'] = 'changed'
        inventory = copy.deepcopy(state['terminal_raw_inventory'])
        if inventory_changed:
            inventory['captured_rows'] = 5
        with patch.object(release.inputs, 'verify_hashes') as pinner, \
                patch.object(release.front, 'validate_plan', side_effect=lambda spec: spec), \
                patch.object(release.recovery, 'verify_attempt', side_effect=lambda a, *args: {'status': 'review'} if result_changed else a['result']) as official, \
                patch.object(release.recovery, 'source_snapshot', return_value=(deck, {}, metadata, 'inventory', proof['full_deck_plan_counts'])), \
                patch.object(release.recovery, 'contract', return_value=expected_contract), \
                patch.object(release.recovery, 'verify_prepared', side_effect=ValueError('prepared payload/order/path mismatch') if prepared_error else None) as prepared, \
                patch.object(release.recovery, 'raw_inventory', return_value=inventory), \
                patch.object(release.adapter, 'verify_preserved'), \
                patch.object(release, 'read', wraps=release.read) as reader:
            # Captured metadata is supplied only to the adapter; state/proof and
            # registered source files remain real immutable JSON fixtures.
            real_read = reader._mock_wraps
            reader.side_effect = lambda p: {'actual': True} if Path(p).parent.name in ('s0', 's1', 's2', 's3') else real_read(p)
            result = release.verify_probe_proof(path, manifest, deck, metadata, {'pools': True})
            self.assertEqual(official.call_count, 4)
            official.assert_any_call(state['attempts'][0], deck, metadata['s0'], {'pools': True})
            prepared.assert_called_once_with(path.parent, state, deck, metadata)
            self.assertTrue(all(Path(path).is_absolute() for path in pinner.call_args_list[0].args[0]))
            self.assertEqual(pinner.call_args_list[-1].args[0], proof['ledger_sha256map'])
            return result

    def test_four_probe_strata_official_verify_and_no_capture_resume(self):
        with tempfile.TemporaryDirectory() as temporary:
            proof, _ = self.verify_probes(self.probe_fixture(temporary))
            self.assertEqual(proof['accepted_count'], 4)
            self.assertFalse(proof['fabricated_parent_deficit'])

    def test_probe_success_or_resume_cannot_be_asserted_without_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self.probe_fixture(temporary)
            fixture[4]['additional_completed_resume_raw'] = 1
            atomic_json(fixture[5], fixture[4])
            with self.assertRaisesRegex(ValueError, 'Four real'):
                self.verify_probes(fixture)
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self.probe_fixture(temporary)
            with self.assertRaisesRegex(ValueError, 'official native'):
                self.verify_probes(fixture, result_changed=True)

    def test_probe_source_hash_or_state_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self.probe_fixture(temporary)
            fixture[3]['stage'] = 'halted'
            atomic_json(Path(temporary) / 'state.json', fixture[3])
            with self.assertRaisesRegex(ValueError, 'state checksum'):
                self.verify_probes(fixture)

    def test_probe_driver_contract_prepared_and_terminal_inventory_are_authoritative(self):
        for option, message in (('contract_changed', 'source/runtime/preparation'),
                                ('prepared_error', 'prepared payload/order/path'),
                                ('inventory_changed', 'terminal raw/ledger')):
            with self.subTest(option=option), tempfile.TemporaryDirectory() as temporary:
                with self.assertRaisesRegex(ValueError, message):
                    self.verify_probes(self.probe_fixture(temporary), **{option: True})

    def resume_fixture(self, root):
        baseline, completed = root / 'baseline', root / 'completed'
        pins = {}
        rows = [dict(source_cell_id='s%02d' % i) for i in range(17)]
        for folder in (baseline, completed):
            folder.mkdir()
            path = folder / 'selected_ledger.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            pins[str(path)] = sha256(path)
            atomic_json(folder / 'status.json', dict(stage='complete', elapsed_sec=3))
            (folder / 'raw.bin').write_bytes(b'raw')
        atomic_json(completed / 'state.json', {'stage': 'complete'})
        manifest = root / 'manifest.json'
        atomic_json(manifest, {'count': 64})

        def snapshot(folder):
            raw = folder / 'raw.bin'
            stat = raw.stat()
            return dict(raw_files={'raw.bin': dict(sha256=sha256(raw), inode=stat.st_ino,
                mtime_ns=stat.st_mtime_ns, size=stat.st_size)}, event_logs={},
                selected_sha256=sha256(folder / 'selected_ledger.jsonl'), cumulative_elapsed_sec=3.)

        before, after = root / 'partial_before.json', root / 'partial_after.json'
        atomic_json(before, dict(snapshot=snapshot(baseline), selected_rows=rows[:16]))
        atomic_json(after, dict(parent_root=str(baseline), source_manifest_sha256=sha256(manifest),
                               snapshot=snapshot(baseline), selected_rows=rows))
        partial = root / 'partial.json'
        atomic_json(partial, dict(passed=True, preserved_selected=16, prior_raw_unchanged=True,
            parent_root=str(baseline), source_manifest_sha256=sha256(manifest),
            before_snapshot_path=str(before), before_snapshot_sha256=sha256(before),
            after_snapshot_path=str(after), after_snapshot_sha256=sha256(after)))
        terminal_before, terminal_after = root / 'terminal_before.json', root / 'terminal_after.json'
        terminal = dict(source_manifest_sha256=sha256(manifest), parent_root=str(baseline),
                        completion_root=str(completed), parent=snapshot(baseline), completion=snapshot(completed))
        atomic_json(terminal_before, terminal)
        atomic_json(terminal_after, terminal)
        proof = root / 'resume.json'
        atomic_json(proof, dict(passed=True, additional_native_captures=0, ledger_sha256=pins,
            before_snapshot_path=str(terminal_before), before_snapshot_sha256=sha256(terminal_before),
            after_snapshot_path=str(terminal_after), after_snapshot_sha256=sha256(terminal_after),
            state_sha256={str(completed / 'state.json'): sha256(completed / 'state.json')},
            status_sha256={str(path): sha256(path) for path in (baseline / 'status.json', completed / 'status.json')}))
        return proof, partial, baseline, completed, manifest

    def test_terminal_resume_and_partial16_require_both_ledger_hashes(self):
        with tempfile.TemporaryDirectory() as temporary:
            proof, partial, baseline, completed, manifest = self.resume_fixture(Path(temporary))
            release.verify_resume(proof, partial, baseline, completed, manifest)
            (baseline / 'selected_ledger.jsonl').write_text('changed')
            with self.assertRaisesRegex(ValueError, 'changed'):
                release.verify_resume(proof, partial, baseline, completed, manifest)

    def test_partial_manifest_snapshot_linkage_and_raw_mutation_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.resume_fixture(Path(temporary))
            (args[2] / 'raw.bin').write_bytes(b'mutated')
            with self.assertRaisesRegex(ValueError, 'raw content'):
                release.verify_resume(*args)
        with tempfile.TemporaryDirectory() as temporary:
            args = self.resume_fixture(Path(temporary))
            partial = release.read(args[1])
            partial['source_manifest_sha256'] = 'foreign'
            atomic_json(args[1], partial)
            with self.assertRaisesRegex(ValueError, 'another parent/manifest'):
                release.verify_resume(*args)

    def test_zero_deficit_full_freeze_writes_blocked_release_without_bulk(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            names = ('manifest', 'resume_proof', 'partial_resume', 'preservation_proof')
            args = Namespace(**{name: root / (name + '.json') for name in names},
                registration=root / 'registration', static=root / 'static', moving=root / 'moving',
                completion=root / 'completion', front_probes=root / 'probes', output=root / 'release')
            deck = dict(config=dict(obstacle_support_policy={}, obstacle_front_support_policy={}, parked_npc_placement={}))
            source_audit = dict(status='OFFLINE_PASS', scenes=5000)
            preservation = dict(passed=True, errors=[], frames=5000, candidate_manifest_sha256='pin')
            state = dict(deficits=[])
            status = dict(replenishment_elapsed_sec=0, replenished=0)
            reg_file = args.registration / 'registration.json'
            probe_file = args.front_probes / 'proof.json'
            static_deck = dict(scenes=[dict(id='s%02d' % i) for i in range(64)])
            rows = [dict(source_cell_id=row['id']) for row in static_deck['scenes']]
            reads = {str(args.manifest.resolve()): deck, str(args.manifest.parent / 'scenario_audit.json'): source_audit,
                     str(args.preservation_proof): preservation, str(reg_file.parent / 'static64/manifest.json'): static_deck,
                     str(args.completion / 'status.json'): status}
            with ExitStack() as stack:
                stack.enter_context(patch.object(release, 'read', side_effect=lambda p: reads[str(p)]))
                stack.enter_context(patch.object(release, 'sha256', side_effect=lambda p: release.CANONICAL_SHA256 if p == release.CANONICAL else 'pin'))
                stack.enter_context(patch.object(release.registration_tool, 'validate_source', return_value=({}, {}, {})))
                stack.enter_context(patch.object(release.inputs, 'collect_runtime', return_value={'/runtime': 'pin'}))
                stack.enter_context(patch.object(release.inputs, 'collect_inputs', return_value={'/input': 'pin'}))
                stack.enter_context(patch.object(release.inputs, 'verify_hashes'))
                stack.enter_context(patch.object(release, 'verify_registration', return_value=({}, reg_file.parent / 'static64/manifest.json', reg_file.parent / 'animated8/manifest.json')))
                stack.enter_context(patch.object(release, 'verify_campaign', side_effect=[([], {'accepted_candidate': 64}), ([{}] * 8, {'accepted_candidate': 8})]))
                stack.enter_context(patch.object(release.completion.spatial.gen, 'context', return_value={}))
                stack.enter_context(patch.object(release.completion, 'pool_map', return_value={}))
                stack.enter_context(patch.object(release, 'verify_completion', return_value=(rows, state, False)))
                stack.enter_context(patch.object(release, 'verify_front_rows', return_value=[{}] * 38))
                stack.enter_context(patch.object(release, 'verify_repaired_cohort', return_value=24))
                stack.enter_context(patch.object(release, 'verify_probe_proof', return_value=({'state_path': str(root / 'probe_state.json')}, probe_file)))
                resume_documents = dict(before_snapshot_path=str(root / 'before.json'), after_snapshot_path=str(root / 'after.json'))
                stack.enter_context(patch.object(release, 'verify_resume', return_value=(resume_documents, resume_documents)))
                stack.enter_context(patch.object(release, 'cost_report', return_value=dict(extrapolated_hours_group_weighted=9, extrapolated_gib_including_qa_and_rejected_group_weighted=30)))
                stack.enter_context(patch.object(release, 'unique_storage', return_value={}))
                stack.enter_context(patch.object(release, 'registration_path', return_value=reg_file))
                stack.enter_context(patch.object(release, 'evidence_pins', return_value={'/evidence': 'pin'}))
                output = release.freeze(args)
            result = json.loads(output.read_text())
            self.assertFalse(result['bulk_ready'])
            self.assertFalse(result['completion_live_qualified'])
            self.assertFalse(result['bulk_started'])
            self.assertEqual(len(result['bulk_blockers']), 1)
            self.assertTrue(result['front_recovery_live_qualified'])
            self.assertEqual(result['schema'], 'base5_obstacle_support_release_v1')
            self.assertIn('/input', result['input_hashes'])
            self.assertIn('/evidence', result['input_hashes'])
            self.assertTrue(result['environments']['sunny13']['capture_output'].endswith('collection_5k_support_v151_sunny13'))


if __name__ == '__main__':
    unittest.main()
