import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json, sha256
import finalize_base5_primary_range_release as release


class PrimaryReleaseTests(unittest.TestCase):
    def test_zero_deficit_basis_names_distinct_evidence_without_fabricating_failure(self):
        basis, blockers = release.qualification_basis(False, 0, [], [], 38)
        self.assertEqual(blockers, [])
        self.assertEqual(basis['mode'], 'zero_deficit_terminal_replay_planned_native_branches_and_cpu_orchestration')
        self.assertEqual(basis['actual_deficit_replenished'], 0)
        self.assertFalse(basis['fabricated_parent_deficit'])
        self.assertEqual(basis['planned_recovery_native_approved'], 4)

    def test_genuine_native_reviews_stay_blocked(self):
        basis, blockers = release.qualification_basis(True, 1, ['static_missing'], ['range_missing'], 37)
        self.assertEqual(basis['actual_deficit_replenished'], 1)
        self.assertEqual(len(blockers), 3)
        self.assertTrue(any('static_missing' in reason for reason in blockers))
        self.assertTrue(any('range_missing' in reason for reason in blockers))

    def test_range_rows_are_checked_against_frozen_source_plan(self):
        registered = dict(range_static=[dict(id='a', range_plan={'plan': 1}), dict(id='b', range_plan={'plan': 2})])
        source = {'a': {'source': 1}, 'b': {'source': 2}}
        row = dict(source_cell_id='a', status='accepted_candidate', capture_root='/raw/a', audit={})
        plan = dict(primary_data_id=10, cap_m=20)
        registered['range_static'][0]['range_plan'] = plan
        actual = dict(spec={}, runtime_actors=[dict(planned_center_distance_m=18)])
        with patch.object(release, 'read', return_value=actual), \
                patch.object(release.primary, 'verify_preserved', return_value=plan) as verifier:
            observed, missing = release.verify_range_rows(registered, [row], source)
            self.assertEqual(missing, ['b'])
            self.assertEqual(observed[0]['actual_primary_distance_m'], 18)
            verifier.assert_called_once_with(source['a'], actual)
        with self.assertRaisesRegex(ValueError, 'review cannot'):
            release.verify_range_rows(registered, [dict(row, status='review')], source)

    def test_cpu_receipt_requires_all_named_checks_source_and_test_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest, receipt, log = root / 'manifest.json', root / 'receipt.json', root / 'tests.log'
            atomic_json(manifest, {})
            log.write_text('Ran 12 tests\nOK\n')
            runtime = {str(root / 'runtime.py'): 'hash'}
            doc = dict(schema=release.ORCHESTRATION_SCHEMA, passed=True, source_manifest_sha256=sha256(manifest),
                runtime_hashes=runtime, tests=dict(command=['python3', '-m', 'unittest'], exit_code=0,
                    tests_run=12, failures=0, errors=0), test_log=dict(path=str(log), sha256=sha256(log)),
                checks={name: True for name in release.ORCHESTRATION_CHECKS})
            atomic_json(receipt, doc)
            with patch.object(release.pins, 'verify_hashes') as verifier:
                self.assertEqual(release.verify_orchestration(receipt, manifest, runtime), doc)
                verifier.assert_any_call({str(log): sha256(log)}, 'CPU orchestration test log')
                for field in ('source_contract_error_halts', 'native_review_never_approved'):
                    modified = copy.deepcopy(doc)
                    modified['checks'][field] = False
                    atomic_json(receipt, modified)
                    with self.assertRaisesRegex(ValueError, 'receipt incomplete'):
                        release.verify_orchestration(receipt, manifest, runtime)

    def probe_fixture(self, root):
        manifest, registration = root / 'manifest.json', root / 'registration.json'
        atomic_json(manifest, {})
        atomic_json(registration, {})
        strata = [dict(source_id='s%d' % index) for index in range(4)]
        metadata = {row['source_id']: {'source': row['source_id']} for row in strata}
        attempts = [dict(source_id=row['source_id'], result=dict(status='accepted_candidate', capture_root=str(root / row['source_id']))) for row in strata]
        inventory = dict(captured_rows=4, files={'/raw': 'sha'}, ledger_sha256={'/ledger': 'sha'})
        contract = dict(runtime_hashes={'/runtime': 'sha'})
        state = dict(stage='complete', attempts=attempts, contract=contract, terminal_inventory=inventory)
        state_path = root / 'state.json'
        atomic_json(state_path, state)
        proof = dict(schema=release.recovery.SCHEMA, passed=True, source_manifest_sha256=sha256(manifest),
            registration_sha256=sha256(registration), prepared_count=4, accepted_count=4, planned_probe=True,
            actual_parent_deficit=False, training_data=False, bulk_started=False, additional_completed_resume_raw=0,
            state_path=str(state_path), state_sha256=sha256(state_path), strata4=strata, ledger_sha256={'/ledger': 'sha'})
        proof_path = root / 'proof.json'
        atomic_json(proof_path, proof)
        return manifest, registration, {}, metadata, state, proof, proof_path

    def verify_probe(self, fixture, changed_contract=False, changed_prepared=False, changed_raw=False, native_review=False):
        manifest, registration, deck, metadata, state, proof, proof_path = fixture
        contract = copy.deepcopy(state['contract'])
        if changed_contract:
            contract['new'] = True
        inventory = copy.deepcopy(state['terminal_inventory'])
        if changed_raw:
            inventory['captured_rows'] = 5
        with patch.object(release.recovery, 'source_inputs', return_value=(deck, {}, metadata, proof['strata4'])), \
                patch.object(release.recovery, 'contract', return_value=contract), \
                patch.object(release.recovery, 'verify_prepared', side_effect=ValueError('prepared mismatch') if changed_prepared else None) as prepared, \
                patch.object(release.recovery, 'snapshot', return_value=inventory), \
                patch.object(release.recovery, 'verify_attempt', side_effect=lambda a, *args: {'status': 'review'} if native_review else a['result']) as official, \
                patch.object(release.pins, 'verify_hashes'), \
                patch.object(release.primary, 'verify_preserved'), \
                patch.object(release, 'read', wraps=release.read) as reader:
            real_read = reader._mock_wraps
            reader.side_effect = lambda p: {} if Path(p).parent.name in metadata else real_read(p)
            result = release.verify_probe_proof(proof_path, manifest, registration, deck, metadata, {})
            prepared.assert_called_once_with(proof_path.parent, state, deck, metadata)
            self.assertEqual(official.call_count, 4)
            return result

    def test_probe_reuses_official_preparation_and_native_verification(self):
        with tempfile.TemporaryDirectory() as temporary:
            result, _ = self.verify_probe(self.probe_fixture(Path(temporary)))
            self.assertFalse(result['actual_parent_deficit'])
            self.assertEqual(result['additional_completed_resume_raw'], 0)

    def test_probe_contract_preparation_raw_and_qa_fail_closed(self):
        for key, message in (('changed_contract', 'contract differs'), ('changed_prepared', 'prepared mismatch'),
                             ('changed_raw', 'raw/ledger'), ('native_review', 'official native approval')):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as temporary:
                with self.assertRaisesRegex(ValueError, message):
                    self.verify_probe(self.probe_fixture(Path(temporary)), **{key: True})

    def test_existing_output_is_not_replaced(self):
        with tempfile.TemporaryDirectory() as temporary:
            from argparse import Namespace
            with self.assertRaises(FileExistsError):
                release.freeze(Namespace(output=Path(temporary)))

    def zero_resume_fixture(self, root):
        parent, completed = root / 'parent', root / 'completed'
        parent.mkdir()
        completed.mkdir()
        rows = [dict(source_cell_id='s%02d' % index, status='accepted_candidate') for index in range(64)]
        for folder in (parent, completed):
            (folder / 'selected_ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
            atomic_json(folder / 'quota.json', dict(target=64, accepted=64, deficit=0, pending=0, cells=[]))
        atomic_json(parent / 'batch_contract.json', {'test': True})
        atomic_json(parent / 'fresh_runtime_contract.json', {'test': True})
        parent_status = dict(stage='complete', elapsed_sec=10., requested=64, accepted_candidate=64)
        atomic_json(parent / 'status.json', parent_status)
        completion_status = dict(stage='complete', elapsed_sec=13.25, replenishment_elapsed_sec=3.25,
            requested=64, accepted_candidate=64, review=0, remaining_deficit=0, replenished=0)
        atomic_json(completed / 'status.json', completion_status)
        manifest = root / 'manifest.json'
        atomic_json(manifest, dict(count=64, scenes=[dict(id=row['source_cell_id']) for row in rows]))
        state = dict(stage='complete', deficits=[], attempts=[], baseline_selected=rows,
            contract=dict(source_sha256=sha256(manifest), baseline=str(parent), capture_contract={'test': True},
                code_hashes={Path(release.__file__).name: sha256(release.__file__)},
                parent_hashes={name: sha256(parent / name) for name in
                    ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json')}))
        atomic_json(completed / 'state.json', state)
        raw = parent / 'captures/s00/points.bin'
        raw.parent.mkdir(parents=True)
        raw.write_bytes(b'parent native fixture')
        event = parent / 'event_cmd_requests.jsonl'
        event.write_text('{"fixture":true}\n')
        stat = raw.stat()
        parent_snapshot = dict(raw_files={'captures/s00/points.bin': dict(sha256=sha256(raw),
            inode=stat.st_ino, size=stat.st_size, mtime_ns=stat.st_mtime_ns)},
            event_logs={'event_cmd_requests.jsonl': sha256(event)},
            selected_sha256=sha256(parent / 'selected_ledger.jsonl'), cumulative_elapsed_sec=10.)
        partial_before, partial_after = root / 'partial_before.json', root / 'partial_after.json'
        atomic_json(partial_before, dict(snapshot=parent_snapshot, selected_rows=rows[:16]))
        atomic_json(partial_after, dict(snapshot=parent_snapshot, selected_rows=rows,
            parent_root=str(parent), source_manifest_sha256=sha256(manifest)))
        partial_path = root / 'partial.json'
        atomic_json(partial_path, dict(passed=True, preserved_selected=16, prior_raw_unchanged=True,
            parent_root=str(parent), source_manifest_sha256=sha256(manifest),
            before_snapshot_path=str(partial_before), before_snapshot_sha256=sha256(partial_before),
            after_snapshot_path=str(partial_after), after_snapshot_sha256=sha256(partial_after)))
        terminal = dict(source_manifest_sha256=sha256(manifest), parent_root=str(parent), completion_root=str(completed),
            parent=parent_snapshot, completion=dict(raw_files={}, event_logs={},
                selected_sha256=sha256(completed / 'selected_ledger.jsonl'), cumulative_elapsed_sec=13.25),
            state_sha256={str(completed / 'state.json'): sha256(completed / 'state.json')})
        terminal_before, terminal_after = root / 'terminal_before.json', root / 'terminal_after.json'
        atomic_json(terminal_before, terminal)
        atomic_json(terminal_after, terminal)
        proof_path = root / 'resume.json'
        atomic_json(proof_path, dict(passed=True, additional_native_captures=0,
            before_snapshot_path=str(terminal_before), before_snapshot_sha256=sha256(terminal_before),
            after_snapshot_path=str(terminal_after), after_snapshot_sha256=sha256(terminal_after),
            ledger_sha256={str(folder / 'selected_ledger.jsonl'): sha256(folder / 'selected_ledger.jsonl')
                for folder in (parent, completed)},
            state_sha256={str(completed / 'state.json'): sha256(completed / 'state.json')},
            status_sha256={str(folder / 'status.json'): sha256(folder / 'status.json') for folder in (parent, completed)}))
        return proof_path, partial_path, parent, completed, manifest

    def verify_zero(self, args):
        with patch.object(release.legacy.completion.adaptive.fa.batch.campaign, 'contract', return_value={'test': True}):
            return release.verify_resume(*args)

    def test_legacy_empty_owned_raw_rejection_is_fixed_only_for_genuine_zero_deficit(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.zero_resume_fixture(Path(temporary))
            with self.assertRaisesRegex(ValueError, 'lacks owned raw'):
                release.legacy.verify_resume(*args)
            proof, partial = self.verify_zero(args)
            self.assertEqual(proof['additional_native_captures'], 0)
            self.assertEqual(partial['preserved_selected'], 16)
            self.assertEqual(release.read(args[3] / 'state.json')['attempts'], [])

    def test_zero_deficit_rejects_owned_native_and_nonempty_attempt_journal(self):
        for edit in ('owned_raw', 'attempt'):
            with self.subTest(edit=edit), tempfile.TemporaryDirectory() as temporary:
                args = self.zero_resume_fixture(Path(temporary))
                if edit == 'owned_raw':
                    path = args[3] / 'captures/hidden/points.bin'
                    path.parent.mkdir(parents=True)
                    path.write_bytes(b'unexpected native data')
                    message = 'unexpected owned native'
                else:
                    state = release.read(args[3] / 'state.json')
                    state['attempts'] = [{'ordinal': 1}]
                    atomic_json(args[3] / 'state.json', state)
                    message = 'genuine zero-deficit64'
                with self.assertRaisesRegex(ValueError, message):
                    self.verify_zero(args)

    def test_zero_deficit_preserves_parent_raw_and_current_runtime_hashes(self):
        for edit in ('parent_raw', 'runtime'):
            with self.subTest(edit=edit), tempfile.TemporaryDirectory() as temporary:
                args = self.zero_resume_fixture(Path(temporary))
                if edit == 'parent_raw':
                    (args[2] / 'captures/s00/points.bin').write_bytes(b'changed parent raw')
                    message = 'raw content or file identity'
                else:
                    state = release.read(args[3] / 'state.json')
                    state['contract']['code_hashes'][Path(release.__file__).name] = '0' * 64
                    atomic_json(args[3] / 'state.json', state)
                    message = 'completion runtime changed'
                with self.assertRaisesRegex(ValueError, message):
                    self.verify_zero(args)

    def test_zero_deficit_checks_terminal_elapsed_even_when_both_snapshots_match(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.zero_resume_fixture(Path(temporary))
            proof = release.read(args[0])
            for which in ('before', 'after'):
                path = Path(proof[which + '_snapshot_path'])
                snapshot = release.read(path)
                snapshot['completion']['cumulative_elapsed_sec'] = 999.
                atomic_json(path, snapshot)
                proof[which + '_snapshot_sha256'] = sha256(path)
            atomic_json(args[0], proof)
            with self.assertRaisesRegex(ValueError, 'cumulative elapsed changed'):
                self.verify_zero(args)

    def test_real_deficit_resume_retains_the_unchanged_legacy_verifier(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.zero_resume_fixture(Path(temporary))
            state = release.read(args[3] / 'state.json')
            state['deficits'] = ['real_missing']
            atomic_json(args[3] / 'state.json', state)
            with patch.object(release.legacy, 'verify_resume', return_value=('legacy', 'unchanged')) as verifier:
                self.assertEqual(release.verify_resume(*args), ('legacy', 'unchanged'))
                verifier.assert_called_once_with(*args)

    def range_completion_fixture(self, root, outcome='accepted_candidate'):
        parent, completed = root / 'range_parent', root / 'range_completion'
        parent.mkdir()
        completed.mkdir()
        entries = []
        for index in range(3):
            meta = root / ('source%d.json' % index)
            atomic_json(meta, {'source': index})
            entries.append(dict(id='s%d' % index, metadata=str(meta)))
        manifest = root / 'range_manifest.json'
        deck = dict(count=3, scenes=entries)
        atomic_json(manifest, deck)
        baseline = [dict(source_cell_id='s%d' % i, status='accepted_candidate', audit={}) for i in range(2)]
        (parent / 'selected_ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in baseline))
        for name in ('quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'):
            atomic_json(parent / name, {'source': 'preserved parent'})
        capture = completed / 'child'
        atomic_json(capture / 'shutdown_safety.json', {'park_verified': True})
        attempts = [dict(id='s2', ordinal=i, group='range:s2', manifest='owned_proposal_%d' % i,
            delta={}, capture=str(capture), result=dict(status=outcome, audit={}, capture_root=str(capture)))
            for i in range(1, 2 if outcome == 'accepted_candidate' else 4)]
        state = dict(stage='complete' if outcome == 'accepted_candidate' else 'complete_with_deficit',
            baseline_selected=baseline, source_order=['s0', 's1', 's2'], deficits=['s2'], attempts=attempts,
            contract=dict(source_sha256=sha256(manifest), baseline=str(parent), capture_contract={'subset': 3},
                code_hashes={Path(release.__file__).name: sha256(release.__file__)}, attempts=3,
                parent_hashes={name: sha256(parent / name) for name in
                    ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json')}))
        rows, missing = release.legacy.completion.merged(state)
        (completed / 'selected_ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
        atomic_json(completed / 'state.json', state)
        status = dict(stage=state['stage'], requested=3, accepted_candidate=len(rows), review=len(missing),
            remaining_deficit=len(missing), replenished=len(rows) - len(baseline),
            elapsed_sec=7., replenishment_elapsed_sec=2.)
        atomic_json(completed / 'status.json', status)
        atomic_json(completed / 'quota.json', dict(target=3, accepted=len(rows), deficit=len(missing),
            pending=0, cells=[dict(id=ident) for ident in missing]))
        return completed, parent, manifest, deck, baseline

    def verify_range_completion(self, args, native_changed=False):
        with patch.object(release.legacy, 'verify_native', side_effect=lambda root, count: release.read(root / 'status.json')) as native, \
                patch.object(release.legacy.completion.adaptive.fa.batch.campaign, 'contract', return_value={'subset': 3}) as contract, \
                patch.object(release.legacy.completion.spatial.gen, 'context', return_value={}), \
                patch.object(release.legacy.completion, 'pool_map', return_value={'pool': True}), \
                patch.object(release.legacy.completion, 'verify', side_effect=lambda a, **kw: {'status': 'review'} if native_changed else a['result']) as verifier:
            result = release.verify_range_completion(*args)
            native.assert_called_once_with(args[0], 3)
            contract.assert_called_once_with(args[2], [0, 1, 2], 2)
            self.assertEqual(verifier.call_count, len(release.read(args[0] / 'state.json')['attempts']))
            return result

    def test_range_subset_completion_is_not_hardcoded_to64_and_preserves_native_parent(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.range_completion_fixture(Path(temporary))
            rows, state, status = self.verify_range_completion(args)
            self.assertEqual(len(rows), 3)
            self.assertEqual(status['replenished'], 1)
            self.assertEqual(rows[:2], args[-1])
            self.assertEqual(state['baseline_selected'], args[-1])

    def test_range_completion_keeps_finite_reviews_as_explicit_missing_slots(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.range_completion_fixture(Path(temporary), 'review')
            rows, state, status = self.verify_range_completion(args)
            self.assertEqual(len(rows), 2)
            self.assertEqual(status['remaining_deficit'], 1)
            self.assertEqual(len(state['attempts']), 3)

    def test_range_completion_park_budget_and_official_verification_fail_closed(self):
        for edit, message in (('park', 'final Park'), ('budget', 'out-of-budget'), ('official', 'official native verification')):
            with self.subTest(edit=edit), tempfile.TemporaryDirectory() as temporary:
                args = self.range_completion_fixture(Path(temporary))
                if edit == 'park':
                    atomic_json(args[0] / 'child/shutdown_safety.json', {'park_verified': False})
                elif edit == 'budget':
                    state = release.read(args[0] / 'state.json')
                    state['attempts'][0]['ordinal'] = 4
                    atomic_json(args[0] / 'state.json', state)
                with self.assertRaisesRegex(ValueError, message):
                    self.verify_range_completion(args, native_changed=edit == 'official')

    def range_resume_fixture(self, root):
        args = self.range_completion_fixture(root)
        completed, parent, manifest, _, _ = args
        atomic_json(parent / 'status.json', dict(stage='complete_with_deficit', elapsed_sec=5.))
        for folder in (parent, completed):
            raw = folder / 'captures/native/points.bin'
            raw.parent.mkdir(parents=True)
            raw.write_bytes(b'real owned fixture raw')
        terminal = dict(source_manifest_sha256=sha256(manifest), parent_root=str(parent), completion_root=str(completed),
            parent=release.capture_snapshot(parent), completion=release.capture_snapshot(completed),
            state_sha256={str(completed / 'state.json'): sha256(completed / 'state.json')})
        before, after = root / 'range_before.json', root / 'range_after.json'
        atomic_json(before, terminal)
        atomic_json(after, terminal)
        proof = root / 'range_resume.json'
        atomic_json(proof, dict(passed=True, additional_native_captures=0,
            source_manifest_sha256=sha256(manifest), parent_root=str(parent), completion_root=str(completed),
            before_snapshot_path=str(before), before_snapshot_sha256=sha256(before),
            after_snapshot_path=str(after), after_snapshot_sha256=sha256(after),
            ledger_sha256={str(folder / 'selected_ledger.jsonl'): sha256(folder / 'selected_ledger.jsonl') for folder in (parent, completed)},
            state_sha256={str(completed / 'state.json'): sha256(completed / 'state.json')},
            status_sha256={str(folder / 'status.json'): sha256(folder / 'status.json') for folder in (parent, completed)}))
        return proof, parent, completed, manifest

    def test_range_completion_terminal_replay_matches_current_raw_and_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = self.range_resume_fixture(Path(temporary))
            proof = release.verify_range_completion_resume(*args)
            self.assertEqual(proof['additional_native_captures'], 0)

    def test_range_completion_terminal_raw_or_elapsed_changes_are_rejected(self):
        for edit in ('raw', 'elapsed', 'state'):
            with self.subTest(edit=edit), tempfile.TemporaryDirectory() as temporary:
                args = self.range_resume_fixture(Path(temporary))
                if edit == 'raw':
                    (args[2] / 'captures/native/points.bin').write_bytes(b'changed')
                elif edit == 'elapsed':
                    status = release.read(args[2] / 'status.json')
                    status['elapsed_sec'] += 1
                    atomic_json(args[2] / 'status.json', status)
                else:
                    state = release.read(args[2] / 'state.json')
                    state['changed'] = True
                    atomic_json(args[2] / 'state.json', state)
                with self.assertRaises(ValueError):
                    release.verify_range_completion_resume(*args)

    def test_optional_range_completion_requires_both_root_and_replay_proof(self):
        from argparse import Namespace
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(ValueError, 'supplied together'):
                release.freeze(Namespace(output=root / 'release', range_completion=root / 'child',
                                         range_completion_proof=None))


if __name__ == '__main__':
    unittest.main()
