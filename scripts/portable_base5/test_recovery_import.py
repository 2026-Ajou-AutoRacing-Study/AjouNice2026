"""Read-only historical handoff tests; no ROS/simulator/native capture."""
import copy
import importlib
import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import recovery_import as handoff


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')
    return path


class RecoveryImportTests(unittest.TestCase):
    def fixture(self, directory):
        base = Path(directory)
        old, new = base / 'halted', base / 'new'
        old.mkdir()
        scripts = base / 'runtime'
        scripts.mkdir()
        driver = write(scripts / 'run_base5_batch_campaign.py', dict(code='original driver'))
        replenish = write(scripts / 'base5_campaign_replenishment.py', dict(code='original replenishment'))
        frozen = write(scripts / 'frozen.py', dict(code='original QA'))
        entries, rows = [], []
        for index in range(4):
            ident = 'scene_%05d' % index
            scenario = write(base / 'source' / (ident + '.json'), dict(scene=index))
            metadata = write(base / 'source' / (ident + '.meta.json'), dict(spec=dict(family='obstacle'),
                runtime_actors=[dict(data_id=10)],split='train',calibration_only=False))
            entries.append(dict(id=ident,scenario=str(scenario),scenario_sha256=handoff.digest(scenario),metadata=str(metadata)))
            capture = old / 'chunk_0000/capture/captures' / ident
            write(capture / 'scenario.json', dict(scene=index))
            write(capture / 'metadata.json', json.loads(metadata.read_text()))
            write(capture / 'audit.json', dict(review_reasons=[]))
            (capture / 'sensor.bin').write_bytes(b'preserved native sensor bytes')
            rows.append(dict(id=ident,source_cell_id=ident,status='accepted_candidate',
                audit=dict(review_reasons=[]),scenario_sha256=handoff.digest(capture/'scenario.json'),
                capture_root=str(capture),between_scene_park=dict(park_verified=True)))
        source = write(base/'source/manifest.json',dict(count=4,scenes=entries))
        native = [{k:v for k,v in row.items() if k != 'source_cell_id'} for row in rows[:3]]
        root = old/'chunk_0000/capture'
        write(old/'chunk_0000/manifest.json',dict(count=4,scenes=entries))
        write(root/'session_contract.json',dict(runner_options=dict(shared_ros_batch=True)))
        write(root/'shutdown_safety.json',dict(park_verified=True))
        (root/'ledger.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in native))
        selected = rows[:2]
        (old/'selected_ledger.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in selected))
        write(old/'status.json',dict(stage='halted',accepted_candidate=2,elapsed_sec=610.25))
        write(old/'quota.json',dict(target=4,accepted=2,deficit=0,pending=2,cells=[]))
        legacy=dict(schema=1,indices=list(range(4)),rounds=2,code={'frozen.py':handoff.digest(frozen)})
        contract=copy.deepcopy(legacy)
        contract.update(batch_driver_sha256=handoff.digest(driver),chunk_size=4,bulk=True,
            replenishment_driver_sha256=handoff.digest(replenish))
        write(old/'batch_contract.json',contract)
        fresh_files = dict(driver_sha256='run_base5_fresh_batch.py',
            spatial_sha256='base5_spatial_diversity.py', recovery_sha256='run_base5_capture_campaign_spatial.py',
            overlap_driver_sha256='run_base5_batch_overlap.py',overlap_worker_sha256='run_base5_capture_overlap.py',
            optimized_driver_sha256='run_base5_batch_optimized.py')
        for name in set(fresh_files.values()) | {'run_base5_capture_optimized.py'}:
            write(scripts/name,dict(code=name))
        fresh={key:handoff.digest(scripts/name) for key,name in fresh_files.items()}
        fresh.update(runtime_overlap=False,manifest_sha256=handoff.digest(source))
        write(old/'fresh_runtime_contract.json',fresh)
        write(old/'optimized_contract.json',dict(schema=1,manifest_sha256=handoff.digest(source),
            driver_sha256=handoff.digest(scripts/'run_base5_batch_optimized.py'),
            adapter_sha256=handoff.digest(scripts/'run_base5_capture_optimized.py'),
            setup_profile='static-ready',chunk_size=4,legacy=legacy))
        write(old/'route_compat_contract.json',dict(schema='base5_borrowed_route_compat_intent_v1',
            manifest_sha256=handoff.digest(source),release_sha256='0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180',
            original_native_qa=True,original_bounded_recovery=True,original_shutdown_safety=True,
            intent_sha256='a'*64,witness_sha256='b'*64,compatibility_files={str(frozen):handoff.digest(frozen)}))
        batch=SimpleNamespace(__file__=str(driver),campaign=SimpleNamespace(contract=lambda *args:copy.deepcopy(legacy)),
            inspect_batch=lambda *args:copy.deepcopy(native),verify_recovery=lambda *args:0)
        path=Path(handoff.__file__).resolve().parents[2]/'dataset/morai_od/scripts/base5_campaign_import.py'
        spec=importlib.util.spec_from_file_location('_original_import_verify_for_tests',str(path))
        importer=importlib.util.module_from_spec(spec);spec.loader.exec_module(importer)
        return base,source,old,new,batch,importer,selected

    def scoped(self, source, batch, importer):
        return patch.multiple(handoff,EXPECTED_SELECTED=2,EXPECTED_SCENES=4,
            CANONICAL_MANIFEST_SHA256=handoff.digest(source),_modules=lambda:(batch,importer))

    def test_plan_only_is_nonmutating(self):
        with tempfile.TemporaryDirectory() as directory:
            base,source,old,new,batch,importer,_=self.fixture(directory)
            before=handoff.tree_snapshot(old)
            with self.scoped(source,batch,importer): result=handoff.plan(source,old,new)
            self.assertEqual(result['selected_count'],2)
            self.assertEqual(result['pending_count'],2)
            self.assertFalse(result['files_written'] or new.exists())
            self.assertEqual(before,handoff.tree_snapshot(old))

    def test_export_preserves_authoritative_rows_and_ignores_extra_raw_success(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,selected=self.fixture(directory)
            before=handoff.tree_snapshot(old)
            with self.scoped(source,batch,importer): result=handoff.export(source,old,new)
            receipt=handoff.read(new/'imported_prefix.json')
            self.assertEqual(receipt['selected'],selected)
            self.assertEqual(receipt['deficits'],[])
            self.assertEqual(receipt['elapsed_sec'],610.25)
            self.assertEqual(before,handoff.tree_snapshot(old))
            self.assertEqual([p.name for p in new.iterdir()],['imported_prefix.json'])
            self.assertTrue(result['original_import_verifier_passed'])
            self.assertFalse(receipt['native_capture_executed'])
            self.assertEqual(importer.verify(new/'imported_prefix.json',source,list(range(4)))[0],selected)

    def test_torn_ledger_refuses_repair_and_preserves_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,_=self.fixture(directory)
            ledger=old/'selected_ledger.jsonl';ledger.write_bytes(ledger.read_bytes().rstrip(b'\n'))
            before=handoff.tree_snapshot(old)
            with self.scoped(source,batch,importer), self.assertRaisesRegex(RuntimeError,'Torn'):
                handoff.export(source,old,new)
            self.assertEqual(before,handoff.tree_snapshot(old));self.assertFalse(new.exists())

    def test_duplicate_ids_and_wrong_count_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,selected=self.fixture(directory)
            selected[1]=copy.deepcopy(selected[0])
            (old/'selected_ledger.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in selected))
            with self.scoped(source,batch,importer), self.assertRaisesRegex(RuntimeError,'unique'):
                handoff.export(source,old,new)

    def test_original_native_validator_failure_never_creates_new_output(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,_=self.fixture(directory)
            def fail(*args):raise ValueError('Original native Park proof failed')
            batch.inspect_batch=fail
            before=handoff.tree_snapshot(old)
            with self.scoped(source,batch,importer), self.assertRaisesRegex(ValueError,'Park'):
                handoff.export(source,old,new)
            self.assertFalse(new.exists());self.assertEqual(before,handoff.tree_snapshot(old))

    def test_mismatched_authoritative_native_row_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,selected=self.fixture(directory)
            selected[0]['unexpected_relabel']=True
            (old/'selected_ledger.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in selected))
            with self.scoped(source,batch,importer), self.assertRaisesRegex(RuntimeError,'completed original recovery'):
                handoff.export(source,old,new)
            self.assertFalse(new.exists())

    def test_old_file_write_attempt_is_blocked_before_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,_=self.fixture(directory)
            target=old/'status.json';before=target.read_bytes()
            def bad_validator(*args):target.write_text('silently repaired')
            batch.inspect_batch=bad_validator
            with self.scoped(source,batch,importer), self.assertRaisesRegex(RuntimeError,'forbids file writes'):
                handoff.export(source,old,new)
            self.assertEqual(target.read_bytes(),before);self.assertFalse(new.exists())

    def test_readonly_scope_forbids_ros_network_child_and_external_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory);output=base/'fresh';old=base/'old';old.mkdir()
            original_socket=socket.socket
            with handoff.readonly_cpu(output):
                for operation in (lambda:importlib.import_module('rospy'),lambda:socket.socket(),
                        lambda:subprocess.Popen(['anything']),lambda:(old/'new.txt').write_text('forbidden'),
                        lambda:(base/'unapproved').mkdir()):
                    with self.assertRaises(RuntimeError):operation()
                old.mkdir(exist_ok=True)
                output.mkdir();(output/'allowed.json').write_text('{}')
            self.assertIs(socket.socket,original_socket)

    def test_changed_old_contract_and_existing_or_nested_output_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,_=self.fixture(directory)
            with self.scoped(source,batch,importer):
                with self.assertRaisesRegex(RuntimeError,'overlap'):
                    handoff.plan(source,old,old/'nested')
                new.mkdir()
                with self.assertRaisesRegex(RuntimeError,'fresh sibling'):
                    handoff.plan(source,old,new)
            self.assertEqual(handoff.read(old/'status.json')['stage'],'halted')

    def test_completed_recovery_selected_is_verified_without_unfinished_other_campaign(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,selected=self.fixture(directory)
            ident=selected[0]['source_cell_id']
            original_inspect=batch.inspect_batch
            def reviewed(*args):
                rows=original_inspect(*args);rows[0]['status']='review';return rows
            batch.inspect_batch=reviewed
            recovery=old/'chunk_0000/recovery'/ident
            write(recovery/'status.json',dict(stage='complete'))
            (recovery/'selected_ledger.jsonl').write_text(json.dumps(selected[0])+'\n')
            write(old/'chunk_0000/recovery/scene_00003/status.json',dict(stage='halted'))
            calls=[]
            batch.verify_recovery=lambda source,index,output:calls.append((index,output)) or 0
            with self.scoped(source,batch,importer):handoff.export(source,old,new)
            self.assertEqual(calls,[(0,recovery)])
            self.assertEqual(handoff.read(new/'imported_prefix.json')['validation']['completed_recovery_count'],1)

    def test_altered_runtime_guard_is_rejected_without_old_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,_=self.fixture(directory)
            guard=handoff.read(old/'optimized_contract.json');guard['setup_profile']='different'
            write(old/'optimized_contract.json',guard)
            before=handoff.tree_snapshot(old)
            with self.scoped(source,batch,importer),self.assertRaisesRegex(RuntimeError,'static-ready profile'):
                handoff.export(source,old,new)
            self.assertEqual(before,handoff.tree_snapshot(old));self.assertFalse(new.exists())

    def test_historical_raw_changed_after_export_is_rejected_by_original_import_verify(self):
        with tempfile.TemporaryDirectory() as directory:
            _,source,old,new,batch,importer,selected=self.fixture(directory)
            with self.scoped(source,batch,importer):handoff.export(source,old,new)
            (Path(selected[0]['capture_root'])/'sensor.bin').write_bytes(b'changed original raw')
            with self.assertRaisesRegex(ValueError,'evidence changed'):
                importer.verify(new/'imported_prefix.json',source,list(range(4)))


if __name__=='__main__':
    unittest.main()
