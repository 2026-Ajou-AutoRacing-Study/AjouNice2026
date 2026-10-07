import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_batch_campaign as batch


class BatchCampaignTest(unittest.TestCase):
    def test_collision_deficit_skips_recapture_and_continues_tail(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            root.mkdir(exist_ok=True);batch.atomic_json(root/'session_contract.json',{})
            records[str(root)]=[dict(id='a',status='failed',failure_kind='physical_collision')]
            return 1
        def recovery(source,index,out,stop,heartbeat):
            self.assertEqual(index,1)
            out.mkdir(parents=True,exist_ok=True);batch.atomic_json(out/'status.json',dict(stage='complete'))
            (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='b',source_cell_id='b',status='accepted_candidate'))+'\n')
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child) as run,patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]),patch.object(batch,'validate_collision_failure'),patch.object(batch,'recover',side_effect=recovery) as recover:
                self.assertEqual(batch.execute(source,[0,1],out,2),0)
                self.assertEqual(batch.execute(source,[0,1],out,2,resume=True),0)
                self.assertEqual(run.call_count,1);self.assertEqual(recover.call_count,2)
            self.assertEqual([r['source_cell_id'] for r in batch.load_ledger(out/'selected_ledger.jsonl')],['b'])
            self.assertEqual(json.loads((out/'quota.json').read_text())['deficit'],1)

    def fixture(self,root):
        source=root/'manifest.json'
        batch.atomic_json(source,dict(scenes=[dict(id='a'),dict(id='b')],config={}))
        return source

    def test_accepted_pause_resume_and_complete_do_not_recapture(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            rows=json.loads(manifest.read_text())['scenes']
            records[str(root)]=[dict(id=r['id'],status='accepted_candidate') for r in rows]
            batch.atomic_json(root/'session_contract.json',{})
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child) as process,patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]):
                self.assertEqual(batch.execute(source,[0,1],out,1,stop_after_chunks=1),0)
                self.assertEqual(process.call_count,1)
                self.assertEqual(batch.execute(source,[0,1],out,1,resume=True),0)
                self.assertEqual(process.call_count,2)
                self.assertEqual(batch.execute(source,[0,1],out,1,resume=True),0)
                self.assertEqual(process.call_count,2)
            rows=batch.load_ledger(out/'selected_ledger.jsonl')
            self.assertEqual([r['source_cell_id'] for r in rows],['a','b'])

    def test_safe_review_recovery_keeps_one_original_quota(self):
        records={};calls=[]
        def child(manifest,root,stop,heartbeat):
            calls.append(1);root.mkdir(exist_ok=True);batch.atomic_json(root/'session_contract.json',{})
            records[str(root)]=[dict(id='a',status='review')] if len(calls)==1 else [dict(id='a',status='review'),dict(id='b',status='accepted_candidate')]
            return 1 if len(calls)==1 else 0
        def recovery(source,index,out,stop,heartbeat):
            out.mkdir(parents=True,exist_ok=True)
            heartbeat(out)
            self.assertEqual(json.loads((out.parents[2]/'status.json').read_text())['current'],str(out))
            (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='a_replace',source_cell_id='a',status='accepted_candidate'))+'\n')
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child),patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]),patch.object(batch,'recover',side_effect=recovery):
                self.assertEqual(batch.execute(source,[0,1],out,2),0)
            self.assertEqual([r['source_cell_id'] for r in batch.load_ledger(out/'selected_ledger.jsonl')],['a','b'])

    def test_completed_resume_never_publishes_partial_snapshot(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            root.mkdir(exist_ok=True);batch.atomic_json(root/'session_contract.json',{})
            records[str(root)]=[dict(id=r['id'],status='accepted_candidate') for r in json.loads(manifest.read_text())['scenes']]
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child),patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]):
                self.assertEqual(batch.execute(source,[0,1],out,1),0)
            before=(out/'selected_ledger.jsonl').read_bytes()
            elapsed=json.loads((out/'status.json').read_text())['elapsed_sec']
            def inspect(root,manifest):
                self.assertEqual((out/'selected_ledger.jsonl').read_bytes(),before)
                return records[str(root)]
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'inspect_batch',side_effect=inspect),patch.object(batch,'run_child') as capture:
                self.assertEqual(batch.execute(source,[0,1],out,1,resume=True),0)
                capture.assert_not_called()
            self.assertEqual((out/'selected_ledger.jsonl').read_bytes(),before)
            self.assertEqual(json.loads((out/'status.json').read_text())['elapsed_sec'],elapsed)

    def test_target_miss_finishes_tail_without_resuming_failed_source(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            root.mkdir(exist_ok=True);batch.atomic_json(root/'session_contract.json',{})
            records[str(root)]=[dict(id='a',status='failed')];return 1
        def recovery(source,index,out,stop,heartbeat):
            out.mkdir(parents=True,exist_ok=True)
            (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='ab'[index],source_cell_id='ab'[index],status='accepted_candidate'))+'\n')
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child) as capture,patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]),patch.object(batch,'recover',side_effect=recovery) as recover:
                self.assertEqual(batch.execute(source,[0,1],out,2),0)
                self.assertEqual(capture.call_count,1);self.assertEqual(recover.call_count,2)
            self.assertEqual([r['source_cell_id'] for r in batch.load_ledger(out/'selected_ledger.jsonl')],['a','b'])

    def test_completed_missing_evidence_halts_without_recapture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out';out.mkdir()
            batch.atomic_json(out/'status.json',dict(stage='complete',elapsed_sec=30))
            before=b'preserved completed snapshot\n';(out/'selected_ledger.jsonl').write_bytes(before)
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child') as capture:
                self.assertEqual(batch.execute(source,[0,1],out,1,resume=True),1)
                capture.assert_not_called()
            self.assertEqual((out/'selected_ledger.jsonl').read_bytes(),before)
            self.assertEqual(json.loads((out/'status.json').read_text())['stage'],'halted')

    def test_unsafe_boundary_never_runs_recovery_or_next_chunk(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',return_value=1) as child,patch.object(batch,'inspect_batch',side_effect=ValueError('unsafe')),patch.object(batch,'recover') as recover:
                self.assertEqual(batch.execute(source,[0,1],out,1),1)
                self.assertEqual(child.call_count,1);recover.assert_not_called()
            self.assertEqual(json.loads((out/'quota.json').read_text())['pending'],2)

    def test_stop_before_load_and_contract_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out';out.mkdir();(out/'STOP').touch()
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child') as child:
                self.assertEqual(batch.execute(source,[0,1],out,resume=True),0);child.assert_not_called()
                with self.assertRaises(ValueError):batch.execute(source,[1,0],out,chunk_size=2,resume=True)

    def test_bulk_and_oversized_chunks_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for indices,size in [(list(range(65)),4),([0],17),([0,0],4)]:
                with self.assertRaises(ValueError):batch.execute(root/'absent',indices,root/'out',size)
            self.assertFalse((root/'out').exists())

    def test_bulk_checkpoint_heartbeat_pause_and_cumulative_time(self):
        records={};snapshots=[]
        def child(manifest,root,stop,heartbeat):
            ledger=out/'selected_ledger.jsonl'
            before=ledger.read_bytes() if ledger.exists() else None
            for _ in range(3):heartbeat(root)
            self.assertEqual(ledger.read_bytes() if ledger.exists() else None,before)
            rows=json.loads(manifest.read_text())['scenes']
            records[str(root)]=[dict(id=r['id'],family='test',status='accepted_candidate') for r in rows]
            batch.atomic_json(root/'session_contract.json',{})
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json';out=root/'out'
            batch.atomic_json(source,dict(scenes=[dict(id=str(i)) for i in range(68)],config={}))
            indices=list(range(68))
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child) as capture,patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]):
                self.assertEqual(batch.execute(source,indices,out,4,stop_after_chunks=2,bulk=True),0)
                paused=json.loads((out/'status.json').read_text());self.assertEqual(paused['finished'],8)
                paused['elapsed_sec']=100.;batch.atomic_json(out/'status.json',paused)
                self.assertEqual(batch.execute(source,indices,out,4,resume=True,bulk=True),0)
                status=json.loads((out/'status.json').read_text())
                self.assertGreaterEqual(status['elapsed_sec'],100.)
                self.assertEqual(status['accepted_bins'],{'["test", null, null]':68})
                self.assertEqual(len(batch.load_ledger(out/'selected_ledger.jsonl')),68)
                calls=capture.call_count;before=(out/'selected_ledger.jsonl').read_bytes()
                self.assertEqual(batch.execute(source,indices,out,4,resume=True,bulk=True),0)
                self.assertEqual(capture.call_count,calls)
                self.assertEqual((out/'selected_ledger.jsonl').read_bytes(),before)

    def test_halted_resume_never_loads(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out';out.mkdir()
            batch.atomic_json(out/'status.json',dict(stage='halted',elapsed_sec=40))
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child') as child:
                with self.assertRaisesRegex(ValueError,'Halted campaign'):batch.execute(source,[0,1],out,resume=True)
                child.assert_not_called()

    def test_pause_marker_finishes_current_chunk_then_resumes(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            records[str(root)]=[dict(id=r['id'],status='accepted_candidate') for r in json.loads(manifest.read_text())['scenes']]
            batch.atomic_json(root/'session_contract.json',{})
            (out/'PAUSE').touch()
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child) as capture,patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]):
                self.assertEqual(batch.execute(source,[0,1],out,1,bulk=True),0)
                self.assertEqual(json.loads((out/'status.json').read_text())['stage'],'paused')
                self.assertEqual(capture.call_count,1)
                (out/'PAUSE').unlink()
                self.assertEqual(batch.execute(source,[0,1],out,1,resume=True,bulk=True),0)
                self.assertEqual(capture.call_count,2)
                self.assertEqual(len(batch.load_ledger(out/'selected_ledger.jsonl')),2)

    def test_inspector_rejects_final_or_boundary_park_and_collision(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);batch.atomic_json(root/'shutdown_safety.json',dict(park_verified=False))
            with self.assertRaises(ValueError):batch.inspect_batch(root,root/'manifest.json')
            batch.atomic_json(root/'shutdown_safety.json',dict(park_verified=True));batch.atomic_json(root/'session_contract.json',{})
            row=dict(id='a',status='review',between_scene_park=dict(park_verified=False))
            with patch.object(batch,'prepare_session',return_value=([row],{})):
                with self.assertRaises(ValueError):batch.inspect_batch(root,root/'manifest.json')
                row['between_scene_park']['park_verified']=True
                row['audit']=dict(review_reasons=['physical_collision_during_capture_window'])
                with self.assertRaises(ValueError):batch.inspect_batch(root,root/'manifest.json')

    def test_parent_interrupt_stops_and_reaps_child(self):
        class Child:
            def poll(self):raise KeyboardInterrupt()
            def wait(self,timeout=None):self.waited=True;return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);child=Child()
            with self.assertRaises(KeyboardInterrupt):batch.wait_child(child,root,root/'parent_STOP',lambda root:None,10)
            self.assertTrue((root/'STOP').exists());self.assertTrue(child.waited)

    def test_parent_stop_is_forwarded_to_child(self):
        class Child:
            def poll(self):return None
            def wait(self,timeout=None):return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);stop=root/'parent_STOP';stop.touch()
            with self.assertRaises(RuntimeError):batch.wait_child(Child(),root,stop,lambda root:None,10)
            self.assertTrue((root/'STOP').exists())

    def test_forced_shutdown_targets_owned_process_group(self):
        class Child:
            pid=12345
            count=0
            def wait(self,timeout=None):
                self.count+=1
                if self.count<3:raise batch.subprocess.TimeoutExpired('child',timeout)
                return 0
        with tempfile.TemporaryDirectory() as d,patch.object(batch.os,'killpg') as kill:
            batch.stop_child(Child(),Path(d))
            self.assertEqual(kill.call_args_list[0].args,(12345,batch.signal.SIGTERM))
            self.assertEqual(kill.call_args_list[1].args,(12345,batch.signal.SIGKILL))


if __name__=='__main__':unittest.main()
