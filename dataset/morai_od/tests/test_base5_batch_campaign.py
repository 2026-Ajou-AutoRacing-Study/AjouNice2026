import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_batch_campaign as batch


class BatchCampaignTest(unittest.TestCase):
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
        def recovery(source,index,out,stop):
            out.mkdir(parents=True,exist_ok=True)
            (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='a_replace',source_cell_id='a',status='accepted_candidate'))+'\n')
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child),patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]),patch.object(batch,'recover',side_effect=recovery):
                self.assertEqual(batch.execute(source,[0,1],out,2),0)
            self.assertEqual([r['source_cell_id'] for r in batch.load_ledger(out/'selected_ledger.jsonl')],['a','b'])

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
