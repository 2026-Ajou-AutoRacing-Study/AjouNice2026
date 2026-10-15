import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json,sha256,validate_stationary_failure
import base5_campaign_import as migration
import run_base5_batch_campaign as batch


class RecoveryTest(unittest.TestCase):
    def test_prepare_only_has_no_capture_or_recovery(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.json';atomic_json(source,dict(scenes=[dict(id='a')],config={}))
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child') as child,patch.object(batch,'recover') as recover:
                self.assertEqual(batch.execute(source,[0],root/'out',prepare_only=True),0)
                child.assert_not_called();recover.assert_not_called()
            self.assertEqual(json.loads((root/'out/status.json').read_text())['stage'],'prepared_waiting_for_resume')

    def test_stationary_deficit_requires_complete_safety_and_matching_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'evidence.json'
            proof=dict(reason='ego_pre_capture_not_stationary',scenario_sha256='scene',no_capture_requested=True,
                continuation_safety=dict(topic_freshness_verified=True,park_control_verified=True,brake_held=True,
                    collision_epoch=dict(matched=True),collision=dict(observed=True,non_ego_uids=[])))
            def row(value):
                atomic_json(path,value)
                return dict(status='failed',failure_kind='ego_stationary_support',scenario_sha256='scene',
                    failure_evidence=str(path),failure_evidence_sha256=sha256(path))
            validate_stationary_failure(row(proof))
            for field in ('topic_freshness_verified','park_control_verified','brake_held'):
                bad=copy.deepcopy(proof);bad['continuation_safety'][field]=False
                with self.assertRaises(ValueError):validate_stationary_failure(row(bad))
            for collision in (dict(observed=False,non_ego_uids=[]),dict(observed=True,non_ego_uids=[8500])):
                bad=copy.deepcopy(proof);bad['continuation_safety']['collision']=collision
                with self.assertRaises(ValueError):validate_stationary_failure(row(bad))
            for field,value in (('scenario_sha256','other'),('no_capture_requested',False),('reason','transport_failure')):
                bad=copy.deepcopy(proof);bad[field]=value
                with self.assertRaises(ValueError):validate_stationary_failure(row(bad))
            r=row(proof);atomic_json(path,{})
            with self.assertRaisesRegex(ValueError,'checksum'):validate_stationary_failure(r)

    def test_stationary_deficit_continues_unattempted_tail(self):
        records={}
        def child(manifest,root,stop,heartbeat):
            root.mkdir(exist_ok=True);atomic_json(root/'session_contract.json',{})
            records[str(root)]=[dict(id='a',status='failed',failure_kind='ego_stationary_support')];return 1
        def recovery(source,index,out,stop,heartbeat):
            self.assertEqual(index,1);out.mkdir(parents=True,exist_ok=True)
            atomic_json(out/'status.json',dict(stage='complete'))
            (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='b',source_cell_id='b',status='accepted_candidate'))+'\n')
            return 0
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.json';atomic_json(source,dict(scenes=[dict(id='a'),dict(id='b')],config={}))
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child',side_effect=child),patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:records[str(root)]),patch.object(batch,'validate_stationary_failure'),patch.object(batch,'recover',side_effect=recovery):
                self.assertEqual(batch.execute(source,[0,1],root/'out',2),0)
            self.assertEqual(json.loads((root/'out/quota.json').read_text())['deficit'],1)

    def test_imported_cells_are_not_recaptured_and_evidence_mutation_is_refused(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.json';atomic_json(source,dict(scenes=[dict(id='a'),dict(id='b')],config={}))
            evidence=root/'proof.json';atomic_json(evidence,dict(safe=True));out=root/'out';out.mkdir()
            receipt=out/'imported_prefix.json'
            atomic_json(receipt,dict(schema=1,manifest_sha256=sha256(source),selected=[dict(source_cell_id='a',status='accepted_candidate')],
                deficits=[dict(id='b',index=1,reasons=['historical_failure'])],elapsed_sec=10.,hashes={str(evidence):sha256(evidence)},sensor_stats={}))
            with patch.object(batch.campaign,'contract',return_value={}),patch.object(batch,'run_child') as child:
                self.assertEqual(batch.execute(source,[0,1],out,resume=True),0)
                self.assertEqual(batch.execute(source,[0,1],out,resume=True),0);child.assert_not_called()
            self.assertEqual(json.loads((out/'quota.json').read_text())['accepted'],1)
            atomic_json(evidence,dict(safe=False))
            with self.assertRaisesRegex(ValueError,'evidence changed'):migration.verify(receipt,source,[0,1])


if __name__=='__main__':unittest.main()
