import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_capture_campaign as campaign


class CampaignTest(unittest.TestCase):
    def test_subset_deck_equivalence_and_no_aliasing(self):
        source=dict(scenes=[dict(id=str(i)) for i in range(10000)],config=dict(a=[1]),count=10000)
        subset=campaign.subset_deck(source,[source['scenes'][9]])
        self.assertEqual(subset,dict(scenes=[dict(id='9')],config=dict(a=[1]),count=1))
        subset['config']['a'].append(2);subset['scenes'][0]['id']='changed'
        self.assertEqual(source['config']['a'],[1])
        self.assertEqual(source['scenes'][9]['id'],'9')

    def fixture(self,root):
        source=root/'source.json'
        source.write_text(json.dumps(dict(scenes=[dict(id='a',family='obstacle'),dict(id='b',family='vehicle')],config={})))
        return source

    def child(self,source,indices,output,**kwargs):
        output.mkdir(parents=True,exist_ok=True)
        ident=json.loads(source.read_text())['scenes'][0]['id']
        report=dict(stage='complete',attempts=[],selected=[dict(id=ident,status='accepted_candidate')],unresolved=[])
        campaign.atomic_json(output/'report.json',report)
        return 0

    def test_pause_resume_does_not_recapture_or_duplicate(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(campaign,'contract',return_value={'schema':1}),patch.object(campaign,'validate_selected'),patch.object(campaign.loop,'execute',side_effect=self.child) as child:
                self.assertEqual(campaign.execute(source,[0,1],out,stop_after=1),0)
                self.assertEqual(child.call_count,1)
                self.assertEqual(campaign.execute(source,[0,1],out,resume=True),0)
                self.assertEqual(child.call_count,2)
                self.assertEqual(campaign.execute(source,[0,1],out,resume=True),0)
                self.assertEqual(child.call_count,2)
            rows=[json.loads(l) for l in (out/'selected_ledger.jsonl').read_text().splitlines()]
            self.assertEqual([r['source_cell_id'] for r in rows],['a','b'])

    def test_unsafe_child_halts_before_second_cell(self):
        def fail(source,indices,output,**kwargs):
            output.mkdir(parents=True);campaign.atomic_json(output/'report.json',dict(stage='halted'))
            return 1
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root)
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign.loop,'execute',side_effect=fail) as child:
                self.assertEqual(campaign.execute(source,[0,1],root/'out'),1)
                self.assertEqual(child.call_count,1)

    def test_exhausted_quality_becomes_explicit_quota_deficit(self):
        def review(source,indices,output,**kwargs):
            output.mkdir(parents=True);campaign.atomic_json(output/'report.json',dict(stage='complete',attempts=[],selected=[],unresolved=['ground']))
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign,'validate_selected'),patch.object(campaign.loop,'execute',side_effect=review):
                self.assertEqual(campaign.execute(source,[0],out,rounds=1),0)
            self.assertEqual(json.loads((out/'quota.json').read_text())['deficit'],1)
            self.assertEqual(json.loads((out/'deficit_manifest.json').read_text())['scenes'][0]['id'],'a')
            self.assertEqual((out/'selected_ledger.jsonl').read_text(),'')

    def test_changed_contract_and_unapproved_bulk_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out';out.mkdir()
            campaign.atomic_json(out/'campaign_contract.json',{'old':1})
            with patch.object(campaign,'contract',return_value={'new':1}):
                with self.assertRaises(ValueError):campaign.execute(source,[0],out,resume=True)
            with self.assertRaises(ValueError):campaign.execute(source,list(range(65)),root/'bulk')

    def test_completed_deficit_verification_never_recaptures(self):
        def review(source,indices,output,**kwargs):
            output.mkdir(parents=True);campaign.atomic_json(output/'report.json',dict(stage='complete',attempts=[],selected=[],unresolved=['visibility']))
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign,'validate_selected'),patch.object(campaign.loop,'execute',side_effect=review) as child:
                self.assertEqual(campaign.execute(source,[0],out,rounds=1),0)
                self.assertEqual(campaign.execute(source,[0],out,rounds=1,resume=True),0)
                self.assertEqual(child.call_count,1)
                (out/'a'/'round_0'/'report.json').unlink()
                self.assertEqual(campaign.execute(source,[0],out,rounds=1,resume=True),1)
                self.assertEqual(child.call_count,1)

    def test_quota_replacement_retains_original_cell_identity(self):
        calls=[]
        def replacement(source,output):
            output.mkdir();data=json.loads(source.read_text());data['scenes'][0]['id']='a_replace'
            campaign.atomic_json(output/'manifest.json',data);return output/'manifest.json'
        def child(source,indices,output,**kwargs):
            calls.append(str(source));output.mkdir(parents=True)
            first=len(calls)==1
            campaign.atomic_json(output/'report.json',dict(stage='complete',attempts=[],
                selected=[] if first else [dict(id='a_replace',status='accepted_candidate')],unresolved=['visibility'] if first else []))
            return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out'
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign,'validate_selected'),patch.object(campaign.loop,'replacement',side_effect=replacement),patch.object(campaign.loop,'execute',side_effect=child):
                self.assertEqual(campaign.execute(source,[0],out),0)
            self.assertEqual(len(calls),2)
            self.assertEqual(json.loads((out/'selected_ledger.jsonl').read_text())['source_cell_id'],'a')

    def test_resume_unsafe_partial_child_never_launches(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'loop';child=out/'a'/'initial';child.mkdir(parents=True)
            campaign.atomic_json(child/'session_contract.json',{})
            with patch.object(campaign.loop.subprocess,'Popen') as process:
                self.assertEqual(campaign.loop.execute(source,[0],out,resume=True),1)
                process.assert_not_called()

    def test_stop_before_capture_preserves_pending_quota(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);out=root/'out';out.mkdir();(out/'STOP').touch()
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign.loop,'execute') as child:
                self.assertEqual(campaign.execute(source,[0],out,resume=True),0);child.assert_not_called()
            self.assertEqual(json.loads((out/'quota.json').read_text())['pending'],1)


if __name__=='__main__':unittest.main()
