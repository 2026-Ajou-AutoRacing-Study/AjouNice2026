import copy
from collections import Counter
import json
from pathlib import Path
import sys
import tempfile
import unittest
from argparse import Namespace
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_fresh_completion as c


def meta():
    return dict(spec=dict(family='obstacle',mission=False,distance_bounds=[10,30],role='front',seed=1),
        split='train',scene_group_id='fresh_v141:foggy13_cell',
        runtime_actors=[dict(name='Box',data_id=1,motion='static')])


class CompletionTests(unittest.TestCase):
    def test_merge_retains_baseline_and_fresh_group(self):
        original=dict(source_cell_id='a',status='accepted_candidate',capture_root='old')
        state=dict(baseline_selected=[original],source_order=['a','b'],deficits=['b'],attempts=[dict(
            id='b',group='fresh_v141:foggy13_b',manifest='m',delta={'replacement_actor_count':3},
            result=dict(source_cell_id='b_fc01',status='accepted_candidate'))])
        before=copy.deepcopy(state);rows,missing=c.merged(state)
        self.assertEqual(rows[0],original);self.assertEqual(rows[1]['scene_group_id'],'fresh_v141:foggy13_b')
        self.assertEqual(missing,[]);self.assertEqual(state,before)
    def test_repeated_review_never_counts_as_fill(self):
        state=dict(baseline_selected=[],source_order=['a'],deficits=['a'],attempts=[dict(id='a',result={'status':'review'})]*3)
        self.assertEqual(c.merged(state),([],['a']))
    def test_duplicate_slot_rejected(self):
        state=dict(baseline_selected=[{'source_cell_id':'a'}],source_order=['a'],deficits=[],attempts=[dict(
            id='a',result={'status':'accepted_candidate'})])
        with self.assertRaises(ValueError):c.merged(state)
    def test_different_motion_or_pool_rejected(self):
        source=meta();other=copy.deepcopy(source);other['runtime_actors'][0]['motion']='walking'
        self.assertFalse(c.compatible(source,other,{1:'obstacle'}))
        other=copy.deepcopy(source);other['runtime_actors'][0]['data_id']=2
        self.assertFalse(c.compatible(source,other,{1:'obstacle',2:'animal'}))
    def test_primary_gait_cannot_be_replaced_by_walking_secondary(self):
        source=meta();source['runtime_actors'][0]['motion']='walking'
        candidate=copy.deepcopy(source);candidate['runtime_actors'][0]['motion']='standing'
        candidate['runtime_actors'].append(dict(data_id=1,motion='walking'))
        self.assertFalse(c.compatible(source,candidate,{1:'obstacle'}))
    def test_candidate_preserves_source_group_before_capture(self):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'placement';source=meta();candidate=meta();candidate['scene_group_id']='donor_group'
            entry=dict(id='foggy13_b',scenario_sha256='old')
            deck=dict(config={'environment_tag':'foggy13'},scenes=[])
            with patch.object(c.flex,'choose',return_value=({'ego':1},candidate)),patch.object(c.adaptive.fa.batch.campaign,'subset_deck',side_effect=lambda deck,rows:dict(deck,scenes=rows)):
                a=c.prepare(entry,source,[],deck,{},None,1,folder,set(),{1:'obstacle'})
            got=c.read(a['entry']['metadata'])
            self.assertEqual(got['scene_group_id'],source['scene_group_id'])
            self.assertEqual(a['group'],source['scene_group_id'])
            self.assertEqual(c.spatial.gen.build,c.spatial.ORIGINAL_BUILD)
    def test_attempt_bound_cannot_be_disabled(self):
        for attempts in (0,6,100000):
            with self.assertRaises(ValueError):c.execute(Namespace(attempts=attempts,time_budget_sec=100))
    def test_rare_primary_floor_cannot_be_spent(self):
        source=meta();other=meta();other['runtime_actors'][0]['data_id']=2
        self.assertEqual(c.floor_donors(source,[other],Counter({1:3}),4),[source])
        self.assertEqual(c.floor_donors(source,[source,other],Counter({1:3}),4),[source])
        self.assertEqual(c.floor_donors(source,[source,other],Counter({1:4}),4),[source,other])
    def test_no_parent_mutation_destination(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with self.assertRaises(ValueError):c.execute(Namespace(attempts=3,time_budget_sec=100,output=root,baseline=root,resume=True))
    def test_partial_child_never_reloads(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)/'partial';root.mkdir()
            a=dict(capture=str(root),manifest='m',moving=False)
            with patch.object(c.subprocess,'Popen') as launch:
                with self.assertRaises(RuntimeError):c.run_child(a,Path(d),lambda root:None)
                launch.assert_not_called()
    def test_parent_verification_cannot_capture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);c.atomic_json(root/'status.json',dict(stage='running'))
            with patch.object(c.adaptive.fa.batch,'execute') as launch:
                with self.assertRaises(ValueError):c.verify_parent(root/'manifest.json',root)
                launch.assert_not_called()
    def test_finite_reviews_and_completed_resume_never_restart(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);parent=root/'parent';parent.mkdir();output=root/'result';mp=root/'metadata.json'
            c.atomic_json(mp,meta());manifest=root/'manifest.json'
            c.atomic_json(manifest,dict(config=dict(fresh_geometry_policy={'policy':'test'},environment_tag='foggy13'),
                scenes=[dict(id='a',metadata=str(mp),scenario_sha256='old')]))
            for n in ('selected_ledger.jsonl','quota.json','batch_contract.json','fresh_runtime_contract.json'):(parent/n).write_text('{}')
            a=Namespace(attempts=3,time_budget_sec=100,output=output,baseline=parent,manifest=manifest,resume=False,run=True)
            def proposal(entry,original,donors,deck,ctx,cal,ordinal,folder,known,pools):
                return dict(id='a',ordinal=ordinal,group=original['scene_group_id'],entry=dict(scenario_sha256=str(ordinal)),
                    manifest='not_used',capture='not_used',delta={},moving=False)
            with patch.object(c,'verify_parent',return_value=([],['a'],1)),patch.object(c.adaptive.fa.batch.campaign,'contract',return_value={}),\
                 patch.object(c.spatial.gen,'context',return_value={}),patch.object(c.adaptive,'prewarm',side_effect=lambda v,config:v),\
                 patch.object(c,'pool_map',return_value={1:'obstacle'}),patch.object(c.adaptive.fa,'normalized_sensor_calibration',return_value={}),\
                 patch.object(c,'prepare',side_effect=proposal),patch.object(c,'run_child',return_value=1) as launch,\
                 patch.object(c,'verify',return_value={'status':'review'}):
                self.assertEqual(c.execute(a),0);self.assertEqual(launch.call_count,3)
                self.assertEqual(c.read(output/'status.json')['stage'],'complete_with_deficit')
                before={n:(output/n).read_bytes() for n in ('state.json','selected_ledger.jsonl','status.json')}
                a.resume=True;self.assertEqual(c.execute(a),0);self.assertEqual(launch.call_count,3)
                self.assertEqual(before,{n:(output/n).read_bytes() for n in before})


if __name__=='__main__':unittest.main()
