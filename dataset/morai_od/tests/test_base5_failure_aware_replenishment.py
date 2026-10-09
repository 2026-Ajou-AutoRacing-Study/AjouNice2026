import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_failure_aware_replenishment as fa


def meta(x=10,y=0,yaw=0,retry=0,split='train'):
    return dict(spec=dict(family='obstacle',index=1),split=split,scene_group_id='g',retry=retry,
        ego=dict(x=0,y=0,z=0,yaw=0),runtime_actors=[dict(data_id=5,motion='static',
        longitudinal_m=x,lateral_m=y,placement_yaw_deg=yaw,position=dict(z=0),
        estimated_size=[1,1,1],geometric_center_estimate=[x,y],planned_center_distance_m=abs(x))])


class FailureAwareTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.mp=self.root/'original_metadata.json'
        fa.atomic_json(self.mp,meta());self.entry=dict(id='cell',metadata=str(self.mp),scenario='old',
            scenario_sha256='oldhash',family='obstacle',motion='static')
        self.deck=dict(config=dict(environment='sunny13'),scenes=[self.entry])

    def test_view_novelty_uses_relative_geometry_not_absolute_location(self):
        a=meta();b=meta();b['ego']['x']=1000
        self.assertEqual(fa.novelty(a,[b]),0)
        self.assertGreater(fa.novelty(meta(x=14),[b]),1)

    def test_yaw_wrap(self):
        self.assertAlmostEqual(fa.novelty(meta(yaw=179),[meta(yaw=-179)]),2/30)

    def test_bounded_search_rejects_same_failed_view(self):
        with patch.object(fa.gen,'build',return_value=({},meta())) as build:
            with self.assertRaisesRegex(ValueError,'No distinct'):
                fa.choose(self.entry,{},None,None,[meta()],1,2)
            self.assertEqual(build.call_count,512)

    def test_rank_candidates_and_keep_spec_asset_split(self):
        def generate(spec,config,ctx,retry):
            m=meta(x=10+retry,retry=retry);return dict(retry=retry),m
        with patch.object(fa.gen,'build',side_effect=generate),patch.object(fa,'geometry_score',
                side_effect=lambda m,c:dict(score=m['retry'])):
            scene,m,proof=fa.choose(self.entry,{},None,None,[meta()],1,4)
        self.assertEqual(proof['valid_candidates'],4)
        self.assertEqual(scene['retry'],4);self.assertEqual(m['spec'],meta()['spec'])
        self.assertEqual(m['runtime_actors'][0]['data_id'],5)

    def test_wrong_split_skipped(self):
        def generate(spec,config,ctx,retry):return {},meta(x=15,retry=retry,split='val' if retry==1 else 'train')
        with patch.object(fa.gen,'build',side_effect=generate),patch.object(fa,'geometry_score',return_value=dict(score=1)):
            _,_,proof=fa.choose(self.entry,{},None,None,[],1,1)
        self.assertEqual(proof['retry'],2)

    def test_changed_primary_rejected(self):
        m=meta();m['runtime_actors'][0]['data_id']=9
        with patch.object(fa.gen,'build',return_value=({},m)):
            with self.assertRaisesRegex(ValueError,'Primary changed'):fa.choose(self.entry,{},None,None,[],1,1)

    def test_changed_population_spec_is_skipped_before_writing_or_capture(self):
        def generate(spec,config,ctx,retry):
            m=meta(x=15,retry=retry)
            if retry==1:m['spec']['actors']=7
            return {},m
        with patch.object(fa.gen,'build',side_effect=generate),patch.object(fa,'geometry_score',return_value=dict(score=1)):
            _,_,proof=fa.choose(self.entry,{},None,None,[],1,1)
        self.assertEqual(proof['retry'],2)

    def test_search_starts_after_history_and_rounds_have_separate_blocks(self):
        with patch.object(fa.gen,'build',return_value=({},meta(x=15))),patch.object(fa,'geometry_score',return_value=dict(score=1)):
            _,_,proof=fa.choose(self.entry,{},None,None,[meta(retry=78)],2,1)
        self.assertEqual(proof['retry'],79+4096)

    def test_prepared_manifest_preserves_identity_config_and_refuses_overwrite(self):
        m=meta(x=15,retry=9);m['scene_group_id']='wrong'
        with patch.object(fa,'choose',return_value=({'test':1},m,{'score':1})):
            p=fa.prepare(self.entry,self.deck,None,None,[],1,self.root/'new')
            fa.check_placement(self.entry,p,self.deck['config'])
            self.assertEqual(fa.read(fa.read(p)['scenes'][0]['metadata'])['scene_group_id'],'g')
            with self.assertRaises(FileExistsError):fa.prepare(self.entry,self.deck,None,None,[],1,self.root/'new')

    def test_proxy_front_camera_and_lidar_not_a_certificate(self):
        calibration=fa.normalized_sensor_calibration(fa.SENSOR)
        near=fa.geometry_score(meta(x=10),calibration)
        far=fa.geometry_score(meta(x=90),calibration)
        self.assertGreater(near['score'],far['score'])
        self.assertIn('no rendered mask',near['limitation'])
        json.dumps(near)  # numpy scalar counts must not break durable proposal writes.

    def test_requires_pause_before_any_generation_or_ros(self):
        with self.assertRaisesRegex(ValueError,'Pause original'):
            fa.execute(self.root/'unused',self.root/'baseline',self.root/'new',run=True)
        self.assertFalse((self.root/'new').exists())

    def test_offline_initialization_and_resume_never_capture_or_change_baseline(self):
        baseline=self.root/'baseline';baseline.mkdir();fa.atomic_json(baseline/'PAUSE',{})
        (baseline/'.batch.lock').touch()
        fa.atomic_json(baseline/'status.json',dict(stage='paused',elapsed_sec=30))
        fa.atomic_json(baseline/'batch_contract.json',dict(chunk_size=4))
        chosen=dict(source_cell_id='cell',id='cell',status='accepted_candidate')
        (baseline/'selected_ledger.jsonl').write_text(json.dumps(chosen)+'\n')
        fa.atomic_json(baseline/'quota.json',dict(deficit=0,cells=[]))
        fa.atomic_json(baseline/'replenishment/state.json',dict(attempts=[]))
        source=self.root/'manifest.json';fa.atomic_json(source,self.deck)
        before={str(p):p.read_bytes() for p in baseline.rglob('*') if p.is_file()}
        def validate(*args,**kw):kw['_verification'].update(selected={'cell':chosen},deficits={});return 0
        with patch.object(fa.batch.campaign,'contract',return_value={}),patch.object(fa.batch,'execute',side_effect=validate),\
                patch('base5_campaign_replenishment.merged',return_value=([chosen],{})),\
                patch.object(fa.subprocess,'Popen') as capture:
            output=self.root/'new'
            self.assertEqual(fa.execute(source,baseline,output),0)
            ledger=(output/'selected_ledger.jsonl').stat()
            self.assertEqual(fa.execute(source,baseline,output,resume=True),0)
            self.assertEqual(ledger.st_mtime_ns,(output/'selected_ledger.jsonl').stat().st_mtime_ns)
            capture.assert_not_called()
            self.assertEqual(fa.read(output/'state.json')['elapsed_sec'],0)
        self.assertEqual(before,{str(p):p.read_bytes() for p in baseline.rglob('*') if p.is_file()})

    def test_resume_refuses_modified_baseline_before_ros(self):
        baseline=self.root/'baseline';baseline.mkdir();fa.atomic_json(baseline/'PAUSE',{})
        fa.atomic_json(baseline/'status.json',dict(stage='paused',elapsed_sec=1))
        for name in ('selected_ledger.jsonl','quota.json','replenishment/state.json'):
            path=baseline/name;path.parent.mkdir(exist_ok=True);path.write_text('{}')
        source=self.root/'manifest.json';fa.atomic_json(source,self.deck)
        output=self.root/'new';output.mkdir();fa.atomic_json(output/'state.json',dict(contract={}))
        with patch.object(fa.batch.campaign,'contract',return_value={}),patch.object(fa.subprocess,'Popen') as capture:
            self.assertEqual(fa.execute(source,baseline,output,resume=True,run=True),1)
            capture.assert_not_called()
        self.assertIn('resume contract',fa.read(output/'preflight_error.json')['error'])

    def simulation(self,status):
        baseline=self.root/'baseline';baseline.mkdir();fa.atomic_json(baseline/'PAUSE',{})
        fa.atomic_json(baseline/'status.json',dict(stage='paused',elapsed_sec=30))
        fa.atomic_json(baseline/'batch_contract.json',dict(chunk_size=4))
        (baseline/'selected_ledger.jsonl').write_text('')
        fa.atomic_json(baseline/'quota.json',dict(deficit=1,cells=[dict(id='cell')]))
        fa.atomic_json(baseline/'replenishment/state.json',dict(attempts=[]))
        source=self.root/'manifest.json';fa.atomic_json(source,self.deck);calls=[]
        def validate(*args,**kw):kw['_verification'].update(selected={},deficits={'cell':{}});return 0
        def propose(entry,deck,ctx,cal,history,ordinal,folder,candidates):
            with patch.object(fa,'choose',return_value=({'ordinal':ordinal},meta(x=15+ordinal),{})):
                return fa.prepare_original(entry,deck,ctx,cal,history,ordinal,folder,candidates)
        def verify(a,entry,deck):return dict(id='cell_fa'+str(a['ordinal']).zfill(2),status=status,
            audit=dict(review_reasons=[] if status=='accepted_candidate' else ['primary_no_sensor_evidence']))
        class Child:
            def __init__(self,cmd,**kw):
                calls.append(cmd);root=Path(cmd[cmd.index('--output')+1]);root.mkdir(parents=True)
                fa.atomic_json(root/'shutdown_safety.json',dict(park_verified=True))
                (root/'ledger.jsonl').write_text(json.dumps(dict(status=status))+'\n')
            def wait(self,timeout=None):return 0
        fa.prepare_original=fa.prepare;self.addCleanup(lambda:delattr(fa,'prepare_original'))
        patches=[patch.object(fa.batch.campaign,'contract',return_value={}),patch.object(fa.batch,'execute',side_effect=validate),
            patch('base5_campaign_replenishment.merged',return_value=([],{'cell':{}})),
            patch.object(fa.gen,'context',return_value={}),patch.object(fa,'prepare',side_effect=propose),
            patch.object(fa,'verify_attempt',side_effect=verify),patch.object(fa.subprocess,'Popen',side_effect=Child)]
        for p in patches:p.start();self.addCleanup(p.stop)
        return source,baseline,self.root/'new',calls

    def test_two_failed_attempts_are_cumulative_and_not_replayed_on_resume(self):
        source,baseline,output,calls=self.simulation('review')
        self.assertEqual(fa.execute(source,baseline,output,run=True),0)
        self.assertEqual(len(calls),2)
        self.assertEqual(fa.read(output/'quota.json')['deficit'],1)
        self.assertEqual(fa.execute(source,baseline,output,resume=True,run=True),0)
        self.assertEqual(len(calls),2)

    def test_success_stops_attempts_and_preserves_selected_on_resume(self):
        source,baseline,output,calls=self.simulation('accepted_candidate')
        self.assertEqual(fa.execute(source,baseline,output,run=True),0)
        self.assertEqual(len(calls),1)
        before=(output/'selected_ledger.jsonl').read_bytes()
        self.assertEqual(fa.execute(source,baseline,output,resume=True,run=True),0)
        self.assertEqual(len(calls),1);self.assertEqual(before,(output/'selected_ledger.jsonl').read_bytes())

    def test_crash_between_raw_completion_and_commit_does_not_recapture(self):
        source,baseline,output,calls=self.simulation('accepted_candidate')
        self.assertEqual(fa.execute(source,baseline,output,run=True),0)
        state=fa.read(output/'state.json');state['stage']='running';state['attempts'][0]['result']=None
        fa.atomic_json(output/'state.json',state)
        self.assertEqual(fa.execute(source,baseline,output,resume=True,run=True),0)
        self.assertEqual(len(calls),1)

    def test_pause_returns_at_boundary_without_capture(self):
        source,baseline,output,calls=self.simulation('accepted_candidate')
        self.assertEqual(fa.execute(source,baseline,output),0);fa.atomic_json(output/'PAUSE',{})
        self.assertEqual(fa.execute(source,baseline,output,resume=True,run=True),0)
        self.assertEqual(len(calls),0);self.assertEqual(fa.read(output/'status.json')['stage'],'paused')

    def test_explicit_import_keeps_completed_success_and_attempt_cap(self):
        source,baseline,old,calls=self.simulation('accepted_candidate')
        self.assertEqual(fa.execute(source,baseline,old,run=True),0)
        state=fa.read(old/'state.json');state['stage']='halted';state['error']='uncaptured proposal contract error'
        state['contract']['code_sha256']='historical_driver'
        fa.atomic_json(old/'state.json',state);before=(old/'state.json').read_bytes()
        new=self.root/'imported'
        self.assertEqual(fa.execute(source,baseline,new,run=True,seed_supplement=old),0)
        self.assertEqual(len(calls),1)
        self.assertEqual((new/'selected_ledger.jsonl').read_bytes(),(old/'selected_ledger.jsonl').read_bytes())
        self.assertEqual(before,(old/'state.json').read_bytes())

    def test_pending_import_rejected_without_capture(self):
        source,baseline,old,calls=self.simulation('accepted_candidate')
        self.assertEqual(fa.execute(source,baseline,old,run=True),0)
        state=fa.read(old/'state.json');state['attempts'][0]['result']=None;fa.atomic_json(old/'state.json',state)
        self.assertEqual(fa.execute(source,baseline,self.root/'imported',run=True,seed_supplement=old),1)
        self.assertEqual(len(calls),1)


if __name__=='__main__':unittest.main()
