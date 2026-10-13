import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_adaptive_completion as ac
from test_base5_failure_aware_replenishment import meta


class AdaptiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.metadata=self.root/'metadata.json'
        ac.fa.atomic_json(self.metadata,meta())
        self.entry=dict(id='cell',metadata=str(self.metadata),scenario=str(self.root/'original.json'),
            scenario_sha256='original',family='obstacle',role='front',distance_bounds=[10,30],motion='static')
        self.deck=dict(config={},scenes=[self.entry])

    def test_strategies_are_reason_aware_and_motion_never_bypassed(self):
        row=lambda r:dict(audit=dict(review_reasons=[r]))
        self.assertEqual(ac.strategy([row('missing_runtime_actor')],False)[0],'spawn_clearance')
        self.assertEqual(ac.strategy([row('static_ground_severe_below')],False)[0],'terrain_and_visibility')
        self.assertEqual(ac.strategy([row('stationary_vehicle_moved_from_placement')],False)[0],'supported_vehicle_pose')
        self.assertEqual(ac.strategy([row('population_visible_actor_deficit')],False)[0],'all_actor_visibility')
        self.assertEqual(ac.strategy([row('primary_no_sensor_evidence')],True)[0],'prepared_motion')

    def test_all_parent_score_penalizes_unseen_secondary(self):
        m=meta();m['runtime_actors']*=2;m['population_budget']=dict(required_visible_actors=2)
        good=dict(score=4,lidar_beam_proxy=1,camera_min_extent_proxy_px=8,angular_occlusion_proxy=0)
        bad=dict(score=0,lidar_beam_proxy=0,camera_min_extent_proxy_px=0,angular_occlusion_proxy=1)
        with patch.object(ac.fa,'geometry_score',side_effect=[good,good]):a=ac.proposal_score(m,None,'all_actor_visibility')
        with patch.object(ac.fa,'geometry_score',side_effect=[good,bad]):b=ac.proposal_score(m,None,'all_actor_visibility')
        self.assertGreater(a['score'],b['score']);self.assertEqual(b['predicted_visible_parents'],1)

    def test_same_view_and_changed_spec_are_not_candidates(self):
        with patch.object(ac.fa.gen,'build',return_value=({},meta())) as build:
            with self.assertRaisesRegex(ValueError,'No distinct'):
                ac.choose(self.entry,{},None,None,[meta()],1,1)
            self.assertEqual(build.call_count,1024)
        m=meta(x=30);m['spec']['actors']=7
        with patch.object(ac.fa.gen,'build',return_value=({},m)):
            with self.assertRaisesRegex(ValueError,'No distinct'):
                ac.choose(self.entry,{},None,None,[],1,1)

    def test_moving_candidate_requires_mapped_witness_pose(self):
        m=meta(x=30);m['runtime_actors'][0]['position'].update(x=30,y=0)
        with patch.object(ac.fa.gen,'build',return_value=({},m)),patch('prepare_base5_empty_ground_probe.target_probe_pose',side_effect=ValueError('unsupported')) as probe:
            with self.assertRaisesRegex(ValueError,'No distinct'):
                ac.choose(self.entry,{},dict(links=[]),None,[],1,1,kind='prepared_motion')
            self.assertEqual(probe.call_args.kwargs['maximum_road_distance'],8.)

    def test_prepare_injection_is_restored_after_exception(self):
        original=ac.fa.choose
        with patch.object(ac.fa,'prepare',side_effect=RuntimeError('test')):
            with self.assertRaises(RuntimeError):ac.prepare(self.entry,self.deck,{},None,[],1,self.root/'candidate','primary_visibility')
        self.assertIs(ac.fa.choose,original)

    def simulation(self,status='review'):
        baseline=self.root/'baseline';baseline.mkdir();seed=self.root/'seed';seed.mkdir()
        previous=self.root/'previous.json';ac.fa.atomic_json(previous,self.deck)
        failure=dict(status='review',audit=dict(review_reasons=['primary_no_sensor_evidence']),scenario_sha256='old_failed')
        ac.fa.atomic_json(seed/'state.json',dict(attempts=[dict(id='cell',manifest=str(previous),result=failure)]))
        ac.fa.atomic_json(seed/'status.json',dict(stage='paused',elapsed_sec=100))
        kept=dict(source_cell_id='kept',id='kept',family='obstacle',role='front',distance_bounds=[10,30],status='accepted_candidate')
        keptentry=dict(self.entry,id='kept');self.deck['scenes'].append(keptentry)
        (seed/'selected_ledger.jsonl').write_text(json.dumps(kept)+'\n')
        ac.fa.atomic_json(seed/'quota.json',dict(cells=[dict(id='cell')]))
        source=self.root/'manifest.json';ac.fa.atomic_json(source,self.deck)
        def importer(source,baseline,output,**kwargs):
            output.mkdir();(output/'selected_ledger.jsonl').write_text((seed/'selected_ledger.jsonl').read_text());return 0
        def prepare(entry,deck,ctx,cal,history,ordinal,folder,kind,heartbeat=None):
            folder.mkdir(parents=True);scene=folder/'scene.json';scene.write_text(json.dumps(dict(ordinal=ordinal)))
            metadata=folder/'metadata.json';m=meta(x=20+ordinal,retry=ordinal);ac.fa.atomic_json(metadata,m)
            manifest=folder/'manifest.json';ac.fa.atomic_json(manifest,dict(config=deck['config'],scenes=[dict(entry,
                scenario=str(scene),metadata=str(metadata),scenario_sha256=ac.fa.sha256(scene))]))
            return manifest
        def verify(a,*args):return dict(id='fresh',status=status,
            audit=dict(review_reasons=[] if status=='accepted_candidate' else ['primary_no_sensor_evidence']))
        patches=[patch.object(ac.fa,'execute',side_effect=importer),patch.object(ac.fa.batch.campaign,'contract',return_value={}),
            patch.object(ac.fa.gen,'context',return_value={}),patch.object(ac,'prewarm',side_effect=lambda c,x:c),
            patch.object(ac.fa,'history_for',return_value=[meta()]),patch.object(ac,'prepare',side_effect=prepare),
            patch.object(ac,'verify',side_effect=verify),patch.object(ac,'run_child',return_value=0)]
        for p in patches:p.start();self.addCleanup(p.stop)
        return source,baseline,seed,self.root/'output',kept

    def test_additional_rounds_are_bounded_and_resume_does_not_reset(self):
        source,base,seed,out,kept=self.simulation()
        before={f.name:f.read_bytes() for f in seed.iterdir()}
        self.assertEqual(ac.execute(source,base,seed,out,run=True),0)
        self.assertEqual(ac.run_child.call_count,3)
        self.assertEqual(ac.fa.read(out/'status.json')['stage'],'complete_with_deficit')
        self.assertEqual(ac.execute(source,base,seed,out,run=True,resume=True),0)
        self.assertEqual(ac.run_child.call_count,3)
        self.assertEqual(before,{f.name:f.read_bytes() for f in seed.iterdir() if f.name!='.supplement.lock'})

    def test_success_preserves_baseline_and_stops_future_rounds(self):
        source,base,seed,out,kept=self.simulation('accepted_candidate')
        self.assertEqual(ac.execute(source,base,seed,out,run=True),0)
        self.assertEqual(ac.run_child.call_count,1)
        self.assertEqual(ac.fa.read(out/'status.json')['stage'],'complete')
        self.assertEqual(ac.fa.load_ledger(out/'selected_ledger.jsonl')[1],kept)
        before=(out/'selected_ledger.jsonl').stat()
        self.assertEqual(ac.execute(source,base,seed,out,run=True,resume=True),0)
        self.assertEqual(ac.run_child.call_count,1)
        self.assertEqual(before.st_mtime_ns,(out/'selected_ledger.jsonl').stat().st_mtime_ns)

    def test_pending_seed_and_running_seed_refused_before_capture(self):
        source,base,seed,out,_=self.simulation()
        state=ac.fa.read(seed/'state.json');state['attempts'][0]['result']=None;ac.fa.atomic_json(seed/'state.json',state)
        self.assertEqual(ac.execute(source,base,seed,out,run=True),1);ac.run_child.assert_not_called()

    def test_failed_child_without_safe_scene_evidence_halts(self):
        source,base,seed,out,_=self.simulation()
        ac.run_child.return_value=1
        self.assertEqual(ac.execute(source,base,seed,out,run=True),1)
        self.assertEqual(ac.run_child.call_count,1)
        self.assertEqual(ac.fa.read(out/'status.json')['stage'],'halted')

    def test_pause_no_capture_and_restricted_cells_are_recorded(self):
        source,base,seed,out,_=self.simulation()
        self.assertEqual(ac.execute(source,base,seed,out,only_cells=('cell',)),0)
        ac.fa.atomic_json(out/'PAUSE',{})
        self.assertEqual(ac.execute(source,base,seed,out,run=True,resume=True,only_cells=('cell',)),0)
        ac.run_child.assert_not_called()
        self.assertEqual(ac.fa.read(out/'status.json')['stage'],'paused')
        self.assertEqual(ac.execute(source,base,seed,out,resume=True),1)

    def test_completed_dynamic_report_is_not_replayed(self):
        root=self.root/'capture';root.mkdir();ac.fa.atomic_json(root/'status.json',dict(stage='complete_with_deficit'))
        with patch.object(ac.subprocess,'Popen') as popen:
            self.assertEqual(ac.run_child(dict(moving=True,capture=str(root),manifest='unused'),lambda r:None),0)
            popen.assert_not_called()

    def test_completed_static_session_is_not_replayed(self):
        root=self.root/'capture';root.mkdir()
        ac.fa.atomic_json(root/'shutdown_safety.json',dict(park_verified=True))
        (root/'ledger.jsonl').write_text(json.dumps(dict(status='review'))+'\n')
        with patch.object(ac.subprocess,'Popen') as popen:
            self.assertEqual(ac.run_child(dict(moving=False,capture=str(root),manifest='unused'),lambda r:None),0)
            popen.assert_not_called()

    def test_partial_static_session_cannot_be_replayed(self):
        root=self.root/'capture';root.mkdir();ac.fa.atomic_json(root/'session_contract.json',{})
        with patch.object(ac.subprocess,'Popen') as popen:
            with self.assertRaisesRegex(RuntimeError,'safety inspection'):
                ac.run_child(dict(moving=False,capture=str(root),manifest='unused'),lambda r:None)
            popen.assert_not_called()

    def test_seed_hash_change_is_not_silently_imported_on_resume(self):
        source,base,seed,out,_=self.simulation()
        self.assertEqual(ac.execute(source,base,seed,out),0)
        before=ac.fa.read(out/'state.json')
        ac.fa.atomic_json(seed/'quota.json',dict(cells=[]))
        self.assertEqual(ac.execute(source,base,seed,out,run=True,resume=True),1)
        ac.run_child.assert_not_called()
        self.assertEqual(ac.fa.read(out/'state.json'),before)

    def test_completed_attempt_evidence_change_blocks_resume(self):
        source,base,seed,out,_=self.simulation('accepted_candidate')
        self.assertEqual(ac.execute(source,base,seed,out,run=True),0)
        ac.verify.side_effect=lambda *args:dict(status='review')
        self.assertEqual(ac.execute(source,base,seed,out,run=True,resume=True),1)
        self.assertEqual(ac.run_child.call_count,1)

    def test_dynamic_identity_change_is_review_not_quota_success(self):
        root=self.root/'capture';root.mkdir();raw=self.root/'raw';raw.mkdir()
        changed=meta();changed['spec']['actors']=5;ac.fa.atomic_json(raw/'metadata.json',changed)
        row=dict(status='accepted_candidate',capture_root=str(raw))
        (root/'selected_ledger.jsonl').write_text(json.dumps(row)+'\n')
        with patch.object(ac.fa.batch,'verify_recovery',return_value=0):
            result=ac.verify_dynamic(dict(manifest='unused',capture=str(root)),self.entry,self.deck)
        self.assertEqual(result['status'],'review')
        self.assertEqual(result['reasons'],['replacement_identity_changed'])
        self.assertEqual(result['observed_selected'],row)


if __name__=='__main__':unittest.main()
