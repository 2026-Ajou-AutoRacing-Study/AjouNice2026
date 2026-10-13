import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_quality_loop as loop
from build_base5_bevfusion_infos import convert


class QualityLoopTest(unittest.TestCase):
    def test_static_budget_does_not_repeat_same_actor_or_unbounded_recapture(self):
        b=loop.StaticQualityBudget(dict(capture=dict(static_quality_budget=dict(
            max_ground_witnesses=8,max_height_recaptures=1,max_placement_recaptures=1))))
        self.assertTrue(b.spend('witness',1));self.assertFalse(b.spend('witness',1))
        self.assertTrue(b.spend('witness',2));self.assertTrue(b.spend('height'))
        self.assertFalse(b.spend('height'));self.assertTrue(b.spend('replacement'))
        self.assertFalse(b.spend('replacement'))
        self.assertEqual(b.reason,'static_quality_budget_exhausted_replacement')

    def test_legacy_budget_remains_opt_in(self):
        b=loop.StaticQualityBudget({})
        for _ in range(20):self.assertTrue(b.spend('witness',1))

    def test_budget_exhaustion_records_deficit_without_false_pass_or_capture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);loop.atomic_json(root/'meta.json',dict(runtime_actors=[dict(motion='static')]))
            config=dict(capture=dict(static_quality_budget=dict(max_ground_witnesses=8,
                max_height_recaptures=0,max_placement_recaptures=1)))
            loop.atomic_json(root/'manifest.json',dict(config=config,scenes=[dict(id='a',metadata=str(root/'meta.json'))]))
            row=dict(id='a',status='review',audit=dict(review_reasons=['actor_annotation_above_ground'],
                ground_support=dict(actors=[dict(reason='actor_annotation_above_ground',plane={'measured':True})])))
            with patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                 patch.object(loop.subprocess,'Popen') as popen:
                self.assertEqual(loop.execute(root/'manifest.json',[0],root/'out',seed_capture=root/'seed'),0)
            popen.assert_not_called();report=loop.gen.geo.load_json(root/'out/report.json')
            self.assertEqual(report['selected'],[])
            self.assertEqual(report['unresolved'][0]['static_quality_budget_reason'],'static_quality_budget_exhausted_height')

    def test_moving_review_returns_to_quota_round_before_ungrounded_recapture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);meta=dict(runtime_actors=[dict(motion='walking')])
            loop.atomic_json(root/'meta.json',meta)
            loop.atomic_json(root/'manifest.json',dict(config={},scenes=[dict(id='a',metadata=str(root/'meta.json'))]))
            row=dict(id='a',status='review',audit=dict(review_reasons=['primary_no_sensor_evidence']))
            with patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                 patch.object(loop.subprocess,'Popen') as popen,patch.object(loop,'replacement') as replacement:
                self.assertEqual(loop.execute(root/'manifest.json',[0],root/'out',seed_capture=root/'seed'),0)
            popen.assert_not_called();replacement.assert_not_called()
            report=json.loads((root/'out/report.json').read_text())
            self.assertEqual(report['selected'],[])
            self.assertEqual(report['unresolved'][0]['next_policy'],'same-quota replacement with fresh motion calibration')

    def test_ground_replacement_prefilters_unobservable_candidate_without_changing_quota(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);meta=dict(retry=0,spec=dict(family='pedestrian_related',role='left'),split='train',
                runtime_actors=[dict(data_id=42)])
            loop.atomic_json(root/'meta.json',meta)
            loop.atomic_json(root/'manifest.json',dict(config={},scenes=[dict(id='a',metadata=str(root/'meta.json'),scenario_sha256='source')]))
            new=dict(meta,runtime_actors=[dict(data_id=42,motion='standing')])
            with patch.object(loop.gen,'context',return_value={'links':{}}),patch.object(loop.gen,'build',return_value=({},new)) as build, \
                 patch('prepare_base5_empty_ground_probe.target_probe_pose',side_effect=[ValueError('not observable'),{}]) as witness:
                result=loop.replacement(root/'manifest.json',root/'out',require_ground_observable=True)
            self.assertEqual(build.call_count,2);self.assertEqual(witness.call_count,2)
            output=json.loads(result.read_text());saved=json.loads(Path(output['scenes'][0]['metadata']).read_text())
            self.assertEqual(saved['spec'],meta['spec']);self.assertEqual(saved['split'],'train')
            self.assertTrue(saved['replacement']['ground_observation_preflight'])

    def test_empty_replacement_retains_environment_group_across_nested_retry(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);meta=dict(retry=0,spec=dict(family='background'),split='train',
                runtime_actors=[],scene_group_id='original_group',environment_split_contract='six-environment')
            loop.atomic_json(root/'meta.json',meta)
            loop.atomic_json(root/'manifest.json',dict(config={},scenes=[dict(id='a',metadata=str(root/'meta.json'),scenario_sha256='source')]))
            new=dict(retry=1,spec=meta['spec'],split='train',runtime_actors=[])
            with patch.object(loop.gen,'context',return_value={}),patch.object(loop.gen,'build',return_value=({},new)):
                first=loop.replacement(root/'manifest.json',root/'first')
                second=loop.replacement(first,root/'second')
            saved=loop.gen.geo.load_json(Path(loop.gen.geo.load_json(second)['scenes'][0]['metadata']))
            self.assertEqual(saved['scene_group_id'],'original_group')
            self.assertEqual(saved['environment_split_contract'],'six-environment')

    def test_legacy_motion_rejected_before_any_calibration_load(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json';meta=root/'meta.json'
            loop.atomic_json(meta,dict(runtime_actors=[dict(motion='walking',
                motion_path=dict(policy='one-way-slow-v1'))]))
            loop.atomic_json(source,dict(scenes=[dict(id='a',metadata=str(meta))]))
            with patch.object(loop.subprocess,'Popen') as popen:
                self.assertEqual(loop.execute(source,[0],root/'out'),1)
                popen.assert_not_called()
            report=json.loads((root/'out/report.json').read_text())
            self.assertIn('before calibration',report['error'])
            self.assertEqual(report['attempts'],[])

    def test_collision_child_continues_only_after_verified_exit(self):
        for safe,expected in ((True,0),(False,1)):
            class Child:
                returncode=1
                def poll(self):return 1
            with tempfile.TemporaryDirectory() as d:
                root=Path(d);source=root/'source.json'
                source.write_text(json.dumps(dict(scenes=[dict(id='a'),dict(id='b')])))
                def child(cmd,**kwargs):
                    out=Path(cmd[cmd.index('--output')+1]);out.mkdir(exist_ok=True,parents=True)
                    evidence=out/'contact.json'
                    loop.atomic_json(evidence,dict(scenario_sha256='scene',non_ego_uids=[8500]))
                    row=dict(id=out.parent.name,status='failed',failure_kind='physical_collision',scenario_sha256='scene',
                        failure_evidence=str(evidence),failure_evidence_sha256=loop.sha256(evidence))
                    (out/'ledger.jsonl').write_text(json.dumps(row)+'\n')
                    loop.atomic_json(out/'shutdown_safety.json',dict(park_verified=safe))
                    loop.atomic_json(out/'status.json',dict(error='Capture halted after connection failure or three consecutive scene failures'))
                    return Child()
                with patch.object(loop.subprocess,'Popen',side_effect=child) as popen:
                    self.assertEqual(loop.execute(source,[0,1],root/'out'),expected)
                    self.assertEqual(popen.call_count,2 if safe else 1)
                self.assertEqual((root/'out/selected_ledger.jsonl').read_text(),'')

    def test_failed_child_stops_before_next_scene(self):
        class Child:
            returncode=1
            def poll(self):return 1
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json'
            source.write_text(json.dumps(dict(scenes=[dict(id='a'),dict(id='b')])))
            with patch.object(loop.subprocess,'Popen',return_value=Child()) as popen:
                self.assertEqual(loop.execute(source,[0,1],root/'out'),1)
                self.assertEqual(popen.call_count,1)
            r=json.loads((root/'out/report.json').read_text())
            self.assertEqual(r['stage'],'halted')
            self.assertEqual(r['selected'],[])
            self.assertEqual((root/'out/selected_ledger.jsonl').read_text(),'')

    def test_shadow_path_has_bounded_complementary_departure_witnesses(self):
        actor=dict(unique_id=1,motion='running',motion_path=dict(
            points=[dict(x=13.2,y=0),dict(x=-3,y=0)],target_position=dict(x=0,y=0)))
        meta=dict(spec=dict(shadow_moving=True),runtime_actors=[actor])
        self.assertEqual(loop.near_motion_probe_requests(meta),[(actor,0.),(actor,16.),(actor,12.)])
        meta['spec']={}
        self.assertEqual(loop.near_motion_probe_requests(meta),[(actor,0.)])
        meta['spec']['shadow_moving']=True
        actor['motion_path']['points'][0]['x']=6.6
        self.assertEqual(loop.near_motion_probe_requests(meta),[(actor,0.),(actor,16.),(actor,12.)])

    def test_empty_probe_cannot_become_training_negative(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cap=root/'capture';cap.mkdir()
            (cap/'metadata.json').write_text(json.dumps(dict(calibration_only=True)))
            ledger=root/'ledger.jsonl';ledger.write_text(json.dumps(dict(id='probe',status='accepted_candidate',capture_root=str(cap)))+'\n')
            sensor=Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
            r=convert(ledger,root/'info',sensor)
            self.assertEqual(sum(r['splits'].values()),0)
            self.assertEqual(r['excluded_review_scenes'],['probe'])

    def test_accepted_label_cannot_override_audit_review(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cap=root/'capture';cap.mkdir()
            (cap/'metadata.json').write_text(json.dumps(dict(runtime_actors=[])))
            (cap/'audit.json').write_text(json.dumps(dict(review_reasons=['ground_support_missing'])))
            ledger=root/'ledger.jsonl';ledger.write_text(json.dumps(dict(id='bad',status='accepted_candidate',capture_root=str(cap)))+'\n')
            sensor=Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
            r=convert(ledger,root/'info',sensor)
            self.assertEqual(sum(r['splits'].values()),0)
            self.assertEqual(r['excluded_review_scenes'],['bad'])


if __name__=='__main__':unittest.main()
