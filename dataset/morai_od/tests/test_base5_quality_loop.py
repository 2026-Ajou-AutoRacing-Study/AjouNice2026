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

    def test_empty_probe_cannot_become_training_negative(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cap=root/'capture';cap.mkdir()
            (cap/'metadata.json').write_text(json.dumps(dict(calibration_only=True)))
            ledger=root/'ledger.jsonl';ledger.write_text(json.dumps(dict(id='probe',status='accepted_candidate',capture_root=str(cap)))+'\n')
            sensor=Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
            r=convert(ledger,root/'info',sensor)
            self.assertEqual(sum(r['splits'].values()),0)
            self.assertEqual(r['excluded_review_scenes'],['probe'])


if __name__=='__main__':unittest.main()
