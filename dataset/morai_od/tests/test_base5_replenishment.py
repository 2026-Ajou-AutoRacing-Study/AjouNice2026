import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_replenishment import eligible,execute,prepare
from report_base5_gt_policies import compare


class ReplenishmentTest(unittest.TestCase):
    def test_only_visibility_review_can_be_replaced(self):
        row=dict(status='review',audit=dict(review_reasons=['primary_no_sensor_evidence']))
        self.assertTrue(eligible(row))
        row['audit']['review_reasons'].append('missing_runtime_actor')
        self.assertFalse(eligible(row))
        row['audit']['review_reasons']=[];self.assertFalse(eligible(row))

    def test_bounded_and_halted_session_has_no_child_launch(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with self.assertRaises(ValueError):prepare(root/'absent',root/'absent',root/'new',5)
            (root/'report.json').write_text(json.dumps(dict(stage='halted')))
            with self.assertRaises(RuntimeError):execute(root/'absent',root)

    def test_gt_policy_keeps_primary_scene_gate(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);capture=root/'capture';capture.mkdir()
            (capture/'metadata.json').write_text(json.dumps(dict(runtime_actors=[dict(unique_id=1)])))
            components=[dict(parent_uid=1,training_class='animal',positive_lidar5=False,
                positive_lidar1=False,positive_lidar1_camera0=True)]
            (capture/'audit.json').write_text(json.dumps(dict(components=components)))
            ledger=root/'ledger.jsonl';ledger.write_text(json.dumps(dict(id='a',status='accepted_candidate',capture_root=str(capture)))+'\n')
            report=compare([ledger])['policies']
            self.assertEqual(report['lidar5']['usable_scenes'],0)
            self.assertEqual(report['lidar1_camera0']['kept_gt'],{'animal':1})

    def test_first_acceptance_stops_cell_and_park_failure_stops_all(self):
        for park_ok in (True,False):
            with tempfile.TemporaryDirectory() as d:
                root=Path(d);manifest=root/'manifest.json'
                rows=[dict(id='a'+str(i),source_cell_id='a') for i in range(3)]
                manifest.write_text(json.dumps(dict(scenes=rows)))
                calls=[]
                def child(cmd,**kwargs):
                    calls.append(cmd);dest=Path(cmd[cmd.index('--output')+1]);dest.mkdir(parents=True)
                    (dest/'shutdown_safety.json').write_text(json.dumps(dict(park_verified=park_ok)))
                    (dest/'ledger.jsonl').write_text(json.dumps(dict(id='a0',status='accepted_candidate'))+'\n')
                    return SimpleNamespace(returncode=0,stdout='',stderr='')
                with patch('base5_replenishment.subprocess.run',side_effect=child):
                    self.assertEqual(execute(manifest,root),0 if park_ok else 1)
                self.assertEqual(len(calls),1)
                self.assertEqual(json.loads((root/'report.json').read_text())['stage'],'complete' if park_ok else 'halted')


if __name__=='__main__':unittest.main()
