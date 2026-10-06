import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_quality_loop as loop


class SeedTest(unittest.TestCase):
    def fixture(self,root):
        session=root/'batch';capture=session/'captures/a';capture.mkdir(parents=True)
        audit=dict(review_reasons=['actor_annotation_above_ground'])
        loop.atomic_json(capture/'audit.json',audit);loop.atomic_json(capture/'metadata.json',{})
        loop.atomic_json(session/'shutdown_safety.json',dict(park_verified=True))
        loop.atomic_json(session/'session_contract.json',dict(runner_options=dict(shared_ros_batch=True)))
        seed=root/'seed.json';loop.atomic_json(seed,dict(capture_root=str(session),manifest=str(root/'manifest.json'),id='a'))
        row=dict(id='a',scenario_sha256='hash',status='review',audit=audit,capture_root=str(capture),between_scene_park=dict(park_verified=True))
        original=dict(id='a',scenario_sha256='hash')
        return seed,session,capture,row,original

    def test_safe_review_reused_without_moving_raw(self):
        with tempfile.TemporaryDirectory() as d:
            seed,session,capture,row,original=self.fixture(Path(d))
            with patch.object(loop,'prepare_session',return_value=([row],{})):
                accepted,source,safety=loop.validate_seed(seed,original)
            self.assertEqual(source,session);self.assertEqual(accepted,row);self.assertIsNot(accepted,row)
            self.assertTrue((capture/'audit.json').exists());self.assertTrue(safety['park_verified'])

    def test_wrong_cell_or_payload_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            seed,session,capture,row,original=self.fixture(Path(d))
            with patch.object(loop,'prepare_session',return_value=([row],{})):
                for changed in [dict(original,id='b'),dict(original,scenario_sha256='different')]:
                    with self.assertRaises(ValueError):loop.validate_seed(seed,changed)

    def test_unsafe_final_or_boundary_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            seed,session,capture,row,original=self.fixture(Path(d))
            loop.atomic_json(session/'shutdown_safety.json',dict(park_verified=False))
            with self.assertRaisesRegex(ValueError,'final Park'):loop.validate_seed(seed,original)
            loop.atomic_json(session/'shutdown_safety.json',dict(park_verified=True));row['between_scene_park']['park_verified']=False
            with patch.object(loop,'prepare_session',return_value=([row],{})):
                with self.assertRaises(ValueError):loop.validate_seed(seed,original)

    def test_calibration_or_changed_audit_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            seed,session,capture,row,original=self.fixture(Path(d))
            with patch.object(loop,'prepare_session',return_value=([row],{})):
                loop.atomic_json(capture/'metadata.json',dict(calibration_only=True))
                with self.assertRaisesRegex(ValueError,'Calibration'):loop.validate_seed(seed,original)
                loop.atomic_json(capture/'metadata.json',{});loop.atomic_json(capture/'audit.json',{})
                with self.assertRaisesRegex(ValueError,'audit differs'):loop.validate_seed(seed,original)

    def test_seed_goes_directly_to_one_corrected_recapture(self):
        class Child:
            returncode=0
            def poll(self):return 0
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json'
            loop.atomic_json(source,dict(scenes=[dict(id='a',scenario_sha256='hash')]))
            row=dict(id='a',status='review',audit=dict(review_reasons=['actor_annotation_above_ground']))
            def prepare(manifest,captures,out):
                out.mkdir();loop.atomic_json(out/'manifest.json',dict(scenes=[dict(id='a_ground')]))
                return out/'manifest.json'
            def capture(command,**kwargs):
                out=Path(command[command.index('--output')+1]);out.mkdir(exist_ok=True)
                loop.atomic_json(out/'shutdown_safety.json',dict(park_verified=True))
                (out/'ledger.jsonl').write_text(json.dumps(dict(id='a_ground',status='accepted_candidate'))+'\n')
                return Child()
            with patch.object(loop,'validate_seed',return_value=(row,root/'batch',dict(park_verified=True))),patch.object(loop,'height_retry',side_effect=prepare),patch.object(loop.subprocess,'Popen',side_effect=capture) as child:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed.json'),0)
                self.assertEqual(child.call_count,1)
                self.assertNotIn('/initial',str(child.call_args.args[0]))
            report=json.loads((root/'out/report.json').read_text())
            self.assertEqual([r['kind'] for r in report['attempts']],['seeded_batch_review','input_z_retry'])
            self.assertTrue(report['attempts'][0]['reused_raw'])


if __name__=='__main__':unittest.main()
