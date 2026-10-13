import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_batch_cost as cost


class CostTest(unittest.TestCase):
    def fixture(self,p):
        cost.atomic_json(p/'status.json',dict(stage='complete',elapsed_sec=30))
        cost.atomic_json(p/'quota.json',dict(target=1,accepted=1))
        manifest=p/'manifest.json';cost.atomic_json(manifest,dict(config=dict(families=dict(highway=10000))))
        selected=dict(id='a_ground',source_cell_id='a',family='highway')
        (p/'selected_ledger.jsonl').write_text(json.dumps(selected)+'\n')
        batch=p/'batch';batch.mkdir()
        cost.atomic_json(batch/'session_contract.json',{})
        cost.atomic_json(batch/'shutdown_safety.json',dict(park_verified=True))
        row=dict(id='a',family='highway',duration_sec=10,bytes=2,between_scene_park=dict(park_verified=True))
        (batch/'ledger.jsonl').write_text(json.dumps(row)+'\n'+json.dumps(row)+'\n')
        child=p/'retry';child.mkdir()
        cost.atomic_json(child/'session_contract.json',{})
        cost.atomic_json(child/'shutdown_safety.json',dict(park_verified=True))
        (child/'ledger.jsonl').write_text(json.dumps(dict(row,id='a_ground'))+'\n')
        cost.atomic_json(p/'report.json',dict(attempts=[dict(kind='seeded_batch_review',capture_root=str(batch),reused_raw=True),dict(kind='input_z_retry',capture_root=str(child))]))
        return manifest,batch

    def test_count_actual_ledgers_not_seed_or_duplicate_resume_row(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);manifest,batch=self.fixture(p);r=cost.report(p,manifest)
            self.assertEqual(r['actual_capture_attempts'],2)
            self.assertEqual(r['scene_duration_sec'],20)
            self.assertEqual(r['parent_startup_preflight_boundary_shutdown_and_other_sec'],10)
            self.assertAlmostEqual(r['extrapolated_10k_hours_family_weighted'],300000/3600)
            self.assertTrue(r['all_quota_and_shutdown_verified'])

    def test_unsafe_shutdown_forbids_estimate(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);manifest,batch=self.fixture(p)
            cost.atomic_json(batch/'shutdown_safety.json',dict(park_verified=False))
            r=cost.report(p,manifest)
            self.assertFalse(r['all_quota_and_shutdown_verified'])
            self.assertIsNone(r['extrapolated_10k_hours_family_weighted'])

    def test_short_error_is_not_a_timeout(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);manifest,batch=self.fixture(p)
            (batch/'event_cmd_requests.jsonl').write_text(json.dumps(dict(error='handler returned None',worker_still_alive=False))+'\n')
            r=cost.report(p,manifest)
            self.assertEqual(r['event_requests'],1)
            self.assertEqual(r['event_timeouts'],0)
            self.assertEqual(r['event_workers_alive'],0)

    def test_explicit_preserved_wall_and_invalid_value(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);manifest,batch=self.fixture(p)
            cost.atomic_json(p/'status.json',dict(stage='complete',elapsed_sec=1))
            self.assertEqual(cost.report(p,manifest,30)['elapsed_sec'],30)
            for value in (-1,float('nan'),float('inf')):
                with self.assertRaises(ValueError):cost.report(p,manifest,value)


if __name__=='__main__':unittest.main()
