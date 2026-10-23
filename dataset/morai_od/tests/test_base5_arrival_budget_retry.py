import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_quality_loop as loop


def safe_miss():
    return dict(id='a',status='failed',scenario_sha256='hash',
        error='Natural target arrival not observed for all moving actors; no capture',
        no_capture_safety=dict(no_capture_requested=True,topic_freshness_verified=True,
            actor_presence_verified=True,ego_stationary_verified=True,actor_z_steps_verified=True,
            collision=dict(no_collision_verified=True)))


class ArrivalTest(unittest.TestCase):
    def test_only_safe_six_second_target_miss_is_eligible(self):
        with tempfile.TemporaryDirectory() as d:
            m=Path(d)/'manifest.json';loop.atomic_json(m,dict(config=dict(capture=dict(natural_setup_budget_sec=6))))
            row=safe_miss();self.assertTrue(loop.arrival_retry_eligible(row,m))
            for key in ('no_capture_requested','topic_freshness_verified','actor_presence_verified','ego_stationary_verified','actor_z_steps_verified'):
                bad=copy.deepcopy(row);bad['no_capture_safety'][key]=False
                self.assertFalse(loop.arrival_retry_eligible(bad,m))
            for error in ('Ego/Object/CollisionData connection stale','motion_vertical_jump'):
                self.assertFalse(loop.arrival_retry_eligible(dict(row,error=error),m))
            bad=copy.deepcopy(row);bad['no_capture_safety']['collision']['no_collision_verified']=False
            self.assertFalse(loop.arrival_retry_eligible(bad,m))
            loop.atomic_json(m,dict(config=dict(capture=dict(natural_setup_budget_sec=9))))
            self.assertFalse(loop.arrival_retry_eligible(row,m))

    def test_miss_evidence_is_hash_and_ledger_bound(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);m=p/'manifest.json';loop.atomic_json(m,dict(config=dict(capture=dict(natural_setup_budget_sec=6))))
            row=safe_miss();e=p/'failure.json'
            loop.atomic_json(e,dict(reason='natural_targets_not_simultaneously_observed',scenario_sha256='hash',no_capture_safety=row['no_capture_safety']))
            row.update(failure_evidence=str(e),failure_evidence_sha256=loop.sha256(e))
            loop.validate_miss_evidence(row,m)
            loop.atomic_json(e,{})
            with self.assertRaises(ValueError):loop.validate_miss_evidence(row,m)

    def test_failed_seed_recaptures_once_with_nine_second_budget(self):
        class Child:
            returncode=0
            def poll(self):return 0
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=p/'manifest.json';row=safe_miss()
            metadata=p/'metadata.json'
            loop.atomic_json(metadata,dict(calibration_only=False,runtime_actors=[]))
            loop.atomic_json(source,dict(config=dict(capture=dict(natural_setup_budget_sec=6)),scenes=[dict(id='a',metadata=str(metadata),scenario_sha256='hash')]))
            def prepare(source,indices,out,budget):
                self.assertEqual(budget,9);out.mkdir();deck=loop.gen.geo.load_json(source)
                deck['config']['capture']['natural_setup_budget_sec']=9
                loop.atomic_json(out/'manifest.json',deck);return out/'manifest.json'
            def capture(command,**kwargs):
                out=Path(command[command.index('--output')+1]);out.mkdir(exist_ok=True)
                loop.atomic_json(out/'shutdown_safety.json',dict(park_verified=True))
                raw=out/'captures'/'a';raw.mkdir(parents=True)
                loop.atomic_json(raw/'metadata.json',loop.gen.geo.load_json(metadata))
                (out/'ledger.jsonl').write_text(json.dumps(dict(id='a',status='accepted_candidate',capture_root=str(raw)))+'\n')
                return Child()
            with patch.object(loop,'validate_seed',return_value=(row,p/'batch',dict(park_verified=True))),patch.object(loop,'budget_retry',side_effect=prepare),patch.object(loop.subprocess,'Popen',side_effect=capture) as child:
                self.assertEqual(loop.execute(source,[0],p/'out',seed_capture=p/'seed.json'),0)
                self.assertEqual(child.call_count,1)
            report=loop.gen.geo.load_json(p/'out/report.json')
            self.assertEqual([a['kind'] for a in report['attempts']],['seeded_batch_target_miss','natural_budget9_retry'])

    def test_nine_second_failure_halts_instead_of_replacing_forever(self):
        class Child:
            returncode=0
            def poll(self):return 0
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=p/'manifest.json';row=safe_miss()
            loop.atomic_json(source,dict(config=dict(capture=dict(natural_setup_budget_sec=6)),scenes=[dict(id='a',scenario_sha256='hash')]))
            def prepare(source,indices,out,budget):
                out.mkdir();deck=loop.gen.geo.load_json(source);deck['config']['capture']['natural_setup_budget_sec']=9
                loop.atomic_json(out/'manifest.json',deck);return out/'manifest.json'
            def capture(command,**kwargs):
                out=Path(command[command.index('--output')+1]);out.mkdir(exist_ok=True)
                loop.atomic_json(out/'shutdown_safety.json',dict(park_verified=True))
                (out/'ledger.jsonl').write_text(json.dumps(row)+'\n');return Child()
            with patch.object(loop,'validate_seed',return_value=(row,p/'batch',dict(park_verified=True))),patch.object(loop,'budget_retry',side_effect=prepare),patch.object(loop.subprocess,'Popen',side_effect=capture) as child,patch.object(loop,'replacement') as replace:
                self.assertEqual(loop.execute(source,[0],p/'out',seed_capture=p/'seed.json'),1)
                self.assertEqual(child.call_count,1);replace.assert_not_called()
            self.assertEqual(loop.gen.geo.load_json(p/'out/report.json')['stage'],'halted')
