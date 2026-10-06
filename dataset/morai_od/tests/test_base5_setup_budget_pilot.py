import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_setup_budget_pilot as pilot


class BudgetTest(unittest.TestCase):
    def fixture(self,p):
        scenario=p/'original.json';pilot.atomic_json(scenario,{})
        actor=dict(data_id=1,unique_id=2,motion='running',geometric_center_estimate=[2,3],estimated_size=[1,1,2],
            motion_path=dict(policy='natural-target-v1',target_position=dict(x=2,y=3,z=4),nominal_speed_mps=2.2,speed_field_value=7.92))
        meta=dict(spec=dict(index=0),ego=dict(x=0),split='train',retry=0,runtime_actors=[actor])
        mp=p/'meta.json';pilot.atomic_json(mp,meta)
        source=p/'source.json';pilot.atomic_json(source,dict(config=dict(capture={}),provenance={},scenes=[dict(id='a',metadata=str(mp),scenario=str(scenario),scenario_sha256=pilot.sha256(scenario))]))
        return source,meta

    def test_only_departure_budget_changes(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source,meta=self.fixture(p)
            with patch.object(pilot.gen,'context',return_value={}),patch.object(pilot.gen,'build',return_value=({},copy.deepcopy(meta))) as build:
                result=pilot.prepare(source,[0],p/'out',6)
            deck=pilot.gen.geo.load_json(result)
            self.assertEqual(deck['config']['capture']['natural_setup_budget_sec'],6)
            self.assertEqual(build.call_args.args[3],0)
            self.assertEqual(pilot.gen.geo.load_json(source)['config']['capture'],{})

    def test_speed_or_target_change_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source,meta=self.fixture(p);changed=copy.deepcopy(meta)
            changed['runtime_actors'][0]['motion_path']['nominal_speed_mps']=.2
            with patch.object(pilot.gen,'context',return_value={}),patch.object(pilot.gen,'build',return_value=({},changed)):
                with self.assertRaisesRegex(ValueError,'target/speed'):pilot.prepare(source,[0],p/'out',6)
            self.assertFalse((p/'out').exists())

    def test_unsafe_budget_and_bulk_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            for ids,budget in [([0],5),([0],10),([0,0],6),(list(range(9)),6)]:
                with self.assertRaises(ValueError):pilot.prepare(p/'none',ids,p/'out',budget)


if __name__=='__main__':unittest.main()
