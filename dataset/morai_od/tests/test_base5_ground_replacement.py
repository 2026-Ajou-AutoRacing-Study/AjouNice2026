import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_quality_loop as loop


class GroundReplacementTests(unittest.TestCase):
    def test_moving_witness_checks_target_not_departure(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);actor=dict(unique_id=8500,data_id=1,motion='walking',position=dict(x=-9,y=0,z=28),
                motion_path=dict(target_position=dict(x=4,y=0,z=28)))
            meta=dict(retry=0,spec={},split='train',runtime_actors=[actor])
            loop.atomic_json(root/'meta.json',meta)
            loop.atomic_json(root/'manifest.json',dict(config={},scenes=[dict(id='a',metadata=str(root/'meta.json'),scenario_sha256='old')]))
            observed=[]
            def probe(links,record,**kwargs):observed.append(record['position']);return {}
            with patch.object(loop.gen,'context',return_value=dict(links={})), \
                 patch.object(loop.gen,'build',return_value=({},copy.deepcopy(meta))), \
                 patch('prepare_base5_empty_ground_probe.target_probe_pose',side_effect=probe):
                output=loop.replacement(root/'manifest.json',root/'out',require_ground_observable=True)
            self.assertEqual(observed,[actor['motion_path']['target_position']])
            entry=json.loads(output.read_text())['scenes'][0]
            result=json.loads(Path(entry['metadata']).read_text())
            self.assertTrue(result['replacement']['ground_observation_preflight'])
            self.assertEqual(result['runtime_actors'][0]['data_id'],1)


if __name__=='__main__':unittest.main()
