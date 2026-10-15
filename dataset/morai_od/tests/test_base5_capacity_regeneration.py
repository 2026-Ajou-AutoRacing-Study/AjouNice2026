import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_capacity_pilot as pilot
from base5_capture_safety import atomic_json,sha256


class CapacityRegenerationTest(unittest.TestCase):
    def test_bounded_regeneration_preserves_primary_split_spec_and_uses_current_policy(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);meta=dict(spec=dict(index=9),retry=0,split='val',runtime_actors=[dict(data_id=42,motion='static')])
            atomic_json(root/'meta.json',meta)
            row=dict(id='scene',metadata=str(root/'meta.json'),scenario_sha256='old',family='obstacle',role='front',motion='static',split='val',distance_bounds=[3,10])
            atomic_json(root/'manifest.json',dict(scenes=[row],config=dict(capture=dict(natural_setup_budget_sec=6))))
            def build(spec,config,ctx,retry):
                self.assertEqual(spec,meta['spec']);self.assertEqual(config['capture']['dynamic_path_policy'],'natural-target-v1')
                self.assertEqual(config['capture']['natural_setup_budget_sec'],6)
                return {'test':True},meta.copy()
            with patch.object(pilot,'COUNTS',{'obstacle':1}),patch.object(pilot.gen,'context',return_value={}),patch.object(pilot.gen,'build',side_effect=build):
                result=pilot.prepare(root/'manifest.json',root/'out',regenerate_current=True)
            deck=json.loads(result.read_text());self.assertEqual(deck['count'],1)
            entry=deck['scenes'][0];self.assertEqual(entry['scenario_sha256'],sha256(Path(entry['scenario'])))
            self.assertEqual(json.loads(Path(entry['metadata']).read_text())['split'],'val')


if __name__=='__main__':unittest.main()
