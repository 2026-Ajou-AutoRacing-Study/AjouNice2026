import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_readiness_pilot as pilot
from base5_capture_safety import atomic_json


class ReadinessContractTest(unittest.TestCase):
    def test_regeneration_keeps_natural_target_and_source_setup_budget(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            atomic_json(root/'meta.json',dict(retry=0,spec={'index':0}))
            atomic_json(root/'manifest.json',dict(config=dict(capture=dict(
                dynamic_path_policy='one-way-slow-v1',natural_setup_budget_sec=6)),scenes=[dict(
                id='sample',metadata=str(root/'meta.json'))]))
            def build(spec,config,ctx,retry):
                self.assertEqual(config['capture']['dynamic_path_policy'],'natural-target-v1')
                self.assertTrue(config['capture']['asset_appearance_policy'])
                self.assertEqual(config['capture']['natural_setup_budget_sec'],6)
                return {},dict(runtime_actors=[dict(motion='walking')])
            with patch.object(pilot.gen,'context',return_value={}),patch.object(pilot.gen,'build',side_effect=build):
                path=pilot.prepare(root/'manifest.json',root/'out',[0],1)
            self.assertEqual(json.loads(path.read_text())['count'],1)


if __name__=='__main__':unittest.main()
