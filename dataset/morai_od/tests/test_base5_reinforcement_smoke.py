import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from generate_base5_reinforcement_smoke import generate


class ReinforcementSmokeTest(unittest.TestCase):
    def test_diversity_margin_and_no_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = json.loads(generate(Path(folder), 'diversity').read_text())
            self.assertEqual(len(manifest['scenes']), 56)
            far = [x['asset_probe'] for x in manifest['scenes'] if x['mission_family'] == 'highway_range']
            self.assertEqual({x['ego_arc'] for x in far}, {80., 120.})
            self.assertEqual(max(abs(x['forward']) for x in far), 55.)
            self.assertEqual(len({x['id'] for x in manifest['scenes']}), 56)

    def test_topology_static_speed_and_split_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = json.loads(generate(Path(folder)).read_text())
            self.assertEqual(len(manifest['scenes']), 27)
            self.assertEqual(sum(x['split'] == 'val' for x in manifest['scenes']), 6)
            self.assertIn('WoodBox', [x['asset_probe']['asset'] for x in manifest['scenes']])
            for row in manifest['scenes']:
                scene = json.loads(Path(row['scenario']).read_text())
                self.assertEqual(scene['spawnPointList'], [])
                self.assertEqual(scene['egoVehicle']['initPosition']['vehicleSaveData']['velocity'], 0.)
                if row['mission_family'] == 'highway_range':
                    self.assertIn(scene['egoVehicle']['initPosition']['initLink'], manifest['generation']['highway_link_path'])
                if scene['objectList']:
                    self.assertEqual(scene['objectList'][0]['scale'], dict(x=1., y=1., z=1.))


if __name__ == '__main__':
    unittest.main()
