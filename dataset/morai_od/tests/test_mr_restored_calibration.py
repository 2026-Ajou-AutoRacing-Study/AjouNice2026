import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_dynamic_object_capture_set as geo
from generate_mr_restored_asset_calibration import generate
from generate_base5_restored_replay import generate_replay


class RestoredCalibrationTest(unittest.TestCase):
    def test_reference_grid_uses_mr_assets_and_preserves_existing_output(self):
        catalog = geo.load_json(geo.DEFAULT_ASSET_CATALOG)
        profile = geo.load_json(geo.DEFAULT_ASSET_COMPATIBILITY)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for kind, field in [('pedestrian', 'pedestrians_and_related'), ('obstacle', 'obstacles_and_related')]:
                document = {}
                for row in catalog[field]:
                    document.setdefault(row['source_list'], []).append(dict(DataID=row['data_id']))
                (root/profile['competition']['generated_from'][kind]['file_name']).write_text(json.dumps(document))
            output = root/'deck'
            manifest = geo.load_json(generate(output, root))
            self.assertEqual(len(manifest['scenes']), 20)
            self.assertTrue(manifest['generation']['requires_lidar_instance_annotation'])
            for row in manifest['scenes']:
                scene = geo.load_json(Path(row['scenario']))
                self.assertEqual(scene['egoVehicle']['initPosition']['vehicleSaveData']['velocity'], 0.)
                self.assertEqual(scene['spawnPointList'], [])
                self.assertEqual(len(scene['objectList'])+len(scene['pedestrianList']), 1)
                if scene['objectList']:
                    self.assertEqual(scene['objectList'][0]['scale'], dict(x=1., y=1., z=1.))
                if scene['pedestrianList']:
                    self.assertEqual(scene['pedestrianList'][0]['speed'], 0.)
            names = {row['asset_probe']['asset'] for row in manifest['scenes']}
            self.assertEqual(names, {'WoodBox', 'CargoBox', 'police_f_1', 'Boy2', 'Scooter_01'})
            with self.assertRaises(FileExistsError):
                generate(output, root)
            replay_output = root/'replay'
            replay = geo.load_json(generate_replay(replay_output, root))
            self.assertEqual(len(replay['scenes']), 20)
            self.assertFalse(replay['generation']['requires_lidar_instance_annotation'])
            self.assertEqual(replay['capture_policy']['captures_per_scene'], 3)
            self.assertEqual(sum(row['split'] == 'val' for row in replay['scenes']), 5)
            self.assertTrue(all(row['id'].startswith('b5rt') for row in replay['scenes']))
            with self.assertRaises(FileExistsError):
                generate_replay(replay_output, root)
