import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from generate_base5_visibility_probe import generate


class VisibilityProbeTest(unittest.TestCase):
    def test_small_static_deck_and_metadata_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = json.loads(generate(Path(folder)).read_text())
            self.assertEqual(len(manifest['scenes']), 7)
            self.assertEqual(manifest['capture_policy']['captures_per_scene'], 3)
            for row in manifest['scenes']:
                scene = json.loads(Path(row['scenario']).read_text())
                self.assertLessEqual(len(row['file_name']), 8)
                self.assertEqual(scene['spawnPointList'], [])
                metadata = json.loads(Path(row['metadata']).read_text())
                self.assertEqual(metadata['runtime_actors'][0]['motion'], 'static')
                self.assertEqual(scene['egoVehicle']['initPosition']['vehicleSaveData']['velocity'], 0)


if __name__ == '__main__':
    unittest.main()
