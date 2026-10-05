import json
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import generate_base5_static as b5


class Base5StaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = b5.geo.load_json(b5.DEFAULT_CONFIG)
        cls.specs = b5.specifications(cls.config)

    def test_exact_independent_frame_plan(self):
        self.assertEqual(len(self.specs), 10000)
        self.assertEqual(Counter(r['family'] for r in self.specs), self.config['families'])
        self.assertEqual(self.specs, b5.specifications(self.config))
        self.assertEqual(len({r['seed'] for r in self.specs}), 10000)

    def test_independent_axes_and_complete_geometry_deck(self):
        for family in self.config['families']:
            rows = [r for r in self.specs if r['family'] == family]
            for key in ('role', 'topology', 'ego_offset_m', 'yaw_delta_deg'):
                if key == 'topology' and family == 'far_vehicle':
                    continue  # Shared resolver independently schedules three legal highway lanes.
                self.assertGreater(len({r[key] for r in rows}), 1)
            if family != 'background':
                self.assertEqual({r['distance_bin'] for r in rows}, {0, 1, 2})

    def test_guard_rejects_bulk_before_loading_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            for start, count in ((0, 10000), (-1, 12), (9999, 12), (0, 0)):
                with self.assertRaises(ValueError):
                    b5.generate_batch(self.config, Path(folder), start, count)
            self.assertEqual(list(Path(folder).iterdir()), [])

    def test_geographic_split_keeps_variants_together(self):
        for i in range(50):
            group = 'link:{}:{}'.format(i, i+1)
            self.assertEqual(b5.split_for_group(group, self.config),
                             b5.split_for_group(group, self.config))
        changed = dict(self.config, captures_per_scene=3)
        with self.assertRaises(ValueError):
            b5.specifications(changed)

    def test_small_real_mr_batch_and_preservation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = b5.generate_batch(self.config, Path(folder), 0, 64)
            manifest = json.loads(path.read_text())
            self.assertEqual(len(manifest['scenes']), 64)
            self.assertEqual(manifest['capture_policy']['captures_per_scene'], 1)
            families = set()
            for row in manifest['scenes']:
                families.add(row['mission_family'])
                scenario = json.loads(Path(row['scenario']).read_text())
                metadata = json.loads(Path(row['metadata']).read_text())
                for actor in metadata['runtime_actors']:
                    if actor['name'].startswith('OBJ_'):
                        self.assertEqual(actor['class'], 'obstacle')
                        self.assertEqual(actor['training_class'], 'vehicle')
                        self.assertEqual(actor['source_list'], 'objectList')
                self.assertEqual(scenario['spawnPointList'], [])
                self.assertEqual(scenario['egoVehicle']['currentVelocity'], 0.)
                self.assertEqual(scenario['egoVehicle']['initPosition']['vehicleSaveData']['velocity'], 0.)
                for actor in scenario['pedestrianList']:
                    self.assertEqual(actor['speed'], 0.)
                for actor in scenario['objectList']:
                    self.assertEqual(actor['scale'], dict(x=1., y=1., z=1.))
                for actor in scenario['vehicleList']:
                    self.assertEqual(actor['constantVelocity'], 0.)
            self.assertEqual(families, set(self.config['families']))
            with self.assertRaises(FileExistsError):
                b5.generate_batch(self.config, Path(folder), 0, 24)


if __name__ == '__main__':
    unittest.main()
