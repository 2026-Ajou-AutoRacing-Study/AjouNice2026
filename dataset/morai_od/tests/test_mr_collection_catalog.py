import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_dynamic_object_capture_set as generator


class MRCollectionCatalogTest(unittest.TestCase):
    def test_mr_source_pools_restore_assets_without_personal_files(self):
        catalog = generator.load_json(generator.DEFAULT_ASSET_CATALOG)
        config = generator.load_json(generator.DEFAULT_ASSET_COMPATIBILITY)
        self.assertEqual(generator.runtime_catalog_source(config), 'competition')
        self.assertEqual(config['policy']['capture_runtime'], 'competition_launcher')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for kind, field in [('vehicle', 'vehicles'), ('pedestrian', 'pedestrians_and_related'),
                                ('obstacle', 'obstacles_and_related')]:
                # Retain catalog source_list, including MR aiPedstrianList.
                document = {}
                for row in catalog[field]:
                    document.setdefault(row['source_list'], []).append({'DataID': row['data_id']})
                name = config['competition']['generated_from'][kind]['file_name']
                (root/name).write_text(json.dumps(document))
            args = (generator.DEFAULT_ASSET_CATALOG, generator.DEFAULT_ASSET_COMPATIBILITY, root)
            vehicle = generator.load_runtime_vehicle_pool(*args)
            ped = generator.load_runtime_pedestrian_pool(*args)
            obstacle = generator.load_runtime_obstacle_pool(*args)
            self.assertEqual(len(vehicle), 43)
            self.assertEqual(len(ped), 28)
            self.assertEqual(len(obstacle), 31)
            names = {row['name'] for row in vehicle+ped+obstacle}
            self.assertTrue({'2017_Hyundai_Grandeur', 'police_f_1', 'Boy2',
                             'Scooter_01', 'WoodBox'}.issubset(names))
            self.assertNotIn('Speed_Bump', names)
            self.assertFalse(any(name.lower().startswith('tree') for name in names))

    def test_historical_profile_is_still_explicitly_replayable(self):
        path = generator.PROJECT_ROOT/'dataset/morai_od/configs/morai_asset_launcher_compatibility_260807.json'
        self.assertEqual(generator.runtime_catalog_source(generator.load_json(path)), 'personal')
