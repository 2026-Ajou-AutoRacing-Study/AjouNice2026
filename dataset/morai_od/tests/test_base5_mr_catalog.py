import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import build_base5_mr_asset_catalog as assets


class CatalogTest(unittest.TestCase):
    def test_inventory_policy_and_containers(self):
        rows = json.loads(assets.DEFAULT_CATALOG.read_text())['assets']
        self.assertEqual(len(rows), 146)
        self.assertEqual(len({r['data_id'] for r in rows}), 146)
        static = [r for r in rows if r['name'].startswith('OBJ_')]
        self.assertEqual(len(static), 17)
        self.assertTrue(all(r['source_list'] == 'objectList' and r['semantic_class'] == 'vehicle' for r in static))
        for row in rows:
            if row.get('ontology_class') in ('Animal', 'Cyclist', 'CompositePersonAnimal'):
                self.assertEqual(row['collection_role'], 'review')
                self.assertIsNone(row['semantic_class'])
            if row['collection_role'] == 'roadside_context':
                self.assertIsNone(row['semantic_class'])
        pools = assets.load_pools()
        self.assertEqual({k: len(v) for k, v in pools.items()},
                         {'near_obstacle': 19, 'far_vehicle': 59, 'pedestrian': 28})
