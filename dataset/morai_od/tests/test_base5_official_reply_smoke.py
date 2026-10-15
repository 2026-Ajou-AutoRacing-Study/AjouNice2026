import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_base5_official_reply_smoke import prepare_smoke
from audit_base5_scenarios import audit
import generate_base5_independent as gen


class OfficialReplySmokeTest(unittest.TestCase):
    def test_twelve_cells_have_valid_side_range_and_exclusion(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest = prepare_smoke(Path(temp) / 'pilot')
            result = audit(manifest)
            self.assertEqual(result['scenes'], 12)
            self.assertEqual(result['errors'], [])
            rows = gen.geo.load_json(manifest)['scenes']
            far = [r for r in rows if r['family'] == 'highway']
            self.assertEqual({r['role'] for r in far}, {'front', 'rear'})
            self.assertTrue(all(r['distance_bounds'] == [80, 99] for r in far))
            for row in rows:
                actors = gen.geo.load_json(Path(row['metadata']))['runtime_actors']
                self.assertNotIn(40100034, [a['data_id'] for a in actors])
