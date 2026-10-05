import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_g80_placement_probe as probe


class ProbeTest(unittest.TestCase):
    def test_manual_actor_and_factorized_variants(self):
        manual=probe.geo.load_json(probe.assets.DEFAULT_RUNTIME/'new_obstacle_list.json')
        original=next(a for a in manual['objectList'] if a['DataID']==40100034)
        with tempfile.TemporaryDirectory() as d:
            path=probe.generate(Path(d))
            m=json.loads(path.read_text())
            self.assertEqual(len(m['scenes']),9)
            last=json.loads(Path(m['scenes'][-1]['scenario']).read_text())
            self.assertEqual(last['objectList'][0],original)
            for s in m['scenes']:
                doc=json.loads(Path(s['scenario']).read_text())
                self.assertEqual(len(doc['objectList']),2)
                self.assertEqual(doc['spawnPointList'],[])
                self.assertEqual(doc['egoVehicle']['currentVelocity'],0.)
                self.assertEqual(doc['objectList'][0]['DataID'],40100034)
            with self.assertRaises(FileExistsError):
                probe.generate(Path(d))
