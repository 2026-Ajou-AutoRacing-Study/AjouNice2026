import copy
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
from revise_base5_ground_placement import revise
from audit_base5_scenarios import audit


class GroundRevisionTest(unittest.TestCase):
    def test_revision_preserves_source_xy_and_audit_rejects_height_tamper(self):
        config=gen.geo.load_json(gen.CONFIG);old=copy.deepcopy(config)
        old.pop('placement_ground',None);old['capture'].pop('static_quality_budget',None)
        with tempfile.TemporaryDirectory() as directory:
            start=next(r['index'] for r in gen.specs(old) if r['family']=='highway')
            root=Path(directory);source=gen.generate(old,root/'old',start,1)
            entry=gen.geo.load_json(source)['scenes'][0];digest=gen.geo.sha256(Path(entry['scenario']))
            report=revise(source,root/'new',config)
            self.assertTrue(report['xy_yaw_roles_splits_preserved'])
            self.assertEqual(gen.geo.sha256(Path(entry['scenario'])),digest)
            self.assertEqual(audit(root/'new/manifest.json')['errors'],[])
            new=gen.geo.load_json(root/'new/manifest.json')['scenes'][0]
            a=gen.geo.load_json(Path(entry['metadata']));b=gen.geo.load_json(Path(new['metadata']))
            self.assertEqual(a['split'],b['split']);self.assertEqual(a['spec'],b['spec'])
            for x,y in zip(a['runtime_actors'],b['runtime_actors']):
                self.assertEqual([x['position'][k] for k in ('x','y')],[y['position'][k] for k in ('x','y')])
            b['runtime_actors'][0]['map_ground_initializer']['height_m']+=.5
            gen.geo.write_json(Path(new['metadata']),b)
            self.assertTrue(any('height differs from sources' in e for _,e in audit(root/'new/manifest.json')['errors']))
            with self.assertRaises(FileExistsError):revise(source,root/'new',config)


if __name__=='__main__':unittest.main()
