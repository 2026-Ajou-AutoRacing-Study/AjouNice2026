import copy
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import revise_base5_dynamic_vehicle_mission as revision


class DynamicVehicleMissionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=gen.geo.load_json(gen.CONFIG)
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)

    def test_only_twelve_common_person_mission_slots_change(self):
        old=copy.deepcopy(self.config);old['mission_dynamic_observation'].pop('vehicle_primary_scenes')
        before={r['index']:r for r in gen.specs(old)}
        after=gen.specs(self.config);chosen=[r for r in after if r.get('dynamic_mission_pool')]
        self.assertEqual(len(chosen),12)
        self.assertEqual(Counter(r['dynamic_observation_phase'] for r in chosen),
            dict(approach=6,crossing=3,yielding=2,clearing=1))
        for row in after:
            previous=before[row['index']]
            if row.get('dynamic_mission_pool'):
                self.assertEqual(previous['subfamily'],'person')
                for key in ('index','seed','actors','dynamic_observation_phase'):
                    self.assertEqual(row[key],previous[key])
                self.assertEqual(row['distance_bounds'],[8,20] if row['dynamic_observation_phase']=='yielding' else previous['distance_bounds'])
            else:self.assertEqual(row,previous)
        self.assertEqual(after,gen.specs(self.config))

    def test_all_twelve_build_on_roads_without_motion(self):
        names=[]
        for row in gen.specs(self.config):
            if not row.get('dynamic_mission_pool'):continue
            for attempt in range(512):
                try:scene,meta=gen.build(row,self.config,self.ctx,attempt);break
                except ValueError:continue
            else:self.fail('Unfillable vehicle mission '+str(row['index']))
            names.append(meta['runtime_actors'][0]['name'])
            self.assertEqual(meta['spec']['asset_index'],row['asset_index'])
            for retry in range(37,549):
                try:_,retried=gen.build(meta['spec'],self.config,self.ctx,retry);break
                except ValueError:continue
            else:self.fail('Retry lost vehicle quota')
            self.assertEqual(retried['runtime_actors'][0]['data_id'],meta['runtime_actors'][0]['data_id'])
            self.assertFalse(scene['waypointDataList'])
            self.assertEqual(meta['details']['spatial_stratum'],'dynamic_obstacle')
            self.assertIn(meta['details']['dynamic_observation']['target_center']['road_pivot']['link_id'],self.ctx['links'])
            for actor in meta['runtime_actors']:
                self.assertEqual(actor['source_list'],'vehicleList')
                self.assertEqual(actor['motion'],'static');self.assertNotIn('motion_path',actor)
            center=meta['runtime_actors'][0]['geometric_center_estimate'];target=meta['details']['dynamic_observation']['target_center']
            self.assertAlmostEqual(center[0],target['x']);self.assertAlmostEqual(center[1],target['y'])
        self.assertEqual(len(set(names)),12)

    def test_invalid_quota_is_rejected(self):
        for count in (-1,True,251):
            config=copy.deepcopy(self.config);config['mission_dynamic_observation']['vehicle_primary_scenes']=count
            with self.assertRaises(ValueError):gen.specs(config)

    def test_revision_never_overwrites_or_accepts_unrelated_change(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.json'
            gen.geo.write_json(source,dict(config=self.config,scenes=[]))
            with self.assertRaises(FileExistsError):revision.revise(source,root,self.config)
            config=copy.deepcopy(self.config);config['seed']+=1
            with self.assertRaises(ValueError):revision.revise(source,root/'new',config)
            self.assertFalse((root/'new').exists())


if __name__=='__main__':unittest.main()
