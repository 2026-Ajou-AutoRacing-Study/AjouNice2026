import copy
import random
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import revise_base5_stationary_mission as revision


class StationaryMissionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=gen.geo.load_json(gen.CONFIG)
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)

    def test_only_static_obstacle_mission_specs_change(self):
        old=copy.deepcopy(self.config);old.pop('stationary_mission_assets')
        before={r['index']:r for r in gen.specs(old)}
        rows=gen.specs(self.config)
        self.assertEqual(rows,gen.specs(self.config))
        expected=Counter(self.config['stationary_mission_assets']['primary_pool_counts']);expected['static_vehicle']+=50
        self.assertEqual(Counter(r['stationary_mission_pool'] for r in rows if r.get('stationary_mission_pool')),expected)
        for r in rows:
            if r.get('stationary_mission_pool'):
                original=before[r['index']]
                for key in ('actors','avoidance_phase','avoidance_side','avoidance_gap_bounds_m','role','seed'):
                    self.assertEqual(r[key],original[key])
                self.assertEqual(r['motion'],'static')
            else:self.assertEqual(r,before[r['index']])

    def test_every_pool_builds_stationary_and_same_asset_on_retry(self):
        rows=gen.specs(self.config)
        for pool in self.config['stationary_mission_assets']['primary_pool_counts']:
            row=next(r for r in rows if r.get('stationary_mission_pool')==pool)
            expected=self.ctx['pools'][pool][row['asset_index']%len(self.ctx['pools'][pool])]
            for initial in (0,37):
                for attempt in range(initial,initial+512):
                    try:scene,meta=gen.build(row,self.config,self.ctx,attempt);break
                    except ValueError:continue
                else:self.fail('Unfillable pool '+pool)
                self.assertEqual(meta['runtime_actors'][0]['data_id'],expected['data_id'])
                self.assertEqual(meta['details']['spatial_stratum'],'static_obstacle')
                self.assertFalse(scene['waypointDataList'])
                for a in meta['runtime_actors']:
                    self.assertIn(a['motion'],('static','standing'))
                    self.assertNotIn('motion_path',a)
                    raw=next(t for t in scene[a['source_list']] if t['UNIQUEID']==a['unique_id'])
                    for key in ('speed','moveSpeed','customSpeed','currentVelocity'):
                        self.assertEqual(raw.get(key,0),0)
                    self.assertFalse(raw.get('isWaypointMode',False))
                    self.assertEqual(raw.get('initPosition',{}).get('vehicleSaveData',{}).get('velocity',0),0)

    def test_complete_asset_coverage_in_650_stationary_mission_cells(self):
        rows=gen.specs(self.config);seen=Counter()
        for r in rows:
            if r.get('stationary_mission_pool'):
                pool=self.ctx['pools'][r['stationary_mission_pool']]
            elif r['mission'] and r['family']=='other_vehicle':pool=self.ctx['pools']['static_vehicle']
            else:continue
            seen[pool[r['asset_index']%len(pool)]['data_id']]+=1
        eligible={a['data_id'] for key in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider') for a in self.ctx['pools'][key]}
        self.assertEqual(set(seen),eligible)
        self.assertEqual(len(eligible),122)
        self.assertGreaterEqual(min(seen.values()),3)
        self.assertEqual(sum(seen.values()),650)
        self.assertFalse(set(seen)&gen.catalog.COMPETITION_EXCLUDED_IDS)

    def test_reject_partial_or_mismatched_pool_plan(self):
        c=copy.deepcopy(self.config);del c['stationary_mission_assets']['primary_pool_counts']['animal']
        with self.assertRaises(ValueError):gen.specs(c)
        c=copy.deepcopy(self.config);c['stationary_mission_assets']['primary_pool_counts']['vehicle']+=1
        with self.assertRaises(ValueError):gen.specs(c)

    def test_revision_refuses_overwrite_and_unrelated_config_change(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.json'
            gen.geo.write_json(source,dict(config=self.config,scenes=[]))
            with self.assertRaises(FileExistsError):revision.revise(source,root,self.config)
            changed=copy.deepcopy(self.config);changed['seed']+=1
            with self.assertRaisesRegex(ValueError,'other source config must match'):
                revision.revise(source,root/'new',changed)
            self.assertFalse((root/'new').exists())


if __name__=='__main__':unittest.main()
