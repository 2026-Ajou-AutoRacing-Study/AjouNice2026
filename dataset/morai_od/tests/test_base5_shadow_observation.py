import copy
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import base5_shadow_observation as shadow
from base5_intersection_placement import regional_motion
from base5_motion_ground import validate_motion_ground


class ShadowObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        cls.config=copy.deepcopy(gen.geo.load_json(gen.CONFIG))
        cls.config['capture']['dynamic_region']='intersection-only-v116'
        cls.config['capture']['static_vehicle_policy']='park-static-snapshot-v119'
        cls.config['shadow_observation']=dict(policy=shadow.POLICY)
        cls.base_row=next(r for r in gen.specs(gen.geo.load_json(gen.CONFIG))
                     if r['family']=='pedestrian_related' and r['subfamily']=='person' and not r['mission'])

    def row(self,phase='interior',moving=False):
        row=copy.deepcopy(self.base_row)
        row.update(shadow_phase=phase,shadow_moving=moving,shadow_direction_sign=1,
            population_mode='shadow_situation',population_layout='road_corridor',actors=1,
            yaw_limit=3,role='front',distance_bounds=[30,60],motion='walking' if moving else 'static')
        return row

    def build(self,row):
        for retry in range(512):
            try:return gen.build(row,self.config,self.ctx,retry)
            except ValueError:pass
        self.fail('Unfillable shadow test')

    def test_default_generation_unchanged_without_opt_in(self):
        config=gen.geo.load_json(gen.CONFIG);first=gen.specs(config);second=gen.specs(config)
        self.assertEqual(first,second)
        self.assertFalse(any(r.get('shadow_phase') for r in first))

    def test_frozen_overrides_and_protected_cells(self):
        rows=gen.specs(gen.geo.load_json(gen.CONFIG));row=self.row()
        config=dict(shadow_observation=dict(policy=shadow.POLICY,scenes=1,spec_overrides=[row]))
        shadow.assign(rows,config);self.assertEqual(rows[row['index']],row)
        mission=next(r for r in rows if r['mission'])
        with self.assertRaisesRegex(ValueError,'Protected'):
            shadow.assign(rows,dict(shadow_observation=dict(policy=shadow.POLICY,scenes=1,spec_overrides=[mission])))

    def test_unknown_and_unfrozen_policy_refused(self):
        with self.assertRaises(ValueError):shadow.assign([],dict(shadow_observation=dict(policy='bad')))
        with self.assertRaisesRegex(ValueError,'frozen'):
            shadow.assign([],dict(shadow_observation=dict(policy=shadow.POLICY)))

    def test_all_three_anchor_phases_follow_original_route(self):
        start,end,span=shadow.bounds(gen,self.ctx)
        self.assertGreater(span,90.)
        for phase in ('entrance','interior','exit'):
            _,_,base,details=shadow.anchor(gen,self.ctx,self.row(phase),self.config,random.Random(1))
            self.assertEqual(details['shadow_observation']['phase'],phase)
            self.assertTrue(start-15<=base['route_arc_m']<=end+15)
            self.assertIn('not a measured',details['shadow_observation']['limitation'])

    def test_static_snapshot_has_no_waypoints_and_preserves_asset(self):
        scene,meta=self.build(self.row())
        self.assertFalse(scene['waypointDataList']);self.assertFalse(scene['spawnPointList'])
        self.assertEqual(meta['runtime_actors'][0]['motion'],'static')
        self.assertEqual(meta['details']['spatial_stratum'],'tunnel_interior')
        self.assertTrue(meta['ego_ground_initializer']['road_supported'])

    def test_shadow_mover_never_borrows_intersection_height_proof(self):
        scene,meta=self.build(self.row(moving=True))
        actor=meta['runtime_actors'][0]
        self.assertNotIn(actor['motion'],('static','standing'))
        self.assertEqual(len(actor['motion_path']['points']),2)
        self.assertNotIn('ground_path_evidence',actor['motion_path'])
        with self.assertRaisesRegex(ValueError,'independent ground'):
            validate_motion_ground(meta,scene)

    def test_ordinary_regions_keep_prior_motion_restriction(self):
        config=copy.deepcopy(self.config)
        asset=self.ctx['pools']['person'][0]
        self.assertEqual(regional_motion(asset,'walking',dict(mission=False),config)[0],'standing')
        self.assertIsNone(regional_motion(asset,'walking',self.row(moving=True),config))
        self.assertEqual(regional_motion(asset,'walking',self.row(),config)[0],'standing')

    def test_rigid_asset_never_moves_from_shadow_flag(self):
        row=self.row(moving=True);row.update(shadow_primary_pool='obstacle',shadow_primary_index=0,
                                          shadow_phase='entrance')
        scene,meta=self.build(row)
        self.assertEqual(meta['runtime_actors'][0]['motion'],'static')
        self.assertFalse(scene['waypointDataList'])


if __name__=='__main__':unittest.main()
