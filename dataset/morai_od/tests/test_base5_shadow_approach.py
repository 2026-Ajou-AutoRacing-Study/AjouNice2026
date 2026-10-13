import copy
import sys
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import base5_shadow_observation as shadow
from revise_base5_shadow_approach import slots, SCHEDULE


class ApproachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        cls.config=copy.deepcopy(gen.geo.load_json(gen.CONFIG))
        cls.config['capture']['dynamic_region']='intersection-only-v116'
        cls.config['capture']['static_vehicle_policy']='park-static-snapshot-v119'
        cls.base=next(r for r in gen.specs(cls.config) if r['family']=='highway' and not r['mission'])
        cls.config['shadow_observation']=dict(policy=shadow.POLICY)

    def build(self,stage,interval,band):
        r=copy.deepcopy(self.base)
        r.update(shadow_phase='approach' if stage.startswith('approach') else stage,
            shadow_observation_stage=stage,shadow_moving=False,shadow_target_fraction_bounds=band,
            shadow_ego_lane_offset_m=0.,shadow_primary_lane_offset_m=0.,shadow_layout='same_lane',
            shadow_primary_pool='vehicle',shadow_primary_index=0,
            role='rear' if stage=='exit' else 'front',distance_bounds=interval,
            yaw_limit=3,motion='static',actors=1,population_mode='shadow_situation',population_layout='road_corridor')
        for retry in range(512):
            try:return gen.build(r,self.config,self.ctx,retry)
            except ValueError:pass
        self.fail('Unfillable approach test '+stage)

    def test_schedule_preserves_500_and_distance_totals(self):
        positive,empty=slots()
        self.assertEqual((len(positive),len(empty)),(456,20))
        bins=Counter(tuple(v) for _,v in positive);bins[(10,30)]+=24
        self.assertEqual(bins,{(3,10):40,(10,30):240,(30,60):160,(60,100):40})
        counts=Counter(st for st,_ in positive+empty);counts['interior']+=24
        self.assertEqual(counts,dict(approach_far=60,approach_near=60,approach_threshold=30,
                                    entrance=90,interior=190,exit=70))

    def test_all_stages_have_joint_inside_target_and_actual_ego_bounds(self):
        for st,interval,band in [('approach_far',[60,100],[.1,.4]),('approach_near',[30,60],[.1,.4]),
            ('approach_threshold',[10,30],[.1,.4]),('entrance',[10,30],[.1,.5]),
            ('interior',[10,30],[.2,.8]),('exit',[10,30],[.2,.9])]:
            with self.subTest(stage=st):
                scene,m=self.build(st,interval,band)
                proof=shadow.validate_approach(gen,self.ctx,m)
                self.assertEqual(proof['stage'],st)
                self.assertFalse(scene['waypointDataList'])
                self.assertGreaterEqual(proof['actual_actor_inside_fractions'][0],band[0]-.002)
                self.assertLessEqual(proof['actual_actor_inside_fractions'][0],band[1]+.002)
                self.assertTrue(m['ego_ground_initializer']['road_supported'])

    def test_projection_is_continuous_not_nearest_vertex(self):
        self.assertAlmostEqual(shadow.project_arc([dict(x=0,y=0),dict(x=10,y=0)], [0,10],[4,3],{}),4.)

    def test_independent_validation_refuses_tampered_final_xy(self):
        _,m=self.build('approach_far',[30,60],[.1,.4])
        start,end,_=shadow.bounds(gen,self.ctx)
        p=gen.geo.continuous_route_pose_at_arc(self.ctx['route'],self.ctx['arc'],end+20.)
        m['runtime_actors'][0]['geometric_center_estimate']=[p['x'],p['y']]
        with self.assertRaisesRegex(ValueError,'inside-target'):shadow.validate_approach(gen,self.ctx,m)

    def test_sensor_gate_never_accepts_placed_but_invisible_forward_target(self):
        meta=dict(spec=dict(shadow_observation_stage='approach_far'),runtime_actors=[dict(unique_id=8500)])
        c=dict(parent_uid=8500,positive_lidar1_camera0=True,center_lidar_m=[50,0,0],center_ego_m=[50,0,0],
               cameras=[],camera_component_ambiguous=False)
        self.assertEqual(shadow.visibility_review(meta,[c])[0],[])
        c['positive_lidar1_camera0']=False
        self.assertIn('shadow_approach_visible_forward_primary_deficit',shadow.visibility_review(meta,[c])[0])
        c['positive_lidar1_camera0']=True;c['center_ego_m']=[-5,0,0]
        self.assertTrue(shadow.visibility_review(meta,[c])[0])
        c['center_ego_m']=[50,0,0];c['center_lidar_m']=[101,0,0]
        self.assertTrue(shadow.visibility_review(meta,[c])[0])
        meta['calibration_only']=True;self.assertEqual(shadow.visibility_review(meta,[]),([],None))

    def test_no_change_to_legacy_shadow_visibility(self):
        self.assertEqual(shadow.visibility_review(dict(spec={},runtime_actors=[]),[]),([],None))

    def test_camera_only_and_ambiguous_components_keep_original_gt_policy(self):
        meta=dict(spec=dict(shadow_observation_stage='approach_far'),runtime_actors=[dict(unique_id=8500)])
        camera=dict(camera='CAMERA_1',crop_320x896_instance_pixels=16,model_extent_px=[4,5])
        c=dict(parent_uid=8500,positive_lidar1_camera0=True,center_lidar_m=[60,0,0],center_ego_m=[60,0,0],
               cameras=[camera],camera_component_ambiguous=False,lidar_points=0)
        reasons,proof=shadow.visibility_review(meta,[c])
        self.assertFalse(reasons);self.assertTrue(proof['front_camera_primary'])
        # A LiDAR-supported ambiguous component can remain positive, but cannot
        # certify a particular camera component's visibility.
        c['camera_component_ambiguous']=True;c['lidar_points']=1
        self.assertFalse(shadow.visibility_review(meta,[c])[1]['front_camera_primary'])


if __name__=='__main__':unittest.main()
