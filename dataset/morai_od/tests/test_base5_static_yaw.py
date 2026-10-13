import math
import sys
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_static_yaw import POLICY, sample


class StaticYawTests(unittest.TestCase):
    def setUp(self):
        self.asset=dict(data_id=1,source_list='pedestrianList')
        self.row=dict(seed=51,static_yaw_policy=dict(policy=POLICY,scope='generic',primary_mode='uniform'))

    def test_legacy_no_opt_in(self):
        self.assertIsNone(sample(self.asset,dict(seed=1),'static',0,0))

    def test_movers_and_vehicles_unchanged(self):
        for motion in ('walking','running'):
            self.assertIsNone(sample(self.asset,self.row,motion,0,0))
        self.assertIsNone(sample(dict(self.asset,source_list='vehicleList'),self.row,'static',0,0))

    def test_generic_object_uniform_kept(self):
        self.assertIsNone(sample(dict(self.asset,source_list='objectList'),self.row,'static',0,0))

    def test_stationary_object_compound_allowed_but_rigid_not(self):
        row=dict(self.row,static_yaw_policy=dict(policy=POLICY,scope='stationary_avoidance',eligible_data_ids=[1],primary_mode='uniform'))
        self.assertIsNotNone(sample(dict(self.asset,source_list='objectList'),row,'static',0,0))
        self.assertIsNone(sample(dict(self.asset,data_id=2),row,'static',0,0))

    def test_deterministic_and_road_covariant(self):
        a=sample(self.asset,self.row,'static',0,12)
        self.assertEqual(a,sample(self.asset,self.row,'static',0,12))
        b=sample(self.asset,self.row,'static',0,22)
        self.assertAlmostEqual(b['placement_yaw_deg']-a['placement_yaw_deg'],10)
        self.assertFalse(a['moving_path_alignment_changed'])

    def test_retry_preserves_mode_but_resamples_angle(self):
        a=sample(self.asset,self.row,'static',0,12,0)
        b=sample(self.asset,self.row,'static',0,12,1)
        self.assertEqual(a['mode'],b['mode'])
        self.assertNotEqual(a['road_relative_yaw_deg'],b['road_relative_yaw_deg'])
        self.assertEqual(b,sample(self.asset,self.row,'static',0,12,1))

    def test_cardinal_all_four_and_jitter(self):
        quadrants=Counter();angles=[]
        for seed in range(1000):
            row=dict(self.row,seed=seed,static_yaw_policy=dict(self.row['static_yaw_policy'],primary_mode='cardinal'))
            a=sample(self.asset,row,'standing',0,0)['road_relative_yaw_deg'];angles.append(a)
            q=round(a/90);quadrants[q%4]+=1
            self.assertLessEqual(abs(a-q*90),15)
        self.assertEqual(len(quadrants),4)
        self.assertGreater(min(quadrants.values()),180)
        self.assertGreater(sum(abs(x-round(x/90)*90)>5 for x in angles),500)

    def test_uniform_covers_all_octants(self):
        bins=Counter(int((sample(self.asset,dict(self.row,seed=s),'static',0,0)['road_relative_yaw_deg']+180)//45) for s in range(1000))
        self.assertEqual(len(bins),8)
        self.assertGreater(min(bins.values()),80)

    def test_unknown_policy_fails_closed(self):
        with self.assertRaises(ValueError):sample(self.asset,dict(self.row,static_yaw_policy=dict(policy='oops')),'static',0,0)

    def test_avoidance_solves_clearance_after_rotation(self):
        import random
        import generate_base5_independent as gen
        route=[dict(x=float(i),y=0.,z=0.,yaw=0.,link_idx='test',point_idx=i,point_ratio=i/200.) for i in range(201)]
        arc=list(range(201));base=dict(route[100],route_arc_m=100.)
        size=[2.4,.8,1.5];off=[.4,.2,.5]
        ctx=dict(geometry={'test':dict(bbox_size_m=size,pivot_to_bbox_center_local_m=off)})
        asset=dict(self.asset,name='test')
        for side in ('left','right'):
          for attempt in (0,9):
            row=dict(seed=9,family='obstacle',avoidance_phase='alongside',avoidance_side=side,
                avoidance_gap_bounds_m=[.3,.5],static_yaw_policy=dict(policy=POLICY,
                    scope='stationary_avoidance',eligible_data_ids=[1],primary_mode='uniform'))
            ego,pose,ref=gen.avoidance.observation(gen,ctx,route,arc,base,asset,row,random.Random(3),yaw_attempt=attempt)
            self.assertAlmostEqual(pose['yaw'],sample(asset,row,'static',0,0,attempt)['placement_yaw_deg'])
            a=math.radians(pose['yaw']);center=[pose['x']+math.cos(a)*off[0]-math.sin(a)*off[1],
                pose['y']+math.sin(a)*off[0]+math.cos(a)*off[1]]
            ea=math.radians(ego['yaw'])
            gap=gen.rectangle(center,size,pose['yaw']).distance(gen.rectangle(
                [ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2.],ego['yaw']))
            self.assertAlmostEqual(gap,ref['requested_surface_gap_m'],places=7)
