import copy
import json
import math
import random
import sys
import tempfile
import unittest
from collections import Counter,defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_static_yaw import FINAL_POLICY,sample
from base5_placement_geometry import POLICY,configure
from base5_capture_safety import atomic_json,sha256
from calibrate_base5_parent_envelopes import parent_envelope
from revise_base5_final_appearance import assign_shadow_direction,assign_yaw,scope
import generate_base5_independent as gen


class FinalAppearanceTests(unittest.TestCase):
    def setUp(self):
        self.asset=dict(data_id=1,source_list='objectList',name='box')
        self.row=dict(seed=9,family='obstacle',mission=True,
            static_yaw_policy=dict(policy=FINAL_POLICY,scope='stationary_avoidance',eligible_data_ids=[1],
                                   obj_vehicle_data_ids=[],primary_mode='uniform',primary_sector=3))

    def test_retry_resamples_inside_same_continuous_sector(self):
        angles=[sample(self.asset,self.row,'static',0,23,a)['road_relative_yaw_deg'] for a in range(100)]
        self.assertTrue(all(-90<x<-60 for x in angles));self.assertEqual(len(set(angles)),100)
        self.assertEqual(sample(self.asset,self.row,'static',0,23,2),sample(self.asset,self.row,'static',0,23,2))

    def test_free_angles_cover_all_twelve_sectors(self):
        got=[]
        for sector in range(12):
            row=copy.deepcopy(self.row);row['static_yaw_policy']['primary_sector']=sector
            angle=sample(self.asset,row,'static',0,0)['road_relative_yaw_deg'];got.append(int((angle+180)//30))
        self.assertEqual(got,list(range(12)))

    def test_traffic_intersection_and_actual_motion_protected(self):
        for patch in (dict(family='highway'),dict(family='roundabout'),dict(dynamic_observation_phase='yielding'),
                      dict(shadow_moving=True)):
            self.assertIsNone(sample(self.asset,dict(self.row,**patch),'static',0,0))
        for motion in ('walking','running'):
            self.assertIsNone(sample(self.asset,self.row,motion,0,0))
        row=copy.deepcopy(self.row);row['static_yaw_policy']['scope']='generic'
        self.assertIsNone(sample(dict(self.asset,source_list='vehicleList'),row,'static',0,0))

    def test_road_prior_only_for_vehicle_assets(self):
        row=copy.deepcopy(self.row);row['static_yaw_policy']['primary_mode']='road_aligned'
        with self.assertRaises(ValueError):sample(self.asset,row,'static',0,0)
        for asset in (dict(self.asset,source_list='vehicleList'),self.asset):
            if asset['source_list']=='objectList':row['static_yaw_policy']['obj_vehicle_data_ids']=[1]
            result=sample(asset,row,'static',0,0)
            x=result['road_relative_yaw_deg']
            self.assertLessEqual(min(abs(x),abs(x-180)),15)

    def test_yaw_before_real_rectangle_gap(self):
        route=[dict(x=float(i),y=0.,z=0.,yaw=0.,link_idx='test',point_idx=i,point_ratio=i/200) for i in range(201)]
        ctx=dict(geometry={'box':dict(bbox_size_m=[3.,.5,1.],pivot_to_bbox_center_local_m=[.4,.1,.5])})
        for sector in range(12):
            row=copy.deepcopy(self.row);row.update(avoidance_phase='alongside',avoidance_side='left',avoidance_gap_bounds_m=[.3,.5])
            row['static_yaw_policy']['primary_sector']=sector
            ego,pose,proof=gen.avoidance.observation(gen,ctx,route,list(range(201)),dict(route[100],route_arc_m=100),
                self.asset,row,random.Random(7))
            a=math.radians(pose['yaw']);center=[pose['x']+.4*math.cos(a)-.1*math.sin(a),pose['y']+.4*math.sin(a)+.1*math.cos(a)]
            ea=math.radians(ego['yaw'])
            gap=gen.rectangle(center,[3.,.5,1.],pose['yaw']).distance(gen.rectangle(
                [ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2.],ego['yaw']))
            self.assertAlmostEqual(gap,proof['requested_surface_gap_m'],places=7)

    def test_parent_envelope_keeps_separate_component_extents(self):
        boxes=[dict(center=np.asarray([x,0.,1.]),size=np.asarray([1.,1.,2.]),rotation_rpy=[0.,0.,0.]) for x in (-1.,1.)]
        ego=dict(x=10.,y=20.,z=3.,heading=91.347)
        lo,hi=parent_envelope(boxes,np.eye(4),ego,[10.,20.,3.],90.)
        np.testing.assert_allclose(lo,[-1.5,-.5,0.],atol=1e-10)
        np.testing.assert_allclose(hi,[1.5,.5,2.],atol=1e-10)

    def test_geometry_is_opt_in_static_only_and_does_not_mutate_base_context(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'reference.json';asset=dict(bbox_size_m=[2.,1.,1.],pivot_to_bbox_center_local_m=[0.,0.,.5],
                                                  observations=2,stationary_only=True)
            atomic_json(path,dict(policy=POLICY,training_gt_modified=False,assets={'new':asset}))
            config=dict(static_placement_geometry=dict(policy=POLICY,path=str(path),sha256=sha256(path)))
            ctx=dict(geometry={'old':asset})
            got=configure(ctx,config,dict(family='obstacle',motion='static'))
            self.assertIn('new',got['geometry']);self.assertNotIn('new',ctx['geometry'])
            for motion in ('walking','running'):
                row=dict(family='pedestrian_related',motion=motion,
                         static_yaw_policy=dict(policy=FINAL_POLICY,scope='generic'))
                self.assertIn('new',configure(ctx,config,row)['geometry'])
                row['dynamic_observation_phase']='crossing'
                self.assertNotIn('new',configure(ctx,config,row)['geometry'])
            for patch in (dict(family='highway'),dict(shadow_moving=True),dict(dynamic_observation_phase='approach'),dict(motion='running')):
                row=dict(family='obstacle',motion='static');row.update(patch)
                self.assertNotIn('new',configure(ctx,config,row)['geometry'])
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'reference changed'):configure(ctx,config,dict(family='obstacle',motion='static'))

    def test_shadow_approach_recede_balanced_per_walk_run_and_retry_spec_stable(self):
        rows=[dict(shadow_moving=True,motion='walking' if i<12 else 'running',role='front' if i%3 else 'rear') for i in range(24)]
        metas=[dict(spec=copy.deepcopy(r)) for r in rows];assign_shadow_direction(rows,metas)
        for motion in ('walking','running'):
            self.assertEqual(Counter(r['shadow_relative_motion'] for r in rows if r['motion']==motion),
                             Counter(approaching=6,receding=6))
        for row in rows:
            sign=-1 if row['role']=='front' else 1
            self.assertEqual(row['shadow_direction_sign'],sign if row['shadow_relative_motion']=='approaching' else -sign)

    def test_static_scope_includes_shadow_but_not_crossing(self):
        base=dict(spec=dict(family='shadow',mission=False,shadow_phase='interior'),
                  runtime_actors=[dict(motion='static')])
        self.assertEqual(scope(base),'shadow_static')
        base['spec']['shadow_moving']=True;self.assertIsNone(scope(base))


if __name__=='__main__':unittest.main()
