import json
from pathlib import Path
import sys
import tempfile
import unittest
from collections import Counter

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
from run_base5_capture import bounded_call
from base5_capture_audit import camera_evidence,annotation_class
from select_base5_pilot import select
import numpy as np
from PIL import Image


class IndependentTest(unittest.TestCase):
    def test_ai_person_retained_without_invented_waypoint_motion(self):
        config=gen.geo.load_json(gen.CONFIG);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        row=gen.specs(config)[81]
        for attempt in range(512):
            try:scene,meta=gen.build(row,config,ctx,attempt);break
            except ValueError:continue
        else:self.fail('AI static cell unfillable')
        self.assertEqual(meta['runtime_actors'][0]['name'],'police1')
        self.assertEqual(meta['runtime_actors'][0]['motion'],'standing')
        self.assertNotIn('motion_path',meta['runtime_actors'][0])
        self.assertNotIn('isWaypointMode',scene['aiPedstrianList'][0])

    def test_bounded_motion_retains_distance_cell(self):
        import math
        for radius,bounds in ((3.5,[3,5]),(8.,[5,10]),(23.,[15,29])):
            pose=dict(x=radius,y=0.,z=0.)
            path=gen.bounded_motion_path(pose,[radius,0.],dict(x=0.,y=0.),bounds)
            self.assertEqual(path['points'][0],path['points'][-1])
            for i in range(101):
                t=i/100.;end=path['points'][1]
                r=math.hypot(radius+t*(end['x']-radius),t*end['y'])
                self.assertTrue(bounds[0]<=r<bounds[1])
        with self.assertRaises(ValueError):gen.bounded_motion_path(dict(x=4.99,y=0,z=0),[4.99,0],dict(x=0,y=0),[3,5])

    def test_avoidance_geometry_rotated_assets_and_both_sides(self):
        import random,math
        route=[dict(x=float(i),y=0.,z=0.,yaw=0.,link_idx='test',point_idx=i,point_ratio=i/200.) for i in range(201)]
        arc=list(range(201));base=dict(route[100],route_arc_m=100.)
        for size in ([.442,2.172,.92],[4.86,2.123,1.50],[12.,2.8,3.5]):
            ctx=dict(geometry={'test':dict(bbox_size_m=size,pivot_to_bbox_center_local_m=[1.,0.,.5])})
            asset=dict(name='test')
            for side in ('left','right'):
                row=dict(family='obstacle',avoidance_phase='alongside',avoidance_side=side,avoidance_gap_bounds_m=[.1,.2])
                ego,pose,ref=gen.avoidance.observation(gen,ctx,route,arc,base,asset,row,random.Random(11))
                a=math.radians(pose['yaw']);center=[pose['x']+math.cos(a),pose['y']+math.sin(a)]
                ea=math.radians(ego['yaw'])
                gap=gen.rectangle(center,size,pose['yaw']).distance(gen.rectangle(
                    [ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2.],ego['yaw']))
                self.assertAlmostEqual(gap,ref['requested_surface_gap_m'],places=7)
                self.assertFalse(ref['collider_clearance_verified'])
    def test_sample_corridor_endpoints_and_point_indices(self):
        links={r['idx']:r for r in gen.geo.load_json(gen.geo.DEFAULT_MGEO/'link_set.json')}
        for c in gen.sample_highway_corridors(links):
            for end,p in ((c['start'],c['route'][0]),(c['end'],c['route'][-1])):
                self.assertEqual(p['point_idx'],end['pointIdx'])
                self.assertEqual(p['link_idx'],end['linkIdx'])
                self.assertEqual([p[k] for k in ('x','y','z')],links[end['linkIdx']]['points'][end['pointIdx']])
                self.assertAlmostEqual(p['point_ratio'],end['pointIdx']/(len(links[end['linkIdx']]['points'])-1))
    def test_exact_quota_and_repeatability(self):
        c=gen.geo.load_json(gen.CONFIG);rows=gen.specs(c)
        self.assertEqual(rows,gen.specs(c));self.assertEqual(len(rows),10000)
        self.assertEqual(Counter(r['family'] for r in rows),c['families'])
        self.assertEqual(Counter(r['family'] for r in rows if r['mission']),{k:v for k,v in c['mission_counts'].items() if v})
        self.assertEqual(len({r['seed'] for r in rows}),10000)
        self.assertEqual(Counter(r['subfamily'] for r in rows if r['subfamily']),c['ped_subfamilies'])
        self.assertTrue(all(r['motion']=='standing' for r in rows if r['subfamily']=='dummy'))
        for family,total in (('obstacle',1200),('other_vehicle',100)):
            subset=[r for r in rows if r['family']==family and r['mission']]
            self.assertEqual(Counter(r['avoidance_phase'] for r in subset),
                dict(approach=total//5,entry=total//5,alongside=total*2//5,clearing=total//5))
            for phase in ('approach','entry','alongside','clearing'):
                sides=Counter(r['avoidance_side'] for r in subset if r['avoidance_phase']==phase)
                self.assertEqual(sides['left'],sides['right'])
        self.assertTrue(all('avoidance_phase' not in r for r in rows if not r['mission']))
    def test_real_batch_geometry_and_guard(self):
        c=gen.geo.load_json(gen.CONFIG)
        with tempfile.TemporaryDirectory() as d:
            path=gen.generate(c,d,0,48);m=gen.geo.load_json(path)
            for e in m['scenes']:
                meta=gen.geo.load_json(Path(e['metadata']));r=meta['spec']
                scene=gen.geo.load_json(Path(e['scenario']))
                if r['subfamily']=='dummy':
                    uid=meta['runtime_actors'][0]['unique_id']
                    actor=next(a for a in scene['pedestrianList'] if a['UNIQUEID']==uid)
                    self.assertEqual(actor['speed'],0.)
                    self.assertFalse(actor['isWaypointMode'])
                floor={3:0,10:3,15:10}[r['yaw_limit']]
                if not r.get('avoidance_phase'):self.assertGreaterEqual(abs(meta['ego_road_yaw_offset_deg']),floor)
                for i,a in enumerate(meta['runtime_actors']):
                    minimum=.10 if i==0 and r.get('avoidance_phase')=='alongside' else .3
                    self.assertGreaterEqual(a['planned_surface_clearance_m'],minimum-1e-6)
            with self.assertRaises(FileExistsError):gen.generate(c,d,0,1)
    def test_bounded_service_call(self):
        import time
        self.assertEqual(bounded_call(lambda:7),7)
        with self.assertRaises(RuntimeError):bounded_call(lambda:time.sleep(.1),.01)
    def test_pilot_selection_is_unique_and_bounded(self):
        rows=[dict(family='highway',subfamily=None,distance_bounds=[6,10],role='front',motion='static'),
              dict(family='obstacle',subfamily=None,distance_bounds=[2,5],role='left',motion='static'),
              dict(family='pedestrian_related',subfamily='animal',distance_bounds=[3,5],role='front',motion='walking')]
        selected=select(dict(scenes=rows),2)
        self.assertEqual(len(set(selected)),2)
        self.assertEqual(selected,select(dict(scenes=rows),2))
    def test_vehicle_annotation_subtypes(self):
        for name in ('Vehicle','SUV','Sedan','Truck','Bus','MPV','Wagon'):
            self.assertEqual(annotation_class(name),'vehicle')
        self.assertIsNone(annotation_class('UnrecognizedMesh'))
    def test_actual_pixels_child_matching_and_model_crop(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cam=root/'CAMERA_1';cam.mkdir()
            pixels=np.zeros((720,1280,4),dtype=np.uint8);pixels[300:340,200:240]=[0,41,0,255]
            Image.fromarray(pixels).save(cam/'a_Instance.png')
            (cam/'a_BBox2D.txt').write_text('Pedestrian 200 300 240 340 -950001 (0, 41, 0, 255)\n')
            self.assertEqual(camera_evidence(root,9500,'Pedestrian')[0]['native_instance_pixels'],1600)
            self.assertEqual(camera_evidence(root,9500,'Pedestrian')[0]['crop_320x896_instance_pixels'],784)
            self.assertEqual(camera_evidence(root,9501,'Pedestrian'),[])


if __name__=='__main__':unittest.main()
