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
    def test_exact_quota_and_repeatability(self):
        c=gen.geo.load_json(gen.CONFIG);rows=gen.specs(c)
        self.assertEqual(rows,gen.specs(c));self.assertEqual(len(rows),10000)
        self.assertEqual(Counter(r['family'] for r in rows),c['families'])
        self.assertEqual(Counter(r['family'] for r in rows if r['mission']),{k:v for k,v in c['mission_counts'].items() if v})
        self.assertEqual(len({r['seed'] for r in rows}),10000)
        self.assertEqual(Counter(r['subfamily'] for r in rows if r['subfamily']),c['ped_subfamilies'])
        self.assertTrue(all(r['motion']=='standing' for r in rows if r['subfamily']=='dummy'))
        self.assertEqual(Counter(r['role'] for r in rows if r['family']=='obstacle'),dict(front=1200,left=600,right=600,rear=600))
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
                self.assertGreaterEqual(abs(meta['ego_road_yaw_offset_deg']),floor)
                for a in meta['runtime_actors']:self.assertGreaterEqual(a['planned_surface_clearance_m'],.3)
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
