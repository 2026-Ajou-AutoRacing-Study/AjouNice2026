#!/usr/bin/env python3
"""CPU-only checks for the frozen-input contract; no simulator or GPU."""
import hashlib
import gzip
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import subprocess

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'perception/detection3d/bevfusion/scripts'))
from qualify_base5_camera_tensorrt import load_inputs
sys.path.insert(0,str(ROOT/'perception/detection3d/bevfusion/tools'))
from compare_base5_common30 import gt_identity


class InputTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)
        self.point=self.root/'points.bin'
        np.array([[1,2,3,.5]],dtype=np.float32).tofile(self.point)
        self.image=self.root/'image.jpg';self.image.write_bytes(b'\xff\xd8synthetic-test-header')
        def item(p):return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        self.manifest=dict(schema='base5_camera_trt_inputs_v1',image_policy='native_jpeg90',
            point_intensity_source='native',samples=[dict(id='frozen-1',points=item(self.point),
                images={'image/'+c:item(self.image) for c in
                    ('camera_front','camera_front_left','camera_front_right','camera_back')})])
        self.path=self.root/'manifest.json'

    def write(self):self.path.write_text(json.dumps(self.manifest))

    def test_valid_sensor_only_input(self):
        self.write();samples,_,pins=load_inputs(self.path,1)
        self.assertEqual(len(samples),1);self.assertEqual(samples[0]['points'].shape,(1,4))
        self.assertIn(str(self.point),pins)

    def test_no_duplicate_frame_inflation(self):
        self.manifest['samples']*=2;self.write()
        with self.assertRaises(ValueError):load_inputs(self.path,2)

    def test_enough_unique_inputs_required(self):
        self.write()
        with self.assertRaises(ValueError):load_inputs(self.path,2)

    def test_modified_payload_rejected(self):
        self.write();self.image.write_bytes(b'\xff\xd8changed')
        with self.assertRaises(ValueError):load_inputs(self.path,1)

    def test_native_vs_ros_intensity_not_silently_mixed(self):
        np.array([[1,2,3,255]],dtype=np.float32).tofile(self.point)
        self.manifest['samples'][0]['points']['sha256']=hashlib.sha256(self.point.read_bytes()).hexdigest()
        self.write()
        with self.assertRaises(ValueError):load_inputs(self.path,1)

    def test_missing_camera_not_ignored(self):
        self.manifest['samples'][0]['images'].pop('image/camera_back');self.write()
        with self.assertRaises(ValueError):load_inputs(self.path,1)

    def test_summary_and_full_roi_metric_on_synthetic_predictions(self):
        # Fixture AP is test data, never a detector performance claim.
        out=self.root/'summary';out.mkdir()
        (out/'INVENTORY.json').write_text('{}')
        (out/'input_sha256.json').write_text('{}')
        models={name:dict(radial30=dict(metrics={f'center/{c}_mean_AP':.9
            for c in ('vehicle','pedestrian','obstacle')})) for name in ('E25_FP32','E12_FP32','E12_AMP','E12_TRT_HEAD')}
        (out/'common30_metrics').mkdir()
        (out/'common30_metrics/comparison.json').write_text(json.dumps(dict(models=models)))
        rows=[dict(token=str(label),gt_boxes=[[0,0,0,1,1,1,0]],gt_labels=[label],gt_points=[1],
            gt_assets=['fixture'],pred_boxes=[[0,0,0,1,1,1,0]],pred_labels=[label],pred_scores=[.9]) for label in range(3)]
        rows.append(dict(token='empty',gt_boxes=[],gt_labels=[],gt_points=[],gt_assets=[],
                         pred_boxes=[],pred_labels=[],pred_scores=[]))
        for backend in ('fp32','amp','trt_head'):
            folder=out/f'full_e12_{backend}';folder.mkdir()
            with gzip.open(folder/'predictions.json.gz','wt') as f:
                json.dump(dict(rows=rows,classes=['vehicle','pedestrian','obstacle'],frames=len(rows),
                    gt_identity=gt_identity(rows)),f)
        prediction={'fixture':[dict(label=0,class_name='vehicle',center=[0,0,0],size=[1,1,1],yaw=0.,score=.9)]}
        for case in ('original','cache','separated','parallel','trt','trt_head'):
            for round_index in (0,1):
                folder=out/f'timing_r{round_index}_{case}';folder.mkdir()
                (folder/'RESULT.json').write_text(json.dumps(dict(complete=True,
                    raw_times=[dict(service_ms=10.,model_ms=8.)],peak_allocated_bytes=1024,
                    peak_reserved_bytes=2048,timings=dict(service_ms=dict(mean=10.)),pool_state={})))
                with gzip.open(folder/'predictions.json.gz','wt') as f:json.dump(prediction,f)
        run=subprocess.run([sys.executable,str(ROOT/'scripts/evaluation/summarize_bevfusion_portfolio_eval.py'),
            '--output',str(out)],capture_output=True,text=True)
        self.assertEqual(run.returncode,0,run.stdout+run.stderr)
        summary=json.loads((out/'SUMMARY.json').read_text())
        self.assertEqual(summary['parity_reference_case'],'original')
        self.assertAlmostEqual(summary['e12_full_roi_ap_percent']['fp32']['mAP'],100.)
        self.assertEqual(len(summary['timings']),6)


if __name__=='__main__':unittest.main()
