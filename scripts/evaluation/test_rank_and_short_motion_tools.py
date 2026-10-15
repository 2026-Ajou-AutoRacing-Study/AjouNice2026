import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts/rosbag/record'),str(ROOT/'scripts/evaluation')]
from run_lidar_short_motion_probe import route_window,steering
from infer_lidar_packet_candidates import drive_window
from qualify_pedestrian_association_ranking import corrected_predictions


class MotionAndRankTools(unittest.TestCase):
    def test_route_requires_road_support_and_stopping_space(self):
        points=np.array([[i,0,0] for i in range(20)])
        _,_,index,goal=route_window(points,[2,0,0],8)
        self.assertEqual(index,2);self.assertEqual(goal,10)
        with self.assertRaises(ValueError):route_window(points,[2,2,0],8)
        with self.assertRaises(ValueError):route_window(points,[17,0,0],8)
        with self.assertRaises(ValueError):route_window(points,[float('nan'),0,0],8)

    def test_steering_direction_and_guard(self):
        points=np.array([[i,0,0] for i in range(20)])
        arc=np.arange(20)
        self.assertAlmostEqual(steering(points,arc,0,0,0,0),0)
        self.assertLess(steering(points,arc,0,0,0,.1),0)
        with self.assertRaises(RuntimeError):steering(points,arc,0,0,0,np.pi)

    def test_motion_window_does_not_require_validator_events(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'events.jsonl').write_text('')
            trace=[dict(time=100+i*.1,speed=1.) for i in range(15)]
            (root/'motion_trace.json').write_text(json.dumps(trace))
            self.assertEqual(drive_window(root/'raw.bag',{}),(100.2,101.2))
            trace=[dict(time=100,speed=0.)]
            (root/'motion_trace.json').write_text(json.dumps(trace))
            with self.assertRaises(ValueError):drive_window(root/'raw.bag',{})

    def test_gt_merge_preserves_prediction_payload_and_rejects_sensor_change(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prediction=root/'p.json.gz';gt=root/'g.json'
            row=dict(token='a',stamp=1.,point_sha256='point',image_sha256={'front':'jpeg'},
                     pred_boxes=[[1,2,3,4,5,6,0]],gt_boxes=[])
            with gzip.open(prediction,'wt') as stream:json.dump(dict(rows=[row]),stream)
            corrected=dict(row,gt_boxes=[[1,2,0,4,5,6,0]])
            gt.write_text(json.dumps(dict(rows=[corrected])))
            output=corrected_predictions(prediction,gt)
            self.assertEqual(output[0]['pred_boxes'],row['pred_boxes'])
            self.assertEqual(output[0]['gt_boxes'],corrected['gt_boxes'])
            corrected['point_sha256']='different';gt.write_text(json.dumps(dict(rows=[corrected])))
            with self.assertRaises(ValueError):corrected_predictions(prediction,gt)


if __name__=='__main__':unittest.main()
