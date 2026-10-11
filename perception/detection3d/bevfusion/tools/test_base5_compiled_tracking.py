import math
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from pathlib import Path

import numpy as np
from replay_base5_compiled_tracking import world_detections, compare_tracks, core_replay


class CompiledTrackingTests(unittest.TestCase):
    def test_world_transform_exact_stamp_and_yaw_wrap(self):
        ego=np.array([[1.,10,20,0,math.pi/2],[1.1,11,20,0,math.pi/2]])
        extrinsic=np.eye(4);extrinsic[0,3]=1
        d=dict(center=[2,0,0],size=[4,2,1],yaw=0,score=.8,label=0,class_name='vehicle')
        r=world_detections([d],1.,ego,extrinsic)[0]
        np.testing.assert_allclose(r['center'],[10,23,0],atol=1e-12)
        self.assertAlmostEqual(r['yaw'],math.pi/2)

    def test_unmeasured_pose_is_not_extrapolated(self):
        ego=np.array([[1.,0,0,0,0],[1.2,1,0,0,0]])
        for t in (.9,1.1,1.3):
            with self.assertRaises(ValueError):world_detections([],t,ego,np.eye(4))

    def test_tracker_difference_is_not_gt_velocity_error(self):
        def frame(vx):return dict(stamp=1.,tracks=[dict(id=3,label=1,confirmed=True,associated=True,
            center=[0.,0.],velocity=[vx,0.],yaw=0.)])
        r=compare_tracks([frame(5.)],[frame(5.01)])
        self.assertAlmostEqual(r['distributions']['speed_mps']['maximum'],.01)
        self.assertEqual(r['matched_id_differences'],0)
        self.assertEqual(r['confirmed_set_changes'],[])

    def test_constant_numeric_id_renumbering_is_not_a_mapping_switch(self):
        def frame(t,identity):return dict(stamp=t,tracks=[dict(id=identity,label=1,confirmed=True,
            center=[0.,0.],velocity=[5.,0.],yaw=0.)])
        r=compare_tracks([frame(1,3),frame(2,3)],[frame(1,8),frame(2,8)])
        self.assertEqual(r['matched_id_differences'],2)
        self.assertEqual(r['id_mapping_changes'],[])

    def test_core_output_is_validated_against_recorded_stamps(self):
        row=dict(token='frame',pred_boxes=[],pred_labels=[],pred_scores=[])
        ego=np.array([[1.,0,0,0,0],[1.1,0,0,0,0]])
        with patch('replay_base5_compiled_tracking.subprocess.run',return_value=SimpleNamespace(stdout='FRAME 1.05 0\n')):
            result=core_replay(Path('/unused'),[row],{'frame':1.05},ego,np.eye(4),(.4,.15,.1))
            self.assertEqual(result,[dict(stamp=1.05,tracks=[])])
        with patch('replay_base5_compiled_tracking.subprocess.run',return_value=SimpleNamespace(stdout='FRAME 1.05 1\n')):
            with self.assertRaises(ValueError):core_replay(Path('/unused'),[row],{'frame':1.05},ego,np.eye(4),(.4,.15,.1))

    def test_range_filter_keeps_empty_time_frames(self):
        row=dict(token='frame',pred_boxes=[[50,0,0,4,2,1,0]],pred_labels=[0],pred_scores=[.8])
        ego=np.array([[1.,0,0,0,0],[1.1,0,0,0,0]])
        with patch('replay_base5_compiled_tracking.subprocess.run',return_value=SimpleNamespace(stdout='FRAME 1.05 0\n')) as run:
            core_replay(Path('/unused'),[row],{'frame':1.05},ego,np.eye(4),(.4,.15,.1),30.)
            self.assertEqual(run.call_args.kwargs['input'],'1.05 0\n')


if __name__=='__main__':unittest.main()
