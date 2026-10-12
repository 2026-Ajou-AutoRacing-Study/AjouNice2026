import copy
import unittest

import numpy as np

from analyze_base5_ros_tracking import compare, indexed, policy_checks
from replay_base5_ros_tracking import pose_at, validate_pair


def obj(x=0.,label=2,vx=0.):
    return dict(id=1,label=label,class_name=('vehicle','pedestrian','obstacle')[label],
                center=[x,0.,0.],size=[2.,1.,1.],yaw=0.,score=.8,
                velocity=[vx,0.],body_velocity=[vx,0.],yaw_rate=0.,
                stationary=vx==0.,orientation_availability=1)


def frame(x=0.,stamp=1.,label=2,vx=0.):
    return dict(stamp=stamp,frame_id='map',objects=[obj(x,label,vx)])


class RosTrackingReplayTests(unittest.TestCase):
    def test_threshold_experiment_does_not_allow_compiled_or_different_options(self):
        off=dict(compile_parts=[],parallel_camera=True)
        validate_pair(off,dict(off),[.2,.1,.12])
        with self.assertRaises(ValueError):validate_pair(off,dict(off),[.05,.1,.12])
        with self.assertRaises(ValueError):validate_pair(off,dict(off,compile_parts=['camera']),[.2,.1,.12])
        validate_pair(off,dict(off,compile_parts=['camera','neck']),None)

    def test_tf_pose_is_interpolated_only_within_recorded_bounds(self):
        ego=np.array([[1.,0.,0.,0.,0.],[1.1,1.,2.,3.,.2]])
        np.testing.assert_allclose(pose_at(ego,1.05),[.5,1.,1.5,.1])
        with self.assertRaises(ValueError):pose_at(ego,.9)
        with self.assertRaises(ValueError):pose_at(ego,1.2)

    def test_timestamp_misses_are_not_hidden_by_common_frame_statistics(self):
        r,rows=compare([frame(stamp=1.),frame(stamp=2.)],[frame(.1,stamp=1.)],.5)
        self.assertEqual(r['reference_only_timestamps'],[2.])
        self.assertEqual(r['matched_timestamps'],1)
        self.assertAlmostEqual(r['by_class']['obstacle']['distributions']['center_xy_m']['mean'],.1)
        self.assertEqual(len(rows),1)

    def test_spatial_unmatched_is_not_counted_as_zero_error(self):
        r,rows=compare([frame()],[frame(.6)],.5)
        self.assertEqual(rows,[])
        self.assertEqual(r['by_class']['obstacle']['removed'],1)
        self.assertEqual(r['by_class']['obstacle']['added'],1)
        self.assertIsNone(r['by_class']['obstacle']['distributions']['center_xy_m']['mean'])

    def test_unknown_policy_keeps_pose_while_zeroing_motion(self):
        raw=[frame(x=.7,vx=3.)];pol=copy.deepcopy(raw)
        pol[0]['objects'][0].update(velocity=[0.,0.],body_velocity=[0.,0.],stationary=True)
        r=policy_checks(raw,pol)
        self.assertEqual(r['unknown_observations'],1)
        self.assertEqual(r['unknown_raw_speed_above_05_mps'],1)
        pol[0]['objects'][0]['center'][0]=0.
        with self.assertRaises(ValueError):policy_checks(raw,pol)

    def test_non_unknown_policy_must_not_modify_velocity(self):
        raw=[frame(label=0,vx=3.)];pol=copy.deepcopy(raw)
        pol[0]['objects'][0]['velocity']=[0.,0.]
        with self.assertRaises(ValueError):policy_checks(raw,pol)

    def test_duplicate_timestamp_or_wrong_frame_rejected(self):
        with self.assertRaises(ValueError):indexed([frame(),frame()])
        f=frame();f['frame_id']='probe_lidar'
        with self.assertRaises(ValueError):indexed([f])

    def test_identical_independent_output_control_has_zero_difference(self):
        r,rows=compare([frame()],[copy.deepcopy(frame())],.5)
        self.assertEqual(r['by_class']['all']['distributions']['center_xy_m']['maximum'],0.)
        self.assertEqual(r['frame_changes'],[])
        self.assertEqual(r['id_mapping_changes'],[])


if __name__=='__main__':unittest.main()
