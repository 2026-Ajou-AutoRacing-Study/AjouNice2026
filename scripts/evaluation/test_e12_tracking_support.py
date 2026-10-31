"""Offline evaluation geometry/conditioning tests; no ROS master or GPU."""
import math
import unittest

import numpy as np

from prepare_e12_postprocessing import transform_for_pose, world_box
from analyze_e12_tracking_support import (adjacent, full_prepare, local_box,
    metric_row, support_groups, velocity_at)
from probe_e12_coasting import configured_harness, mature_vehicle_core


def frame():
    return dict(stamp=1.,native_index=1,token='fixture',
        ego=dict(x=20.,y=-10.,z=3.,yaw=.7),
        gt=[dict(key='actor',label=0,class_name='vehicle',lidar_points=0,asset='car',
                 local_box=[10.,2.,1.,4.,2.,1.5,.2])],detections=[],tracks=[])


class E12TrackingSupportTest(unittest.TestCase):
    def test_bottom_to_gravity_once_and_inverse_roundtrip(self):
        f=frame();pose=[f['ego'][k]for k in ('x','y','z','yaw')]
        ex=np.eye(4);ex[:3,3]=[1.,.2,1.7]
        b=[10.,2.,1.,4.,2.,1.5,.2]
        obj=world_box(b,pose,ex,True)
        self.assertTrue(np.allclose(local_box(obj,f,ex),b,atol=1e-10))
        gt=world_box([10.,2.,1.75,*b[3:]],pose,ex,False)
        self.assertTrue(np.allclose(gt['center'],obj['center']))

    def test_rigid_transform_uses_ego_yaw_without_extra_morai_correction(self):
        t=transform_for_pose([0.,0.,0.,math.pi/2],np.eye(4))
        self.assertTrue(np.allclose(t@[1.,0.,0.,1.],[0.,1.,0.,1.]))

    def test_temporal_adjacency_needs_original_index_and_bounded_time(self):
        a=dict(stamp=1.,native_index=3)
        self.assertTrue(adjacent(a,dict(stamp=1.1,native_index=4)))
        self.assertFalse(adjacent(a,dict(stamp=1.1,native_index=5)))
        self.assertFalse(adjacent(a,dict(stamp=2.,native_index=4)))
        self.assertFalse(adjacent(a,dict(stamp=1.,native_index=4)))

    def test_velocity_uses_bbox_center_secant_not_gap_extrapolation(self):
        series=[[1.,[0.,0.,0.]],[1.1,[1.,0.,0.]],[2.,[100.,0.,0.]]]
        self.assertTrue(np.allclose(velocity_at(series,1.1),[10.,0.]))
        self.assertIsNone(velocity_at(series,2.))
        self.assertIsNone(velocity_at(series,1.5))

    def test_velocity_cannot_bridge_excluded_original_indices(self):
        series=[[1.,[0.,0.,0.],1],[1.1,[10.,0.,0.],3]]
        self.assertIsNone(velocity_at(series,1.))
        self.assertIsNone(velocity_at(series,1.1))

    def test_coast_probe_only_changes_bounded_moving_configuration(self):
        text='  config.confirmed_track_max_coast_time_sec = .50;\nOTHER'
        self.assertEqual(configured_harness(text,.75),
            '  config.confirmed_track_max_coast_time_sec = 0.75;\nOTHER')
        with self.assertRaises(ValueError):
            configured_harness(text,6.)
        with self.assertRaises(ValueError):
            configured_harness('wrong source anchor',.75)

    def test_mature_vehicle_probe_has_no_gt_or_pedestrian_extension(self):
        source='''    const double maximum_coast_time = stationary
        ? config_.confirmed_stationary_track_max_coast_time_sec
        : config_.confirmed_track_max_coast_time_sec;'''
        result=mature_vehicle_core(source,.75)
        self.assertIn('ObjectClass::CAR',result)
        self.assertIn('ObjectClass::TRUCK',result)
        self.assertIn('track.initialization_time >= 1.0',result)
        self.assertNotIn('ObjectClass::PEDESTRIAN',result)
        self.assertNotIn('gt',result)

    def test_roi_filter_keeps_track_objects_ids_aligned(self):
        f=frame();f['ego']=dict(x=0.,y=0.,z=0.,yaw=0.)
        def obj(x,ident):
            return dict(center=[x,0.,1.],size=[4.,2.,1.],yaw=0.,label=0,
                        class_name='vehicle',score=.8,id=ident,confirmed=True)
        f['tracks']=[obj(200.,99),obj(10.,7)]
        r=full_prepare([metric_row(f,'tracker',np.eye(4))])[0]
        self.assertEqual(r['pred_track_ids'].tolist(),[7])
        self.assertEqual(r['pred_objects'][0]['id'],7)
        self.assertEqual(len(r['gt_points']),1)

    def test_zero_support_condition_does_not_claim_unacquired_track_retention(self):
        base=dict(class_name='vehicle',band='80_90m',points=0,tracked=True,detected=False,
            associated=False,center_error_m=.2,last_measurement_age_sec=.2,
            previous_positive_in_continuous_segment=False,
            previous_track_acquisition_in_continuous_segment=False)
        groups=support_groups([base])
        self.assertEqual(groups['vehicle/all/zero']['recovered_without_detection'],1)
        self.assertNotIn('vehicle/all/zero_after_track_acquisition',groups)

    def test_empty_support_is_absent_not_fabricated_zero_percent(self):
        self.assertEqual(support_groups([]),{})


if __name__=='__main__':
    unittest.main()
