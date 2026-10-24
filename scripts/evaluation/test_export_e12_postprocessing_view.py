"""Pure CPU geometry and clock alignment tests; no ROS master/GPU."""
import math
import unittest
import numpy as np

from export_e12_postprocessing_view import align_receipts,box_corners,sensor_inverse,to_sensor


class ExportViewTest(unittest.TestCase):
    def test_gravity_center_is_not_shifted_up_again(self):
        obj=dict(center=[10.,0.,2.],size=[4.,2.,2.],yaw=0.)
        corners=np.asarray(box_corners(obj,np.eye(4)))
        self.assertAlmostEqual(corners[:,2].min(),1.)
        self.assertAlmostEqual(corners[:,2].max(),3.)
        self.assertTrue(np.allclose(corners.mean(0),obj['center']))

    def test_current_sensor_inverse_rotates_world_geometry_once(self):
        frame=dict(ego=dict(x=10.,y=20.,z=3.,yaw=math.pi/2))
        extrinsic=np.eye(4);extrinsic[:3,3]=[1.,0.,2.]
        inverse=sensor_inverse(frame,extrinsic)
        self.assertTrue(np.allclose(to_sensor([10.,21.,5.],inverse),[0.,0.,0.],atol=1e-10))
        self.assertTrue(np.allclose(to_sensor([10.,23.,5.],inverse),[2.,0.,0.],atol=1e-10))

    def test_unique_header_match_preserves_different_receipt_clock(self):
        frames=[dict(stamp=1.,token='a'),dict(stamp=1.1,token='b')]
        source={1.:[dict(receipt_nsec=1200000000,source_header_nsec=1000000000,frame_id='lidar1')],
            1.1:[dict(receipt_nsec=1300000000,source_header_nsec=1100000000,frame_id='lidar1')]}
        result=align_receipts(frames,source)
        self.assertEqual(result[0]['receipt_nsec'],1200000000)
        self.assertEqual(result[0]['source_header_nsec'],1000000000)

    def test_missing_ambiguous_or_wrong_frame_is_rejected(self):
        frames=[dict(stamp=1.,token='a')]
        for source in ({}, {1.:[{},{}]}, {1.:[dict(frame_id='base_link',receipt_nsec=1)]}):
            with self.assertRaises(ValueError):align_receipts(frames,source)

    def test_nonincreasing_receipt_cannot_drive_playback(self):
        frames=[dict(stamp=1.,token='a'),dict(stamp=2.,token='b')]
        source={1.:[dict(receipt_nsec=2,frame_id='lidar1')],2.:[dict(receipt_nsec=1,frame_id='lidar1')]}
        with self.assertRaises(ValueError):align_receipts(frames,source)


if __name__=='__main__':unittest.main()
