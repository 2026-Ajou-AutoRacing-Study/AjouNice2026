import math
from types import SimpleNamespace as S
import unittest
import numpy as np
from prepare_base5_rosbag_quick import nearest, xyzi, local_box, count_support, interpolate_scalar
from summarize_base5_rosbag_quick import bridge_nms
from analyze_base5_bag_continuity import miss_runs


class BagQuickTests(unittest.TestCase):
    def test_miss_runs_split_at_gt_and_sensor_holes(self):
        records=[(0,0.,False),(1,.1,False),(2,.2,True),(3,.3,False),
                 (5,.5,False),(6,1.5,False),(7,1.6,False)]
        runs=miss_runs(records,.1)
        self.assertEqual([r['frames'] for r in runs],[2,1,1,2])
        np.testing.assert_allclose([r['seconds'] for r in runs],[.2,.1,.1,.2])

    def test_no_miss_episode_for_complete_detection(self):
        self.assertEqual(miss_runs([(0,0.,True),(1,.1,True)],.1),[])

    def test_bridge_nms_preserves_gt_and_separates_classes(self):
        r=dict(gt_boxes=[[1,2,3,1,1,1,0]],pred_boxes=[[0,0,0,2,2,1,0]]*4,
               pred_labels=[2,2,1,0],pred_scores=[.2,.8,.3,.01])
        out=bridge_nms(r)
        self.assertEqual(out['pred_scores'],[.8,.3])
        self.assertEqual(out['pred_labels'],[2,1])
        self.assertIs(out['gt_boxes'],r['gt_boxes'])
        self.assertEqual(len(r['pred_boxes']),4)

    def test_nearest_tolerance_and_tie(self):
        rows=[(1.,'a'),(1.04,'b')]
        self.assertEqual(nearest(rows,1.025,.05),(1.04,'b'))
        self.assertIsNone(nearest(rows,2.,.05))
        self.assertIsNone(nearest([],1.,.05))

    def test_padded_organized_cloud_layout(self):
        data=bytearray(64)
        expected=np.array([[1,2,3,135],[4,5,6,255]],dtype='<f4')
        data[:16]=expected[0].tobytes();data[32:48]=expected[1].tobytes()
        message=S(height=2,width=1,is_bigendian=False,point_step=16,row_step=32,data=data,
                  fields=[S(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))])
        np.testing.assert_array_equal(xyzi(message),expected)

    def test_world_to_lidar_rotation_and_center(self):
        point=S(x=10.,y=22.,z=4.,yaw=math.pi/2,length=4.,width=2.,height=1.)
        ego=S(x=10.,y=20.,yaw=math.pi/2)
        box=local_box(point,ego,2.,[.855,0,1.388])
        np.testing.assert_allclose(box,[1.145,0,.612,4,2,1,0],atol=1e-8)
        self.assertEqual(count_support(np.array([[1.145,0,.612,10],[5,5,5,10]]),box),1)

    def test_height_interpolation_never_extrapolates(self):
        rows=[(1.,S(position=S(z=4.))),(1.1,S(position=S(z=6.)))]
        self.assertAlmostEqual(interpolate_scalar(rows,1.05,.15),5.)
        self.assertIsNone(interpolate_scalar(rows,.5,.15))
        self.assertIsNone(interpolate_scalar(rows,1.05,.05))


if __name__=='__main__':unittest.main()
