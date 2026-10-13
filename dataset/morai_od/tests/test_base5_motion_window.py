import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import motion_capture_review
from generate_base5_independent import one_way_motion_path


class MotionWindowTest(unittest.TestCase):
    def test_moving_endpoint_and_missing_trace(self):
        actor=dict(unique_id=7,motion='walking',motion_path=dict(points=[dict(x=0,y=0),dict(x=4,y=0)]))
        objects=lambda x:dict(pedestrian_list=[dict(unique_id=7,position=dict(x=x,y=0))])
        trace=[dict(uid=7,t=t,x=t,y=0) for t in (.8,.9,1.)]
        self.assertFalse(motion_capture_review(actor,objects(1),objects(2),trace,1.)['review_reasons'])
        self.assertIn('motion_endpoint_reached_during_capture_window',motion_capture_review(actor,objects(1),objects(4),trace,1.)['review_reasons'])
        self.assertIn('motion_not_observed_immediately_before_capture',motion_capture_review(actor,objects(1),objects(2),[],1.)['review_reasons'])
        actor['motion']='standing'
        self.assertFalse(motion_capture_review(actor,None,None,[],1.)['required'])

    def test_wide_cell_allows_faster_profile_without_changing_bounds(self):
        pose=dict(x=20,y=0,z=0,yaw=0);ego=dict(x=0,y=0)
        slow=one_way_motion_path(pose,[20,0],ego,[15,50],'running',{},6.)
        faster=one_way_motion_path(pose,[20,0],ego,[15,50],'running',{},6.,True)
        self.assertGreater(faster['nominal_speed_mps'],slow['nominal_speed_mps'])
        self.assertEqual(faster['distance_bounds_m'],[15,50])
        self.assertEqual(len(faster['points']),2)


if __name__=='__main__':unittest.main()
