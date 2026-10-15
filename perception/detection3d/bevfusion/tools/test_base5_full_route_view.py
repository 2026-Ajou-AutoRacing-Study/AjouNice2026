import unittest
from view_base5_full_route_comparison import filtered, markers


class FullRouteViewTests(unittest.TestCase):
    def row(self, x=10, score=.8, label=0):
        return dict(pred_boxes=[[x,0,-1,4,2,2,0]],pred_scores=[score],pred_labels=[label])

    def test_bottom_center_is_lifted_only_for_visualization(self):
        row=self.row();out=filtered(row,[.4,.15,.1])
        self.assertEqual(out[0]['center'],[10,0,0])
        self.assertEqual(row['pred_boxes'][0][2],-1)

    def test_common_roi_does_not_hide_full_range_export(self):
        self.assertEqual(filtered(self.row(50),[.4,.15,.1],True),[])
        self.assertEqual(len(filtered(self.row(50),[.4,.15,.1])),1)

    def test_class_specific_thresholds_and_nms(self):
        self.assertEqual(filtered(self.row(score=.3),[.4,.15,.1]),[])
        self.assertEqual(len(filtered(self.row(score=.3,label=2),[.4,.15,.1])),1)
        row=self.row();row['pred_boxes']*=2;row['pred_scores']=[.8,.7];row['pred_labels']=[0,0]
        self.assertEqual(len(filtered(row,[.4,.15,.1])),1)

    def test_empty_frames_clear_previous_boxes(self):
        import rospy
        from visualization_msgs.msg import Marker
        result=markers([],rospy.Time.from_sec(123.),'E9',(0,1,1))
        self.assertEqual(len(result.markers),1)
        self.assertEqual(result.markers[0].action,Marker.DELETEALL)

    def test_marker_corners_match_geometric_height(self):
        import rospy
        out=markers(filtered(self.row(),[.4,.15,.1]),rospy.Time.from_sec(123.),'E9',(0,1,1))
        self.assertEqual(len(out.markers[1].points),24)
        self.assertEqual({p.z for p in out.markers[1].points},{-1.,1.})
        self.assertEqual(out.markers[1].header.frame_id,'lidar1')


if __name__=='__main__':unittest.main()
