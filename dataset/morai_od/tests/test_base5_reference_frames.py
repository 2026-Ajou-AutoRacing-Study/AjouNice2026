import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_reference_frames import references,center_in_roi,center_in_bev,yaw_runtime_review


class ReferenceFramesTest(unittest.TestCase):
    def test_three_degrees_does_not_exclude_forward_100m(self):
        r=references([100,0],dict(x=0,y=0,yaw=3),0)
        self.assertAlmostEqual(r['ego_longitudinal_m'],100*math.cos(math.radians(3)))
        self.assertAlmostEqual(r['ego_lateral_m'],-100*math.sin(math.radians(3)))
        self.assertEqual(r['road_longitudinal_m'],100)
        self.assertTrue(center_in_roi([r['ego_longitudinal_m'],r['ego_lateral_m'],0]))

    def test_curve_plus_yaw_can_exit_lateral_roi(self):
        r=references([80,-14],dict(x=0,y=0,yaw=15),0)
        self.assertLess(r['ego_lateral_m'],-30)
        self.assertFalse(center_in_roi([r['ego_longitudinal_m'],r['ego_lateral_m'],0]))
        self.assertEqual(r['road_role'],'front')

    def test_training_bounds_are_strict_xy_only(self):
        self.assertFalse(center_in_bev([100,0,0]))
        self.assertTrue(center_in_bev([99,0,100]))

    def test_frozen_actor_gate_rejects_runtime_motion(self):
        m={'runtime_actors':[dict(unique_id=1,position=dict(x=0,y=0))]}
        def message(x):return {'npc_list':[dict(unique_id=1,position=dict(x=x,y=0))]}
        self.assertEqual(yaw_runtime_review(m,message(.01),message(.02)),[])
        self.assertIn('yaw_probe_actor_not_at_fixed_pose',yaw_runtime_review(m,message(5)))
        self.assertIn('yaw_probe_actor_moved',yaw_runtime_review(m,message(0),message(.2)))


if __name__=='__main__':unittest.main()
