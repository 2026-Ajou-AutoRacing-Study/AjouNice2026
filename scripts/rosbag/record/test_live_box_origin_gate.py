import copy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).parent))
from run_live_box_origin_gate import legacy_shadow


class ShadowTests(unittest.TestCase):
    def test_inverse_shadow_does_not_mutate_delivered_message(self):
        obj=NS(shape=NS(dimensions=NS(z=1.8)),kinematics=NS(pose_with_covariance=NS(pose=NS(position=NS(z=2.)))))
        message=NS(objects=[obj],header=NS(stamp=123,frame_id='lidar1'))
        shadow=legacy_shadow(message)
        self.assertEqual(message.objects[0].kinematics.pose_with_covariance.pose.position.z,2.)
        self.assertAlmostEqual(shadow.objects[0].kinematics.pose_with_covariance.pose.position.z,1.1)
        self.assertEqual(shadow.header.stamp,123)
        self.assertIsNot(shadow.header,message.header)
        message.objects[0].shape.dimensions.z=float('nan')
        with self.assertRaises(ValueError):legacy_shadow(message)


if __name__=='__main__':unittest.main()
