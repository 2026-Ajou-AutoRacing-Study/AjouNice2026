"""CPU geometry/metadata contract tests; never starts a ROS master or GUI."""
import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

from prepare_e12_common_rviz import (map_from_lidar, map_from_morai,
    marker_pose, pose_matrix, quaternion_for_matrix, transform_array, yaw_matrix)


def marker(action=0):
    return NS(header=NS(frame_id='lidar1', stamp=123), action=action,
        pose=NS(position=NS(x=3., y=-2., z=1.), orientation=NS(x=0., y=0., z=0., w=1.)),
        points=[NS(x=1., y=2., z=3.)], color=NS(r=.1,g=.9,b=.2,a=1.),
        scale=NS(x=.1,y=.1,z=.4), id=7, ns='test', text='T7', lifetime=.35)


class CommonViewGeometryTest(unittest.TestCase):
    def test_quaternion_roundtrip_near_half_turn(self):
        for yaw in (0., .7, math.pi, -math.pi, 2.9):
            matrix = yaw_matrix([1.,2.,3.], yaw)
            np.testing.assert_allclose(pose_matrix([1.,2.,3.], quaternion_for_matrix(matrix)), matrix, atol=1.e-14)

    def test_pose_only_transform_keeps_local_points_color_and_height(self):
        source = NS(markers=[marker()])
        transform = yaw_matrix([100.,200.,5.], math.pi/2)
        output, error = transform_array(source, transform)
        self.assertLess(error, 1.e-12)
        self.assertEqual(output.markers[0].points, source.markers[0].points)
        self.assertEqual(output.markers[0].color, source.markers[0].color)
        self.assertEqual(output.markers[0].header.stamp, 123)
        self.assertEqual(output.markers[0].header.frame_id, 'map')
        self.assertEqual(source.markers[0].header.frame_id, 'lidar1')
        self.assertAlmostEqual(output.markers[0].pose.position.z, 6.)
        point = np.array([1.,2.,3.,1.])
        np.testing.assert_allclose(marker_pose(output.markers[0])@point,
                                   transform@marker_pose(source.markers[0])@point, atol=1.e-12)

    def test_deleteall_unset_quaternion_is_not_treated_as_geometry(self):
        source = marker(action=3)
        source.pose.orientation.w = 0.
        output, error = transform_array(NS(markers=[source]), np.eye(4))
        self.assertEqual(output.markers[0].header.frame_id, 'map')
        self.assertEqual(output.markers[0].pose.orientation.w, 0.)
        self.assertEqual(error, 0.)

    def test_pack_yaw_and_mount_are_each_applied_once(self):
        frame = dict(ego=dict(x=10., y=20., z=3., yaw=math.pi/2))
        mount = yaw_matrix([1.,0.,2.], 0.)
        alignment = map_from_morai(dict(xyz=[100.,200.,0.],rpy=[0.,0.,0.]))
        result = map_from_lidar(frame,mount,alignment)@np.array([2.,0.,0.,1.])
        np.testing.assert_allclose(result,[110.,223.,5.,1.], atol=1.e-12)

    def test_invalid_source_frame_and_pose_are_rejected(self):
        source = marker()
        source.header.frame_id = 'base_link'
        with self.assertRaises(ValueError):
            transform_array(NS(markers=[source]), np.eye(4))
        with self.assertRaises(ValueError):
            pose_matrix([0,0,0],[0,0,0,0])
        with self.assertRaises(ValueError):
            map_from_morai(dict(xyz=[0,0,0],rpy=[.1,0,0]))


if __name__ == '__main__':
    unittest.main()
