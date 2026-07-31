#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def load_marker_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "current_vehicle_marker.py",
        )
    )
    spec = importlib.util.spec_from_file_location("current_vehicle_marker", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CurrentVehicleMarkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_marker_module()

    def test_builds_oriented_car_icon_at_odometry_pose(self):
        message = Odometry()
        message.header.frame_id = "map"
        message.pose.pose.position.x = 10.0
        message.pose.pose.position.y = 20.0
        message.pose.pose.position.z = 3.0
        message.pose.pose.orientation.z = math.sin(math.pi * 0.25)
        message.pose.pose.orientation.w = math.cos(math.pi * 0.25)

        result = self.module.make_vehicle_marker_array(message)

        self.assertEqual(len(result.markers), 10)
        self.assertEqual(result.markers[0].type, Marker.CUBE)
        self.assertEqual(result.markers[1].type, Marker.CUBE)
        self.assertEqual(result.markers[2].type, Marker.LINE_STRIP)
        self.assertEqual(result.markers[-1].type, Marker.ARROW)
        self.assertAlmostEqual(result.markers[0].pose.position.x, 10.0, places=3)
        self.assertGreater(result.markers[0].pose.position.y, 21.0)
        self.assertAlmostEqual(result.markers[0].scale.x, 4.635)
        self.assertEqual(
            {marker.id for marker in result.markers[3:7]}, {0, 1, 2, 3}
        )

    def test_empty_frame_falls_back_to_map(self):
        message = Odometry()
        message.pose.pose.orientation.w = 1.0

        result = self.module.make_vehicle_marker_array(message)

        self.assertTrue(result.markers)
        self.assertTrue(
            all(marker.header.frame_id == "map" for marker in result.markers)
        )

    def test_icon_scale_enlarges_shape_without_moving_its_center(self):
        message = Odometry()
        message.pose.pose.orientation.w = 1.0

        normal = self.module.make_vehicle_marker_array(message, icon_scale=1.0)
        enlarged = self.module.make_vehicle_marker_array(message, icon_scale=2.0)

        self.assertAlmostEqual(
            enlarged.markers[0].pose.position.x,
            normal.markers[0].pose.position.x,
        )
        self.assertAlmostEqual(
            enlarged.markers[0].scale.x,
            2.0 * normal.markers[0].scale.x,
        )
        self.assertAlmostEqual(
            enlarged.markers[0].scale.y,
            2.0 * normal.markers[0].scale.y,
        )


if __name__ == "__main__":
    unittest.main()
