#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest

from morai_msgs.msg import ObjectStatus, ObjectStatusList
from visualization_msgs.msg import Marker


def load_visualizer_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "validation",
            "scripts",
            "morai_perception_visualizer.py",
        )
    )
    spec = importlib.util.spec_from_file_location(
        "morai_perception_visualizer", script_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def detected_object(unique_id, x, y, z, heading, size):
    output = ObjectStatus()
    output.unique_id = unique_id
    output.position.x = x
    output.position.y = y
    output.position.z = z
    output.heading = heading
    output.size.x, output.size.y, output.size.z = size
    return output


class MoraiPerceptionVisualizerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_visualizer_module()

    def test_creates_oriented_boxes_and_labels_for_each_class(self):
        message = ObjectStatusList()
        message.header.frame_id = "morai_map"
        message.npc_list.append(
            detected_object(10, 1.0, 2.0, 3.0, 90.0, (4.5, 1.8, 1.6))
        )
        message.pedestrian_list.append(
            detected_object(20, 4.0, 5.0, 6.0, 0.0, (0.4, 0.6, 1.8))
        )
        message.obstacle_list.append(
            detected_object(30, 7.0, 8.0, 9.0, 0.0, (0.8, 0.8, 0.5))
        )

        result = self.module.make_marker_array(message)

        self.assertEqual(len(result.markers), 10)
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)

        vehicle_fill = result.markers[1]
        vehicle_outline = result.markers[2]
        vehicle_label = result.markers[3]
        self.assertEqual(vehicle_fill.type, Marker.CUBE)
        self.assertEqual(vehicle_outline.type, Marker.LINE_LIST)
        self.assertEqual(len(vehicle_outline.points), 24)
        self.assertEqual(vehicle_fill.header.frame_id, "morai_map")
        self.assertAlmostEqual(vehicle_fill.pose.position.z, 3.8)
        self.assertAlmostEqual(
            vehicle_fill.pose.orientation.z, math.sin(math.pi * 0.25)
        )
        self.assertEqual(vehicle_label.text, "VEHICLE #10")

        pedestrian_fill = result.markers[4]
        obstacle_fill = result.markers[7]
        self.assertGreater(pedestrian_fill.color.r, pedestrian_fill.color.b)
        self.assertGreater(obstacle_fill.color.r, obstacle_fill.color.g)

    def test_empty_frame_uses_fallback_and_invalid_size_is_bounded(self):
        message = ObjectStatusList()
        message.obstacle_list.append(
            detected_object(1, 0.0, 0.0, 0.0, 0.0, (0.0, -0.02, 0.0))
        )

        result = self.module.make_marker_array(
            message, position_is_bottom=False, show_labels=False
        )

        self.assertEqual(len(result.markers), 3)
        fill = result.markers[1]
        self.assertEqual(fill.header.frame_id, "morai_map")
        self.assertEqual(fill.scale.x, 0.10)
        self.assertEqual(fill.scale.y, 0.10)
        self.assertEqual(fill.scale.z, 0.10)
        self.assertEqual(fill.pose.position.z, 0.0)


if __name__ == "__main__":
    unittest.main()
