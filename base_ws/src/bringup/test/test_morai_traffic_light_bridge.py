#!/usr/bin/env python3

import importlib.util
import os
import unittest

import rospy
from autoware_perception_msgs.msg import TrafficLightElement
from morai_msgs.msg import GetTrafficLightStatus


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "validation",
    "scripts",
    "morai_traffic_light_bridge.py",
)
SPEC = importlib.util.spec_from_file_location("morai_traffic_light_bridge", SCRIPT_PATH)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


def morai_signal(signal_id="C1256W000016", status=1):
    output = GetTrafficLightStatus()
    output.header.stamp = rospy.Time.from_sec(12.0)
    output.trafficLightIndex = signal_id
    output.trafficLightStatus = status
    return output


class MoraiTrafficLightBridgeTest(unittest.TestCase):
    def test_red_maps_to_the_lanelet_group(self):
        message = morai_signal(status=BRIDGE.MORAI_RED)
        output = BRIDGE.make_traffic_light_group_array(
            message, {"C1256W000016": 201290}
        )

        self.assertEqual(output.stamp, message.header.stamp)
        self.assertEqual(len(output.traffic_light_groups), 1)
        group = output.traffic_light_groups[0]
        self.assertEqual(group.traffic_light_group_id, 201290)
        self.assertEqual(group.elements[0].color, TrafficLightElement.RED)
        self.assertEqual(group.elements[0].shape, TrafficLightElement.CIRCLE)
        self.assertEqual(group.elements[0].status, TrafficLightElement.SOLID_ON)

    def test_red_with_green_left_keeps_each_lamp(self):
        elements = BRIDGE.elements_from_morai_state(
            BRIDGE.MORAI_RED | BRIDGE.MORAI_GREEN_LEFT
        )
        lamps = {(element.color, element.shape) for element in elements}

        self.assertEqual(
            lamps,
            {
                (TrafficLightElement.RED, TrafficLightElement.CIRCLE),
                (TrafficLightElement.GREEN, TrafficLightElement.LEFT_ARROW),
            },
        )

    def test_unmapped_signal_publishes_no_wrong_group(self):
        output = BRIDGE.make_traffic_light_group_array(
            morai_signal(signal_id="UNKNOWN"), {"C1256W000016": 201290}
        )

        self.assertEqual(output.traffic_light_groups, [])

    def test_v60_mapping_contains_the_current_route_signal(self):
        mapping_file = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "morai_traffic_light_map_v60.yaml",
        )

        mapping = BRIDGE.load_traffic_light_map(mapping_file)

        self.assertEqual(mapping["C1256W000016"], 201290)

    def test_no_simulator_state_is_not_green(self):
        elements = BRIDGE.elements_from_morai_state(-1)

        self.assertEqual(len(elements), 1)
        self.assertEqual(elements[0].color, TrafficLightElement.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
