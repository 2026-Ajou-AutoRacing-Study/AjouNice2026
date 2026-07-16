#!/usr/bin/env python3

import unittest

import rospy
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_traffic_light_module import (
    TrafficLightModuleManager,
    TrafficLightParameters,
)


def synthetic_map(turn_direction="straight"):
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="2" lat="2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="3" lat="3.0" lon="10.0"><tag k="ele" v="4.0" /></node>
  <node id="4" lat="3.0" lon="11.0"><tag k="ele" v="4.0" /></node>
  <way id="100">
    <nd ref="1" /><nd ref="2" /><tag k="type" v="stop_line" />
  </way>
  <way id="101">
    <nd ref="3" /><nd ref="4" /><tag k="type" v="traffic_light" />
  </way>
  <relation id="200">
    <member type="way" ref="100" role="ref_line" />
    <member type="way" ref="101" role="refers" />
    <tag k="type" v="regulatory_element" />
    <tag k="subtype" v="traffic_light" />
  </relation>
  <relation id="1">
    <member type="relation" ref="200" role="regulatory_element" />
    <tag k="type" v="lanelet" /><tag k="subtype" v="road" />
    <tag k="turn_direction" v="{turn_direction}" />
  </relation>
</osm>
""".format(turn_direction=turn_direction)
    return BehaviorVelocityMap.from_osm_bytes(
        xml.encode("utf-8"), projector_type="none"
    )


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in (0.0, 5.0, 10.0, 15.0, 20.0):
        path_point = PathPoint()
        path_point.pose.position.x = x
        path_point.pose.orientation.w = 1.0
        path_point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = path_point
        item.lane_ids = [1]
        path.points.append(item)
    return path


def make_odometry(speed=0.0):
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


def make_signal(color, shape=TrafficLightElement.CIRCLE, stamp=10.0):
    element = TrafficLightElement()
    element.color = int(color)
    element.shape = int(shape)
    element.status = TrafficLightElement.SOLID_ON
    element.confidence = 1.0
    group = TrafficLightGroup()
    group.traffic_light_group_id = 200
    group.elements = [element]
    message = TrafficLightGroupArray()
    message.stamp = rospy.Time.from_sec(float(stamp))
    message.traffic_light_groups = [group]
    return message


class TrafficLightModuleTest(unittest.TestCase):
    def parameters(self):
        return TrafficLightParameters(
            stop_margin=1.0,
            stop_time_hysteresis=0.1,
            base_link_to_front=0.0,
            is_simulation=False,
        )

    def run_module(self, manager, traffic_signals, current_time, speed=0.0, lane_map=None):
        input_path = make_path()
        output_path = to_path(input_path)
        result = manager.modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=lane_map or synthetic_map(),
                odometry=make_odometry(speed),
                traffic_signals=traffic_signals,
                current_time=float(current_time),
            ),
        )
        return result, output_path

    def test_parses_traffic_light_regulatory_element(self):
        lane_map = synthetic_map()

        self.assertEqual(lane_map.traffic_light_count(), 1)
        traffic_light = lane_map.traffic_light_candidates()[0]
        self.assertEqual(traffic_light.regulatory_id, 200)
        self.assertEqual(traffic_light.stop_line_id, 100)
        self.assertEqual(traffic_light.traffic_light_ids, (101,))
        self.assertEqual(traffic_light.lane_ids, (1,))
        self.assertEqual(traffic_light.turn_direction(1), "straight")

    def test_red_inserts_stop_after_hysteresis(self):
        manager = TrafficLightModuleManager(self.parameters())
        signal = make_signal(TrafficLightElement.RED)

        first, _ = self.run_module(manager, signal, 10.0)
        second, output_path = self.run_module(manager, signal, 10.2)

        self.assertFalse(first.modified)
        self.assertTrue(second.modified)
        self.assertEqual(len(second.planning_factors), 1)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(9.0, positions)
        stop_index = positions.index(9.0)
        self.assertTrue(
            all(
                point.longitudinal_velocity_mps == 0.0
                for point in output_path.points[stop_index:]
            )
        )

    def test_green_circle_passes(self):
        manager = TrafficLightModuleManager(self.parameters())

        result, output_path = self.run_module(
            manager, make_signal(TrafficLightElement.GREEN), 10.0
        )

        self.assertFalse(result.modified)
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points)
        )

    def test_straight_green_arrow_overrides_red_circle(self):
        message = make_signal(TrafficLightElement.RED)
        arrow = TrafficLightElement()
        arrow.color = TrafficLightElement.GREEN
        arrow.shape = TrafficLightElement.UP_ARROW
        arrow.status = TrafficLightElement.SOLID_ON
        arrow.confidence = 1.0
        message.traffic_light_groups[0].elements.append(arrow)
        manager = TrafficLightModuleManager(self.parameters())

        result, _ = self.run_module(manager, message, 10.0)

        self.assertFalse(result.modified)

    def test_timed_out_signal_stops(self):
        manager = TrafficLightModuleManager(self.parameters())
        signal = make_signal(TrafficLightElement.GREEN, stamp=8.0)

        self.run_module(manager, signal, 10.0)
        result, _ = self.run_module(manager, signal, 10.2)

        self.assertTrue(result.modified)
        self.assertIn("TIMEOUT", result.status)

    def test_missing_signal_policy_differs_between_real_and_simulation(self):
        real = TrafficLightModuleManager(self.parameters())
        simulation_parameters = self.parameters()
        simulation_parameters.is_simulation = True
        simulation = TrafficLightModuleManager(simulation_parameters)

        self.run_module(real, None, 10.0)
        real_result, _ = self.run_module(real, None, 10.2)
        simulation_result, _ = self.run_module(simulation, None, 10.2)

        self.assertTrue(real_result.modified)
        self.assertFalse(simulation_result.modified)

    def test_judge_line_distance_matches_autoware_formula(self):
        distance = TrafficLightModuleManager.calc_judge_line_distance(
            10.0, 0.0, -2.8, -5.0, 0.5
        )

        self.assertAlmostEqual(distance, 25.620556, places=5)


if __name__ == "__main__":
    unittest.main()
