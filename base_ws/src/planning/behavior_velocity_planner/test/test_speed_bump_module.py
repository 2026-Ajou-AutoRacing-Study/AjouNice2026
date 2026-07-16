#!/usr/bin/env python3

import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_speed_bump_module import (
    SpeedBumpModuleManager,
    SpeedBumpParameters,
)


def synthetic_map(slow_down_speed=None, attached_lane_id=1):
    speed_tag = (
        '<tag k="slow_down_speed" v="{}" />'.format(slow_down_speed)
        if slow_down_speed is not None
        else ""
    )
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="9.0"><tag k="ele" v="0.0" /></node>
  <node id="2" lat="-2.0" lon="11.0"><tag k="ele" v="0.0" /></node>
  <node id="3" lat="2.0" lon="11.0"><tag k="ele" v="0.0" /></node>
  <node id="4" lat="2.0" lon="9.0"><tag k="ele" v="0.0" /></node>
  <way id="100">
    <nd ref="1" /><nd ref="2" /><nd ref="3" /><nd ref="4" />
    <tag k="area" v="yes" /><tag k="height" v="0.15" />
    {speed_tag}<tag k="type" v="speed_bump" />
  </way>
  <relation id="200">
    <member type="way" ref="100" role="refers" />
    <tag k="type" v="regulatory_element" />
    <tag k="subtype" v="speed_bump" />
  </relation>
  <relation id="{lane_id}">
    <member type="relation" ref="200" role="regulatory_element" />
    <tag k="type" v="lanelet" /><tag k="subtype" v="road" />
  </relation>
</osm>
""".format(speed_tag=speed_tag, lane_id=attached_lane_id)
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


def make_odometry(x):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    return odometry


class SpeedBumpModuleTest(unittest.TestCase):
    def parameters(self):
        return SpeedBumpParameters(
            slow_start_margin=1.0,
            slow_end_margin=1.0,
            min_height=0.05,
            max_height=0.30,
            min_speed=1.39,
            max_speed=2.78,
            base_link_to_front=0.0,
            base_link_to_rear=0.0,
        )

    def test_parses_regulatory_element_and_lanelet_link(self):
        lanelet_map = synthetic_map()

        self.assertEqual(lanelet_map.speed_bump_count(), 1)
        speed_bump = lanelet_map.speed_bump_candidates()[0]
        self.assertEqual(speed_bump.regulatory_id, 200)
        self.assertEqual(speed_bump.polygon_id, 100)
        self.assertEqual(speed_bump.lane_ids, (1,))

    def test_inserts_slow_interval_and_height_based_speed(self):
        input_path = make_path()
        output_path = to_path(input_path)
        data = PlannerData(
            lanelet_map=synthetic_map(),
            odometry=make_odometry(0.0),
            current_time=10.0,
        )
        manager = SpeedBumpModuleManager(self.parameters())

        result = manager.modify_path_velocity(input_path, output_path, data)

        expected_speed = 2.78 + (0.15 - 0.05) / (0.30 - 0.05) * (1.39 - 2.78)
        self.assertTrue(result.modified)
        self.assertEqual(len(result.planning_factors), 1)
        self.assertEqual(result.planning_factors[0].behavior, 2)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(8.0, positions)
        self.assertIn(12.0, positions)
        start_index = positions.index(8.0)
        end_index = positions.index(12.0)
        for point in output_path.points[start_index : end_index + 1]:
            self.assertAlmostEqual(point.longitudinal_velocity_mps, expected_speed)
        self.assertEqual(output_path.points[0].longitudinal_velocity_mps, 5.0)
        self.assertEqual(output_path.points[-1].longitudinal_velocity_mps, 5.0)

    def test_slow_down_speed_annotation_overrides_height(self):
        manager = SpeedBumpModuleManager(self.parameters())

        speed = manager.calculate_slow_speed(
            synthetic_map(slow_down_speed=7.2).speed_bump_candidates()[0].tags
        )

        self.assertAlmostEqual(speed, 2.0)

    def test_ignores_speed_bump_attached_to_other_lanelet(self):
        input_path = make_path()
        output_path = to_path(input_path)
        data = PlannerData(
            lanelet_map=synthetic_map(attached_lane_id=2),
            odometry=make_odometry(0.0),
            current_time=10.0,
        )
        manager = SpeedBumpModuleManager(self.parameters())

        result = manager.modify_path_velocity(input_path, output_path, data)

        self.assertFalse(result.modified)
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points)
        )


if __name__ == "__main__":
    unittest.main()
