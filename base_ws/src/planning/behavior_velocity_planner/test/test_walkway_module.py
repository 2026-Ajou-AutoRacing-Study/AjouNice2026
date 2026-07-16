#!/usr/bin/env python3

import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_walkway_module import WalkwayModuleManager, WalkwayParameters


def synthetic_map():
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="10.0" />
  <node id="2" lat="2.0" lon="10.0" />
  <node id="3" lat="-2.0" lon="12.0" />
  <node id="4" lat="2.0" lon="12.0" />
  <way id="100"><nd ref="1"/><nd ref="2"/></way>
  <way id="101"><nd ref="3"/><nd ref="4"/></way>
  <relation id="600">
    <member type="way" ref="100" role="left"/>
    <member type="way" ref="101" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="walkway"/>
  </relation>
</osm>
"""
    return BehaviorVelocityMap.from_osm_bytes(
        xml.encode("utf-8"), projector_type="none"
    )


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in (0.0, 5.0, 10.0, 15.0, 20.0):
        point = PathPoint()
        point.pose.position.x = x
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [1]
        path.points.append(item)
    return path


def make_odometry(x=0.0, speed=0.0):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


class WalkwayModuleTest(unittest.TestCase):
    def setUp(self):
        self.manager = WalkwayModuleManager(
            WalkwayParameters(
                base_link_to_front=0.0,
                stop_duration=0.1,
            )
        )

    def run_module(self, x, speed, current_time):
        input_path = make_path()
        output_path = to_path(input_path)
        result = self.manager.modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=synthetic_map(),
                odometry=make_odometry(x, speed),
                current_time=float(current_time),
            ),
        )
        return result, output_path

    def test_parses_walkway(self):
        lane_map = synthetic_map()

        self.assertEqual(lane_map.pedestrian_crossing_count("walkway"), 1)
        self.assertEqual(
            lane_map.pedestrian_crossing_candidates("walkway")[0].lanelet_id,
            600,
        )

    def test_approach_always_inserts_mandatory_stop(self):
        result, output_path = self.run_module(0.0, 3.0, 10.0)

        self.assertTrue(result.modified)
        self.assertEqual(result.stop_reasons[0].reason, "Walkway")
        self.assertIn(6.5, [point.pose.position.x for point in output_path.points])
        self.assertIn("APPROACH", result.status)

    def test_stop_duration_releases_walkway(self):
        first, _ = self.run_module(6.0, 0.0, 10.0)
        second, _ = self.run_module(6.0, 0.0, 10.2)
        third, _ = self.run_module(6.0, 0.0, 10.3)
        fourth, output_path = self.run_module(6.0, 0.0, 10.4)

        self.assertTrue(first.modified)
        self.assertTrue(second.modified)
        self.assertFalse(third.modified)
        self.assertFalse(fourth.modified)
        self.assertIn("SURPASSED", fourth.status)
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points)
        )


if __name__ == "__main__":
    unittest.main()
