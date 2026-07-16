#!/usr/bin/env python3

import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_stop_line_module import (
    StopLineModuleManager,
    StopLineParameters,
)


def synthetic_map(traffic_light_only=False):
    stop_line_id = 101 if traffic_light_only else 100
    relation = ""
    if traffic_light_only:
        relation = """
  <relation id="300">
    <member type="way" ref="101" role="ref_line" />
    <member type="way" ref="102" role="refers" />
    <tag k="type" v="regulatory_element" />
    <tag k="subtype" v="traffic_light" />
  </relation>
"""
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="2" lat="2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="3" lat="-2.0" lon="11.0"><tag k="ele" v="0.0" /></node>
  <node id="4" lat="2.0" lon="11.0"><tag k="ele" v="0.0" /></node>
  <way id="{stop_line_id}">
    <nd ref="1" /><nd ref="2" />
    <tag k="type" v="line_thin" /><tag k="subtype" v="stop_line" />
  </way>
  <way id="102"><nd ref="3" /><nd ref="4" /><tag k="type" v="traffic_light" /></way>
  {relation}
</osm>
""".format(stop_line_id=stop_line_id, relation=relation)
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


def make_odometry(x, speed):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


class StopLineModuleTest(unittest.TestCase):
    def parameters(self):
        return StopLineParameters(
            stop_margin=1.0,
            base_link_to_front=3.0,
            required_stop_duration_sec=1.0,
            vehicle_stopped_duration_threshold=0.1,
            hold_stop_margin_distance=0.5,
        )

    def test_inserts_exact_stop_pose_and_zero_velocity(self):
        input_path = make_path()
        output_path = to_path(input_path)
        data = PlannerData(
            lanelet_map=synthetic_map(),
            odometry=make_odometry(0.0, 5.0),
            current_time=10.0,
        )
        manager = StopLineModuleManager(self.parameters())

        result = manager.modify_path_velocity(input_path, output_path, data)

        self.assertTrue(result.modified)
        self.assertEqual(len(result.stop_reasons), 1)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(6.0, positions)
        stop_index = positions.index(6.0)
        self.assertTrue(
            all(
                point.longitudinal_velocity_mps == 0.0
                for point in output_path.points[stop_index:]
            )
        )
        self.assertAlmostEqual(
            result.stop_reasons[0].stop_factors[0].dist_to_stop_pose, 6.0
        )

    def test_excludes_traffic_light_reference_line(self):
        input_path = make_path()
        output_path = to_path(input_path)
        data = PlannerData(
            lanelet_map=synthetic_map(traffic_light_only=True),
            odometry=make_odometry(0.0, 5.0),
            current_time=10.0,
        )
        manager = StopLineModuleManager(self.parameters())

        result = manager.modify_path_velocity(input_path, output_path, data)

        self.assertFalse(result.modified)
        self.assertEqual(len(output_path.points), len(input_path.points))
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points)
        )

    def test_releases_after_required_continuous_stop(self):
        input_path = make_path()
        manager = StopLineModuleManager(self.parameters())
        data = PlannerData(
            lanelet_map=synthetic_map(),
            odometry=make_odometry(6.0, 0.0),
            current_time=10.0,
        )

        first = manager.modify_path_velocity(input_path, to_path(input_path), data)
        self.assertTrue(first.modified)
        self.assertIn("STOPPED", first.status)

        data.current_time = 10.5
        holding = manager.modify_path_velocity(input_path, to_path(input_path), data)
        self.assertTrue(holding.modified)

        data.current_time = 11.1
        released_output = to_path(input_path)
        released = manager.modify_path_velocity(input_path, released_output, data)
        self.assertFalse(released.modified)
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in released_output.points)
        )

    def test_missing_odometry_never_releases_stop(self):
        input_path = make_path()
        manager = StopLineModuleManager(self.parameters())
        data = PlannerData(lanelet_map=synthetic_map(), current_time=10.0)

        first = manager.modify_path_velocity(input_path, to_path(input_path), data)
        data.current_time = 30.0
        later = manager.modify_path_velocity(input_path, to_path(input_path), data)

        self.assertTrue(first.modified)
        self.assertTrue(later.modified)
        self.assertIn("APPROACHING", later.status)


if __name__ == "__main__":
    unittest.main()
