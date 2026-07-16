#!/usr/bin/env python3

import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_no_drivable_lane_module import (
    NoDrivableLaneModuleManager,
    NoDrivableLaneParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def synthetic_map(tagged=True):
    tag = '<tag k="no_drivable_lane" v="yes" />' if tagged else ""
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="2" lat="2.0" lon="20.0"><tag k="ele" v="0.0" /></node>
  <node id="3" lat="-2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="4" lat="-2.0" lon="20.0"><tag k="ele" v="0.0" /></node>
  <way id="10"><nd ref="1" /><nd ref="2" /></way>
  <way id="11"><nd ref="3" /><nd ref="4" /></way>
  <relation id="1">
    <member type="way" ref="10" role="left" />
    <member type="way" ref="11" role="right" />
    <tag k="type" v="lanelet" /><tag k="subtype" v="road" />{tag}
  </relation>
</osm>
""".format(tag=tag)
    return BehaviorVelocityMap.from_osm_bytes(
        xml.encode("utf-8"), projector_type="none"
    )


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0):
        point = PathPoint()
        point.pose.position.x = x
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [1]
        path.points.append(item)
    return path


def make_odometry(x, speed=2.0):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


class NoDrivableLaneModuleTest(unittest.TestCase):
    def parameters(self):
        return NoDrivableLaneParameters(
            stop_margin=1.5,
            base_link_to_front=0.0,
            stopped_velocity_threshold=0.01,
        )

    def run_module(self, x, speed=2.0, tagged=True, manager=None):
        input_path = make_path()
        output_path = to_path(input_path)
        manager = manager or NoDrivableLaneModuleManager(self.parameters())
        result = manager.modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=synthetic_map(tagged),
                odometry=make_odometry(x, speed),
                current_time=10.0,
            ),
        )
        return result, output_path, manager

    def test_tag_is_parsed_on_road_lanelet(self):
        lanelet = synthetic_map().road_lanelets[1]
        self.assertEqual(lanelet.tags["no_drivable_lane"], "yes")

    def test_approaching_stops_before_lane(self):
        result, output_path, _ = self.run_module(0.0)
        self.assertTrue(result.modified)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(8.5, positions)
        self.assertIn(STATE_APPROACHING, result.status)

    def test_started_inside_stops_at_current_pose(self):
        result, output_path, _ = self.run_module(12.0)
        self.assertTrue(result.modified)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(12.0, positions)
        self.assertIn(STATE_INSIDE, result.status)

    def test_stopped_state_holds_current_pose(self):
        manager = NoDrivableLaneModuleManager(self.parameters())
        result, _, manager = self.run_module(12.0, speed=0.0, manager=manager)
        held, _, _ = self.run_module(13.0, speed=0.0, manager=manager)
        self.assertIn(STATE_STOPPED, result.status)
        self.assertIn(STATE_STOPPED, held.status)

    def test_untagged_lane_is_ignored(self):
        result, output_path, _ = self.run_module(0.0, tagged=False)
        self.assertFalse(result.modified)
        self.assertTrue(all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points))


STATE_APPROACHING = "APPROACHING"
STATE_INSIDE = "INSIDE_NO_DRIVABLE_LANE"
STATE_STOPPED = "STOPPED"


if __name__ == "__main__":
    unittest.main()
