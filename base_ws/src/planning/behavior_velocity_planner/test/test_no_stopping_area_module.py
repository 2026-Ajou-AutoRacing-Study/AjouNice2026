#!/usr/bin/env python3

import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
)
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_no_stopping_area_module import (
    NoStoppingAreaModuleManager,
    NoStoppingAreaParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def synthetic_map():
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="2" lat="-2.0" lon="15.0"><tag k="ele" v="0.0" /></node>
  <node id="3" lat="2.0" lon="15.0"><tag k="ele" v="0.0" /></node>
  <node id="4" lat="2.0" lon="10.0"><tag k="ele" v="0.0" /></node>
  <node id="5" lat="-3.0" lon="9.0"><tag k="ele" v="0.0" /></node>
  <node id="6" lat="3.0" lon="9.0"><tag k="ele" v="0.0" /></node>
  <way id="100">
    <nd ref="1" /><nd ref="2" /><nd ref="3" /><nd ref="4" />
    <tag k="area" v="yes" /><tag k="type" v="no_stopping_area" />
  </way>
  <way id="101">
    <nd ref="5" /><nd ref="6" /><tag k="type" v="stop_line" />
  </way>
  <relation id="200">
    <member type="way" ref="100" role="refers" />
    <member type="way" ref="101" role="ref_line" />
    <tag k="type" v="regulatory_element" />
    <tag k="subtype" v="no_stopping_area" />
  </relation>
  <relation id="1">
    <member type="relation" ref="200" role="regulatory_element" />
    <tag k="type" v="lanelet" /><tag k="subtype" v="road" />
  </relation>
</osm>
"""
    return BehaviorVelocityMap.from_osm_bytes(
        xml.encode("utf-8"), projector_type="none"
    )


def make_path(zero_at=None):
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
        point = PathPoint()
        point.pose.position.x = x
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 0.0 if zero_at == x else 5.0
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


def make_objects(x=None, speed=0.0, label=ObjectClassification.CAR):
    objects = PredictedObjects()
    if x is None:
        return objects
    detected = PredictedObject()
    classification = ObjectClassification()
    classification.label = label
    classification.probability = 1.0
    detected.classification = [classification]
    detected.kinematics.initial_pose_with_covariance.pose.position.x = float(x)
    detected.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    detected.kinematics.initial_twist_with_covariance.twist.linear.x = float(speed)
    detected.shape.dimensions.x = 4.5
    detected.shape.dimensions.y = 1.8
    detected.shape.dimensions.z = 1.5
    objects.objects = [detected]
    return objects


class NoStoppingAreaModuleTest(unittest.TestCase):
    def parameters(self):
        return NoStoppingAreaParameters(
            state_clear_time=2.0,
            stuck_vehicle_vel_thr=0.833,
            stop_margin=0.0,
            dead_line_margin=1.0,
            stop_line_margin=1.0,
            detection_area_length=200.0,
            stuck_vehicle_front_margin=6.0,
            base_link_to_front=0.0,
            base_link_to_rear=0.0,
            vehicle_length=4.5,
            vehicle_width=2.0,
        )

    def run_module(self, objects=None, zero_at=None, x=0.0, speed=0.0, now=10.0, manager=None):
        input_path = make_path(zero_at)
        output_path = to_path(input_path)
        data = PlannerData(
            lanelet_map=synthetic_map(),
            odometry=make_odometry(x, speed),
            dynamic_objects=objects or make_objects(),
            current_time=now,
        )
        manager = manager or NoStoppingAreaModuleManager(self.parameters())
        result = manager.modify_path_velocity(input_path, output_path, data)
        return result, output_path, manager

    def test_parses_regulatory_area_stop_line_and_lanelet_link(self):
        lanelet_map = synthetic_map()
        self.assertEqual(lanelet_map.no_stopping_area_count(), 1)
        area = lanelet_map.no_stopping_area_candidates()[0]
        self.assertEqual(area.regulatory_id, 200)
        self.assertEqual(area.polygon_id, 100)
        self.assertEqual(area.stop_line.line_id, 101)
        self.assertEqual(area.lane_ids, (1,))

    def test_clear_area_keeps_velocity(self):
        result, output_path, _ = self.run_module()
        self.assertFalse(result.modified)
        self.assertTrue(all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points))

    def test_stuck_vehicle_inserts_stop_at_custom_line(self):
        result, output_path, _ = self.run_module(make_objects(17.0, 0.0))
        self.assertTrue(result.modified)
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(9.0, positions)
        stop_index = positions.index(9.0)
        self.assertTrue(all(point.longitudinal_velocity_mps == 0.0 for point in output_path.points[stop_index:]))
        self.assertIn("stuck_vehicle", result.status)

    def test_moving_vehicle_is_ignored(self):
        result, _, _ = self.run_module(make_objects(17.0, 2.0))
        self.assertFalse(result.modified)

    def test_existing_stop_velocity_inside_area_triggers_stop(self):
        result, _, _ = self.run_module(zero_at=15.0)
        self.assertTrue(result.modified)
        self.assertIn("stop_velocity", result.status)

    def test_clear_time_holds_then_releases_stop(self):
        manager = NoStoppingAreaModuleManager(self.parameters())
        first, _, manager = self.run_module(make_objects(17.0), now=10.0, manager=manager)
        held, _, manager = self.run_module(now=11.0, manager=manager)
        released, _, _ = self.run_module(now=13.0, manager=manager)
        self.assertTrue(first.modified)
        self.assertTrue(held.modified)
        self.assertFalse(released.modified)

    def test_unstoppable_high_speed_passes_through(self):
        result, _, _ = self.run_module(make_objects(17.0), x=8.5, speed=10.0)
        self.assertFalse(result.modified)
        self.assertIn("PASS", result.status)


if __name__ == "__main__":
    unittest.main()
