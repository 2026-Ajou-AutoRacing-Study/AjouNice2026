#!/usr/bin/env python3

import unittest

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
)
from autoware_planning_msgs.msg import PathPoint

from behavior_velocity_crosswalk_module import (
    CrosswalkModuleManager,
    CrosswalkParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def synthetic_map(with_stop_line=False, safety_speed=None):
    stop_line = ""
    regulatory = ""
    road_regulatory = ""
    if with_stop_line:
        stop_line = """
  <node id="5" lat="-2.0" lon="8.0" />
  <node id="6" lat="2.0" lon="8.0" />
  <way id="800"><nd ref="5"/><nd ref="6"/><tag k="subtype" v="stop_line"/></way>
"""
        regulatory = """
  <relation id="900">
    <member type="relation" ref="500" role="refers"/>
    <member type="way" ref="800" role="ref_line"/>
    <tag k="type" v="regulatory_element"/><tag k="subtype" v="crosswalk"/>
  </relation>
"""
        road_regulatory = '<member type="relation" ref="900" role="regulatory_element"/>'
    speed_tag = (
        '<tag k="safety_slow_down_speed" v="{}"/>'.format(safety_speed)
        if safety_speed is not None
        else ""
    )
    xml = """<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="-2.0" lon="10.0" />
  <node id="2" lat="2.0" lon="10.0" />
  <node id="3" lat="-2.0" lon="12.0" />
  <node id="4" lat="2.0" lon="12.0" />
  <way id="100"><nd ref="1"/><nd ref="2"/></way>
  <way id="101"><nd ref="3"/><nd ref="4"/></way>
  {stop_line}
  <relation id="500">
    <member type="way" ref="100" role="left"/>
    <member type="way" ref="101" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="crosswalk"/>{speed_tag}
  </relation>
  {regulatory}
  <relation id="1">{road_regulatory}<tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation>
</osm>
""".format(
        stop_line=stop_line,
        regulatory=regulatory,
        road_regulatory=road_regulatory,
        speed_tag=speed_tag,
    )
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


def make_odometry(x=0.0, speed=2.0):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


def make_pedestrian(start_y=-4.0, speed=1.4, label=ObjectClassification.PEDESTRIAN):
    detected = PredictedObject()
    detected.object_id.uuid = list(range(16))
    classification = ObjectClassification()
    classification.label = int(label)
    classification.probability = 1.0
    detected.classification = [classification]
    pose = detected.kinematics.initial_pose_with_covariance.pose
    pose.position.x = 11.0
    pose.position.y = float(start_y)
    pose.orientation.w = 1.0
    detected.kinematics.initial_twist_with_covariance.twist.linear.y = float(speed)
    prediction = PredictedPath()
    for y in (start_y, -2.0, 0.0, 2.0, 4.0):
        point = Pose()
        point.position.x = 11.0
        point.position.y = float(y)
        point.orientation.w = 1.0
        prediction.path.append(point)
    prediction.confidence = 1.0
    detected.kinematics.predicted_paths = [prediction]
    objects = PredictedObjects()
    objects.objects = [detected]
    return objects


class CrosswalkModuleTest(unittest.TestCase):
    def parameters(self):
        return CrosswalkParameters(
            base_link_to_front=0.0,
            base_link_to_rear=0.0,
            enable_no_stop_decision=False,
        )

    def run_module(self, objects=None, lane_map=None):
        input_path = make_path()
        output_path = to_path(input_path)
        result = CrosswalkModuleManager(self.parameters()).modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=lane_map or synthetic_map(),
                odometry=make_odometry(),
                dynamic_objects=objects,
                current_time=10.0,
            ),
        )
        return result, output_path

    def test_parses_crosswalk_polygon_and_regulatory_stop_line(self):
        lane_map = synthetic_map(with_stop_line=True)

        self.assertEqual(lane_map.pedestrian_crossing_count("crosswalk"), 1)
        crossing = lane_map.pedestrian_crossing_candidates("crosswalk")[0]
        self.assertEqual(crossing.lanelet_id, 500)
        self.assertEqual(crossing.road_lane_ids, (1,))
        self.assertEqual(crossing.stop_lines[0].line_id, 800)
        self.assertEqual(len(crossing.points), 4)

    def test_crossing_pedestrian_inserts_yield_stop(self):
        result, output_path = self.run_module(make_pedestrian())

        self.assertTrue(result.modified)
        self.assertEqual(result.stop_reasons[0].reason, "Crosswalk")
        positions = [point.pose.position.x for point in output_path.points]
        self.assertIn(6.5, positions)
        stop_index = positions.index(6.5)
        self.assertTrue(
            all(
                point.longitudinal_velocity_mps == 0.0
                for point in output_path.points[stop_index:]
            )
        )
        self.assertIn("YIELD", result.status)

    def test_no_object_passes_without_velocity_change(self):
        result, output_path = self.run_module()

        self.assertFalse(result.modified)
        self.assertTrue(
            all(point.longitudinal_velocity_mps == 5.0 for point in output_path.points)
        )

    def test_non_vru_object_is_ignored(self):
        result, _ = self.run_module(
            make_pedestrian(label=ObjectClassification.CAR)
        )

        self.assertFalse(result.modified)

    def test_regulatory_stop_line_has_priority(self):
        result, output_path = self.run_module(
            make_pedestrian(), synthetic_map(with_stop_line=True)
        )

        positions = [point.pose.position.x for point in output_path.points]
        self.assertTrue(result.modified)
        self.assertIn(8.0, positions)

    def test_map_safety_slow_down_speed_is_applied(self):
        result, output_path = self.run_module(
            lane_map=synthetic_map(safety_speed=2.5)
        )

        self.assertTrue(result.modified)
        self.assertIn(2.5, [point.longitudinal_velocity_mps for point in output_path.points])


if __name__ == "__main__":
    unittest.main()
