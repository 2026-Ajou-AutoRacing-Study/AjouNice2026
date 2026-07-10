#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_perception_msgs.msg import PredictedObjects
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import (
    PathModuleBase,
    apply_window_shift,
    make_avoidance_debug_array,
    object_pose,
    object_speed,
    project_pose_to_path,
    set_velocity_window,
    uuid_to_string,
)


class DynamicObstacleAvoidanceBPP(PathModuleBase):
    def __init__(self):
        self.objects = None

        self.min_dynamic_speed = float(rospy.get_param("~min_dynamic_speed", 0.5))
        self.max_dynamic_speed = float(rospy.get_param("~max_dynamic_speed", 100.0))
        self.max_forward_distance = float(rospy.get_param("~max_forward_distance", 80.0))
        self.object_check_lateral_range = float(rospy.get_param("~object_check_lateral_range", 2.0))
        self.slow_down_before_distance = float(rospy.get_param("~slow_down_before_distance", 8.0))
        self.slow_down_after_distance = float(rospy.get_param("~slow_down_after_distance", 8.0))
        self.slow_velocity_mps = float(rospy.get_param("~slow_velocity_mps", 1.0))
        self.enable_lateral_nudge = bool(rospy.get_param("~enable_lateral_nudge", True))
        self.nudge_shift_length = float(rospy.get_param("~nudge_shift_length", 0.3))
        self.prepare_distance = float(rospy.get_param("~prepare_distance", 10.0))
        self.return_distance = float(rospy.get_param("~return_distance", 10.0))
        self.default_avoid_direction = rospy.get_param("~default_avoid_direction", "right").lower()

        objects_topic = rospy.get_param("~objects_topic", "~input/objects")
        rospy.Subscriber(objects_topic, PredictedObjects, self.objects_callback, queue_size=1)
        self.avoidance_debug_pub = rospy.Publisher(
            "~debug/avoidance", AvoidanceDebugMsgArray, queue_size=1
        )

        super().__init__("dynamic_obstacle_avoidance_bpp")

    def objects_callback(self, msg):
        self.objects = msg

    def nudge_sign(self, lateral_distance):
        if abs(lateral_distance) < 1.0e-3:
            return -1.0 if self.default_avoid_direction == "right" else 1.0
        return -1.0 if lateral_distance > 0.0 else 1.0

    def collect_candidates(self, path):
        if self.objects is None:
            return []

        candidates = []
        for obj in self.objects.objects:
            speed = object_speed(obj)
            projection = project_pose_to_path(path, object_pose(obj))
            if projection is None:
                continue

            candidate = {
                "object_id": uuid_to_string(obj.object_id),
                "longitudinal_distance": projection.longitudinal_distance,
                "lateral_distance": projection.signed_lateral_distance,
                "path_distance": projection.distance,
                "shift_length": 0.0,
                "allow_avoidance": False,
            }

            if speed < self.min_dynamic_speed:
                candidate["failed_reason"] = "static_or_slow_object"
                candidates.append(candidate)
                continue
            if speed > self.max_dynamic_speed:
                candidate["failed_reason"] = "too_fast"
                candidates.append(candidate)
                continue
            if projection.longitudinal_distance < 0.0:
                candidate["failed_reason"] = "behind_ego"
                candidates.append(candidate)
                continue
            if projection.longitudinal_distance > self.max_forward_distance:
                candidate["failed_reason"] = "too_far"
                candidates.append(candidate)
                continue
            if abs(projection.signed_lateral_distance) > self.object_check_lateral_range:
                candidate["failed_reason"] = "outside_path_corridor"
                candidates.append(candidate)
                continue

            candidate["allow_avoidance"] = True
            if self.enable_lateral_nudge:
                candidate["shift_length"] = self.nudge_sign(
                    projection.signed_lateral_distance
                ) * self.nudge_shift_length
            candidate["failed_reason"] = ""
            candidates.append(candidate)

        return candidates

    def process_path(self, path, stamp):
        candidates = self.collect_candidates(path)
        self.avoidance_debug_pub.publish(make_avoidance_debug_array(path.header, candidates))

        valid_candidates = [candidate for candidate in candidates if candidate["allow_avoidance"]]
        if not valid_candidates:
            return path, "dynamic_obstacle_avoidance: pass-through, no dynamic target"

        target = min(valid_candidates, key=lambda item: item["longitudinal_distance"])
        start_s = max(0.0, target["longitudinal_distance"] - self.slow_down_before_distance)
        end_s = target["longitudinal_distance"] + self.slow_down_after_distance
        output = set_velocity_window(path, start_s, end_s, self.slow_velocity_mps, mode="min")

        if self.enable_lateral_nudge and abs(target["shift_length"]) > 1.0e-6:
            output = apply_window_shift(
                output,
                target["shift_length"],
                target["longitudinal_distance"],
                self.prepare_distance,
                self.return_distance,
            )

        return output, (
            "dynamic_obstacle_avoidance: target {} slowed to {:.2f} m/s".format(
                target["object_id"], self.slow_velocity_mps
            )
        )


def main():
    rospy.init_node("dynamic_obstacle_avoidance_bpp")
    DynamicObstacleAvoidanceBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
