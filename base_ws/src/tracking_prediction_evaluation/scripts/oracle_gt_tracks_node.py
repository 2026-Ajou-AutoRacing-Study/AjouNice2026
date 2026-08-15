#!/usr/bin/env python3
"""Adapt MORAI ObjectStatusList GT to map-frame TrackedObjects for E0 viewing."""

from __future__ import annotations

import hashlib
import math
from collections import defaultdict, deque
from pathlib import Path

import rospy
import yaml
from autoware_perception_msgs.msg import (
    ObjectClassification,
    Shape,
    TrackedObject,
    TrackedObjectKinematics,
    TrackedObjects,
)
from geometry_msgs.msg import Quaternion
from morai_msgs.msg import ObjectStatusList

from tracking_prediction_evaluation.evaluator import estimate_motion_state
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, normalize_angle


CLASS_LABELS = {
    "vehicle": ObjectClassification.CAR,
    "pedestrian": ObjectClassification.PEDESTRIAN,
    "obstacle": ObjectClassification.UNKNOWN,
}


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def stable_uuid(class_name, unique_id, asset_name):
    key = "morai-gt:{}:{}:{}".format(class_name, unique_id, asset_name)
    return hashlib.md5(key.encode("utf-8")).digest()


class OracleGtTracksNode:
    def __init__(self):
        self.history_sec = float(rospy.get_param("~history_sec", 0.5))
        self.maximum_gap_sec = float(rospy.get_param("~maximum_gap_sec", 0.5))
        self.stationary_speed_mps = float(
            rospy.get_param("~stationary_speed_mps", 0.25)
        )
        static_tf_file = Path(rospy.get_param("~morai_static_tf")).expanduser()
        document = yaml.safe_load(static_tf_file.read_text(encoding="utf-8")) or {}
        transform = document["map_to_morai_map"]
        self.translation = tuple(float(value) for value in transform["xyz"])
        rpy = tuple(float(value) for value in transform["rpy"])
        if len(self.translation) != 3 or len(rpy) != 3:
            raise ValueError("map_to_morai_map xyz/rpy must contain three values")
        if abs(rpy[0]) > 1.0e-9 or abs(rpy[1]) > 1.0e-9:
            raise ValueError("oracle adapter supports a yaw-only static transform")
        self.transform_yaw = rpy[2]
        self.histories = defaultdict(deque)

        input_topic = rospy.get_param("~input", "/Object_topic")
        output_topic = rospy.get_param(
            "~output", "/evaluation/oracle/tracking/objects"
        )
        self.publisher = rospy.Publisher(output_topic, TrackedObjects, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[oracle_gt_tracks] %s -> %s, history=%.2fs, frame=map",
            input_topic,
            output_topic,
            self.history_sec,
        )

    def transform_pose(self, position, heading_deg):
        cosine = math.cos(self.transform_yaw)
        sine = math.sin(self.transform_yaw)
        x = cosine * position.x - sine * position.y + self.translation[0]
        y = sine * position.x + cosine * position.y + self.translation[1]
        z = position.z + self.translation[2]
        yaw = normalize_angle(math.radians(heading_deg) + self.transform_yaw)
        return x, y, z, yaw

    def update_history(self, class_name, status, timestamp):
        key = (class_name, int(status.unique_id), str(status.name))
        x, y, z, yaw = self.transform_pose(status.position, status.heading)
        history = self.histories[key]
        if history and (
            timestamp <= history[-1].timestamp
            or timestamp - history[-1].timestamp > self.maximum_gap_sec
            or math.hypot(x - history[-1].x, y - history[-1].y) > 8.0
        ):
            history.clear()
        history.append(
            TrajectoryPoint(
                timestamp=timestamp,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                longitudinal_velocity=float(status.velocity.x),
                lateral_velocity=float(status.velocity.y),
                longitudinal_acceleration=float(status.acceleration.x),
                lateral_acceleration=float(status.acceleration.y),
                class_name=class_name,
                object_id=int(status.unique_id),
                asset_name=str(status.name),
            )
        )
        cutoff = timestamp - self.history_sec
        while len(history) > 1 and history[0].timestamp < cutoff:
            history.popleft()
        return history

    def make_object(self, class_name, status, history):
        current = history[-1]
        state = estimate_motion_state(tuple(history))
        output = TrackedObject()
        output.object_id.uuid = stable_uuid(
            class_name, status.unique_id, status.name
        )
        output.existence_probability = 1.0
        output.classification = [
            ObjectClassification(label=CLASS_LABELS[class_name], probability=1.0)
        ]
        output.kinematics.pose_with_covariance.pose.position.x = current.x
        output.kinematics.pose_with_covariance.pose.position.y = current.y
        output.kinematics.pose_with_covariance.pose.position.z = current.z
        output.kinematics.pose_with_covariance.pose.orientation = quaternion_from_yaw(
            current.yaw
        )
        output.kinematics.orientation_availability = (
            TrackedObjectKinematics.AVAILABLE
            if class_name == "vehicle"
            else TrackedObjectKinematics.SIGN_UNKNOWN
        )
        speed = state.speed if state is not None else 0.0
        acceleration = state.acceleration if state is not None else 0.0
        output.kinematics.twist_with_covariance.twist.linear.x = speed
        output.kinematics.acceleration_with_covariance.accel.linear.x = acceleration
        output.kinematics.is_stationary = abs(speed) < self.stationary_speed_mps
        output.shape.type = Shape.BOUNDING_BOX
        output.shape.dimensions.x = max(0.01, float(status.size.x))
        output.shape.dimensions.y = max(0.01, float(status.size.y))
        output.shape.dimensions.z = max(0.01, float(status.size.z))
        return output

    def callback(self, message):
        stamp = message.header.stamp
        if stamp.secs == 0 and stamp.nsecs == 0:
            stamp = rospy.Time.now()
        timestamp = stamp.to_sec()
        output = TrackedObjects()
        output.header.stamp = stamp
        output.header.frame_id = "map"
        groups = (
            ("vehicle", message.npc_list),
            ("pedestrian", message.pedestrian_list),
            ("obstacle", message.obstacle_list),
        )
        active_keys = set()
        for class_name, statuses in groups:
            for status in statuses:
                key = (class_name, int(status.unique_id), str(status.name))
                active_keys.add(key)
                history = self.update_history(class_name, status, timestamp)
                output.objects.append(self.make_object(class_name, status, history))

        stale_before = timestamp - self.maximum_gap_sec
        for key in list(self.histories):
            if key not in active_keys and self.histories[key][-1].timestamp < stale_before:
                del self.histories[key]
        self.publisher.publish(output)


def main():
    rospy.init_node("oracle_gt_tracks")
    OracleGtTracksNode()
    rospy.spin()


if __name__ == "__main__":
    main()
