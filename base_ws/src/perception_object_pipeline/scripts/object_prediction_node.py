#!/usr/bin/env python3
"""Convert standardized tracked objects into planning-ready predictions.

Vehicle tracks use the team-validated CTRV model.  Pedestrians, bicycles and
unknown/obstacle objects use constant Cartesian velocity because their box yaw
is often sign-ambiguous in the MORAI training labels.
"""

from __future__ import annotations

import copy
import math

import rospy
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    TrackedObjects,
)
from geometry_msgs.msg import Pose, Quaternion


OMEGA_EPS = 1.0e-3
VEHICLE_LABELS = {
    ObjectClassification.CAR,
    ObjectClassification.TRUCK,
    ObjectClassification.BUS,
    ObjectClassification.TRAILER,
}


def quaternion_to_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def ctrv_predict(x: float, y: float, yaw: float, speed: float, yaw_rate: float, dt: float):
    if abs(yaw_rate) < OMEGA_EPS:
        return (
            x + speed * dt * math.cos(yaw),
            y + speed * dt * math.sin(yaw),
            yaw,
        )
    next_yaw = yaw + yaw_rate * dt
    return (
        x + speed / yaw_rate * (math.sin(next_yaw) - math.sin(yaw)),
        y + speed / yaw_rate * (math.cos(yaw) - math.cos(next_yaw)),
        next_yaw,
    )


def dominant_label(tracked) -> int:
    if not tracked.classification:
        return ObjectClassification.UNKNOWN
    return max(tracked.classification, key=lambda item: item.probability).label


def predict_pose(tracked, horizon: float) -> Pose:
    current = tracked.kinematics.pose_with_covariance.pose
    twist = tracked.kinematics.twist_with_covariance.twist
    yaw = quaternion_to_yaw(current.orientation)
    label = dominant_label(tracked)

    if label in VEHICLE_LABELS:
        x, y, predicted_yaw = ctrv_predict(
            current.position.x,
            current.position.y,
            yaw,
            twist.linear.x,
            twist.angular.z,
            horizon,
        )
    else:
        # Twist is expressed in the object body frame. Rotate its longitudinal
        # and lateral components into map coordinates for VRU/unknown CV.
        velocity_x = math.cos(yaw) * twist.linear.x - math.sin(yaw) * twist.linear.y
        velocity_y = math.sin(yaw) * twist.linear.x + math.cos(yaw) * twist.linear.y
        x = current.position.x + velocity_x * horizon
        y = current.position.y + velocity_y * horizon
        predicted_yaw = yaw
        if math.hypot(velocity_x, velocity_y) > 0.2:
            predicted_yaw = math.atan2(velocity_y, velocity_x)

    output = Pose()
    output.position.x = x
    output.position.y = y
    output.position.z = current.position.z
    output.orientation = yaw_to_quaternion(predicted_yaw)
    return output


def make_predicted_objects(message: TrackedObjects, horizons) -> PredictedObjects:
    output = PredictedObjects()
    output.header = copy.copy(message.header)
    for tracked in message.objects:
        predicted = PredictedObject()
        predicted.object_id = tracked.object_id
        predicted.existence_probability = tracked.existence_probability
        predicted.classification = copy.deepcopy(tracked.classification)
        predicted.shape = copy.deepcopy(tracked.shape)
        predicted.kinematics.initial_pose_with_covariance = copy.deepcopy(
            tracked.kinematics.pose_with_covariance
        )
        predicted.kinematics.initial_twist_with_covariance = copy.deepcopy(
            tracked.kinematics.twist_with_covariance
        )
        predicted.kinematics.initial_acceleration_with_covariance = copy.deepcopy(
            tracked.kinematics.acceleration_with_covariance
        )

        path = PredictedPath()
        # Autoware's PredictedPath convention places the current object pose at
        # index 0; subsequent entries are separated by ``time_step``.
        path.path = [
            copy.deepcopy(tracked.kinematics.pose_with_covariance.pose)
        ] + [predict_pose(tracked, horizon) for horizon in horizons]
        path.time_step = rospy.Duration(horizons[0])
        path.confidence = 1.0
        predicted.kinematics.predicted_paths = [path]
        output.objects.append(predicted)
    return output


class ObjectPredictionNode:
    def __init__(self):
        self.horizons = [
            float(value)
            for value in rospy.get_param("~horizons", [0.3, 0.6, 0.9, 1.2, 1.5])
        ]
        self._validate_horizons()
        self.publisher = rospy.Publisher(
            "~output/objects", PredictedObjects, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[object_prediction] standardized CTRV/CV prediction, horizons=%s",
            self.horizons,
        )

    def _validate_horizons(self):
        if not self.horizons or self.horizons[0] <= 0.0:
            raise ValueError("~horizons must contain positive values")
        step = self.horizons[0]
        for index, horizon in enumerate(self.horizons, start=1):
            if abs(horizon - step * index) > 1.0e-6:
                raise ValueError(
                    "~horizons must be uniformly spaced from one time_step; "
                    f"received {self.horizons}"
                )

    def callback(self, message):
        if message.header.frame_id != "map":
            rospy.logwarn_throttle(
                1.0,
                "[object_prediction] expected map-frame tracks, got %s; dropping frame",
                message.header.frame_id or "<empty>",
            )
            return
        self.publisher.publish(make_predicted_objects(message, self.horizons))


def main():
    rospy.init_node("object_prediction")
    ObjectPredictionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
