#!/usr/bin/env python3
"""Convert standardized tracked objects into planning-ready predictions.

Vehicle tracks use the team-validated CTRV model.  Pedestrians, bicycles and
unknown/obstacle objects use constant Cartesian velocity because their box yaw
is often sign-ambiguous in the MORAI training labels.
"""

from __future__ import annotations

import copy
import rospy
from autoware_perception_msgs.msg import (
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    TrackedObjects,
)
from perception_object_pipeline.kinematic_prediction import (
    build_prediction_horizons,
    predict_pose,
    yaw_to_quaternion,
)
from perception_object_pipeline.time_conversion import builtin_duration_from_seconds


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
        path.time_step = builtin_duration_from_seconds(horizons[0])
        path.confidence = 1.0
        predicted.kinematics.predicted_paths = [path]
        output.objects.append(predicted)
    return output


class ObjectPredictionNode:
    def __init__(self):
        self.prediction_time_step = float(
            rospy.get_param("~prediction_time_step", 0.1)
        )
        self.prediction_horizon = float(
            rospy.get_param("~prediction_horizon", 3.0)
        )
        self.horizons = build_prediction_horizons(
            self.prediction_time_step, self.prediction_horizon
        )
        self.publisher = rospy.Publisher(
            "~output/objects", PredictedObjects, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[object_prediction] standardized CTRV/CV prediction, "
            "time_step=%.3fs, horizon=%.3fs, path_points=%d",
            self.prediction_time_step,
            self.prediction_horizon,
            len(self.horizons) + 1,
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
