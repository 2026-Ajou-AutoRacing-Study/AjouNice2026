#!/usr/bin/env python3
"""ROS1 Lanelet2 map-based vehicle predictor with kinematic fallback."""

from __future__ import annotations

import copy
import time

import rospy
from autoware_perception_msgs.msg import (
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    TrackedObjects,
)
from geometry_msgs.msg import Pose

from perception_object_pipeline.kinematic_prediction import (
    VEHICLE_LABELS,
    build_prediction_horizons,
    dominant_label,
    predict_pose,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    MapPredictionConfig,
)


def _copy_common_fields(tracked) -> PredictedObject:
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
    return predicted


def _kinematic_path(tracked, horizons) -> PredictedPath:
    path = PredictedPath()
    path.path = [
        copy.deepcopy(tracked.kinematics.pose_with_covariance.pose)
    ] + [predict_pose(tracked, horizon) for horizon in horizons]
    path.time_step = rospy.Duration(horizons[0])
    path.confidence = 1.0
    return path


def make_map_based_predicted_objects(message, horizons, network):
    output = PredictedObjects()
    output.header = copy.copy(message.header)
    mapped_count = 0
    fallback_count = 0
    path_count = 0

    for tracked in message.objects:
        predicted = _copy_common_fields(tracked)
        current = tracked.kinematics.pose_with_covariance.pose
        twist = tracked.kinematics.twist_with_covariance.twist
        label = dominant_label(tracked)
        trajectories = ()
        if label in VEHICLE_LABELS:
            trajectories = network.predict(
                x=current.position.x,
                y=current.position.y,
                z=current.position.z,
                yaw=quaternion_to_yaw(current.orientation),
                speed=twist.linear.x,
                horizons=horizons,
            )

        if trajectories:
            mapped_count += 1
            for trajectory in trajectories:
                path = PredictedPath()
                path.path = [copy.deepcopy(current)]
                for x, y, z, yaw in trajectory.poses:
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation = yaw_to_quaternion(yaw)
                    path.path.append(pose)
                path.time_step = rospy.Duration(horizons[0])
                path.confidence = trajectory.confidence
                predicted.kinematics.predicted_paths.append(path)
            path_count += len(trajectories)
        else:
            fallback_count += 1
            predicted.kinematics.predicted_paths = [_kinematic_path(tracked, horizons)]
            path_count += 1
        output.objects.append(predicted)

    return output, mapped_count, fallback_count, path_count


class MapBasedPredictionNode:
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
        config = MapPredictionConfig(
            maximum_lane_distance_m=float(
                rospy.get_param("~maximum_lane_distance_m", 3.0)
            ),
            maximum_lane_yaw_error_rad=float(
                rospy.get_param("~maximum_lane_yaw_error_rad", 0.7853981634)
            ),
            lane_distance_sigma_m=float(
                rospy.get_param("~lane_distance_sigma_m", 0.75)
            ),
            lane_yaw_sigma_rad=float(
                rospy.get_param("~lane_yaw_sigma_rad", 0.2617993878)
            ),
            minimum_speed_mps=float(rospy.get_param("~minimum_speed_mps", 0.5)),
            lateral_convergence_time_sec=float(
                rospy.get_param("~lateral_convergence_time_sec", 1.5)
            ),
            route_length_padding_m=float(
                rospy.get_param("~route_length_padding_m", 3.0)
            ),
            maximum_nearest_lanelets=int(
                rospy.get_param("~maximum_nearest_lanelets", 12)
            ),
            maximum_start_lanelets=int(
                rospy.get_param("~maximum_start_lanelets", 3)
            ),
            maximum_paths=int(rospy.get_param("~maximum_paths", 6)),
            maximum_route_depth=int(
                rospy.get_param("~maximum_route_depth", 12)
            ),
        )
        map_file = rospy.get_param("~map_file")
        projector_file = rospy.get_param("~projector_file")
        load_started = time.perf_counter()
        self.network = LaneletNetwork.load(map_file, projector_file, config)
        load_duration = time.perf_counter() - load_started

        self.publisher = rospy.Publisher(
            "~output/objects", PredictedObjects, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[map_based_prediction] loaded %d vehicle lanelets in %.3fs, "
            "map_sha256=%s, time_step=%.3fs, horizon=%.3fs",
            len(self.network.records),
            load_duration,
            self.network.map_sha256[:12],
            self.prediction_time_step,
            self.prediction_horizon,
        )

    def callback(self, message):
        if message.header.frame_id != "map":
            rospy.logwarn_throttle(
                1.0,
                "[map_based_prediction] expected map-frame tracks, got %s; "
                "dropping frame",
                message.header.frame_id or "<empty>",
            )
            return
        started = time.perf_counter()
        output, mapped, fallback, paths = make_map_based_predicted_objects(
            message, self.horizons, self.network
        )
        self.publisher.publish(output)
        rospy.loginfo_throttle(
            5.0,
            "[map_based_prediction] objects=%d mapped=%d fallback=%d paths=%d "
            "latency=%.2fms",
            len(message.objects),
            mapped,
            fallback,
            paths,
            (time.perf_counter() - started) * 1000.0,
        )


def main():
    rospy.init_node("map_based_prediction")
    MapBasedPredictionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
