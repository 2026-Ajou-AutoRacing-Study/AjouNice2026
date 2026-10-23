#!/usr/bin/env python3
"""ROS1 Lanelet2 map-based vehicle predictor with kinematic fallback."""

from __future__ import annotations

import copy
import json
import math
import threading
import time

import rospy
from autoware_perception_msgs.msg import (
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    TrackedObjects,
)
from geometry_msgs.msg import Pose
from std_msgs.msg import String

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
from perception_object_pipeline.message_copy import (
    copy_accel_covariance, copy_classification, copy_pose,
    copy_pose_covariance, copy_shape, copy_twist_covariance,
)
from perception_object_pipeline.time_conversion import builtin_duration_from_seconds
from perception_object_pipeline.node_processing_timing import NodeProcessingTiming


def _copy_common_fields(tracked) -> PredictedObject:
    predicted = PredictedObject()
    predicted.object_id = copy.copy(tracked.object_id)
    predicted.object_id.uuid = copy.copy(tracked.object_id.uuid)
    predicted.existence_probability = tracked.existence_probability
    predicted.classification = copy_classification(tracked.classification)
    predicted.shape = copy_shape(tracked.shape)
    predicted.kinematics.initial_pose_with_covariance = copy_pose_covariance(
        tracked.kinematics.pose_with_covariance
    )
    predicted.kinematics.initial_twist_with_covariance = copy_twist_covariance(
        tracked.kinematics.twist_with_covariance
    )
    predicted.kinematics.initial_acceleration_with_covariance = copy_accel_covariance(
        tracked.kinematics.acceleration_with_covariance
    )
    return predicted


def _kinematic_path(tracked, horizons) -> PredictedPath:
    path = PredictedPath()
    path.path = [
        copy_pose(tracked.kinematics.pose_with_covariance.pose)
    ] + [predict_pose(tracked, horizon) for horizon in horizons]
    path.time_step = builtin_duration_from_seconds(horizons[0])
    path.confidence = 1.0
    return path


class MixedHorizonExtension:
    """Optional append-only extension; the ordinary predictor remains authoritative.

    Failures in this optional path retain the *whole object's* original paths.
    Required map loading / frame validation deliberately remain outside it.
    """

    def __init__(self, network, horizons, extended_horizons, selector):
        self.network = network
        self.horizons = tuple(horizons)
        self.extended_horizons = tuple(extended_horizons)
        self.selector = selector
        self.lock = threading.RLock()
        self.extended_objects = 0
        self.extension_errors = 0
        self.reasons = {}

    def begin(self, message):
        self.frame_available = True
        self.extended_objects = 0
        self.extension_errors = 0
        self.reasons = {}
        try:
            self.selector.prune(message.header.stamp.to_sec(),
                                [tracked.object_id.uuid for tracked in message.objects])
        except Exception as error:
            self.frame_available = False
            self._error(error)

    def _error(self, error):
        self.extension_errors += 1
        rospy.logwarn_throttle(5.0, "[map_based_prediction] optional horizon extension "
                               "unavailable; retaining original horizon: %s", str(error))

    def extend(self, tracked, trajectories, stamp):
        if not self.frame_available:
            return trajectories
        try:
            current = tracked.kinematics.pose_with_covariance.pose
            speed = tracked.kinematics.twist_with_covariance.twist.linear.x
            decision = self.selector.decide(tracked.object_id.uuid, stamp.to_sec(),
                                            current.position.x, current.position.y,
                                            speed, trajectories)
            self.reasons[decision.reason] = self.reasons.get(decision.reason, 0) + 1
            if not decision.extend or not trajectories:
                return trajectories
            extended = self.network.extend_trajectories(
                current.position.x, current.position.y, current.position.z,
                quaternion_to_yaw(current.orientation), speed, self.horizons,
                self.extended_horizons, trajectories,
                allowed_lanelet_ids=self.selector.highway_lanelet_ids)
            # Defensive contract check, before converting any optional results
            # into ROS messages. Never publish a partially extended object.
            if len(extended) != len(trajectories) or any(
                    new.confidence != old.confidence or
                    new.lanelet_ids[:len(old.lanelet_ids)] != old.lanelet_ids or
                    new.poses[:len(old.poses)] != old.poses or
                    len(new.poses) != len(self.extended_horizons)
                    for old, new in zip(trajectories, extended)):
                self.reasons['route_not_extendable'] = self.reasons.get('route_not_extendable', 0) + 1
                return trajectories
            self.extended_objects += 1
            return extended
        except Exception as error:
            self._error(error)
            return trajectories


def make_map_based_predicted_objects(message, horizons, network, mixed_extension=None):
    # Only the opt-in stateful selector needs serialization across subscriber
    # connections. Feature OFF follows the original computation path.
    if mixed_extension is not None:
        with mixed_extension.lock:
            mixed_extension.begin(message)
            return _make_map_based_predicted_objects(message, horizons, network, mixed_extension)
    return _make_map_based_predicted_objects(message, horizons, network)


def _make_map_based_predicted_objects(message, horizons, network, mixed_extension=None):
    output = PredictedObjects()
    output.header = copy.copy(message.header)
    output.header.stamp = copy.copy(message.header.stamp)
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

        # Empty map paths / a non-vehicle classification also reset the
        # selector's continuous-highway history for this UUID.
        if mixed_extension is not None:
            trajectories = mixed_extension.extend(tracked, trajectories, message.header.stamp)

        if trajectories:
            mapped_count += 1
            for trajectory in trajectories:
                path = PredictedPath()
                path.path = [copy_pose(current)]
                for x, y, z, yaw in trajectory.poses:
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation = yaw_to_quaternion(yaw)
                    path.path.append(pose)
                path.time_step = builtin_duration_from_seconds(horizons[0])
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
            trajectory_cluster_distance_m=float(
                rospy.get_param("~trajectory_cluster_distance_m", 0.75)
            ),
        )
        map_file = rospy.get_param("~map_file")
        projector_file = rospy.get_param("~projector_file")
        load_started = time.perf_counter()
        self.network = LaneletNetwork.load(map_file, projector_file, config)
        load_duration = time.perf_counter() - load_started

        self.mixed_extension = None
        if bool(rospy.get_param("~mixed_horizon_enabled", False)):
            try:
                # Optional feature has no additional map file, service or
                # credential dependency. Unknown semantic metadata means 3s.
                from perception_object_pipeline.mixed_horizon import MixedHorizonSelector
                if (not math.isclose(self.prediction_horizon, 3.0, abs_tol=1.e-9) or
                        not math.isclose(self.prediction_time_step, 0.1, abs_tol=1.e-9)):
                    raise ValueError("mixed horizon candidate requires baseline 3.0s / 0.1s")
                selector = MixedHorizonSelector(self.network)
                if not selector.highway_lanelet_ids:
                    raise ValueError("no explicit highway lanelet metadata; using baseline")
                self.mixed_extension = MixedHorizonExtension(
                    self.network, self.horizons, build_prediction_horizons(0.1, 6.0), selector)
                rospy.loginfo("[map_based_prediction] optional mixed horizon ON: "
                              "baseline=3s highway=6s, highway_lanelets=%d",
                              len(selector.highway_lanelet_ids))
            except Exception as error:
                rospy.logwarn("[map_based_prediction] optional mixed horizon disabled: %s; "
                              "continuing ordinary predictor", str(error))

        self.publisher = rospy.Publisher(
            "~output/objects", PredictedObjects, queue_size=1
        )
        self.processing_diagnostics_publisher = None
        if bool(rospy.get_param("~publish_processing_diagnostics", False)):
            self.processing_diagnostics_publisher = rospy.Publisher(
                "~diagnostics/processing_time_ms", String, queue_size=1
            )
            self.processing_timing = NodeProcessingTiming()
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
        diagnostics = self.processing_diagnostics_publisher
        token = self.processing_timing.start(message, rospy.Time.now().to_nsec()) if diagnostics else None
        make_start = time.perf_counter() if diagnostics else None
        mixed_extension = getattr(self, 'mixed_extension', None)
        output, mapped, fallback, paths = make_map_based_predicted_objects(
            message, self.horizons, self.network, mixed_extension)
        make_end = time.perf_counter() if diagnostics else None
        self.publisher.publish(output)
        publish_end = time.perf_counter() if diagnostics else None
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
        if diagnostics:
            sample = self.processing_timing.finish(token, make_start, make_end, publish_end,
                                                  mapped=mapped, fallback=fallback, paths=paths)
            if mixed_extension is not None:
                sample.update(mixed_extended_objects=mixed_extension.extended_objects,
                              mixed_extension_errors=mixed_extension.extension_errors,
                              mixed_horizon_reasons=dict(mixed_extension.reasons))
            diagnostics.publish(String(data=json.dumps(sample, allow_nan=False, separators=(",", ":"))))


def main():
    rospy.init_node("map_based_prediction")
    MapBasedPredictionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
