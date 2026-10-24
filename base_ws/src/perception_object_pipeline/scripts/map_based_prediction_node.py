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
    PredictedTrajectory,
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
    """Select native highway coverage or the legacy append-only experiment.

    Append mode preserves the entire baseline when its suffix cannot be added.
    Native mode requests a longer Map-CV path, truncated at real map terminals.
    A path can end at any sample up to the requested duration; its valid prefix
    is not replaced by CTRV. No tangent extrapolation or long CTRV is used.
    Long-horizon Map-CV may
    change route ranking/confidences. Map loading/frame checks remain mandatory.
    """

    def __init__(self, network, horizons, extended_horizons, selector, mode='append',
                 highway_direction_recovery=False):
        if mode not in ('append', 'native'):
            raise ValueError("highway prediction mode must be append or native")
        self.network = network
        self.horizons = tuple(horizons)
        self.extended_horizons = tuple(extended_horizons)
        self.selector = selector
        self.mode = mode
        self.highway_direction_recovery = highway_direction_recovery
        self.recovered_yaw = None
        self.lock = threading.RLock()
        self.extended_objects = 0
        self.extension_errors = 0
        self.reasons = {}
        self.output_horizons = self.horizons
        self.native_map_objects = 0
        self.direction_recovered_objects = 0
        self.native_short_objects = 0
        self.native_kinematic_objects = 0

    def begin(self, message):
        self.frame_available = True
        self.extended_objects = 0
        self.extension_errors = 0
        self.reasons = {}
        self.output_horizons = self.horizons
        self.native_map_objects = 0
        self.direction_recovered_objects = 0
        self.native_short_objects = 0
        self.native_kinematic_objects = 0
        try:
            self.selector.prune(message.header.stamp.to_sec(),
                                [tracked.object_id.uuid for tracked in message.objects])
        except Exception as error:
            self.frame_available = False
            self._error(error)

    def _error(self, error, retaining_highway_coverage=False):
        self.extension_errors += 1
        rospy.logwarn_throttle(5.0, "[map_based_prediction] optional horizon extension "
                               "unavailable; %s: %s",
                               ("retaining highway horizon with kinematic prediction"
                                if retaining_highway_coverage else "retaining original horizon"),
                               str(error))

    def _reason(self, name):
        self.reasons[name] = self.reasons.get(name, 0) + 1

    def _eligibility_paths(self, tracked, trajectories):
        if self.mode != 'native':
            return trajectories
        if dominant_label(tracked) not in VEHICLE_LABELS:
            return ()
        if trajectories:
            return trajectories
        # A road ending before three seconds has no baseline trajectory, but
        # can still have unambiguous current-lane evidence. Use the existing
        # candidate ordering/cap, not a new ranker or synthetic output path.
        pose = tracked.kinematics.pose_with_covariance.pose
        candidates = self.network.lane_candidates(
            pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation))
        speed = tracked.kinematics.twist_with_covariance.twist.linear.x
        if (not candidates and self.highway_direction_recovery and
                speed >= -self.network.config.minimum_speed_mps):
            self.recovered_yaw = self.network.recover_lane_direction(
                pose.position.x, pose.position.y, self.selector.highway_lanelet_ids)
            if self.recovered_yaw is not None:
                candidates = self.network.lane_candidates(
                    pose.position.x, pose.position.y, self.recovered_yaw)
        return tuple(PredictedTrajectory((), 1., (candidate.lanelet_id,))
                     for candidate in candidates)

    def _complete_map_routes(self, tracked, trajectories, horizons):
        """Reject fake coverage from a short/clamped or malformed map route."""
        if not trajectories:
            return False
        pose = tracked.kinematics.pose_with_covariance.pose
        speed = tracked.kinematics.twist_with_covariance.twist.linear.x
        if abs(speed) < self.network.config.minimum_speed_mps:
            speed = max(0., speed)
        for trajectory in trajectories:
            if (len(trajectory.poses) > len(horizons) or not trajectory.lanelet_ids
                    or not math.isfinite(trajectory.confidence)
                    or not 0. <= trajectory.confidence <= 1.
                    or any(len(sample) != 4 or not all(math.isfinite(v) for v in sample)
                           for sample in trajectory.poses)):
                return False
            if speed > 1.e-9 and len(trajectory.poses) > 1 and all(
                    abs(first - second) <= 1.e-9
                    for first, second in zip(trajectory.poses[-2][:3], trajectory.poses[-1][:3])):
                return False
            geometry, projection = self.network._route_geometry(trajectory.lanelet_ids)
            start = projection.project(pose.position.x, pose.position.y)
            # Ordinary Map-CV tolerates/clamps up to 0.5 m near an endpoint.
            # That is not a genuine full-horizon path, so native mode does not
            # publish it as if the NPC stopped at the last centerline point.
            if (start is None or not math.isfinite(geometry.length)
                    or not math.isfinite(start.arc_length) or speed < 0.):
                return False
            available = geometry.length - start.arc_length
            expected_count = len(horizons)
            if available < speed * horizons[-1] - 1.e-9:
                if self.network.records[trajectory.lanelet_ids[-1]].following_ids:
                    return False
                expected_count = sum(speed * t <= available + 1.e-9 for t in horizons)
            if len(trajectory.poses) != expected_count:
                return False
        return True

    def _complete_native_routes(self, tracked, trajectories):
        return self._complete_map_routes(tracked, trajectories, self.extended_horizons)

    def _native_prediction(self, tracked, baseline_trajectories):
        """Use the available map-only prefix up to the requested highway cap."""
        pose = tracked.kinematics.pose_with_covariance.pose
        speed = tracked.kinematics.twist_with_covariance.twist.linear.x
        if abs(speed) < self.network.config.minimum_speed_mps:
            speed = max(0., speed)
        yaw = (quaternion_to_yaw(pose.orientation) if self.recovered_yaw is None
               else self.recovered_yaw)
        try:
            if self.recovered_yaw is not None:
                self.direction_recovered_objects += 1
                self._reason('highway_direction_recovered')
                baseline_trajectories = self.network.predict(
                    x=pose.position.x, y=pose.position.y, z=pose.position.z,
                    yaw=yaw, speed=speed, horizons=self.horizons, truncate_at_route_end=True,
                    allow_low_speed=True)
            native = self.network.predict(
                x=pose.position.x, y=pose.position.y, z=pose.position.z,
                yaw=yaw, speed=speed,
                horizons=self.extended_horizons, truncate_at_route_end=True, allow_low_speed=True)
            if self._complete_native_routes(tracked, native):
                self.output_horizons = self.extended_horizons
                self.extended_objects += 1
                self.native_map_objects += 1
                self._reason('native_map')
                return native
            failure = 'no_map_paths' if not native else 'invalid_map_paths'
        except Exception as error:
            self._error(error)
            failure = 'exception'
        self.native_short_objects += 1
        self._reason('native_short_' + failure)
        # Preserve valid short Map-CV including all hypotheses/confidences.
        # If it is empty, the ordinary converter produces baseline CTRV.
        # Never replace a good map prefix with long CTRV or invented road.
        return baseline_trajectories

    def extend(self, tracked, trajectories, stamp):
        self.output_horizons = self.horizons
        self.recovered_yaw = None
        if not self.frame_available:
            return trajectories
        try:
            current = tracked.kinematics.pose_with_covariance.pose
            speed = tracked.kinematics.twist_with_covariance.twist.linear.x
            if self.mode == 'native' and not all(math.isfinite(v) for v in (
                    current.position.x, current.position.y, current.position.z,
                    quaternion_to_yaw(current.orientation), speed,
                    tracked.kinematics.twist_with_covariance.twist.angular.z)):
                # Reset only this identity, not other confirmed objects in the
                # same input frame. The selector already owns malformed-state
                # removal and its bounded per-ID cache.
                self.selector.decide(tracked.object_id.uuid, stamp.to_sec(),
                                     float('nan'), current.position.y, speed, ())
                self._reason('invalid_native_motion')
                return trajectories
            decision = self.selector.decide(tracked.object_id.uuid, stamp.to_sec(),
                                            current.position.x, current.position.y,
                                            speed, self._eligibility_paths(tracked, trajectories))
            self._reason(decision.reason)
            if not decision.extend:
                return trajectories
            if self.mode == 'native':
                return self._native_prediction(tracked, trajectories)
            if not trajectories:
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
                self._reason('route_not_extendable')
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
        object_horizons = horizons
        if label in VEHICLE_LABELS:
            trajectories = network.predict(
                x=current.position.x,
                y=current.position.y,
                z=current.position.z,
                yaw=quaternion_to_yaw(current.orientation),
                speed=twist.linear.x,
                horizons=horizons,
                truncate_at_route_end=True,
            )

        # Empty map paths / a non-vehicle classification also reset the
        # selector's continuous-highway history for this UUID.
        if mixed_extension is not None:
            trajectories = mixed_extension.extend(tracked, trajectories, message.header.stamp)
            object_horizons = getattr(mixed_extension, 'output_horizons', horizons)

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
                path.time_step = builtin_duration_from_seconds(object_horizons[0])
                path.confidence = trajectory.confidence
                predicted.kinematics.predicted_paths.append(path)
            path_count += len(trajectories)
        else:
            fallback_count += 1
            predicted.kinematics.predicted_paths = [_kinematic_path(tracked, object_horizons)]
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
                from perception_object_pipeline.mixed_horizon import (
                    MixedHorizonSelector, build_highway_horizons,
                )
                mode = rospy.get_param("~highway_prediction_mode", 'append')
                if mode not in ('append', 'native'):
                    raise ValueError("highway prediction mode must be append or native")
                if mode == 'append' and not math.isclose(self.prediction_horizon, 3.0, abs_tol=1.e-9):
                    raise ValueError("legacy append mode requires baseline 3.0s")
                extended_horizons = build_highway_horizons(
                    self.horizons, self.prediction_time_step,
                    rospy.get_param("~highway_prediction_horizon", 6.0))
                selector = MixedHorizonSelector(
                    self.network, current_lanelet_only=(mode == 'native'),
                    entry_confirm_sec=0. if mode == 'native' else .5)
                if not selector.highway_lanelet_ids:
                    raise ValueError("no explicit highway lanelet metadata; using baseline")
                self.mixed_extension = MixedHorizonExtension(
                    self.network, self.horizons, extended_horizons, selector, mode=mode,
                    highway_direction_recovery=bool(rospy.get_param('~highway_direction_recovery', True)))
                rospy.loginfo("[map_based_prediction] optional mixed horizon ON: "
                              "mode=%s baseline=%.1fs highway=%.1fs, highway_lanelets=%d",
                              mode, self.prediction_horizon, extended_horizons[-1], len(selector.highway_lanelet_ids))
            except Exception as error:
                rospy.logwarn("[map_based_prediction] optional mixed horizon disabled: %s; "
                              "continuing ordinary predictor", str(error))

        self.publisher = rospy.Publisher(
            "~output/objects", PredictedObjects, queue_size=1
        )
        self.publish_processing_diagnostics = bool(
            rospy.get_param("~publish_processing_diagnostics", False)
        )
        self.diagnostics_auto_on_subscribe = bool(
            rospy.get_param("~diagnostics_auto_on_subscribe", True)
        )
        self.processing_diagnostics_publisher = None
        self._processing_diagnostics_active = False
        if self.publish_processing_diagnostics or self.diagnostics_auto_on_subscribe:
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

    def _processing_diagnostics_for_callback(self):
        publisher = self.processing_diagnostics_publisher
        # Existing fixtures without the new switches retain their explicit
        # publisher-based behaviour. ROS constructors always set both switches.
        enabled = publisher is not None and getattr(
            self, "publish_processing_diagnostics", True
        )
        if publisher is not None and not enabled and getattr(
            self, "diagnostics_auto_on_subscribe", False
        ):
            try:
                # Local rospy connection state, not a parameter-server RPC.
                enabled = publisher.get_num_connections() > 0
            except Exception as error:
                rospy.logwarn_throttle(
                    5., "[map_based_prediction] optional diagnostics unavailable: %s", str(error)
                )
        if enabled and not getattr(self, "_processing_diagnostics_active", False):
            # A disconnected interval is not a callback-processing stall.
            self.processing_timing = NodeProcessingTiming()
        self._processing_diagnostics_active = enabled
        return publisher if enabled else None

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
        diagnostics = self._processing_diagnostics_for_callback()
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
                              mixed_horizon_reasons=dict(mixed_extension.reasons),
                              mixed_horizon_mode=mixed_extension.mode,
                              mixed_highway_horizon_sec=mixed_extension.extended_horizons[-1],
                              mixed_native_map_objects=mixed_extension.native_map_objects,
                              mixed_direction_recovered_objects=mixed_extension.direction_recovered_objects,
                              mixed_native_short_objects=mixed_extension.native_short_objects,
                              mixed_native_kinematic_objects=mixed_extension.native_kinematic_objects)
            diagnostics.publish(String(data=json.dumps(sample, allow_nan=False, separators=(",", ":"))))


def main():
    rospy.init_node("map_based_prediction")
    MapBasedPredictionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
