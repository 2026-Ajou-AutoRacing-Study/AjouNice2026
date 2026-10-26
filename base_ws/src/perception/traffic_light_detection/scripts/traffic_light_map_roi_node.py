#!/usr/bin/env python3
"""System-Python timestamped TF/odometry bridge for opt-in map association.

No Ego/native simulator input, no TF broadcaster, no map/planning mutation.
Failures publish explicit missing packets and do not terminate the process.
"""
import json
import math
import threading
from collections import deque

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt64

from traffic_light_map_selector import (RigidTransform, load_camera, load_geometry, project_heads,
                                        covariance_sigmas, pose_tf_consistency, selector_config,
                                        DEFAULT_SELECTOR_CONFIG, SELECTOR_CONTRACT_VERSION)


def transform_parts(message):
    t, q = message.transform.translation, message.transform.rotation
    return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)


def motion_evidence(poses, aligned_pose, now, map_frame, base_frame):
    """Evidence only: no velocity inference from displacement or Ego/native data.

    Use both the image-aligned pose and the latest received localization sample.
    Missing/invalid motion does not invalidate otherwise useful map projection.
    """
    result = {"schema_version": 1, "valid": False}
    try:
        latest, receipt = poses[-1]
        for pose in (aligned_pose, latest):
            if (pose.header.frame_id.strip('/') != map_frame.strip('/') or
                    pose.child_frame_id.strip('/') != base_frame.strip('/')):
                return result
        speeds = [math.sqrt(sum(float(getattr(pose.twist.twist.linear, axis))**2
                                for axis in 'xyz')) for pose in (aligned_pose, latest)]
        if not all(math.isfinite(v) for v in speeds):
            return result
        result.update(valid=True, aligned_pose_stamp_ns=aligned_pose.header.stamp.to_nsec(),
                      latest_pose_stamp_ns=latest.header.stamp.to_nsec(),
                      latest_pose_receipt_stamp_ns=receipt.to_nsec(),
                      aligned_speed_mps=speeds[0], latest_speed_mps=speeds[1])
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        pass
    return result


class TrafficLightMapRoiNode:
    def __init__(self):
        self.geometry = load_geometry(rospy.get_param("~geometry_path"))
        self.camera = load_camera(rospy.get_param("~sensor_config"))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.route_stale = float(rospy.get_param("~route_group_id_stale_sec", 2.))
        self.pose_delta = float(rospy.get_param("~max_pose_image_delta_sec", .15))
        self.pose_age = float(rospy.get_param("~max_pose_receipt_age_sec", .5))
        self.selector_config = selector_config({key: rospy.get_param("~"+key, value)
                                               for key, value in DEFAULT_SELECTOR_CONFIG.to_dict().items()})
        self.max_position_sigma = self.selector_config.max_position_sigma_m
        self.max_vertical_sigma = self.selector_config.max_vertical_sigma_m
        self.max_yaw_sigma = self.selector_config.max_yaw_sigma_degrees
        self.projection_options = self.selector_config.projection_kwargs()
        self._lock = threading.Lock()
        self._poses = deque(maxlen=100)
        self._group_id, self._group_receipt = 0, rospy.Time(0)
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher(rospy.get_param("~output_topic"), String, queue_size=3)
        self.route_sub = rospy.Subscriber(rospy.get_param("~route_group_id_topic"), UInt64,
                                          self.route_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(rospy.get_param("~odometry_topic", "/localization/kinematic_state"),
                                         Odometry, self.pose_callback, queue_size=30)
        self.image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/sensor/camera/front/compressed"),
                                          CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)

    def route_callback(self, message):
        with self._lock:
            self._group_id, self._group_receipt = int(message.data), rospy.Time.now()

    def pose_callback(self, message):
        with self._lock:
            self._poses.append((message, rospy.Time.now()))

    def image_callback(self, message):
        now, stamp = rospy.Time.now(), message.header.stamp
        with self._lock:
            group_id, group_receipt, poses = self._group_id, self._group_receipt, list(self._poses)
        packet = {"schema_version": 1, "source_stamp_ns": stamp.to_nsec(),
                  "selector_contract_version": SELECTOR_CONTRACT_VERSION,
                  "selector_config": self.selector_config.to_dict(),
                  "generated_stamp_ns": now.to_nsec(), "camera_frame": self.camera.frame,
                  "geometry_frame": self.geometry["geometry_frame"], "group_id": group_id,
                  "image_width": self.camera.width, "image_height": self.camera.height,
                  "status": "missing", "reason": "not_ready", "rois": [], "rejected_heads": [],
                  "visibility_ground_truth": False, "additional_heading_correction_degrees": 0.}
        try:
            route_age = (now-group_receipt).to_sec()
            if group_id <= 0 or not 0 <= route_age <= self.route_stale:
                raise ValueError("route_missing_or_stale")
            if not any(head.get("group_id") == group_id for head in self.geometry["heads"]):
                raise ValueError("route_group_has_no_vehicle_geometry")
            if stamp.to_nsec() <= 0:
                raise ValueError("image_stamp_missing")
            if message.header.frame_id.strip("/") != self.camera.frame.strip("/"):
                raise ValueError("image_camera_frame_mismatch")
            if not poses:
                raise ValueError("localization_missing")
            pose, receipt = min(poses, key=lambda item: abs((item[0].header.stamp-stamp).to_sec()))
            delta, receipt_age = abs((pose.header.stamp-stamp).to_sec()), (now-receipt).to_sec()
            if pose.header.stamp.to_nsec() <= 0 or delta > self.pose_delta or not 0 <= receipt_age <= self.pose_age:
                raise ValueError("localization_stale_or_not_image_aligned")
            if (pose.header.frame_id.strip("/") != self.map_frame.strip("/")
                    or pose.child_frame_id.strip("/") != self.base_frame.strip("/")):
                raise ValueError("localization_frame_mismatch")
            position_sigma, vertical_sigma, yaw_sigma = covariance_sigmas(pose.pose.covariance,
                self.max_position_sigma, self.max_yaw_sigma, self.max_vertical_sigma)
            body_tf = self.buffer.lookup_transform(self.map_frame, self.base_frame,
                                                   stamp, rospy.Duration(.03))
            tf_position, tf_quaternion = transform_parts(body_tf)
            p, q, twist = pose.pose.pose.position, pose.pose.pose.orientation, pose.twist.twist
            consistency = pose_tf_consistency((p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                tf_position, tf_quaternion, delta,
                math.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2), twist.angular.z)
            # Validate the actual existing sensor TF against the explicitly loaded calibration.
            # Time(0) is used ONLY for this fixed extrinsic; dynamic world TF uses image time.
            extrinsic = self.buffer.lookup_transform(self.base_frame, self.camera.frame,
                                                     rospy.Time(0), rospy.Duration(.03))
            translation, quaternion = transform_parts(extrinsic)
            if not all(math.isfinite(v) for v in translation + quaternion):
                raise ValueError("nonfinite_camera_tf")
            translation_error = math.sqrt(sum((translation[i]-self.camera.position[i])**2 for i in range(3)))
            qnorm = math.sqrt(sum(v*v for v in quaternion))
            if qnorm < 1e-9:
                raise ValueError("invalid_camera_tf_quaternion")
            dot = min(1., abs(sum(quaternion[i]/qnorm*self.camera.quaternion[i] for i in range(4))))
            angle_error = math.degrees(2*math.acos(dot))
            if translation_error > .02 or angle_error > .2:
                raise ValueError("camera_tf_calibration_mismatch")
            actual = self.buffer.lookup_transform(self.camera.frame, self.geometry["geometry_frame"],
                                                  stamp, rospy.Duration(.04))
            transform = RigidTransform.from_quaternion(*transform_parts(actual))
            rois, rejected = project_heads(self.geometry, self.camera, transform, group_id,
                position_sigma, yaw_sigma, vertical_sigma_m=vertical_sigma, **self.projection_options)
            packet.update(status="ready", reason="projected" if any(r["eligible"] for r in rois)
                          else "no_visible_route_head", rois=rois, rejected_heads=rejected,
                          pose_stamp_ns=pose.header.stamp.to_nsec(), pose_image_delta_sec=delta,
                          motion_evidence=motion_evidence(poses, pose, now, self.map_frame, self.base_frame),
                          position_sigma_m=position_sigma, yaw_sigma_radians=yaw_sigma,
                          position_sigma_xy_m=position_sigma, position_sigma_z_m=vertical_sigma,
                          position_sigma_m_legacy_alias_means_XY_only=True,
                          pose_covariance_diagonal_xyz_yaw=[pose.pose.covariance[i] for i in (0, 7, 14, 35)],
                          uncertainty_model="separate_XY_GNSS_Z_with_vertical_depth_coupling_v2",
                          localization_tf_position_error_m=consistency[0],
                          localization_tf_angle_error_degrees=consistency[1],
                          camera_tf_translation_error_m=translation_error,
                          camera_tf_angle_error_degrees=angle_error)
        except (ValueError, KeyError, TypeError, OverflowError, tf2_ros.TransformException) as error:
            packet["reason"] = str(error)
            rospy.logwarn_throttle(5., "[Traffic map ROI] missing: %s", error)
        packet["generated_stamp_ns"] = rospy.Time.now().to_nsec()
        self.pub.publish(String(data=json.dumps(packet, allow_nan=False, separators=(",", ":"))))


if __name__ == "__main__":
    rospy.init_node("traffic_light_map_roi")
    TrafficLightMapRoiNode()
    rospy.spin()
