#!/usr/bin/env python3

import math
import threading

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from morai_msgs.msg import EgoVehicleStatus


PARAM_NS = "/localization_manager"


def param(name, default=None):
    return rospy.get_param(PARAM_NS + "/" + name, default)


def normalized_frame(frame):
    return frame.lstrip("/")


def transform_matrix(translation, rotation):
    matrix = tft.quaternion_matrix(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    )
    matrix[0:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class GnssImuInitialPosePublisher:
    """Publish the configured competition start pose once for the global EKF."""

    def __init__(self):
        self.lock = threading.Lock()
        self.initialized = False

        self.map_frame = normalized_frame(param("frames/map", "map"))
        self.morai_map_frame = normalized_frame(
            param("frames/morai_map", "morai_map")
        )
        self.ego_topic = param("topics/ego", "/Ego_topic")
        self.output_topic = param(
            "topics/initial_pose", "/localization/initialpose"
        )
        self.tf_timeout = float(param("runtime/tf_lookup_timeout_sec", 0.05))

        initialization_ns = "providers/gnss_imu/initialization"
        self.source = str(
            param(initialization_ns + "/source", "ego")
        ).strip().lower()
        if self.source not in ("ego", "manual"):
            raise rospy.ROSInitException(
                "Unsupported GNSS+IMU initialization source '{}'; "
                "expected ego or manual".format(self.source)
            )

        heading_unit = str(
            param(initialization_ns + "/heading_unit", "degrees")
        ).strip().lower()
        if heading_unit not in ("degrees", "radians"):
            raise rospy.ROSInitException(
                "Unsupported heading unit '{}'; expected degrees or radians".format(
                    heading_unit
                )
            )
        self.heading_in_degrees = heading_unit == "degrees"
        self.use_heading_correction = bool(
            rospy.get_param("~use_heading_correction", False)
        )
        self.heading_yaw_offset_deg = float(
            rospy.get_param("~heading_yaw_offset_deg", 0.0)
        )
        if not math.isfinite(self.heading_yaw_offset_deg):
            raise rospy.ROSInitException(
                "~heading_yaw_offset_deg must be finite"
            )
        self.heading_yaw_offset_rad = math.radians(
            self.heading_yaw_offset_deg
        )

        position_stddev = self.positive_param(
            initialization_ns + "/position_stddev_m", 0.1
        )
        yaw_stddev = self.positive_param(
            initialization_ns + "/yaw_stddev_rad", math.radians(1.0)
        )
        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = position_stddev**2
        self.pose_covariance[7] = position_stddev**2
        self.pose_covariance[14] = 1.0e6
        self.pose_covariance[21] = 1.0e6
        self.pose_covariance[28] = 1.0e6
        self.pose_covariance[35] = yaw_stddev**2

        self.publisher = rospy.Publisher(
            self.output_topic,
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ego_subscriber = None
        self.manual_timer = None
        if self.source == "ego":
            self.ego_subscriber = rospy.Subscriber(
                self.ego_topic,
                EgoVehicleStatus,
                self.ego_callback,
                queue_size=100,
            )
        else:
            self.manual_frame = normalized_frame(
                param(
                    initialization_ns + "/manual/frame_id",
                    self.morai_map_frame,
                )
            )
            self.manual_position = np.asarray(
                param(
                    initialization_ns + "/manual/position",
                    [0.0, 0.0, 0.0],
                ),
                dtype=float,
            )
            self.manual_rpy = np.radians(
                np.asarray(
                    param(
                        initialization_ns + "/manual/rpy_degrees",
                        [0.0, 0.0, 0.0],
                    ),
                    dtype=float,
                )
            )
            if self.manual_position.shape != (3,) or self.manual_rpy.shape != (3,):
                raise rospy.ROSInitException(
                    "manual position and rpy_degrees must each contain three values"
                )
            if not np.all(np.isfinite(self.manual_position)) or not np.all(
                np.isfinite(self.manual_rpy)
            ):
                raise rospy.ROSInitException(
                    "manual position and rpy_degrees must be finite"
                )
            self.manual_timer = rospy.Timer(
                rospy.Duration(0.05), self.manual_timer_callback
            )

        rospy.loginfo(
            "gnss_imu_initial_pose_publisher source=%s: %s -> %s, "
            "Ego heading correction=%s (%+.3f deg)",
            self.source,
            self.ego_topic if self.source == "ego" else self.manual_frame,
            self.output_topic,
            self.use_heading_correction,
            self.heading_yaw_offset_deg,
        )

    @staticmethod
    def positive_param(name, default):
        value = float(param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException(
                "{} must be a positive finite value".format(
                    PARAM_NS + "/" + name
                )
            )
        return value

    def transform_pose_to_map(self, source_frame, source_pose):
        if source_frame == self.map_frame:
            return source_pose
        try:
            map_from_source_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for initial-pose TF %s -> %s: %s",
                source_frame,
                self.map_frame,
                str(error),
            )
            return None
        map_from_source = transform_matrix(
            map_from_source_msg.transform.translation,
            map_from_source_msg.transform.rotation,
        )
        return np.dot(map_from_source, source_pose)

    def publish_once(self, stamp, source_frame, source_pose):
        with self.lock:
            if self.initialized:
                return
            map_pose = self.transform_pose_to_map(source_frame, source_pose)
            if map_pose is None:
                return

            quaternion = tft.quaternion_from_matrix(map_pose)
            message = PoseWithCovarianceStamped()
            message.header.stamp = stamp
            message.header.frame_id = self.map_frame
            message.pose.pose.position.x = map_pose[0, 3]
            message.pose.pose.position.y = map_pose[1, 3]
            message.pose.pose.position.z = map_pose[2, 3]
            message.pose.pose.orientation.x = quaternion[0]
            message.pose.pose.orientation.y = quaternion[1]
            message.pose.pose.orientation.z = quaternion[2]
            message.pose.pose.orientation.w = quaternion[3]
            message.pose.covariance = self.pose_covariance
            self.publisher.publish(message)
            self.initialized = True

        rospy.loginfo(
            "Published GNSS+IMU initial pose from %s at %.9f: "
            "map position=[%.3f, %.3f, %.3f]",
            self.source,
            stamp.to_sec(),
            map_pose[0, 3],
            map_pose[1, 3],
            map_pose[2, 3],
        )

    def ego_callback(self, message):
        if self.initialized:
            return
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(
                2.0, "Cannot initialize GNSS+IMU from zero-stamped Ego"
            )
            return
        values = np.asarray(
            [
                message.position.x,
                message.position.y,
                message.position.z,
                message.heading,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            rospy.logwarn_throttle(
                2.0, "Cannot initialize GNSS+IMU from non-finite Ego pose"
            )
            return

        yaw = message.heading
        if self.heading_in_degrees:
            yaw = math.radians(yaw)
        if self.use_heading_correction:
            yaw += self.heading_yaw_offset_rad
        source_pose = tft.euler_matrix(0.0, 0.0, yaw)
        source_pose[0:3, 3] = values[0:3]
        self.publish_once(
            message.header.stamp, self.morai_map_frame, source_pose
        )

    def manual_timer_callback(self, _event):
        if self.initialized:
            if self.manual_timer is not None:
                self.manual_timer.shutdown()
            return
        stamp = rospy.Time.now()
        if stamp.is_zero():
            return
        source_pose = tft.euler_matrix(*self.manual_rpy)
        source_pose[0:3, 3] = self.manual_position
        self.publish_once(stamp, self.manual_frame, source_pose)


def main():
    rospy.init_node("gnss_imu_initial_pose_publisher")
    GnssImuInitialPosePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
