#!/usr/bin/env python3

import math
import threading

import message_filters
import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry


PARAM_NS = "/localization_manager"


def param(name, default=None):
    return rospy.get_param(PARAM_NS + "/" + name, default)


def normalized_frame(frame):
    return frame.lstrip("/")


def quaternion_is_valid(quaternion):
    values = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float
    )
    return np.all(np.isfinite(values)) and np.linalg.norm(values) > 1.0e-9


def transform_matrix(translation, rotation):
    matrix = tft.quaternion_matrix(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    )
    matrix[0:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def pose_matrix(position, orientation):
    matrix = tft.quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    matrix[0:3, 3] = [position.x, position.y, position.z]
    return matrix


class MoraiMapCameraInitBroadcaster:
    def __init__(self):
        self.morai_map_frame = normalized_frame(
            param("frames/morai_map", "morai_map")
        )
        self.camera_init_frame = normalized_frame(
            param("frames/camera_init", "camera_init")
        )
        self.body_frame = normalized_frame(param("frames/body", "body"))
        self.base_link_frame = normalized_frame(
            param("frames/base_link", "base_link")
        )
        self.ego_topic = param("topics/ego", "/Ego_topic")
        self.odom_topic = param("topics/fast_lio_odom", "/localization/odom")
        self.initial_pose_topic = param(
            "topics/initial_pose", "/localization/initialpose"
        )
        self.provider = rospy.get_param("~provider", "fast_lio")
        if self.provider not in ("fast_lio", "fast_lio_gnss"):
            raise rospy.ROSInitException(
                "Unsupported localization provider '{}'".format(self.provider)
            )
        initialization_ns = "providers/{}/initialization".format(self.provider)
        self.max_time_difference = float(
            param(initialization_ns + "/max_time_difference_sec", 0.05)
        )
        heading_unit = str(
            param(initialization_ns + "/heading_unit", "degrees")
        ).strip().lower()
        if heading_unit not in ("degrees", "radians"):
            raise rospy.ROSInitException(
                "Unsupported FAST-LIO initialization heading unit '{}'; "
                "expected 'degrees' or 'radians'".format(heading_unit)
            )
        self.heading_in_degrees = heading_unit == "degrees"
        self.tf_timeout = float(param("runtime/tf_lookup_timeout_sec", 0.05))

        self.source = str(param(initialization_ns + "/source", "ego")).lower()
        if self.source not in ("ego", "manual"):
            raise rospy.ROSInitException(
                "Unsupported initialization source '{}'; expected ego or manual".format(
                    self.source
                )
            )

        self.lock = threading.Lock()
        self.morai_camera_init = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.initial_pose_publisher = rospy.Publisher(
            self.initial_pose_topic,
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )

        if self.source == "ego":
            self.ego_sync_sub = message_filters.Subscriber(
                self.ego_topic, EgoVehicleStatus
            )
            self.odom_sync_sub = message_filters.Subscriber(
                self.odom_topic, Odometry
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.ego_sync_sub, self.odom_sync_sub],
                queue_size=100,
                slop=self.max_time_difference,
                allow_headerless=False,
            )
            self.sync.registerCallback(self.initialize_from_ego)
        else:
            self.manual_frame = normalized_frame(
                param(initialization_ns + "/manual/frame_id", self.morai_map_frame)
            )
            self.manual_position = np.asarray(
                param(initialization_ns + "/manual/position", [0.0, 0.0, 0.0]),
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
            self.manual_odom_sub = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.initialize_from_manual,
                queue_size=100,
            )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.publish_alignment, queue_size=100
        )

        rospy.loginfo(
            "morai_map_camera_init_broadcaster provider=%s source=%s; initial pose topic=%s",
            self.provider,
            self.source,
            self.initial_pose_topic,
        )

    def initialize_from_ego(self, ego, odom):
        with self.lock:
            if self.morai_camera_init is not None:
                return

        if ego.header.stamp.is_zero() or odom.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Cannot initialize from a zero timestamp")
            return

        time_difference = abs((ego.header.stamp - odom.header.stamp).to_sec())
        if time_difference > self.max_time_difference:
            return

        if (
            normalized_frame(odom.header.frame_id) != self.camera_init_frame
            or normalized_frame(odom.child_frame_id) != self.body_frame
        ):
            rospy.logerr_throttle(
                2.0,
                "Expected odom frames %s -> %s, received %s -> %s",
                self.camera_init_frame,
                self.body_frame,
                odom.header.frame_id,
                odom.child_frame_id,
            )
            return

        ego_values = np.array(
            [ego.position.x, ego.position.y, ego.position.z, ego.heading], dtype=float
        )
        if not np.all(np.isfinite(ego_values)) or not quaternion_is_valid(
            odom.pose.pose.orientation
        ):
            rospy.logwarn_throttle(2.0, "Invalid Ego or FAST-LIO initialization pose")
            return

        yaw = ego.heading
        if self.heading_in_degrees:
            yaw = math.radians(yaw)
        ego_quaternion = tft.quaternion_from_euler(0.0, 0.0, yaw)
        morai_base = tft.quaternion_matrix(ego_quaternion)
        morai_base[0:3, 3] = ego_values[0:3]

        self.initialize_alignment(morai_base, odom, time_difference)

    def initialize_from_manual(self, odom):
        with self.lock:
            if self.morai_camera_init is not None:
                return
        manual_base = tft.euler_matrix(*self.manual_rpy)
        manual_base[0:3, 3] = self.manual_position
        if self.manual_frame != self.morai_map_frame:
            try:
                morai_manual_msg = self.tf_buffer.lookup_transform(
                    self.morai_map_frame,
                    self.manual_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout),
                )
            except tf2_ros.TransformException as error:
                rospy.logwarn_throttle(2.0, "Waiting for manual pose frame TF: %s", str(error))
                return
            morai_manual = transform_matrix(
                morai_manual_msg.transform.translation,
                morai_manual_msg.transform.rotation,
            )
            morai_base = np.dot(morai_manual, manual_base)
        else:
            morai_base = manual_base
        self.initialize_alignment(morai_base, odom, 0.0)

    def initialize_alignment(self, morai_base, odom, time_difference):
        if odom.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Cannot initialize from a zero odometry timestamp")
            return
        if (
            normalized_frame(odom.header.frame_id) != self.camera_init_frame
            or normalized_frame(odom.child_frame_id) != self.body_frame
        ):
            rospy.logerr_throttle(
                2.0,
                "Expected odom frames %s -> %s, received %s -> %s",
                self.camera_init_frame,
                self.body_frame,
                odom.header.frame_id,
                odom.child_frame_id,
            )
            return
        if not quaternion_is_valid(odom.pose.pose.orientation):
            rospy.logwarn_throttle(2.0, "Invalid FAST-LIO initialization orientation")
            return

        try:
            body_base_msg = self.tf_buffer.lookup_transform(
                self.body_frame,
                self.base_link_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0, "Waiting for initialization static TF: %s", str(error)
            )
            return

        camera_body = pose_matrix(
            odom.pose.pose.position, odom.pose.pose.orientation
        )
        body_base = transform_matrix(
            body_base_msg.transform.translation,
            body_base_msg.transform.rotation,
        )

        morai_camera = np.dot(
            np.dot(morai_base, np.linalg.inv(body_base)),
            np.linalg.inv(camera_body),
        )

        if not np.all(np.isfinite(morai_camera)):
            rospy.logerr("Computed morai_map -> camera_init contains non-finite values")
            return

        with self.lock:
            if self.morai_camera_init is not None:
                return
            self.morai_camera_init = morai_camera

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = odom.header.stamp
        initial_pose.header.frame_id = self.morai_map_frame
        initial_pose.pose.pose.position.x = morai_base[0, 3]
        initial_pose.pose.pose.position.y = morai_base[1, 3]
        initial_pose.pose.pose.position.z = morai_base[2, 3]
        base_quaternion = tft.quaternion_from_matrix(morai_base)
        initial_pose.pose.pose.orientation.x = base_quaternion[0]
        initial_pose.pose.pose.orientation.y = base_quaternion[1]
        initial_pose.pose.pose.orientation.z = base_quaternion[2]
        initial_pose.pose.pose.orientation.w = base_quaternion[3]
        self.initial_pose_publisher.publish(initial_pose)

        roll, pitch, aligned_yaw = tft.euler_from_matrix(morai_camera)
        rospy.loginfo(
            "Initialized %s -> %s: xyz=[%.6f, %.6f, %.6f], "
            "rpy_deg=[%.3f, %.3f, %.3f], time_difference=%.6f sec",
            self.morai_map_frame,
            self.camera_init_frame,
            morai_camera[0, 3],
            morai_camera[1, 3],
            morai_camera[2, 3],
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(aligned_yaw),
            time_difference,
        )

    def publish_alignment(self, odom):
        if odom.header.stamp.is_zero():
            return
        with self.lock:
            if self.morai_camera_init is None:
                return
            matrix = self.morai_camera_init.copy()

        quaternion = tft.quaternion_from_matrix(matrix)
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.morai_map_frame
        transform.child_frame_id = self.camera_init_frame
        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("morai_map_camera_init_broadcaster")
    MoraiMapCameraInitBroadcaster()
    rospy.spin()


if __name__ == "__main__":
    main()
