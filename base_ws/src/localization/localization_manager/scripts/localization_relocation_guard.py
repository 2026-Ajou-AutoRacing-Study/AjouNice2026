#!/usr/bin/env python3

"""Detect large, persistent map-pose relocations without changing production by default."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RelocationDetector:
    """Pure state machine used by both the ROS node and unit tests."""

    def __init__(self, threshold_m=5.0, consecutive_samples=3, cooldown_sec=5.0):
        if not math.isfinite(threshold_m) or threshold_m <= 0.0:
            raise ValueError("threshold_m must be positive and finite")
        if consecutive_samples < 1:
            raise ValueError("consecutive_samples must be at least one")
        if not math.isfinite(cooldown_sec) or cooldown_sec < 0.0:
            raise ValueError("cooldown_sec must be finite and non-negative")
        self.threshold_m = float(threshold_m)
        self.consecutive_samples = int(consecutive_samples)
        self.cooldown_sec = float(cooldown_sec)
        self.count = 0
        self.last_trigger_sec = None

    def observe(self, innovation_m, stamp_sec):
        if not math.isfinite(innovation_m) or not math.isfinite(stamp_sec):
            self.count = 0
            return False
        if innovation_m < self.threshold_m:
            self.count = 0
            return False
        if (
            self.last_trigger_sec is not None
            and stamp_sec - self.last_trigger_sec < self.cooldown_sec
        ):
            self.count = 0
            return False
        self.count += 1
        if self.count < self.consecutive_samples:
            return False
        self.count = 0
        self.last_trigger_sec = stamp_sec
        return True


def finite_pose(position, orientation):
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return all(math.isfinite(value) for value in values)


class LocalizationRelocationGuard:
    def __init__(self):
        self.lock = threading.Lock()
        self.monitor_only = bool(rospy.get_param("~monitor_only", True))
        self.map_frame = str(rospy.get_param("~map_frame", "map")).lstrip("/")
        self.max_state_age_sec = float(rospy.get_param("~max_state_age_sec", 0.1))
        self.max_orientation_age_sec = float(
            rospy.get_param("~max_orientation_age_sec", 0.1)
        )
        self.detector = RelocationDetector(
            threshold_m=float(rospy.get_param("~innovation_threshold_m", 5.0)),
            consecutive_samples=int(rospy.get_param("~consecutive_samples", 3)),
            cooldown_sec=float(rospy.get_param("~cooldown_sec", 5.0)),
        )
        position_stddev = float(rospy.get_param("~reset_position_stddev_m", 0.1))
        yaw_stddev = math.radians(
            float(rospy.get_param("~reset_yaw_stddev_deg", 1.0))
        )
        if position_stddev <= 0.0 or yaw_stddev <= 0.0:
            raise rospy.ROSInitException("reset covariance standard deviations must be positive")
        self.covariance = [0.0] * 36
        self.covariance[0] = position_stddev**2
        self.covariance[7] = position_stddev**2
        self.covariance[14] = 1.0e6
        self.covariance[21] = 1.0e6
        self.covariance[28] = 1.0e6
        self.covariance[35] = yaw_stddev**2

        self.latest_state = None
        self.latest_orientation = None
        self.reset_publisher = rospy.Publisher(
            rospy.get_param("~reset_topic", "/localization/initialpose"),
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=False,
        )
        rospy.Subscriber(
            rospy.get_param("~state_topic", "/localization/kinematic_state"),
            Odometry,
            self.state_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            rospy.get_param("~orientation_topic", "/localization/gt/orientation"),
            Imu,
            self.orientation_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            rospy.get_param("~gnss_topic", "/localization/gnss/odom"),
            Odometry,
            self.gnss_callback,
            queue_size=100,
        )
        rospy.loginfo(
            "localization_relocation_guard monitor_only=%s threshold=%.2fm consecutive=%d",
            self.monitor_only,
            self.detector.threshold_m,
            self.detector.consecutive_samples,
        )

    def state_callback(self, message):
        with self.lock:
            self.latest_state = message

    def orientation_callback(self, message):
        with self.lock:
            self.latest_orientation = message

    @staticmethod
    def stamp_sec(message):
        return message.header.stamp.to_sec()

    def gnss_callback(self, message):
        stamp = self.stamp_sec(message)
        if stamp <= 0.0:
            return
        with self.lock:
            state = self.latest_state
            orientation = self.latest_orientation
        if state is None:
            return
        state_age = abs(stamp - self.stamp_sec(state))
        if state_age > self.max_state_age_sec:
            rospy.logwarn_throttle(
                2.0, "Relocation guard waiting for co-timed state (age %.3fs)", state_age
            )
            self.detector.count = 0
            return
        dx = message.pose.pose.position.x - state.pose.pose.position.x
        dy = message.pose.pose.position.y - state.pose.pose.position.y
        innovation = math.hypot(dx, dy)
        if not self.monitor_only:
            if orientation is None:
                self.detector.count = 0
                rospy.logerr_throttle(
                    2.0, "Relocation reset unavailable: corrected orientation is missing"
                )
                return
            orientation_age = abs(stamp - self.stamp_sec(orientation))
            if orientation_age > self.max_orientation_age_sec:
                self.detector.count = 0
                rospy.logerr_throttle(
                    2.0,
                    "Relocation reset unavailable: corrected orientation age %.3fs exceeds %.3fs",
                    orientation_age,
                    self.max_orientation_age_sec,
                )
                return
        if not self.detector.observe(innovation, stamp):
            return

        if self.monitor_only:
            rospy.logwarn(
                "MONITOR ONLY: persistent localization relocation candidate: "
                "innovation=%.3fm GNSS=(%.3f, %.3f) fused=(%.3f, %.3f)",
                innovation,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                state.pose.pose.position.x,
                state.pose.pose.position.y,
            )
            return
        if not finite_pose(message.pose.pose.position, orientation.orientation):
            rospy.logerr("Relocation reset suppressed: non-finite pose")
            return

        reset = PoseWithCovarianceStamped()
        reset.header.stamp = message.header.stamp
        reset.header.frame_id = self.map_frame
        reset.pose.pose.position = message.pose.pose.position
        reset.pose.pose.orientation = orientation.orientation
        reset.pose.covariance = self.covariance
        self.reset_publisher.publish(reset)
        rospy.logwarn(
            "Published guarded Global EKF relocation reset: innovation=%.3fm position=(%.3f, %.3f)",
            innovation,
            reset.pose.pose.position.x,
            reset.pose.pose.position.y,
        )


def main():
    rospy.init_node("localization_relocation_guard")
    LocalizationRelocationGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
