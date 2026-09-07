#!/usr/bin/env python3
"""Publish measurement-driven EKF tracks on a fixed causal ROS-time grid."""

import threading

import rospy
from autoware_perception_msgs.msg import TrackedObjects

from perception_object_pipeline.tracking_resampler import (
    extrapolate_tracked_objects,
)


class FixedRateTrackedObjectsNode:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.maximum_age_sec = float(rospy.get_param("~maximum_age_sec", 0.25))
        if self.rate_hz <= 0.0:
            raise ValueError("~rate_hz must be positive")
        if self.maximum_age_sec <= 0.0:
            raise ValueError("~maximum_age_sec must be positive")

        self._lock = threading.Lock()
        self._latest = None
        self._timer = None
        self._published = 0
        self._skipped_stale = 0
        self._skipped_before_measurement = 0

        self._publisher = rospy.Publisher(
            "~output/objects", TrackedObjects, queue_size=10
        )
        self._subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self._callback, queue_size=10
        )
        rospy.loginfo(
            "[tracked_objects_fixed_rate] rate=%.3fHz maximum_age=%.3fs",
            self.rate_hz,
            self.maximum_age_sec,
        )

    def _callback(self, message):
        with self._lock:
            self._latest = message
            if self._timer is None:
                # Create the timer only after /clock and the first measurement
                # exist. Starting at ROS time zero before a historical bag's
                # large initial clock jump can starve rospy.Timer callbacks.
                self._timer = rospy.Timer(
                    rospy.Duration.from_sec(1.0 / self.rate_hz),
                    self._timer_callback,
                )

    def _timer_callback(self, event):
        with self._lock:
            message = self._latest
        if message is None:
            return

        # current_expected is the exact ROS-time grid. It is deterministic in
        # rosbag replay and avoids converting loop scheduling jitter into state
        # motion. Never publish a state before its source measurement.
        target_stamp = event.current_expected
        age_sec = target_stamp.to_sec() - message.header.stamp.to_sec()
        if age_sec < -1.0e-9:
            self._skipped_before_measurement += 1
            return
        if age_sec > self.maximum_age_sec:
            self._skipped_stale += 1
            rospy.logwarn_throttle(
                2.0,
                "[tracked_objects_fixed_rate] latest track is stale: %.3fs",
                age_sec,
            )
            return

        self._publisher.publish(
            extrapolate_tracked_objects(message, target_stamp)
        )
        self._published += 1
        rospy.logdebug_throttle(
            1.0,
            "[tracked_objects_fixed_rate] published=%d stale_skips=%d "
            "pre_measurement_skips=%d age=%.3fs",
            self._published,
            self._skipped_stale,
            self._skipped_before_measurement,
            age_sec,
        )


def main():
    rospy.init_node("tracked_objects_fixed_rate")
    FixedRateTrackedObjectsNode()
    rospy.spin()


if __name__ == "__main__":
    main()
