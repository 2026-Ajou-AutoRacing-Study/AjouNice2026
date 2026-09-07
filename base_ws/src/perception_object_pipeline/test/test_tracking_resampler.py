#!/usr/bin/env python3

import math
import unittest

import rospy
from autoware_perception_msgs.msg import (
    ObjectClassification,
    TrackedObject,
    TrackedObjects,
)

from perception_object_pipeline.kinematic_prediction import (
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from perception_object_pipeline.tracking_resampler import (
    extrapolate_tracked_objects,
)


class TrackingResamplerTest(unittest.TestCase):
    def _message(self):
        message = TrackedObjects()
        message.header.stamp = rospy.Time.from_sec(10.0)
        tracked = TrackedObject()
        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        tracked.classification.append(classification)
        pose = tracked.kinematics.pose_with_covariance.pose
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.orientation = yaw_to_quaternion(0.0)
        twist = tracked.kinematics.twist_with_covariance.twist
        twist.linear.x = 4.0
        twist.angular.z = 0.0
        message.objects.append(tracked)
        return message

    def test_constant_velocity_extrapolation(self):
        source = self._message()
        output = extrapolate_tracked_objects(
            source, rospy.Time.from_sec(10.25)
        )
        self.assertAlmostEqual(output.objects[0].kinematics.pose_with_covariance.pose.position.x, 2.0)
        self.assertAlmostEqual(output.objects[0].kinematics.pose_with_covariance.pose.position.y, 2.0)
        self.assertEqual(output.header.stamp, rospy.Time.from_sec(10.25))
        self.assertEqual(source.header.stamp, rospy.Time.from_sec(10.0))
        self.assertAlmostEqual(source.objects[0].kinematics.pose_with_covariance.pose.position.x, 1.0)

    def test_ctrv_extrapolation(self):
        source = self._message()
        source.objects[0].kinematics.twist_with_covariance.twist.angular.z = 1.0
        output = extrapolate_tracked_objects(
            source, rospy.Time.from_sec(11.0)
        )
        pose = output.objects[0].kinematics.pose_with_covariance.pose
        self.assertAlmostEqual(pose.position.x, 1.0 + 4.0 * math.sin(1.0), places=6)
        self.assertAlmostEqual(pose.position.y, 2.0 + 4.0 * (1.0 - math.cos(1.0)), places=6)
        self.assertAlmostEqual(quaternion_to_yaw(pose.orientation), 1.0, places=6)

    def test_rejects_retrograde_target(self):
        with self.assertRaises(ValueError):
            extrapolate_tracked_objects(
                self._message(), rospy.Time.from_sec(9.9)
            )


if __name__ == "__main__":
    unittest.main()
