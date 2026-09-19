#!/usr/bin/env python3

import unittest

from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects

from perception_object_pipeline.object_motion_policy import apply_object_motion_policy


class ObjectMotionPolicyTest(unittest.TestCase):
    def _object(self, label):
        tracked = TrackedObject()
        tracked.classification = [
            ObjectClassification(label=label, probability=0.9)
        ]
        tracked.kinematics.pose_with_covariance.pose.position.x = 12.5
        tracked.kinematics.twist_with_covariance.twist.linear.x = 3.0
        tracked.kinematics.twist_with_covariance.twist.linear.y = -1.0
        tracked.kinematics.twist_with_covariance.twist.angular.z = 0.4
        tracked.kinematics.twist_with_covariance.covariance[0] = 2.5
        tracked.kinematics.acceleration_with_covariance.accel.linear.x = 1.2
        tracked.kinematics.acceleration_with_covariance.accel.angular.z = 0.3
        tracked.kinematics.is_stationary = False
        return tracked

    def test_unknown_motion_is_zeroed_without_changing_pose_or_covariance(self):
        message = TrackedObjects()
        message.objects = [self._object(ObjectClassification.UNKNOWN)]

        output, count = apply_object_motion_policy(message)
        tracked = output.objects[0]

        self.assertEqual(count, 1)
        self.assertEqual(tracked.kinematics.pose_with_covariance.pose.position.x, 12.5)
        self.assertEqual(tracked.kinematics.twist_with_covariance.twist.linear.x, 0.0)
        self.assertEqual(tracked.kinematics.twist_with_covariance.twist.linear.y, 0.0)
        self.assertEqual(tracked.kinematics.twist_with_covariance.twist.angular.z, 0.0)
        self.assertEqual(tracked.kinematics.acceleration_with_covariance.accel.linear.x, 0.0)
        self.assertEqual(tracked.kinematics.acceleration_with_covariance.accel.angular.z, 0.0)
        self.assertTrue(tracked.kinematics.is_stationary)
        self.assertEqual(tracked.kinematics.twist_with_covariance.covariance[0], 2.5)
        self.assertEqual(message.objects[0].kinematics.twist_with_covariance.twist.linear.x, 3.0)

    def test_vehicle_and_pedestrian_motion_are_unchanged(self):
        message = TrackedObjects()
        message.objects = [
            self._object(ObjectClassification.CAR),
            self._object(ObjectClassification.PEDESTRIAN),
        ]

        output, count = apply_object_motion_policy(message)

        self.assertEqual(count, 0)
        for tracked in output.objects:
            self.assertEqual(tracked.kinematics.twist_with_covariance.twist.linear.x, 3.0)
            self.assertEqual(tracked.kinematics.twist_with_covariance.twist.linear.y, -1.0)
            self.assertEqual(tracked.kinematics.twist_with_covariance.twist.angular.z, 0.4)
            self.assertFalse(tracked.kinematics.is_stationary)

    def test_policy_can_be_disabled_for_ablation(self):
        message = TrackedObjects()
        message.objects = [self._object(ObjectClassification.UNKNOWN)]

        output, count = apply_object_motion_policy(
            message, force_unknown_stationary=False
        )

        self.assertEqual(count, 0)
        self.assertEqual(output.objects[0].kinematics.twist_with_covariance.twist.linear.x, 3.0)
        self.assertFalse(output.objects[0].kinematics.is_stationary)

    def test_missing_classification_is_treated_as_unknown(self):
        message = TrackedObjects()
        tracked = self._object(ObjectClassification.CAR)
        tracked.classification = []
        message.objects = [tracked]

        output, count = apply_object_motion_policy(message)

        self.assertEqual(count, 1)
        self.assertEqual(output.objects[0].kinematics.twist_with_covariance.twist.linear.x, 0.0)
        self.assertTrue(output.objects[0].kinematics.is_stationary)


if __name__ == "__main__":
    unittest.main()
