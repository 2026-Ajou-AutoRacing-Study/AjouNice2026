#!/usr/bin/env python3
"""Pre-drive simulator velocity measurement checks; no ROS master required."""
import unittest
from autoware_perception_msgs.msg import DetectedObject, DetectedObjects, ObjectClassification
from run_obstaclecruise_batch import velocity_observation_count


class VelocityInputGateTest(unittest.TestCase):
    def message(self, speed=33.333):
        obj=DetectedObject()
        obj.classification=[ObjectClassification(label=ObjectClassification.CAR,probability=1.0)]
        k=obj.kinematics
        k.has_twist=k.has_twist_covariance=True
        k.twist_with_covariance.twist.linear.x=speed
        k.twist_with_covariance.covariance[0]=1.0
        return DetectedObjects(objects=[obj])

    def test_valid_and_stationary(self):
        self.assertEqual(velocity_observation_count(self.message()),1)
        self.assertEqual(velocity_observation_count(self.message(0)),1)

    def test_stale_script_position_only_rejected(self):
        m=self.message();m.objects[0].kinematics.has_twist=False
        self.assertEqual(velocity_observation_count(m),0)

    def test_nonfinite_or_unbounded_rejected(self):
        for speed in (float('nan'),float('inf'),51.0):
            self.assertEqual(velocity_observation_count(self.message(speed)),0)

    def test_invalid_covariance_rejected(self):
        for variance in (0.0,-1.0,float('nan')):
            m=self.message();m.objects[0].kinematics.twist_with_covariance.covariance[0]=variance
            self.assertEqual(velocity_observation_count(m),0)

    def test_unknown_or_empty_not_a_vehicle_measurement(self):
        m=self.message();m.objects[0].classification[0].label=ObjectClassification.UNKNOWN
        self.assertEqual(velocity_observation_count(m),0)
        self.assertEqual(velocity_observation_count(DetectedObjects()),0)


if __name__=='__main__':
    unittest.main()
