#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    PredictedObjects,
    Shape,
    TrackedObjects,
)


class StandardPipelineTest(unittest.TestCase):
    def setUp(self):
        self.tracked = []
        self.predicted = []
        self.publisher = rospy.Publisher(
            "/test/detections", DetectedObjects, queue_size=1
        )
        self.tracked_subscriber = rospy.Subscriber(
            "/test/tracks", TrackedObjects, self.tracked.append, queue_size=10
        )
        self.predicted_subscriber = rospy.Subscriber(
            "/test/predictions", PredictedObjects, self.predicted.append, queue_size=10
        )

    def _wait_for(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while not predicate() and time.time() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.02)
        return predicate()

    def test_detection_to_prediction_contract(self):
        self.assertTrue(
            self._wait_for(lambda: self.publisher.get_num_connections() > 0),
            "EKF did not subscribe to standardized detections",
        )

        base_stamp = rospy.Time.now()
        for index in range(6):
            message = DetectedObjects()
            message.header.stamp = base_stamp + rospy.Duration(0.1 * index)
            message.header.frame_id = "map"
            detected = DetectedObject()
            detected.existence_probability = 0.82
            detected.classification = [
                ObjectClassification(
                    label=ObjectClassification.CAR, probability=0.91
                )
            ]
            detected.kinematics.pose_with_covariance.pose.position.x = 10.0 + 0.2 * index
            detected.kinematics.pose_with_covariance.pose.orientation.w = 1.0
            detected.kinematics.orientation_availability = (
                DetectedObjectKinematics.AVAILABLE
            )
            detected.shape.type = Shape.BOUNDING_BOX
            detected.shape.dimensions.x = 4.5
            detected.shape.dimensions.y = 1.8
            detected.shape.dimensions.z = 1.6
            message.objects = [detected]
            self.publisher.publish(message)
            rospy.sleep(0.12)

        self.assertTrue(
            self._wait_for(lambda: any(message.objects for message in self.predicted)),
            "No planning-ready prediction was produced",
        )
        tracked_message = [message for message in self.tracked if message.objects][-1]
        predicted_message = [message for message in self.predicted if message.objects][-1]
        tracked = tracked_message.objects[0]
        predicted = predicted_message.objects[0]

        self.assertEqual(tracked_message.header.frame_id, "map")
        self.assertEqual(predicted_message.header.frame_id, "map")
        self.assertEqual(bytes(tracked.object_id.uuid[:4]), b"AJOU")
        self.assertEqual(predicted.object_id, tracked.object_id)
        self.assertEqual(tracked.classification[0].label, ObjectClassification.CAR)
        self.assertAlmostEqual(tracked.existence_probability, 0.82, delta=0.1)
        self.assertGreater(
            tracked.kinematics.pose_with_covariance.covariance[0], 0.0
        )
        self.assertAlmostEqual(
            tracked.kinematics.twist_with_covariance.twist.linear.x,
            2.0,
            delta=0.15,
        )

        path = predicted.kinematics.predicted_paths[0]
        self.assertEqual(len(path.path), 5)
        self.assertAlmostEqual(path.time_step.to_sec(), 0.3)
        self.assertGreater(path.path[-1].position.x, path.path[0].position.x)


if __name__ == "__main__":
    rospy.init_node("standard_object_pipeline_test")
    rostest.rosrun(
        "perception_object_pipeline",
        "standard_object_pipeline",
        StandardPipelineTest,
    )
