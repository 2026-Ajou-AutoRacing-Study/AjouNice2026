#!/usr/bin/env python3
"""Fixed-schema copies retain wire values and isolate every mutable field."""
import copy
import io
import math
from pathlib import Path
import sys
import unittest

from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects
from geometry_msgs.msg import Point32

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from map_based_prediction_node import make_map_based_predicted_objects
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord
from perception_object_pipeline.message_copy import copy_tracked_object
from perception_object_pipeline.object_motion_policy import apply_object_motion_policy


def wire(message):
    output = io.BytesIO()
    message.serialize(output)
    return output.getvalue()


def mutable_descendants(value):
    """IDs of ROS submessages/lists, not primitive values or immutable tuples."""
    result = set()
    if hasattr(value, "__slots__"):
        result.add(id(value))
        for field in value.__slots__:
            result.update(mutable_descendants(getattr(value, field)))
    elif isinstance(value, (list, tuple)):
        if isinstance(value, list):
            result.add(id(value))
        for item in value:
            result.update(mutable_descendants(item))
    return result


class MessageCopyTest(unittest.TestCase):
    def track(self, label=ObjectClassification.CAR):
        tracked = TrackedObject()
        tracked.object_id.uuid = list(range(16))
        tracked.existence_probability = 0.123456789
        tracked.classification = [ObjectClassification(label=label, probability=0.91),
                                  ObjectClassification(label=ObjectClassification.UNKNOWN, probability=0.09)]
        tracked.shape.type = 2
        tracked.shape.dimensions.x = 4.2
        tracked.shape.footprint.points = [Point32(x=1.,y=2.,z=3.), Point32(x=-1.,y=-2.,z=-3.)]
        kin = tracked.kinematics
        kin.pose_with_covariance.pose.position.x = 1.
        kin.pose_with_covariance.pose.orientation.w = 1.
        kin.twist_with_covariance.twist.linear.x = 2.
        kin.acceleration_with_covariance.accel.angular.z = -0.3
        for wrapper in (kin.pose_with_covariance, kin.twist_with_covariance,
                        kin.acceleration_with_covariance):
            wrapper.covariance = [float(i)/100 for i in range(36)]
            wrapper.covariance[0] = -0.
        return tracked

    def test_full_track_matches_deepcopy_without_mutable_aliases(self):
        for tuple_covariance in (False, True):
            for empty in (False, True):
                original = self.track()
                if tuple_covariance:
                    original.kinematics.pose_with_covariance.covariance = tuple(original.kinematics.pose_with_covariance.covariance)
                if empty:
                    original.classification = []
                    original.shape.footprint.points = []
                cloned = copy_tracked_object(original)
                self.assertEqual(wire(cloned), wire(copy.deepcopy(original)))
                self.assertFalse(mutable_descendants(cloned) & mutable_descendants(original))

    def test_unknown_policy_is_independent_and_does_not_touch_input(self):
        message = TrackedObjects(objects=[self.track(ObjectClassification.UNKNOWN)])
        before = wire(message)
        output, count = apply_object_motion_policy(message)
        self.assertEqual(count, 1)
        self.assertFalse(mutable_descendants(output.objects[0]) & mutable_descendants(message.objects[0]))
        output.objects[0].object_id.uuid[0] = 100
        output.objects[0].kinematics.pose_with_covariance.covariance[0] = 999.
        output.objects[0].shape.footprint.points[0].x = 999.
        self.assertEqual(wire(message), before)

    def test_prediction_output_is_independent_of_input_and_other_predictions(self):
        message = TrackedObjects(objects=[self.track()])
        message.header.frame_id = "map"
        message.header.stamp.secs = 123
        before = wire(message)
        network = LaneletNetwork([LaneletRecord(1, ((0.,0.,0.),(20.,0.,0.)))])
        output = make_map_based_predicted_objects(message, [.1,.2,.3], network)[0]
        second = make_map_based_predicted_objects(message, [.1,.2,.3], network)[0]
        second_wire = wire(second)
        self.assertFalse(mutable_descendants(output) & mutable_descendants(message))
        self.assertFalse(mutable_descendants(output) & mutable_descendants(second))
        output.header.stamp.secs = 999
        obj = output.objects[0]
        obj.object_id.uuid[0] = 255
        obj.classification[0].label = 0
        obj.shape.dimensions.x = 999.
        obj.shape.footprint.points[0].x = 999.
        obj.kinematics.initial_pose_with_covariance.covariance[0] = 999.
        obj.kinematics.initial_twist_with_covariance.twist.linear.x = 999.
        obj.kinematics.initial_acceleration_with_covariance.accel.linear.x = 999.
        obj.kinematics.predicted_paths[0].path[0].position.x = 999.
        obj.kinematics.predicted_paths[0].path[1].orientation.w = 999.
        self.assertEqual(wire(message), before)
        self.assertEqual(wire(second), second_wire)


if __name__ == "__main__":
    unittest.main()
