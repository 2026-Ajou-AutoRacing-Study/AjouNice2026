#!/usr/bin/env python3

import importlib.util
import io
from pathlib import Path
import unittest

import rospy
from morai_msgs.msg import EgoVehicleStatus


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "morai_gt_kinematic_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("morai_gt_publisher", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MoraiGtContractTest(unittest.TestCase):
    def test_acceleration_message_preserves_measurement_and_header(self):
        ego = EgoVehicleStatus()
        ego.header.seq = 17
        ego.header.stamp = rospy.Time(123, 456)
        ego.acceleration.x = 1.25
        ego.acceleration.y = -0.5
        ego.acceleration.z = 0.125

        output = MODULE.acceleration_message(ego, "base_link")

        self.assertEqual(output.header.seq, ego.header.seq)
        self.assertEqual(output.header.stamp, ego.header.stamp)
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertAlmostEqual(output.accel.accel.linear.x, 1.25)
        self.assertAlmostEqual(output.accel.accel.linear.y, -0.5)
        self.assertAlmostEqual(output.accel.accel.linear.z, 0.125)
        self.assertEqual(output.accel.covariance, [0.0] * 36)

        serialized = io.BytesIO()
        output.serialize(serialized)
        self.assertGreater(len(serialized.getvalue()), 0)


if __name__ == "__main__":
    unittest.main()
