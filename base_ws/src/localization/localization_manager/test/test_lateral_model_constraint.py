#!/usr/bin/env python3

import importlib.util
import io
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from morai_msgs.msg import EgoVehicleStatus


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "morai_gt_motion_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("gt_motion_adapter", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

NOISE_NS = MODULE.DEFAULT_NOISE_NS
MODEL_NS = NOISE_NS + "/lateral_model"
MODEL_TOPIC = "/localization/constraints/lateral_velocity"


def serialized(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    return buffer.getvalue()


class LateralModelConstraintTest(unittest.TestCase):
    def make_adapter(self, parameters=None):
        parameters = dict(parameters or {})
        publishers = {}

        def publisher(topic, _message_type, **_kwargs):
            publishers[topic] = Mock()
            return publishers[topic]

        def get_param(name, default=None):
            return parameters.get(name, default)

        patches = (
            patch.object(MODULE.rospy, "get_param", side_effect=get_param),
            patch.object(MODULE.rospy, "Publisher", side_effect=publisher),
            patch.object(MODULE.rospy, "Subscriber"),
            patch.object(MODULE.rospy, "loginfo"),
            patch.object(MODULE.rospy, "logwarn_once"),
            patch.object(MODULE.rospy, "logwarn_throttle"),
            patch.object(MODULE.tf2_ros, "Buffer"),
            patch.object(MODULE.tf2_ros, "TransformListener"),
        )
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        adapter = MODULE.MoraiGtMotionAdapter()
        adapter.map_from_source_quaternion = Mock(
            return_value=(0.0, 0.0, 0.0, 1.0)
        )
        return adapter, publishers

    @staticmethod
    def motion(stamp_sec=10.0):
        message = EgoVehicleStatus()
        message.header.seq = 17
        message.header.stamp = MODULE.rospy.Time.from_sec(stamp_sec)
        message.header.frame_id = "morai_map"
        message.heading = 65.0
        message.velocity.x = 12.5
        # Withheld/native vy must not influence the pseudo measurement.
        message.velocity.y = 19.0
        return message

    def test_disabled_has_no_constraint_publisher_and_original_covariance(self):
        adapter, publishers = self.make_adapter(
            {MODEL_NS + "/stddev_mps": float("nan")}
        )
        self.assertFalse(adapter.lateral_model_enabled)
        self.assertIsNone(adapter.lateral_velocity_publisher)
        self.assertEqual(
            set(publishers),
            {"/localization/gt/orientation", "/localization/gt/twist"},
        )
        adapter.motion_callback(self.motion())
        twist = publishers["/localization/gt/twist"].publish.call_args[0][0]
        self.assertEqual(twist.twist.twist.linear.x, 12.5)
        self.assertEqual(twist.twist.twist.linear.y, 0.0)
        self.assertEqual(twist.twist.covariance[7], 1.0e6)

    def test_enabled_primary_outputs_are_byte_identical(self):
        disabled, off_publishers = self.make_adapter()
        enabled, on_publishers = self.make_adapter(
            {MODEL_NS + "/enabled": True}
        )
        message = self.motion()
        disabled.motion_callback(message)
        enabled.motion_callback(message)
        for topic in ("/localization/gt/orientation", "/localization/gt/twist"):
            self.assertEqual(
                serialized(off_publishers[topic].publish.call_args[0][0]),
                serialized(on_publishers[topic].publish.call_args[0][0]),
            )

    def test_enabled_publishes_separate_y_prior_with_source_header(self):
        adapter, publishers = self.make_adapter(
            {MODEL_NS + "/enabled": True, MODEL_NS + "/stddev_mps": 0.15}
        )
        message = self.motion()
        adapter.motion_callback(message)
        prior = publishers[MODEL_TOPIC].publish.call_args[0][0]
        self.assertEqual(prior.header.seq, message.header.seq)
        self.assertEqual(prior.header.stamp, message.header.stamp)
        self.assertEqual(prior.header.frame_id, "base_link")
        self.assertIsNot(prior.header, message.header)
        self.assertEqual(prior.twist.twist.linear.y, 0.0)
        self.assertAlmostEqual(prior.twist.covariance[7], 0.15**2)
        for index in (0, 14, 21, 28, 35):
            self.assertEqual(prior.twist.covariance[index], 1.0e6)
        for index in set(range(36)) - {0, 7, 14, 21, 28, 35}:
            self.assertEqual(prior.twist.covariance[index], 0.0)
        self.assertEqual(message.header.frame_id, "morai_map")

    def test_rejected_motion_does_not_publish_constraint(self):
        adapter, publishers = self.make_adapter({MODEL_NS + "/enabled": True})
        for stamp in (0.0,):
            adapter.motion_callback(self.motion(stamp))
        invalid = self.motion()
        invalid.heading = float("nan")
        adapter.motion_callback(invalid)
        publishers[MODEL_TOPIC].publish.assert_not_called()
        adapter.motion_callback(self.motion(10.0))
        adapter.motion_callback(self.motion(10.0))
        adapter.motion_callback(self.motion(9.0))
        self.assertEqual(publishers[MODEL_TOPIC].publish.call_count, 1)
        self.assertEqual(adapter.last_stamp, MODULE.rospy.Time.from_sec(10.0))

    def test_missing_transform_blocks_constraint(self):
        adapter, publishers = self.make_adapter({MODEL_NS + "/enabled": True})
        adapter.map_from_source_quaternion.return_value = None
        adapter.motion_callback(self.motion())
        publishers[MODEL_TOPIC].publish.assert_not_called()
        self.assertIsNone(adapter.last_stamp)

    def test_enabled_requires_actual_bool(self):
        for value in ("false", "true", 0, 1):
            with self.subTest(value=value):
                with self.assertRaises(MODULE.rospy.ROSInitException):
                    self.make_adapter({MODEL_NS + "/enabled": value})

    def test_enabled_requires_positive_finite_standard_deviation(self):
        for value in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(MODULE.rospy.ROSInitException):
                    self.make_adapter(
                        {MODEL_NS + "/enabled": True, MODEL_NS + "/stddev_mps": value}
                    )

    def test_custom_constraint_topic_and_default_sigma(self):
        topic = "/test/lateral_prior"
        adapter, publishers = self.make_adapter(
            {MODEL_NS + "/enabled": True, MODEL_NS + "/topic": topic}
        )
        adapter.motion_callback(self.motion())
        prior = publishers[topic].publish.call_args[0][0]
        self.assertTrue(math.isclose(prior.twist.covariance[7], 0.20**2))

    def test_launch_pins_prior_after_loading_noise_profile(self):
        launch = ET.parse(SCRIPT.parents[1] / "launch" / "gnss_imu_gt_ctra.launch")
        elements = list(launch.getroot())
        resets = [
            (index, element)
            for index, element in enumerate(elements)
            if element.tag == "param"
            and element.get("name")
            == "$(arg noise_namespace)/lateral_model/enabled"
        ]
        self.assertEqual(len(resets), 1)
        reset_index, reset = resets[0]
        self.assertEqual(reset.get("type"), "bool")
        self.assertEqual(reset.get("value"), "$(arg use_lateral_velocity_constraint)")
        noise_load_index = next(
            index
            for index, element in enumerate(elements)
            if element.tag == "rosparam"
            and element.get("file") == "$(arg measurement_noise_config)"
        )
        adapter_index = next(
            index
            for index, element in enumerate(elements)
            if element.tag == "node"
            and element.get("name") == "morai_gt_motion_adapter"
        )
        self.assertLess(noise_load_index, reset_index)
        self.assertLess(reset_index, adapter_index)


if __name__ == "__main__":
    unittest.main()
