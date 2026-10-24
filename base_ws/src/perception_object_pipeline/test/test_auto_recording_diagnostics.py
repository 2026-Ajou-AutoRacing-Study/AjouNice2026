#!/usr/bin/env python3
"""Recorder subscription enables diagnostics without changing prediction payloads."""
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import rospy
from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import map_based_prediction_node as map_node
import tracked_object_motion_policy_node as motion_node
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord


def wire(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return stream.getvalue()


class Publisher:
    def __init__(self, topic, *_args, **_kwargs):
        self.topic = topic
        self.messages = []
        self.connections = 0
        self.connection_checks = 0
        self.connection_error = False

    def get_num_connections(self):
        self.connection_checks += 1
        if self.connection_error:
            raise RuntimeError("connection state unavailable")
        return self.connections

    def publish(self, message):
        self.messages.append(message)


class AutoRecordingDiagnosticsTest(unittest.TestCase):
    MODULES = ((map_node, map_node.MapBasedPredictionNode),
               (motion_node, motion_node.TrackedObjectMotionPolicyNode))

    def message(self):
        message = TrackedObjects()
        message.header.stamp = rospy.Time.from_sec(100.)
        message.header.frame_id = "map"
        for index, label in enumerate((ObjectClassification.CAR,
                                      ObjectClassification.PEDESTRIAN,
                                      ObjectClassification.UNKNOWN)):
            obj = TrackedObject()
            obj.object_id.uuid = [index] * 16
            obj.classification = [ObjectClassification(label=label, probability=1.)]
            obj.kinematics.pose_with_covariance.pose.orientation.w = 1.
            obj.kinematics.pose_with_covariance.pose.position.x = 3. + index
            obj.kinematics.pose_with_covariance.pose.position.y = .2
            obj.kinematics.twist_with_covariance.twist.linear.x = 3.
            obj.shape.dimensions.x = 4.
            obj.shape.dimensions.y = 2.
            obj.shape.dimensions.z = 1.5
            message.objects.append(obj)
        return message

    @contextlib.contextmanager
    def node(self, module, node_type, force=False, auto=True, use_defaults=False):
        overrides = {} if use_defaults else {
            "~publish_processing_diagnostics": force,
            "~diagnostics_auto_on_subscribe": auto,
        }
        publishers = []

        def make_publisher(*args, **kwargs):
            publisher = Publisher(*args, **kwargs)
            publishers.append(publisher)
            return publisher

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(module.rospy, "get_param",
                side_effect=lambda name, default=None: overrides.get(name, default)))
            stack.enter_context(patch.object(module.rospy, "Publisher", side_effect=make_publisher))
            stack.enter_context(patch.object(module.rospy, "Subscriber"))
            for name in ("loginfo", "loginfo_throttle", "logwarn_throttle", "logdebug_throttle"):
                stack.enter_context(patch.object(module.rospy, name))
            stack.enter_context(patch.object(module.rospy.Time, "now",
                                            return_value=rospy.Time.from_sec(100.1)))
            if module is map_node:
                network = LaneletNetwork([LaneletRecord(1, ((0., 0., 0.), (100., 0., 0.)))])
                stack.enter_context(patch.object(map_node.LaneletNetwork, "load", return_value=network))
            node = node_type()
            yield node, publishers

    def test_default_auto_advertises_without_measuring_until_subscribed(self):
        for module, node_type in self.MODULES:
            with self.subTest(node=node_type.__name__), self.node(
                module, node_type, use_defaults=True
            ) as (node, publishers):
                self.assertFalse(node.publish_processing_diagnostics)
                self.assertTrue(node.diagnostics_auto_on_subscribe)
                self.assertEqual(len(publishers), 2)
                node.callback(self.message())
                self.assertEqual(len(node.publisher.messages), 1)
                self.assertEqual(node.processing_diagnostics_publisher.messages, [])

    def test_connect_two_subscribers_disconnect_and_reconnect(self):
        for module, node_type in self.MODULES:
            with self.subTest(node=node_type.__name__), self.node(module, node_type) as (node, _):
                publisher = node.processing_diagnostics_publisher
                message = self.message()
                original = wire(message)
                for count in (0, 1, 2, 1, 0, 1):
                    publisher.connections = count
                    node.callback(message)
                self.assertEqual(len(publisher.messages), 4)
                records = [json.loads(item.data) for item in publisher.messages]
                self.assertEqual([item["callback_index"] for item in records], [0, 1, 2, 0])
                self.assertIsNone(records[-1]["callback_start_gap_ms"])
                self.assertIsNone(records[-1]["input_header_step_ms"])
                self.assertEqual(wire(message), original)
                self.assertEqual(len({wire(item) for item in node.publisher.messages}), 1)

    def test_explicit_force_keeps_legacy_on_without_subscriber(self):
        for module, node_type in self.MODULES:
            for auto in (False, True):
                with self.subTest(node=node_type.__name__, auto=auto), self.node(
                    module, node_type, force=True, auto=auto
                ) as (node, _):
                    node.callback(self.message())
                    self.assertEqual(len(node.processing_diagnostics_publisher.messages), 1)
                    self.assertEqual(node.processing_diagnostics_publisher.connection_checks, 0)

    def test_explicit_opt_out_never_advertises_diagnostics(self):
        for module, node_type in self.MODULES:
            with self.subTest(node=node_type.__name__), self.node(
                module, node_type, force=False, auto=False
            ) as (node, publishers):
                self.assertIsNone(node.processing_diagnostics_publisher)
                self.assertEqual(len(publishers), 1)
                with patch.object(module.rospy.Time, "now") as now:
                    node.callback(self.message())
                    now.assert_not_called()
                self.assertEqual(len(node.publisher.messages), 1)

    def test_connection_introspection_error_does_not_prevent_main_output(self):
        for module, node_type in self.MODULES:
            with self.subTest(node=node_type.__name__), self.node(module, node_type) as (node, _):
                node.processing_diagnostics_publisher.connection_error = True
                node.callback(self.message())
                self.assertEqual(len(node.publisher.messages), 1)
                self.assertEqual(node.processing_diagnostics_publisher.messages, [])

    def test_force_auto_and_opt_out_have_exact_main_payload(self):
        for module, node_type in self.MODULES:
            with self.subTest(node=node_type.__name__):
                outputs = []
                for force, auto, connections in ((False, False, 0), (False, True, 0),
                                                  (False, True, 1), (False, True, 2),
                                                  (True, False, 0), (True, True, 0)):
                    with self.node(module, node_type, force=force, auto=auto) as (node, _):
                        if node.processing_diagnostics_publisher is not None:
                            node.processing_diagnostics_publisher.connections = connections
                        message = self.message()
                        original = wire(message)
                        node.callback(message)
                        self.assertEqual(wire(message), original)
                        outputs.append(wire(node.publisher.messages[0]))
                self.assertEqual(len(set(outputs)), 1)

    def test_prediction_launch_has_explicit_auto_opt_out(self):
        root = ET.parse(Path(__file__).resolve().parents[1] /
                        "launch/object_prediction.launch").getroot()
        defaults = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
        self.assertEqual(defaults["publish_processing_diagnostics"], "false")
        self.assertEqual(defaults["diagnostics_auto_on_subscribe"], "true")
        node = next(node for node in root.iter("node")
                    if node.attrib["name"] == "map_based_prediction")
        params = {param.attrib["name"]: param.attrib.get("value") for param in node.findall("param")}
        self.assertEqual(params["diagnostics_auto_on_subscribe"],
                         "$(arg diagnostics_auto_on_subscribe)")


if __name__ == "__main__":
    unittest.main()
