#!/usr/bin/env python3
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rospy
from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import map_based_prediction_node as map_node
import tracked_object_motion_policy_node as motion_node
from perception_object_pipeline import node_processing_timing as timing
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord


def wire(message):
    output = io.BytesIO()
    message.serialize(output)
    return output.getvalue()


class Publisher:
    def __init__(self):
        self.messages = []
    def publish(self, message):
        self.messages.append(message)


class NodeProcessingTimingTest(unittest.TestCase):
    def message(self):
        message = TrackedObjects()
        message.header.stamp = rospy.Time.from_sec(100.)
        message.header.frame_id = "map"
        obj = TrackedObject()
        obj.classification = [ObjectClassification(label=ObjectClassification.UNKNOWN, probability=1.)]
        obj.kinematics.pose_with_covariance.pose.orientation.w = 1.
        message.objects = [obj]
        return message

    def test_units_and_unverified_header_age_are_explicit(self):
        observer = timing.NodeProcessingTiming()
        message = self.message()
        with patch.object(timing.time,"perf_counter",return_value=1.), \
             patch.object(timing.time,"thread_time",return_value=.010):
            token = observer.start(message, 100_237_000_000)
        with patch.object(timing.time,"perf_counter",return_value=1.005), \
             patch.object(timing.time,"thread_time",return_value=.012):
            result = observer.finish(token,1.,1.001,1.004,mapped=0)
        self.assertEqual(result["input_header_age_ms"],237.)
        self.assertFalse(result["header_age_is_detector_observation_age"])
        self.assertIsNone(result["callback_start_gap_ms"])
        self.assertAlmostEqual(result["message_make_ms"],1.)
        self.assertAlmostEqual(result["synchronous_publish_ms"],3.)
        self.assertAlmostEqual(result["callback_wall_ms_excluding_diagnostics"],5.)
        self.assertAlmostEqual(result["callback_thread_cpu_ms_excluding_diagnostics"],2.)
        message.header.stamp = rospy.Time.from_sec(100.033)
        with patch.object(timing.time,"perf_counter",return_value=1.034):
            second = observer.start(message,100_267_000_000)[0]
        self.assertAlmostEqual(second["callback_start_gap_ms"],34.)
        self.assertAlmostEqual(second["input_header_step_ms"],33.)

    def test_zero_and_future_stamp_are_not_clamped_to_fake_freshness(self):
        observer = timing.NodeProcessingTiming()
        message = self.message()
        message.header.stamp = rospy.Time()
        self.assertIsNone(observer.start(message, 100_000_000_000)[0]["input_header_age_ms"])
        message.header.stamp = rospy.Time.from_sec(101.)
        self.assertEqual(observer.start(message,100_000_000_000)[0]["input_header_age_ms"],-1000.)

    def test_optional_map_and_motion_diagnostics_do_not_change_main_wire(self):
        for node_module, node_type in ((map_node,map_node.MapBasedPredictionNode),
                                      (motion_node,motion_node.TrackedObjectMotionPolicyNode)):
            outputs = []
            for enabled in (False,True):
                node = node_type.__new__(node_type)
                node.publisher = Publisher()
                node.processing_diagnostics_publisher = Publisher() if enabled else None
                node.processing_timing = timing.NodeProcessingTiming()
                if node_module is map_node:
                    node.horizons = [.1,.2,.3]
                    node.network = LaneletNetwork([LaneletRecord(1,((0.,0.,0.),(20.,0.,0.)))])
                else:
                    node.force_unknown_stationary = True
                message = self.message()
                before = wire(message)
                with patch.object(node_module.rospy.Time,"now",return_value=rospy.Time.from_sec(100.1)) as now, \
                     patch.object(node_module.rospy,"loginfo_throttle"), \
                     patch.object(node_module.rospy,"logdebug_throttle"):
                    node.callback(message)
                self.assertEqual(wire(message),before)
                outputs.append(wire(node.publisher.messages[0]))
                if enabled:
                    self.assertEqual(len(node.processing_diagnostics_publisher.messages),1)
                    record=json.loads(node.processing_diagnostics_publisher.messages[0].data)
                    self.assertEqual(record["input_stamp_ns"],100_000_000_000)
                    self.assertEqual(record["input_objects"],1)
                    self.assertGreaterEqual(record["synchronous_publish_ms"],0.)
                else:
                    now.assert_not_called()
            self.assertEqual(outputs[0],outputs[1])

    def test_map_bad_frame_keeps_existing_drop_and_no_diagnostics(self):
        node = map_node.MapBasedPredictionNode.__new__(map_node.MapBasedPredictionNode)
        node.publisher = Publisher()
        node.processing_diagnostics_publisher = Publisher()
        message = self.message()
        message.header.frame_id = "lidar"
        with patch.object(map_node.rospy,"logwarn_throttle"):
            node.callback(message)
        self.assertEqual(node.publisher.messages,[])
        self.assertEqual(node.processing_diagnostics_publisher.messages,[])


if __name__ == "__main__":
    unittest.main()
