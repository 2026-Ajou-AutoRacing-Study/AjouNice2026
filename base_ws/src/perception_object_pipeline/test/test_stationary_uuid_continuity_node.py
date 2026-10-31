#!/usr/bin/env python3
"""Node plumbing must use raw native evidence, and remain OFF by default."""
from pathlib import Path
import json
import sys
import unittest
from unittest.mock import Mock, patch

from test_stationary_uuid_continuity import actor, identity, message
from perception_object_pipeline.stationary_uuid_continuity import StationaryUuidContinuity
from perception_object_pipeline.node_processing_timing import NodeProcessingTiming

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import tracked_object_motion_policy_node as motion


class StationaryUuidContinuityNodeTest(unittest.TestCase):
    def node(self, diagnostics=False):
        node = motion.TrackedObjectMotionPolicyNode.__new__(motion.TrackedObjectMotionPolicyNode)
        node.stationary_uuid_continuity = StationaryUuidContinuity(enabled=True)
        node.force_unknown_stationary = True
        node.publisher = Mock()
        node.processing_diagnostics_publisher = Mock() if diagnostics else None
        node.publish_processing_diagnostics = diagnostics
        node.processing_timing = NodeProcessingTiming()
        return node

    def test_private_param_defaults_off_and_requires_existing_stationary_policy(self):
        for force, requested, expected in ((True, None, False), (True, False, False),
                                            (True, True, True), (False, True, False)):
            with self.subTest(force=force, requested=requested):
                def parameter(name, default=None):
                    if name == '~force_unknown_stationary':
                        return force
                    if name == '~stationary_unknown_uuid_continuity' and requested is not None:
                        return requested
                    return default
                with patch.object(motion.rospy, 'get_param', side_effect=parameter), \
                        patch.object(motion.rospy, 'Publisher'), \
                        patch.object(motion.rospy, 'Subscriber'), \
                        patch.object(motion.rospy, 'loginfo'):
                    node = motion.TrackedObjectMotionPolicyNode()
                    self.assertEqual(node.stationary_uuid_continuity.enabled, expected)

    def test_zeroed_unknown_output_does_not_become_stationarity_evidence(self):
        node = motion.TrackedObjectMotionPolicyNode.__new__(motion.TrackedObjectMotionPolicyNode)
        node.stationary_uuid_continuity = StationaryUuidContinuity(enabled=True)
        node.force_unknown_stationary = True
        node.publisher = Mock()
        node.processing_diagnostics_publisher = None
        with patch.object(motion.rospy, 'logdebug_throttle'):
            for index in range(9):
                node.callback(message(1. + index * .1, actor(1, speed=2.)))
            node.callback(message(1.9))
            node.callback(message(2.2, actor(2)))
        output = node.publisher.publish.call_args[0][0]
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(2))
        self.assertEqual(node.stationary_uuid_continuity.last_events, [])
        self.assertEqual(output.objects[0].kinematics.twist_with_covariance.twist.linear.x, 0.)

    def test_existing_diagnostics_report_id_only_handoff_and_reset(self):
        node = self.node(diagnostics=True)
        with patch.object(motion.rospy, 'logdebug_throttle'), \
                patch.object(motion.rospy.Time, 'now', return_value=motion.rospy.Time.from_sec(10.)):
            for index in range(9):
                node.callback(message(1. + index * .1, actor(1)))
            node.callback(message(1.9))
            node.callback(message(2.2, actor(2, x=.1)))
            report = json.loads(node.processing_diagnostics_publisher.publish.call_args[0][0].data)
            self.assertEqual(report['uuid_continuity']['remapped_objects'], 1)
            event = report['uuid_continuity']['events'][0]
            self.assertEqual(event['kind'], 'reuse')
            self.assertEqual(event['raw_uuid'], identity(2).hex())
            self.assertEqual(event['logical_uuid'], identity(1).hex())
            node.callback(message(.5, actor(3)))
            report = json.loads(node.processing_diagnostics_publisher.publish.call_args[0][0].data)
            self.assertEqual(report['uuid_continuity']['remapped_objects'], 0)
            self.assertEqual(report['uuid_continuity']['events'][0]['reason'], 'backward_time')

    def test_diagnostics_details_are_bounded_without_altering_objects(self):
        node = self.node(diagnostics=True)
        source = message(1., *(actor(index, x=3. * index) for index in range(20)))
        with patch.object(motion.rospy, 'logdebug_throttle'), \
                patch.object(motion.rospy.Time, 'now', return_value=motion.rospy.Time.from_sec(10.)):
            node.callback(source)
        report = json.loads(node.processing_diagnostics_publisher.publish.call_args[0][0].data)
        diagnostic = report['uuid_continuity']
        self.assertTrue(diagnostic['enabled'])
        self.assertEqual(len(diagnostic['rejections']), 8)
        self.assertEqual(diagnostic['rejections_omitted'], 12)
        self.assertEqual(diagnostic['remapped_objects'], 0)
        self.assertEqual(len(node.publisher.publish.call_args[0][0].objects), 20)


if __name__ == '__main__':
    unittest.main()
