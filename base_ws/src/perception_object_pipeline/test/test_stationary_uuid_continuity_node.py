#!/usr/bin/env python3
"""Node plumbing must use raw native evidence, and remain OFF by default."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from test_stationary_uuid_continuity import actor, identity, message
from perception_object_pipeline.stationary_uuid_continuity import StationaryUuidContinuity

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import tracked_object_motion_policy_node as motion


class StationaryUuidContinuityNodeTest(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
