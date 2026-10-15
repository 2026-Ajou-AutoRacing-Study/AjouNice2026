#!/usr/bin/env python3
"""Apply class-conditioned motion constraints after object tracking."""

from __future__ import annotations

import json
import time
import rospy
from autoware_perception_msgs.msg import TrackedObjects
from std_msgs.msg import String

from perception_object_pipeline.object_motion_policy import apply_object_motion_policy
from perception_object_pipeline.node_processing_timing import NodeProcessingTiming


class TrackedObjectMotionPolicyNode:
    def __init__(self):
        self.force_unknown_stationary = bool(
            rospy.get_param("~force_unknown_stationary", True)
        )
        self.publisher = rospy.Publisher(
            "~output/objects", TrackedObjects, queue_size=10
        )
        self.processing_diagnostics_publisher = None
        if bool(rospy.get_param("~publish_processing_diagnostics", False)):
            self.processing_diagnostics_publisher = rospy.Publisher(
                "~diagnostics/processing_time_ms", String, queue_size=1
            )
            self.processing_timing = NodeProcessingTiming()
        self.subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self.callback, queue_size=10
        )
        rospy.loginfo(
            "[tracked_object_motion_policy] force_unknown_stationary=%s",
            self.force_unknown_stationary,
        )

    def callback(self, message):
        diagnostics = self.processing_diagnostics_publisher
        token = self.processing_timing.start(message, rospy.Time.now().to_nsec()) if diagnostics else None
        make_start = time.perf_counter() if diagnostics else None
        output, constrained_count = apply_object_motion_policy(
            message,
            force_unknown_stationary=self.force_unknown_stationary,
        )
        make_end = time.perf_counter() if diagnostics else None
        self.publisher.publish(output)
        publish_end = time.perf_counter() if diagnostics else None
        rospy.logdebug_throttle(
            1.0,
            "[tracked_object_motion_policy] constrained UNKNOWN objects=%d",
            constrained_count,
        )
        if diagnostics:
            sample = self.processing_timing.finish(token, make_start, make_end, publish_end,
                                                  unknown_constrained=constrained_count)
            diagnostics.publish(String(data=json.dumps(sample, allow_nan=False, separators=(",", ":"))))


def main():
    rospy.init_node("tracked_object_motion_policy")
    TrackedObjectMotionPolicyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
