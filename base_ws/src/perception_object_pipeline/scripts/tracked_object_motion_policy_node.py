#!/usr/bin/env python3
"""Apply class-conditioned motion constraints after object tracking."""

from __future__ import annotations

import rospy
from autoware_perception_msgs.msg import TrackedObjects

from perception_object_pipeline.object_motion_policy import apply_object_motion_policy


class TrackedObjectMotionPolicyNode:
    def __init__(self):
        self.force_unknown_stationary = bool(
            rospy.get_param("~force_unknown_stationary", True)
        )
        self.publisher = rospy.Publisher(
            "~output/objects", TrackedObjects, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            "~input/objects", TrackedObjects, self.callback, queue_size=10
        )
        rospy.loginfo(
            "[tracked_object_motion_policy] force_unknown_stationary=%s",
            self.force_unknown_stationary,
        )

    def callback(self, message):
        output, constrained_count = apply_object_motion_policy(
            message,
            force_unknown_stationary=self.force_unknown_stationary,
        )
        self.publisher.publish(output)
        rospy.logdebug_throttle(
            1.0,
            "[tracked_object_motion_policy] constrained UNKNOWN objects=%d",
            constrained_count,
        )


def main():
    rospy.init_node("tracked_object_motion_policy")
    TrackedObjectMotionPolicyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
