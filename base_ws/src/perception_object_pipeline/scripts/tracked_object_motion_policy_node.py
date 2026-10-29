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
from perception_object_pipeline.stationary_uuid_continuity import StationaryUuidContinuity


class TrackedObjectMotionPolicyNode:
    def __init__(self):
        self.force_unknown_stationary = bool(
            rospy.get_param("~force_unknown_stationary", True)
        )
        self.stationary_uuid_continuity = StationaryUuidContinuity(enabled=(
            self.force_unknown_stationary and bool(
                rospy.get_param("~stationary_unknown_uuid_continuity", False))))
        self.publisher = rospy.Publisher(
            "~output/objects", TrackedObjects, queue_size=10
        )
        self.publish_processing_diagnostics = bool(
            rospy.get_param("~publish_processing_diagnostics", False)
        )
        self.diagnostics_auto_on_subscribe = bool(
            rospy.get_param("~diagnostics_auto_on_subscribe", True)
        )
        self.processing_diagnostics_publisher = None
        self._processing_diagnostics_active = False
        if self.publish_processing_diagnostics or self.diagnostics_auto_on_subscribe:
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

    def _processing_diagnostics_for_callback(self):
        publisher = self.processing_diagnostics_publisher
        enabled = publisher is not None and getattr(
            self, "publish_processing_diagnostics", True
        )
        if publisher is not None and not enabled and getattr(
            self, "diagnostics_auto_on_subscribe", False
        ):
            try:
                # Local rospy connection state, not a parameter-server RPC.
                enabled = publisher.get_num_connections() > 0
            except Exception as error:
                rospy.logwarn_throttle(
                    5., "[tracked_object_motion_policy] optional diagnostics unavailable: %s", str(error)
                )
        if enabled and not getattr(self, "_processing_diagnostics_active", False):
            self.processing_timing = NodeProcessingTiming()
        self._processing_diagnostics_active = enabled
        return publisher if enabled else None

    def callback(self, message):
        diagnostics = self._processing_diagnostics_for_callback()
        token = self.processing_timing.start(message, rospy.Time.now().to_nsec()) if diagnostics else None
        make_start = time.perf_counter() if diagnostics else None
        # Identity support must see the raw native velocity/pose, before this
        # node zeros UNKNOWN motion. The opt-in helper never alters geometry.
        continuity = getattr(self, "stationary_uuid_continuity", None)
        identity_input = continuity.apply(message) if continuity is not None else message
        output, constrained_count = apply_object_motion_policy(
            identity_input,
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
            # Reuse the existing opt-in diagnostic channel: identity decisions
            # must be auditable without extra publishers or per-frame console
            # noise. This is raw-track output evidence, not detector freshness.
            limit = 8
            events = continuity.last_events if continuity is not None else []
            rejections = continuity.last_rejections if continuity is not None else []
            identity = {
                "enabled": continuity.enabled if continuity is not None else False,
                "remapped_objects": sum(
                    bytes(before.object_id.uuid) != bytes(after.object_id.uuid)
                    for before, after in zip(message.objects, identity_input.objects)),
                "events": events[:limit],
                "rejections": rejections[:limit],
                "events_omitted": max(0, len(events) - limit),
                "rejections_omitted": max(0, len(rejections) - limit),
            }
            sample = self.processing_timing.finish(token, make_start, make_end, publish_end,
                                                  unknown_constrained=constrained_count,
                                                  uuid_continuity=identity)
            diagnostics.publish(String(data=json.dumps(sample, allow_nan=False, separators=(",", ":"))))


def main():
    rospy.init_node("tracked_object_motion_policy")
    TrackedObjectMotionPolicyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
