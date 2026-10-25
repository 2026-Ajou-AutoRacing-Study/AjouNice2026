#!/usr/bin/env python3
"""Publish the next route traffic-light group ID from the ROS Python runtime.

The Lanelet2 Python binding shipped with ROS Noetic is tied to Python 3.8,
whereas the RTX 50-series compatible PyTorch runtime uses Python 3.11.  This
small node deliberately stays on /usr/bin/python3 and isolates map/route
matching from GPU inference.
"""

import os

import rospy
from std_msgs.msg import UInt64

from lanelet_route_matcher import LaneletRouteGroupIdMatcher


# The MORAI/Lanelet group IDs in the checked table are positive six-digit
# values. Zero is the package's existing "no valid route match" fallback and
# lets the split process preserve the monolithic matcher's invalid state.
INVALID_GROUP_ID = 0


EXCLUDED_MAP_ERROR_IDS = {201395, 201396}
MISSION_GROUP_IDS = frozenset((201351, 201365, 201290, 201326))


def restrict_route_groups(lanelet_group_id, allowed_group_ids):
    """Opt-in candidate filter: skip non-mission lights, not an invalid lease.

    Empty list preserves the legacy route provider, including its D-1
    passthrough. The image node independently enforces the same mission scope.
    """
    if not isinstance(allowed_group_ids, list) or any(
            type(value) is not int or value not in MISSION_GROUP_IDS for value in allowed_group_ids):
        raise ValueError('allowed_group_ids must be a list of explicit A/B/C/D mission IDs')
    allowed = set(allowed_group_ids)
    return {lane:group for lane,group in lanelet_group_id.items() if not allowed or group in allowed}


class TrafficLightRouteGroupIdNode:
    def __init__(self):
        map_file = rospy.get_param("~lanelet2_map_file")
        projector_file = rospy.get_param("~lanelet2_projector_file")
        projector_type = rospy.get_param("~lanelet2_projector_type", "LocalCartesianUTM")
        table_path = rospy.get_param("~group_id_table_path")
        route_topic = rospy.get_param("~route_topic", "/planning/mission_planning/route")
        route_state_topic = rospy.get_param("~route_state_topic", "/planning/route_state")
        odometry_topic = rospy.get_param("~odometry_topic", "/localization/kinematic_state")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/traffic_light_recognition/route_group_id"
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))

        for path, label in ((map_file, "lanelet map"), (projector_file, "projector"), (table_path, "group table")):
            if not os.path.isfile(path):
                raise RuntimeError("{} file not found: {}".format(label, path))

        self._matcher = LaneletRouteGroupIdMatcher(
            map_file=map_file,
            projector_type=projector_type,
            projector_file=projector_file,
            excluded_group_ids=EXCLUDED_MAP_ERROR_IDS,
            route_progress_guard=rospy.get_param('~route_progress_guard', False),
            route_progress_hysteresis_m=rospy.get_param('~route_progress_hysteresis_m', 1.0),
            route_progress_max_lateral_m=rospy.get_param('~route_progress_max_lateral_m', 8.0),
            route_progress_pose_jump_m=rospy.get_param('~route_progress_pose_jump_m', 8.0),
        )
        if not self._matcher.available:
            raise RuntimeError("Lanelet route matcher is unavailable in the ROS runtime")
        self._matcher.lanelet_group_id = restrict_route_groups(
            self._matcher.lanelet_group_id, rospy.get_param('~allowed_group_ids', []))
        self._matcher.start_subscribers(route_topic, odometry_topic, route_state_topic)

        self._publisher = rospy.Publisher(output_topic, UInt64, queue_size=1, latch=True)
        period = 1.0 / max(publish_rate_hz, 0.1)
        self._timer = rospy.Timer(rospy.Duration(period), self._publish)
        rospy.loginfo(
            "[traffic_light_route_group_id] ready: %s at %.1f Hz", output_topic, publish_rate_hz
        )

    def _publish(self, _event):
        group_id = self._matcher.get_group_id()
        self._publisher.publish(
            UInt64(data=INVALID_GROUP_ID if group_id is None else int(group_id))
        )


def main():
    rospy.init_node("traffic_light_route_group_id")
    TrafficLightRouteGroupIdNode()
    rospy.spin()


if __name__ == "__main__":
    main()
