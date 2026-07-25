#!/usr/bin/env python3

import math
import os
import sys
import uuid

import rospy
from autoware_internal_planning_msgs.srv import (
    ClearRoute,
    SetLaneletRoute,
    SetLaneletRouteRequest,
)
from autoware_planning_msgs.msg import LaneletPrimitive, LaneletSegment
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
BPP_SCRIPT_DIR = os.path.realpath(
    os.path.join(
        SCRIPT_DIR,
        "..",
        "..",
        "..",
        "planning",
        "behavior_path_planner",
        "scripts",
    )
)
if BPP_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, BPP_SCRIPT_DIR)

from bpp_map_tools import CenterlineMap  # noqa: E402


COMPETITION_LANE_IDS = [
    200889,
    200392,
    200178,
    200337,
    200287,
    200348,
    200346,
    200291,
    200292,
    200305,
    200841,
    200950,
    200310,
    200308,
    200258,
    200261,
    200315,
    200445,
    200311,
    200968,
    201095,
    201098,
    201096,
    201087,
    201086,
    200101,
    200128,
    200102,
    200134,
    200360,
    201399,
    200088,
    201401,
    200183,
    200037,
    200471,
    201071,
    200182,
    200361,
    200181,
    200892,
]


def quaternion_from_yaw(yaw):
    pose = Pose()
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose.orientation


def parse_lane_ids(value):
    if isinstance(value, str):
        value = value.strip()
        if not value:
            return list(COMPETITION_LANE_IDS)
        value = value.strip("[]")
        return [int(token.strip()) for token in value.split(",") if token.strip()]
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    return list(COMPETITION_LANE_IDS)


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class KcityCompetitionLiveRoutePublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.map_file = rospy.get_param(
            "~map_file",
            os.path.expanduser(
                "~/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v44.osm"
            ),
        )
        self.projector_file = rospy.get_param(
            "~projector_file",
            os.path.expanduser(
                "~/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml"
            ),
        )
        self.projector_type = rospy.get_param("~map_projector_type", "LocalCartesianUTM")
        self.lane_ids = parse_lane_ids(
            rospy.get_param("~route_lane_ids", COMPETITION_LANE_IDS)
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/kinematic_state"
        )
        self.clear_service = rospy.get_param(
            "~clear_service", "/planning/mission_planning/mission_planner/clear_route"
        )
        self.route_service = rospy.get_param(
            "~route_service",
            "/planning/mission_planning/mission_planner/set_lanelet_route",
        )
        self.keep_alive = bool(rospy.get_param("~keep_alive", True))
        self.clear_existing_route = bool(rospy.get_param("~clear_existing_route", True))
        self.route_retry_timeout = float(rospy.get_param("~route_retry_timeout", 30.0))
        self.marker_pub = rospy.Publisher(
            "/planning/mission_planning/competition_route/checkpoints",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.centerline_map = CenterlineMap.load(
            self.map_file,
            self.projector_file,
            False,
            self.projector_type,
        )
        missing = [lane_id for lane_id in self.lane_ids if not self.centerline_map.has_lane(lane_id)]
        if missing:
            raise RuntimeError("route lane ids are missing in map: {}".format(missing))

    def lane_goal_pose(self):
        centerline = self.centerline_map.get_centerline(self.lane_ids[-1])
        if not centerline or len(centerline) < 2:
            raise RuntimeError("last route lane has no usable centerline")
        previous = centerline[-2]
        last = centerline[-1]
        yaw = math.atan2(last[1] - previous[1], last[0] - previous[0])
        pose = Pose()
        pose.position.x = last[0]
        pose.position.y = last[1]
        pose.position.z = last[2]
        pose.orientation = quaternion_from_yaw(yaw)
        return pose

    def make_segments(self):
        segments = []
        for lane_id in self.lane_ids:
            primitive = LaneletPrimitive(id=int(lane_id), primitive_type="lane")
            segment = LaneletSegment()
            segment.preferred_primitive = primitive
            segment.primitives = [primitive]
            segments.append(segment)
        return segments

    def clear_route(self):
        if not self.clear_existing_route:
            return
        try:
            rospy.wait_for_service(self.clear_service, timeout=3.0)
            response = rospy.ServiceProxy(self.clear_service, ClearRoute)()
            if not response.status.success:
                rospy.logwarn("live route clear rejected: %s", response.status.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("live route clear unavailable: %s", error)

    def publish_markers(self, odometry, goal_pose):
        markers = []
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers.append(clear)

        entries = [
            ("LIVE EGO START", odometry.pose.pose, color(1.0, 0.25, 0.0)),
            ("ROUTE GOAL", goal_pose, color(1.0, 0.1, 0.1)),
        ]
        for index, (label, pose, marker_color) in enumerate(entries):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "kcity_live_route_points"
            sphere.id = index * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 4.0
            sphere.color = marker_color
            markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "kcity_live_route_labels"
            text.id = index * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose
            text.pose.position.z += 4.0
            text.scale.z = 2.8
            text.color = marker_color
            text.text = label
            markers.append(text)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def set_route(self):
        rospy.loginfo("waiting for live odometry on %s", self.odometry_topic)
        odometry = rospy.wait_for_message(self.odometry_topic, Odometry, timeout=10.0)
        goal_pose = self.lane_goal_pose()
        self.publish_markers(odometry, goal_pose)

        self.clear_route()
        rospy.wait_for_service(self.route_service, timeout=10.0)
        request = SetLaneletRouteRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.frame_id
        request.goal_pose = goal_pose
        request.segments = self.make_segments()
        request.uuid.uuid = list(uuid.uuid4().bytes)
        request.allow_modification = False

        service = rospy.ServiceProxy(self.route_service, SetLaneletRoute)
        deadline = rospy.Time.now() + rospy.Duration(self.route_retry_timeout)
        last_message = "route service did not become ready"
        while not rospy.is_shutdown():
            request.header.stamp = rospy.Time.now()
            try:
                response = service(request)
                if response.status.success:
                    rospy.loginfo(
                        "K-City live competition route set: %d lanelets, ego=(%.2f, %.2f), goal=(%.2f, %.2f)",
                        len(self.lane_ids),
                        odometry.pose.pose.position.x,
                        odometry.pose.pose.position.y,
                        goal_pose.position.x,
                        goal_pose.position.y,
                    )
                    return
                last_message = response.status.message
            except rospy.ServiceException as error:
                last_message = str(error)
            if rospy.Time.now() >= deadline:
                break
            rospy.logwarn_throttle(2.0, "K-City live route waiting: %s", last_message)
            rospy.sleep(0.5)
        raise RuntimeError(last_message)

    def run(self):
        self.set_route()
        if self.keep_alive:
            rospy.spin()


def main():
    rospy.init_node("kcity_competition_live_route_publisher")
    try:
        KcityCompetitionLiveRoutePublisher().run()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("Failed to set K-City live competition route: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
