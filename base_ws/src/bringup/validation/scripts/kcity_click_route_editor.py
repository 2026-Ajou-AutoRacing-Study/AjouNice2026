#!/usr/bin/env python3

import colorsys
import math
import os
import sys
import threading
import uuid

import rospy
import yaml
from autoware_internal_planning_msgs.srv import (
    ClearRoute,
    SetLaneletRoute,
    SetLaneletRouteRequest,
)
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletPrimitive, LaneletRoute, LaneletSegment
from geometry_msgs.msg import Point, PointStamped, Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BPP_SCRIPT_DIR = os.path.abspath(
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
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from bpp_map_tools import CenterlineMap, project_point_to_polyline  # noqa: E402
from kcity_competition_live_route_publisher import (  # noqa: E402
    COMPETITION_CANDIDATE_LANE_IDS,
    COMPETITION_LANE_IDS,
    COMPETITION_PREFERRED_LANE_IDS,
    quaternion_from_yaw,
)


def lane_ids_param_is_set(value):
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip() not in ("", "[]")
    if isinstance(value, (list, tuple)):
        return len(value) > 0
    return True


def parse_lane_ids(value, default_lane_ids=None):
    default_lane_ids = list(default_lane_ids or [])
    if isinstance(value, str):
        stripped = value.strip().strip("[]")
        if not stripped:
            return list(default_lane_ids)
        return [
            int(token.strip())
            for token in stripped.replace(";", ",").split(",")
            if token.strip()
        ]
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    if value is None:
        return list(default_lane_ids)
    return [int(value)]


def unique_lane_ids(lane_ids):
    result = []
    for lane_id in lane_ids:
        lane_id = int(lane_id)
        if lane_id not in result:
            result.append(lane_id)
    return result


def color(r, g, b, a=1.0):
    value = ColorRGBA()
    value.r = float(r)
    value.g = float(g)
    value.b = float(b)
    value.a = float(a)
    return value


def point_from_tuple(values, z_offset=0.0):
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2]) + float(z_offset)
    return point


def lane_color(lane_id, alpha=0.9):
    hue = ((int(lane_id) * 0.61803398875) % 1.0)
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.72, 1.0)
    return color(red, green, blue, alpha)


def lane_end_pose(centerline):
    if not centerline or len(centerline) < 2:
        raise RuntimeError("route goal lane has no usable centerline")
    previous = centerline[-2]
    last = centerline[-1]
    yaw = math.atan2(last[1] - previous[1], last[0] - previous[0])
    pose = Pose()
    pose.position.x = last[0]
    pose.position.y = last[1]
    pose.position.z = last[2]
    pose.orientation = quaternion_from_yaw(yaw)
    return pose


def midpoint(points):
    if not points:
        return None
    return points[len(points) // 2]


class KcityClickRouteEditor:
    def __init__(self):
        self.lock = threading.RLock()
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.map_file = rospy.get_param("~map_file", "")
        self.vector_map_topic = rospy.get_param("~vector_map_topic", "/map/vector_map")
        self.projector_file = rospy.get_param(
            "~projector_file",
            os.path.expanduser(
                "~/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml"
            ),
        )
        self.projector_type = rospy.get_param("~map_projector_type", "LocalCartesianUTM")
        self.map_wait_timeout = float(rospy.get_param("~map_wait_timeout", 10.0))
        self.clicked_point_topic = rospy.get_param("~clicked_point_topic", "/clicked_point")
        self.delete_clicked_point_topic = rospy.get_param(
            "~delete_clicked_point_topic", "/clicked_point_delete"
        )
        self.route_topic = rospy.get_param("~route_topic", "/planning/mission_planning/route")
        self.clear_service = rospy.get_param(
            "~clear_service", "/planning/mission_planning/mission_planner/clear_route"
        )
        self.route_service = rospy.get_param(
            "~route_service",
            "/planning/mission_planning/mission_planner/set_lanelet_route",
        )
        self.clear_existing_route = bool(rospy.get_param("~clear_existing_route", True))
        self.set_route_on_start = bool(rospy.get_param("~set_route_on_start", True))
        self.sync_from_route_topic = bool(rospy.get_param("~sync_from_route_topic", False))
        self.allow_duplicate_lanelets = bool(rospy.get_param("~allow_duplicate_lanelets", False))
        self.delete_all_matching_lanelets = bool(
            rospy.get_param("~delete_all_matching_lanelets", False)
        )
        self.selection_distance_threshold = float(
            rospy.get_param("~selection_distance_threshold", 6.0)
        )
        self.route_retry_timeout = float(rospy.get_param("~route_retry_timeout", 15.0))
        self.route_output_file = rospy.get_param(
            "~route_output_file", "/tmp/kcity_route_editor_lane_ids.yaml"
        )
        self.publish_colored_lanelets = bool(rospy.get_param("~publish_colored_lanelets", True))
        self.publish_lanelet_labels = bool(rospy.get_param("~publish_lanelet_labels", False))
        self.colored_lanelet_width = float(rospy.get_param("~colored_lanelet_width", 0.22))
        self.route_lanelet_width = float(rospy.get_param("~route_lanelet_width", 0.55))
        route_lane_ids = rospy.get_param("~route_lane_ids", [])
        if lane_ids_param_is_set(route_lane_ids):
            parsed_route_lane_ids = parse_lane_ids(route_lane_ids, [])
            split_index = min(len(COMPETITION_PREFERRED_LANE_IDS), len(parsed_route_lane_ids))
            self.preferred_lane_ids = parsed_route_lane_ids[:split_index]
            self.candidate_lane_ids = parsed_route_lane_ids[split_index:]
        else:
            self.preferred_lane_ids = parse_lane_ids(
                rospy.get_param("~preferred_lane_ids", COMPETITION_PREFERRED_LANE_IDS),
                COMPETITION_PREFERRED_LANE_IDS,
            )
            self.candidate_lane_ids = parse_lane_ids(
                rospy.get_param("~candidate_lane_ids", COMPETITION_CANDIDATE_LANE_IDS),
                COMPETITION_CANDIDATE_LANE_IDS,
            )
        self.last_added_lane_id = None
        self.last_removed_lane_id = None

        self.centerline_map = self.load_centerline_map()
        self.preferred_lane_ids = unique_lane_ids(
            lane_id for lane_id in self.preferred_lane_ids if self.centerline_map.has_lane(lane_id)
        )
        self.candidate_lane_ids = unique_lane_ids(
            lane_id
            for lane_id in self.candidate_lane_ids
            if self.centerline_map.has_lane(lane_id) and lane_id not in self.preferred_lane_ids
        )
        self.sync_route_lane_ids()
        if not self.preferred_lane_ids:
            raise RuntimeError("click route editor has no valid preferred route lane IDs")

        self.colored_lanelet_pub = rospy.Publisher(
            "/planning/mission_planning/click_route_editor/lanelet_markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.route_marker_pub = rospy.Publisher(
            "/planning/mission_planning/click_route_editor/route_markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            self.clicked_point_topic, PointStamped, self.on_add_clicked_point, queue_size=1
        )
        if self.delete_clicked_point_topic:
            rospy.Subscriber(
                self.delete_clicked_point_topic,
                PointStamped,
                self.on_delete_clicked_point,
                queue_size=1,
            )
        if self.sync_from_route_topic:
            rospy.Subscriber(self.route_topic, LaneletRoute, self.on_route, queue_size=1)

        if self.publish_colored_lanelets:
            self.publish_all_lanelet_markers()
        self.publish_route_markers()
        self.write_route_file()

        if self.set_route_on_start:
            rospy.Timer(rospy.Duration(0.5), self.set_initial_route_once, oneshot=True)

        rospy.loginfo(
            "kcity_click_route_editor: add with RViz Publish Point on %s, delete on %s; "
            "initial route has %d lanelets",
            self.clicked_point_topic,
            self.delete_clicked_point_topic,
            len(self.route_lane_ids),
        )

    def load_centerline_map(self):
        if self.map_file:
            return CenterlineMap.load(
                self.map_file,
                self.projector_file,
                False,
                self.projector_type,
            )

        rospy.loginfo("click route editor waiting for planning map on %s", self.vector_map_topic)
        message = rospy.wait_for_message(
            self.vector_map_topic, LaneletMapBin, timeout=self.map_wait_timeout
        )
        if not message.data:
            raise RuntimeError("received empty planning map on {}".format(self.vector_map_topic))

        centerline_map = CenterlineMap.from_osm_bytes(
            message.data,
            message.name_map or "LaneletMapBin",
            False,
            self.projector_type,
            self.projector_file,
        )
        rospy.loginfo(
            "click route editor loaded centerline map from %s: lanes=%d",
            self.vector_map_topic,
            centerline_map.lane_count(),
        )
        return centerline_map

    def sync_route_lane_ids(self):
        self.route_lane_ids = self.preferred_lane_ids + [
            lane_id
            for lane_id in self.candidate_lane_ids
            if lane_id not in self.preferred_lane_ids
        ]

    def set_initial_route_once(self, _event):
        try:
            self.set_route()
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            rospy.logwarn("initial clickable route was not set yet: %s", error)

    def on_route(self, message):
        preferred_lane_ids = unique_lane_ids(
            int(segment.preferred_primitive.id)
            for segment in message.segments
            if int(segment.preferred_primitive.id) != 0
            and self.centerline_map.has_lane(int(segment.preferred_primitive.id))
        )
        candidate_lane_ids = []
        for segment in message.segments:
            for primitive in segment.primitives:
                lane_id = int(primitive.id)
                if (
                    lane_id
                    and lane_id not in preferred_lane_ids
                    and lane_id not in candidate_lane_ids
                    and self.centerline_map.has_lane(lane_id)
                ):
                    candidate_lane_ids.append(lane_id)
        if not preferred_lane_ids:
            return
        with self.lock:
            if (
                preferred_lane_ids == self.preferred_lane_ids
                and candidate_lane_ids == self.candidate_lane_ids
            ):
                return
            self.preferred_lane_ids = preferred_lane_ids
            self.candidate_lane_ids = candidate_lane_ids
            self.sync_route_lane_ids()
            self.last_added_lane_id = None
            self.last_removed_lane_id = None
            self.publish_route_markers()
            self.write_route_file()

    def clicked_lanelet_id(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            rospy.logwarn(
                "Ignoring clicked point in frame %s; expected %s",
                message.header.frame_id,
                self.frame_id,
            )
            return None

        clicked = (message.point.x, message.point.y, message.point.z)
        result = self.closest_lanelet(clicked)
        if result is None:
            rospy.logwarn("No lanelet centerline is available for clicked point")
            return None
        lane_id, distance = result
        if distance > self.selection_distance_threshold:
            rospy.logwarn(
                "Clicked point is %.2f m from nearest lanelet %s; threshold is %.2f m",
                distance,
                lane_id,
                self.selection_distance_threshold,
            )
            return None
        return lane_id

    def on_add_clicked_point(self, message):
        lane_id = self.clicked_lanelet_id(message)
        if lane_id is None:
            return

        with self.lock:
            if not self.allow_duplicate_lanelets and lane_id in self.route_lane_ids:
                rospy.loginfo(
                    "Lanelet %s is already in the route; not adding a duplicate candidate",
                    lane_id,
                )
                self.last_added_lane_id = lane_id
                self.last_removed_lane_id = None
                self.publish_route_markers()
                return

            self.candidate_lane_ids.append(lane_id)
            self.sync_route_lane_ids()
            self.last_added_lane_id = lane_id
            self.last_removed_lane_id = None
            self.publish_route_markers()
            self.write_route_file()

        try:
            self.set_route()
            rospy.loginfo(
                "Added lanelet %s as route candidate: preferred=%d candidates=%d",
                lane_id,
                len(self.preferred_lane_ids),
                len(self.candidate_lane_ids),
            )
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            rospy.logerr("Failed to publish route after adding lanelet %s: %s", lane_id, error)

    def on_delete_clicked_point(self, message):
        lane_id = self.clicked_lanelet_id(message)
        if lane_id is None:
            return

        with self.lock:
            if lane_id not in self.route_lane_ids:
                rospy.loginfo("Lanelet %s is not in the route; nothing to delete", lane_id)
                self.last_added_lane_id = None
                self.last_removed_lane_id = lane_id
                self.publish_route_markers()
                return
            if lane_id in self.preferred_lane_ids and len(self.preferred_lane_ids) == 1:
                rospy.logwarn(
                    "Cannot delete lanelet %s because it is the only preferred route lanelet",
                    lane_id,
                )
                return

            if self.delete_all_matching_lanelets:
                self.candidate_lane_ids = [
                    candidate_lane_id
                    for candidate_lane_id in self.candidate_lane_ids
                    if candidate_lane_id != lane_id
                ]
                self.preferred_lane_ids = [
                    preferred_lane_id
                    for preferred_lane_id in self.preferred_lane_ids
                    if preferred_lane_id != lane_id
                ]
            else:
                target_lane_ids = (
                    self.candidate_lane_ids
                    if lane_id in self.candidate_lane_ids
                    else self.preferred_lane_ids
                )
                for index in range(len(target_lane_ids) - 1, -1, -1):
                    if target_lane_ids[index] == lane_id:
                        del target_lane_ids[index]
                        break
            self.sync_route_lane_ids()
            self.last_added_lane_id = None
            self.last_removed_lane_id = lane_id
            self.publish_route_markers()
            self.write_route_file()

        try:
            self.set_route()
            rospy.loginfo(
                "Deleted lanelet %s from clickable route: preferred=%d candidates=%d",
                lane_id,
                len(self.preferred_lane_ids),
                len(self.candidate_lane_ids),
            )
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            rospy.logerr("Failed to publish route after deleting lanelet %s: %s", lane_id, error)

    def closest_lanelet(self, clicked):
        best = None
        for lane_id_text, centerline in self.centerline_map.centerlines.items():
            projection = project_point_to_polyline(centerline, clicked)
            if projection is None:
                continue
            distance = float(projection[1])
            lane_id = int(lane_id_text)
            if best is None or distance < best[1]:
                best = (lane_id, distance)
        return best

    def closest_preferred_lane_id(self, lane_id):
        centerline = self.centerline_map.get_centerline(lane_id)
        if not centerline:
            return None
        lane_midpoint = centerline[len(centerline) // 2]
        best = None
        for preferred_lane_id in self.preferred_lane_ids:
            preferred_centerline = self.centerline_map.get_centerline(preferred_lane_id)
            projection = project_point_to_polyline(preferred_centerline, lane_midpoint)
            if projection is None:
                continue
            distance = float(projection[1])
            if best is None or distance < best[0]:
                best = (distance, preferred_lane_id)
        return best[1] if best is not None else None

    def candidate_assignments(self):
        assignments = {lane_id: [] for lane_id in self.preferred_lane_ids}
        for lane_id in self.candidate_lane_ids:
            if lane_id in assignments:
                continue
            preferred_lane_id = self.closest_preferred_lane_id(lane_id)
            if preferred_lane_id is not None and lane_id not in assignments[preferred_lane_id]:
                assignments[preferred_lane_id].append(lane_id)
        return assignments

    def make_segments(self):
        segments = []
        assignments = self.candidate_assignments()
        for lane_id in self.preferred_lane_ids:
            primitive = LaneletPrimitive(id=int(lane_id), primitive_type="lane")
            segment = LaneletSegment()
            segment.preferred_primitive = primitive
            segment.primitives = [primitive]
            for candidate_lane_id in assignments.get(lane_id, []):
                segment.primitives.append(
                    LaneletPrimitive(id=int(candidate_lane_id), primitive_type="lane")
                )
            segments.append(segment)
        return segments

    def route_goal_pose(self):
        centerline = self.centerline_map.get_centerline(self.preferred_lane_ids[-1])
        return lane_end_pose(centerline)

    def clear_route(self):
        if not self.clear_existing_route:
            return
        try:
            rospy.wait_for_service(self.clear_service, timeout=3.0)
            response = rospy.ServiceProxy(self.clear_service, ClearRoute)()
            if not response.status.success:
                rospy.logwarn("click route clear rejected: %s", response.status.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("click route clear unavailable: %s", error)

    def set_route(self):
        if not self.preferred_lane_ids:
            raise RuntimeError("preferred route lane ID list is empty")
        self.clear_route()
        rospy.wait_for_service(self.route_service, timeout=10.0)
        request = SetLaneletRouteRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.frame_id
        request.goal_pose = self.route_goal_pose()
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
                    return
                last_message = response.status.message
            except rospy.ServiceException as error:
                last_message = str(error)
            if rospy.Time.now() >= deadline:
                break
            rospy.logwarn_throttle(2.0, "click route waiting: %s", last_message)
            rospy.sleep(0.5)
        raise RuntimeError(last_message)

    def write_route_file(self):
        if not self.route_output_file:
            return
        directory = os.path.dirname(self.route_output_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary_path = "{}.tmp".format(self.route_output_file)
        data = {
            "preferred_lane_ids": [int(lane_id) for lane_id in self.preferred_lane_ids],
            "candidate_lane_ids": [int(lane_id) for lane_id in self.candidate_lane_ids],
            "route_lane_ids": [int(lane_id) for lane_id in self.route_lane_ids],
            "count": len(self.route_lane_ids),
            "preferred_count": len(self.preferred_lane_ids),
            "candidate_count": len(self.candidate_lane_ids),
            "candidate_assignments": {
                int(preferred_lane_id): [int(lane_id) for lane_id in candidate_lane_ids]
                for preferred_lane_id, candidate_lane_ids in self.candidate_assignments().items()
                if candidate_lane_ids
            },
            "last_added_lane_id": self.last_added_lane_id,
            "last_removed_lane_id": self.last_removed_lane_id,
        }
        with open(temporary_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        os.replace(temporary_path, self.route_output_file)

    def publish_all_lanelet_markers(self):
        markers = [self.delete_all_marker("click_editor_colored_lanelets")]
        for index, lane_id in enumerate(sorted(int(lane_id) for lane_id in self.centerline_map.centerlines)):
            centerline = self.centerline_map.get_centerline(lane_id)
            if not centerline:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "click_editor_colored_lanelets"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.colored_lanelet_width
            marker.color = lane_color(lane_id, 0.78)
            marker.points = [point_from_tuple(point, 0.28) for point in centerline]
            markers.append(marker)

            if self.publish_lanelet_labels:
                label_point = midpoint(centerline)
                if label_point is not None:
                    markers.append(
                        self.text_marker(
                            marker_id=100000 + index,
                            namespace="click_editor_lanelet_labels",
                            lane_id=lane_id,
                            text=str(lane_id),
                            point=label_point,
                            marker_color=lane_color(lane_id, 0.95),
                            z_offset=1.2,
                            scale=1.1,
                        )
                    )
        self.colored_lanelet_pub.publish(MarkerArray(markers=markers))

    def publish_route_markers(self):
        markers = [self.delete_all_marker("click_editor_route")]
        marker_entries = [
            ("P", index, lane_id, False)
            for index, lane_id in enumerate(self.preferred_lane_ids)
        ] + [
            ("C", index, lane_id, True)
            for index, lane_id in enumerate(self.candidate_lane_ids)
        ]
        for marker_index, (prefix, index, lane_id, is_candidate) in enumerate(marker_entries):
            centerline = self.centerline_map.get_centerline(lane_id)
            if not centerline:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "click_editor_route"
            marker.id = marker_index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.route_lanelet_width * (0.82 if is_candidate else 1.0)
            if lane_id == self.last_added_lane_id:
                marker.color = color(1.0, 0.15, 0.08, 1.0)
            elif is_candidate:
                marker.color = color(1.0, 0.72, 0.08, 1.0)
            else:
                marker.color = color(0.08, 1.0, 0.95, 1.0)
            marker.points = [point_from_tuple(point, 0.72) for point in centerline]
            markers.append(marker)

            label_point = midpoint(centerline)
            if label_point is not None:
                markers.append(
                    self.text_marker(
                        marker_id=10000 + marker_index,
                        namespace="click_editor_route_labels",
                        lane_id=lane_id,
                        text="{}{}: {}".format(prefix, index + 1, lane_id),
                        point=label_point,
                        marker_color=color(1.0, 1.0, 1.0, 0.95),
                        z_offset=2.0,
                        scale=1.35,
                    )
                )
        self.route_marker_pub.publish(MarkerArray(markers=markers))

    def delete_all_marker(self, namespace):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        return marker

    def text_marker(
        self,
        marker_id,
        namespace,
        lane_id,
        text,
        point,
        marker_color,
        z_offset,
        scale,
    ):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position = point_from_tuple(point, z_offset)
        marker.scale.z = float(scale)
        marker.color = marker_color
        marker.text = str(text)
        return marker


def main():
    rospy.init_node("kcity_click_route_editor")
    try:
        KcityClickRouteEditor()
        rospy.spin()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("Failed to start K-City click route editor: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
