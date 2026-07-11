#!/usr/bin/env python3

import math
import os
import sys
import tempfile
import threading
import time
import uuid
from copy import deepcopy

import rospy
import tf2_geometry_msgs
import tf2_ros
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_common_msgs.msg import ResponseStatus
from autoware_internal_debug_msgs.msg import Float64Stamped
from autoware_internal_planning_msgs.msg import RouteState
from autoware_internal_planning_msgs.srv import (
    ClearRoute,
    ClearRouteResponse,
    SetLaneletRoute,
    SetLaneletRouteResponse,
    SetWaypointRoute,
    SetWaypointRouteResponse,
)
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import (
    LaneletPrimitive,
    LaneletRoute,
    LaneletSegment,
    PoseWithUuidStamped,
)
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from lanelet2_route_planner import (  # noqa: E402
    Lanelet2RoutePlanner,
    RoutePlanningError,
    normalize_angle,
    yaw_from_quaternion,
)


def quaternion_from_yaw(yaw):
    quaternion = Quaternion()
    quaternion.w = math.cos(yaw * 0.5)
    quaternion.z = math.sin(yaw * 0.5)
    return quaternion


def pose_xy(pose):
    return (float(pose.position.x), float(pose.position.y), float(pose.position.z))


def distance_xy(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def make_response(success, message, code=0):
    status = ResponseStatus()
    status.success = bool(success)
    status.code = int(code)
    status.message = str(message)
    return status


def uuid_msg_from_uuid(value):
    return [int(byte) for byte in value.bytes]


def uuid_values(value):
    return tuple(int(byte) for byte in value.uuid)


class MissionPlanner:
    def __init__(self):
        self.lock = threading.RLock()
        self.frame_id = rospy.get_param("~map_frame", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.map_file = rospy.get_param("~map_file", "")
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.legacy_lane_ids = self.parse_lane_ids(rospy.get_param("~route_lane_ids", []))

        self.goal_angle_threshold_deg = float(
            rospy.get_param("~goal_angle_threshold_deg", 45.0)
        )
        self.enable_correct_goal_pose = bool(
            rospy.get_param("~enable_correct_goal_pose", False)
        )
        self.consider_no_drivable_lanes = bool(
            rospy.get_param("~consider_no_drivable_lanes", False)
        )
        self.check_footprint_inside_lanes = bool(
            rospy.get_param("~check_footprint_inside_lanes", True)
        )
        self.vehicle_front_offset = float(rospy.get_param("~vehicle_front_offset", 3.79))
        self.vehicle_rear_offset = float(rospy.get_param("~vehicle_rear_offset", 1.10))
        self.vehicle_left_offset = float(rospy.get_param("~vehicle_left_offset", 0.948))
        self.vehicle_right_offset = float(rospy.get_param("~vehicle_right_offset", 0.948))
        self.preferred_primitive_type = str(
            rospy.get_param("~preferred_primitive_type", "")
        )

        self.allow_reroute_in_autonomous_mode = bool(
            rospy.get_param("~allow_reroute_in_autonomous_mode", True)
        )
        self.reroute_time_threshold = float(
            rospy.get_param("~reroute_time_threshold", 10.0)
        )
        self.minimum_reroute_length = float(
            rospy.get_param("~minimum_reroute_length", 30.0)
        )

        self.arrival_check_angle = math.radians(
            float(rospy.get_param("~arrival_check_angle_deg", 45.0))
        )
        self.arrival_check_distance = float(
            rospy.get_param("~arrival_check_distance", 1.0)
        )
        self.arrival_check_duration = float(
            rospy.get_param("~arrival_check_duration", 1.0)
        )
        self.arrival_stop_velocity_threshold = float(
            rospy.get_param("~arrival_stop_velocity_threshold", 0.01)
        )

        self.auto_publish_initial_route = bool(
            rospy.get_param("~auto_publish_initial_route", False)
        )
        self.default_start_x = float(rospy.get_param("~default_start_x", 0.0))
        self.default_start_y = float(rospy.get_param("~default_start_y", 0.0))
        self.default_start_yaw = float(rospy.get_param("~default_start_yaw", 0.0))
        self.default_goal_x = float(rospy.get_param("~default_goal_x", 60.0))
        self.default_goal_y = float(rospy.get_param("~default_goal_y", 0.0))
        self.default_goal_yaw = float(rospy.get_param("~default_goal_yaw", 0.0))
        self.allow_modification_default = bool(rospy.get_param("~allow_modification", True))

        self.odometry = None
        self.operation_mode = None
        self.vector_map = None
        self.route_planner = None
        self.current_route = None
        self.current_preferred_lane_ids = []
        self.route_state = RouteState.INITIALIZING
        self.stopped_since = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("~input/odometry", Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber(
            "~input/operation_mode_state",
            OperationModeState,
            self.operation_mode_callback,
            queue_size=1,
        )
        rospy.Subscriber("~input/vector_map", LaneletMapBin, self.vector_map_callback, queue_size=1)
        rospy.Subscriber(
            "~input/modified_goal",
            PoseWithUuidStamped,
            self.modified_goal_callback,
            queue_size=1,
        )

        self.route_pub = rospy.Publisher("~route", LaneletRoute, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("~state", RouteState, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~debug/route_marker", MarkerArray, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher("~debug/goal_pose", PoseStamped, queue_size=1, latch=True)
        self.processing_time_pub = rospy.Publisher(
            "~debug/processing_time_ms", Float64Stamped, queue_size=1
        )

        rospy.Service("~clear_route", ClearRoute, self.handle_clear_route)
        rospy.Service("~set_lanelet_route", SetLaneletRoute, self.handle_set_lanelet_route)
        rospy.Service("~set_waypoint_route", SetWaypointRoute, self.handle_set_waypoint_route)

        if self.map_file:
            self.load_route_planner(self.map_file)
        self.publish_state()

        if self.auto_publish_initial_route:
            rospy.Timer(rospy.Duration(0.5), self.publish_initial_route_once, oneshot=True)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 0.1)), self.on_timer)

    def parse_lane_ids(self, value):
        if isinstance(value, str):
            stripped = value.strip().strip("[]")
            if not stripped:
                return []
            return [
                int(token.strip())
                for token in stripped.replace(";", ",").split(",")
                if token.strip()
            ]
        if isinstance(value, (list, tuple)):
            return [int(item) for item in value]
        if value is None:
            return []
        return [int(value)]

    def planner_options(self):
        return {
            "projector_type": self.projector_type,
            "projector_file": self.projector_file,
            "goal_angle_threshold_deg": self.goal_angle_threshold_deg,
            "enable_correct_goal_pose": self.enable_correct_goal_pose,
            "consider_no_drivable_lanes": self.consider_no_drivable_lanes,
            "check_footprint_inside_lanes": self.check_footprint_inside_lanes,
            "vehicle_front_offset": self.vehicle_front_offset,
            "vehicle_rear_offset": self.vehicle_rear_offset,
            "vehicle_left_offset": self.vehicle_left_offset,
            "vehicle_right_offset": self.vehicle_right_offset,
            "preferred_primitive_type": self.preferred_primitive_type,
        }

    def load_route_planner(self, map_file):
        try:
            planner = Lanelet2RoutePlanner(map_file, **self.planner_options())
            with self.lock:
                self.route_planner = planner
                self.map_file = map_file
                self.update_ready_state()
            rospy.loginfo(
                "mission_planner: Lanelet2 routing graph ready with %d lanelets, projector=%s",
                planner.lanelet_count,
                planner.projector_type,
            )
        except Exception as error:
            with self.lock:
                self.route_planner = None
            rospy.logerr("mission_planner: failed to load Lanelet2 routing graph: %s", error)

    def vector_map_callback(self, message):
        with self.lock:
            self.vector_map = message
            if self.route_planner is not None or not message.data:
                return
        raw = bytes(message.data)
        if not raw.lstrip().startswith(b"<osm") and not raw.lstrip().startswith(b"<?xml"):
            rospy.logwarn_throttle(
                10.0,
                "mission_planner: ROS1 binary LaneletMapBin loading is unavailable; pass map_file",
            )
            return
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".osm", delete=False) as temporary:
                temporary.write(raw)
                temporary_path = temporary.name
            self.load_route_planner(temporary_path)
        finally:
            if temporary_path and os.path.exists(temporary_path):
                os.unlink(temporary_path)

    def update_ready_state(self):
        if (
            self.route_state == RouteState.INITIALIZING
            and (self.route_planner is not None or self.legacy_lane_ids)
            and self.odometry is not None
        ):
            self.route_state = RouteState.UNSET
            self.publish_state()

    def odometry_callback(self, message):
        with self.lock:
            self.odometry = message
            linear = message.twist.twist.linear
            speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
            if speed <= self.arrival_stop_velocity_threshold:
                if self.stopped_since is None:
                    self.stopped_since = rospy.Time.now()
            else:
                self.stopped_since = None
            self.update_ready_state()

    def operation_mode_callback(self, message):
        with self.lock:
            self.operation_mode = message

    def is_ready(self):
        return (self.route_planner is not None or bool(self.legacy_lane_ids)) and self.odometry is not None

    def is_autonomous_driving(self):
        return bool(
            self.operation_mode is not None
            and self.operation_mode.mode == OperationModeState.AUTONOMOUS
            and self.operation_mode.is_autoware_control_enabled
        )

    def publish_initial_route_once(self, _event):
        with self.lock:
            if not self.is_ready():
                rospy.logwarn("mission_planner: initial route skipped because planner is not ready")
                return
            goal = Pose()
            goal.position.x = self.default_goal_x
            goal.position.y = self.default_goal_y
            goal.orientation = quaternion_from_yaw(self.default_goal_yaw)
            self.process_waypoint_route(
                goal,
                [],
                None,
                self.allow_modification_default,
                rospy.Time.now(),
                "initial route",
            )

    def request_precondition_error(self):
        if self.route_state not in (RouteState.UNSET, RouteState.SET):
            return "route cannot be set in the current state"
        if not self.is_ready():
            return "mission planner is not ready; map and odometry are required"
        if self.route_state == RouteState.SET and self.operation_mode is None:
            return "operation mode state is required for rerouting"
        if (
            self.route_state == RouteState.SET
            and self.is_autonomous_driving()
            and not self.allow_reroute_in_autonomous_mode
        ):
            return "reroute is not allowed in autonomous mode"
        return None

    def transform_pose(self, pose, header):
        source_frame = header.frame_id or self.frame_id
        if source_frame == self.frame_id:
            return deepcopy(pose)
        pose_stamped = PoseStamped()
        pose_stamped.header = deepcopy(header)
        pose_stamped.pose = deepcopy(pose)
        transform = self.tf_buffer.lookup_transform(
            self.frame_id, source_frame, rospy.Time(0), rospy.Duration(0.5)
        )
        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose

    def current_start_pose(self):
        return deepcopy(self.odometry.pose.pose)

    def make_plan_segments(self, plan):
        segments = []
        for section_data in plan.sections:
            segment = LaneletSegment()
            segment.preferred_primitive.id = int(section_data.preferred_id)
            segment.preferred_primitive.primitive_type = self.preferred_primitive_type
            for lane_id in section_data.primitive_ids:
                primitive = LaneletPrimitive()
                primitive.id = int(lane_id)
                primitive.primitive_type = "lane"
                segment.primitives.append(primitive)
            segments.append(segment)
        return segments

    def make_legacy_segments(self, lane_ids):
        segments = []
        for lane_id in lane_ids:
            segment = LaneletSegment()
            segment.preferred_primitive.id = int(lane_id)
            segment.preferred_primitive.primitive_type = self.preferred_primitive_type
            primitive = LaneletPrimitive()
            primitive.id = int(lane_id)
            primitive.primitive_type = "lane"
            segment.primitives = [primitive]
            segments.append(segment)
        return segments

    def lane_ids_from_segments(self, segments):
        return [int(segment.preferred_primitive.id) for segment in segments]

    def build_waypoint_route(
        self, goal_pose, waypoints, route_uuid, allow_modification, stamp
    ):
        start_pose = self.current_start_pose()
        plan = None
        refined_goal = deepcopy(goal_pose)
        if self.legacy_lane_ids:
            segments = self.make_legacy_segments(self.legacy_lane_ids)
            preferred_ids = list(self.legacy_lane_ids)
        else:
            plan = self.route_planner.plan(start_pose, waypoints, goal_pose)
            segments = self.make_plan_segments(plan)
            preferred_ids = list(plan.path_ids)
            refined_goal.position.x = plan.goal_position[0]
            refined_goal.position.y = plan.goal_position[1]
            refined_goal.position.z = plan.goal_position[2]
            if plan.goal_yaw is not None:
                refined_goal.orientation = quaternion_from_yaw(plan.goal_yaw)

        route = LaneletRoute()
        route.header.stamp = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        route.header.frame_id = self.frame_id
        route.start_pose = start_pose
        route.goal_pose = refined_goal
        route.segments = segments
        if route_uuid is None:
            route.uuid.uuid = uuid_msg_from_uuid(uuid.uuid4())
        else:
            route.uuid = deepcopy(route_uuid)
        route.allow_modification = bool(allow_modification)
        return route, preferred_ids, plan

    def build_lanelet_route(
        self, goal_pose, segments, route_uuid, allow_modification, stamp
    ):
        if not segments:
            raise RoutePlanningError("requested lanelet route is empty")
        route = LaneletRoute()
        route.header.stamp = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        route.header.frame_id = self.frame_id
        route.start_pose = self.current_start_pose()
        route.goal_pose = deepcopy(goal_pose)
        route.segments = deepcopy(list(segments))
        route.uuid = deepcopy(route_uuid)
        route.allow_modification = bool(allow_modification)
        return route, self.lane_ids_from_segments(route.segments), None

    def reroute_is_safe(self, target_preferred_ids):
        if self.current_route is None:
            return True
        if self.route_planner is None:
            return list(target_preferred_ids) == self.current_preferred_lane_ids
        distance = self.route_planner.route_change_distance(
            self.current_preferred_lane_ids,
            target_preferred_ids,
            self.odometry.pose.pose,
        )
        velocity = abs(float(self.odometry.twist.twist.linear.x))
        required = max(self.minimum_reroute_length, velocity * self.reroute_time_threshold)
        if distance < required:
            rospy.logwarn(
                "mission_planner: reroute rejected: change_distance=%.2f m, required=%.2f m",
                distance,
                required,
            )
            return False
        return True

    def commit_route(self, route, preferred_ids, plan, source, elapsed_ms):
        self.current_route = route
        self.current_preferred_lane_ids = list(preferred_ids)
        if plan is not None and self.route_planner is not None:
            self.route_planner.commit_route(plan)
        elif self.route_planner is not None:
            self.route_planner.commit_message_route(route.segments)
        self.route_pub.publish(route)
        self.route_state = RouteState.SET
        self.publish_state()
        self.marker_pub.publish(self.make_route_markers(route, preferred_ids))
        self.publish_goal_pose(route.goal_pose, route.header.stamp)
        self.publish_processing_time(elapsed_ms)
        rospy.loginfo(
            "mission_planner: route set from %s, segments=%d, preferred_lane_ids=%s",
            source,
            len(route.segments),
            preferred_ids,
        )

    def process_waypoint_route(
        self, goal_pose, waypoints, route_uuid, allow_modification, stamp, source
    ):
        tic = time.monotonic()
        is_reroute = self.route_state == RouteState.SET
        self.route_state = RouteState.REROUTING if is_reroute else RouteState.ROUTING
        self.publish_state()
        try:
            route, preferred_ids, plan = self.build_waypoint_route(
                goal_pose, waypoints, route_uuid, allow_modification, stamp
            )
            if is_reroute and self.is_autonomous_driving() and not self.reroute_is_safe(
                preferred_ids
            ):
                raise RoutePlanningError("new route is not safe for autonomous rerouting")
            self.commit_route(
                route, preferred_ids, plan, source, (time.monotonic() - tic) * 1000.0
            )
            return True, "waypoint route set"
        except Exception as error:
            self.route_state = RouteState.SET if is_reroute else RouteState.UNSET
            self.publish_state()
            self.publish_processing_time((time.monotonic() - tic) * 1000.0)
            rospy.logwarn("mission_planner: route planning failed: %s", error)
            return False, str(error)

    def process_lanelet_route(
        self, goal_pose, segments, route_uuid, allow_modification, stamp, source
    ):
        tic = time.monotonic()
        is_reroute = self.route_state == RouteState.SET
        self.route_state = RouteState.REROUTING if is_reroute else RouteState.ROUTING
        self.publish_state()
        try:
            route, preferred_ids, plan = self.build_lanelet_route(
                goal_pose, segments, route_uuid, allow_modification, stamp
            )
            if is_reroute and self.is_autonomous_driving() and not self.reroute_is_safe(
                preferred_ids
            ):
                raise RoutePlanningError("new route is not safe for autonomous rerouting")
            self.commit_route(
                route, preferred_ids, plan, source, (time.monotonic() - tic) * 1000.0
            )
            return True, "lanelet route set"
        except Exception as error:
            self.route_state = RouteState.SET if is_reroute else RouteState.UNSET
            self.publish_state()
            self.publish_processing_time((time.monotonic() - tic) * 1000.0)
            rospy.logwarn("mission_planner: lanelet route failed: %s", error)
            return False, str(error)

    def handle_clear_route(self, _request):
        with self.lock:
            if not self.is_ready():
                return ClearRouteResponse(
                    make_response(False, "mission planner is not ready", ResponseStatus.UNKNOWN)
                )
            self.current_route = None
            self.current_preferred_lane_ids = []
            if self.route_planner is not None:
                self.route_planner.clear_route()
            self.route_state = RouteState.UNSET
            self.publish_state()
            self.marker_pub.publish(self.make_clear_markers())
            return ClearRouteResponse(make_response(True, "route cleared"))

    def handle_set_lanelet_route(self, request):
        with self.lock:
            error = self.request_precondition_error()
            if error:
                return SetLaneletRouteResponse(
                    make_response(False, error, ResponseStatus.UNKNOWN)
                )
            try:
                goal = self.transform_pose(request.goal_pose, request.header)
            except Exception as transform_error:
                return SetLaneletRouteResponse(
                    make_response(False, transform_error, ResponseStatus.TRANSFORM_ERROR)
                )
            success, message = self.process_lanelet_route(
                goal,
                request.segments,
                request.uuid,
                request.allow_modification,
                request.header.stamp,
                "set_lanelet_route service",
            )
            return SetLaneletRouteResponse(
                make_response(success, message, 0 if success else ResponseStatus.UNKNOWN)
            )

    def handle_set_waypoint_route(self, request):
        with self.lock:
            error = self.request_precondition_error()
            if error:
                return SetWaypointRouteResponse(
                    make_response(False, error, ResponseStatus.UNKNOWN)
                )
            try:
                goal = self.transform_pose(request.goal_pose, request.header)
                waypoints = [
                    self.transform_pose(waypoint, request.header)
                    for waypoint in request.waypoints
                ]
            except Exception as transform_error:
                return SetWaypointRouteResponse(
                    make_response(False, transform_error, ResponseStatus.TRANSFORM_ERROR)
                )
            success, message = self.process_waypoint_route(
                goal,
                waypoints,
                request.uuid,
                request.allow_modification,
                request.header.stamp,
                "set_waypoint_route service",
            )
            return SetWaypointRouteResponse(
                make_response(success, message, 0 if success else ResponseStatus.UNKNOWN)
            )

    def modified_goal_callback(self, message):
        with self.lock:
            if (
                self.current_route is None
                or not self.current_route.allow_modification
                or uuid_values(message.uuid) != uuid_values(self.current_route.uuid)
                or not self.is_ready()
            ):
                return
            try:
                goal = self.transform_pose(message.pose, message.header)
            except Exception as error:
                rospy.logwarn("mission_planner: modified goal transform failed: %s", error)
                return
            success, detail = self.process_waypoint_route(
                goal,
                [],
                self.current_route.uuid,
                self.current_route.allow_modification,
                message.header.stamp,
                "modified goal",
            )
            if not success:
                rospy.logwarn("mission_planner: modified goal rejected: %s", detail)

    def publish_state(self):
        message = RouteState()
        message.stamp = rospy.Time.now()
        message.state = self.route_state
        self.state_pub.publish(message)

    def publish_goal_pose(self, pose, stamp):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose = deepcopy(pose)
        self.goal_pose_pub.publish(message)

    def publish_processing_time(self, value_ms):
        message = Float64Stamped()
        message.stamp = rospy.Time.now()
        message.data = float(value_ms)
        self.processing_time_pub.publish(message)

    def make_clear_markers(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def make_route_markers(self, route, preferred_ids):
        marker = Marker()
        marker.header.stamp = route.header.stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "mission_route"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        points = (
            self.route_planner.centerline_points(preferred_ids)
            if self.route_planner is not None
            else []
        )
        if len(points) < 2:
            points = [pose_xy(route.start_pose), pose_xy(route.goal_pose)]
        marker.points = [Point(x=point[0], y=point[1], z=point[2]) for point in points]
        return MarkerArray(markers=[marker])

    def update_arrival_state(self):
        if self.route_state != RouteState.SET or self.current_route is None or self.odometry is None:
            return
        ego_pose = self.odometry.pose.pose
        goal_pose = self.current_route.goal_pose
        if distance_xy(pose_xy(ego_pose), pose_xy(goal_pose)) > self.arrival_check_distance:
            return
        yaw_difference = abs(
            normalize_angle(
                yaw_from_quaternion(ego_pose.orientation)
                - yaw_from_quaternion(goal_pose.orientation)
            )
        )
        if yaw_difference > self.arrival_check_angle:
            return
        if self.stopped_since is None:
            return
        if (rospy.Time.now() - self.stopped_since).to_sec() < self.arrival_check_duration:
            return
        self.route_state = RouteState.ARRIVED
        self.publish_state()

    def on_timer(self, _event):
        with self.lock:
            self.update_ready_state()
            self.update_arrival_state()
            self.publish_state()


def main():
    rospy.init_node("mission_planner")
    MissionPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
