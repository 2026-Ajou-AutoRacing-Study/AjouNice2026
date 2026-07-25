#!/usr/bin/env python3

import json
import math
import os
import sys
from copy import deepcopy

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import (
    PathPointWithLaneId,
    PathWithLaneId,
    Scenario,
    VelocityLimit,
)
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import (
    LaneletRoute,
    Path as AutowarePath,
    PathPoint,
    PoseWithUuidStamped,
)
from autoware_vehicle_msgs.msg import HazardLightsCommand, TurnIndicatorsCommand
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray, LateralOffset, RerouteAvailability
from tier4_rtc_msgs.msg import Module, State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_lane_change_tools import build_route_lane_change_path, route_lane_change_target
from bpp_dynamic_avoidance_tools import DynamicAvoidanceScene
from bpp_goal_planner_tools import GoalPlannerScene
from bpp_map_tools import (
    CenterlineMap,
    closest_lane_id,
    resample_polyline,
    stitch_route_bounds,
    stitch_route_centerlines,
    trim_polyline,
)
from bpp_path_tools import (
    apply_s_curve_shift,
    make_avoidance_debug_array,
    object_pose,
    object_width,
    project_pose_to_path,
    to_debug_path,
    to_autoware_path,
)
from bpp_route_tools import forward_lane_sequence, reference_lane_sequence
from bpp_side_shift_tools import SideShiftScene
from bpp_start_planner_tools import StartPlannerScene
from bpp_static_avoidance_tools import StaticAvoidanceScene
from rtc_interface_ros1 import RTCInterfaceROS1, RTCSceneController


def param_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def distance_2d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def is_pose_position_set(pose):
    p = pose.position
    return abs(p.x) > 1.0e-6 or abs(p.y) > 1.0e-6 or abs(p.z) > 1.0e-6


def pose_to_point3(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


class BehaviorPathPlanner:
    def __init__(self):
        self.route = None
        self.odometry = None
        self.scenario = None
        self.vector_map = None
        self.velocity_limit = None
        self.objects = None
        self.operation_mode = None
        self.lane_change_request = None
        self.lane_change_request_time = None
        self.side_shift_request = None
        self.side_shift_request_time = None
        self.centerline_map = None
        self.static_avoidance_scene = None
        self.dynamic_avoidance_scene = None
        self.start_planner_scene = None
        self.goal_planner_scene = None
        self.side_shift_scene = None

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        self.path_interval = max(float(rospy.get_param("~path_interval", 2.0)), 0.1)
        self.backward_path_length = max(
            float(rospy.get_param("~backward_path_length", 5.0)), 0.0
        )
        self.vehicle_rear_overhang = max(
            float(rospy.get_param("~vehicle_rear_overhang", 1.1)), 0.0
        )
        self.backward_path_margin = max(
            float(rospy.get_param("~backward_path_margin", 1.0)), 0.0
        )
        self.forward_path_length = max(
            float(rospy.get_param("~forward_path_length", 300.0)), 1.0
        )
        self.goal_bound_extension = max(
            float(rospy.get_param("~goal_bound_extension", 5.0)), 0.0
        )
        self.fallback_path_length = max(float(rospy.get_param("~fallback_path_length", 60.0)), 1.0)
        self.default_velocity_mps = max(float(rospy.get_param("~default_velocity_mps", 5.0)), 0.0)
        self.stop_at_goal = param_bool(rospy.get_param("~stop_at_goal", True))
        self.use_ego_as_path_start = param_bool(rospy.get_param("~use_ego_as_path_start", True))
        self.require_lane_driving_scenario = param_bool(
            rospy.get_param("~require_lane_driving_scenario", False)
        )
        self.use_map_centerline = param_bool(rospy.get_param("~use_map_centerline", True))
        self.map_file = rospy.get_param("~map_file", rospy.get_param("~centerline_map_file", ""))
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.flatten_map_z = param_bool(rospy.get_param("~flatten_map_z", False))

        self.enable_lane_change = param_bool(rospy.get_param("~enable_lane_change", True))
        self.enable_static_obstacle_avoidance = param_bool(
            rospy.get_param("~enable_static_obstacle_avoidance", True)
        )
        self.enable_dynamic_obstacle_avoidance = param_bool(
            rospy.get_param("~enable_dynamic_obstacle_avoidance", True)
        )
        self.enable_start_planner = param_bool(rospy.get_param("~enable_start_planner", True))
        self.enable_goal_planner = param_bool(rospy.get_param("~enable_goal_planner", False))
        self.enable_side_shift = param_bool(rospy.get_param("~enable_side_shift", False))

        self.load_module_params()
        self.create_rtc_interfaces()

        self.subscribe_inputs()
        self.create_publishers()
        self.load_centerline_map()
        self.static_avoidance_scene = StaticAvoidanceScene(
            "~static_obstacle_avoidance", self.centerline_map
        )
        self.dynamic_avoidance_scene = DynamicAvoidanceScene(
            "~dynamic_obstacle_avoidance"
        )
        self.start_planner_scene = StartPlannerScene("~start_planner")
        self.goal_planner_scene = GoalPlannerScene("~goal_planner")
        self.side_shift_scene = SideShiftScene("~side_shift")
        if self.side_shift_request is not None:
            self.side_shift_scene.update_request(
                self.side_shift_request.lateral_offset,
                self.side_shift_request.stamp,
            )

        period = rospy.Duration(1.0 / max(self.planning_hz, 0.1))
        self.timer = rospy.Timer(period, self.on_timer)

    def load_module_params(self):
        self.lane_change_lane_width = float(rospy.get_param("~lane_change/lane_width", 3.5))
        self.lane_change_direction = rospy.get_param("~lane_change/direction", "none").lower()
        self.lane_change_target_lateral_offset = float(
            rospy.get_param("~lane_change/target_lateral_offset", 0.0)
        )
        self.lane_change_min_request_offset = float(
            rospy.get_param("~lane_change/min_request_offset", 0.1)
        )
        self.lane_change_request_timeout = float(rospy.get_param("~lane_change/request_timeout", 0.0))
        self.lane_change_prepare_distance = float(
            rospy.get_param("~lane_change/prepare_distance", 3.0)
        )
        self.lane_change_distance = float(rospy.get_param("~lane_change/lane_change_distance", 28.0))
        self.lane_change_target_forward_length = float(
            rospy.get_param("~lane_change/target_forward_length", 300.0)
        )
        self.lane_change_transition_interval = float(
            rospy.get_param("~lane_change/transition_interval", 1.0)
        )
        self.lane_change_target_interval = float(
            rospy.get_param("~lane_change/target_interval", 4.0)
        )
        self.lane_change_prepare_velocity_ratio = float(
            rospy.get_param("~lane_change/prepare_velocity_ratio", 2.0 / 3.0)
        )
        self.lane_change_enable_safety_check = param_bool(
            rospy.get_param("~lane_change/enable_safety_check", True)
        )
        self.lane_change_safety_lateral_margin = float(
            rospy.get_param("~lane_change/safety_lateral_margin", 1.2)
        )
        self.lane_change_safety_forward_margin = float(
            rospy.get_param("~lane_change/safety_forward_margin", 10.0)
        )
        self.lane_change_enable_rtc_left = param_bool(
            rospy.get_param("~lane_change/rtc/enable_left", False)
        )
        self.lane_change_enable_rtc_right = param_bool(
            rospy.get_param("~lane_change/rtc/enable_right", False)
        )

    def create_rtc_interfaces(self):
        self.lane_change_rtc_interfaces = {
            "left": RTCInterfaceROS1(
                Module.LANE_CHANGE_LEFT,
                rospy.get_param(
                    "~lane_change/rtc/left/status_topic",
                    "/planning/cooperate_status/lane_change_left",
                ),
                rospy.get_param(
                    "~lane_change/rtc/left/command_service",
                    "/planning/cooperate_commands/lane_change_left",
                ),
                rospy.get_param(
                    "~lane_change/rtc/left/auto_mode_service",
                    "/planning/enable_auto_mode/lane_change_left",
                ),
                self.lane_change_enable_rtc_left,
            ),
            "right": RTCInterfaceROS1(
                Module.LANE_CHANGE_RIGHT,
                rospy.get_param(
                    "~lane_change/rtc/right/status_topic",
                    "/planning/cooperate_status/lane_change_right",
                ),
                rospy.get_param(
                    "~lane_change/rtc/right/command_service",
                    "/planning/cooperate_commands/lane_change_right",
                ),
                rospy.get_param(
                    "~lane_change/rtc/right/auto_mode_service",
                    "/planning/enable_auto_mode/lane_change_right",
                ),
                self.lane_change_enable_rtc_right,
            ),
        }
        self.lane_change_rtc_controller = RTCSceneController(
            self.lane_change_rtc_interfaces
        )

    def subscribe_inputs(self):
        route_topic = rospy.get_param("~route_topic", "~input/route")
        odometry_topic = rospy.get_param("~odometry_topic", "~input/odometry")
        scenario_topic = rospy.get_param("~scenario_topic", "~input/scenario")
        vector_map_topic = rospy.get_param("~vector_map_topic", "~input/vector_map")
        objects_topic = rospy.get_param("~objects_topic", "~input/objects")
        operation_mode_topic = rospy.get_param("~operation_mode_topic", "~input/operation_mode")
        lane_change_offset_topic = rospy.get_param(
            "~lane_change_lateral_offset_topic", "~input/lane_change_lateral_offset"
        )
        side_shift_offset_topic = rospy.get_param(
            "~side_shift_lateral_offset_topic", "~input/side_shift_lateral_offset"
        )
        max_velocity_topic = rospy.get_param(
            "~max_velocity_topic", "/planning/scenario_planning/max_velocity"
        )

        rospy.Subscriber(route_topic, LaneletRoute, self.route_callback, queue_size=1)
        rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber(scenario_topic, Scenario, self.scenario_callback, queue_size=1)
        rospy.Subscriber(vector_map_topic, LaneletMapBin, self.vector_map_callback, queue_size=1)
        rospy.Subscriber(objects_topic, PredictedObjects, self.objects_callback, queue_size=1)
        rospy.Subscriber(
            operation_mode_topic, OperationModeState, self.operation_mode_callback, queue_size=1
        )
        rospy.Subscriber(
            lane_change_offset_topic, LateralOffset, self.lane_change_offset_callback, queue_size=1
        )
        rospy.Subscriber(
            side_shift_offset_topic, LateralOffset, self.side_shift_offset_callback, queue_size=1
        )
        if max_velocity_topic:
            rospy.Subscriber(max_velocity_topic, VelocityLimit, self.velocity_limit_callback, queue_size=1)

    def create_publishers(self):
        self.path_pub = rospy.Publisher("~output/path", PathWithLaneId, queue_size=1)
        self.turn_pub = rospy.Publisher(
            "~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1
        )
        self.hazard_pub = rospy.Publisher(
            "~output/hazard_lights_cmd", HazardLightsCommand, queue_size=1
        )
        self.modified_goal_pub = rospy.Publisher(
            "~output/modified_goal", PoseWithUuidStamped, queue_size=1
        )
        self.reroute_pub = rospy.Publisher(
            "~output/is_reroute_available", RerouteAvailability, queue_size=1
        )
        self.status_pub = rospy.Publisher("~debug/module_status", String, queue_size=1)
        self.lane_change_candidate_pubs = {
            "left": rospy.Publisher(
                "~output/lane_change_candidate_path_left", AutowarePath, queue_size=1
            ),
            "right": rospy.Publisher(
                "~output/lane_change_candidate_path_right", AutowarePath, queue_size=1
            ),
        }
        self.lane_change_reference_pubs = {
            "left": rospy.Publisher(
                "~output/lane_change_reference_path_left", AutowarePath, queue_size=1
            ),
            "right": rospy.Publisher(
                "~output/lane_change_reference_path_right", AutowarePath, queue_size=1
            ),
        }
        self.static_avoidance_candidate_pub = rospy.Publisher(
            "~output/static_obstacle_avoidance_candidate_path",
            AutowarePath,
            queue_size=1,
        )
        self.static_avoidance_reference_pub = rospy.Publisher(
            "~output/static_obstacle_avoidance_reference_path",
            AutowarePath,
            queue_size=1,
        )
        self.static_avoidance_debug_pub = rospy.Publisher(
            "~debug/static_obstacle_avoidance", AvoidanceDebugMsgArray, queue_size=1
        )
        self.dynamic_avoidance_debug_pub = rospy.Publisher(
            "~debug/dynamic_obstacle_avoidance", AvoidanceDebugMsgArray, queue_size=1
        )
        self.start_planner_reference_pub = rospy.Publisher(
            "~output/start_planner_reference_path", AutowarePath, queue_size=1
        )
        self.start_planner_candidate_pub = rospy.Publisher(
            "~output/start_planner_candidate_path", AutowarePath, queue_size=1
        )
        self.start_planner_debug_pub = rospy.Publisher(
            "~debug/start_planner_safety", AvoidanceDebugMsgArray, queue_size=1
        )
        self.goal_planner_reference_pub = rospy.Publisher(
            "~output/goal_planner_reference_path", AutowarePath, queue_size=1
        )
        self.side_shift_reference_pub = rospy.Publisher(
            "~output/side_shift_reference_path", AutowarePath, queue_size=1
        )
        self.side_shift_candidate_pub = rospy.Publisher(
            "~output/side_shift_candidate_path", AutowarePath, queue_size=1
        )

        self.debug_path_pub = rospy.Publisher("~debug/path", Path, queue_size=1)
        self.debug_stage_pubs = {
            "lane_following": rospy.Publisher("~debug/lane_following_path", Path, queue_size=1),
            "lane_change": rospy.Publisher("~debug/lane_change_path", Path, queue_size=1),
            "static_obstacle_avoidance": rospy.Publisher(
                "~debug/static_obstacle_avoidance_path", Path, queue_size=1
            ),
            "dynamic_obstacle_avoidance": rospy.Publisher(
                "~debug/dynamic_obstacle_avoidance_path", Path, queue_size=1
            ),
            "start_planner": rospy.Publisher("~debug/start_planner_path", Path, queue_size=1),
            "goal_planner": rospy.Publisher("~debug/goal_planner_path", Path, queue_size=1),
            "side_shift": rospy.Publisher("~debug/side_shift_path", Path, queue_size=1),
        }

    def load_centerline_map(self):
        if not self.map_file:
            return
        try:
            self.centerline_map = CenterlineMap.load(
                self.map_file,
                self.projector_file,
                self.flatten_map_z,
                self.projector_type,
            )
            rospy.loginfo(
                "behavior_path_planner: loaded %d centerlines from %s",
                self.centerline_map.lane_count(),
                self.centerline_map.source,
            )
        except Exception as exc:
            self.centerline_map = None
            rospy.logwarn("behavior_path_planner: failed to load centerline map: %s", exc)

    def route_callback(self, msg):
        self.route = msg

    def odometry_callback(self, msg):
        self.odometry = msg

    def scenario_callback(self, msg):
        self.scenario = msg

    def vector_map_callback(self, msg):
        self.vector_map = msg
        if self.centerline_map is not None or not self.use_map_centerline:
            return
        if not msg.data:
            return

        raw = bytes(msg.data)
        if not raw.lstrip().startswith(b"<osm"):
            rospy.logwarn_throttle(
                10.0,
                "behavior_path_planner: LaneletMapBin binary parsing is not available; pass ~map_file for centerline generation",
            )
            return

        try:
            self.centerline_map = CenterlineMap.from_osm_bytes(
                raw, "LaneletMapBin", self.flatten_map_z, self.projector_type
            )
            if self.static_avoidance_scene is not None:
                self.static_avoidance_scene.set_centerline_map(self.centerline_map)
            rospy.loginfo(
                "behavior_path_planner: loaded %d centerlines from LaneletMapBin OSM payload",
                self.centerline_map.lane_count(),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0, "behavior_path_planner: failed to parse LaneletMapBin OSM payload: %s", exc
            )

    def velocity_limit_callback(self, msg):
        self.velocity_limit = msg

    def objects_callback(self, msg):
        self.objects = msg

    def operation_mode_callback(self, msg):
        self.operation_mode = msg

    def lane_change_offset_callback(self, msg):
        self.lane_change_request = msg
        self.lane_change_request_time = rospy.Time.now()

    def side_shift_offset_callback(self, msg):
        self.side_shift_request = msg
        self.side_shift_request_time = rospy.Time.now()
        if self.side_shift_scene is not None:
            self.side_shift_scene.update_request(msg.lateral_offset, msg.stamp)

    def on_timer(self, _event):
        if rospy.is_shutdown():
            return
        if self.route is None:
            rospy.logwarn_throttle(5.0, "behavior_path_planner: waiting for LaneletRoute")
            return
        if not self.is_lane_driving_enabled():
            rospy.logwarn_throttle(5.0, "behavior_path_planner: scenario is not LaneDriving")
            return

        stamp = rospy.Time.now()
        statuses = []
        turn_command = TurnIndicatorsCommand.NO_COMMAND

        path, status = self.build_lane_following_path(stamp)
        statuses.append(status)
        self.publish_debug_stage("lane_following", path)

        if self.enable_lane_change:
            path, status, command = self.apply_lane_change(path, stamp)
            statuses.append(status)
            turn_command = self.prefer_command(turn_command, command)
        else:
            self.lane_change_rtc_controller.complete(stamp)
            statuses.append("lane_change: disabled")
        self.publish_debug_stage("lane_change", path)

        if self.enable_static_obstacle_avoidance:
            path, status, command = self.apply_static_obstacle_avoidance(path, stamp)
            statuses.append(status)
            turn_command = self.prefer_command(turn_command, command)
        else:
            self.static_avoidance_scene.rtc_controller.complete(stamp)
            self.static_avoidance_scene.return_rtc_controller.complete(stamp)
            self.publish_static_debug(path, [])
            statuses.append("static_obstacle_avoidance: disabled")
        self.publish_debug_stage("static_obstacle_avoidance", path)

        if self.enable_dynamic_obstacle_avoidance:
            path, status = self.apply_dynamic_obstacle_avoidance(path, stamp)
            statuses.append(status)
        else:
            self.publish_dynamic_debug(path, [])
            statuses.append("dynamic_obstacle_avoidance: disabled")
        self.publish_debug_stage("dynamic_obstacle_avoidance", path)

        if self.enable_start_planner:
            path, status, command = self.apply_start_planner(path, stamp)
            statuses.append(status)
            turn_command = self.prefer_command(turn_command, command)
        else:
            self.start_planner_scene.rtc_controller.complete(stamp)
            statuses.append("start_planner: disabled")
        self.publish_debug_stage("start_planner", path)

        if self.enable_goal_planner:
            path, status = self.apply_goal_planner(path, stamp)
            statuses.append(status)
        else:
            self.publish_goal_side_outputs(path, stamp)
            statuses.append("goal_planner: disabled")
        self.publish_debug_stage("goal_planner", path)

        if self.enable_side_shift:
            path, status, command = self.apply_side_shift(path)
            statuses.append(status)
            turn_command = self.prefer_command(turn_command, command)
        else:
            statuses.append("side_shift: disabled")
        self.publish_debug_stage("side_shift", path)

        path.header.stamp = stamp
        if not path.header.frame_id:
            path.header.frame_id = self.frame_id
        self.path_pub.publish(path)
        self.debug_path_pub.publish(to_debug_path(path))
        self.publish_turn_command(stamp, turn_command)
        self.publish_hazard_command(stamp, HazardLightsCommand.NO_COMMAND)
        self.publish_status(statuses)

    def is_lane_driving_enabled(self):
        if not self.require_lane_driving_scenario:
            return True
        if self.scenario is None:
            return False
        return self.scenario.current_scenario in ("", Scenario.LANEDRIVING)

    def build_lane_following_path(self, stamp):
        lane_ids = self.extract_lane_ids(self.route)
        start_pose = self.select_start_pose(self.route)
        goal_pose = self.route.goal_pose

        points = self.build_centerline_path_points(start_pose, goal_pose, lane_ids)
        status = "lane_following: map centerline"
        if points is None:
            points = self.interpolate_path_points(start_pose, goal_pose, lane_ids)
            status = "lane_following: fallback interpolation"

        msg = PathWithLaneId()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.points = points
        msg.left_bound, msg.right_bound = self.build_drivable_bounds(
            points, lane_ids, goal_pose
        )
        return msg, status

    def build_drivable_bounds(self, path_points, lane_ids, goal_pose):
        if self.centerline_map is None or not path_points or not lane_ids:
            return [], []
        start = path_points[0].point.pose.position
        start_point = (start.x, start.y, start.z)
        goal_point = pose_to_point3(goal_pose) if is_pose_position_set(goal_pose) else None
        left, right = stitch_route_bounds(
            self.centerline_map, lane_ids, start_point=start_point
        )

        output = []
        for boundary in (left, right):
            if len(boundary) < 2:
                output.append([])
                continue
            boundary = trim_polyline(
                boundary,
                start_point=start_point,
                goal_point=goal_point,
                end_offset=self.goal_bound_extension,
                max_forward_length=(
                    self.forward_path_length
                    + self.backward_path_length
                    + self.vehicle_rear_overhang
                    + self.backward_path_margin
                ),
            )
            boundary = resample_polyline(boundary, self.path_interval)
            output.append([Point(x=x, y=y, z=z) for x, y, z in boundary])
        return output[0], output[1]

    def extract_lane_ids(self, route):
        if self.centerline_map is not None and self.odometry is not None:
            lane_ids = reference_lane_sequence(
                route, self.centerline_map, self.odometry.pose.pose
            )
            if lane_ids:
                return lane_ids

        selected = []
        for index, segment in enumerate(route.segments):
            primitive_ids = [
                int(primitive.id)
                for primitive in segment.primitives
                if int(primitive.id) != 0
            ]
            preferred_id = int(segment.preferred_primitive.id)
            if preferred_id and preferred_id not in primitive_ids:
                primitive_ids.append(preferred_id)
            if not primitive_ids:
                continue

            lane_id = preferred_id or primitive_ids[0]
            if (
                index == 0
                and self.centerline_map is not None
                and self.odometry is not None
            ):
                nearest = closest_lane_id(
                    self.centerline_map,
                    primitive_ids,
                    pose_to_point3(self.odometry.pose.pose),
                )
                if nearest is not None:
                    lane_id = nearest
            if lane_id not in selected:
                selected.append(lane_id)
        return selected

    def select_start_pose(self, route):
        if self.use_ego_as_path_start and self.odometry is not None:
            return deepcopy(self.odometry.pose.pose)
        if is_pose_position_set(route.start_pose):
            return deepcopy(route.start_pose)
        return deepcopy(route.goal_pose)

    def build_centerline_path_points(self, start_pose, goal_pose, lane_ids):
        if not self.use_map_centerline or self.centerline_map is None or not lane_ids:
            return None

        start_point = pose_to_point3(start_pose) if is_pose_position_set(start_pose) else None
        goal_point = pose_to_point3(goal_pose) if is_pose_position_set(goal_pose) else None

        centerline, used_lane_ids = stitch_route_centerlines(
            self.centerline_map, lane_ids, start_point=start_point
        )
        if len(centerline) < 2:
            rospy.logwarn_throttle(
                5.0,
                "behavior_path_planner: no map centerline matched route lane ids; falling back to straight path",
            )
            return None

        centerline = resample_polyline(centerline, self.path_interval)
        effective_backward_length = (
            self.backward_path_length
            + self.vehicle_rear_overhang
            + self.backward_path_margin
        )
        centerline = trim_polyline(
            centerline,
            start_point=start_point,
            goal_point=goal_point,
            start_offset=-effective_backward_length,
            include_previous_start=len(lane_ids) > 1,
            max_forward_length=self.forward_path_length,
        )
        centerline = resample_polyline(centerline, self.path_interval)
        if len(centerline) < 2:
            return None

        reaches_goal = goal_point is not None and math.hypot(
            centerline[-1][0] - goal_point[0], centerline[-1][1] - goal_point[1]
        ) <= max(self.path_interval, 0.1)
        if reaches_goal and math.hypot(
            centerline[-1][0] - goal_point[0], centerline[-1][1] - goal_point[1]
        ) > 1.0e-3:
            centerline.append(goal_point)

        return self.centerline_points_to_path_points(
            centerline, used_lane_ids or lane_ids, reaches_goal
        )

    def centerline_points_to_path_points(self, centerline, lane_ids, reaches_goal=True):
        velocity = self.current_target_velocity()
        path_points = []
        for idx, point in enumerate(centerline):
            if idx < len(centerline) - 1:
                next_point = centerline[idx + 1]
                yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            else:
                prev_point = centerline[idx - 1]
                yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])

            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            pose.orientation = quaternion_from_yaw(yaw)

            path_point = PathPoint()
            path_point.pose = pose
            path_point.longitudinal_velocity_mps = velocity
            path_point.lateral_velocity_mps = 0.0
            path_point.heading_rate_rps = 0.0
            is_last = idx == len(centerline) - 1
            path_point.is_final = False
            if self.stop_at_goal and reaches_goal and is_last:
                path_point.longitudinal_velocity_mps = 0.0

            point_with_lane_id = PathPointWithLaneId()
            point_with_lane_id.point = path_point
            point_with_lane_id.lane_ids = self.centerline_map.lane_ids_near_point(
                lane_ids, point
            )
            path_points.append(point_with_lane_id)

        return path_points

    def interpolate_path_points(self, start_pose, goal_pose, lane_ids):
        start = start_pose.position
        goal = goal_pose.position
        route_length = distance_2d(start, goal) if is_pose_position_set(goal_pose) else 0.0

        if route_length > 1.0e-3:
            yaw = math.atan2(goal.y - start.y, goal.x - start.x)
            point_count = max(2, int(math.ceil(route_length / self.path_interval)) + 1)
            poses = []
            for idx in range(point_count):
                ratio = float(idx) / float(point_count - 1)
                pose = Pose()
                pose.position.x = start.x + (goal.x - start.x) * ratio
                pose.position.y = start.y + (goal.y - start.y) * ratio
                pose.position.z = start.z + (goal.z - start.z) * ratio
                pose.orientation = quaternion_from_yaw(yaw)
                poses.append(pose)
        else:
            yaw = yaw_from_quaternion(start_pose.orientation)
            point_count = max(2, int(math.ceil(self.fallback_path_length / self.path_interval)) + 1)
            poses = []
            for idx in range(point_count):
                distance = self.path_interval * idx
                pose = Pose()
                pose.position.x = start.x + distance * math.cos(yaw)
                pose.position.y = start.y + distance * math.sin(yaw)
                pose.position.z = start.z
                pose.orientation = quaternion_from_yaw(yaw)
                poses.append(pose)

        velocity = self.current_target_velocity()
        path_points = []
        for idx, pose in enumerate(poses):
            path_point = PathPoint()
            path_point.pose = pose
            path_point.longitudinal_velocity_mps = velocity
            path_point.lateral_velocity_mps = 0.0
            path_point.heading_rate_rps = 0.0
            is_last = idx == len(poses) - 1
            path_point.is_final = False
            if self.stop_at_goal and is_last:
                path_point.longitudinal_velocity_mps = 0.0

            point_with_lane_id = PathPointWithLaneId()
            point_with_lane_id.point = path_point
            point_with_lane_id.lane_ids = list(lane_ids)
            path_points.append(point_with_lane_id)

        return path_points

    def current_target_velocity(self):
        if self.velocity_limit is not None and self.velocity_limit.max_velocity > 0.0:
            return min(self.default_velocity_mps, self.velocity_limit.max_velocity)
        return self.default_velocity_mps

    def requested_lane_change_offset(self):
        if self.lane_change_request is not None:
            if self.lane_change_request_timeout <= 0.0:
                return self.lane_change_request.lateral_offset
            age = (rospy.Time.now() - self.lane_change_request_time).to_sec()
            if age <= self.lane_change_request_timeout:
                return self.lane_change_request.lateral_offset

        if abs(self.lane_change_target_lateral_offset) >= self.lane_change_min_request_offset:
            return self.lane_change_target_lateral_offset
        if self.lane_change_direction == "left":
            return abs(self.lane_change_lane_width)
        if self.lane_change_direction == "right":
            return -abs(self.lane_change_lane_width)
        return 0.0

    def apply_lane_change_rtc_gate(
        self,
        reference,
        candidate,
        safe,
        target_offset,
        signature,
        stamp,
        description,
    ):
        direction = "left" if target_offset > 0.0 else "right"
        activated, state, _, auto_mode = self.lane_change_rtc_controller.evaluate(
            direction,
            signature,
            safe,
            self.lane_change_prepare_distance,
            self.lane_change_prepare_distance + self.lane_change_distance,
            stamp,
        )
        reference.header.stamp = stamp
        candidate.header.stamp = stamp
        self.lane_change_reference_pubs[direction].publish(to_autoware_path(reference))
        self.lane_change_candidate_pubs[direction].publish(to_autoware_path(candidate))

        state_name = {
            State.WAITING_FOR_EXECUTION: "waiting",
            State.RUNNING: "running",
            State.ABORTING: "aborting",
            State.SUCCEEDED: "succeeded",
            State.FAILED: "failed",
        }.get(state, "unknown")
        mode_name = "auto" if auto_mode else "manual"
        safety_name = "safe" if safe else "unsafe"
        if not safe:
            description = "lane_change: blocked by object in target lane; " + description
        status = "{}; rtc={} {} {}, activated={}".format(
            description,
            mode_name,
            safety_name,
            state_name,
            activated,
        )
        return (
            candidate if activated else reference,
            status,
            self.turn_command_for_offset(target_offset),
        )

    def apply_lane_change(self, path, stamp):
        route_target = None
        if self.odometry is not None:
            route_target = route_lane_change_target(
                self.route,
                self.centerline_map,
                self.odometry.pose.pose,
            )
        if route_target is not None:
            target_offset = route_target.lateral_offset
            target_lane_ids = forward_lane_sequence(
                self.route,
                self.centerline_map,
                route_target.section_index,
                route_target.target_lane_id,
            )
            candidate = build_route_lane_change_path(
                path,
                self.centerline_map,
                self.odometry.pose.pose,
                route_target,
                self.lane_change_prepare_distance,
                self.lane_change_distance,
                self.lane_change_target_forward_length,
                self.lane_change_transition_interval,
                self.lane_change_target_interval,
                self.lane_change_prepare_velocity_ratio,
                target_lane_ids,
                pose_to_point3(self.route.goal_pose),
                self.stop_at_goal,
            )
            safe = self.is_lane_change_safe(path, target_offset)
            return self.apply_lane_change_rtc_gate(
                path,
                candidate,
                safe,
                target_offset,
                (
                    "route",
                    int(route_target.source_lane_id),
                    int(route_target.target_lane_id),
                    str(route_target.direction),
                ),
                stamp,
                "lane_change: route {} -> {} ({})".format(
                    route_target.source_lane_id,
                    route_target.target_lane_id,
                    route_target.direction,
                ),
            )

        target_offset = self.requested_lane_change_offset()
        if abs(target_offset) < self.lane_change_min_request_offset:
            self.lane_change_rtc_controller.complete(stamp)
            return path, "lane_change: pass-through, no request", TurnIndicatorsCommand.NO_COMMAND

        candidate = apply_s_curve_shift(
            path,
            target_offset,
            self.lane_change_prepare_distance,
            self.lane_change_prepare_distance + self.lane_change_distance,
        )
        safe = self.is_lane_change_safe(path, target_offset)
        return self.apply_lane_change_rtc_gate(
            path,
            candidate,
            safe,
            target_offset,
            ("offset", round(float(target_offset), 3)),
            stamp,
            "lane_change: applied lateral offset {:.2f} m".format(target_offset),
        )

    def is_lane_change_safe(self, path, target_offset):
        if not self.lane_change_enable_safety_check or self.objects is None:
            return True

        check_end = (
            self.lane_change_prepare_distance
            + self.lane_change_distance
            + self.lane_change_safety_forward_margin
        )
        for obj in self.objects.objects:
            projection = project_pose_to_path(path, object_pose(obj))
            if projection is None:
                continue
            if projection.longitudinal_distance < 0.0 or projection.longitudinal_distance > check_end:
                continue
            target_lane_error = abs(projection.signed_lateral_distance - target_offset)
            if target_lane_error <= self.lane_change_safety_lateral_margin:
                return False
        return True

    def apply_static_obstacle_avoidance(self, path, stamp):
        result = self.static_avoidance_scene.process(
            path, self.objects, self.odometry, stamp
        )
        self.publish_static_debug(
            path, [candidate.debug_dict() for candidate in result.candidates]
        )

        result.reference_path.header.stamp = stamp
        result.candidate_path.header.stamp = stamp
        self.static_avoidance_reference_pub.publish(
            to_autoware_path(result.reference_path)
        )
        self.static_avoidance_candidate_pub.publish(
            to_autoware_path(result.candidate_path)
        )

        command = TurnIndicatorsCommand.DISABLE
        if self.static_avoidance_scene.engine.turn_signal_required(result):
            command = (
                TurnIndicatorsCommand.ENABLE_LEFT
                if result.direction == "left"
                else TurnIndicatorsCommand.ENABLE_RIGHT
            )
        return result.output_path, result.status, command

    def apply_dynamic_obstacle_avoidance(self, path, stamp):
        result = self.dynamic_avoidance_scene.process(
            path, self.objects, self.odometry, stamp
        )
        self.publish_dynamic_debug(
            path, [candidate.debug_dict() for candidate in result.candidates]
        )
        return result.output_path, result.status

    def apply_start_planner(self, path, stamp):
        result = self.start_planner_scene.process(
            path, self.objects, self.odometry, stamp
        )
        self.start_planner_reference_pub.publish(to_autoware_path(result.reference_path))
        self.start_planner_candidate_pub.publish(to_autoware_path(result.candidate_path))
        self.start_planner_debug_pub.publish(
            make_avoidance_debug_array(
                path.header, [candidate.debug_dict() for candidate in result.candidates]
            )
        )
        return (
            result.output_path,
            result.status,
            self.start_planner_scene.turn_command(result),
        )

    def apply_goal_planner(self, path, stamp):
        result = self.goal_planner_scene.process(path, self.route)
        inactive_reference = AutowarePath()
        inactive_reference.header.stamp = stamp
        inactive_reference.header.frame_id = path.header.frame_id or self.frame_id
        self.goal_planner_reference_pub.publish(inactive_reference)
        self.publish_goal_side_outputs(result.output_path, stamp, result.modified_goal_pose)
        return result.output_path, result.status

    def apply_side_shift(self, path):
        result = self.side_shift_scene.process(path, self.odometry)
        self.side_shift_reference_pub.publish(to_autoware_path(result.reference_path))
        self.side_shift_candidate_pub.publish(to_autoware_path(result.candidate_path))
        return (
            result.output_path,
            result.status,
            self.side_shift_scene.turn_command(result),
        )

    def publish_goal_side_outputs(self, path, stamp, modified_goal_pose=None):
        if modified_goal_pose is not None:
            modified_goal = PoseWithUuidStamped()
            modified_goal.header.stamp = stamp
            modified_goal.header.frame_id = path.header.frame_id or self.frame_id
            modified_goal.pose = modified_goal_pose
            if self.route is not None:
                modified_goal.uuid = self.route.uuid
            self.modified_goal_pub.publish(modified_goal)

        reroute = RerouteAvailability()
        reroute.stamp = stamp
        reroute.availability = self.goal_planner_scene.engine.parameters.allow_goal_modification
        self.reroute_pub.publish(reroute)

    def publish_static_debug(self, path, candidates):
        self.static_avoidance_debug_pub.publish(make_avoidance_debug_array(path.header, candidates))

    def publish_dynamic_debug(self, path, candidates):
        self.dynamic_avoidance_debug_pub.publish(make_avoidance_debug_array(path.header, candidates))

    def publish_debug_stage(self, name, path):
        self.debug_stage_pubs[name].publish(to_debug_path(path))

    def publish_turn_command(self, stamp, command):
        msg = TurnIndicatorsCommand()
        msg.stamp = stamp
        msg.command = command
        self.turn_pub.publish(msg)

    def publish_hazard_command(self, stamp, command):
        msg = HazardLightsCommand()
        msg.stamp = stamp
        msg.command = command
        self.hazard_pub.publish(msg)

    def publish_status(self, statuses):
        msg = String()
        msg.data = json.dumps(statuses, ensure_ascii=True)
        self.status_pub.publish(msg)

    def prefer_command(self, current, candidate):
        active_commands = (
            TurnIndicatorsCommand.ENABLE_LEFT,
            TurnIndicatorsCommand.ENABLE_RIGHT,
        )
        if candidate in active_commands:
            return candidate
        if current in active_commands:
            return current
        if candidate == TurnIndicatorsCommand.DISABLE:
            return candidate
        return current

    def turn_command_for_offset(self, offset):
        if offset > self.lane_change_min_request_offset:
            return TurnIndicatorsCommand.ENABLE_LEFT
        if offset < -self.lane_change_min_request_offset:
            return TurnIndicatorsCommand.ENABLE_RIGHT
        return TurnIndicatorsCommand.NO_COMMAND


def main():
    rospy.init_node("behavior_path_planner")
    BehaviorPathPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
