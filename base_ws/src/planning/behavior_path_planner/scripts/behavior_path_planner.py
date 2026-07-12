#!/usr/bin/env python3

import json
import math
import os
import sys
from copy import deepcopy

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
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
from autoware_planning_msgs.msg import LaneletRoute, PathPoint, PoseWithUuidStamped
from autoware_vehicle_msgs.msg import HazardLightsCommand, TurnIndicatorsCommand
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray, LateralOffset, RerouteAvailability

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_map_tools import CenterlineMap, resample_polyline, stitch_route_centerlines, trim_polyline
from bpp_path_tools import (
    apply_goal_shift,
    apply_pull_out_shift,
    apply_s_curve_shift,
    apply_window_shift,
    make_avoidance_debug_array,
    object_pose,
    object_speed,
    object_width,
    path_length,
    project_pose_to_path,
    set_velocity_window,
    to_debug_path,
    uuid_to_string,
)


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

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        self.path_interval = max(float(rospy.get_param("~path_interval", 2.0)), 0.1)
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
        self.flatten_map_z = param_bool(rospy.get_param("~flatten_map_z", False))

        self.enable_lane_change = param_bool(rospy.get_param("~enable_lane_change", True))
        self.enable_static_obstacle_avoidance = param_bool(
            rospy.get_param("~enable_static_obstacle_avoidance", True)
        )
        self.enable_dynamic_obstacle_avoidance = param_bool(
            rospy.get_param("~enable_dynamic_obstacle_avoidance", True)
        )
        self.enable_start_planner = param_bool(rospy.get_param("~enable_start_planner", True))
        self.enable_goal_planner = param_bool(rospy.get_param("~enable_goal_planner", True))
        self.enable_side_shift = param_bool(rospy.get_param("~enable_side_shift", True))

        self.load_module_params()

        self.subscribe_inputs()
        self.create_publishers()
        self.load_centerline_map()

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
            rospy.get_param("~lane_change/prepare_distance", 15.0)
        )
        self.lane_change_distance = float(rospy.get_param("~lane_change/lane_change_distance", 30.0))
        self.lane_change_enable_safety_check = param_bool(
            rospy.get_param("~lane_change/enable_safety_check", True)
        )
        self.lane_change_safety_lateral_margin = float(
            rospy.get_param("~lane_change/safety_lateral_margin", 1.2)
        )
        self.lane_change_safety_forward_margin = float(
            rospy.get_param("~lane_change/safety_forward_margin", 10.0)
        )

        self.static_speed_threshold = float(
            rospy.get_param("~static_obstacle_avoidance/static_speed_threshold", 0.5)
        )
        self.static_max_forward_distance = float(
            rospy.get_param("~static_obstacle_avoidance/max_forward_distance", 80.0)
        )
        self.static_object_check_lateral_range = float(
            rospy.get_param("~static_obstacle_avoidance/object_check_lateral_range", 2.0)
        )
        self.static_base_shift_length = float(
            rospy.get_param("~static_obstacle_avoidance/base_shift_length", 0.8)
        )
        self.static_max_shift_length = float(
            rospy.get_param("~static_obstacle_avoidance/max_shift_length", 1.5)
        )
        self.static_prepare_distance = float(
            rospy.get_param("~static_obstacle_avoidance/prepare_distance", 15.0)
        )
        self.static_return_distance = float(
            rospy.get_param("~static_obstacle_avoidance/return_distance", 15.0)
        )
        self.static_default_avoid_direction = rospy.get_param(
            "~static_obstacle_avoidance/default_avoid_direction", "right"
        ).lower()

        self.dynamic_min_speed = float(
            rospy.get_param("~dynamic_obstacle_avoidance/min_dynamic_speed", 0.5)
        )
        self.dynamic_max_speed = float(
            rospy.get_param("~dynamic_obstacle_avoidance/max_dynamic_speed", 100.0)
        )
        self.dynamic_max_forward_distance = float(
            rospy.get_param("~dynamic_obstacle_avoidance/max_forward_distance", 80.0)
        )
        self.dynamic_object_check_lateral_range = float(
            rospy.get_param("~dynamic_obstacle_avoidance/object_check_lateral_range", 2.0)
        )
        self.dynamic_slow_down_before_distance = float(
            rospy.get_param("~dynamic_obstacle_avoidance/slow_down_before_distance", 8.0)
        )
        self.dynamic_slow_down_after_distance = float(
            rospy.get_param("~dynamic_obstacle_avoidance/slow_down_after_distance", 8.0)
        )
        self.dynamic_slow_velocity_mps = float(
            rospy.get_param("~dynamic_obstacle_avoidance/slow_velocity_mps", 1.0)
        )
        self.dynamic_enable_lateral_nudge = param_bool(
            rospy.get_param("~dynamic_obstacle_avoidance/enable_lateral_nudge", True)
        )
        self.dynamic_nudge_shift_length = float(
            rospy.get_param("~dynamic_obstacle_avoidance/nudge_shift_length", 0.3)
        )
        self.dynamic_prepare_distance = float(
            rospy.get_param("~dynamic_obstacle_avoidance/prepare_distance", 10.0)
        )
        self.dynamic_return_distance = float(
            rospy.get_param("~dynamic_obstacle_avoidance/return_distance", 10.0)
        )
        self.dynamic_default_avoid_direction = rospy.get_param(
            "~dynamic_obstacle_avoidance/default_avoid_direction", "right"
        ).lower()

        self.start_pull_out_lateral_offset = float(
            rospy.get_param("~start_planner/pull_out_lateral_offset", 0.0)
        )
        self.start_pull_out_distance = float(rospy.get_param("~start_planner/pull_out_distance", 20.0))
        self.start_hold_until_autonomous = param_bool(
            rospy.get_param("~start_planner/hold_until_autonomous", False)
        )
        self.start_require_operation_mode = param_bool(
            rospy.get_param("~start_planner/require_operation_mode", False)
        )
        self.start_min_offset_to_signal = float(
            rospy.get_param("~start_planner/min_offset_to_signal", 0.1)
        )

        self.goal_lateral_offset = float(rospy.get_param("~goal_planner/goal_lateral_offset", 0.0))
        self.goal_pull_over_distance = float(rospy.get_param("~goal_planner/pull_over_distance", 20.0))
        self.goal_allow_modification = param_bool(
            rospy.get_param("~goal_planner/allow_goal_modification", True)
        )

        self.side_shift_default_lateral_offset = float(
            rospy.get_param("~side_shift/default_lateral_offset", 0.0)
        )
        self.side_shift_min_distance_to_start = float(
            rospy.get_param("~side_shift/min_distance_to_start_shifting", 5.0)
        )
        self.side_shift_min_shifting_distance = float(
            rospy.get_param("~side_shift/min_shifting_distance", 5.0)
        )
        self.side_shift_min_request_offset = float(rospy.get_param("~side_shift/min_request_offset", 0.05))
        self.side_shift_request_time_limit = float(
            rospy.get_param("~side_shift/shift_request_time_limit", 0.0)
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
        self.static_avoidance_debug_pub = rospy.Publisher(
            "~debug/static_obstacle_avoidance", AvoidanceDebugMsgArray, queue_size=1
        )
        self.dynamic_avoidance_debug_pub = rospy.Publisher(
            "~debug/dynamic_obstacle_avoidance", AvoidanceDebugMsgArray, queue_size=1
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
                self.map_file, self.projector_file, self.flatten_map_z
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
            self.centerline_map = CenterlineMap.from_osm_bytes(raw, "LaneletMapBin", self.flatten_map_z)
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

    def on_timer(self, _event):
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
            statuses.append("lane_change: disabled")
        self.publish_debug_stage("lane_change", path)

        if self.enable_static_obstacle_avoidance:
            path, status = self.apply_static_obstacle_avoidance(path)
            statuses.append(status)
        else:
            self.publish_static_debug(path, [])
            statuses.append("static_obstacle_avoidance: disabled")
        self.publish_debug_stage("static_obstacle_avoidance", path)

        if self.enable_dynamic_obstacle_avoidance:
            path, status = self.apply_dynamic_obstacle_avoidance(path)
            statuses.append(status)
        else:
            self.publish_dynamic_debug(path, [])
            statuses.append("dynamic_obstacle_avoidance: disabled")
        self.publish_debug_stage("dynamic_obstacle_avoidance", path)

        if self.enable_start_planner:
            path, status, command = self.apply_start_planner(path)
            statuses.append(status)
            turn_command = self.prefer_command(turn_command, command)
        else:
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
        return msg, status

    def extract_lane_ids(self, route):
        lane_ids = []
        seen = set()
        for segment in route.segments:
            primitives = [segment.preferred_primitive] + list(segment.primitives)
            for primitive in primitives:
                if primitive.id == 0 or primitive.id in seen:
                    continue
                seen.add(primitive.id)
                lane_ids.append(primitive.id)
        return lane_ids

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

        centerline = trim_polyline(centerline, start_point=start_point, goal_point=goal_point)
        centerline = resample_polyline(centerline, self.path_interval)
        if len(centerline) < 2:
            return None

        return self.centerline_points_to_path_points(centerline, used_lane_ids or lane_ids)

    def centerline_points_to_path_points(self, centerline, lane_ids):
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
            path_point.is_final = idx == len(centerline) - 1
            if self.stop_at_goal and path_point.is_final:
                path_point.longitudinal_velocity_mps = 0.0

            point_with_lane_id = PathPointWithLaneId()
            point_with_lane_id.point = path_point
            point_with_lane_id.lane_ids = list(lane_ids)
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
            path_point.is_final = idx == len(poses) - 1
            if self.stop_at_goal and path_point.is_final:
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

    def apply_lane_change(self, path, stamp):
        target_offset = self.requested_lane_change_offset()
        if abs(target_offset) < self.lane_change_min_request_offset:
            return path, "lane_change: pass-through, no request", TurnIndicatorsCommand.NO_COMMAND

        if not self.is_lane_change_safe(path, target_offset):
            return path, "lane_change: blocked by object in target lane", TurnIndicatorsCommand.NO_COMMAND

        output = apply_s_curve_shift(
            path,
            target_offset,
            self.lane_change_prepare_distance,
            self.lane_change_prepare_distance + self.lane_change_distance,
        )
        return (
            output,
            "lane_change: applied lateral offset {:.2f} m".format(target_offset),
            self.turn_command_for_offset(target_offset),
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

    def collect_static_candidates(self, path):
        if self.objects is None:
            return []

        candidates = []
        for obj in self.objects.objects:
            speed = object_speed(obj)
            projection = project_pose_to_path(path, object_pose(obj))
            if projection is None:
                continue

            candidate = {
                "object_id": uuid_to_string(obj.object_id),
                "longitudinal_distance": projection.longitudinal_distance,
                "lateral_distance": projection.signed_lateral_distance,
                "path_distance": projection.distance,
                "shift_length": 0.0,
                "allow_avoidance": False,
            }

            if speed > self.static_speed_threshold:
                candidate["failed_reason"] = "moving_object"
            elif projection.longitudinal_distance < 0.0:
                candidate["failed_reason"] = "behind_ego"
            elif projection.longitudinal_distance > self.static_max_forward_distance:
                candidate["failed_reason"] = "too_far"
            elif abs(projection.signed_lateral_distance) > self.static_object_check_lateral_range:
                candidate["failed_reason"] = "outside_path_corridor"
            else:
                shift_sign = self.static_shift_sign(projection.signed_lateral_distance)
                shift = min(self.static_max_shift_length, self.static_base_shift_length + 0.5 * object_width(obj))
                candidate["shift_length"] = shift_sign * shift
                candidate["allow_avoidance"] = True
                candidate["failed_reason"] = ""
            candidates.append(candidate)

        return candidates

    def static_shift_sign(self, lateral_distance):
        if abs(lateral_distance) < 1.0e-3:
            return -1.0 if self.static_default_avoid_direction == "right" else 1.0
        return -1.0 if lateral_distance > 0.0 else 1.0

    def apply_static_obstacle_avoidance(self, path):
        candidates = self.collect_static_candidates(path)
        self.publish_static_debug(path, candidates)

        valid_candidates = [candidate for candidate in candidates if candidate["allow_avoidance"]]
        if not valid_candidates:
            return path, "static_obstacle_avoidance: pass-through, no static target"

        target = min(valid_candidates, key=lambda item: item["longitudinal_distance"])
        output = apply_window_shift(
            path,
            target["shift_length"],
            target["longitudinal_distance"],
            self.static_prepare_distance,
            self.static_return_distance,
        )
        return output, "static_obstacle_avoidance: avoiding object {} with shift {:.2f} m".format(
            target["object_id"], target["shift_length"]
        )

    def collect_dynamic_candidates(self, path):
        if self.objects is None:
            return []

        candidates = []
        for obj in self.objects.objects:
            speed = object_speed(obj)
            projection = project_pose_to_path(path, object_pose(obj))
            if projection is None:
                continue

            candidate = {
                "object_id": uuid_to_string(obj.object_id),
                "longitudinal_distance": projection.longitudinal_distance,
                "lateral_distance": projection.signed_lateral_distance,
                "path_distance": projection.distance,
                "shift_length": 0.0,
                "allow_avoidance": False,
            }

            if speed < self.dynamic_min_speed:
                candidate["failed_reason"] = "static_or_slow_object"
            elif speed > self.dynamic_max_speed:
                candidate["failed_reason"] = "too_fast"
            elif projection.longitudinal_distance < 0.0:
                candidate["failed_reason"] = "behind_ego"
            elif projection.longitudinal_distance > self.dynamic_max_forward_distance:
                candidate["failed_reason"] = "too_far"
            elif abs(projection.signed_lateral_distance) > self.dynamic_object_check_lateral_range:
                candidate["failed_reason"] = "outside_path_corridor"
            else:
                candidate["allow_avoidance"] = True
                if self.dynamic_enable_lateral_nudge:
                    candidate["shift_length"] = (
                        self.dynamic_shift_sign(projection.signed_lateral_distance)
                        * self.dynamic_nudge_shift_length
                    )
                candidate["failed_reason"] = ""
            candidates.append(candidate)

        return candidates

    def dynamic_shift_sign(self, lateral_distance):
        if abs(lateral_distance) < 1.0e-3:
            return -1.0 if self.dynamic_default_avoid_direction == "right" else 1.0
        return -1.0 if lateral_distance > 0.0 else 1.0

    def apply_dynamic_obstacle_avoidance(self, path):
        candidates = self.collect_dynamic_candidates(path)
        self.publish_dynamic_debug(path, candidates)

        valid_candidates = [candidate for candidate in candidates if candidate["allow_avoidance"]]
        if not valid_candidates:
            return path, "dynamic_obstacle_avoidance: pass-through, no dynamic target"

        target = min(valid_candidates, key=lambda item: item["longitudinal_distance"])
        start_s = max(0.0, target["longitudinal_distance"] - self.dynamic_slow_down_before_distance)
        end_s = target["longitudinal_distance"] + self.dynamic_slow_down_after_distance
        output = set_velocity_window(path, start_s, end_s, self.dynamic_slow_velocity_mps, mode="min")

        if self.dynamic_enable_lateral_nudge and abs(target["shift_length"]) > 1.0e-6:
            output = apply_window_shift(
                output,
                target["shift_length"],
                target["longitudinal_distance"],
                self.dynamic_prepare_distance,
                self.dynamic_return_distance,
            )

        return output, "dynamic_obstacle_avoidance: target {} slowed to {:.2f} m/s".format(
            target["object_id"], self.dynamic_slow_velocity_mps
        )

    def apply_start_planner(self, path):
        output = path
        status_parts = []
        turn_command = TurnIndicatorsCommand.NO_COMMAND

        if abs(self.start_pull_out_lateral_offset) > self.start_min_offset_to_signal:
            output = apply_pull_out_shift(
                output, self.start_pull_out_lateral_offset, self.start_pull_out_distance
            )
            status_parts.append(
                "pull-out merge from offset {:.2f} m".format(self.start_pull_out_lateral_offset)
            )
            if self.start_pull_out_lateral_offset > 0.0:
                turn_command = TurnIndicatorsCommand.ENABLE_RIGHT
            else:
                turn_command = TurnIndicatorsCommand.ENABLE_LEFT
        else:
            status_parts.append("no pull-out offset")

        if self.start_hold_until_autonomous and not self.is_autonomous_enabled():
            output = set_velocity_window(output, 0.0, path_length(output), 0.0, mode="set")
            status_parts.append("holding velocity at 0 until autonomous")

        return output, "start_planner: " + ", ".join(status_parts), turn_command

    def is_autonomous_enabled(self):
        if self.operation_mode is None:
            return not self.start_require_operation_mode
        return (
            self.operation_mode.mode == OperationModeState.AUTONOMOUS
            and self.operation_mode.is_autoware_control_enabled
            and not self.operation_mode.is_in_transition
        )

    def apply_goal_planner(self, path, stamp):
        output = apply_goal_shift(path, self.goal_lateral_offset, self.goal_pull_over_distance)

        if self.stop_at_goal and output.points:
            output.points[-1].point.longitudinal_velocity_mps = 0.0
            output.points[-1].point.is_final = True

        self.publish_goal_side_outputs(output, stamp)

        if abs(self.goal_lateral_offset) > 1.0e-6:
            return output, "goal_planner: pull-over offset {:.2f} m over {:.2f} m".format(
                self.goal_lateral_offset, min(self.goal_pull_over_distance, path_length(output))
            )
        return output, "goal_planner: fixed goal pass-through"

    def requested_side_shift_offset(self):
        if self.side_shift_request is not None:
            if self.side_shift_request_time_limit <= 0.0:
                return self.side_shift_request.lateral_offset
            age = (rospy.Time.now() - self.side_shift_request_time).to_sec()
            if age <= self.side_shift_request_time_limit:
                return self.side_shift_request.lateral_offset
        return self.side_shift_default_lateral_offset

    def apply_side_shift(self, path):
        offset = self.requested_side_shift_offset()
        if abs(offset) < self.side_shift_min_request_offset:
            return path, "side_shift: pass-through, no offset request", TurnIndicatorsCommand.NO_COMMAND

        output = apply_s_curve_shift(
            path,
            offset,
            self.side_shift_min_distance_to_start,
            self.side_shift_min_distance_to_start + self.side_shift_min_shifting_distance,
        )
        return (
            output,
            "side_shift: applied lateral offset {:.2f} m".format(offset),
            self.turn_command_for_offset(offset),
        )

    def publish_goal_side_outputs(self, path, stamp):
        modified_goal = PoseWithUuidStamped()
        modified_goal.header.stamp = stamp
        modified_goal.header.frame_id = path.header.frame_id or self.frame_id
        if path.points:
            modified_goal.pose = path.points[-1].point.pose
        elif self.route is not None:
            modified_goal.pose = self.route.goal_pose
        if self.route is not None:
            modified_goal.uuid = self.route.uuid
        self.modified_goal_pub.publish(modified_goal)

        reroute = RerouteAvailability()
        reroute.stamp = stamp
        reroute.availability = self.goal_allow_modification
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
        if candidate != TurnIndicatorsCommand.NO_COMMAND:
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
