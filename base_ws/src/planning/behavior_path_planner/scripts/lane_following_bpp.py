#!/usr/bin/env python3

import math
import os
import sys
from copy import deepcopy

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path

from autoware_internal_planning_msgs.msg import (
    PathPointWithLaneId,
    PathWithLaneId,
    Scenario,
    VelocityLimit,
)
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute, PathPoint, PoseWithUuidStamped
from autoware_vehicle_msgs.msg import HazardLightsCommand, TurnIndicatorsCommand
from tier4_planning_msgs.msg import RerouteAvailability

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_map_tools import (
    CenterlineMap,
    closest_lane_id,
    resample_polyline,
    stitch_route_bounds,
    stitch_route_centerlines,
    trim_polyline,
)
from bpp_route_tools import reference_lane_sequence


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


class LaneFollowingBPP:
    def __init__(self):
        self.route = None
        self.odometry = None
        self.scenario = None
        self.vector_map = None
        self.velocity_limit = None
        self.centerline_map = None

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
        self.stop_at_goal = bool(rospy.get_param("~stop_at_goal", True))
        self.use_ego_as_path_start = bool(rospy.get_param("~use_ego_as_path_start", True))
        self.require_lane_driving_scenario = bool(
            rospy.get_param("~require_lane_driving_scenario", False)
        )
        self.use_map_centerline = bool(rospy.get_param("~use_map_centerline", True))
        self.map_file = rospy.get_param("~map_file", rospy.get_param("~centerline_map_file", ""))
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.flatten_map_z = bool(rospy.get_param("~flatten_map_z", False))

        route_topic = rospy.get_param("~route_topic", "~input/route")
        odometry_topic = rospy.get_param("~odometry_topic", "~input/odometry")
        scenario_topic = rospy.get_param("~scenario_topic", "~input/scenario")
        vector_map_topic = rospy.get_param("~vector_map_topic", "~input/vector_map")
        max_velocity_topic = rospy.get_param(
            "~max_velocity_topic", "/planning/scenario_planning/max_velocity"
        )

        rospy.Subscriber(route_topic, LaneletRoute, self.route_callback, queue_size=1)
        rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber(scenario_topic, Scenario, self.scenario_callback, queue_size=1)
        rospy.Subscriber(vector_map_topic, LaneletMapBin, self.vector_map_callback, queue_size=1)
        if max_velocity_topic:
            rospy.Subscriber(max_velocity_topic, VelocityLimit, self.velocity_limit_callback, queue_size=1)

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
        self.debug_path_pub = rospy.Publisher("~debug/path", Path, queue_size=1)

        self.load_centerline_map()

        period = rospy.Duration(1.0 / max(self.planning_hz, 0.1))
        self.timer = rospy.Timer(period, self.on_timer)

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
                "lane_following_bpp: loaded %d centerlines from %s",
                self.centerline_map.lane_count(),
                self.centerline_map.source,
            )
        except Exception as exc:
            self.centerline_map = None
            rospy.logwarn("lane_following_bpp: failed to load centerline map: %s", exc)

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
        stripped = raw.lstrip()
        if not (stripped.startswith(b"<osm") or stripped.startswith(b"<?xml")):
            rospy.logwarn_throttle(
                10.0,
                "lane_following_bpp: LaneletMapBin binary payload is stored but direct binary parsing is not available; use ~map_file for centerline generation",
            )
            return

        try:
            self.centerline_map = CenterlineMap.from_osm_bytes(
                raw,
                "LaneletMapBin",
                self.flatten_map_z,
                self.projector_type,
                self.projector_file,
            )
            rospy.loginfo(
                "lane_following_bpp: loaded %d centerlines from LaneletMapBin OSM payload",
                self.centerline_map.lane_count(),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0, "lane_following_bpp: failed to parse LaneletMapBin OSM payload: %s", exc
            )

    def velocity_limit_callback(self, msg):
        self.velocity_limit = msg

    def on_timer(self, _event):
        if self.route is None:
            rospy.logwarn_throttle(5.0, "lane_following_bpp: waiting for LaneletRoute")
            return
        if not self.is_lane_driving_enabled():
            rospy.logwarn_throttle(5.0, "lane_following_bpp: scenario is not LaneDriving")
            return

        now = rospy.Time.now()
        path = self.build_lane_following_path(now)
        self.path_pub.publish(path)
        self.debug_path_pub.publish(self.to_debug_path(path))
        self.publish_default_commands(now)
        self.publish_route_side_outputs(now)

    def is_lane_driving_enabled(self):
        if not self.require_lane_driving_scenario:
            return True
        if self.scenario is None:
            return False
        return self.scenario.current_scenario in ("", Scenario.LANEDRIVING)

    def build_lane_following_path(self, stamp):
        route = self.route
        lane_ids = self.extract_lane_ids(route)
        start_pose = self.select_start_pose(route)
        goal_pose = route.goal_pose

        points = self.build_centerline_path_points(start_pose, goal_pose, lane_ids)
        if points is None:
            points = self.interpolate_path_points(start_pose, goal_pose, lane_ids)

        msg = PathWithLaneId()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.points = points
        msg.left_bound, msg.right_bound = self.build_drivable_bounds(
            points, lane_ids, goal_pose
        )
        return msg

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
        if not self.use_map_centerline:
            return None
        if self.centerline_map is None:
            return None
        if not lane_ids:
            return None

        start_point = pose_to_point3(start_pose) if is_pose_position_set(start_pose) else None
        goal_point = pose_to_point3(goal_pose) if is_pose_position_set(goal_pose) else None

        centerline, used_lane_ids = stitch_route_centerlines(
            self.centerline_map, lane_ids, start_point=start_point
        )
        if len(centerline) < 2:
            rospy.logwarn_throttle(
                5.0,
                "lane_following_bpp: no map centerline matched route lane ids; falling back to straight path",
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

    def to_debug_path(self, path_with_lane_id):
        msg = Path()
        msg.header = path_with_lane_id.header
        for point in path_with_lane_id.points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_with_lane_id.header
            pose_stamped.pose = point.point.pose
            msg.poses.append(pose_stamped)
        return msg

    def publish_default_commands(self, stamp):
        turn = TurnIndicatorsCommand()
        turn.stamp = stamp
        turn.command = TurnIndicatorsCommand.NO_COMMAND
        self.turn_pub.publish(turn)

        hazard = HazardLightsCommand()
        hazard.stamp = stamp
        hazard.command = HazardLightsCommand.NO_COMMAND
        self.hazard_pub.publish(hazard)

    def publish_route_side_outputs(self, stamp):
        modified_goal = PoseWithUuidStamped()
        modified_goal.header.stamp = stamp
        modified_goal.header.frame_id = self.frame_id
        modified_goal.pose = self.route.goal_pose
        modified_goal.uuid = self.route.uuid
        self.modified_goal_pub.publish(modified_goal)

        reroute = RerouteAvailability()
        reroute.stamp = stamp
        reroute.availability = True
        self.reroute_pub.publish(reroute)


def main():
    rospy.init_node("lane_following_bpp")
    LaneFollowingBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
