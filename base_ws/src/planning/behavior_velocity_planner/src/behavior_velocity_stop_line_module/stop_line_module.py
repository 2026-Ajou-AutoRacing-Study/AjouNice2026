from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    first_polyline_intersection,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_positions,
    path_with_lane_id_positions,
    project_pose_to_path,
    set_velocity_from_index,
)


APPROACHING = "APPROACHING"
STOPPED = "STOPPED"
STARTED = "STARTED"


@dataclass
class StopLineParameters:
    stop_margin: float = 0.0
    required_stop_duration_sec: float = 1.0
    vehicle_stopped_duration_threshold: float = 0.0
    hold_stop_margin_distance: float = 2.0
    use_initialization_stop_state: bool = True
    base_link_to_front: float = 3.79
    max_detection_distance: float = 200.0
    passed_margin: float = 1.0
    state_expiration_sec: float = 5.0
    include_traffic_light_ref_lines: bool = False
    vehicle_width: float = 1.9

    @classmethod
    def from_ros(cls, prefix="~stop_line"):
        parameter = cls()
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            value = rospy.get_param("{}/{}".format(prefix, field_name), current)
            if isinstance(current, bool):
                if isinstance(value, str):
                    value = value.lower() in ("true", "1", "yes", "on")
                else:
                    value = bool(value)
            setattr(parameter, field_name, type(current)(value))
        return parameter


@dataclass
class StopLineState:
    state: str = APPROACHING
    stopped_since: float = None
    completed_at: float = None
    last_seen: float = 0.0


class StopLineModuleManager(SceneModuleManagerInterface):
    module_name = "stop_line"

    def __init__(self, parameters=None):
        self.parameters = parameters or StopLineParameters.from_ros()
        self.states = {}
        self.lanelet_map = None

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.reset()

    def reset(self):
        self.states = {}

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "stop_line: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "stop_line: input path is too short"
            return result

        current_time = float(planner_data.current_time)
        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        route_lane_ids = {
            int(lane_id)
            for point in path_with_lane_id.points
            for lane_id in point.lane_ids
        }
        ego_station = self._ego_station(output_path, planner_data.odometry)
        current_speed = ego_speed(planner_data.odometry)
        candidates = []
        seen_ids = set()

        for stop_line in lanelet_map.stop_line_candidates(
            self.parameters.include_traffic_light_ref_lines
        ):
            if stop_line.lane_ids and route_lane_ids.isdisjoint(stop_line.lane_ids):
                continue
            intersection = first_polyline_intersection(path_xyz, stop_line.points)
            if intersection is None:
                continue
            distance_from_ego = intersection["station"] - ego_station
            if distance_from_ego < -self.parameters.passed_margin:
                continue
            if distance_from_ego > self.parameters.max_detection_distance:
                continue
            candidates.append((intersection["station"], stop_line, intersection))
            seen_ids.add(stop_line.line_id)

        self._expire_states(seen_ids, current_time)
        candidates.sort(key=lambda item: item[0])
        for collision_station, stop_line, intersection in candidates:
            state = self.states.setdefault(stop_line.line_id, StopLineState())
            state.last_seen = current_time
            stop_station = max(
                0.0,
                collision_station
                - self.parameters.stop_margin
                - self.parameters.base_link_to_front,
            )
            should_stop = self._update_state(
                state,
                stop_station,
                ego_station,
                current_speed,
                current_time,
                planner_data.odometry is not None,
            )
            if not should_stop:
                continue

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            distance_to_stop = stop_station - ego_station
            collision_point = self._to_point(intersection["point"])
            result.modified = True
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance_to_stop, collision_point)
            )
            result.planning_factors.append(
                self._make_planning_factor(
                    output_path.header,
                    stop_pose,
                    distance_to_stop,
                    stop_line.line_id,
                    state.state,
                )
            )
            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    stop_line,
                    stop_pose,
                    intersection["point"],
                    state.state,
                )
            )
            result.status = (
                "stop_line: line={} state={} distance={:.2f}m"
            ).format(stop_line.line_id, state.state, distance_to_stop)
            return result

        if candidates:
            result.status = "stop_line: passed released stop line(s)"
        else:
            result.status = "stop_line: no stop line on path"
        return result

    def _ego_station(self, output_path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(output_path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    def _update_state(
        self,
        state,
        stop_station,
        ego_station,
        speed,
        current_time,
        has_odometry,
    ):
        if state.state == STARTED:
            return False

        distance = stop_station - ego_station
        stopped = (
            has_odometry
            and speed <= self.parameters.vehicle_stopped_duration_threshold
        )
        near_stop_pose = abs(distance) <= self.parameters.hold_stop_margin_distance

        if state.state == APPROACHING and near_stop_pose and stopped:
            state.state = STOPPED
            state.stopped_since = current_time

        if state.state == STOPPED:
            if not stopped:
                state.state = APPROACHING
                state.stopped_since = None
                return True
            stopped_since = (
                current_time if state.stopped_since is None else state.stopped_since
            )
            stopped_duration = current_time - float(stopped_since)
            if stopped_duration >= self.parameters.required_stop_duration_sec:
                state.state = STARTED
                state.completed_at = current_time
                return False
        return True

    def _expire_states(self, seen_ids, current_time):
        if not self.parameters.use_initialization_stop_state:
            return
        expired = [
            line_id
            for line_id, state in self.states.items()
            if line_id not in seen_ids
            and current_time - state.last_seen >= self.parameters.state_expiration_sec
        ]
        for line_id in expired:
            self.states.pop(line_id, None)

    @staticmethod
    def _to_point(point):
        output = Point()
        output.x = float(point[0])
        output.y = float(point[1])
        output.z = float(point[2]) if len(point) > 2 else 0.0
        return output

    @staticmethod
    def _make_stop_reason(stop_pose, distance_to_stop, collision_point):
        stop_factor = StopFactor()
        stop_factor.stop_pose = deepcopy(stop_pose)
        stop_factor.dist_to_stop_pose = float(distance_to_stop)
        stop_factor.stop_factor_points = [deepcopy(collision_point)]
        reason = StopReason()
        reason.reason = StopReason.STOP_LINE
        reason.stop_factors = [stop_factor]
        return reason

    @staticmethod
    def _make_planning_factor(header, stop_pose, distance, line_id, state):
        control_point = ControlPoint()
        control_point.pose = deepcopy(stop_pose)
        control_point.velocity = 0.0
        control_point.shift_length = 0.0
        control_point.distance = float(distance)

        factor = PlanningFactor()
        factor.module = "stop_line"
        factor.is_driving_forward = True
        factor.control_points = [control_point]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "stop_line_id={} state={}".format(line_id, state)
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = True
        factor.safety_factors.detail = "map stop line"
        return factor

    def _make_markers(self, header, stop_line, stop_pose, collision_xyz, state):
        marker_id = int(stop_line.line_id % 2000000000)
        stop_line_marker = self._base_marker(header, marker_id, Marker.LINE_STRIP)
        stop_line_marker.ns = "bvp_stop_line/map_line"
        stop_line_marker.scale.x = 0.25
        stop_line_marker.color.r = 1.0
        stop_line_marker.color.g = 0.65
        stop_line_marker.color.b = 0.0
        stop_line_marker.color.a = 1.0
        stop_line_marker.points = [self._to_point(point) for point in stop_line.points]

        stop_pose_marker = self._base_marker(header, marker_id + 1, Marker.SPHERE)
        stop_pose_marker.ns = "bvp_stop_line/stop_pose"
        stop_pose_marker.pose = deepcopy(stop_pose)
        stop_pose_marker.scale.x = 0.8
        stop_pose_marker.scale.y = 0.8
        stop_pose_marker.scale.z = 0.8
        stop_pose_marker.color.r = 1.0
        stop_pose_marker.color.g = 0.0
        stop_pose_marker.color.b = 0.25
        stop_pose_marker.color.a = 1.0

        collision_marker = self._base_marker(header, marker_id + 2, Marker.SPHERE)
        collision_marker.ns = "bvp_stop_line/collision"
        collision_marker.pose.position = self._to_point(collision_xyz)
        collision_marker.pose.orientation.w = 1.0
        collision_marker.scale.x = 0.5
        collision_marker.scale.y = 0.5
        collision_marker.scale.z = 0.5
        collision_marker.color.r = 0.1
        collision_marker.color.g = 0.55
        collision_marker.color.b = 1.0
        collision_marker.color.a = 1.0

        wall_marker = self._base_marker(header, marker_id + 3, Marker.CUBE)
        wall_marker.ns = "bvp_stop_line/virtual_wall"
        wall_marker.pose = deepcopy(stop_pose)
        wall_marker.pose.position.z += 1.0
        wall_marker.scale.x = 0.12
        wall_marker.scale.y = self.parameters.vehicle_width
        wall_marker.scale.z = 2.0
        wall_marker.color.r = 1.0
        wall_marker.color.g = 0.15
        wall_marker.color.b = 0.15
        wall_marker.color.a = 0.45

        text_marker = self._base_marker(header, marker_id + 4, Marker.TEXT_VIEW_FACING)
        text_marker.ns = "bvp_stop_line/status"
        text_marker.pose = deepcopy(stop_pose)
        text_marker.pose.position.z += 2.3
        text_marker.scale.z = 0.7
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "STOP LINE {}\n{}".format(stop_line.line_id, state)
        return [
            stop_line_marker,
            stop_pose_marker,
            collision_marker,
            wall_marker,
            text_marker,
        ]

    @staticmethod
    def _base_marker(header, marker_id, marker_type):
        marker = Marker()
        marker.header = deepcopy(header)
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.5)
        return marker
