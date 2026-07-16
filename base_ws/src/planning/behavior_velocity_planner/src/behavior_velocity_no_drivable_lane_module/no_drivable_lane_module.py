from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    point_at_station,
    point_in_polygon,
    polyline_polygon_intersections,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_with_lane_id_positions,
    project_pose_to_path,
    set_velocity_from_index,
)


STATE_INIT = "INIT"
STATE_APPROACHING = "APPROACHING"
STATE_INSIDE = "INSIDE_NO_DRIVABLE_LANE"
STATE_STOPPED = "STOPPED"


@dataclass
class NoDrivableLaneParameters:
    stop_margin: float = 1.5
    print_debug_info: bool = False
    base_link_to_front: float = 3.79
    stopped_velocity_threshold: float = 0.01

    @classmethod
    def from_ros(cls, prefix="~no_drivable_lane"):
        parameter = cls()
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            value = rospy.get_param("{}/{}".format(prefix, field_name), current)
            if isinstance(current, bool):
                value = (
                    value.lower() in ("true", "1", "yes", "on")
                    if isinstance(value, str)
                    else bool(value)
                )
            setattr(parameter, field_name, type(current)(value))
        return parameter


class NoDrivableLaneModuleManager(SceneModuleManagerInterface):
    module_name = "no_drivable_lane"

    def __init__(self, parameters=None):
        self.parameters = parameters or NoDrivableLaneParameters.from_ros()
        self.lanelet_map = None
        self.states = {}

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.states.clear()

    def reset(self):
        self.states.clear()

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "no_drivable_lane: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "no_drivable_lane: input path is too short"
            return result

        route_lane_ids = {
            int(lane_id)
            for path_point in path_with_lane_id.points
            for lane_id in path_point.lane_ids
        }
        targets = [
            lanelet
            for lanelet in lanelet_map.road_lanelet_candidates()
            if lanelet.lanelet_id in route_lane_ids
            and lanelet.tags.get("no_drivable_lane", "no") == "yes"
        ]
        target_ids = {target.lanelet_id for target in targets}
        self.states = {
            key: value for key, value in self.states.items() if key in target_ids
        }
        input_xyz = path_with_lane_id_positions(path_with_lane_id)
        ego_station = self._ego_station(output_path, planner_data.odometry)
        ego_front_station = ego_station + self.parameters.base_link_to_front
        active = []

        for target in targets:
            intersection = self._path_polygon_intersection(input_xyz, target.polygon)
            if intersection is None:
                continue
            entry_station, exit_station, starts_inside = intersection
            state = self.states.get(target.lanelet_id, STATE_INIT)
            if state == STATE_INIT:
                state = (
                    STATE_INSIDE
                    if starts_inside
                    or entry_station - ego_front_station <= self.parameters.stop_margin
                    else STATE_APPROACHING
                )

            stopped = ego_speed(planner_data.odometry) <= self.parameters.stopped_velocity_threshold
            if state == STATE_APPROACHING:
                stop_station = max(
                    0.0,
                    entry_station
                    - self.parameters.stop_margin
                    - self.parameters.base_link_to_front,
                )
                if (
                    entry_station - ego_front_station <= self.parameters.stop_margin
                    and stopped
                ):
                    state = STATE_STOPPED
                    stop_station = ego_station
            elif state == STATE_INSIDE:
                stop_station = ego_station
                if stopped:
                    state = STATE_STOPPED
            else:
                stop_station = ego_station

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            distance = stop_station - ego_station
            factor_point = point_at_station(input_xyz, entry_station) or input_xyz[0]
            result.modified = True
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance, factor_point)
            )
            result.planning_factors.append(
                self._make_planning_factor(
                    output_path.header,
                    stop_pose,
                    distance,
                    target,
                    state,
                    factor_point,
                )
            )
            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    target,
                    entry_station,
                    exit_station,
                    input_xyz,
                    stop_pose,
                    state,
                )
            )
            self.states[target.lanelet_id] = state
            active.append("{}:{}".format(target.lanelet_id, state))

        result.status = (
            "no_drivable_lane: " + ", ".join(active)
            if active
            else "no_drivable_lane: no tagged lanelet on path"
        )
        return result

    @staticmethod
    def _path_polygon_intersection(path_xyz, polygon):
        crossings = polyline_polygon_intersections(path_xyz, polygon)
        starts_inside = point_in_polygon(path_xyz[0], polygon)
        if not crossings:
            return None
        stations = [float(item["station"]) for item in crossings]
        entry_station = 0.0 if starts_inside else stations[0]
        total_length = cumulative_lengths(path_xyz)[-1]
        later = [station for station in stations if station > entry_station + 1.0e-6]
        exit_station = later[0] if later else total_length
        return entry_station, exit_station, starts_inside

    @staticmethod
    def _ego_station(path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _make_stop_reason(stop_pose, distance, factor_point):
        factor = StopFactor()
        factor.stop_pose = deepcopy(stop_pose)
        factor.dist_to_stop_pose = float(distance)
        factor.stop_factor_points = [NoDrivableLaneModuleManager._to_point(factor_point)]
        reason = StopReason()
        reason.reason = StopReason.NO_DRIVABLE_LANE
        reason.stop_factors = [factor]
        return reason

    def _make_planning_factor(
        self, header, stop_pose, distance, target, state, factor_point
    ):
        control = ControlPoint()
        control.pose = deepcopy(stop_pose)
        control.velocity = 0.0
        control.distance = float(distance)
        factor = PlanningFactor()
        factor.module = self.module_name
        factor.is_driving_forward = True
        factor.control_points = [control]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "lanelet_id={} state={}".format(target.lanelet_id, state)
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = False
        factor.safety_factors.detail = "no drivable lane: takeover required"
        return factor

    def _make_markers(
        self, header, target, entry_station, exit_station, input_xyz, stop_pose, state
    ):
        marker_id = int(target.lanelet_id % 2000000000)
        area = self._base_marker(header, marker_id, Marker.TRIANGLE_LIST)
        area.ns = "bvp_no_drivable_lane/area"
        area.color.r = 0.95
        area.color.g = 0.10
        area.color.b = 0.95
        area.color.a = 0.28
        anchor = target.polygon[0]
        for index in range(1, len(target.polygon) - 1):
            area.points.extend(
                [
                    self._to_point(anchor),
                    self._to_point(target.polygon[index]),
                    self._to_point(target.polygon[index + 1]),
                ]
            )

        intersections = self._base_marker(header, marker_id + 1, Marker.POINTS)
        intersections.ns = "bvp_no_drivable_lane/intersections"
        intersections.scale.x = intersections.scale.y = 0.45
        intersections.color.r = 1.0
        intersections.color.a = 1.0
        intersections.points = [
            self._to_point(point_at_station(input_xyz, entry_station)),
            self._to_point(point_at_station(input_xyz, exit_station)),
        ]

        stop = self._base_marker(header, marker_id + 2, Marker.CUBE)
        stop.ns = "bvp_no_drivable_lane/stop"
        stop.pose = deepcopy(stop_pose)
        stop.scale.x = 0.25
        stop.scale.y = 5.0
        stop.scale.z = 1.5
        stop.color.r = 1.0
        stop.color.a = 0.95

        text = self._base_marker(header, marker_id + 3, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_no_drivable_lane/state"
        text.pose = deepcopy(stop_pose)
        text.pose.position.z += 2.0
        text.scale.z = 0.7
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "NO DRIVABLE LANE\n{}".format(state)
        return [area, intersections, stop, text]

    @staticmethod
    def _base_marker(header, marker_id, marker_type):
        marker = Marker()
        marker.header = deepcopy(header)
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _to_point(value):
        point = Point()
        if value is None:
            return point
        if hasattr(value, "x"):
            point.x = float(value.x)
            point.y = float(value.y)
            point.z = float(value.z)
        else:
            point.x = float(value[0])
            point.y = float(value[1])
            point.z = float(value[2]) if len(value) > 2 else 0.0
        return point
