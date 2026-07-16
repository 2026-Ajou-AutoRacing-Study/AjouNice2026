from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    first_polyline_intersection,
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


APPROACH = "APPROACH"
STOP = "STOP"
SURPASSED = "SURPASSED"


@dataclass
class WalkwayParameters:
    stop_duration: float = 0.1
    stop_distance_from_crosswalk: float = 3.5
    stop_distance_threshold: float = 1.0
    stopped_velocity_threshold: float = 0.01
    base_link_to_front: float = 3.79
    max_detection_distance: float = 200.0

    @classmethod
    def from_ros(cls, prefix="~walkway"):
        parameter = cls()
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            value = rospy.get_param(
                "{}/{}".format(prefix, field_name), current
            )
            setattr(parameter, field_name, type(current)(value))
        return parameter


@dataclass
class WalkwayState:
    mode: str = APPROACH
    stopped_since: float = None


class WalkwayModuleManager(SceneModuleManagerInterface):
    module_name = "walkway"

    def __init__(self, parameters=None):
        self.parameters = parameters or WalkwayParameters.from_ros()
        self.lanelet_map = None
        self.states = {}

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.reset()

    def reset(self):
        self.states = {}

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "walkway: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "walkway: input path is too short"
            return result

        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        route_lane_ids = {
            int(lane_id)
            for point in path_with_lane_id.points
            for lane_id in point.lane_ids
        }
        ego_station = self._ego_station(output_path, planner_data.odometry)
        candidates = []
        for walkway in lanelet_map.pedestrian_crossing_candidates("walkway"):
            if walkway.road_lane_ids and route_lane_ids.isdisjoint(
                walkway.road_lane_ids
            ):
                continue
            intersections = polyline_polygon_intersections(path_xyz, walkway.points)
            if len(intersections) < 2:
                continue
            entry_station = float(intersections[0]["station"])
            if entry_station - ego_station > self.parameters.max_detection_distance:
                continue
            candidates.append((entry_station, walkway))

        candidates.sort(key=lambda item: item[0])
        statuses = []
        for entry_station, walkway in candidates:
            state = self.states.setdefault(walkway.lanelet_id, WalkwayState())
            stop_station = self._stop_station(walkway, path_xyz, entry_station)
            distance_to_stop = stop_station - ego_station
            stopped = (
                planner_data.odometry is not None
                and ego_speed(planner_data.odometry)
                <= self.parameters.stopped_velocity_threshold
            )

            if state.mode == APPROACH:
                stop_index, stop_pose = insert_path_point(output_path, stop_station)
                set_velocity_from_index(output_path, stop_index, 0.0)
                result.modified = True
                result.stop_reasons.append(
                    self._make_stop_reason(
                        stop_pose,
                        distance_to_stop,
                        self._to_point(
                            polyline_polygon_intersections(path_xyz, walkway.points)[0]["point"]
                        ),
                    )
                )
                result.planning_factors.append(
                    self._make_planning_factor(
                        output_path.header,
                        stop_pose,
                        distance_to_stop,
                        walkway,
                    )
                )
                result.markers.extend(
                    self._make_markers(output_path.header, walkway, stop_pose, APPROACH)
                )
                if distance_to_stop < self.parameters.stop_distance_threshold:
                    if stopped:
                        if state.stopped_since is None:
                            state.stopped_since = float(planner_data.current_time)
                        elif (
                            float(planner_data.current_time) - state.stopped_since
                            >= self.parameters.stop_duration
                        ):
                            state.mode = STOP
                    else:
                        state.stopped_since = None
                else:
                    state.stopped_since = None
                result.status = (
                    "walkway: lanelet={} APPROACH stop_distance={:.2f}m"
                ).format(walkway.lanelet_id, distance_to_stop)
                return result

            if state.mode == STOP:
                if stopped:
                    state.mode = SURPASSED
                statuses.append("{}:STOP".format(walkway.lanelet_id))
                result.markers.extend(
                    self._make_markers(output_path.header, walkway, None, STOP)
                )
                continue

            statuses.append("{}:SURPASSED".format(walkway.lanelet_id))
            result.markers.extend(
                self._make_markers(output_path.header, walkway, None, SURPASSED)
            )

        result.status = (
            "walkway: " + ", ".join(statuses)
            if statuses
            else "walkway: no walkway on path"
        )
        return result

    def _stop_station(self, walkway, path_xyz, entry_station):
        intersections = []
        for stop_line in walkway.stop_lines:
            item = first_polyline_intersection(path_xyz, stop_line.points)
            if item is not None:
                intersections.append(float(item["station"]))
        if intersections:
            return max(0.0, min(intersections) - self.parameters.base_link_to_front)
        return max(
            0.0,
            entry_station
            - self.parameters.stop_distance_from_crosswalk
            - self.parameters.base_link_to_front,
        )

    @staticmethod
    def _ego_station(path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _make_stop_reason(stop_pose, distance, entry_point):
        stop_factor = StopFactor()
        stop_factor.stop_pose = deepcopy(stop_pose)
        stop_factor.dist_to_stop_pose = float(distance)
        stop_factor.stop_factor_points = [deepcopy(entry_point)]
        reason = StopReason()
        reason.reason = StopReason.WALKWAY
        reason.stop_factors = [stop_factor]
        return reason

    @staticmethod
    def _make_planning_factor(header, stop_pose, distance, walkway):
        control_point = ControlPoint()
        control_point.pose = deepcopy(stop_pose)
        control_point.velocity = 0.0
        control_point.distance = float(distance)
        factor = PlanningFactor()
        factor.module = "walkway"
        factor.is_driving_forward = True
        factor.control_points = [control_point]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "walkway_id={} walkway_stop".format(walkway.lanelet_id)
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = True
        factor.safety_factors.detail = "mandatory walkway stop"
        return factor

    def _make_markers(self, header, walkway, stop_pose, state):
        marker_id = int(walkway.lanelet_id % 2000000000)
        fill = self._base_marker(header, marker_id, Marker.TRIANGLE_LIST)
        fill.ns = "bvp_walkway/area"
        fill.color.r = 0.2
        fill.color.g = 1.0
        fill.color.b = 0.35
        fill.color.a = 0.3
        anchor = walkway.points[0]
        for index in range(1, len(walkway.points) - 1):
            fill.points.extend(
                [
                    self._to_point(anchor),
                    self._to_point(walkway.points[index]),
                    self._to_point(walkway.points[index + 1]),
                ]
            )
        markers = [fill]
        if stop_pose is not None:
            stop = self._base_marker(header, marker_id + 1, Marker.CUBE)
            stop.ns = "bvp_walkway/stop_pose"
            stop.pose = deepcopy(stop_pose)
            stop.scale.x = 0.5
            stop.scale.y = 2.5
            stop.scale.z = 0.25
            stop.color.r = 1.0
            stop.color.g = 0.05
            stop.color.b = 0.2
            stop.color.a = 1.0
            markers.append(stop)
        text = self._base_marker(header, marker_id + 2, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_walkway/status"
        text.pose.position = self._to_point(
            walkway.points[len(walkway.points) // 2]
        )
        text.pose.position.z += 2.2
        text.scale.z = 0.65
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "WALKWAY {}\n{}".format(walkway.lanelet_id, state)
        markers.append(text)
        return markers

    @staticmethod
    def _to_point(value):
        point = Point()
        point.x = float(value[0])
        point.y = float(value[1])
        point.z = float(value[2]) if len(value) > 2 else 0.0
        return point

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
