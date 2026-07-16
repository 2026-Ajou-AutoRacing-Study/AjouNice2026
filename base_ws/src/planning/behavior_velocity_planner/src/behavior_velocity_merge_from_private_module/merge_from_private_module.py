import math
from dataclasses import dataclass

import rospy

from autoware_internal_planning_msgs.msg import PlanningFactor
from tier4_planning_msgs.msg import StopReason

from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    first_polyline_intersection,
    point_at_station,
    point_in_polygon,
    segment_intersection,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.module_utils import (
    ego_station,
    line_marker,
    make_planning_factor,
    make_stop_reason,
    polygon_marker,
    text_marker,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_with_lane_id_positions,
    set_velocity_from_index,
)


@dataclass
class MergeFromPrivateParameters:
    stopline_margin: float = 3.0
    stop_duration_sec: float = 1.0
    stop_distance_threshold: float = 1.0
    stopped_velocity_threshold: float = 0.01
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    vehicle_width: float = 1.896
    path_interpolation_ds: float = 0.1

    @classmethod
    def from_ros(cls, prefix="~merge_from_private"):
        parameter = cls()
        for name in parameter.__dataclass_fields__:
            current = getattr(parameter, name)
            value = rospy.get_param("{}/{}".format(prefix, name), current)
            setattr(parameter, name, type(current)(value))
        return parameter


class MergeFromPrivateModuleManager(SceneModuleManagerInterface):
    module_name = "merge_from_private"

    def __init__(self, parameters=None):
        self.parameters = parameters or MergeFromPrivateParameters.from_ros()
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
            result.status = "merge_from_private: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2:
            result.status = "merge_from_private: input path is too short"
            return result

        route_order = self._route_order(path_with_lane_id)
        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        lengths = cumulative_lengths(path_xyz)
        current_station = ego_station(output_path, planner_data.odometry)
        speed = ego_speed(planner_data.odometry)
        now = float(planner_data.current_time)
        active = []
        for lane_id, next_lane_id in zip(route_order[:-1], route_order[1:]):
            lane = lanelet_map.road_lanelets.get(lane_id)
            next_lane = lanelet_map.road_lanelets.get(next_lane_id)
            if lane is None or next_lane is None:
                continue
            if not self._is_private(lane) or self._is_private(next_lane):
                continue
            transition = self._transition_station(
                path_with_lane_id, path_xyz, lengths, lane_id, lane
            )
            if transition is None:
                continue
            conflict_station = self._first_conflict_station(
                path_xyz,
                lengths,
                lanelet_map,
                set(route_order),
                lane_id,
                transition,
            )
            if conflict_station is None:
                stop_station = max(
                    0.0,
                    transition
                    - self.parameters.stopline_margin
                    - self.parameters.base_link_to_front,
                )
            else:
                stop_station = max(
                    0.0, conflict_station - self.parameters.stopline_margin
                )
            state = self.states.setdefault(
                lane_id, {"state": "STOP", "stopped_since": None}
            )
            distance = stop_station - current_station
            if current_station > transition + self.parameters.stop_distance_threshold:
                state.update(state="GO", stopped_since=None)
            elif state["state"] != "GO":
                near_line = abs(distance) <= self.parameters.stop_distance_threshold
                stopped = speed <= self.parameters.stopped_velocity_threshold
                if near_line and stopped:
                    if state["stopped_since"] is None:
                        state["stopped_since"] = now
                    elif now - state["stopped_since"] >= self.parameters.stop_duration_sec:
                        state["state"] = "GO"
                elif not stopped:
                    state["stopped_since"] = None

            stop_line = self._transition_line(lane)
            result.markers.extend(
                self._markers(output_path.header, lane, next_lane, stop_line, state["state"])
            )
            if state["state"] == "GO":
                active.append("{}->{}:GO".format(lane_id, next_lane_id))
                continue

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            detail = "private_lane={} public_lane={} mandatory_stop".format(
                lane_id, next_lane_id
            )
            result.modified = True
            result.stop_reasons.append(
                make_stop_reason(
                    StopReason.MERGE_FROM_PRIVATE_ROAD,
                    stop_pose,
                    distance,
                    [stop_line[0], stop_line[-1]],
                )
            )
            result.planning_factors.append(
                make_planning_factor(
                    self.module_name,
                    output_path.header,
                    stop_pose,
                    distance,
                    PlanningFactor.STOP,
                    detail,
                    safety_points=stop_line,
                )
            )
            active.append("{}->{}:STOP".format(lane_id, next_lane_id))

        result.status = (
            "merge_from_private: " + ", ".join(active)
            if active
            else "merge_from_private: no private-to-public transition on path"
        )
        return result

    @staticmethod
    def _route_order(path_with_lane_id):
        output = []
        for point in path_with_lane_id.points:
            for lane_id in point.lane_ids:
                lane_id = int(lane_id)
                if not output or output[-1] != lane_id:
                    output.append(lane_id)
        return output

    @staticmethod
    def _is_private(lane):
        return str(lane.tags.get("location", "")).lower() == "private"

    @staticmethod
    def _transition_station(path_with_lane_id, path_xyz, lengths, lane_id, lane):
        crossing = first_polyline_intersection(
            path_xyz, (lane.left_bound[-1], lane.right_bound[-1])
        )
        if crossing is not None:
            return float(crossing["station"])
        indices = [
            index
            for index, point in enumerate(path_with_lane_id.points)
            if lane_id in [int(value) for value in point.lane_ids]
        ]
        if not indices:
            return None
        return float(lengths[indices[-1]])

    @staticmethod
    def _transition_line(lane):
        return (lane.left_bound[-1], lane.right_bound[-1])

    def _first_conflict_station(
        self, path_xyz, lengths, lanelet_map, route_ids, lane_id, transition
    ):
        private_indices = []
        private_lane = lanelet_map.road_lanelets.get(lane_id)
        if private_lane is None:
            return None
        for point in private_lane.centerline:
            from behavior_velocity_planner_common.geometry import project_point_to_polyline

            projection = project_point_to_polyline(path_xyz, point)
            if projection is not None:
                private_indices.append(float(projection[0]))
        if not private_indices:
            return None
        search_start = max(min(private_indices), 0.0)
        search_end = min(max(transition, max(private_indices)), lengths[-1])
        candidates = [
            lane
            for candidate_id, lane in lanelet_map.road_lanelets.items()
            if candidate_id not in route_ids
            and lane.tags.get("turn_direction") in ("left", "right", "straight")
        ]
        if not candidates:
            return None

        station = search_start
        step = max(self.parameters.path_interpolation_ds, 0.02)
        while station <= search_end + 1.0e-6:
            footprint = self._footprint(path_xyz, lengths[-1], station)
            if any(
                self._polygons_intersect(footprint, candidate.polygon)
                for candidate in candidates
            ):
                return station
            station += step
        return None

    def _footprint(self, path_xyz, path_length, station):
        center = point_at_station(path_xyz, station)
        ahead = point_at_station(
            path_xyz,
            min(station + max(self.parameters.path_interpolation_ds, 0.05), path_length),
        )
        yaw = math.atan2(ahead[1] - center[1], ahead[0] - center[0])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        half_width = 0.5 * self.parameters.vehicle_width
        corners = (
            (self.parameters.base_link_to_front, half_width),
            (self.parameters.base_link_to_front, -half_width),
            (-self.parameters.base_link_to_rear, -half_width),
            (-self.parameters.base_link_to_rear, half_width),
        )
        return tuple(
            (
                center[0] + cosine * longitudinal - sine * lateral,
                center[1] + sine * longitudinal + cosine * lateral,
                center[2],
            )
            for longitudinal, lateral in corners
        )

    @staticmethod
    def _polygons_intersect(first, second):
        for first_a, first_b in zip(first, first[1:] + first[:1]):
            for second_a, second_b in zip(second, second[1:] + second[:1]):
                if segment_intersection(first_a, first_b, second_a, second_b):
                    return True
        return any(point_in_polygon(item, second) for item in first) or any(
            point_in_polygon(item, first) for item in second
        )

    @staticmethod
    def _markers(header, private_lane, public_lane, stop_line, state):
        private_id = private_lane.lanelet_id
        return [
            polygon_marker(
                header,
                "merge_from_private/private_lane",
                private_id,
                private_lane.polygon,
                (0.72, 0.2, 0.95, 0.28),
            ),
            polygon_marker(
                header,
                "merge_from_private/public_lane",
                public_lane.lanelet_id,
                public_lane.polygon,
                (0.1, 0.75, 0.95, 0.22),
            ),
            line_marker(
                header,
                "merge_from_private/stop_line",
                private_id,
                stop_line,
                (1.0, 0.15, 0.1, 1.0),
                0.5,
            ),
            text_marker(
                header,
                "merge_from_private/label",
                private_id,
                stop_line[0],
                "PRIVATE -> PUBLIC {}".format(state),
                (1.0, 0.9, 0.2, 1.0),
            ),
        ]
