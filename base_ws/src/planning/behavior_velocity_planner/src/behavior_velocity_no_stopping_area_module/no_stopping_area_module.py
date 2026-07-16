import math
from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import (
    ControlPoint,
    PlanningFactor,
    SafetyFactor,
)
from autoware_perception_msgs.msg import ObjectClassification
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    first_polyline_intersection,
    point_at_station,
    polyline_polygon_intersections,
    project_point_to_polyline,
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


@dataclass
class NoStoppingAreaParameters:
    state_clear_time: float = 2.0
    stuck_vehicle_vel_thr: float = 0.833
    stop_margin: float = 0.0
    dead_line_margin: float = 1.0
    stop_line_margin: float = 1.0
    detection_area_length: float = 200.0
    stuck_vehicle_front_margin: float = 6.0
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    vehicle_length: float = 4.89
    vehicle_width: float = 1.90
    max_accel: float = -2.8
    max_jerk: float = -5.0

    @classmethod
    def from_ros(cls, prefix="~no_stopping_area"):
        parameter = cls()
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            value = rospy.get_param("{}/{}".format(prefix, field_name), current)
            setattr(parameter, field_name, type(current)(value))
        return parameter


class NoStoppingAreaModuleManager(SceneModuleManagerInterface):
    module_name = "no_stopping_area"

    def __init__(self, parameters=None):
        self.parameters = parameters or NoStoppingAreaParameters.from_ros()
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
            result.status = "no_stopping_area: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "no_stopping_area: input path is too short"
            return result

        input_xyz = path_with_lane_id_positions(path_with_lane_id)
        route_lane_ids = {
            int(lane_id)
            for path_point in path_with_lane_id.points
            for lane_id in path_point.lane_ids
        }
        ego_station = self._ego_station(output_path, planner_data.odometry)
        now = float(planner_data.current_time)
        active = []

        candidates = lanelet_map.no_stopping_area_candidates()
        candidate_ids = {item.regulatory_id for item in candidates}
        self.states = {
            key: value for key, value in self.states.items() if key in candidate_ids
        }
        for area in candidates:
            if area.lane_ids and route_lane_ids.isdisjoint(area.lane_ids):
                continue
            crossings = polyline_polygon_intersections(input_xyz, area.points)
            if not crossings:
                continue
            entry_station = float(crossings[0]["station"])
            exit_station = float(crossings[-1]["station"])
            if exit_station <= entry_station and len(crossings) == 1:
                exit_station = cumulative_lengths(input_xyz)[-1]

            stop_line_station = self._stop_line_station(
                input_xyz, area, entry_station
            )
            stop_station = max(
                0.0,
                stop_line_station
                - self.parameters.stop_margin
                - self.parameters.base_link_to_front,
            )
            state = self.states.setdefault(
                area.regulatory_id,
                {"state": "GO", "last_prohibited": None, "pass_judged": False},
            )
            if ego_station > stop_station + self.parameters.dead_line_margin:
                state.update(state="GO", pass_judged=False)
                continue

            decision = self._entry_prohibited(
                output_path,
                planner_data,
                entry_station,
                exit_station,
                ego_station,
            )
            prohibited = decision is not None
            if prohibited:
                state["state"] = "STOP"
                state["last_prohibited"] = now
            elif (
                state["state"] == "STOP"
                and state["last_prohibited"] is not None
                and now - state["last_prohibited"] < self.parameters.state_clear_time
            ):
                prohibited = True
                decision = {"kind": "state_hold", "object": None, "point": None}
            else:
                state["state"] = "GO"
                state["last_prohibited"] = None
                state["pass_judged"] = False

            if not prohibited:
                active.append("{}:GO".format(area.regulatory_id))
                result.markers.extend(self._make_markers(output_path.header, area, None, None))
                continue

            if not self._is_stoppable(
                output_path,
                planner_data,
                ego_station,
                stop_station,
                state,
            ):
                state["state"] = "GO"
                active.append("{}:PASS".format(area.regulatory_id))
                result.markers.extend(self._make_markers(output_path.header, area, None, decision))
                continue

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            distance = stop_station - ego_station
            factor_point = decision.get("point") or (
                point_at_station(input_xyz, entry_station) or input_xyz[0]
            )
            result.modified = True
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance, factor_point)
            )
            result.planning_factors.append(
                self._make_planning_factor(
                    output_path.header,
                    stop_pose,
                    distance,
                    area,
                    decision,
                    factor_point,
                )
            )
            result.markers.extend(
                self._make_markers(output_path.header, area, stop_pose, decision)
            )
            active.append("{}:STOP({})".format(area.regulatory_id, decision["kind"]))

        result.status = (
            "no_stopping_area: " + ", ".join(active)
            if active
            else "no_stopping_area: no regulatory area on path"
        )
        return result

    def _entry_prohibited(
        self, output_path, planner_data, entry_station, exit_station, ego_station
    ):
        stuck_end = min(
            entry_station + self.parameters.detection_area_length,
            exit_station
            + self.parameters.stop_line_margin
            + self.parameters.vehicle_length
            + self.parameters.stuck_vehicle_front_margin,
        )
        objects = getattr(planner_data.dynamic_objects, "objects", [])
        path_xyz = path_positions(output_path)
        for detected in objects:
            if not self._is_vehicle(detected):
                continue
            if self._object_speed(detected) > self.parameters.stuck_vehicle_vel_thr:
                continue
            position = self._object_position(detected)
            projection = project_point_to_polyline(path_xyz, position)
            if projection is None:
                continue
            station, lateral_distance = float(projection[0]), float(projection[1])
            half_width = max(float(detected.shape.dimensions.y) * 0.5, 0.5)
            if (
                max(entry_station, ego_station) <= station <= stuck_end
                and lateral_distance
                <= self.parameters.vehicle_width * 0.5 + half_width
            ):
                return {
                    "kind": "stuck_vehicle",
                    "object": detected,
                    "point": position,
                }

        stop_end = min(
            entry_station + self.parameters.detection_area_length,
            exit_station
            + self.parameters.stop_line_margin
            + self.parameters.stop_margin
            + self.parameters.base_link_to_rear,
        )
        lengths = cumulative_lengths(path_xyz)
        for index, path_point in enumerate(output_path.points):
            if lengths[index] >= lengths[-1] - 1.0:
                continue
            if float(path_point.longitudinal_velocity_mps) > 1.0e-6:
                continue
            if max(entry_station, ego_station) <= lengths[index] <= stop_end:
                point = path_xyz[index]
                return {"kind": "stop_velocity", "object": None, "point": point}
        return None

    def _stop_line_station(self, path_xyz, area, entry_station):
        if area.stop_line is not None:
            crossing = first_polyline_intersection(path_xyz, area.stop_line.points)
            if crossing is not None:
                return float(crossing["station"])
        return max(0.0, entry_station - self.parameters.stop_line_margin)

    def _is_stoppable(self, output_path, planner_data, ego_station, stop_station, state):
        if state["pass_judged"]:
            return bool(state.get("is_stoppable", True))
        distance = stop_station - ego_station
        speed = ego_speed(planner_data.odometry)
        acceleration = 0.0
        if planner_data.acceleration is not None:
            acceleration = float(planner_data.acceleration.accel.accel.linear.x)
        required = self._stopping_distance(speed, acceleration)
        if required < distance:
            return True
        state["pass_judged"] = True
        state["is_stoppable"] = speed < 2.0
        return bool(state["is_stoppable"])

    def _stopping_distance(self, speed, acceleration):
        speed = max(float(speed), 0.0)
        target_acceleration = min(float(self.parameters.max_accel), -1.0e-3)
        jerk = min(float(self.parameters.max_jerk), -1.0e-3)
        acceleration = min(float(acceleration), 0.0)
        if acceleration <= target_acceleration:
            return speed * speed / (-2.0 * target_acceleration)
        ramp_time = max((target_acceleration - acceleration) / jerk, 0.0)
        ramp_distance = max(
            speed * ramp_time
            + 0.5 * acceleration * ramp_time**2
            + jerk * ramp_time**3 / 6.0,
            0.0,
        )
        ramp_speed = max(
            speed + acceleration * ramp_time + 0.5 * jerk * ramp_time**2,
            0.0,
        )
        return ramp_distance + ramp_speed**2 / (-2.0 * target_acceleration)

    @staticmethod
    def _is_vehicle(detected):
        labels = sorted(
            detected.classification,
            key=lambda item: float(item.probability),
            reverse=True,
        )
        target = {
            ObjectClassification.CAR,
            ObjectClassification.TRUCK,
            ObjectClassification.BUS,
            ObjectClassification.TRAILER,
            ObjectClassification.MOTORCYCLE,
        }
        return bool(labels and int(labels[0].label) in target)

    @staticmethod
    def _object_speed(detected):
        velocity = detected.kinematics.initial_twist_with_covariance.twist.linear
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    @staticmethod
    def _object_position(detected):
        point = detected.kinematics.initial_pose_with_covariance.pose.position
        return float(point.x), float(point.y), float(point.z)

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
        factor.stop_factor_points = [NoStoppingAreaModuleManager._to_point(factor_point)]
        reason = StopReason()
        reason.reason = StopReason.NO_STOPPING_AREA
        reason.stop_factors = [factor]
        return reason

    def _make_planning_factor(
        self, header, stop_pose, distance, area, decision, factor_point
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
        factor.detail = "regulatory_id={} {}".format(
            area.regulatory_id, decision["kind"]
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = False
        factor.safety_factors.detail = decision["kind"]
        detected = decision.get("object")
        if detected is not None:
            safety = SafetyFactor()
            safety.type = SafetyFactor.OBJECT
            safety.object_id = deepcopy(detected.object_id)
            safety.points = [self._to_point(factor_point)]
            safety.is_safe = False
            safety.detail = "stuck vehicle in no stopping area"
            factor.safety_factors.factors = [safety]
        return factor

    def _make_markers(self, header, area, stop_pose, decision):
        marker_id = int(area.regulatory_id % 2000000000)
        polygon = self._polygon_marker(
            header,
            "bvp_no_stopping_area/area",
            marker_id,
            area.points,
            (1.0, 0.35, 0.05, 0.25),
        )
        markers = [polygon]
        if stop_pose is not None:
            stop = self._base_marker(header, marker_id + 1, Marker.CUBE)
            stop.ns = "bvp_no_stopping_area/stop"
            stop.pose = deepcopy(stop_pose)
            stop.scale.x = 0.25
            stop.scale.y = 5.0
            stop.scale.z = 1.5
            stop.color.r = 1.0
            stop.color.a = 0.9
            markers.append(stop)
        if decision is not None and decision.get("point") is not None:
            point = self._base_marker(header, marker_id + 2, Marker.SPHERE)
            point.ns = "bvp_no_stopping_area/factor"
            point.pose.position = self._to_point(decision["point"])
            point.pose.orientation.w = 1.0
            point.scale.x = point.scale.y = point.scale.z = 0.8
            point.color.r = 1.0
            point.color.g = 0.1
            point.color.a = 1.0
            markers.append(point)
        return markers

    @staticmethod
    def _polygon_marker(header, namespace, marker_id, points, color):
        marker = NoStoppingAreaModuleManager._base_marker(
            header, marker_id, Marker.TRIANGLE_LIST
        )
        marker.ns = namespace
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        anchor = points[0]
        for index in range(1, len(points) - 1):
            marker.points.extend(
                [
                    NoStoppingAreaModuleManager._to_point(anchor),
                    NoStoppingAreaModuleManager._to_point(points[index]),
                    NoStoppingAreaModuleManager._to_point(points[index + 1]),
                ]
            )
        return marker

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
        if hasattr(value, "x"):
            point.x = float(value.x)
            point.y = float(value.y)
            point.z = float(value.z)
        else:
            point.x = float(value[0])
            point.y = float(value[1])
            point.z = float(value[2]) if len(value) > 2 else 0.0
        return point
