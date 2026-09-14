import math
from copy import deepcopy
from dataclasses import dataclass, replace as dataclass_replace

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
    point_in_polygon,
    project_point_to_polyline,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.path_utils import (
    downstream_turn_direction,
    ego_speed,
    insert_path_point,
    ordered_lane_ids_from_path,
    path_with_lane_id_positions,
    project_pose_to_path,
    set_velocity_from_index,
)


DECISION_SAFE = "Safe"
DECISION_UNSAFE = "Unsafe"
DECISION_OVER_PASS = "OverPassJudge"
DECISION_INTERNAL_ERROR = "InternalError"


@dataclass
class BlindSpotParameters:
    backward_attention_length: float = 100.0
    ttc_start_margin: float = 2.0
    ttc_end_margin: float = 1.0
    minimum_default_velocity: float = 1.388
    collision_judge_debounce: float = 0.5
    critical_stopline_margin: float = 1.0
    detect_car: bool = False
    detect_truck: bool = False
    detect_bus: bool = False
    detect_trailer: bool = False
    critical_deceleration: float = -6.0
    critical_jerk: float = -5.0
    semi_critical_deceleration: float = -4.0
    semi_critical_jerk: float = -5.0
    critical_ttc_upper: float = 3.0
    semi_critical_ttc_lower: float = 5.0
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    vehicle_width: float = 1.90
    minimum_attention_width: float = 0.65

    @classmethod
    def from_ros(cls, prefix="~blind_spot"):
        parameters = cls()
        names = {
            "backward_attention_length": "backward_attention_length",
            "ttc_start_margin": "ttc_start_margin",
            "ttc_end_margin": "ttc_end_margin",
            "minimum_default_velocity": "minimum_default_velocity",
            "collision_judge_debounce": "collision_judge_debounce",
            "critical_stopline_margin": "critical_stopline_margin",
            "detect_car": "detect_non_vru/car",
            "detect_truck": "detect_non_vru/truck",
            "detect_bus": "detect_non_vru/bus",
            "detect_trailer": "detect_non_vru/trailer",
            "critical_deceleration": "brake/critical/deceleration",
            "critical_jerk": "brake/critical/jerk",
            "semi_critical_deceleration": "brake/semi_critical/deceleration",
            "semi_critical_jerk": "brake/semi_critical/jerk",
            "critical_ttc_upper": "brake_for_ttc/critical_threshold_ub",
            "semi_critical_ttc_lower": "brake_for_ttc/semi_critical_threshold_lb",
        }
        for field_name, parameter_name in names.items():
            current = getattr(parameters, field_name)
            setattr(
                parameters,
                field_name,
                rospy.get_param("{}/{}".format(prefix, parameter_name), current),
            )
        return parameters


class BlindSpotModuleManager(SceneModuleManagerInterface):
    module_name = "blind_spot"

    def __init__(self, parameters=None):
        self.parameters = parameters or BlindSpotParameters.from_ros()
        self.safe_since = {}
        self.stop_latched = {}
        self.over_passed = set()

    def reset(self):
        self.safe_since.clear()
        self.stop_latched.clear()
        self.over_passed.clear()

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map
        if lanelet_map is None or not path_with_lane_id.points:
            result.status = "blind_spot: waiting for map/path"
            return result

        ordered_lane_ids = ordered_lane_ids_from_path(path_with_lane_id)
        route_lane_ids = set(ordered_lane_ids)
        targets = lanelet_map.road_lanelet_candidates(("left", "right"))
        targets = [item for item in targets if item.lanelet_id in route_lane_ids]

        # D-1처럼 신호등이 걸린 lanelet 자체가 직/좌/우로 갈라지는 구조라
        # turn_direction 태그를 직접 못 채우는 lanelet들을, 경로가 실제로
        # 이어지는 다음 lanelet의 turn_direction으로 판단해서 대상에 추가.
        # (2026-09-14, 신호등 모듈과 공통 문제로 확인되어 같은 방식 적용)
        directly_covered_ids = {item.lanelet_id for item in targets}
        for lane_id in ordered_lane_ids:
            if lane_id in directly_covered_ids:
                continue
            lanelet = lanelet_map.road_lanelet_by_id(lane_id)
            if lanelet is None or lanelet.turn_direction in ("left", "right"):
                continue
            resolved_direction = downstream_turn_direction(
                lanelet_map, lane_id, ordered_lane_ids
            )
            if resolved_direction not in ("left", "right"):
                continue
            resolved_tags = dict(lanelet.tags)
            resolved_tags["turn_direction"] = resolved_direction
            targets.append(dataclass_replace(lanelet, tags=resolved_tags))
        statuses = []
        for target in targets:
            if self._has_neighboring_turn_lanelet(target, lanelet_map):
                statuses.append("{}:SkippedNeighbor".format(target.lanelet_id))
                continue
            geometry = self._geometry(path_with_lane_id, target)
            if geometry is None:
                statuses.append("{}:{}".format(target.lanelet_id, DECISION_INTERNAL_ERROR))
                continue
            decision = self._decision(target, geometry, output_path, planner_data)
            statuses.append("{}:{}".format(target.lanelet_id, decision["state"]))
            result.markers.extend(
                self._make_markers(
                    path_with_lane_id.header, target, geometry, decision
                )
            )
            if decision["state"] != DECISION_UNSAFE:
                continue

            stop_index, stop_pose = insert_path_point(
                output_path, decision["stop_station"]
            )
            set_velocity_from_index(output_path, stop_index, 0.0)
            distance = decision["stop_station"] - geometry["ego_station"]
            factor_point = decision.get("collision_point") or geometry["entry_point"]
            result.modified = True
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance, factor_point)
            )
            result.planning_factors.append(
                self._make_planning_factor(
                    path_with_lane_id.header,
                    stop_pose,
                    distance,
                    target,
                    decision,
                    factor_point,
                )
            )

        result.status = "blind_spot[{}]".format(
            ", ".join(statuses) if statuses else "inactive"
        )
        return result

    def _geometry(self, path, target):
        path_xyz = path_with_lane_id_positions(path)
        if len(path_xyz) < 2:
            return None
        lengths = cumulative_lengths(path_xyz)
        entry = first_polyline_intersection(
            path_xyz, (target.left_bound[0], target.right_bound[0])
        )
        exit_point = first_polyline_intersection(
            path_xyz, (target.left_bound[-1], target.right_bound[-1])
        )
        lane_indices = [
            index
            for index, path_point in enumerate(path.points)
            if target.lanelet_id in path_point.lane_ids
        ]
        if not lane_indices:
            return None
        entry_station = (
            float(entry["station"])
            if entry is not None
            else float(lengths[max(lane_indices[0] - 1, 0)])
        )
        exit_station = (
            float(exit_point["station"])
            if exit_point is not None
            else float(lengths[min(lane_indices[-1] + 1, len(lengths) - 1)])
        )
        if exit_station <= entry_station + 0.1:
            return None

        raw_stop_station = max(
            0.0, entry_station - self.parameters.base_link_to_front
        )
        # Autoware searches its 0.2 m interpolated path for the first pose whose
        # vehicle footprint touches the turn-entry line.
        stop_station = min(
            lengths[-1], math.ceil(raw_stop_station / 0.2 - 1.0e-9) * 0.2
        )
        critical_station = max(
            stop_station,
            entry_station + 0.5 * self.parameters.vehicle_width
            - self.parameters.base_link_to_front
            - self.parameters.critical_stopline_margin,
        )
        attention_start = max(
            0.0, entry_station - self.parameters.backward_attention_length
        )
        half_lane_width = self._blind_side_half_width(target)
        near_offset = 0.5 * self.parameters.vehicle_width
        far_offset = max(
            half_lane_width,
            near_offset + self.parameters.minimum_attention_width,
        )
        direction = 1.0 if target.turn_direction == "left" else -1.0
        attention = self._offset_strip(
            path_xyz,
            attention_start,
            exit_station,
            direction * near_offset,
            direction * far_offset,
        )
        conflict = self._offset_strip(
            path_xyz,
            entry_station,
            exit_station,
            -0.5 * self.parameters.vehicle_width,
            0.5 * self.parameters.vehicle_width,
        )
        if len(attention) < 3 or len(conflict) < 3:
            return None
        return {
            "path_xyz": path_xyz,
            "lengths": lengths,
            "entry_station": entry_station,
            "exit_station": exit_station,
            "stop_station": stop_station,
            "critical_station": critical_station,
            "entry_point": point_at_station(path_xyz, entry_station),
            "exit_point": point_at_station(path_xyz, exit_station),
            "attention_polygon": attention,
            "conflict_polygon": conflict,
            "ego_station": 0.0,
            "near_offset": near_offset,
            "far_offset": far_offset,
        }

    def _decision(self, target, geometry, output_path, planner_data):
        lane_id = target.lanelet_id
        ego_station = self._ego_station(output_path, planner_data.odometry)
        geometry["ego_station"] = ego_station
        if lane_id in self.over_passed:
            return self._result(DECISION_OVER_PASS, None, "already over pass judge")

        unsafe = self._unsafe_objects(target, geometry, output_path, planner_data)
        now = float(planner_data.current_time)
        is_safe_now = not unsafe
        if not is_safe_now:
            self.stop_latched[lane_id] = True
            self.safe_since.pop(lane_id, None)
        elif self.stop_latched.get(lane_id, False):
            first_safe = self.safe_since.setdefault(lane_id, now)
            if now - first_safe >= self.parameters.collision_judge_debounce:
                self.stop_latched[lane_id] = False
                self.safe_since.pop(lane_id, None)

        is_safe = not self.stop_latched.get(lane_id, False)
        braking_distance = self._braking_distance(
            ego_speed(planner_data.odometry), planner_data.acceleration
        )
        distance_to_critical = geometry["critical_station"] - ego_station
        can_stop = distance_to_critical > braking_distance
        if is_safe:
            if distance_to_critical <= braking_distance:
                self.over_passed.add(lane_id)
            return self._result(DECISION_SAFE, None, "no critical blind-spot object")
        if not can_stop or ego_station > geometry["critical_station"]:
            self.over_passed.add(lane_id)
            return self._result(
                DECISION_OVER_PASS,
                None,
                "unsafe but too late to stop before conflict area",
            )

        collision = unsafe[0] if unsafe else None
        stop_station = geometry["stop_station"]
        if stop_station <= ego_station:
            stop_station = min(
                geometry["critical_station"], ego_station + braking_distance
            )
        return self._result(
            DECISION_UNSAFE,
            stop_station,
            "blind-spot TTC conflict" if not is_safe_now else "collision debounce hold",
            collision,
        )

    def _unsafe_objects(self, target, geometry, output_path, planner_data):
        dynamic_objects = planner_data.dynamic_objects
        if dynamic_objects is None:
            return []
        ego_interval = self._ego_passage_interval(output_path, geometry)
        unsafe = []
        for detected in dynamic_objects.objects:
            if not self._is_target_object(detected):
                continue
            position = detected.kinematics.initial_pose_with_covariance.pose.position
            current = (float(position.x), float(position.y), float(position.z))
            if not point_in_polygon(current, geometry["attention_polygon"]):
                continue
            if not self._is_vru(detected):
                lateral_gap = self._signed_lateral(
                    geometry["path_xyz"], current
                )[1]
                available = max(
                    geometry["far_offset"] - abs(lateral_gap), 0.0
                )
                if float(detected.shape.dimensions.y) >= available:
                    continue
            for predicted in detected.kinematics.predicted_paths:
                passage = self._object_passage_interval(
                    target, detected, predicted, geometry
                )
                if passage is None:
                    continue
                object_entry, object_exit, collision_point = passage
                critical = self._critical_time(
                    ego_interval, (object_entry, object_exit)
                )
                if critical is None:
                    continue
                unsafe.append(
                    {
                        "object": detected,
                        "predicted_path": predicted,
                        "object_entry": object_entry,
                        "object_exit": object_exit,
                        "ttc": critical,
                        "collision_point": collision_point,
                    }
                )
        return sorted(unsafe, key=lambda item: item["ttc"])

    def _ego_passage_interval(self, output_path, geometry):
        positions = [
            (point.pose.position.x, point.pose.position.y, point.pose.position.z)
            for point in output_path.points
        ]
        lengths = cumulative_lengths(positions)
        start = geometry["ego_station"]
        entry = max(geometry["entry_station"] - self.parameters.base_link_to_front, start)
        exit_station = geometry["exit_station"] + self.parameters.base_link_to_rear
        passing_time = 0.0
        entry_time = 0.0
        for index in range(len(lengths) - 1):
            segment_start = max(lengths[index], start)
            segment_end = lengths[index + 1]
            if segment_end <= start:
                continue
            velocity = max(
                0.5
                * (
                    float(output_path.points[index].longitudinal_velocity_mps)
                    + float(output_path.points[index + 1].longitudinal_velocity_mps)
                ),
                self.parameters.minimum_default_velocity,
            )
            if entry >= segment_start and entry <= segment_end:
                entry_time = passing_time + (entry - segment_start) / velocity
            consumed_end = min(segment_end, exit_station)
            if consumed_end > segment_start:
                passing_time += (consumed_end - segment_start) / velocity
            if segment_end >= exit_station:
                break
        return entry_time, passing_time

    def _object_passage_interval(self, target, detected, predicted, geometry):
        poses = list(predicted.path)
        if len(poses) < 2:
            return None
        time_step = self._duration_seconds(predicted.time_step)
        if time_step <= 0.0:
            return None
        samples = self._resample_prediction(poses, time_step, 0.1)
        object_half_width = max(float(detected.shape.dimensions.y) * 0.5, 0.2)
        side_sign = 1.0 if target.turn_direction == "left" else -1.0
        first_threshold = 0.5 * self.parameters.vehicle_width + object_half_width
        second_threshold = -0.5 * self.parameters.vehicle_width + object_half_width
        entry = None
        exit_time = None
        collision_point = None
        for elapsed, position in samples:
            station, lateral, _ = self._signed_lateral(
                geometry["path_xyz"], position
            )
            side_lateral = side_sign * lateral
            longitudinal_relevant = (
                geometry["entry_station"] - object_half_width
                <= station
                <= geometry["exit_station"] + object_half_width
            )
            crosses_entry_line = (
                abs(station - geometry["entry_station"])
                <= object_half_width + 0.3
            )
            if entry is None and longitudinal_relevant and (
                side_lateral <= first_threshold or crosses_entry_line
            ):
                entry = elapsed
                collision_point = position
            if entry is not None and longitudinal_relevant and side_lateral <= second_threshold:
                exit_time = elapsed
                break
        if entry is None or exit_time is None:
            return None
        return entry, exit_time, collision_point

    def _critical_time(self, ego_interval, object_interval):
        ego_entry, ego_exit = ego_interval
        object_entry, object_exit = object_interval
        if ego_entry > object_exit + self.parameters.ttc_end_margin:
            return None
        if object_entry - self.parameters.ttc_start_margin < ego_exit:
            return object_entry
        if ego_entry - self.parameters.ttc_end_margin < object_exit:
            return ego_entry
        return None

    def _is_target_object(self, detected):
        if not detected.classification:
            return False
        label = int(
            max(
                detected.classification,
                key=lambda item: float(item.probability),
            ).label
        )
        enabled = {
            ObjectClassification.PEDESTRIAN: True,
            ObjectClassification.BICYCLE: True,
            ObjectClassification.MOTORCYCLE: True,
            ObjectClassification.CAR: self.parameters.detect_car,
            ObjectClassification.TRUCK: self.parameters.detect_truck,
            ObjectClassification.BUS: self.parameters.detect_bus,
            ObjectClassification.TRAILER: self.parameters.detect_trailer,
        }
        return bool(enabled.get(label, False))

    @staticmethod
    def _is_vru(detected):
        if not detected.classification:
            return False
        label = int(detected.classification[0].label)
        return label in (
            ObjectClassification.PEDESTRIAN,
            ObjectClassification.BICYCLE,
            ObjectClassification.MOTORCYCLE,
        )

    def _braking_distance(self, speed, acceleration_message):
        speed = max(float(speed), 0.0)
        acceleration = 0.0
        if acceleration_message is not None:
            acceleration = float(acceleration_message.accel.accel.linear.x)
        deceleration = max(abs(self.parameters.critical_deceleration), 1.0e-3)
        reaction = max(acceleration, 0.0) * 0.1
        return speed * speed / (2.0 * deceleration) + reaction

    @staticmethod
    def _ego_station(path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _duration_seconds(duration):
        if hasattr(duration, "to_sec"):
            return float(duration.to_sec())
        return float(getattr(duration, "secs", 0.0)) + 1.0e-9 * float(
            getattr(duration, "nsecs", 0.0)
        )

    @staticmethod
    def _resample_prediction(poses, time_step, sample_step):
        horizon = time_step * float(len(poses) - 1)
        output = []
        step_count = max(int(math.ceil(horizon / sample_step)), 1)
        for step in range(step_count + 1):
            elapsed = min(step * sample_step, horizon)
            source = elapsed / time_step
            index = min(int(math.floor(source)), len(poses) - 2)
            ratio = max(0.0, min(source - index, 1.0))
            first = poses[index].position
            second = poses[index + 1].position
            output.append(
                (
                    elapsed,
                    (
                        float(first.x) + (float(second.x) - float(first.x)) * ratio,
                        float(first.y) + (float(second.y) - float(first.y)) * ratio,
                        float(first.z) + (float(second.z) - float(first.z)) * ratio,
                    ),
                )
            )
        return output

    @staticmethod
    def _signed_lateral(path_xyz, target):
        projection = project_point_to_polyline(path_xyz, target)
        if projection is None:
            return 0.0, 0.0, (0.0, 0.0, 0.0)
        station, distance, index, _, projected = projection
        following = min(index + 1, len(path_xyz) - 1)
        dx = float(path_xyz[following][0]) - float(path_xyz[index][0])
        dy = float(path_xyz[following][1]) - float(path_xyz[index][1])
        norm = max(math.hypot(dx, dy), 1.0e-9)
        lateral = (
            dx * (float(target[1]) - projected[1])
            - dy * (float(target[0]) - projected[0])
        ) / norm
        z = float(target[2]) if len(target) > 2 else 0.0
        return float(station), float(lateral), (projected[0], projected[1], z)

    @staticmethod
    def _blind_side_half_width(target):
        side = target.left_bound if target.turn_direction == "left" else target.right_bound
        distances = []
        for point in side:
            projection = project_point_to_polyline(target.centerline, point)
            if projection is not None:
                distances.append(float(projection[1]))
        return max(sum(distances) / len(distances), 1.0) if distances else 1.75

    @staticmethod
    def _offset_strip(path_xyz, start_station, end_station, first_offset, second_offset):
        lengths = cumulative_lengths(path_xyz)
        stations = [float(start_station)]
        stations.extend(
            value
            for value in lengths
            if start_station < value < end_station
        )
        stations.append(float(end_station))

        def shifted(station, offset):
            point = point_at_station(path_xyz, station)
            before = point_at_station(path_xyz, max(station - 0.25, 0.0))
            after = point_at_station(path_xyz, min(station + 0.25, lengths[-1]))
            dx = float(after[0]) - float(before[0])
            dy = float(after[1]) - float(before[1])
            norm = max(math.hypot(dx, dy), 1.0e-9)
            return (
                float(point[0]) - dy / norm * float(offset),
                float(point[1]) + dx / norm * float(offset),
                float(point[2]),
            )

        first = [shifted(station, first_offset) for station in stations]
        second = [shifted(station, second_offset) for station in stations]
        return tuple(first + list(reversed(second)))

    @staticmethod
    def _has_neighboring_turn_lanelet(target, lanelet_map):
        target_side = target.right_bound if target.turn_direction == "right" else target.left_bound
        target_start = target_side[0]
        for other in lanelet_map.road_lanelets.values():
            if other.lanelet_id == target.lanelet_id:
                continue
            if other.turn_direction != target.turn_direction:
                continue
            other_side = other.left_bound if target.turn_direction == "right" else other.right_bound
            if math.hypot(
                float(other_side[0][0]) - float(target_start[0]),
                float(other_side[0][1]) - float(target_start[1]),
            ) < 0.1:
                return True
        return False

    @staticmethod
    def _result(state, stop_station, detail, collision=None):
        result = {"state": state, "stop_station": stop_station, "detail": detail}
        if collision is not None:
            result.update(collision)
        return result

    @staticmethod
    def _make_stop_reason(stop_pose, distance, factor_point):
        factor = StopFactor()
        factor.stop_pose = deepcopy(stop_pose)
        factor.dist_to_stop_pose = float(distance)
        factor.stop_factor_points = [BlindSpotModuleManager._to_point(factor_point)]
        reason = StopReason()
        reason.reason = StopReason.BLIND_SPOT
        reason.stop_factors = [factor]
        return reason

    def _make_planning_factor(
        self, header, stop_pose, distance, target, decision, factor_point
    ):
        control = ControlPoint()
        control.pose = deepcopy(stop_pose)
        control.velocity = 0.0
        control.distance = float(distance)
        factor = PlanningFactor()
        factor.module = "blind_spot"
        factor.is_driving_forward = True
        factor.control_points = [control]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "lanelet_id={} {}".format(
            target.lanelet_id, decision["detail"]
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = False
        factor.safety_factors.detail = decision["state"]
        detected = decision.get("object")
        if detected is not None:
            safety = SafetyFactor()
            safety.type = SafetyFactor.OBJECT
            safety.object_id = deepcopy(detected.object_id)
            safety.predicted_path = deepcopy(decision["predicted_path"])
            safety.ttc_begin = float(decision["object_entry"])
            safety.ttc_end = float(decision["object_exit"])
            safety.points = [self._to_point(factor_point)]
            safety.is_safe = False
            safety.detail = "blind-spot TTC conflict"
            factor.safety_factors.factors = [safety]
        return factor

    def _make_markers(self, header, target, geometry, decision):
        marker_id = int(target.lanelet_id % 2000000000)
        markers = [
            self._polygon_marker(
                header,
                "bvp_blind_spot/attention_area",
                marker_id,
                geometry["attention_polygon"],
                (0.15, 0.55, 1.0, 0.25),
            ),
            self._polygon_marker(
                header,
                "bvp_blind_spot/conflict_area",
                marker_id + 1,
                geometry["conflict_polygon"],
                (1.0, 0.25, 0.05, 0.22),
            ),
        ]
        stop = self._base_marker(header, marker_id + 2, Marker.CUBE)
        stop.ns = "bvp_blind_spot/stop_line"
        stop.pose.position = self._to_point(
            point_at_station(geometry["path_xyz"], geometry["stop_station"])
        )
        stop.scale.x = 0.25
        stop.scale.y = 3.0
        stop.scale.z = 0.20
        stop.color.r = 1.0
        stop.color.g = 0.05
        stop.color.b = 0.15
        stop.color.a = 1.0
        markers.append(stop)
        predicted = decision.get("predicted_path")
        if predicted is not None:
            line = self._base_marker(header, marker_id + 3, Marker.LINE_STRIP)
            line.ns = "bvp_blind_spot/unsafe_prediction"
            line.scale.x = 0.30
            line.color.r = 1.0
            line.color.g = 0.1
            line.color.b = 0.1
            line.color.a = 1.0
            line.points = [deepcopy(pose.position) for pose in predicted.path]
            markers.append(line)
        text = self._base_marker(header, marker_id + 4, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_blind_spot/status"
        text.pose.position = self._to_point(geometry["entry_point"])
        text.pose.position.z += 2.5
        text.scale.z = 0.65
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "BLIND SPOT {} {}\n{}".format(
            target.lanelet_id,
            target.turn_direction.upper(),
            decision["state"],
        )
        markers.append(text)
        return markers

    @staticmethod
    def _polygon_marker(header, namespace, marker_id, polygon, color):
        marker = BlindSpotModuleManager._base_marker(
            header, marker_id, Marker.TRIANGLE_LIST
        )
        marker.ns = namespace
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        if len(polygon) >= 3:
            anchor = polygon[0]
            for index in range(1, len(polygon) - 1):
                marker.points.extend(
                    [
                        BlindSpotModuleManager._to_point(anchor),
                        BlindSpotModuleManager._to_point(polygon[index]),
                        BlindSpotModuleManager._to_point(polygon[index + 1]),
                    ]
                )
        return marker

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
