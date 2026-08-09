#!/usr/bin/env python3

import math
from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass, field
from itertools import combinations

import rospy

from autoware_perception_msgs.msg import ObjectClassification
from tier4_rtc_msgs.msg import Module, State

from geometry_msgs.msg import Point

from bpp_map_tools import (
    _point_in_polygon,
    project_point_to_polyline,
    project_point_to_polyline_with_point,
)
from bpp_path_tools import (
    clamp,
    object_pose,
    object_speed,
    path_cumulative_lengths,
    project_pose_to_path,
    quaternion_from_yaw,
    set_velocity_window,
    shifted_pose,
    uuid_to_string,
    yaw_from_quaternion,
)
from rtc_interface_ros1 import RTCInterfaceROS1, RTCSceneController


VEHICLE_LABELS = {
    ObjectClassification.CAR,
    ObjectClassification.TRUCK,
    ObjectClassification.BUS,
    ObjectClassification.TRAILER,
    ObjectClassification.MOTORCYCLE,
}


def _param_name(prefix, name):
    return "{}/{}".format(prefix.rstrip("/"), name.lstrip("/"))


def _get_param(prefix, name, default):
    return rospy.get_param(_param_name(prefix, name), default)


def _as_bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def _stamp_seconds(stamp):
    if stamp is None:
        return rospy.Time.now().to_sec()
    if hasattr(stamp, "to_sec"):
        return stamp.to_sec()
    return float(stamp)


def _distance_2d(first, second):
    return math.hypot(first.x - second.x, first.y - second.y)


def _quintic_smoothstep(start, finish, value):
    if finish <= start:
        return 1.0 if value >= finish else 0.0
    ratio = clamp((value - start) / (finish - start), 0.0, 1.0)
    return ratio * ratio * ratio * (10.0 + ratio * (-15.0 + 6.0 * ratio))


def _highest_probability_label(predicted_object):
    if not predicted_object.classification:
        return ObjectClassification.UNKNOWN
    return int(max(predicted_object.classification, key=lambda item: item.probability).label)


def _object_dimensions(predicted_object):
    dimensions = predicted_object.shape.dimensions
    length = max(float(dimensions.x), 0.1)
    width = max(float(dimensions.y), 0.1)
    if predicted_object.shape.footprint.points:
        xs = [float(point.x) for point in predicted_object.shape.footprint.points]
        ys = [float(point.y) for point in predicted_object.shape.footprint.points]
        length = max(max(xs) - min(xs), length)
        width = max(max(ys) - min(ys), width)
    return length, width


def _path_tangent_yaw(path, index):
    if not path.points:
        return 0.0
    index = int(clamp(index, 0, len(path.points) - 1))
    if len(path.points) == 1:
        return yaw_from_quaternion(path.points[0].point.pose.orientation)
    before_index = max(index - 1, 0)
    after_index = min(index + 1, len(path.points) - 1)
    before = path.points[before_index].point.pose.position
    after = path.points[after_index].point.pose.position
    if _distance_2d(before, after) < 1.0e-6:
        return yaw_from_quaternion(path.points[index].point.pose.orientation)
    return math.atan2(after.y - before.y, after.x - before.x)


def _projected_object_half_extents(predicted_object, path_yaw):
    length, width = _object_dimensions(predicted_object)
    object_yaw = yaw_from_quaternion(object_pose(predicted_object).orientation)
    relative_yaw = object_yaw - path_yaw
    longitudinal = 0.5 * (
        abs(math.cos(relative_yaw)) * length + abs(math.sin(relative_yaw)) * width
    )
    lateral = 0.5 * (
        abs(math.sin(relative_yaw)) * length + abs(math.cos(relative_yaw)) * width
    )
    return longitudinal, lateral


def _lateral_jerk_distance(shift_length, jerk, velocity):
    """Approximate Autoware's constant-jerk lateral shift distance."""
    if abs(shift_length) < 1.0e-6:
        return 0.0
    jerk = max(abs(float(jerk)), 1.0e-3)
    velocity = max(abs(float(velocity)), 0.1)
    phase_time = (abs(float(shift_length)) / (2.0 * jerk)) ** (1.0 / 3.0)
    return 4.0 * phase_time * velocity


def _point_segment_distance(point, first, second):
    vx = second[0] - first[0]
    vy = second[1] - first[1]
    length_squared = vx * vx + vy * vy
    if length_squared < 1.0e-12:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    ratio = clamp(
        ((point[0] - first[0]) * vx + (point[1] - first[1]) * vy)
        / length_squared,
        0.0,
        1.0,
    )
    closest_x = first[0] + ratio * vx
    closest_y = first[1] + ratio * vy
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def _orientation(first, second, third):
    value = (second[1] - first[1]) * (third[0] - second[0]) - (
        second[0] - first[0]
    ) * (third[1] - second[1])
    if abs(value) < 1.0e-9:
        return 0
    return 1 if value > 0.0 else 2


def _point_on_segment(first, point, second):
    return (
        min(first[0], second[0]) - 1.0e-9
        <= point[0]
        <= max(first[0], second[0]) + 1.0e-9
        and min(first[1], second[1]) - 1.0e-9
        <= point[1]
        <= max(first[1], second[1]) + 1.0e-9
    )


def _segments_intersect(first_a, first_b, second_a, second_b):
    orientations = (
        _orientation(first_a, first_b, second_a),
        _orientation(first_a, first_b, second_b),
        _orientation(second_a, second_b, first_a),
        _orientation(second_a, second_b, first_b),
    )
    if orientations[0] != orientations[1] and orientations[2] != orientations[3]:
        return True
    return (
        (orientations[0] == 0 and _point_on_segment(first_a, second_a, first_b))
        or (orientations[1] == 0 and _point_on_segment(first_a, second_b, first_b))
        or (orientations[2] == 0 and _point_on_segment(second_a, first_a, second_b))
        or (orientations[3] == 0 and _point_on_segment(second_a, first_b, second_b))
    )


def _polygons_intersect(first, second):
    if not first or not second:
        return False
    if _point_in_polygon(first[0], second) or _point_in_polygon(second[0], first):
        return True
    first_edges = zip(first, first[1:] + first[:1])
    second_edges = list(zip(second, second[1:] + second[:1]))
    return any(
        _segments_intersect(first_a, first_b, second_a, second_b)
        for first_a, first_b in first_edges
        for second_a, second_b in second_edges
    )


def _polygon_distance(first, second):
    if _polygons_intersect(first, second):
        return 0.0
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    if not first_edges or not second_edges:
        return float("inf")
    first_to_second = min(
        _point_segment_distance(point, edge_start, edge_end)
        for point in first
        for edge_start, edge_end in second_edges
    )
    second_to_first = min(
        _point_segment_distance(point, edge_start, edge_end)
        for point in second
        for edge_start, edge_end in first_edges
    )
    return min(first_to_second, second_to_first)


@dataclass
class ObjectPolicy:
    moving_speed_threshold: float
    moving_time_threshold: float
    soft_margin: float
    hard_margin: float
    parked_hard_margin: float
    longitudinal_margin: float
    envelope_buffer: float


@dataclass
class StaticAvoidanceParameters:
    use_lane_type: str = "opposite_direction_lane"
    use_intersection_areas: bool = True
    use_static_detection_area: bool = False
    min_forward_distance: float = 50.0
    max_forward_distance: float = 150.0
    backward_distance: float = 10.0
    max_compensation_time: float = 2.0
    parked_offset_threshold: float = 1.0
    parked_shiftable_ratio: float = 0.8
    min_road_shoulder_width: float = 0.5
    ambiguous_policy: str = "manual"
    ambiguous_stopped_time: float = 3.0
    ambiguous_moving_distance: float = 1.0
    parking_violation_policy: str = "ignore"
    parking_violation_border_distance: float = 0.8
    adjacent_lane_stop_policy: str = "auto"
    close_vehicle_policy: str = "ignore"
    unstable_classification_time: float = 2.0
    enable_safety_check: bool = True
    check_other_objects: bool = True
    check_all_predicted_paths: bool = False
    safety_backward_distance: float = 100.0
    hysteresis_expand_rate: float = 1.5
    hysteresis_safe_count: int = 3
    safety_lateral_threshold: float = 2.0
    safety_longitudinal_threshold: float = 3.0
    min_prepare_time: float = 1.0
    max_prepare_time: float = 2.0
    min_prepare_distance: float = 1.0
    min_slow_down_speed: float = 1.38
    nominal_avoidance_speed: float = 8.33
    nominal_return_speed: float = 8.33
    soft_drivable_bound_margin: float = 0.3
    hard_drivable_bound_margin: float = 0.3
    max_right_shift_length: float = 5.0
    max_left_shift_length: float = 5.0
    max_deviation_from_lane: float = 0.2
    lateral_execution_threshold: float = 0.09
    quantize_size: float = 0.1
    resample_interval_for_planning: float = 0.3
    avoid_min_jerk: float = 0.2
    return_min_jerk: float = 0.2
    return_reanchor_lateral_error_threshold: float = 0.3
    lateral_max_jerk: float = 1.0
    return_approval_ratio: float = 0.5
    stop_max_distance: float = 20.0
    stop_buffer: float = 1.0
    nominal_deceleration: float = -1.0
    max_acceleration: float = 0.5
    path_generation_method: str = "shift_line_base"
    approval_policy: str = "per_shift_line"
    lateral_margin_policy: str = "best_effort"
    candidate_turn_signal_policy: str = "none"
    turn_signal_on_swerving: bool = True
    turn_signal_shift_length_threshold: float = 0.3
    turn_signal_remaining_distance_to_bound_threshold: float = 1.3
    default_avoid_direction: str = "right"
    vehicle_width: float = 1.896
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.1
    enable_rtc_left: bool = False
    enable_rtc_right: bool = False
    debug_log_unknown_objects: bool = False
    target_labels: set = field(default_factory=set)
    object_policies: dict = field(default_factory=dict)

    @classmethod
    def from_ros(cls, prefix="~static_obstacle_avoidance"):
        params = cls()
        params.use_lane_type = str(
            _get_param(prefix, "use_lane_type", params.use_lane_type)
        )
        params.debug_log_unknown_objects = _as_bool(
            _get_param(
                prefix,
                "debug/log_unknown_objects",
                params.debug_log_unknown_objects,
            )
        )
        params.use_intersection_areas = _as_bool(
            _get_param(
                prefix,
                "use_intersection_areas",
                params.use_intersection_areas,
            )
        )
        params.use_static_detection_area = _as_bool(
            _get_param(
                prefix,
                "target_filtering/detection_area/static",
                params.use_static_detection_area,
            )
        )
        params.min_forward_distance = float(
            _get_param(
                prefix,
                "target_filtering/detection_area/min_forward_distance",
                params.min_forward_distance,
            )
        )
        params.max_forward_distance = float(
            _get_param(
                prefix,
                "target_filtering/detection_area/max_forward_distance",
                params.max_forward_distance,
            )
        )
        params.backward_distance = float(
            _get_param(
                prefix,
                "target_filtering/detection_area/backward_distance",
                params.backward_distance,
            )
        )
        params.max_compensation_time = float(
            _get_param(
                prefix,
                "target_filtering/max_compensation_time",
                params.max_compensation_time,
            )
        )
        params.parked_offset_threshold = float(
            _get_param(
                prefix,
                "target_filtering/parked_vehicle/th_offset_from_centerline",
                params.parked_offset_threshold,
            )
        )
        params.parked_shiftable_ratio = float(
            _get_param(
                prefix,
                "target_filtering/parked_vehicle/th_shiftable_ratio",
                params.parked_shiftable_ratio,
            )
        )
        params.min_road_shoulder_width = float(
            _get_param(
                prefix,
                "target_filtering/parked_vehicle/min_road_shoulder_width",
                params.min_road_shoulder_width,
            )
        )
        params.ambiguous_policy = str(
            _get_param(
                prefix,
                "target_filtering/avoidance_for_ambiguous_vehicle/policy",
                params.ambiguous_policy,
            )
        ).lower()
        params.ambiguous_stopped_time = float(
            _get_param(
                prefix,
                "target_filtering/avoidance_for_ambiguous_vehicle/condition/th_stopped_time",
                params.ambiguous_stopped_time,
            )
        )
        params.ambiguous_moving_distance = float(
            _get_param(
                prefix,
                "target_filtering/avoidance_for_ambiguous_vehicle/condition/th_moving_distance",
                params.ambiguous_moving_distance,
            )
        )
        params.enable_safety_check = _as_bool(
            _get_param(prefix, "safety_check/enable", params.enable_safety_check)
        )
        params.check_other_objects = _as_bool(
            _get_param(
                prefix,
                "safety_check/check_other_object",
                params.check_other_objects,
            )
        )
        params.check_all_predicted_paths = _as_bool(
            _get_param(
                prefix,
                "safety_check/check_all_predicted_path",
                params.check_all_predicted_paths,
            )
        )
        params.safety_backward_distance = float(
            _get_param(
                prefix,
                "safety_check/safety_check_backward_distance",
                params.safety_backward_distance,
            )
        )
        params.hysteresis_expand_rate = float(
            _get_param(
                prefix,
                "safety_check/hysteresis_factor_expand_rate",
                params.hysteresis_expand_rate,
            )
        )
        params.hysteresis_safe_count = int(
            _get_param(
                prefix,
                "safety_check/hysteresis_factor_safe_count",
                params.hysteresis_safe_count,
            )
        )
        params.safety_lateral_threshold = float(
            _get_param(
                prefix,
                "safety_check/lateral_distance_max_threshold",
                params.safety_lateral_threshold,
            )
        )
        params.safety_longitudinal_threshold = float(
            _get_param(
                prefix,
                "safety_check/longitudinal_distance_min_threshold",
                params.safety_longitudinal_threshold,
            )
        )
        params.min_prepare_time = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/min_prepare_time",
                params.min_prepare_time,
            )
        )
        params.max_prepare_time = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/max_prepare_time",
                params.max_prepare_time,
            )
        )
        params.min_prepare_distance = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/min_prepare_distance",
                params.min_prepare_distance,
            )
        )
        params.min_slow_down_speed = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/min_slow_down_speed",
                params.min_slow_down_speed,
            )
        )
        params.nominal_avoidance_speed = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/nominal_avoidance_speed",
                params.nominal_avoidance_speed,
            )
        )
        params.nominal_return_speed = float(
            _get_param(
                prefix,
                "avoidance/longitudinal/nominal_return_speed",
                params.nominal_avoidance_speed,
            )
        )
        params.soft_drivable_bound_margin = float(
            _get_param(
                prefix,
                "avoidance/lateral/soft_drivable_bound_margin",
                params.soft_drivable_bound_margin,
            )
        )
        params.hard_drivable_bound_margin = float(
            _get_param(
                prefix,
                "avoidance/lateral/hard_drivable_bound_margin",
                params.hard_drivable_bound_margin,
            )
        )
        params.max_right_shift_length = float(
            _get_param(
                prefix,
                "avoidance/lateral/max_right_shift_length",
                params.max_right_shift_length,
            )
        )
        params.max_left_shift_length = float(
            _get_param(
                prefix,
                "avoidance/lateral/max_left_shift_length",
                params.max_left_shift_length,
            )
        )
        params.max_deviation_from_lane = float(
            _get_param(
                prefix,
                "avoidance/lateral/max_deviation_from_lane",
                params.max_deviation_from_lane,
            )
        )
        params.lateral_execution_threshold = float(
            _get_param(
                prefix,
                "avoidance/lateral/th_avoid_execution",
                params.lateral_execution_threshold,
            )
        )
        params.return_approval_ratio = float(
            _get_param(
                prefix,
                "avoidance/lateral/ratio_for_return_shift_approval",
                params.return_approval_ratio,
            )
        )
        params.quantize_size = float(
            _get_param(prefix, "shift_line_pipeline/trim/quantize_size", params.quantize_size)
        )
        params.resample_interval_for_planning = max(
            float(
                _get_param(
                    prefix,
                    "resample_interval_for_planning",
                    params.resample_interval_for_planning,
                )
            ),
            0.1,
        )
        avoid_jerks = _get_param(
            prefix,
            "constraints/lateral/min_jerk_values/avoid",
            [params.avoid_min_jerk] * 3,
        )
        return_jerks = _get_param(
            prefix,
            "constraints/lateral/min_jerk_values/return",
            [params.return_min_jerk] * 3,
        )
        max_jerks = _get_param(
            prefix,
            "constraints/lateral/max_jerk_values",
            [params.lateral_max_jerk] * 3,
        )
        params.avoid_min_jerk = float(avoid_jerks[0])
        params.return_min_jerk = float(return_jerks[0])
        params.return_reanchor_lateral_error_threshold = max(
            float(
                _get_param(
                    prefix,
                    "avoidance/lateral/return_reanchor_lateral_error_threshold",
                    params.return_reanchor_lateral_error_threshold,
                )
            ),
            0.0,
        )
        params.lateral_max_jerk = float(max_jerks[0])
        params.stop_max_distance = float(
            _get_param(prefix, "stop/max_distance", params.stop_max_distance)
        )
        params.stop_buffer = float(_get_param(prefix, "stop/stop_buffer", params.stop_buffer))
        params.nominal_deceleration = float(
            _get_param(
                prefix,
                "constraints/longitudinal/nominal_deceleration",
                params.nominal_deceleration,
            )
        )
        params.max_acceleration = max(
            float(
                _get_param(
                    prefix,
                    "constraints/longitudinal/max_acceleration",
                    params.max_acceleration,
                )
            ),
            0.0,
        )
        params.path_generation_method = str(
            _get_param(prefix, "path_generation_method", params.path_generation_method)
        )
        params.approval_policy = str(
            _get_param(prefix, "policy/make_approval_request", params.approval_policy)
        )
        params.lateral_margin_policy = str(
            _get_param(prefix, "policy/lateral_margin", params.lateral_margin_policy)
        )
        params.candidate_turn_signal_policy = str(
            _get_param(
                prefix,
                "policy/candidate_path_turn_signal",
                params.candidate_turn_signal_policy,
            )
        )
        params.turn_signal_on_swerving = _as_bool(
            _get_param(
                prefix,
                "turn_signal/on_swerving",
                params.turn_signal_on_swerving,
            )
        )
        params.turn_signal_shift_length_threshold = float(
            _get_param(
                prefix,
                "turn_signal/shift_length_threshold",
                params.turn_signal_shift_length_threshold,
            )
        )
        params.turn_signal_remaining_distance_to_bound_threshold = float(
            _get_param(
                prefix,
                "turn_signal/remaining_distance_to_bound_threshold",
                params.turn_signal_remaining_distance_to_bound_threshold,
            )
        )
        params.default_avoid_direction = str(
            _get_param(prefix, "default_avoid_direction", params.default_avoid_direction)
        ).lower()
        params.vehicle_width = float(
            _get_param(prefix, "vehicle/width", params.vehicle_width)
        )
        params.base_link_to_front = float(
            _get_param(prefix, "vehicle/base_link_to_front", params.base_link_to_front)
        )
        params.base_link_to_rear = float(
            _get_param(prefix, "vehicle/base_link_to_rear", params.base_link_to_rear)
        )
        params.enable_rtc_left = _as_bool(
            _get_param(prefix, "rtc/enable_left", params.enable_rtc_left)
        )
        params.enable_rtc_right = _as_bool(
            _get_param(prefix, "rtc/enable_right", params.enable_rtc_right)
        )

        label_config = [
            (ObjectClassification.CAR, "car", 1.0, 2.0, 0.5, 0.2, 0.2, 0.0, 0.1),
            (ObjectClassification.TRUCK, "truck", 1.0, 2.0, 0.5, 0.2, 0.2, 0.0, 0.1),
            (ObjectClassification.BUS, "bus", 1.0, 2.0, 0.5, 0.2, 0.2, 0.0, 0.1),
            (ObjectClassification.TRAILER, "trailer", 1.0, 2.0, 0.5, 0.2, 0.2, 0.0, 0.1),
            (ObjectClassification.UNKNOWN, "unknown", 0.28, 1.0, 0.4, 0.2, 0.7, 0.0, 0.1),
            (ObjectClassification.BICYCLE, "bicycle", 0.28, 1.0, 0.7, 0.5, 0.2, 0.6, 0.5),
            (ObjectClassification.MOTORCYCLE, "motorcycle", 1.0, 1.0, 0.5, 0.3, 0.2, 0.6, 0.5),
            (ObjectClassification.PEDESTRIAN, "pedestrian", 0.28, 1.0, 0.7, 0.5, 0.2, 0.6, 0.5),
        ]
        for label, name, speed, moving_time, soft, hard, parked_hard, lon, envelope in label_config:
            enabled = _as_bool(
                _get_param(prefix, "target_filtering/target_type/{}".format(name), True)
            )
            if enabled:
                params.target_labels.add(int(label))
            object_prefix = "target_object/{}".format(name)
            params.object_policies[int(label)] = ObjectPolicy(
                moving_speed_threshold=float(
                    _get_param(prefix, object_prefix + "/th_moving_speed", speed)
                ),
                moving_time_threshold=float(
                    _get_param(prefix, object_prefix + "/th_moving_time", moving_time)
                ),
                soft_margin=float(
                    _get_param(prefix, object_prefix + "/lateral_margin/soft_margin", soft)
                ),
                hard_margin=float(
                    _get_param(prefix, object_prefix + "/lateral_margin/hard_margin", hard)
                ),
                parked_hard_margin=float(
                    _get_param(
                        prefix,
                        object_prefix + "/lateral_margin/hard_margin_for_parked_vehicle",
                        parked_hard,
                    )
                ),
                longitudinal_margin=float(
                    _get_param(prefix, object_prefix + "/longitudinal_margin", lon)
                ),
                envelope_buffer=float(
                    _get_param(prefix, object_prefix + "/envelope_buffer_margin", envelope)
                ),
            )
        return params


@dataclass
class ObjectTrack:
    first_seen: float
    last_seen: float
    stopped_since: float
    unknown_since: float
    last_x: float
    last_y: float
    total_movement: float
    message: object


@dataclass
class AvoidanceObject:
    object_id: str
    message: object
    label: int
    longitudinal: float
    relative_longitudinal: float
    lateral: float
    half_longitudinal: float
    half_lateral: float
    current_left_bound: float
    current_right_bound: float
    expanded_left_bound: float
    expanded_right_bound: float
    stopped_time: float
    total_movement: float
    is_parked: bool = False
    request_operator: bool = False
    avoid_required: bool = False
    allow_avoidance: bool = False
    shift_length: float = 0.0
    minimum_shift_length: float = 0.0
    preferred_shift_length: float = 0.0
    available_shift: float = 0.0
    uses_hard_drivable_bound: bool = False
    uses_diagonal_profile: bool = False
    profile_min_bound_clearance: float = float("inf")
    profile_min_object_clearance: float = float("inf")
    profile_failure_station: float = 0.0
    required_jerk: float = 0.0
    start_s: float = 0.0
    avoid_end_s: float = 0.0
    return_start_s: float = 0.0
    return_end_s: float = 0.0
    failed_reason: str = ""
    direction_failures: dict = field(default_factory=dict)

    @property
    def direction(self):
        return "left" if self.shift_length > 0.0 else "right"

    def debug_dict(self):
        return {
            "object_id": self.object_id,
            "allow_avoidance": self.allow_avoidance,
            "longitudinal_distance": self.relative_longitudinal,
            "lateral_distance": self.lateral,
            "path_distance": abs(self.lateral),
            "shift_length": self.shift_length,
            "required_jerk": self.required_jerk,
            "maximum_jerk": 1.0,
            "failed_reason": self.failed_reason,
        }


@dataclass
class ShiftLine:
    object_ids: list
    direction: str
    shift_length: float
    start_s: float
    avoid_end_s: float
    return_start_s: float
    return_end_s: float
    uses_hard_drivable_bound: bool = False
    uses_diagonal_profile: bool = False

    def value_at(self, station):
        if station <= self.start_s:
            return 0.0
        if station < self.avoid_end_s:
            return self.shift_length * _quintic_smoothstep(
                self.start_s, self.avoid_end_s, station
            )
        if station <= self.return_start_s:
            return self.shift_length
        if station < self.return_end_s:
            return self.shift_length * (
                1.0
                - _quintic_smoothstep(
                    self.return_start_s, self.return_end_s, station
                )
            )
        return 0.0


@dataclass
class StaticAvoidanceResult:
    reference_path: object
    candidate_path: object
    output_path: object
    candidates: list
    targets: list
    shift_lines: list
    direction: str = None
    safe: bool = True
    comfortable: bool = True
    valid: bool = True
    ready: bool = False
    request_operator: bool = False
    activated: bool = False
    auto_mode: bool = True
    rtc_state: int = State.SUCCEEDED
    start_distance: float = 0.0
    finish_distance: float = 0.0
    return_start_distance: float = 0.0
    return_finish_distance: float = 0.0
    return_direction: str = None
    return_ready: bool = False
    max_shift: float = 0.0
    stop_distance: float = 0.0
    signature: tuple = field(default_factory=tuple)
    status: str = "static_obstacle_avoidance: pass-through, no static target"


class StaticAvoidanceEngine:
    def __init__(self, parameters, centerline_map=None):
        self.params = parameters
        self.centerline_map = centerline_map
        self.tracks = {}
        self._unsafe_latched = False
        self._safe_count = 0
        self._last_signature = None
        self._unknown_log_times = {}

    def set_centerline_map(self, centerline_map):
        self.centerline_map = centerline_map

    def turn_signal_required(self, result):
        if not self.params.turn_signal_on_swerving or not result.direction:
            return False
        candidate_signal = self.params.candidate_turn_signal_policy == "all_candidate"
        if not result.activated and not candidate_signal:
            return False
        if result.max_shift < self.params.turn_signal_shift_length_threshold:
            return False
        if not result.targets:
            return False

        target = max(result.targets, key=lambda item: abs(item.shift_length))
        bound_distance = (
            target.current_left_bound
            if result.direction == "left"
            else target.current_right_bound
        )
        remaining_to_bound = (
            bound_distance - result.max_shift - 0.5 * self.params.vehicle_width
        )
        return (
            remaining_to_bound
            > self.params.turn_signal_remaining_distance_to_bound_threshold
        )

    def _policy(self, label):
        return self.params.object_policies.get(
            int(label), self.params.object_policies[ObjectClassification.UNKNOWN]
        )

    def _log_unknown_candidate(self, candidate, detection_distance, now):
        """Emit one concise, per-object diagnostic each second for UNKNOWN objects."""
        if (
            not self.params.debug_log_unknown_objects
            or candidate.label != ObjectClassification.UNKNOWN
        ):
            return

        last_log_time = self._unknown_log_times.get(candidate.object_id)
        if last_log_time is not None and now - last_log_time < 1.0:
            return
        self._unknown_log_times[candidate.object_id] = now
        self._unknown_log_times = {
            object_id: logged_at
            for object_id, logged_at in self._unknown_log_times.items()
            if now - logged_at < 30.0
        }

        pose = object_pose(candidate.message)
        policy = self._policy(candidate.label)
        direction = (
            "left"
            if candidate.shift_length > 0.0
            else "right" if candidate.shift_length < 0.0 else "none"
        )
        rospy.loginfo(
            "static_obstacle_avoidance [UNKNOWN] id=%s map=(%.2f, %.2f) "
            "relative=(s=%.2fm, l=%.2fm) detection=%.1fm "
            "bounds=current[L=%.2f,R=%.2f] expanded[L=%.2f,R=%.2f]m "
            "lane_policy=%s margin[hard=%.2f,soft=%.2f] "
            "decision[required=%s, allowed=%s, shift=%s %.2fm, "
            "available=%.2fm, stopped=%.1fs, reason=%s]",
            candidate.object_id,
            pose.position.x,
            pose.position.y,
            candidate.relative_longitudinal,
            candidate.lateral,
            detection_distance,
            candidate.current_left_bound,
            candidate.current_right_bound,
            candidate.expanded_left_bound,
            candidate.expanded_right_bound,
            self.params.use_lane_type,
            policy.parked_hard_margin if candidate.is_parked else policy.hard_margin,
            policy.soft_margin,
            candidate.avoid_required,
            candidate.allow_avoidance,
            direction,
            candidate.shift_length,
            candidate.available_shift,
            candidate.stopped_time,
            candidate.failed_reason or "ok",
        )

    def _update_tracks(self, objects, stamp):
        now = _stamp_seconds(stamp)
        seen = set()
        for predicted_object in objects:
            object_id = uuid_to_string(predicted_object.object_id)
            seen.add(object_id)
            pose = object_pose(predicted_object)
            label = _highest_probability_label(predicted_object)
            policy = self._policy(label)
            speed = object_speed(predicted_object)
            track = self.tracks.get(object_id)
            if track is None:
                track = ObjectTrack(
                    first_seen=now,
                    last_seen=now,
                    stopped_since=now,
                    unknown_since=now,
                    last_x=pose.position.x,
                    last_y=pose.position.y,
                    total_movement=0.0,
                    message=deepcopy(predicted_object),
                )
                self.tracks[object_id] = track
            else:
                track.total_movement += math.hypot(
                    pose.position.x - track.last_x, pose.position.y - track.last_y
                )
                track.last_x = pose.position.x
                track.last_y = pose.position.y
                track.last_seen = now
                track.message = deepcopy(predicted_object)
            if speed > policy.moving_speed_threshold:
                track.stopped_since = now
            if label != ObjectClassification.UNKNOWN:
                track.unknown_since = now

        expired = [
            object_id
            for object_id, track in self.tracks.items()
            if now - track.last_seen > self.params.max_compensation_time
        ]
        for object_id in expired:
            self.tracks.pop(object_id, None)

        tracked = []
        for object_id, track in self.tracks.items():
            if object_id in seen or now - track.last_seen <= self.params.max_compensation_time:
                tracked.append((track.message, track, now))
        return tracked

    def _ego_state(self, path, odometry):
        ego_s = 0.0
        ego_speed = 0.0
        if odometry is not None:
            projection = project_pose_to_path(path, odometry.pose.pose)
            if projection is not None:
                ego_s = projection.longitudinal_distance
            linear = odometry.twist.twist.linear
            ego_speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        return ego_s, ego_speed

    def _detection_distance(self, ego_speed):
        if self.params.use_static_detection_area:
            return self.params.max_forward_distance
        max_shift = max(
            self.params.max_left_shift_length, self.params.max_right_shift_length
        )
        speed = max(ego_speed, self.params.min_slow_down_speed)
        dynamic = 1.5 * _lateral_jerk_distance(
            max_shift, self.params.avoid_min_jerk, speed
        )
        prepare = max(
            speed * self.params.max_prepare_time,
            self.params.min_prepare_distance,
        )
        return clamp(
            dynamic + prepare,
            self.params.min_forward_distance,
            self.params.max_forward_distance,
        )

    def _lane_id_at(self, path, index):
        if not path.points:
            return None
        index = int(clamp(index, 0, len(path.points) - 1))
        for lane_id in path.points[index].lane_ids:
            if self.centerline_map is None or self.centerline_map.has_lane(lane_id):
                return int(lane_id)
        return None

    def _path_bound_distance(self, path, position, left):
        points = path.left_bound if left else path.right_bound
        if len(points) < 2:
            return None
        polyline = [(point.x, point.y, point.z) for point in points]
        projection = project_point_to_polyline(
            polyline, (position.x, position.y, position.z)
        )
        return float(projection[1]) if projection is not None else None

    def _bound_distances(self, path, projection):
        index = int(clamp(projection.index, 0, len(path.points) - 1))
        # Query the map at the object's exact projection onto the path.  Using
        # the beginning of a coarse path segment can substantially underestimate
        # the available space where an intersection or shoulder widens quickly.
        position = (
            projection.projected_position
            if projection.projected_position is not None
            else path.points[index].point.pose.position
        )
        current_left = self._path_bound_distance(path, position, True)
        current_right = self._path_bound_distance(path, position, False)
        current_left = current_left if current_left is not None else 1.75
        current_right = current_right if current_right is not None else 1.75
        expanded_left = current_left
        expanded_right = current_right
        lane_id = self._lane_id_at(path, index)
        if self.centerline_map is not None and lane_id is not None:
            map_distances = self.centerline_map.expanded_bound_distances(
                lane_id,
                (position.x, position.y, position.z),
                self.params.use_lane_type,
                self.params.use_intersection_areas,
            )
            if map_distances[0] is not None:
                expanded_left = max(expanded_left, map_distances[0])
            if map_distances[1] is not None:
                expanded_right = max(expanded_right, map_distances[1])
        return current_left, current_right, expanded_left, expanded_right

    def _create_candidate(self, path, predicted_object, track, now, ego_s, detection):
        pose = object_pose(predicted_object)
        projection = project_pose_to_path(path, pose)
        if projection is None:
            return None
        label = _highest_probability_label(predicted_object)
        policy = self._policy(label)
        path_yaw = _path_tangent_yaw(path, projection.index)
        half_longitudinal, half_lateral = _projected_object_half_extents(
            predicted_object, path_yaw
        )
        half_longitudinal += policy.envelope_buffer
        half_lateral += policy.envelope_buffer
        bounds = self._bound_distances(path, projection)
        stopped_time = max(0.0, now - track.stopped_since)
        relative_longitudinal = projection.longitudinal_distance - ego_s
        candidate = AvoidanceObject(
            object_id=uuid_to_string(predicted_object.object_id),
            message=predicted_object,
            label=label,
            longitudinal=projection.longitudinal_distance,
            relative_longitudinal=relative_longitudinal,
            lateral=projection.signed_lateral_distance,
            half_longitudinal=half_longitudinal,
            half_lateral=half_lateral,
            current_left_bound=bounds[0],
            current_right_bound=bounds[1],
            expanded_left_bound=bounds[2],
            expanded_right_bound=bounds[3],
            stopped_time=stopped_time,
            total_movement=track.total_movement,
        )

        if label not in self.params.target_labels:
            candidate.failed_reason = "is_not_target_object"
            return candidate
        if object_speed(predicted_object) > policy.moving_speed_threshold:
            candidate.failed_reason = "moving_object"
            return candidate
        if stopped_time < policy.moving_time_threshold:
            candidate.failed_reason = "unstable_stopped_object"
            return candidate
        if label == ObjectClassification.UNKNOWN and (
            now - track.unknown_since < self.params.unstable_classification_time
        ):
            candidate.failed_reason = "unstable_classification"
            return candidate
        if relative_longitudinal < -self.params.backward_distance:
            candidate.failed_reason = "out_of_target_area_behind"
            return candidate
        if relative_longitudinal > detection:
            candidate.failed_reason = "further_than_detection_range"
            return candidate

        object_side_bound = (
            candidate.current_left_bound
            if candidate.lateral >= 0.0
            else candidate.current_right_bound
        )
        if abs(candidate.lateral) - candidate.half_lateral > object_side_bound:
            candidate.failed_reason = "out_of_target_area_lateral"
            return candidate

        if label in VEHICLE_LABELS:
            physical_half_width = max(
                candidate.half_lateral - policy.envelope_buffer, 0.05
            )
            shiftable_distance = max(
                object_side_bound
                - physical_half_width
                + self.params.min_road_shoulder_width,
                0.1,
            )
            shiftable_ratio = abs(candidate.lateral) / shiftable_distance
            candidate.is_parked = (
                abs(candidate.lateral) >= self.params.parked_offset_threshold
                and shiftable_ratio >= self.params.parked_shiftable_ratio
            )
            if not candidate.is_parked:
                is_ambiguous = (
                    stopped_time >= self.params.ambiguous_stopped_time
                    and track.total_movement <= self.params.ambiguous_moving_distance
                )
                if not is_ambiguous or self.params.ambiguous_policy == "ignore":
                    candidate.failed_reason = "is_not_parking_object"
                    return candidate
                candidate.request_operator = self.params.ambiguous_policy == "manual"

        candidate.avoid_required = True
        return candidate

    def _avoidance_direction_order(self, candidate):
        if abs(candidate.lateral) < 1.0e-3:
            primary = self.params.default_avoid_direction
        else:
            # Keep Autoware's rule as the first choice: pass on the side
            # opposite to the object's position relative to the centerline.
            primary = "left" if candidate.lateral < 0.0 else "right"
        alternate = "right" if primary == "left" else "left"
        return primary, alternate

    def _desired_shift(
        self, candidate, ego_speed, path_end, requested_direction=None
    ):
        candidate.allow_avoidance = False
        candidate.shift_length = 0.0
        candidate.minimum_shift_length = 0.0
        candidate.preferred_shift_length = 0.0
        candidate.available_shift = 0.0
        candidate.uses_hard_drivable_bound = False
        candidate.uses_diagonal_profile = False
        candidate.profile_min_bound_clearance = float("inf")
        candidate.profile_min_object_clearance = float("inf")
        candidate.profile_failure_station = 0.0
        candidate.required_jerk = 0.0
        candidate.start_s = 0.0
        candidate.avoid_end_s = 0.0
        candidate.return_start_s = 0.0
        candidate.return_end_s = 0.0
        candidate.failed_reason = ""
        policy = self._policy(candidate.label)
        vehicle_half_width = 0.5 * self.params.vehicle_width
        hard_margin = (
            policy.parked_hard_margin if candidate.is_parked else policy.hard_margin
        )
        preferred_margin = vehicle_half_width + hard_margin + policy.soft_margin
        minimum_margin = vehicle_half_width + hard_margin

        primary_direction, alternate_direction = self._avoidance_direction_order(
            candidate
        )

        options = []
        for direction in ("left", "right"):
            if direction == "left":
                object_outer_edge = candidate.lateral + candidate.half_lateral
                preferred = max(object_outer_edge + preferred_margin, 0.0)
                minimum = max(object_outer_edge + minimum_margin, 0.0)
                soft_capacity = max(
                    min(
                        candidate.expanded_left_bound
                        - vehicle_half_width
                        - self.params.soft_drivable_bound_margin,
                        self.params.max_left_shift_length,
                    ),
                    0.0,
                )
                hard_capacity = max(
                    min(
                        candidate.expanded_left_bound
                        - vehicle_half_width
                        - self.params.hard_drivable_bound_margin,
                        self.params.max_left_shift_length,
                    ),
                    0.0,
                )
            else:
                object_outer_edge = candidate.lateral - candidate.half_lateral
                preferred_magnitude = max(preferred_margin - object_outer_edge, 0.0)
                minimum_magnitude = max(minimum_margin - object_outer_edge, 0.0)
                soft_capacity = max(
                    min(
                        candidate.expanded_right_bound
                        - vehicle_half_width
                        - self.params.soft_drivable_bound_margin,
                        self.params.max_right_shift_length,
                    ),
                    0.0,
                )
                hard_capacity = max(
                    min(
                        candidate.expanded_right_bound
                        - vehicle_half_width
                        - self.params.hard_drivable_bound_margin,
                        self.params.max_right_shift_length,
                    ),
                    0.0,
                )
                preferred = -preferred_magnitude
                minimum = -minimum_magnitude

            required_magnitude = abs(minimum)
            soft_feasible = soft_capacity + 1.0e-6 >= required_magnitude
            hard_feasible = hard_capacity + 1.0e-6 >= required_magnitude
            uses_hard_bound = not soft_feasible and hard_feasible
            capacity = (
                hard_capacity
                if uses_hard_bound or not soft_feasible
                else soft_capacity
            )
            sign = 1.0 if direction == "left" else -1.0
            signed_capacity = sign * capacity
            shift = sign * min(abs(preferred), capacity)
            options.append(
                {
                    "direction": direction,
                    "preferred": preferred,
                    "minimum": minimum,
                    "available": signed_capacity,
                    "shift": shift,
                    "hard_clearance": hard_capacity - required_magnitude,
                    "feasible": soft_feasible or hard_feasible,
                    "uses_hard_bound": uses_hard_bound,
                }
            )

        option_by_direction = {
            option["direction"]: option for option in options
        }
        if requested_direction is None:
            selected = next(
                (
                    option_by_direction[direction]
                    for direction in (primary_direction, alternate_direction)
                    if option_by_direction[direction]["feasible"]
                ),
                None,
            )
        else:
            selected = option_by_direction.get(requested_direction)
            if selected is not None and not selected["feasible"]:
                selected = None

        if selected is None:
            requested = option_by_direction.get(
                requested_direction, option_by_direction[primary_direction]
            )
            candidate.preferred_shift_length = requested["preferred"]
            candidate.minimum_shift_length = requested["minimum"]
            candidate.available_shift = requested["available"]
            candidate.uses_hard_drivable_bound = requested["uses_hard_bound"]
            candidate.failed_reason = "insufficient_drivable_space"
            return

        candidate.preferred_shift_length = selected["preferred"]
        candidate.minimum_shift_length = selected["minimum"]
        candidate.available_shift = selected["available"]
        candidate.uses_hard_drivable_bound = selected["uses_hard_bound"]
        shift = selected["shift"]
        if abs(shift) < self.params.lateral_execution_threshold:
            candidate.failed_reason = "less_than_execution_threshold"
            candidate.avoid_required = False
            return

        quantize = max(self.params.quantize_size, 1.0e-3)
        raw_shift = shift
        shift = round(raw_shift / quantize) * quantize
        if abs(shift) + 1.0e-6 < abs(selected["minimum"]):
            quantized_minimum = (
                math.ceil(abs(selected["minimum"]) / quantize - 1.0e-9)
                * quantize
            )
            if quantized_minimum <= abs(selected["available"]) + 1.0e-6:
                shift = math.copysign(quantized_minimum, raw_shift)
            else:
                # Preserve the exact feasible capacity instead of rounding below
                # the hard object clearance requirement.
                shift = raw_shift
        candidate.shift_length = shift

        planning_speed = max(ego_speed, self.params.min_slow_down_speed)
        front_constant = (
            self.params.base_link_to_front
            + candidate.half_longitudinal
            + policy.longitudinal_margin
        )
        rear_constant = (
            self.params.base_link_to_rear
            + candidate.half_longitudinal
            + policy.longitudinal_margin
        )
        # Match Autoware's envelope-based anchors: finish the lateral shift
        # before the vehicle front reaches the object envelope, hold the full
        # shift beside it, and only start returning after the vehicle rear has
        # cleared the envelope.
        candidate.avoid_end_s = candidate.longitudinal - front_constant
        nominal_prepare = max(
            planning_speed * self.params.max_prepare_time,
            self.params.min_prepare_distance,
        )
        avoid_distance = max(
            _lateral_jerk_distance(
                shift, self.params.avoid_min_jerk, planning_speed
            ),
            _lateral_jerk_distance(
                shift,
                self.params.avoid_min_jerk,
                max(ego_speed, self.params.nominal_avoidance_speed),
            ),
        )
        candidate.start_s = candidate.avoid_end_s - max(avoid_distance, nominal_prepare)
        ego_s = candidate.longitudinal - candidate.relative_longitudinal
        minimum_start_s = ego_s + self.params.min_prepare_distance
        latest_start_s = candidate.avoid_end_s - _lateral_jerk_distance(
            shift, self.params.lateral_max_jerk, planning_speed
        )
        if latest_start_s + 1.0e-6 < minimum_start_s:
            candidate.failed_reason = "insufficient_longitudinal_distance"
            return
        # Do not emit a profile whose nominal start is already behind ego;
        # relax its jerk only as far as the configured maximum permits.
        candidate.start_s = max(candidate.start_s, minimum_start_s)
        candidate.return_start_s = candidate.longitudinal + rear_constant
        return_distance = max(
            _lateral_jerk_distance(
                shift, self.params.return_min_jerk, planning_speed
            ),
            _lateral_jerk_distance(
                shift,
                self.params.return_min_jerk,
                max(ego_speed, self.params.nominal_return_speed),
            ),
        )
        candidate.return_end_s = min(
            path_end,
            candidate.return_start_s + max(return_distance, nominal_prepare),
        )

        longitudinal_span = max(candidate.avoid_end_s - candidate.start_s, 0.1)
        phase_time = longitudinal_span / max(planning_speed, 0.1) / 4.0
        candidate.required_jerk = abs(shift) / max(2.0 * phase_time**3, 1.0e-6)
        candidate.allow_avoidance = True
        if abs(shift) + 1.0e-6 < abs(preferred):
            candidate.failed_reason = "shortened_lateral_margin"

    def _merge_shift_lines(self, targets):
        ordered = sorted(targets, key=lambda item: item.longitudinal)
        lines = []
        conflict = False
        for target in ordered:
            line = ShiftLine(
                object_ids=[target.object_id],
                direction=target.direction,
                shift_length=target.shift_length,
                start_s=target.start_s,
                avoid_end_s=target.avoid_end_s,
                return_start_s=target.return_start_s,
                return_end_s=target.return_end_s,
                uses_hard_drivable_bound=target.uses_hard_drivable_bound,
                uses_diagonal_profile=target.uses_diagonal_profile,
            )
            if lines:
                previous = lines[-1]
                overlaps = line.start_s <= previous.return_end_s
                if overlaps and line.direction == previous.direction:
                    previous.object_ids.extend(line.object_ids)
                    if abs(line.shift_length) > abs(previous.shift_length):
                        previous.shift_length = line.shift_length
                    previous.start_s = min(previous.start_s, line.start_s)
                    previous.avoid_end_s = min(previous.avoid_end_s, line.avoid_end_s)
                    previous.return_start_s = max(previous.return_start_s, line.return_start_s)
                    previous.return_end_s = max(previous.return_end_s, line.return_end_s)
                    previous.uses_hard_drivable_bound = (
                        previous.uses_hard_drivable_bound
                        or line.uses_hard_drivable_bound
                    )
                    previous.uses_diagonal_profile = (
                        previous.uses_diagonal_profile
                        or line.uses_diagonal_profile
                    )
                    continue
                if overlaps and line.direction != previous.direction:
                    conflict = True
            lines.append(line)
        return lines, conflict

    def _profile(self, lines, station):
        values = [line.value_at(station) for line in lines]
        if not values:
            return 0.0
        return max(values, key=abs)

    def _shift_path(self, path, lines, expand_bounds=True):
        output = deepcopy(path)
        if expand_bounds:
            self._expand_drivable_bounds(output)
        distances = path_cumulative_lengths(output)
        offsets = []
        for point, station in zip(output.points, distances):
            offset = self._profile(lines, station)
            offsets.append(offset)
            point.point.pose = shifted_pose(point.point.pose, offset)

        for index, point in enumerate(output.points):
            if len(output.points) < 2:
                break
            before_index = max(index - 1, 0)
            after_index = min(index + 1, len(output.points) - 1)
            before = output.points[before_index].point.pose.position
            after = output.points[after_index].point.pose.position
            if _distance_2d(before, after) > 1.0e-6:
                point.point.pose.orientation = quaternion_from_yaw(
                    math.atan2(after.y - before.y, after.x - before.x)
                )
        return output, offsets

    def _expanded_profile_bounds(self, path):
        """Return Autoware-style hard vehicle-center limits along the path."""
        expanded = deepcopy(path)
        self._expand_drivable_bounds(expanded)
        stations = path_cumulative_lengths(expanded)
        vehicle_half_width = 0.5 * self.params.vehicle_width
        # Autoware validates the complete shift line against the hard bound and
        # permits the configured maximum lane deviation in that final check.
        margin = (
            vehicle_half_width
            + self.params.hard_drivable_bound_margin
            - self.params.max_deviation_from_lane
        )
        lower = []
        upper = []
        point_count = len(expanded.points)
        direct_bounds = (
            len(expanded.left_bound) == point_count
            and len(expanded.right_bound) == point_count
        )
        for index, path_point in enumerate(expanded.points):
            position = path_point.point.pose.position
            yaw = _path_tangent_yaw(expanded, index)
            normal = (-math.sin(yaw), math.cos(yaw))
            if direct_bounds:
                left = expanded.left_bound[index]
                right = expanded.right_bound[index]
                left_point = (left.x, left.y, left.z)
                right_point = (right.x, right.y, right.z)
            else:
                left_point = self._boundary_point(expanded.left_bound, position)
                right_point = self._boundary_point(expanded.right_bound, position)
            if left_point is None or right_point is None:
                lower.append(float("inf"))
                upper.append(float("-inf"))
                continue
            left_distance = (
                (left_point[0] - position.x) * normal[0]
                + (left_point[1] - position.y) * normal[1]
            )
            right_distance = (
                (right_point[0] - position.x) * normal[0]
                + (right_point[1] - position.y) * normal[1]
            )
            if left_distance <= right_distance:
                left_distance = math.hypot(
                    left_point[0] - position.x, left_point[1] - position.y
                )
                right_distance = -math.hypot(
                    right_point[0] - position.x, right_point[1] - position.y
                )
            lower.append(right_distance + margin)
            upper.append(left_distance - margin)
        return expanded, stations, lower, upper

    @staticmethod
    def _path_position_at(path, stations, station):
        if not path.points:
            return None
        if len(path.points) == 1 or not stations:
            point = path.points[0].point.pose.position
            return (point.x, point.y, point.z)
        station = clamp(station, stations[0], stations[-1])
        after = min(max(bisect_left(stations, station), 1), len(stations) - 1)
        before = after - 1
        span = max(stations[after] - stations[before], 1.0e-6)
        ratio = (station - stations[before]) / span
        first = path.points[before].point.pose.position
        second = path.points[after].point.pose.position
        return (
            first.x + (second.x - first.x) * ratio,
            first.y + (second.y - first.y) * ratio,
            first.z + (second.z - first.z) * ratio,
        )

    def _shifted_center_at(self, path, stations, line, station):
        position = self._path_position_at(path, stations, station)
        if position is None:
            return None
        delta = 0.05
        before = self._path_position_at(path, stations, station - delta)
        after = self._path_position_at(path, stations, station + delta)
        if before is None or after is None:
            return None
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        offset = line.value_at(station)
        return (
            position[0] - math.sin(yaw) * offset,
            position[1] + math.cos(yaw) * offset,
            position[2],
        )

    def _vehicle_polygon_at(self, path, stations, line, station):
        delta = 0.05
        center = self._shifted_center_at(path, stations, line, station)
        before = self._shifted_center_at(path, stations, line, station - delta)
        after = self._shifted_center_at(path, stations, line, station + delta)
        if center is None or before is None or after is None:
            return []
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        half_width = 0.5 * self.params.vehicle_width
        corners = []
        for longitudinal, lateral in (
            (self.params.base_link_to_front, half_width),
            (self.params.base_link_to_front, -half_width),
            (-self.params.base_link_to_rear, -half_width),
            (-self.params.base_link_to_rear, half_width),
        ):
            corners.append(
                (
                    center[0] + cosine * longitudinal - sine * lateral,
                    center[1] + sine * longitudinal + cosine * lateral,
                )
            )
        return corners

    def _expanded_object_polygon(self, candidate):
        pose = object_pose(candidate.message)
        length, width = _object_dimensions(candidate.message)
        policy = self._policy(candidate.label)
        hard_margin = (
            policy.parked_hard_margin if candidate.is_parked else policy.hard_margin
        )
        half_length = 0.5 * length + policy.envelope_buffer + hard_margin
        half_width = 0.5 * width + policy.envelope_buffer + hard_margin
        yaw = yaw_from_quaternion(pose.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return [
            (
                pose.position.x + cosine * longitudinal - sine * lateral,
                pose.position.y + sine * longitudinal + cosine * lateral,
            )
            for longitudinal, lateral in (
                (half_length, half_width),
                (half_length, -half_width),
                (-half_length, -half_width),
                (-half_length, half_width),
            )
        ]

    def _profile_footprint_clearance(
        self, path, stations, line, candidate, sampling_interval=None
    ):
        """Validate an oriented ego footprint against bounds and target polygon."""
        if len(path.points) < 2 or len(stations) != len(path.points):
            return False, "footprint_path_unavailable", 0.0, 0.0, line.start_s

        local_start = max(
            line.start_s - self.params.base_link_to_rear - 1.0,
            stations[0],
        )
        local_end = min(
            line.return_end_s + self.params.base_link_to_front + 1.0,
            stations[-1],
        )
        first_index = max(bisect_left(stations, local_start) - 1, 0)
        last_index = min(bisect_left(stations, local_end) + 1, len(stations) - 1)
        direct_bounds = (
            len(path.left_bound) == len(path.points)
            and len(path.right_bound) == len(path.points)
        )
        left_source = (
            path.left_bound[first_index : last_index + 1]
            if direct_bounds
            else path.left_bound
        )
        right_source = (
            path.right_bound[first_index : last_index + 1]
            if direct_bounds
            else path.right_bound
        )
        left = [(point.x, point.y) for point in left_source]
        right = [(point.x, point.y) for point in right_source]
        drivable_polygon = left + list(reversed(right))
        if len(left) < 2 or len(right) < 2:
            return False, "footprint_bound_unavailable", 0.0, 0.0, line.start_s

        bound_edges = list(
            zip(
                drivable_polygon,
                drivable_polygon[1:] + drivable_polygon[:1],
            )
        )
        object_polygon = self._expanded_object_polygon(candidate)
        required_bound_clearance = max(
            self.params.hard_drivable_bound_margin
            - self.params.max_deviation_from_lane,
            0.0,
        )
        interval = max(
            float(
                self.params.resample_interval_for_planning
                if sampling_interval is None
                else sampling_interval
            ),
            0.1,
        )
        profile_start = max(line.start_s, stations[0])
        # The diagonal relaxation only changes the avoidance leg.  The return
        # leg already passed the conventional center-bound fit above.
        profile_end = min(line.return_start_s, stations[-1])
        sample_count = max(
            int(math.ceil((profile_end - profile_start) / interval)),
            1,
        )
        sample_stations = [
            profile_start + (profile_end - profile_start) * index / sample_count
            for index in range(sample_count + 1)
        ]
        sample_stations.extend(
            [line.start_s, line.avoid_end_s, line.return_start_s, line.return_end_s]
        )

        minimum_bound_clearance = float("inf")
        minimum_object_clearance = float("inf")
        failure_station = profile_start
        for station in sorted(
            {
                clamp(value, profile_start, profile_end)
                for value in sample_stations
            }
        ):
            footprint = self._vehicle_polygon_at(path, stations, line, station)
            if len(footprint) != 4:
                return False, "footprint_generation_failed", 0.0, 0.0, station

            footprint_edges = list(
                zip(footprint, footprint[1:] + footprint[:1])
            )
            crosses_bound = any(
                _segments_intersect(
                    vehicle_start,
                    vehicle_end,
                    bound_start,
                    bound_end,
                )
                for vehicle_start, vehicle_end in footprint_edges
                for bound_start, bound_end in bound_edges
            )
            station_bound_clearance = min(
                min(
                    _point_segment_distance(point, first, second)
                    for point in footprint
                    for first, second in bound_edges
                ),
                min(
                    _point_segment_distance(point, first, second)
                    for point in drivable_polygon
                    for first, second in footprint_edges
                ),
            )
            if crosses_bound or not all(
                _point_in_polygon(point, drivable_polygon)
                for point in footprint
            ):
                station_bound_clearance = -station_bound_clearance
            if station_bound_clearance < minimum_bound_clearance:
                minimum_bound_clearance = station_bound_clearance
                failure_station = station
            if station_bound_clearance + 1.0e-3 < required_bound_clearance:
                return (
                    False,
                    "footprint_outside_drivable_bounds",
                    minimum_bound_clearance,
                    minimum_object_clearance,
                    station,
                )

            object_clearance = _polygon_distance(footprint, object_polygon)
            minimum_object_clearance = min(
                minimum_object_clearance, object_clearance
            )
            if _polygons_intersect(footprint, object_polygon):
                return (
                    False,
                    "target_hard_margin_collision",
                    minimum_bound_clearance,
                    0.0,
                    station,
                )

        return (
            True,
            "safe",
            minimum_bound_clearance,
            minimum_object_clearance,
            failure_station,
        )

    @staticmethod
    def _line_is_within_bounds(line, stations, lower, upper, leg=None):
        tolerance = 1.0e-3
        for station, minimum, maximum in zip(stations, lower, upper):
            if leg == "avoid" and station > line.avoid_end_s + tolerance:
                continue
            if leg == "return" and station < line.return_start_s - tolerance:
                continue
            offset = line.value_at(station)
            if offset < minimum - tolerance or offset > maximum + tolerance:
                return False
        return True

    @staticmethod
    def _required_lateral_jerk(shift_length, distance, velocity):
        distance = max(float(distance), 1.0e-3)
        phase_time = distance / max(float(velocity), 0.1) / 4.0
        return abs(float(shift_length)) / max(2.0 * phase_time**3, 1.0e-6)

    def _fit_shift_line_to_bounds(self, line, stations, lower, upper, ego_speed):
        """Relax a shift line up to max jerk while keeping the footprint in bounds."""
        speed = max(float(ego_speed), self.params.min_slow_down_speed)
        minimum_transition = _lateral_jerk_distance(
            line.shift_length, self.params.lateral_max_jerk, speed
        )
        latest_start = line.avoid_end_s - minimum_transition
        earliest_end = line.return_start_s + minimum_transition
        if latest_start <= line.start_s:
            avoid_starts = [line.start_s]
        else:
            avoid_starts = [line.start_s]
            avoid_starts.extend(
                station
                for station in stations
                if line.start_s < station < latest_start
            )
            avoid_starts.append(latest_start)

        original_start = line.start_s
        fitted_start = None
        for start in sorted(set(avoid_starts)):
            line.start_s = start
            if self._line_is_within_bounds(
                line, stations, lower, upper, leg="avoid"
            ):
                fitted_start = start
                break
        if fitted_start is None:
            line.start_s = original_start
            return False

        original_end = line.return_end_s
        if earliest_end >= original_end:
            return_ends = [original_end]
        else:
            return_ends = [original_end]
            return_ends.extend(
                station
                for station in stations
                if earliest_end < station < original_end
            )
            return_ends.append(earliest_end)
        fitted_end = None
        for end in sorted(set(return_ends), reverse=True):
            line.return_end_s = end
            if self._line_is_within_bounds(
                line, stations, lower, upper, leg="return"
            ):
                fitted_end = end
                break
        if fitted_end is None:
            line.start_s = original_start
            line.return_end_s = original_end
            return False

        avoid_jerk = self._required_lateral_jerk(
            line.shift_length, line.avoid_end_s - line.start_s, speed
        )
        return_jerk = self._required_lateral_jerk(
            line.shift_length, line.return_end_s - line.return_start_s, speed
        )
        return max(avoid_jerk, return_jerk) <= self.params.lateral_max_jerk + 0.1

    def _fit_diagonal_shift_line(
        self,
        candidate,
        base_line,
        expanded_path,
        stations,
        lower,
        upper,
        ego_speed,
    ):
        """Search a later, hard-margin shift using oriented-footprint checks."""
        quantize = max(self.params.quantize_size, 1.0e-3)
        preferred_magnitude = abs(base_line.shift_length)
        quantized_minimum = (
            math.ceil(abs(candidate.minimum_shift_length) / quantize - 1.0e-9)
            * quantize
        )
        minimum_magnitude = min(quantized_minimum, preferred_magnitude)
        if minimum_magnitude > preferred_magnitude + 1.0e-6:
            return None, "insufficient_drivable_space", None

        magnitude_count = max(
            int(math.floor((preferred_magnitude - minimum_magnitude) / quantize)),
            0,
        )
        magnitudes = [
            preferred_magnitude - quantize * index
            for index in range(magnitude_count + 1)
        ]
        if not magnitudes or magnitudes[-1] > minimum_magnitude + 1.0e-6:
            magnitudes.append(minimum_magnitude)

        maximum_avoid_end = min(
            candidate.longitudinal,
            candidate.return_start_s - 0.05,
        )
        if maximum_avoid_end <= candidate.avoid_end_s + 1.0e-6:
            return None, "shift_profile_outside_drivable_bounds", None
        avoid_ends = []
        value = candidate.avoid_end_s
        search_interval = max(
            self.params.resample_interval_for_planning,
            0.1,
        )
        while value < maximum_avoid_end - 1.0e-6:
            avoid_ends.append(value)
            value += search_interval
        avoid_ends.append(maximum_avoid_end)

        sign = 1.0 if base_line.direction == "left" else -1.0
        last_failure = "shift_profile_outside_drivable_bounds"
        last_metrics = None
        speed = max(float(ego_speed), self.params.min_slow_down_speed)
        for magnitude in magnitudes:
            bound_valid_ends = []
            for avoid_end in avoid_ends:
                if avoid_end <= base_line.start_s + 1.0e-6:
                    continue
                line = deepcopy(base_line)
                line.shift_length = sign * magnitude
                line.avoid_end_s = avoid_end
                line.uses_diagonal_profile = True
                required_jerk = self._required_lateral_jerk(
                    line.shift_length,
                    line.avoid_end_s - line.start_s,
                    speed,
                )
                if required_jerk > self.params.lateral_max_jerk + 0.1:
                    last_failure = "insufficient_longitudinal_distance"
                    continue
                if not self._line_is_within_bounds(
                    line, stations, lower, upper
                ):
                    last_failure = "shift_profile_outside_drivable_bounds"
                    continue
                bound_valid_ends.append(avoid_end)

            # The earliest bound-valid completion keeps the most clearance in
            # front of the obstacle.  Check only the first few neighboring
            # samples instead of repeating full footprint geometry for every
            # 0.3 m anchor.
            footprint_ends = bound_valid_ends[:2]
            if bound_valid_ends and bound_valid_ends[-1] not in footprint_ends:
                footprint_ends.append(bound_valid_ends[-1])
            for avoid_end in footprint_ends:
                line = deepcopy(base_line)
                line.shift_length = sign * magnitude
                line.avoid_end_s = avoid_end
                line.uses_diagonal_profile = True
                metrics = self._profile_footprint_clearance(
                    expanded_path,
                    stations,
                    line,
                    candidate,
                )
                valid, reason = metrics[:2]
                if valid:
                    return line, "", metrics
                last_failure = reason
                last_metrics = metrics
        return None, last_failure, last_metrics

    def _fit_directional_candidate(
        self,
        candidate,
        direction,
        ego_speed,
        path_end,
        expanded_path,
        stations,
        lower,
        upper,
    ):
        """Build and fit one direction without mutating the tracked candidate."""
        trial = deepcopy(candidate)
        self._desired_shift(
            trial,
            ego_speed,
            path_end,
            requested_direction=direction,
        )
        if not trial.allow_avoidance:
            return None, None, trial.failed_reason

        line = ShiftLine(
            object_ids=[trial.object_id],
            direction=trial.direction,
            shift_length=trial.shift_length,
            start_s=trial.start_s,
            avoid_end_s=trial.avoid_end_s,
            return_start_s=trial.return_start_s,
            return_end_s=trial.return_end_s,
            uses_hard_drivable_bound=trial.uses_hard_drivable_bound,
        )
        standard_fitted = self._fit_shift_line_to_bounds(
            line, stations, lower, upper, ego_speed
        )
        metrics = None
        failure = "shift_profile_outside_drivable_bounds"

        if not standard_fitted:
            line, diagonal_failure, diagonal_metrics = self._fit_diagonal_shift_line(
                trial,
                ShiftLine(
                    object_ids=[trial.object_id],
                    direction=trial.direction,
                    shift_length=trial.shift_length,
                    start_s=trial.start_s,
                    avoid_end_s=trial.avoid_end_s,
                    return_start_s=trial.return_start_s,
                    return_end_s=trial.return_end_s,
                    uses_hard_drivable_bound=trial.uses_hard_drivable_bound,
                ),
                expanded_path,
                stations,
                lower,
                upper,
                ego_speed,
            )
            if line is None:
                if diagonal_metrics is not None:
                    trial.profile_min_bound_clearance = diagonal_metrics[2]
                    trial.profile_min_object_clearance = diagonal_metrics[3]
                    trial.profile_failure_station = diagonal_metrics[4]
                    diagonal_failure = (
                        "{}(s={:.2f},bound={:.2f},object={:.2f})"
                    ).format(
                        diagonal_failure or failure,
                        diagonal_metrics[4],
                        diagonal_metrics[2],
                        diagonal_metrics[3],
                    )
                return None, None, diagonal_failure or failure
            metrics = diagonal_metrics

        trial.shift_length = line.shift_length
        trial.start_s = line.start_s
        trial.avoid_end_s = line.avoid_end_s
        trial.return_start_s = line.return_start_s
        trial.return_end_s = line.return_end_s
        trial.uses_diagonal_profile = line.uses_diagonal_profile
        if metrics is not None:
            trial.profile_min_bound_clearance = metrics[2]
            trial.profile_min_object_clearance = metrics[3]
            trial.profile_failure_station = metrics[4]
        planning_speed = max(ego_speed, self.params.min_slow_down_speed)
        trial.required_jerk = max(
            self._required_lateral_jerk(
                trial.shift_length,
                trial.avoid_end_s - trial.start_s,
                planning_speed,
            ),
            self._required_lateral_jerk(
                trial.shift_length,
                trial.return_end_s - trial.return_start_s,
                planning_speed,
            ),
        )
        return trial, line, ""

    @staticmethod
    def _prioritized_direction_combinations(option_groups, limit=16):
        """Yield primary-first combinations without constructing 2**N entries."""
        if not option_groups:
            yield tuple()
            return
        fallback_indices = [
            index for index, group in enumerate(option_groups) if len(group) > 1
        ]
        emitted = 0
        for fallback_count in range(len(fallback_indices) + 1):
            for selected_indices in combinations(
                fallback_indices, fallback_count
            ):
                selected = set(selected_indices)
                yield tuple(
                    group[1] if index in selected else group[0]
                    for index, group in enumerate(option_groups)
                )
                emitted += 1
                if emitted >= max(int(limit), 1):
                    return

    @staticmethod
    def _boundary_point(boundary, position):
        if len(boundary) < 2:
            return None
        polyline = [(point.x, point.y, point.z) for point in boundary]
        projection = project_point_to_polyline_with_point(
            polyline,
            (position.x, position.y, position.z),
        )
        return projection[2] if projection is not None else None

    def _expand_drivable_bounds(self, path):
        """Propagate map drivable-area expansion to Path left/right bounds."""
        if self.centerline_map is None or not path.points:
            return

        source_bounds = (list(path.left_bound), list(path.right_bound))
        expanded = ([], [])
        for index, path_point in enumerate(path.points):
            position = path_point.point.pose.position
            point = (position.x, position.y, position.z)
            yaw = _path_tangent_yaw(path, index)
            left_normal = (-math.sin(yaw), math.cos(yaw))
            lane_id = self._lane_id_at(path, index)
            map_points = (None, None)
            if lane_id is not None:
                map_points = self.centerline_map.expanded_bound_points(
                    lane_id,
                    point,
                    self.params.use_lane_type,
                    self.params.use_intersection_areas,
                )

            for side_index in (0, 1):
                candidates = [
                    self._boundary_point(source_bounds[side_index], position),
                    map_points[side_index],
                ]
                candidates = [candidate for candidate in candidates if candidate is not None]
                if not candidates:
                    continue

                def lateral(candidate):
                    return (
                        (candidate[0] - position.x) * left_normal[0]
                        + (candidate[1] - position.y) * left_normal[1]
                    )

                if side_index == 0:
                    same_side = [candidate for candidate in candidates if lateral(candidate) >= 0.0]
                    selected = max(same_side or candidates, key=lateral)
                else:
                    same_side = [candidate for candidate in candidates if lateral(candidate) <= 0.0]
                    selected = min(same_side or candidates, key=lateral)
                expanded[side_index].append(
                    Point(x=selected[0], y=selected[1], z=selected[2])
                )

        if len(expanded[0]) >= 2:
            path.left_bound = expanded[0]
        if len(expanded[1]) >= 2:
            path.right_bound = expanded[1]

    def _raw_safety(self, candidate_path, targets, all_objects, conflict):
        if conflict:
            return False, "opposite_shift_lines_overlap"
        if not self.params.enable_safety_check:
            return True, "safety_check_disabled"

        target_ids = {target.object_id for target in targets}
        for target in targets:
            projection = project_pose_to_path(candidate_path, object_pose(target.message))
            if projection is None:
                return False, "target_projection_failed"
            policy = self._policy(target.label)
            hard_margin = (
                policy.parked_hard_margin if target.is_parked else policy.hard_margin
            )
            clearance = (
                abs(projection.signed_lateral_distance)
                - 0.5 * self.params.vehicle_width
                - target.half_lateral
            )
            if clearance + 1.0e-3 < hard_margin:
                return False, "target_hard_margin_collision"

        for predicted_object in all_objects:
            object_id = uuid_to_string(predicted_object.object_id)
            if object_id in target_ids:
                continue
            projection = project_pose_to_path(candidate_path, object_pose(predicted_object))
            if projection is None:
                continue
            if projection.longitudinal_distance < -self.params.safety_backward_distance:
                continue
            _, half_lateral = _projected_object_half_extents(
                predicted_object, _path_tangent_yaw(candidate_path, projection.index)
            )
            policy = self._policy(_highest_probability_label(predicted_object))
            speed = object_speed(predicted_object)
            if (
                self.params.check_other_objects
                and speed <= policy.moving_speed_threshold
            ):
                clearance = (
                    abs(projection.signed_lateral_distance)
                    - 0.5 * self.params.vehicle_width
                    - half_lateral
                )
                if clearance + 1.0e-3 < max(policy.hard_margin, 0.0):
                    return False, "static_other_object_collision"
                continue
            lateral_threshold = (
                0.5 * self.params.vehicle_width
                + half_lateral
                + self.params.safety_lateral_threshold
            )
            if abs(projection.signed_lateral_distance) < lateral_threshold:
                if speed > policy.moving_speed_threshold:
                    return False, "predicted_object_collision"
        return True, "safe"

    def _hysteresis_safety(self, raw_safe, signature):
        if signature != self._last_signature:
            self._last_signature = signature
            self._unsafe_latched = not raw_safe
            self._safe_count = 0
            return raw_safe
        if not raw_safe:
            self._unsafe_latched = True
            self._safe_count = 0
            return False
        if not self._unsafe_latched:
            return True
        self._safe_count += 1
        if self._safe_count >= max(self.params.hysteresis_safe_count, 1):
            self._unsafe_latched = False
            self._safe_count = 0
            return True
        return False

    def _stop_path(self, path, ego_s, nearest_target):
        if nearest_target is None:
            return deepcopy(path), 0.0
        if self._target_fully_passed(nearest_target):
            return deepcopy(path), 0.0
        stop_s = (
            nearest_target.longitudinal
            - nearest_target.half_longitudinal
            - self.params.base_link_to_front
            - self.params.stop_buffer
        )
        stop_s = max(ego_s, min(stop_s, ego_s + self.params.stop_max_distance))
        output = deepcopy(path)
        distances = path_cumulative_lengths(output)
        deceleration = max(abs(self.params.nominal_deceleration), 0.1)
        for point, station in zip(output.points, distances):
            remaining = stop_s - station
            if remaining <= 0.0:
                point.point.longitudinal_velocity_mps = 0.0
            else:
                feasible_velocity = math.sqrt(2.0 * deceleration * remaining)
                point.point.longitudinal_velocity_mps = min(
                    point.point.longitudinal_velocity_mps, feasible_velocity
                )
        return output, stop_s - ego_s

    def _target_fully_passed(self, target):
        return (
            target.relative_longitudinal
            + target.half_longitudinal
            + self.params.base_link_to_rear
            < 0.0
        )

    def plan(self, path, objects_message, odometry, stamp):
        reference = deepcopy(path)
        if not path.points:
            return StaticAvoidanceResult(
                reference, reference, reference, [], [], [], valid=False,
                status="static_obstacle_avoidance: empty input path",
            )

        object_messages = list(objects_message.objects) if objects_message is not None else []
        tracked = self._update_tracks(object_messages, stamp)
        ego_s, ego_speed = self._ego_state(path, odometry)
        detection = self._detection_distance(ego_speed)
        path_distances = path_cumulative_lengths(path)
        path_end = path_distances[-1] if path_distances else 0.0

        candidates = []
        now = _stamp_seconds(stamp)
        for predicted_object, track, track_now in tracked:
            candidate = self._create_candidate(
                path, predicted_object, track, track_now, ego_s, detection
            )
            if candidate is None:
                continue
            if candidate.avoid_required:
                self._desired_shift(candidate, ego_speed, path_end)
            self._log_unknown_candidate(candidate, detection, now)
            candidates.append(candidate)

        blocking = [candidate for candidate in candidates if candidate.avoid_required]
        if not blocking:
            self._last_signature = None
            self._unsafe_latched = False
            self._safe_count = 0
            return StaticAvoidanceResult(
                reference_path=reference,
                candidate_path=deepcopy(reference),
                output_path=deepcopy(reference),
                candidates=candidates,
                targets=[],
                shift_lines=[],
            )

        (
            expanded_reference,
            profile_stations,
            profile_lower,
            profile_upper,
        ) = self._expanded_profile_bounds(reference)

        # Preserve Autoware's object-opposite direction as the primary choice.
        # Only build an alternate same-side candidate when the complete primary
        # profile cannot satisfy geometry, kinematics, or the final safety check.
        option_groups = []
        for candidate in blocking:
            primary, alternate = self._avoidance_direction_order(candidate)
            failures = {}
            group = []
            for direction in (primary, alternate):
                trial, line, failure = self._fit_directional_candidate(
                    candidate,
                    direction,
                    ego_speed,
                    path_end,
                    expanded_reference,
                    profile_stations,
                    profile_lower,
                    profile_upper,
                )
                if trial is None:
                    failures[direction] = failure
                    continue
                trial.direction_failures = dict(failures)
                group.append((trial, line))
            candidate.direction_failures = failures
            if group:
                # Attach failures from both attempts after all candidates are
                # evaluated so status output explains why fallback was used.
                for trial, _line in group:
                    trial.direction_failures = dict(failures)
                option_groups.append(group)
            else:
                candidate.allow_avoidance = False
                candidate.failed_reason = failures.get(
                    primary,
                    failures.get(alternate, "no_valid_avoidance_direction"),
                )

        first_plan = None
        selected_plan = None
        for combination in self._prioritized_direction_combinations(
            option_groups
        ):
            trial_targets = [item[0] for item in combination]
            trial_lines, trial_conflict = self._merge_shift_lines(
                trial_targets
            )
            trial_profile_valid = bool(trial_lines) and all(
                self._line_is_within_bounds(
                    line,
                    profile_stations,
                    profile_lower,
                    profile_upper,
                )
                for line in trial_lines
            )
            trial_path, _ = self._shift_path(
                expanded_reference, trial_lines, expand_bounds=False
            )
            if not trial_profile_valid:
                trial_raw_safe = False
                trial_safety_reason = (
                    "shift_profile_outside_drivable_bounds"
                )
            else:
                trial_raw_safe, trial_safety_reason = self._raw_safety(
                    trial_path,
                    trial_targets,
                    object_messages,
                    trial_conflict,
                )
            trial_comfortable = all(
                item.required_jerk
                <= self.params.lateral_max_jerk + 0.1
                for item in trial_targets
            )
            trial_valid = (
                bool(trial_lines)
                and not trial_conflict
                and trial_profile_valid
            )
            plan = {
                "targets": trial_targets,
                "lines": trial_lines,
                "conflict": trial_conflict,
                "profile_valid": trial_profile_valid,
                "candidate_path": trial_path,
                "raw_safe": trial_raw_safe,
                "safety_reason": trial_safety_reason,
                "comfortable": trial_comfortable,
                "valid": trial_valid,
            }
            if first_plan is None:
                first_plan = plan
            if trial_raw_safe and trial_comfortable and trial_valid:
                selected_plan = plan
                break

        if selected_plan is None:
            selected_plan = first_plan
        if selected_plan is None:
            selected_plan = {
                "targets": [],
                "lines": [],
                "conflict": False,
                "profile_valid": False,
                "candidate_path": deepcopy(expanded_reference),
                "raw_safe": False,
                "safety_reason": "no_valid_avoidance_direction",
                "comfortable": True,
                "valid": False,
            }
        elif first_plan is not None and selected_plan is not first_plan:
            primary_failure = first_plan["safety_reason"]
            if not first_plan["comfortable"]:
                primary_failure = "insufficient_longitudinal_distance"
            elif not first_plan["valid"]:
                primary_failure = "invalid_combined_shift_profile"
            for target in selected_plan["targets"]:
                primary, _alternate = self._avoidance_direction_order(target)
                if target.direction != primary:
                    target.direction_failures.setdefault(
                        primary, primary_failure
                    )

        targets = selected_plan["targets"]
        lines = selected_plan["lines"]
        conflict = selected_plan["conflict"]
        profile_valid = selected_plan["profile_valid"]
        candidate_path = selected_plan["candidate_path"]
        raw_safe = selected_plan["raw_safe"]
        safety_reason = selected_plan["safety_reason"]
        comfortable = selected_plan["comfortable"]
        valid = selected_plan["valid"]

        selected_by_id = {target.object_id: target for target in targets}
        candidates = [
            selected_by_id.get(candidate.object_id, candidate)
            for candidate in candidates
        ]
        blocking = [candidate for candidate in candidates if candidate.avoid_required]
        signature = tuple(
            (item.object_id, item.direction, round(item.shift_length, 2))
            for item in targets
        )
        safe = self._hysteresis_safety(raw_safe, signature)
        request_operator = any(item.request_operator for item in blocking)
        ready = safe and comfortable and valid and not request_operator

        nearest = min(blocking, key=lambda item: item.relative_longitudinal)
        unpassed_blocking = [
            candidate
            for candidate in blocking
            if not self._target_fully_passed(candidate)
        ]
        stop_target = min(
            unpassed_blocking,
            key=lambda item: item.relative_longitudinal,
            default=None,
        )
        stopped_path, stop_distance = self._stop_path(
            reference, ego_s, stop_target
        )
        direction = lines[0].direction if lines else None
        start_distance = min((line.start_s for line in lines), default=ego_s) - ego_s
        finish_distance = max((line.return_start_s for line in lines), default=ego_s) - ego_s
        return_start_distance = finish_distance
        return_finish_distance = max(
            (line.return_end_s for line in lines), default=ego_s
        ) - ego_s
        max_shift = max((abs(line.shift_length) for line in lines), default=0.0)
        reason = safety_reason
        if not targets:
            reason = nearest.failed_reason or "unavoidable"
        elif request_operator:
            reason = "operator_approval_required"
        elif not comfortable:
            reason = "insufficient_longitudinal_distance"
        fallback_details = []
        for target in targets:
            primary, _alternate = self._avoidance_direction_order(target)
            if target.direction == primary:
                continue
            fallback_details.append(
                "{}:{}->{}({})".format(
                    target.object_id[:8],
                    primary,
                    target.direction,
                    target.direction_failures.get(primary, "primary_invalid"),
                )
            )
        if fallback_details:
            reason += "; alternate_direction_fallback=" + ",".join(
                fallback_details
            )
        elif not targets and nearest.direction_failures:
            primary, alternate = self._avoidance_direction_order(nearest)
            reason += "; alternate_{}={}".format(
                alternate,
                nearest.direction_failures.get(alternate, "not_available"),
            )
        diagonal_details = [
            "{}:{} shift={:.2f} end={:.2f} bound={:.2f} object={:.2f}".format(
                target.object_id[:8],
                target.direction,
                target.shift_length,
                target.avoid_end_s,
                target.profile_min_bound_clearance,
                target.profile_min_object_clearance,
            )
            for target in targets
            if target.uses_diagonal_profile
        ]
        if diagonal_details:
            reason += "; diagonal_profile=" + ",".join(diagonal_details)

        return StaticAvoidanceResult(
            reference_path=reference,
            candidate_path=candidate_path,
            output_path=stopped_path,
            candidates=candidates,
            targets=targets,
            shift_lines=lines,
            direction=direction,
            safe=safe,
            comfortable=comfortable,
            valid=valid,
            ready=ready,
            request_operator=request_operator,
            start_distance=start_distance,
            finish_distance=finish_distance,
            return_start_distance=return_start_distance,
            return_finish_distance=return_finish_distance,
            return_direction=("right" if direction == "left" else "left") if direction else None,
            max_shift=max_shift,
            stop_distance=stop_distance,
            signature=signature,
            status=(
                "static_obstacle_avoidance: candidate {} shift {:.2f} m, "
                "safe={}, comfortable={}, valid={}, reason={}"
            ).format(direction or "none", max_shift, safe, comfortable, valid, reason),
        )


class StaticAvoidanceScene:
    """Autoware-shaped static-avoidance scene with directional RTC gating."""

    def __init__(self, prefix="~static_obstacle_avoidance", centerline_map=None):
        self.prefix = prefix
        self.params = StaticAvoidanceParameters.from_ros(prefix)
        self.engine = StaticAvoidanceEngine(self.params, centerline_map)
        self.rtc_interfaces = {
            "left": RTCInterfaceROS1(
                Module.AVOIDANCE_LEFT,
                _get_param(
                    prefix,
                    "rtc/left/status_topic",
                    "/planning/cooperate_status/static_obstacle_avoidance_left",
                ),
                _get_param(
                    prefix,
                    "rtc/left/command_service",
                    "/planning/cooperate_commands/static_obstacle_avoidance_left",
                ),
                _get_param(
                    prefix,
                    "rtc/left/auto_mode_service",
                    "/planning/enable_auto_mode/static_obstacle_avoidance_left",
                ),
                self.params.enable_rtc_left,
            ),
            "right": RTCInterfaceROS1(
                Module.AVOIDANCE_RIGHT,
                _get_param(
                    prefix,
                    "rtc/right/status_topic",
                    "/planning/cooperate_status/static_obstacle_avoidance_right",
                ),
                _get_param(
                    prefix,
                    "rtc/right/command_service",
                    "/planning/cooperate_commands/static_obstacle_avoidance_right",
                ),
                _get_param(
                    prefix,
                    "rtc/right/auto_mode_service",
                    "/planning/enable_auto_mode/static_obstacle_avoidance_right",
                ),
                self.params.enable_rtc_right,
            ),
        }
        self.avoid_rtc_controller = RTCSceneController(self.rtc_interfaces)
        self.return_rtc_controller = RTCSceneController(self.rtc_interfaces)
        # Kept as a compatibility alias for launch/integration code.
        self.rtc_controller = self.avoid_rtc_controller
        self._return_in_progress = False
        self._registered_return_path = None
        self._registered_return_reference = None
        self._registered_return_stations = []
        self._registered_return_offsets = []
        self._registered_avoid_direction = None
        self._registered_return_direction = None
        self._registered_avoid_signature = None
        self._registered_return_signature = None
        self._registered_max_shift = 0.0
        self._registered_return_start_s = 0.0
        self._registered_return_end_s = 0.0
        self._registered_return_planning_speed = 0.0
        self._registered_return_speed_limit = None
        self._avoid_in_progress = False
        self._committed_avoid_reference = None
        self._committed_avoid_stations = []
        self._committed_avoid_offsets = []
        self._committed_full_offsets = []
        self._committed_shift_lines = []
        self._committed_object_ids = set()
        self._committed_direction = None
        self._committed_return_direction = None
        self._committed_signature = None
        self._committed_max_shift = 0.0

    def set_centerline_map(self, centerline_map):
        self.engine.set_centerline_map(centerline_map)

    @staticmethod
    def _path_xyz(path):
        return [
            (
                point.point.pose.position.x,
                point.point.pose.position.y,
                point.point.pose.position.z,
            )
            for point in path.points
        ]

    def _align_registered_return(self, path):
        output = deepcopy(path)
        if self._registered_return_reference is None:
            return output, 0
        registered_points = self._path_xyz(self._registered_return_reference)
        if len(registered_points) < 2:
            return output, 0
        maximum_distance = max(self.params.max_deviation_from_lane + 1.0, 2.0)
        aligned_count = 0
        for index, point in enumerate(output.points):
            position = point.point.pose.position
            projection = project_point_to_polyline_with_point(
                registered_points,
                (position.x, position.y, position.z),
            )
            if projection is None or projection[1] > maximum_distance:
                continue
            shift = self._interpolate_registered_shift(projection[0])
            yaw = _path_tangent_yaw(path, index)
            position.x += -math.sin(yaw) * shift
            position.y += math.cos(yaw) * shift
            aligned_count += 1
        for index, point in enumerate(output.points):
            point.point.pose.orientation = quaternion_from_yaw(
                _path_tangent_yaw(output, index)
            )
        return output, aligned_count

    def _interpolate_registered_shift(self, station):
        stations = self._registered_return_stations
        offsets = self._registered_return_offsets
        if not stations or not offsets:
            return 0.0
        if station <= stations[0]:
            return offsets[0]
        if station >= stations[-1]:
            return offsets[-1]
        for index in range(len(stations) - 1):
            if station > stations[index + 1]:
                continue
            length = max(stations[index + 1] - stations[index], 1.0e-6)
            ratio = (station - stations[index]) / length
            return offsets[index] + ratio * (
                offsets[index + 1] - offsets[index]
            )
        return offsets[-1]

    @staticmethod
    def _odometry_speed(odometry):
        if odometry is None:
            return 0.0
        linear = odometry.twist.twist.linear
        return math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )

    def _return_velocity_limit(self, ego_speed):
        # Do not pin a vehicle that happened to stop at the return start.  In
        # that case permit a controlled restart up to the configured crawl
        # speed; otherwise hold the measured return-entry speed.
        restart_speed = min(
            self.params.min_slow_down_speed,
            self.params.nominal_return_speed,
        )
        if ego_speed < 0.1:
            return restart_speed
        return min(ego_speed, self.params.nominal_return_speed)

    def _apply_longitudinal_constraints(
        self,
        path,
        reference,
        odometry,
        maneuver_end_s,
        return_start_s=None,
        return_speed_limit=None,
    ):
        """Limit acceleration and prevent acceleration during the return leg."""
        output = deepcopy(path)
        if not output.points or not reference.points or odometry is None:
            return output
        projection = project_pose_to_path(reference, odometry.pose.pose)
        if projection is None:
            return output

        ego_s = projection.longitudinal_distance
        ego_speed = self._odometry_speed(odometry)
        acceleration = max(self.params.max_acceleration, 0.0)
        stations = path_cumulative_lengths(reference)
        count = min(len(output.points), len(stations))
        for index in range(count):
            station = stations[index]
            if station + 1.0e-6 < ego_s or station > maneuver_end_s + 1.0e-6:
                continue
            distance = max(station - ego_s, 0.0)
            acceleration_limit = math.sqrt(
                max(ego_speed * ego_speed + 2.0 * acceleration * distance, 0.0)
            )
            velocity = min(
                output.points[index].point.longitudinal_velocity_mps,
                acceleration_limit,
            )
            if (
                return_start_s is not None
                and return_speed_limit is not None
                and station >= return_start_s - 1.0e-6
            ):
                velocity = min(velocity, return_speed_limit)
            output.points[index].point.longitudinal_velocity_mps = max(velocity, 0.0)
        return output

    def _maybe_latch_return_speed(self, odometry):
        if self._registered_return_reference is None or odometry is None:
            return
        projection = project_pose_to_path(
            self._registered_return_reference, odometry.pose.pose
        )
        if projection is None:
            return
        if (
            self._registered_return_speed_limit is None
            and projection.longitudinal_distance
            >= self._registered_return_start_s - 1.0e-3
        ):
            self._registered_return_speed_limit = self._return_velocity_limit(
                self._odometry_speed(odometry)
            )

    def _extend_registered_return_for_speed(self, odometry):
        """Stretch the remaining registered profile if its speed assumption is stale."""
        if (
            self._registered_return_reference is None
            or not self._registered_return_stations
            or not self._registered_return_offsets
            or odometry is None
        ):
            return False
        projection = project_pose_to_path(
            self._registered_return_reference, odometry.pose.pose
        )
        if projection is None:
            return False

        ego_speed = self._odometry_speed(odometry)
        planning_speed = max(
            ego_speed,
            self.params.min_slow_down_speed,
            self.params.nominal_return_speed,
        )
        if planning_speed <= self._registered_return_planning_speed + 0.05:
            return False

        anchor_s = max(
            projection.longitudinal_distance, self._registered_return_start_s
        )
        anchor_shift = self._interpolate_registered_shift(anchor_s)
        remaining_distance = max(self._registered_return_end_s - anchor_s, 0.0)
        required_distance = _lateral_jerk_distance(
            anchor_shift, self.params.return_min_jerk, planning_speed
        )
        if required_distance <= remaining_distance + 0.05:
            self._registered_return_planning_speed = planning_speed
            return False

        original_stations = list(self._registered_return_stations)
        original_offsets = list(self._registered_return_offsets)

        def original_shift(station):
            if station <= original_stations[0]:
                return original_offsets[0]
            if station >= original_stations[-1]:
                return original_offsets[-1]
            for index in range(len(original_stations) - 1):
                if station > original_stations[index + 1]:
                    continue
                length = max(
                    original_stations[index + 1] - original_stations[index],
                    1.0e-6,
                )
                ratio = (station - original_stations[index]) / length
                return original_offsets[index] + ratio * (
                    original_offsets[index + 1] - original_offsets[index]
                )
            return original_offsets[-1]

        scale = required_distance / max(remaining_distance, 1.0e-3)
        extended_end = min(
            original_stations[-1], anchor_s + required_distance
        )
        stretched_offsets = []
        for station in original_stations:
            if station <= anchor_s:
                stretched_offsets.append(original_shift(station))
            elif station < extended_end:
                source_station = anchor_s + (station - anchor_s) / scale
                stretched_offsets.append(original_shift(source_station))
            else:
                stretched_offsets.append(0.0)
        self._registered_return_offsets = stretched_offsets
        self._registered_return_end_s = extended_end
        self._registered_return_planning_speed = planning_speed
        return True

    def _reanchor_registered_return(
        self,
        odometry,
        requested_shift,
        actual_shift,
    ):
        """Rebuild the remaining return from the lateral state actually reached."""
        threshold = self.params.return_reanchor_lateral_error_threshold
        if (
            threshold <= 0.0
            or odometry is None
            or self._registered_return_reference is None
            or len(self._registered_return_stations) < 2
            or len(self._registered_return_offsets) < 2
            or abs(actual_shift - requested_shift) <= threshold
        ):
            return False

        projection = project_pose_to_path(
            self._registered_return_reference,
            odometry.pose.pose,
        )
        if projection is None:
            return False

        anchor_s = clamp(
            projection.longitudinal_distance,
            self._registered_return_stations[0],
            self._registered_return_stations[-1],
        )
        # Use the live reference-path error for the lateral anchor.  The
        # registered reference is retained only for its longitudinal station;
        # on parallel lanelets its nearest-segment lateral sign can be stale.
        anchor_shift = float(actual_shift)
        completion_threshold = max(
            self.params.lateral_execution_threshold,
            self.params.max_deviation_from_lane,
        )
        if abs(anchor_shift) <= completion_threshold:
            self._registered_return_offsets = [
                0.0 for _station in self._registered_return_stations
            ]
            self._registered_return_start_s = anchor_s
            self._registered_return_end_s = anchor_s
            return True

        planning_speed = max(
            self._odometry_speed(odometry),
            self.params.min_slow_down_speed,
            self.params.nominal_return_speed,
        )
        nominal_prepare = max(
            planning_speed * self.params.max_prepare_time,
            self.params.min_prepare_distance,
        )
        return_distance = max(
            _lateral_jerk_distance(
                anchor_shift,
                self.params.return_min_jerk,
                planning_speed,
            ),
            nominal_prepare,
        )
        return_end_s = min(
            self._registered_return_stations[-1],
            anchor_s + return_distance,
        )
        if return_end_s <= anchor_s + 0.1:
            return False

        offsets = []
        for station in self._registered_return_stations:
            if station <= anchor_s:
                offsets.append(anchor_shift)
            elif station < return_end_s:
                offsets.append(
                    anchor_shift
                    * (
                        1.0
                        - _quintic_smoothstep(
                            anchor_s,
                            return_end_s,
                            station,
                        )
                    )
                )
            else:
                offsets.append(0.0)

        self._registered_return_offsets = offsets
        self._registered_return_start_s = anchor_s
        self._registered_return_end_s = return_end_s
        self._registered_return_planning_speed = planning_speed
        return True

    @staticmethod
    def _requested_shift_at(reference, candidate, index):
        if not reference.points or not candidate.points:
            return 0.0
        index = int(
            clamp(index, 0, min(len(reference.points), len(candidate.points)) - 1)
        )
        reference_position = reference.points[index].point.pose.position
        candidate_position = candidate.points[index].point.pose.position
        yaw = _path_tangent_yaw(reference, index)
        delta_x = candidate_position.x - reference_position.x
        delta_y = candidate_position.y - reference_position.y
        return -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y

    @staticmethod
    def _interpolate_profile(stations, offsets, station):
        if not stations or not offsets:
            return 0.0
        if station <= stations[0]:
            return offsets[0]
        if station >= stations[-1]:
            return offsets[-1]
        for index in range(len(stations) - 1):
            if station > stations[index + 1]:
                continue
            length = max(stations[index + 1] - stations[index], 1.0e-6)
            ratio = (station - stations[index]) / length
            return offsets[index] + ratio * (
                offsets[index + 1] - offsets[index]
            )
        return offsets[-1]

    def _align_committed_avoidance(self, path, offsets):
        output = deepcopy(path)
        if self._committed_avoid_reference is None:
            return output, 0
        registered_points = self._path_xyz(self._committed_avoid_reference)
        if len(registered_points) < 2:
            return output, 0
        maximum_distance = max(self.params.max_deviation_from_lane + 1.0, 2.0)
        aligned_count = 0
        for index, point in enumerate(output.points):
            position = point.point.pose.position
            projection = project_point_to_polyline_with_point(
                registered_points,
                (position.x, position.y, position.z),
            )
            if projection is None or projection[1] > maximum_distance:
                continue
            shift = self._interpolate_profile(
                self._committed_avoid_stations, offsets, projection[0]
            )
            yaw = _path_tangent_yaw(path, index)
            position.x += -math.sin(yaw) * shift
            position.y += math.cos(yaw) * shift
            aligned_count += 1
        for index, point in enumerate(output.points):
            point.point.pose.orientation = quaternion_from_yaw(
                _path_tangent_yaw(output, index)
            )
        return output, aligned_count

    def _register_avoidance(self, result, avoid_candidate, full_candidate):
        count = min(
            len(result.reference_path.points),
            len(avoid_candidate.points),
            len(full_candidate.points),
        )
        self._avoid_in_progress = True
        self._committed_avoid_reference = deepcopy(result.reference_path)
        self._committed_avoid_stations = path_cumulative_lengths(
            result.reference_path
        )[:count]
        self._committed_avoid_offsets = [
            self._requested_shift_at(result.reference_path, avoid_candidate, index)
            for index in range(count)
        ]
        self._committed_full_offsets = [
            self._requested_shift_at(result.reference_path, full_candidate, index)
            for index in range(count)
        ]
        self._committed_shift_lines = deepcopy(result.shift_lines)
        self._committed_object_ids = {
            object_id
            for line in result.shift_lines
            for object_id in line.object_ids
        }
        self._committed_direction = result.direction
        self._committed_return_direction = result.return_direction
        self._committed_signature = result.signature
        self._committed_max_shift = result.max_shift

    def _clear_committed_avoidance(self, stamp, complete_rtc=False):
        self._avoid_in_progress = False
        self._committed_avoid_reference = None
        self._committed_avoid_stations = []
        self._committed_avoid_offsets = []
        self._committed_full_offsets = []
        self._committed_shift_lines = []
        self._committed_object_ids = set()
        self._committed_direction = None
        self._committed_return_direction = None
        self._committed_signature = None
        self._committed_max_shift = 0.0
        if complete_rtc:
            self.avoid_rtc_controller.complete(stamp)
            self.return_rtc_controller.complete(stamp)

    def _committed_lines_on_reference(self, reference, odometry):
        lines = deepcopy(self._committed_shift_lines)
        if (
            not lines
            or self._committed_avoid_reference is None
            or odometry is None
        ):
            return lines
        registered_projection = project_pose_to_path(
            self._committed_avoid_reference, odometry.pose.pose
        )
        current_projection = project_pose_to_path(reference, odometry.pose.pose)
        if registered_projection is None or current_projection is None:
            return lines
        delta = (
            current_projection.longitudinal_distance
            - registered_projection.longitudinal_distance
        )
        for line in lines:
            line.start_s += delta
            line.avoid_end_s += delta
            line.return_start_s += delta
            line.return_end_s += delta
        return lines

    def _stop_on_committed_avoidance(self, path, result, targets, odometry):
        projection = (
            project_pose_to_path(result.reference_path, odometry.pose.pose)
            if odometry is not None
            else None
        )
        ego_s = projection.longitudinal_distance if projection is not None else 0.0
        blocking = [
            target for target in targets if not self.engine._target_fully_passed(target)
        ]
        if blocking:
            nearest = min(blocking, key=lambda target: target.relative_longitudinal)
            return self.engine._stop_path(path, ego_s, nearest)[0]
        output = deepcopy(path)
        for point, station in zip(
            output.points, path_cumulative_lengths(result.reference_path)
        ):
            if station >= ego_s - 1.0e-3:
                point.point.longitudinal_velocity_mps = 0.0
        return output

    def _continue_committed_avoidance(self, result, objects, odometry, stamp):
        expanded_reference = deepcopy(result.reference_path)
        self.engine._expand_drivable_bounds(expanded_reference)
        avoid_candidate, avoid_count = self._align_committed_avoidance(
            expanded_reference, self._committed_avoid_offsets
        )
        full_candidate, full_count = self._align_committed_avoidance(
            expanded_reference, self._committed_full_offsets
        )
        if min(avoid_count, full_count) < 2:
            self._clear_committed_avoidance(stamp, complete_rtc=True)
            return False

        lines = self._committed_lines_on_reference(result.reference_path, odometry)
        ego_projection = (
            project_pose_to_path(result.reference_path, odometry.pose.pose)
            if odometry is not None
            else None
        )
        ego_s = ego_projection.longitudinal_distance if ego_projection is not None else 0.0
        ego_index = ego_projection.index if ego_projection is not None else 0
        requested_shift = self._requested_shift_at(
            result.reference_path, avoid_candidate, ego_index
        )
        actual_shift = (
            ego_projection.signed_lateral_distance
            if ego_projection is not None
            else 0.0
        )
        registered_targets = [
            candidate
            for candidate in result.candidates
            if candidate.object_id in self._committed_object_ids
        ]
        object_messages = list(objects.objects) if objects is not None else []
        raw_safe, safety_reason = self.engine._raw_safety(
            avoid_candidate,
            registered_targets,
            object_messages,
            False,
        )

        start_distance = min((line.start_s for line in lines), default=ego_s) - ego_s
        finish_distance = max(
            (line.return_start_s for line in lines), default=ego_s
        ) - ego_s
        self.avoid_rtc_controller.evaluate(
            self._committed_direction,
            self._committed_signature,
            raw_safe,
            start_distance,
            finish_distance,
            stamp,
        )

        result.targets = registered_targets
        result.shift_lines = lines
        result.direction = self._committed_direction
        result.return_direction = self._committed_return_direction
        result.max_shift = self._committed_max_shift
        result.signature = self._committed_signature
        result.safe = raw_safe
        result.comfortable = True
        result.valid = True
        result.ready = raw_safe
        result.activated = True
        result.start_distance = start_distance
        result.finish_distance = finish_distance

        if not raw_safe:
            result.candidate_path = avoid_candidate
            result.output_path = self._stop_on_committed_avoidance(
                avoid_candidate, result, registered_targets, odometry
            )
            result.return_ready = False
            result.status += (
                "; committed avoidance unsafe, holding geometry and stopping, "
                "reason={}"
            ).format(safety_reason)
            return True

        result.output_path = self._apply_longitudinal_constraints(
            avoid_candidate,
            result.reference_path,
            odometry,
            max((line.return_end_s for line in lines), default=ego_s),
        )
        remaining_ratio = max(
            1.0 - abs(requested_shift) / max(self._committed_max_shift, 1.0e-6),
            0.0,
        )
        result.return_ready = (
            remaining_ratio < self.params.return_approval_ratio + 1.0e-3
        )
        return_signature = self._committed_signature + (
            ("return", self._committed_return_direction),
        )
        return_start_distance = max(
            (line.return_start_s for line in lines), default=ego_s
        ) - ego_s
        return_finish_distance = max(
            (line.return_end_s for line in lines), default=ego_s
        ) - ego_s
        return_activated, return_state, _, _ = self.return_rtc_controller.evaluate(
            self._committed_return_direction,
            return_signature,
            result.return_ready,
            return_start_distance,
            return_finish_distance,
            stamp,
        )
        result.return_start_distance = return_start_distance
        result.return_finish_distance = return_finish_distance
        result.candidate_path = full_candidate
        if return_activated and result.return_ready:
            result.output_path = self._apply_longitudinal_constraints(
                full_candidate,
                result.reference_path,
                odometry,
                max((line.return_end_s for line in lines), default=ego_s),
                min((line.return_start_s for line in lines), default=ego_s),
                self._return_velocity_limit(self._odometry_speed(odometry)),
            )
            self._register_return(
                result, full_candidate, return_signature, odometry
            )
            self._clear_committed_avoidance(stamp)

        return_state_name = {
            State.WAITING_FOR_EXECUTION: "waiting",
            State.RUNNING: "running",
            State.ABORTING: "aborting",
            State.SUCCEEDED: "succeeded",
            State.FAILED: "failed",
        }.get(return_state, "unknown")
        result.status += (
            "; committed avoidance active, requested_shift={:.2f} m, "
            "actual_shift={:.2f} m, safe={}, return_rtc={} {}, activated={}"
        ).format(
            requested_shift,
            actual_shift,
            raw_safe,
            "ready" if result.return_ready else "held",
            return_state_name,
            bool(return_activated and result.return_ready),
        )
        return True

    def _registered_return_state(self, reference, candidate, odometry):
        projection = project_pose_to_path(reference, odometry.pose.pose)
        if projection is None:
            return 0.0, 0.0
        requested_shift = self._requested_shift_at(
            reference, candidate, projection.index
        )
        return requested_shift, projection.signed_lateral_distance

    def _register_return(self, result, candidate_path, return_signature, odometry):
        new_return = (
            not self._return_in_progress
            or self._registered_return_signature != return_signature
        )
        self._return_in_progress = True
        self._registered_return_path = deepcopy(candidate_path)
        self._registered_return_reference = deepcopy(result.reference_path)
        count = min(
            len(result.reference_path.points), len(candidate_path.points)
        )
        self._registered_return_stations = path_cumulative_lengths(
            result.reference_path
        )[:count]
        self._registered_return_offsets = [
            self._requested_shift_at(result.reference_path, candidate_path, index)
            for index in range(count)
        ]
        self._registered_avoid_direction = result.direction
        self._registered_return_direction = result.return_direction
        self._registered_avoid_signature = result.signature
        self._registered_return_signature = return_signature
        self._registered_max_shift = result.max_shift
        self._registered_return_start_s = min(
            (line.return_start_s for line in result.shift_lines), default=0.0
        )
        self._registered_return_end_s = max(
            (line.return_end_s for line in result.shift_lines), default=0.0
        )
        self._registered_return_planning_speed = max(
            self._odometry_speed(odometry),
            self.params.min_slow_down_speed,
            self.params.nominal_return_speed,
        )
        if new_return:
            self._registered_return_speed_limit = None
        self._maybe_latch_return_speed(odometry)

    def _clear_registered_return(self, stamp):
        self._return_in_progress = False
        self._registered_return_path = None
        self._registered_return_reference = None
        self._registered_return_stations = []
        self._registered_return_offsets = []
        self._registered_avoid_direction = None
        self._registered_return_direction = None
        self._registered_avoid_signature = None
        self._registered_return_signature = None
        self._registered_max_shift = 0.0
        self._registered_return_start_s = 0.0
        self._registered_return_end_s = 0.0
        self._registered_return_planning_speed = 0.0
        self._registered_return_speed_limit = None
        self.avoid_rtc_controller.complete(stamp)
        self.return_rtc_controller.complete(stamp)

    def _continue_registered_return(self, result, odometry, stamp):
        profile_extended = self._extend_registered_return_for_speed(odometry)
        continuation, aligned_count = self._align_registered_return(
            result.candidate_path
        )
        requested_shift, actual_shift = self._registered_return_state(
            result.reference_path, continuation, odometry
        )
        tracking_error = actual_shift - requested_shift
        profile_reanchored = self._reanchor_registered_return(
            odometry,
            requested_shift,
            actual_shift,
        )
        if profile_reanchored:
            continuation, aligned_count = self._align_registered_return(
                result.candidate_path
            )
            requested_shift, actual_shift = self._registered_return_state(
                result.reference_path,
                continuation,
                odometry,
            )
        completion_threshold = self.params.lateral_execution_threshold
        actual_completion_threshold = max(
            completion_threshold, self.params.max_deviation_from_lane
        )
        complete = (
            aligned_count < 2
            or (
                abs(requested_shift) <= completion_threshold
                and abs(actual_shift) <= actual_completion_threshold
            )
        )
        if complete:
            self._clear_registered_return(stamp)
            result.candidate_path = result.reference_path
            result.output_path = result.reference_path
            result.status += "; registered return completed"
            return False

        self.avoid_rtc_controller.evaluate(
            self._registered_avoid_direction,
            self._registered_avoid_signature,
            True,
            0.0,
            0.0,
            stamp,
        )
        self.return_rtc_controller.evaluate(
            self._registered_return_direction,
            self._registered_return_signature,
            True,
            0.0,
            0.0,
            stamp,
        )
        result.candidate_path = continuation
        result.output_path = continuation
        result.direction = self._registered_avoid_direction
        result.return_direction = self._registered_return_direction
        result.max_shift = self._registered_max_shift
        result.safe = True
        result.comfortable = True
        result.valid = True
        result.ready = True
        result.activated = True
        result.return_ready = True
        self._maybe_latch_return_speed(odometry)
        return_speed_limit = self._registered_return_speed_limit
        if return_speed_limit is None:
            return_speed_limit = self.params.nominal_return_speed
        continuation = self._apply_longitudinal_constraints(
            continuation,
            result.reference_path,
            odometry,
            self._registered_return_end_s,
            self._registered_return_start_s,
            return_speed_limit,
        )
        result.candidate_path = continuation
        result.output_path = continuation
        result.status += (
            "; registered return active, requested_shift={:.2f} m, "
            "actual_shift={:.2f} m, speed_limit={:.2f} m/s, extended={}, "
            "reanchored={}, tracking_error={:.2f} m"
        ).format(
            requested_shift,
            actual_shift,
            return_speed_limit,
            profile_extended,
            profile_reanchored,
            tracking_error,
        )
        return True

    def process(self, path, objects, odometry, stamp):
        result = self.engine.plan(path, objects, odometry, stamp)
        if self._avoid_in_progress and self._continue_committed_avoidance(
            result, objects, odometry, stamp
        ):
            return result
        if result.direction is None or not result.shift_lines:
            if self._return_in_progress and self._continue_registered_return(
                result, odometry, stamp
            ):
                return result
            self.avoid_rtc_controller.complete(stamp)
            self.return_rtc_controller.complete(stamp)
            has_blocking_object = any(
                candidate.avoid_required for candidate in result.candidates
            )
            if not has_blocking_object:
                result.output_path = result.reference_path
            return result

        full_candidate = deepcopy(result.candidate_path)
        path_end = path_cumulative_lengths(result.reference_path)[-1]
        avoid_only_lines = deepcopy(result.shift_lines)
        for line in avoid_only_lines:
            line.return_start_s = path_end + 1.0
            line.return_end_s = path_end + 2.0
        expanded_reference = deepcopy(result.reference_path)
        expanded_reference.left_bound = deepcopy(result.candidate_path.left_bound)
        expanded_reference.right_bound = deepcopy(result.candidate_path.right_bound)
        avoid_candidate, _ = self.engine._shift_path(
            expanded_reference, avoid_only_lines, expand_bounds=False
        )

        execution_ready = (
            result.safe
            and result.comfortable
            and result.valid
            and not result.request_operator
        )
        rtc_execution_ready = execution_ready or self._return_in_progress
        activated, state, _, auto_mode = self.avoid_rtc_controller.evaluate(
            result.direction,
            result.signature,
            rtc_execution_ready,
            result.start_distance,
            result.finish_distance,
            stamp,
            requested=result.request_operator,
        )
        result.activated = bool(
            activated and (result.ready or self._return_in_progress)
        )
        result.auto_mode = auto_mode
        result.rtc_state = state
        result.candidate_path = avoid_candidate

        if result.activated:
            result.output_path = self._apply_longitudinal_constraints(
                avoid_candidate,
                result.reference_path,
                odometry,
                max(
                    (line.return_end_s for line in result.shift_lines),
                    default=path_end,
                ),
            )
        else:
            self.return_rtc_controller.complete(stamp)

        state_name = {
            State.WAITING_FOR_EXECUTION: "waiting",
            State.RUNNING: "running",
            State.ABORTING: "aborting",
            State.SUCCEEDED: "succeeded",
            State.FAILED: "failed",
        }.get(state, "unknown")
        result.status += "; rtc={} {}, activated={}, request_operator={}".format(
            "auto" if auto_mode else "manual",
            state_name,
            result.activated,
            result.request_operator,
        )

        if (
            result.activated
            and self.params.approval_policy == "per_shift_line"
            and not self._avoid_in_progress
        ):
            self._register_avoidance(
                result, avoid_candidate, full_candidate
            )

        if not result.activated or self.params.approval_policy != "per_shift_line":
            if result.activated and self.params.approval_policy != "per_shift_line":
                result.candidate_path = full_candidate
                return_start_s = min(
                    (line.return_start_s for line in result.shift_lines),
                    default=path_end,
                )
                result.output_path = self._apply_longitudinal_constraints(
                    result.candidate_path,
                    result.reference_path,
                    odometry,
                    max(
                        (line.return_end_s for line in result.shift_lines),
                        default=path_end,
                    ),
                    return_start_s,
                    self._return_velocity_limit(self._odometry_speed(odometry)),
                )
            return result

        ego_s, _ = self.engine._ego_state(result.reference_path, odometry)
        main_shift = max(result.shift_lines, key=lambda line: abs(line.shift_length))
        current_shift = main_shift.value_at(ego_s)
        remaining_ratio = (
            (main_shift.shift_length - current_shift) / main_shift.shift_length
            if abs(main_shift.shift_length) > 1.0e-6
            else 0.0
        )
        result.return_ready = self._return_in_progress or (
            execution_ready
            and remaining_ratio < self.params.return_approval_ratio + 1.0e-3
        )
        return_signature = result.signature + (("return", result.return_direction),)
        return_activated, return_state, _, _ = self.return_rtc_controller.evaluate(
            result.return_direction,
            return_signature,
            result.return_ready,
            result.return_start_distance,
            result.return_finish_distance,
            stamp,
        )
        result.candidate_path = full_candidate
        if return_activated and result.return_ready:
            self._register_return(
                result, full_candidate, return_signature, odometry
            )
            self._clear_committed_avoidance(stamp)
            return_speed_limit = self._registered_return_speed_limit
            if return_speed_limit is None:
                distance_to_return = max(main_shift.return_start_s - ego_s, 0.0)
                return_speed_limit = min(
                    self.params.nominal_return_speed,
                    math.sqrt(
                        self._odometry_speed(odometry) ** 2
                        + 2.0
                        * self.params.max_acceleration
                        * distance_to_return
                    ),
                )
            result.output_path = self._apply_longitudinal_constraints(
                full_candidate,
                result.reference_path,
                odometry,
                max(
                    (line.return_end_s for line in result.shift_lines),
                    default=path_end,
                ),
                min(
                    (line.return_start_s for line in result.shift_lines),
                    default=path_end,
                ),
                return_speed_limit,
            )
        return_state_name = {
            State.WAITING_FOR_EXECUTION: "waiting",
            State.RUNNING: "running",
            State.ABORTING: "aborting",
            State.SUCCEEDED: "succeeded",
            State.FAILED: "failed",
        }.get(return_state, "unknown")
        result.status += "; return_rtc={} {}, activated={}".format(
            "ready" if result.return_ready else "held",
            return_state_name,
            bool(return_activated and result.return_ready),
        )
        return result
