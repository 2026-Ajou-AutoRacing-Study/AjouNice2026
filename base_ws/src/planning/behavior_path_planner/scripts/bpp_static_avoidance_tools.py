#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass, field

import rospy

from autoware_perception_msgs.msg import ObjectClassification
from tier4_rtc_msgs.msg import Module, State

from geometry_msgs.msg import Point

from bpp_map_tools import (
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
    avoid_min_jerk: float = 0.2
    return_min_jerk: float = 0.2
    lateral_max_jerk: float = 1.0
    return_approval_ratio: float = 0.5
    stop_max_distance: float = 20.0
    stop_buffer: float = 1.0
    nominal_deceleration: float = -1.0
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
            (ObjectClassification.UNKNOWN, "unknown", 0.28, 1.0, 0.5, 0.2, 0.7, 0.0, 0.1),
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
    preferred_shift_length: float = 0.0
    available_shift: float = 0.0
    required_jerk: float = 0.0
    start_s: float = 0.0
    avoid_end_s: float = 0.0
    return_start_s: float = 0.0
    return_end_s: float = 0.0
    failed_reason: str = ""

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
        position = path.points[index].point.pose.position
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

    def _desired_shift(self, candidate, ego_speed, path_end):
        policy = self._policy(candidate.label)
        vehicle_half_width = 0.5 * self.params.vehicle_width
        hard_margin = (
            policy.parked_hard_margin if candidate.is_parked else policy.hard_margin
        )
        preferred_margin = vehicle_half_width + hard_margin + policy.soft_margin
        minimum_margin = vehicle_half_width + hard_margin

        if abs(candidate.lateral) < 1.0e-3:
            object_on_right = self.params.default_avoid_direction == "left"
        else:
            object_on_right = candidate.lateral < 0.0

        if object_on_right:
            inner_edge = candidate.lateral + candidate.half_lateral
            preferred = inner_edge + preferred_margin
            minimum = inner_edge + minimum_margin
            available = (
                candidate.expanded_left_bound
                - vehicle_half_width
                - self.params.soft_drivable_bound_margin
            )
            preferred = max(preferred, 0.0)
            minimum = max(minimum, 0.0)
            limit = self.params.max_left_shift_length
        else:
            inner_edge = candidate.lateral - candidate.half_lateral
            preferred = inner_edge - preferred_margin
            minimum = inner_edge - minimum_margin
            available = -(
                candidate.expanded_right_bound
                - vehicle_half_width
                - self.params.soft_drivable_bound_margin
            )
            preferred = min(preferred, 0.0)
            minimum = min(minimum, 0.0)
            limit = -self.params.max_right_shift_length

        candidate.preferred_shift_length = preferred
        candidate.available_shift = available
        if object_on_right:
            shift = min(preferred, available, limit)
            hard_space_available = available + 1.0e-6 >= minimum
        else:
            shift = max(preferred, available, limit)
            hard_space_available = available - 1.0e-6 <= minimum

        if not hard_space_available:
            candidate.failed_reason = "insufficient_drivable_space"
            return
        if abs(shift) < self.params.lateral_execution_threshold:
            candidate.failed_reason = "less_than_execution_threshold"
            candidate.avoid_required = False
            return

        quantize = max(self.params.quantize_size, 1.0e-3)
        shift = round(shift / quantize) * quantize
        candidate.shift_length = shift

        planning_speed = max(ego_speed, self.params.min_slow_down_speed)
        rear_constant = (
            self.params.base_link_to_rear
            + candidate.half_longitudinal
            + policy.longitudinal_margin
        )
        candidate.avoid_end_s = candidate.longitudinal + rear_constant
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
        candidate.return_start_s = candidate.avoid_end_s
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
        """Return vehicle-center shift limits along an expanded reference path."""
        expanded = deepcopy(path)
        self._expand_drivable_bounds(expanded)
        stations = path_cumulative_lengths(expanded)
        margin = 0.5 * self.params.vehicle_width + self.params.soft_drivable_bound_margin
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

        targets = [candidate for candidate in candidates if candidate.allow_avoidance]
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

        expanded_reference = None
        profile_stations = []
        profile_lower = []
        profile_upper = []
        if targets:
            (
                expanded_reference,
                profile_stations,
                profile_lower,
                profile_upper,
            ) = self._expanded_profile_bounds(reference)
            fitted_targets = []
            for target in targets:
                line = ShiftLine(
                    object_ids=[target.object_id],
                    direction=target.direction,
                    shift_length=target.shift_length,
                    start_s=target.start_s,
                    avoid_end_s=target.avoid_end_s,
                    return_start_s=target.return_start_s,
                    return_end_s=target.return_end_s,
                )
                if not self._fit_shift_line_to_bounds(
                    line,
                    profile_stations,
                    profile_lower,
                    profile_upper,
                    ego_speed,
                ):
                    target.allow_avoidance = False
                    target.failed_reason = "shift_profile_outside_drivable_bounds"
                    continue
                target.start_s = line.start_s
                target.avoid_end_s = line.avoid_end_s
                target.return_start_s = line.return_start_s
                target.return_end_s = line.return_end_s
                planning_speed = max(ego_speed, self.params.min_slow_down_speed)
                target.required_jerk = max(
                    self._required_lateral_jerk(
                        target.shift_length,
                        target.avoid_end_s - target.start_s,
                        planning_speed,
                    ),
                    self._required_lateral_jerk(
                        target.shift_length,
                        target.return_end_s - target.return_start_s,
                        planning_speed,
                    ),
                )
                fitted_targets.append(target)
            targets = fitted_targets

        lines, conflict = self._merge_shift_lines(targets)
        profile_valid = bool(lines) and all(
            self._line_is_within_bounds(
                line, profile_stations, profile_lower, profile_upper
            )
            for line in lines
        )
        if expanded_reference is None:
            expanded_reference, _, _, _ = self._expanded_profile_bounds(reference)
        candidate_path, _ = self._shift_path(
            expanded_reference, lines, expand_bounds=False
        )
        signature = tuple(
            (item.object_id, item.direction, round(item.shift_length, 2))
            for item in targets
        )
        if not profile_valid:
            raw_safe, safety_reason = False, "shift_profile_outside_drivable_bounds"
        else:
            raw_safe, safety_reason = self._raw_safety(
                candidate_path, targets, object_messages, conflict
            )
        safe = self._hysteresis_safety(raw_safe, signature)
        comfortable = all(
            item.required_jerk <= self.params.lateral_max_jerk + 0.1 for item in targets
        )
        valid = bool(lines) and not conflict and profile_valid
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

    def _registered_return_state(self, reference, candidate, odometry):
        projection = project_pose_to_path(reference, odometry.pose.pose)
        if projection is None:
            return 0.0, 0.0
        requested_shift = self._requested_shift_at(
            reference, candidate, projection.index
        )
        return requested_shift, projection.signed_lateral_distance

    def _register_return(self, result, candidate_path, return_signature):
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
        self.avoid_rtc_controller.complete(stamp)
        self.return_rtc_controller.complete(stamp)

    def _continue_registered_return(self, result, odometry, stamp):
        continuation, aligned_count = self._align_registered_return(
            result.candidate_path
        )
        requested_shift, actual_shift = self._registered_return_state(
            result.reference_path, continuation, odometry
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
        result.status += (
            "; registered return active, requested_shift={:.2f} m, "
            "actual_shift={:.2f} m"
        ).format(requested_shift, actual_shift)
        return True

    def process(self, path, objects, odometry, stamp):
        result = self.engine.plan(path, objects, odometry, stamp)
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
            result.output_path = avoid_candidate
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

        if not result.activated or self.params.approval_policy != "per_shift_line":
            if result.activated and self.params.approval_policy != "per_shift_line":
                result.candidate_path = full_candidate
                result.output_path = result.candidate_path
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
            result.output_path = full_candidate
            self._register_return(result, full_candidate, return_signature)
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
