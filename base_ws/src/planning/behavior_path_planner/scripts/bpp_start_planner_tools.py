#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass, field

import rospy

from autoware_perception_msgs.msg import ObjectClassification
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from tier4_rtc_msgs.msg import Module

from bpp_path_tools import (
    clamp,
    object_pose,
    object_speed,
    path_cumulative_lengths,
    project_pose_to_path,
    shifted_pose,
    smoothstep,
    uuid_to_string,
)
from rtc_interface_ros1 import RTCInterfaceROS1, RTCSceneController


VEHICLE_LABELS = {
    ObjectClassification.CAR,
    ObjectClassification.TRUCK,
    ObjectClassification.BUS,
    ObjectClassification.TRAILER,
    ObjectClassification.MOTORCYCLE,
    ObjectClassification.BICYCLE,
    ObjectClassification.PEDESTRIAN,
    ObjectClassification.UNKNOWN,
}


def _name(prefix, key):
    return "{}/{}".format(prefix.rstrip("/"), key.lstrip("/"))


def _param(prefix, key, default):
    return rospy.get_param(_name(prefix, key), default)


def _bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def _label(obj):
    if not obj.classification:
        return ObjectClassification.UNKNOWN
    return int(max(obj.classification, key=lambda item: item.probability).label)


def _object_half_width(obj):
    width = max(float(obj.shape.dimensions.y), 0.1)
    if obj.shape.footprint.points:
        ys = [float(point.y) for point in obj.shape.footprint.points]
        width = max(width, max(ys) - min(ys))
    return 0.5 * width


def _ego_speed(odometry):
    if odometry is None:
        return 0.0
    linear = odometry.twist.twist.linear
    return math.hypot(linear.x, linear.y)


def _shift_time(offset, jerk, lateral_acceleration):
    lateral = abs(float(offset))
    jerk = abs(float(jerk))
    acceleration = abs(float(lateral_acceleration))
    if lateral < 1.0e-8:
        return 0.0
    if jerk < 1.0e-8 or acceleration < 1.0e-8:
        return float("inf")

    jerk_time = acceleration / jerk
    acceleration_time = (
        math.sqrt(
            acceleration * acceleration
            + 4.0 * jerk * jerk * lateral / acceleration
        )
        - 3.0 * acceleration
    ) / (2.0 * jerk)
    if acceleration_time < 0.0:
        jerk_time = (lateral / (2.0 * jerk)) ** (1.0 / 3.0)
        acceleration_time = 0.0
    return 4.0 * jerk_time + 2.0 * acceleration_time


def _pull_out_distance(
    offset,
    jerk,
    lateral_acceleration,
    road_velocity,
    maximum_longitudinal_acceleration,
    maximum_curvature,
    minimum,
):
    shift_time = _shift_time(offset, jerk, lateral_acceleration)
    if not math.isfinite(shift_time) or shift_time <= 0.0:
        return 0.0, 0.0
    longitudinal_acceleration = clamp(
        abs(float(road_velocity)) / shift_time,
        0.0,
        max(float(maximum_longitudinal_acceleration), 1.0e-3),
    )
    acceleration_distance = 0.5 * longitudinal_acceleration * shift_time * shift_time
    curvature = max(abs(float(maximum_curvature)), 1.0e-6)
    lateral = abs(float(offset))
    curvature_distance = math.sqrt(
        max(4.0 * lateral / curvature - lateral * lateral, 0.0)
    )
    distance = max(float(minimum), acceleration_distance, curvature_distance)
    return distance, longitudinal_acceleration * shift_time


def _apply_pull_out(path, start_station, finish_station, initial_offset):
    output = deepcopy(path)
    for item, station in zip(output.points, path_cumulative_lengths(output)):
        if station <= start_station:
            offset = initial_offset
        elif station >= finish_station:
            offset = 0.0
        else:
            offset = initial_offset * (
                1.0 - smoothstep(start_station, finish_station, station)
            )
        item.point.pose = shifted_pose(item.point.pose, offset)
    return output


def _stop_path(path):
    output = deepcopy(path)
    for item in output.points:
        item.point.longitudinal_velocity_mps = 0.0
    return output


@dataclass
class StartPlannerParameters:
    arrived_distance: float = 1.0
    stopped_velocity: float = 0.01
    stopped_time: float = 1.0
    collision_margins: list = field(default_factory=lambda: [2.0, 1.0, 0.5, 0.1])
    front_object_margin: float = 5.0
    rear_object_margin: float = 2.0
    moving_object_velocity: float = 1.0
    middle_of_road_threshold: float = 0.5
    minimum_pull_out_distance: float = 0.0
    minimum_shift_length: float = 0.5
    lateral_jerk: float = 0.5
    minimum_lateral_acceleration: float = 0.15
    maximum_lateral_acceleration: float = 0.5
    maximum_curvature: float = 0.07
    maximum_longitudinal_acceleration: float = 1.0
    prepare_time: float = 0.0
    enable_safety_check: bool = True
    object_forward_distance: float = 10.0
    object_backward_distance: float = 100.0
    ignore_object_velocity: float = 0.25
    rear_reaction_time: float = 2.0
    rear_safety_time_margin: float = 1.0
    safety_lateral_threshold: float = 2.0
    safety_longitudinal_threshold: float = 3.0
    vehicle_width: float = 1.896
    vehicle_length: float = 4.89

    @classmethod
    def from_ros(cls, prefix="~start_planner"):
        p = cls()
        p.arrived_distance = float(_param(prefix, "th_arrived_distance", p.arrived_distance))
        p.stopped_velocity = float(_param(prefix, "th_stopped_velocity", p.stopped_velocity))
        p.stopped_time = float(_param(prefix, "th_stopped_time", p.stopped_time))
        p.collision_margins = [
            float(value) for value in _param(prefix, "collision_check_margins", p.collision_margins)
        ]
        p.front_object_margin = float(
            _param(prefix, "collision_check_margin_from_front_object", p.front_object_margin)
        )
        p.rear_object_margin = float(
            _param(prefix, "back_objects_collision_check_margin", p.rear_object_margin)
        )
        p.moving_object_velocity = float(
            _param(prefix, "th_moving_object_velocity", p.moving_object_velocity)
        )
        p.middle_of_road_threshold = float(
            _param(prefix, "th_distance_to_middle_of_the_road", p.middle_of_road_threshold)
        )
        p.minimum_pull_out_distance = float(
            _param(prefix, "minimum_shift_pull_out_distance", p.minimum_pull_out_distance)
        )
        p.minimum_shift_length = float(
            _param(prefix, "minimum_shift_length", p.minimum_shift_length)
        )
        p.lateral_jerk = float(_param(prefix, "lateral_jerk", p.lateral_jerk))
        p.minimum_lateral_acceleration = float(
            _param(prefix, "minimum_lateral_acc", p.minimum_lateral_acceleration)
        )
        p.maximum_lateral_acceleration = float(
            _param(prefix, "maximum_lateral_acc", p.maximum_lateral_acceleration)
        )
        p.maximum_curvature = float(
            _param(prefix, "maximum_curvature", p.maximum_curvature)
        )
        p.maximum_longitudinal_acceleration = float(
            _param(
                prefix,
                "maximum_longitudinal_acceleration",
                p.maximum_longitudinal_acceleration,
            )
        )
        p.prepare_time = float(_param(prefix, "prepare_time_before_start", p.prepare_time))
        p.enable_safety_check = _bool(
            _param(
                prefix,
                "path_safety_check/safety_check_params/enable_safety_check",
                p.enable_safety_check,
            )
        )
        p.object_forward_distance = float(
            _param(
                prefix,
                "path_safety_check/target_filtering/object_check_forward_distance",
                p.object_forward_distance,
            )
        )
        p.object_backward_distance = float(
            _param(
                prefix,
                "path_safety_check/target_filtering/object_check_backward_distance",
                p.object_backward_distance,
            )
        )
        p.ignore_object_velocity = float(
            _param(
                prefix,
                "path_safety_check/target_filtering/ignore_object_velocity_threshold",
                p.ignore_object_velocity,
            )
        )
        p.rear_reaction_time = float(
            _param(
                prefix,
                "path_safety_check/safety_check_params/rss_params/rear_vehicle_reaction_time",
                p.rear_reaction_time,
            )
        )
        p.rear_safety_time_margin = float(
            _param(
                prefix,
                "path_safety_check/safety_check_params/rss_params/rear_vehicle_safety_time_margin",
                p.rear_safety_time_margin,
            )
        )
        p.safety_lateral_threshold = float(
            _param(
                prefix,
                "path_safety_check/safety_check_params/rss_params/lateral_distance_max_threshold",
                p.safety_lateral_threshold,
            )
        )
        p.safety_longitudinal_threshold = float(
            _param(
                prefix,
                "path_safety_check/safety_check_params/rss_params/longitudinal_distance_min_threshold",
                p.safety_longitudinal_threshold,
            )
        )
        p.vehicle_width = float(_param(prefix, "vehicle_width", p.vehicle_width))
        p.vehicle_length = float(_param(prefix, "vehicle_length", p.vehicle_length))
        return p


@dataclass
class StartPlannerCandidate:
    object_id: str
    longitudinal_distance: float
    lateral_distance: float
    speed: float
    safe: bool
    failed_reason: str

    def debug_dict(self):
        return {
            "object_id": self.object_id,
            "allow_avoidance": self.safe,
            "longitudinal_distance": self.longitudinal_distance,
            "lateral_distance": self.lateral_distance,
            "path_distance": abs(self.lateral_distance),
            "shift_length": 0.0,
            "required_jerk": 0.0,
            "maximum_jerk": 0.0,
            "failed_reason": self.failed_reason,
        }


@dataclass
class StartPlannerResult:
    reference_path: object
    candidate_path: object
    stop_path: object
    output_path: object
    requested: bool = False
    safe: bool = True
    activated: bool = False
    lateral_offset: float = 0.0
    start_distance: float = 0.0
    finish_distance: float = 0.0
    direction: str = "none"
    candidates: list = field(default_factory=list)
    status: str = "start_planner: pass-through"


class StartPlannerEngine:
    def __init__(self, parameters=None):
        self.parameters = parameters or StartPlannerParameters()

    def _check_objects(self, candidate_path, objects, ego_station, finish_station):
        if objects is None:
            return True, [], ""
        p = self.parameters
        safe = True
        debug = []
        reason = ""
        margins = sorted(p.collision_margins, reverse=True)
        minimum_margin = margins[-1] if margins else 0.1
        for obj in objects.objects:
            if _label(obj) not in VEHICLE_LABELS:
                continue
            projection = project_pose_to_path(candidate_path, object_pose(obj))
            if projection is None:
                continue
            relative_s = projection.longitudinal_distance - ego_station
            speed = object_speed(obj)
            half_width = _object_half_width(obj)
            lateral_clearance = (
                abs(projection.signed_lateral_distance)
                - 0.5 * p.vehicle_width
                - half_width
            )
            object_safe = True
            failed_reason = ""
            if speed < p.moving_object_velocity:
                in_check_range = -p.rear_object_margin <= relative_s <= (
                    finish_station - ego_station + p.front_object_margin
                )
                if in_check_range and lateral_clearance < minimum_margin:
                    object_safe = False
                    failed_reason = "static_object_clearance"
            elif p.enable_safety_check:
                in_dynamic_range = (
                    -p.object_backward_distance <= relative_s <= p.object_forward_distance
                )
                if in_dynamic_range and abs(projection.signed_lateral_distance) < p.safety_lateral_threshold:
                    rear_closing_distance = (
                        speed * (p.rear_reaction_time + p.rear_safety_time_margin)
                        + p.safety_longitudinal_threshold
                    )
                    if relative_s < 0.0 and abs(relative_s) < rear_closing_distance:
                        object_safe = False
                        failed_reason = "rear_dynamic_object_risk"
            if not object_safe and safe:
                reason = failed_reason
            safe = safe and object_safe
            debug.append(
                StartPlannerCandidate(
                    uuid_to_string(obj.object_id),
                    relative_s,
                    projection.signed_lateral_distance,
                    speed,
                    object_safe,
                    failed_reason,
                )
            )
        return safe, debug, reason

    def plan(self, path, objects, odometry):
        reference = deepcopy(path)
        if not path.points or odometry is None:
            return StartPlannerResult(reference, reference, _stop_path(reference), reference)
        p = self.parameters
        ego_projection = project_pose_to_path(path, odometry.pose.pose)
        if ego_projection is None:
            return StartPlannerResult(reference, reference, _stop_path(reference), reference)
        offset = ego_projection.signed_lateral_distance
        remaining = path_cumulative_lengths(path)[-1] - ego_projection.longitudinal_distance
        stopped = _ego_speed(odometry) < p.stopped_velocity
        requested = (
            abs(offset) >= p.middle_of_road_threshold
            and remaining > p.arrived_distance
            and stopped
        )
        if not requested:
            reason = "on centerline" if abs(offset) < p.middle_of_road_threshold else "not stopped"
            return StartPlannerResult(
                reference, reference, _stop_path(reference), reference,
                requested=False, lateral_offset=offset,
                status="start_planner: pass-through, {}".format(reason),
            )

        road_velocity = abs(
            float(
                path.points[
                    min(ego_projection.index, len(path.points) - 1)
                ].point.longitudinal_velocity_mps
            )
        )
        start_station = ego_projection.longitudinal_distance + max(
            0.0, p.prepare_time * road_velocity
        )
        shift_distance, terminal_velocity = _pull_out_distance(
            offset,
            p.lateral_jerk,
            p.minimum_lateral_acceleration,
            road_velocity,
            p.maximum_longitudinal_acceleration,
            p.maximum_curvature,
            p.minimum_pull_out_distance,
        )
        finish_station = min(path_cumulative_lengths(path)[-1], start_station + shift_distance)
        candidate = _apply_pull_out(path, start_station, finish_station, offset)
        for item, station in zip(candidate.points, path_cumulative_lengths(candidate)):
            if ego_projection.longitudinal_distance <= station <= finish_station:
                item.point.longitudinal_velocity_mps = min(
                    item.point.longitudinal_velocity_mps, terminal_velocity
                )
        safe, object_debug, failed_reason = self._check_objects(
            candidate, objects, ego_projection.longitudinal_distance, finish_station
        )
        direction = "right" if offset > 0.0 else "left"
        stop = _stop_path(reference)
        status = "start_planner: shift pull-out {} {:.2f} m, safe={}".format(
            direction, abs(offset), str(safe).lower()
        )
        if failed_reason:
            status += ", {}".format(failed_reason)
        return StartPlannerResult(
            reference,
            candidate,
            stop,
            stop,
            requested=True,
            safe=safe,
            lateral_offset=offset,
            start_distance=max(0.0, start_station - ego_projection.longitudinal_distance),
            finish_distance=max(0.0, finish_station - ego_projection.longitudinal_distance),
            direction=direction,
            candidates=object_debug,
            status=status,
        )


class StartPlannerScene:
    def __init__(self, prefix="~start_planner"):
        self.prefix = prefix
        self.engine = StartPlannerEngine(StartPlannerParameters.from_ros(prefix))
        interface = RTCInterfaceROS1(
            Module.START_PLANNER,
            str(_param(prefix, "rtc/status_topic", "/planning/cooperate_status/start_planner")),
            str(_param(prefix, "rtc/command_service", "/planning/cooperate_commands/start_planner")),
            str(_param(prefix, "rtc/auto_mode_service", "/planning/enable_auto_mode/start_planner")),
            _bool(_param(prefix, "rtc/enable", False)),
        )
        self.rtc_controller = RTCSceneController({"start": interface})

    def process(self, path, objects, odometry, stamp):
        result = self.engine.plan(path, objects, odometry)
        if not result.requested:
            self.rtc_controller.complete(stamp)
            result.output_path = result.reference_path
            return result
        signature = (
            round(result.lateral_offset, 3),
            round(result.finish_distance, 2),
        )
        activated, state, _, auto_mode = self.rtc_controller.evaluate(
            "start",
            signature,
            result.safe,
            result.start_distance,
            result.finish_distance,
            stamp,
        )
        result.activated = activated
        result.output_path = result.candidate_path if activated and result.safe else result.stop_path
        result.status += "; rtc={} {}, state={}".format(
            "auto" if auto_mode else "manual",
            "running" if activated else "waiting",
            state,
        )
        return result

    @staticmethod
    def turn_command(result):
        if not result.requested:
            return TurnIndicatorsCommand.NO_COMMAND
        return (
            TurnIndicatorsCommand.ENABLE_RIGHT
            if result.direction == "right"
            else TurnIndicatorsCommand.ENABLE_LEFT
        )
