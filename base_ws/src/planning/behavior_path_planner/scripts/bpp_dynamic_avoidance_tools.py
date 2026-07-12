#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass, field

import rospy
from geometry_msgs.msg import Point

from autoware_perception_msgs.msg import ObjectClassification, Shape

from bpp_path_tools import (
    clamp,
    object_pose,
    path_cumulative_lengths,
    project_pose_to_path,
    uuid_to_string,
    yaw_from_quaternion,
)


REGULATED_LABELS = {
    ObjectClassification.CAR,
    ObjectClassification.TRUCK,
    ObjectClassification.BUS,
    ObjectClassification.TRAILER,
    ObjectClassification.MOTORCYCLE,
}
UNREGULATED_LABELS = {
    ObjectClassification.UNKNOWN,
    ObjectClassification.BICYCLE,
    ObjectClassification.PEDESTRIAN,
}


def _param(prefix, name, default):
    return rospy.get_param("{}/{}".format(prefix.rstrip("/"), name.lstrip("/")), default)


def _bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def _stamp_seconds(stamp):
    if stamp is None:
        return rospy.Time.now().to_sec()
    return stamp.to_sec() if hasattr(stamp, "to_sec") else float(stamp)


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _label(obj):
    if not obj.classification:
        return ObjectClassification.UNKNOWN
    return int(max(obj.classification, key=lambda item: item.probability).label)


def _dimensions(obj):
    length = max(float(obj.shape.dimensions.x), 0.1)
    width = max(float(obj.shape.dimensions.y), 0.1)
    if obj.shape.type == Shape.CYLINDER:
        width = length
    if obj.shape.footprint.points:
        xs = [float(point.x) for point in obj.shape.footprint.points]
        ys = [float(point.y) for point in obj.shape.footprint.points]
        length = max(length, max(xs) - min(xs))
        width = max(width, max(ys) - min(ys))
    return length, width


def _path_yaw(path, index):
    index = int(clamp(index, 0, len(path.points) - 1))
    if len(path.points) < 2:
        return yaw_from_quaternion(path.points[index].point.pose.orientation)
    before = path.points[max(index - 1, 0)].point.pose.position
    after = path.points[min(index + 1, len(path.points) - 1)].point.pose.position
    if _distance(before, after) < 1.0e-6:
        return yaw_from_quaternion(path.points[index].point.pose.orientation)
    return math.atan2(after.y - before.y, after.x - before.x)


def _projected_half_extents(obj, path_yaw):
    length, width = _dimensions(obj)
    relative = _normalize_angle(yaw_from_quaternion(object_pose(obj).orientation) - path_yaw)
    lon = 0.5 * (abs(math.cos(relative)) * length + abs(math.sin(relative)) * width)
    lat = 0.5 * (abs(math.sin(relative)) * length + abs(math.cos(relative)) * width)
    return lon, lat


def _object_velocity(obj, path_yaw):
    twist = obj.kinematics.initial_twist_with_covariance.twist.linear
    relative = _normalize_angle(yaw_from_quaternion(object_pose(obj).orientation) - path_yaw)
    tangent = math.cos(relative) * twist.x - math.sin(relative) * twist.y
    normal = math.sin(relative) * twist.x + math.cos(relative) * twist.y
    return tangent, normal, math.hypot(twist.x, twist.y)


def _ego_speed(odometry):
    if odometry is None:
        return 0.0
    linear = odometry.twist.twist.linear
    return math.hypot(linear.x, linear.y)


def _nearest_point_on_polyline(polyline, target):
    if not polyline:
        return None
    if len(polyline) == 1:
        return deepcopy(polyline[0])
    best = None
    best_distance = float("inf")
    for first, second in zip(polyline[:-1], polyline[1:]):
        vx = second.x - first.x
        vy = second.y - first.y
        length_sq = vx * vx + vy * vy
        ratio = 0.0
        if length_sq > 1.0e-12:
            ratio = clamp(
                ((target.x - first.x) * vx + (target.y - first.y) * vy) / length_sq,
                0.0,
                1.0,
            )
        point = Point()
        point.x = first.x + ratio * vx
        point.y = first.y + ratio * vy
        point.z = first.z + ratio * (second.z - first.z)
        distance = _distance(point, target)
        if distance < best_distance:
            best = point
            best_distance = distance
    return best


def _interpolate_path(path, cumulative, station):
    if station <= 0.0:
        return deepcopy(path.points[0].point.pose)
    if station >= cumulative[-1]:
        return deepcopy(path.points[-1].point.pose)
    for index in range(len(cumulative) - 1):
        if station > cumulative[index + 1]:
            continue
        span = max(cumulative[index + 1] - cumulative[index], 1.0e-6)
        ratio = (station - cumulative[index]) / span
        first = path.points[index].point.pose
        second = path.points[index + 1].point.pose
        pose = deepcopy(first)
        pose.position.x += ratio * (second.position.x - first.position.x)
        pose.position.y += ratio * (second.position.y - first.position.y)
        pose.position.z += ratio * (second.position.z - first.position.z)
        yaw = math.atan2(
            second.position.y - first.position.y,
            second.position.x - first.position.x,
        )
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(0.5 * yaw)
        pose.orientation.w = math.cos(0.5 * yaw)
        return pose
    return deepcopy(path.points[-1].point.pose)


def _offset_point(pose, offset):
    yaw = yaw_from_quaternion(pose.orientation)
    point = Point()
    point.x = pose.position.x - math.sin(yaw) * offset
    point.y = pose.position.y + math.cos(yaw) * offset
    point.z = pose.position.z
    return point


@dataclass
class DynamicAvoidanceParameters:
    enable_debug_info: bool = False
    avoid_car: bool = True
    avoid_truck: bool = True
    avoid_bus: bool = True
    avoid_trailer: bool = True
    avoid_unknown: bool = False
    avoid_bicycle: bool = True
    avoid_motorcycle: bool = True
    avoid_pedestrian: bool = True
    max_obstacle_vel: float = 100.0
    min_obstacle_vel: float = 0.0
    successive_entry_count: int = 5
    successive_exit_count: int = 1
    min_obj_lat_offset: float = 0.0
    max_obj_lat_offset: float = 1.0
    min_cut_in_time: float = 1.0
    min_cut_in_lon_offset: float = 0.0
    min_cut_in_velocity: float = 0.5
    max_cut_out_time: float = 2.0
    min_cut_out_lat_velocity: float = 0.3
    min_cut_out_velocity: float = 0.5
    min_overtaking_crossing_velocity: float = 1.0
    max_overtaking_crossing_angle: float = 1.05
    min_oncoming_crossing_velocity: float = 1.0
    max_oncoming_crossing_angle: float = 0.523
    max_pedestrian_crossing_velocity: float = 0.8
    max_front_object_angle: float = 0.785
    min_front_object_velocity: float = -0.5
    max_front_cover_ratio: float = 0.3
    max_stopped_object_velocity: float = 0.5
    lateral_margin: float = 1.0
    pedestrian_margin: float = 2.0
    predicted_path_end_time: float = 2.0
    predicted_path_confidence: float = 0.0
    max_lateral_avoidance: float = 0.5
    max_object_lateral_shift_time: float = 0.0
    lateral_lpf_gain: float = 0.9
    max_ego_lateral_acceleration: float = 0.3
    max_ego_lateral_jerk: float = 0.3
    ego_shift_delay: float = 1.0
    max_overtaking_ttc: float = 40.0
    overtaking_start_duration: float = 1.0
    overtaking_end_duration: float = 1.0
    overtaking_hold_duration: float = 3.0
    max_oncoming_ttc: float = 40.0
    oncoming_start_duration: float = 1.0
    oncoming_end_duration: float = 0.0
    vehicle_width: float = 1.896
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.1
    default_bound_width: float = 2.5

    @classmethod
    def from_ros(cls, prefix="~dynamic_obstacle_avoidance"):
        p = cls()
        names = {
            "enable_debug_info": "common/enable_debug_info",
            "avoid_car": "target_object/car",
            "avoid_truck": "target_object/truck",
            "avoid_bus": "target_object/bus",
            "avoid_trailer": "target_object/trailer",
            "avoid_unknown": "target_object/unknown",
            "avoid_bicycle": "target_object/bicycle",
            "avoid_motorcycle": "target_object/motorcycle",
            "avoid_pedestrian": "target_object/pedestrian",
            "max_obstacle_vel": "target_object/max_obstacle_vel",
            "min_obstacle_vel": "target_object/min_obstacle_vel",
            "successive_entry_count": "target_object/successive_num_to_entry_dynamic_avoidance_condition",
            "successive_exit_count": "target_object/successive_num_to_exit_dynamic_avoidance_condition",
            "min_obj_lat_offset": "target_object/min_obj_lat_offset_to_ego_path",
            "max_obj_lat_offset": "target_object/max_obj_lat_offset_to_ego_path",
            "min_cut_in_time": "target_object/cut_in_object/min_time_to_start_cut_in",
            "min_cut_in_lon_offset": "target_object/cut_in_object/min_lon_offset_ego_to_object",
            "min_cut_in_velocity": "target_object/cut_in_object/min_object_vel",
            "max_cut_out_time": "target_object/cut_out_object/max_time_from_outside_ego_path",
            "min_cut_out_lat_velocity": "target_object/cut_out_object/min_object_lat_vel",
            "min_cut_out_velocity": "target_object/cut_out_object/min_object_vel",
            "min_overtaking_crossing_velocity": "target_object/crossing_object/min_overtaking_object_vel",
            "max_overtaking_crossing_angle": "target_object/crossing_object/max_overtaking_object_angle",
            "min_oncoming_crossing_velocity": "target_object/crossing_object/min_oncoming_object_vel",
            "max_oncoming_crossing_angle": "target_object/crossing_object/max_oncoming_object_angle",
            "max_pedestrian_crossing_velocity": "target_object/crossing_object/max_pedestrian_crossing_vel",
            "max_front_object_angle": "target_object/front_object/max_object_angle",
            "min_front_object_velocity": "target_object/front_object/min_object_vel",
            "max_front_cover_ratio": "target_object/front_object/max_ego_path_lat_cover_ratio",
            "max_stopped_object_velocity": "target_object/stopped_object/max_object_vel",
            "lateral_margin": "drivable_area_generation/lat_offset_from_obstacle",
            "pedestrian_margin": "drivable_area_generation/margin_distance_around_pedestrian",
            "predicted_path_end_time": "drivable_area_generation/predicted_path/end_time_to_consider",
            "predicted_path_confidence": "drivable_area_generation/predicted_path/threshold_confidence",
            "max_lateral_avoidance": "drivable_area_generation/max_lat_offset_to_avoid",
            "max_object_lateral_shift_time": "drivable_area_generation/max_time_for_object_lat_shift",
            "lateral_lpf_gain": "drivable_area_generation/lpf_gain_for_lat_avoid_to_offset",
            "max_ego_lateral_acceleration": "drivable_area_generation/max_ego_lat_acc",
            "max_ego_lateral_jerk": "drivable_area_generation/max_ego_lat_jerk",
            "ego_shift_delay": "drivable_area_generation/delay_time_ego_shift",
            "max_overtaking_ttc": "drivable_area_generation/overtaking_object/max_time_to_collision",
            "overtaking_start_duration": "drivable_area_generation/overtaking_object/start_duration_to_avoid",
            "overtaking_end_duration": "drivable_area_generation/overtaking_object/end_duration_to_avoid",
            "overtaking_hold_duration": "drivable_area_generation/overtaking_object/duration_to_hold_avoidance",
            "max_oncoming_ttc": "drivable_area_generation/oncoming_object/max_time_to_collision",
            "oncoming_start_duration": "drivable_area_generation/oncoming_object/start_duration_to_avoid",
            "oncoming_end_duration": "drivable_area_generation/oncoming_object/end_duration_to_avoid",
            "vehicle_width": "vehicle_width",
            "base_link_to_front": "base_link_to_front",
            "base_link_to_rear": "base_link_to_rear",
            "default_bound_width": "default_bound_width",
        }
        bool_fields = {
            "enable_debug_info", "avoid_car", "avoid_truck", "avoid_bus", "avoid_trailer",
            "avoid_unknown", "avoid_bicycle", "avoid_motorcycle", "avoid_pedestrian",
        }
        int_fields = {"successive_entry_count", "successive_exit_count"}
        for field_name, ros_name in names.items():
            value = _param(prefix, ros_name, getattr(p, field_name))
            if field_name in bool_fields:
                value = _bool(value)
            elif field_name in int_fields:
                value = int(value)
            else:
                value = float(value)
            setattr(p, field_name, value)
        return p

    def enabled_labels(self):
        values = {
            ObjectClassification.CAR: self.avoid_car,
            ObjectClassification.TRUCK: self.avoid_truck,
            ObjectClassification.BUS: self.avoid_bus,
            ObjectClassification.TRAILER: self.avoid_trailer,
            ObjectClassification.UNKNOWN: self.avoid_unknown,
            ObjectClassification.BICYCLE: self.avoid_bicycle,
            ObjectClassification.MOTORCYCLE: self.avoid_motorcycle,
            ObjectClassification.PEDESTRIAN: self.avoid_pedestrian,
        }
        return {label for label, enabled in values.items() if enabled}


@dataclass
class DynamicCandidate:
    object_id: str
    label: int
    longitudinal_distance: float = 0.0
    lateral_distance: float = 0.0
    path_distance: float = 0.0
    tangent_velocity: float = 0.0
    normal_velocity: float = 0.0
    time_to_collision: float = float("inf")
    collision_side: str = ""
    start_station: float = 0.0
    end_station: float = 0.0
    inner_bound_offset: float = 0.0
    outer_bound_offset: float = 0.0
    allow_avoidance: bool = False
    failed_reason: str = ""

    def debug_dict(self):
        ttc = self.time_to_collision if math.isfinite(self.time_to_collision) else 0.0
        return {
            "object_id": self.object_id,
            "allow_avoidance": self.allow_avoidance,
            "longitudinal_distance": self.longitudinal_distance,
            "lateral_distance": self.lateral_distance,
            "path_distance": self.path_distance,
            "shift_length": abs(self.inner_bound_offset),
            "required_jerk": ttc,
            "maximum_jerk": abs(self.normal_velocity),
            "failed_reason": self.failed_reason,
        }


@dataclass
class DynamicAvoidanceResult:
    output_path: object
    candidates: list = field(default_factory=list)
    targets: list = field(default_factory=list)
    status: str = "dynamic_obstacle_avoidance: pass-through"


@dataclass
class _ObjectTrack:
    counter: int = 0
    valid: bool = False
    latest_inside_time: object = None
    previous_inner_bound: object = None
    last_candidate: object = None
    last_info: object = None


class DynamicAvoidanceEngine:
    def __init__(self, parameters=None):
        self.parameters = parameters or DynamicAvoidanceParameters()
        self.tracks = {}

    def _rough_filter(self, path, obj, now):
        params = self.parameters
        obj_id = uuid_to_string(obj.object_id)
        label = _label(obj)
        candidate = DynamicCandidate(obj_id, label)
        if label not in params.enabled_labels():
            candidate.failed_reason = "unsupported_object_type"
            return candidate, None

        projection = project_pose_to_path(path, object_pose(obj))
        if projection is None:
            candidate.failed_reason = "path_projection_failed"
            return candidate, None
        candidate.longitudinal_distance = projection.longitudinal_distance
        candidate.lateral_distance = projection.signed_lateral_distance
        candidate.path_distance = projection.distance
        yaw = _path_yaw(path, projection.index)
        tangent, normal, speed = _object_velocity(obj, yaw)
        candidate.tangent_velocity = tangent
        candidate.normal_velocity = normal
        regulated = label in REGULATED_LABELS
        checked_speed = abs(tangent) if regulated else speed
        if checked_speed < params.min_obstacle_vel or checked_speed > params.max_obstacle_vel:
            candidate.failed_reason = "outside_velocity_range"
            return candidate, None

        if regulated:
            object_angle = abs(
                _normalize_angle(yaw_from_quaternion(object_pose(obj).orientation) - yaw)
            )
            crossing_limit = (
                params.max_overtaking_crossing_angle
                if tangent >= 0.0
                else params.max_oncoming_crossing_angle
            )
            crossing_speed = (
                params.min_overtaking_crossing_velocity
                if tangent >= 0.0
                else params.min_oncoming_crossing_velocity
            )
            crossing = crossing_limit < object_angle < math.pi - crossing_limit
            if crossing and speed > crossing_speed:
                candidate.failed_reason = "crossing_object"
                return candidate, None
            _, half_lat = _projected_half_extents(obj, yaw)
            minimum_gap = max(
                0.0,
                abs(projection.signed_lateral_distance)
                - 0.5 * params.vehicle_width
                - math.hypot(*_dimensions(obj)) * 0.5,
            )
            if minimum_gap > params.max_obj_lat_offset:
                candidate.failed_reason = "outside_path_corridor"
                return candidate, None
        elif abs(normal) > params.max_pedestrian_crossing_velocity:
            candidate.failed_reason = "fast_unregulated_crossing_object"
            return candidate, None

        on_ego_path = abs(projection.signed_lateral_distance) < (
            0.5 * params.vehicle_width + params.min_obj_lat_offset
        )
        track = self.tracks.setdefault(obj_id, _ObjectTrack())
        if on_ego_path:
            track.latest_inside_time = now
        info = {
            "object": obj,
            "projection": projection,
            "path_yaw": yaw,
            "tangent": tangent,
            "normal": normal,
            "speed": speed,
            "regulated": regulated,
            "on_ego_path": on_ego_path,
            "half_lat": half_lat if regulated else 0.5 * _dimensions(obj)[1],
        }
        track.last_candidate = deepcopy(candidate)
        track.last_info = info
        return candidate, info

    def _update_tracks(self, eligible_ids):
        params = self.parameters
        for obj_id, track in list(self.tracks.items()):
            if obj_id in eligible_ids:
                track.counter = min(params.successive_entry_count, max(1, track.counter + 1))
            else:
                track.counter = max(0, track.counter - 1)
            if not track.valid and track.counter >= params.successive_entry_count:
                track.valid = True
            if track.valid and track.counter < params.successive_exit_count:
                track.valid = False
            if track.counter == 0 and not track.valid:
                del self.tracks[obj_id]

    def _predicted_path_cuts_in(self, path, obj, object_s, tangent, ego_s, ego_speed):
        params = self.parameters
        if tangent < params.min_cut_in_velocity:
            return False
        predicted_paths = list(obj.kinematics.predicted_paths)
        if not predicted_paths:
            return False
        predicted = max(predicted_paths, key=lambda item: item.confidence)
        will_enter = any(
            abs(project_pose_to_path(path, pose).signed_lateral_distance)
            < 0.5 * params.vehicle_width
            for pose in predicted.path
            if project_pose_to_path(path, pose) is not None
        )
        if not will_enter:
            return False
        relative = ego_speed - tangent
        threshold = max(params.min_cut_in_lon_offset, relative * params.min_cut_in_time)
        return object_s - ego_s >= threshold

    def _precise_filter(self, path, odometry, candidate, info, now):
        params = self.parameters
        obj = info["object"]
        projection = info["projection"]
        tangent = info["tangent"]
        normal = info["normal"]
        object_s = projection.longitudinal_distance
        ego_projection = project_pose_to_path(path, odometry.pose.pose) if odometry else None
        ego_s = ego_projection.longitudinal_distance if ego_projection else 0.0
        ego_speed = _ego_speed(odometry)
        lon_half, lat_half = _projected_half_extents(obj, info["path_yaw"])

        if info["regulated"]:
            object_angle = abs(
                _normalize_angle(
                    yaw_from_quaternion(object_pose(obj).orientation) - info["path_yaw"]
                )
            )
            aligned = (
                object_angle < params.max_front_object_angle
                or math.pi - params.max_front_object_angle < object_angle
            )
            if info["on_ego_path"] and aligned and params.min_front_object_velocity < tangent:
                candidate.failed_reason = "front_object_to_follow"
                return candidate
            if self._predicted_path_cuts_in(path, obj, object_s, tangent, ego_s, ego_speed):
                candidate.failed_reason = "cut_in_object"
                return candidate
            track = self.tracks[candidate.object_id]
            moving_outward = (
                candidate.lateral_distance > 0.0 and normal > params.min_cut_out_lat_velocity
            ) or (
                candidate.lateral_distance < 0.0 and normal < -params.min_cut_out_lat_velocity
            )
            if (
                tangent >= params.min_cut_out_velocity
                and track.latest_inside_time is not None
                and now - track.latest_inside_time <= params.max_cut_out_time
                and moving_outward
            ):
                candidate.failed_reason = "cut_out_object"
                return candidate

        clearance = object_s - ego_s - lon_half - params.base_link_to_front
        if clearance < 0.0:
            candidate.failed_reason = "behind_ego"
            return candidate
        relative_velocity = ego_speed - tangent
        ttc = clearance / relative_velocity if relative_velocity > 1.0e-6 else float("inf")
        candidate.time_to_collision = max(ttc, 0.0)
        if info["speed"] > params.max_stopped_object_velocity:
            max_ttc = params.max_overtaking_ttc if tangent >= 0.0 else params.max_oncoming_ttc
            if not math.isfinite(ttc) or ttc > max_ttc:
                candidate.failed_reason = "time_to_collision_too_large"
                return candidate
            if ttc < -params.overtaking_hold_duration:
                candidate.failed_reason = "collision_already_passed"
                return candidate

        collision_left = candidate.lateral_distance > 0.0
        candidate.collision_side = "left" if collision_left else "right"
        near_edge = (
            candidate.lateral_distance - lat_half
            if collision_left
            else candidate.lateral_distance + lat_half
        )
        if info["regulated"] and tangent > params.min_front_object_velocity:
            covered = min(candidate.lateral_distance + lat_half, 0.5 * params.vehicle_width) - max(
                candidate.lateral_distance - lat_half, -0.5 * params.vehicle_width
            )
            if covered > params.vehicle_width * params.max_front_cover_ratio:
                candidate.failed_reason = "front_object_covers_ego_path"
                return candidate

        reserve = 0.5 * params.vehicle_width - params.max_lateral_avoidance
        lateral_shift = max(0.0, normal * (-1.0 if collision_left else 1.0))
        raw_inner = near_edge - (params.lateral_margin + lateral_shift * params.max_object_lateral_shift_time) * (
            1.0 if collision_left else -1.0
        )
        inner = max(raw_inner, reserve) if collision_left else min(raw_inner, -reserve)
        track = self.tracks[candidate.object_id]
        if track.previous_inner_bound is not None:
            gain = clamp(params.lateral_lpf_gain, 0.0, 1.0)
            inner = gain * track.previous_inner_bound + (1.0 - gain) * inner
        track.previous_inner_bound = inner
        candidate.inner_bound_offset = inner
        candidate.outer_bound_offset = (
            candidate.lateral_distance + lat_half + params.lateral_margin
            if collision_left
            else candidate.lateral_distance - lat_half - params.lateral_margin
        )

        if tangent < params.max_stopped_object_velocity:
            start_margin = min(max(ttc, 0.0), 3.5) * abs(tangent) + abs(relative_velocity) * params.oncoming_start_duration
            end_margin = abs(relative_velocity) * params.oncoming_end_duration
        else:
            start_margin = abs(relative_velocity) * params.overtaking_start_duration
            end_margin = min(max(ttc, 0.0), 3.0) * tangent + abs(relative_velocity) * params.overtaking_end_duration
        candidate.start_station = max(0.0, object_s - lon_half - start_margin)
        candidate.end_station = min(
            path_cumulative_lengths(path)[-1], object_s + lon_half + max(0.0, end_margin)
        )
        candidate.allow_avoidance = True
        candidate.failed_reason = ""
        return candidate

    def _apply_bound_cutouts(self, path, targets):
        output = deepcopy(path)
        cumulative = path_cumulative_lengths(path)
        if not cumulative or not targets:
            return output
        stations = list(cumulative)
        for target in targets:
            stations.extend((target.start_station, target.end_station))
        stations = sorted(set(round(clamp(station, 0.0, cumulative[-1]), 6) for station in stations))
        left_bound = []
        right_bound = []
        for station in stations:
            pose = _interpolate_path(path, cumulative, station)
            center = pose.position
            left_original = _nearest_point_on_polyline(path.left_bound, center)
            right_original = _nearest_point_on_polyline(path.right_bound, center)
            yaw = yaw_from_quaternion(pose.orientation)
            nx = -math.sin(yaw)
            ny = math.cos(yaw)
            left_offset = (
                (left_original.x - center.x) * nx + (left_original.y - center.y) * ny
                if left_original is not None
                else self.parameters.default_bound_width
            )
            right_offset = (
                (right_original.x - center.x) * nx + (right_original.y - center.y) * ny
                if right_original is not None
                else -self.parameters.default_bound_width
            )
            left_offset = max(left_offset, 0.05)
            right_offset = min(right_offset, -0.05)
            for target in targets:
                if target.start_station - 1.0e-6 <= station <= target.end_station + 1.0e-6:
                    if target.collision_side == "left":
                        left_offset = min(left_offset, target.inner_bound_offset)
                    else:
                        right_offset = max(right_offset, target.inner_bound_offset)
            left_bound.append(_offset_point(pose, left_offset))
            right_bound.append(_offset_point(pose, right_offset))
        output.left_bound = left_bound
        output.right_bound = right_bound
        return output

    def plan(self, path, objects, odometry, stamp=None):
        output = deepcopy(path)
        if not path.points or objects is None:
            return DynamicAvoidanceResult(output_path=output)
        now = _stamp_seconds(stamp)
        rough = []
        eligible = {}
        for obj in objects.objects:
            candidate, info = self._rough_filter(path, obj, now)
            rough.append(candidate)
            if info is not None:
                eligible[candidate.object_id] = (candidate, info)
        self._update_tracks(set(eligible))

        for obj_id, track in self.tracks.items():
            if obj_id in eligible or not track.valid or track.last_info is None:
                continue
            eligible[obj_id] = (deepcopy(track.last_candidate), track.last_info)
            rough.append(eligible[obj_id][0])

        candidates = []
        for candidate in rough:
            entry = eligible.get(candidate.object_id)
            if entry is None:
                candidates.append(candidate)
                continue
            track = self.tracks.get(candidate.object_id)
            if track is None or not track.valid:
                candidate.failed_reason = "insufficient_successive_observations"
                candidates.append(candidate)
                continue
            candidates.append(self._precise_filter(path, odometry, candidate, entry[1], now))
        targets = [candidate for candidate in candidates if candidate.allow_avoidance]
        output = self._apply_bound_cutouts(path, targets)
        if targets:
            details = ", ".join(
                "{} {} s={:.1f}-{:.1f}m ttc={:.1f}s".format(
                    item.object_id[:8], item.collision_side, item.start_station,
                    item.end_station, item.time_to_collision,
                )
                for item in targets
            )
            status = "dynamic_obstacle_avoidance: {} target(s), bounds cut: {}".format(
                len(targets), details
            )
        else:
            status = "dynamic_obstacle_avoidance: pass-through, no dynamic target"
        return DynamicAvoidanceResult(output, candidates, targets, status)


class DynamicAvoidanceScene:
    def __init__(self, prefix="~dynamic_obstacle_avoidance"):
        self.engine = DynamicAvoidanceEngine(DynamicAvoidanceParameters.from_ros(prefix))

    def process(self, path, objects, odometry, stamp=None):
        return self.engine.plan(path, objects, odometry, stamp)
