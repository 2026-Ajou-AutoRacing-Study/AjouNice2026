#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass

import rospy

from autoware_vehicle_msgs.msg import TurnIndicatorsCommand

from bpp_path_tools import (
    clamp,
    path_cumulative_lengths,
    project_pose_to_path,
    shifted_pose,
    smoothstep,
)


BEFORE_SHIFT = "BEFORE_SHIFT"
SHIFTING = "SHIFTING"
AFTER_SHIFT = "AFTER_SHIFT"


def _param(prefix, name, default):
    return rospy.get_param("{}/{}".format(prefix.rstrip("/"), name.lstrip("/")), default)


def _nearest_distance(polyline, target):
    if not polyline:
        return float("inf")
    best = float("inf")
    for first, second in zip(polyline[:-1], polyline[1:]):
        vx = second.x - first.x
        vy = second.y - first.y
        norm = vx * vx + vy * vy
        ratio = 0.0 if norm < 1.0e-12 else (
            (target.x - first.x) * vx + (target.y - first.y) * vy
        ) / norm
        ratio = clamp(ratio, 0.0, 1.0)
        x = first.x + ratio * vx
        y = first.y + ratio * vy
        best = min(best, math.hypot(target.x - x, target.y - y))
    return best


def _shift_distance(offset, jerk, velocity, minimum):
    if abs(offset) < 1.0e-6:
        return minimum
    phase_time = (abs(offset) / (2.0 * max(abs(jerk), 1.0e-3))) ** (1.0 / 3.0)
    return max(float(minimum), 4.0 * phase_time * max(abs(velocity), 0.1))


def _apply_shift(path, current_offset, target_offset, start_station, finish_station):
    output = deepcopy(path)
    for item, station in zip(output.points, path_cumulative_lengths(output)):
        if station <= start_station:
            offset = current_offset
        elif station >= finish_station:
            offset = target_offset
        else:
            offset = current_offset + (target_offset - current_offset) * smoothstep(
                start_station, finish_station, station
            )
        item.point.pose = shifted_pose(item.point.pose, offset)
    return output


@dataclass
class SideShiftParameters:
    minimum_start_distance: float = 5.0
    time_to_start: float = 1.0
    lateral_jerk: float = 0.2
    minimum_shift_distance: float = 5.0
    minimum_shift_speed: float = 5.56
    request_time_limit: float = 1.0
    drivable_area_check_mode: int = 1
    minimum_drivable_margin: float = 0.3
    vehicle_width: float = 1.896

    @classmethod
    def from_ros(cls, prefix="~side_shift"):
        p = cls()
        p.minimum_start_distance = float(
            _param(prefix, "min_distance_to_start_shifting", p.minimum_start_distance)
        )
        p.time_to_start = float(_param(prefix, "time_to_start_shifting", p.time_to_start))
        p.lateral_jerk = float(_param(prefix, "shifting_lateral_jerk", p.lateral_jerk))
        p.minimum_shift_distance = float(
            _param(prefix, "min_shifting_distance", p.minimum_shift_distance)
        )
        p.minimum_shift_speed = float(
            _param(prefix, "min_shifting_speed", p.minimum_shift_speed)
        )
        p.request_time_limit = float(
            _param(prefix, "shift_request_time_limit", p.request_time_limit)
        )
        p.drivable_area_check_mode = int(
            _param(prefix, "drivable_area_check_mode", p.drivable_area_check_mode)
        )
        p.minimum_drivable_margin = float(
            _param(prefix, "min_drivable_area_margin", p.minimum_drivable_margin)
        )
        p.vehicle_width = float(_param(prefix, "vehicle_width", p.vehicle_width))
        return p


@dataclass
class SideShiftResult:
    reference_path: object
    candidate_path: object
    output_path: object
    requested_offset: float = 0.0
    applied_offset: float = 0.0
    start_distance: float = 0.0
    finish_distance: float = 0.0
    state: str = AFTER_SHIFT
    clamped: bool = False
    status: str = "side_shift: pass-through"


class SideShiftEngine:
    def __init__(self, parameters=None):
        self.parameters = parameters or SideShiftParameters()
        self.requested_offset = 0.0
        self.pending_offset = None
        self.last_request_time = None
        self.current_offset = 0.0
        self.target_offset = 0.0
        self.shift_start_station = 0.0
        self.shift_finish_station = 0.0
        self.state = AFTER_SHIFT
        self.request_changed = False

    def update_request(self, offset, stamp):
        now = stamp.to_sec() if hasattr(stamp, "to_sec") else float(stamp)
        if self.last_request_time is not None:
            if now - self.last_request_time < self.parameters.request_time_limit:
                return False
        self.last_request_time = now
        offset = float(offset)
        if self.state == SHIFTING:
            self.pending_offset = offset
            return False
        if abs(offset - self.requested_offset) < 1.0e-4:
            return False
        self.requested_offset = offset
        self.request_changed = True
        return True

    def _safe_offset(self, path, requested):
        p = self.parameters
        if p.drivable_area_check_mode == 0 or not path.points:
            return requested
        half_vehicle = 0.5 * p.vehicle_width
        left_width = min(
            (_nearest_distance(path.left_bound, item.point.pose.position) for item in path.points),
            default=float("inf"),
        )
        right_width = min(
            (_nearest_distance(path.right_bound, item.point.pose.position) for item in path.points),
            default=float("inf"),
        )
        left_limit = max(0.0, left_width - half_vehicle - p.minimum_drivable_margin)
        right_limit = max(0.0, right_width - half_vehicle - p.minimum_drivable_margin)
        return clamp(requested, -right_limit, left_limit)

    def _update_state(self, ego_station):
        if self.state == BEFORE_SHIFT and ego_station > self.shift_start_station:
            self.state = SHIFTING
        if self.state == SHIFTING and ego_station >= self.shift_finish_station:
            self.state = AFTER_SHIFT
            self.current_offset = self.target_offset
            if self.pending_offset is not None:
                pending = self.pending_offset
                self.pending_offset = None
                if abs(pending - self.requested_offset) >= 1.0e-4:
                    self.requested_offset = pending
                    self.request_changed = True

    def plan(self, path, odometry):
        reference = deepcopy(path)
        if not path.points:
            return SideShiftResult(reference, reference, reference)
        projection = (
            project_pose_to_path(path, odometry.pose.pose) if odometry is not None else None
        )
        ego_station = projection.longitudinal_distance if projection is not None else 0.0
        speed = 0.0
        if odometry is not None:
            linear = odometry.twist.twist.linear
            speed = math.hypot(linear.x, linear.y)
        self._update_state(ego_station)

        safe_offset = self._safe_offset(path, self.requested_offset)
        clamped = abs(safe_offset - self.requested_offset) > 1.0e-4
        if self.state != SHIFTING and abs(safe_offset - self.target_offset) > 1.0e-4:
            self.request_changed = True
        if self.request_changed and self.state in (BEFORE_SHIFT, AFTER_SHIFT):
            distance_to_start = max(
                self.parameters.minimum_start_distance,
                speed * self.parameters.time_to_start,
            )
            shift_distance = _shift_distance(
                safe_offset - self.current_offset,
                self.parameters.lateral_jerk,
                max(speed, self.parameters.minimum_shift_speed),
                self.parameters.minimum_shift_distance,
            )
            self.shift_start_station = ego_station + distance_to_start
            self.shift_finish_station = self.shift_start_station + shift_distance
            self.target_offset = safe_offset
            self.request_changed = False
            self.state = BEFORE_SHIFT

        if abs(self.target_offset) < 1.0e-4 and abs(self.current_offset) < 1.0e-4:
            return SideShiftResult(
                reference,
                reference,
                reference,
                state=AFTER_SHIFT,
                status="side_shift: pass-through, no offset request",
            )
        candidate = _apply_shift(
            path,
            self.current_offset,
            self.target_offset,
            self.shift_start_station,
            self.shift_finish_station,
        )
        status = "side_shift: {} requested={:.2f} m applied={:.2f} m".format(
            self.state, self.requested_offset, self.target_offset
        )
        if clamped:
            status += ", clamped by drivable bound"
        return SideShiftResult(
            reference,
            candidate,
            candidate,
            requested_offset=self.requested_offset,
            applied_offset=self.target_offset,
            start_distance=max(0.0, self.shift_start_station - ego_station),
            finish_distance=max(0.0, self.shift_finish_station - ego_station),
            state=self.state,
            clamped=clamped,
            status=status,
        )

    @staticmethod
    def turn_command(result):
        # Autoware's externally requested Side Shift module does not own turn signals.
        return TurnIndicatorsCommand.DISABLE


class SideShiftScene:
    def __init__(self, prefix="~side_shift"):
        self.engine = SideShiftEngine(SideShiftParameters.from_ros(prefix))

    def update_request(self, offset, stamp):
        return self.engine.update_request(offset, stamp)

    def process(self, path, odometry):
        return self.engine.plan(path, odometry)

    def turn_command(self, result):
        return self.engine.turn_command(result)
