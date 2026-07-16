#!/usr/bin/env python3

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import minimize

from .trajectory_utils import (
    TrajectorySample,
    crop,
    curvature,
    nearest_point_on_polyline,
    resample,
)


@dataclass
class MPTParameters:
    num_points: int = 100
    delta_arc_length: float = 1.0
    output_backward_traj_length: float = 5.0
    vehicle_width: float = 1.896
    wheel_base: float = 2.79
    max_steer_angle: float = 0.61
    soft_clearance_from_road: float = 0.1
    min_drivable_width: float = 0.2
    num_fixed_points: int = 3
    lat_error_weight: float = 1.0
    yaw_error_weight: float = 2.0
    steer_input_weight: float = 8.0
    steer_rate_weight: float = 30.0
    terminal_lat_error_weight: float = 100.0
    goal_lat_error_weight: float = 1000.0
    max_iteration: int = 2000
    tolerance: float = 1.0e-8
    enable_warm_start: bool = True
    enable_optimization_validation: bool = True
    max_validation_lat_error: float = 5.0


@dataclass
class OptimizationResult:
    points: list
    reference_points: list
    fixed_points: list
    offsets: list
    lower_bounds: list
    upper_bounds: list
    success: bool
    status: str


def difference_matrix(count, order):
    if count <= order:
        return np.zeros((0, count), dtype=float)
    matrix = np.eye(count, dtype=float)
    for _ in range(order):
        matrix = np.diff(matrix, axis=0)
    return matrix


class ModelPredictiveTrajectory:
    def __init__(self, parameters=None):
        self.parameters = parameters or MPTParameters()
        self.previous_offsets = None

    @staticmethod
    def _boundary_xy(boundary):
        return [(float(point[0]), float(point[1])) for point in boundary]

    def lateral_bounds(self, points, left_bound, right_bound):
        parameter = self.parameters
        margin = 0.5 * parameter.vehicle_width + parameter.soft_clearance_from_road
        left_xy = self._boundary_xy(left_bound)
        right_xy = self._boundary_xy(right_bound)
        lower = []
        upper = []
        for point in points:
            normal = (-math.sin(point.yaw), math.cos(point.yaw))
            left = nearest_point_on_polyline(left_xy, point.x, point.y)
            right = nearest_point_on_polyline(right_xy, point.x, point.y)
            left_distance = (left[0] - point.x) * normal[0] + (left[1] - point.y) * normal[1]
            right_distance = (right[0] - point.x) * normal[0] + (right[1] - point.y) * normal[1]
            if left_distance <= right_distance:
                left_distance = math.hypot(left[0] - point.x, left[1] - point.y)
                right_distance = -math.hypot(right[0] - point.x, right[1] - point.y)
            lower.append(float(right_distance + margin))
            upper.append(float(left_distance - margin))
        return np.asarray(lower), np.asarray(upper)

    def _objective(self, offsets, reference, normals, matrices, goal_contained):
        parameter = self.parameters
        coordinates = reference + normals * offsets[:, None]
        value = parameter.lat_error_weight * float(offsets.dot(offsets))
        gradient = 2.0 * parameter.lat_error_weight * offsets
        terms = (
            (matrices[0], parameter.yaw_error_weight),
            (matrices[1], parameter.steer_input_weight),
            (matrices[2], parameter.steer_rate_weight),
        )
        for matrix, weight in terms:
            if not len(matrix) or weight <= 0.0:
                continue
            residual = matrix.dot(coordinates)
            back_projected = matrix.T.dot(residual)
            value += float(weight) * float(np.sum(residual * residual))
            gradient += 2.0 * float(weight) * np.sum(back_projected * normals, axis=1)
        terminal_weight = (
            parameter.goal_lat_error_weight if goal_contained else parameter.terminal_lat_error_weight
        )
        value += terminal_weight * offsets[-1] * offsets[-1]
        gradient[-1] += 2.0 * terminal_weight * offsets[-1]
        return value, gradient

    def optimize(self, source, ego_x, ego_y, left_bound, right_bound):
        parameter = self.parameters
        if len(source) < 2 or len(left_bound) < 2 or len(right_bound) < 2:
            return OptimizationResult([], [], [], [], [], [], False, "invalid input")
        cropped = crop(
            source,
            ego_x,
            ego_y,
            parameter.num_points * parameter.delta_arc_length,
            parameter.output_backward_traj_length,
        )
        goal_contained = (
            math.hypot(cropped[-1].x - source[-1].x, cropped[-1].y - source[-1].y) < 1.0e-3
        )
        reference = resample(cropped, parameter.delta_arc_length, preserve_stop=False)
        reference = reference[: parameter.num_points]
        if len(reference) < 3:
            return OptimizationResult([], reference, [], [], [], [], False, "short reference")
        lower, upper = self.lateral_bounds(reference, left_bound, right_bound)
        infeasible = lower > upper
        if np.any(infeasible):
            return OptimizationResult(
                [],
                reference,
                [],
                [],
                lower.tolist(),
                upper.tolist(),
                False,
                "drivable area narrower than vehicle footprint",
            )
        fixed_count = min(max(int(parameter.num_fixed_points), 1), len(reference))
        lower[:fixed_count] = 0.0
        upper[:fixed_count] = 0.0
        if goal_contained:
            lower[-2:] = 0.0
            upper[-2:] = 0.0
        bounds = list(zip(lower.tolist(), upper.tolist()))
        initial = np.zeros(len(reference), dtype=float)
        if (
            parameter.enable_warm_start
            and self.previous_offsets is not None
            and len(self.previous_offsets) == len(reference)
        ):
            initial = np.clip(self.previous_offsets, lower, upper)
        reference_xy = np.asarray([[point.x, point.y] for point in reference], dtype=float)
        normals = np.asarray(
            [[-math.sin(point.yaw), math.cos(point.yaw)] for point in reference], dtype=float
        )
        matrices = tuple(difference_matrix(len(reference), order) for order in (1, 2, 3))

        def objective(offsets):
            return self._objective(offsets, reference_xy, normals, matrices, goal_contained)

        result = minimize(
            lambda offsets: objective(offsets)[0],
            initial,
            jac=lambda offsets: objective(offsets)[1],
            method="L-BFGS-B",
            bounds=bounds,
            options={
                "maxiter": int(parameter.max_iteration),
                "ftol": float(parameter.tolerance),
                "gtol": float(parameter.tolerance),
                "maxls": 50,
            },
        )
        if not result.success or not np.all(np.isfinite(result.x)):
            return OptimizationResult(
                [], reference, reference[:fixed_count], [], lower.tolist(), upper.tolist(), False,
                "solver failure: {}".format(result.message),
            )
        offsets = np.asarray(result.x, dtype=float)
        if parameter.enable_optimization_validation and np.max(np.abs(offsets)) > parameter.max_validation_lat_error:
            return OptimizationResult(
                [], reference, reference[:fixed_count], offsets.tolist(), lower.tolist(), upper.tolist(), False,
                "lateral validation failure",
            )
        coordinates = reference_xy + normals * offsets[:, None]
        output = [
            replace(point, x=float(position[0]), y=float(position[1]))
            for point, position in zip(reference, coordinates)
        ]
        for index, point in enumerate(output):
            first = output[max(index - 1, 0)]
            last = output[min(index + 1, len(output) - 1)]
            output[index] = replace(
                point,
                yaw=math.atan2(last.y - first.y, last.x - first.x),
            )
        curvatures = curvature(output)
        for index, point in enumerate(output):
            steer = max(
                -parameter.max_steer_angle,
                min(parameter.max_steer_angle, math.atan(parameter.wheel_base * curvatures[index])),
            )
            output[index] = replace(
                point,
                heading_rate_rps=curvatures[index] * point.longitudinal_velocity_mps,
                front_wheel_angle_rad=steer,
                rear_wheel_angle_rad=0.0,
            )
        self.previous_offsets = offsets.copy()
        return OptimizationResult(
            output,
            reference,
            reference[:fixed_count],
            offsets.tolist(),
            lower.tolist(),
            upper.tolist(),
            True,
            "optimized",
        )

    def reset(self):
        self.previous_offsets = None
