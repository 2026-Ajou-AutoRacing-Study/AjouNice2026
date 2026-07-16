#!/usr/bin/env python3

from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import minimize

from .trajectory_utils import crop, resample


@dataclass
class ElasticBandParameters:
    num_points: int = 100
    delta_arc_length: float = 1.0
    num_joint_points: int = 3
    clearance_for_fix: float = 0.0
    clearance_for_joint: float = 0.1
    clearance_for_smooth: float = 0.1
    smooth_weight: float = 1.0
    lat_error_weight: float = 0.001
    max_iteration: int = 10000
    eps_abs: float = 1.0e-7
    eps_rel: float = 1.0e-7
    enable_warm_start: bool = True
    enable_optimization_validation: bool = False
    max_validation_error: float = 3.0
    output_backward_traj_length: float = 5.0


class ElasticBand:
    def __init__(self, parameters=None):
        self.parameters = parameters or ElasticBandParameters()
        self.previous_offsets = None
        self.previous_points = None
        self.fixed_points = []

    @staticmethod
    def _smoothness_matrix(count):
        difference = np.zeros((max(count - 2, 0), count), dtype=float)
        for index in range(count - 2):
            difference[index, index : index + 3] = (1.0, -2.0, 1.0)
        return difference.T.dot(difference)

    def _bounds(self, count, valid_count, goal_contained):
        parameter = self.parameters
        bounds = []
        for index in range(count):
            clearance = parameter.clearance_for_smooth
            if index == 0:
                clearance = parameter.clearance_for_fix
            elif goal_contained and (index >= count - 2 or index >= valid_count - 2):
                clearance = parameter.clearance_for_fix
            elif index < parameter.num_joint_points + 1:
                clearance = parameter.clearance_for_joint
            bounds.append((-float(clearance), float(clearance)))
        return bounds

    def _solve_offsets(self, points, valid_count, goal_contained):
        parameter = self.parameters
        count = len(points)
        smoothness = self._smoothness_matrix(count)
        raw = np.zeros((count * 2, count * 2), dtype=float)
        raw[:count, :count] = smoothness
        raw[count:, count:] = smoothness
        raw *= float(parameter.smooth_weight)

        theta = np.zeros((count, count * 2), dtype=float)
        values = np.zeros(count * 2, dtype=float)
        for index, point in enumerate(points):
            theta[index, index] = -np.sin(point.yaw)
            theta[index, index + count] = np.cos(point.yaw)
            values[index] = point.x
            values[index + count] = point.y

        theta_raw = theta.dot(raw)
        quadratic = theta_raw.dot(theta.T)
        quadratic += float(parameter.lat_error_weight) * np.eye(count)
        linear = theta_raw.dot(values)
        bounds = self._bounds(count, valid_count, goal_contained)
        initial = np.zeros(count, dtype=float)
        if (
            parameter.enable_warm_start
            and self.previous_offsets is not None
            and len(self.previous_offsets) == count
        ):
            initial = np.clip(self.previous_offsets, [item[0] for item in bounds], [item[1] for item in bounds])

        result = minimize(
            lambda value: 0.5 * value.dot(quadratic).dot(value) + linear.dot(value),
            initial,
            jac=lambda value: quadratic.dot(value) + linear,
            bounds=bounds,
            method="L-BFGS-B",
            options={
                "maxiter": int(parameter.max_iteration),
                "ftol": float(parameter.eps_abs),
                "gtol": float(parameter.eps_rel),
                "maxls": 50,
            },
        )
        if not result.success or not np.all(np.isfinite(result.x)):
            return None
        self.previous_offsets = np.asarray(result.x, dtype=float)
        return self.previous_offsets

    def smooth(self, points, ego_x, ego_y):
        parameter = self.parameters
        if len(points) < 2:
            return list(points)
        forward_length = int(parameter.num_points) * float(parameter.delta_arc_length)
        cropped = crop(
            points,
            ego_x,
            ego_y,
            forward_length,
            parameter.output_backward_traj_length,
        )
        goal_contained = (
            abs(cropped[-1].x - points[-1].x) < 1.0e-3
            and abs(cropped[-1].y - points[-1].y) < 1.0e-3
        )
        resampled = resample(cropped, parameter.delta_arc_length, preserve_stop=False)
        valid_count = min(int(parameter.num_points), len(resampled))
        padded = list(resampled[:valid_count])
        while len(padded) < int(parameter.num_points):
            padded.append(padded[-1])

        offsets = self._solve_offsets(padded, valid_count, goal_contained)
        if offsets is None:
            return list(self.previous_points) if self.previous_points is not None else list(points)

        output = []
        bounds = self._bounds(len(padded), valid_count, goal_contained)
        self.fixed_points = [
            point
            for point, bound in zip(padded, bounds)
            if abs(bound[0]) < 1.0e-12 and abs(bound[1]) < 1.0e-12
        ]
        for point, offset in zip(padded[:valid_count], offsets[:valid_count]):
            if parameter.enable_optimization_validation and abs(offset) > parameter.max_validation_error:
                return list(self.previous_points) if self.previous_points is not None else list(points)
            output.append(
                replace(
                    point,
                    x=point.x - np.sin(point.yaw) * float(offset),
                    y=point.y + np.cos(point.yaw) * float(offset),
                )
            )
        for index, point in enumerate(output):
            if len(output) < 2:
                break
            first = output[index - 1] if index > 0 else output[index]
            second = output[index + 1] if index + 1 < len(output) else output[index]
            if index == 0:
                first = output[0]
                second = output[1]
            elif index == len(output) - 1:
                first = output[-2]
                second = output[-1]
            output[index] = replace(
                point,
                yaw=float(np.arctan2(second.y - first.y, second.x - first.x)),
            )
        self.previous_points = list(output)
        return output

    def reset(self):
        self.previous_offsets = None
        self.previous_points = None
        self.fixed_points = []
