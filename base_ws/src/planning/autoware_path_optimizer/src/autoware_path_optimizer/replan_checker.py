#!/usr/bin/env python3

import math

from .trajectory_utils import (
    cumulative_lengths,
    distance_2d,
    interpolate_samples,
    project_point,
)


class ReplanChecker:
    def __init__(
        self,
        max_path_shape_around_ego_lat_dist=2.0,
        max_path_shape_forward_lon_dist=100.0,
        max_path_shape_forward_lat_dist=0.1,
        max_ego_moving_dist=5.0,
        max_goal_moving_dist=15.0,
        max_delta_time_sec=1.0,
    ):
        self.around_threshold = float(max_path_shape_around_ego_lat_dist)
        self.forward_length = float(max_path_shape_forward_lon_dist)
        self.forward_threshold = float(max_path_shape_forward_lat_dist)
        self.ego_threshold = float(max_ego_moving_dist)
        self.goal_threshold = float(max_goal_moving_dist)
        self.time_threshold = float(max_delta_time_sec)
        self.previous_input = None
        self.previous_ego = None
        self.previous_time = None

    def reset_required(self, points, ego):
        if self.previous_input is None or self.previous_ego is None:
            return True
        if math.hypot(ego[0] - self.previous_ego[0], ego[1] - self.previous_ego[1]) > self.ego_threshold:
            return True
        current_distance = project_point(points, ego[0], ego[1])[2]
        previous_distance = project_point(self.previous_input, ego[0], ego[1])[2]
        if abs(current_distance - previous_distance) > self.around_threshold:
            return True
        return distance_2d(points[-1], self.previous_input[-1]) > self.goal_threshold

    def replan_required(self, points, now, ego=None):
        if self.previous_input is None or self.previous_time is None:
            return True
        if self.time_threshold > 0.0 and now - self.previous_time >= self.time_threshold:
            return True
        if len(points) < 2 or len(self.previous_input) < 2:
            return True
        ego_x, ego_y = ego if ego is not None else (points[0].x, points[0].y)
        _, current_ego_station, _, _ = project_point(points, ego_x, ego_y)
        _, previous_ego_station, _, _ = project_point(
            self.previous_input, ego_x, ego_y
        )
        current_stations = cumulative_lengths(points)
        previous_stations = cumulative_lengths(self.previous_input)
        common_forward = min(
            self.forward_length,
            float(current_stations[-1]) - current_ego_station,
            float(previous_stations[-1]) - previous_ego_station,
        )
        if common_forward <= 0.0:
            return True
        sample_count = 20
        offsets = [
            common_forward * float(index) / float(sample_count - 1)
            for index in range(sample_count)
        ]
        current_samples = interpolate_samples(
            points, [current_ego_station + offset for offset in offsets]
        )
        previous_samples = interpolate_samples(
            self.previous_input,
            [previous_ego_station + offset for offset in offsets],
        )
        return any(
            distance_2d(current, previous) > self.forward_threshold
            for current, previous in zip(current_samples, previous_samples)
        )

    def update(self, points, ego, replanned, now):
        self.previous_input = list(points)
        self.previous_ego = tuple(ego)
        if replanned:
            self.previous_time = float(now)

    def reset(self):
        self.previous_input = None
        self.previous_ego = None
        self.previous_time = None
