#!/usr/bin/env python3

import math

from .trajectory_utils import distance_2d, project_point


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

    def replan_required(self, points, now):
        if self.previous_input is None or self.previous_time is None:
            return True
        if self.time_threshold > 0.0 and now - self.previous_time >= self.time_threshold:
            return True
        count = min(len(points), len(self.previous_input))
        if count < 2:
            return True
        stride = max(count // 20, 1)
        return any(
            distance_2d(points[index], self.previous_input[index]) > self.forward_threshold
            for index in range(0, count, stride)
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
