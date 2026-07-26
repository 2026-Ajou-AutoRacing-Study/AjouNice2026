import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def plausible_ego_travel_step_m(
    previous_x,
    previous_y,
    current_x,
    current_y,
    dt_s,
    previous_velocity_mps,
    current_velocity_mps,
    trajectory_length_m,
):
    """Return an XY travel step, or None when it looks like a position jump."""
    values = (
        previous_x,
        previous_y,
        current_x,
        current_y,
        dt_s,
        previous_velocity_mps,
        current_velocity_mps,
        trajectory_length_m,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or dt_s <= 0.0
        or trajectory_length_m <= 0.0
    ):
        return None
    displacement_m = math.hypot(
        current_x - previous_x,
        current_y - previous_y,
    )
    expected_distance_m = (
        0.5
        * (abs(previous_velocity_mps) + abs(current_velocity_mps))
        * dt_s
    )
    # Keep this trajectory-independent. The margin absorbs low-speed pose noise,
    # while the velocity-scaled term accommodates sparse samples at high speed.
    maximum_plausible_distance_m = max(
        5.0,
        0.05 * trajectory_length_m,
        3.0 * expected_distance_m + 1.0,
    )
    return (
        displacement_m
        if displacement_m <= maximum_plausible_distance_m
        else None
    )


def goal_conditions_met(
    final_distance_m,
    goal_tolerance_m,
    matched_station_m,
    final_station_m,
    goal_progress_margin_m,
    ego_travel_distance_m,
    min_travel_ratio_for_goal,
):
    """Combine endpoint, trajectory-progress, and physical-travel goal guards."""
    final_progress_start_m = max(
        0.0, final_station_m - goal_progress_margin_m
    )
    minimum_travel_distance_m = (
        final_station_m * min_travel_ratio_for_goal
    )
    return (
        final_distance_m <= goal_tolerance_m
        and matched_station_m >= final_progress_start_m
        and ego_travel_distance_m >= minimum_travel_distance_m
    )


class TrajectoryMatcher:
    def __init__(self, trajectory):
        if len(trajectory.points) < 2:
            raise ValueError("trajectory matcher requires at least two points")
        self.trajectory = trajectory
        self.stations = [0.0]
        for first, second in zip(trajectory.points, trajectory.points[1:]):
            dx = second.pose.position.x - first.pose.position.x
            dy = second.pose.position.y - first.pose.position.y
            self.stations.append(self.stations[-1] + math.hypot(dx, dy))

    def match(self, x, y, ego_yaw, ego_velocity):
        best = None
        for index in range(len(self.trajectory.points) - 1):
            first = self.trajectory.points[index]
            second = self.trajectory.points[index + 1]
            ax = first.pose.position.x
            ay = first.pose.position.y
            dx = second.pose.position.x - ax
            dy = second.pose.position.y - ay
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                ratio = 0.0
            else:
                ratio = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_squared))
            projected_x = ax + ratio * dx
            projected_y = ay + ratio * dy
            distance_squared = (x - projected_x) ** 2 + (y - projected_y) ** 2
            if best is None or distance_squared < best[0]:
                best = (distance_squared, index, ratio, projected_x, projected_y)

        _, segment, ratio, projected_x, projected_y = best
        matched_index = segment + (1 if ratio >= 0.5 else 0)
        matched_point = self.trajectory.points[matched_index]
        gt_yaw = yaw_from_quaternion(matched_point.pose.orientation)
        target_velocity = float(matched_point.longitudinal_velocity_mps)
        error_x = x - projected_x
        error_y = y - projected_y
        station = self.stations[segment] + ratio * (
            self.stations[segment + 1] - self.stations[segment]
        )
        return {
            "matched_gt_index": matched_index,
            "matched_gt_station_m": station,
            "matched_gt_x_m": projected_x,
            "matched_gt_y_m": projected_y,
            "matched_gt_yaw_rad": gt_yaw,
            "target_velocity_mps": target_velocity,
            "lateral_error_m": -math.sin(gt_yaw) * error_x + math.cos(gt_yaw) * error_y,
            "longitudinal_error_m": math.cos(gt_yaw) * error_x + math.sin(gt_yaw) * error_y,
            "heading_error_rad": normalize_angle(ego_yaw - gt_yaw),
            "velocity_error_mps": ego_velocity - target_velocity,
        }


def metric_summary(samples):
    valid = [sample for sample in samples if sample.get("control_pair_valid")]

    def values(name):
        return [abs(float(sample[name])) for sample in valid]

    def mean(data):
        return sum(data) / len(data) if data else None

    def rms(data):
        return math.sqrt(sum(value * value for value in data) / len(data)) if data else None

    lateral = values("lateral_error_m")
    heading = values("heading_error_rad")
    velocity = values("velocity_error_mps")
    return {
        "reached_goal": any(sample.get("reached_goal") for sample in samples),
        "collision_detected": any(sample.get("collision_detected") for sample in samples),
        "mean_abs_lateral_error_m": mean(lateral),
        "rms_lateral_error_m": rms(lateral),
        "max_abs_lateral_error_m": max(lateral) if lateral else None,
        "mean_abs_heading_error_rad": mean(heading),
        "max_abs_heading_error_rad": max(heading) if heading else None,
        "mean_abs_velocity_error_mps": mean(velocity),
        "max_abs_velocity_error_mps": max(velocity) if velocity else None,
    }
