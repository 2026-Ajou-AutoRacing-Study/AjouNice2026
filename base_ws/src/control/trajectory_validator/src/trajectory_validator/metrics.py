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


def percentile(data, percentage):
    values = sorted(float(value) for value in data)
    if not values:
        return None
    position = (len(values) - 1) * float(percentage) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    ratio = position - lower
    return values[lower] * (1.0 - ratio) + values[upper] * ratio


def value_summary(samples, name):
    values = [float(sample[name]) for sample in samples]
    absolute = [abs(value) for value in values]
    if not values:
        return {
            "mean": None,
            "mean_abs": None,
            "rms": None,
            "p95_abs": None,
            "max_abs": None,
        }
    return {
        "mean": sum(values) / len(values),
        "mean_abs": sum(absolute) / len(absolute),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95_abs": percentile(absolute, 95.0),
        "max_abs": max(absolute),
    }


def _distance(first, second):
    return math.hypot(second.x - first.x, second.y - first.y)


def trajectory_curvatures(trajectory, sample_distance_m=1.0):
    points = [point.pose.position for point in trajectory.points]
    if len(points) < 3:
        return [0.0] * len(points)
    previous = [0] * len(points)
    following = [len(points) - 1] * len(points)
    left = 0
    for index in range(len(points)):
        while left + 1 < index and _distance(points[left + 1], points[index]) >= sample_distance_m:
            left += 1
        previous[index] = left
    right = 0
    for index in range(len(points)):
        right = min(len(points) - 1, max(right, index + 1))
        while right < len(points) - 1 and _distance(points[index], points[right]) < sample_distance_m:
            right += 1
        following[index] = right

    output = []
    for index, middle in enumerate(points):
        first = points[previous[index]]
        last = points[following[index]]
        first_length = _distance(first, middle)
        second_length = _distance(middle, last)
        chord = _distance(first, last)
        if first_length <= 1e-6 or second_length <= 1e-6 or chord <= 1e-6:
            output.append(0.0)
            continue
        cross = (
            (middle.x - first.x) * (last.y - first.y)
            - (middle.y - first.y) * (last.x - first.x)
        )
        output.append(abs(2.0 * cross / (first_length * second_length * chord)))
    return output


CURVATURE_BINS = (
    ("straight", 0.0, 0.01),
    ("gentle", 0.01, 0.02),
    ("medium", 0.02, 0.04),
    ("sharp", 0.04, 0.08),
    ("very_sharp", 0.08, None),
)


def evaluation_summary(
    samples,
    trajectory,
    lateral_tolerance_m=0.3,
    speed_overshoot_tolerance_mps=0.1,
):
    valid = [sample for sample in samples if sample.get("control_pair_valid")]
    valid_count = len(valid)
    lateral_absolute = [abs(float(sample["lateral_error_m"])) for sample in valid]
    within_count = sum(value <= lateral_tolerance_m for value in lateral_absolute)

    speed_overshoots = [
        max(0.0, float(sample["ego_velocity_mps"]) - float(sample["target_velocity_mps"]))
        for sample in valid
    ]
    overshoot_percentages = [
        100.0
        * max(0.0, float(sample["ego_velocity_mps"]) - float(sample["target_velocity_mps"]))
        / float(sample["target_velocity_mps"])
        for sample in valid
        if float(sample["target_velocity_mps"]) > 0.1
    ]

    curvatures = trajectory_curvatures(trajectory)
    curvature_sections = {}
    for label, minimum, maximum in CURVATURE_BINS:
        section = []
        for sample in valid:
            index = min(max(0, int(sample["matched_gt_index"])), len(curvatures) - 1)
            curvature = curvatures[index] if curvatures else 0.0
            if curvature >= minimum and (maximum is None or curvature < maximum):
                section.append(sample)
        curvature_sections[label] = {
            "curvature_min_1pm": minimum,
            "curvature_max_1pm": maximum,
            "sample_count": len(section),
            "sample_ratio_percent": 100.0 * len(section) / valid_count if valid_count else None,
            "lateral_error_m": value_summary(section, "lateral_error_m"),
            "longitudinal_error_m": value_summary(section, "longitudinal_error_m"),
            "velocity_error_mps": value_summary(section, "velocity_error_mps"),
            "steering_error_rad": value_summary(section, "steering_error_rad"),
        }

    command_accel = [float(sample["cmd_accel"]) for sample in valid]
    command_brake = [float(sample["cmd_brake"]) for sample in valid]
    steering_rates = []
    for previous, current in zip(valid, valid[1:]):
        dt = float(current["dt_s"])
        if 0.0 < dt <= 1.0:
            steering_rates.append(
                (float(current["cmd_steering_rad"]) - float(previous["cmd_steering_rad"]))
                / dt
            )
    return {
        "schema_version": 1,
        "definitions": {
            "lateral_retention": "percentage of valid samples with abs(lateral_error_m) <= tolerance",
            "p95": "linearly interpolated 95th percentile of absolute error",
            "speed_overshoot": "max(ego_velocity_mps - target_velocity_mps, 0)",
            "speed_overshoot_occurrence": (
                "percentage above target velocity by more than 0.0 m/s"
            ),
            "speed_overshoot_including_errors": (
                "percentage above target velocity by more than "
                "{} m/s".format(speed_overshoot_tolerance_mps)
            ),
            "curvature": "absolute three-point curvature using approximately 1 m path spacing",
        },
        "sample_quality": {
            "total_sample_count": len(samples),
            "valid_sample_count": valid_count,
            "valid_sample_ratio_percent": 100.0 * valid_count / len(samples) if samples else None,
        },
        "lateral": {
            "error_m": value_summary(valid, "lateral_error_m"),
            "retention_tolerance_m": lateral_tolerance_m,
            "retention_sample_count": within_count,
            "retention_percent": 100.0 * within_count / valid_count if valid_count else None,
            "heading_error_rad": value_summary(valid, "heading_error_rad"),
        },
        "longitudinal": {
            "position_error_m": value_summary(valid, "longitudinal_error_m"),
            "velocity_error_mps": value_summary(valid, "velocity_error_mps"),
            "ego_acceleration_mps2": value_summary(valid, "ego_acceleration_mps2"),
            "speed_overshoot": {
                "max_mps": max(speed_overshoots) if speed_overshoots else None,
                "max_kph": max(speed_overshoots) * 3.6 if speed_overshoots else None,
                "max_percent": max(overshoot_percentages) if overshoot_percentages else None,
                "p95_mps": percentile(speed_overshoots, 95.0),
                "p95_percent": percentile(overshoot_percentages, 95.0),
                "overshoot_sample_ratio_percent": (
                    100.0
                    * sum(value > 0.0 for value in speed_overshoots)
                    / len(speed_overshoots)
                    if speed_overshoots
                    else None
                ),
                "including_errors_sample_ratio_percent": (
                    100.0
                    * sum(
                        value > speed_overshoot_tolerance_mps
                        for value in speed_overshoots
                    )
                    / len(speed_overshoots)
                    if speed_overshoots
                    else None
                ),
            },
            "actuator_command": {
                "accel_mean": sum(command_accel) / len(command_accel) if command_accel else None,
                "accel_p95": percentile(command_accel, 95.0),
                "accel_max": max(command_accel) if command_accel else None,
                "brake_mean": sum(command_brake) / len(command_brake) if command_brake else None,
                "brake_p95": percentile(command_brake, 95.0),
                "brake_max": max(command_brake) if command_brake else None,
            },
        },
        "steering": {
            "command_rad": value_summary(valid, "cmd_steering_rad"),
            "actual_rad": value_summary(valid, "actual_steering_rad"),
            "error_rad": value_summary(valid, "steering_error_rad"),
            "command_rate_rad_s": {
                "p95_abs": percentile([abs(value) for value in steering_rates], 95.0),
                "max_abs": max([abs(value) for value in steering_rates], default=None),
            },
        },
        "curvature_sections": curvature_sections,
    }


def compact_evaluation_summary(samples, trajectory, lateral_tolerance_m=0.3):
    """Return only the headline metrics written to summary.json."""
    detailed = evaluation_summary(samples, trajectory, lateral_tolerance_m)

    def formatted_percent(value):
        return f"{float(value):.3f}%" if value is not None else None

    overshoot = detailed["longitudinal"]["speed_overshoot"]
    return {
        "valid_data_percent": formatted_percent(
            detailed["sample_quality"]["valid_sample_ratio_percent"]
        ),
        "lateral_retention_percent": formatted_percent(
            detailed["lateral"]["retention_percent"]
        ),
        "speed_overshoot_occurrence_percent": formatted_percent(
            overshoot["overshoot_sample_ratio_percent"]
        ),
        "speed_overshoot_including_errors_percent": formatted_percent(
            overshoot["including_errors_sample_ratio_percent"]
        ),
        "max_speed_overshoot_percent": formatted_percent(overshoot["max_percent"]),
        "p95_speed_overshoot_percent": formatted_percent(overshoot["p95_percent"]),
        "collision_detected": any(
            sample.get("collision_detected") for sample in samples
        ),
        "reached_goal": any(sample.get("reached_goal") for sample in samples),
    }
