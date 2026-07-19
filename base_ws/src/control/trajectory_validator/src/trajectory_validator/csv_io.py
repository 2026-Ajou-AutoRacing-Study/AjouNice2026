import csv

from trajectory_validator.metrics import yaw_from_quaternion


GT_FIELDS = [
    "gt_index", "gt_time_from_start_s", "gt_x_m", "gt_y_m", "gt_z_m",
    "gt_qx", "gt_qy", "gt_qz", "gt_qw", "gt_yaw_rad",
    "gt_longitudinal_velocity_mps", "gt_lateral_velocity_mps",
    "gt_acceleration_mps2", "gt_heading_rate_rps",
    "gt_front_wheel_angle_rad", "gt_rear_wheel_angle_rad",
]

CONTROL_FIELDS = [
    "control_index", "ros_time_ns", "elapsed_time_s", "dt_s",
    "state_received_ros_ns", "command_received_ros_ns",
    "state_to_command_delay_s", "matched_gt_index", "matched_gt_station_m",
    "matched_gt_x_m", "matched_gt_y_m", "matched_gt_yaw_rad",
    "target_velocity_mps", "ego_unique_id", "ego_x_m", "ego_y_m", "ego_z_m",
    "ego_yaw_rad", "ego_heading_raw", "ego_velocity_x_mps",
    "ego_velocity_y_mps", "ego_velocity_z_mps", "ego_velocity_mps",
    "ego_acceleration_x_mps2", "ego_acceleration_y_mps2",
    "ego_acceleration_z_mps2", "ego_acceleration_mps2", "ego_accel",
    "ego_brake", "ego_wheel_angle_raw", "actual_steering_rad",
    "ego_lateral_offset_m", "cmd_longl_type", "cmd_accel", "cmd_brake",
    "cmd_steering_rad", "cmd_velocity_mps", "cmd_acceleration_mps2",
    "lateral_error_m", "longitudinal_error_m", "heading_error_rad",
    "velocity_error_mps", "steering_error_rad", "collision_detected",
    "reached_goal", "control_pair_valid",
]

RESPONSE_FIELDS = [
    "response_index", "control_index", "command_received_ros_ns",
    "response_received_ros_ns", "response_elapsed_time_s", "requested_response_delay_s",
    "actual_response_delay_s", "cmd_accel", "cmd_brake", "cmd_steering_rad",
    "cmd_velocity_mps", "initial_ego_velocity_mps", "response_ego_velocity_mps",
    "velocity_change_mps", "initial_actual_steering_rad",
    "response_actual_steering_rad", "steering_change_rad",
    "initial_lateral_error_m", "response_lateral_error_m",
    "lateral_error_change_m", "initial_heading_error_rad",
    "response_heading_error_rad", "heading_error_change_rad",
    "response_pair_valid",
]


class RunCsvWriter:
    def __init__(self, gt_path, control_path, response_path):
        self.gt_stream = gt_path.open("w", newline="")
        self.control_stream = control_path.open("w", newline="")
        self.response_stream = response_path.open("w", newline="")
        self.gt_writer = csv.DictWriter(self.gt_stream, fieldnames=GT_FIELDS)
        self.control_writer = csv.DictWriter(self.control_stream, fieldnames=CONTROL_FIELDS)
        self.response_writer = csv.DictWriter(self.response_stream, fieldnames=RESPONSE_FIELDS)
        for writer in (self.gt_writer, self.control_writer, self.response_writer):
            writer.writeheader()
        self.gt_count = 0
        self.control_count = 0
        self.response_count = 0

    def write_gt(self, trajectory):
        if self.gt_count:
            raise RuntimeError("GT rows were already written")
        for index, point in enumerate(trajectory.points):
            orientation = point.pose.orientation
            self.gt_writer.writerow({
                "gt_index": index,
                "gt_time_from_start_s": point.time_from_start.to_sec(),
                "gt_x_m": point.pose.position.x,
                "gt_y_m": point.pose.position.y,
                "gt_z_m": point.pose.position.z,
                "gt_qx": orientation.x,
                "gt_qy": orientation.y,
                "gt_qz": orientation.z,
                "gt_qw": orientation.w,
                "gt_yaw_rad": yaw_from_quaternion(orientation),
                "gt_longitudinal_velocity_mps": point.longitudinal_velocity_mps,
                "gt_lateral_velocity_mps": point.lateral_velocity_mps,
                "gt_acceleration_mps2": point.acceleration_mps2,
                "gt_heading_rate_rps": point.heading_rate_rps,
                "gt_front_wheel_angle_rad": point.front_wheel_angle_rad,
                "gt_rear_wheel_angle_rad": point.rear_wheel_angle_rad,
            })
            self.gt_count += 1
        self.gt_stream.flush()

    def write_control(self, row):
        self.control_writer.writerow({field: row[field] for field in CONTROL_FIELDS})
        self.control_count += 1
        self.control_stream.flush()

    def write_response(self, row):
        self.response_writer.writerow({field: row[field] for field in RESPONSE_FIELDS})
        self.response_count += 1
        self.response_stream.flush()

    def close(self):
        for stream in (self.gt_stream, self.control_stream, self.response_stream):
            if not stream.closed:
                stream.flush()
                stream.close()


def _load(path, fields, label):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(fields).difference(reader.fieldnames or [])
        if missing:
            raise ValueError("{} CSV missing fields: {}".format(label, sorted(missing)))
        return list(reader)


def load_run_csv(gt_path, control_path, response_path):
    gt_rows = _load(gt_path, GT_FIELDS, "GT")
    control_rows = _load(control_path, CONTROL_FIELDS, "control cycles")
    response_rows = _load(response_path, RESPONSE_FIELDS, "vehicle responses")
    gt_rows.sort(key=lambda row: int(row["gt_index"]))
    control_rows.sort(key=lambda row: int(row["control_index"]))
    response_rows.sort(key=lambda row: int(row["response_index"]))
    return gt_rows, control_rows, response_rows


def as_float(row, field, default=0.0):
    value = row.get(field, "")
    return float(value) if value != "" else default


def as_int(row, field, default=0):
    value = row.get(field, "")
    return int(value) if value != "" else default


def as_bool(row, field):
    return str(row.get(field, "")).lower() in ("1", "true", "yes")
