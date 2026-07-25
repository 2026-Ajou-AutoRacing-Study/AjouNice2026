import csv


# Schema v5: the permanent compact runtime contract. Configuration and other
# run-constant context belong in metadata.json, not in every sample row.
TRACKING_FIELDS = [
    "sample_index",
    "elapsed_time_s",
    "matched_gt_station_m",
    "ego_x_m",
    "ego_y_m",
    "ego_yaw_rad",
    "ego_velocity_mps",
    "ego_yaw_rate_rps",
    "target_velocity_mps",
    "pp_lookahead_distance_m",
    "pp_path_curvature_1pm",
    "pp_target_curvature_1pm",
    "lateral_error_m",
    "pp_lateral_error_m",
    "heading_error_rad",
    "cmd_steering_rad",
    "actual_steering_rad",
    "control_pair_valid",
    "pp_controller_valid",
    "pp_debug_fresh",
    "collision_detected",
]

# Schema v3/v4 contracts remain reader-only so existing history can still be
# replayed after new recordings switch to the compact v5 contract.
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
    "pp_controller_valid", "pp_status_fresh", "pp_status_age_s",
    "pp_failure_reason", "pp_lookahead_distance_m", "pp_path_curvature_1pm",
    "pp_target_curvature_1pm", "pp_lateral_error_m", "pp_steering_command_rad",
    "pp_debug_fresh", "pp_debug_age_s",
]

# v4 adds the Autoware Pure Pursuit telemetry columns above.  Keep v3 histories
# replayable: only the v3 contract is required when reading a pre-existing CSV.
CONTROL_REQUIRED_FIELDS = CONTROL_FIELDS[:46]

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


class TrackingCsvWriter:
    def __init__(self, path):
        self.stream = path.open("w", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=TRACKING_FIELDS)
        self.writer.writeheader()
        self.count = 0

    def write(self, row):
        self.writer.writerow({field: row[field] for field in TRACKING_FIELDS})
        self.count += 1
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def _load(path, fields, label):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(fields).difference(reader.fieldnames or [])
        if missing:
            raise ValueError("{} CSV missing fields: {}".format(label, sorted(missing)))
        return list(reader)


def load_legacy_run_csv(gt_path, control_path, response_path):
    gt_rows = _load(gt_path, GT_FIELDS, "GT")
    control_rows = _load(control_path, CONTROL_REQUIRED_FIELDS, "control cycles")
    response_rows = _load(response_path, RESPONSE_FIELDS, "vehicle responses")
    gt_rows.sort(key=lambda row: int(row["gt_index"]))
    control_rows.sort(key=lambda row: int(row["control_index"]))
    response_rows.sort(key=lambda row: int(row["response_index"]))
    return gt_rows, control_rows, response_rows


def load_tracking_csv(path):
    rows = _load(path, TRACKING_FIELDS, "tracking samples")
    rows.sort(key=lambda row: int(row["sample_index"]))
    for expected_index, row in enumerate(rows):
        if int(row["sample_index"]) != expected_index:
            raise ValueError("tracking sample indices are not contiguous from zero")
    return rows


# Backward-compatible import name used by older publisher modules.
load_run_csv = load_legacy_run_csv


def as_float(row, field, default=0.0):
    value = row.get(field, "")
    return float(value) if value != "" else default


def as_int(row, field, default=0):
    value = row.get(field, "")
    return int(value) if value != "" else default


def as_bool(row, field):
    return str(row.get(field, "")).lower() in ("1", "true", "yes")
