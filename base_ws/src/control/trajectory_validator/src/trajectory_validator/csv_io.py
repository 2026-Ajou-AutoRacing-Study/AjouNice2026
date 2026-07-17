import csv

from trajectory_validator.metrics import yaw_from_quaternion


COMMON_FIELDS = ["schema_version", "row_type", "traj_sample", "run_id"]
GT_FIELDS = [
    "gt_index",
    "gt_time_from_start_s",
    "gt_x_m",
    "gt_y_m",
    "gt_z_m",
    "gt_qx",
    "gt_qy",
    "gt_qz",
    "gt_qw",
    "gt_yaw_rad",
    "gt_longitudinal_velocity_mps",
    "gt_lateral_velocity_mps",
    "gt_acceleration_mps2",
    "gt_heading_rate_rps",
    "gt_front_wheel_angle_rad",
    "gt_rear_wheel_angle_rad",
]
SAMPLE_FIELDS = [
    "sample_index",
    "ros_time_ns",
    "elapsed_time_s",
    "dt_s",
    "matched_gt_index",
    "matched_gt_station_m",
    "target_velocity_mps",
    "ego_unique_id",
    "ego_x_m",
    "ego_y_m",
    "ego_z_m",
    "ego_yaw_rad",
    "ego_heading_raw",
    "ego_velocity_x_mps",
    "ego_velocity_y_mps",
    "ego_velocity_z_mps",
    "ego_velocity_mps",
    "ego_acceleration_x_mps2",
    "ego_acceleration_y_mps2",
    "ego_acceleration_z_mps2",
    "ego_acceleration_mps2",
    "ego_accel",
    "ego_brake",
    "ego_wheel_angle_raw",
    "actual_steering_rad",
    "ego_lateral_offset_m",
    "cmd_longl_type",
    "cmd_accel",
    "cmd_brake",
    "cmd_steering_rad",
    "cmd_velocity_mps",
    "cmd_acceleration_mps2",
    "lateral_error_m",
    "longitudinal_error_m",
    "heading_error_rad",
    "velocity_error_mps",
    "collision_detected",
    "reached_goal",
    "sample_valid",
]
ALL_FIELDS = COMMON_FIELDS + GT_FIELDS + SAMPLE_FIELDS


class RunCsvWriter:
    def __init__(self, path, schema_version, traj_sample, run_id):
        self.path = path
        self.schema_version = schema_version
        self.traj_sample = traj_sample
        self.run_id = run_id
        self.stream = path.open("w", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=ALL_FIELDS)
        self.writer.writeheader()
        self.gt_count = 0
        self.sample_count = 0

    def _write(self, row_type, values):
        row = {field: "" for field in ALL_FIELDS}
        row.update(
            {
                "schema_version": self.schema_version,
                "row_type": row_type,
                "traj_sample": self.traj_sample,
                "run_id": self.run_id,
            }
        )
        row.update(values)
        self.writer.writerow(row)

    def write_gt(self, trajectory):
        if self.gt_count:
            raise RuntimeError("GT rows were already written")
        for index, point in enumerate(trajectory.points):
            orientation = point.pose.orientation
            self._write(
                "GT_POINT",
                {
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
                },
            )
            self.gt_count += 1
        self.stream.flush()

    def write_sample(self, sample):
        self._write("SAMPLE", sample)
        self.sample_count += 1
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def load_run_csv(path):
    gt_rows = []
    sample_rows = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(ALL_FIELDS).difference(reader.fieldnames or [])
        if missing:
            raise ValueError("history CSV missing fields: {}".format(sorted(missing)))
        for row in reader:
            if row["row_type"] == "GT_POINT":
                gt_rows.append(row)
            elif row["row_type"] == "SAMPLE":
                sample_rows.append(row)
            else:
                raise ValueError("unknown history row_type: {}".format(row["row_type"]))
    gt_rows.sort(key=lambda row: int(row["gt_index"]))
    sample_rows.sort(key=lambda row: int(row["sample_index"]))
    return gt_rows, sample_rows


def as_float(row, field, default=0.0):
    value = row.get(field, "")
    return float(value) if value != "" else default


def as_int(row, field, default=0):
    value = row.get(field, "")
    return int(value) if value != "" else default


def as_bool(row, field):
    return str(row.get(field, "")).lower() in ("1", "true", "yes")
