#!/usr/bin/python3
"""Convert MORAI ROS 1 bags into uniformly sampled lateral-ID CSV files.

Required signals are /Ego_topic (actual wheel angle and vehicle velocity) and
/sensor/imu (yaw rate and lateral acceleration). /ctrl_cmd is optional and is
exported only as a diagnostic command trace. The output input/output order is
compatible with create_lateral_id_model.m:
    u = [delta_rad, vx_mps]
    y = [yaw_rate_radps, ay_mps2, vy_mps]
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import rosbag


EGO_FIELDS = ("delta_rad", "vx_mps", "vy_mps", "heading_rad")
IMU_FIELDS = ("yaw_rate_radps", "ay_mps2")
CTRL_FIELDS = ("ctrl_steering_rad", "ctrl_accel", "ctrl_brake")
CSV_FIELDS = ("t_s", "segment_id", "delta_rad", "vx_mps", "vy_mps",
              "yaw_rate_radps", "ay_mps2") + CTRL_FIELDS + ("heading_rad",)


def finite(value, label):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(label))
    return value


def timestamp(message, bag_time):
    header = getattr(message, "header", None)
    if header is not None and header.stamp is not None and header.stamp.to_sec() > 0.0:
        return header.stamp.to_sec()
    return bag_time.to_sec()


def add_sample(stream, sample_time, values):
    if stream and sample_time <= stream[-1][0]:
        # ROS bags normally have time-ordered records. Retain monotonic data
        # deterministically if a producer emitted an old Header timestamp.
        if sample_time == stream[-1][0]:
            stream[-1] = (sample_time, values)
        return
    stream.append((sample_time, values))


def interpolate(stream, sample_time, maximum_gap):
    """Return linear interpolation or None when outside/gapped."""
    if not stream or sample_time < stream[0][0] or sample_time > stream[-1][0]:
        return None
    times = [sample[0] for sample in stream]
    upper = bisect.bisect_left(times, sample_time)
    if upper < len(stream) and times[upper] == sample_time:
        return stream[upper][1]
    if upper == 0 or upper == len(stream):
        return None
    before_time, before_values = stream[upper - 1]
    after_time, after_values = stream[upper]
    interval = after_time - before_time
    if interval <= 0.0 or interval > maximum_gap:
        return None
    fraction = (sample_time - before_time) / interval
    return tuple(before + fraction * (after - before)
                 for before, after in zip(before_values, after_values))


def transform_velocity(vx, vy, heading, frame, yaw_offset):
    if frame == "body":
        return vx, vy
    yaw = heading + yaw_offset
    # R(-yaw) maps map-frame velocity into vehicle body coordinates.
    return math.cos(yaw) * vx + math.sin(yaw) * vy, \
        -math.sin(yaw) * vx + math.cos(yaw) * vy


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="input ROS 1 .bag file")
    parser.add_argument("--output", required=True, help="output CSV path")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--min-speed", type=float, default=3.0,
                        help="discard rows below this body speed [m/s]; use 0 to retain all")
    parser.add_argument("--max-gap-sec", type=float, default=0.15)
    parser.add_argument("--wheel-angle-unit", choices=("deg", "rad"), default="deg")
    parser.add_argument("--ego-velocity-frame", choices=("body", "map"), default="body")
    parser.add_argument("--heading-yaw-offset-rad", type=float, default=0.0,
                        help="used only for --ego-velocity-frame map")
    parser.add_argument("--steering-sign", type=float, default=1.0)
    parser.add_argument("--yaw-rate-sign", type=float, default=1.0)
    parser.add_argument("--ay-sign", type=float, default=1.0)
    parser.add_argument("--vy-sign", type=float, default=1.0)
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--imu-topic", default="/sensor/imu")
    parser.add_argument("--ctrl-topic", default="/ctrl_cmd")
    arguments = parser.parse_args()
    for name in ("sample_hz", "max_gap_sec", "steering_sign", "yaw_rate_sign", "ay_sign", "vy_sign"):
        finite(getattr(arguments, name), name)
    if arguments.sample_hz <= 0.0 or arguments.max_gap_sec <= 0.0:
        parser.error("--sample-hz and --max-gap-sec must be positive")
    if arguments.min_speed < 0.0:
        parser.error("--min-speed cannot be negative")
    return arguments


def read_streams(arguments):
    ego, imu, ctrl = [], [], []
    counts = {arguments.ego_topic: 0, arguments.imu_topic: 0, arguments.ctrl_topic: 0}
    topics = list(counts)
    with rosbag.Bag(arguments.bag, "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            sample_time = timestamp(message, bag_time)
            counts[topic] += 1
            if topic == arguments.ego_topic:
                delta = finite(message.wheel_angle, "wheel_angle")
                if arguments.wheel_angle_unit == "deg":
                    delta = math.radians(delta)
                heading = math.radians(finite(message.heading, "heading"))
                add_sample(ego, sample_time, (
                    arguments.steering_sign * delta,
                    finite(message.velocity.x, "velocity.x"),
                    finite(message.velocity.y, "velocity.y"),
                    heading))
            elif topic == arguments.imu_topic:
                add_sample(imu, sample_time, (
                    arguments.yaw_rate_sign * finite(message.angular_velocity.z, "angular_velocity.z"),
                    arguments.ay_sign * finite(message.linear_acceleration.y, "linear_acceleration.y")))
            elif topic == arguments.ctrl_topic:
                add_sample(ctrl, sample_time, (
                    finite(message.steering, "ctrl_cmd.steering"),
                    finite(message.accel, "ctrl_cmd.accel"),
                    finite(message.brake, "ctrl_cmd.brake")))
    if not ego:
        raise RuntimeError("no usable messages on {}".format(arguments.ego_topic))
    if not imu:
        raise RuntimeError("no usable messages on {}".format(arguments.imu_topic))
    return ego, imu, ctrl, counts


def write_csv(arguments, ego, imu, ctrl):
    output = Path(arguments.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    sample_period = 1.0 / arguments.sample_hz
    start = max(ego[0][0], imu[0][0])
    end = min(ego[-1][0], imu[-1][0])
    if end <= start:
        raise RuntimeError("Ego and IMU streams have no common time range")

    rows_written = 0
    rows_rejected_gap = 0
    rows_rejected_speed = 0
    segment_id = 0
    previous_kept_time = None
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        index = 0
        while True:
            current_time = start + index * sample_period
            if current_time > end + 1e-9:
                break
            ego_values = interpolate(ego, current_time, arguments.max_gap_sec)
            imu_values = interpolate(imu, current_time, arguments.max_gap_sec)
            if ego_values is None or imu_values is None:
                rows_rejected_gap += 1
                index += 1
                continue
            delta, raw_vx, raw_vy, heading = ego_values
            vx, vy = transform_velocity(raw_vx, raw_vy, heading,
                                        arguments.ego_velocity_frame,
                                        arguments.heading_yaw_offset_rad)
            vy *= arguments.vy_sign
            if math.hypot(vx, vy) < arguments.min_speed:
                rows_rejected_speed += 1
                index += 1
                continue
            if previous_kept_time is not None and current_time - previous_kept_time > 1.5 * sample_period:
                segment_id += 1
            ctrl_values = interpolate(ctrl, current_time, arguments.max_gap_sec)
            row = {
                "t_s": "{:.9f}".format(current_time - start),
                "segment_id": segment_id,
                "delta_rad": "{:.9f}".format(delta),
                "vx_mps": "{:.9f}".format(vx),
                "vy_mps": "{:.9f}".format(vy),
                "yaw_rate_radps": "{:.9f}".format(imu_values[0]),
                "ay_mps2": "{:.9f}".format(imu_values[1]),
                "heading_rad": "{:.9f}".format(heading),
                "ctrl_steering_rad": "" if ctrl_values is None else "{:.9f}".format(ctrl_values[0]),
                "ctrl_accel": "" if ctrl_values is None else "{:.9f}".format(ctrl_values[1]),
                "ctrl_brake": "" if ctrl_values is None else "{:.9f}".format(ctrl_values[2]),
            }
            writer.writerow(row)
            rows_written += 1
            previous_kept_time = current_time
            index += 1
    return output, {
        "time_start_epoch_s": start,
        "time_end_epoch_s": end,
        "sample_hz": arguments.sample_hz,
        "rows_written": rows_written,
        "rows_rejected_gap": rows_rejected_gap,
        "rows_rejected_speed": rows_rejected_speed,
        "segments": segment_id + 1 if rows_written else 0,
    }


def main():
    arguments = parse_arguments()
    bag_path = Path(arguments.bag).expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag does not exist: {}".format(bag_path))
    ego, imu, ctrl, counts = read_streams(arguments)
    output, summary = write_csv(arguments, ego, imu, ctrl)
    metadata = {
        "input_bag": str(bag_path),
        "output_csv": str(output),
        "topics": {
            "ego": arguments.ego_topic,
            "imu": arguments.imu_topic,
            "ctrl": arguments.ctrl_topic,
        },
        "options": vars(arguments),
        "source_message_counts": counts,
        "summary": summary,
        "model_contract": {
            "input": ["delta_rad", "vx_mps"],
            "output": ["yaw_rate_radps", "ay_mps2", "vy_mps"],
        },
    }
    metadata_path = output.with_suffix(output.suffix + ".metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("wrote {} rows to {}".format(summary["rows_written"], output))
    print("metadata: {}".format(metadata_path))


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, rosbag.ROSBagException) as error:
        raise SystemExit("bag_to_lateral_id_csv: {}".format(error))
