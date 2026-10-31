#!/usr/bin/env python3
"""Evaluate a body-frame localization result at the vehicle base_link.

The script reads the IMU mounting pose from a MORAI Sensor JSON, applies
body -> base_link, and performs the same initial-Ego-pose alignment and
evaluation as eval_localization_pose_with_rviz.py.
"""

import argparse
import contextlib
import io
import json
import math
import os

import numpy as np

import eval_localization_pose_with_rviz as core


def read_imu_pose(sensor_config, imu_name):
    with open(sensor_config, "r") as source:
        config = json.load(source)

    candidates = config.get("IMUList", [])
    for entry in candidates:
        sensor_name = str(entry.get("ic", {}).get("sensorName", ""))
        if sensor_name == imu_name:
            position = entry.get("pos", {})
            rotation = entry.get("rot", {})
            xyz = np.array(
                [
                    float(position.get("x", 0.0)),
                    float(position.get("y", 0.0)),
                    float(position.get("z", 0.0)),
                ],
                dtype=float,
            )
            rpy_deg = np.array(
                [
                    float(rotation.get("roll", 0.0)),
                    float(rotation.get("pitch", 0.0)),
                    float(rotation.get("yaw", 0.0)),
                ],
                dtype=float,
            )
            return sensor_name, xyz, rpy_deg

    available = [str(entry.get("ic", {}).get("sensorName", "")) for entry in candidates]
    raise RuntimeError(
        f"IMU sensor '{imu_name}' not found in {sensor_config}; available={available}"
    )


def inverse_planar_mount(base_to_body_xyz, base_to_body_rpy_deg):
    roll_deg, pitch_deg, yaw_deg = base_to_body_rpy_deg
    if abs(roll_deg) > 1e-9 or abs(pitch_deg) > 1e-9:
        raise RuntimeError(
            "This evaluator currently supports a yaw-only IMU mounting rotation; "
            f"got roll={roll_deg}, pitch={pitch_deg} deg"
        )

    yaw_base_body = math.radians(yaw_deg)
    yaw_body_base = -yaw_base_body
    cosine = math.cos(yaw_body_base)
    sine = math.sin(yaw_body_base)
    rotation_body_base = np.array(
        [[cosine, -sine], [sine, cosine]], dtype=float
    )
    translation_body_base_xy = -rotation_body_base @ base_to_body_xyz[:2]
    translation_body_base = np.array(
        [translation_body_base_xy[0], translation_body_base_xy[1], -base_to_body_xyz[2]],
        dtype=float,
    )
    return translation_body_base, yaw_body_base


def body_samples_to_base(samples, translation_body_base, yaw_body_base):
    corrected = []
    offset_xy = translation_body_base[:2]
    for sample in samples:
        cosine = math.cos(sample.yaw)
        sine = math.sin(sample.yaw)
        rotation_camera_body = np.array(
            [[cosine, -sine], [sine, cosine]], dtype=float
        )
        base_xy = (
            np.array([sample.x, sample.y], dtype=float)
            + rotation_camera_body @ offset_xy
        )
        corrected.append(
            core.PoseSample(
                stamp=sample.stamp,
                bag_time=sample.bag_time,
                x=float(base_xy[0]),
                y=float(base_xy[1]),
                z=float(sample.z + translation_body_base[2]),
                yaw=core.wrap_angle(sample.yaw + yaw_body_base),
            )
        )
    return corrected


def print_and_save_corrected_report(
    bag_path,
    csv_path,
    sensor_config,
    imu_name,
    base_to_body_xyz,
    base_to_body_rpy_deg,
    translation_body_base,
    yaw_body_base,
    matched,
    relative_times,
    errors,
    yaw_errors,
    alignment_yaw,
    segments,
):
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        print("=== Base-link-corrected Localization vs MORAI Ego ===")
        print(f"sensor config: {sensor_config}")
        print(f"IMU sensor: {imu_name}")
        print(
            "T_base_body xyz: "
            f"[{base_to_body_xyz[0]:.3f}, {base_to_body_xyz[1]:.3f}, "
            f"{base_to_body_xyz[2]:.3f}] m"
        )
        print(
            "T_base_body rpy: "
            f"[{base_to_body_rpy_deg[0]:.3f}, {base_to_body_rpy_deg[1]:.3f}, "
            f"{base_to_body_rpy_deg[2]:.3f}] deg"
        )
        print(
            "T_body_base xyz: "
            f"[{translation_body_base[0]:.3f}, {translation_body_base[1]:.3f}, "
            f"{translation_body_base[2]:.3f}] m"
        )
        print(f"T_body_base yaw: {math.degrees(yaw_body_base):.3f} deg")
        print("------------------------------------------------------")
        core.print_report(
            bag_path,
            matched,
            relative_times,
            errors,
            yaw_errors,
            alignment_yaw,
            segments,
        )

    report = buffer.getvalue()
    print(report, end="")
    if not csv_path:
        return

    report_path = os.path.splitext(os.path.abspath(csv_path))[0] + ".txt"
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w") as output:
        output.write(report)
    print(f"Text report saved: {report_path}")


def main():
    default_sensor_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "configs",
            "sensors",
            "Sensor_competition_local.json",
        )
    )
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate localization after converting an IMU/body pose to the "
            "vehicle base_link using a MORAI Sensor JSON."
        )
    )
    parser.add_argument("bag", help="Bag containing /Ego_topic and /localization/odom")
    parser.add_argument("--out", default="", help="Output CSV path; a matching .txt is also saved")
    parser.add_argument("--sensor-config", default=default_sensor_config)
    parser.add_argument("--imu-name", default="imu")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--odom-topic", default="/localization/odom")
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--error-threshold", type=float, default=1.0)
    parser.add_argument("--publish-rviz", action="store_true")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--marker-stride", type=int, default=1)
    args = parser.parse_args()

    imu_name, base_to_body_xyz, base_to_body_rpy_deg = read_imu_pose(
        args.sensor_config, args.imu_name
    )
    translation_body_base, yaw_body_base = inverse_planar_mount(
        base_to_body_xyz, base_to_body_rpy_deg
    )

    ego, body_odom = core.read_bag_topics(args.bag, args.ego_topic, args.odom_topic)
    if not ego:
        raise RuntimeError(f"No messages found on {args.ego_topic}")
    if not body_odom:
        raise RuntimeError(f"No messages found on {args.odom_topic}")

    base_odom = body_samples_to_base(
        body_odom, translation_body_base, yaw_body_base
    )
    matched = core.match_samples(ego, base_odom, args.max_dt)
    if len(matched) < 10:
        raise RuntimeError(
            f"Too few unique matched samples: {len(matched)}. Increase --max-dt or check timestamps."
        )

    ego_xyz, aligned_xyz, aligned_yaw, alignment_yaw = core.align_from_initial_pose(matched)
    errors = np.linalg.norm((aligned_xyz - ego_xyz)[:, :2], axis=1)
    yaw_errors = np.array(
        [
            core.wrap_angle(aligned_yaw[index] - matched[index][0].yaw)
            for index in range(len(matched))
        ]
    )
    relative_times = np.array(
        [match[0].stamp - matched[0][0].stamp for match in matched]
    )
    segments = core.threshold_segments(errors, relative_times, args.error_threshold)

    print_and_save_corrected_report(
        args.bag,
        args.out,
        os.path.abspath(args.sensor_config),
        imu_name,
        base_to_body_xyz,
        base_to_body_rpy_deg,
        translation_body_base,
        yaw_body_base,
        matched,
        relative_times,
        errors,
        yaw_errors,
        alignment_yaw,
        segments,
    )
    core.write_csv(
        args.out,
        matched,
        relative_times,
        ego_xyz,
        aligned_xyz,
        aligned_yaw,
        errors,
        yaw_errors,
    )

    if args.publish_rviz:
        core.publish_rviz(
            args.frame_id,
            matched,
            ego_xyz,
            aligned_xyz,
            aligned_yaw,
            alignment_yaw,
            errors,
            args.error_threshold,
            args.marker_stride,
        )


if __name__ == "__main__":
    main()
