#!/usr/bin/env python3
"""Evaluate a localization pose against MORAI Ego GT with optional RViz overlays.

The alignment uses only the first synchronized Ego/odometry pair.  It does not
use a best-fit rotation over the complete GT trajectory, so later drift remains
visible in the reported errors.
"""

import argparse
import contextlib
import csv
import io
import math
import os
from bisect import bisect_left
from dataclasses import dataclass

import numpy as np
import rosbag


@dataclass
class PoseSample:
    stamp: float
    bag_time: float
    x: float
    y: float
    z: float
    yaw: float


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def message_time(msg, bag_time):
    if hasattr(msg, "header") and not msg.header.stamp.is_zero():
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def read_bag_topics(bag_path, ego_topic, odom_topic):
    ego = []
    odom = []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=[ego_topic, odom_topic]):
            stamp = message_time(msg, bag_time)
            if topic == ego_topic:
                ego.append(
                    PoseSample(
                        stamp=stamp,
                        bag_time=bag_time.to_sec(),
                        x=float(msg.position.x),
                        y=float(msg.position.y),
                        z=float(msg.position.z),
                        yaw=math.radians(float(msg.heading)),
                    )
                )
            elif topic == odom_topic:
                pose = msg.pose.pose
                odom.append(
                    PoseSample(
                        stamp=stamp,
                        bag_time=bag_time.to_sec(),
                        x=float(pose.position.x),
                        y=float(pose.position.y),
                        z=float(pose.position.z),
                        yaw=yaw_from_quat(pose.orientation),
                    )
                )

    ego.sort(key=lambda sample: sample.stamp)
    odom.sort(key=lambda sample: sample.stamp)
    return ego, odom


def nearest_index(times, target, max_dt):
    insertion = bisect_left(times, target)
    candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(times)]
    if not candidates:
        return None, None
    best = min(candidates, key=lambda index: abs(times[index] - target))
    dt = abs(times[best] - target)
    return (best, dt) if dt <= max_dt else (None, None)


def match_samples(ego, odom, max_dt):
    odom_times = [sample.stamp for sample in odom]
    matched = []
    last_odom_index = -1

    for ego_sample in ego:
        odom_index, dt = nearest_index(odom_times, ego_sample.stamp, max_dt)
        if odom_index is None or odom_index == last_odom_index:
            continue
        matched.append((ego_sample, odom[odom_index], dt))
        last_odom_index = odom_index

    return matched


def align_from_initial_pose(matched):
    ego0, odom0, _ = matched[0]
    alignment_yaw = wrap_angle(ego0.yaw - odom0.yaw)
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)

    ego_xyz = np.array([[m[0].x, m[0].y, m[0].z] for m in matched], dtype=float)
    odom_xyz = np.array([[m[1].x, m[1].y, m[1].z] for m in matched], dtype=float)

    odom_xy_relative = odom_xyz[:, :2] - odom_xyz[0, :2]
    aligned_xy = odom_xy_relative @ rotation.T + ego_xyz[0, :2]
    aligned_z = odom_xyz[:, 2] - odom_xyz[0, 2] + ego_xyz[0, 2]
    aligned_xyz = np.column_stack((aligned_xy, aligned_z))
    aligned_yaw = np.array(
        [wrap_angle(m[1].yaw + alignment_yaw) for m in matched], dtype=float
    )
    return ego_xyz, aligned_xyz, aligned_yaw, alignment_yaw


def threshold_segments(errors, relative_times, threshold):
    segments = []
    start = None
    for index, error in enumerate(errors):
        above = error >= threshold
        if above and start is None:
            start = index
        if start is not None and (not above or index == len(errors) - 1):
            end = index if above and index == len(errors) - 1 else index - 1
            local_max = start + int(np.argmax(errors[start : end + 1]))
            segments.append(
                {
                    "start_index": start,
                    "end_index": end,
                    "start_time": float(relative_times[start]),
                    "end_time": float(relative_times[end]),
                    "max_index": local_max,
                    "max_error": float(errors[local_max]),
                }
            )
            start = None
    return segments


def write_csv(path, matched, relative_times, ego_xyz, aligned_xyz, aligned_yaw, errors, yaw_errors):
    if not path:
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "t_rel",
                "ego_stamp",
                "ego_bag_time",
                "odom_stamp",
                "odom_bag_time",
                "sync_dt",
                "ego_x",
                "ego_y",
                "ego_z",
                "odom_aligned_x",
                "odom_aligned_y",
                "odom_aligned_z",
                "err_x",
                "err_y",
                "err_z",
                "err_xy",
                "ego_yaw_deg",
                "odom_aligned_yaw_deg",
                "yaw_err_deg",
            ]
        )
        error_xyz = aligned_xyz - ego_xyz
        for index, (ego_sample, odom_sample, sync_dt) in enumerate(matched):
            writer.writerow(
                [
                    relative_times[index],
                    ego_sample.stamp,
                    ego_sample.bag_time,
                    odom_sample.stamp,
                    odom_sample.bag_time,
                    sync_dt,
                    *ego_xyz[index],
                    *aligned_xyz[index],
                    *error_xyz[index],
                    errors[index],
                    math.degrees(ego_sample.yaw),
                    math.degrees(aligned_yaw[index]),
                    math.degrees(yaw_errors[index]),
                ]
            )
    print(f"CSV saved: {path}")


def print_report(bag_path, matched, relative_times, errors, yaw_errors, alignment_yaw, segments):
    ego_xy = np.array([[m[0].x, m[0].y] for m in matched], dtype=float)
    odom_xy = np.array([[m[1].x, m[1].y] for m in matched], dtype=float)
    ego_distance = float(np.linalg.norm(np.diff(ego_xy, axis=0), axis=1).sum())
    odom_distance = float(np.linalg.norm(np.diff(odom_xy, axis=0), axis=1).sum())
    path_length_ratio = odom_distance / ego_distance if ego_distance > 1e-9 else float("nan")
    path_length_error = odom_distance - ego_distance
    worst_index = int(np.argmax(errors))
    final_error = float(errors[-1])
    drift_percent = final_error / ego_distance * 100.0 if ego_distance > 1e-9 else float("nan")
    sync_dt = np.array([m[2] for m in matched], dtype=float)

    print("======== Localization Pose vs MORAI Ego Evaluation ========")
    print(f"bag: {bag_path}")
    print(f"matched samples: {len(matched)}")
    print(f"duration: {relative_times[-1]:.3f} sec")
    print(f"ego path length:      {ego_distance:.3f} m")
    print(f"state path length:    {odom_distance:.3f} m")
    print(f"path length ratio:    {path_length_ratio:.5f}")
    print(f"path length error:    {path_length_error:+.3f} m")
    print(f"initial alignment yaw: {math.degrees(alignment_yaw):.3f} deg")
    print(f"time sync | mean: {sync_dt.mean():.4f} sec, max: {sync_dt.max():.4f} sec")
    print("------------------------------------------------------")
    print(f"XY mean error:  {errors.mean():.3f} m")
    print(f"XY RMSE:        {math.sqrt(float(np.mean(errors ** 2))):.3f} m")
    print(f"XY max error:   {errors[worst_index]:.3f} m")
    print(f"XY final error: {final_error:.3f} m")
    print(f"drift:          {drift_percent:.3f} %")
    print(f"Yaw mean abs:   {np.degrees(np.abs(yaw_errors)).mean():.3f} deg")
    print(f"Yaw max abs:    {np.degrees(np.abs(yaw_errors)).max():.3f} deg")
    print("------------------------------------------------------")
    worst_ego, worst_odom, _ = matched[worst_index]
    print(
        "worst sample: "
        f"t_rel={relative_times[worst_index]:.3f}s, "
        f"ego_stamp={worst_ego.stamp:.9f}, odom_stamp={worst_odom.stamp:.9f}, "
        f"error={errors[worst_index]:.3f}m"
    )
    print(f"error segments above threshold: {len(segments)}")
    for number, segment in enumerate(segments, start=1):
        print(
            f"  [{number}] {segment['start_time']:.3f}s ~ {segment['end_time']:.3f}s, "
            f"max={segment['max_error']:.3f}m at "
            f"{relative_times[segment['max_index']]:.3f}s"
        )
    print("======================================================")


def print_and_save_report(
    bag_path,
    csv_path,
    matched,
    relative_times,
    errors,
    yaw_errors,
    alignment_yaw,
    segments,
):
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        print_report(
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
        return ""

    report_path = os.path.splitext(os.path.abspath(csv_path))[0] + ".txt"
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w") as output:
        output.write(report)
    print(f"Text report saved: {report_path}")
    return report_path


def quaternion_from_yaw(yaw, quaternion):
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)


def publish_rviz(
    frame_id,
    matched,
    ego_xyz,
    aligned_xyz,
    aligned_yaw,
    alignment_yaw,
    errors,
    threshold,
    marker_stride,
):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
    from nav_msgs.msg import Path
    from visualization_msgs.msg import Marker, MarkerArray

    rospy.init_node("localization_pose_eval_visualizer", anonymous=True)
    ego_publisher = rospy.Publisher(
        "/localization_pose_eval/ego_path", Path, queue_size=1, latch=True
    )
    state_publisher = rospy.Publisher(
        "/localization_pose_eval/state_aligned_path", Path, queue_size=1, latch=True
    )
    marker_publisher = rospy.Publisher(
        "/localization_pose_eval/error_markers", MarkerArray, queue_size=1, latch=True
    )
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    stamp = rospy.Time.now()
    ego_path = Path()
    ego_path.header.frame_id = frame_id
    ego_path.header.stamp = stamp
    state_path = Path()
    state_path.header = ego_path.header

    for index, (ego_sample, _, _) in enumerate(matched):
        ego_pose = PoseStamped()
        ego_pose.header = ego_path.header
        ego_pose.pose.position.x = float(ego_xyz[index, 0])
        ego_pose.pose.position.y = float(ego_xyz[index, 1])
        ego_pose.pose.position.z = float(ego_xyz[index, 2])
        quaternion_from_yaw(ego_sample.yaw, ego_pose.pose.orientation)
        ego_path.poses.append(ego_pose)

        state_pose = PoseStamped()
        state_pose.header = state_path.header
        state_pose.pose.position.x = float(aligned_xyz[index, 0])
        state_pose.pose.position.y = float(aligned_xyz[index, 1])
        state_pose.pose.position.z = float(aligned_xyz[index, 2])
        quaternion_from_yaw(aligned_yaw[index], state_pose.pose.orientation)
        state_path.poses.append(state_pose)

    points_marker = Marker()
    points_marker.header = ego_path.header
    points_marker.ns = "above_threshold"
    points_marker.id = 0
    points_marker.type = Marker.SPHERE_LIST
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = 0.6
    points_marker.color.r = 1.0
    points_marker.color.g = 0.15
    points_marker.color.b = 0.05
    points_marker.color.a = 0.9

    lines_marker = Marker()
    lines_marker.header = ego_path.header
    lines_marker.ns = "error_vectors"
    lines_marker.id = 1
    lines_marker.type = Marker.LINE_LIST
    lines_marker.action = Marker.ADD
    lines_marker.pose.orientation.w = 1.0
    lines_marker.scale.x = 0.08
    lines_marker.color.r = 1.0
    lines_marker.color.g = 0.65
    lines_marker.color.b = 0.0
    lines_marker.color.a = 0.8

    stride = max(1, marker_stride)
    for index in range(0, len(errors), stride):
        if errors[index] < threshold:
            continue
        ego_point = Point(
            x=float(ego_xyz[index, 0]),
            y=float(ego_xyz[index, 1]),
            z=float(ego_xyz[index, 2]),
        )
        state_point = Point(
            x=float(aligned_xyz[index, 0]),
            y=float(aligned_xyz[index, 1]),
            z=float(aligned_xyz[index, 2]),
        )
        points_marker.points.append(state_point)
        lines_marker.points.extend([ego_point, state_point])

    ego_publisher.publish(ego_path)
    state_publisher.publish(state_path)
    marker_publisher.publish(MarkerArray(markers=[points_marker, lines_marker]))

    ego0, odom0, _ = matched[0]
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = frame_id
    transform.child_frame_id = "camera_init"
    transform.transform.translation.x = ego0.x - (cosine * odom0.x - sine * odom0.y)
    transform.transform.translation.y = ego0.y - (sine * odom0.x + cosine * odom0.y)
    transform.transform.translation.z = ego0.z - odom0.z
    quaternion_from_yaw(alignment_yaw, transform.transform.rotation)
    static_tf_broadcaster.sendTransform(transform)

    rospy.loginfo(
        "Published localization evaluation paths and markers in frame '%s'",
        frame_id,
    )
    rospy.loginfo(
        "RViz topics: /localization_pose_eval/ego_path, "
        "/localization_pose_eval/state_aligned_path, "
        "/localization_pose_eval/error_markers"
    )
    rospy.loginfo("Published evaluation TF: %s -> camera_init", frame_id)
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate localization using the first synchronized Ego pose and "
            "publish optional RViz overlays."
        )
    )
    parser.add_argument("bag", help="Bag containing /Ego_topic and /localization/odom")
    parser.add_argument("--out", default="", help="Output CSV path")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--odom-topic", default="/localization/odom")
    parser.add_argument("--max-dt", type=float, default=0.05, help="Maximum sync difference [sec]")
    parser.add_argument(
        "--error-threshold", type=float, default=1.0, help="Error threshold for segments/markers [m]"
    )
    parser.add_argument("--publish-rviz", action="store_true", help="Publish latched RViz topics and wait")
    parser.add_argument("--frame-id", default="map", help="Frame for generated RViz messages")
    parser.add_argument("--marker-stride", type=int, default=1, help="Plot every Nth threshold sample")
    args = parser.parse_args()

    ego, odom = read_bag_topics(args.bag, args.ego_topic, args.odom_topic)
    if not ego:
        raise RuntimeError(f"No messages found on {args.ego_topic}")
    if not odom:
        raise RuntimeError(f"No messages found on {args.odom_topic}")

    matched = match_samples(ego, odom, args.max_dt)
    if len(matched) < 10:
        raise RuntimeError(
            f"Too few unique matched samples: {len(matched)}. Increase --max-dt or check timestamps."
        )

    ego_xyz, aligned_xyz, aligned_yaw, alignment_yaw = align_from_initial_pose(matched)
    error_xyz = aligned_xyz - ego_xyz
    errors = np.linalg.norm(error_xyz[:, :2], axis=1)
    yaw_errors = np.array(
        [wrap_angle(aligned_yaw[i] - matched[i][0].yaw) for i in range(len(matched))]
    )
    relative_times = np.array([m[0].stamp - matched[0][0].stamp for m in matched])
    segments = threshold_segments(errors, relative_times, args.error_threshold)

    print_and_save_report(
        args.bag,
        args.out,
        matched,
        relative_times,
        errors,
        yaw_errors,
        alignment_yaw,
        segments,
    )
    write_csv(
        args.out, matched, relative_times, ego_xyz, aligned_xyz, aligned_yaw, errors, yaw_errors
    )

    if args.publish_rviz:
        publish_rviz(
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
