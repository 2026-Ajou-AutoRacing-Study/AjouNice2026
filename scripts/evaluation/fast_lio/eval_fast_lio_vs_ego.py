#!/usr/bin/env python3
import argparse
import csv
import math
import os
from bisect import bisect_left

import numpy as np
import rosbag


def stamp_to_sec(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def yaw_from_quat(q):
    # ROS quaternion: x, y, z, w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def nearest_by_time(times, target_t, max_dt):
    i = bisect_left(times, target_t)

    best_idx = None
    best_dt = None

    for j in (i - 1, i):
        if 0 <= j < len(times):
            dt = abs(times[j] - target_t)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = j

    if best_idx is None or best_dt is None or best_dt > max_dt:
        return None, None

    return best_idx, best_dt


def best_fit_2d_rotation(src_xy, dst_xy):
    """
    Find R that minimizes || R @ src - dst ||.
    src_xy, dst_xy: Nx2 relative trajectories.
    """
    h = src_xy.T @ dst_xy
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T

    # Avoid reflection
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T

    theta = math.atan2(r[1, 0], r[0, 0])
    return r, theta


def read_bag_topics(bag_path, ego_topic, odom_topic):
    ego = []
    odom = []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[ego_topic, odom_topic]):
            if topic == ego_topic:
                # Prefer message header stamp when valid, otherwise bag time.
                if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0:
                    ts = stamp_to_sec(msg.header.stamp)
                else:
                    ts = t.to_sec()

                x = msg.position.x
                y = msg.position.y
                yaw = math.radians(msg.heading)
                ego.append((ts, x, y, yaw))

            elif topic == odom_topic:
                if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0:
                    ts = stamp_to_sec(msg.header.stamp)
                else:
                    ts = t.to_sec()

                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = yaw_from_quat(q)
                odom.append((ts, p.x, p.y, yaw))

    ego.sort(key=lambda v: v[0])
    odom.sort(key=lambda v: v[0])

    return ego, odom


def evaluate(
    bag_path,
    out_csv,
    ego_topic="/Ego_topic",
    odom_topic="/localization/odom",
    max_time_diff=0.05,
):
    ego, odom = read_bag_topics(bag_path, ego_topic, odom_topic)

    if not ego:
        raise RuntimeError(f"No messages found on {ego_topic}")
    if not odom:
        raise RuntimeError(f"No messages found on {odom_topic}")

    odom_times = [v[0] for v in odom]

    matched = []
    for e in ego:
        idx, dt = nearest_by_time(odom_times, e[0], max_time_diff)
        if idx is None:
            continue
        o = odom[idx]
        matched.append((e, o, dt))

    if len(matched) < 10:
        raise RuntimeError(
            f"Too few matched samples: {len(matched)}. "
            f"Try increasing --max-dt or check timestamps."
        )

    ego_arr = np.array([[m[0][1], m[0][2]] for m in matched], dtype=float)
    odom_arr = np.array([[m[1][1], m[1][2]] for m in matched], dtype=float)

    ego_yaw = np.array([m[0][3] for m in matched], dtype=float)
    odom_yaw = np.array([m[1][3] for m in matched], dtype=float)
    times = np.array([m[0][0] for m in matched], dtype=float)
    sync_dt = np.array([m[2] for m in matched], dtype=float)

    # Convert to relative trajectories from first matched pose.
    ego_rel = ego_arr - ego_arr[0]
    odom_rel = odom_arr - odom_arr[0]

    # Align odom relative trajectory to ego relative trajectory by 2D rotation.
    r, align_yaw = best_fit_2d_rotation(odom_rel, ego_rel)
    odom_aligned = odom_rel @ r.T

    err_xy_vec = odom_aligned - ego_rel
    err_xy = np.linalg.norm(err_xy_vec, axis=1)

    # Relative yaw comparison. This is optional/debug metric.
    ego_dyaw = np.array([wrap_angle(y - ego_yaw[0]) for y in ego_yaw])
    odom_dyaw = np.array([wrap_angle(y - odom_yaw[0] + align_yaw) for y in odom_yaw])
    yaw_err = np.array([wrap_angle(o - e) for o, e in zip(odom_dyaw, ego_dyaw)])
    yaw_err_deg = np.degrees(yaw_err)

    # Path length from ego relative trajectory.
    ego_steps = np.linalg.norm(np.diff(ego_rel, axis=0), axis=1)
    ego_distance = float(np.sum(ego_steps))

    mean_err = float(np.mean(err_xy))
    rmse = float(math.sqrt(np.mean(err_xy ** 2)))
    max_err = float(np.max(err_xy))
    final_err = float(err_xy[-1])
    drift_percent = float(final_err / ego_distance * 100.0) if ego_distance > 1e-6 else float("nan")

    duration = float(times[-1] - times[0])

    print("========== FAST-LIO vs MORAI Ego Evaluation ==========")
    print(f"bag: {bag_path}")
    print(f"ego_topic:  {ego_topic} ({len(ego)} msgs)")
    print(f"odom_topic: {odom_topic} ({len(odom)} msgs)")
    print(f"matched samples: {len(matched)}")
    print(f"duration: {duration:.3f} sec")
    print(f"ego distance: {ego_distance:.3f} m")
    print(f"time sync | mean: {np.mean(sync_dt):.4f} sec, max: {np.max(sync_dt):.4f} sec")
    print(f"2D alignment yaw: {math.degrees(align_yaw):.3f} deg")
    print("------------------------------------------------------")
    print(f"XY mean error:  {mean_err:.3f} m")
    print(f"XY RMSE:        {rmse:.3f} m")
    print(f"XY max error:   {max_err:.3f} m")
    print(f"XY final error: {final_err:.3f} m")
    print(f"drift:          {drift_percent:.3f} %")
    print("------------------------------------------------------")
    print(f"Yaw error mean: {np.mean(np.abs(yaw_err_deg)):.3f} deg")
    print(f"Yaw error max:  {np.max(np.abs(yaw_err_deg)):.3f} deg")
    print("======================================================")

    if out_csv:
        os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)

        with open(out_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t",
                "ego_x_rel",
                "ego_y_rel",
                "odom_x_rel",
                "odom_y_rel",
                "odom_x_aligned",
                "odom_y_aligned",
                "err_x",
                "err_y",
                "err_xy",
                "ego_yaw_rel_deg",
                "odom_yaw_rel_aligned_deg",
                "yaw_err_deg",
                "sync_dt",
            ])

            for i in range(len(matched)):
                writer.writerow([
                    times[i],
                    ego_rel[i, 0],
                    ego_rel[i, 1],
                    odom_rel[i, 0],
                    odom_rel[i, 1],
                    odom_aligned[i, 0],
                    odom_aligned[i, 1],
                    err_xy_vec[i, 0],
                    err_xy_vec[i, 1],
                    err_xy[i],
                    math.degrees(ego_dyaw[i]),
                    math.degrees(odom_dyaw[i]),
                    yaw_err_deg[i],
                    sync_dt[i],
                ])

        print(f"CSV saved: {out_csv}")


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate FAST-LIO /localization/odom against MORAI /Ego_topic."
    )
    parser.add_argument("bag", help="Input rosbag containing /Ego_topic and /localization/odom")
    parser.add_argument("--out", default="", help="Output CSV path")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--odom-topic", default="/localization/odom")
    parser.add_argument(
        "--max-dt",
        type=float,
        default=0.05,
        help="Maximum timestamp difference for nearest-neighbor sync [sec]",
    )

    args = parser.parse_args()

    evaluate(
        bag_path=args.bag,
        out_csv=args.out,
        ego_topic=args.ego_topic,
        odom_topic=args.odom_topic,
        max_time_diff=args.max_dt,
    )


if __name__ == "__main__":
    main()
