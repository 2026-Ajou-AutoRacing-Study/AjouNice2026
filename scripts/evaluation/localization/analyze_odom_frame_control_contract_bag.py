#!/usr/bin/env python3
"""Read-only offline frame-contract A/B on a recorded control/planning bag."""

import bisect
import math
import os
import sys
from pathlib import Path

import numpy as np
import rosbag

ROOT = Path(os.environ.get("AJOUNICE_ROOT", Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(ROOT / "base_ws/src/localization/localization_manager/scripts"))
from smooth_control_state_publisher import PlanarAlignmentSmoother


TOPICS = {
    "/localization/kinematic_state": "global",
    "/localization/control_kinematic_state": "smooth",
    "/localization/odometry/local": "local",
    "/planning/trajectory": "trajectory",
    "/sensor/gps": "gps",
}


def pose(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    p = msg.pose.pose.position
    return (p.x, p.y, yaw)


def interp_local(items, ts, at):
    i = bisect.bisect_right(ts, at) - 1
    if i < 0 or at - ts[i] > 0.05:
        return None
    t, p, v = items[i]
    dt = at - t
    c, s = math.cos(p[2]), math.sin(p[2])
    return (p[0] + (c * v[0] - s * v[1]) * dt,
            p[1] + (s * v[0] + c * v[1]) * dt,
            p[2] + v[2] * dt)


def align(g, l):
    yaw = math.atan2(math.sin(g[2] - l[2]), math.cos(g[2] - l[2]))
    c, s = math.cos(yaw), math.sin(yaw)
    return (g[0] - c * l[0] + s * l[1],
            g[1] - s * l[0] - c * l[1], yaw)


def compose(a, l):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * l[0] - s * l[1],
            a[1] + s * l[0] + c * l[1], a[2] + l[2])


def lateral_error(p, path):
    xy = np.asarray(p[:2])
    first, second = path[:-1], path[1:]
    delta = second - first
    sq = np.sum(delta * delta, axis=1)
    valid = sq > 1e-8
    u = np.clip(np.sum((xy - first) * delta, axis=1) / np.maximum(sq, 1e-8), 0, 1)
    nearest = first + delta * u[:, None]
    distances = np.sum((xy - nearest) ** 2, axis=1)
    distances[~valid] = np.inf
    j = int(np.argmin(distances))
    cross = delta[j, 0] * (xy[1] - nearest[j, 1]) - delta[j, 1] * (xy[0] - nearest[j, 0])
    return math.copysign(math.sqrt(distances[j]), cross)


def pct(values):
    a = np.abs(np.asarray(values))
    return (np.percentile(a, 50), np.percentile(a, 95), np.percentile(a, 99), np.max(a))


def report(name, values):
    p = pct(values)
    print("%-24s n=%5d abs p50/p95/p99/max (cm) = %.2f / %.2f / %.2f / %.2f" %
          (name, len(values), *(100*x for x in p)))


def main(path):
    groups = {key: [] for key in TOPICS.values()}
    with rosbag.Bag(path) as bag:
        for topic, msg, received in bag.read_messages(topics=list(TOPICS)):
            key = TOPICS[topic]
            if key == "gps":
                groups[key].append((msg.header.stamp.to_sec(),
                                    bool(msg.latitude != 0.0 and msg.longitude != 0.0)))
            elif key == "trajectory":
                if msg.header.frame_id.lstrip("/") != "map" or len(msg.points) < 3:
                    continue
                points = np.asarray([(p.pose.position.x, p.pose.position.y) for p in msg.points], dtype=float)
                groups[key].append((received.to_sec(), msg.header.stamp.to_sec(), points))
            elif key == "local":
                tw = msg.twist.twist
                groups[key].append((msg.header.stamp.to_sec(), pose(msg),
                                    (tw.linear.x, tw.linear.y, tw.angular.z)))
            else:
                groups[key].append((received.to_sec(), msg.header.stamp.to_sec(), pose(msg)))
    for key in groups:
        groups[key].sort()
    local = groups["local"]
    lt = [x[0] for x in local]
    global_ = groups["global"]
    gt = [x[1] for x in global_]
    smooth = groups["smooth"]
    st = [x[1] for x in smooth]
    computed_smooth = []
    smoother = PlanarAlignmentSmoother(0.5, 0.5, reset_gap_sec=0.5,
                                      reset_translation_error_m=0.5,
                                      reset_yaw_error_rad=math.radians(20.0))
    for received, stamp, gp in global_:
        lp = interp_local(local, lt, stamp)
        if lp is None:
            continue
        try:
            smoothed = smoother.update(align(gp, lp), stamp, reference_pose=lp)
        except ValueError:
            continue
        computed_smooth.append((stamp, compose(smoothed, lp)))
    cst = [x[0] for x in computed_smooth]
    trajectory = groups["trajectory"]
    trt = [x[0] for x in trajectory]

    # Frame correction jumps delivered to a path frozen once per planner update.
    anchor_xy = []
    for tr_received, tr_source, _ in trajectory:
        m = bisect.bisect_right(gt, tr_received) - 1
        if m < 0 or tr_received - gt[m] > 0.5:
            continue
        anchor_local = interp_local(local, lt, gt[m])
        if anchor_local is not None:
            anchor_xy.append((tr_received, align(global_[m][2], anchor_local),
                              gt[m]))
    anchor_steps = []
    for first, second in zip(anchor_xy, anchor_xy[1:]):
        if second[0] - first[0] <= 0.5:
            reference = interp_local(local, lt, second[2])
            if reference is not None:
                p1, p2 = compose(first[1], reference), compose(second[1], reference)
                anchor_steps.append((second[0], math.hypot(p1[0] - p2[0],
                                                            p1[1] - p2[1])))

    reacquisition_times = []
    blank_start = None
    for stamp, valid in groups["gps"]:
        if not valid and blank_start is None:
            blank_start = stamp
        elif valid and blank_start is not None:
            if stamp - blank_start >= 1.0:
                reacquisition_times.append((stamp, stamp - blank_start))
            blank_start = None

    # Compare at the exact Global timestamp; Smooth is emitted on its callback.
    vals = {x: [] for x in ["global", "smooth", "odom_latest", "odom_held_receipt", "odom_held_source"]}
    differences = {x: [] for x in ["dynamic_global", "held_smooth_receipt", "held_smooth_source"]}
    ages, lags = [], []
    after_reacquisition = {name: [] for name in vals}
    for received, stamp, gp in global_:
        j = bisect.bisect_right(trt, received) - 1
        k = bisect.bisect_left(st, stamp)
        cs = bisect.bisect_left(cst, stamp)
        if j < 0 or cs >= len(cst) or abs(cst[cs] - stamp) > 1e-5:
            continue
        tr_received, tr_source, pathxy = trajectory[j]
        if received - tr_received > 0.5 or not pathxy.shape[0] >= 3:
            continue
        lp = interp_local(local, lt, stamp)
        if lp is None:
            continue
        sp = computed_smooth[cs][1]
        raw = align(gp, lp)
        # A: transform every path and car with the same current T; invariant.
        dynamic = compose(raw, lp)

        # B: freeze the path->odom transform once per new trajectory.
        anchors = []
        for anchor_time in (tr_received, tr_source):
            m = bisect.bisect_right(gt, anchor_time) - 1
            if m < 0 or anchor_time - gt[m] > 0.5:
                anchors.append(None)
                continue
            anchor_local = interp_local(local, lt, gt[m])
            anchors.append(align(global_[m][2], anchor_local) if anchor_local else None)
        if None in anchors:
            continue
        held_receipt = compose(anchors[0], lp)
        held_source = compose(anchors[1], lp)

        loc = {
            "global": gp,
            "smooth": sp,
            "odom_latest": dynamic,
            "odom_held_receipt": held_receipt,
            "odom_held_source": held_source,
        }
        errors = {name: lateral_error(p, pathxy) for name, p in loc.items()}
        for name, err in errors.items():
            vals[name].append(err)
            if any(0.0 <= stamp - start < 1.0 for start, _ in reacquisition_times):
                after_reacquisition[name].append(err)
        differences["dynamic_global"].append(errors["odom_latest"] - errors["global"])
        differences["held_smooth_receipt"].append(errors["odom_held_receipt"] - errors["smooth"])
        differences["held_smooth_source"].append(errors["odom_held_source"] - errors["smooth"])
        ages.append(received - tr_received)
        lags.append(tr_received - tr_source)

    print("bag:", path)
    print("n global/local/recorded_smooth/trajectory:", *(len(groups[k]) for k in ("global", "local", "smooth", "trajectory")))
    if smooth:
        mismatch = []
        for stamp, p in computed_smooth:
            k = bisect.bisect_left(st, stamp)
            if k < len(st) and abs(st[k] - stamp) < 1e-5:
                rp = smooth[k][2]
                mismatch.append(math.hypot(p[0] - rp[0], p[1] - rp[1]))
        if mismatch:
            print("computed vs recorded Smooth XY p50/p95 (cm):", *(round(100*x, 4) for x in np.percentile(mismatch,[50,95])))
    print("matched controls:", len(ages))
    if anchor_steps:
        print("held-path alignment step p50/p95/p99/max (cm):", *(round(100*x, 2) for x in pct([x[1] for x in anchor_steps])))
        print("largest held-path alignment step bag time / size:",
              max(anchor_steps, key=lambda x: x[1]))
    print("GPS blank reacquisition time / blank duration (sec):", reacquisition_times)
    print("trajectory receipt age p50/p95 ms:", *(round(1000*x, 1) for x in np.percentile(ages,[50,95])))
    print("trajectory source-to-receipt lag p50/p95 ms:", *(round(1000*x, 1) for x in np.percentile(lags,[50,95])))
    for name, v in vals.items():
        report(name, v)
    for name, v in after_reacquisition.items():
        if v:
            report("reacq_1s_" + name, v)
    for name, v in differences.items():
        report("difference_"+name, v)


if __name__ == "__main__":
    main(sys.argv[1])
