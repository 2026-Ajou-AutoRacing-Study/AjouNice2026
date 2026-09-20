#!/usr/bin/env python3
"""Compare raw MORAI GT support for paired CargoBox and WoodBox scenes."""

import argparse
import bisect
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

from audit_base3_online_bag import build_capture_windows, message_stamp
from audit_base3_online_gt_alignment import cloud_xyz_i, ego_relative_position, wrapped_radians


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def nearest(stamp, rows):
    stamps = [row[0] for row in rows]
    index = bisect.bisect_left(stamps, stamp)
    candidates = rows[max(0, index - 1):min(len(rows), index + 1)]
    return min(candidates, key=lambda row: abs(row[0] - stamp))


def raw_box_lidar(ego, actor, calibration):
    pivot = ego_relative_position(ego, actor)
    transform = np.asarray(calibration["lidar"]["ego_to_lidar"], dtype=np.float64)
    center = (transform @ pivot)[:3]
    size = np.asarray([actor.size.x, actor.size.y, actor.size.z], dtype=np.float64)
    # ObjectStatus position is the ground-contact pivot, whereas a 3D box is
    # centre based.  Use MORAI's size-derived z centre without any learned
    # asset-specific x/y correction; this is the contract under audit.
    center[2] += size[2] * 0.5
    yaw = wrapped_radians(math.radians(float(actor.heading) - float(ego.heading)))
    return np.r_[center, size, yaw]


def local_coordinates(points, box):
    translated = points[:, :3] - box[:3]
    cosine, sine = math.cos(float(box[6])), math.sin(float(box[6]))
    return np.c_[
        cosine * translated[:, 0] + sine * translated[:, 1],
        -sine * translated[:, 0] + cosine * translated[:, 1],
        translated[:, 2],
    ]


def support_counts(local, size):
    values = {}
    for tolerance_cm in (0, 3, 5, 10, 20):
        tolerance = tolerance_cm / 100.0
        half = size / 2.0 + tolerance
        values[str(tolerance_cm)] = int(np.all(np.abs(local) <= half, axis=1).sum())
    return values


def draw_closeup(path, points, box, title):
    local = local_coordinates(points, box)
    half = box[3:6] / 2.0
    keep = np.all(np.abs(local) <= half + np.asarray([0.5, 0.5, 0.4]), axis=1)
    xy = points[keep, :2]
    local_keep = local[keep]
    inside = np.all(np.abs(local_keep) <= half, axis=1)
    figure, axis = plt.subplots(figsize=(7, 7))
    if len(xy):
        axis.scatter(xy[inside, 0], xy[inside, 1], s=8, c="#2ca02c", label="inside raw GT")
        axis.scatter(xy[~inside, 0], xy[~inside, 1], s=12, c="#d62728", label="outside raw GT")
    cosine, sine = math.cos(float(box[6])), math.sin(float(box[6]))
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    for tolerance, color, label in ((0.0, "black", "raw GT"), (0.03, "#9467bd", "+3 cm")):
        hx, hy = box[3] / 2 + tolerance, box[4] / 2 + tolerance
        corners = np.asarray([[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy], [hx, hy]])
        world = corners @ rotation.T + box[:2]
        axis.plot(world[:, 0], world[:, 1], color=color, linewidth=2, label=label)
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.25)
    axis.set_title(title)
    axis.set_xlabel("LiDAR x [m]")
    axis.set_ylabel("LiDAR y [m]")
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    args = arguments()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    metadata = {row["id"]: row["asset_probe"] for row in manifest["scenes"]}
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)

    markers = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, message, stamp in bag.read_messages(topics=["/dataset/scenario_marker"]):
            markers.append((stamp.to_sec(), json.loads(message.data)))
    windows, errors = build_capture_windows(markers)
    if errors:
        raise SystemExit("marker errors: {}".format(errors))

    samples = defaultdict(lambda: defaultdict(list))
    topics = ("/sensor/lidar1", "/Ego_topic", "/Object_topic")
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, record_stamp in bag.read_messages(topics=topics):
            record = record_stamp.to_sec()
            for window in windows:
                if window["start"] <= record <= window["end"]:
                    samples[window["scene_id"]][topic].append(
                        (message_stamp(message, record_stamp), message)
                    )
                    break

    rows = []
    for window in windows:
        scene_id = window["scene_id"]
        scene = samples[scene_id]
        if any(not scene[topic] for topic in topics):
            raise SystemExit("missing topic in {}".format(scene_id))
        middle = (window["start"] + window["end"]) / 2.0
        lidar_stamp, lidar = nearest(middle, scene["/sensor/lidar1"])
        _, ego = nearest(lidar_stamp, scene["/Ego_topic"])
        _, objects = nearest(lidar_stamp, scene["/Object_topic"])
        expected_name = metadata[scene_id]["asset_name"]
        candidates = [actor for actor in objects.obstacle_list if str(actor.name) == expected_name]
        if len(candidates) != 1:
            raise SystemExit("{} expected one {}, got {}".format(scene_id, expected_name, len(candidates)))
        actor = candidates[0]
        points = cloud_xyz_i(lidar)
        box = raw_box_lidar(ego, actor, calibration)
        local = local_coordinates(points, box)
        counts = support_counts(local, box[3:6])
        within10 = counts["10"]
        row = {
            "scene_id": scene_id,
            **metadata[scene_id],
            "lidar_stamp": lidar_stamp,
            "lidar_points": len(points),
            "raw_box_lidar": box.tolist(),
            "points_by_gt_expansion_cm": counts,
            "outside_raw_within_10cm": within10 - counts["0"],
            "outside_raw_fraction_within_10cm": (
                (within10 - counts["0"]) / within10 if within10 else None
            ),
        }
        rows.append(row)
        draw_closeup(
            args.output_dir / (scene_id + "_bev.png"), points, box,
            "{} {}m yaw {}deg: raw={} +3cm={}".format(
                expected_name, row["distance_m"], row["relative_yaw_deg"],
                counts["0"], counts["3"],
            ),
        )

    grouped = {}
    for asset in ("CargoBox", "WoodBox"):
        selected = [row for row in rows if row["asset_name"] == asset]
        totals = {
            key: sum(row["points_by_gt_expansion_cm"][key] for row in selected)
            for key in ("0", "3", "5", "10")
        }
        grouped[asset] = {
            "scenes": len(selected),
            "totals_by_gt_expansion_cm": totals,
            "raw_support_zero_scenes": sum(row["points_by_gt_expansion_cm"]["0"] == 0 for row in selected),
            "outside_raw_fraction_within_10cm": (
                (totals["10"] - totals["0"]) / totals["10"] if totals["10"] else None
            ),
        }
    report = {
        "schema_version": 1,
        "bag": str(args.bag.resolve()),
        "manifest": str(args.manifest.resolve()),
        "summary": grouped,
        "samples": rows,
    }
    target = args.output_dir / "woodbox_alignment_audit.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(grouped, ensure_ascii=False, indent=2))
    print("report={}".format(target))


if __name__ == "__main__":
    main()
