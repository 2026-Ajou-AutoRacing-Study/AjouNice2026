#!/usr/bin/env python3
"""Audit LRASPP pedestrian scores at MORAI GT boxes and LiDAR returns.

Extraction uses the ROS system Python. Inference uses the PointPainting Conda
environment. Keeping the two phases separate avoids mixing ROS Noetic's Python
3.8 Cryptodome extension with the Python 3.11 training environment.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
CAMERA_TOPIC = "/sensor/camera/front/compressed"


def message_time(message, bag_time):
    stamp = message.header.stamp
    if stamp and (stamp.secs or stamp.nsecs):
        return float(stamp.to_sec())
    return float(bag_time.to_sec())


def nearest(records, timestamp):
    times = [item[0] for item in records]
    right = bisect.bisect_left(times, timestamp)
    candidates = range(max(0, right - 1), min(len(records), right + 2))
    return records[min(candidates, key=lambda index: abs(times[index] - timestamp))]


def world_to_ego(x, y, z, ego):
    yaw = math.radians(float(ego.heading))
    dx = float(x) - float(ego.position.x)
    dy = float(y) - float(ego.position.y)
    return np.asarray(
        [
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            float(z) - float(ego.position.z),
        ],
        dtype=np.float64,
    )


def project_xyz(xyz, lidar_to_camera, intrinsic):
    if len(xyz) == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty((0,), dtype=bool)
    homogeneous = np.concatenate(
        [np.asarray(xyz, dtype=np.float64), np.ones((len(xyz), 1))], axis=1
    )
    camera = (lidar_to_camera @ homogeneous.T).T[:, :3]
    valid = camera[:, 2] > 1.0e-3
    pixels_h = (intrinsic @ camera.T).T
    pixels = pixels_h[:, :2] / pixels_h[:, 2:3]
    return pixels, valid


def object_corners_ego(obj, ego):
    yaw = math.radians(float(obj.heading))
    result = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            local_x = sx * float(obj.size.x)
            local_y = sy * float(obj.size.y)
            world_x = (
                float(obj.position.x)
                + math.cos(yaw) * local_x
                - math.sin(yaw) * local_y
            )
            world_y = (
                float(obj.position.y)
                + math.sin(yaw) * local_x
                + math.cos(yaw) * local_y
            )
            for top in (0.0, 1.0):
                result.append(
                    world_to_ego(
                        world_x,
                        world_y,
                        float(obj.position.z) + top * float(obj.size.z),
                        ego,
                    )
                )
    return np.asarray(result)


def lidar_points_in_object(points, obj, ego, lidar_to_ego):
    center_ego = world_to_ego(
        obj.position.x,
        obj.position.y,
        float(obj.position.z) + 0.5 * float(obj.size.z),
        ego,
    )
    center_lidar = (np.linalg.inv(lidar_to_ego) @ np.r_[center_ego, 1.0])[:3]
    yaw = math.radians(float(obj.heading) - float(ego.heading))
    delta = points[:, :3] - center_lidar
    local_x = math.cos(yaw) * delta[:, 0] + math.sin(yaw) * delta[:, 1]
    local_y = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
    # Keep a small XY tolerance for the camera/ObjectStatus time offset, but
    # reject the road surface under the actor. Near pedestrians otherwise
    # collect many ground returns inside the footprint and falsely make the
    # semantic model look background-biased.
    margin_xy = 0.08
    bottom = -0.5 * float(obj.size.z) + 0.15
    top = 0.5 * float(obj.size.z) + 0.08
    return (
        (np.abs(local_x) <= 0.5 * float(obj.size.x) + margin_xy)
        & (np.abs(local_y) <= 0.5 * float(obj.size.y) + margin_xy)
        & (delta[:, 2] >= bottom)
        & (delta[:, 2] <= top)
    )


def extract(args):
    sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
    sys.path.insert(0, str(ROOT / "dataset/morai_od/scripts"))
    import cv2
    import rosbag
    from sensor_msgs import point_cloud2
    import build_fastbev_sunny13_1500_dataset as base

    calibration = base.calibration_from_sensor_config(
        args.sensor_config, ("front", "front_left", "front_right", "back")
    )
    camera = calibration["cameras"]["front"]
    lidar_to_camera = np.asarray(camera["lidar_to_camera_optical"], dtype=np.float64)
    intrinsic = np.asarray(camera["intrinsic"], dtype=np.float64)
    ego_to_camera = np.asarray(camera["ego_to_camera_optical"], dtype=np.float64)
    lidar_to_ego = np.asarray(calibration["lidar_to_ego"], dtype=np.float64)
    width, height = map(int, camera["resolution"])

    egos, objects, cameras, lidars = [], [], [], []
    topics = ["/Ego_topic", "/Object_topic", CAMERA_TOPIC, "/sensor/lidar1"]
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            record = (message_time(message, bag_time), message)
            if topic == "/Ego_topic":
                egos.append(record)
            elif topic == "/Object_topic":
                objects.append(record)
            elif topic == CAMERA_TOPIC:
                cameras.append(record)
            else:
                lidars.append(record)

    args.cache.mkdir(parents=True, exist_ok=True)
    image_dir = args.cache / "images"
    image_dir.mkdir(exist_ok=True)
    rows = []
    sampled_frames = 0
    for lidar_index in range(0, len(lidars), args.stride):
        lidar_time, cloud = lidars[lidar_index]
        camera_time, camera_message = nearest(cameras, lidar_time)
        ego_time, ego = nearest(egos, lidar_time)
        object_time, object_list = nearest(objects, lidar_time)
        if not object_list.pedestrian_list:
            continue
        points = np.asarray(
            list(
                point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True
                )
            ),
            dtype=np.float32,
        )
        image_name = "frame_{:05d}.jpg".format(sampled_frames)
        (image_dir / image_name).write_bytes(bytes(camera_message.data))
        image = cv2.imdecode(np.frombuffer(camera_message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise RuntimeError("unexpected front image shape")
        for obj in object_list.pedestrian_list:
            corners_ego = object_corners_ego(obj, ego)
            corners_h = np.concatenate([corners_ego, np.ones((8, 1))], axis=1)
            corners_camera = (ego_to_camera @ corners_h.T).T[:, :3]
            if np.count_nonzero(corners_camera[:, 2] > 0.0) < 4:
                continue
            corners_h2 = (intrinsic @ corners_camera.T).T
            corners_uv = corners_h2[:, :2] / corners_h2[:, 2:3]
            x1, y1 = np.min(corners_uv, axis=0)
            x2, y2 = np.max(corners_uv, axis=0)
            clipped = [
                max(0.0, min(width - 1.0, x1)),
                max(0.0, min(height - 1.0, y1)),
                max(0.0, min(width - 1.0, x2)),
                max(0.0, min(height - 1.0, y2)),
            ]
            if clipped[2] - clipped[0] < 2.0 or clipped[3] - clipped[1] < 4.0:
                continue
            mask = lidar_points_in_object(points, obj, ego, lidar_to_ego)
            point_uv, point_valid = project_xyz(
                points[mask, :3], lidar_to_camera, intrinsic
            )
            point_uv = point_uv[point_valid]
            point_uv = point_uv[
                (point_uv[:, 0] >= 0)
                & (point_uv[:, 0] < width)
                & (point_uv[:, 1] >= 0)
                & (point_uv[:, 1] < height)
            ]
            distance = float(np.linalg.norm(world_to_ego(
                obj.position.x, obj.position.y, obj.position.z, ego
            )[:2]))
            rows.append(
                {
                    "image": image_name,
                    "lidar_index": lidar_index,
                    "distance_m": distance,
                    "bbox_xyxy": [float(value) for value in clipped],
                    "gt_lidar_point_uv": point_uv.tolist(),
                    "gt_lidar_point_count_3d": int(mask.sum()),
                    "camera_lidar_dt_sec": float(camera_time - lidar_time),
                    "ego_lidar_dt_sec": float(ego_time - lidar_time),
                    "object_lidar_dt_sec": float(object_time - lidar_time),
                }
            )
        sampled_frames += 1
    manifest = {
        "bag": str(args.bag),
        "sensor_config": str(args.sensor_config),
        "camera": "front",
        "resolution": [width, height],
        "frames": sampled_frames,
        "objects": rows,
    }
    (args.cache / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"frames": sampled_frames, "objects": len(rows)}, indent=2))


def statistics(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return {"count": 0}
    return {
        "count": int(len(values)),
        "mean": float(np.mean(values)),
        "p10": float(np.percentile(values, 10)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
    }


def infer(args):
    import cv2
    import torch
    import torch.nn.functional as functional
    import torchvision
    from PIL import Image
    from torchvision import transforms

    manifest = json.loads((args.cache / "manifest.json").read_text())
    try:
        model = torchvision.models.segmentation.lraspp_mobilenet_v3_large(
            weights=None, weights_backbone=None, num_classes=4
        )
    except TypeError:
        model = torchvision.models.segmentation.lraspp_mobilenet_v3_large(
            pretrained=False, pretrained_backbone=False, num_classes=4
        )
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    state = checkpoint.get("model_state_dict", checkpoint.get("state_dict", checkpoint))
    state = {(key[7:] if key.startswith("module.") else key): value for key, value in state.items()}
    model.load_state_dict(state, strict=True)
    device = torch.device(args.device)
    model.to(device).eval()
    preprocess = transforms.Compose(
        [
            transforms.Resize(args.short_side, antialias=True),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
            ),
        ]
    )
    by_image = {}
    for row in manifest["objects"]:
        by_image.setdefault(row["image"], []).append(row)
    results = []
    for image_name, objects in by_image.items():
        image_path = args.cache / "images" / image_name
        rgb = Image.open(image_path).convert("RGB")
        width, height = rgb.size
        tensor = preprocess(rgb).unsqueeze(0).to(device)
        with torch.inference_mode():
            logits = model(tensor)["out"]
            probability = torch.softmax(logits.float(), dim=1)
            probability = functional.interpolate(
                probability, size=(height, width), mode="bilinear", align_corners=False
            )[0].cpu().numpy()
        label = np.argmax(probability, axis=0)
        for source in objects:
            x1, y1, x2, y2 = source["bbox_xyxy"]
            ix1, iy1 = int(math.floor(x1)), int(math.floor(y1))
            ix2, iy2 = int(math.ceil(x2)), int(math.ceil(y2))
            roi = probability[:, iy1 : iy2 + 1, ix1 : ix2 + 1]
            labels = label[iy1 : iy2 + 1, ix1 : ix2 + 1]
            lidar_uv = np.asarray(source["gt_lidar_point_uv"], dtype=np.float64)
            if len(lidar_uv):
                u = np.clip(np.floor(lidar_uv[:, 0]).astype(int), 0, width - 1)
                v = np.clip(np.floor(lidar_uv[:, 1]).astype(int), 0, height - 1)
                lidar_probs = probability[:, v, u]
            else:
                lidar_probs = np.empty((4, 0), dtype=np.float32)
            result = dict(source)
            result.update(
                {
                    "bbox_ped_probability_mean": float(np.mean(roi[1])),
                    "bbox_ped_probability_p50": float(np.median(roi[1])),
                    "bbox_ped_probability_p90": float(np.percentile(roi[1], 90)),
                    "bbox_ped_argmax_fraction": float(np.mean(labels == 1)),
                    "bbox_argmax_background_fraction": float(np.mean(labels == 0)),
                    "bbox_argmax_vehicle_fraction": float(np.mean(labels == 2)),
                    "bbox_argmax_obstacle_fraction": float(np.mean(labels == 3)),
                    "lidar_visible_point_count": int(lidar_probs.shape[1]),
                    "lidar_ped_probability_mean": (
                        float(np.mean(lidar_probs[1])) if lidar_probs.shape[1] else None
                    ),
                    "lidar_ped_argmax_fraction": (
                        float(np.mean(np.argmax(lidar_probs, axis=0) == 1))
                        if lidar_probs.shape[1]
                        else None
                    ),
                    "lidar_probability_mean_by_class": (
                        [float(value) for value in np.mean(lidar_probs, axis=1)]
                        if lidar_probs.shape[1]
                        else None
                    ),
                    "lidar_argmax_fraction_by_class": (
                        [
                            float(np.mean(np.argmax(lidar_probs, axis=0) == index))
                            for index in range(4)
                        ]
                        if lidar_probs.shape[1]
                        else None
                    ),
                }
            )
            results.append(result)
    usable_lidar = [row for row in results if row["lidar_ped_probability_mean"] is not None]
    distance_bins = {}
    for label, low, high in (
        ("0-7m", 0.0, 7.0),
        ("7-12m", 7.0, 12.0),
        ("12-16m", 12.0, 16.0),
        ("16m+", 16.0, float("inf")),
    ):
        selected = [row for row in usable_lidar if low <= row["distance_m"] < high]
        distance_bins[label] = {
            "objects": len(selected),
            "lidar_ped_probability_mean": statistics(
                [row["lidar_ped_probability_mean"] for row in selected]
            ),
            "lidar_ped_argmax_fraction": statistics(
                [row["lidar_ped_argmax_fraction"] for row in selected]
            ),
        }
    probability_by_class = []
    argmax_by_class = []
    for class_index in range(4):
        probability_by_class.append(
            statistics(
                [row["lidar_probability_mean_by_class"][class_index] for row in usable_lidar]
            )
        )
        argmax_by_class.append(
            statistics(
                [row["lidar_argmax_fraction_by_class"][class_index] for row in usable_lidar]
            )
        )
    summary = {
        "bag": manifest["bag"],
        "checkpoint": str(args.checkpoint),
        "short_side": args.short_side,
        "object_count": len(results),
        "bbox_ped_probability_mean": statistics(
            [row["bbox_ped_probability_mean"] for row in results]
        ),
        "bbox_ped_argmax_fraction": statistics(
            [row["bbox_ped_argmax_fraction"] for row in results]
        ),
        "lidar_objects_with_visible_points": len(usable_lidar),
        "lidar_visible_point_count": statistics(
            [row["lidar_visible_point_count"] for row in usable_lidar]
        ),
        "lidar_ped_probability_mean": statistics(
            [row["lidar_ped_probability_mean"] for row in usable_lidar]
        ),
        "lidar_ped_argmax_fraction": statistics(
            [row["lidar_ped_argmax_fraction"] for row in usable_lidar]
        ),
        "class_order": ["background", "pedestrian", "vehicle", "obstacle"],
        "lidar_probability_mean_by_class": probability_by_class,
        "lidar_argmax_fraction_by_class": argmax_by_class,
        "distance_bins": distance_bins,
        "records": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "records"}, indent=2))


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="command", required=True)
    extract_parser = commands.add_parser("extract")
    extract_parser.add_argument("bag", type=Path)
    extract_parser.add_argument("cache", type=Path)
    extract_parser.add_argument("--stride", type=int, default=3)
    extract_parser.add_argument(
        "--sensor-config", type=Path, default=ROOT / "configs/sensors/Sensor_6.json"
    )
    extract_parser.set_defaults(function=extract)
    infer_parser = commands.add_parser("infer")
    infer_parser.add_argument("cache", type=Path)
    infer_parser.add_argument("output", type=Path)
    infer_parser.add_argument("--checkpoint", type=Path, required=True)
    infer_parser.add_argument("--short-side", type=int, default=384)
    infer_parser.add_argument("--device", default="cuda")
    infer_parser.set_defaults(function=infer)
    return result


def main():
    args = parser().parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
