#!/usr/bin/env python3
"""
Build MORAI Sunny13 1500-frame PointPainting dataset (9 features).

Input dataset:
    /path/to/pointpainting_sunny13_1500_semantic_v1

Default processed splits:
    train (1050 frames)
    val   (228 frames)

Test is intentionally NOT processed by default.

Point feature order:
    0 x
    1 y
    2 z
    3 intensity
    4 background
    5 pedestrian
    6 vehicle
    7 obstacle
    8 unknown

Important:
- Camera *_Intensity.png is used as LRASPP input.
- MORAI *_Semantic.png / LIDAR *_Semantic.bin / *_Instance.bin
  are NEVER used as model input.
- LiDAR raw format is float32 Nx4 [x,y,z,intensity].
- Dummy/no-return points are removed only by range >= 0.5 m.
  We do NOT require intensity > 0.
- Multi-camera semantic fusion is equal averaging over cameras in which
  a LiDAR point is visible.
- unknown=1 only if the point is unseen by all four cameras.
"""

import argparse
import csv
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
import torchvision
import torchvision.transforms.functional as TF

try:
    import cv2
except ImportError:
    cv2 = None


CAMERA_ORDER = ("front", "front_left", "front_right", "back")
CLASS_NAMES = ("Background", "Pedestrian", "Vehicle", "Obstacle")
CLASS_TO_OUTPUT = {
    "pedestrian": "Pedestrian",
    "vehicle": "Vehicle",
    "obstacle": "Obstacle",
}

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


def parse_args():
    p = argparse.ArgumentParser()

    p.add_argument(
        "--input-root",
        required=True,
        help="Path to pointpainting_sunny13_1500_semantic_v1 dataset.",
    )
    p.add_argument(
        "--output-root",
        required=True,
        help="Output directory for the 9-feature PointPainting dataset.",
    )
    p.add_argument(
        "--checkpoint",
        required=True,
        help="Path to the LRASPP 4-class checkpoint.",
    )
    p.add_argument(
        "--splits",
        nargs="+",
        default=["train", "val"],
        choices=["train", "val", "test"],
        help="Default intentionally excludes sealed test.",
    )
    p.add_argument(
        "--short-side",
        type=int,
        default=384,
        help="LRASPP short-side resize used by current baseline.",
    )
    p.add_argument(
        "--preprocess-backend",
        choices=("torchvision", "opencv"),
        default="torchvision",
        help=(
            "Camera resize/tensor/normalize backend. torchvision preserves "
            "the training baseline; opencv matches the optional ROS runtime."
        ),
    )
    p.add_argument(
        "--min-lidar-range",
        type=float,
        default=0.5,
        help="Remove MORAI dummy/no-return points below this 3D range.",
    )
    p.add_argument(
        "--min-camera-depth",
        type=float,
        default=0.05,
    )
    p.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="0 = all selected frames; positive = smoke test.",
    )
    p.add_argument(
        "--device",
        default="cuda",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
    )
    return p.parse_args()


def load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_state_dict_flexible(path):
    ckpt = torch.load(path, map_location="cpu")

    if isinstance(ckpt, dict):
        for key in (
            "model_state",
            "model_state_dict",
            "state_dict",
            "model",
        ):
            if key in ckpt and isinstance(ckpt[key], dict):
                ckpt = ckpt[key]
                break

    if not isinstance(ckpt, dict):
        raise TypeError(
            f"Unsupported checkpoint object: {type(ckpt)}"
        )

    # Handle DataParallel checkpoints if necessary.
    if any(k.startswith("module.") for k in ckpt.keys()):
        ckpt = {
            (k[7:] if k.startswith("module.") else k): v
            for k, v in ckpt.items()
        }

    return ckpt


def build_lraspp_4class(checkpoint, device):
    """
    torchvision 0.11-compatible construction without downloading weights.
    """
    try:
        model = torchvision.models.segmentation.lraspp_mobilenet_v3_large(
            pretrained=False,
            pretrained_backbone=False,
            num_classes=4,
        )
    except TypeError:
        # Fallback for newer torchvision API.
        model = torchvision.models.segmentation.lraspp_mobilenet_v3_large(
            weights=None,
            weights_backbone=None,
            num_classes=4,
        )

    state = load_state_dict_flexible(checkpoint)

    try:
        model.load_state_dict(state, strict=True)
    except RuntimeError as exc:
        print("\n[checkpoint strict load failed]")
        print(exc)
        print("\nModel classifier:")
        print(model.classifier)
        raise

    model.to(device)
    model.eval()
    return model


def resize_short_side_pil(image, short_side):
    """
    Match torchvision Resize(short_side): preserve aspect ratio.
    """
    w, h = image.size

    if min(w, h) == short_side:
        return image

    if w < h:
        new_w = short_side
        new_h = int(short_side * h / w)
    else:
        new_h = short_side
        new_w = int(short_side * w / h)

    # torchvision/PIL baseline uses bilinear for image resize.
    return TF.resize(
        image,
        [new_h, new_w],
        interpolation=Image.BILINEAR,
    )


def preprocess_image(path, short_side, backend):
    if backend == "opencv":
        if cv2 is None:
            raise RuntimeError("OpenCV backend requested but cv2 is unavailable")
        image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ValueError(f"OpenCV failed to decode image: {path}")
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        orig_h, orig_w = image_rgb.shape[:2]
        if orig_h <= orig_w:
            new_h = short_side
            new_w = int(short_side * orig_w / orig_h)
        else:
            new_w = short_side
            new_h = int(short_side * orig_h / orig_w)
        resized = cv2.resize(
            image_rgb,
            (new_w, new_h),
            interpolation=cv2.INTER_LINEAR,
        )
        tensor = (
            torch.from_numpy(resized)
            .permute(2, 0, 1)
            .to(dtype=torch.float32)
            .div_(255.0)
        )
        mean = torch.as_tensor(IMAGENET_MEAN, dtype=torch.float32)[:, None, None]
        std = torch.as_tensor(IMAGENET_STD, dtype=torch.float32)[:, None, None]
        return tensor.sub_(mean).div_(std), (orig_w, orig_h)

    with Image.open(path) as im:
        image = im.convert("RGB")

    orig_w, orig_h = image.size
    resized = resize_short_side_pil(image, short_side)

    tensor = TF.to_tensor(resized)
    tensor = TF.normalize(
        tensor,
        mean=IMAGENET_MEAN,
        std=IMAGENET_STD,
    )

    return tensor, (orig_w, orig_h)


@torch.no_grad()
def infer_four_cameras(model, camera_paths, short_side, device, backend):
    """
    Inference is grouped by resized tensor shape.
    front/back generally form one batch and side cameras another batch.
    Returns:
        probs_original[camera] = float32 [4,H_original,W_original]
    """
    prepared = {}
    groups = defaultdict(list)

    for camera_name in CAMERA_ORDER:
        tensor, original_size = preprocess_image(
            camera_paths[camera_name],
            short_side,
            backend,
        )
        prepared[camera_name] = {
            "tensor": tensor,
            "original_size": original_size,
        }
        groups[tuple(tensor.shape[-2:])].append(camera_name)

    outputs = {}

    for _, camera_names in groups.items():
        batch = torch.stack(
            [prepared[name]["tensor"] for name in camera_names],
            dim=0,
        ).to(device, non_blocking=True)

        logits = model(batch)["out"]
        probs = torch.softmax(logits, dim=1)

        for batch_index, camera_name in enumerate(camera_names):
            orig_w, orig_h = prepared[camera_name]["original_size"]

            prob_original = F.interpolate(
                probs[batch_index:batch_index + 1],
                size=(orig_h, orig_w),
                mode="bilinear",
                align_corners=False,
            )[0]

            outputs[camera_name] = (
                prob_original.detach()
                .cpu()
                .numpy()
                .astype(np.float32)
            )

    return outputs


def load_lidar_intensity(path, min_range):
    raw = np.fromfile(path, dtype=np.float32)

    if raw.size % 4 != 0:
        raise ValueError(
            f"{path}: float count {raw.size} is not divisible by 4"
        )

    points = raw.reshape(-1, 4)

    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points[:, :3], axis=1)
    valid = finite & (ranges >= float(min_range))

    return points[valid].astype(np.float32), int(len(points)), int(valid.sum())


def project_points_to_camera(
    xyz,
    intrinsic,
    lidar_to_camera_optical,
    width,
    height,
    min_depth,
):
    """
    xyz: Nx3 LiDAR coordinates.
    Returns point indices and integer original-image pixels.
    """
    n = len(xyz)
    xyz_h = np.concatenate(
        [
            xyz.astype(np.float64),
            np.ones((n, 1), dtype=np.float64),
        ],
        axis=1,
    )

    cam_h = (
        np.asarray(lidar_to_camera_optical, dtype=np.float64)
        @ xyz_h.T
    ).T

    cam_xyz = cam_h[:, :3]
    depth = cam_xyz[:, 2]

    valid_depth = np.isfinite(cam_xyz).all(axis=1) & (depth > min_depth)

    point_indices = np.nonzero(valid_depth)[0]
    if len(point_indices) == 0:
        return (
            np.zeros((0,), dtype=np.int64),
            np.zeros((0,), dtype=np.int64),
            np.zeros((0,), dtype=np.int64),
        )

    p = cam_xyz[point_indices]

    uvw = (
        np.asarray(intrinsic, dtype=np.float64)
        @ p.T
    ).T

    u = uvw[:, 0] / uvw[:, 2]
    v = uvw[:, 1] / uvw[:, 2]

    inside = (
        np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0.0)
        & (u < float(width))
        & (v >= 0.0)
        & (v < float(height))
    )

    point_indices = point_indices[inside]
    u = np.floor(u[inside]).astype(np.int64)
    v = np.floor(v[inside]).astype(np.int64)

    # Safety clamp against numerical edge cases.
    u = np.clip(u, 0, width - 1)
    v = np.clip(v, 0, height - 1)

    return point_indices, u, v


def paint_points(points_xyzi, probs_by_camera, calibration, min_camera_depth):
    n = len(points_xyzi)

    semantic_sum = np.zeros((n, 4), dtype=np.float32)
    seen_count = np.zeros((n,), dtype=np.int32)
    camera_visible_counts = {}

    xyz = points_xyzi[:, :3]

    for camera_name in CAMERA_ORDER:
        cam_calib = calibration["cameras"][camera_name]

        width, height = [
            int(x)
            for x in cam_calib["resolution"]
        ]

        point_idx, u, v = project_points_to_camera(
            xyz=xyz,
            intrinsic=cam_calib["intrinsic"],
            lidar_to_camera_optical=cam_calib["lidar_to_camera_optical"],
            width=width,
            height=height,
            min_depth=min_camera_depth,
        )

        camera_visible_counts[camera_name] = int(len(point_idx))

        if len(point_idx) == 0:
            continue

        probs = probs_by_camera[camera_name]
        if probs.shape != (4, height, width):
            raise ValueError(
                f"{camera_name}: probability shape {probs.shape} "
                f"does not match calibration resolution "
                f"(4,{height},{width})"
            )

        sampled = probs[:, v, u].T  # Mx4

        semantic_sum[point_idx] += sampled
        seen_count[point_idx] += 1

    final_semantic = np.zeros((n, 5), dtype=np.float32)

    seen = seen_count > 0
    if np.any(seen):
        final_semantic[seen, :4] = (
            semantic_sum[seen]
            / seen_count[seen, None].astype(np.float32)
        )
        final_semantic[seen, 4] = 0.0

    final_semantic[~seen, :4] = 0.0
    final_semantic[~seen, 4] = 1.0

    painted = np.concatenate(
        [points_xyzi.astype(np.float32), final_semantic],
        axis=1,
    )

    return painted, seen_count, camera_visible_counts


def build_box_lookup(box_rows):
    lookup = {}

    for row in box_rows:
        frame_id = row["frame_id"]

        boxes = []
        for obj in row.get("boxes", []):
            cls = str(obj.get("class", "")).strip().lower()

            if cls not in CLASS_TO_OUTPUT:
                continue

            box = np.asarray(
                obj["box_lidar"],
                dtype=np.float64,
            ).reshape(-1)

            if box.size != 7:
                raise ValueError(
                    f"{frame_id}: bad box_lidar size {box.size}"
                )

            if not np.isfinite(box).all():
                continue

            if np.any(box[3:6] <= 0):
                continue

            boxes.append(
                {
                    "box": box,
                    "class": CLASS_TO_OUTPUT[cls],
                }
            )

        lookup[frame_id] = boxes

    return lookup


def write_label_file(path, boxes):
    with open(path, "w", encoding="utf-8") as f:
        for item in boxes:
            b = item["box"]
            f.write(
                f"{b[0]:.6f} {b[1]:.6f} {b[2]:.6f} "
                f"{b[3]:.6f} {b[4]:.6f} {b[5]:.6f} "
                f"{b[6]:.6f} {item['class']}\n"
            )


def verify_split_files(input_root, frames):
    expected_scene_sets = {}

    for split in ("train", "val", "test"):
        path = input_root / "splits" / f"{split}.txt"
        with open(path, "r", encoding="utf-8") as f:
            expected_scene_sets[split] = {
                line.strip()
                for line in f
                if line.strip()
            }

    for row in frames:
        split = row["split"]
        scene_id = row["scene_id"]

        if scene_id not in expected_scene_sets[split]:
            raise RuntimeError(
                f"Split mismatch: frame={row['frame_id']} "
                f"metadata split={split}, scene={scene_id}"
            )


def main():
    args = parse_args()

    input_root = Path(args.input_root)
    output_root = Path(args.output_root)

    points_dir = output_root / "points"
    labels_dir = output_root / "labels"
    imagesets_dir = output_root / "ImageSets"

    for d in (
        output_root,
        points_dir,
        labels_dir,
        imagesets_dir,
    ):
        d.mkdir(parents=True, exist_ok=True)

    frames_path = input_root / "metadata" / "frames.jsonl"
    boxes_path = input_root / "annotations" / "boxes_3d.jsonl"
    calibration_path = (
        input_root
        / "metadata"
        / "calibrations"
        / "sensor6_v1.json"
    )

    frames = load_jsonl(frames_path)
    box_rows = load_jsonl(boxes_path)

    verify_split_files(input_root, frames)

    with open(calibration_path, "r", encoding="utf-8") as f:
        calibration = json.load(f)

    box_lookup = build_box_lookup(box_rows)

    selected_splits = set(args.splits)
    selected_frames = [
        row
        for row in frames
        if row["split"] in selected_splits
    ]

    if args.max_frames > 0:
        selected_frames = selected_frames[:args.max_frames]

    print("\n========== SUNNY13 1500 -> POINTPAINTING 9FEAT ==========")
    print("input root       :", input_root)
    print("output root      :", output_root)
    print("checkpoint       :", args.checkpoint)
    print("selected splits  :", sorted(selected_splits))
    print("selected frames  :", len(selected_frames))
    print("short side       :", args.short_side)
    print("preprocess       :", args.preprocess_backend)
    print("min LiDAR range  :", args.min_lidar_range)
    print("feature order    : x y z intensity bg ped vehicle obstacle unknown")
    print("MORAI GT semantic: NOT USED AS INPUT")
    print("=========================================================\n")

    if not selected_frames:
        raise RuntimeError("No frames selected.")

    device = torch.device(args.device)

    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but torch.cuda.is_available() is False")

        print("GPU:", torch.cuda.get_device_name(0))

    model = build_lraspp_4class(
        args.checkpoint,
        device,
    )

    split_frame_ids = defaultdict(list)

    manifest_path = output_root / "manifest.csv"

    manifest_fields = [
        "frame_id",
        "scene_id",
        "split",
        "raw_points",
        "valid_points",
        "seen_by_any_camera",
        "unknown_points",
        "visible_front",
        "visible_front_left",
        "visible_front_right",
        "visible_back",
        "gt_total",
        "gt_pedestrian",
        "gt_vehicle",
        "gt_obstacle",
        "segmentation_ms",
        "painting_ms",
        "total_ms",
        "points_file",
        "label_file",
    ]

    stats = Counter()
    class_argmax_counts = Counter()
    timings = []

    with open(
        manifest_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as manifest_file:
        writer = csv.DictWriter(
            manifest_file,
            fieldnames=manifest_fields,
        )
        writer.writeheader()

        for index, frame in enumerate(selected_frames, start=1):
            t0 = time.perf_counter()

            frame_id = frame["frame_id"]
            split = frame["split"]

            out_points = points_dir / f"{frame_id}.npy"
            out_label = labels_dir / f"{frame_id}.txt"

            if (
                not args.overwrite
                and out_points.exists()
                and out_label.exists()
            ):
                print(
                    f"[{index}/{len(selected_frames)}] "
                    f"skip existing {frame_id}"
                )
                split_frame_ids[split].append(frame_id)
                continue

            lidar_path = input_root / frame["lidar"]["intensity_bin"]

            points_xyzi, raw_count, valid_count = load_lidar_intensity(
                lidar_path,
                args.min_lidar_range,
            )

            camera_paths = {
                camera_name: (
                    input_root
                    / frame["cameras"][camera_name]["intensity"]
                )
                for camera_name in CAMERA_ORDER
            }

            t_seg0 = time.perf_counter()

            probs_by_camera = infer_four_cameras(
                model=model,
                camera_paths=camera_paths,
                short_side=args.short_side,
                device=device,
                backend=args.preprocess_backend,
            )

            if device.type == "cuda":
                torch.cuda.synchronize()

            t_seg1 = time.perf_counter()

            painted, seen_count, camera_visible_counts = paint_points(
                points_xyzi=points_xyzi,
                probs_by_camera=probs_by_camera,
                calibration=calibration,
                min_camera_depth=args.min_camera_depth,
            )

            t_paint1 = time.perf_counter()

            if painted.shape[1] != 9:
                raise RuntimeError(
                    f"{frame_id}: painted shape {painted.shape}, expected Nx9"
                )

            if not np.isfinite(painted).all():
                raise RuntimeError(
                    f"{frame_id}: painted points contain NaN/Inf"
                )

            seen = seen_count > 0
            unknown_count = int((~seen).sum())

            # Visible semantic probabilities should approximately sum to 1.
            if np.any(seen):
                sums = painted[seen, 4:8].sum(axis=1)
                if not np.allclose(
                    sums,
                    1.0,
                    atol=2e-3,
                    rtol=2e-3,
                ):
                    raise RuntimeError(
                        f"{frame_id}: visible semantic probability sum "
                        f"outside tolerance: min={sums.min()} max={sums.max()}"
                    )

                predicted_class = np.argmax(
                    painted[seen, 4:8],
                    axis=1,
                )

                for cls_idx, cls_name in enumerate(CLASS_NAMES):
                    class_argmax_counts[cls_name] += int(
                        (predicted_class == cls_idx).sum()
                    )

            # Unknown feature contract.
            if unknown_count:
                unseen_sem = painted[~seen, 4:9]
                expected = np.zeros_like(unseen_sem)
                expected[:, 4] = 1.0

                if not np.allclose(
                    unseen_sem,
                    expected,
                    atol=1e-6,
                ):
                    raise RuntimeError(
                        f"{frame_id}: unknown feature contract violated"
                    )

            np.save(
                out_points,
                painted.astype(np.float32),
            )

            boxes = box_lookup.get(frame_id, [])
            write_label_file(out_label, boxes)

            split_frame_ids[split].append(frame_id)

            gt_counts = Counter(
                item["class"]
                for item in boxes
            )

            total_ms = (
                time.perf_counter() - t0
            ) * 1000.0

            segmentation_ms = (
                t_seg1 - t_seg0
            ) * 1000.0

            painting_ms = (
                t_paint1 - t_seg1
            ) * 1000.0

            timings.append(total_ms)

            stats["frames"] += 1
            stats["raw_points"] += raw_count
            stats["valid_points"] += valid_count
            stats["seen_points"] += int(seen.sum())
            stats["unknown_points"] += unknown_count
            stats["gt_total"] += len(boxes)
            stats["gt_pedestrian"] += gt_counts["Pedestrian"]
            stats["gt_vehicle"] += gt_counts["Vehicle"]
            stats["gt_obstacle"] += gt_counts["Obstacle"]

            writer.writerow(
                {
                    "frame_id": frame_id,
                    "scene_id": frame["scene_id"],
                    "split": split,
                    "raw_points": raw_count,
                    "valid_points": valid_count,
                    "seen_by_any_camera": int(seen.sum()),
                    "unknown_points": unknown_count,
                    "visible_front": camera_visible_counts["front"],
                    "visible_front_left": camera_visible_counts["front_left"],
                    "visible_front_right": camera_visible_counts["front_right"],
                    "visible_back": camera_visible_counts["back"],
                    "gt_total": len(boxes),
                    "gt_pedestrian": gt_counts["Pedestrian"],
                    "gt_vehicle": gt_counts["Vehicle"],
                    "gt_obstacle": gt_counts["Obstacle"],
                    "segmentation_ms": f"{segmentation_ms:.3f}",
                    "painting_ms": f"{painting_ms:.3f}",
                    "total_ms": f"{total_ms:.3f}",
                    "points_file": str(out_points),
                    "label_file": str(out_label),
                }
            )

            if (
                index <= 3
                or index % 25 == 0
                or index == len(selected_frames)
            ):
                print(
                    f"[{index:4d}/{len(selected_frames)}] "
                    f"{split:5s} {frame_id} | "
                    f"points {raw_count}->{valid_count} | "
                    f"seen {int(seen.sum())} "
                    f"unknown {unknown_count} | "
                    f"GT P/V/O="
                    f"{gt_counts['Pedestrian']}/"
                    f"{gt_counts['Vehicle']}/"
                    f"{gt_counts['Obstacle']} | "
                    f"{total_ms:.1f} ms"
                )

    # Write frame-level ImageSets, not scene IDs.
    for split in ("train", "val", "test"):
        ids = split_frame_ids.get(split, [])

        if not ids:
            continue

        with open(
            imagesets_dir / f"{split}.txt",
            "w",
            encoding="utf-8",
        ) as f:
            for frame_id in ids:
                f.write(frame_id + "\n")

    info_path = output_root / "dataset_info.txt"

    with open(info_path, "w", encoding="utf-8") as f:
        f.write("Sunny13 1500 PointPainting 9-feature dataset\n")
        f.write("===========================================\n")
        f.write(f"input_root={input_root}\n")
        f.write(f"checkpoint={args.checkpoint}\n")
        f.write(f"splits={sorted(selected_splits)}\n")
        f.write(f"short_side={args.short_side}\n")
        f.write(f"preprocess_backend={args.preprocess_backend}\n")
        f.write(f"min_lidar_range={args.min_lidar_range}\n")
        f.write(
            "point_features="
            "x,y,z,intensity,background,pedestrian,"
            "vehicle,obstacle,unknown\n"
        )
        f.write("MORAI_GT_semantic_used_as_input=False\n")
        f.write(f"frames={stats['frames']}\n")
        f.write(f"raw_points={stats['raw_points']}\n")
        f.write(f"valid_points={stats['valid_points']}\n")
        f.write(f"seen_points={stats['seen_points']}\n")
        f.write(f"unknown_points={stats['unknown_points']}\n")
        f.write(f"gt_total={stats['gt_total']}\n")
        f.write(f"gt_pedestrian={stats['gt_pedestrian']}\n")
        f.write(f"gt_vehicle={stats['gt_vehicle']}\n")
        f.write(f"gt_obstacle={stats['gt_obstacle']}\n")

        for split in ("train", "val", "test"):
            f.write(
                f"{split}_frames="
                f"{len(split_frame_ids.get(split, []))}\n"
            )

        for cls in CLASS_NAMES:
            f.write(
                f"semantic_argmax_{cls.lower()}="
                f"{class_argmax_counts[cls]}\n"
            )

        if timings:
            arr = np.asarray(timings, dtype=np.float64)
            f.write(f"total_ms_avg={arr.mean():.3f}\n")
            f.write(f"total_ms_p95={np.percentile(arr,95):.3f}\n")
            f.write(f"total_ms_max={arr.max():.3f}\n")

    print("\n================ BUILD COMPLETE ================")
    print("output:", output_root)
    print("frames:", stats["frames"])
    print(
        "split frames:",
        {
            k: len(v)
            for k, v in split_frame_ids.items()
        },
    )
    print(
        "GT P/V/O:",
        stats["gt_pedestrian"],
        stats["gt_vehicle"],
        stats["gt_obstacle"],
    )

    if stats["valid_points"] > 0:
        print(
            "seen ratio:",
            f"{stats['seen_points']/stats['valid_points']*100:.2f}%",
        )
        print(
            "unknown ratio:",
            f"{stats['unknown_points']/stats['valid_points']*100:.2f}%",
        )

    print(
        "semantic argmax:",
        dict(class_argmax_counts),
    )

    if timings:
        arr = np.asarray(timings, dtype=np.float64)
        print(
            "total ms avg/p95/max:",
            f"{arr.mean():.1f}/"
            f"{np.percentile(arr,95):.1f}/"
            f"{arr.max():.1f}",
        )

    print("manifest:", manifest_path)
    print("info    :", info_path)
    print("================================================")


if __name__ == "__main__":
    main()
