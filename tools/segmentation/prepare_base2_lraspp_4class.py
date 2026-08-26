#!/usr/bin/env python3
"""Derive clip-safe LRASPP masks/manifests from the frozen Base2 capture.

The MORAI semantic palette was verified over every Base2 2D GT box centre.
Source images are referenced in place; only compact class-ID masks and CSV
manifests are written to the output directory.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image


TARGET_COLORS = {
    (98, 2, 255): 1,       # Pedestrian.
    (255, 2, 2): 2,        # Vehicle semantic variant A.
    (255, 105, 105): 2,    # Vehicle semantic variant B.
    (236, 255, 2): 3,      # Obstacle.
}
PACKED_TARGET_COLORS = {
    (red << 16) | (green << 8) | blue: class_id
    for (red, green, blue), class_id in TARGET_COLORS.items()
}
CAMERAS = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def read_scene_split(source_root: Path):
    result = {}
    for split in ("train", "val", "test"):
        path = source_root / "splits" / f"{split}_scenes.txt"
        if not path.is_file():
            raise FileNotFoundError(path)
        for line in path.read_text(encoding="utf-8").splitlines():
            scene = line.strip()
            if not scene:
                continue
            if scene in result:
                raise RuntimeError(f"scene appears in multiple splits: {scene}")
            result[scene] = split
    return result


def frame_scene(stem: str):
    # r0822212456_t2base2b001p_drc001_f05 -> t2base2b001p_drc001.
    payload = stem.split("_", 1)[1]
    return payload.rsplit("_f", 1)[0]


def semantic_to_mask(path: Path):
    rgb = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
    packed = (
        (rgb[:, :, 0].astype(np.uint32) << 16)
        | (rgb[:, :, 1].astype(np.uint32) << 8)
        | rgb[:, :, 2].astype(np.uint32)
    )
    mask = np.zeros(rgb.shape[:2], dtype=np.uint8)
    for color, class_id in PACKED_TARGET_COLORS.items():
        mask[packed == color] = class_id
    return mask


def main():
    args = parse_args()
    source_root = args.source_root.resolve()
    output_root = args.output_root.resolve()
    scene_split = read_scene_split(source_root)
    mask_root = output_root / "masks"
    mask_root.mkdir(parents=True, exist_ok=True)

    rows = {name: [] for name in ("train", "val", "test")}
    pixel_counts = {name: Counter() for name in rows}
    image_presence = {name: Counter() for name in rows}
    seen_frames = Counter()

    for camera in CAMERAS:
        camera_root = source_root / camera
        if not camera_root.is_dir():
            raise FileNotFoundError(camera_root)
        output_camera = mask_root / camera
        output_camera.mkdir(parents=True, exist_ok=True)
        semantic_paths = sorted(camera_root.glob("*_Semantic.png"))
        if not semantic_paths:
            raise RuntimeError(f"no semantic images under {camera_root}")

        for index, semantic_path in enumerate(semantic_paths, start=1):
            stem = semantic_path.name[: -len("_Semantic.png")]
            intensity_path = camera_root / f"{stem}_Intensity.png"
            if not intensity_path.is_file():
                raise FileNotFoundError(intensity_path)
            scene = frame_scene(stem)
            split = scene_split.get(scene)
            if split is None:
                raise RuntimeError(f"frame scene is missing from frozen split: {scene}")
            mask_path = output_camera / f"{stem}_4class.png"
            if args.overwrite or not mask_path.is_file():
                mask = semantic_to_mask(semantic_path)
                Image.fromarray(mask, mode="L").save(mask_path, compress_level=1)
            else:
                mask = np.asarray(Image.open(mask_path).convert("L"))

            counts = np.bincount(mask.reshape(-1), minlength=4)
            present = np.flatnonzero(counts)
            for class_id in range(4):
                pixel_counts[split][class_id] += int(counts[class_id])
            for class_id in present:
                image_presence[split][int(class_id)] += 1
            seen_frames[(split, scene, stem)] += 1
            rows[split].append(
                {
                    "scene_key": scene,
                    "sample_key": stem,
                    "camera": camera,
                    "image": str(intensity_path),
                    "mask": str(mask_path),
                    "background_pixels": int(counts[0]),
                    "pedestrian_pixels": int(counts[1]),
                    "vehicle_pixels": int(counts[2]),
                    "obstacle_pixels": int(counts[3]),
                }
            )
            if index % 1000 == 0:
                print(f"{camera}: {index}/{len(semantic_paths)}")

    fieldnames = [
        "scene_key", "sample_key", "camera", "image", "mask",
        "background_pixels", "pedestrian_pixels", "vehicle_pixels",
        "obstacle_pixels",
    ]
    output_root.mkdir(parents=True, exist_ok=True)
    for split, split_rows in rows.items():
        path = output_root / f"{split}.csv"
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(split_rows)

    split_scenes = {
        split: sorted({row["scene_key"] for row in split_rows})
        for split, split_rows in rows.items()
    }
    if set(split_scenes["train"]) & set(split_scenes["val"]):
        raise RuntimeError("train/val scene leakage")
    if set(split_scenes["train"]) & set(split_scenes["test"]):
        raise RuntimeError("train/test scene leakage")
    if set(split_scenes["val"]) & set(split_scenes["test"]):
        raise RuntimeError("val/test scene leakage")

    report = {
        "schema": "morai-base2-lraspp-4class-v1",
        "source_root": str(source_root),
        "output_root": str(output_root),
        "class_order": ["background", "pedestrian", "vehicle", "obstacle"],
        "semantic_rgb_to_class": {
            ",".join(map(str, color)): class_id
            for color, class_id in TARGET_COLORS.items()
        },
        "splits": {
            split: {
                "scenes": len(split_scenes[split]),
                "images": len(rows[split]),
                "pixels": {
                    str(class_id): pixel_counts[split][class_id]
                    for class_id in range(4)
                },
                "positive_images": {
                    str(class_id): image_presence[split][class_id]
                    for class_id in range(1, 4)
                },
            }
            for split in rows
        },
        "source_images_copied": False,
    }
    (output_root / "audit.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
