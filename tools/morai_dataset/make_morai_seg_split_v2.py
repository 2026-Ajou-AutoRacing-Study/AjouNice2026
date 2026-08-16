#!/usr/bin/env python3
"""
Create reproducible 70/15/15 Train/Validation/Test manifests for
the frozen MORAI 4-class semantic dataset.

Rules
-----
- Source images/masks are NEVER modified, copied, renamed, or moved.
- CAMERA_1~4 samples sharing the same FULL sample key stay in the same split.
- The full sample key is used as group/timestep ID because MORAI f00/f01...
  frame tokens can repeat across capture segments.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

CAMERAS_DEFAULT = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")
VALID_IDS = (0, 1, 2, 3)
ID_TO_NAME = {
    0: "background",
    1: "pedestrian",
    2: "vehicle",
    3: "obstacle",
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", required=True, type=Path)
    p.add_argument("--output-root", required=True, type=Path)
    p.add_argument("--train-ratio", type=float, default=0.70)
    p.add_argument("--val-ratio", type=float, default=0.15)
    p.add_argument("--test-ratio", type=float, default=0.15)
    p.add_argument("--cameras", nargs="+", default=list(CAMERAS_DEFAULT))
    return p.parse_args()


def remove_suffix(name: str, suffix: str) -> str:
    if not name.endswith(suffix):
        raise ValueError(f"Unexpected filename: {name}")
    return name[:-len(suffix)]


def collect_camera(dataset_root: Path, camera: str):
    image_dir = dataset_root / "images" / camera
    mask_dir = dataset_root / "masks" / camera

    if not image_dir.is_dir():
        raise FileNotFoundError(f"Missing images directory: {image_dir}")
    if not mask_dir.is_dir():
        raise FileNotFoundError(f"Missing masks directory: {mask_dir}")

    images = {}
    for p in sorted(image_dir.glob("*_Intensity.png")):
        key = remove_suffix(p.name, "_Intensity.png")
        if key in images:
            raise RuntimeError(f"{camera}: duplicate FULL image key: {key}")
        images[key] = p

    masks = {}
    for p in sorted(mask_dir.glob("*_4class.png")):
        key = remove_suffix(p.name, "_4class.png")
        if key in masks:
            raise RuntimeError(f"{camera}: duplicate FULL mask key: {key}")
        masks[key] = p

    if set(images) != set(masks):
        only_images = sorted(set(images) - set(masks))
        only_masks = sorted(set(masks) - set(images))
        raise RuntimeError(
            f"{camera}: image/mask mismatch\n"
            f"  images without masks: {only_images[:10]}\n"
            f"  masks without images: {only_masks[:10]}"
        )

    return {
        key: {
            "sample_key": key,
            "image": images[key],
            "mask": masks[key],
        }
        for key in sorted(images)
    }


def save_manifest(csv_path: Path, rows):
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    fields = ["group_index", "sample_key", "camera", "image", "mask"]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    txt_path = csv_path.with_suffix(".txt")
    with txt_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(f'{row["image"]} {row["mask"]}\n')


def analyze_split(dataset_root: Path, rows):
    pixel_counts = Counter()
    images_containing_class = Counter()
    invalid_masks = []

    for row in rows:
        arr = np.array(Image.open(dataset_root / row["mask"]))
        ids, counts = np.unique(arr, return_counts=True)

        present = set()
        for cid, count in zip(ids, counts):
            cid = int(cid)
            if cid not in VALID_IDS:
                invalid_masks.append({
                    "mask": row["mask"],
                    "invalid_id": cid,
                })
                continue

            pixel_counts[cid] += int(count)
            present.add(cid)

        for cid in present:
            images_containing_class[cid] += 1

    return {
        "pixel_counts": {
            ID_TO_NAME[i]: int(pixel_counts[i]) for i in VALID_IDS
        },
        "images_containing_class": {
            ID_TO_NAME[i]: int(images_containing_class[i]) for i in VALID_IDS
        },
        "invalid_masks": invalid_masks,
    }


def main():
    args = parse_args()

    dataset_root = args.dataset_root.resolve()
    output_root = args.output_root.resolve()

    ratios = [args.train_ratio, args.val_ratio, args.test_ratio]
    if any(r <= 0 for r in ratios):
        raise SystemExit("All split ratios must be > 0.")
    if abs(sum(ratios) - 1.0) > 1e-9:
        raise SystemExit(
            f"train/val/test ratios must sum to 1.0; got {sum(ratios):.6f}"
        )

    camera_data = {}
    for camera in args.cameras:
        camera_data[camera] = collect_camera(dataset_root, camera)
        print(f"{camera}: {len(camera_data[camera])} samples")

    key_sets = [set(camera_data[c]) for c in args.cameras]
    reference = key_sets[0]

    if any(keys != reference for keys in key_sets[1:]):
        print("\nSample-key mismatch across cameras:")
        for camera in args.cameras:
            keys = set(camera_data[camera])
            print(
                f"  {camera}: count={len(keys)}, "
                f"missing_vs_CAMERA_1={len(reference - keys)}, "
                f"extra_vs_CAMERA_1={len(keys - reference)}"
            )
        raise SystemExit(
            "STOPPED: CAMERA_1~4 do not contain exactly the same FULL sample keys."
        )

    # Critical fix:
    # Do NOT extract f00/f01 as frame IDs. MORAI may reuse them.
    # The entire filename stem is the synchronized group key.
    group_keys = sorted(reference)

    n_groups = len(group_keys)
    if n_groups == 0:
        raise SystemExit("No synchronized groups found.")

    n_train = round(n_groups * args.train_ratio)
    n_val = round(n_groups * args.val_ratio)
    n_test = n_groups - n_train - n_val

    if min(n_train, n_val, n_test) <= 0:
        raise SystemExit(
            f"Dataset too small: train={n_train}, val={n_val}, test={n_test}"
        )

    split_groups = {
        "train": group_keys[:n_train],
        "val": group_keys[n_train:n_train + n_val],
        "test": group_keys[n_train + n_val:],
    }

    output_root.mkdir(parents=True, exist_ok=True)

    stats = {
        "dataset_root": str(dataset_root),
        "split_method": (
            "deterministic contiguous split using full MORAI sample key; "
            "same-key CAMERA_1~4 samples remain together"
        ),
        "group_identifier": "full filename stem, not fXX token",
        "target_classes": {
            "0": "background",
            "1": "pedestrian",
            "2": "vehicle",
            "3": "obstacle",
        },
        "cameras": list(args.cameras),
        "total_groups": n_groups,
        "total_images": n_groups * len(args.cameras),
        "requested_ratios": {
            "train": args.train_ratio,
            "val": args.val_ratio,
            "test": args.test_ratio,
        },
        "splits": {},
    }

    for split_name, keys in split_groups.items():
        rows = []

        for group_index, key in enumerate(keys):
            for camera in args.cameras:
                item = camera_data[camera][key]
                rows.append({
                    "group_index": group_index,
                    "sample_key": key,
                    "camera": camera,
                    "image": str(item["image"].relative_to(dataset_root)),
                    "mask": str(item["mask"].relative_to(dataset_root)),
                })

        save_manifest(output_root / f"{split_name}.csv", rows)
        analysis = analyze_split(dataset_root, rows)

        stats["splits"][split_name] = {
            "groups": len(keys),
            "images": len(rows),
            "actual_ratio_by_group": len(keys) / n_groups,
            "first_sample_key": keys[0],
            "last_sample_key": keys[-1],
            **analysis,
        }

    invalid_total = sum(
        len(stats["splits"][name]["invalid_masks"])
        for name in ("train", "val", "test")
    )

    (output_root / "split_stats.json").write_text(
        json.dumps(stats, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    readme_lines = [
        "MORAI semantic split v1",
        "",
        f"Dataset: {dataset_root}",
        "",
        "Rules:",
        "- source images/masks are never modified, copied, renamed, or moved",
        "- split files are manifests only",
        "- CAMERA_1~4 with the same full sample key stay together",
        "- full sample key is used because fXX tokens can repeat",
        "- deterministic contiguous 70/15/15 split",
        "",
        f"Total: {n_groups} groups / {n_groups * len(args.cameras)} images",
        f"Train: {n_train} groups / {n_train * len(args.cameras)} images",
        f"Validation: {n_val} groups / {n_val * len(args.cameras)} images",
        f"Test: {n_test} groups / {n_test * len(args.cameras)} images",
        "",
    ]

    (output_root / "README_split.txt").write_text(
        "\n".join(readme_lines), encoding="utf-8"
    )

    if invalid_total:
        raise SystemExit(
            f"STOPPED: {invalid_total} invalid mask entries found. "
            "Inspect split_stats.json."
        )

    print("\nDone.")
    print(f"  total groups     : {n_groups}")
    print(f"  train            : {n_train} groups / {n_train * len(args.cameras)} images")
    print(f"  validation       : {n_val} groups / {n_val * len(args.cameras)} images")
    print(f"  test             : {n_test} groups / {n_test * len(args.cameras)} images")
    print(f"  output           : {output_root}")
    print("\nNext: inspect split_stats.json before training.")


if __name__ == "__main__":
    main()
