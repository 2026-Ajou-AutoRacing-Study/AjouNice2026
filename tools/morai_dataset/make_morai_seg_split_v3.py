#!/usr/bin/env python3
"""
Scene-aware, class-balanced 70/15/15 split for MORAI 4-class segmentation.

Paper/reproducibility rules
---------------------------
1. Frozen dataset is read-only. No source image/mask is modified, copied,
   renamed, or moved.
2. CAMERA_1~4 with the same full sample key always stay together.
3. All consecutive frame keys belonging to the same scene prefix
   (everything before "_fXX_") always stay in the same split.
4. Scene-level candidate partitions are searched deterministically.
   The selected split minimizes deviation from 70/15/15 for:
     - number of samples
     - Pedestrian pixels
     - Vehicle pixels
     - Obstacle pixels
     - images containing each foreground class
5. Test labels are used only for stratified split construction, never for
   model/hyperparameter selection after the split is frozen.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import re
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

CAMERAS_DEFAULT = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")
VALID_IDS = (0, 1, 2, 3)
FOREGROUND_IDS = (1, 2, 3)
ID_TO_NAME = {
    0: "background",
    1: "pedestrian",
    2: "vehicle",
    3: "obstacle",
}
SCENE_RE = re.compile(r"^(.*)_f\d+_", re.IGNORECASE)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", required=True, type=Path)
    p.add_argument("--output-root", required=True, type=Path)
    p.add_argument("--train-ratio", type=float, default=0.70)
    p.add_argument("--val-ratio", type=float, default=0.15)
    p.add_argument("--test-ratio", type=float, default=0.15)
    p.add_argument("--cameras", nargs="+", default=list(CAMERAS_DEFAULT))
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--candidates", type=int, default=20000,
                   help="Number of deterministic random scene partitions to score.")
    return p.parse_args()


def remove_suffix(name: str, suffix: str) -> str:
    if not name.endswith(suffix):
        raise ValueError(f"Unexpected filename: {name}")
    return name[:-len(suffix)]


def scene_key_from_sample(sample_key: str) -> str:
    m = SCENE_RE.search(sample_key)
    if not m:
        raise ValueError(
            f"Could not derive scene key from '{sample_key}'. "
            "Expected a token like _f00_."
        )
    return m.group(1)


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
            raise RuntimeError(f"{camera}: duplicate image key: {key}")
        images[key] = p

    masks = {}
    for p in sorted(mask_dir.glob("*_4class.png")):
        key = remove_suffix(p.name, "_4class.png")
        if key in masks:
            raise RuntimeError(f"{camera}: duplicate mask key: {key}")
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
        key: {"image": images[key], "mask": masks[key]}
        for key in sorted(images)
    }


def build_scene_stats(dataset_root: Path, cameras, camera_data, sample_keys):
    scenes = {}

    for sample_key in sample_keys:
        scene_key = scene_key_from_sample(sample_key)

        if scene_key not in scenes:
            scenes[scene_key] = {
                "sample_keys": [],
                "num_samples": 0,
                "num_images": 0,
                "pixel_counts": Counter(),
                "image_presence": Counter(),
            }

        scene = scenes[scene_key]
        scene["sample_keys"].append(sample_key)
        scene["num_samples"] += 1

        for camera in cameras:
            mask_path = camera_data[camera][sample_key]["mask"]
            arr = np.array(Image.open(mask_path))
            ids, counts = np.unique(arr, return_counts=True)

            present = set()
            for cid, count in zip(ids, counts):
                cid = int(cid)
                if cid not in VALID_IDS:
                    raise RuntimeError(
                        f"Invalid class id {cid} in {mask_path}"
                    )
                scene["pixel_counts"][cid] += int(count)
                present.add(cid)

            for cid in present:
                scene["image_presence"][cid] += 1

            scene["num_images"] += 1

    for scene in scenes.values():
        scene["sample_keys"].sort()

    return scenes


def aggregate_scene_set(scene_names, scenes):
    out = {
        "num_scenes": len(scene_names),
        "num_samples": 0,
        "num_images": 0,
        "pixel_counts": Counter(),
        "image_presence": Counter(),
    }
    for name in scene_names:
        s = scenes[name]
        out["num_samples"] += s["num_samples"]
        out["num_images"] += s["num_images"]
        out["pixel_counts"].update(s["pixel_counts"])
        out["image_presence"].update(s["image_presence"])
    return out


def partition_score(parts, scenes, ratios):
    split_names = ("train", "val", "test")
    aggs = {name: aggregate_scene_set(parts[name], scenes) for name in split_names}

    totals = aggregate_scene_set(list(scenes.keys()), scenes)
    score = 0.0

    # Sample-count balance.
    total_samples = totals["num_samples"]
    for name, target in zip(split_names, ratios):
        actual = aggs[name]["num_samples"] / total_samples
        score += 2.0 * ((actual - target) / target) ** 2

    # Foreground pixel balance.
    for cid in FOREGROUND_IDS:
        total = totals["pixel_counts"][cid]
        if total <= 0:
            continue
        for name, target in zip(split_names, ratios):
            actual = aggs[name]["pixel_counts"][cid] / total
            score += 4.0 * ((actual - target) / target) ** 2

    # Foreground image-presence balance.
    for cid in FOREGROUND_IDS:
        total = totals["image_presence"][cid]
        if total <= 0:
            continue
        for name, target in zip(split_names, ratios):
            actual = aggs[name]["image_presence"][cid] / total
            score += 1.0 * ((actual - target) / target) ** 2

    # Reject candidates missing any foreground class in any split.
    for name in split_names:
        for cid in FOREGROUND_IDS:
            if aggs[name]["pixel_counts"][cid] <= 0:
                return float("inf"), aggs

    return score, aggs


def search_partition(scene_names, scenes, ratios, seed, candidates):
    rng = random.Random(seed)
    n = len(scene_names)

    n_train = round(n * ratios[0])
    n_val = round(n * ratios[1])
    n_test = n - n_train - n_val

    best = None
    best_score = float("inf")
    best_aggs = None

    base = list(scene_names)

    for _ in range(candidates):
        shuffled = base[:]
        rng.shuffle(shuffled)

        parts = {
            "train": sorted(shuffled[:n_train]),
            "val": sorted(shuffled[n_train:n_train + n_val]),
            "test": sorted(shuffled[n_train + n_val:]),
        }

        score, aggs = partition_score(parts, scenes, ratios)
        if score < best_score:
            best_score = score
            best = parts
            best_aggs = aggs

    if best is None:
        raise RuntimeError("Could not find a valid scene-level split.")

    return best, best_score, best_aggs


def save_manifest(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scene_key", "sample_key", "camera", "image", "mask"
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    with path.with_suffix(".txt").open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(f'{r["image"]} {r["mask"]}\n')


def main():
    args = parse_args()
    dataset_root = args.dataset_root.resolve()
    output_root = args.output_root.resolve()

    ratios = (args.train_ratio, args.val_ratio, args.test_ratio)
    if any(r <= 0 for r in ratios) or abs(sum(ratios) - 1.0) > 1e-9:
        raise SystemExit("Ratios must be > 0 and sum to 1.0.")

    camera_data = {}
    for camera in args.cameras:
        camera_data[camera] = collect_camera(dataset_root, camera)
        print(f"{camera}: {len(camera_data[camera])} samples")

    key_sets = [set(camera_data[c]) for c in args.cameras]
    reference = key_sets[0]
    if any(keys != reference for keys in key_sets[1:]):
        raise SystemExit(
            "STOPPED: CAMERA_1~4 do not contain exactly the same sample keys."
        )

    sample_keys = sorted(reference)
    scenes = build_scene_stats(
        dataset_root, args.cameras, camera_data, sample_keys
    )
    scene_names = sorted(scenes)

    print(f"scene groups: {len(scene_names)}")
    print(f"searching {args.candidates} candidate partitions...")

    parts, best_score, aggs = search_partition(
        scene_names, scenes, ratios, args.seed, args.candidates
    )

    output_root.mkdir(parents=True, exist_ok=True)

    stats = {
        "dataset_root": str(dataset_root),
        "split_version": "scene_balanced_v1",
        "split_method": (
            "scene-level stratified candidate search; scene key is filename "
            "prefix before _fXX_; all frames and CAMERA_1~4 for a scene stay together"
        ),
        "seed": args.seed,
        "candidate_partitions_evaluated": args.candidates,
        "selection_score": best_score,
        "requested_ratios": {
            "train": ratios[0],
            "val": ratios[1],
            "test": ratios[2],
        },
        "target_classes": {
            "0": "background",
            "1": "pedestrian",
            "2": "vehicle",
            "3": "obstacle",
        },
        "total_scenes": len(scene_names),
        "total_samples": len(sample_keys),
        "total_images": len(sample_keys) * len(args.cameras),
        "splits": {},
    }

    for split_name in ("train", "val", "test"):
        rows = []
        for scene_key in parts[split_name]:
            for sample_key in scenes[scene_key]["sample_keys"]:
                for camera in args.cameras:
                    item = camera_data[camera][sample_key]
                    rows.append({
                        "scene_key": scene_key,
                        "sample_key": sample_key,
                        "camera": camera,
                        "image": str(item["image"].relative_to(dataset_root)),
                        "mask": str(item["mask"].relative_to(dataset_root)),
                    })

        save_manifest(output_root / f"{split_name}.csv", rows)

        agg = aggs[split_name]
        total_pixels = sum(agg["pixel_counts"].values())

        stats["splits"][split_name] = {
            "scenes": len(parts[split_name]),
            "samples": agg["num_samples"],
            "images": agg["num_images"],
            "actual_ratio_by_sample": agg["num_samples"] / len(sample_keys),
            "scene_keys": parts[split_name],
            "pixel_counts": {
                ID_TO_NAME[i]: int(agg["pixel_counts"][i])
                for i in VALID_IDS
            },
            "pixel_percent_within_split": {
                ID_TO_NAME[i]: (
                    100.0 * agg["pixel_counts"][i] / total_pixels
                    if total_pixels else 0.0
                )
                for i in VALID_IDS
            },
            "images_containing_class": {
                ID_TO_NAME[i]: int(agg["image_presence"][i])
                for i in VALID_IDS
            },
        }

    (output_root / "split_stats.json").write_text(
        json.dumps(stats, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    readme = [
        "MORAI semantic split - scene balanced v1",
        "",
        "Frozen dataset is read-only.",
        "All frames from one scene and all four cameras stay in one split.",
        "Split selected by deterministic class-balanced candidate search.",
        f"Seed: {args.seed}",
        f"Candidates: {args.candidates}",
        f"Score: {best_score}",
        "",
    ]
    (output_root / "README_split.txt").write_text(
        "\n".join(readme), encoding="utf-8"
    )

    print("\nDone.")
    print(f"  scenes           : {len(scene_names)}")
    print(f"  samples          : {len(sample_keys)}")
    for name in ("train", "val", "test"):
        x = stats["splits"][name]
        print(
            f"  {name:10s}: {x['scenes']} scenes / "
            f"{x['samples']} sample groups / {x['images']} images"
        )
    print(f"  output           : {output_root}")
    print("\nInspect split_stats.json before freezing this split.")


if __name__ == "__main__":
    main()
