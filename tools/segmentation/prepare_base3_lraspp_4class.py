#!/usr/bin/env python3
"""Build the exact Base3 LRASPP 4-class masks and split manifests.

Only scenes marked completed by each selected capture run are eligible.  The
selection file also removes superseded captures explicitly, so a recursive
glob can never leak failed/rejected Base3 frames into training.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

from prepare_base2_lraspp_4class import CAMERAS, TARGET_COLORS, semantic_to_mask


SPLITS = ("train", "val", "test")
FIELDNAMES = [
    "scene_key", "sample_key", "capture_run", "mission_family", "camera",
    "frame_index", "image", "mask", "background_pixels",
    "pedestrian_pixels", "vehicle_pixels", "obstacle_pixels",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def selected_clips(capture_root: Path, selection):
    clips = []
    seen = set()
    for run_spec in selection["runs"]:
        run_name = run_spec["name"]
        run_root = capture_root / run_name
        summary_path = run_root / "summary.json"
        if not summary_path.is_file():
            raise FileNotFoundError(summary_path)
        summary = load_json(summary_path)
        excluded = set(run_spec.get("exclude_scene_ids", []))
        completed = summary.get("completed_scene_ids", [])
        missing_exclusions = excluded - set(completed)
        if missing_exclusions:
            raise RuntimeError(
                f"{run_name}: exclusions are not completed scenes: "
                f"{sorted(missing_exclusions)}"
            )
        for scene_id in completed:
            if scene_id in excluded:
                continue
            if scene_id in seen:
                raise RuntimeError(f"duplicate selected scene: {scene_id}")
            seen.add(scene_id)
            clip_path = run_root / "dynamic_metadata" / f"{scene_id}.clip.json"
            if not clip_path.is_file():
                raise FileNotFoundError(clip_path)
            clip = load_json(clip_path)
            split = clip.get("split")
            if split not in SPLITS:
                raise RuntimeError(f"invalid split {split!r}: {clip_path}")
            if int(clip.get("frame_count", -1)) != 3:
                raise RuntimeError(f"Base3 clip must contain three frames: {clip_path}")
            frames_path = run_root / clip.get(
                "frames_jsonl", f"dynamic_metadata/{scene_id}.frames.jsonl"
            )
            if not frames_path.is_file():
                raise FileNotFoundError(frames_path)
            clips.append({
                "run_name": run_name,
                "run_root": run_root,
                "scene_id": scene_id,
                "split": split,
                "mission_family": clip.get("mission_family", "unknown"),
                "frames_path": frames_path,
            })
    return clips


def scene_camera_files(clip, camera):
    semantic_paths = []
    records = [
        json.loads(line) for line in clip["frames_path"].read_text(
            encoding="utf-8"
        ).splitlines() if line.strip()
    ]
    if len(records) != 3:
        raise RuntimeError(
            f"expected three frame records for {clip['scene_id']}, "
            f"found {len(records)}"
        )
    for expected_index, record in enumerate(records):
        if not record.get("verified", False):
            raise RuntimeError(
                f"selected frame is not verified: {clip['scene_id']} "
                f"frame {expected_index}"
            )
        if int(record.get("frame_index", -1)) != expected_index:
            raise RuntimeError(
                f"non-contiguous frame index: {clip['scene_id']} {record}"
            )
        candidates = [
            relative for relative in record.get("new_files", [])
            if relative.startswith(f"{camera}/")
            and relative.endswith("_Semantic.png")
        ]
        if len(candidates) != 1:
            raise RuntimeError(
                f"expected one recorded {camera} semantic artifact for "
                f"{clip['scene_id']} frame {expected_index}, found {candidates}"
            )
        semantic_path = clip["run_root"] / candidates[0]
        if not semantic_path.is_file():
            raise FileNotFoundError(semantic_path)
        semantic_paths.append(semantic_path)
    return semantic_paths


def build_dataset(capture_root: Path, selection, output_root: Path, overwrite=False):
    clips = selected_clips(capture_root, selection)
    rows = {split: [] for split in SPLITS}
    pixel_counts = {split: Counter() for split in SPLITS}
    positive_images = {split: Counter() for split in SPLITS}
    family_clips = {split: Counter() for split in SPLITS}
    selected_scenes = {split: set() for split in SPLITS}

    for clip_index, clip in enumerate(clips, start=1):
        split = clip["split"]
        scene_id = clip["scene_id"]
        selected_scenes[split].add(scene_id)
        family_clips[split][clip["mission_family"]] += 1
        for camera in CAMERAS:
            mask_dir = output_root / "masks" / clip["run_name"] / camera
            mask_dir.mkdir(parents=True, exist_ok=True)
            for semantic_path in scene_camera_files(clip, camera):
                stem = semantic_path.name[:-len("_Semantic.png")]
                intensity_path = semantic_path.with_name(f"{stem}_Intensity.png")
                if not intensity_path.is_file():
                    raise FileNotFoundError(intensity_path)
                mask_path = mask_dir / f"{stem}_4class.png"
                if overwrite or not mask_path.is_file():
                    mask = semantic_to_mask(semantic_path)
                    Image.fromarray(mask, mode="L").save(mask_path, compress_level=1)
                else:
                    mask = np.asarray(Image.open(mask_path).convert("L"))
                if Image.open(intensity_path).size != (mask.shape[1], mask.shape[0]):
                    raise RuntimeError(
                        f"image/mask dimension mismatch: {intensity_path} {mask_path}"
                    )
                counts = np.bincount(mask.reshape(-1), minlength=4)
                for class_id in range(4):
                    pixel_counts[split][class_id] += int(counts[class_id])
                for class_id in np.flatnonzero(counts):
                    positive_images[split][int(class_id)] += 1
                frame_index = int(stem.rsplit("_f", 1)[1])
                rows[split].append({
                    "scene_key": scene_id,
                    "sample_key": stem,
                    "capture_run": clip["run_name"],
                    "mission_family": clip["mission_family"],
                    "camera": camera,
                    "frame_index": frame_index,
                    "image": str(intensity_path),
                    "mask": str(mask_path.resolve()),
                    "background_pixels": int(counts[0]),
                    "pedestrian_pixels": int(counts[1]),
                    "vehicle_pixels": int(counts[2]),
                    "obstacle_pixels": int(counts[3]),
                })
        if clip_index % 100 == 0 or clip_index == len(clips):
            print(f"clips: {clip_index}/{len(clips)}", flush=True)

    for split in SPLITS:
        rows[split].sort(key=lambda row: (
            row["scene_key"], row["frame_index"], row["camera"]
        ))
        csv_path = output_root / f"{split}.csv"
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(rows[split])
        (output_root / f"{split}_scenes.txt").write_text(
            "\n".join(sorted(selected_scenes[split])) + "\n", encoding="utf-8"
        )

    if selected_scenes["train"] & selected_scenes["val"]:
        raise RuntimeError("train/val scene leakage")
    if selected_scenes["train"] & selected_scenes["test"]:
        raise RuntimeError("train/test scene leakage")
    if selected_scenes["val"] & selected_scenes["test"]:
        raise RuntimeError("val/test scene leakage")

    expected = selection["expected"]
    actual_clips = len(clips)
    actual_frames = actual_clips * 3
    actual_samples = sum(len(value) for value in rows.values())
    actual_split_clips = {
        split: len(selected_scenes[split]) for split in SPLITS
    }
    checks = {
        "clips": actual_clips == int(expected["clips"]),
        "frames": actual_frames == int(expected["frames"]),
        "camera_samples": actual_samples == int(expected["camera_samples"]),
        "split_clips": actual_split_clips == expected["split_clips"],
    }
    if not all(checks.values()):
        raise RuntimeError(f"selection contract failed: {checks}")

    report = {
        "schema": "morai-base3-lraspp-4class-v1",
        "dataset_name": selection["dataset_name"],
        "selection_id": selection["selection_id"],
        "capture_root": str(capture_root),
        "output_root": str(output_root),
        "class_order": ["background", "pedestrian", "vehicle", "obstacle"],
        "semantic_rgb_to_class": {
            ",".join(map(str, color)): class_id
            for color, class_id in TARGET_COLORS.items()
        },
        "totals": {
            "clips": actual_clips,
            "frames": actual_frames,
            "camera_samples": actual_samples,
        },
        "contract_checks": checks,
        "splits": {
            split: {
                "clips": len(selected_scenes[split]),
                "images": len(rows[split]),
                "families": dict(sorted(family_clips[split].items())),
                "pixels": {
                    str(class_id): pixel_counts[split][class_id]
                    for class_id in range(4)
                },
                "positive_images": {
                    str(class_id): positive_images[split][class_id]
                    for class_id in range(1, 4)
                },
                "empty_images": sum(
                    1 for row in rows[split]
                    if all(int(row[name]) == 0 for name in (
                        "pedestrian_pixels", "vehicle_pixels", "obstacle_pixels"
                    ))
                ),
            }
            for split in SPLITS
        },
        "source_images_copied": False,
        "depth_and_instance_required": False,
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "audit.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return report


def main():
    args = parse_args()
    capture_root = args.capture_root.resolve()
    selection_path = args.selection.resolve()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    report = build_dataset(
        capture_root,
        load_json(selection_path),
        output_root,
        overwrite=args.overwrite,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
