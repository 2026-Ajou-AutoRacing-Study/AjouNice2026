#!/usr/bin/env python3
"""Combine static and dynamic painted PointPainting datasets clip-safely."""

import argparse
import json
import os
import shutil
from collections import Counter, defaultdict
from pathlib import Path


TARGET_SPLITS = {"train": 2100, "val": 450, "test": 450}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-root", type=Path, required=True)
    parser.add_argument("--dynamic-root", type=Path, required=True)
    parser.add_argument("--dynamic-metadata-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    return parser.parse_args()


def read_ids(root, split):
    return [
        row.strip()
        for row in (root / "ImageSets" / f"{split}.txt").read_text(
            encoding="utf-8"
        ).splitlines()
        if row.strip()
    ]


def read_jsonl(path):
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def link_or_copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        target.unlink()
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def validate_payload(root, frame_ids):
    missing = []
    for frame_id in frame_ids:
        for directory, suffix in (("points", ".npy"), ("labels", ".txt")):
            path = root / directory / f"{frame_id}{suffix}"
            if not path.is_file():
                missing.append(str(path))
    if missing:
        raise RuntimeError(f"missing painted payloads: {missing[:5]}")


def main():
    args = parse_args()
    static_root = args.static_root.resolve()
    dynamic_root = args.dynamic_root.resolve()
    metadata_root = args.dynamic_metadata_root.resolve()
    output = args.output_root.resolve()

    sources = {
        "static": {
            split: read_ids(static_root, split)
            for split in TARGET_SPLITS
        },
        "dynamic": {
            split: read_ids(dynamic_root, split)
            for split in TARGET_SPLITS
        },
    }
    static_ids = set(sum(sources["static"].values(), []))
    dynamic_ids = set(sum(sources["dynamic"].values(), []))
    if static_ids & dynamic_ids:
        raise RuntimeError("static/dynamic frame ids overlap")
    if len(static_ids) != 1500 or len(dynamic_ids) != 1500:
        raise RuntimeError(
            f"source frame counts are not 1500/1500: "
            f"{len(static_ids)}/{len(dynamic_ids)}"
        )

    dynamic_rows = read_jsonl(metadata_root / "metadata/frames.jsonl")
    dynamic_by_frame = {row["frame_id"]: row for row in dynamic_rows}
    val_scenes = defaultdict(list)
    for frame_id in sources["dynamic"]["val"]:
        val_scenes[dynamic_by_frame[frame_id]["scene_id"]].append(frame_id)
    candidates = sorted(
        (scene_id, ids)
        for scene_id, ids in val_scenes.items()
        if len(ids) == 3
    )
    if not candidates:
        raise RuntimeError("no 3-frame dynamic val clip available for rebalance")
    moved_scene, moved_ids = candidates[0]
    moved_set = set(moved_ids)
    sources["dynamic"]["val"] = [
        item for item in sources["dynamic"]["val"] if item not in moved_set
    ]
    sources["dynamic"]["test"].extend(sorted(moved_ids))

    combined = {}
    split_by_frame = {}
    source_by_frame = {}
    for split in TARGET_SPLITS:
        ids = sorted(sources["static"][split] + sources["dynamic"][split])
        combined[split] = ids
        if len(ids) != TARGET_SPLITS[split]:
            raise RuntimeError(
                f"{split}: {len(ids)} != {TARGET_SPLITS[split]}"
            )
        for frame_id in ids:
            split_by_frame[frame_id] = split
            source_by_frame[frame_id] = (
                "static1500" if frame_id in static_ids else "dynamic1500"
            )

    validate_payload(static_root, static_ids)
    validate_payload(dynamic_root, dynamic_ids)
    for frame_id in sorted(static_ids | dynamic_ids):
        root = static_root if frame_id in static_ids else dynamic_root
        link_or_copy(root / "points" / f"{frame_id}.npy", output / "points" / f"{frame_id}.npy")
        link_or_copy(root / "labels" / f"{frame_id}.txt", output / "labels" / f"{frame_id}.txt")

    for split, ids in combined.items():
        path = output / "ImageSets" / f"{split}.txt"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(f"{item}\n" for item in ids), encoding="utf-8")

    combined_rows = []
    for frame_id in sorted(static_ids | dynamic_ids):
        row = {
            "frame_id": frame_id,
            "split": split_by_frame[frame_id],
            "source": source_by_frame[frame_id],
        }
        if frame_id in dynamic_by_frame:
            dynamic = dynamic_by_frame[frame_id]
            row.update(
                {
                    "scene_id": dynamic["scene_id"],
                    "frame_index": dynamic["frame_index"],
                    "role": dynamic["role"],
                    "purpose": dynamic["purpose"],
                    "triggered_unix": dynamic.get("triggered_unix"),
                    "ego": dynamic.get("ego"),
                    "objects": dynamic.get("objects"),
                    "quality": dynamic.get("quality"),
                }
            )
        combined_rows.append(row)
    metadata_path = output / "metadata/frames.jsonl"
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    with metadata_path.open("w", encoding="utf-8") as stream:
        for row in combined_rows:
            stream.write(
                json.dumps(row, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )

    class_counts = Counter()
    for frame_id in sorted(static_ids | dynamic_ids):
        root = static_root if frame_id in static_ids else dynamic_root
        for line in (root / "labels" / f"{frame_id}.txt").read_text(
            encoding="utf-8"
        ).splitlines():
            if line.strip():
                class_counts[line.split()[-1]] += 1

    dataset = {
        "schema_version": 1,
        "dataset_id": "morai_pointpainting_sunny13_static1500_dynamic1500_v1",
        "frames": 3000,
        "split_frames": TARGET_SPLITS,
        "source_frames": {"static1500": 1500, "dynamic1500": 1500},
        "class_instances": dict(class_counts),
        "point_features": [
            "x", "y", "z", "intensity", "background", "pedestrian",
            "vehicle", "obstacle", "unknown",
        ],
        "preprocess_backend": "opencv",
        "short_side": 384,
        "split_unit": "scene_or_temporal_clip",
        "rebalance": {
            "scene_id": moved_scene,
            "frames": sorted(moved_ids),
            "from": "val",
            "to": "test",
        },
        "velocity_contract": {
            "current_box_labels": "7D only; velocity is not yet a detector target",
            "dynamic_metadata": "ego/object velocity and acceleration retained when available",
            "static_velocity": "scenario-defined zero; not materialized in current 7D labels",
        },
    }
    write_json(output / "metadata/dataset.json", dataset)
    (output / "README.md").write_text(
        "# MORAI PointPainting Sunny13 static1500 + dynamic1500\n\n"
        "- 3,000 painted LiDAR frames with 9 point features.\n"
        "- Train/Val/Test: 2,100/450/450 frames.\n"
        "- Split unit is a static scene or dynamic temporal clip.\n"
        "- `points/` and `labels/` are directly consumable by OpenPCDet.\n"
        "- Current labels are 7D boxes; dynamic velocity metadata is preserved "
        "under `metadata/frames.jsonl` for a future velocity head.\n",
        encoding="utf-8",
    )
    print(f"dataset: {output}")
    print(f"splits: {TARGET_SPLITS}")
    print(f"class instances: {dict(class_counts)}")
    print(f"val->test clip: {moved_scene} ({len(moved_ids)} frames)")


if __name__ == "__main__":
    main()
