#!/usr/bin/env python3
"""Audit whether MORAI Base4 infos can drive causal query-memory training."""

from __future__ import annotations

import argparse
import json
import math
import pickle
from collections import Counter, defaultdict
from pathlib import Path


REQUIRED_KEYS = (
    "sample_idx",
    "scene_id",
    "timestamp",
    "sequence_frame_index",
    "sequence_length",
    "ego2global",
    "lidar2global",
    "instances",
)


def load_rows(path: Path):
    with path.open("rb") as stream:
        payload = pickle.load(stream)
    return payload["data_list"] if isinstance(payload, dict) else payload


def valid_matrix(value):
    return (
        isinstance(value, (list, tuple))
        and len(value) == 4
        and all(isinstance(row, (list, tuple)) and len(row) == 4 for row in value)
        and all(math.isfinite(float(cell)) for row in value for cell in row)
    )


def audit_split(path: Path, reset_gap_sec: float):
    rows = load_rows(path)
    scenes = defaultdict(list)
    issues = Counter()
    ordered_scene_blocks = []
    previous_scene = None
    closed_scenes = set()

    for row_index, row in enumerate(rows):
        missing = [key for key in REQUIRED_KEYS if key not in row]
        if missing:
            issues["rows_missing_required_keys"] += 1
            continue
        scene_id = str(row["scene_id"])
        if scene_id != previous_scene:
            if previous_scene is not None:
                closed_scenes.add(previous_scene)
            if scene_id in closed_scenes:
                issues["interleaved_scene_blocks"] += 1
            ordered_scene_blocks.append(scene_id)
            previous_scene = scene_id
        if not valid_matrix(row["lidar2global"]):
            issues["invalid_lidar2global"] += 1
        if not valid_matrix(row["ego2global"]):
            issues["invalid_ego2global"] += 1
        instances = row.get("instances", [])
        instance_ids = [item.get("instance_id") for item in instances]
        if any(value is None for value in instance_ids):
            issues["instances_missing_id"] += 1
        if len(set(instance_ids)) != len(instance_ids):
            issues["duplicate_instance_ids_in_frame"] += 1
        scenes[scene_id].append((row_index, row))

    causal_edges = 0
    reset_edges = 0
    dt_values = []
    for scene_rows in scenes.values():
        sorted_rows = sorted(
            scene_rows,
            key=lambda item: (int(item[1]["sequence_frame_index"]),
                              float(item[1]["timestamp"])),
        )
        file_indices = [item[0] for item in sorted_rows]
        if file_indices != sorted(file_indices):
            issues["frames_not_ordered_in_file"] += 1
        frame_indices = [int(item[1]["sequence_frame_index"])
                         for item in sorted_rows]
        if len(set(frame_indices)) != len(frame_indices):
            issues["duplicate_frame_indices"] += 1
        for (_, previous), (_, current) in zip(sorted_rows, sorted_rows[1:]):
            dt = float(current["timestamp"]) - float(previous["timestamp"])
            if dt <= 0.0:
                issues["noncausal_timestamp_edges"] += 1
                continue
            dt_values.append(dt)
            if dt > reset_gap_sec:
                reset_edges += 1
            else:
                causal_edges += 1

    fatal_names = {
        "rows_missing_required_keys",
        "interleaved_scene_blocks",
        "invalid_lidar2global",
        "invalid_ego2global",
        "instances_missing_id",
        "duplicate_instance_ids_in_frame",
        "frames_not_ordered_in_file",
        "duplicate_frame_indices",
        "noncausal_timestamp_edges",
    }
    fatal_count = sum(issues[name] for name in fatal_names)
    return {
        "path": str(path),
        "rows": len(rows),
        "scenes": len(scenes),
        "scene_blocks": len(ordered_scene_blocks),
        "causal_edges_at_or_below_reset_gap": causal_edges,
        "edges_requiring_memory_reset": reset_edges,
        "dt_sec": {
            "min": min(dt_values) if dt_values else None,
            "median": sorted(dt_values)[len(dt_values) // 2] if dt_values else None,
            "max": max(dt_values) if dt_values else None,
        },
        "issues": dict(sorted(issues.items())),
        "metadata_contract_pass": fatal_count == 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--info-root", type=Path, required=True)
    parser.add_argument("--splits", nargs="+", default=["train", "val"])
    parser.add_argument("--reset-gap-sec", type=float, default=0.30)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    reports = {}
    missing = []
    for split in args.splits:
        path = args.info_root / f"morai_base3_infos_{split}.pkl"
        if not path.is_file():
            missing.append(str(path))
            continue
        reports[split] = audit_split(path, args.reset_gap_sec)
    result = {
        "info_root": str(args.info_root),
        "reset_gap_sec": args.reset_gap_sec,
        "splits": reports,
        "missing_files": missing,
        "metadata_contract_pass": (
            bool(reports)
            and not missing
            and all(item["metadata_contract_pass"] for item in reports.values())
        ),
        "integration_requirements": {
            "pack_meta_keys": [
                "scene_id", "timestamp", "sequence_frame_index",
                "sequence_length", "ego2global", "lidar2global",
            ],
            "training_sampler": "scene-ordered, shuffle=False, batch_size=1",
            "augmentation": "one shared transform per sequence; P0 disables global augmentation",
        },
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered + "\n")
    raise SystemExit(0 if result["metadata_contract_pass"] else 1)


if __name__ == "__main__":
    main()
