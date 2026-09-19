#!/usr/bin/env python3
"""Create a small self-contained Base3 subset for team GPU smoke tests."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import pickle
import shutil
from typing import Any, Dict, List


SERIALIZED_ROOT = Path("/datasets/base3")
CLASS_NAMES = ("vehicle", "pedestrian", "obstacle")


def relative_capture_path(value: str) -> Path:
    path = Path(value)
    try:
        return path.relative_to(SERIALIZED_ROOT)
    except ValueError as exc:
        raise ValueError("path is not portable under /datasets/base3: {}".format(path)) from exc


def frame_bucket(frame: Dict[str, Any]) -> str:
    labels = {int(instance["bbox_label_3d"]) for instance in frame.get("instances", [])}
    if not labels:
        return "empty"
    # Prefer a single-class label; class-centric Base3 frames usually satisfy it.
    for label, name in enumerate(CLASS_NAMES):
        if labels == {label}:
            return name
    return CLASS_NAMES[min(labels)]


def select_balanced(frames: List[Dict[str, Any]], count: int) -> List[Dict[str, Any]]:
    buckets = defaultdict(list)
    for frame in frames:
        buckets[frame_bucket(frame)].append(frame)
    order = ("vehicle", "pedestrian", "obstacle", "empty")
    selected = []
    cursor = Counter()
    while len(selected) < min(count, len(frames)):
        progressed = False
        for bucket in order:
            index = cursor[bucket]
            if index < len(buckets[bucket]) and len(selected) < count:
                selected.append(buckets[bucket][index])
                cursor[bucket] += 1
                progressed = True
        if not progressed:
            break
    return selected


def copy_frame_files(
    frame: Dict[str, Any], capture_root: Path, output_capture: Path,
    link_files: bool, symlink_files: bool,
) -> None:
    paths = [frame["lidar_points"]["lidar_path"]]
    paths.extend(camera["img_path"] for camera in frame["images"].values())
    for serialized in paths:
        relative = relative_capture_path(serialized)
        source = capture_root / relative
        destination = output_capture / relative
        if not source.is_file():
            raise FileNotFoundError(source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            continue
        if symlink_files:
            destination.symlink_to(source.resolve())
            continue
        if link_files:
            try:
                os.link(source, destination)
                continue
            except OSError:
                pass
        shutil.copy2(source, destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--info-root", type=Path, required=True)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--samples-per-split", type=int, default=16)
    parser.add_argument(
        "--link-files", action="store_true",
        help="hard-link files when possible (for same-filesystem packaging staging)",
    )
    parser.add_argument(
        "--symlink-files", action="store_true",
        help="stage absolute symlinks; archive with tar --dereference",
    )
    args = parser.parse_args()
    if args.samples_per_split < 0:
        parser.error("--samples-per-split must be nonnegative (0 means all)")
    if args.link_files and args.symlink_files:
        parser.error("--link-files and --symlink-files are mutually exclusive")
    if args.output_root.exists() and any(args.output_root.iterdir()):
        raise SystemExit("refusing to overwrite non-empty output: {}".format(args.output_root))

    output_capture = args.output_root / "capture"
    output_infos = args.output_root / "infos"
    output_capture.mkdir(parents=True, exist_ok=True)
    output_infos.mkdir(parents=True, exist_ok=True)
    audit = {"samples_per_split_requested": args.samples_per_split, "splits": {}}

    for split in ("train", "val", "test"):
        input_path = args.info_root / "morai_base3_infos_{}.pkl".format(split)
        with input_path.open("rb") as handle:
            payload = pickle.load(handle)
        selected = (
            list(payload["data_list"])
            if args.samples_per_split == 0
            else select_balanced(payload["data_list"], args.samples_per_split)
        )
        for frame in selected:
            copy_frame_files(
                frame, args.capture_root, output_capture,
                args.link_files, args.symlink_files,
            )
        output_payload = dict(payload)
        output_payload["data_list"] = selected
        with (output_infos / input_path.name).open("wb") as handle:
            pickle.dump(output_payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
        buckets = Counter(frame_bucket(frame) for frame in selected)
        audit["splits"][split] = {"frames": len(selected), "buckets": dict(sorted(buckets.items()))}

    (args.output_root / "SMOKE_BUNDLE.json").write_text(
        json.dumps(audit, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(audit, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
