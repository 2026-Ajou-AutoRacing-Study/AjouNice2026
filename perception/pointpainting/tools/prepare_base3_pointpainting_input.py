#!/usr/bin/env python3
"""Build the exact Base3-Static raw index consumed by the 9-feature painter."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
SEGMENTATION_TOOLS = REPO_ROOT / "tools/segmentation"
DATASET_TOOLS = REPO_ROOT / "dataset/morai_od/scripts"
sys.path.insert(0, str(SEGMENTATION_TOOLS))
sys.path.insert(0, str(DATASET_TOOLS))

from prepare_base3_lraspp_4class import SPLITS, load_json, selected_clips  # noqa: E402
from build_fastbev_sunny13_1500_dataset import (  # noqa: E402
    CAMERA_DIRS,
    calibration_from_sensor_config,
    coarse_class,
    parse_lidar_instances,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--sensor-config", type=Path, default=REPO_ROOT / "configs/sensors/Sensor_6.json")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--max-clips", type=int, default=0)
    return parser.parse_args()


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def frame_records(clip) -> list[dict]:
    rows = [
        json.loads(line)
        for line in clip["frames_path"].read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(rows) != 3:
        raise RuntimeError(f"{clip['scene_id']}: expected three frames, found {len(rows)}")
    for index, row in enumerate(rows):
        if not row.get("verified", False) or int(row.get("frame_index", -1)) != index:
            raise RuntimeError(f"{clip['scene_id']}: invalid frame contract at {index}")
    return rows


def artifact(
    record: dict,
    run_root: Path,
    directory: str,
    suffix: str,
    *,
    allow_empty: bool = False,
) -> Path:
    candidates = [
        run_root / relative
        for relative in record.get("new_files", [])
        if relative.startswith(directory + "/") and relative.endswith(suffix)
    ]
    if len(candidates) != 1:
        raise RuntimeError(
            f"{record.get('frame_id')}: expected one {directory}/*{suffix}, found {candidates}"
        )
    if not candidates[0].is_file():
        raise FileNotFoundError(candidates[0])
    if candidates[0].stat().st_size <= 0 and not allow_empty:
        raise FileNotFoundError(candidates[0])
    return candidates[0].resolve()


def boxes_for_frame(instance_path: Path) -> list[dict]:
    output = []
    for unique_id, record in sorted(parse_lidar_instances(instance_path).items()):
        box = np.asarray(
            record["center"] + record["size"] + [record["rotation_rpy"][2]],
            dtype=np.float64,
        )
        if box.shape != (7,) or not np.isfinite(box).all() or np.any(box[3:6] <= 0.0):
            continue
        if not math.isfinite(float(record["distance"])):
            continue
        output.append({
            "instance_id": int(unique_id),
            "class": coarse_class(record["raw_class"]),
            "raw_class": record["raw_class"],
            "box_lidar": box.tolist(),
            "distance_m": float(record["distance"]),
        })
    return output


def main() -> None:
    args = parse_args()
    capture_root = args.capture_root.expanduser().resolve()
    selection = load_json(args.selection.expanduser().resolve())
    output_root = args.output_root.expanduser().resolve()
    clips = selected_clips(capture_root, selection)
    if args.max_clips > 0:
        clips = clips[: args.max_clips]

    calibration = calibration_from_sensor_config(
        args.sensor_config.expanduser().resolve(), tuple(CAMERA_DIRS)
    )
    write_json(output_root / "metadata/calibrations/sensor6_v1.json", calibration)

    scenes = []
    frames = []
    annotations = []
    split_scenes = defaultdict(list)
    split_frames = Counter()
    class_boxes = Counter()
    for clip_index, clip in enumerate(clips, start=1):
        split = clip["split"]
        scene_id = clip["scene_id"]
        split_scenes[split].append(scene_id)
        scenes.append({
            "scene_id": scene_id,
            "split": split,
            "capture_run": clip["run_name"],
            "mission_family": clip["mission_family"],
            "frame_count": 3,
        })
        for record in frame_records(clip):
            frame_id = str(record["frame_id"])
            cameras = {}
            for camera_name, directory in CAMERA_DIRS.items():
                cameras[camera_name] = {
                    "intensity": str(artifact(record, clip["run_root"], directory, "_Intensity.png"))
                }
            lidar_intensity = artifact(record, clip["run_root"], "LIDAR_5", "_Intensity.bin")
            # MORAI emits a zero-byte Instance.txt for legitimate empty/background
            # frames.  It is an empty annotation, not a missing artifact.
            lidar_instance = artifact(
                record,
                clip["run_root"],
                "LIDAR_5",
                "_Instance.txt",
                allow_empty=True,
            )
            boxes = boxes_for_frame(lidar_instance)
            class_boxes.update(box["class"] for box in boxes)
            frames.append({
                "frame_id": frame_id,
                "scene_id": scene_id,
                "frame_index": int(record["frame_index"]),
                "split": split,
                "capture_run": clip["run_name"],
                "mission_family": clip["mission_family"],
                "calibration_id": "sensor6_v1",
                "triggered_unix": record.get("triggered_unix"),
                "cameras": cameras,
                "lidar": {
                    "intensity_bin": str(lidar_intensity),
                    "instance_txt": str(lidar_instance),
                },
            })
            annotations.append({"frame_id": frame_id, "scene_id": scene_id, "boxes": boxes})
            split_frames[split] += 1
        if clip_index % 250 == 0 or clip_index == len(clips):
            print(f"clips: {clip_index}/{len(clips)}", flush=True)

    if not args.max_clips:
        expected = selection["expected"]
        actual_split_clips = {split: len(split_scenes[split]) for split in SPLITS}
        if len(clips) != int(expected["clips"]):
            raise RuntimeError("clip count mismatch")
        if len(frames) != int(expected["frames"]):
            raise RuntimeError("frame count mismatch")
        if actual_split_clips != expected["split_clips"]:
            raise RuntimeError(
                f"split clip mismatch: {actual_split_clips} != {expected['split_clips']}"
            )

    write_jsonl(output_root / "metadata/scenes.jsonl", scenes)
    write_jsonl(output_root / "metadata/frames.jsonl", frames)
    write_jsonl(output_root / "annotations/boxes_3d.jsonl", annotations)
    for split in SPLITS:
        path = output_root / "splits" / f"{split}.txt"
        path.parent.mkdir(parents=True, exist_ok=True)
        # The raw painter verifies split isolation at scene level.  Its output
        # ImageSets are generated separately at frame level.
        path.write_text(
            "".join(scene + "\n" for scene in sorted(split_scenes[split])),
            encoding="utf-8",
        )

    report = {
        "schema_version": 1,
        "dataset_id": "morai_base3_static_10k_pointpainting_input_v1",
        "source_selection_id": selection["selection_id"],
        "capture_root": str(capture_root),
        "clips": len(clips),
        "frames": len(frames),
        "split_clips": {split: len(split_scenes[split]) for split in SPLITS},
        "split_frames": {split: split_frames[split] for split in SPLITS},
        "raw_lidar_boxes": dict(sorted(class_boxes.items())),
        "camera_payload": "Intensity PNG only",
        "lidar_payload": "Intensity bin plus Instance txt GT",
        "depth_or_instance_image_required": False,
    }
    write_json(output_root / "metadata/dataset.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
