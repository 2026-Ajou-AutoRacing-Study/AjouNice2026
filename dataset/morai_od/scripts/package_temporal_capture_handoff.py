#!/usr/bin/env python3
"""Package complete temporal capture clips into a compact team handoff."""

import argparse
import json
import os
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path


CAMERA_DIRS = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")
CAMERA_REQUIRED = ("_Intensity.png", "_Semantic.png")
CAMERA_OPTIONAL = ("_BBox2D.txt", "_BBox3D.txt")
LIDAR_REQUIRED = (
    "_Intensity.bin",
    "_Instance.txt",
    "_Instance_8Points.txt",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    return parser.parse_args()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path):
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(
                json.dumps(row, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )


def link_or_copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError("refusing to overwrite {}".format(target))
    try:
        os.link(str(source), str(target))
    except OSError:
        shutil.copy2(str(source), str(target))


def index_sensor_files(run):
    """Index each large sensor directory once instead of once per scene."""

    result = {}
    pattern = re.compile(r"_(t2.+?)_f\d+_")
    for directory in CAMERA_DIRS + ("LIDAR_5",):
        by_scene = defaultdict(list)
        for path in (run / directory).iterdir():
            if not path.is_file():
                continue
            match = pattern.search(path.name)
            if match:
                by_scene[match.group(1)].append(path)
        result[directory] = {
            scene_id: sorted(paths) for scene_id, paths in by_scene.items()
        }
    return result


def keep_camera(path):
    return path.name.endswith(CAMERA_REQUIRED + CAMERA_OPTIONAL)


def keep_lidar(path):
    return path.name.endswith(LIDAR_REQUIRED)


def frame_ids(frame_rows):
    return [row["frame_id"] for row in frame_rows]


def main():
    args = parse_args()
    session = args.session_root.resolve()
    output = args.output_root.resolve()
    if not session.is_dir():
        raise RuntimeError("session root does not exist: {}".format(session))
    if output.exists():
        raise RuntimeError("output already exists: {}".format(output))

    runs = sorted(session.glob("batch*_primary"))
    if not runs:
        raise RuntimeError("no batch primary runs found under {}".format(session))

    output.mkdir(parents=True)
    selected_scenes = []
    selected_frames = []
    split_scene_ids = {name: [] for name in ("train", "val", "test")}
    split_frame_ids = {name: [] for name in ("train", "val", "test")}
    family_scenes = Counter()
    split_scenes = Counter()
    copied_files = Counter()
    copied_bytes = Counter()

    calibration_written = False
    for run in runs:
        sensor_index = index_sensor_files(run)
        scene_path = run / "scenes.jsonl"
        if not scene_path.is_file():
            raise RuntimeError("missing scene record: {}".format(scene_path))
        for record in read_jsonl(scene_path):
            if record.get("status") != "completed":
                continue
            metrics = record.get("metrics", {})
            if not metrics.get("quality_verified"):
                continue
            if int(record.get("capture_count", 0)) != 8:
                continue

            scene_id = record["id"]
            split = record["split"]
            frames_relative = metrics.get("frames_metadata")
            clip_relative = metrics.get("clip_metadata")
            if split not in split_scene_ids or not frames_relative or not clip_relative:
                raise RuntimeError("invalid completed scene record: {}".format(scene_id))
            frames_source = run / frames_relative
            clip_source = run / clip_relative
            frames = read_jsonl(frames_source)
            clip = read_json(clip_source)
            if len(frames) != 8 or not all(row.get("verified") for row in frames):
                raise RuntimeError("incomplete frame metadata: {}".format(scene_id))

            family = frames[0].get("mission_family", "unknown")
            family_scenes[family] += 1
            split_scenes[split] += 1
            split_scene_ids[split].append(scene_id)
            split_frame_ids[split].extend(frame_ids(frames))

            expected_camera_required = 8 * len(CAMERA_REQUIRED)
            for directory in CAMERA_DIRS:
                candidates = sensor_index[directory].get(scene_id, [])
                required = [
                    path for path in candidates if path.name.endswith(CAMERA_REQUIRED)
                ]
                if len(required) != expected_camera_required:
                    raise RuntimeError(
                        "{} {} required camera files, expected {} got {}".format(
                            scene_id,
                            directory,
                            expected_camera_required,
                            len(required),
                        )
                    )
                for source in candidates:
                    if not keep_camera(source):
                        continue
                    target = output / directory / source.name
                    link_or_copy(source, target)
                    copied_files[directory] += 1
                    copied_bytes[directory] += source.stat().st_size

            lidar = sensor_index["LIDAR_5"].get(scene_id, [])
            for suffix in LIDAR_REQUIRED:
                count = sum(path.name.endswith(suffix) for path in lidar)
                if count != 8:
                    raise RuntimeError(
                        "{} LiDAR {} expected 8 got {}".format(
                            scene_id, suffix, count
                        )
                    )
            for source in lidar:
                if not keep_lidar(source):
                    continue
                target = output / "LIDAR_5" / source.name
                link_or_copy(source, target)
                copied_files["LIDAR_5"] += 1
                copied_bytes["LIDAR_5"] += source.stat().st_size

            for suffix in (".frames.jsonl", ".clip.json", ".timing.jsonl"):
                source = run / "dynamic_metadata" / "{}{}".format(scene_id, suffix)
                if not source.is_file():
                    raise RuntimeError("missing temporal metadata: {}".format(source))
                target = output / "metadata/clips" / source.name
                link_or_copy(source, target)
                copied_files["metadata"] += 1
                copied_bytes["metadata"] += source.stat().st_size

            if not calibration_written:
                calibration = run / "dynamic_metadata/calibration.json"
                if not calibration.is_file():
                    raise RuntimeError("missing calibration: {}".format(calibration))
                link_or_copy(calibration, output / "metadata/calibration.json")
                calibration_written = True

            selected_scenes.append(
                {
                    "scene_id": scene_id,
                    "batch": run.name,
                    "split": split,
                    "mission_family": family,
                    "frame_count": 8,
                    "source_record": record,
                    "clip": clip,
                }
            )
            selected_frames.extend(frames)

    for split in ("train", "val", "test"):
        split_dir = output / "splits"
        split_dir.mkdir(parents=True, exist_ok=True)
        (split_dir / "{}_scenes.txt".format(split)).write_text(
            "".join(value + "\n" for value in split_scene_ids[split]),
            encoding="utf-8",
        )
        (split_dir / "{}_frames.txt".format(split)).write_text(
            "".join(value + "\n" for value in split_frame_ids[split]),
            encoding="utf-8",
        )

    write_jsonl(output / "metadata/scenes.jsonl", selected_scenes)
    write_jsonl(output / "metadata/frames.jsonl", selected_frames)
    report = {
        "schema_version": 1,
        "dataset_id": args.dataset_id,
        "source_session": str(session),
        "selection_policy": "complete quality-verified 8-frame clips only",
        "scene_count": len(selected_scenes),
        "frame_count": len(selected_frames),
        "split_scenes": dict(split_scenes),
        "split_frames": {
            split: len(values) for split, values in split_frame_ids.items()
        },
        "family_scenes": dict(sorted(family_scenes.items())),
        "kept_files": dict(copied_files),
        "kept_bytes": dict(copied_bytes),
        "retained": {
            "camera": list(CAMERA_REQUIRED + CAMERA_OPTIONAL),
            "lidar": list(LIDAR_REQUIRED),
            "temporal_metadata": [
                ".frames.jsonl",
                ".clip.json",
                ".timing.jsonl",
            ],
        },
        "excluded": [
            "failed and partial clips",
            "camera Depth.png",
            "camera Instance.png",
            "LiDAR Semantic.bin",
            "LiDAR Instance.bin",
            "sensor prime files and capture logs",
        ],
        "quality_warning": (
            "Base1 contains generator-distribution defects fixed in Base2. "
            "Use for diagnostic experiments, not final production claims."
        ),
    }
    write_json(output / "metadata/dataset.json", report)
    print("dataset:", output)
    print("scenes:", report["scene_count"])
    print("frames:", report["frame_count"])
    print("split frames:", report["split_frames"])
    print("family scenes:", report["family_scenes"])
    print("kept GiB: {:.3f}".format(sum(copied_bytes.values()) / 1024 ** 3))


if __name__ == "__main__":
    main()
