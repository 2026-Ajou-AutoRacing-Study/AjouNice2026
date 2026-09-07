#!/usr/bin/env python3
"""Build one model-neutral index from an accepted multi-run capture set."""

import argparse
import importlib.util
import json
import pickle
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BASE_BUILDER_PATH = ROOT / "dataset/morai_od/scripts/build_fastbev_sunny13_1500_dataset.py"
CAMERA_ORDER = ("front", "front_left", "front_right", "back")
TARGET_SPLIT_FRAMES = {"train": 1050, "val": 225, "test": 225}


def load_base_builder():
    spec = importlib.util.spec_from_file_location("morai_single_frame_builder", BASE_BUILDER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_base_builder()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--sensor-config",
        type=Path,
        default=ROOT / "configs/sensors/Sensor_6.json",
    )
    parser.add_argument(
        "--preserve-splits",
        action="store_true",
        help=(
            "Keep manifest clip splits and infer split frame counts instead "
            "of enforcing the 1,500-frame production target."
        ),
    )
    return parser.parse_args()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path):
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def scene_records(run):
    return {row["id"]: row for row in read_jsonl(run / "scenes.jsonl")}


def static_files(run, scene_id, frame_index):
    marker = "{}_".format(scene_id)
    frame_marker = "_f{:02d}_".format(frame_index)
    paths = []
    for directory in ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4", "LIDAR_5"):
        for path in (run / directory).glob("*{}*".format(scene_id)):
            if marker in path.name and frame_marker in path.name:
                paths.append(path)
    return sorted(str(path.relative_to(run)) for path in paths)


def frames_for_scene(run, record, frames_per_scene):
    metadata_relative = record.get("metrics", {}).get("frames_metadata")
    if metadata_relative:
        rows = read_jsonl(run / metadata_relative)
        if len(rows) != frames_per_scene:
            raise RuntimeError("temporal frame count mismatch for {}".format(record["id"]))
        return rows
    rows = []
    for index in range(frames_per_scene):
        files = static_files(run, record["id"], index)
        if not files:
            raise RuntimeError("no sensor files for {} frame {}".format(record["id"], index))
        intensity = next(
            run / item for item in files
            if item.startswith("CAMERA_1/") and item.endswith("_Intensity.png")
        )
        rows.append(
            {
                "frame_id": "{}_f{:06d}".format(record["id"], index),
                "capture_id": intensity.name[: -len("_Intensity.png")],
                "frame_index": index,
                "triggered_unix": intensity.stat().st_mtime,
                "sensor_files": files,
                "ego": None,
                "objects": None,
            }
        )
    return rows


def rebalance_splits(scenes):
    counts = Counter()
    for scene in scenes:
        counts[scene["split"]] += scene["frame_count"]
    for source in ("val", "test"):
        excess = counts[source] - TARGET_SPLIT_FRAMES[source]
        candidates = sorted(
            (scene for scene in scenes if scene["split"] == source and scene["frame_count"] == 3),
            key=lambda scene: scene["scene_id"],
        )
        while excess > 0:
            if excess < 3 or not candidates:
                raise RuntimeError("cannot rebalance {} frame excess {}".format(source, excess))
            scene = candidates.pop(0)
            scene["split"] = "train"
            counts[source] -= 3
            counts["train"] += 3
            excess -= 3
    if dict(counts) != TARGET_SPLIT_FRAMES:
        raise RuntimeError("split frame targets do not match: {}".format(dict(counts)))
    return counts


def main():
    args = parse_args()
    selection = read_json(args.selection.resolve())
    calibration = base.calibration_from_sensor_config(args.sensor_config.resolve(), CAMERA_ORDER)
    scenes = []
    cache = {}
    for component_index, component in enumerate(selection["components"]):
        run = Path(component["capture_dir"])
        records = cache.setdefault(str(run), scene_records(run))
        for scene_id in component["scene_ids"]:
            record = records[scene_id]
            scenes.append(
                {
                    "scene_id": scene_id,
                    "split": record["split"],
                    "frame_count": component["frames_per_scene"],
                    "role": component["role"],
                    "purpose": component["purpose"],
                    "capture_dir": str(run),
                    "record": record,
                    "component_index": component_index,
                }
            )
    if args.preserve_splits:
        split_counts = Counter()
        for scene in scenes:
            split_counts[scene["split"]] += scene["frame_count"]
    else:
        split_counts = rebalance_splits(scenes)

    frame_rows, box_rows = [], []
    fastbev_infos = defaultdict(list)
    rejected = []
    class_counts = Counter()
    for scene in scenes:
        run = Path(scene["capture_dir"])
        raw_frames = frames_for_scene(run, scene["record"], scene["frame_count"])
        accepted_frames = []
        for raw in raw_frames:
            capture = {"new_files": raw["sensor_files"]}
            inspected = base.inspect_capture(capture, run, calibration)
            # Empty LiDAR text labels are valid negative frames. Other
            # completeness failures remain fatal for an accepted clip.
            missing = [item for item in inspected["missing"] if not item.endswith(("_Instance.txt", "_Instance_8Points.txt"))]
            if missing:
                rejected.append(
                    {
                        "scene_id": scene["scene_id"],
                        "frame_id": raw["frame_id"],
                        "missing": missing,
                        "camera_visible_missing_lidar_ids": inspected["camera_visible_missing_lidar_ids"],
                    }
                )
                continue
            cameras = {
                name: {
                    "intensity": paths["intensity"],
                    "semantic": paths["semantic"],
                    "instance": paths["instance"],
                    "depth": paths["depth"],
                    "bbox2d": paths["bbox2d"],
                    "bbox3d": paths["bbox3d"],
                }
                for name, paths in inspected["camera_paths"].items()
            }
            frame = {
                "frame_id": raw["frame_id"],
                "scene_id": scene["scene_id"],
                "frame_index": raw["frame_index"],
                "split": scene["split"],
                "role": scene["role"],
                "purpose": scene["purpose"],
                "calibration_id": "sensor6_v1",
                "triggered_unix": raw.get("triggered_unix"),
                "cameras": cameras,
                "lidar": inspected["lidar_paths"],
                "ego": raw.get("ego"),
                "objects": raw.get("objects"),
                "boxes": inspected["boxes"],
                "quality": {"complete": True, "camera_lidar_cross_checked": True},
            }
            frame["quality"]["camera_only_object_ids"] = inspected[
                "camera_visible_missing_lidar_ids"
            ]
            accepted_frames.append(frame)
        if len(accepted_frames) != scene["frame_count"]:
            continue
        for frame in accepted_frames:
            frame_rows.append({key: value for key, value in frame.items() if key != "boxes"})
            box_rows.append({"frame_id": frame["frame_id"], "scene_id": frame["scene_id"], "boxes": frame["boxes"]})
            class_counts.update(box["class"] for box in frame["boxes"])
            fastbev_infos[frame["split"]].append(base.build_fastbev_info(frame, calibration))

    if rejected or len(frame_rows) != selection["total_frames"]:
        write_jsonl(args.output_root / "reports/rejected_frames.jsonl", rejected)
        raise RuntimeError(
            "accepted dataset inspection failed: frames={} rejected={}".format(len(frame_rows), len(rejected))
        )

    output = args.output_root.resolve()
    write_json(output / "metadata/calibrations/sensor6_v1.json", calibration)
    write_jsonl(output / "metadata/frames.jsonl", frame_rows)
    write_jsonl(output / "annotations/boxes_3d.jsonl", box_rows)
    write_jsonl(output / "reports/rejected_frames.jsonl", [])
    for split in ("train", "val", "test"):
        ids = sorted(scene["scene_id"] for scene in scenes if scene["split"] == split)
        path = output / "splits" / "{}.txt".format(split)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(item + "\n" for item in ids), encoding="utf-8")
        info_path = output / "fastbev_infos" / "morai_infos_{}.pkl".format(split)
        info_path.parent.mkdir(parents=True, exist_ok=True)
        with info_path.open("wb") as stream:
            pickle.dump(
                {"infos": fastbev_infos[split], "metadata": {"version": selection["dataset_id"], "classes": ["vehicle", "pedestrian", "obstacle"]}},
                stream,
                protocol=4,
            )
    report = {
        "schema_version": 1,
        "dataset_id": selection["dataset_id"],
        "selected_scenes": len(scenes),
        "selected_frames": len(frame_rows),
        "split_frames": dict(split_counts),
        "class_instances": dict(class_counts),
        "has_temporal_state": sum(1 for row in frame_rows if row.get("ego") is not None),
        "source_selection": str(args.selection.resolve()),
    }
    write_json(output / "metadata/dataset.json", report)
    write_json(output / "reports/build_summary.json", report)
    print("dataset:", output)
    print("scenes:", len(scenes))
    print("frames:", len(frame_rows))
    print("split frames:", dict(split_counts))
    print("class instances:", dict(class_counts))
    print("temporal-state frames:", report["has_temporal_state"])


if __name__ == "__main__":
    main()
