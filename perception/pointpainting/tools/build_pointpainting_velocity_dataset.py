#!/usr/bin/env python3
"""Build a 9D MORAI PointPainting dataset without repainting LiDAR points.

The input is the combined static/dynamic painted dataset.  Dynamic box IDs and
actor motion come from the model-neutral SaveSensorData index, so velocity is
joined by MORAI unique_id rather than by box order or nearest centre.
"""

import argparse
import json
import math
import os
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from temporal_geometry import derive_actor_velocities  # noqa: E402


CLASS_NAMES = {
    "vehicle": "Vehicle",
    "pedestrian": "Pedestrian",
    "obstacle": "Obstacle",
}
SPLITS = ("train", "val", "test")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--dynamic-source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--nearest-state-tolerance-sec",
        type=float,
        default=1.5,
        help="Maximum same-scene time gap used when one Object_topic snapshot is missing.",
    )
    parser.add_argument(
        "--invalid-dynamic-policy",
        choices=("drop", "zero"),
        default="drop",
        help="Policy for a moving box with no trustworthy derived velocity.",
    )
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
                json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
            )


def link_or_copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        target.unlink()
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def parse_7d_labels(path):
    boxes = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 8:
            raise RuntimeError(f"{path}: expected 7D label, got {len(fields) - 1}D")
        boxes.append(([float(value) for value in fields[:7]], fields[7]))
    return boxes


def actor_time(frame):
    objects = frame.get("objects") or {}
    if objects.get("timestamp") is not None:
        return float(objects["timestamp"])
    if frame.get("triggered_unix") is not None:
        return float(frame["triggered_unix"])
    return None


def actor_velocity_lidar(actor):
    # Sensor_6 LiDAR and Ego axes are parallel.  The capture metadata rotates
    # map velocity into Ego axes without subtracting Ego velocity, yielding
    # the absolute object velocity expected by the detector/tracker contract.
    velocity = actor.get("simulation_velocity_lidar_mps")
    if velocity is None:
        return None
    vx = float(velocity["x"])
    vy = float(velocity["y"])
    if not math.isfinite(vx) or not math.isfinite(vy):
        return None
    return vx, vy


def build_actor_tracks(frames):
    tracks = defaultdict(list)
    for frame in frames:
        timestamp = actor_time(frame)
        if timestamp is None:
            continue
        for actor in (frame.get("objects") or {}).get("actors", []):
            tracks[(frame["scene_id"], int(actor["unique_id"]))].append(
                (timestamp, actor)
            )
    for values in tracks.values():
        values.sort(key=lambda item: item[0])
    return tracks


def velocity_for_actor(frame, unique_id, actors, tracks, tolerance):
    actor = actors.get(unique_id)
    if actor is not None:
        velocity = actor_velocity_lidar(actor)
        if velocity is not None:
            return velocity, "exact"

    timestamp = actor_time(frame)
    if timestamp is None:
        return None, "missing"
    candidates = []
    for candidate_time, candidate in tracks.get((frame["scene_id"], unique_id), []):
        velocity = actor_velocity_lidar(candidate)
        if velocity is not None:
            candidates.append((abs(candidate_time - timestamp), velocity))
    if not candidates:
        return None, "missing"
    gap, velocity = min(candidates, key=lambda item: item[0])
    if gap > tolerance:
        return None, "missing"
    return velocity, "nearest"


def validate_dynamic_geometry(frame_id, source_boxes, input_labels):
    comparable = [
        (box, CLASS_NAMES.get(str(item.get("class", "")).lower()))
        for item in source_boxes
        for box in [[float(value) for value in item["box_lidar"][:7]]]
        if CLASS_NAMES.get(str(item.get("class", "")).lower()) is not None
    ]
    if len(comparable) != len(input_labels):
        raise RuntimeError(
            f"{frame_id}: model-neutral/input label count mismatch "
            f"{len(comparable)} != {len(input_labels)}"
        )
    unmatched = list(input_labels)
    for box, class_name in comparable:
        best_index = None
        best_error = float("inf")
        for index, (candidate, candidate_class) in enumerate(unmatched):
            if candidate_class != class_name:
                continue
            error = math.hypot(box[0] - candidate[0], box[1] - candidate[1])
            if error < best_error:
                best_error = error
                best_index = index
        if best_index is None or best_error > 1.0e-3:
            raise RuntimeError(
                f"{frame_id}: 7D labels do not match model-neutral instance boxes"
            )
        unmatched.pop(best_index)


def format_label(box, velocity, class_name):
    values = list(box[:7]) + [float(velocity[0]), float(velocity[1])]
    return " ".join(f"{value:.6f}" for value in values) + f" {class_name}\n"


def main():
    args = parse_args()
    input_root = args.input_root.resolve()
    dynamic_root = args.dynamic_source_root.resolve()
    output_root = args.output_root.resolve()

    combined_rows = read_jsonl(input_root / "metadata/frames.jsonl")
    combined_by_id = {row["frame_id"]: row for row in combined_rows}
    dynamic_frames = read_jsonl(dynamic_root / "metadata/frames.jsonl")
    derive_actor_velocities(dynamic_frames)
    dynamic_by_id = {row["frame_id"]: row for row in dynamic_frames}
    dynamic_boxes = {
        row["frame_id"]: row.get("boxes", [])
        for row in read_jsonl(dynamic_root / "annotations/boxes_3d.jsonl")
    }
    actor_tracks = build_actor_tracks(dynamic_frames)

    split_ids = {}
    for split in SPLITS:
        split_ids[split] = [
            row.strip()
            for row in (input_root / "ImageSets" / f"{split}.txt").read_text(
                encoding="utf-8"
            ).splitlines()
            if row.strip()
        ]

    all_ids = [frame_id for split in SPLITS for frame_id in split_ids[split]]
    if len(all_ids) != len(set(all_ids)):
        raise RuntimeError("split frame IDs overlap")
    if set(all_ids) != set(combined_by_id):
        raise RuntimeError("ImageSets and combined metadata frame IDs differ")

    counters = Counter()
    speeds = defaultdict(list)
    target_rows = []
    dropped_rows = []
    for frame_id in all_ids:
        combined = combined_by_id[frame_id]
        input_labels = parse_7d_labels(input_root / "labels" / f"{frame_id}.txt")
        output_lines = []

        if combined["source"] == "static1500":
            for box, class_name in input_labels:
                output_lines.append(format_label(box, (0.0, 0.0), class_name))
                counters[(class_name, "static_zero")] += 1
                target_rows.append(
                    {
                        "frame_id": frame_id,
                        "instance_id": None,
                        "class": class_name.lower(),
                        "velocity_lidar_mps": {"x": 0.0, "y": 0.0},
                        "source": "static_zero",
                    }
                )
        else:
            frame = dynamic_by_id.get(frame_id)
            boxes = dynamic_boxes.get(frame_id)
            if frame is None or boxes is None:
                raise RuntimeError(f"{frame_id}: missing dynamic source metadata")
            validate_dynamic_geometry(frame_id, boxes, input_labels)
            actors = {
                int(actor["unique_id"]): actor
                for actor in (frame.get("objects") or {}).get("actors", [])
            }
            static_role = "static" in str(frame.get("role", ""))
            for item in boxes:
                class_key = str(item.get("class", "")).lower()
                class_name = CLASS_NAMES.get(class_key)
                if class_name is None:
                    continue
                unique_id = int(item["instance_id"])
                source = "static_zero"
                velocity = (0.0, 0.0)
                if class_key != "obstacle" and not static_role:
                    velocity, source = velocity_for_actor(
                        frame,
                        unique_id,
                        actors,
                        actor_tracks,
                        args.nearest_state_tolerance_sec,
                    )
                    if velocity is None:
                        if args.invalid_dynamic_policy == "zero":
                            velocity = (0.0, 0.0)
                            source = "invalid_zero"
                        else:
                            dropped_rows.append(
                                {
                                    "frame_id": frame_id,
                                    "scene_id": frame["scene_id"],
                                    "instance_id": unique_id,
                                    "class": class_key,
                                    "reason": "missing_derived_velocity",
                                }
                            )
                            counters[(class_name, "dropped_missing_velocity")] += 1
                            continue
                box = [float(value) for value in item["box_lidar"][:7]]
                output_lines.append(format_label(box, velocity, class_name))
                counters[(class_name, source)] += 1
                speeds[class_name].append(math.hypot(*velocity))
                target_rows.append(
                    {
                        "frame_id": frame_id,
                        "instance_id": unique_id,
                        "class": class_key,
                        "velocity_lidar_mps": {
                            "x": float(velocity[0]),
                            "y": float(velocity[1]),
                        },
                        "source": source,
                    }
                )

        label_path = output_root / "labels" / f"{frame_id}.txt"
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("".join(output_lines), encoding="utf-8")
        link_or_copy(
            input_root / "points" / f"{frame_id}.npy",
            output_root / "points" / f"{frame_id}.npy",
        )

    for split, ids in split_ids.items():
        path = output_root / "ImageSets" / f"{split}.txt"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(f"{frame_id}\n" for frame_id in ids), encoding="utf-8")

    write_jsonl(output_root / "metadata/velocity_targets.jsonl", target_rows)
    write_jsonl(output_root / "reports/dropped_velocity_boxes.jsonl", dropped_rows)
    speed_summary = {}
    for class_name, values in speeds.items():
        ordered = sorted(values)
        if not ordered:
            continue
        percentile = lambda fraction: ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]
        speed_summary[class_name] = {
            "count": len(ordered),
            "p50_mps": percentile(0.50),
            "p90_mps": percentile(0.90),
            "p99_mps": percentile(0.99),
            "max_mps": ordered[-1],
        }
    report = {
        "schema_version": 1,
        "dataset_id": "morai_pointpainting_static1500_dynamic1500_velocity9d_v1",
        "input_root": str(input_root),
        "dynamic_source_root": str(dynamic_root),
        "frames": len(all_ids),
        "split_frames": {key: len(value) for key, value in split_ids.items()},
        "box_target_counts": {
            f"{class_name}/{source}": count
            for (class_name, source), count in sorted(counters.items())
        },
        "dropped_dynamic_boxes": len(dropped_rows),
        "speed_summary": speed_summary,
        "box_layout": ["x", "y", "z", "dx", "dy", "dz", "yaw", "vx", "vy"],
        "velocity_frame": "lidar, absolute object velocity, metres_per_second",
        "velocity_source": (
            "Object_topic position finite differences on an inferred physical "
            "simulator time axis"
        ),
    }
    write_json(output_root / "metadata/dataset.json", report)
    write_json(output_root / "reports/velocity_build_summary.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
