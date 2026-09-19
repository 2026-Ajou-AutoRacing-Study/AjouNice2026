#!/usr/bin/env python3
"""Recover compact progress from verified capture filenames only.

This intentionally does not open legacy report JSON or console logs.
"""

import argparse
import json
import re
import time
from pathlib import Path


CAMERA_SUFFIXES = (
    "_Intensity.png",
    "_Semantic.png",
    "_Instance.png",
    "_Depth.png",
)
LIDAR_SUFFIXES = (
    "_Intensity.bin",
    "_Semantic.bin",
    "_Instance.bin",
    "_Instance.txt",
    "_Instance_8Points.txt",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--scene-prefix", required=True)
    parser.add_argument("--scene-count", type=int, required=True)
    parser.add_argument("--environment-id", required=True)
    parser.add_argument("--frames-per-scene", type=int, default=6)
    return parser.parse_args()


def write_json(path, document):
    path.write_text(
        json.dumps(document, ensure_ascii=False, separators=(",", ":"))
        + "\n",
        encoding="utf-8",
    )


def frame_is_complete(run_dir, token):
    for camera in ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4"):
        for suffix in CAMERA_SUFFIXES:
            if not (run_dir / camera / (token + suffix)).is_file():
                return False
    for suffix in LIDAR_SUFFIXES:
        if not (run_dir / "LIDAR_5" / (token + suffix)).is_file():
            return False
    return True


def main():
    args = parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    expression = re.compile(
        r"^({prefix}\d+)_({environment})_f(\d+)_.*_Intensity\.png$".format(
            prefix=re.escape(args.scene_prefix),
            environment=re.escape(args.environment_id),
        )
    )
    complete_frames = {}
    first_capture_unix = None
    last_capture_unix = None
    for path in (run_dir / "CAMERA_1").glob("*_Intensity.png"):
        match = expression.match(path.name)
        if not match:
            continue
        modified = path.stat().st_mtime
        first_capture_unix = (
            modified
            if first_capture_unix is None
            else min(first_capture_unix, modified)
        )
        last_capture_unix = (
            modified
            if last_capture_unix is None
            else max(last_capture_unix, modified)
        )
        scene_id = match.group(1)
        frame_index = int(match.group(3))
        token = path.name[: -len("_Intensity.png")]
        if frame_is_complete(run_dir, token):
            complete_frames.setdefault(scene_id, set()).add(frame_index)

    width = max(3, len(str(args.scene_count)))
    requested_ids = [
        "{}{:0{}d}".format(args.scene_prefix, index, width)
        for index in range(1, args.scene_count + 1)
    ]
    expected_frames = set(range(args.frames_per_scene))
    completed_ids = [
        scene_id
        for scene_id in requested_ids
        if complete_frames.get(scene_id, set()) >= expected_frames
    ]
    failed_ids = [
        scene_id for scene_id in requested_ids if scene_id not in completed_ids
    ]

    scenes_path = run_dir / "scenes.jsonl"
    with scenes_path.open("w", encoding="utf-8") as stream:
        for scene_id in requested_ids:
            frames = len(complete_frames.get(scene_id, set()))
            record = {
                "id": scene_id,
                "status": (
                    "completed" if scene_id in completed_ids else "failed"
                ),
                "capture_count": frames,
                "recovered_from_verified_files": True,
            }
            stream.write(
                json.dumps(record, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )

    failures_path = run_dir / "failures.jsonl"
    with failures_path.open("w", encoding="utf-8") as stream:
        for scene_id in failed_ids:
            stream.write(
                json.dumps(
                    {
                        "id": scene_id,
                        "status": "failed",
                        "capture_count": len(
                            complete_frames.get(scene_id, set())
                        ),
                        "reason": (
                            "legacy detail not parsed; retry with compact logger"
                        ),
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
                + "\n"
            )

    summary = {
        "schema_version": 2,
        "dataset_id": args.dataset_id,
        "environment": {"id": args.environment_id},
        "started_unix": first_capture_unix or run_dir.stat().st_mtime,
        "checkpoint_unix": time.time(),
        "motion_mode": "rolling-grouped",
        "capture_dir": str(run_dir),
        "interrupted": False,
        "finished": True,
        "finished_unix": last_capture_unix or time.time(),
        "requested_scenes": args.scene_count,
        "attempted_scenes": args.scene_count,
        "completed_scenes": len(completed_ids),
        "failed_scenes": len(failed_ids),
        "completed_captures": len(completed_ids) * args.frames_per_scene,
        "completed_scene_ids": completed_ids,
        "failed_scene_ids": failed_ids,
        "recent_failed_scene_ids": failed_ids[-8:],
        "latest_scene": {
            "id": requested_ids[-1],
            "status": (
                "completed"
                if requested_ids[-1] in completed_ids
                else "failed"
            ),
        },
        "recovered_from_verified_files": True,
        "files": {
            "scene_records": "scenes.jsonl",
            "failure_details": "failures.jsonl",
        },
    }
    write_json(run_dir / "summary.json", summary)
    print("summary:", run_dir / "summary.json")
    print("completed scenes:", len(completed_ids))
    print("failed/pending scenes:", len(failed_ids))
    print("verified frames:", summary["completed_captures"])


if __name__ == "__main__":
    main()
