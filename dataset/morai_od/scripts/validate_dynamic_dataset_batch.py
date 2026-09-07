#!/usr/bin/env python3
"""Validate generated manifests and exact frame quotas for one production batch."""

import argparse
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def main():
    args = parse_args()
    tag = f"batch{args.batch_index:03d}"
    generated = DATASET_ROOT / "metadata/production_batches" / tag
    plan = read_json(generated / "batch_plan.json")
    errors = []
    ids = set()
    rows = []
    primary_frames = 0
    reserve_frames = 0
    for component in plan["components"]:
        role = component["role"]
        group = f"dynamic_dataset_{tag}_{role}"
        if component["capture_mode"] == "timed-grouped":
            runtime_path = DATASET_ROOT / "manifests" / f"{group}_runtime.json"
            source_path = DATASET_ROOT / "manifests" / f"{group}.json"
            # The runtime manifest is created by install_dynamic_capture_set.py.
            # A --no-install generation must still be fully verifiable before it
            # is allowed to touch the MORAI installation directory.
            manifest_path = runtime_path if runtime_path.is_file() else source_path
        else:
            manifest_path = DATASET_ROOT / "manifests" / f"{group}_sunny_t13.json"
        if not manifest_path.is_file():
            errors.append(f"missing manifest: {manifest_path}")
            continue
        manifest = read_json(manifest_path)
        scenes = manifest.get("scenes", [])
        if len(scenes) != int(component["scenes"]):
            errors.append(f"{role}: scenes {len(scenes)} != {component['scenes']}")
        duplicate = [scene["id"] for scene in scenes if scene["id"] in ids]
        if duplicate:
            errors.append(f"{role}: duplicate ids {duplicate[:3]}")
        ids.update(scene["id"] for scene in scenes)
        actual_frames = len(scenes) * int(component["frames_per_scene"])
        if actual_frames != int(component["frames"]):
            errors.append(f"{role}: frames {actual_frames} != {component['frames']}")
        if component["reserve"]:
            reserve_frames += actual_frames
        else:
            primary_frames += actual_frames
        rows.append({**component, "manifest": str(manifest_path.relative_to(PROJECT_ROOT)), "actual_scenes": len(scenes), "actual_frames": actual_frames})

    if primary_frames != 1500:
        errors.append(f"primary frames {primary_frames} != 1500")
    expected_reserve_frames = sum(plan["reserve_frame_quotas"].values())
    if reserve_frames != expected_reserve_frames:
        errors.append(
            f"reserve frames {reserve_frames} != {expected_reserve_frames}"
        )
    report = {
        "schema_version": 1,
        "batch_id": tag,
        "status": "pass" if not errors else "fail",
        "primary_frames": primary_frames,
        "reserve_frames": reserve_frames,
        "unique_scene_ids": len(ids),
        "components": rows,
        "errors": errors,
    }
    output = args.output or (DATASET_ROOT / "reports" / f"{tag}_generation_validation.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"status: {report['status']}")
    print(f"primary frames: {primary_frames}")
    print(f"reserve frames: {reserve_frames}")
    print(f"unique scene ids: {len(ids)}")
    print(f"report: {output}")
    if errors:
        for error in errors:
            print(f"error: {error}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
