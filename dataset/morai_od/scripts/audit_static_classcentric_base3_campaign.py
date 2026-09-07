#!/usr/bin/env python3
"""Audit completed Base3 capture runs and emit exact replacement quota keys."""

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_jsonl(path):
    if not Path(path).is_file():
        return []
    with Path(path).open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def quota_key(metadata):
    spatial = metadata.get("spatial_sampling") or {}
    return {
        "family": metadata.get("mission_family"),
        "split": metadata.get("split"),
        "spatial_stratum": spatial.get("spatial_stratum") or "unspecified",
        "highway_phase_variant": spatial.get("highway_phase_variant"),
        "ordinary_two_lane_side": spatial.get("ordinary_two_lane_side"),
        "roundabout_phase": spatial.get("roundabout_phase"),
    }


def compact_key(row):
    return "|".join(
        str(row.get(name) or "-")
        for name in (
            "family", "split", "spatial_stratum",
            "highway_phase_variant", "ordinary_two_lane_side",
            "roundabout_phase",
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", action="append", type=Path, required=True)
    parser.add_argument("--run-dir", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if len(args.manifest) != len(args.run_dir):
        raise ValueError("--manifest and --run-dir counts must match")

    requested = Counter()
    completed = Counter()
    failed = Counter()
    replacement_requests = []
    batches = []
    for manifest_path, run_dir in zip(args.manifest, args.run_dir):
        manifest = load_json(manifest_path)
        metadata_by_id = {}
        for entry in manifest.get("scenes", []):
            metadata = load_json(resolve(entry["metadata"]))
            metadata_by_id[entry["id"]] = metadata
            requested[compact_key(quota_key(metadata))] += 1

        latest = {}
        for row in load_jsonl(run_dir / "scenes.jsonl"):
            latest[row.get("id")] = row
        batch_completed = 0
        batch_failed = 0
        missing = []
        for scene_id, metadata in metadata_by_id.items():
            row = latest.get(scene_id)
            key = quota_key(metadata)
            encoded = compact_key(key)
            if row and row.get("status") == "completed":
                completed[encoded] += 1
                batch_completed += 1
            else:
                failed[encoded] += 1
                batch_failed += 1
                replacement_requests.append({
                    "source_scene_id": scene_id,
                    "source_manifest": str(manifest_path.resolve()),
                    "source_run_dir": str(run_dir.resolve()),
                    "status": row.get("status") if row else "missing",
                    "error": row.get("error") if row else "no scene record",
                    "quota": key,
                })
                missing.append(scene_id)
        batches.append({
            "dataset_id": manifest.get("dataset_id"),
            "manifest": str(manifest_path.resolve()),
            "run_dir": str(run_dir.resolve()),
            "requested_clips": len(metadata_by_id),
            "completed_clips": batch_completed,
            "replacement_clips": batch_failed,
            "missing_or_failed_scene_ids": missing,
        })

    keys = sorted(set(requested) | set(completed) | set(failed))
    quota_rows = [
        {
            "quota_key": key,
            "requested": requested[key],
            "completed": completed[key],
            "deficit": requested[key] - completed[key],
        }
        for key in keys
    ]
    report = {
        "schema_version": 1,
        "requested_clips": sum(requested.values()),
        "completed_clips": sum(completed.values()),
        "replacement_clip_count": len(replacement_requests),
        "batches": batches,
        "quota_inventory": quota_rows,
        "replacement_requests": replacement_requests,
        "status": "complete" if not replacement_requests else "needs_replacement",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("requested/completed: {}/{}".format(
        report["requested_clips"], report["completed_clips"]
    ))
    print("replacement clips: {}".format(report["replacement_clip_count"]))
    print("status: {}".format(report["status"]))
    print("report: {}".format(args.output))


if __name__ == "__main__":
    main()
