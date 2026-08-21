#!/usr/bin/env python3
"""Classify interleaved MORAI asset probes separately from ROS failures."""

import argparse
import json
from collections import defaultdict
from pathlib import Path


def load(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_failures(run_dir):
    failures = {}
    path = run_dir / "failures.jsonl"
    if not path.exists():
        return failures
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            failures[row["id"]] = row.get("error", "")
    return failures


def run_status(run_dir):
    summary = load(run_dir / "summary.json")
    completed = set(summary["completed_scene_ids"])
    failed = set(summary["failed_scene_ids"])
    return completed, failed, read_failures(run_dir)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("run_dirs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    manifest = load(args.manifest)
    scene_by_id = {scene["id"]: scene for scene in manifest["scenes"]}
    grouped = defaultdict(list)

    for run_index, run_dir in enumerate(args.run_dirs, 1):
        completed, failed, errors = run_status(run_dir)
        selected = completed | failed
        for scene_id in selected:
            scene = scene_by_id[scene_id]
            probe = scene["asset_probe"]
            key = (probe["group"], int(probe["candidate_data_id"]))
            grouped[key].append(
                {
                    "run_index": run_index,
                    "trial_index": int(probe["trial_index"]),
                    "role": probe["role"],
                    "scene_id": scene_id,
                    "status": "completed" if scene_id in completed else "failed",
                    "error": errors.get(scene_id),
                }
            )

    results = []
    for (group, data_id), observations in sorted(grouped.items()):
        sample_scene = next(
            scene for scene in manifest["scenes"]
            if int(scene["asset_probe"]["candidate_data_id"]) == data_id
            and scene["asset_probe"]["group"] == group
        )
        probe_info = sample_scene["asset_probe"]
        trials = []
        for run_index in sorted({row["run_index"] for row in observations}):
            run_rows = [row for row in observations if row["run_index"] == run_index]
            for trial_index in sorted({row["trial_index"] for row in run_rows}):
                pair = {
                    row["role"]: row
                    for row in run_rows
                    if row["trial_index"] == trial_index
                }
                control = pair.get("control")
                probe = pair.get("probe")
                if not control or control["status"] != "completed":
                    classification = "inconclusive_control_or_network_failure"
                elif probe and probe["status"] == "completed":
                    classification = "asset_loaded"
                elif probe:
                    classification = "asset_missing_with_healthy_control"
                else:
                    classification = "incomplete_pair"
                trials.append(
                    {
                        "run_index": run_index,
                        "trial_index": trial_index,
                        "control_scene": control and control["scene_id"],
                        "control_status": control and control["status"],
                        "probe_scene": probe and probe["scene_id"],
                        "probe_status": probe and probe["status"],
                        "classification": classification,
                    }
                )

        classifications = [row["classification"] for row in trials]
        if classifications and all(value == "asset_loaded" for value in classifications):
            decision = "rehabilitate_for_personal_automatic_capture"
        elif classifications and all(
            value == "asset_missing_with_healthy_control"
            for value in classifications
        ):
            decision = "confirmed_personal_launcher_incompatible"
        elif any("inconclusive" in value for value in classifications):
            decision = "inconclusive_retest_after_network_recovery"
        else:
            decision = "intermittent_manual_review"
        results.append(
            {
                "group": group,
                "data_id": data_id,
                "name": probe_info["candidate_name"],
                "template_source": probe_info["candidate_template_source"],
                "decision": decision,
                "trials": trials,
            }
        )

    report = {
        "schema_version": 1,
        "manifest": str(args.manifest),
        "run_dirs": [str(path) for path in args.run_dirs],
        "method": (
            "Every probe is preceded by a known-good same-type control. "
            "A missing probe is asset-specific only when its control loaded."
        ),
        "results": results,
    }
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(args.output)
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
