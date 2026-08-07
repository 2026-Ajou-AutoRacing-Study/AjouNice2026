#!/usr/bin/env python3
"""Build a partial-success handoff index for the Sunny 13:00 smoke set."""

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path


DATASET_ID = "morai_sunny13_180_smoke_v1"


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report",
        type=Path,
        required=True,
        help="ros_capture_poc_report.json produced by the capture runner.",
    )
    parser.add_argument("--project-root", type=Path, default=project_root)
    return parser.parse_args()


def write_json(path, document):
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path, rows):
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")


def group_files(report_dir, relative_files):
    grouped = {}
    for relative in relative_files:
        path = report_dir / relative
        if not path.is_file() or path.stat().st_size <= 0:
            raise FileNotFoundError(path)
        sensor = Path(relative).parts[0]
        grouped.setdefault(sensor, []).append(relative)
    return {key: sorted(value) for key, value in sorted(grouped.items())}


def copy_metadata(project_root, destination):
    metadata_root = destination / "handoff_metadata"
    files = (
        "dataset/morai_od/configs/od_sunny13_180_smoke.yaml",
        "dataset/morai_od/configs/morai_asset_catalog_260807.json",
        "dataset/morai_od/manifests/sunny13_180_smoke_sunny_t13.json",
        "dataset/morai_od/manifests/sunny13_180_smoke_runs.jsonl",
        "dataset/morai_od/manifests/sunny13_180_smoke_train.txt",
        "dataset/morai_od/manifests/sunny13_180_smoke_val.txt",
        "dataset/morai_od/manifests/sunny13_180_smoke_test.txt",
        "dataset/morai_od/reports/sunny13_180_smoke_asset_coverage.json",
        "dataset/morai_od/reports/sunny13_180_smoke_generation_summary.json",
        "configs/sensors/Sensor_6.json",
    )
    for relative in files:
        source = project_root / relative
        target = metadata_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for relative in (
        "dataset/morai_od/scenarios/sunny13_180_smoke",
        "dataset/morai_od/metadata/sunny13_180_smoke",
    ):
        source = project_root / relative
        target = metadata_root / relative
        if target.exists():
            shutil.rmtree(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, target)
    return metadata_root


def main():
    args = parse_args()
    project_root = args.project_root.expanduser().resolve()
    report_path = args.report.expanduser().resolve()
    report_dir = report_path.parent
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("dataset_id") != DATASET_ID:
        raise ValueError(
            "unexpected dataset_id: {}".format(report.get("dataset_id"))
        )
    expected_manifest = json.loads(
        (
            project_root
            / (
                "dataset/morai_od/manifests/"
                "sunny13_180_smoke_sunny_t13.json"
            )
        ).read_text(encoding="utf-8")
    )
    expected_by_id = {
        scene["id"]: scene for scene in expected_manifest["scenes"]
    }

    samples = []
    failures = []
    successful_scenes = Counter()
    for scene in report["scenes"]:
        if scene.get("status") != "completed":
            failures.append(
                {
                    "scenario_id": scene.get("id"),
                    "base_scenario_id": scene.get("base_scenario_id"),
                    "file_name": scene.get("file_name"),
                    "split": scene.get("split"),
                    "error": scene.get("error", "unknown"),
                }
            )
            continue
        captures = scene.get("captures", [])
        if len(captures) != 6 or not all(
            capture.get("verified") for capture in captures
        ):
            failures.append(
                {
                    "scenario_id": scene.get("id"),
                    "base_scenario_id": scene.get("base_scenario_id"),
                    "file_name": scene.get("file_name"),
                    "split": scene.get("split"),
                    "error": (
                        "completed scene does not contain six verified frames"
                    ),
                }
            )
            continue
        successful_scenes[scene["split"]] += 1
        for frame_index, capture in enumerate(captures):
            samples.append(
                {
                    "sample_id": "{}_sunny_t13_f{:02d}".format(
                        scene["base_scenario_id"], frame_index
                    ),
                    "base_scenario_id": scene["base_scenario_id"],
                    "environment_id": "sunny_t13",
                    "split": scene["split"],
                    "frame_index": frame_index,
                    "capture_id": capture["capture_id"],
                    "morai_timestamp_stem": capture.get(
                        "morai_timestamp_stem"
                    ),
                    "files": group_files(
                        report_dir, capture.get("new_files", [])
                    ),
                }
            )
    attempted_ids = {scene.get("id") for scene in report["scenes"]}
    for scenario_id, scene in expected_by_id.items():
        if scenario_id in attempted_ids:
            continue
        failures.append(
            {
                "scenario_id": scenario_id,
                "base_scenario_id": scene.get("base_scenario_id"),
                "file_name": scene.get("file_name"),
                "split": scene.get("split"),
                "error": "not_attempted_before_process_end",
            }
        )

    sample_split_counts = Counter(item["split"] for item in samples)
    write_jsonl(report_dir / "samples.jsonl", samples)
    write_jsonl(report_dir / "failed_scenarios.jsonl", failures)
    metadata_root = copy_metadata(project_root, report_dir)
    handoff = {
        "schema_version": 1,
        "dataset_id": DATASET_ID,
        "environment": {"weather": "sunny", "time_hour": 13},
        "requested_scenarios": len(expected_by_id),
        "attempted_scenarios": len(report["scenes"]),
        "successful_scenarios": sum(successful_scenes.values()),
        "failed_scenarios": len(failures),
        "successful_scenarios_by_split": dict(
            sorted(successful_scenes.items())
        ),
        "successful_samples": len(samples),
        "successful_samples_by_split": dict(
            sorted(sample_split_counts.items())
        ),
        "sample_index": "samples.jsonl",
        "failure_index": "failed_scenarios.jsonl",
        "source_report": report_path.name,
        "metadata_bundle": str(metadata_root.relative_to(report_dir)),
        "policy": (
            "Partial success is accepted for this smoke dataset. Failed or "
            "incomplete scenarios are excluded from samples.jsonl and listed "
            "in failed_scenarios.jsonl."
        ),
    }
    write_json(report_dir / "handoff_manifest.json", handoff)
    print("Sunny 13 handoff complete")
    print("successful scenarios:", sum(successful_scenes.values()))
    print("failed scenarios:", len(failures))
    print("successful samples:", len(samples))
    print("split samples:", dict(sorted(sample_split_counts.items())))
    print("manifest:", report_dir / "handoff_manifest.json")


if __name__ == "__main__":
    main()
