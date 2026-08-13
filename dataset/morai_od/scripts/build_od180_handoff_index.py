#!/usr/bin/env python3
"""Validate six completed OD180 batches and build a portable sample index."""

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path


ENVIRONMENT_IDS = (
    "sunny_t11",
    "sunny_t13",
    "sunny_t15",
    "foggy_t11",
    "foggy_t13",
    "foggy_t15",
)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=project_root)
    return parser.parse_args()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def select_complete_report(environment_root, environment_id):
    candidates = sorted(
        environment_root.rglob("ros_capture_poc_report.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    diagnostics = []
    for path in candidates:
        report = load_json(path)
        summary = report.get("summary", {})
        environment = report.get("environment", {})
        valid = (
            report.get("dataset_id") == "morai_od180_v1"
            and environment.get("id") == environment_id
            and summary.get("requested_scenes") == 30
            and summary.get("completed_scenes") == 30
            and summary.get("failed_scenes") == 0
            and summary.get("completed_captures") == 180
        )
        diagnostics.append(
            {
                "path": str(path),
                "valid": valid,
                "summary": summary,
                "environment": environment,
            }
        )
        if valid:
            return path, report, diagnostics
    raise RuntimeError(
        "No complete 30-scene/180-frame report for {} under {}. "
        "Candidates: {}".format(
            environment_id,
            environment_root,
            json.dumps(diagnostics, ensure_ascii=False),
        )
    )


def group_files(capture_root, report_dir, relative_files):
    grouped = {}
    for relative in relative_files:
        source = report_dir / relative
        if not source.is_file() or source.stat().st_size <= 0:
            raise FileNotFoundError(source)
        path_from_root = str(source.relative_to(capture_root))
        sensor = Path(relative).parts[0]
        grouped.setdefault(sensor, []).append(path_from_root)
    return {key: sorted(value) for key, value in sorted(grouped.items())}


def copy_metadata_bundle(project_root, capture_root):
    destination = capture_root / "handoff_metadata"
    destination.mkdir(parents=True, exist_ok=True)
    files = (
        "dataset/morai_od/configs/morai_asset_catalog_260807.json",
        "dataset/morai_od/configs/od_180_pilot.yaml",
        "dataset/morai_od/manifests/od180_base.json",
        "dataset/morai_od/manifests/od180_runs.jsonl",
        "dataset/morai_od/manifests/od180_train.txt",
        "dataset/morai_od/manifests/od180_val.txt",
        "dataset/morai_od/manifests/od180_test.txt",
        "dataset/morai_od/reports/od180_asset_coverage.json",
        "dataset/morai_od/reports/od180_generation_summary.json",
        "configs/sensors/Sensor_6.json",
    )
    for relative in files:
        source = project_root / relative
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for relative_dir in (
        "dataset/morai_od/scenarios/od180_base",
        "dataset/morai_od/metadata/od180_base",
    ):
        source = project_root / relative_dir
        target = destination / relative_dir
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(source, target)
    return destination


def main():
    args = parse_args()
    project_root = args.project_root.resolve()
    capture_root = args.capture_root.expanduser().resolve()
    samples = []
    selected_reports = []

    for environment_id in ENVIRONMENT_IDS:
        environment_root = capture_root / "captures" / environment_id
        report_path, report, _diagnostics = select_complete_report(
            environment_root, environment_id
        )
        selected_reports.append(str(report_path.relative_to(capture_root)))
        for scene in report["scenes"]:
            if scene.get("status") != "completed":
                raise RuntimeError(
                    "incomplete scene in selected report: {}".format(
                        scene.get("id")
                    )
                )
            for frame_index, capture in enumerate(scene["captures"]):
                if not capture.get("verified"):
                    raise RuntimeError(
                        "unverified capture: {}".format(
                            capture.get("capture_id")
                        )
                    )
                sample_id = "{}_{}_f{:02d}".format(
                    scene["base_scenario_id"],
                    environment_id,
                    frame_index,
                )
                samples.append(
                    {
                        "sample_id": sample_id,
                        "base_scenario_id": scene["base_scenario_id"],
                        "environment_id": environment_id,
                        "split": scene["split"],
                        "frame_index": frame_index,
                        "capture_id": capture["capture_id"],
                        "files": group_files(
                            capture_root,
                            report_path.parent,
                            capture["new_files"],
                        ),
                    }
                )

    if len(samples) != 1080:
        raise RuntimeError(
            "expected 1080 paired samples, found {}".format(len(samples))
        )
    if len({item["sample_id"] for item in samples}) != len(samples):
        raise RuntimeError("duplicate sample_id in handoff index")
    split_counts = Counter(item["split"] for item in samples)
    if split_counts != Counter({"train": 720, "val": 180, "test": 180}):
        raise RuntimeError("unexpected sample split counts: {}".format(split_counts))

    index_path = capture_root / "samples.jsonl"
    with index_path.open("w", encoding="utf-8") as stream:
        for sample in samples:
            stream.write(json.dumps(sample, ensure_ascii=False) + "\n")

    metadata_bundle = copy_metadata_bundle(project_root, capture_root)
    handoff = {
        "schema_version": 1,
        "dataset_id": "morai_od180_v1",
        "base_scenarios": 30,
        "environment_variants": 6,
        "samples": len(samples),
        "split_samples": dict(sorted(split_counts.items())),
        "selected_reports": selected_reports,
        "sample_index": str(index_path.relative_to(capture_root)),
        "metadata_bundle": str(metadata_bundle.relative_to(capture_root)),
        "leakage_rule": (
            "All six environment variants and all frames of one "
            "base_scenario_id stay in the same split."
        ),
    }
    handoff_path = capture_root / "handoff_manifest.json"
    handoff_path.write_text(
        json.dumps(handoff, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("OD180 handoff index complete")
    print("samples:", len(samples))
    print("split:", dict(sorted(split_counts.items())))
    print("manifest:", handoff_path)
    print("sample index:", index_path)
    print("metadata:", metadata_bundle)


if __name__ == "__main__":
    main()
