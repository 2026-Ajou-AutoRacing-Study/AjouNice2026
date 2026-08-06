#!/usr/bin/env python3
"""Generate a small deterministic scenario set for ROS load/capture testing."""

import argparse
import json
import subprocess
import sys
from pathlib import Path


SCENE_SPECS = (
    {
        "id": "p01",
        "stem": "od_poc_01",
        "anchor_index": 500,
        "seed": 26080601,
        "npc_count": 4,
        "pedestrian_count": 3,
        "obstacle_count": 2,
    },
    {
        "id": "p02",
        "stem": "od_poc_02",
        "anchor_index": 2516,
        "seed": 26080602,
        "npc_count": 5,
        "pedestrian_count": 4,
        "obstacle_count": 2,
    },
    {
        "id": "p03",
        "stem": "od_poc_03",
        "anchor_index": 3500,
        "seed": 26080603,
        "npc_count": 4,
        "pedestrian_count": 5,
        "obstacle_count": 3,
    },
)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="Generate three MORAI ROS auto-capture proof-of-concept scenes."
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=project_root,
    )
    parser.add_argument(
        "--install-dir",
        type=Path,
        default=None,
        help=(
            "Optional current-map MORAI scenario directory. When supplied, "
            "generated JSON files are copied there."
        ),
    )
    return parser.parse_args()


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(source.read_bytes())


def main():
    args = parse_args()
    project_root = args.project_root.resolve()
    workspace = project_root / "dataset/morai_od"
    generator = workspace / "scripts/generate_morai_od_prototype.py"
    manifest_path = workspace / "manifests/ros_capture_poc.json"
    scenes = []

    for spec in SCENE_SPECS:
        command = [
            sys.executable,
            str(generator),
            "--output-root",
            str(workspace),
            "--scenario-group",
            "ros_capture_poc",
            "--output-stem",
            spec["stem"],
            "--anchor-index",
            str(spec["anchor_index"]),
            "--seed",
            str(spec["seed"]),
            "--npc-count",
            str(spec["npc_count"]),
            "--pedestrian-count",
            str(spec["pedestrian_count"]),
            "--obstacle-count",
            str(spec["obstacle_count"]),
            "--weather",
            "sunny",
            "--time-hour",
            "13",
        ]
        subprocess.run(command, check=True)

        scenario_rel = Path(
            "dataset/morai_od/scenarios/ros_capture_poc"
        ) / f"{spec['stem']}.json"
        metadata_rel = Path(
            "dataset/morai_od/metadata/ros_capture_poc"
        ) / f"{spec['stem']}.metadata.json"
        metadata = json.loads(
            (project_root / metadata_rel).read_text(encoding="utf-8")
        )
        if args.install_dir is not None:
            copy_file(
                project_root / scenario_rel,
                args.install_dir.expanduser().resolve() / scenario_rel.name,
            )

        scenes.append(
            {
                "id": spec["id"],
                "file_name": spec["stem"],
                "scenario": str(scenario_rel),
                "metadata": str(metadata_rel),
                "anchor_index": spec["anchor_index"],
                "expected_ego": metadata["ego"],
                "expected_counts": {
                    "npc": spec["npc_count"],
                    "pedestrian": spec["pedestrian_count"],
                    "obstacle": spec["obstacle_count"],
                },
            }
        )

    manifest = {
        "schema_version": 1,
        "purpose": "MORAI ROS ScenarioLoad and SaveSensorData proof of concept",
        "map_name": "R_KR_PR_K-city_2025",
        "environment": {
            "weather": "sunny",
            "time_hour": 13,
            "note": "Set once in the MORAI UI before running this PoC.",
        },
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": {
            "captures_per_scene": 6,
            "capture_spacing_m": 0.6,
            "ego_target_speed_kph": 3.6,
            "maximum_motion_m": 4.0,
        },
        "scenes": scenes,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("MORAI ROS capture PoC set generated")
    print(f"manifest: {manifest_path}")
    print(f"scenes: {len(scenes)}")
    if args.install_dir is None:
        print("MORAI install: skipped (--install-dir was not supplied)")
    else:
        print(f"MORAI install: {args.install_dir.expanduser().resolve()}")


if __name__ == "__main__":
    main()
