#!/usr/bin/env python3
"""Generate one parameterized static or static-hard MORAI component set."""

import argparse
import json
import sys
from pathlib import Path

import yaml

import generate_sunny13_600_production_v6_set as production_v6


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CAPTURE_CONFIG = (
    PROJECT_ROOT / "dataset/morai_od/configs/od_static_random_1998_v1.yaml"
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component-spec", type=Path, required=True)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--capture-config", type=Path, default=DEFAULT_CAPTURE_CONFIG)
    parser.add_argument("--asset-scenario-dir", type=Path)
    parser.add_argument("--runtime-asset-scenario-dir", type=Path)
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def build_scene_specs(spec):
    split_sequence = []
    for split in ("train", "val", "test"):
        split_sequence.extend([split] * int(spec["split_counts"][split]))
    if len(split_sequence) != int(spec["scenario_count"]):
        raise ValueError("split_counts do not sum to scenario_count")

    cursor = 0
    values = []
    for scene_type in spec["scene_types"]:
        count = int(scene_type["count"])
        values.append(
            {
                **scene_type,
                "count": count,
                "splits": tuple(split_sequence[cursor : cursor + count]),
                "npc_range": tuple(scene_type["npc_range"]),
                "pedestrian_range": tuple(scene_type["pedestrian_range"]),
                "obstacle_range": tuple(scene_type["obstacle_range"]),
                "topology_preference": tuple(scene_type["topology_preference"]),
            }
        )
        cursor += count
    if cursor != int(spec["scenario_count"]):
        raise ValueError("scene type counts do not sum to scenario_count")
    return tuple(values)


def main():
    args = parse_args()
    spec = load_json(args.component_spec)
    generator = production_v6.generator
    generator.DATASET_ID = spec["dataset_id"]
    generator.SCENARIO_GROUP = spec["scenario_group"]
    generator.MANIFEST_STEM = spec["manifest_stem"]
    generator.SCENE_ID_PREFIX = spec["scene_id_prefix"]
    generator.SCENARIO_FILE_PREFIX = spec["scenario_file_prefix"]
    generator.BASE_SCENARIO_COUNT = int(spec["scenario_count"])
    generator.SPLIT_COUNTS = {
        key: int(value) for key, value in spec["split_counts"].items()
    }
    generator.MASTER_SEED = int(spec["seed"])
    generator.ALLOW_ANCHOR_REUSE = True
    generator.ENVIRONMENTS = (
        {"id": "sunny_t13", "weather": "sunny", "time_hour": 13},
    )
    generator.SCENE_TYPE_SPECS = build_scene_specs(spec)

    # The base generator intentionally rejects a stale capture YAML. Derive a
    # component-local copy so its declared counts remain truthful instead of
    # weakening that validation globally.
    capture_config = yaml.safe_load(args.capture_config.read_text(encoding="utf-8"))
    count = int(spec["scenario_count"])
    frames_per_scene = int(capture_config["capture"]["frames_per_run"])
    capture_config["capture"]["total_runs"] = count
    capture_config["capture"]["unique_geometry_layouts"] = count
    capture_config["capture"]["environment_variants_per_layout"] = 1
    capture_config["capture"]["total_multisensor_frames"] = count * frames_per_scene
    capture_config["environment"]["runs_per_variant"] = count
    capture_config["split"].update(spec["split_counts"])
    derived_config = args.component_spec.with_suffix(".capture.yaml")
    derived_config.write_text(
        yaml.safe_dump(capture_config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    forwarded = [sys.argv[0], "--config", str(derived_config), "--seed", str(spec["seed"])]
    if args.install_dir is not None:
        forwarded.extend(["--install-dir", str(args.install_dir)])
    if args.asset_scenario_dir is not None:
        forwarded.extend(["--asset-scenario-dir", str(args.asset_scenario_dir)])
    if args.runtime_asset_scenario_dir is not None:
        forwarded.extend(
            ["--runtime-asset-scenario-dir", str(args.runtime_asset_scenario_dir)]
        )
    sys.argv = forwarded
    generator.main()


if __name__ == "__main__":
    main()
