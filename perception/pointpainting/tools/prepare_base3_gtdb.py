#!/usr/bin/env python3
"""Build and stratify the Base3 PointPainting ground-truth database.

The natural Base3 training index is intentionally left untouched.  Only the
Pedestrian candidates used by GT database sampling are rebalanced so that
single-frame walking/running poses are sampled more often than standing poses.
"""

from __future__ import annotations

import argparse
import json
import pickle
import random
from collections import Counter
from pathlib import Path
from typing import Any

import yaml
from easydict import EasyDict


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--dataset-config", type=Path, required=True)
    parser.add_argument("--metadata-root", type=Path, required=True)
    parser.add_argument(
        "--output-name", default="custom_dbinfos_train_b3s_pp1_v20p10o4.pkl"
    )
    parser.add_argument("--report-name", default="base3_gtdb_pp1_audit.json")
    parser.add_argument("--moving-pedestrian-share", type=float, default=0.60)
    parser.add_argument("--pedestrian-min-points", type=int, default=5)
    parser.add_argument("--vehicle-min-points", type=int, default=5)
    parser.add_argument("--obstacle-min-points", type=int, default=8)
    parser.add_argument("--seed", type=int, default=260830)
    parser.add_argument("--force-raw-database", action="store_true")
    return parser.parse_args()


def sample_id_from_db_info(info: dict[str, Any]) -> str:
    name = str(info["name"])
    filename = Path(info["path"]).name
    marker = f"_{name}_{int(info['gt_idx'])}.bin"
    if not filename.endswith(marker):
        raise ValueError(f"cannot recover sample id from GT DB path: {filename}")
    return filename[: -len(marker)]


def load_dynamic_pedestrian_profiles(metadata_root: Path) -> dict[str, str]:
    profiles: dict[str, str] = {}
    pattern = "static_classcentric_base3_batch008_dynamic_ped*/*.metadata.json"
    for path in sorted(metadata_root.glob(pattern)):
        payload = json.loads(path.read_text(encoding="utf-8"))
        scene_id = str(payload.get("scene_id", ""))
        if not scene_id:
            continue
        actors = [
            actor
            for actor in payload.get("runtime_actors", [])
            if actor.get("class") == "pedestrian"
        ]
        profile = str(actors[0].get("difficulty_profile", "moving")) if actors else "moving"
        profiles[scene_id] = profile
    if not profiles:
        raise ValueError(f"no Batch 008 pedestrian metadata found below {metadata_root}")
    return profiles


def dynamic_profile(sample_id: str, profiles: dict[str, str]) -> str | None:
    scene_id = sample_id.rsplit("_f", 1)[0]
    return profiles.get(scene_id)


def filter_min_points(
    entries: list[dict[str, Any]], minimum: int
) -> list[dict[str, Any]]:
    return [entry for entry in entries if int(entry["num_points_in_gt"]) >= minimum]


def ensure_raw_database(
    root: Path, config_path: Path, force: bool
) -> Path:
    raw_path = root / "custom_dbinfos_train.pkl"
    database_dir = root / "gt_database"
    if raw_path.exists() and database_dir.is_dir() and not force:
        return raw_path

    from pcdet.datasets.custom.custom_dataset import CustomDataset
    from pcdet.utils import common_utils

    dataset_cfg = EasyDict(yaml.safe_load(config_path.read_text(encoding="utf-8")))
    class_names = list(dataset_cfg.MAP_CLASS_TO_KITTI)
    dataset = CustomDataset(
        dataset_cfg=dataset_cfg,
        class_names=class_names,
        root_path=root,
        training=False,
        logger=common_utils.create_logger(),
    )
    info_path = root / "custom_infos_train.pkl"
    if not info_path.exists():
        raise FileNotFoundError(f"Base3 train info is missing: {info_path}")
    dataset.create_groundtruth_database(
        info_path=info_path, used_classes=class_names, split="train"
    )
    return raw_path


def main() -> None:
    args = parse_args()
    root = args.dataset_root.expanduser().resolve()
    config_path = args.dataset_config.expanduser().resolve()
    metadata_root = args.metadata_root.expanduser().resolve()
    if not 0.0 < args.moving_pedestrian_share < 1.0:
        raise ValueError("--moving-pedestrian-share must be between zero and one")

    raw_path = ensure_raw_database(root, config_path, args.force_raw_database)
    with raw_path.open("rb") as stream:
        raw: dict[str, list[dict[str, Any]]] = pickle.load(stream)

    thresholds = {
        "Pedestrian": args.pedestrian_min_points,
        "Vehicle": args.vehicle_min_points,
        "Obstacle": args.obstacle_min_points,
    }
    filtered = {
        name: filter_min_points(list(raw.get(name, [])), minimum)
        for name, minimum in thresholds.items()
    }

    profiles = load_dynamic_pedestrian_profiles(metadata_root)
    moving: list[dict[str, Any]] = []
    standing: list[dict[str, Any]] = []
    moving_profiles: Counter[str] = Counter()
    for entry in filtered["Pedestrian"]:
        sample_id = sample_id_from_db_info(entry)
        profile = dynamic_profile(sample_id, profiles)
        enriched = dict(entry)
        if profile is None:
            enriched["pedestrian_pose_group"] = "standing"
            standing.append(enriched)
        else:
            enriched["pedestrian_pose_group"] = "moving"
            enriched["pedestrian_motion_profile"] = profile
            moving.append(enriched)
            moving_profiles[profile] += 1

    if not moving or not standing:
        raise ValueError(
            f"invalid Pedestrian strata: moving={len(moving)}, standing={len(standing)}"
        )

    # Keep every scarce moving-pose crop and deterministically downsample the
    # abundant standing pool.  This avoids duplicate DB entries and therefore
    # avoids collision rejection caused by selecting the same crop twice.
    target_standing = round(
        len(moving)
        * (1.0 - args.moving_pedestrian_share)
        / args.moving_pedestrian_share
    )
    target_standing = min(target_standing, len(standing))
    rng = random.Random(args.seed)
    selected_standing = rng.sample(standing, target_standing)
    pedestrian_pool = moving + selected_standing
    rng.shuffle(pedestrian_pool)

    output = {
        "Pedestrian": pedestrian_pool,
        "Vehicle": filtered["Vehicle"],
        "Obstacle": filtered["Obstacle"],
    }
    output_path = root / args.output_name
    with output_path.open("wb") as stream:
        pickle.dump(output, stream)

    actual_share = len(moving) / len(pedestrian_pool)
    report = {
        "schema_version": 1,
        "dataset_root": str(root),
        "raw_db_info": str(raw_path),
        "output_db_info": str(output_path),
        "seed": args.seed,
        "minimum_points": thresholds,
        "sampling_targets_per_scene": {
            "Vehicle": 20,
            "Pedestrian": 10,
            "Obstacle": 4,
        },
        "raw_counts": {name: len(raw.get(name, [])) for name in thresholds},
        "filtered_counts": {name: len(entries) for name, entries in filtered.items()},
        "pedestrian_pool": {
            "moving": len(moving),
            "standing": len(selected_standing),
            "total": len(pedestrian_pool),
            "moving_share": actual_share,
            "moving_profiles": dict(sorted(moving_profiles.items())),
            "contract": "statistical pool share; natural Base3 train labels are unchanged",
        },
    }
    report_path = root / args.report_name
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
