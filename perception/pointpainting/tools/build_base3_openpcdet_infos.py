#!/usr/bin/env python3
"""Generate and audit Base3 OpenPCDet info files without a GT database."""

from __future__ import annotations

import argparse
import json
import pickle
from collections import Counter
from pathlib import Path

import yaml
from easydict import EasyDict


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--dataset-config", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.dataset_root.expanduser().resolve()
    config_path = args.dataset_config.expanduser().resolve()
    dataset_cfg = EasyDict(yaml.safe_load(config_path.read_text(encoding="utf-8")))

    from pcdet.datasets.custom.custom_dataset import CustomDataset
    from pcdet.utils import common_utils

    class_names = list(dataset_cfg.MAP_CLASS_TO_KITTI)
    dataset = CustomDataset(
        dataset_cfg=dataset_cfg,
        class_names=class_names,
        root_path=root,
        training=False,
        logger=common_utils.create_logger(),
    )
    feature_count = len(dataset_cfg.POINT_FEATURE_ENCODING.src_feature_list)
    report = {
        "schema_version": 1,
        "dataset_root": str(root),
        "dataset_config": str(config_path),
        "minimum_gt_points": int(dataset_cfg.get("MIN_GT_POINTS", 0)),
        "point_count_box_tolerance_m": float(
            dataset_cfg.get("GT_POINT_COUNT_BOX_TOLERANCE_M", 0.0)
        ),
        "splits": {},
    }

    for split in ("train", "val", "test"):
        dataset.set_split(split)
        sample_ids = list(dataset.sample_id_list or [])
        infos = dataset.get_infos(
            class_names,
            num_workers=args.workers,
            has_label=True,
            sample_id_list=sample_ids,
            num_features=feature_count,
        )
        output = root / f"custom_infos_{split}.pkl"
        with output.open("wb") as stream:
            pickle.dump(infos, stream)

        kept = Counter()
        rejected = Counter()
        minimum_kept = None
        for info in infos:
            annos = info["annos"]
            kept.update(str(name) for name in annos["name"])
            counts = [int(value) for value in annos.get("num_points_in_gt", [])]
            if counts:
                candidate = min(counts)
                minimum_kept = candidate if minimum_kept is None else min(minimum_kept, candidate)
            sparse = info.get("sparse_gt", {})
            rejected.update(str(name) for name in sparse.get("name", []))

        report["splits"][split] = {
            "frames": len(infos),
            "kept_gt": dict(sorted(kept.items())),
            "rejected_sparse_gt": dict(sorted(rejected.items())),
            "minimum_kept_points": minimum_kept,
            "info_path": str(output),
        }

    report_path = root / "base3_openpcdet_info_audit.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
