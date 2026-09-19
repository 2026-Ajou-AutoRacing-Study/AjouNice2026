#!/usr/bin/env python3
"""Load several Base3-Online samples through the real BEVFusion pipeline."""

import argparse
import json
from pathlib import Path

from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import DATASETS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--ann-file", required=True)
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args()

    config = Config.fromfile(args.config)
    custom = config.get("custom_imports")
    if custom:
        import_modules_from_strings(**custom)
    init_default_scope(config.get("default_scope", "mmdet3d"))
    dataset_config = config.train_dataloader.dataset.copy()
    dataset_config["ann_file"] = args.ann_file
    dataset_config["filter_empty_gt"] = False
    dataset = DATASETS.build(dataset_config)
    rows = []
    for index in range(min(args.samples, len(dataset))):
        sample = dataset[index]
        inputs = sample["inputs"]
        data_sample = sample["data_samples"]
        points = inputs["points"]
        images = inputs["img"]
        gt = data_sample.gt_instances_3d
        rows.append(
            {
                "index": index,
                "sample_idx": data_sample.metainfo.get("sample_idx"),
                "point_shape": list(points.shape),
                "image_shape": list(images.shape),
                "gt_boxes": int(len(gt.bboxes_3d)),
                "gt_labels": gt.labels_3d.tolist(),
            }
        )
    print(json.dumps({"valid": True, "dataset_size": len(dataset), "samples": rows}, indent=2))


if __name__ == "__main__":
    main()
