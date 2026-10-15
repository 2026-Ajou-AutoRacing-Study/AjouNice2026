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
    parser.add_argument("--pipeline", choices=('train', 'test'), default='train')
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    config = Config.fromfile(args.config)
    custom = config.get("custom_imports")
    if custom:
        import_modules_from_strings(**custom)
    init_default_scope(config.get("default_scope", "mmdet3d"))
    dataset_config = config.train_dataloader.dataset.copy()
    dataset_config["ann_file"] = args.ann_file
    dataset_config["filter_empty_gt"] = False
    if args.pipeline == 'test':
        dataset_config['pipeline'] = config.test_pipeline
    dataset = DATASETS.build(dataset_config)
    rows = []
    for index in range(min(args.samples, len(dataset))):
        sample = dataset[index]
        inputs = sample["inputs"]
        data_sample = sample["data_samples"]
        points = inputs["points"]
        images = inputs["img"]
        gt = data_sample.gt_instances_3d
        # Inference pipelines intentionally do not pack labels. Read the
        # unaugmented dataset annotations for a diagnostic count, not metrics.
        if args.pipeline == 'test':
            annotation = dataset.get_ann_info(index)
            box_count = len(annotation['gt_bboxes_3d'])
            labels = annotation['gt_labels_3d'].tolist()
        else:
            box_count = len(gt.bboxes_3d)
            labels = gt.labels_3d.tolist()
        rows.append(
            {
                "index": index,
                "sample_idx": data_sample.metainfo.get("sample_idx"),
                "point_shape": list(points.shape),
                "image_shape": list(images.shape),
                "gt_boxes": int(box_count),
                "gt_labels": labels,
            }
        )
    report = {"valid": True, "pipeline": args.pipeline, "dataset_size": len(dataset), "samples": rows}
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(valid=True, dataset_size=len(dataset), tested=len(rows),
                             retained_gt=sum(row['gt_boxes'] for row in rows),
                             zero_gt_frames=sum(row['gt_boxes'] == 0 for row in rows),
                             output=str(args.json_output)), indent=2))
    else:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
