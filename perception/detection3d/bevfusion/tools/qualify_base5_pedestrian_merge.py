#!/usr/bin/env python3
"""Read-only whole-split semantic/geometry check and perfect-GT metric oracle."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import pickle

import numpy as np
import torch
from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmengine.runner import Runner
from mmengine.structures import InstanceData
from mmdet3d.registry import METRICS
from prepare_base5_portable_dataset import validate, sha256


def qualify(source, data, config, output):
    source, data = source.resolve(strict=True), data.resolve(strict=True)
    before, after = validate(source), validate(data)
    if after.get('training_label_policy') != 'animal_to_pedestrian_v1':
        raise ValueError('Not the merged training dataset')
    # All payload except the derived infos is byte-identical by inventory.
    for name, item in before['files'].items():
        if name.endswith('.pkl') and name.startswith('infos/'):
            continue
        if after['files'][name] != item:
            raise ValueError('Payload identity changed: ' + name)
    frames = {}
    for profile in before['profiles']:
        for split in ('train', 'val', 'test'):
            name = f'infos/{profile}/morai_base5_infos_{split}.pkl'
            old = pickle.loads((source / name).read_bytes())
            new = pickle.loads((data / name).read_bytes())
            restored = deepcopy(new)
            restored['metainfo'].pop('training_label_policy')
            restored['metainfo'].pop('source_classes')
            restored['metainfo']['classes'] = old['metainfo']['classes']
            for row in restored['data_list']:
                for instance in row['instances']:
                    instance['bbox_label_3d'] = instance.pop('original_bbox_label_3d')
                    instance.pop('original_semantic_class')
            if restored != old:
                raise ValueError('Non-label data changed: ' + name)
            frames[profile + ':' + split] = len(new['data_list'])
    cfg = Config.fromfile(str(config))
    init_default_scope('mmdet3d')
    loader_cfg = cfg.val_dataloader
    loader_cfg.num_workers = 0
    loader_cfg.persistent_workers = False
    loader_cfg.dataset.data_root = str(data)
    loader_cfg.dataset.data_prefix = dict(pts='', img='')
    loader_cfg.dataset.ann_file = str(data / 'infos/512x1056/morai_base5_infos_val.pkl')
    dataset = Runner.build_dataloader(loader_cfg).dataset
    metric = METRICS.build(cfg.val_evaluator)
    metric.dataset_meta = dataset.metainfo
    for index in range(len(dataset)):
        ann = dataset.get_ann_info(index)
        if len(ann['original_semantic_class']) != len(ann['gt_labels_3d']):
            raise ValueError('Dataset lost subgroup alignment')
        metric.process({}, [dict(eval_ann_info=ann, pred_instances_3d=InstanceData(
            bboxes_3d=ann['gt_bboxes_3d'],
            labels_3d=torch.as_tensor(ann['gt_labels_3d']),
            scores_3d=torch.ones(len(ann['gt_labels_3d']))))])
    scores = metric.compute_metrics(metric.results)
    if any(value != 1. for key, value in scores.items() if key.startswith('subgroup/') and 'recall_' in key):
        raise ValueError('Perfect-GT subgroup oracle failed')
    mean_ap = scores['center/mAP']
    if not np.isclose(mean_ap, 1.):
        raise ValueError('Perfect-GT three-class AP oracle failed')
    report = dict(valid=True, source_sha256=sha256(source / 'manifest.json'),
                  merged_sha256=sha256(data / 'manifest.json'), frames=frames,
                  validation_frames=len(dataset), scores=scores,
                  scope='No model or optimizer updates; oracle is a label/metric integrity check, not learned accuracy')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'scores'}, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'data', 'config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    qualify(args.source, args.data, args.config, args.output)
