#!/usr/bin/env python3
"""Real PNG/points/GT CPU pipeline check, not model accuracy or GPU cost."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
from mmengine.config import Config
from mmengine.dataset import Compose
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import DATASETS
from mmdet3d.utils import register_all_modules
import torch


def diagnostic_pipeline(pipeline):
    # Freeze geometry ONLY for this comparison: new RNG draws must not make
    # unrelated BEV augmentation differ between the two pixel-domain paths.
    pipeline = deepcopy(pipeline)
    result = []
    for transform in pipeline:
        if transform['type'] == 'PointShuffle':
            continue
        if transform['type'] == 'BEVFusionGlobalRotScaleTrans':
            transform.update(scale_ratio_range=[1., 1.], rot_range=[0., 0.], translation_std=0.)
        result.append(transform)
    return Compose(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--per-split', type=int, default=24)
    args = parser.parse_args()
    if args.output.exists() or args.per_split < 1:
        raise ValueError('Fresh output and positive sample count required')
    configs = Path(__file__).resolve().parents[1] / 'configs'
    original = Config.fromfile(str(configs / 'bevfusion_base5_dense_512_separated_candidate.py'))
    selected = Config.fromfile(str(configs / 'bevfusion_base5_dense_512_jpeg90_train.py'))
    register_all_modules(init_default_scope=True)
    import_modules_from_strings(**selected.custom_imports)
    torch.set_num_threads(1)
    np.random.seed(261010)
    records = []
    for split in ('train', 'val'):
        dataset_cfg = deepcopy(selected[split + '_dataloader'].dataset)
        dataset_cfg.update(data_root=str(args.data.resolve()), data_prefix=dict(pts='', img=''),
            ann_file=str(args.data.resolve() / 'infos/512x1056' /
                         ('morai_base5_infos_' + split + '.pkl')), pipeline=[])
        dataset = DATASETS.build(dataset_cfg)
        before_pipeline = diagnostic_pipeline(original[split + '_dataloader'].dataset.pipeline)
        after_pipeline = diagnostic_pipeline(selected[split + '_dataloader'].dataset.pipeline)
        indices = sorted(range(len(dataset)), key=lambda i:
            hashlib.sha256((split + ':' + str(i)).encode()).digest())[:args.per_split]
        for index in indices:
            data = dataset.get_data_info(index)
            before = before_pipeline(deepcopy(data))
            after = after_pipeline(deepcopy(data))
            old, new = before['data_samples'], after['data_samples']
            if split == 'train':
                old_boxes, new_boxes = old.gt_instances_3d.bboxes_3d, new.gt_instances_3d.bboxes_3d
                old_labels, new_labels = old.gt_instances_3d.labels_3d, new.gt_instances_3d.labels_3d
            else:
                # Test-mode pipeline keeps GT in eval_ann_info, not the inputs.
                old_boxes, new_boxes = old.eval_ann_info['gt_bboxes_3d'], new.eval_ann_info['gt_bboxes_3d']
                old_labels, new_labels = old.eval_ann_info['gt_labels_3d'], new.eval_ann_info['gt_labels_3d']
            torch.testing.assert_close(old_boxes.tensor, new_boxes.tensor, rtol=0, atol=0)
            np.testing.assert_array_equal(old_labels, new_labels)
            torch.testing.assert_close(before['inputs']['points'], after['inputs']['points'], rtol=0, atol=0)
            for key in ('img_aug_matrix', 'lidar_aug_matrix', 'lidar2img', 'cam2img', 'lidar2cam'):
                if key in old.metainfo or key in new.metainfo:
                    np.testing.assert_array_equal(old.metainfo[key], new.metainfo[key])
            image_before, image_after = before['inputs']['img'], after['inputs']['img']
            if tuple(image_after.shape) != (4, 3, 512, 1056):
                raise ValueError('Unexpected model image shape')
            changed = [not torch.equal(left, right) for left, right in zip(image_before, image_after)]
            if split == 'val' and any(changed):
                raise ValueError('Validation pixels changed')
            records.append(dict(split=split, index=index, scene_id=str(data.get('scene_id', '')),
                labels=new_labels.tolist(), gt_count=len(new_labels),
                changed_cameras=changed, image_shape=list(image_after.shape)))
    changed_train = sum(sum(r['changed_cameras']) for r in records if r['split'] == 'train')
    if not changed_train:
        raise ValueError('JPEG policy did not change any training image')
    result = dict(valid=True, samples=records, train_changed_camera_observations=changed_train,
        val_changed_camera_observations=0,
        scope='Actual CPU file loading, packing, unchanged GT/points/calibration; BEV randomization frozen for paired comparison. No model forward, training or AP claim.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'samples'}, indent=2))


if __name__ == '__main__':
    main()
