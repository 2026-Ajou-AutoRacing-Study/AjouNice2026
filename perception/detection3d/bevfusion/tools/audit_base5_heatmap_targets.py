#!/usr/bin/env python3
"""CPU-only 5K target audit: real GT, actual BEV augmentation and box roundtrip.

Reads captures/infos without changing them. Fresh output is required. The live
parent's dense-target block is extracted for diagnosis, not used as a new target.
No image/model inference, GPU benchmark, training or accuracy claim is made.
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import inspect
import json
from pathlib import Path
import pickle
import textwrap
import time

import numpy as np
import torch
from mmengine.config import Config
from mmengine.dataset import Compose
from mmengine.structures import InstanceData
from mmengine.utils import import_modules_from_strings
from mmdet3d.structures import LiDARInstance3DBoxes, LiDARPoints
from mmdet3d.utils import register_all_modules
from projects.BEVFusion.bevfusion.transfusion_head import TransFusionHead
from projects.BEVFusion.bevfusion.utils import TransFusionBBoxCoder
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import MoraiRectangularTransFusionHead


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def legacy_target_block():
    function = TransFusionHead.get_targets_single
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    body = tree.body[0].body
    def assignment_to(node, name):
        return isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets)
    start = next(i for i, node in enumerate(body) if assignment_to(node, 'device'))
    end = next(i for i, node in enumerate(body) if assignment_to(node, 'mean_iou'))
    code = compile(ast.Module(body=body[start:end], type_ignores=[]),
                   inspect.getsourcefile(function), 'exec')
    def build(head, gt):
        namespace = dict(function.__globals__, self=head, labels=gt.labels_3d,
                         gt_bboxes_3d=gt.bboxes_3d, gt_labels_3d=gt.labels_3d)
        exec(code, namespace)
        return namespace['heatmap'][None]
    return build


def ground_truth(rows):
    return InstanceData(
        bboxes_3d=LiDARInstance3DBoxes(
            torch.tensor([row['bbox_3d'] for row in rows], dtype=torch.float32).reshape(-1, 7),
            origin=(.5, .5, .5)),
        labels_3d=torch.tensor([row['bbox_label_3d'] for row in rows], dtype=torch.long))


def draw_examples(head, legacy, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(12, 5), constrained_layout=True)
    bounds = head.train_cfg['point_cloud_range']
    extent = [bounds[0], bounds[3], bounds[1], bounds[4]]
    for row_index, x in enumerate((40., -40.)):
        gt = ground_truth([dict(bbox_3d=[x, 0, 0, 4.5, 1.8, 1.5, 0], bbox_label_3d=0)])
        for column, (name, target) in enumerate((('Before: swapped target', legacy(head, gt)),
                                                 ('After: aligned target', head.build_dense_heatmap_targets(gt)))):
            axis = axes[row_index, column]
            axis.imshow(target[0, 0].numpy(), origin='lower', extent=extent,
                        cmap='Blues', vmin=0, vmax=1, aspect='auto')
            axis.scatter([x], [0], marker='x', color='red', s=80, label='Actual GT center')
            axis.set_title(f'{name} | GT X={x:g} m')
            axis.set_xlabel('LiDAR X (m): rear < 0 < front')
            axis.set_ylabel('LiDAR Y (m)')
            if not target.max():
                axis.text(.03, .9, 'No positive target peak', transform=axis.transAxes, color='darkred')
            axis.legend(loc='lower right', fontsize=8)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--augmentation-repeats', type=int, default=2)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh evidence output directory')
    if not 1 <= args.augmentation_repeats <= 8:
        parser.error('Bounded augmentation repeats required')
    torch.set_num_threads(1)
    register_all_modules(init_default_scope=True)
    config = Config.fromfile(str(args.config))
    import_modules_from_strings(**config.custom_imports)
    head = object.__new__(MoraiRectangularTransFusionHead)
    torch.nn.Module.__init__(head)
    head.train_cfg = deepcopy(config.model.bbox_head.train_cfg)
    head.num_classes = config.model.bbox_head.num_classes
    legacy = legacy_target_block()
    coder = TransFusionBBoxCoder(**{key: value for key, value in config.model.bbox_head.bbox_coder.items()
                                   if key != 'type'})
    pitch = torch.tensor(head.train_cfg.voxel_size[:2]) * head.train_cfg.out_size_factor
    origin = torch.tensor(head.train_cfg.point_cloud_range[:2])
    x_size, y_size = (int(size // head.train_cfg.out_size_factor) for size in head.train_cfg.grid_size[:2])
    positions = head.create_2D_grid(x_size, y_size)
    # These are the actual configured geometry transforms. Empty points are
    # intentional: they exercise GT/augmentation math without sensor I/O.
    geometry = Compose([deepcopy(item) for item in config.train_dataloader.dataset.pipeline
                        if item['type'] in ('BEVFusionGlobalRotScaleTrans', 'ObjectRangeFilter', 'ObjectNameFilter')])
    paths = [args.data/'manifest.json'] + [args.data/'infos/512x1056'/f'morai_base5_infos_{split}.pkl'
                                          for split in ('train', 'val')]
    before_hashes = {str(path): sha256(path) for path in paths}
    args.output.mkdir(parents=True)
    stages = {}
    maximum_roundtrip_error = 0.
    start = time.monotonic()
    for split, path in zip(('train', 'val'), paths[1:]):
        records = pickle.loads(path.read_bytes())['data_list']
        for repeat in range(-1, args.augmentation_repeats if split == 'train' else 0):
            key = split + (':original' if repeat == -1 else f':augmentation_{repeat+1}')
            stats = Counter()
            class_counts = Counter()
            for record in records:
                gt = ground_truth(record['instances'])
                before = len(gt)
                if repeat >= 0:
                    seed = int.from_bytes(hashlib.sha256((record['token']+f':heatmap:{repeat}').encode()).digest()[:4], 'big')
                    np.random.seed(seed)
                    transformed = geometry(dict(
                        points=LiDARPoints(torch.empty(0, 4), points_dim=4),
                        gt_bboxes_3d=gt.bboxes_3d,
                        gt_labels_3d=gt.labels_3d.numpy().copy()))
                    gt = InstanceData(bboxes_3d=transformed['gt_bboxes_3d'],
                                      labels_3d=torch.as_tensor(transformed['gt_labels_3d'], dtype=torch.long))
                original_boxes = gt.bboxes_3d.tensor.clone()
                target = head.build_dense_heatmap_targets(gt)
                previous = legacy(head, gt)
                stats['frames'] += 1
                stats['gt'] += len(gt)
                stats['range_filtered'] += before - len(gt)
                if not len(gt):
                    stats['empty_frames'] += 1
                    assert not target.any()
                    continue
                centers = ((gt.bboxes_3d.tensor[:, :2] - origin) / pitch).int()
                x, y = centers.unbind(1)
                labels = gt.labels_3d
                assert bool((target[0, labels, y, x] == 1).all()), (key, record['token'])
                stats['aligned_peaks'] += len(gt)
                stats['legacy_hits_at_correct_cell'] += int((previous[0, labels, y, x] == 1).sum())
                stats['legacy_swapped_center_outside'] += int((x >= y_size).sum())
                torch.testing.assert_close(positions[0, y*x_size+x], centers.float()+.5, rtol=0, atol=0)
                encoded = coder.encode(gt.bboxes_3d.tensor)
                torch.testing.assert_close(encoded[:, :2], (original_boxes[:, :2]-origin)/pitch, rtol=0, atol=0)
                scores = torch.zeros(1, head.num_classes, len(gt))
                scores[0, labels, torch.arange(len(gt))] = 1
                decoded = coder.decode(scores, encoded[:, 6:8].T[None],
                    encoded[:, 3:6].T[None], encoded[:, :2].T[None].clone(),
                    encoded[:, 2:3].T[None], None)[0]['bboxes']
                # Yaw is periodic; compare sin/cos rather than raw angles.
                error = float((decoded[:, :6] - original_boxes[:, :6]).abs().max())
                maximum_roundtrip_error = max(maximum_roundtrip_error, error)
                # FP32 division followed by the coder's chained multiplications
                # has several ULPs of error around 100m; allow 0.05mm, not a cell.
                torch.testing.assert_close(decoded[:, :6], original_boxes[:, :6], rtol=0, atol=5e-5)
                torch.testing.assert_close(torch.sin(decoded[:, 6]), torch.sin(original_boxes[:, 6]), rtol=0, atol=2e-6)
                torch.testing.assert_close(torch.cos(decoded[:, 6]), torch.cos(original_boxes[:, 6]), rtol=0, atol=2e-6)
                torch.testing.assert_close(gt.bboxes_3d.tensor, original_boxes, rtol=0, atol=0)
                class_counts.update(labels.tolist())
            stages[key] = dict(stats, classes={config.classes[index]: class_counts[index] for index in range(head.num_classes)})
            print(key, json.dumps(stages[key]), flush=True)
    assert before_hashes == {str(path): sha256(path) for path in paths}
    draw_examples(head, legacy, args.output/'before_after_targets.png')
    report = dict(valid=True, scope='CPU target geometry/gradient prerequisites; not AP or model latency',
        stages=stages, box_roundtrip_max_abs_error_m=maximum_roundtrip_error,
        config_sha256=sha256(args.config), unchanged_input_sha256=before_hashes,
        legacy_head_source=inspect.getsourcefile(TransFusionHead),
        legacy_head_sha256=sha256(inspect.getsourcefile(TransFusionHead)),
        target_plugin_sha256=sha256(inspect.getsourcefile(MoraiRectangularTransFusionHead)),
        elapsed_sec=time.monotonic()-start,
        geometry_pipeline=[item for item in config.train_dataloader.dataset.pipeline
                           if item['type'] in ('BEVFusionGlobalRotScaleTrans', 'ObjectRangeFilter', 'ObjectNameFilter')])
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print('PASS', json.dumps({key: value for key, value in report.items() if key not in ('stages', 'unchanged_input_sha256')}), flush=True)


if __name__ == '__main__':
    main()
