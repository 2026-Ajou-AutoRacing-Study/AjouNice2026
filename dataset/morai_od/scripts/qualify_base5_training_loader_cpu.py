#!/usr/bin/env python3
"""Read-only CPU dataset/pipeline qualification; no model or CUDA execution.

Run inside the existing team image, without --gpus, with the repository and
qualified packages mounted read-only. The output directory alone may be RW.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import pickle
import random
import time

import numpy as np


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def expected_after_bev(instances, matrix, roi):
    boxes = np.asarray([i['bbox_3d'] for i in instances], dtype=np.float64).reshape(-1, 7)
    centers = np.c_[boxes[:, :3], np.ones(len(boxes))] @ np.asarray(matrix).T
    mask = ((centers[:, :2] > np.asarray(roi[:2])) &
            (centers[:, :2] < np.asarray(roi[3:5]))).all(axis=1)
    scale = np.linalg.norm(np.asarray(matrix)[:3, 0])
    return centers[mask, :3], boxes[mask, 3:6] * scale, np.asarray([
        i['bbox_label_3d'] for i in instances], dtype=np.int64)[mask]


def run(qualification, config, output, seed=123):
    # CUDA is deliberately unavailable. Imports register CPU dataset/transforms
    # but never instantiate the configured model, optimizer or runner.
    import torch
    from mmengine.config import Config
    from mmdet3d.registry import DATASETS
    from mmdet3d.utils import register_all_modules

    require(not torch.cuda.is_available(), 'Run without GPU exposure')
    output = Path(output)
    require(not output.exists(), 'Never overwrite previous loader qualification')
    source = json.loads(Path(qualification).read_text())
    torch.set_num_threads(2)
    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(config)
    roi = cfg.point_cloud_range
    cohort_results = {}
    started = time.monotonic()
    for name, cohort in source['cohorts'].items():
        package = Path(cohort['training3_package'])
        manifest_sha = sha256(package / 'manifest.json')
        require(manifest_sha == cohort['training3_manifest_sha256'], 'Qualified package changed')
        counts, frames = Counter(), []
        for split in ('train', 'val', 'test'):
            ann = package / 'infos/512x1056' / f'morai_base5_infos_{split}.pkl'
            original = pickle.loads(ann.read_bytes())['data_list']
            if not original:
                continue
            datasets = {}
            for mode in ('train', 'val'):
                spec = deepcopy(cfg[f'{mode}_dataloader']['dataset'])
                spec['data_root'], spec['ann_file'] = str(package), str(ann)
                datasets[mode] = DATASETS.build(spec)
                require(len(datasets[mode]) == len(original), 'Dataset lost qualified frames')
            for index, record in enumerate(original):
                parsed = datasets['train'].get_data_info(index)
                native = np.asarray([i['bbox_3d'] for i in record['instances']],
                                    dtype=np.float64).reshape(-1, 7)
                np.testing.assert_allclose(parsed['ann_info']['gt_bboxes_3d'].gravity_center.numpy(),
                                           native[:, :3], rtol=0, atol=1e-5)
                require(len(parsed['ann_info']['gt_labels_3d']) == len(native),
                        'Parsed annotations lost component identities')
                frame = dict(token=record['token'], original_split=split,
                             source_gt=len(native), modes={})
                for mode, dataset in datasets.items():
                    random.seed(seed + index)
                    np.random.seed(seed + index)
                    torch.manual_seed(seed + index)
                    packed = dataset.prepare_data(index)
                    require(packed is not None, 'CPU pipeline returned no sample')
                    sample, inputs = packed['data_samples'], packed['inputs']
                    image, points = inputs['img'], inputs['points']
                    require(tuple(image.shape) == (4, 3, 512, 1056), 'Unexpected packed camera shape')
                    require(image.dtype == torch.uint8, 'Current uint8 JPEG image policy changed')
                    require(points.ndim == 2 and points.shape[1] == 4
                            and torch.isfinite(points).all(), 'Invalid packed XYZI')
                    require(not (points[:, :3] == 0).all(dim=1).any(), 'Zero XYZ survived pipeline')
                    p = points[:, :3].numpy()
                    require(((p >= roi[:3]) & (p <= roi[3:])).all(), 'Packed points outside model ROI')
                    result = dict(image_shape=list(image.shape), image_dtype=str(image.dtype),
                                  points=len(points))
                    if mode == 'train':
                        matrix = np.asarray(sample.metainfo['lidar_aug_matrix'])
                        centers, sizes, labels = expected_after_bev(record['instances'], matrix, roi)
                        actual = sample.gt_instances_3d
                        np.testing.assert_array_equal(actual.labels_3d.numpy(), labels)
                        np.testing.assert_allclose(actual.bboxes_3d.gravity_center.numpy(), centers,
                                                   rtol=0, atol=3e-5)
                        np.testing.assert_allclose(actual.bboxes_3d.dims.numpy(), sizes, rtol=0, atol=1e-5)
                        result.update(gt=len(labels), legitimate_augmented_roi_removals=len(native) - len(labels),
                                      lidar_aug_matrix=matrix.tolist())
                        counts['source_gt'] += len(native)
                        counts['packed_augmented_gt'] += len(labels)
                    frame['modes'][mode] = result
                    counts[mode + '_frames'] += 1
                frames.append(frame)
        require(sha256(package / 'manifest.json') == manifest_sha, 'Source package mutated')
        cohort_results[name] = dict(counts=dict(counts), frames=frames, package_manifest_sha256=manifest_sha)
    report = dict(schema='base5_cpu_training_loader_qualification_v1',
        qualification_report=str(Path(qualification).resolve()), qualification_sha256=sha256(qualification),
        config=str(Path(config).resolve()), config_sha256=sha256(config),
        resolved_config_sha256=hashlib.sha256(cfg.pretty_text.encode()).hexdigest(),
        script_sha256=sha256(__file__), torch=torch.__version__, cuda_available=False,
        seed=seed, elapsed_sec=time.monotonic() - started, cohorts=cohort_results,
        gpu_execution=False, training_started=False,
        scope='Actual current CPU MMDet dataset parse + train/validation transforms and packing; no neural network/loss/backward/batching throughput validation.',
        limitations=['Each frame receives one deterministic BEV augmentation draw, including val frames for diagnostic purposes only; original splits are unchanged.',
                     'No dataloader multiprocessing, model data-preprocessor/voxelization or GPU/NN validation.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({name: value['counts'] for name, value in cohort_results.items()}, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=123)
    options = parser.parse_args()
    run(options.qualification, options.config, options.output, options.seed)
