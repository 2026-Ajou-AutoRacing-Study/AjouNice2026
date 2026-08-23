#!/usr/bin/env python3
"""Run one augmented 9D PointPainting train step and one inference step."""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import torch


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--openpcdet-root', default='/home/luv/OpenPCDet')
    parser.add_argument('--cfg-file', required=True)
    parser.add_argument('--data-path', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--batch-size', type=int, default=1)
    parser.add_argument('--expect-zero-velocity-gradient', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    openpcdet_root = Path(args.openpcdet_root).expanduser().resolve()
    data_path = Path(args.data_path).expanduser().resolve()
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    sys.path.insert(0, str(openpcdet_root))

    from pcdet.config import cfg, cfg_from_yaml_file
    from pcdet.datasets import build_dataloader
    from pcdet.models import build_network, load_data_to_gpu, model_fn_decorator
    from pcdet.utils import common_utils

    cfg_file = Path(args.cfg_file).resolve()
    # OpenPCDet resolves _BASE_CONFIG_ from its tools working directory.
    os.chdir(openpcdet_root / 'tools')
    cfg_from_yaml_file(str(cfg_file), cfg)
    cfg.DATA_CONFIG.DATA_PATH = str(data_path)
    logger = common_utils.create_logger('/tmp/pointpainting_velocity_smoke.log')

    dataset, loader, _ = build_dataloader(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        batch_size=args.batch_size,
        dist=False,
        workers=0,
        logger=logger,
        training=True,
        total_epochs=1,
    )
    batch = None
    moving_targets = 0
    for index, candidate in enumerate(loader):
        candidate_velocity = candidate['gt_boxes'][..., 7:9]
        moving_targets = int(
            (np.linalg.norm(candidate_velocity, axis=-1) > 1.0e-3).sum()
        )
        if moving_targets:
            batch = candidate
            break
        if index >= 63:
            break
    if batch is None:
        raise RuntimeError('no non-zero velocity GT found in the first 64 batches')
    expected_gt_width = 11 if cfg.MODEL.DENSE_HEAD.NAME == 'AnchorHeadSingleVelocityWeighted' else 10
    if batch['gt_boxes'].shape[-1] != expected_gt_width:
        raise RuntimeError(
            'expected collated weighted 9D GT boxes, got %s' %
            (batch['gt_boxes'].shape,)
        )

    model = build_network(cfg.MODEL, len(cfg.CLASS_NAMES), dataset)
    model.load_params_from_file(
        filename=str(checkpoint),
        logger=logger,
        to_cpu=True,
    )
    model.cuda().train()
    model_func = model_fn_decorator()
    result = model_func(model, batch)
    loss = result.loss
    if not math.isfinite(float(loss.detach().cpu())):
        raise RuntimeError('non-finite smoke loss: %s' % loss)
    loss.backward()
    box_gradient = model.dense_head.conv_box.weight.grad
    velocity_gradient_l1 = float(
        box_gradient.reshape(-1, 9, *box_gradient.shape[1:])[:, 7:9]
        .abs().sum().detach().cpu()
    )
    if args.expect_zero_velocity_gradient and velocity_gradient_l1 > 1.0e-8:
        raise RuntimeError(
            'masked velocity target leaked gradient: %s' % velocity_gradient_l1
        )
    if not args.expect_zero_velocity_gradient and velocity_gradient_l1 <= 0.0:
        raise RuntimeError('velocity output channels received zero gradient')
    geometry_gradient_l1 = float(
        box_gradient.reshape(-1, 9, *box_gradient.shape[1:])[:, :7]
        .abs().sum().detach().cpu()
    )
    if geometry_gradient_l1 <= 0.0:
        raise RuntimeError('geometry output channels received zero gradient')
    metrics = result.tb_dict
    if cfg.MODEL.DENSE_HEAD.NAME == 'AnchorHeadSingleVelocityWeighted':
        required = ('rpn_loss_geometry', 'rpn_loss_velocity', 'velocity_positive_weight_mean')
        missing = [name for name in required if name not in metrics]
        if missing:
            raise RuntimeError('weighted velocity metrics missing: %s' % missing)

    model.eval()
    eval_batch = next(iter(loader))
    load_data_to_gpu(eval_batch)
    with torch.no_grad():
        predictions, _ = model(eval_batch)
    pred_shape = tuple(predictions[0]['pred_boxes'].shape)
    if pred_shape[-1] != 9:
        raise RuntimeError('expected 9D prediction boxes, got %s' % (pred_shape,))
    serialized = dataset.generate_prediction_dicts(
        eval_batch, predictions, cfg.CLASS_NAMES
    )
    serialized_shape = tuple(serialized[0]['boxes_lidar'].shape)
    if serialized_shape[-1] != 9:
        raise RuntimeError('expected serialized 9D boxes, got %s' % (serialized_shape,))

    print('dataset frames:', len(dataset))
    print('augmented gt shape:', tuple(batch['gt_boxes'].shape))
    print('moving velocity targets:', moving_targets)
    print('train loss:', float(loss.detach().cpu()))
    print('velocity-channel gradient L1:', velocity_gradient_l1)
    print('geometry-channel gradient L1:', geometry_gradient_l1)
    print('loss components:', {
        key: metrics[key] for key in metrics
        if key in ('rpn_loss_geometry', 'rpn_loss_velocity', 'velocity_positive_weight_mean')
    })
    print('prediction shape:', pred_shape)
    print('serialized prediction shape:', serialized_shape)
    print('PointPainting velocity 9D forward/backward smoke: OK')


if __name__ == '__main__':
    main()
