#!/usr/bin/env python3
"""
Competition-oriented MORAI detection evaluator for the current OpenPCDet project.

Main metrics (nuScenes-style center-distance protocol):
- Center AP @ 0.5 / 1 / 2 / 4 m
- Per-class mean AP
- Overall center mAP (equal class weighting)
- Best-F1 operating point @ 1 m
- TP box quality @ 2 m: mATE / mASE / mAOE
- Distance diagnostics: 0-10 / 10-20 / 20-30 m

Project assumptions:
- OpenPCDet CustomDataset
- Classes: Pedestrian / Vehicle / Obstacle
- LiDAR boxes: [x, y, z, dx, dy, dz, yaw]
- Overall GT/prediction filtering uses DATA_CONFIG.POINT_CLOUD_RANGE
- OpenPCDet class ids are converted from 1-based to 0-based for evaluation
- Official NDS is not reported because velocity/attribute GT is unavailable

This is a team/competition evaluator, not the official nuScenes devkit.
"""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import build_dataloader
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils


CENTER_THRESHOLDS = (0.5, 1.0, 2.0, 4.0)
DISTANCE_BINS = ((0.0, 10.0), (10.0, 20.0), (20.0, 30.0))
TP_THRESHOLD = 2.0


@dataclass
class MetricData:
    recall: np.ndarray
    precision: np.ndarray
    confidence: np.ndarray
    translation_error: np.ndarray
    scale_error: np.ndarray
    orientation_error: np.ndarray

    @property
    def max_recall_index(self):
        nonzero = np.nonzero(self.confidence)[0]
        return int(nonzero[-1]) if len(nonzero) else 0


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        '--cfg_file',
        default='cfgs/custom_models/pointpillar_morai_pointpainting_sunny13_1500_9feat.yaml'
    )
    p.add_argument(
        '--ckpt',
        required=True,
        help='Path to the PointPillars checkpoint.'
    )
    p.add_argument('--workers', type=int, default=4)
    p.add_argument(
        '--score_thresh', type=float, default=0.01,
        help='Low inference floor for PR-curve evaluation; not the ROS publish threshold.'
    )
    p.add_argument(
        '--max_frames', type=int, default=0,
        help='0 = full validation set; positive value = smoke test subset.'
    )
    p.add_argument('--min_recall', type=float, default=0.1)
    p.add_argument('--min_precision', type=float, default=0.1)
    p.add_argument(
        '--output_json',
        default='/tmp/morai_competition_metrics.json'
    )
    return p.parse_args()


def empty_metric_data():
    recall = np.linspace(0.0, 1.0, 101)
    return MetricData(
        recall=recall,
        precision=np.zeros(101),
        confidence=np.zeros(101),
        translation_error=np.ones(101),
        scale_error=np.ones(101),
        orientation_error=np.ones(101),
    )


def cummean(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return values
    return np.cumsum(values) / np.arange(1, len(values) + 1)


def scale_error(gt_size, pred_size):
    """nuScenes ASE primitive: 1 - aligned 3D size IoU."""
    gt_size = np.maximum(np.asarray(gt_size, dtype=np.float64), 1e-8)
    pred_size = np.maximum(np.asarray(pred_size, dtype=np.float64), 1e-8)
    inter = np.prod(np.minimum(gt_size, pred_size))
    union = np.prod(gt_size) + np.prod(pred_size) - inter
    return float(1.0 - inter / max(union, 1e-8))


def orientation_error(gt_yaw, pred_yaw, period=2.0 * np.pi):
    diff = (pred_yaw - gt_yaw + period / 2.0) % period - period / 2.0
    return float(abs(diff))


def center_range_mask(boxes, point_cloud_range):
    if boxes.size == 0:
        return np.zeros((0,), dtype=bool)
    r = np.asarray(point_cloud_range, dtype=np.float64).reshape(-1)
    return (
        (boxes[:, 0] >= r[0]) & (boxes[:, 0] <= r[3]) &
        (boxes[:, 1] >= r[1]) & (boxes[:, 1] <= r[4]) &
        (boxes[:, 2] >= r[2]) & (boxes[:, 2] <= r[5])
    )


def filter_distance_bin(samples, distance_range):
    lower, upper = distance_range
    out = []
    for sample in samples:
        gd = np.linalg.norm(sample['gt_boxes'][:, :2], axis=1)
        pd = np.linalg.norm(sample['pred_boxes'][:, :2], axis=1)
        gm = (gd >= lower) & (gd < upper)
        pm = (pd >= lower) & (pd < upper)
        out.append({
            'frame_id': sample['frame_id'],
            'gt_boxes': sample['gt_boxes'][gm],
            'gt_labels': sample['gt_labels'][gm],
            'pred_boxes': sample['pred_boxes'][pm],
            'pred_labels': sample['pred_labels'][pm],
            'pred_scores': sample['pred_scores'][pm],
        })
    return out


def accumulate(samples, class_index, distance_threshold):
    """Global score ranking + per-frame one-to-one nearest-center matching."""
    positives = sum(
        int(np.count_nonzero(s['gt_labels'] == class_index)) for s in samples
    )

    ranked = []
    for sample_index, sample in enumerate(samples):
        indices = np.nonzero(sample['pred_labels'] == class_index)[0]
        ranked.extend(
            (float(sample['pred_scores'][i]), sample_index, int(i))
            for i in indices
        )
    ranked.sort(key=lambda x: x[0], reverse=True)

    if positives == 0:
        return empty_metric_data(), 0, len(ranked)

    taken = set()
    tp, fp, confidence = [], [], []
    match_confidence = []
    trans_err, scale_err, orient_err = [], [], []

    for score, sample_index, pred_index in ranked:
        sample = samples[sample_index]
        candidates = np.nonzero(sample['gt_labels'] == class_index)[0]
        best_distance = np.inf
        best_gt = None

        for gt_index in candidates:
            key = (sample_index, int(gt_index))
            if key in taken:
                continue
            dist = np.linalg.norm(
                sample['gt_boxes'][gt_index, :2] - sample['pred_boxes'][pred_index, :2]
            )
            if dist < best_distance:
                best_distance = float(dist)
                best_gt = int(gt_index)

        if best_gt is not None and best_distance < distance_threshold:
            taken.add((sample_index, best_gt))
            tp.append(1.0)
            fp.append(0.0)

            gt_box = sample['gt_boxes'][best_gt]
            pred_box = sample['pred_boxes'][pred_index]
            trans_err.append(best_distance)
            scale_err.append(scale_error(gt_box[3:6], pred_box[3:6]))
            orient_err.append(orientation_error(gt_box[6], pred_box[6]))
            match_confidence.append(score)
        else:
            tp.append(0.0)
            fp.append(1.0)

        confidence.append(score)

    if not match_confidence:
        return empty_metric_data(), positives, len(ranked)

    cumulative_tp = np.cumsum(tp, dtype=np.float64)
    cumulative_fp = np.cumsum(fp, dtype=np.float64)
    raw_recall = cumulative_tp / float(positives)
    raw_precision = cumulative_tp / np.maximum(cumulative_tp + cumulative_fp, 1e-12)

    recall = np.linspace(0.0, 1.0, 101)
    precision = np.interp(recall, raw_recall, raw_precision, right=0.0)
    interp_confidence = np.interp(
        recall, raw_recall, np.asarray(confidence, dtype=np.float64), right=0.0
    )

    errors = []
    reference_confidence = np.asarray(match_confidence, dtype=np.float64)[::-1]
    for values in (trans_err, scale_err, orient_err):
        cumulative = cummean(values)
        errors.append(
            np.interp(
                interp_confidence[::-1],
                reference_confidence,
                cumulative[::-1],
            )[::-1]
        )

    return MetricData(
        recall=recall,
        precision=precision,
        confidence=interp_confidence,
        translation_error=errors[0],
        scale_error=errors[1],
        orientation_error=errors[2],
    ), positives, len(ranked)


def calculate_ap(metric_data, min_recall=0.1, min_precision=0.1):
    first = round(100 * min_recall) + 1
    precision = metric_data.precision[first:].copy() - min_precision
    precision[precision < 0.0] = 0.0
    return float(np.mean(precision) / (1.0 - min_precision))


def calculate_tp(metric_data, name, min_recall=0.1):
    first = round(100 * min_recall) + 1
    last = metric_data.max_recall_index
    if last < first:
        return 1.0
    return float(np.mean(getattr(metric_data, name)[first:last + 1]))


def best_f1(metric_data):
    denom = metric_data.precision + metric_data.recall
    f1 = np.divide(
        2.0 * metric_data.precision * metric_data.recall,
        denom,
        out=np.zeros_like(denom),
        where=denom > 0.0,
    )
    idx = int(np.argmax(f1))
    return {
        'F1': float(f1[idx]),
        'precision': float(metric_data.precision[idx]),
        'recall': float(metric_data.recall[idx]),
        'score': float(metric_data.confidence[idx]),
    }


def evaluate_center(samples, class_names, min_recall, min_precision, prefix,
                    include_tp=True, include_operating=True):
    metrics = {}
    class_mean_aps = []

    for class_index, class_name in enumerate(class_names):
        aps = []
        for threshold in CENTER_THRESHOLDS:
            md, positives, predictions = accumulate(samples, class_index, threshold)
            ap = calculate_ap(md, min_recall, min_precision)
            metrics[f'{prefix}/{class_name}_AP_{threshold:g}m'] = ap
            metrics[f'{prefix}/{class_name}_GT'] = float(positives)
            metrics[f'{prefix}/{class_name}_predictions'] = float(predictions)

            if include_operating:
                op = best_f1(md)
                op_prefix = f'{prefix}/{class_name}_{threshold:g}m_best'
                metrics[f'{op_prefix}_F1'] = op['F1']
                metrics[f'{op_prefix}_precision'] = op['precision']
                metrics[f'{op_prefix}_recall'] = op['recall']
                metrics[f'{op_prefix}_score'] = op['score']

            aps.append(ap)

        class_mean = float(np.mean(aps))
        metrics[f'{prefix}/{class_name}_mean_AP'] = class_mean
        class_mean_aps.append(class_mean)

        if include_tp:
            md, _, _ = accumulate(samples, class_index, TP_THRESHOLD)
            metrics[f'tp/{class_name}_mATE'] = calculate_tp(
                md, 'translation_error', min_recall
            )
            metrics[f'tp/{class_name}_mASE'] = calculate_tp(
                md, 'scale_error', min_recall
            )
            metrics[f'tp/{class_name}_mAOE'] = calculate_tp(
                md, 'orientation_error', min_recall
            )

    metrics[f'{prefix}/mAP'] = float(np.mean(class_mean_aps))
    return metrics


def collect_samples(args):
    cfg_from_yaml_file(args.cfg_file, cfg)
    cfg.MODEL.POST_PROCESSING.SCORE_THRESH = float(args.score_thresh)

    logger = common_utils.create_logger()
    dataset, loader, _ = build_dataloader(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        batch_size=1,
        dist=False,
        workers=args.workers,
        logger=logger,
        training=False,
    )

    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(cfg.CLASS_NAMES),
        dataset=dataset,
    )
    model.load_params_from_file(args.ckpt, logger=logger, to_cpu=False)
    model.cuda().eval()

    pc_range = np.asarray(cfg.DATA_CONFIG.POINT_CLOUD_RANGE, dtype=np.float64)
    limit = len(dataset) if args.max_frames <= 0 else min(len(dataset), args.max_frames)
    samples = []

    print('\n========== MORAI COMPETITION EVALUATION ==========')
    print('classes         :', list(cfg.CLASS_NAMES))
    print('checkpoint      :', args.ckpt)
    print('frames          :', limit)
    print('score floor     :', args.score_thresh)
    print('pointcloud range:', pc_range.tolist())
    print('==================================================\n')

    for frame_index, batch in enumerate(loader):
        if frame_index >= limit:
            break

        frame_id = str(batch['frame_id'][0])
        raw_gt = np.asarray(batch['gt_boxes'][0], dtype=np.float64).copy()

        if raw_gt.size:
            valid = (
                np.isfinite(raw_gt).all(axis=1) &
                (raw_gt[:, 3] > 0) &
                (raw_gt[:, 4] > 0) &
                (raw_gt[:, 5] > 0) &
                (raw_gt[:, -1] > 0)
            )
            raw_gt = raw_gt[valid]
            gt_boxes = raw_gt[:, :7]
            gt_labels = raw_gt[:, -1].astype(np.int64) - 1
            mask = center_range_mask(gt_boxes, pc_range)
            gt_boxes, gt_labels = gt_boxes[mask], gt_labels[mask]
        else:
            gt_boxes = np.zeros((0, 7), dtype=np.float64)
            gt_labels = np.zeros((0,), dtype=np.int64)

        load_data_to_gpu(batch)
        with torch.no_grad():
            pred_dicts, _ = model(batch)

        pred = pred_dicts[0]
        pred_boxes = pred['pred_boxes'].detach().cpu().numpy().astype(np.float64)
        pred_scores = pred['pred_scores'].detach().cpu().numpy().astype(np.float64)
        pred_labels = pred['pred_labels'].detach().cpu().numpy().astype(np.int64) - 1

        mask = center_range_mask(pred_boxes, pc_range)
        pred_boxes = pred_boxes[mask]
        pred_scores = pred_scores[mask]
        pred_labels = pred_labels[mask]

        samples.append({
            'frame_id': frame_id,
            'gt_boxes': gt_boxes,
            'gt_labels': gt_labels,
            'pred_boxes': pred_boxes,
            'pred_labels': pred_labels,
            'pred_scores': pred_scores,
        })

        if (frame_index + 1) % 25 == 0 or (frame_index + 1) == limit:
            print(f'processed {frame_index + 1}/{limit}')

    return samples, list(cfg.CLASS_NAMES), pc_range


def print_report(metrics, class_names):
    print('\n===== CENTER AP =====')
    for class_name in class_names:
        print(f'\n[{class_name}]')
        for threshold in CENTER_THRESHOLDS:
            print(
                f'AP@{threshold:g}m='
                f'{metrics[f"center/{class_name}_AP_{threshold:g}m"]:.6f}'
            )
        print(f'mean AP={metrics[f"center/{class_name}_mean_AP"]:.6f}')
        print(
            'GT / predictions='
            f'{int(metrics[f"center/{class_name}_GT"])} / '
            f'{int(metrics[f"center/{class_name}_predictions"])}'
        )

    print('\nOVERALL CENTER mAP=', f'{metrics["center/mAP"]:.6f}')

    print('\n===== BOX QUALITY @2m =====')
    for class_name in class_names:
        print(
            f'{class_name:10s} '
            f'mATE={metrics[f"tp/{class_name}_mATE"]:.6f}  '
            f'mASE={metrics[f"tp/{class_name}_mASE"]:.6f}  '
            f'mAOE={metrics[f"tp/{class_name}_mAOE"]:.6f}'
        )

    print('\n===== OPERATING POINT @1m =====')
    for class_name in class_names:
        p = f'center/{class_name}_1m_best'
        print(
            f'{class_name:10s} '
            f'F1={metrics[f"{p}_F1"]:.6f}  '
            f'P={metrics[f"{p}_precision"]:.6f}  '
            f'R={metrics[f"{p}_recall"]:.6f}  '
            f'score={metrics[f"{p}_score"]:.6f}'
        )

    print('\n===== DISTANCE BINS =====')
    for lower, upper in DISTANCE_BINS:
        prefix = f'range/{lower:g}_{upper:g}m'
        print(f'\n[{lower:g}-{upper:g}m] mAP={metrics[f"{prefix}/mAP"]:.6f}')
        for class_name in class_names:
            print(
                f'  {class_name:10s} mean_AP='
                f'{metrics[f"{prefix}/{class_name}_mean_AP"]:.6f}'
            )

    print('\nOfficial NDS available: NO')


def main():
    args = parse_args()
    samples, class_names, pc_range = collect_samples(args)

    metrics = evaluate_center(
        samples,
        class_names,
        args.min_recall,
        args.min_precision,
        prefix='center',
        include_tp=True,
        include_operating=True,
    )

    for lower, upper in DISTANCE_BINS:
        ranged = filter_distance_bin(samples, (lower, upper))
        metrics.update(
            evaluate_center(
                ranged,
                class_names,
                args.min_recall,
                args.min_precision,
                prefix=f'range/{lower:g}_{upper:g}m',
                include_tp=False,
                include_operating=False,
            )
        )

    metrics['meta/official_NDS_available'] = 0.0
    print_report(metrics, class_names)

    payload = {
        'meta': {
            'checkpoint': args.ckpt,
            'frames': len(samples),
            'classes': class_names,
            'point_cloud_range': pc_range.tolist(),
            'inference_score_floor': args.score_thresh,
            'center_thresholds_m': list(CENTER_THRESHOLDS),
            'tp_threshold_m': TP_THRESHOLD,
            'min_recall': args.min_recall,
            'min_precision': args.min_precision,
            'distance_bins_m': [list(v) for v in DISTANCE_BINS],
            'official_NDS_available': False,
        },
        'metrics': {k: float(v) for k, v in metrics.items()},
    }

    out = Path(args.output_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding='utf-8')
    print('\nsaved:', out)


if __name__ == '__main__':
    main()
