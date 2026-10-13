#!/usr/bin/env python3
"""Accuracy deltas of matched runtime exports, with explicit split provenance.

The benchmark's 300-frame representative/stress set contains train and val.
Mixed selections diagnose optimization changes, not held-out accuracy. An
all-val selection evaluates the existing validation split, not a new test set,
threshold selection or deployment approval.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import pickle
import sys

import numpy as np
from compare_base5_common30 import CLASSES, gt_identity, scope, validate_payload
from summarize_base5_rosbag_quick import bridge_nms

ROOT = Path(__file__).resolve().parents[4]
ROI = [-100., -30., -3., 100., 30., 2.]


def make_row(token, authored, detections):
    instances = authored['instances']
    boxes = np.asarray([r['bbox_3d'] for r in instances], dtype=float).reshape(-1, 7)
    boxes[:, 2] -= boxes[:, 5] / 2
    if any(not 0 <= d['label'] < len(CLASSES) or
           d['class_name'] != CLASSES[d['label']] for d in detections):
        raise ValueError('Runtime label/class contract differs')
    return dict(token=token, gt_boxes=boxes.tolist(),
        gt_labels=[r['bbox_label_3d'] for r in instances],
        gt_points=[r['num_lidar_pts'] for r in instances],
        gt_assets=[r.get('asset', 'unknown') for r in instances],
        pred_boxes=[d['center']+d['size']+[d['yaw']] for d in detections],
        pred_labels=[d['label'] for d in detections],
        pred_scores=[d['score'] for d in detections])


def summarize(data, report_path, output):
    if output.exists():
        raise ValueError('Fresh accuracy report required')
    report = json.loads(report_path.read_text())
    if not report.get('preserve_learned_config'):
        raise ValueError('Do not evaluate a shape-changed COST fixture as a learned model')
    if report['model_geometry']['classes'] != list(CLASSES):
        raise ValueError('Three learned classes required')
    if report['decoder'] != dict(score_floor=.1, max_detections=100):
        raise ValueError('ROS postprocess comparison requires decoder floor0.1/top100')
    authored = {}
    splits = {}
    for split in ('train', 'val'):
        with (data/f'infos/512x1056/morai_base5_infos_{split}.pkl').open('rb') as f:
            for row in pickle.load(f)['data_list']:
                if row['token'] in authored:
                    raise ValueError('Split leakage/duplicate token in source infos')
                authored[row['token']] = row
                splits[row['token']] = split
    spec = importlib.util.spec_from_file_location('runtime_metric',
        ROOT/'perception/detection3d/common/evaluation/morai_detection.py')
    metric = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = metric
    spec.loader.exec_module(metric)
    result = dict(scope='Matched runtime-optimization accuracy diagnostics; train/val mixed selection is NOT held-out validation.',
        postprocess='Decoder floor/top-k from benchmark; class-aware rotated BEV NMS IoU0.5.',
        decoder=report['decoder'], precision=report['precision'], cases=[], gt_identity=None)
    reference_tokens = None
    for index, case in enumerate(report['cases'], 1):
        path = report_path.parent/f'{index}_{case["variant"]}_predictions.json'
        predictions = json.loads(path.read_text())
        tokens = [r['id'] for r in case['per_sample']]
        if len(set(tokens)) != len(tokens) or set(tokens) != set(predictions):
            raise ValueError('Incomplete or repeated runtime exports')
        if reference_tokens is None:
            reference_tokens = tokens
            result['split_counts'] = {s: sum(splits[t] == s for t in tokens) for s in ('train', 'val')}
            if result['split_counts']['train'] == 0:
                result['scope'] = ('Matched runtime-optimization comparison on the existing '
                    'validation split; not an independent test or deployment approval.')
        elif tokens != reference_tokens:
            raise ValueError('Runtime variants did not see identical ordered frames')
        rows = [make_row(t, authored[t], predictions[t]) for t in tokens]
        identity = gt_identity(rows)
        validate_payload(dict(classes=CLASSES, frames=len(rows), rows=rows, gt_identity=identity))
        if result['gt_identity'] is not None and result['gt_identity'] != identity:
            raise ValueError('GT differs between runtime variants')
        result['gt_identity'] = identity
        samples = []
        for row in rows:
            filtered = bridge_nms(row)
            sample = {key: np.asarray(filtered[key]) for key in
                ('gt_boxes', 'gt_labels', 'pred_boxes', 'pred_labels', 'pred_scores')}
            for key in ('gt_boxes', 'pred_boxes'):
                sample[key] = sample[key].astype(float).reshape(-1, 7)
            samples.append(sample)
        samples = metric._filter_center_box_range(samples, ROI)
        metrics = {}
        for label, selected in [('full_box', samples),
                ('radial30_common', [scope(r) for r in samples]),
                *[(f'range/{lo}_{hi}m', metric._filter_range(samples, (lo, hi)))
                  for lo, hi in ((0,30), (30,50), (50,80), (80,100))]]:
            metrics.update(metric._evaluate_center(selected, list(CLASSES),
                metric.DEFAULT_CENTER_THRESHOLDS, 2., .1, .1, label,
                include_tp=label == 'full_box', include_operating_points=False))
        result['cases'].append(dict(variant=case['variant'], frames=len(tokens), metrics=metrics,
            decoded_parity=case['parity']['passed'], prediction_counts=case['prediction_counts']))
    baseline = result['cases'][0]['metrics']
    for case in result['cases']:
        case['delta_pp_from_first_baseline'] = {k: (v-baseline[k])*100
            for k, v in case['metrics'].items() if k.endswith(('mean_AP', '/mAP'))}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({c['variant']+'_'+str(i): {k:round(v*100,3) for k,v in c['metrics'].items()
         if k.startswith('full_box/') and k.endswith(('mean_AP','/mAP'))}
         for i,c in enumerate(result['cases'])}, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('data', 'report', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    a = p.parse_args()
    summarize(a.data, a.report, a.output)
