#!/usr/bin/env python3
"""CPU attribution of frozen prediction misses, never relabels nearby boxes.

The common Base3 val is a development regression set, not an independent test.
This preserves the validated score-sorted one-to-one center matching policy.
Nearest-box diagnostics do not change TP counts or certify class confusions.
"""

import argparse
from collections import Counter, defaultdict
import gzip
import json
import math
from pathlib import Path
import pickle
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'perception/detection3d/bevfusion/tools'))
from calibrate_base5_thresholds import assignments
from compare_base5_common30 import CLASSES, scope, validate_payload
from summarize_base5_rosbag_quick import bridge_nms
from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms, rotated_bev_iou
from summarize_base3_cross_validation import source_assets

from audit_base5_offline_support import sha256, range_bin, point_bin, percentiles


def angle_error_pi(a, b):
    """BBox axis error: pi-periodic; distinct from directed motion heading."""
    return abs((float(a) - float(b) + math.pi / 2) % math.pi - math.pi / 2)


def diagnostic_nms_floor(row, floor):
    """Explicit offline server-floor candidate, not an installed runtime change."""
    if floor == .1:
        return bridge_nms(row)
    indices = sorted(range(len(row['pred_scores'])), key=lambda i: -row['pred_scores'][i])
    indices = [i for i in indices if row['pred_scores'][i] >= floor][:100]
    detections = [dict(center=row['pred_boxes'][i][:3], size=row['pred_boxes'][i][3:6],
                       yaw=row['pred_boxes'][i][6], score=row['pred_scores'][i],
                       class_name=CLASSES[row['pred_labels'][i]], index=i) for i in indices]
    kept = [d['index'] for d in class_aware_rotated_bev_nms(detections, .5)]
    result = dict(row)
    for key in ('pred_boxes', 'pred_labels', 'pred_scores'):
        result[key] = [row[key][i] for i in kept]
    return result


def nearest_diagnostics(row, g, label, score):
    gt = row['gt_boxes'][g]
    boxes = row['pred_boxes']
    distances = np.linalg.norm(boxes[:, :2] - gt[:2], axis=1)
    own = row['pred_labels'] == label
    qualifying = row['pred_scores'] >= score
    nearby = np.flatnonzero(own & (distances < 1.))
    high = np.flatnonzero(own & qualifying)
    other = np.flatnonzero(~own & qualifying & (distances < 1.))
    best = int(nearby[np.argmax(row['pred_scores'][nearby])]) if len(nearby) else None
    nearest = int(high[np.argmin(distances[high])]) if len(high) else None
    if best is not None and row['pred_scores'][best] < score:
        kind = 'near_correct_class_below_threshold'
    elif best is not None:
        kind = 'near_eligible_box_taken_by_other_gt'
    elif len(other):
        kind = 'eligible_other_class_box_within1m'
    elif nearest is not None and distances[nearest] < 2:
        kind = 'eligible_correct_class_center1_to2m'
    elif nearest is not None and distances[nearest] < 4:
        kind = 'eligible_correct_class_center2_to4m'
    else:
        kind = 'no_eligible_same_class_box_within4m'
    return {
        'kind': kind,
        'best_own_score_within1m': float(row['pred_scores'][best]) if best is not None else None,
        'nearest_eligible_own_distance_m': float(distances[nearest]) if nearest is not None else None,
        'other_class_near1m_count': len(other),
    }


def analyze_rows(rows, assets=None, threshold=.1, class_index=2):
    by_group, examples = defaultdict(list), []
    unmatched = Counter()
    for raw in rows:
        prepared = scope(raw)
        boxes = prepared['gt_boxes']
        labels = prepared['gt_labels']
        score_array = np.asarray(prepared['pred_scores'])
        label_array = np.asarray(prepared['pred_labels'])
        prepared['pred_labels'], prepared['pred_scores'] = label_array, score_array
        matched = {}
        for gate in (.5, 1., 2., 4.):
            pairs = assignments(prepared, class_index, gate)
            matched[gate] = {g: p for p, g in pairs.items() if score_array[p] >= threshold}
        eligible = np.flatnonzero((label_array == class_index) & (score_array >= threshold))
        matched_preds = set(matched[1.].values())
        own_gt = boxes[labels == class_index]
        for p in eligible:
            if p in matched_preds:
                continue
            nearest = float(np.linalg.norm(own_gt[:, :2] - prepared['pred_boxes'][p, :2], axis=1).min()) if len(own_gt) else math.inf
            unmatched['same_retained_gt_center_lt1m' if nearest < 1 else
                      'same_retained_gt_center1_to2m' if nearest < 2 else
                      'not_near_retained_same_class_gt'] += 1
        for g in np.flatnonzero(labels == class_index):
            box = boxes[g]
            detail = {
                'token': raw['token'], 'gt_index_scoped': int(g),
                'asset': str(prepared.get('gt_asset_keys', ['unknown'] * len(boxes))[g]),
                'range_m': float(np.linalg.norm(box[:2])),
                'range_bin': range_bin(np.linalg.norm(box[:2])),
                'points': int(prepared['gt_points'][g]),
                'point_bin': point_bin(prepared['gt_points'][g]),
                'hit05m': g in matched[.5], 'hit1m': g in matched[1.],
                'hit2m': g in matched[2.], 'hit4m': g in matched[4.],
            }
            if g in matched[1.]:
                p = matched[1.][g]
                pred = prepared['pred_boxes'][p]
                detail.update(center_error_m=float(np.linalg.norm(pred[:2] - box[:2])),
                              center_z_abs_error_m=float(abs(pred[2] - box[2])),
                              axis_yaw_error_deg=math.degrees(angle_error_pi(pred[6], box[6])),
                              length_abs_error_m=float(abs(pred[3] - box[3])),
                              width_abs_error_m=float(abs(pred[4] - box[4])),
                              score=float(score_array[p]), miss_kind=None)
            else:
                diagnostic = nearest_diagnostics(prepared, g, class_index, threshold)
                detail['miss_kind'] = diagnostic.pop('kind')
                detail.update(diagnostic)
                if len(examples) < 100:
                    examples.append(detail)
            for key in ['all', 'range/' + detail['range_bin'], 'points/' + detail['point_bin'],
                        'asset/' + detail['asset']]:
                by_group[key].append(detail)
    summary = {}
    for key, values in sorted(by_group.items()):
        hits = [r for r in values if r['hit1m']]
        report = dict(gt=len(values), tp05m=sum(r['hit05m'] for r in values),
                      tp1m=len(hits), tp2m=sum(r['hit2m'] for r in values),
                      tp4m=sum(r['hit4m'] for r in values),
                      miss_partition=dict(Counter(r['miss_kind'] for r in values if r['miss_kind'])),
                      center_error_m=percentiles([r['center_error_m'] for r in hits]),
                      axis_yaw_error_deg=percentiles([r['axis_yaw_error_deg'] for r in hits]),
                      length_abs_error_m=percentiles([r['length_abs_error_m'] for r in hits]),
                      width_abs_error_m=percentiles([r['width_abs_error_m'] for r in hits]),
                      score=percentiles([r['score'] for r in hits]))
        if report['gt'] - report['tp1m'] != sum(report['miss_partition'].values()):
            raise ValueError('Miss partition does not preserve original GT count')
        summary[key] = report
    return dict(groups=summary, eligible_unmatched_proximity=dict(unmatched), miss_examples=examples)


def duplicate_diagnostics(rows, threshold=.1, label=2):
    """Unique GT anchor and nearby unmatched boxes; a diagnostic, not true dedup GT."""
    records = []
    for raw in rows:
        r = scope(raw)
        for key in ('pred_labels', 'pred_scores'):
            r[key] = np.asarray(r[key])
        pairs = assignments(r, label, 1.)
        anchors = {g: p for p, g in pairs.items() if r['pred_scores'][p] >= threshold}
        extra = [p for p in np.flatnonzero((r['pred_labels'] == label) &
                 (r['pred_scores'] >= threshold)) if p not in pairs]
        own_gt = np.flatnonzero(r['gt_labels'] == label)
        for p in extra:
            if not len(own_gt):
                continue
            ds = np.linalg.norm(r['gt_boxes'][own_gt, :2] - r['pred_boxes'][p, :2], axis=1)
            near = own_gt[ds < 1.]
            if len(near) != 1 or int(near[0]) not in anchors:
                continue  # Distinct/ambiguous nearby objects are not declared duplicates.
            g = int(near[0])
            q = anchors[g]
            def detection(index):
                box = r['pred_boxes'][index]
                return dict(center=box[:3], size=box[3:6], yaw=box[6])
            a, b = r['pred_boxes'][p], r['pred_boxes'][q]
            records.append(dict(
                token=raw['token'], gt_index_scoped=g,
                asset=str(r.get('gt_asset_keys', ['unknown'] * len(r['gt_boxes']))[g]),
                gt_range_m=float(np.linalg.norm(r['gt_boxes'][g, :2])),
                pair_center_distance_m=float(np.linalg.norm(a[:2] - b[:2])),
                pair_iou=float(rotated_bev_iou(detection(p), detection(q))),
                pair_axis_yaw_deg=math.degrees(angle_error_pi(a[6], b[6])),
                length_ratio=float(max(a[3], b[3]) / min(a[3], b[3])),
                width_ratio=float(max(a[4], b[4]) / min(a[4], b[4])),
                extra_score=float(r['pred_scores'][p]), anchor_score=float(r['pred_scores'][q])))
    return dict(
        extra_box_observations=len(records),
        anchored_gt_observations=len({(r['token'], r['gt_index_scoped']) for r in records}),
        frames=len({r['token'] for r in records}),
        by_asset=dict(Counter(r['asset'] for r in records)),
        pair_center_distance_m=percentiles([r['pair_center_distance_m'] for r in records]),
        pair_iou=percentiles([r['pair_iou'] for r in records]),
        pair_axis_yaw_deg=percentiles([r['pair_axis_yaw_deg'] for r in records]),
        width_ratio=percentiles([r['width_ratio'] for r in records]),
        length_ratio=percentiles([r['length_ratio'] for r in records]),
        examples=records[:30],
        limitation='Same retained single GT proximity with matched anchor; not independently validated duplicate truth.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions', type=Path, nargs='+', required=True)
    parser.add_argument('--base3-val-infos', type=Path, required=True)
    parser.add_argument('--capture-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Use a new output to preserve earlier audit')
    source_hashes = {str(p): sha256(p) for p in [args.base3_val_infos, *args.predictions]}
    with args.base3_val_infos.open('rb') as stream:
        infos = pickle.load(stream)['data_list']
    assets, asset_audit = source_assets(infos, args.capture_root)
    if asset_audit['unresolved_gt_observations']:
        raise ValueError('Cannot assign asset identity without exact recorded metadata')
    report = {
        'schema': 'base5_frozen_native_regression_diagnostics_v1',
        'scope': 'Frozen Base3 val radial30 intersected with common model box; diagnostic, not new held-out test.',
        'obstacle_score': .1, 'class': 'obstacle',
        'matching': 'Validated score-sorted greedy, one-to-one, center gate strictly < value.',
        'inputs_sha256': source_hashes, 'asset_provenance': asset_audit,
        'models': {},
        'limitations': [
            'Near eligible other-class box is a diagnostic candidate, not verified semantic confusion.',
            'No labels added for omitted native <5-point instances; they are not treated as hallucinations.',
            'BBox axis error is pi-periodic; full heading and instantaneous motion are different.',
            'Geometry metrics condition on matched GT and can be easier for a model that misses harder GT.',
            'Old/new models differ in training, depth, voxel/camera grids, camera crop and inputs; not a causal ablation.',
        ],
    }
    identity = None
    for path in args.predictions:
        with gzip.open(path, 'rt') as stream:
            payload = json.load(stream)
        validate_payload(payload)
        if payload['info_sha256'] != sha256(args.base3_val_infos):
            raise ValueError('Prediction/infos identity differs')
        if identity is not None and identity != payload['gt_identity']:
            raise ValueError('Model GT identity differs')
        identity = payload['gt_identity']
        rows = [dict(r, gt_asset_keys=assets[r['token']]) for r in payload['rows']]
        report['models'][payload['name']] = dict(
            checkpoint_sha256=payload['checkpoint_sha256'], frames=payload['frames'],
            raw=analyze_rows(rows), nms=analyze_rows([bridge_nms(r) for r in rows]),
            duplicates_after_nms=duplicate_diagnostics([bridge_nms(r) for r in rows]),
            diagnostic_floor_sweeps={})
        for floor in (.025, .05, .075, .1, .12):
            selected = [diagnostic_nms_floor(r, floor) for r in rows]
            value = analyze_rows(selected, threshold=floor)
            report['models'][payload['name']]['diagnostic_floor_sweeps'][str(floor)] = dict(
                overall=value['groups']['all'],
                ranges={k: v for k, v in value['groups'].items() if k.startswith('range/')},
                eligible_unmatched_proximity=value['eligible_unmatched_proximity'],
                policy='Offline floor=class score, top100/class-aware NMS.50; not ROS or tracker safety qualified.')
    if source_hashes != {str(p): sha256(p) for p in [args.base3_val_infos, *args.predictions]}:
        raise RuntimeError('Input changed during audit')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({name: {variant: value[variant]['groups']['all'] for variant in ('raw', 'nms')}
                      for name, value in report['models'].items()}, indent=2))


if __name__ == '__main__':
    main()
