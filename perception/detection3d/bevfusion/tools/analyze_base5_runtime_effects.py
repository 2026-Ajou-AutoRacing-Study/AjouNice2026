#!/usr/bin/env python3
"""Matched runtime drift AFTER real bridge filtering; not tracker certification.

Unordered native validation scenes cannot be replayed as a continuous drive.
This diagnoses delivery, footprint and tracker confidence-regime changes only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
from bevfusion_ros.rotated_bev_nms import (
    _rectangle_corners, class_aware_rotated_bev_nms, rotated_bev_iou)

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
POLICIES = {'current': (.4, .15, .1), 'obstacle015_candidate': (.4, .15, .15)}


def delivered(detections, thresholds):
    for d in detections:
        if (len(d['center']) != 3 or len(d['size']) != 3 or
                not 0 <= d['score'] <= 1 or any(v <= 0 for v in d['size']) or
                not 0 <= d['label'] < 3 or d['class_name'] != CLASSES[d['label']] or
                not np.isfinite([*d['center'], *d['size'], d['yaw'], d['score']]).all()):
            raise ValueError('Finite three-class runtime outputs required')
    selected = sorted(detections, key=lambda d: -d['score'])
    selected = [d for d in selected if d['score'] >= .1][:100]
    selected = class_aware_rotated_bev_nms(selected, .5)
    return [d for d in selected if d['score'] >= thresholds[d['label']]]


def match(old, new, gate=.5):
    if not old or not new:
        return [], list(range(len(old))), list(range(len(new)))
    costs = np.linalg.norm(np.array([d['center'][:2] for d in old])[:, None] -
                           np.array([d['center'][:2] for d in new])[None], axis=-1)
    rows, cols = linear_sum_assignment(np.where(costs < gate, costs, 1e6))
    pairs = [(int(i), int(j)) for i, j in zip(rows, cols) if costs[i, j] < gate]
    return pairs, [i for i in range(len(old)) if i not in {p[0] for p in pairs}], \
        [j for j in range(len(new)) if j not in {p[1] for p in pairs}]


def drift(old, new):
    a = np.asarray(_rectangle_corners(old['center'], old['size'], old['yaw']))
    b = np.asarray(_rectangle_corners(new['center'], new['size'], new['yaw']))
    # Cyclic corner matching respects the pi ambiguity and square symmetry.
    corner = min(float(np.linalg.norm(a-np.roll(b, k, axis=0), axis=1).max()) for k in range(4))
    delta = old['yaw']-new['yaw']
    return dict(center_xy_m=float(np.linalg.norm(np.array(old['center'][:2])-new['center'][:2])),
        size_linf_m=float(np.max(np.abs(np.array(old['size'])-new['size']))),
        yaw_deg=abs(float(np.degrees(np.arctan2(np.sin(delta), np.cos(delta))))),
        score_abs=abs(float(old['score']-new['score'])),
        footprint_corner_m=corner, rotated_bev_iou=rotated_bev_iou(old, new),
        tracker_confidence_regime_flip=(old['score'] < .5) != (new['score'] < .5))


def distribution(values):
    if not values:
        return dict(count=0, p50=None, p95=None, minimum=None, maximum=None)
    v = np.asarray(values, float)
    return dict(count=len(v), p50=float(np.percentile(v, 50)), p95=float(np.percentile(v, 95)),
                minimum=float(v.min()), maximum=float(v.max()))


def compare(reference, candidate, tokens, thresholds):
    result = {}
    for label, name in enumerate(CLASSES):
        pairs = []; changes = []; flips = []
        old_count = new_count = removed = added = 0
        for token in tokens:
            old = [d for d in delivered(reference[token], thresholds) if d['label'] == label]
            new = [d for d in delivered(candidate[token], thresholds) if d['label'] == label]
            old_count += len(old); new_count += len(new)
            matched, missing, extra = match(old, new)
            if missing or extra:
                changes.append(dict(id=token, removed=[old[i] for i in missing], added=[new[j] for j in extra]))
            removed += len(missing); added += len(extra)
            for i, j in matched:
                row = drift(old[i], new[j]); pairs.append(dict(id=token, **row))
                if row['tracker_confidence_regime_flip']:
                    flips.append(dict(id=token, old_score=old[i]['score'], new_score=new[j]['score']))
        stats = {k: distribution([row[k] for row in pairs]) for k in
            ('center_xy_m', 'size_linf_m', 'yaw_deg', 'score_abs', 'footprint_corner_m', 'rotated_bev_iou')}
        result[name] = dict(reference_count=old_count, candidate_count=new_count, matched=len(pairs),
            removed=removed, added=added, changed_delivery_frames=len(changes), delivery_changes=changes,
            tracker_confidence_regime_flips=flips, distributions=stats,
            yaw_gate_exceedances=sum(bool(row['yaw_deg'] > np.degrees(.02)) for row in pairs),
            score_gate_exceedances=sum(row['score_abs'] > .002 for row in pairs),
            largest_footprint_changes=sorted(pairs, key=lambda row: -row['footprint_corner_m'])[:10])
    return result


def analyze(report_path, output):
    if output.exists():
        raise ValueError('Fresh effects report required')
    blob = report_path.read_bytes(); report = json.loads(blob)
    if (not report.get('preserve_learned_config') or
            report['model_geometry']['classes'] != list(CLASSES) or
            report['decoder'] != dict(score_floor=.1, max_detections=100)):
        raise ValueError('Unchanged learned three-class ROS decoder contract required')
    tokens = [r['id'] for r in report['cases'][0]['per_sample']]
    if len(set(tokens)) != len(tokens):
        raise ValueError('Repeated scene tokens')
    reference = None; result = dict(source_report_sha256=hashlib.sha256(blob).hexdigest(),
        scope='Matched delivery/footprint diagnostics; not temporal tracking, held-out accuracy or live ROS certification.',
        matching='Class-preserving XY Hungarian association, strict <0.5m gate.',
        tracker_confidence_note='Current EKF multiplies measurement R by 10 when score <0.5; this is not a delivery threshold.',
        policies={k:list(v) for k,v in POLICIES.items()}, frames=len(tokens), cases=[])
    for index, case in enumerate(report['cases'], 1):
        if [r['id'] for r in case['per_sample']] != tokens:
            raise ValueError('Different ordered runtime scenes')
        data = json.loads((report_path.parent/f'{index}_{case["variant"]}_predictions.json').read_text())
        if set(data) != set(tokens):
            raise ValueError('Incomplete runtime export')
        if reference is None:
            reference = data
        result['cases'].append(dict(variant=case['variant'], policies={name: compare(reference,data,tokens,scores)
            for name,scores in POLICIES.items()}))
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for case in result['cases']:
        print(case['variant'], {k: dict(changed=v['changed_delivery_frames'],flips=len(v['tracker_confidence_regime_flips']),
            corner_max=v['distributions']['footprint_corner_m']['maximum']) for k,v in case['policies']['current'].items()})
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); analyze(a.report,a.output)
