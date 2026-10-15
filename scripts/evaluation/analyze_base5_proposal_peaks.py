#!/usr/bin/env python3
"""CPU-only paired analysis of frozen-feature query proposals.

Native full semantic ROI and previously evaluated bag radial30 are separate.
Same-class proximity extras are diagnostics, not adjudicated duplicate truth.
No production config, GT or dataset file is modified.
"""

import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'perception/detection3d/bevfusion/tools'))
from calibrate_base5_thresholds import assignments, roi
from compare_base5_common30 import scope
from summarize_base5_rosbag_quick import bridge_nms
from audit_base5_offline_regression import angle_error_pi

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
THRESHOLDS = (.4, .1, .1)
BANDS = ((0, 10), (10, 20), (20, 30), (30, 50), (50, 80), (80, 100), (100, float('inf')))


def distribution(values):
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), mean=float(a.mean()) if len(a) else None,
                **{k: float(np.percentile(a, p)) if len(a) else None
                   for k, p in (('p50', 50), ('p90', 90), ('p95', 95), ('max', 100))})


def prepared(raw, final=False):
    value = bridge_nms(raw) if final else dict(raw)
    value['gt_parent_uids'] = value['parent_uids']
    value = roi(value) if raw['source'] == 'native' else scope(value)
    value['parent_uids'] = value['gt_parent_uids']
    for key in ('pred_labels', 'pred_scores', 'gt_labels', 'gt_points', 'parent_uids'):
        value[key] = np.asarray(value[key])
    return value


def eligible_matches(row, label, tolerance=1):
    pairs = assignments(row, label, tolerance)
    return {g: p for p, g in pairs.items() if row['pred_scores'][p] >= THRESHOLDS[label]}


def near_score_factor(raw, gt_index, label):
    r = prepared(raw)
    candidates = np.flatnonzero((r['pred_labels'] == label) &
        (np.linalg.norm(r['pred_boxes'][:, :2] - r['gt_boxes'][gt_index, :2], axis=1) < 1))
    if not len(candidates):
        return None
    p = int(candidates[np.argmax(r['pred_scores'][candidates])])
    score = float(r['pred_scores'][p])
    queries = [q for q in raw['query_scores'] if q['label'] == label and
               math.isclose(q['product'], score, rel_tol=2e-6, abs_tol=1e-9)]
    return dict(score=score, dense=queries[0]['dense'] if len(queries) == 1 else None,
                classifier=queries[0]['classifier'] if len(queries) == 1 else None,
                factor_match_unique=len(queries) == 1)


def summarize(rows, final):
    counts = {name: dict(gt=0, tp1m=0, tp2m=0, predictions=0, unmatched=0, proximity_extras=0,
                        gt_ambiguous_extra=0, bands={}) for name in CLASSES}
    stats, queries = defaultdict(list), Counter()
    for raw in rows:
        row = prepared(raw, final)
        for query in raw['query_scores']:
            label = query['label']
            queries[CLASSES[label]] += 1
            for component in ('dense', 'classifier', 'product'):
                stats[f'query/{CLASSES[label]}/{component}'].append(query[component])
        for label, name in enumerate(CLASSES):
            gt = np.flatnonzero(row['gt_labels'] == label)
            ps = np.flatnonzero((row['pred_labels'] == label) & (row['pred_scores'] >= THRESHOLDS[label]))
            matched, match2 = eligible_matches(row, label), eligible_matches(row, label, 2)
            c = counts[name]
            c['gt'] += len(gt)
            c['tp1m'] += len(matched)
            c['tp2m'] += len(match2)
            c['predictions'] += len(ps)
            c['unmatched'] += len(ps) - len(matched)
            for p in ps:
                if p in matched.values():
                    continue
                ds = np.linalg.norm(row['gt_boxes'][gt, :2] - row['pred_boxes'][p, :2], axis=1)
                near = gt[ds < 1]
                if len(near) == 1 and int(near[0]) in matched:
                    c['proximity_extras'] += 1
                elif len(near) > 1:
                    c['gt_ambiguous_extra'] += 1
            for g in gt:
                distance = float(np.linalg.norm(row['gt_boxes'][g, :2]))
                lo, hi = next((a, b) for a, b in BANDS if a <= distance < b)
                key = f'{lo:g}_{hi:g}m'
                band = c['bands'].setdefault(key, dict(gt=0, tp1m=0, ge20_points_gt=0, ge20_points_tp1m=0))
                band['gt'] += 1
                band['tp1m'] += int(g) in matched
                band['ge20_points_gt'] += row['gt_points'][g] >= 20
                band['ge20_points_tp1m'] += row['gt_points'][g] >= 20 and int(g) in matched
                own = np.flatnonzero((row['pred_labels'] == label) &
                        (np.linalg.norm(row['pred_boxes'][:, :2] - row['gt_boxes'][g, :2], axis=1) < 1))
                if len(own):
                    p = int(own[np.argmax(row['pred_scores'][own])])
                    score = float(row['pred_scores'][p])
                    stats[f'gt_near_raw/{name}/score'].append(score)
                    # Query order is not final score-sorted prediction order.
                    # Attribute factors only when the product gives a unique match.
                    qs = [q for q in raw['query_scores'] if q['label'] == label and
                          math.isclose(q['product'], score, rel_tol=2e-6, abs_tol=1e-9)]
                    if len(qs) == 1:
                        for component in ('dense', 'classifier'):
                            stats[f'gt_near_raw/{name}/{component}'].append(qs[0][component])
                    else:
                        queries[f'gt_factor_unresolved/{name}'] += 1
    return dict(frames=len(rows), classes=counts, query_counts=dict(queries),
                distributions={k: distribution(v) for k, v in stats.items()})


def paired(left, right, final=True):
    if [r['token'] for r in left] != [r['token'] for r in right]:
        raise ValueError('Paired input tokens differ')
    totals = {name: Counter() for name in CLASSES}
    changes, examples, close = defaultdict(list), [], []
    lost_kinds, lost_assets = Counter(), Counter()
    for a, b in zip(left, right):
        if any(a[k] != b[k] for k in ('gt_boxes', 'gt_labels', 'parent_uids', 'point_sha256', 'image_sha256')):
            raise ValueError('Paired GT/raw identity changed')
        ra, rb = prepared(a, final), prepared(b, final)
        if not np.array_equal(ra['gt_boxes'], rb['gt_boxes']):
            raise ValueError('Prepared GT differs')
        for label, name in enumerate(CLASSES):
            ma, mb = eligible_matches(ra, label), eligible_matches(rb, label)
            appeared, lost = set(mb) - set(ma), set(ma) - set(mb)
            totals[name].update(appeared=len(appeared), lost=len(lost), both=len(set(ma) & set(mb)))
            for kind, ids in (('appeared', appeared), ('lost', lost)):
                for g in sorted(ids):
                    before = near_score_factor(a, g, label)
                    after = near_score_factor(b, g, label)
                    if kind == 'lost':
                        lost_kinds[f'{name}/' + ('no_near_same_class_raw_box' if after is None else
                            'near_same_class_below_threshold' if after['score'] < THRESHOLDS[label] else
                            'near_eligible_box_taken_or_suppressed')] += 1
                        lost_assets[f'{name}/{ra["gt_assets"][g]}'] += 1
                        if before is not None and after is not None:
                            for factor in ('score', 'dense', 'classifier'):
                                if before[factor] is not None and after[factor] is not None:
                                    changes[f'lost/{name}/baseline_{factor}'].append(before[factor])
                                    changes[f'lost/{name}/candidate_{factor}'].append(after[factor])
                    if len(examples) < 100:
                        examples.append(dict(token=a['token'], cls=name, kind=kind, asset=str(ra['gt_assets'][g]),
                                             range_m=float(np.linalg.norm(ra['gt_boxes'][g, :2])),
                                             gt_points=int(ra['gt_points'][g]), before=before, after=after))
            for g in set(ma) & set(mb):
                pa, pb = ra['pred_boxes'][ma[g]], rb['pred_boxes'][mb[g]]
                for metric, value in dict(
                    center_delta_m=np.linalg.norm(pa[:2] - pb[:2]),
                    axis_yaw_delta_deg=math.degrees(angle_error_pi(pa[6], pb[6])),
                    score_delta=abs(ra['pred_scores'][ma[g]] - rb['pred_scores'][mb[g]]),
                    length_delta_m=abs(pa[3] - pb[3]), width_delta_m=abs(pa[4] - pb[4]),
                    baseline_center_error_m=np.linalg.norm(pa[:2] - ra['gt_boxes'][g, :2]),
                    candidate_center_error_m=np.linalg.norm(pb[:2] - ra['gt_boxes'][g, :2]),
                    baseline_axis_error_deg=math.degrees(angle_error_pi(pa[6], ra['gt_boxes'][g, 6])),
                    candidate_axis_error_deg=math.degrees(angle_error_pi(pb[6], ra['gt_boxes'][g, 6]))).items():
                    changes[f'{name}/{metric}'].append(float(value))
            if a['source'] != 'native':
                continue
            gs = np.flatnonzero(ra['gt_labels'] == label)
            for i, g in enumerate(gs):
                for h in gs[i+1:]:
                    parents = ra['parent_uids'][[g, h]]
                    if any(v is None for v in parents) or parents[0] == parents[1]:
                        continue
                    gap = float(np.linalg.norm(ra['gt_boxes'][g, :2] - ra['gt_boxes'][h, :2]))
                    if gap < 1.5:
                        close.append(dict(token=a['token'], cls=name, assets=[str(ra['gt_assets'][v]) for v in (g,h)],
                                          centers_m=gap, baseline_hits=sum(int(v) in ma for v in (g,h)),
                                          candidate_hits=sum(int(v) in mb for v in (g,h))))
    return dict(classes={k: dict(v) for k, v in totals.items()},
                distributions={k: distribution(v) for k, v in changes.items()},
                lost_partition=dict(lost_kinds), lost_by_asset=dict(lost_assets),
                changed_hit_examples=examples, distinct_close_pairs=close)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError('Use a fresh output')
    payloads = {}
    for policy in ('legacy_none', 'peak3', 'peak5'):
        path = args.inputs / (policy + '.json.gz')
        with gzip.open(path, 'rt') as stream:
            payloads[policy] = json.load(stream)
    identities = {(v['input_sha256'], v['checkpoint_sha256']) for v in payloads.values()}
    if len(identities) != 1:
        raise ValueError('Different model or input identity')
    report = dict(schema='base5_query_peak_frozen_feature_v1', input_identity=list(identities)[0],
                  operating_thresholds=dict(zip(CLASSES, THRESHOLDS)), floor=.1, top=100, nms_iou=.5,
                  limitations=['One shared E24 backbone feature per frame isolates head query policy; no independently executed candidate backbone latency claim.',
                               'Native full semantic ROI and dev radial30 GT kept separate. Development sets are not untouched test evidence.',
                               'Four-bag50-frame subsets are not consecutive; no ID/coasting/temporal certification.',
                               'Proximity extra with single matched GT is not independently adjudicated duplicate truth.',
                               'Rare close distinct-parent pairs are an explicit failure check, not a sufficient close-object population.',
                               'Raw product factor attribution omitted when non-unique. Distribution bins are not AP.'], groups={})
    by_group = defaultdict(dict)
    for policy, payload in payloads.items():
        for source in sorted({r['case'] for r in payload['rows']}):
            by_group[source][policy] = [r for r in payload['rows'] if r['case'] == source]
    for name, data in by_group.items():
        result = dict(policies={policy: dict(raw=summarize(rows, False), final=summarize(rows, True))
                                for policy, rows in data.items()})
        result['paired_final'] = {policy: paired(data['legacy_none'], data[policy]) for policy in ('peak3', 'peak5')}
        report['groups'][name] = result
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False, default=lambda v: int(v) if isinstance(v, np.integer) else bool(v)) + '\n')
    print(json.dumps({name: {policy: {cls: (r['gt'], r['tp1m'], r['unmatched'], r['proximity_extras'])
                                    for cls, r in values['final']['classes'].items()}
                            for policy, values in group['policies'].items()}
                      for name, group in report['groups'].items()}, indent=2))


if __name__ == '__main__':
    main()
