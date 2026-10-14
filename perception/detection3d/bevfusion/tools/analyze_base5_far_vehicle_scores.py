#!/usr/bin/env python3
"""CPU-only vehicle score diagnostics on frozen full-ROI development outputs.

No ROS replay, map filter, track birth or coasting is simulated. Unmatched
predictions are spatial diagnostics, not adjudicated semantic false positives.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from compare_base5_common30 import validate_payload
from evaluate_base5_far_retention import (
    BANDS, band_name, classify_unmatched, full_prepare, range_operating,
)
from summarize_base5_rosbag_quick import bridge_nms

DEFAULT_SCORES = (.4, .35, .3, .2, .1)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def prepare_cases(records, payload):
    validate_payload(payload)
    sources = {r['token']: r for r in records}
    predictions = {r['token']: r for r in payload['rows']}
    if len(sources) != len(records) or set(sources) != set(predictions):
        raise ValueError('Frozen input tokens must be unique and match predictions exactly')
    cases = defaultdict(list)
    excluded = Counter()
    for source in records:
        row = predictions[source['token']]
        expected = np.asarray([i['bbox_3d'] for i in source['instances']], float).reshape(-1, 7)
        expected[:, 2] -= expected[:, 5] / 2
        actual = np.asarray(row['gt_boxes'], float).reshape(-1, 7)
        if expected.shape != actual.shape or not np.allclose(expected, actual, atol=1e-5, rtol=0):
            raise ValueError('Authored GT geometry differs from prediction GT')
        if [i['bbox_label_3d'] for i in source['instances']] != row['gt_labels']:
            raise ValueError('Authored GT labels differ from prediction GT')
        if 'scoring_eligible' not in source or 'raw_lidar_index' not in source:
            raise ValueError('Explicit full-ROI scoring eligibility and original frame index required')
        case = source['scene_id']
        if not source['scoring_eligible']:
            excluded[case] += 1
            continue
        merged = dict(row, case=case, stamp=source['timestamp'], native_index=source['raw_lidar_index'],
                      gt_track_keys=[case + '/' + str(i['track_key']) for i in source['instances']])
        # Keep wider catalog GT for diagnostics even when scoring excludes an
        # actor outside the semantic ROI. Matching itself uses ALL ROI GT.
        merged = bridge_nms(merged)
        merged.update(known_gt_boxes=np.asarray(row['gt_boxes'], float).reshape(-1, 7),
                      known_gt_labels=np.asarray(row['gt_labels']))
        cases[case].append(merged)
    prepared = {}
    for case, rows in cases.items():
        rows.sort(key=lambda r: r['stamp'])
        if any(b['stamp'] <= a['stamp'] or b['native_index'] <= a['native_index']
               for a, b in zip(rows, rows[1:])):
            raise ValueError('Sensor time and original indices must increase within each case')
        prepared[case] = full_prepare(rows)
    return prepared, dict(excluded)


def pooled_ranges(cases):
    result = {}
    for lo, hi in BANDS:
        name = f'{lo:g}_{hi:g}m'
        rows = [case['by_range'][name] for case in cases.values()]
        gt = sum(r['gt'] for r in rows)
        pred = sum(r['pred_in_band'] for r in rows)
        tp = {str(t): sum(r['matched_gt_counts'][str(t)] for r in rows) for t in (1., 2., 4.)}
        unmatched = {str(t): sum(r['unmatched_predictions_by_center_gate'][str(t)] for r in rows)
                     for t in (1., 2., 4.)}
        result[name] = dict(gt=gt, pred_in_band=pred, matched_gt_counts=tp,
                            recall_by_center_gate={t: n / gt if gt else None for t, n in tp.items()},
                            unmatched_predictions_by_center_gate=unmatched,
                            precision_by_center_gate={t: (pred - n) / pred if pred else None
                                                      for t, n in unmatched.items()})
    return result


def score_diagnostics(rows, score, reference_score=.4):
    kinds = Counter()
    by_band = defaultdict(Counter)
    gains = Counter()
    examples = defaultdict(list)
    for row in rows:
        for p in np.flatnonzero((row['pred_labels'] == 0) & (row['pred_scores'] >= score)):
            box = row['pred_boxes'][p]
            band = band_name(float(np.linalg.norm(box[:2])))
            if p in row['assignments'][0][2.]:
                if row['pred_scores'][p] < reference_score:
                    g = row['assignments'][0][2.][p]
                    gains[band_name(float(np.linalg.norm(row['gt_boxes'][g, :2])))] += 1
                continue
            kind, own, other = classify_unmatched(
                box, 0, row['known_gt_boxes'], row['known_gt_labels'])
            kinds[kind] += 1
            by_band[band][kind] += 1
            key = band + '/' + kind
            if len(examples[key]) < 5:
                examples[key].append(dict(token=row['token'], case=row['case'], stamp=row['stamp'],
                                          native_index=row['native_index'], box=box.tolist(),
                                          score=float(row['pred_scores'][p]), nearest_same_gt_m=own,
                                          nearest_other_gt_m=other))
    return dict(unmatched_kinds_gate2m=dict(kinds), unmatched_by_band_gate2m=dict(by_band),
                additional_matched_gt_observations_below_reference=dict(gains), examples=dict(examples))


def check_reference(result, reference):
    if reference['gt_identity'] != result['gt_identity']:
        raise ValueError('Reference GT identity differs')
    if reference['source_checkpoint_sha256'] != result['checkpoint_sha256']:
        raise ValueError('Reference checkpoint differs')
    baseline = result['scores']['0.4']['cases']
    for case, row in baseline.items():
        old = reference['detection']['local'][case]['vehicle']['by_range']
        for band, values in row['by_range'].items():
            for key in ('gt', 'pred_in_band', 'matched_gt_counts',
                        'unmatched_predictions_by_center_gate'):
                if values[key] != old[band][key]:
                    raise ValueError(f'Reference baseline changed: {case}/{band}/{key}')


def analyze(args):
    if args.output.exists():
        raise ValueError('Fresh output path required; do not overwrite frozen evidence')
    scores = tuple(float(s) for s in args.scores)
    if len(set(scores)) != len(scores) or .4 not in scores or any(
            not np.isfinite(s) or not .1 <= s <= 1 for s in scores):
        raise ValueError('Unique finite scores >= server floor 0.1, including baseline 0.4, required')
    paths = [args.inputs / 'infos_val.pkl', args.inputs / 'manifest.json', args.predictions]
    if args.reference_report:
        paths.append(args.reference_report)
    hashes = {str(p): sha256(p) for p in paths}
    records = pickle.loads(paths[0].read_bytes())['data_list']
    manifest = json.loads(paths[1].read_text())
    with gzip.open(args.predictions, 'rt') as f:
        payload = json.load(f)
    prepared, excluded = prepare_cases(records, payload)
    scoring_frames = sum(map(len, prepared.values()))
    if manifest['frames'] != len(records) or manifest['scoring_frames'] != scoring_frames:
        raise ValueError('Manifest input/scoring denominators differ')
    result = dict(scope='CPU detection-only development score sweep; production unchanged',
                  created_at_utc=datetime.now(timezone.utc).isoformat(),
                  model_name=payload['name'], checkpoint_sha256=payload['checkpoint_sha256'],
                  gt_identity=payload['gt_identity'], input_frames=len(records), scoring_frames=scoring_frames,
                  excluded_by_case=excluded, source_sha256=hashes,
                  contract=dict(server_floor=.1, server_topk=100, class_aware_rotated_nms=.5,
                                semantic_roi=[-100, -30, -3, 100, 30, 2],
                                pedestrian_score=.1, obstacle_score=.1, vehicle_reference_score=.4),
                  limitations=['Stored E24 detections only; no GPU, ROS tracker, map filter or planner execution.',
                               'Full-ROI matching precedes GT/prediction range attribution; boundaries can differ.',
                               'Catalog/interpolated GT omits map background; unmatched is not certified semantic FP.',
                               'Only scoring-eligible frames contribute; original frame gaps break miss episodes.',
                               'Repeated observations are correlated; AP/score are not detection probabilities.',
                               'Miss duration is sampled exposure, not exact invisibility or coasting duration.',
                               'Five examples per diagnostic cell are chronological, not a representative FP sample.'],
                  scores={})
    for score in scores:
        cases = {}
        for case, rows in prepared.items():
            periods = np.diff([r['stamp'] for r in rows])
            usable = periods[(periods > 0) & (periods <= .25)]
            period = float(np.median(usable)) if len(usable) else .1
            cases[case] = dict(frames=len(rows), nominal_period_sec=period,
                              by_range=range_operating(rows, 0, score, period),
                              diagnostics=score_diagnostics(rows, score))
        result['scores'][f'{score:g}'] = dict(cases=cases, pooled_by_range=pooled_ranges(cases))
    result = json_safe(result)
    if args.reference_report:
        check_reference(result, json.loads(args.reference_report.read_text()))
        result['baseline_reference_exact_counts'] = True
    if hashes != {str(p): sha256(p) for p in paths}:
        raise ValueError('Frozen input changed during analysis')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps(dict(report=str(args.output), scoring_frames=scoring_frames,
                          baseline_reference_exact_counts=result.get('baseline_reference_exact_counts')), indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--predictions', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference-report', type=Path)
    parser.add_argument('--scores', type=float, nargs='+', default=DEFAULT_SCORES)
    analyze(parser.parse_args())
