#!/usr/bin/env python3
"""CPU-only comparison of fresh checkpoints on frozen full-ROI bag inputs.

Uses existing catalog-GT eligibility, bridge NMS, AP, and greedy matching.
Does not infer semantic truth for unmatched boxes or execute ROS/GPU/tracker.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_far_vehicle_scores import prepare_cases, json_safe
from calibrate_base5_thresholds import score_curve
from compare_base5_common30 import CLASSES, scope, validate_payload
from evaluate_base5_far_retention import full_prepare, metric_module, range_operating


VERSION = 'base5_frozen_bag_comparison_v1'
THRESHOLDS = (.4, .1, .1)
BANDS = ((0., 10.), (10., 20.), (20., 30.), (30., 40.), (40., 50.),
         (0., 30.), (30., 50.), (50., 80.), (80., 100.))
RUNTIME_KEYS = ('config_sha256', 'info_sha256', 'inference_precision',
                'point_intensity_scale', 'input_pipeline', 'inference_optimization',
                'script_sha256', 'gpu', 'torch_version')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare_contract(reference, candidate):
    """Comparison changes weights only, not the authored input or runtime."""
    if reference['gt_identity'] != candidate['gt_identity']:
        raise ValueError('Checkpoints must use identical ordered GT tokens')
    for key in RUNTIME_KEYS:
        if key not in reference or key not in candidate or reference[key] != candidate[key]:
            raise ValueError('Checkpoint runtime/input contract differs: ' + key)
    optimization = reference['inference_optimization']
    if optimization.get('compile_parts') or optimization.get('camera_cuda_graph'):
        raise ValueError('Compile/CUDA-graph candidates are outside this comparison')
    if reference['point_intensity_scale'] != 1. / 255.:
        raise ValueError('Frozen ROS XYZI requires the explicit single /255 adapter')
    if not reference.get('strict_state_load') or not candidate.get('strict_state_load'):
        raise ValueError('Both checkpoint exports must strictly load state')


def verify_authored_support(records, payload):
    authored = {r['token']: r for r in records}
    for row in payload['rows']:
        source = authored.get(row['token'])
        if source is None:
            raise ValueError('Unknown prediction token')
        instances = source['instances']
        if row['gt_points'] != [i['num_lidar_pts'] for i in instances]:
            raise ValueError('Authored GT support differs from export')
        if row['gt_assets'] != [i.get('asset', 'legacy_unknown') for i in instances]:
            raise ValueError('Authored GT assets differ from export')


def ap_summary(rows, metric):
    values = metric._evaluate_center(rows, list(CLASSES), metric.DEFAULT_CENTER_THRESHOLDS,
                                    2., .1, .1, 'center', include_tp=False,
                                    include_operating_points=False)
    classes = {}
    present = []
    for name in CLASSES:
        gt = int(values['center/' + name + '_GT'])
        ap = values['center/' + name + '_mean_AP'] if gt else None
        if ap is not None:
            present.append(ap)
        classes[name] = dict(gt=gt, ap=ap, ap_by_gate={
            str(t): values[f'center/{name}_AP_{t:g}m'] if gt else None
            for t in metric.DEFAULT_CENTER_THRESHOLDS})
    return dict(frames=len(rows), classes=classes,
                present_classes_mAP=float(np.mean(present)) if present else None)


def summarize_rows(rows, metric, period=None):
    result = dict(semantic_roi=ap_summary(rows, metric), ap_by_range={},
                  common30=ap_summary([scope(r) for r in rows], metric), operating={})
    for lo, hi in BANDS:
        result['ap_by_range'][f'{lo:g}_{hi:g}m'] = ap_summary(
            metric._filter_range(rows, (lo, hi)), metric)
    for label, name in enumerate(CLASSES):
        ranges = range_operating(rows, label, THRESHOLDS[label], period or .1)
        if period is None:
            # Pooled cases have distinct sensor cadences. Counts and geometric
            # errors pool correctly, but no single nominal period defines time.
            for band in ranges.values():
                band['miss_runs_gate2m'] = None
        result['operating'][name] = dict(
            semantic_roi=score_curve(rows, label, THRESHOLDS[label],
                                     temporal=period is not None, period=period or .1),
            by_range=ranges)
    common = full_prepare([scope(r) for r in rows])
    result['common30_operating'] = {name: score_curve(
        common, label, THRESHOLDS[label], temporal=period is not None, period=period or .1)
        for label, name in enumerate(CLASSES)}
    return result


def summarize(args):
    if args.output.exists():
        raise ValueError('Fresh output required; never overwrite frozen evidence')
    inputs = args.inputs.resolve()
    paths = [inputs / 'infos_val.pkl', inputs / 'manifest.json', *args.predictions]
    hashes = {str(p): digest(p) for p in paths}
    records = pickle.loads(paths[0].read_bytes())['data_list']
    manifest = json.loads(paths[1].read_text())
    if len(records) != manifest['frames'] or not 0 < len(records) <= args.max_frames:
        raise ValueError('Explicit bounded full-input frame count required')
    if not 0 < manifest.get('scoring_frames', 0) <= len(records):
        raise ValueError('Full-ROI frozen scoring eligibility is required')
    report = dict(version=VERSION, complete=False, input_frames=len(records),
                  scoring_frames=manifest['scoring_frames'], source_sha256=hashes,
                  script_sha256=digest(__file__), models={},
                  contract=dict(class_scores=dict(zip(CLASSES, THRESHOLDS)),
                                server_floor=.1, server_top100=True, class_aware_nms_iou=.5,
                                semantic_roi=[-100., -30., -3., 100., 30., 2.],
                                box_z='bottom', ap_xy_gates=[.5, 1., 2., 4.],
                                gt_reference=manifest.get('gt_reference'),
                                known_development_bags=True),
                  limitations=[
                      'Object/Ego interpolation and audited catalog geometry, not synchronous native annotation.',
                      'Raw Ego heading correction was applied by input preparation; do not apply it again.',
                      'Unmatched/FP counts are catalog-relative diagnostics, not adjudicated semantic ghosts.',
                      'Correlated repeated bag observations; this is not untouched test or real-time certification.',
                      'Scoring-ineligible frames are excluded identically; no LiDAR-support GT deletion.',
                      'AP filters each distance band; fixed-score range matching uses all semantic-ROI GT first.',
                      'Pooled miss durations are null; use case-specific cadence and gap-bounded miss runs.',
                      'No tracker, map-CV, planner, ROS queue, or LAN timing is executed.',
                      'present_classes_mAP excludes GT-empty classes; class AP is null when GT is zero.'])
    metric = metric_module()
    reference = None
    for path in args.predictions:
        with gzip.open(path, 'rt') as handle:
            payload = json.load(handle)
        validate_payload(payload)
        if payload['info_sha256'] != hashes[str(paths[0])]:
            raise ValueError('Prediction info SHA differs from frozen input')
        compare_contract(payload if reference is None else reference, payload)
        reference = payload if reference is None else reference
        verify_authored_support(records, payload)
        cases, excluded = prepare_cases(records, payload)
        if sum(map(len, cases.values())) != manifest['scoring_frames']:
            raise ValueError('Scoring-frame count differs from frozen manifest')
        if payload['name'] in report['models']:
            raise ValueError('Unique checkpoint names required')
        model = dict(receipt={k: v for k, v in payload.items() if k != 'rows'},
                     excluded_by_case=excluded, cases={})
        for case, rows in cases.items():
            deltas = np.diff([r['stamp'] for r in rows])
            usable = deltas[(deltas > 0) & (deltas <= .25)]
            period = float(np.median(usable)) if len(usable) else .1
            model['cases'][case] = summarize_rows(rows, metric, period)
        model['pooled'] = summarize_rows([r for rows in cases.values() for r in rows], metric)
        report['models'][payload['name']] = model
        print(json.dumps(dict(event='scored', model=payload['name'],
                              scoring_frames=manifest['scoring_frames'])), flush=True)
    if hashes != {str(p): digest(p) for p in paths}:
        raise ValueError('Frozen comparison inputs changed while scoring')
    report.update(gt_identity=reference['gt_identity'], complete=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--predictions', type=Path, nargs=2, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=2000)
    args = parser.parse_args()
    summarize(args)


if __name__ == '__main__':
    main()
