#!/usr/bin/env python3
"""Checkpoint selection on frozen development GT, never a production promotion.

attach: reuse the already audited far-GT package without rebuilding trajectories.
curves: sweep 71 scores on the frozen scoring-eligible near/full-ROI population.
summary: aggregate exact counts, retain per-case/range results and validate ROS
independent repeats. Native validation and correlated bag observations remain
separate. Every output must be fresh; no sensor, source prediction or GT edits.
"""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from compare_base5_common30 import CLASSES, gt_identity, validate_payload


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def attach_gt(payload, sources):
    validate_payload(payload)
    if len(sources) != payload['frames'] or set(sources) != {r['token'] for r in payload['rows']}:
        raise ValueError('Frozen complete sensor tokens differ')
    result = copy.deepcopy(payload)
    for row in result['rows']:
        instances = sources[row['token']]['instances']
        boxes = np.asarray([r['bbox_3d'] for r in instances], float).reshape(-1, 7)
        boxes[:, 2] -= boxes[:, 5] / 2
        row.update(gt_boxes=boxes.tolist(), gt_labels=[r['bbox_label_3d'] for r in instances],
                   gt_points=[r['num_lidar_pts'] for r in instances],
                   gt_assets=[r.get('asset', 'unknown') for r in instances])
    result['gt_identity'] = gt_identity(result['rows'])
    for old, new in zip(payload['rows'], result['rows']):
        if any(old[key] != new[key] for key in ('token', 'pred_boxes', 'pred_labels', 'pred_scores')):
            raise ValueError('Detector outputs changed during GT attachment')
    validate_payload(result)
    return result


def attach(args):
    if args.output.exists():
        raise ValueError('Fresh output required')
    manifest = json.loads((args.inputs / 'manifest.json').read_text())
    info_path = args.inputs / 'infos_val.pkl'
    infos = pickle.loads(info_path.read_bytes())['data_list']
    sources = {r['token']: r for r in infos}
    if len(sources) != len(infos) or sum(bool(r['scoring_eligible']) for r in infos) != manifest['scoring_frames']:
        raise ValueError('Duplicate tokens or scoring mask mismatch')
    if not all(c['old_gt_geometry_unchanged'] for c in manifest['cases'].values()):
        raise ValueError('Audited geometry preservation required')
    results = []
    original_contract = None
    for path in args.predictions:
        with gzip.open(path, 'rt') as f:
            old = json.load(f)
        contract = (old['info_sha256'], old['gt_identity'], old['config_sha256'],
                    old['inference_precision'], old['inference_optimization'])
        if original_contract is not None and contract != original_contract:
            raise ValueError('Model comparison differs in inputs/GT/config/precision/optimization')
        original_contract = contract
        if old['info_sha256'] != manifest['original_info_sha256']:
            raise ValueError('Far GT does not originate from these frozen input infos')
        new = attach_gt(old, sources)
        new.update(info_sha256=digest(info_path), gt_extension=dict(
            original_prediction_file=str(path.resolve()), original_prediction_sha256=digest(path),
            original_gt_identity=old['gt_identity'], detector_outputs_unchanged=True,
            frozen_gt_manifest_sha256=digest(args.inputs / 'manifest.json')))
        results.append(new)
    if len({r['name'] for r in results}) != len(results) or len({r['gt_identity'] for r in results}) != 1:
        raise ValueError('Unique model names and identical attached GT required')
    args.output.mkdir(parents=True)
    for payload in results:
        with gzip.open(args.output / (payload['name'] + '.json.gz'), 'wt') as f:
            json.dump(payload, f, allow_nan=False)
    (args.output / 'receipt.json').write_text(json.dumps(dict(
        frozen_gt_inputs=str(args.inputs.resolve()), info_sha256=digest(info_path),
        input_frames=len(infos), scoring_frames=manifest['scoring_frames'],
        gt_identity=results[0]['gt_identity'], models=[{k: r[k] for k in (
            'name','checkpoint_sha256','inference_precision','inference_optimization','gt_extension')}
            for r in results]), indent=2, allow_nan=False) + '\n')


def curves(args):
    from calibrate_base5_thresholds import roi, sweep
    from compare_base5_common30 import scope
    from summarize_base5_rosbag_quick import bridge_nms
    if args.output.exists():
        raise ValueError('Fresh curves report required')
    info_path = args.inputs / 'infos_val.pkl'
    infos = pickle.loads(info_path.read_bytes())['data_list']
    sources = {r['token']: r for r in infos}
    result = dict(scope='Frozen scoring-eligible dev bags; 71 scores 0.10..0.80, not a new test or automatic optimum.',
                  models={}, source_sha256={})
    identity = None
    for path in args.predictions:
        with gzip.open(path, 'rt') as f:
            p = json.load(f)
        validate_payload(p)
        if p['info_sha256'] != digest(info_path) or set(sources) != {r['token'] for r in p['rows']}:
            raise ValueError('Curves require complete predictions attached to exactly these frozen infos')
        if identity is not None and identity != p['gt_identity']:
            raise ValueError('Different model GT populations')
        identity = p['gt_identity']
        cases = {}
        for row in p['rows']:
            src = sources[row['token']]
            if not src['scoring_eligible']:
                continue
            instances = src['instances']
            r = bridge_nms(dict(row, gt_track_keys=[str(v['track_key']) for v in instances]))
            r.update(stamp=src['timestamp'], native_index=src['raw_lidar_index'])
            cases.setdefault(src['scene_id'], []).append(r)
        result['models'][p['name']] = {case: dict(
            full_roi=sweep([roi(r) for r in sorted(rows, key=lambda r:r['stamp'])], True),
            radial30=sweep([scope(r) for r in sorted(rows, key=lambda r:r['stamp'])], True))
            for case,rows in cases.items()}
        result['source_sha256'][str(path)] = digest(path)
        print('Swept frozen near/far', p['name'], flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


def aggregate(classes):
    result = {}
    for cls in CLASSES:
        rows = [r[cls] for r in classes]
        gt = sum(r['overall']['gt'] for r in rows)
        pred = sum(r['overall']['pred'] for r in rows)
        tp = sum(r['overall']['tp'] for r in rows)
        tp2 = sum(r['overall']['center2m']['tp'] for r in rows)
        result[cls] = dict(gt=gt, pred=pred, tp1m=tp, unmatched1m=pred-tp,
            tp2m=tp2, unmatched2m=pred-tp2, recall1m=tp/gt if gt else None,
            recall2m=tp2/gt if gt else None, precision2m=tp2/pred if pred else None,
            miss_ge0p5s=sum(r['overall']['miss_runs']['episodes_ge_0p5s'] for r in rows),
            miss_ge1s=sum(r['overall']['miss_runs']['episodes_ge_1s'] for r in rows), by_range={})
        for band in rows[0]['by_range']:
            bs = [r['by_range'][band] for r in rows]
            g = sum(b['gt'] for b in bs)
            p = sum(b['pred_in_band'] for b in bs)
            matched = {t: sum(b['matched_gt_counts'][t] for b in bs) for t in ('1.0','2.0','4.0')}
            unmatched = {t: sum(b['unmatched_predictions_by_center_gate'][t] for b in bs)
                         for t in matched}
            result[cls]['by_range'][band] = dict(gt=g, pred_in_band=p,
                matched_gt_counts=matched, unmatched_predictions_by_center_gate=unmatched,
                recall={t: matched[t]/g if g else None for t in matched})
        if all('unmatched_persistence' in r for r in rows):
            result[cls]['persistence'] = dict(
                episodes_ge0p5s=sum(r['unmatched_persistence']['episodes_ge_0p5s'] for r in rows),
                episodes_ge1s=sum(r['unmatched_persistence']['episodes_ge_1s'] for r in rows),
                no_catalog_actor_observations=sum(r['unmatched_persistence']['kinds'].get(
                    'no_catalog_actor_within4m', 0) for r in rows),
                forward_corridor_no_actor=sum(r['unmatched_persistence']['forward_corridor_kinds'].get(
                    'no_catalog_actor_within4m', 0) for r in rows))
    return result


def verify_repeat(replay):
    receipt = json.loads((replay / 'receipt.json').read_text())
    with gzip.open(replay / 'messages.json.gz', 'rt') as f:
        data = json.load(f)
    for case in receipt['cases']:
        key = case['case']
        for stage in ('raw', 'policy'):
            a, b = (data[f'{key}/{mode}/{stage}'] for mode in ('off', 'off_repeat'))
            if len(a) != case['input_frames'] or a != b:
                raise ValueError('Missing output or nondeterministic independent reference repeat')
            candidate = data[f'{key}/on/{stage}']
            if len(candidate) != case['input_frames'] or [v['stamp'] for v in a] != [v['stamp'] for v in candidate]:
                raise ValueError('Candidate output coverage differs')
            if len({v['stamp'] for v in a}) != len(a):
                raise ValueError('Repeated measurement output timestamps')
    return dict(input_frames=sum(c['input_frames'] for c in receipt['cases']),
                exact_reference_repeat=True, receipt_sha256=digest(replay / 'receipt.json'))


def summarize(args):
    if args.output.exists():
        raise ValueError('Fresh summary required')
    result = dict(scope='Checkpoint/threshold selection on existing development data, not held-out or safety certification.',
        caveats=['Native 745-frame val and bag observations are separate populations.',
                 'Bag GT is bounded status/catalog approximation; unmatched is not necessarily a hallucination.',
                 'Repeated frames are correlated; misses split at original frame/GT/scoring gaps.',
                 'Frozen 1,570-frame replay includes 157 frames excluded only from accuracy.',
                 'Semantic ROI X±100/Y±30 and radial bands are different; 100m corners stay separate.'],
        models={}, replay_checks={})
    contracts = []
    for path in args.reports:
        r = json.loads(path.read_text())
        name = r['model_name']
        if name in result['models']:
            raise ValueError('Duplicate model reports')
        contracts.append((r['gt_identity'], r['input_frames'], r['scoring_frames']))
        detections = {k: aggregate(list(cases.values())) for k, cases in r['detection'].items()
                      if k != 'ap_by_range'}
        tracking = {k: {mode: dict(thresholds=v['thresholds'],
                            aggregate=aggregate(list(v['cases'].values()))) for mode,v in modes.items()}
                    for k,modes in r['tracking'].items()}
        result['models'][name] = dict(checkpoint_sha256=r['source_checkpoint_sha256'],
            report_path=str(path), report_sha256=digest(path), ap_by_range=r['detection']['ap_by_range'],
            detection=detections, tracking=tracking,
            per_case='See linked report; never replace case/band results with only pooled counts.')
    if len(set(contracts)) != 1:
        raise ValueError('Model scoring populations/GT differ')
    result.update(gt_identity=contracts[0][0], input_frames=contracts[0][1], scoring_frames=contracts[0][2])
    replay_contract = None
    for path in args.replays:
        receipt = json.loads((path / 'receipt.json').read_text())
        contract = (receipt['binary_sha256'], receipt['source_sha256'],
                    receipt['reference_coast_times'], receipt['fixed_output_rate_hz'],
                    receipt['maximum_range_m'], receipt['gt_identity'])
        if replay_contract is not None and contract != replay_contract:
            raise ValueError('Checkpoint comparison changed tracker, policy, scoring GT or output contract')
        replay_contract = contract
        if receipt.get('candidate_coast_times') is not None or receipt['gt_identity'] != result['gt_identity']:
            raise ValueError('Threshold-only comparison on identical frozen GT required')
        if receipt['checkpoint_sha256'] not in {v['checkpoint_sha256'] for v in result['models'].values()}:
            raise ValueError('Replay checkpoint absent from detector comparison')
        result['replay_checks'][str(path)] = verify_repeat(path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    a = sub.add_parser('attach')
    a.add_argument('--inputs', type=Path, required=True)
    a.add_argument('--predictions', type=Path, nargs='+', required=True)
    a.add_argument('--output', type=Path, required=True)
    c = sub.add_parser('curves')
    c.add_argument('--inputs', type=Path, required=True)
    c.add_argument('--predictions', type=Path, nargs='+', required=True)
    c.add_argument('--output', type=Path, required=True)
    s = sub.add_parser('summary')
    s.add_argument('--reports', type=Path, nargs='+', required=True)
    s.add_argument('--replays', type=Path, nargs='*', default=[])
    s.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    {'attach': attach, 'curves': curves, 'summary': summarize}[args.action](args)
