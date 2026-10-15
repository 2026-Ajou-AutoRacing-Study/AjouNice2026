#!/usr/bin/env python3
"""Uncensored fixed-actor diagnostics around native stale/fresh births.

Raw birth position chooses a clear nearby GT actor for evaluation only. The
old track remains an identity hypothesis, not proof of a rendered object or
a false positive. Its out-of-gate states are retained instead of disappearing
from the usual current-2m conditional metrics. No inputs to the EKF are made.
"""
import argparse
import gzip
import json
import math
from pathlib import Path

import numpy as np

from audit_tracking_continuity_events import sha
from audit_vehicle_reacquisition_core import vehicle_births
from qualify_vehicle_covariance_recovery import interpolate_gt, distribution


def load(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def actor_at_birth(gt, event):
    candidates = []
    for index, series in enumerate(gt['gt']):
        if not series[1] or series[1][0].get('class_name') != 'vehicle':
            continue
        point = interpolate_gt(*series, event['stamp'])
        if point is not None:
            candidates.append((float(np.linalg.norm(point - event['raw_measurement']['center'][:2])), index))
    candidates.sort()
    if not candidates or candidates[0][0] > 2.:
        return None
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < .5:
        return None
    return candidates[0][1]


def select_lineage(rows, initialization_time, reference_center, stamp):
    available = [(abs(row['stamp'] - stamp), row) for row in rows
                 if abs(row['stamp'] - stamp) <= .3]
    if not available:
        return None
    frame = min(available, key=lambda item: item[0])[1]
    tracks = [item for item in frame['tracks']
              if abs(item['initialization_time'] - initialization_time) <= 1.e-6]
    if not tracks:
        return None
    ranked = sorted((math.dist(item['center'][:2], reference_center[:2]), item['id']) for item in tracks)
    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < .5:
        return None
    return ranked[0][1], initialization_time


def fixed_actor_rows(rows, series, old_lineage, fresh_lineage, start, end):
    result = []
    for frame in rows:
        stamp = frame['stamp']
        if not start <= stamp <= end:
            continue
        current = interpolate_gt(*series, stamp)
        if current is None:
            continue
        selected = []
        for role, lineage in (('old', old_lineage), ('fresh', fresh_lineage)):
            if lineage is None:
                continue
            for item in frame['tracks']:
                if (item['id'] == lineage[0] and
                        abs(item['initialization_time'] - lineage[1]) <= 1.e-6 and
                        item['confirmed']):
                    before = interpolate_gt(*series, stamp - .05)
                    after = interpolate_gt(*series, stamp + .05)
                    velocity = np.asarray(item['velocity'])
                    sample = dict(role=role, id=item['id'], associated=item['associated'],
                                  last_measurement_time=item['last_measurement_time'],
                                  current_error_m=float(np.linalg.norm(np.asarray(item['center'][:2])-current)),
                                  speed_mps=float(np.linalg.norm(velocity)), velocity=item['velocity'])
                    if before is not None and after is not None:
                        target = (after - before) / .1
                        sample['gt_speed_mps'] = float(np.linalg.norm(target))
                        sample['velocity_error_mps'] = float(np.linalg.norm(velocity-target))
                    for horizon in (1., 3.):
                        future = interpolate_gt(*series, stamp + horizon)
                        if future is not None:
                            predicted = np.asarray(item['center'][:2])+horizon*velocity
                            sample[f'cartesian_cv_{horizon:g}s_m'] = float(np.linalg.norm(predicted-future))
                    selected.append(sample)
        if not selected:
            continue
        # Evaluation-only representative. Both outputs remain in the rows;
        # this is not a change to actual planner selection or publication.
        primary = max(selected, key=lambda item: (item['last_measurement_time'], item['associated'], item['role'] == 'fresh'))
        result.append(dict(stamp=stamp, outputs=selected, primary=primary,
                           concurrent_old_fresh=len(selected) == 2))
    return result


def paired_fixed_actor(baseline, candidate):
    left, right = ({round(row['stamp'], 6): row['primary'] for row in rows}
                   for rows in (baseline, candidate))
    keys = sorted(set(left) & set(right))
    result = dict(paired=len(keys), baseline_only=len(set(left)-set(right)),
                  candidate_only=len(set(right)-set(left)))
    for metric in ('current_error_m', 'velocity_error_mps', 'cartesian_cv_1s_m', 'cartesian_cv_3s_m'):
        eligible = [key for key in keys if metric in left[key] and metric in right[key]]
        result[metric] = dict(baseline=distribution([left[key][metric] for key in eligible]),
                             candidate=distribution([right[key][metric] for key in eligible]),
                             delta=distribution([right[key][metric]-left[key][metric] for key in eligible]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--gt', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--variant', action='append', required=True, help='NAME=core.json.gz')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh diagnostic output required')
    args.output.mkdir(parents=True)
    frames, gt, baseline = load(args.frames), load(args.gt), load(args.baseline)
    variants = dict(baseline=baseline)
    variant_sources = {}
    for argument in args.variant:
        name, filename = argument.split('=', 1)
        if name in variants:
            raise ValueError('Duplicate variant')
        variants[name] = load(Path(filename))
        variant_sources[name] = dict(path=filename, sha256=sha(Path(filename)))
    report = dict(events=[], gt_sha256=sha(args.gt), frames_sha256=sha(args.frames),
                  baseline_sha256=sha(args.baseline),
                  variant_sources=variant_sources,
                  scope='Fixed actor hypotheses around native birth; no current-distance censoring, Cartesian-CV only, no AP or actual planner selection claim.')
    for event in vehicle_births(frames, baseline):
        actor_index = actor_at_birth(gt, event)
        if actor_index is None:
            continue
        series = gt['gt'][actor_index]
        for stale in event['stale_candidates']:
            initialization = stale['after']['initialization_time']
            row = dict(stamp=event['stamp'], actor=series[1][0]['key'], asset=series[1][0]['asset'],
                       baseline_old_id=stale['prior']['id'], baseline_fresh_id=event['born']['id'],
                       old_track_identity_is_hypothesis=True, variants={}, paired={})
            outputs = {}
            for name, source in variants.items():
                old = select_lineage(source, initialization, stale['prior']['center'], event['stamp'])
                fresh = select_lineage(source, event['stamp'], event['born']['center'], event['stamp'])
                values = fixed_actor_rows(source, series, old, fresh, event['stamp']-.3, event['stamp']+.8)
                row['variants'][name] = dict(old_lineage=old, fresh_lineage=fresh,
                    uncensored_frames=len(values), concurrent_old_fresh_frames=sum(value['concurrent_old_fresh'] for value in values))
                outputs[name] = values
            for name, values in outputs.items():
                if name != 'baseline':
                    row['paired'][name] = paired_fixed_actor(outputs['baseline'], values)
            index = len(report['events'])
            with gzip.open(args.output/f'event_{index}_rows.json.gz', 'wt') as stream:
                json.dump(outputs, stream, allow_nan=False)
            report['events'].append(row)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(events=len(report['events']), output=str(args.output))), flush=True)


if __name__ == '__main__':
    main()
