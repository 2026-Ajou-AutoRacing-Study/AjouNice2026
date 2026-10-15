#!/usr/bin/env python3
"""Frozen new-scene direction guard audit; no production changes or GT input.

Center/ID/state parity is mandatory. Motion GT is a bracketed catalog-center
diagnostic, not visibility-qualified AP or a live 30Hz/closed-loop benchmark.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import gzip
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze_live_perception_candidates import tracker_frames, correspondence, paired, ROOT
from audit_tracking_continuity import match, sample_series, stats, in_roi
from audit_tracking_continuity_events import build_original, replay, core_summary, sha
from audit_vehicle_reacquisition_core import vehicle_births
from build_vehicle_direction_probe import build as build_guard


def save_gzip(path, value):
    with gzip.open(path, 'wt') as stream:
        json.dump(value, stream, allow_nan=False)


def prepare_repeat_inputs(source, output, per_case=20):
    """Deterministic moving-Ego A/A subset; never use it as tracker history."""
    if output.exists():
        raise FileExistsError('Fresh repeat input path required')
    if per_case < 1:
        raise ValueError('Positive bounded repeat sample count required')
    original = json.loads(source.read_text())
    groups = defaultdict(list)
    for row in original['rows']:
        if row.get('scene_epoch_valid', False):
            groups[row['case']].append(row)
    selected = []
    for name, rows in sorted(groups.items()):
        moving = []
        for previous, row in zip(rows, rows[1:]):
            dt = row['stamp']-previous['stamp']
            distance = math.hypot(row['ego']['x']-previous['ego']['x'], row['ego']['y']-previous['ego']['y'])
            if 0 < dt <= .2 and distance/dt >= .5:
                moving.append(row)
        if not moving:
            raise ValueError('Moving cohort required: '+name)
        indices = np.unique(np.rint(np.linspace(0, len(moving)-1, min(per_case, len(moving)))).astype(int))
        selected.extend(moving[i] for i in indices)
    payload = dict(original, rows=selected, cases=dict(Counter(r['case'] for r in selected)),
                   continuous_for_tracker=False, source_inputs_sha256=sha(source),
                   scope='Moving-Ego raw forward A/A only; not continuous tracking or GT model performance')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(repeat_frames=len(selected), per_case=per_case)), flush=True)


def audit_repeat_outputs(full_path, repeat_folder, output):
    if output.exists():
        raise FileExistsError('Fresh repeat report required')
    with gzip.open(full_path, 'rt') as stream:
        full = json.load(stream)['rows']
    payloads = []
    for mode in ('nearest_A', 'nearest_B'):
        with gzip.open(repeat_folder/(mode+'.json.gz'), 'rt') as stream:
            payloads.append(json.load(stream)['rows'])
    by_token = {row['token']: row for row in full}
    selected = [by_token[row['token']] for row in payloads[0]]
    report = dict(full_source_sha256=sha(full_path),
        repeated_A_B=paired(payloads[0], payloads[1]), full_vs_repeat=paired(selected, payloads[0]),
        scope='Identical original moving inputs, raw and operating NMS output; not latency certification')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value['counts'] for key, value in report.items() if isinstance(value, dict)}), flush=True)


def velocity_at(series, stamp, maximum_gap=.2):
    """Strictly bracket a center derivative; never interpolate across a birth."""
    times, objects = series
    index = bisect_left(times, stamp)
    if index <= 0 or index >= len(times)-1:
        return None
    left, right = index-1, index+1
    span = times[right]-times[left]
    if span <= 0 or any(b-a > maximum_gap for a, b in zip(times[left:right], times[left+1:right+1])):
        return None
    return (np.asarray(objects[right]['center'][:2])-objects[left]['center'][:2])/span


def motion_metrics(rows, gt, gate=2., allowed_stamps=None):
    counts, values, events = Counter(), defaultdict(list), []
    series_by_key = {str(series[1][0]['key']): series for series in gt['gt'] if series[1]}
    for frame in rows:
        if allowed_stamps is not None and frame['stamp'] not in allowed_stamps:
            counts['gt_ineligible_frames_excluded'] += 1
            continue
        tracks = [dict(track, class_name='vehicle') for track in frame['tracks']
                  if track['confirmed'] and track['core_class'] in (1, 2)]
        truth = [point for series in gt['gt'] for point in [sample_series(series, frame['stamp'], .15)]
                 if point is not None and point['class_name'] == 'vehicle']
        if 'ego' in gt:
            ego = sample_series(gt['ego'], frame['stamp'], .15)
            if ego is None:
                counts['unbracketed_ego'] += 1
                continue
            truth = [point for point in truth if in_roi(point, ego)]
        counts['vehicle_outputs'] += len(tracks)
        counts['vehicle_gt'] += len(truth)
        paired = match(truth, tracks, gate)[0]
        counts['matched'] += len(paired)
        counts['unmatched_output'] += len(tracks)-len(paired)
        for g, t, distance in paired:
            target = velocity_at(series_by_key[str(truth[g]['key'])], frame['stamp'])
            if target is None:
                counts['unbracketed_motion'] += 1
                continue
            track = tracks[t]
            predicted = np.asarray(track['velocity'])
            speed, pred_speed = np.linalg.norm(target), np.linalg.norm(predicted)
            young = frame['stamp']-track['initialization_time'] <= 2.
            counts['motion_pairs'] += 1
            values['center_error_m'].append(distance)
            values['velocity_vector_error_mps'].append(float(np.linalg.norm(predicted-target)))
            values['speed_abs_error_mps'].append(float(abs(pred_speed-speed)))
            if speed >= 1. and pred_speed >= .5:
                opposite = float(np.dot(target, predicted)) < -.5*speed*pred_speed
                counts['direction_eligible'] += 1
                counts['opposite_direction'] += int(opposite)
                if opposite:
                    events.append(dict(stamp=frame['stamp'], id=track['id'], gt_key=truth[g]['key'],
                        center_error_m=distance, age_sec=frame['stamp']-track['initialization_time'],
                        world_velocity=predicted.tolist(), gt_velocity=target.tolist(),
                        score=track.get('score'), associated=track.get('associated'),
                        last_measurement_time=track.get('last_measurement_time')))
                if young:
                    counts['young_direction_eligible'] += 1
                    counts['young_opposite_direction'] += int(opposite)
            for horizon in (1., 3.):
                future = sample_series(series_by_key[str(truth[g]['key'])], frame['stamp']+horizon, .15)
                if future is not None:
                    endpoint = np.asarray(track['center'][:2])+horizon*predicted
                    values[f'cartesian_cv_{horizon:g}s_fde_m'].append(float(np.linalg.norm(endpoint-future['center'][:2])))
    return dict(counts=dict(counts), distributions={key: stats(value) for key, value in values.items()},
                opposite_direction_events=events,
                scope='Catalog-center GT correspondence and bracketed motion; Cartesian-CV diagnostic, not actual Map-CV or AP')


def audit(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    params = yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())['configure']
    params['pedestrian_gaussian_association_ranking'] = True
    baseline, receipt = build_original(args.output/'native_core', params,
                                      dict(gaussian_xy_ranking='pedestrian'), extended_state=True)
    guard = build_guard(args.output/'guard_core', params)
    with gzip.open(args.predictions, 'rt') as stream:
        data = json.load(stream)
    full_gt = None
    if args.gt_pack:
        with gzip.open(args.gt_pack, 'rt') as stream:
            full_gt = json.load(stream)
    cases = sorted({row['case'] for row in data['rows']})
    report = dict(cases={}, source_sha256=sha(args.predictions), native_build=receipt,
                  independent_scenes=True, production_changed=False,
                  scope='Same new continuous detections, actual native measurement-time core. No live scheduler or closed-loop certification')
    for name in cases:
        all_raw = [row for row in data['rows'] if row['case'] == name]
        # Recorder begins before ScenarioLoad: previous-scene prehistory must
        # never initialize the newly loaded scene's tracker or count as its GT.
        raw = [row for row in all_raw if row.get('scene_epoch_valid', True)]
        frames, gt = tracker_frames(raw)
        if full_gt is not None:
            # The full pack provides actors, not Ego interpolation. Keep the
            # audited header-pose Ego sequence exported with the original rows.
            gt = dict(gt, gt=full_gt['cases'][name]['gt'],
                      unresolved=full_gt['cases'][name].get('unresolved', []))
        eligible = {row['stamp'] for row in raw if row.get('scoring_eligible', row.get('actor_geometry_resolved', False))}
        if not eligible:
            raise ValueError('No qualified GT frames; preserve raw without claiming motion accuracy')
        original = replay(baseline, frames)
        off = replay(guard, frames, extra_args=('0', '1'))
        on = replay(guard, frames, extra_args=('1', '1'))
        if len(original) != len(off) or any(a['stamp'] != b['stamp'] or a['tracks'] != b['tracks'] for a, b in zip(original, off)):
            raise RuntimeError('Guard observer changes independent native baseline')
        changes = []
        for a, b in zip(off, on):
            if a['stamp'] != b['stamp'] or len(a['tracks']) != len(b['tracks']):
                raise RuntimeError('Output stamp or lifecycle changed')
            for u, v in zip(a['tracks'], b['tracks']):
                if {key: value for key, value in u.items() if key != 'velocity'} != {key: value for key, value in v.items() if key != 'velocity'}:
                    raise RuntimeError('Direction guard changes position/identity/shape state')
                if u['velocity'] != v['velocity']:
                    changes.append(dict(stamp=a['stamp'], id=u['id'], original=u['velocity'], candidate=v['velocity']))
        for suffix, payload in [('frames', frames), ('gt', gt), ('core', original),
                                ('off', off), ('on', on), ('changes', changes)]:
            save_gzip(args.output/(name+'_'+suffix+'.json.gz'), payload)
        births = vehicle_births(frames, original, params['max_association_dist_m'])
        save_gzip(args.output/(name+'_births.json.gz'), births)
        row = dict(frames=len(frames), independent_native_exact=True, direction_changed=len(changes),
                   previous_scene_rows_excluded=len(all_raw)-len(raw),
                   gt_eligible_frames=len(eligible), gt_ineligible_frames=len(raw)-len(eligible),
                   nearby_stale_birth_candidates=len(births),
                   detector=correspondence([r for r in raw if r['stamp'] in eligible]),
                   off=core_summary([r for r in off if r['stamp'] in eligible], gt),
                   on=core_summary([r for r in on if r['stamp'] in eligible], gt),
                   motion_2m={mode: motion_metrics(value, gt, 2., eligible) for mode, value in [('off', off), ('on', on)]},
                   motion_5m={mode: motion_metrics(value, gt, 5., eligible) for mode, value in [('off', off), ('on', on)]})
        report['cases'][name] = row
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(case=name, frames=len(frames), changes=len(changes), stale_births=len(births))), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gt-pack', type=Path, help='Full source actor histories, including outside the semantic ROI')
    parser.add_argument('--prepare-repeat-inputs', type=Path)
    parser.add_argument('--repeat-outputs', type=Path)
    args = parser.parse_args()
    if args.prepare_repeat_inputs:
        prepare_repeat_inputs(args.prepare_repeat_inputs, args.output)
    elif args.repeat_outputs and args.predictions:
        audit_repeat_outputs(args.predictions, args.repeat_outputs, args.output)
    elif args.predictions:
        audit(args)
    else:
        parser.error('Require --predictions or --prepare-repeat-inputs')
