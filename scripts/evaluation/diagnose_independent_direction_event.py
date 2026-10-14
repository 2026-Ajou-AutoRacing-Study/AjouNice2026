#!/usr/bin/env python3
"""Diagnose independent direction events using actual associated core history.

Adds read-only stdout to an isolated copy of an already qualified harness.
Never infers association by nearest detections, changes a filter, or launches
ROS/GPU. GT evaluates outputs only and never enters the tracker or guard.
"""
import argparse
from collections import defaultdict
import copy
import gzip
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from audit_independent_vehicle_direction import velocity_at
from audit_tracking_continuity import sample_series, stats
from audit_tracking_continuity_events import replay, sha


def load(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def encoded_frames(frames):
    lines = []
    for frame in frames:
        objects = frame['world_objects']
        lines.append(f"{frame['stamp']:.17g} {len(objects)}")
        for obj in objects:
            label = ('vehicle', 'pedestrian', 'obstacle').index(obj['class_name'])
            lines.append(' '.join(format(v, '.17g') for v in
                (label, *obj['center'], obj['yaw'], *obj['size'], obj['score'])))
    return '\n'.join(lines)+'\n'


def build_history_harness(guard_folder, output):
    output.mkdir(parents=True, exist_ok=False)
    receipt = json.loads((guard_folder/'build.json').read_text())
    inputs = receipt['input_sha256']
    if any(sha(Path(path)) != value for path, value in inputs.items()):
        raise ValueError('Qualified core/header/guard source changed')
    original = (guard_folder/'harness.cpp').read_text()
    anchor = '<< policy.fit_valid << " " << policy.trusted_stamp << " " << policy.fit_residual_m << "\\n";'
    if original.count(anchor) != 1:
        raise ValueError('Pinned GUARD stdout anchor changed')
    logging = '''
            int observed_index = 0;
            for (const auto& p : t.center_history) {
                std::cout << "HISTORY " << time << " " << t.track_id << " " << observed_index++ << " "
                          << p.time_stamp << " " << p.x << " " << p.y << "\\n";
            }
'''
    harness = output/'actual_history_harness.cpp'
    harness.write_text(original.replace(anchor, anchor+'\n'+logging))
    binary = output/'actual_history_probe'
    old_command = receipt['command']
    original_harness = old_command[old_command.index('-o')-2]
    original_binary = old_command[old_command.index('-o')+1]
    command = [str(harness) if arg == original_harness else str(binary) if arg == original_binary else arg
               for arg in old_command]
    subprocess.run(command, check=True)
    if any(sha(Path(path)) != value for path, value in inputs.items()):
        raise ValueError('Source changed during read-only diagnostic build')
    (output/'build.json').write_text(json.dumps(dict(input_sha256=inputs,
        source_harness_sha256=sha(guard_folder/'harness.cpp'), harness_sha256=sha(harness),
        binary_sha256=sha(binary), command=command, production_modified=False), indent=2)+'\n')
    return binary


def parse_observers(stdout):
    guards, histories = {}, defaultdict(list)
    for line in stdout.splitlines():
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == 'GUARD':
            key = (float(tokens[1]), int(tokens[2]))
            guards[key] = dict(guarded=bool(int(tokens[3])), fit_valid=bool(int(tokens[4])),
                              trusted_stamp=float(tokens[5]), fit_residual_m=float(tokens[6]))
        elif tokens[0] == 'HISTORY':
            key = (float(tokens[1]), int(tokens[2]))
            histories[key].append(dict(index=int(tokens[3]), stamp=float(tokens[4]),
                                       center=[float(tokens[5]), float(tokens[6])]))
    return guards, dict(histories)


def fit_diagnostic(history, window=.45):
    if not history:
        return dict(count=0, span_sec=0., sufficient_count=False, sufficient_span=False)
    stamp = history[-1]['stamp']
    selected = [row for row in history if stamp-row['stamp'] <= window+1.e-6]
    times = np.asarray([row['stamp']-stamp for row in selected])
    centers = np.asarray([row['center'] for row in selected])
    dt = times-times.mean()
    span = float(times[-1]-times[0])
    result = dict(count=len(selected), span_sec=span,
                  sufficient_count=len(selected) >= 4, sufficient_span=span+1.e-6 >= .30)
    if float(dt@dt) > 1.e-9:
        velocity = (dt[:, None]*(centers-centers.mean(axis=0))).sum(axis=0)/(dt@dt)
        residual = centers-centers.mean(axis=0)-dt[:, None]*velocity
        result.update(velocity=velocity.tolist(), speed_mps=float(np.linalg.norm(velocity)),
            rms_residual_m=float(np.sqrt(np.square(residual).sum()/len(selected))))
    return result


def associated_measurement(history, frames_by_stamp):
    """Exact actual-history identity lookup, never a distance-based guess."""
    if not history:
        return None
    current = history[-1]
    frame = frames_by_stamp.get(current['stamp'])
    if frame is None:
        raise ValueError('Associated native history stamp absent from input')
    exact = [(i, obj) for i, obj in enumerate(frame['world_objects'])
             if obj['class_name'] == 'vehicle' and obj['center'][:2] == current['center']]
    if len(exact) != 1:
        raise ValueError('Actual associated center must identify exactly one input measurement')
    index, obj = exact[0]
    return dict(input_index=index, **obj)


def event_ablations(binary, frames, event, current, previous):
    """Evidence-only interventions, not proposed runtime changes; no GT input."""
    index = next(i for i, row in enumerate(frames) if row['stamp'] == event['stamp'])
    input_index = current['actual_associated_input']['input_index']
    results = {}
    for variant in ('yaw_equivalent_plus_pi', 'reuse_previous_associated_yaw', 'replace_center_with_native_prior'):
        candidate = copy.deepcopy(frames)
        obj = candidate[index]['world_objects'][input_index]
        if variant == 'yaw_equivalent_plus_pi':
            obj['yaw'] += math.pi
        elif variant == 'reuse_previous_associated_yaw':
            obj['yaw'] = previous['actual_associated_input']['yaw']
        else:
            obj['center'][:2] = current['prior']['center']
        rows = replay(binary, candidate, extra_args=('0', '1'))
        track = next(t for t in rows[index]['tracks'] if t['id'] == int(event['id']))
        results[variant] = dict(track=track, changed_single_input_dimension=True, production_changed=False)
    return results


def analyze(args):
    args.output.mkdir(parents=True, exist_ok=False)
    report = json.loads((args.audit/'report.json').read_text())
    binary = build_history_harness(args.audit/'guard_core', args.output/'probe')
    result = dict(source_audit_sha256=sha(args.audit/'report.json'), production_changed=False,
        scope='Actual associated measurement history and isolated native core; no live ROS/planner or AP claim.',
        observer_parity=[], events=[])
    input_payload = None if args.inputs is None else json.loads(args.inputs.read_text())
    packet_payload = None if args.packet_audit is None else json.loads(args.packet_audit.read_text())
    result['supplemental_source_sha256'] = {str(p.resolve()):sha(p)
        for p in (args.inputs, args.packet_audit) if p is not None}
    for name, case in report['cases'].items():
        events = case['motion_2m']['off'].get('opposite_direction_events', [])
        if not events:
            continue
        frames = load(args.audit/(name+'_frames.json.gz'))
        baseline = load(args.audit/(name+'_off.json.gz'))
        native = load(args.audit/(name+'_core.json.gz'))
        gt = load(args.audit/(name+'_gt.json.gz'))
        probe = replay(binary, frames, extra_args=('0', '1'))
        if probe != baseline:
            raise ValueError('Read-only history stdout altered native output')
        result['observer_parity'].append(dict(case=name, frames=len(frames), exact=True))
        run = subprocess.run([str(binary.resolve()), '0', '1'], input=encoded_frames(frames),
            text=True, capture_output=True, check=True, timeout=240)
        stdout_path = args.output/(name+'_actual_history.log.gz')
        with gzip.open(stdout_path, 'wt') as stream:
            stream.write(run.stdout)
        guards, histories = parse_observers(run.stdout)
        by_frame = {f['stamp']:f for f in frames}
        truth = {str(s[1][0]['key']):s for s in gt['gt'] if s[1]}
        for event in events:
            identity, stamp = int(event['id']), event['stamp']
            trace = []
            for off, before in zip(baseline, native):
                track = next((t for t in off['tracks'] if t['id'] == identity), None)
                if track is None or not (event['stamp']-1. <= off['stamp'] <= event['stamp']+1.):
                    continue
                key = off['stamp'], identity
                history = histories[key]
                measurement = associated_measurement(history, by_frame)
                prior = next((t for t in before['before'] if t['id'] == identity), None)
                current_gt = sample_series(truth[str(event['gt_key'])], off['stamp'], .15)
                gt_velocity = velocity_at(truth[str(event['gt_key'])], off['stamp'])
                raw_error = None if current_gt is None else math.dist(measurement['center'][:2], current_gt['center'][:2])
                track_error = None if current_gt is None else math.dist(track['center'][:2], current_gt['center'][:2])
                fresh_observation = history[-1]['stamp'] == off['stamp']
                innovation = None if prior is None or not fresh_observation else (np.asarray(measurement['center'][:2])-np.asarray(prior['center']))
                raw_maha = None
                if innovation is not None:
                    xx, xy, yy = prior['covariance']
                    raw_maha = float(np.sqrt(innovation@np.linalg.solve(np.asarray([[xx,xy],[xy,yy]]), innovation)))
                trace.append(dict(stamp=off['stamp'], relative_event_sec=off['stamp']-stamp,
                    track=track, actual_associated_history=history, actual_guard=guards[key], fit=fit_diagnostic(history),
                    actual_associated_input=measurement, raw_center_gt_error_m=raw_error,
                    track_center_gt_error_m=track_error,
                    fresh_actual_associated_observation=fresh_observation,
                    gt_velocity=None if gt_velocity is None else gt_velocity.tolist(), prior=prior,
                    input_innovation=None if innovation is None else innovation.tolist(),
                    legacy_p_only_mahalanobis=raw_maha))
            current = next(row for row in trace if row['stamp'] == stamp)
            previous = next(row for row in trace if row['stamp'] == current['actual_associated_history'][-2]['stamp'])
            delta_time = current['stamp']-previous['stamp']
            delta = np.asarray(current['actual_associated_input']['center'][:2])-previous['actual_associated_input']['center'][:2]
            yaw_delta = current['actual_associated_input']['yaw']-previous['actual_associated_input']['yaw']
            future_error = {}
            event_gt = sample_series(truth[str(event['gt_key'])], stamp, .15)
            event_gt_velocity = velocity_at(truth[str(event['gt_key'])], stamp)
            for horizon in (1., 3.):
                future = sample_series(truth[str(event['gt_key'])], stamp+horizon, .15)
                if future is not None:
                    endpoint = np.asarray(current['track']['center'][:2])+horizon*np.asarray(current['track']['velocity'])
                    future_error[f'{horizon:g}s_cartesian_cv_fde_m'] = float(np.linalg.norm(endpoint-future['center'][:2]))
                    if event_gt is not None and event_gt_velocity is not None:
                        oracle = np.asarray(event_gt['center'][:2])+horizon*event_gt_velocity
                        future_error[f'{horizon:g}s_perfect_current_gt_cv_fde_m'] = float(np.linalg.norm(oracle-future['center'][:2]))
            timeline = [row for row in trace if row['stamp']-current['track']['initialization_time'] <= 2.]
            sensor_context = None
            if input_payload is not None:
                input_row = next(row for row in input_payload['rows'] if row['case'] == name and row['stamp'] == stamp)
                gt_index = input_row['parent_uids'].index(str(event['gt_key']))
                current_gt = sample_series(truth[str(event['gt_key'])], stamp, .15)
                sensor_context = dict(token=input_row['token'], raw_lidar_index=input_row['raw_lidar_index'],
                    lidar_points_in_catalog_box=input_row['gt_points'][gt_index], asset=input_row['gt_assets'][gt_index],
                    actor_lineage=input_row['gt_actor_metadata'][gt_index],
                    camera_offsets_sec=input_row['camera_offsets_sec'], ego=input_row['ego'],
                    gt_range_xy_m=math.hypot(current_gt['center'][0]-input_row['ego']['x'],current_gt['center'][1]-input_row['ego']['y']))
            if packet_payload is not None:
                packet_case = next(case for case in packet_payload['cases'] if case['scene']+'_'+case['native_policy'] == name)
                packet_rows = load(args.packet_audit.parent/packet_case['row_artifact'])['recorded']
                packet_row = next(row for row in packet_rows if row['header'] == stamp)
                sensor_context = dict(sensor_context or {}, recorded_packet_context=packet_row)
            result['events'].append(dict(case=name, event=event, current=current, trace=trace,
                sensor_context=sensor_context,
                actual_associated_center_step=dict(dt_sec=delta_time, delta_xy_m=delta.tolist(), norm_m=float(np.linalg.norm(delta)),
                    implied_xy_velocity_mps=(delta/delta_time).tolist(), yaw_signed_deg=math.degrees((yaw_delta+math.pi)%(2*math.pi)-math.pi),
                    yaw_box_axis_deg=abs(math.degrees((yaw_delta+math.pi/2)%math.pi-math.pi/2))),
                event_cartesian_cv=future_error,
                evidence_only_ablations=event_ablations(binary, frames, event, current, previous),
                actual_history_event_count=current['fit']['count'], actual_history_event_span_sec=current['fit']['span_sec'],
                no_trusted_direction=current['actual_guard']['trusted_stamp'] == 0.,
                causal_guard_blockers=[name for name, passed in (
                    ('minimum_four_actual_associated_observations',current['fit']['sufficient_count']),
                    ('minimum_0p30sec_actual_associated_span',current['fit']['sufficient_span'])) if not passed],
                near_event_distribution=dict(
                    raw_center_gt_error_m=stats([r['raw_center_gt_error_m'] for r in timeline if r['raw_center_gt_error_m'] is not None]),
                    track_center_gt_error_m=stats([r['track_center_gt_error_m'] for r in timeline if r['track_center_gt_error_m'] is not None]),
                    velocity_vector_gt_error_mps=stats([math.dist(r['track']['velocity'],r['gt_velocity'])
                        for r in timeline if r['gt_velocity'] is not None]),
                    associated_score=stats([r['actual_associated_input']['score'] for r in timeline]),
                    filtered_score=stats([r['track']['score'] for r in timeline]),
                    fit_residual_m=stats([r['fit']['rms_residual_m'] for r in timeline if 'rms_residual_m' in r['fit']]))))
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(events=len(result['events']), exact=result['observer_parity'],
        blockers=[e['causal_guard_blockers'] for e in result['events']])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, help='Optional frozen sensor manifest for exact event token/range/support evidence')
    parser.add_argument('--packet-audit', type=Path, help='Optional native packet audit for exact event phase/footer evidence')
    analyze(parser.parse_args())
