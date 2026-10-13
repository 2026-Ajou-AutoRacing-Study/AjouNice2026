#!/usr/bin/env python3
"""Frozen detections through isolated real-EKF probes, with GT-only diagnostics.

No GT enters association. All 1570 inputs preserve tracking history; scoring
uses only resolved full-ROI frames. No ROS node, scheduler, planner or GPU.
"""
import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle
import subprocess

import numpy as np

from analyze_base5_far_vehicle_scores import prepare_cases, json_safe, pooled_ranges
from analyze_base5_runtime_effects import distribution, drift
from evaluate_base5_far_retention import full_prepare, range_operating, persistent_unmatched, band_name
from replay_base5_compiled_tracking import ego_trajectory, world_detections, load_sensor4_calibration
from summarize_base5_rosbag_quick import bridge_nms

POLICIES = {
    'baseline': dict(floor=.4, secondary=False, gate=2., window=.5, position_only=True),
    'baseline_repeat': dict(floor=.4, secondary=False, gate=2., window=.5, position_only=True),
    'weak030_xy': dict(floor=.3, secondary=True, gate=2., window=.5, position_only=True),
    'weak020_xy': dict(floor=.2, secondary=True, gate=2., window=.5, position_only=True),
    'weak010_xy': dict(floor=.1, secondary=True, gate=2., window=.5, position_only=True),
    'weak010_class080_xy': dict(floor=.1, secondary=True, gate=2., window=.5, position_only=True, class_min=.8),
    'weak010_class090_xy': dict(floor=.1, secondary=True, gate=2., window=.5, position_only=True, class_min=.9),
    'weak010_moving_class090_xy': dict(floor=.1, secondary=True, gate=2., window=.5, position_only=True, class_min=.9, moving_min=2.),
    'weak020_class080_xy': dict(floor=.2, secondary=True, gate=2., window=.5, position_only=True, class_min=.8),
    'weak020_full': dict(floor=.2, secondary=True, gate=2., window=.5, position_only=False),
    'weak020_window1': dict(floor=.2, secondary=True, gate=2., window=1., position_only=True),
    'naive020_birth': dict(floor=.2, secondary=False, gate=2., window=.5, position_only=False),
}


def pose_at(ego, t):
    i = int(np.searchsorted(ego[:, 0], t))
    if i < len(ego) and abs(ego[i, 0] - t) < 1e-9:
        return ego[i, 1:]
    if i == 0 or i == len(ego) or ego[i, 0] - ego[i - 1, 0] > .15:
        raise ValueError('Recorded Ego pose lacks bounded interpolation')
    return ego[i - 1, 1:] + (ego[i, 1:] - ego[i - 1, 1:]) * (t - ego[i - 1, 0]) / (ego[i, 0] - ego[i - 1, 0])


def replay(binary, rows, sources, ego, extrinsic, policy):
    lines = []
    for row in rows:
        t = sources[row['token']]['timestamp']
        selected = []
        for box, label, score in zip(row['pred_boxes'], row['pred_labels'], row['pred_scores']):
            floor = policy['floor'] if label == 0 else .1
            if score < floor:
                continue
            if policy['secondary'] and label == 0 and score < .4 and np.linalg.norm(box[:2]) < 30:
                continue
            selected.append(dict(label=label, center=box[:3], size=box[3:6], yaw=box[6], score=score))
        measurements = world_detections(selected, t, ego, extrinsic)
        lines.append(f'{t:.17g} {len(measurements)}')
        lines.extend(' '.join(f'{v:.17g}' for v in (d['label'], *d['center'], d['yaw'], *d['size'], d['score']))
                     for d in measurements)
    args = [str(binary.resolve()), str(policy['floor'] if policy['secondary'] else .4),
            str(policy['gate']), str(policy['window']), str(int(policy['position_only'])),
            str(policy.get('class_min', 0.)), str(policy.get('moving_min', 0.))]
    run = subprocess.run(args, input='\n'.join(lines) + '\n', capture_output=True, text=True, check=True, timeout=180)
    frames, weak = [], []
    for line in run.stdout.splitlines():
        v = line.split()
        if not v:
            continue
        if v[0] == 'WEAK':
            weak.append(dict(stamp=float(v[1]), id=int(v[2]), measurement_index=int(v[3]),
                             class_probability=float(v[4]) if len(v) > 4 else None))
        elif v[0] == 'FRAME':
            frames.append(dict(stamp=float(v[1]), expected_tracks=int(v[2]), tracks=[]))
        elif v[0] == 'TRACK':
            frames[-1]['tracks'].append(dict(id=int(v[1]), core_class=int(v[2]), confirmed=bool(int(v[3])),
                associated=bool(int(v[4])), center=[float(x) for x in v[5:8]],
                velocity=[float(x) for x in v[8:10]], yaw=float(v[10]), size=[float(x) for x in v[11:14]],
                score=float(v[14]), last_measurement_time=float(v[15]), age=int(v[16])))
    if len(frames) != len(rows) or any(f['expected_tracks'] != len(f['tracks']) for f in frames):
        raise ValueError('Incomplete probe frame protocol')
    if [f['stamp'] for f in frames] != [sources[r['token']]['timestamp'] for r in rows]:
        raise ValueError('Probe sensor stamps changed')
    return dict(frames=frames, weak_updates=weak)


def local_box(track, pose, extrinsic):
    c, s = math.cos(pose[3]), math.sin(pose[3])
    dx, dy = np.asarray(track['center'][:2]) - pose[:2]
    base = np.array([c * dx + s * dy, -s * dx + c * dy, track['center'][2] - pose[2], 1.])
    xyz = np.linalg.inv(extrinsic) @ base
    sensor_yaw = math.atan2(extrinsic[1, 0], extrinsic[0, 0])
    return [*xyz[:3], *track['size'], track['yaw'] - pose[3] - sensor_yaw]


def trajectory_velocities(rows, ego, extrinsic):
    segments, current, previous = [], defaultdict(list), None
    for row in rows:
        keys = set(row['gt_track_keys'])
        gap = previous is not None and (row['native_index'] != previous[0] + 1 or row['stamp'] - previous[1] > .25)
        for key in list(current):
            if gap or key not in keys:
                segments.append(current.pop(key))
        for g, key in enumerate(row['gt_track_keys']):
            pose = pose_at(ego, row['stamp'])
            world = world_detections([dict(center=row['gt_boxes'][g, :3], size=row['gt_boxes'][g, 3:6],
                                          yaw=row['gt_boxes'][g, 6])], row['stamp'], ego, extrinsic)[0]
            current[key].append((row['token'], g, row['stamp'], np.asarray(world['center'][:2])))
        previous = (row['native_index'], row['stamp'])
    segments.extend(current.values())
    result = {}
    for segment in segments:
        for token, g, t, xy in segment:
            neighbours = [r for r in segment if abs(r[2] - t) <= .200001]
            if len(neighbours) < 3 or neighbours[-1][2] - neighbours[0][2] < .15:
                continue
            dt = np.asarray([r[2] - t for r in neighbours])
            positions = np.asarray([r[3] for r in neighbours])
            fit = np.linalg.lstsq(np.column_stack((dt, np.ones(len(dt)))), positions, rcond=None)[0]
            result[(token, g)] = fit[0]
    return result


def confidence_diagnostics(rows):
    segments, active = [], {}
    previous = None
    by_band = defaultdict(Counter)
    for row in rows:
        keys = {key for g, key in enumerate(row['gt_track_keys']) if row['gt_labels'][g] == 0}
        gap = previous is not None and (row['native_index'] != previous[0] + 1 or row['stamp'] - previous[1] > .25)
        for key in list(active):
            if gap or key not in keys:
                segments.append(active.pop(key))
        paired = {g: p for p, g in row['assignments'][0][2.].items()}
        for g, key in enumerate(row['gt_track_keys']):
            if row['gt_labels'][g] != 0:
                continue
            band = band_name(float(np.linalg.norm(row['gt_boxes'][g, :2])))
            score = float(row['pred_scores'][paired[g]]) if g in paired else None
            record = active.setdefault(key, dict(key=key, observations=[], first_high=None))
            if score is not None and score >= .4:
                kind = 'vehicle_match_ge040'
                if record['first_high'] is None:
                    record['first_high'] = dict(stamp=row['stamp'], distance=float(np.linalg.norm(row['gt_boxes'][g, :2])))
            elif score is not None:
                kind = 'weak_before_first_high' if record['first_high'] is None else 'weak_after_first_high'
            else:
                distance = np.linalg.norm(row['pred_boxes'][:, :2] - row['gt_boxes'][g, :2], axis=1)
                if np.any((distance < 2.) & (row['pred_labels'] == 0)):
                    kind = 'near_vehicle_candidate_not_globally_matched'
                elif np.any((distance < 2.) & (row['pred_labels'] != 0)):
                    kind = 'nonvehicle_candidate_within2m'
                elif np.any((distance < 4.) & (row['pred_labels'] == 0)):
                    kind = 'vehicle_candidate_2_to_4m_position_error'
                else:
                    kind = 'no_candidate_within2m_at_floor010'
            by_band[band][kind] += 1
            record['observations'].append(dict(token=row['token'], stamp=row['stamp'], band=band,
                                               distance=float(np.linalg.norm(row['gt_boxes'][g, :2])),
                                               lidar_x=float(row['gt_boxes'][g, 0]), lidar_y=float(row['gt_boxes'][g, 1]),
                                               score=score, kind=kind))
        previous = (row['native_index'], row['stamp'])
    segments.extend(active.values())
    for segment in segments:
        obs = segment['observations']
        segment['first_high_left_censored'] = bool(segment['first_high'] and segment['first_high']['stamp'] == obs[0]['stamp'])
        segment['distance_change_m'] = obs[-1]['distance'] - obs[0]['distance']
    return dict(by_band=dict(by_band), segments=segments,
                segment_count=len(segments), never_crossed040_segments=sum(s['first_high'] is None for s in segments),
                note='GT-guided diagnosis only; segments break at scoring/ROI/index/time gaps. No GT enters tracker.')


def score_case(rows, frames, ego, extrinsic, velocities, period):
    frame_by_stamp = {f['stamp']: f for f in frames}
    replaced, tracks_by_token = [], {}
    for row in rows:
        tracks = [t for t in frame_by_stamp[row['stamp']]['tracks'] if t['confirmed']]
        tracks_by_token[row['token']] = tracks
        labels = [0 if t['core_class'] in (1, 2) else 1 if t['core_class'] in (3, 4) else 2 for t in tracks]
        replaced.append(dict(row, pred_boxes=[local_box(t, pose_at(ego, row['stamp']), extrinsic) for t in tracks],
                             pred_labels=labels, pred_scores=[t['score'] for t in tracks],
                             pred_track_ids=[t['id'] for t in tracks]))
    prepared = full_prepare(replaced)
    result, observations = {}, {}
    for label, name in enumerate(('vehicle', 'pedestrian', 'obstacle')):
        positions, speeds, corners, updates, ages = [], [], [], [], []
        ids, switches = {}, 0
        for row in prepared:
            tracks = {t['id']: t for t in tracks_by_token[row['token']]}
            for p, g in row['assignments'][label][4.].items():
                track = tracks[int(row['pred_track_ids'][p])]
                error = float(np.linalg.norm(row['pred_boxes'][p, :2] - row['gt_boxes'][g, :2]))
                positions.append(error)
                old = dict(center=row['gt_boxes'][g, :3], size=row['gt_boxes'][g, 3:6], yaw=row['gt_boxes'][g, 6], score=0.)
                new = dict(center=row['pred_boxes'][p, :3], size=row['pred_boxes'][p, 3:6], yaw=row['pred_boxes'][p, 6], score=0.)
                corner = drift(old, new)['footprint_corner_m']; corners.append(corner)
                ref = velocities.get((row['token'], g)); speed_error = None
                if ref is not None:
                    velocity = track['velocity'] if label != 2 else [0., 0.]
                    speed_error = float(np.linalg.norm(np.asarray(velocity) - ref)); speeds.append(speed_error)
                updates.append(track['associated']); ages.append(row['stamp'] - track['last_measurement_time'])
                observations[row['token'] + '/' + row['gt_track_keys'][g]] = dict(label=name, position=error,
                    velocity=speed_error, corner=corner, id=track['id'], associated=track['associated'],
                    band=band_name(float(np.linalg.norm(row['gt_boxes'][g, :2]))), gt_key=row['gt_track_keys'][g])
            for p, g in row['assignments'][label][2.].items():
                key = row['gt_track_keys'][g]; current = (row['native_index'], row['stamp'], int(row['pred_track_ids'][p]))
                if key in ids:
                    old = ids[key]
                    if current[0] == old[0] + 1 and current[1] - old[1] <= .25 and current[2] != old[2]:
                        switches += 1
                ids[key] = current
        result[name] = dict(by_range=range_operating(prepared, label, 0., period),
            position_gate4m=distribution(positions), velocity_reference_gate4m=distribution(speeds),
            footprint_corner_gate4m=distribution(corners), observation_age_gate4m=distribution(ages),
            matched_associated_frames=sum(updates), matched_coasting_frames=len(updates) - sum(updates),
            adjacent_gt_key_id_changes=switches, unmatched=persistent_unmatched(prepared, label, period))
    return result, observations


def paired_quality(reference, candidate):
    result = {}
    for label in ('vehicle', 'pedestrian', 'obstacle'):
        keys = sorted(k for k in reference.keys() & candidate.keys() if reference[k]['label'] == candidate[k]['label'] == label)
        result[label] = {metric: dict(baseline=distribution([reference[k][metric] for k in keys if reference[k][metric] is not None and candidate[k][metric] is not None]),
            candidate=distribution([candidate[k][metric] for k in keys if reference[k][metric] is not None and candidate[k][metric] is not None]))
            for metric in ('position', 'velocity', 'corner')}
        result[label]['by_range'] = {}
        for band in ('0_30m', '30_50m', '50_80m', '80_100m'):
            selected = [k for k in keys if reference[k]['band'] == band]
            result[label]['by_range'][band] = {metric: dict(
                baseline=distribution([reference[k][metric] for k in selected if reference[k][metric] is not None and candidate[k][metric] is not None]),
                candidate=distribution([candidate[k][metric] for k in selected if reference[k][metric] is not None and candidate[k][metric] is not None]))
                for metric in ('position', 'velocity', 'corner')}
        result[label]['per_gt_far'] = {}
        for gt_key in sorted({reference[k]['gt_key'] for k in keys if reference[k]['band'] in ('50_80m', '80_100m')}):
            selected = [k for k in keys if reference[k]['gt_key'] == gt_key and reference[k]['band'] in ('50_80m', '80_100m')]
            result[label]['per_gt_far'][gt_key] = {metric: dict(
                baseline=distribution([reference[k][metric] for k in selected if reference[k][metric] is not None and candidate[k][metric] is not None]),
                candidate=distribution([candidate[k][metric] for k in selected if reference[k][metric] is not None and candidate[k][metric] is not None]))
                for metric in ('position', 'velocity', 'corner')}
    return result


def stability_metrics(frames, sources):
    stamps = {r['timestamp']: r for r in sources.values()}
    previous = {}
    values = defaultdict(lambda: dict(velocity_step=[], cv_position_residual=[]))
    for frame in frames:
        source = stamps[frame['stamp']]
        active = {}
        for t in frame['tracks']:
            if not t['confirmed'] or t['core_class'] not in (1, 2) or not source['scoring_eligible']:
                continue
            active[t['id']] = (source, t)
            if t['id'] not in previous:
                continue
            old_source, old = previous[t['id']]
            dt = source['timestamp'] - old_source['timestamp']
            if source['raw_lidar_index'] != old_source['raw_lidar_index'] + 1 or not 0 < dt <= .25:
                continue
            key = 'associated' if t['associated'] else 'coasting'
            values[key]['velocity_step'].append(float(np.linalg.norm(np.asarray(t['velocity']) - old['velocity'])))
            residual = np.asarray(t['center'][:2]) - old['center'][:2] - np.asarray(old['velocity']) * dt
            values[key]['cv_position_residual'].append(float(np.linalg.norm(residual)))
        previous = active
    return {k: {metric: distribution(v) for metric, v in d.items()} for k, d in values.items()}


def analyze(args):
    if args.output.exists():
        raise ValueError('Fresh result directory required')
    paths = [args.inputs / 'infos_val.pkl', args.inputs / 'manifest.json', args.predictions,
             args.build / 'build.json', args.build / 'original', args.build / 'probe', args.sensor_config]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    records = pickle.loads(paths[0].read_bytes())['data_list']; sources = {r['token']: r for r in records}
    manifest = json.loads(paths[1].read_text())
    with gzip.open(args.predictions, 'rt') as f:
        payload = json.load(f)
    prepared, excluded = prepare_cases(records, payload)
    extrinsic = load_sensor4_calibration(args.sensor_config)['base_from_lidar']
    if manifest['sensor_sha256'] != hashlib.sha256(args.sensor_config.read_bytes()).hexdigest():
        raise ValueError('Frozen sensor calibration differs')
    args.output.mkdir(parents=True)
    report = dict(scope='Offline real EKF core probe, not ROS/node/TF/scheduler/planner safety certification',
        source_sha256=hashes, analysis_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        checkpoint_sha256=payload['checkpoint_sha256'], input_frames=len(records),
        scoring_frames=sum(map(len, prepared.values())), excluded_by_case=excluded, policies=POLICIES,
        notes=['No GT enters association or track birth.', 'Low candidates are Vehicle only, radial>=30m; high association first.',
               'Weak stage only confirmed unclaimed CAR/TRUCK, XY<=2m, pi-symmetric yaw<=60deg, length/width ratio<=2.',
               'Weak updates bounded by time since last high Vehicle measurement; lifecycle still counts last measurement.',
               'Position-only mode preserves size/Z/class and makes yaw measurement uninformative, not frozen EKF yaw.',
               'Class-margin and moving-only variants gate weak updates only; stopped vehicles retain the baseline policy.',
               'Speed reference is corrected BBox-center local fit +-0.2s, >=3 samples/span>=0.15s; not instantaneous native velocity GT.',
               'Full tracking history uses all input frames; scoring excludes unresolved GT frames.',
               'Input/output Z is bottom reference in this core probe; XY/BEV footprint only, not standardized ROS geometry certification.',
               'UNKNOWN output velocity is set to zero for quality metrics, simulating policy only; raw EKF is saved.',
               'Matched metrics exclude misses; paired common GT observations reduce selection bias.',
               'Greedy Mahalanobis core matching is unchanged; nearest GT ID changes are diagnostics, not authoritative IDSW.'],
        cases={}, results={}, baseline_repeat_exact=True, disabled_probe_exact=True)
    all_frames = {}
    for case, rows in prepared.items():
        ego = ego_trajectory(Path(manifest['cases'][case]['raw_bag']))
        velocities = trajectory_velocities(rows, ego, extrinsic)
        periods = np.diff([r['stamp'] for r in rows]); period = float(np.median(periods[(periods > 0) & (periods <= .25)]))
        report['cases'][case] = dict(frames=len(rows), confidence=confidence_diagnostics(rows))
        raw = sorted([bridge_nms(r) for r in payload['rows'] if sources[r['token']]['scene_id'] == case],
                     key=lambda r: sources[r['token']]['timestamp'])
        reference_obs = None; baseline = None
        for policy, options in POLICIES.items():
            binary = args.build / ('probe' if options['secondary'] else 'original')
            result = replay(binary, raw, sources, ego, extrinsic, options)
            all_frames[case + '/' + policy] = result
            if policy == 'baseline':
                baseline = result
                disabled = replay(args.build / 'probe', raw, sources, ego, extrinsic, options)
                if disabled != baseline:
                    raise ValueError('Disabled experimental source differs from original core')
            if policy == 'baseline_repeat' and result != baseline:
                raise ValueError('Original-source repeat differs')
            metrics, observations = score_case(rows, result['frames'], ego, extrinsic, velocities, period)
            if reference_obs is None:
                reference_obs = observations
            report['results'].setdefault(policy, {})[case] = dict(classes=metrics,
                weak_updates=len(result['weak_updates']), common_matched_quality=paired_quality(reference_obs, observations),
                stability=stability_metrics(result['frames'], {row['token']: sources[row['token']] for row in raw}))
            print(case, policy, 'weak_updates', len(result['weak_updates']), flush=True)
    for policy, cases in report['results'].items():
        report.setdefault('pooled', {})[policy] = {name: pooled_ranges(
            {case: result['classes'][name] for case, result in cases.items()}) for name in ('vehicle', 'pedestrian', 'obstacle')}
    if hashes != {str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}:
        raise ValueError('Frozen source changed')
    with gzip.open(args.output / 'frames.json.gz', 'wt') as f:
        json.dump(json_safe(all_frames), f, allow_nan=False)
    (args.output / 'report.json').write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n')
    print('Saved', args.output, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs', 'predictions', 'build', 'output', 'sensor-config'):
        p.add_argument('--' + name, type=Path, required=True)
    analyze(p.parse_args())
