#!/usr/bin/env python3
"""Frozen E12 vehicle forecasts: Cartesian CV, production CTRV and actual Map-CV.

Offline diagnostics only. Future GT defines evaluation targets/axes, never runtime
forecasts or association. GT-longitudinal-speed replacement is a named diagnostic
oracle on a separately reported paired subset, not a deployable predictor.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from copy import deepcopy
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from analyze_e12_map_horizon import (ROOT, CORE_PATH, NODE_PATH, E12_SHA, CANONICAL,
    current_assignment, digest, future_reference, key_tuple, load, load_predictors,
    path_errors, prepare_case)
from audit_vehicle_map_prediction import distribution

A = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'
ALLOWED_OUTPUT = A/'motion_ablation_v1'
BASE_METHODS = ('cv', 'ctrv', 'mapcv3_top1')
DIAGNOSTICS = ('cv_gt_longitudinal', 'ctrv_gt_longitudinal', 'mapcv3_fixed_route_gt_longitudinal')
TIMES = np.arange(1, 31, dtype=float)/10.


def causal_velocity(series, stamp, minimum_window=.1, maximum_window=.25, max_gap=.15,
                    with_span=False):
    """Backward secant of eligible adjacent original GT samples, never t+future."""
    times, points, indices = series
    end = bisect_left(times, stamp)
    if end == len(times) or abs(times[end]-stamp) > 1.e-6:
        return (None, 'no_exact_current_gt', None) if with_span else (None, 'no_exact_current_gt')
    start = end-1
    while start >= 0 and stamp-times[start] < minimum_window-1.e-6:
        start -= 1
    if start < 0 or stamp-times[start] > maximum_window+1.e-6:
        return (None, 'insufficient_bounded_past_gt', None) if with_span else (None, 'insufficient_bounded_past_gt')
    if any(indices[i+1] != indices[i]+1 or times[i+1]-times[i] > max_gap
           for i in range(start, end)):
        return (None, 'nonadjacent_or_gapped_past_gt', None) if with_span else (None, 'nonadjacent_or_gapped_past_gt')
    velocity = (np.asarray(points[end][:2])-np.asarray(points[start][:2]))/(stamp-times[start])
    span = dict(stamps=times[start:end+1], native_indices=indices[start:end+1])
    return (velocity, None, span) if with_span else (velocity, None)


def evaluation_axes(current_gt, reference, minimum_speed=.5):
    """GT-trajectory tangent at each future sample, solely an evaluation frame.

    Centered .2s finite differences except the final .1s backward difference.
    Samples with <.5m/s tangent are omitted from component metrics, not assigned
    the detector heading. Euclidean errors retain every sample and origin.
    """
    points = np.vstack([np.asarray(current_gt)[:2], reference])
    velocity = np.empty_like(reference)
    velocity[:-1] = (points[2:]-points[:-2])/.2
    velocity[-1] = (points[-1]-points[-2])/.1
    speed = np.linalg.norm(velocity, axis=1)
    return [((v/s).tolist() if s >= minimum_speed else None) for v, s in zip(velocity, speed)]


def error_metrics(predicted, reference, axes, horizon):
    count = int(horizon*10)
    predicted, reference = np.asarray(predicted)[:count], np.asarray(reference)[:count]
    if len(predicted) != count or len(reference) != count:
        raise ValueError('Full fixed-horizon trajectory required')
    error = predicted-reference
    norms = np.linalg.norm(error, axis=1)
    result = dict(ADE_m=float(norms.mean()), FDE_m=float(norms[-1]))
    components = [(float(np.dot(e, axis)), float(np.dot(e, [-axis[1], axis[0]])))
                  for e, axis in zip(error, axes[:count]) if axis is not None]
    result['component_samples'] = len(components)
    for j, name in enumerate(('longitudinal', 'lateral')):
        values = [v[j] for v in components]
        result[name+'_signed_ADE_m'] = float(np.mean(values)) if values else None
        result[name+'_abs_ADE_m'] = float(np.mean(np.abs(values))) if values else None
        if axes[count-1] is None:
            signed = None
        else:
            axis = axes[count-1]
            signed = float(np.dot(error[-1], axis if j == 0 else [-axis[1], axis[0]]))
        result[name+'_signed_FDE_m'] = signed
        result[name+'_abs_FDE_m'] = abs(signed) if signed is not None else None
    return result


def make_ctrv():
    """Use the actual production predict_pose function and body-frame contract."""
    sys.path.insert(0, str(ROOT/'base_ws/src/perception_object_pipeline/src'))
    from autoware_perception_msgs.msg import ObjectClassification, TrackedObject
    from perception_object_pipeline.kinematic_prediction import predict_pose, yaw_to_quaternion

    def predict(obj):
        tracked = TrackedObject()
        tracked.classification = [ObjectClassification(label=obj['label'], probability=1.)]
        pose = tracked.kinematics.pose_with_covariance.pose
        pose.position.x, pose.position.y, pose.position.z = obj['center']
        pose.orientation = yaw_to_quaternion(obj['yaw'])
        twist = tracked.kinematics.twist_with_covariance.twist
        twist.linear.x, twist.linear.y = obj['body_velocity']
        twist.angular.z = obj['yaw_rate']
        return [[p.position.x, p.position.y] for p in (predict_pose(tracked, float(t)) for t in TIMES)]
    return predict


def cartesian_cv(obj):
    return (np.asarray(obj['center'][:2])+TIMES[:, None]*np.asarray(obj['world_velocity'])).tolist()


def replace_longitudinal(obj, gt_velocity):
    result = deepcopy(obj)
    c, s = math.cos(obj['yaw']), math.sin(obj['yaw'])
    long_speed = float(np.dot(gt_velocity, [c, s]))
    lateral = obj['body_velocity'][1]
    result['body_velocity'] = [long_speed, lateral]
    result['world_velocity'] = [c*long_speed-s*lateral, s*long_speed+c*lateral]
    return result


def fixed_route_prediction(network, obj, route_ids, reject_outside=True):
    """Actual production geometry/convergence, with a fixed original route.

    No lane search, route enumeration, reranking, endpoint clamp or extension.
    Signed speed may differ from original route's supported domain: abstain.
    This deliberately freezes the original mapped decision/minimum-speed gate.
    """
    geometry, projection = network._route_geometry(tuple(route_ids))
    start = projection.project(*obj['center'][:2])
    if start is None:
        return None, 'fixed_route_projection_missing'
    distances = start.arc_length+obj['body_velocity'][0]*TIMES
    if reject_outside and (distances.min() < -1.e-9 or distances.max() > geometry.length+1.e-9):
        return None, 'oracle_speed_exceeds_fixed_route_domain'
    offset = np.asarray(obj['center'])-np.asarray([start.x, start.y, start.z])
    convergence = network.config.lateral_convergence_time_sec
    paths = []
    for t, distance in zip(TIMES, distances):
        residual = max(0., 1.-t/convergence) if convergence > 1.e-6 else 0.
        point = np.asarray(geometry.sample(float(distance)))+residual*offset
        paths.append(point[:2].tolist())
    return paths, None


def lifecycle(case):
    """Causal input-history labels, not a claim of 30Hz scheduler continuity."""
    previous = {}; last_reacquired = {}; seen_associated = set(); result = {}
    for index, frame in enumerate(case['frames']):
        native = int(frame.get('native_index', index)); stamp = frame['stamp']
        current = {}
        for track in frame['tracks']:
            if not track['confirmed'] or track['tracker_label'] not in (1, 2):
                continue
            identity = str(track['id']); prior = previous.get(identity)
            associated = bool(track['associated'])
            event = bool(prior and prior['native']+1 == native and stamp-prior['stamp'] <= .25
                         and not prior['associated'] and associated and identity in seen_associated)
            if event:
                last_reacquired[identity] = stamp
            if associated:
                seen_associated.add(identity)
            age = stamp-float(track.get('birth_stamp', stamp))
            recent = stamp-last_reacquired.get(identity, -math.inf) < 1.
            stage = ('initial_lt1s' if age < 1. else 'coasting' if not associated else
                     'recent_same_id_reacquisition_lt1s' if recent else 'mature_associated_no_recent_reacq')
            result[(stamp, identity)] = dict(stage=stage, associated=associated,
                track_age_s=age, recent_reacquisition=recent, reacquisition_event=event,
                measurement_age_s=stamp-float(track.get('last_measurement_stamp', stamp)),
                source_native_index=native)
            current[identity] = dict(native=native, stamp=stamp, associated=associated)
        previous = current
    return result


def frozen_key(row):
    return (row['case'], row['stamp'], str(row['id']), key_tuple(row['gt_key']), row['horizon_s'])


def compare_frozen(row, frozen, horizon):
    expected = frozen[frozen_key(dict(row, horizon_s=horizon))]
    metric = row['metrics']['mapcv3_top1'][str(horizon)]
    differences = [abs(metric[name]-expected['top1_'+name]) for name in ('ADE_m', 'FDE_m')]
    differences.extend(abs(np.asarray(row['paths']['mapcv3_top1'][horizon*10-1])
                           -np.asarray(expected['top1_endpoint'])))
    if (max(differences) > 1.e-7 or row['mapped'] != expected['mapped']
            or row['top1_lanelet_ids'] != expected['top1_lanelet_ids']):
        raise ValueError('Baseline Map-CV does not reproduce frozen v4')
    return float(max(differences))


def summarize(rows, methods=BASE_METHODS+DIAGNOSTICS):
    result = dict(origins=len(rows), actors=len({(r['case'], key_tuple(r['gt_key'])) for r in rows}),
                  tracks=len({(r['case'], str(r['id'])) for r in rows}),
                  episodes=len({r['episode_id'] for r in rows}), methods={})
    for method in methods:
        selected = [r for r in rows if method in r['metrics']]
        result['methods'][method] = {str(h): {metric: distribution([r['metrics'][method][str(h)][metric]
            for r in selected if r['metrics'][method][str(h)][metric] is not None])
            for metric in ('ADE_m', 'FDE_m', 'longitudinal_signed_ADE_m', 'longitudinal_abs_ADE_m',
                           'lateral_signed_ADE_m', 'lateral_abs_ADE_m', 'longitudinal_signed_FDE_m',
                           'longitudinal_abs_FDE_m', 'lateral_signed_FDE_m', 'lateral_abs_FDE_m')}
            for h in (1, 2, 3)}
    return result


def paired(rows, first, second):
    chosen = [r for r in rows if first in r['metrics'] and second in r['metrics']]
    result = dict(paired_origins=len(chosen), by_horizon={})
    for h in (1, 2, 3):
        deltas = [r['metrics'][second][str(h)]['FDE_m']-r['metrics'][first][str(h)]['FDE_m'] for r in chosen]
        actors = defaultdict(list); episodes = defaultdict(list); cases = defaultdict(list)
        for row, delta in zip(chosen, deltas):
            actors[(row['case'], key_tuple(row['gt_key']))].append(delta)
            episodes[row['episode_id']].append(delta); cases[row['case']].append(delta)
        result['by_horizon'][str(h)] = dict(delta_FDE_m=distribution(deltas),
            delta_ADE_m=distribution([r['metrics'][second][str(h)]['ADE_m']-r['metrics'][first][str(h)]['ADE_m'] for r in chosen]),
            worsened_gt_1m=sum(d > 1. for d in deltas), improved_gt_1m=sum(d < -1. for d in deltas),
            worsened_gt_1m_fraction=sum(d > 1. for d in deltas)/len(deltas) if deltas else None,
            improved_gt_1m_fraction=sum(d < -1. for d in deltas)/len(deltas) if deltas else None,
            actor_mean_delta_FDE_m=distribution([float(np.mean(v)) for v in actors.values()]),
            episode_mean_delta_FDE_m=distribution([float(np.mean(v)) for v in episodes.values()]),
            case_mean_delta_FDE_m={k: float(np.mean(v)) for k, v in cases.items()})
    return result


def all_summaries(rows):
    strata = dict(all=lambda r: True, mapped=lambda r: r['mapped'], fallback=lambda r: not r['mapped'],
        static_gt=lambda r: r['gt_causal_speed_mps'] is not None and r['gt_causal_speed_mps'] < .5,
        moving_gt=lambda r: r['gt_causal_speed_mps'] is not None and r['gt_causal_speed_mps'] >= .5,
        initial_lt1s=lambda r: r['stage'] == 'initial_lt1s', coasting=lambda r: not r['associated'],
        recent_reacquisition=lambda r: r['recent_reacquisition'],
        mature_associated_no_recent_reacq=lambda r: r['stage'] == 'mature_associated_no_recent_reacq')
    pairs = [('cv', 'ctrv'), ('cv', 'mapcv3_top1'), ('ctrv', 'mapcv3_top1')]
    pairs += list(zip(BASE_METHODS, DIAGNOSTICS))
    return {name: dict(**summarize(selected), paired={a+'_to_'+b: paired(selected, a, b) for a, b in pairs})
            for name, choose in strata.items() for selected in [[r for r in rows if choose(r)]]}


def evaluate_case(name, case, frames, trajectories, predict, ctrv, network, frozen):
    counts = Counter(); rows = []; lifecycle_by_key = lifecycle(case)
    episodes = Counter(); prior_origins = {}; frozen_seen = set(); max_difference = 0.
    for frame_index, frame in enumerate(frames):
        predicted, _ = predict(frame, 3)
        counts['all_input_frames'] += 1
        if not frame['scoring_eligible']:
            counts['excluded_scoring_frames'] += 1
            continue
        counts['scoring_frames'] += 1; counts['track_origins'] += len(frame['objects'])
        pairs = current_assignment(frame['objects'], frame['gt'])
        counts['current_gt_origins'] += len(frame['gt']); counts['current_2m_matches'] += len(pairs)
        counts['unmatched_track_origins'] += len(frame['objects'])-len(pairs)
        counts['unmatched_gt_origins'] += len(frame['gt'])-len(pairs)
        for oi, gi, current_error in pairs:
            obj, gt = frame['objects'][oi], frame['gt'][gi]; key = key_tuple(gt['key'])
            reference, reason = future_reference(trajectories[key], frame['stamp'], 3)
            counts['future_3s_'+(reason or 'supported')] += 1
            if reference is None:
                continue
            velocity, velocity_reason, velocity_span = causal_velocity(trajectories[key], frame['stamp'], with_span=True)
            axes = evaluation_axes(gt['center'], reference)
            meta = lifecycle_by_key[(frame['stamp'], str(obj['id']))]
            episode_key = (str(obj['id']), key)
            prior = prior_origins.get(episode_key)
            if prior is None or meta['source_native_index'] != prior+1:
                episodes[episode_key] += 1
            prior_origins[episode_key] = meta['source_native_index']
            actual = predicted[oi]
            paths = dict(cv=cartesian_cv(obj), ctrv=ctrv(obj), mapcv3_top1=actual['paths'][0]['xy'])
            diagnostic_reason = velocity_reason
            if velocity is not None:
                diagnostic = replace_longitudinal(obj, velocity)
                paths['cv_gt_longitudinal'] = cartesian_cv(diagnostic)
                paths['ctrv_gt_longitudinal'] = ctrv(diagnostic)
                if actual['mapped']:
                    path, diagnostic_reason = fixed_route_prediction(network, diagnostic, actual['paths'][0]['lanelet_ids'])
                    # Verify exact production geometry before using diagnostic route math.
                    baseline, baseline_reason = fixed_route_prediction(network, obj, actual['paths'][0]['lanelet_ids'], reject_outside=False)
                    if baseline_reason or np.max(np.abs(np.asarray(baseline)-np.asarray(paths['mapcv3_top1']))) > 1.e-7:
                        raise ValueError('Fixed route sampler differs from production baseline')
                    if path is not None:
                        paths['mapcv3_fixed_route_gt_longitudinal'] = path
                else:
                    paths['mapcv3_fixed_route_gt_longitudinal'] = paths['ctrv_gt_longitudinal']
            row = dict(case=name, stamp=frame['stamp'], token=frame['token'], id=obj['id'], gt_key=gt['key'],
                episode_id=repr((name, episode_key, episodes[episode_key])), current_error_m=current_error,
                mapped=actual['mapped'], fallback_reason=actual['reason'], top1_lanelet_ids=actual['paths'][0]['lanelet_ids'],
                center=obj['center'], yaw=obj['yaw'], world_velocity=obj['world_velocity'],
                body_velocity=obj['body_velocity'], yaw_rate=obj['yaw_rate'], **meta,
                gt_center=gt['center'], reference_xy=reference.tolist(), evaluation_axes_xy=axes,
                gt_causal_velocity=velocity.tolist() if velocity is not None else None,
                gt_velocity_window=velocity_span,
                gt_causal_speed_mps=float(np.linalg.norm(velocity)) if velocity is not None else None,
                gt_velocity_unavailable_reason=velocity_reason, diagnostic_map_unavailable_reason=diagnostic_reason,
                gt_lidar_points=gt.get('lidar_points'),
                current_gt_range_m=math.hypot(*gt['local_box'][:2]) if gt.get('local_box') else None,
                paths=paths, map_paths=actual['paths'], metrics={method: {str(h): error_metrics(path, reference[:h*10], axes, h)
                        for h in (1, 2, 3)} for method, path in paths.items()},
                map_minK_oracle={str(h): {k: v for k, v in path_errors(actual['paths'], reference[:h*10], h).items()
                    if k in ('minK_ADE_m', 'minK_FDE_m', 'oracle_ADE_path_index', 'oracle_FDE_path_index')}
                    for h in (1, 2, 3)})
            for h in (1, 2, 3):
                max_difference = max(max_difference, compare_frozen(row, frozen, h))
                frozen_seen.add(frozen_key(dict(row, horizon_s=h)))
            rows.append(row)
    return rows, dict(counts=counts, frozen_max_abs_difference=max_difference, frozen_seen=frozen_seen)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack', type=Path, default=A/'tracker_v1/replay_pack.json.gz')
    parser.add_argument('--frozen-rows', type=Path, default=A/'map_horizon/map_horizon_v4/rows.json.gz')
    parser.add_argument('--parameters', type=Path, default=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml')
    parser.add_argument('--transform', type=Path, default=ROOT/'configs/tf/morai_lanelet_static_tf.yaml')
    parser.add_argument('--map-file', type=Path, default=ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm')
    parser.add_argument('--projector-file', type=Path, default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--output', type=Path, default=ALLOWED_OUTPUT)
    args = parser.parse_args()
    args.output.resolve().relative_to(A.resolve())
    if args.output.exists():
        raise FileExistsError('Fresh output required; frozen evidence must never be overwritten')
    paths = [args.pack, args.frozen_rows, args.parameters, args.transform, args.map_file, args.projector_file,
             Path(__file__), Path(__file__).with_name('analyze_e12_map_horizon.py'), CORE_PATH, NODE_PATH,
             ROOT/'scripts/evaluation/audit_vehicle_map_prediction.py',
             ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/kinematic_prediction.py']
    pins = {str(p): digest(p) for p in paths}
    packed = load(args.pack)
    if (packed.get('schema') != 'e12_postprocessing_core_pack_v1' or not packed.get('complete')
        or any(c['provenance']['recorded_model']['checkpoint_sha256'] != E12_SHA for c in packed['cases'].values())):
        raise ValueError('Completed selected E12 checkpoint pack required')
    frozen = {frozen_key(r): r for r in load(args.frozen_rows)
              if r['output_horizon_s'] == 3 and r['horizon_s'] in (1, 2, 3)}
    expected = {k for k in frozen if k[-1] == 3}
    parameters = yaml.safe_load(args.parameters.read_text())
    parameters = parameters.get('map_based_prediction', parameters)
    transform = yaml.safe_load(args.transform.read_text())['map_to_morai_map']
    predict, map_sha = load_predictors(parameters, args.map_file, args.projector_file)
    from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
    config = MapPredictionConfig(**{k: v for k, v in parameters.items() if k in MapPredictionConfig.__dataclass_fields__})
    network = LaneletNetwork.load(args.map_file, args.projector_file, config)
    ctrv = make_ctrv()
    args.output.mkdir(parents=True)
    report = dict(schema='e12_motion_ablation_v1', source_sha256=pins, map_sha256=map_sha,
        production_changed=False, canonicalization=CANONICAL, cases={},
        protocol=dict(association='Current XY 2m one-to-one assignment frozen before future GT',
            cohort='Identical valid continuous original-index 3s GT cohort for all methods and 1/2/3s endpoints',
            axes='Per-future-sample GT trajectory tangent; positive longitudinal ahead, positive lateral left; speed<0.5m/s samples excluded from components, retained in Euclidean metrics',
            causal_gt_velocity='Backward secant .1-.25s, exact eligible current sample, native adjacency and every gap<=.15s, no t+future',
            diagnostics='Replace only signed body-x component with dot(causal_GT_velocity, tracker_heading), preserve pose/yaw/yaw-rate/body-y. Original Map mapped/fallback and Top1 route fixed; no reroute or clamp; unsupported fixed-route domain abstains',
            stages='Initial elapsed birth age<1s; mature_associated_no_recent_reacq is not proof of stability. Same-ID reacquisition requires prior association then coasting and contiguous confirmed false->true; recent within1s. Coasting/reacquisition strata overlap age labels.'),
        limitations=['Saved eager detector-input-stamp EKF core replay, not live ROS 30Hz/LAN/closed-loop certification.',
            'Conditional on current2m match and valid3s future; misses/unmatched/current coverage separately counted.',
            'Frame observations are correlated; actor/episode/case macro paired deltas are descriptive, not independent significance.',
            'Cartesian CV uses full body-x/body-y world velocity, while actual vehicle CTRV/Map ignores body-y; Map versus CTRV is the cleaner map-guidance comparison.',
            'Backward GT BBox-center secant velocity is not instantaneous MORAI rear-axle speed.',
            'Future GT tangent is evaluation-only and undefined for stationary samples. GT-speed diagnostic is not a runtime candidate.',
            'Map minK is a future-GT oracle lower bound, never an implemented route selector.'])
    (args.output/'STARTED.json').write_text(json.dumps(report, indent=2))
    all_rows = []; seen = set(); max_difference = 0.
    for name, case in packed['cases'].items():
        frames, trajectories = prepare_case(case, transform)
        rows, detail = evaluate_case(name, case, frames, trajectories, predict, ctrv, network, frozen)
        all_rows.extend(rows); seen.update(detail.pop('frozen_seen'))
        max_difference = max(max_difference, detail['frozen_max_abs_difference'])
        report['cases'][name] = dict(**detail, **all_summaries(rows))
        print(json.dumps(dict(case=name, origins=len(rows), counts=detail['counts'])), flush=True)
    observed = {k for k in seen if k[-1] == 3}
    if observed != expected:
        raise ValueError('3s cohort differs from frozen v4: missing=%d extra=%d' % (len(expected-observed), len(observed-expected)))
    report['frozen_v4_validation'] = dict(origins=len(expected), horizons_checked=3,
        maximum_absolute_metric_or_endpoint_difference=max_difference, exact_cohort=True)
    report['aggregate'] = all_summaries(all_rows)
    report['aggregate']['counts'] = dict(sum((Counter(v['counts']) for v in report['cases'].values()), Counter()))
    report['diagnostic_coverage'] = dict(causal_velocity_reasons=dict(Counter(r['gt_velocity_unavailable_reason'] or 'available' for r in all_rows)),
        map_fixed_route_reasons=dict(Counter(r['diagnostic_map_unavailable_reason'] or 'available' for r in all_rows)))
    report['map_minK_oracle'] = {str(h): {metric: distribution([r['map_minK_oracle'][str(h)][metric] for r in all_rows])
        for metric in ('minK_ADE_m', 'minK_FDE_m')} for h in (1, 2, 3)}
    report['outliers'] = {method: sorted([dict(case=r['case'], stamp=r['stamp'], id=r['id'], gt_key=r['gt_key'],
        stage=r['stage'], mapped=r['mapped'], metrics=r['metrics'][method]['3']) for r in all_rows if method in r['metrics']],
        key=lambda r: -r['metrics']['FDE_m'])[:20] for method in BASE_METHODS}
    if pins != {str(p): digest(p) for p in paths}:
        raise RuntimeError('Pinned source/input changed during analysis')
    with gzip.open(args.output/'rows.json.gz', 'xt') as stream:
        json.dump(all_rows, stream, allow_nan=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,
        report_sha256=digest(args.output/'report.json'), rows_sha256=digest(args.output/'rows.json.gz'),
        source_pins_unchanged=True), indent=2)+'\n')


if __name__ == '__main__':
    main()
