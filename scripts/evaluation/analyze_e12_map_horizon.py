#!/usr/bin/env python3
"""Replay the unchanged Map-CV at 3/5/6 s on saved E12 EKF-core outputs.

No detector/GPU/ROS node/MORAI execution and no production changes. ROS message
classes and the real conversion function are imported offline, without init_node.
Current-frame class-aware 2 m assignment is frozen before any future GT lookup.
Top-1 always means the predictor's first output; min-K is an oracle metric only.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import linear_sum_assignment
import yaml

from audit_vehicle_map_prediction import distribution, interpolate, normalize_frames

ROOT = Path(__file__).resolve().parents[2]
ALLOWED_OUTPUT = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1/map_horizon'
CORE_PATH = ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py'
NODE_PATH = ROOT/'base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py'
E12_SHA = 'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8'
OUTPUT_HORIZONS = (3, 5, 6)
EVALUATION_HORIZONS = (1, 2, 3, 5, 6)
CANONICAL = dict(enabled=True, minimum_speed=1., enter=math.radians(100), exit=math.radians(80))


def key_tuple(value):
    # Reconstructed actor keys may be scalar strings or nested JSON arrays.
    # Never split a string into its characters while canonicalizing arrays.
    return tuple(key_tuple(item) for item in value) if isinstance(value, (list, tuple)) else value


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024**2), b''):
            value.update(block)
    return value.hexdigest()


def load(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as stream:
        return json.load(stream)


def map_point(point, transform):
    if transform.get('parent') != 'map' or transform.get('child') != 'morai_map':
        raise ValueError('Require the existing map <- morai_map rigid transform')
    roll, pitch, yaw = transform['rpy']
    if roll != 0 or pitch != 0 or len(point) != 3:
        raise ValueError('Audited planar map TF and XYZ centers are required')
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = point; dx, dy, dz = transform['xyz']
    return [c*x-s*y+dx, s*x+c*y+dy, z+dz]


def prepare_case(case, transform):
    """Use the existing node-side yaw/body adapter, not GT motion corrections."""
    source = case['frames']
    frames = []
    for frame in source:
        tracks = []
        for track in frame['tracks']:
            if track['tracker_label'] not in (1, 2):
                continue
            if track['label'] != 0:
                raise ValueError('Tracker CAR/TRUCK and evaluation vehicle class disagree')
            tracks.append(dict(track, core_class=track['tracker_label']))
        frames.append(dict(stamp=frame['stamp'], tracks=tracks))
    normalized = normalize_frames(frames, 'core', CANONICAL)
    for frame, original in zip(normalized, source):
        frame['scoring_eligible'] = bool(original['scoring_eligible'])
        frame['token'] = original['token']
        frame['gt'] = [dict(obj, key=key_tuple(obj['key']), center=map_point(obj['center'], transform))
                       for obj in original['gt'] if obj['label'] == 0]
        for obj in frame['objects']:
            obj['center'] = map_point(obj['center'], transform)
            obj['yaw'] += transform['rpy'][2]
            c, s = math.cos(transform['rpy'][2]), math.sin(transform['rpy'][2])
            vx, vy = obj['world_velocity']
            obj['world_velocity'] = [c*vx-s*vy, s*vx+c*vy]
            # Rotating both axes and yaw leaves body_velocity unchanged.
    trajectories = {}
    # An eligible origin must not use uncertain scene-epoch GT as a future
    # target. Keep all input tracker history, but mask BOTH ends of evaluation.
    eligible_stamps = {float(frame['stamp']) for frame in source if frame['scoring_eligible']}
    native_indices = {float(frame['stamp']): int(frame.get('native_index', index))
                      for index, frame in enumerate(source)}
    for item in case['gt_trajectories']:
        if item['label'] != 0:
            continue
        key = key_tuple(item['key'])
        if key in trajectories:
            raise ValueError('Duplicate GT trajectory key')
        supported = [row for row in item['series'] if float(row[0]) in eligible_stamps]
        times = [float(row[0]) for row in supported]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError('GT trajectory requires increasing timestamps')
        trajectories[key] = (times, [map_point(row[1], transform) for row in supported],
                             [native_indices[t] for t in times])
    return normalized, trajectories


def current_assignment(objects, truth, gate=2.):
    """One-to-one, vehicle-only current XY association; never sees future GT."""
    if not objects or not truth:
        return []
    costs = np.linalg.norm(np.asarray([obj['center'][:2] for obj in objects])[:, None]
                           - np.asarray([obj['center'][:2] for obj in truth])[None], axis=2)
    ii, jj = linear_sum_assignment(np.where(costs <= gate, costs, 1.e9))
    return [(int(i), int(j), float(costs[i, j])) for i, j in zip(ii, jj) if costs[i, j] <= gate]


def future_reference(series, stamp, horizon, gap=.15):
    if len(series) > 2:
        times, _, indices = series
        # A 15Hz excluded frame can leave only a .133s gap. A time bound
        # alone must not bridge that invalid scene-epoch/native frame.
        low = max(0, bisect_left(times, stamp)-1)
        high = min(len(times)-1, bisect_left(times, stamp+horizon))
        for index in range(low, high):
            if times[index] < stamp+horizon and times[index+1] > stamp and indices[index+1] != indices[index]+1:
                return None, 'nonadjacent_or_ineligible_original_gt_indices'
    points = [interpolate(series[:2], stamp+i/10., gap) for i in range(1, horizon*10+1)]
    if all(point is not None for point in points):
        return np.asarray(points)[:, :2], None
    if not series[0] or stamp+horizon > series[0][-1]+1.e-6:
        return None, 'recording_or_actor_segment_ended'
    if stamp+.1 < series[0][0]-1.e-6:
        return None, 'actor_segment_not_started'
    return None, 'interpolation_gap_or_missing_samples'


def path_errors(paths, reference, horizon):
    count = horizon*10
    if not paths or any(len(path['xy']) < count for path in paths):
        raise ValueError('Prediction silently terminated before the requested output horizon')
    error = np.linalg.norm(np.asarray([path['xy'][:count] for path in paths])-reference[None], axis=2)
    ade, fde = error.mean(axis=1), error[:, -1]
    return dict(top1_ADE_m=float(ade[0]), top1_FDE_m=float(fde[0]),
        minK_ADE_m=float(ade.min()), minK_FDE_m=float(fde.min()),
        oracle_ADE_path_index=int(ade.argmin()), oracle_FDE_path_index=int(fde.argmin()),
        top1_endpoint=paths[0]['xy'][count-1], gt_endpoint=reference[-1].tolist(),
        step_group_ADE_m={f'{first}_{last}s': float(error[0, first*10:last*10].mean())
            for first, last in ((0, 1), (1, 2), (2, 3), (3, 5), (5, 6)) if last <= horizon})


def prefix_extension_index(short, longer):
    """Research only: first ranked compatible real path, never GT-selected."""
    if not short['mapped'] or not longer['mapped']:
        return None
    baseline = short['paths'][0]
    prefix = baseline['lanelet_ids']
    if not prefix:
        return None
    for index, path in enumerate(longer['paths']):
        if path['lanelet_ids'][:len(prefix)] != prefix:
            continue
        distances = np.linalg.norm(np.asarray(path['xy'][:30])-np.asarray(baseline['xy'][:30]), axis=1)
        if len(distances) == 30 and distances.max() <= .75:
            return index
    return None


def direct_prefix_rows(frame, predicted, case):
    """Compare actual output coordinates without any GT or path-ID proxy."""
    rows = []
    for first, second in ((3, 5), (3, 6), (5, 6)):
        for obj, a, b in zip(frame['objects'], predicted[first], predicted[second]):
            xy_a = np.asarray(a['paths'][0]['xy'][:first*10])
            xy_b = np.asarray(b['paths'][0]['xy'][:first*10])
            distances = np.linalg.norm(xy_b-xy_a, axis=1)
            rows.append(dict(case=case, stamp=frame['stamp'], token=frame['token'], id=obj['id'],
                from_horizon_s=first, to_horizon_s=second,
                prefix_mean_xy_difference_m=float(distances.mean()),
                prefix_max_xy_difference_m=float(distances.max()),
                endpoint_difference_m={str(h): float(distances[h*10-1])
                    for h in EVALUATION_HORIZONS if h <= first},
                from_mapped=a['mapped'], to_mapped=b['mapped'],
                from_top1_route=a['paths'][0]['lanelet_ids'], to_top1_route=b['paths'][0]['lanelet_ids'],
                from_path_count=len(a['paths']), to_path_count=len(b['paths'])))
    return rows


def summarize_prefix(rows):
    result = {}
    for first, second in ((3, 5), (3, 6), (5, 6)):
        selected = [r for r in rows if r['from_horizon_s'] == first and r['to_horizon_s'] == second]
        result[f'{first}s_to_{second}s'] = dict(observations=len(selected),
            changed_prefix_gt1e9=sum(r['prefix_max_xy_difference_m'] > 1.e-9 for r in selected),
            mean_xy_difference_m=distribution([r['prefix_mean_xy_difference_m'] for r in selected]),
            maximum_xy_difference_m=distribution([r['prefix_max_xy_difference_m'] for r in selected]),
            endpoint_difference_m={str(h): distribution([r['endpoint_difference_m'][str(h)] for r in selected])
                for h in EVALUATION_HORIZONS if h <= first})
    return result


def summarize_steps(rows, fixed6=False):
    result = {}
    for output_h in OUTPUT_HORIZONS:
        selected = [r for r in rows if r['output_horizon_s'] == output_h and r['horizon_s'] == output_h
                    and (not fixed6 or r['gt_supported_through_6s'])]
        result[str(output_h)] = {f'{first}_{last}s': distribution([
            r['step_group_ADE_m'][f'{first}_{last}s'] for r in selected])
            for first, last in ((0, 1), (1, 2), (2, 3), (3, 5), (5, 6)) if last <= output_h}
    return result


def summarize_strata(rows):
    result = {}
    for label, select in (
            ('80_90m', lambda r: r['current_gt_range_m'] is not None and 80 <= r['current_gt_range_m'] < 90),
            ('90_100m', lambda r: r['current_gt_range_m'] is not None and 90 <= r['current_gt_range_m'] < 100),
            ('lidar_zero', lambda r: r['gt_lidar_points'] == 0),
            ('lidar_positive', lambda r: r['gt_lidar_points'] is not None and r['gt_lidar_points'] > 0),
            ('front_80_100m', lambda r: r['gt_front'] is True and r['current_gt_range_m'] is not None and 80 <= r['current_gt_range_m'] < 100),
            ('rear_80_100m', lambda r: r['gt_front'] is False and r['current_gt_range_m'] is not None and 80 <= r['current_gt_range_m'] < 100)):
        selected = [r for r in rows if select(r)]
        result[label] = dict(metrics={str(out): summarize_rows([r for r in selected if r['output_horizon_s'] == out]) for out in OUTPUT_HORIZONS},
            fixed_6s_gt_cohort_metrics={str(out): summarize_rows([r for r in selected
                if r['output_horizon_s'] == out and r['gt_supported_through_6s']]) for out in OUTPUT_HORIZONS})
    return result


def summarize_extension(rows):
    result = {}
    for out in (5, 6):
        by_step = {}
        for h in EVALUATION_HORIZONS:
            if h > out:
                continue
            matched = [r for r in rows if r['output_horizon_s'] == out and r['horizon_s'] == h]
            selected = [r for r in matched if r['prefix_extension_available']]
            by_step[str(h)] = dict(default_supported_pairs=len(matched), extension_supported_pairs=len(selected),
                no_extension=len(matched)-len(selected),
                default_FDE_m=distribution([r['top1_FDE_m'] for r in selected]),
                candidate_FDE_m=distribution([r['prefix_extension_top1_FDE_m'] for r in selected]),
                candidate_ADE_m=distribution([r['prefix_extension_top1_ADE_m'] for r in selected]),
                delta_FDE_m=distribution([r['prefix_extension_top1_FDE_m']-r['top1_FDE_m'] for r in selected]),
                delta_ADE_m=distribution([r['prefix_extension_top1_ADE_m']-r['top1_ADE_m'] for r in selected]),
                improved_gt1m=sum(r['prefix_extension_top1_FDE_m'] < r['top1_FDE_m']-1 for r in selected),
                worsened_gt1m=sum(r['prefix_extension_top1_FDE_m'] > r['top1_FDE_m']+1 for r in selected))
        result[str(out)] = by_step
    return result


def route_diagnostic(network, obj, horizon, mapped):
    if mapped:
        return 'mapped'
    speed = obj['body_velocity'][0]
    if speed < network.config.minimum_speed_mps:
        return 'signed_speed_below_minimum'
    candidates = network.lane_candidates(*obj['center'][:2], obj['yaw'])
    if not candidates:
        return 'no_lane_within_distance_and_yaw_gate'
    required = speed*horizon+network.config.route_length_padding_m
    lengths = []
    for candidate in candidates:
        for ids, _ in network._route_sequences(candidate.lanelet_id, candidate.projection.arc_length, required):
            geometry, projection = network._route_geometry(ids)
            start = projection.project(*obj['center'][:2])
            if start is not None:
                lengths.append(geometry.length-start.arc_length)
    return ('route_ends_or_depth_limit_before_horizon' if lengths and max(lengths)+.5 < speed*horizon
            else 'no_scored_complete_map_hypothesis')


def load_predictors(parameters, map_file, projector_file):
    """Import and run actual production math, not a rewritten map/CV model."""
    sys.path.insert(0, str(ROOT/'base_ws/src/perception_object_pipeline/src'))
    import perception_object_pipeline.map_based_prediction as core
    from perception_object_pipeline.kinematic_prediction import yaw_to_quaternion
    from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects
    spec = importlib.util.spec_from_file_location('offline_e12_horizon_map_node', NODE_PATH)
    node = importlib.util.module_from_spec(spec); spec.loader.exec_module(node)
    config = core.MapPredictionConfig(**{key: value for key, value in parameters.items()
                                       if key in core.MapPredictionConfig.__dataclass_fields__})
    loaded = core.LaneletNetwork.load(map_file, projector_file, config)
    networks = {h: core.LaneletNetwork(tuple(loaded.records.values()), config,
                    map_sha256=loaded.map_sha256, lanelet_layer=loaded._lanelet_layer)
                for h in OUTPUT_HORIZONS}

    def predict(frame, horizon):
        msg = TrackedObjects(); msg.header.frame_id = 'map'
        for obj in frame['objects']:
            tracked = TrackedObject()
            tracked.classification = [ObjectClassification(label=obj['label'], probability=1.)]
            p = tracked.kinematics.pose_with_covariance.pose
            p.position.x, p.position.y, p.position.z = obj['center']
            p.orientation = yaw_to_quaternion(obj['yaw'])
            twist = tracked.kinematics.twist_with_covariance.twist
            twist.linear.x, twist.linear.y = obj['body_velocity']; twist.angular.z = obj['yaw_rate']
            msg.objects.append(tracked)
        network = networks[horizon]; original = network.predict; mapped = []

        def observe(*args, **kwargs):
            value = original(*args, **kwargs)
            mapped.append(value)
            return value

        network.predict = observe
        try:
            started = time.perf_counter()
            output, mapped_n, fallback_n, path_n = node.make_map_based_predicted_objects(
                msg, [i/10. for i in range(1, horizon*10+1)], network)
            elapsed = (time.perf_counter()-started)*1000.
        finally:
            network.predict = original
        if len(mapped) != len(frame['objects']) or len(output.objects) != len(mapped):
            raise ValueError('Actual node call count/object order differs')
        result = []
        for obj, predicted, trajectories in zip(frame['objects'], output.objects, mapped):
            paths = []
            for index, path in enumerate(predicted.kinematics.predicted_paths):
                if len(path.path) != horizon*10+1:
                    raise ValueError('Actual node did not return current pose plus complete future samples')
                paths.append(dict(xy=[[p.position.x, p.position.y] for p in path.path[1:]],
                    confidence=float(path.confidence),
                    lanelet_ids=list(trajectories[index].lanelet_ids) if trajectories else []))
            result.append(dict(mapped=bool(trajectories), paths=paths,
                reason=route_diagnostic(network, obj, horizon, bool(trajectories))))
        return result, dict(ms=elapsed, mapped=mapped_n, fallback=fallback_n, paths=path_n)

    return predict, loaded.map_sha256


def summarize_rows(rows):
    return {str(h): {metric: distribution([r[metric] for r in rows if r['horizon_s'] == h])
                    for metric in ('top1_ADE_m', 'top1_FDE_m', 'minK_ADE_m', 'minK_FDE_m')}
            for h in EVALUATION_HORIZONS}


def evaluate_case(frames, trajectories, predict, case_name, max_gap=.15):
    counts = Counter(); coverage = {h: Counter() for h in OUTPUT_HORIZONS}
    timings = {h: [] for h in OUTPUT_HORIZONS}; rows = []; frame_rows = []; prefix_rows = []
    for frame_index, frame in enumerate(frames):
        counts['all_input_frames'] += 1
        # Different horizon changes route enumeration, pruning and clustering.
        # Never truncate one 6-second prediction to represent 3/5 seconds.
        order = OUTPUT_HORIZONS[frame_index%3:]+OUTPUT_HORIZONS[:frame_index%3]
        predicted = {}; timing = {}
        for output_h in order:
            predicted[output_h], timing[output_h] = predict(frame, output_h)
            if len(predicted[output_h]) != len(frame['objects']):
                raise ValueError('Predictor changed object order/count')
            if frame_index >= 30 and frame['objects']:
                timings[output_h].append(timing[output_h]['ms'])
        frame_rows.append(dict(stamp=frame['stamp'], vehicles=len(frame['objects']),
            timing_ms={str(h): timing[h]['ms'] for h in OUTPUT_HORIZONS}))
        if not frame['scoring_eligible']:
            counts['excluded_scoring_frames'] += 1
            continue
        counts['scoring_frames'] += 1; counts['vehicle_track_observations'] += len(frame['objects'])
        prefix_rows.extend(direct_prefix_rows(frame, predicted, case_name))
        counts['current_vehicle_gt_observations'] += len(frame['gt'])
        pairs = current_assignment(frame['objects'], frame['gt'])
        counts['current_2m_matched_observations'] += len(pairs)
        counts['unmatched_track_observations'] += len(frame['objects'])-len(pairs)
        counts['unmatched_current_gt_observations'] += len(frame['gt'])-len(pairs)
        for output_h in OUTPUT_HORIZONS:
            for object_index, result in enumerate(predicted[output_h]):
                coverage[output_h]['vehicle_track_observations'] += 1
                coverage[output_h]['map_output' if result['mapped'] else 'kinematic_fallback'] += 1
                coverage[output_h][result['reason']] += 1
                if output_h > 3:
                    extension = prefix_extension_index(predicted[3][object_index], result)
                    coverage[output_h]['prefix_extension_available' if extension is not None else 'prefix_extension_unavailable'] += 1
        for index, gt_index, current_error in pairs:
            obj, truth = frame['objects'][index], frame['gt'][gt_index]
            key = key_tuple(truth['key'])
            if key not in trajectories:
                counts['matched_gt_without_trajectory'] += 1
                continue
            series = trajectories[key]
            full6, _ = future_reference(series, frame['stamp'], 6, max_gap)
            for horizon in EVALUATION_HORIZONS:
                reference, reason = future_reference(series, frame['stamp'], horizon, max_gap)
                counts['future_'+str(horizon)+'s_'+('supported' if reason is None else reason)] += 1
                if reason is not None:
                    continue
                for output_h in OUTPUT_HORIZONS:
                    if horizon > output_h:
                        continue
                    result = predicted[output_h][index]
                    errors = path_errors(result['paths'], reference, horizon)
                    extension = prefix_extension_index(predicted[3][index], result) if output_h > 3 else None
                    extension_errors = (path_errors([result['paths'][extension]], reference, horizon)
                                        if extension is not None else {})
                    local_box = truth.get('local_box')
                    rows.append(dict(case=case_name, stamp=frame['stamp'], token=frame['token'],
                        id=obj['id'], gt_key=truth['key'], output_horizon_s=output_h, horizon_s=horizon,
                        current_error_m=current_error, gt_supported_through_6s=full6 is not None,
                        mapped=result['mapped'], fallback_reason=result['reason'],
                        top1_lanelet_ids=result['paths'][0]['lanelet_ids'],
                        path_count=len(result['paths']), top1_confidence=result['paths'][0]['confidence'], **errors))
                    rows[-1].update(current_gt_range_m=math.hypot(*local_box[:2]) if local_box else None,
                        gt_front=bool(local_box[0] >= 0) if local_box else None,
                        gt_lidar_points=truth.get('lidar_points'),
                        prefix_extension_available=extension is not None, prefix_extension_index=extension,
                        prefix_extension_top1_ADE_m=extension_errors.get('top1_ADE_m'),
                        prefix_extension_top1_FDE_m=extension_errors.get('top1_FDE_m'))
    metrics = {str(out): summarize_rows([r for r in rows if r['output_horizon_s'] == out]) for out in OUTPUT_HORIZONS}
    fixed = {str(out): summarize_rows([r for r in rows if r['output_horizon_s'] == out and r['gt_supported_through_6s']]) for out in OUTPUT_HORIZONS}
    return dict(counts=dict(counts), coverage={str(h): dict(v) for h, v in coverage.items()},
        metrics=metrics, fixed_6s_gt_cohort_metrics=fixed,
        nonempty_frame_cpu_ms_excluding_first30={str(h): distribution(v) for h, v in timings.items()},
        direct_prefix=summarize_prefix(prefix_rows), step_groups=summarize_steps(rows),
        fixed_6s_gt_cohort_step_groups=summarize_steps(rows, True), range_support_strata=summarize_strata(rows),
        offline_prefix_extension_candidate=summarize_extension(rows),
        fixed_6s_gt_cohort_prefix_extension=summarize_extension([r for r in rows if r['gt_supported_through_6s']])), rows, frame_rows, prefix_rows


def paired_horizon_effects(rows):
    result = {}
    for first, second in ((3, 5), (3, 6), (5, 6)):
        def indexed(output):
            return {(r['case'], r['stamp'], key_tuple(r['gt_key']), r['horizon_s']): r
                    for r in rows if r['output_horizon_s'] == output}
        a, b = indexed(first), indexed(second)
        keys = sorted(set(a)&set(b), key=repr)
        result[f'{first}s_to_{second}s'] = {str(h): dict(
            paired_observations=sum(key[-1] == h for key in keys),
            delta_ADE_m=distribution([b[key]['top1_ADE_m']-a[key]['top1_ADE_m'] for key in keys if key[-1] == h]),
            delta_FDE_m=distribution([b[key]['top1_FDE_m']-a[key]['top1_FDE_m'] for key in keys if key[-1] == h]),
            top1_lanelet_route_changed=sum(a[key]['top1_lanelet_ids'] != b[key]['top1_lanelet_ids'] for key in keys if key[-1] == h),
            mapped_to_fallback=sum(a[key]['mapped'] and not b[key]['mapped'] for key in keys if key[-1] == h),
            fallback_to_mapped=sum(not a[key]['mapped'] and b[key]['mapped'] for key in keys if key[-1] == h))
            for h in EVALUATION_HORIZONS if h <= first}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, default=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml')
    parser.add_argument('--transform', type=Path, default=ROOT/'configs/tf/morai_lanelet_static_tf.yaml')
    parser.add_argument('--map-file', type=Path, default=ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm')
    parser.add_argument('--projector-file', type=Path, default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--case', action='append', help='Exact saved case name; absent preserves every case')
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    a.output.resolve().relative_to(ALLOWED_OUTPUT.resolve())
    if a.output.exists():
        raise FileExistsError('New output directory required; previous evidence is never overwritten')
    paths = [a.pack, a.parameters, a.transform, a.map_file, a.projector_file, Path(__file__), CORE_PATH, NODE_PATH,
        ROOT/'scripts/evaluation/audit_vehicle_map_prediction.py',
        ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/kinematic_prediction.py']
    pins = {str(path): digest(path) for path in paths}
    packed = load(a.pack)
    if (packed.get('schema') != 'e12_postprocessing_core_pack_v1' or packed.get('complete') is not True
            or not packed.get('cases') or any(case.get('provenance', {}).get('recorded_model', {}).get(
                'checkpoint_sha256') != E12_SHA for case in packed['cases'].values())):
        raise ValueError('Every completed pack case must bind the selected E12 checkpoint SHA')
    parameters = yaml.safe_load(a.parameters.read_text())
    parameters = parameters.get('map_based_prediction', parameters)
    transform = yaml.safe_load(a.transform.read_text())['map_to_morai_map']
    predict, map_sha = load_predictors(parameters, a.map_file, a.projector_file)
    a.output.mkdir(parents=True)
    report = dict(schema='base5_e12_map_horizon_offline_v1', source_sha256=pins,
        pack_provenance=packed['provenance'], output_horizons_s=OUTPUT_HORIZONS,
        evaluation_horizons_s=EVALUATION_HORIZONS, dt_sec=.1, interpolation_max_gap_sec=.15,
        current_association_gate_m=2., map_sha256=map_sha, canonicalization=CANONICAL,
        coordinate_note='Saved world has Ego heading correction once. Apply map rigid TF only; no new heading/time correction.',
        production_changed=False, cases={}, aggregate={},
        limitations=[
            'Vehicle-only current 2m matched conditional forecasts; unmatched/missing tracks counted separately, not hidden in ADE/FDE.',
            'All input history preserved; accuracy origins AND future GT trajectory samples require original scoring_eligible timestamps.',
            'ADE excludes current pose and uses every future 0.1s sample. FDE uses the same continuously supported GT cohort; no extrapolation or bridging nonadjacent original GT indices.',
            'Independent 3/5/6s real predictor invocations. Longer horizon can change even 1/2/3s paths by route pruning/clustering.',
            'Top1 is actual output order. minK ADE/FDE are separate future-GT oracle lower bounds, not deployed path selection.',
            'Ground truth is catalog-center/recorded-clock evaluation, not annotation-complete tracking/closed-loop safety certification.',
            'Actual EKF core + explicit node yaw/body/map adapter, not native 30Hz ROS scheduler or async TF/queue reproduction.',
            'CPU timing is offline make-message math, GC enabled, no publish/serialize/transport; not team-PC/LAN latency.'])
    report['offline_extension_candidate'] = dict(production_enabled=False,
        rule='Choose first existing long-horizon path with identical original 3s Top1 lanelet prefix and first3s maximum XY difference <=0.75m. No future GT, route creation or extrapolation.',
        missing_extension='Abstain and report coverage; conditional metrics must not be interpreted as every-track improvement.')
    with (a.output/'STARTED.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    all_rows = []; timing_rows = {}; all_prefix_rows = []
    for name, case in packed['cases'].items():
        if a.case and name not in a.case:
            continue
        frames, trajectories = prepare_case(case, transform)
        summary, rows, timing, prefixes = evaluate_case(frames, trajectories, predict, name)
        summary.update(raw_bag=case.get('raw_bag'), provenance=case.get('provenance'),
                       paired_horizon_effects=paired_horizon_effects(rows))
        report['cases'][name] = summary; all_rows.extend(rows); timing_rows[name] = timing
        all_prefix_rows.extend(prefixes)
        print(json.dumps(dict(case=name, counts=summary['counts'])), flush=True)
    if a.case and set(a.case) != set(report['cases']):
        raise ValueError('Requested case is missing')
    report['aggregate'] = dict(metrics={str(h): summarize_rows([r for r in all_rows if r['output_horizon_s'] == h]) for h in OUTPUT_HORIZONS},
        paired_horizon_effects=paired_horizon_effects(all_rows), direct_prefix=summarize_prefix(all_prefix_rows),
        step_groups=summarize_steps(all_rows), fixed_6s_gt_cohort_step_groups=summarize_steps(all_rows, True),
        range_support_strata=summarize_strata(all_rows), offline_prefix_extension_candidate=summarize_extension(all_rows),
        fixed_6s_gt_cohort_prefix_extension=summarize_extension([r for r in all_rows if r['gt_supported_through_6s']]))
    report['outliers'] = {str(out): sorted([r for r in all_rows if r['output_horizon_s'] == out
        and r['horizon_s'] == out], key=lambda r: -r['top1_FDE_m'])[:20] for out in OUTPUT_HORIZONS}
    if pins != {str(path): digest(path) for path in paths}:
        raise RuntimeError('Pinned input/source changed during offline replay')
    for name, value in [('rows.json.gz', all_rows), ('timing_rows.json.gz', timing_rows), ('direct_prefix_rows.json.gz', all_prefix_rows)]:
        with gzip.open(a.output/name, 'xt') as stream:
            json.dump(value, stream, allow_nan=False)
    with (a.output/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
    with (a.output/'COMPLETE.json').open('x') as stream:
        json.dump(dict(complete=True, report_sha256=digest(a.output/'report.json'),
            rows_sha256=digest(a.output/'rows.json.gz'), source_pins_unchanged=True), stream, indent=2)


if __name__ == '__main__':
    main()
