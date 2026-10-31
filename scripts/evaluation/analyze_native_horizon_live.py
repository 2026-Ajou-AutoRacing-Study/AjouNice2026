#!/usr/bin/env python3
"""Read-only native global 3/6-second prediction audit of closed validator bags.

This deliberately does NOT invoke MixedHorizonExtension: a longer native search
may change route representatives, confidence and even the first three seconds.
Each recorded TrackedObjects is sent unchanged through both public converters.
Separate live drives describe functional behavior, not controlled causal proof.
"""
import argparse
from collections import Counter, defaultdict
from copy import copy
from datetime import datetime
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import struct
import sys
import time

import yaml

from analyze_map_mixed_horizon_live import (
    TRACKS, OUTPUT, TIMING, ROUTE, ODOM, input_key, normalized_wire,
    receipt_check, roundabout_internal_ids,
)
from qualify_e12_mixed_horizon import (
    ROOT, SOURCE, CORE_PATH, NODE_PATH, clone_network, digest, distribution,
    receipt_sources, wire,
)

LABELS = {0: 'UNKNOWN', 1: 'CAR', 2: 'TRUCK', 3: 'BUS', 4: 'TRAILER',
          5: 'MOTORCYCLE', 6: 'BICYCLE', 7: 'PEDESTRIAN'}
VALIDATION = '/planning/validation_status'
CONTROL = '/ctrl_cmd'


def class_name(obj):
    label = max(obj.classification, key=lambda c: c.probability).label if obj.classification else 0
    return LABELS.get(label, 'label_'+str(label))


def static_wire(message):
    """Exclude only predictions and publisher seq; retain current state exactly."""
    result = copy(message)
    result.objects = []
    for original in message.objects:
        obj = copy(original)
        obj.kinematics = copy(original.kinematics)
        obj.kinematics.predicted_paths = []
        result.objects.append(obj)
    return normalized_wire(result)


def prefix_geometry_equal(first, second, count=31):
    return (len(first.path) >= count and len(second.path) >= count
            and first.path[:count] == second.path[:count]
            and first.time_step == second.time_step)


def prefix_distances(first, second, count=31):
    if len(first.path) < count or len(second.path) < count:
        raise ValueError('A complete three-second prefix is required')
    # Current pose is shared by construction; exclude t=0 from error averages.
    return [math.hypot(a.position.x-b.position.x, a.position.y-b.position.y)
            for a, b in zip(first.path[1:count], second.path[1:count])]


def confidence_on_wire(path):
    """PredictedPath.confidence is float32; use what downstream actually sees."""
    return struct.unpack('<f', struct.pack('<f', path.confidence))[0]


def compare_native(baseline, longer):
    """Describe changes, never assume append-only or silently pair reordered IDs."""
    if input_key(baseline) != input_key(longer) or static_wire(baseline) != static_wire(longer):
        raise ValueError('Current-state/header/class/ordered UUID contract changed')
    rows = []
    for first, second in zip(baseline.objects, longer.objects):
        paths3, paths6 = first.kinematics.predicted_paths, second.kinematics.predicted_paths
        if not paths3 or not paths6:
            raise ValueError('Native public converter returned an object without paths')
        if any(len(p.path) != 31 for p in paths3) or any(len(p.path) != 61 for p in paths6):
            raise ValueError('Native 3/6-second output must contain exactly 31/61 poses')
        nearest = []
        for index3, path3 in enumerate(paths3):
            candidates = [prefix_distances(path3, path6) for path6 in paths6]
            means = [sum(values)/len(values) for values in candidates]
            index6 = min(range(len(means)), key=means.__getitem__)
            errors = candidates[index6]
            nearest.append(dict(index3=index3, index6=index6, mean_xy_m=means[index6],
                max_xy_m=max(errors), terminal3_xy_m=errors[-1],
                exact_geometry=prefix_geometry_equal(path3, paths6[index6]),
                confidence3=float(path3.confidence), confidence6=float(paths6[index6].confidence)))
        reverse = []
        for index6, path6 in enumerate(paths6):
            candidates = [prefix_distances(path3, path6) for path3 in paths3]
            means = [sum(values)/len(values) for values in candidates]
            index3 = min(range(len(means)), key=means.__getitem__)
            errors = candidates[index3]
            reverse.append(dict(index6=index6, index3=index3, mean_xy_m=means[index3],
                max_xy_m=max(errors), terminal3_xy_m=errors[-1],
                exact_geometry=prefix_geometry_equal(paths3[index3], path6),
                has_exact_native3_prefix=any(prefix_geometry_equal(path3, path6) for path3 in paths3),
                confidence3=confidence_on_wire(paths3[index3]), confidence6=confidence_on_wire(path6),
                eligible_at_0_05=confidence_on_wire(path6) >= .05,
                has_eligible_exact_native3_prefix=any(confidence_on_wire(path3) >= .05 and
                    prefix_geometry_equal(path3, path6) for path3 in paths3)))
        common = min(len(paths3), len(paths6))
        ordered_geometry = len(paths3) == len(paths6) and all(
            prefix_geometry_equal(a, b) for a, b in zip(paths3, paths6))
        ordered_confidence = ([confidence_on_wire(p) for p in paths3] ==
                              [confidence_on_wire(p) for p in paths6])
        same_index_top1 = prefix_distances(paths3[0], paths6[0])
        rows.append(dict(uuid=bytes(first.object_id.uuid).hex(), class_name=class_name(first),
            topk3=len(paths3), topk6=len(paths6),
            eligible_paths3_at_0_05=sum(confidence_on_wire(p) >= .05 for p in paths3),
            eligible_paths6_at_0_05=sum(confidence_on_wire(p) >= .05 for p in paths6),
            path_count_changed=len(paths3) != len(paths6),
            ordered_prefix_geometry_exact=ordered_geometry,
            ordered_confidence_exact=ordered_confidence,
            whole_ordered_prefix_exact=ordered_geometry and ordered_confidence,
            prefix_geometry_index_changes=sum(row['index3'] != row['index6'] and row['exact_geometry']
                                               for row in nearest),
            top1_same_index_mean_xy_m=sum(same_index_top1)/len(same_index_top1),
            top1_same_index_terminal3_xy_m=same_index_top1[-1],
            ordered_common_paths=common, nearest_prefix_representatives=nearest,
            native6_to_native3_nearest_prefix=reverse))
    return rows


def class_horizons(message):
    result = defaultdict(Counter)
    for obj in message.objects:
        counts = result[class_name(obj)]
        lengths = [len(path.path) for path in obj.kinematics.predicted_paths]
        counts['object_observations'] += 1
        counts['objects_all_61'] += bool(lengths) and all(length == 61 for length in lengths)
        counts['objects_all_31'] += bool(lengths) and all(length == 31 for length in lengths)
        counts['empty_or_mixed_or_invalid'] += not lengths or len(set(lengths)) != 1 or any(
            length not in (31, 61) for length in lengths)
        for length in lengths:
            counts['paths_'+str(length)+'_poses'] += 1
    return {key: dict(value) for key, value in result.items()}


def merge_class_counts(target, message):
    for label, counts in class_horizons(message).items():
        target[label].update(counts)


def gap_summary(samples):
    values = [1000.*(b-a) for a, b in zip(samples, samples[1:])]
    positive_duration = samples[-1]-samples[0] if len(samples) > 1 else 0.
    return dict(messages=len(samples), gaps_ms=distribution(values),
        backwards_steps=sum(value < 0 for value in values),
        gaps_over_100ms=sum(value > 100. for value in values),
        gaps_over_250ms=sum(value > 250. for value in values),
        frequency_hz=(len(samples)-1)/positive_duration if positive_duration > 0. else None)


def held_time(samples, start, end, predicate, maximum_gap=.25):
    """Integrate only observed short receipt intervals; never fill long outages."""
    total, matching = 0., 0.
    for (stamp, value), (following, _) in zip(samples, samples[1:]):
        lo, hi = max(stamp, start), min(following, end)
        if hi <= lo or following-stamp > maximum_gap:
            continue
        total += hi-lo
        if predicate(value):
            matching += hi-lo
    return dict(matching_sec=matching, covered_sec=total,
        uncovered_sec=max(0., end-start-total), maximum_integrated_gap_sec=maximum_gap)


def drive_window(run):
    events_path = run/'events.jsonl'
    events = [json.loads(line) for line in events_path.read_text().splitlines() if line]
    begin = next((row for row in events if row.get('event') == 'CONTROLLER_START'), None)
    end = next((row for row in events if row.get('event') == 'RUN_FINISHED'), None)
    if end is None:
        end = next((row for row in events if row.get('event') == 'CASE_RESULT'), None)
    if begin is None or end is None:
        return dict(available=False, start_sec=None, end_sec=None,
                    reason='Missing explicit controller start or run termination event')
    start, finish = (datetime.fromisoformat(row['time']).timestamp() for row in (begin, end))
    return dict(available=finish >= start, start_sec=start, end_sec=finish,
                start_event=begin['event'], end_event=end['event'],
                definition='Controller-start to run-finish, or CASE_RESULT fallback; includes startup acceleration and destination approach, not pure traffic yield')


def comparison_summary(rows):
    by_class = {}
    for label in sorted({row['class_name'] for row in rows}):
        selected = [row for row in rows if row['class_name'] == label]
        nearest = [p for row in selected for p in row['nearest_prefix_representatives']]
        reverse = [p for row in selected for p in row['native6_to_native3_nearest_prefix']]
        by_class[label] = dict(object_observations=len(selected),
            path_count_changed=sum(row['path_count_changed'] for row in selected),
            native3_total_paths=sum(row['topk3'] for row in selected),
            native6_total_paths=sum(row['topk6'] for row in selected),
            native6_net_additional_path_observations=sum(row['topk6']-row['topk3'] for row in selected),
            native3_eligible_paths_at_0_05=sum(row['eligible_paths3_at_0_05'] for row in selected),
            native6_eligible_paths_at_0_05=sum(row['eligible_paths6_at_0_05'] for row in selected),
            eligible_path_count_increased_objects=sum(row['eligible_paths6_at_0_05'] > row['eligible_paths3_at_0_05'] for row in selected),
            eligible_path_count_decreased_objects=sum(row['eligible_paths6_at_0_05'] < row['eligible_paths3_at_0_05'] for row in selected),
            ordered_prefix_geometry_exact=sum(row['ordered_prefix_geometry_exact'] for row in selected),
            ordered_confidence_changed=sum(not row['ordered_confidence_exact'] for row in selected),
            whole_ordered_prefix_exact=sum(row['whole_ordered_prefix_exact'] for row in selected),
            whole_ordered_prefix_exact_fraction=sum(row['whole_ordered_prefix_exact'] for row in selected)/len(selected),
            prefix_geometry_index_changes=sum(row['prefix_geometry_index_changes'] for row in selected),
            top1_same_index_mean_xy_m=distribution([row['top1_same_index_mean_xy_m'] for row in selected]),
            top1_same_index_terminal3_xy_m=distribution([row['top1_same_index_terminal3_xy_m'] for row in selected]),
            nearest_prefix_mean_xy_m=distribution([p['mean_xy_m'] for p in nearest]),
            nearest_prefix_terminal3_xy_m=distribution([p['terminal3_xy_m'] for p in nearest]),
            nearest_prefix_max_xy_m=distribution([p['max_xy_m'] for p in nearest]),
            nearest_prefix_exact_geometry=sum(p['exact_geometry'] for p in nearest),
            nearest_prefix_path_count=len(nearest),
            native6_to_native3_nearest_prefix_mean_xy_m=distribution([p['mean_xy_m'] for p in reverse]),
            native6_to_native3_nearest_prefix_terminal3_xy_m=distribution([p['terminal3_xy_m'] for p in reverse]),
            native6_to_native3_prefix_exact_geometry=sum(p['exact_geometry'] for p in reverse),
            native6_to_native3_any_exact_prefix=sum(p['has_exact_native3_prefix'] for p in reverse),
            native6_to_native3_prefix_path_count=len(reverse),
            native6_prefix_not_exactly_present_in_native3=sum(not p['has_exact_native3_prefix'] for p in reverse),
            native6_eligible_prefix_without_eligible_exact_native3=sum(p['eligible_at_0_05'] and
                not p['has_eligible_exact_native3_prefix'] for p in reverse))
    return by_class


def audit_execution(execution, core, node):
    import rosbag
    from perception_object_pipeline.kinematic_prediction import build_prediction_horizons, quaternion_to_yaw

    bags = [bag for status in ('success', 'failure', 'failed') for bag in execution.glob(status+'/*/*.bag')]
    if len(bags) != 1:
        raise ValueError('Exactly one closed primary bag is required per execution: '+str(execution))
    bag_path, run = bags[0], bags[0].parent
    if Path(str(bag_path)+'.active').exists():
        raise ValueError('Active recorder is not a stable input')
    params_path = run/'ros_parameters.yaml'
    params = yaml.safe_load(params_path.read_text())['map_based_prediction']
    live_horizon = float(params['prediction_horizon'])
    if live_horizon not in (3., 6.) or abs(float(params['prediction_time_step'])-.1) > 1.e-9:
        raise ValueError('Only explicit native 3/6-second, 0.1-second runs are accepted')
    if params.get('mixed_horizon_enabled', False):
        raise ValueError('Native audit must not silently audit append-only mixed extension')
    map_file, projector = Path(params['map_file']), Path(params['projector_file'])
    metadata_path = next(run.glob('planbag*.json'))
    stable = [bag_path, params_path, map_file, projector, execution/'execution.json',
              run/'runtime_receipt.json', metadata_path, run/'events.jsonl']
    stable.extend(path for path in (run/'route_probe_contract.json',) if path.exists())
    hashes = {str(path): digest(path) for path in stable}
    config = core.MapPredictionConfig(**{key: value for key, value in params.items()
        if key in core.MapPredictionConfig.__dataclass_fields__})
    network = core.LaneletNetwork.load(map_file, projector, config)
    networks = {3: clone_network(network), 6: clone_network(network)}
    horizons = {value: build_prediction_horizons(.1, value) for value in (3, 6)}
    ring_ids = roundabout_internal_ids(map_file)
    window = drive_window(run)
    start, end = window['start_sec'], window['end_sec']
    counts, odom_ids, invalid_flags = Counter(), Counter(), Counter()
    classes = defaultdict(Counter)
    comparisons, rows, diagnostics, routes = [], [], [], []
    computed, recorded = defaultdict(list), []
    receipts, headers, speed_samples, brake_samples = [], [], [], []
    offline_times = {3: [], 6: []}
    input_hash = hashlib.sha256()
    with rosbag.Bag(str(bag_path)) as bag:
        topic_counts = {topic: value.message_count for topic, value in bag.get_type_and_topic_info().topics.items()}
        bag_duration = bag.get_end_time()-bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=[TRACKS, OUTPUT, TIMING, ROUTE, ODOM,
                                                         '/CollisionData', VALIDATION, CONTROL]):
            when = stamp.to_sec()
            in_drive = window['available'] and start <= when <= end
            if topic == TRACKS:
                if msg.header.frame_id != 'map':
                    raise ValueError('Recorded TrackedObjects frame must be map')
                before = wire(msg)
                input_hash.update(before)
                index = counts['input_frames']
                values, timing = {}, {}
                for horizon in ((3, 6) if index % 2 == 0 else (6, 3)):
                    wall, cpu = time.perf_counter(), time.thread_time()
                    values[horizon] = node.make_map_based_predicted_objects(msg, horizons[horizon], networks[horizon])
                    timing[horizon] = dict(wall_ms=(time.perf_counter()-wall)*1000.,
                                           thread_cpu_ms=(time.thread_time()-cpu)*1000.)
                    if index >= 30 and msg.objects:
                        offline_times[horizon].append(timing[horizon])
                if wire(msg) != before:
                    raise ValueError('Public converter mutated the full recorded input')
                difference = compare_native(values[3][0], values[6][0])
                comparisons.extend(difference)
                computed[input_key(msg)].append(normalized_wire(values[int(live_horizon)][0]))
                counts.update(input_frames=1, input_objects=len(msg.objects),
                    exact_serialized_prefix_frames=int(normalized_wire(values[3][0]) == normalized_wire(values[6][0], prefix=True)),
                    native3_mapped=values[3][1], native3_fallback=values[3][2], native3_paths=values[3][3],
                    native6_mapped=values[6][1], native6_fallback=values[6][2], native6_paths=values[6][3])
                rows.append(dict(stamp_ns=msg.header.stamp.to_nsec(), receipt_ns=stamp.to_nsec(),
                    objects=difference, timing_ms=timing))
            elif topic == OUTPUT:
                recorded.append(msg)
                merge_class_counts(classes, msg)
                if in_drive:
                    receipts.append(when)
                    headers.append(msg.header.stamp.to_sec())
            elif topic == TIMING:
                sample = json.loads(msg.data)
                if in_drive:
                    diagnostics.append(sample)
            elif topic == ROUTE:
                ids = [int(s.preferred_primitive.id) for s in msg.segments]
                routes.append(dict(preferred_lanelet_ids=ids, internal_ring_ids=sorted(set(ids) & ring_ids)))
            elif topic == ODOM:
                if in_drive:
                    twist = msg.twist.twist.linear
                    speed_samples.append((when, math.hypot(twist.x, twist.y)))
                pose = msg.pose.pose
                candidates = network.lane_candidates(pose.position.x, pose.position.y,
                                                      quaternion_to_yaw(pose.orientation))
                if candidates:
                    odom_ids[candidates[0].lanelet_id] += 1
            elif topic == CONTROL and in_drive:
                brake_samples.append((when, float(msg.brake)))
            elif topic == VALIDATION and in_drive:
                counts['drive_planner_validation_messages'] += 1
                flags = [name for name in msg.__slots__ if name.startswith('is_valid_') and not getattr(msg, name)]
                invalid_flags.update(flags)
                counts['drive_planner_validation_any_invalid'] += bool(flags)
                counts['drive_planner_validation_nonlatency_invalid'] += any(name != 'is_valid_latency' for name in flags)
            elif topic == '/CollisionData':
                counts['collision_messages_with_objects'] += bool(msg.collision_object or msg.collision_objecta)
    live_matches = Counter()
    for msg in recorded:
        matches = computed.get(input_key(msg), ())
        if len(matches) == 1:
            live_matches['matched_unique_input_key'] += 1
            exact = normalized_wire(msg) == matches[0]
            live_matches['full_wire_exact' if exact else 'full_wire_mismatch'] += 1
        else:
            live_matches['ambiguous_input_key' if matches else 'unmatched_input_key'] += 1
    if any(digest(path) != expected for path, expected in hashes.items()):
        raise ValueError('A closed input/protocol file changed while auditing')
    metadata = json.loads(metadata_path.read_text())
    return dict(execution=str(execution), bag=str(bag_path), input_sha256=hashes,
        input_frames_sha256=input_hash.hexdigest(), provenance=receipt_check(execution, run),
        recorded_parameters=params, native_horizon_sec=live_horizon,
        counts=dict(counts), same_input_native3_vs_native6_by_class=comparison_summary(comparisons),
        offline_timing_ms={str(h): {key: distribution([row[key] for row in samples])
            for key in ('wall_ms', 'thread_cpu_ms')} for h, samples in offline_times.items()},
        live_output=dict(frames=len(recorded), per_class={label: dict(values,
            actual_six_second_object_fraction=values['objects_all_61']/values['object_observations'])
            for label, values in classes.items()}, per_class_scope='Whole closed bag, including pre-drive/stop', comparison=dict(live_matches),
            drive_receipt_timing=gap_summary(receipts), drive_header_timing=gap_summary(headers)),
        live_callback=dict(samples=len(diagnostics), metrics={key: distribution([
            row[key] for row in diagnostics if row.get(key) is not None]) for key in (
                'callback_wall_ms_excluding_diagnostics', 'callback_thread_cpu_ms_excluding_diagnostics',
                'message_make_ms', 'synchronous_publish_ms', 'input_header_age_ms', 'callback_start_gap_ms')}),
        route=dict(recorded_routes=routes, ego_sample_scope='Whole closed bag, including pre-drive/stop', ego_best_centerline_candidate_samples=dict(odom_ids),
            ego_internal_ring_best_candidate_samples=sum(n for identity, n in odom_ids.items() if identity in ring_ids)),
        drive=dict(status=metadata.get('status'), reason=metadata.get('reason'), metrics=metadata.get('metrics'),
            safety=metadata.get('safety'), model=metadata.get('model'), window=window,
            stopped_below_0_1mps=(held_time(speed_samples, start, end, lambda value: value < .1)
                                 if window['available'] else dict(available=False)),
            brake_above_0_1=(held_time(brake_samples, start, end, lambda value: value > .1)
                            if window['available'] else dict(available=False)),
            planner_invalid_flags=dict(invalid_flags), failure_analysis=metadata.get('failure_analysis')),
        bag_topics=topic_counts, bag_duration_sec=bag_duration), rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execution', action='append', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'planner_logs/analysis/map_native6_261024_v1/live_audit')
    args = parser.parse_args()
    args.output.resolve().relative_to((ROOT/'planner_logs/analysis').resolve())
    if args.output.exists():
        raise FileExistsError('Fresh output directory required')
    sys.path.insert(0, str(SOURCE))
    import perception_object_pipeline.map_based_prediction as core
    if Path(core.__file__).resolve() != CORE_PATH.resolve():
        raise ValueError('Explicit source core import was not honored')
    spec = importlib.util.spec_from_file_location('native_horizon_actual_map_node', NODE_PATH)
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    sources, installed = receipt_sources()
    sources[str(Path(__file__).resolve())] = digest(Path(__file__))
    sources[str(Path(__file__).with_name('analyze_map_mixed_horizon_live.py'))] = digest(
        Path(__file__).with_name('analyze_map_mixed_horizon_live.py'))
    if not all(row['matches_source'] for row in installed.values()):
        raise ValueError('Source/install predictor mismatch')
    results, frames = {}, {}
    for execution in args.execution:
        result, rows = audit_execution(execution.resolve(), core, node)
        results[execution.name], frames[execution.name] = result, rows
        print(json.dumps(dict(execution=execution.name, native_horizon=result['native_horizon_sec'],
            counts=result['counts'], live_output=result['live_output']['comparison']), ensure_ascii=False), flush=True)
    if any(digest(path) != expected for path, expected in sources.items()):
        raise ValueError('Source changed during audit')
    report = dict(schema='native_global_horizon_live_audit_v2', source_sha256=sources,
        installed_files=installed, executions=results,
        host=dict(platform=platform.platform(), cpu_count=os.cpu_count(), load_average=os.getloadavg()),
        limitations=[
            'Native global horizon=6, mixed extension OFF; unlike mixed3/6, first-three-second route/Top-K/confidence invariance is not assumed.',
            'Both native converters receive the same full recorded ROS TrackedObjects, with byte-level input nonmutation checked.',
            'Nearest prefix representative is a many-to-one geometric diagnostic, not a future-GT oracle, route identity or correct-path accuracy score.',
            'Live output uses exact stamp plus ordered UUID pairing; ambiguous inputs are excluded. Only publisher-owned outer header.seq is ignored.',
            'No future GT, detection AP or trajectory accuracy certification is inferred from a successful live drive.',
            'Different live runs have different native physics, spawn timing, object counts and CPU load; changes in braking/wait are descriptive, not isolated horizon causality.',
            'Read-only converter replay does not replay downstream planner/controller; actual planning behavior comes only from the distinct closed-loop runs.',
            'Stopped/brake time integrates short observed receipt intervals during controller-start to run-finish and reports missing coverage separately.',
            'Local od_gt plus corrected Ego localization is not E12 real-detector, competition LAN or 4090 full-stack certification.',
            'Recorded route plus Ego nearest-lanelet samples corroborate traversal but do not certify polygon occupancy or full-circle coverage.'])
    args.output.mkdir(parents=True)
    for name, expected in sources.items():
        destination = args.output/'source_snapshots'/Path(name).resolve().relative_to(ROOT)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(name, destination)
        if digest(destination) != expected:
            raise ValueError('Source changed while snapshotting')
    with gzip.open(args.output/'frames.json.gz', 'wt', encoding='utf-8') as stream:
        json.dump(frames, stream, ensure_ascii=False, allow_nan=False)
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,
        report_sha256=digest(args.output/'report.json'), frames_sha256=digest(args.output/'frames.json.gz')), indent=2)+'\n')


if __name__ == '__main__':
    main()
