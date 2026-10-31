#!/usr/bin/env python3
"""Offline wire-prefix qualification of optional highway 6-second extension.

Uses the actual public ROS message conversion function, but no ROS master,
publisher, node initialization, simulator or controller. Frozen numeric core IDs
are reconstructed into stable synthetic UUIDs; this is not a claim to reproduce
unavailable original ROS covariance/UUID fields or live scheduling latency.
"""
import argparse
from collections import Counter
from copy import deepcopy
import gzip
import hashlib
import importlib.util
from io import BytesIO
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import uuid

import yaml

from analyze_e12_map_horizon import (
    ROOT, CORE_PATH, NODE_PATH, E12_SHA, CANONICAL, current_assignment,
    digest, future_reference, load, map_point, path_errors, prepare_case,
)
from audit_vehicle_map_prediction import distribution

A = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'
OUTPUT = A/'mixed_horizon_261024_v1'
SOURCE = ROOT/'base_ws/src/perception_object_pipeline/src'
SELECTOR_PATH = SOURCE/'perception_object_pipeline/mixed_horizon.py'
PACK_SHA = 'f1caea8bd7de57235ea4d6285989a8f480284e63b19b18301919c0ecb797bebf'


def wire(message):
    stream = BytesIO()
    message.serialize(stream)
    return stream.getvalue()


def trim_prefix(message, count=31):
    """Only remove future suffixes, retaining every non-path field verbatim."""
    result = deepcopy(message)
    for obj in result.objects:
        for path in obj.kinematics.predicted_paths:
            if len(path.path) not in (count, 61):
                raise ValueError('Unexpected truncated/padded prediction length')
            path.path = path.path[:count]
    return result


def compare_wire_prefix(baseline, mixed, baseline_pose_count=31):
    if len(baseline.objects) != len(mixed.objects):
        raise ValueError('Object count changed')
    extended = 0
    for first, second in zip(baseline.objects, mixed.objects):
        paths = first.kinematics.predicted_paths
        changed = second.kinematics.predicted_paths
        if len(paths) != len(changed):
            raise ValueError('Top-K path count changed')
        if any(len(path.path) != baseline_pose_count for path in paths):
            raise ValueError('Baseline does not contain complete 3-second paths')
        lengths = {len(path.path) for path in changed}
        if not lengths.issubset({baseline_pose_count, 61}) or len(lengths) != 1:
            raise ValueError('Partially extended object or invalid output length')
        extended += lengths == {61}
    original, trimmed = wire(baseline), wire(trim_prefix(mixed, baseline_pose_count))
    if original != trimmed:
        raise ValueError('Serialized prefix/current state/header/UUID/Top-K/confidence differs')
    return int(extended), original, trimmed


def synthetic_uuid(case_name, identity):
    return list(uuid.uuid5(uuid.NAMESPACE_URL, 'e12-frozen-core:'+case_name+':'+str(identity)).bytes)


def build_message(case_name, index, raw_frame, vehicle_frame, transform):
    """Frozen source ordering, production vehicle yaw adapter, UNKNOWN stationary."""
    import rospy
    from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects
    from perception_object_pipeline.kinematic_prediction import yaw_to_quaternion

    message = TrackedObjects()
    message.header.frame_id = 'map'
    message.header.seq = int(raw_frame.get('native_index', index))
    message.header.stamp = rospy.Time.from_sec(raw_frame['stamp'])
    vehicles = {str(obj['id']): obj for obj in vehicle_frame['objects']}
    for raw in raw_frame['tracks']:
        if not raw['confirmed']:
            continue
        core_label = int(raw['tracker_label'])
        # mc_mot's PEDESTRIAN enum is 3, while ROS Autoware's is 7 (3=BUS).
        label = {0: ObjectClassification.UNKNOWN, 1: ObjectClassification.CAR,
                 2: ObjectClassification.TRUCK, 3: ObjectClassification.PEDESTRIAN}[core_label]
        if core_label in (1, 2):
            source = vehicles[str(raw['id'])]
            center, yaw = source['center'], source['yaw']
            body_velocity, yaw_rate = source['body_velocity'], source['yaw_rate']
        else:
            center = map_point(raw['center'], transform)
            yaw = float(raw['yaw'])+transform['rpy'][2]
            vx, vy = raw['velocity']
            body_velocity = [math.cos(raw['yaw'])*vx+math.sin(raw['yaw'])*vy,
                             -math.sin(raw['yaw'])*vx+math.cos(raw['yaw'])*vy]
            yaw_rate = float(raw['yaw_rate'])
            if label == 0:  # production UNKNOWN stationary output policy
                body_velocity, yaw_rate = [0., 0.], 0.
        tracked = TrackedObject()
        tracked.object_id.uuid = synthetic_uuid(case_name, raw['id'])
        tracked.classification = [ObjectClassification(label=label, probability=1.)]
        tracked.existence_probability = float(raw['score'])
        pose = tracked.kinematics.pose_with_covariance.pose
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation = yaw_to_quaternion(yaw)
        twist = tracked.kinematics.twist_with_covariance.twist
        twist.linear.x, twist.linear.y = body_velocity
        twist.angular.z = yaw_rate
        tracked.shape.type = 0  # BOUNDING_BOX
        tracked.shape.dimensions.x, tracked.shape.dimensions.y, tracked.shape.dimensions.z = raw['size']
        message.objects.append(tracked)
    return message


def clone_network(network):
    return type(network)(tuple(network.records.values()), network.config,
                         map_sha256=network.map_sha256, lanelet_layer=network._lanelet_layer)


def qualify_case(case_name, case, transform, loaded, node):
    from perception_object_pipeline.mixed_horizon import MixedHorizonSelector
    from perception_object_pipeline.kinematic_prediction import build_prediction_horizons

    baseline_network, mixed_network = clone_network(loaded), clone_network(loaded)
    baseline_horizons = build_prediction_horizons(.1, 3.)
    extension = node.MixedHorizonExtension(mixed_network, baseline_horizons,
        build_prediction_horizons(.1, 6.), MixedHorizonSelector(mixed_network))
    frames, trajectories = prepare_case(case, transform)
    counts, reasons, gt_exclusions = Counter(), Counter(), Counter()
    baseline_times, mixed_times, rows, gt_rows = [], [], [], []
    baseline_hash, trimmed_hash, input_hash = hashlib.sha256(), hashlib.sha256(), hashlib.sha256()
    for index, (raw, frame) in enumerate(zip(case['frames'], frames)):
        message = build_message(case_name, index, raw, frame, transform)
        before = wire(message)
        input_hash.update(before)
        values, elapsed = {}, {}
        for variant in (('baseline', 'mixed') if index % 2 == 0 else ('mixed', 'baseline')):
            started = time.perf_counter()
            values[variant] = node.make_map_based_predicted_objects(message, baseline_horizons,
                baseline_network if variant == 'baseline' else mixed_network,
                mixed_extension=None if variant == 'baseline' else extension)
            elapsed[variant] = (time.perf_counter()-started)*1000.
        if wire(message) != before:
            raise ValueError('Predictor mutated its input TrackedObjects')
        baseline, mapped, fallback, paths = values['baseline']
        mixed, mapped_mixed, fallback_mixed, paths_mixed = values['mixed']
        if (mapped, fallback, paths) != (mapped_mixed, fallback_mixed, paths_mixed):
            raise ValueError('Mapped/fallback classification or Top-K count changed')
        extended, first, second = compare_wire_prefix(baseline, mixed)
        if extended != extension.extended_objects:
            raise ValueError('Published extension and diagnostic counts differ')
        baseline_hash.update(first)
        trimmed_hash.update(second)
        reasons.update(extension.reasons)
        counts.update(dict(all_input_frames=1, scoring_frames=int(raw['scoring_eligible']),
            input_objects=len(message.objects), mapped_objects=mapped,
            kinematic_fallback_objects=fallback, paths=paths,
            extended_objects=extended, exact_prefix_frames=1,
            optional_extension_errors=extension.extension_errors))
        if index >= 30 and message.objects:
            baseline_times.append(elapsed['baseline'])
            mixed_times.append(elapsed['mixed'])
        rows.append(dict(case=case_name, stamp=frame['stamp'], input_objects=len(message.objects),
            extended_objects=extended, decision_reasons=dict(extension.reasons),
            timing_ms=elapsed, exact_prefix=True))
        if not raw['scoring_eligible']:
            continue
        # Freeze current assignment before querying any future reference.
        pairs = current_assignment(frame['objects'], frame['gt'])
        output_by_id = {bytes(obj.object_id.uuid): obj for obj in mixed.objects}
        for oi, gi, current_error in pairs:
            obj, truth = frame['objects'][oi], frame['gt'][gi]
            output = output_by_id[bytes(synthetic_uuid(case_name, obj['id']))]
            published_paths = output.kinematics.predicted_paths
            if any(len(path.path) != 61 for path in published_paths):
                gt_exclusions['not_extended'] += 1
                continue
            series = trajectories.get(truth['key'])
            if series is None:
                gt_exclusions['matched_gt_without_trajectory'] += 1
                continue
            reference, reason = future_reference(series, frame['stamp'], 6)
            if reason is not None:
                gt_exclusions[reason] += 1
                continue
            paths_xy = [dict(xy=[[p.position.x, p.position.y] for p in path.path[1:]])
                        for path in published_paths]
            gt_rows.append(dict(case=case_name, stamp=frame['stamp'], id=obj['id'],
                gt_key=truth['key'], current_xy_error_m=current_error,
                **path_errors(paths_xy, reference, 6)))
    summary = dict(counts=dict(counts), decision_reasons=dict(reasons),
        serialized_prefix_exact=baseline_hash.hexdigest() == trimmed_hash.hexdigest(),
        serialized_baseline_sha256=baseline_hash.hexdigest(),
        serialized_mixed_trimmed_sha256=trimmed_hash.hexdigest(),
        reconstructed_input_sha256=input_hash.hexdigest(),
        timing_ms=dict(baseline3=distribution(baseline_times), mixed36=distribution(mixed_times)),
        six_second_gt=dict(eligible_extended_matched_origins=len(gt_rows), exclusions=dict(gt_exclusions),
            **{key: distribution([row[key] for row in gt_rows])
               for key in ('top1_ADE_m', 'top1_FDE_m', 'minK_ADE_m', 'minK_FDE_m')}))
    return summary, rows, gt_rows


def receipt_sources():
    paths = [CORE_PATH, NODE_PATH, SELECTOR_PATH, Path(__file__),
             Path(__file__).with_name('analyze_e12_map_horizon.py'),
             Path(__file__).with_name('audit_vehicle_map_prediction.py'),
             SOURCE/'perception_object_pipeline/kinematic_prediction.py']
    hashes = {str(path): digest(path) for path in paths}
    installed = ROOT/'base_ws/install_aw149'
    installation = {}
    for source in (CORE_PATH, SELECTOR_PATH):
        target = installed/'lib/python3/dist-packages/perception_object_pipeline'/source.name
        installation[str(target)] = dict(sha256=digest(target) if target.is_file() else None,
            matches_source=target.is_file() and digest(target) == hashes[str(source)])
    target = installed/'lib/perception_object_pipeline/map_based_prediction_node.py'
    installation[str(target)] = dict(sha256=digest(target) if target.is_file() else None,
        matches_source=target.is_file() and digest(target) == hashes[str(NODE_PATH)])
    return hashes, installation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack', type=Path, default=A/'tracker_v1/replay_pack.json.gz')
    parser.add_argument('--parameters', type=Path,
        default=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml')
    parser.add_argument('--transform', type=Path, default=ROOT/'configs/tf/morai_lanelet_static_tf.yaml')
    parser.add_argument('--map-file', type=Path, default=ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm')
    parser.add_argument('--projector-file', type=Path,
        default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    args.output.resolve().relative_to(A.resolve())
    if args.output.exists():
        raise FileExistsError('Fresh output required; do not overwrite evidence')
    pack = load(args.pack)
    if (digest(args.pack) != PACK_SHA or not pack.get('complete')
        or pack.get('schema') != 'e12_postprocessing_core_pack_v1'
        or any(c['provenance']['recorded_model']['checkpoint_sha256'] != E12_SHA
               for c in pack['cases'].values())):
        raise ValueError('Exact completed frozen E12 pack is required')
    sys.path.insert(0, str(SOURCE))
    import perception_object_pipeline.map_based_prediction as core
    if Path(core.__file__).resolve() != CORE_PATH.resolve():
        raise ValueError('Source core was not imported explicitly')
    spec = importlib.util.spec_from_file_location('qualification_current_map_node', NODE_PATH)
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    source_hashes, installed = receipt_sources()
    other_inputs = {str(path): digest(path) for path in
        (args.pack, args.parameters, args.transform, args.map_file, args.projector_file)}
    parameters = yaml.safe_load(args.parameters.read_text())
    config = core.MapPredictionConfig(**{key: value for key, value in parameters.items()
        if key in core.MapPredictionConfig.__dataclass_fields__})
    transform = yaml.safe_load(args.transform.read_text())['map_to_morai_map']
    loaded = core.LaneletNetwork.load(args.map_file, args.projector_file, config)
    report = dict(schema='e12_mixed_horizon_qualification_v1', production_changed=False,
        map_sha256=loaded.map_sha256, pack_sha256=digest(args.pack),
        source_sha256=source_hashes, installed_files=installed, input_sha256=other_inputs,
        canonicalization=CANONICAL, cases={},
        protocol=dict(input='All 14 frozen cases in stored frame and confirmed object order; stable per-case synthetic UUIDs; original numeric core IDs retained only as adapter keys',
            fields='Pose/yaw/body motion/shape/score reconstructed; unavailable original covariance fields remain default zero. UNKNOWN stationary policy applied. Same resulting TrackedObjects passed to both variants.',
            comparison='Actual public make_map_based_predicted_objects; baseline3 vs mixed3/6; trim only suffix after pose31 and require complete serialized bytes identical; input must not mutate',
            state='Fresh selector and networks per case; no future GT used for selection or horizon decision',
            timing='CPU Python conversion+map math, normal GC, alternate variant execution order, first30 frames and empty inputs excluded; not ROS latency/real-time/fullstack certification',
            six_second_gt='Current vehicle XY2m one-to-one match before future lookup; scoring eligible origins and adjacent native-index GT only; independently best minADE-K/minFDE-K are oracle evaluation, not runtime route selection'),
        host=dict(platform=platform.platform(), cpu_count=os.cpu_count(), load_average=os.getloadavg()),
        limitations=['Not live ROS30Hz, detector TRT performance, network age, or planner safety evidence.',
            'Accepted start acceleration periods remain in diagnostic GT metrics; this does not propose modeling acceleration.',
            'Missing or ambiguous highway/route extension correctly retains entire original3s output.'])
    all_rows, all_gt = [], []
    for name, case in pack['cases'].items():
        result, rows, gt_rows = qualify_case(name, case, transform, loaded, node)
        report['cases'][name] = result
        all_rows.extend(rows)
        all_gt.extend(gt_rows)
        print(json.dumps(dict(case=name, **result['counts']), sort_keys=True), flush=True)
    if any(digest(path) != value for path, value in {**source_hashes, **other_inputs}.items()):
        raise ValueError('Source/input changed during qualification; use a fresh stable run')
    report['aggregate'] = dict(counts=dict(sum((Counter(c['counts']) for c in report['cases'].values()), Counter())),
        decision_reasons=dict(sum((Counter(c['decision_reasons']) for c in report['cases'].values()), Counter())),
        serialized_prefix_exact=all(c['serialized_prefix_exact'] for c in report['cases'].values()),
        timing_ms={variant: distribution([row['timing_ms'][variant] for name, case in pack['cases'].items()
            for row in [r for r in all_rows if r['case'] == name][30:] if row['input_objects']])
            for variant in ('baseline', 'mixed')},
        six_second_gt=dict(origins=len(all_gt), **{key: distribution([row[key] for row in all_gt])
            for key in ('top1_ADE_m', 'top1_FDE_m', 'minK_ADE_m', 'minK_FDE_m')}))
    args.output.mkdir(parents=True)
    for name, value in (('frames.json.gz', all_rows), ('six_second_gt.json.gz', all_gt)):
        with gzip.open(args.output/name, 'wt', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                                   allow_nan=False)+'\n', encoding='utf-8')
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,
        report_sha256=digest(args.output/'report.json'),
        frames_sha256=digest(args.output/'frames.json.gz'),
        gt_rows_sha256=digest(args.output/'six_second_gt.json.gz')),
        indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report['aggregate'], indent=2, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
