#!/usr/bin/env python3
"""Audit closed validator bags and replay their full recorded ROS tracked inputs.

No ROS master, publishers, controls or simulator calls. This is a deterministic
same-input map-prediction qualification, not a full-stack real-time certificate.
Every output directory is fresh; source, installation and closed bag hashes are
checked again before evidence is completed.
"""
import argparse
from collections import Counter, defaultdict
from copy import copy
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import xml.etree.ElementTree as ET

import yaml

from qualify_e12_mixed_horizon import (
    ROOT, SOURCE, CORE_PATH, NODE_PATH, clone_network, compare_wire_prefix,
    digest, distribution, receipt_sources, wire,
)

TRACKS = '/perception/object_recognition/tracking/objects'
OUTPUT = '/perception/object_recognition/objects'
TIMING = '/map_based_prediction/diagnostics/processing_time_ms'
ROUTE = '/planning/mission_planning/route'
ODOM = '/localization/kinematic_state'
DEFAULT_EXECUTIONS = (
    'map_horizon_highway_baseline3_261024_261024_005413',
    'map_horizon_highway_mixed36_261024_261024_005612',
    'map_horizon_roundabout_baseline3_261024_261024_005752',
)
HISTORICAL_SOURCE_RECEIPT = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1/mixed_horizon_261024_v1/report.json'


def input_key(message):
    """Exact header time and ordered UUIDs; never nearest-time/position pairing."""
    return (message.header.stamp.to_nsec(),
            tuple(bytes(obj.object_id.uuid) for obj in message.objects))


def normalized_wire(message, prefix=False):
    """Read-only shallow view; only seq and optional suffix list are replaced.

    ROS serialization reads but does not mutate shared pose/covariance values.
    Cloning every Pose deeply adds substantial analysis cost without changing
    the bytes being compared. Current fields and original objects stay intact.
    """
    result = copy(message)
    result.header = copy(message.header)
    result.header.seq = 0
    if prefix:
        result.objects = []
        for original in message.objects:
            obj = copy(original)
            obj.kinematics = copy(original.kinematics)
            obj.kinematics.predicted_paths = []
            for original_path in original.kinematics.predicted_paths:
                if len(original_path.path) not in (31, 61):
                    raise ValueError('Unexpected truncated/padded prediction length')
                path = copy(original_path)
                path.path = original_path.path[:31]
                obj.kinematics.predicted_paths.append(path)
            result.objects.append(obj)
    return wire(result)


def path_lengths(message):
    counts = Counter()
    invalid_objects = 0
    for obj in message.objects:
        lengths = {len(path.path) for path in obj.kinematics.predicted_paths}
        counts.update(len(path.path) for path in obj.kinematics.predicted_paths)
        invalid_objects += not lengths or len(lengths) != 1 or not lengths.issubset({31, 61})
    return counts, int(invalid_objects)


def roundabout_internal_ids(map_file):
    """Read current map regulatory roles, not a historical hard-coded ID list."""
    result = set()
    for _, element in ET.iterparse(str(map_file), events=('end',)):
        if element.tag != 'relation':
            continue
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in element.findall('tag')}
        if tags.get('subtype') == 'roundabout':
            result.update(int(member.attrib['ref']) for member in element.findall('member')
                          if member.attrib.get('role') == 'internal')
        element.clear()
    return result


def receipt_check(execution, run):
    """Validate frozen execution files and retain explicit provenance omissions."""
    manifest = json.loads((execution/'execution.json').read_text())
    entries = {}
    for relative, expected in manifest.get('files', {}).items():
        path = execution/relative
        actual = digest(path) if path.is_file() else None
        entries[relative] = dict(expected=expected, actual=actual, matches=actual == expected)
    runtime_path = run/'runtime_receipt.json'
    runtime = json.loads(runtime_path.read_text()) if runtime_path.exists() else {}
    historical = {}
    if HISTORICAL_SOURCE_RECEIPT.exists():
        previous = json.loads(HISTORICAL_SOURCE_RECEIPT.read_text())
        matches = {name: Path(name).is_file() and digest(name) == expected
                   for name, expected in previous.get('source_sha256', {}).items()}
        historical = dict(path=str(HISTORICAL_SOURCE_RECEIPT),
            sha256=digest(HISTORICAL_SOURCE_RECEIPT), current_source_matches=matches,
            installed_files_at_qualification=previous.get('installed_files'),
            note='Contemporaneous offline qualification receipt; current hash equality corroborates shared source, not proof of which Python executable/module a live process loaded')
    probe_path = run/'route_probe_contract.json'
    probe = None
    if probe_path.exists():
        contents = json.loads(probe_path.read_text())
        probe = dict(path=str(probe_path), sha256=digest(probe_path), contents=contents,
            current_predictor_files_match={row['source']: (
                Path(row['source']).is_file() and Path(row['installed']).is_file()
                and digest(row['source']) == row['source_sha256']
                and digest(row['installed']) == row['installed_sha256']
                and row['source_sha256'] == row['installed_sha256'])
                for row in contents.get('source_installed', ())})
    return dict(execution_files=entries,
        execution_files_match=bool(entries) and all(row['matches'] for row in entries.values()),
        runtime_receipt_sha256=digest(runtime_path) if runtime_path.exists() else None,
        runtime_git_sha=runtime.get('git_sha'),
        runtime_tracked_modified_files=runtime.get('tracked_modified_files'),
        contemporaneous_source_receipt=historical, per_run_route_probe_contract=probe,
        live_predictor_source_hash_attested=False,
        provenance_limit='od_gt runtime/localization receipt does not directly attest predictor Python hashes; current source/install hashes alone are not a historical live source attestation')


def compare_recorded(recorded, computed):
    """Ambiguous repeated input keys are excluded rather than selected by output."""
    counts = Counter()
    mismatches = []
    for message in recorded:
        matches = computed.get(input_key(message), ())
        if len(matches) != 1:
            counts['ambiguous_input_key' if matches else 'unmatched_input_key'] += 1
            continue
        baseline_bytes, mixed_bytes = matches[0]
        counts['matched_unique_input_key'] += 1
        try:
            prefix = normalized_wire(message, prefix=True)
        except ValueError as error:
            counts['invalid_path_length'] += 1
            mismatches.append(dict(stamp_ns=message.header.stamp.to_nsec(), error=str(error)))
            continue
        counts['prefix_exact'] += prefix == baseline_bytes
        counts['prefix_mismatch'] += prefix != baseline_bytes
        counts['full_mixed_wire_exact'] += normalized_wire(message) == mixed_bytes
        if prefix != baseline_bytes and len(mismatches) < 20:
            mismatches.append(dict(stamp_ns=message.header.stamp.to_nsec(),
                error='Full serialized baseline prefix differs (only header.seq normalized)'))
    return dict(counts=dict(counts), mismatch_examples=mismatches,
        normalization='Only outer ROS header.seq; header.stamp, frame_id, ordered UUIDs, classification, state/covariance/shape, Top-K order, confidence, time_step and all first31 poses stay exact')


def audit_execution(execution, core, node):
    import rosbag
    from perception_object_pipeline.kinematic_prediction import (
        build_prediction_horizons, quaternion_to_yaw,
    )
    from perception_object_pipeline.mixed_horizon import MixedHorizonSelector

    bags = [bag for status in ('success', 'failure', 'failed')
            for bag in execution.glob(status+'/*/*.bag')]
    if len(bags) != 1:
        raise ValueError('Exactly one closed primary bag per execution is required: '+str(execution))
    bag_path, run = bags[0], bags[0].parent
    if Path(str(bag_path)+'.active').exists():
        raise ValueError('Refusing an active recorder')
    params_path = run/'ros_parameters.yaml'
    parameters = yaml.safe_load(params_path.read_text())['map_based_prediction']
    config = core.MapPredictionConfig(**{key: value for key, value in parameters.items()
        if key in core.MapPredictionConfig.__dataclass_fields__})
    map_file, projector_file = Path(parameters['map_file']), Path(parameters['projector_file'])
    stable_paths = [bag_path, params_path, map_file, projector_file,
                    execution/'execution.json', run/'runtime_receipt.json']
    stable_paths.extend(path for path in (HISTORICAL_SOURCE_RECEIPT, run/'route_probe_contract.json') if path.exists())
    original_hashes = {str(path): digest(path) for path in stable_paths}
    network = core.LaneletNetwork.load(map_file, projector_file, config)
    baseline_network, mixed_network = clone_network(network), clone_network(network)
    horizons = build_prediction_horizons(parameters['prediction_time_step'], parameters['prediction_horizon'])
    if len(horizons) != 30 or abs(horizons[-1]-3.) > 1.e-9:
        raise ValueError('This qualification requires the explicit 0.1s / 3s baseline')
    extension = node.MixedHorizonExtension(mixed_network, horizons,
        build_prediction_horizons(.1, 6.), MixedHorizonSelector(mixed_network))
    ring_ids = roundabout_internal_ids(map_file)
    counts, reasons, lengths = Counter(), Counter(), Counter()
    computed = defaultdict(list)
    recorded, diagnostics, routes, rows = [], [], [], []
    odom_ids = Counter()
    offline_timing = {'baseline3': [], 'mixed36': []}
    input_hash, prefix_hash, trimmed_hash = (hashlib.sha256() for _ in range(3))
    previous_stamp = None
    topics = [TRACKS, OUTPUT, TIMING, ROUTE, ODOM, '/CollisionData']
    with rosbag.Bag(str(bag_path)) as bag:
        topic_counts = {topic: info.message_count for topic, info in
                        bag.get_type_and_topic_info().topics.items()}
        bag_duration = bag.get_end_time()-bag.get_start_time()
        for topic, message, stamp in bag.read_messages(topics=topics):
            if topic == TRACKS:
                if message.header.frame_id != 'map':
                    raise ValueError('Recorded input frame is not map')
                original = wire(message)
                input_hash.update(original)
                stamp_ns = message.header.stamp.to_nsec()
                counts['backwards_input_stamp_steps'] += previous_stamp is not None and stamp_ns < previous_stamp
                previous_stamp = stamp_ns
                results, times = {}, {}
                index = counts['input_frames']
                for variant in (('baseline3', 'mixed36') if index % 2 == 0 else ('mixed36', 'baseline3')):
                    start, cpu = time.perf_counter(), time.thread_time()
                    results[variant] = node.make_map_based_predicted_objects(message, horizons,
                        baseline_network if variant == 'baseline3' else mixed_network,
                        mixed_extension=None if variant == 'baseline3' else extension)
                    times[variant] = dict(wall_ms=(time.perf_counter()-start)*1000,
                                         thread_cpu_ms=(time.thread_time()-cpu)*1000)
                    if index >= 30 and message.objects:
                        offline_timing[variant].append(times[variant])
                if wire(message) != original:
                    raise ValueError('Public predictor mutated full recorded ROS input')
                baseline, mixed = results['baseline3'][0], results['mixed36'][0]
                extended, first, second = compare_wire_prefix(baseline, mixed)
                if results['baseline3'][1:] != results['mixed36'][1:]:
                    raise ValueError('Mapped/fallback/path counts changed')
                prefix_hash.update(first)
                trimmed_hash.update(second)
                computed[input_key(message)].append((normalized_wire(baseline), normalized_wire(mixed)))
                reasons.update(extension.reasons)
                counts.update(input_frames=1, input_objects=len(message.objects),
                    mapped_objects=results['baseline3'][1], fallback_objects=results['baseline3'][2],
                    predicted_paths=results['baseline3'][3], extended_objects=extended,
                    optional_extension_errors=extension.extension_errors, exact_prefix_frames=1)
                rows.append(dict(stamp_ns=stamp_ns, bag_receipt_ns=stamp.to_nsec(),
                    objects=len(message.objects), extended=extended,
                    reasons=dict(extension.reasons), timing=times, exact_prefix=True))
            elif topic == OUTPUT:
                recorded.append(message)
                shape, invalid = path_lengths(message)
                lengths.update(shape)
                counts['live_partially_extended_or_invalid_objects'] += invalid
            elif topic == TIMING:
                diagnostics.append(json.loads(message.data))
            elif topic == ROUTE:
                ids = [int(segment.preferred_primitive.id) for segment in message.segments]
                routes.append(dict(stamp_ns=message.header.stamp.to_nsec(), preferred_lanelet_ids=ids,
                                   internal_ring_ids=sorted(set(ids) & ring_ids)))
            elif topic == ODOM:
                pose = message.pose.pose
                candidates = network.lane_candidates(pose.position.x, pose.position.y,
                    quaternion_to_yaw(pose.orientation))
                if candidates:
                    odom_ids[candidates[0].lanelet_id] += 1
            elif topic == '/CollisionData':
                counts['collision_messages_with_objects'] += bool(message.collision_object or message.collision_objecta)
    metadata = json.loads(next(run.glob('planbag*.json')).read_text())
    if any(digest(path) != expected for path, expected in original_hashes.items()):
        raise ValueError('Closed bag or map/parameters changed during audit')
    diagnostic_keys = ('callback_wall_ms_excluding_diagnostics',
        'callback_thread_cpu_ms_excluding_diagnostics', 'message_make_ms',
        'synchronous_publish_ms', 'input_header_age_ms', 'callback_start_gap_ms')
    result = dict(execution=str(execution), bag=str(bag_path), input_sha256=original_hashes,
        recorded_parameters=parameters, provenance=receipt_check(execution, run),
        input_frames_sha256=input_hash.hexdigest(), counts=dict(counts),
        deterministic_same_input=dict(exact_prefix=True, baseline_sha256=prefix_hash.hexdigest(),
            mixed_trimmed_sha256=trimmed_hash.hexdigest(), decisions=dict(reasons),
            timing_ms={variant: {key: distribution([row[key] for row in samples])
                for key in ('wall_ms', 'thread_cpu_ms')} for variant, samples in offline_timing.items()}),
        live_output=dict(frames=len(recorded), path_pose_counts=dict(lengths),
            compared_to_same_recorded_inputs=compare_recorded(recorded, computed)),
        live_callback=dict(samples=len(diagnostics), metrics={key: distribution([
            sample[key] for sample in diagnostics if sample.get(key) is not None]) for key in diagnostic_keys},
            extended_objects=sum(sample.get('mixed_extended_objects', 0) for sample in diagnostics),
            extension_errors=sum(sample.get('mixed_extension_errors', 0) for sample in diagnostics),
            mapped_objects=sum(sample.get('mapped', 0) for sample in diagnostics),
            fallback_objects=sum(sample.get('fallback', 0) for sample in diagnostics),
            predicted_paths=sum(sample.get('paths', 0) for sample in diagnostics),
            decisions=dict(sum((Counter(sample.get('mixed_horizon_reasons', {})) for sample in diagnostics), Counter()))),
        bag_topics=topic_counts, bag_duration_sec=bag_duration,
        route=dict(recorded_routes=routes, map_internal_ring_ids=sorted(ring_ids),
            ego_best_centerline_candidate_samples=dict(odom_ids),
            ego_internal_ring_best_candidate_samples=sum(n for identity, n in odom_ids.items() if identity in ring_ids),
            diagnostic_only='Pose/yaw centerline candidate, not polygon occupancy or continuous full-ring proof'),
        drive_result=dict(status=metadata.get('status'), reason=metadata.get('reason'),
            metrics=metadata.get('metrics'), safety=metadata.get('safety'), model=metadata.get('model')))
    return result, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execution', action='append', type=Path)
    parser.add_argument('--output', type=Path,
        default=ROOT/'planner_logs/analysis/map_mixed_horizon_261024_v1/live_bag_audit')
    args = parser.parse_args()
    args.output.resolve().relative_to((ROOT/'planner_logs/analysis').resolve())
    if args.output.exists():
        raise FileExistsError('Fresh output directory required')
    sys.path.insert(0, str(SOURCE))
    import perception_object_pipeline.map_based_prediction as core
    if Path(core.__file__).resolve() != CORE_PATH.resolve():
        raise ValueError('Explicit source core import was not honored')
    spec = importlib.util.spec_from_file_location('live_bag_actual_map_node', NODE_PATH)
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    sources, installed = receipt_sources()
    sources[str(Path(__file__).resolve())] = digest(Path(__file__))
    if not all(row['matches_source'] for row in installed.values()):
        raise ValueError('Source/install mismatch; audit only one known implementation')
    executions = args.execution or [ROOT/'planner_logs/validation'/name for name in DEFAULT_EXECUTIONS]
    results, rows = {}, {}
    for execution in executions:
        result, case_rows = audit_execution(execution.resolve(), core, node)
        results[execution.name], rows[execution.name] = result, case_rows
        print(json.dumps(dict(execution=execution.name, counts=result['counts'],
            live_comparison=result['live_output']['compared_to_same_recorded_inputs']['counts'],
            ring_samples=result['route']['ego_internal_ring_best_candidate_samples']), ensure_ascii=False), flush=True)
    if any(digest(path) != expected for path, expected in sources.items()):
        raise ValueError('Source changed during audit; use a fresh stable run')
    report = dict(schema='map_mixed_horizon_live_bag_audit_v1', source_sha256=sources,
        installed_files=installed, executions=results,
        host=dict(platform=platform.platform(), cpu_count=os.cpu_count(), load_average=os.getloadavg()),
        limitations=[
            'Local MORAI, od_gt detector and corrected Ego localization; not real E12 detection or competition LAN/4090 timing certification.',
            'Offline public converter uses identical full ROS messages in bag receipt order for both variants. Live subscriber queue drops/concurrent ordering are not replayed.',
            'Recorded output pairing uses exact stamp plus ordered UUIDs. Missing/ambiguous keys are excluded, never nearest-time matched.',
            'Only outer header.seq is ignored for live-wire comparison because rospy publisher rewrites it; every other byte and first31 path poses remain exact.',
            'Live full mixed-wire mismatch can reflect selector history when live queue drops inputs; prefix invariance and horizon histograms are reported separately.',
            'Offline timing excludes first30/empty inputs; live callback includes conversion/publish bookkeeping, not downstream delivery or age since last detection.',
            'Simultaneous MORAI CPU load and unequal live inputs mean paired drives are functional regressions, not a controlled end-to-end latency benchmark.',
            'Successful endpoint/stop/P does not prove ring coverage: actual preferred route and Ego map candidates are separately reported.',
            'Map-CV Top-K route representatives/confidence are preserved; this audit does not claim Top1 equals oracle min-K.'])
    args.output.mkdir(parents=True)
    for name, expected in sources.items():
        source = Path(name)
        destination = args.output/'source_snapshots'/source.resolve().relative_to(ROOT)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        if digest(destination) != expected:
            raise ValueError('Source changed while archiving audit implementation')
    with gzip.open(args.output/'frames.json.gz', 'wt', encoding='utf-8') as stream:
        json.dump(rows, stream, ensure_ascii=False, allow_nan=False)
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,
        report_sha256=digest(args.output/'report.json'),
        frames_sha256=digest(args.output/'frames.json.gz')), indent=2)+'\n')


if __name__ == '__main__':
    main()
