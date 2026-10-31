#!/usr/bin/env python3
"""Compare frozen/native tracker policies with saved observations, entirely offline.

No ROS master, node, simulator, installation, production parameter write or GT
feedback. Full-route outputs are descriptive consistency measurements, not
AP/MOTA. Numeric-ID comparisons and geometric correspondence are both reported;
neither is an identity oracle. Existing outputs are never overwritten.
"""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
from copy import deepcopy
import gzip
import io
import json
import math
from pathlib import Path
import pickle
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import time

import numpy as np
import yaml

import audit_tracking_continuity_events as events

ROOT = Path(__file__).resolve().parents[2]
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'
BASELINE_REF = 'ea1429e13c7c99944607ad0aea18dc7ce14a1522'
ARTIFACTS = ROOT / 'planner_logs/analysis/261024_tracker_consistency_v1'
OLD = ROOT / 'planner_logs/analysis/261024_035142_full_route/duplicate_rootcause'
DEFAULT_CACHE = OLD / 'world_cache.pkl'
WINDOWS = {
    'obstacle_0134': (29.6, 31.4, [71., 403.], 8.),
    'truck_0156': (117.25, 118.25, [202., 643.], 8.),
}
NESTED_CONFIG = {
    'class_transition_guard_config.enabled': bool,
    'class_transition_guard_config.required_consecutive': int,
    'class_transition_guard_config.maximum_gap_sec': float,
    'class_transition_guard_config.shape_ratio': float,
    'class_transition_guard_config.minimum_dimension_m': float,
}


def write_json(path, value):
    with Path(path).open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def distribution(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'count': 0}
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite diagnostic measurement')
    return dict(count=len(a), mean=float(a.mean()), minimum=float(a.min()),
                p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def class_name(label):
    return 'vehicle' if label in (1, 2) else 'pedestrian' if label in (3, 4) else 'obstacle'


def freeze_tracker(output, source=TRACKER, revision=None):
    """Create a fresh, self-contained source snapshot and record its provenance."""
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    head = subprocess.check_output(['git', '-C', str(source), 'rev-parse', revision or 'HEAD'], text=True).strip()
    if revision:
        archive = subprocess.check_output(['git', '-C', str(source), 'archive', head])
        with tarfile.open(fileobj=io.BytesIO(archive)) as handle:
            for member in handle.getmembers():
                if member.name.startswith('/') or '..' in Path(member.name).parts:
                    raise ValueError('Unsafe archive member')
            handle.extractall(output)
        dirty = None
    else:
        # Never link against a working file that another task may be updating.
        for name in ('include', 'src', 'config'):
            shutil.copytree(source / name, output / name)
        dirty = subprocess.check_output(['git', '-C', str(source), 'diff', '--no-ext-diff', 'HEAD'], text=True)
    files = {str(p.relative_to(output)): events.sha(p) for p in sorted(output.rglob('*')) if p.is_file()}
    receipt = dict(source=str(source), git_ref=head, from_git_archive=bool(revision),
                   working_tree_diff=dirty, files_sha256=files)
    write_json(output / 'snapshot_receipt.json', receipt)
    return receipt


def operational_parameters(snapshot):
    params = yaml.safe_load((snapshot / 'config/config.yaml').read_text())['configure']
    params.update(pedestrian_gaussian_association_ranking=True,
                  time_aware_track_lifecycle=True, confirmed_track_max_coast_time_sec=.5,
                  confirmed_stationary_track_max_coast_time_sec=1., suppress_duplicate_track_birth=True)
    return params


def nested_assignments(overrides):
    lines = []
    for name, expected in NESTED_CONFIG.items():
        if name not in overrides:
            continue
        value = overrides[name]
        if (expected is bool and type(value) is not bool or
                expected is int and type(value) is not int or
                expected is float and (isinstance(value, bool) or not isinstance(value, (int, float)))):
            raise ValueError('Invalid nested configuration type: '+name)
        if not isinstance(value, bool) and not math.isfinite(value):
            raise ValueError('Nonfinite native configuration value')
        literal = str(value).lower() if isinstance(value, bool) else str(value)
        lines.append('  config.{} = {};'.format(name, literal))
    return lines


@contextmanager
def linked_tracker(snapshot, assignments=()):
    sys.path.insert(0, str(ROOT / 'perception/detection3d/bevfusion/tools'))
    import build_base5_low_score_probe as builder
    original, harness = builder.TRACKER, builder.HARNESS
    builder.TRACKER = snapshot
    if assignments:
        anchor = '  auto tracker ='
        if harness.count(anchor) != 1:
            raise ValueError('Pinned native harness anchor changed')
        builder.HARNESS = harness.replace(anchor, '\n'.join(assignments)+'\n'+anchor, 1)
    try:
        yield
    finally:
        builder.TRACKER = original
        builder.HARNESS = harness


def build(snapshot, output, overrides):
    params = operational_parameters(snapshot)
    header = (snapshot / 'include/algorithm/ekf_multi_object_tracking.hpp').read_text()
    fields = set(re.findall(r'^\s+(?:bool|double|int)\s+(\w+)\s*(?:\{|=)', header, re.M))
    unknown = set(overrides) - fields - NESTED_CONFIG.keys()
    if unknown:
        raise ValueError('Unknown explicit native configuration fields: ' + ', '.join(sorted(unknown)))
    if any(not isinstance(v, (bool, int, float)) for v in overrides.values()):
        raise ValueError('Only scalar native configuration overrides are allowed')
    assignments = nested_assignments(overrides)
    params.update({k: v for k, v in overrides.items() if k not in NESTED_CONFIG})
    policy = {'gaussian_xy_ranking': 'pedestrian'} if params.get('pedestrian_gaussian_association_ranking') else {}
    with linked_tracker(snapshot, assignments):
        binary, receipt = events.build_original(output, params, policy=policy, extended_state=True)
    receipt['explicit_nested_assignments'] = assignments
    write_json(output / 'qualification_settings.json', dict(parameters=params, explicit_overrides=overrides,
               note='Saved-operational wrapper lifecycle/pedestrian defaults, isolated native harness only.'))
    return binary, receipt


def lifecycle_summary(rows):
    seen, previous, confirmed = {}, {}, set()
    births, confirmations, flips = [], [], []
    speed, jumps, ages = [], [], []
    counts = Counter()
    for frame in rows:
        now = {t['id']: t for t in frame['tracks']}
        for identity, track in now.items():
            stamp, cls = frame['stamp'], class_name(track['core_class'])
            counts[cls + '/all_samples'] += 1
            counts[cls + '/confirmed_samples'] += bool(track['confirmed'])
            counts[cls + '/associated_samples'] += bool(track['associated'])
            if identity not in seen:
                seen[identity] = stamp
                births.append(dict(stamp=stamp, id=identity, core_class=track['core_class'], center=track['center']))
            if track['confirmed'] and identity not in confirmed:
                confirmed.add(identity)
                confirmations.append(dict(stamp=stamp, id=identity, core_class=track['core_class'],
                    since_first_native_sample_sec=stamp-seen[identity], age=track['age']))
            if identity in previous:
                old = previous[identity]
                jumps.append(math.dist(old['center'][:2], track['center'][:2]))
                if old['core_class'] != track['core_class']:
                    flips.append(dict(stamp=stamp, id=identity, before=old['core_class'], after=track['core_class']))
            speed.append(math.hypot(*track['velocity']))
            ages.append(stamp-track['last_measurement_time'])
        previous = now
    return dict(core=events.core_summary(rows), counts=dict(counts), total_births=len(births),
        total_confirmations=len(confirmations), representative_class_flip_count=len(flips),
        confirmation_delay_sec=distribution([r['since_first_native_sample_sec'] for r in confirmations]),
        native_speed_mps=distribution(speed), inter_observation_center_step_m=distribution(jumps),
        last_measurement_age_sec=distribution(ages), births=births, confirmations=confirmations, class_flips=flips)


def greedy_geometry(first, second, gate=2.):
    """Class-independent nearest XY diagnostic correspondence, never identity GT."""
    edges = sorted((math.dist(a['center'][:2], b['center'][:2]), i, j)
                   for i, a in enumerate(first) for j, b in enumerate(second))
    used_a, used_b, pairs = set(), set(), []
    for distance, i, j in edges:
        if distance <= gate and i not in used_a and j not in used_b:
            pairs.append((first[i], second[j], distance))
            used_a.add(i)
            used_b.add(j)
    return pairs, len(first)-len(used_a), len(second)-len(used_b)


def compare_rows(baseline, candidate):
    if [r['stamp'] for r in baseline] != [r['stamp'] for r in candidate]:
        raise ValueError('Replay stamp/count mismatch')
    counts, centers, velocities, speeds = Counter(), [], [], []
    changed = []
    for index, (one, two) in enumerate(zip(baseline, candidate)):
        if one != two:
            changed.append(dict(index=index, stamp=one['stamp']))
        a, b = ({t['id']: t for t in r['tracks']} for r in (one, two))
        counts['frames_with_native_id_set_change'] += a.keys() != b.keys()
        counts['only_baseline_native_id_samples'] += len(a.keys()-b.keys())
        counts['only_candidate_native_id_samples'] += len(b.keys()-a.keys())
        for identity in a.keys() & b.keys():
            counts['same_numeric_id_class_changes'] += a[identity]['core_class'] != b[identity]['core_class']
            counts['same_numeric_id_confirmation_changes'] += a[identity]['confirmed'] != b[identity]['confirmed']
        pairs, missing_a, missing_b = greedy_geometry(
            [t for t in one['tracks'] if t['confirmed']], [t for t in two['tracks'] if t['confirmed']])
        counts['geometric_pairs'] += len(pairs)
        counts['geometrically_unmatched_baseline_confirmed_samples'] += missing_a
        counts['geometrically_unmatched_candidate_confirmed_samples'] += missing_b
        for a, b, distance in pairs:
            centers.append(distance)
            velocities.append(math.dist(a['velocity'], b['velocity']))
            speeds.append(abs(math.hypot(*a['velocity'])-math.hypot(*b['velocity'])))
            counts['geometric_pair_class_changes'] += a['core_class'] != b['core_class']
            counts['geometric_pair_numeric_id_changes'] += a['id'] != b['id']
    return dict(exact_native_state_and_prior_covariance=not changed, frames=len(baseline),
        changed_frames=len(changed), first_changed_frames=changed[:20], counts=dict(counts),
        geometric_center_delta_m=distribution(centers), geometric_velocity_delta_mps=distribution(velocities),
        geometric_speed_delta_mps=distribution(speeds),
        correspondence='Confirmed tracks: class-independent XY-nearest greedy one-to-one <=2m. Not physical identity ground truth; numeric IDs may shift after changed births.')


def parse_variant(value):
    name, separator, payload = value.partition('=')
    if not separator or not name.replace('_', '').isalnum() or name in ('baseline', 'candidate', 'policy_off'):
        raise argparse.ArgumentTypeError('Expected unique NAME=JSON, excluding baseline/candidate/policy_off')
    try:
        params = json.loads(payload)
    except json.JSONDecodeError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if not isinstance(params, dict):
        raise argparse.ArgumentTypeError('Variant overrides must be a JSON object')
    return name, params


def episode_summary(rows, start, window):
    lo, hi, center, radius = window
    selected = [dict(stamp=f['stamp'], t=f['stamp']-start,
                    tracks=[t for t in f['tracks'] if math.dist(t['center'][:2], center) < radius])
                for f in rows if lo <= f['stamp']-start <= hi]
    tracks = [t for f in selected for t in f['tracks'] if t['confirmed']]
    return dict(window_seconds_from_bag_start=[lo, hi], center_xy=center, radius_m=radius,
        unique_native_ids=sorted({t['id'] for t in tracks}), confirmed_samples=len(tracks),
        simultaneous_confirmed_count=distribution([sum(t['confirmed'] for t in f['tracks']) for f in selected]),
        rows=selected, caveat='Spatial-window co-presence is descriptive; no physical duplicate or FP oracle.')


def replay_timed(binary, frames, repeats=1):
    rows, times = None, []
    for _ in range(repeats):
        started = time.perf_counter()
        current = events.replay(binary, frames)
        times.append((time.perf_counter()-started)*1000.)
        if rows is not None and current != rows:
            raise ValueError('Fresh-core repeat is not exact')
        rows = current
    return rows, dict(batch_wall_ms=distribution(times),
        amortized_wall_ms_per_frame=distribution([v/len(rows) for v in times]), repeats=repeats,
        scope='Offline process launch + Python serialization + native prediction/update + diagnostic stdout/parse; not callback CPU or live 30Hz latency.')


def timing_harness(text, settings):
    """No trace formatting/IO within the measured input loop; final-state check."""
    text = text.replace('#include <memory>', '#include <memory>\n#include <chrono>\n#include <vector>')
    anchor = '  auto tracker ='
    if text.count(anchor) != 1:
        raise ValueError('Native timing configuration anchor changed')
    text = text.replace(anchor, '\n'.join(settings)+'\n'+anchor, 1)
    text = text.replace('  double time = 0, previous = 0;',
        '  std::vector<std::pair<double,double>> timing;\n  double time = 0, previous = 0;', 1)
    anchor = '    if (previous) tracker->RunPrediction(time - previous);\n    tracker->RunUpdate(frame);'
    if text.count(anchor) != 1:
        raise ValueError('Native timing update anchor changed')
    text = text.replace(anchor, '''    const auto predict_start = std::chrono::steady_clock::now();
    if (previous) tracker->RunPrediction(time - previous);
    const auto update_start = std::chrono::steady_clock::now();
    tracker->RunUpdate(frame);
    const auto update_end = std::chrono::steady_clock::now();
    timing.emplace_back(std::chrono::duration<double,std::milli>(update_start-predict_start).count(),
                        std::chrono::duration<double,std::milli>(update_end-update_start).count());''', 1)
    start = text.index('    const auto result = tracker->GetTrackResults();')
    end = text.index('    previous = time;', start)
    final_state = text[start:end].replace('"FRAME " << time', '"FRAME " << previous')
    text = text[:start]+text[end:]
    anchor = '    previous = time;\n  }\n}'
    if text.count(anchor) != 1:
        raise ValueError('Native timing loop anchor changed')
    text = text.replace(anchor, '''    previous = time;
  }
  for (size_t i=0; i<timing.size(); ++i)
    std::cout << "BENCH " << i << " " << timing[i].first << " " << timing[i].second << "\\n";
'''+final_state+'}', 1)
    return text


def build_timing(build_directory, receipt, output):
    sys.path.insert(0, str(ROOT/'perception/detection3d/bevfusion/tools'))
    import build_base5_low_score_probe as builder
    output.mkdir()
    harness = output/'no_io_timing_harness.cpp'
    harness.write_text(timing_harness(builder.HARNESS, receipt['settings']+receipt.get('explicit_nested_assignments', [])))
    source = build_directory/'candidate_core.cpp'
    if not source.exists():
        source = next(Path(p) for p in receipt['production_source_sha256'] if p.endswith('.cpp'))
    header = next(Path(p) for p in receipt['production_source_sha256'] if p.endswith('.hpp'))
    binary = output/'no_io_timing_core'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    command = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I'+str(header.parents[1]),
               str(harness), str(source), '-o', str(binary), *flags]
    subprocess.run(command, check=True)
    write_json(output/'build_receipt.json', dict(command=command, harness_sha256=events.sha(harness),
        linked_source_sha256=events.sha(source), header_sha256=events.sha(header), binary_sha256=events.sha(binary),
        settings=receipt['settings'], nested_assignments=receipt.get('explicit_nested_assignments', [])))
    return binary


def benchmark_native(binary, frames, expected_last, repeats):
    lines = []
    for frame in frames:
        if frame['world_objects'] is None:
            continue
        lines.append('{} {}'.format(format(frame['stamp'], '.17g'), len(frame['world_objects'])))
        for obj in frame['world_objects']:
            label = ('vehicle', 'pedestrian', 'obstacle').index(obj['class_name'])
            lines.append(' '.join(format(v, '.17g') for v in (label, *obj['center'], obj['yaw'], *obj['size'], obj['score'])))
    payload = '\n'.join(lines)+'\n'
    predictions, updates, sums = [], [], []
    for _ in range(repeats):
        result = subprocess.run([str(binary.resolve())], input=payload, text=True, capture_output=True, check=True, timeout=240)
        times = [list(map(float, s.split()[2:])) for s in result.stdout.splitlines() if s.startswith('BENCH ')]
        if len(times) != len([f for f in frames if f['world_objects'] is not None]):
            raise ValueError('Incomplete native no-IO timing run')
        for prediction, update in times[30:]:
            predictions.append(prediction)
            updates.append(update)
            sums.append(prediction+update)
        # The final normal TRACK record (only after all timings) independently
        # checks the no-IO harness configuration and state against traced replay.
        actual = [s.split()[1:] for s in result.stdout.splitlines() if s.startswith('TRACK ')]
        wanted = [[t['id'], t['core_class'], int(t['confirmed']), int(t['associated']), *t['center'],
                   *t['velocity'], t['yaw'], *t['size'], t['score'], t['last_measurement_time'], t['age']]
                  for t in expected_last['tracks']]
        if [[float(v) for v in row] for row in actual] != [[float(v) for v in row] for row in wanted]:
            raise ValueError('Native no-IO timing final state differs from full traced replay')
    return dict(prediction_ms=distribution(predictions), update_ms=distribution(updates),
        prediction_plus_update_ms=distribution(sums), repeats=repeats, warmup_excluded_per_repeat=30,
        final_primary_state_exact=True,
        scope='steady_clock around native RunPrediction/RunUpdate only; input parse and all diagnostic formatting/IO outside timing. Fresh sequential native process, not live callback/ROS scheduling latency.')


def load_pack(path):
    with gzip.open(path, 'rt') as handle:
        pack = json.load(handle)
    if pack.get('schema') != 'e12_postprocessing_core_pack_v1' or not pack.get('complete'):
        raise ValueError('Complete frozen E12/E24 core pack required')
    return pack


def pack_track(track):
    """Lossless schema adapter for the established catalog evaluation tool."""
    label = ('vehicle', 'pedestrian', 'obstacle').index(class_name(track['core_class']))
    return dict(id=track['id'], tracker_label=track['core_class'], label=label, class_name=class_name(track['core_class']),
        confirmed=track['confirmed'], associated=track['associated'], center=track['center'], yaw=track['yaw'],
        velocity=track['velocity'], yaw_rate=track['yaw_rate'], size=track['size'], score=track['score'],
        age_frames=track['age'], last_measurement_stamp=track['last_measurement_time'], birth_stamp=track['initialization_time'])


def gt_consistency(rows, raw_frames):
    """Eligible frozen-case XY2m correspondence; deliberately not AP/MOTA."""
    counts, xy, velocity = Counter(), [], []
    old_gt, old_match, previous_index, previous_stamp = {}, {}, None, None
    for row, frame in zip(rows, raw_frames):
        now_gt = {(g['class_name'], str(g['key'])): g for g in frame['gt']}
        adjacent = (previous_stamp is not None and 0 < frame['stamp']-previous_stamp <= .2
                    and frame['native_index'] == previous_index+1)
        matched_ids = {}
        if frame['scoring_eligible']:
            counts['scoring_frames'] += 1
            for cls in ('vehicle', 'pedestrian', 'obstacle'):
                truth = [g for g in frame['gt'] if g['class_name'] == cls]
                tracks = [t for t in row['tracks'] if t['confirmed'] and class_name(t['core_class']) == cls]
                pairs, missed, extra = greedy_geometry(truth, tracks)
                counts[cls+'/gt_samples'] += len(truth)
                counts[cls+'/matched_samples'] += len(pairs)
                counts[cls+'/unmatched_gt_samples'] += missed
                counts[cls+'/unmatched_output_samples'] += extra
                for g, track, distance in pairs:
                    key = (cls, str(g['key']))
                    matched_ids[key] = track['id']
                    xy.append(distance)
                    if adjacent and key in old_gt:
                        dt = frame['stamp']-previous_stamp
                        gt_v = [(g['center'][j]-old_gt[key]['center'][j])/dt for j in (0, 1)]
                        velocity.append(math.dist(gt_v, track['velocity']))
                    if adjacent and key in old_match:
                        counts[cls+'/adjacent_matched_identity_switches'] += old_match[key] != track['id']
        old_gt, old_match = now_gt, matched_ids
        previous_index, previous_stamp = frame['native_index'], frame['stamp']
    return dict(counts=dict(counts), matched_center_error_m=distribution(xy),
        matched_adjacent_secant_velocity_error_mps=distribution(velocity),
        protocol='Scoring-eligible frames only, class-wise greedy nearest XY<=2m one-to-one. GT secant velocity only across consecutive original native indices <=.2s. Not AP/MOTA or 30Hz ROS tracking evaluation.')


def qualify_case(name, frames, binaries, repeats, output, start=None):
    results, rows_by_policy = {}, {}
    for policy, binary in binaries.items():
        rows, timing = replay_timed(binary, frames, repeats)
        rows_by_policy[policy] = rows
        results[policy] = dict(summary=lifecycle_summary(rows), timing=timing)
        if start is not None:
            results[policy]['episodes'] = {key: episode_summary(rows, start, window) for key, window in WINDOWS.items()}
        with gzip.open(output / (policy + '_rows.json.gz'), 'xt') as handle:
            json.dump(rows, handle, allow_nan=False)
    comparisons = {key: compare_rows(rows_by_policy['baseline'], rows) for key, rows in rows_by_policy.items() if key != 'baseline'}
    if 'policy_off' in comparisons and not comparisons['policy_off']['exact_native_state_and_prior_covariance']:
        write_json(output / 'off_equivalence_failure.json', comparisons['policy_off'])
        raise ValueError('Policy OFF changed baseline state/prior covariance in '+name)
    report = dict(name=name, input_frames=len(frames), usable_tf_frames=sum(f['world_objects'] is not None for f in frames),
        policies=results, comparisons=comparisons)
    write_json(output / 'report.json', report)
    return report


def run(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required: '+str(args.output))
    args.output.mkdir(parents=True)
    pins = {str(p): events.sha(p) for p in (args.world_cache, Path(__file__))}
    if args.pack:
        pins[str(args.pack)] = events.sha(args.pack)
    cached_inputs = []
    if args.world_cache_manifest:
        pins[str(args.world_cache_manifest)] = events.sha(args.world_cache_manifest)
        cached_inputs = json.loads(args.world_cache_manifest.read_text())
        for value in cached_inputs:
            path = Path(value['world_cache'])
            pins[str(path)] = events.sha(path)
            if pins[str(path)] != value['cache_sha256']:
                raise ValueError('Additional cached-world-input digest mismatch')
            if events.sha(value['bag']) != value['bag_sha256']:
                raise ValueError('Recorded bag differs from cache source receipt')
    if args.baseline_build:
        baseline = args.baseline_build / 'original_core'
        receipt = json.loads((args.baseline_build / 'build_receipt.json').read_text())
        if events.sha(baseline) != receipt['binary_sha256']:
            raise ValueError('Frozen baseline binary digest mismatch')
        for path, digest in receipt['production_source_sha256'].items():
            if events.sha(path) != digest:
                raise ValueError('Frozen baseline source digest mismatch')
        baseline_receipt = receipt
    else:
        freeze_tracker(args.output / 'baseline_snapshot', revision=args.baseline_ref)
        baseline, baseline_receipt = build(args.output / 'baseline_snapshot', args.output / 'baseline_build', {})
    freeze_tracker(args.output / 'candidate_snapshot', source=args.candidate_root)
    binaries, receipts = {'baseline': baseline}, {'baseline': baseline_receipt}
    build_directories = {'baseline': args.baseline_build or args.output/'baseline_build'}
    variants = [('candidate', args.candidate_params), ('policy_off', args.policy_off_params), *args.variant]
    if len({name for name, _ in variants}) != len(variants):
        raise ValueError('Duplicate named policy variant')
    for name, overrides in variants:
        if overrides is not None:
            binaries[name], receipts[name] = build(args.output / 'candidate_snapshot', args.output / (name+'_build'), overrides)
            build_directories[name] = args.output/(name+'_build')
    start, saved, excluded = pickle.loads(args.world_cache.read_bytes())
    directory = args.output / 'full_route'
    directory.mkdir()
    reports = {'full_route': qualify_case('full_route', saved[events.DET], binaries, args.repeats, directory, start)}
    reports['full_route']['excluded_tf_stamps'] = excluded
    if args.reference_rows:
        pins[str(args.reference_rows)] = events.sha(args.reference_rows)
        frozen = pickle.loads(args.reference_rows.read_bytes())
        with gzip.open(directory / 'baseline_rows.json.gz', 'rt') as handle:
            reports['full_route']['previous_native_reference'] = compare_rows(frozen, json.load(handle))
    for index, cached in enumerate(cached_inputs):
        start, frames, excluded = pickle.loads(Path(cached['world_cache']).read_bytes())
        directory = args.output / ('recorded_bag_'+str(index).zfill(2))
        directory.mkdir()
        name = 'recorded_bag/'+str(index).zfill(2)
        reports[name] = qualify_case(name, frames[events.DET], binaries, args.repeats, directory)
        reports[name]['recorded_input'] = cached
        reports[name]['excluded_tf_stamps'] = excluded
        print(name, 'complete', flush=True)
    if args.pack:
        pack = load_pack(args.pack)
        packs = {policy: deepcopy(pack) for policy in binaries if policy != 'policy_off'} if args.catalog_metrics else {}
        for index, (name, case) in enumerate(pack['cases'].items()):
            directory = args.output / ('independent_'+str(index).zfill(2))
            directory.mkdir()
            frames = [dict(stamp=f['stamp'], world_objects=f['detections']) for f in case['frames']]
            reports[name] = qualify_case(name, frames, binaries, args.repeats, directory)
            reports[name]['frozen_source_provenance'] = case['provenance']
            for policy in binaries:
                with gzip.open(directory / (policy+'_rows.json.gz'), 'rt') as handle:
                    rows = json.load(handle)
                reports[name]['policies'][policy]['frozen_gt_consistency'] = gt_consistency(rows, case['frames'])
                if policy in packs:
                    for raw, row in zip(packs[policy]['cases'][name]['frames'], rows):
                        raw['tracks'] = [pack_track(t) for t in row['tracks']]
                    if policy == 'baseline':
                        reports[name]['original_e12_pack_track_state_exact'] = all(
                            old['tracks'] == new['tracks'] for old, new in zip(case['frames'], packs[policy]['cases'][name]['frames']))
            print(name, len(frames), 'complete', flush=True)
        for policy, variant in packs.items():
            path = args.output / (policy+'_catalog_pack.json.gz')
            variant['provenance']['qualification_build'] = receipts[policy]
            variant['provenance']['scope'] = 'Isolated native candidate replay; saved input/GT unchanged; not ROS scheduler.'
            with gzip.open(path, 'xt') as handle:
                json.dump(variant, handle, allow_nan=False)
            subprocess.run([sys.executable, str(ROOT/'scripts/evaluation/analyze_e12_tracking_support.py'),
                            '--pack', str(path), '--output', str(args.output/(policy+'_catalog_metrics'))], check=True)
    if args.native_timing:
        for policy in binaries:
            benchmark = build_timing(build_directories[policy], receipts[policy], args.output/(policy+'_timing_build'))
            for name, result in reports.items():
                if name == 'full_route':
                    frames, directory = saved[events.DET], args.output/'full_route'
                elif name.startswith('recorded_bag/'):
                    index = int(name.split('/')[-1])
                    _, topics, _ = pickle.loads(Path(cached_inputs[index]['world_cache']).read_bytes())
                    frames, directory = topics[events.DET], args.output/('recorded_bag_'+str(index).zfill(2))
                else:
                    index = list(pack['cases']).index(name)
                    frames = [dict(stamp=f['stamp'], world_objects=f['detections']) for f in pack['cases'][name]['frames']]
                    directory = args.output/('independent_'+str(index).zfill(2))
                with gzip.open(directory/(policy+'_rows.json.gz'), 'rt') as handle:
                    last = json.load(handle)[-1]
                result['policies'][policy]['native_no_io_timing'] = benchmark_native(benchmark, frames, last, args.repeats)
    if pins != {p: events.sha(p) for p in pins}:
        raise ValueError('Pinned input or evaluator changed during qualification')
    report = dict(complete=True, production_modified=False, protocol='Measurement-stamp native replay using saved world observations, not live ROS scheduling.',
        no_gt_full_route_metrics='No AP, MOTA, empirical FP or planner causal claim; output consistency/lifecycle only.',
        command=sys.argv, source_input_sha256=pins, builds=receipts, cases=reports)
    write_json(args.output / 'report.json', report)
    print(json.dumps({name: {key: value['changed_frames'] for key, value in result['comparisons'].items()}
                      for name, result in reports.items()}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world-cache', type=Path, default=DEFAULT_CACHE,
                        help='Trusted local pickle from recorded TF: (bag_start, topic_frames, excluded_stamps)')
    parser.add_argument('--reference-rows', type=Path, default=OLD / 'native_rows.pkl')
    parser.add_argument('--baseline-build', type=Path, help='Reuse a frozen extended-state helper build with verified receipt')
    parser.add_argument('--baseline-ref', default=BASELINE_REF)
    parser.add_argument('--candidate-root', type=Path, default=TRACKER)
    parser.add_argument('--candidate-params', type=json.loads, default={}, help='JSON object of native configuration overrides')
    parser.add_argument('--policy-off-params', type=json.loads, help='JSON object: additionally require this OFF build to exactly equal baseline')
    parser.add_argument('--variant', type=parse_variant, action='append', default=[], metavar='NAME=JSON',
                        help='Additional named native-policy ablation, built against the same frozen source snapshot')
    parser.add_argument('--world-cache-manifest', type=Path,
                        help='Optional JSON list of additional independent trusted world caches, with bag/cache SHA256 receipts')
    parser.add_argument('--pack', type=Path, help='Optional complete E12/E24 saved-world-input core pack; all cases replay independently')
    parser.add_argument('--catalog-metrics', action='store_true', help='Also reuse established frozen-catalog AP/coverage/velocity/support metrics for every policy; requires --pack')
    parser.add_argument('--native-timing', action='store_true', help='Also build no-trace native timing copies; verify final primary state against full traced replay')
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be >=1')
    if args.catalog_metrics and not args.pack:
        parser.error('--catalog-metrics requires --pack')
    for params in (args.candidate_params, args.policy_off_params):
        if params is not None and not isinstance(params, dict):
            parser.error('Native parameter overrides must be JSON objects')
    run(args)


if __name__ == '__main__':
    main()
