#!/usr/bin/env python3
"""CPU-only native EKF recovery candidates; production files are never edited.

The P+R gate and original hard 5 m distance limit admit a vehicle pair. Only
when that pair would have failed the original P-only gate, decouple XY from
the six latent dynamic states before the native update. The two preregistered
structural alternatives keep the predicted position (decouple) or reacquire
the observed position (reset). Neither uses GT nor overwrites velocity.
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import shlex
import subprocess

import numpy as np
from scipy.optimize import linear_sum_assignment

from audit_tracking_continuity_events import core_summary, replay, sha
from audit_vehicle_reacquisition_core import vehicle_births, write_gzip
from qualify_vehicle_innovation_gate import ADDITION, ANCHOR, TRACKER, noisy_cases, synthetic_cases

ROOT = Path(__file__).resolve().parents[2]


def generated_core(original, profile):
    if profile not in ('decouple', 'reset'):
        raise ValueError('Unknown covariance recovery profile')

    def replace_once(old, new):
        nonlocal original
        if original.count(old) != 1:
            raise ValueError('Pinned native recovery anchor changed: ' + old[:100])
        original = original.replace(old, new, 1)

    replace_once('#include "algorithm/ekf_multi_object_tracking.hpp"',
                 '#include "algorithm/ekf_multi_object_tracking.hpp"\n'
                 '#include "vehicle_covariance_recovery.hpp"\n'
                 '#include <cstdlib>\n#include <fstream>\n#include <iomanip>\n'
                 'static_assert(S_X == 0 && S_Y == 1 && S_YAW == 2 && S_VX == 3 &&\n'
                 '              S_VY == 4 && S_YAW_RATE == 5 && S_AX == 6 && S_AY == 7,\n'
                 '              "Pinned native recovery state order changed");')
    replace_once('    Eigen::MatrixXd ranking;',
                 '    Eigen::MatrixXi recovery_pair = Eigen::MatrixXi::Zero(i_meas_num, MAX_TRACKS);\n'
                 '    Eigen::MatrixXd ranking;')
    replace_once(ANCHOR, ANCHOR + '\n            const double legacy_vehicle_cost = maha_distance;' + ADDITION)
    replace_once('            cost_matrix(meas_idx, track_idx) = maha_distance;', r'''
            const auto recovery_class = measurements.meas[meas_idx].classification;
            const auto recovery_prior_class = all_tracks_[track_idx].getRepClass();
            const bool both_vehicles =
                (recovery_class == mc_mot::ObjectClass::CAR || recovery_class == mc_mot::ObjectClass::TRUCK) &&
                (recovery_prior_class == mc_mot::ObjectClass::CAR || recovery_prior_class == mc_mot::ObjectClass::TRUCK);
            if (both_vehicles && all_tracks_[track_idx].is_init &&
                legacy_vehicle_cost >= config_.max_association_dist_m &&
                maha_distance < config_.max_association_dist_m &&
                l2_distance <= config_.max_association_dist_m) {
                recovery_pair(meas_idx, track_idx) = 1;
            }
            cost_matrix(meas_idx, track_idx) = maha_distance;''')
    replace_once('            UpdateTrack(all_tracks_[track_idx], measurements.meas[meas_idx]);',
                 '            UpdateTrack(all_tracks_[track_idx], measurements.meas[meas_idx],\n'
                 '                        recovery_pair(meas_idx, track_idx) != 0);')
    replace_once('void EkfMultiObjectTracking::UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement) {',
                 'void EkfMultiObjectTracking::UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement,\n'
                 '                                          const bool recovery_pair) {')
    # This is before K is calculated, not a post-update speed clamp. Pvv is
    # preserved; future F creates new Pxv from elapsed time and latent Pvv.
    replace_once('    Eigen::Matrix3d S = H_ * track.state_cov * H_.transpose() + R;',
                 '    if (recovery_pair) {\n'
                 '        if (const char* log_path = std::getenv("BASE5_VEHICLE_RECOVERY_TRACE")) {\n'
                 '            std::ofstream log(log_path, std::ios::app);\n'
                 '            log << std::setprecision(17) << measurement.state.time_stamp << " " << track.track_id\n'
                 '                << " " << measurement.state.x << " " << measurement.state.y\n'
                 '                << " " << track.state_vec(S_X) << " " << track.state_vec(S_Y)\n'
                 '                << " " << track.state_vec(S_VX) << " " << track.state_vec(S_VY)\n'
                 '                << " " << measurement.detection_confidence << "\\n";\n'
                 '        }\n'
                 '        vehicle_recovery::SeparatePosition(track.state_vec, track.state_cov,\n'
                 '            measurement_vec.head<2>().eval(), R.block<2, 2>(0, 0).eval(), '
                 + ('true' if profile == 'reset' else 'false') + ');\n'
                 '    }\n'
                 '    Eigen::Matrix3d S = H_ * track.state_cov * H_.transpose() + R;')
    if profile == 'reset':
        # The observed XY has already initialized its independent covariance
        # R. Applying the same observation again would spuriously halve R and
        # count one center twice. Preserve the native yaw update only.
        gain_anchor = ('    Eigen::Matrix3d S = H_ * track.state_cov * H_.transpose() + R;\n'
                       '    Eigen::Matrix8_3d K = track.state_cov * H_.transpose() * S.inverse();')
        replace_once(gain_anchor, gain_anchor + '\n'
                     '    if (recovery_pair) K.block<8, 2>(0, 0).setZero();')
    return original


def build_recovery(baseline, output, profile):
    source = baseline / 'core/candidate_core.cpp'
    harness = baseline / 'core/original_diagnostic_harness.cpp'
    header = TRACKER / 'include/algorithm/ekf_multi_object_tracking.hpp'
    helper = Path(__file__).with_name('vehicle_covariance_recovery.hpp')
    output.mkdir(parents=True)
    include = output / 'include'
    (include / 'algorithm').mkdir(parents=True)
    declaration = '    void UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement);'
    original_header = header.read_text()
    if original_header.count(declaration) != 1:
        raise ValueError('Pinned update declaration changed')
    (include / 'algorithm/ekf_multi_object_tracking.hpp').write_text(original_header.replace(
        declaration, '    void UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement,\n'
        '                     bool recovery_pair = false);', 1))
    source_copy = output / 'vehicle_recovery_core.cpp'
    source_copy.write_text(generated_core(source.read_text(), profile))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    binary = output / 'vehicle_recovery_core'
    command = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I' + str(include),
               '-I' + str(helper.parent), str(harness), str(source_copy), '-o', str(binary), *flags]
    subprocess.run(command, check=True)
    receipt = dict(profile=profile, native_source_sha256=sha(source), harness_sha256=sha(harness),
                   native_header_sha256=sha(header), helper_sha256=sha(helper),
                   generated_source_sha256=sha(source_copy), binary_sha256=sha(binary), command=command,
                   production_changed=False, gt_is_evaluation_only=True,
                   scope='P+R only for vehicle/vehicle edges; 5 m limit unchanged; reset XY/dynamic cross-covariance only on originally ineligible admitted pairs.')
    (output / 'build_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return binary, receipt


def replay_recovery(binary, frames, trace_path):
    """Reuse the standard native trace parser; log actual recovery edges only.

    Environment and append-only diagnostics are isolated to this process and
    its one child. No production field, Kalman operation or stdout is changed.
    """
    trace_path.write_text('')
    key = 'BASE5_VEHICLE_RECOVERY_TRACE'
    previous = os.environ.get(key)
    try:
        os.environ[key] = str(trace_path.resolve())
        rows = replay(binary, frames)
    finally:
        if previous is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = previous
    events = []
    for line in trace_path.read_text().splitlines():
        tokens = line.split()
        if len(tokens) != 9:
            raise ValueError('Incomplete actual recovery diagnostic')
        events.append(dict(stamp=float(tokens[0]), id=int(tokens[1]),
                           measurement_xy=list(map(float, tokens[2:4])),
                           prior_xy=list(map(float, tokens[4:6])),
                           prior_velocity=list(map(float, tokens[6:8])),
                           score=float(tokens[8])))
    return rows, events


def interpolate_gt(times, observations, stamp):
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1.e-6:
        return np.asarray(observations[index]['center'][:2], dtype=float)
    if index == 0 or index == len(times) or times[index] - times[index - 1] > .15:
        return None
    alpha = (stamp - times[index - 1]) / (times[index] - times[index - 1])
    return ((1. - alpha) * np.asarray(observations[index - 1]['center'][:2], dtype=float) +
            alpha * np.asarray(observations[index]['center'][:2], dtype=float))


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'n': 0}
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite metric')
    return dict(n=len(values), mean=float(values.mean()), p50=float(np.median(values)),
                p90=float(np.percentile(values, 90)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), maximum=float(values.max()))


def evaluate_motion(rows, gt, allowed_stamps=None):
    """Current 2 m one-to-one GT and 1/3 s Cartesian-CV diagnostic, not AP.

    GT velocity is central finite difference on clean GT, only for evaluation.
    The actual downstream Map-CV needs a separate audit with these outputs.
    """
    if gt is None:
        return {}, []
    cohorts = gt['gt']
    samples = []
    for frame in rows:
        stamp = frame['stamp']
        if allowed_stamps is not None and stamp not in allowed_stamps:
            continue
        targets = [(index, interpolate_gt(times, points, stamp))
                   for index, (times, points) in enumerate(cohorts)
                   if points and points[0].get('class_name') == 'vehicle']
        targets = [(index, pos) for index, pos in targets if pos is not None]
        tracks = [item for item in frame['tracks'] if item['confirmed'] and item['core_class'] in (1, 2)]
        if not targets or not tracks:
            continue
        cost = np.asarray([[np.linalg.norm(np.asarray(item['center'][:2]) - pos) for _, pos in targets]
                           for item in tracks])
        associations = zip(*linear_sum_assignment(cost))
        for row, column in associations:
            if cost[row, column] > 2.:
                continue
            item = tracks[row]
            target_index, target_center = targets[column]
            times, points = cohorts[target_index]
            before = interpolate_gt(times, points, stamp - .05)
            after = interpolate_gt(times, points, stamp + .05)
            velocity = np.asarray(item['velocity'], dtype=float)
            sample = dict(stamp=stamp, gt=target_index, id=item['id'], center_error_m=float(cost[row, column]),
                          speed_mps=float(np.linalg.norm(velocity)))
            if before is not None and after is not None:
                target_velocity = (after - before) / .1
                sample['velocity_error_mps'] = float(np.linalg.norm(velocity - target_velocity))
                sample['opposite_direction'] = bool(np.dot(velocity, target_velocity) < -.5)
            for horizon in (1., 3.):
                future = interpolate_gt(times, points, stamp + horizon)
                if future is not None:
                    predicted = np.asarray(item['center'][:2]) + horizon * velocity
                    sample['fde_{}s_m'.format(int(horizon))] = float(np.linalg.norm(predicted - future))
            samples.append(sample)
    summary = {key: distribution([s[key] for s in samples if key in s])
               for key in ('center_error_m', 'velocity_error_mps', 'speed_mps', 'fde_1s_m', 'fde_3s_m')}
    summary['opposite_direction'] = sum(s.get('opposite_direction', False) for s in samples)
    return summary, samples


def compare_motion(baseline, candidate):
    """Pair by stamp and clean-GT cohort, independent of changed track IDs."""
    left = {(round(row['stamp'], 6), row['gt']): row for row in baseline}
    right = {(round(row['stamp'], 6), row['gt']): row for row in candidate}
    paired = sorted(set(left) & set(right))
    result = dict(paired=len(paired), baseline_only=len(set(left) - set(right)),
                  candidate_only=len(set(right) - set(left)))
    for metric in ('center_error_m', 'velocity_error_mps', 'fde_1s_m', 'fde_3s_m'):
        deltas = [right[key][metric] - left[key][metric] for key in paired
                  if metric in left[key] and metric in right[key]]
        result[metric] = dict(delta=distribution(deltas), improved_gt1m=sum(d < -1. for d in deltas),
                              worsened_gt1m=sum(d > 1. for d in deltas))
    return result


def motion_counterexamples():
    """Independent signed motion, acceleration, corner and center-offset tests."""
    result = {}
    for label in ('born_reverse', 'stop_reverse', 'accelerate', 'brake_stop', 'quarter_turn',
                  'localization_step', 'single_outlier', 'two_actor_close_outlier'):
        frames, truths = [], [[], []] if label == 'two_actor_close_outlier' else [[]]
        for index in range(100):
            t = index * .1
            if label == 'born_reverse':
                position, yaw = [-4. * t, 0.], 0.
            elif label == 'stop_reverse':
                position, yaw = [min(t, 3.) * 4. - max(0., t - 5.) * 4., 0.], 0.
            elif label == 'accelerate':
                position, yaw = [2. * t + .8 * t*t, 0.], 0.
            elif label == 'brake_stop':
                capped = min(t, 4.)
                position, yaw = [8. * capped - capped*capped, 0.], 0.
            elif label == 'quarter_turn':
                angle = min(t * .2, math.pi/2.)
                position, yaw = [15. * math.sin(angle), 15. * (1. - math.cos(angle))], angle
            else:
                position, yaw = [4. * t, 0.], 0.
            objects = []
            for actor in range(len(truths)):
                center = [position[0], position[1] + actor * 3.5, .8]
                obj = dict(class_name='vehicle', center=center, yaw=yaw, size=[4.5, 1.8, 1.6], score=.85)
                truths[actor].append(dict(obj, key=str(actor), asset='synthetic_car'))
                # Short absence does not implicitly mean the object stopped.
                if 40 <= index <= 42:
                    continue
                offset = 2.5 if label == 'localization_step' and index >= 43 else (
                    2.5 if label in ('single_outlier', 'two_actor_close_outlier') and index == 43 and actor == 0 else 0.)
                objects.append(dict(obj, center=[center[0] + offset, center[1], .8]))
            if index % 2:
                objects.reverse()
            frames.append(dict(stamp=100. + t, world_objects=objects))
        times = [frame['stamp'] for frame in frames]
        gt = dict(ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in times]),
                  gt=[(times, points) for points in truths], unresolved=[])
        result[label] = (frames, gt)
    return result


def load_cases(baseline, latest_gt=None):
    result = {}
    for cache in sorted(baseline.glob('*_frames.json.gz')):
        name = cache.name[:-len('_frames.json.gz')]
        frames = json.load(gzip.open(cache, 'rt'))
        gt_path = baseline / (name + '_gt.json.gz')
        gt = json.load(gzip.open(gt_path, 'rt')) if gt_path.exists() else None
        if latest_gt and name.startswith('bag0_'):
            gt = json.load(gzip.open(latest_gt, 'rt'))
        result[name] = (frames, gt)
    result.update(synthetic_cases())
    result.update(noisy_cases())
    result.update(motion_counterexamples())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('decouple', 'reset'), required=True)
    parser.add_argument('--latest-raw-uid-gt', type=Path)
    parser.add_argument('--extra-frames', type=Path, action='append', default=[],
                        help='Additional frozen world-object cache; GT is sibling NAME_gt.json.gz if present')
    parser.add_argument('--extra-scoring-source', type=Path,
                        help='Original prediction payload with case/stamp/scoring_eligible; only evaluation is restricted')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh evidence directory required')
    args.output.mkdir(parents=True)
    binary, receipt = build_recovery(args.baseline, args.output / 'core', args.profile)
    report = dict(build=receipt, cases={}, production_changed=False,
                  baseline_receipt_sha256=sha(args.baseline / 'core/build_receipt.json'),
                  latest_raw_uid_gt_sha256=sha(args.latest_raw_uid_gt) if args.latest_raw_uid_gt else None,
                  limits=['Measurement stamp CPU diagnostic, not actual asynchronous ROS.',
                          'Velocity GT and Cartesian-CV are diagnostics, not AP or full Map-CV.',
                          'Deleting XY/dynamic covariance assumes a reacquisition localization bias; wrong association remains possible.'])
    cases = load_cases(args.baseline, args.latest_raw_uid_gt)
    allowed_by_case = {}
    if args.extra_scoring_source:
        scoring = json.load(gzip.open(args.extra_scoring_source, 'rt'))
        for row in scoring['rows']:
            allowed_by_case.setdefault(row['case'], set())
            if row.get('scoring_eligible', row.get('actor_geometry_resolved', False)):
                allowed_by_case[row['case']].add(row['stamp'])
        report['extra_scoring_source_sha256'] = sha(args.extra_scoring_source)
    for path in args.extra_frames:
        name = path.name[:-len('_frames.json.gz')] if path.name.endswith('_frames.json.gz') else path.stem
        if name in cases:
            raise ValueError('Additional cache name collides with regression: ' + name)
        frames = json.load(gzip.open(path, 'rt'))
        frames = [frame for frame in frames if frame['world_objects'] is not None]
        gt_path = path.with_name(name + '_gt.json.gz')
        gt = json.load(gzip.open(gt_path, 'rt')) if gt_path.exists() else None
        cases[name] = frames, gt
        report.setdefault('additional_provenance', {})[name] = dict(frames_path=str(path), frames_sha256=sha(path),
            gt_path=str(gt_path) if gt_path.exists() else None, gt_sha256=sha(gt_path) if gt_path.exists() else None)
    for name, (frames, gt) in cases.items():
        original = replay(args.baseline / 'core/original_core', frames)
        candidate, events = replay_recovery(binary, frames, args.output / (name + '_recovery_events.log'))
        write_gzip(args.output / (name + '_baseline_core.json.gz'), original)
        write_gzip(args.output / (name + '_candidate_core.json.gz'), candidate)
        write_gzip(args.output / (name + '_recovery_events.json.gz'), events)
        allowed = allowed_by_case.get(name)
        if name in report.get('additional_provenance', {}) and args.extra_scoring_source and allowed is None:
            raise ValueError('Independent case lacks scoring-eligibility source: ' + name)
        scored_original = original if allowed is None else [row for row in original if row['stamp'] in allowed]
        scored_candidate = candidate if allowed is None else [row for row in candidate if row['stamp'] in allowed]
        if allowed is not None and not scored_original:
            raise ValueError('Independent case has no native frames with qualified GT: ' + name)
        baseline_motion, baseline_samples = evaluate_motion(original, gt, allowed)
        candidate_motion, candidate_samples = evaluate_motion(candidate, gt, allowed)
        write_gzip(args.output / (name + '_motion_samples.json.gz'), dict(baseline=baseline_samples, candidate=candidate_samples))
        report['cases'][name] = dict(frames=len(frames), scoring_frames=len(scored_original),
            evaluation_excluded_frames=len(original)-len(scored_original),
            actual_recovery_updates=len(events),
            exact_states=original == candidate,
            baseline=core_summary(scored_original, gt), candidate=core_summary(scored_candidate, gt),
            baseline_motion=baseline_motion, candidate_motion=candidate_motion,
            paired_motion=compare_motion(baseline_samples, candidate_samples),
            baseline_stale_birth_candidates=len(vehicle_births(frames, original)),
            candidate_stale_birth_candidates=len(vehicle_births(frames, candidate)))
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        paired = report['cases'][name]['paired_motion']
        print(json.dumps(dict(profile=args.profile, case=name, frames=len(frames),
                              exact=original == candidate, paired=paired['paired'],
                              cv_3s_delta=paired['fde_3s_m']['delta'])), flush=True)


if __name__ == '__main__':
    main()
