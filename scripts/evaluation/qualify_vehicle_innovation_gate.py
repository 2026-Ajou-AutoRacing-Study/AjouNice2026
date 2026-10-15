#!/usr/bin/env python3
"""Isolated actual-innovation vehicle gate candidate, never a runtime patch.

The existing 5 m Cartesian limit, greedy assignment and all non-vehicle
edges remain. Only vehicle/vehicle Mahalanobis covariance uses P + R,
including the same tenfold low-confidence R used in the actual EKF update.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
import shlex
import subprocess

import numpy as np

from audit_tracking_continuity_events import core_summary, replay, sha
from audit_vehicle_reacquisition_core import vehicle_births, write_gzip

ROOT = Path(__file__).resolve().parents[2]
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'
ANCHOR = '            maha_distance = CalculateMahalanobisDistance(measurements.meas[meas_idx].state, all_tracks_[track_idx]);'
ADDITION = r'''
            // Isolated research overlay: same measurement uncertainty as
            // UpdateTrack, but keep the original 5 m Cartesian gate below.
            const auto observation_class = measurements.meas[meas_idx].classification;
            const auto prior_class = all_tracks_[track_idx].getRepClass();
            if ((observation_class == mc_mot::ObjectClass::CAR ||
                 observation_class == mc_mot::ObjectClass::TRUCK) &&
                all_tracks_[track_idx].is_init &&
                (prior_class == mc_mot::ObjectClass::CAR ||
                 prior_class == mc_mot::ObjectClass::TRUCK)) {
                Eigen::Vector2d innovation;
                innovation << measurements.meas[meas_idx].state.x - all_tracks_[track_idx].state_vec(S_X),
                              measurements.meas[meas_idx].state.y - all_tracks_[track_idx].state_vec(S_Y);
                Eigen::Matrix2d covariance = all_tracks_[track_idx].state_cov.block<2, 2>(S_X, S_X);
                covariance += (measurements.meas[meas_idx].detection_confidence < 0.5 ? 10.0 : 1.0) *
                              R_.block<2, 2>(0, 0);
                covariance = (0.5 * (covariance + covariance.transpose())).eval();
                if (covariance.allFinite() && covariance.determinant() > 1.e-12 &&
                    covariance(0, 0) > 0.0 && covariance(1, 1) > 0.0) {
                    const double squared = innovation.dot(covariance.inverse() * innovation);
                    if (std::isfinite(squared) && squared >= 0.0) {
                        maha_distance = std::sqrt(squared);
                        maha_distance = std::max(innovation.norm() / 5.0,
                                                std::min(innovation.norm() * 3.0, maha_distance));
                    }
                }
            }
'''


def build_overlay(baseline, output, rescue_xy_noise_scale=1.0):
    source = baseline / 'core/candidate_core.cpp'
    harness = baseline / 'core/original_diagnostic_harness.cpp'
    header = TRACKER / 'include/algorithm/ekf_multi_object_tracking.hpp'
    original = source.read_text()
    if original.count(ANCHOR) != 1:
        raise ValueError('Pinned native pair-cost anchor changed')
    output.mkdir(parents=True)
    copy = output / 'innovation_core.cpp'
    generated = original.replace(ANCHOR, ANCHOR + ADDITION, 1)
    include = TRACKER / 'include'
    if rescue_xy_noise_scale != 1.0:
        def replace_once(old, new):
            nonlocal generated
            if generated.count(old) != 1:
                raise ValueError('Native recovery anchor changed: ' + old[:80])
            generated = generated.replace(old, new, 1)
        replace_once('    Eigen::MatrixXd ranking;',
            '    Eigen::MatrixXd rescued_xy_scale(i_meas_num, MAX_TRACKS);\n'
            '    rescued_xy_scale.setOnes();\n    Eigen::MatrixXd ranking;')
        replace_once(ANCHOR, ANCHOR + '\n            const double legacy_vehicle_cost = maha_distance;')
        replace_once('            cost_matrix(meas_idx, track_idx) = maha_distance;',
            '            if (legacy_vehicle_cost >= config_.max_association_dist_m &&\n'
            '                maha_distance < config_.max_association_dist_m &&\n'
            '                l2_distance <= config_.max_association_dist_m)\n'
            '                rescued_xy_scale(meas_idx, track_idx) = ' + str(rescue_xy_noise_scale) + ';\n'
            '            cost_matrix(meas_idx, track_idx) = maha_distance;')
        replace_once('            UpdateTrack(all_tracks_[track_idx], measurements.meas[meas_idx]);',
            '            UpdateTrack(all_tracks_[track_idx], measurements.meas[meas_idx],\n'
            '                        rescued_xy_scale(meas_idx, track_idx));')
        replace_once('void EkfMultiObjectTracking::UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement) {',
            'void EkfMultiObjectTracking::UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement,\n'
            '                                          const double rescued_xy_noise_scale) {')
        replace_once('    Eigen::Matrix3d S = H_ * track.state_cov * H_.transpose() + R;',
            '    R.block<2, 2>(0, 0) *= rescued_xy_noise_scale;\n'
            '    Eigen::Matrix3d S = H_ * track.state_cov * H_.transpose() + R;')
        include = output / 'include'
        (include / 'algorithm').mkdir(parents=True)
        declaration = '    void UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement);'
        header_text = header.read_text()
        if header_text.count(declaration) != 1:
            raise ValueError('Native update declaration changed')
        (include / 'algorithm/ekf_multi_object_tracking.hpp').write_text(header_text.replace(declaration,
            '    void UpdateTrack(mc_mot::TrackStruct &track, const mc_mot::Meastruct &measurement,\n'
            '                     double rescued_xy_noise_scale = 1.0);', 1))
    copy.write_text(generated)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    binary = output / 'innovation_core'
    command = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I' + str(include),
               str(harness), str(copy), '-o', str(binary), *flags]
    subprocess.run(command, check=True)
    receipt = dict(native_copy_sha256=sha(source), harness_sha256=sha(harness), header_sha256=sha(header),
                   candidate_source_sha256=sha(copy), binary_sha256=sha(binary), command=command,
                   production_changed=False, rescue_xy_noise_scale=rescue_xy_noise_scale,
                   scope='Vehicle/vehicle P+R covariance; Cartesian gate <=5 m unchanged. Optional XY-R scaling only on newly rescued, formerly ineligible pairs.')
    (output / 'build_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return binary, receipt


def synthetic_cases():
    """Known two-actor identity stress; no map or GT is passed to the EKF."""
    cases = {}
    for separation in (2.0, 3.5, 5.0, 7.0):
        for second_axis in (0.0, math.pi):
            name = 'adjacent_{}_yaw{}'.format(separation, round(second_axis))
            frames, truth = [], [[], []]
            for index in range(65):
                stamp = 100.0 + index * .1
                objects = []
                for actor in (0, 1):
                    center = [index * .8, actor * separation, 0.8]
                    obj = dict(class_name='vehicle', center=center, yaw=second_axis if actor else 0.0,
                               size=[4.5, 1.8, 1.6], score=.85)
                    truth[actor].append(dict(obj, key=str(actor), asset='synthetic_car'))
                    # Short and longer independent occlusions, including a
                    # surviving adjacent actor. Detection order alternates.
                    if not (actor == 0 and 20 <= index <= 24) and not (actor == 1 and 40 <= index <= 44):
                        objects.append(obj)
                if index % 2:
                    objects.reverse()
                frames.append(dict(stamp=stamp, world_objects=objects))
            times = [frame['stamp'] for frame in frames]
            gt = dict(ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in times]),
                      gt=[(times, points) for points in truth], unresolved=[])
            cases[name] = (frames, gt)
    # Crossing vehicles: same boxes, opposite real motion. Intersection
    # ambiguity is intentionally not hidden by distinct class/shape IDs.
    frames, truth = [], [[], []]
    for index in range(65):
        stamp = 100.0 + index * .1
        objects = []
        for actor in (0, 1):
            direction = 1.0 if actor == 0 else -1.0
            obj = dict(class_name='vehicle', center=[direction * (index * .6 - 18.), actor * 3.5, .8],
                       yaw=0. if actor == 0 else math.pi, size=[4.5, 1.8, 1.6], score=.85)
            truth[actor].append(dict(obj, key=str(actor), asset='synthetic_car'))
            if not (actor == 0 and 26 <= index <= 30):
                objects.append(obj)
        if index % 2:
            objects.reverse()
        frames.append(dict(stamp=stamp, world_objects=objects))
    times = [frame['stamp'] for frame in frames]
    cases['opposite_directions_short_occlusion'] = (frames, dict(
        ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in times]),
        gt=[(times, points) for points in truth], unresolved=[]))
    return cases


def noisy_cases():
    """Independent noisy nearby vehicles; matches are scored against clean GT."""
    cases = {}
    for noise in (.15, .35, .65):
        for dropout in (.1, .25):
            for seed in (7, 23):
                rng = np.random.RandomState(seed)
                frames, truth = [], [[], [], []]
                for index in range(140):
                    stamp = 100.0 + index * .1
                    objects = []
                    for actor in range(3):
                        center = [index * .4 + actor * 2.0, actor * 3.5, .8]
                        obj = dict(class_name='vehicle', center=center, yaw=0., size=[4.5, 1.8, 1.6], score=.85)
                        truth[actor].append(dict(obj, key=str(actor), asset='synthetic_car'))
                        if index >= 12 and (rng.uniform() < dropout or actor == 0 and 35 <= index <= 39):
                            continue
                        measured = dict(obj, center=[center[0]+rng.normal(0., noise),
                                                    center[1]+rng.normal(0., noise), center[2]])
                        measured['score'] = .45 if rng.uniform() < .15 else .85
                        objects.append(measured)
                    if index % 2:
                        objects.reverse()
                    frames.append(dict(stamp=stamp, world_objects=objects))
                times = [frame['stamp'] for frame in frames]
                gt = dict(ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in times]),
                          gt=[(times, points) for points in truth], unresolved=[])
                cases['noisy_{}_drop{}_seed{}'.format(noise, dropout, seed)] = (frames, gt)
    # A newly appearing independent car outside the hard spatial limit must
    # retain a new track even while the old confirmed car temporarily coasts.
    frames, truth = [], [[], []]
    for index in range(45):
        stamp = 100.0 + index * .1
        first = dict(class_name='vehicle', center=[index * .1, 0., .8], yaw=0., size=[4.5, 1.8, 1.6], score=.85)
        truth[0].append(dict(first, key='first', asset='synthetic_car'))
        second = dict(first, center=[index * .1, 7., .8])
        if index >= 25:
            truth[1].append(dict(second, key='new', asset='synthetic_car'))
        objects = [] if 25 <= index <= 28 else [first]
        if index >= 25:
            objects.append(second)
        frames.append(dict(stamp=stamp, world_objects=objects))
    times = [frame['stamp'] for frame in frames]
    cases['independent_birth_outside_5m'] = (frames, dict(
        ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in times]),
        gt=[(times, truth[0]), (times[25:], truth[1])], unresolved=[]))
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rescue-xy-noise-scale', type=float, default=1.0, choices=(1., 2., 4., 10.))
    parser.add_argument('--latest-raw-uid-gt', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    binary, receipt = build_overlay(args.baseline, args.output / 'core', args.rescue_xy_noise_scale)
    baseline_binary = args.baseline / 'core/original_core'
    cases = {}
    for cache in args.baseline.glob('*_frames.json.gz'):
        name = cache.name[:-len('_frames.json.gz')]
        frames = json.load(gzip.open(cache, 'rt'))
        gt_path = args.baseline / (name + '_gt.json.gz')
        gt = json.load(gzip.open(gt_path, 'rt')) if gt_path.exists() else None
        if args.latest_raw_uid_gt and name.startswith('bag0_'):
            gt = json.load(gzip.open(args.latest_raw_uid_gt, 'rt'))
        cases[name] = (frames, gt)
    cases.update(synthetic_cases())
    cases.update(noisy_cases())
    report = dict(build=receipt, cases={}, production_changed=False,
                  baseline_build_receipt_sha256=sha(args.baseline / 'core/build_receipt.json'),
                  latest_raw_uid_gt_sha256=sha(args.latest_raw_uid_gt) if args.latest_raw_uid_gt else None,
                  limits=['Known same actor is evaluation-only; candidate does not merge/delete IDs.',
                          'Measurement-stamp replay is not 30 Hz asynchronous ROS or a planner safety certificate.',
                          'Four older bags use the frozen previously prepared E24 prediction cohort; omitted prehistory is not restored.'])
    for name, (frames, gt) in cases.items():
        baseline = replay(baseline_binary, frames)
        candidate = replay(binary, frames)
        write_gzip(args.output / (name + '_baseline_core.json.gz'), baseline)
        write_gzip(args.output / (name + '_candidate_core.json.gz'), candidate)
        result = dict(frames=len(frames), baseline=core_summary(baseline, gt), candidate=core_summary(candidate, gt),
                      exact_states=baseline == candidate,
                      baseline_stale_birth_candidates=len(vehicle_births(frames, baseline)),
                      candidate_stale_birth_candidates=len(vehicle_births(frames, candidate)))
        report['cases'][name] = result
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(case=name, frames=len(frames), exact=result['exact_states'],
                              baseline_ids=result['baseline']['confirmed_unique_ids'],
                              candidate_ids=result['candidate']['confirmed_unique_ids'])), flush=True)


if __name__ == '__main__':
    main()
