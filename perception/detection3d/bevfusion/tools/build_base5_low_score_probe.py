#!/usr/bin/env python3
"""Build an isolated two-stage EKF probe; never patch/install production code.

The generated copy reuses the pinned EKF update/prediction and lifecycle. Only
the experimental secondary association is added; disabled mode is controlled
against an independently compiled original-source binary.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[4]
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'

SECONDARY = r'''
    if (base5_probe_floor < .4) {
        Eigen::MatrixXd secondary(i_meas_num, MAX_TRACKS);
        secondary.setConstant(1000.0);
        for (int m = 0; m < i_meas_num; ++m) {
            const auto &d = measurements.meas[m];
            if (d.classification != mc_mot::ObjectClass::CAR ||
                d.detection_confidence >= .4f || d.detection_confidence < base5_probe_floor) continue;
            for (int t = 0; t < MAX_TRACKS; ++t) {
                const auto &track = all_tracks_[t];
                if (!track.is_init || !track.is_confirmed || assignment_track[t] != -1 ||
                    track.getRepClassProb() < base5_probe_class_min ||
                    (track.getRepClass() != mc_mot::ObjectClass::CAR &&
                     track.getRepClass() != mc_mot::ObjectClass::TRUCK) ||
                    measurements.time_stamp - base5_probe_last_strong_[t] > base5_probe_window) continue;
                if (base5_probe_moving_min > 0.0 &&
                    std::hypot(track.state_vec(S_VX), track.state_vec(S_VY)) <= base5_probe_moving_min) continue;
                const double distance = CalculateDistance(d.state, track.state_vec);
                const double heading_error = std::acos(std::min(1.0, std::abs(
                    std::cos(d.state.yaw - track.state_vec(S_YAW)))));
                if (distance > base5_probe_gate || heading_error > M_PI / 3.0) continue;
                auto compatible = [](double a, double b) {
                    return a > 0 && b > 0 && std::max(a, b) / std::min(a, b) <= 2.0;
                };
                if (!compatible(d.dimension.length, track.dimension.length) ||
                    !compatible(d.dimension.width, track.dimension.width)) continue;
                secondary(m, t) = CalculateMahalanobisDistance(d.state, track);
            }
        }
        std::vector<int> weak_assignment, weak_track_assignment;
        MatchPairs(secondary, weak_assignment, weak_track_assignment);
        for (int m = 0; m < i_meas_num; ++m) {
            if (weak_assignment[m] != -1) assignment[m] = weak_assignment[m];
        }
    }
'''

UPDATE = r'''
            const auto &observation = measurements.meas[meas_idx];
            const double prior_class_probability = all_tracks_[track_idx].getRepClassProb();
            const bool weak = base5_probe_floor < .4 &&
                observation.classification == mc_mot::ObjectClass::CAR &&
                observation.detection_confidence < .4f;
            if (weak && base5_probe_position_only) {
                auto position = observation;
                position.state.yaw = all_tracks_[track_idx].state_vec(S_YAW);
                position.state.z = all_tracks_[track_idx].object_z;
                position.dimension = all_tracks_[track_idx].dimension;
                const auto classes = all_tracks_[track_idx].class_score_arr;
                double saved_classes[CLASS_NUM];
                std::copy(classes, classes + CLASS_NUM, saved_classes);
                const double yaw_variance = R_(2, 2);
                R_(2, 2) *= 1.0e12; // no artificial yaw observation
                UpdateTrack(all_tracks_[track_idx], position);
                R_(2, 2) = yaw_variance;
                std::copy(saved_classes, saved_classes + CLASS_NUM,
                          all_tracks_[track_idx].class_score_arr);
            } else {
                UpdateTrack(all_tracks_[track_idx], observation);
            }
            if (!weak && observation.classification == mc_mot::ObjectClass::CAR)
                base5_probe_last_strong_[track_idx] = measurements.time_stamp;
            if (weak) std::cout << "WEAK " << measurements.time_stamp << " "
                << all_tracks_[track_idx].track_id << " " << meas_idx << " "
                << prior_class_probability << "\n";
'''

HARNESS = r'''
#include "algorithm/ekf_multi_object_tracking.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
double base5_probe_floor = .4, base5_probe_gate = 2.0, base5_probe_window = .5;
double base5_probe_class_min = 0.0;
double base5_probe_moving_min = 0.0;
bool base5_probe_position_only = true;
int main(int argc, char **argv) {
  ros::Time::init();
  if (argc > 1) base5_probe_floor = std::stod(argv[1]);
  if (argc > 2) base5_probe_gate = std::stod(argv[2]);
  if (argc > 3) base5_probe_window = std::stod(argv[3]);
  if (argc > 4) base5_probe_position_only = std::stoi(argv[4]) != 0;
  if (argc > 5) base5_probe_class_min = std::stod(argv[5]);
  if (argc > 6) base5_probe_moving_min = std::stod(argv[6]);
  MultiClassObjectTrackingConfig config;
  config.global_coord_track = true;
  config.prediction_model = mc_mot::PredictionModel::CTRV;
  config.max_association_dist_m = 5.;
  config.system_noise_std_xy_m = .01;
  config.system_noise_std_yaw_deg = .5;
  config.system_noise_std_vx_vy_ms = 1.;
  config.system_noise_std_yaw_rate_degs = 1.;
  config.system_noise_std_ax_ay_ms2 = .6;
  config.meas_noise_std_xy_m = .4;
  config.meas_noise_std_yaw_deg = 10.;
  config.detection_velocity_fusion_mode = 0;
  config.dimension_filter_alpha = .3;
  config.use_yaw_rate_filtering = true;
  auto tracker = std::make_unique<EkfMultiObjectTracking>(config);
  double time = 0, previous = 0;
  int count = 0;
  std::cout << std::setprecision(17);
  while (std::cin >> time >> count) {
    if (count < 0 || count > MAX_TRACKS || (previous && time <= previous)) return 2;
    mc_mot::Meastructs frame; frame.time_stamp = time;
    for (int i = 0; i < count; ++i) {
      int label; mc_mot::Meastruct d; d.state = {};
      if (!(std::cin >> label >> d.state.x >> d.state.y >> d.state.z >> d.state.yaw
          >> d.dimension.length >> d.dimension.width >> d.dimension.height >> d.detection_confidence)) return 3;
      if (label < 0 || label > 2) return 4;
      d.classification = label == 0 ? mc_mot::ObjectClass::CAR :
          label == 1 ? mc_mot::ObjectClass::PEDESTRIAN : mc_mot::ObjectClass::UNKNOWN;
      d.state.time_stamp = time; frame.meas.push_back(d);
    }
    if (previous) tracker->RunPrediction(time - previous);
    tracker->RunUpdate(frame);
    const auto result = tracker->GetTrackResults();
    int n = 0; for (const auto &t : result.track) if (t.is_init) ++n;
    std::cout << "FRAME " << time << " " << n << "\n";
    for (const auto &t : result.track) if (t.is_init) {
      std::cout << "TRACK " << t.track_id << " " << t.getRepClass() << " "
          << t.is_confirmed << " " << t.is_associated << " "
          << t.state_vec(S_X) << " " << t.state_vec(S_Y) << " " << t.object_z << " "
          << t.state_vec(S_VX) << " " << t.state_vec(S_VY) << " " << t.state_vec(S_YAW) << " "
          << t.dimension.length << " " << t.dimension.width << " " << t.dimension.height << " "
          << t.detection_confidence << " " << t.last_measurement_time << " " << t.age << "\n";
    }
    previous = time;
  }
}
'''


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Pinned source anchor changed: ' + old[:90])
    return text.replace(old, new, 1)


def probe_source(source):
    source = replace_once(source, 'cost_matrix(meas_idx, track_idx) = maha_distance;',
        'if (base5_probe_floor < .4 && measurements.meas[meas_idx].classification == mc_mot::ObjectClass::CAR &&\n'
        '                measurements.meas[meas_idx].detection_confidence < .4f) maha_distance = 1000.;\n'
        '            cost_matrix(meas_idx, track_idx) = maha_distance;')
    source = replace_once(source, 'MatchPairs(cost_matrix, assignment, assignment_track);',
                          'MatchPairs(cost_matrix, assignment, assignment_track);\n' + SECONDARY)
    source = replace_once(source, '            UpdateTrack(all_tracks_[track_idx], measurements.meas[meas_idx]);', UPDATE)
    source = replace_once(source, '        if (ShouldSuppressDuplicateTrackBirth(measurements.meas[meas_idx])) {',
        '        if (base5_probe_floor < .4 && measurements.meas[meas_idx].classification == mc_mot::ObjectClass::CAR &&\n'
        '            measurements.meas[meas_idx].detection_confidence < .4f) continue;\n'
        '        if (ShouldSuppressDuplicateTrackBirth(measurements.meas[meas_idx])) {')
    source = replace_once(source, '        all_tracks_[cur_track_id_] = new_track;',
                          '        base5_probe_last_strong_[cur_track_id_] = measurements.time_stamp;\n'
                          '        all_tracks_[cur_track_id_] = new_track;')
    return source


def build(output):
    if output.exists():
        raise ValueError('Fresh probe directory required')
    source = TRACKER / 'src/algorithm/ekf_multi_object_tracking.cpp'
    header = TRACKER / 'include/algorithm/ekf_multi_object_tracking.hpp'
    inputs = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, header)}
    source_text, header_text = source.read_text(), header.read_text()
    transformed = probe_source(source_text)
    probe_header = replace_once(header_text, '    int cur_track_id_{0}; // Current track id',
        '    std::array<double, MAX_TRACKS> base5_probe_last_strong_{};\n'
        '    int cur_track_id_{0}; // Current track id')
    probe_header = 'extern double base5_probe_floor, base5_probe_gate, base5_probe_window, base5_probe_class_min, base5_probe_moving_min;\nextern bool base5_probe_position_only;\n' + probe_header
    output.mkdir(parents=True)
    include = output / 'include/algorithm'; include.mkdir(parents=True)
    (include / header.name).write_text(probe_header)
    generated = output / 'probe.cpp'; generated.write_text(transformed)
    harness = output / 'harness.cpp'; harness.write_text(HARNESS)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    for name, cpp, inc in [('original', source, TRACKER / 'include'), ('probe', generated, output / 'include')]:
        command = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I' + str(inc),
                   str(harness), str(cpp), '-o', str(output / name), *flags]
        subprocess.run(command, check=True)
    if inputs != {str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in inputs}:
        raise ValueError('Production source changed during build')
    receipt = dict(production_unchanged=True, input_sha256=inputs,
                   generated_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in (generated, include / header.name, harness)},
                   binaries={n: hashlib.sha256((output / n).read_bytes()).hexdigest() for n in ('original', 'probe')},
                   scope='Isolated offline actual EKF core copies; no installed/node/ROS scheduler/TF/planner changes')
    (output / 'build.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    build(p.parse_args().output)
