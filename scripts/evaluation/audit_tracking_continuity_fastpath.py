#!/usr/bin/env python3
"""Isolated original-core inactive-slot fast path, exact state and CPU study.

Production source is read-only. Both binaries have the current installed
Release optimization flags. Only calls whose result is immediately replaced
by the same 1000.0 ineligible cost are skipped; matching/gates/tie order stay.
No ROS master, nodes, GPU, model inference, driver, planner or vehicle control.
"""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess

import yaml

from audit_tracking_continuity import DET, ROOT, stats
from audit_tracking_continuity_events import load, sha
from audit_tracking_continuity_saved_outputs import prepared_cases

TRACKER = ROOT/'base_ws/src/ekf-multi-object-tracking'
ANCHOR = '        for (int track_idx = 0; track_idx < MAX_TRACKS; ++track_idx) {\n            l2_distance = CalculateDistance('
FAST = ('        for (int track_idx = 0; track_idx < MAX_TRACKS; ++track_idx) {\n'
        '            if (!all_tracks_[track_idx].is_init) {\n'
        '                cost_matrix(meas_idx, track_idx) = 1000.0;\n'
        '                continue;\n'
        '            }\n'
        '            l2_distance = CalculateDistance(')


def fast_source(original):
    if original.count(ANCHOR) != 1:
        raise ValueError('Original cost-loop anchor changed')
    return original.replace(ANCHOR, FAST, 1)


HARNESS = r'''
#include "algorithm/ekf_multi_object_tracking.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <chrono>
using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point begin) {
  return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
}
MultiClassObjectTrackingConfig settings() {
  MultiClassObjectTrackingConfig config;
  config.prediction_model = mc_mot::PredictionModel::CTRV;
SETTINGS
  return config;
}
mc_mot::Meastruct measurement(int index, double time) {
  mc_mot::Meastruct d; d.state = {};
  d.classification = mc_mot::ObjectClass::CAR;
  d.state.time_stamp = time; d.state.x = index*10.; d.state.y = 0.;
  d.dimension.length = 4.; d.dimension.width = 2.; d.dimension.height = 1.5;
  d.detection_confidence = .8f;
  return d;
}
mc_mot::Meastructs synthetic(int count, double time) {
  mc_mot::Meastructs frame; frame.time_stamp = time;
  for (int i=0;i<count;++i) frame.meas.push_back(measurement(i,time));
  return frame;
}
void full(const mc_mot::TrackStruct &t) {
  // classification has no initializer and is not used by the production
  // representative-class policy; do not read that undefined raw enum.
  std::cout << "FULL " << t.track_id << " " << t.initialization_time << " " << t.update_time
            << " " << t.last_measurement_time << " " << t.has_detector_velocity
            << " " << t.detector_velocity_x << " " << t.detector_velocity_y
            << " " << t.detector_velocity_time << " " << t.detector_velocity_variance_mps2
            << " " << t.detection_confidence << " " << t.age << " " << t.direction_score
            << " " << t.object_z << " " << t.last_center_motion_fusion_time
            << " " << t.has_center_motion_velocity << " " << t.center_motion_velocity_x
            << " " << t.center_motion_velocity_y << " " << t.center_motion_velocity_noise_std_mps;
  for (int i=0;i<8;++i) std::cout << " " << t.state_vec(i);
  for (int i=0;i<8;++i) for (int j=0;j<8;++j) std::cout << " " << t.state_cov(i,j);
  for (int i=0;i<MAX_HISTORY;++i) std::cout << " " << t.detection_arr[i];
  for (int i=0;i<CLASS_NUM;++i) std::cout << " " << t.class_score_arr[i];
  std::cout << " " << t.center_history.size();
  for (const auto &p:t.center_history) std::cout << " " << p.time_stamp << " " << p.x << " " << p.y;
  std::cout << "\n";
}
void dump(EkfMultiObjectTracking &tracker, double time) {
  const auto result = tracker.GetTrackResults();
  int n=0; for (const auto &t:result.track) if (t.is_init) ++n;
  std::cout << "FRAME " << time << " " << n << "\n";
  for (const auto &t:result.track) if (t.is_init) {
    std::cout << "TRACK " << t.track_id << " " << t.getRepClass() << " "
      << t.is_confirmed << " " << t.is_associated << " "
      << t.state_vec(S_X) << " " << t.state_vec(S_Y) << " " << t.object_z << " "
      << t.state_vec(S_VX) << " " << t.state_vec(S_VY) << " " << t.state_vec(S_YAW) << " "
      << t.dimension.length << " " << t.dimension.width << " " << t.dimension.height << " "
      << t.detection_confidence << " " << t.last_measurement_time << " " << t.age << "\n";
    full(t);
  }
}
int bench(int repetitions) {
  double sink=0.;
  for (int active: {0,1,100,500}) {
    auto seed = std::make_unique<EkfMultiObjectTracking>(settings());
    for (int i=0;i<4;++i) seed->RunUpdate(synthetic(active,100.+i*.01));
    const auto seeded = seed->GetTrackResults();
    int confirmed=0, initialized=0;
    for (const auto &t:seeded.track) {initialized+=t.is_init; confirmed+=t.is_confirmed;}
    if (initialized!=active || (active && confirmed!=active)) return 8;
    for (int count: {0,1,100}) {
      const auto frame = synthetic(count,100.04);
      for (int i=-40;i<repetitions;++i) {
        // Independent copies keep active slot density constant. Copy and
        // synthetic input construction are outside RunUpdate timing.
        auto tracker = std::make_unique<EkfMultiObjectTracking>(*seed);
        auto begin=Clock::now(); tracker->RunUpdate(frame); double cost=elapsed_ms(begin);
        const auto after=tracker->GetTrackResults(); sink+=after.time_stamp;
        if (i>=0) std::cout << "BENCH " << active << " " << count << " " << i << " " << cost << "\n";
      }
    }
  }
  std::cerr << "sink " << sink << "\n";
  return 0;
}
int trace_synthetic() {
  for (int active: {0,1,100,500}) for (int count: {0,1,100}) {
    auto tracker=std::make_unique<EkfMultiObjectTracking>(settings());
    for (int i=0;i<4;++i) tracker->RunUpdate(synthetic(active,100.+i*.01));
    tracker->RunUpdate(synthetic(count,100.04));
    std::cout << "SYNTHETIC " << active << " " << count << "\n";
    dump(*tracker,100.04);
  }
  return 0;
}
int main(int argc,char **argv) {
  ros::Time::init(); std::cout << std::setprecision(17);
  if (argc>1 && std::string(argv[1])=="--bench") return bench(std::stoi(argv[2]));
  if (argc>1 && std::string(argv[1])=="--synthetic-trace") return trace_synthetic();
  auto tracker=std::make_unique<EkfMultiObjectTracking>(settings());
  double time=0.,previous=0.; int count=0,index=0;
  while (std::cin >> time >> count) {
    if (count<0 || count>MAX_TRACKS || (previous && time<=previous)) return 2;
    mc_mot::Meastructs frame; frame.time_stamp=time;
    for (int i=0;i<count;++i) {
      int label; mc_mot::Meastruct d; d.state={};
      if (!(std::cin >> label >> d.state.x >> d.state.y >> d.state.z >> d.state.yaw
            >> d.dimension.length >> d.dimension.width >> d.dimension.height >> d.detection_confidence)) return 3;
      if (label<0 || label>2) return 4;
      d.classification=label==0 ? mc_mot::ObjectClass::CAR :
        label==1 ? mc_mot::ObjectClass::PEDESTRIAN : mc_mot::ObjectClass::UNKNOWN;
      d.state.time_stamp=time; frame.meas.push_back(d);
    }
    int initialized=0;
    for (const auto &t:tracker->GetTrackResults().track) initialized+=t.is_init;
    auto prediction_begin=Clock::now(); if (previous) tracker->RunPrediction(time-previous);
    double prediction_ms=elapsed_ms(prediction_begin);
    auto update_begin=Clock::now(); tracker->RunUpdate(frame); double update_ms=elapsed_ms(update_begin);
    std::cout << "METRIC " << index << " " << time << " " << count << " " << initialized
              << " " << update_ms << " " << prediction_ms << "\n";
    dump(*tracker,time); previous=time; ++index;
  }
  return 0;
}
'''


def build(output, parameters):
    source = TRACKER/'src/algorithm/ekf_multi_object_tracking.cpp'
    header = TRACKER/'include/algorithm/ekf_multi_object_tracking.hpp'
    original = source.read_text()
    fields = set(re.findall(r'^\s+(?:bool|double|int)\s+(\w+)\s*(?:\{|=)', header.read_text(), re.M))
    settings = ['  config.{} = {};'.format(k, str(v).lower() if isinstance(v, bool) else v)
                for k, v in parameters.items() if k in fields and isinstance(v, (bool, int, float))]
    if parameters['prediction_model'] != 1:
        raise ValueError('The fixed replay contract requires CTRV')
    output.mkdir(parents=True)
    harness = output/'fastpath_harness.cpp'
    harness.write_text(HARNESS.replace('SETTINGS', '\n'.join(settings)))
    original_copy = output/'original_core.cpp'; original_copy.write_text(original)
    candidate_copy = output/'candidate_core.cpp'; candidate_copy.write_text(fast_source(original))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    binaries, commands = {}, {}
    for name, linked in [('baseline', original_copy), ('candidate', candidate_copy)]:
        binary = output/name
        command = ['g++', '-std=c++14', '-O3', '-DNDEBUG', '-I/usr/include/eigen3', '-I'+str(TRACKER/'include'),
                   str(harness), str(linked), '-o', str(binary), *flags]
        subprocess.run(command, check=True)
        binaries[name] = binary; commands[name] = command
    installed_flags = ROOT/'base_ws/build_aw149/ekf_multi_object_tracking/CMakeFiles/ekf_multi_object_tracking.dir/flags.make'
    receipt = dict(source_sha256=sha(source), header_sha256=sha(header), original_copy_sha256=sha(original_copy),
        candidate_copy_sha256=sha(candidate_copy), harness_sha256=sha(harness),
        binaries={k: sha(p) for k, p in binaries.items()}, commands=commands,
        installed_release_flags=str(installed_flags), installed_flags_sha256=sha(installed_flags),
        settings=settings, production_modified=False,
        purity='CalculateDistance and Mahalanobis read const state/track and local Eigen temporaries, with no logs, member writes or RNG. Caller overwrites every inactive cost with 1000.0.',
        gate_tie_order='The same ineligible 1000.0 cost is assigned to the same matrix cell, and all active calls/loop/MatchPairs ordering stay unchanged.')
    return binaries, receipt


def trace(binary, frames=None, synthetic=False):
    lines = []
    if frames is not None:
        for frame in frames:
            objects = frame['world_objects']
            lines.append('{} {}'.format(format(frame['stamp'], '.17g'), len(objects)))
            for obj in objects:
                label = ('vehicle', 'pedestrian', 'obstacle').index(obj['class_name'])
                lines.append(' '.join(format(v, '.17g') for v in (label, *obj['center'], obj['yaw'], *obj['size'], obj['score'])))
    command = [str(binary.resolve())]+(['--synthetic-trace'] if synthetic else [])
    result = subprocess.run(command, input='\n'.join(lines)+'\n', text=True, capture_output=True, check=True, timeout=240)
    state_lines, costs = [], []
    for line in result.stdout.splitlines():
        if line.startswith('METRIC '):
            _, i, stamp, measurements, active, update, prediction = line.split()
            costs.append(dict(frame=int(i), stamp=float(stamp), measurements=int(measurements), active=int(active),
                              update_ms=float(update), prediction_ms=float(prediction)))
        else:
            state_lines.append(line)
    return '\n'.join(state_lines)+'\n', costs


def benchmarks(binary, repetitions):
    result = subprocess.run([str(binary.resolve()), '--bench', str(repetitions)], capture_output=True,
                            text=True, check=True, timeout=240)
    grouped = defaultdict(list)
    for line in result.stdout.splitlines():
        if line.startswith('BENCH '):
            _, active, count, index, cost = line.split()
            grouped['active{}_measurement{}'.format(active, count)].append(float(cost))
    if len(grouped) != 12 or any(len(v) != repetitions for v in grouped.values()):
        raise ValueError('Incomplete synthetic benchmark')
    return dict(grouped)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--parameters', type=Path, required=True)
    p.add_argument('--packet-root', type=Path, required=True)
    p.add_argument('--teammate-bag', type=Path, required=True)
    p.add_argument('--repetitions', type=int, default=250)
    p.add_argument('--rounds', type=int, default=6)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Fresh output required')
    a.output.mkdir(parents=True)
    parameters = yaml.safe_load(a.parameters.read_text())['configure']
    binaries, receipt = build(a.output/'build', parameters)
    report = dict(build=receipt, parameters_sha256=sha(a.parameters), parameters_source=str(a.parameters),
        scope='Current original C++ core only, same greedy cost/gate; no full ROS/TF/scheduler/planner/live performance certification.',
        production_modified=False, equivalence={}, real_trace_cost={}, benchmark_rounds=[],
        limits=['Synthetic copies preserve active density; core copy, input construction, seeding and GetTrackResults/serialization are outside timing.',
                'Warmup40 per synthetic density/count; isolated single process per variant, alternating process order. Other host work is not controlled.',
                'Trace cost excludes RunPrediction and output serialization, logged separately. Raw frame order and gaps are preserved.',
                'Exact trace covers every defined active state/8x8 covariance/history/class vote, flags, score and lifecycle; undefined unused raw classification enum is not read.',
                'Wall-time CPU microbenchmark is not ROS callback throughput or input age.'])
    sources = {}
    packet_inputs = list(sorted(a.packet_root.glob('case*_core.json.gz')))
    for path in packet_inputs:
        payload = json.load(gzip.open(path, 'rt'))
        name = 'packet_'+path.name.replace('_core.json.gz', '')
        sources[name] = (payload['input_frames'], dict(path=str(path), sha256=sha(path)))
    old, _ = prepared_cases(ROOT/'artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/inputs',
        ROOT/'artifacts/evaluation/bevfusion/base5_compile_bag_261012/E24_amp_parallel_v1/predictions.json.gz',
        ROOT/'configs/sensors/legacy/Sensor_4.json')
    for name, case in old.items():
        sources['old_'+name] = (case['frames'], dict(raw_bag=case['raw_bag'], input_frames=len(case['frames'])))
    _, latest, excluded = load(a.teammate_bag)
    sources['teammate_full'] = ([f for f in latest[DET] if f['world_objects'] is not None],
        dict(raw_bag=str(a.teammate_bag), sha256=sha(a.teammate_bag), startup_unbracketed_tf_stamps=excluded))
    sources['synthetic_matrix'] = (None, dict(active_densities=[0,1,100,500], measurements=[0,1,100]))
    for name, (frames, provenance) in sources.items():
        # Alternate trace order as well. State comparison ignores only timings.
        runs, costs = {}, {}
        for mode in ('baseline', 'candidate') if len(report['equivalence']) % 2 == 0 else ('candidate', 'baseline'):
            runs[mode], costs[mode] = trace(binaries[mode], frames, synthetic=frames is None)
            with gzip.open(a.output/(name+'_'+mode+'_trace.txt.gz'), 'wt') as handle:
                handle.write(runs[mode])
        identical = runs['baseline'] == runs['candidate']
        report['equivalence'][name] = dict(exact=identical, source=provenance,
            frames=len(frames) if frames is not None else 12,
            original_trace_sha256=hashlib.sha256(runs['baseline'].encode()).hexdigest(),
            candidate_trace_sha256=hashlib.sha256(runs['candidate'].encode()).hexdigest())
        if not identical:
            raise ValueError('Core state changed: '+name)
        report['real_trace_cost'][name] = {mode: dict(update_ms=stats([r['update_ms'] for r in rows]),
            prediction_ms=stats([r['prediction_ms'] for r in rows]), measurements=stats([r['measurements'] for r in rows]),
            active_tracks=stats([r['active'] for r in rows])) for mode, rows in costs.items()}
        with gzip.open(a.output/(name+'_costs.json.gz'), 'wt') as handle:
            json.dump(costs, handle)
        print('exact', name, report['equivalence'][name]['frames'], flush=True)
    accum = {mode: defaultdict(list) for mode in binaries}
    for i in range(a.rounds):
        order = ('baseline', 'candidate') if i % 2 == 0 else ('candidate', 'baseline')
        values = {}
        for mode in order:
            values[mode] = benchmarks(binaries[mode], a.repetitions)
            for key, costs in values[mode].items():
                accum[mode][key].extend(costs)
        report['benchmark_rounds'].append(dict(index=i, order=order,
            profiles={mode: {k: stats(v) for k, v in costs.items()} for mode, costs in values.items()}))
        print('benchmark round', i, order, flush=True)
    report['synthetic_benchmark'] = {key: {mode: stats(accum[mode][key]) for mode in binaries}
                                    for key in accum['baseline']}
    if receipt['source_sha256'] != sha(TRACKER/'src/algorithm/ekf_multi_object_tracking.cpp'):
        raise ValueError('Production core changed during read-only experiment')
    (a.output/'fastpath_report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
