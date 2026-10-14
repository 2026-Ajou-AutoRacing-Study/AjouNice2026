#!/usr/bin/env python3
"""Build actual EKF with an isolated, causal early direction output guard.

No installed source or production defaults are modified. Associated center
history uses the existing native observer with state fusion/output override
disabled. Guard OFF/ON share identical core states, association and timing.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess

import yaml
from audit_tracking_continuity_events import sha

ROOT = Path(__file__).resolve().parents[2]
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'
HARNESS = r'''
#include "algorithm/ekf_multi_object_tracking.hpp"
#include "vehicle_motion_direction_guard.hpp"
#include <iomanip>
#include <memory>
#include <unordered_set>
int main(int argc, char** argv) {
    ros::Time::init(); std::cout << std::setprecision(17);
    const bool enabled = argc > 1 && std::stoi(argv[1]) != 0;
    MultiClassObjectTrackingConfig config;
    config.prediction_model = mc_mot::PredictionModel::CTRV;
SETTINGS
    config.center_motion_velocity_fusion = true;
    config.center_motion_fuse_ekf_state = false;
    config.center_motion_override_output_velocity = false;
    auto tracker = std::make_unique<EkfMultiObjectTracking>(config);
    ajou_evaluation::DirectionGuardConfig guard_config;
    guard_config.measurement_time_decision = argc > 2 && std::stoi(argv[2]) != 0;
    ajou_evaluation::VehicleMotionDirectionGuard guard(guard_config);
    double time = 0.0, previous = 0.0; int count = 0;
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
        int n = 0; std::unordered_set<int> active;
        for (const auto& t : result.track) if (t.is_init) { ++n; active.insert(t.track_id); }
        std::cout << "FRAME " << time << " " << n << "\n";
        for (const auto& t : result.track) if (t.is_init) {
            const auto policy = guard.Apply(t.track_id, time, t.initialization_time,
                t.state_vec(S_YAW), t.state_vec(S_VX), t.state_vec(S_VY),
                t.getRepClass() == mc_mot::ObjectClass::CAR || t.getRepClass() == mc_mot::ObjectClass::TRUCK,
                t.center_history);
            std::cout << "TRACK " << t.track_id << " " << t.getRepClass() << " "
                << t.is_confirmed << " " << t.is_associated << " "
                << t.state_vec(S_X) << " " << t.state_vec(S_Y) << " " << t.object_z << " "
                << (enabled ? policy.vx : t.state_vec(S_VX)) << " "
                << (enabled ? policy.vy : t.state_vec(S_VY)) << " " << t.state_vec(S_YAW) << " "
                << t.dimension.length << " " << t.dimension.width << " " << t.dimension.height << " "
                << t.detection_confidence << " " << t.last_measurement_time << " " << t.age << " "
                << t.state_vec(S_YAW_RATE) << " " << t.initialization_time << "\n";
            std::cout << "GUARD " << time << " " << t.track_id << " " << policy.guarded << " "
                << policy.fit_valid << " " << policy.trusted_stamp << " " << policy.fit_residual_m << "\n";
        }
        guard.RetainOnly(active); previous = time;
    }
}
'''


def build(output, params, source=None):
    if output.exists():
        raise FileExistsError('Fresh output directory required')
    output.mkdir(parents=True)
    source = source or TRACKER / 'src/algorithm/ekf_multi_object_tracking.cpp'
    header = TRACKER / 'include/algorithm/ekf_multi_object_tracking.hpp'
    guard = Path(__file__).with_name('vehicle_motion_direction_guard.hpp')
    inputs = {str(p): sha(p) for p in (source, header, guard)}
    fields = set(re.findall(r'^\s+(?:bool|double|int)\s+(\w+)\s*(?:\{|=)', header.read_text(), re.M))
    settings = ['    config.{} = {};'.format(k, str(v).lower() if isinstance(v, bool) else v)
                for k, v in params.items() if k in fields and isinstance(v, (bool, int, float))]
    harness = output / 'harness.cpp'
    harness.write_text(HARNESS.replace('SETTINGS', '\n'.join(settings)))
    binary = output / 'direction_probe'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    command = ['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3', '-I' + str(TRACKER / 'include'),
               '-I' + str(guard.parent), str(harness), str(source), '-o', str(binary), *flags]
    subprocess.run(command, check=True)
    if inputs != {str(p): sha(Path(p)) for p in inputs}:
        raise RuntimeError('Inputs changed during isolated build')
    receipt = dict(input_sha256=inputs, command=command, binary_sha256=sha(binary),
                   parameters=params, enabled_center_history_observer_only=True,
                   production_modified=False, scope='actual measurement-time core, output policy only; no ROS scheduler/planner')
    (output / 'build.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return binary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--core-source', type=Path,
                        help='Explicit isolated candidate source; never installs it')
    args = parser.parse_args()
    parameters = yaml.safe_load((TRACKER / 'config/config.yaml').read_text())['configure']
    parameters['pedestrian_gaussian_association_ranking'] = True
    print(build(args.output, parameters, args.core_source))
