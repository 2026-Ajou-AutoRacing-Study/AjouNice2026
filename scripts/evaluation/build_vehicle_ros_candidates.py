#!/usr/bin/env python3
"""Build isolated real ROS wrappers for vehicle output/association research.

Never installs binaries, edits the tracker submodule, or changes its defaults.
The runtime guard switch is private to these explicitly built research nodes.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

from audit_tracking_continuity_events import sha

ROOT = Path(__file__).resolve().parents[2]
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Wrapper source anchor changed: ' + old[:80])
    return text.replace(old, new, 1)


def build(output, innovation_source):
    if output.exists():
        raise FileExistsError('Fresh isolated build directory required')
    wrapper = TRACKER / 'src/ekf_multi_object_tracking_node.cpp'
    header = TRACKER / 'include/ekf_multi_object_tracking_node.hpp'
    core = TRACKER / 'src/algorithm/ekf_multi_object_tracking.cpp'
    algorithm = TRACKER / 'include/algorithm/ekf_multi_object_tracking.hpp'
    canonical = TRACKER / 'include/vehicle_orientation_canonicalizer.hpp'
    input_receipt = TRACKER / 'include/detection_input_receipt.hpp'
    guard = Path(__file__).with_name('vehicle_motion_direction_guard.hpp')
    inputs = {str(p): sha(p) for p in (wrapper, header, core, algorithm, canonical, input_receipt, guard, innovation_source)}
    output.mkdir(parents=True)
    include = output / 'include'
    (include / 'algorithm').mkdir(parents=True)
    (include / 'algorithm' / algorithm.name).write_text(algorithm.read_text())
    (include / canonical.name).write_text(canonical.read_text())
    (include / input_receipt.name).write_text(input_receipt.read_text())
    (include / guard.name).write_text(guard.read_text())
    header_text = replace_once(header.read_text(), '#include "vehicle_orientation_canonicalizer.hpp"',
        '#include "vehicle_orientation_canonicalizer.hpp"\n#include "vehicle_motion_direction_guard.hpp"')
    header_text = replace_once(header_text, '    vehicle_orientation::Canonicalizer vehicle_orientation_canonicalizer_;',
        '    vehicle_orientation::Canonicalizer vehicle_orientation_canonicalizer_;\n'
        '    ajou_evaluation::VehicleMotionDirectionGuard research_direction_guard_;\n'
        '    bool research_direction_guard_enabled_{false};\n'
        '    ros::Publisher research_guard_publisher_;\n'
        '    ros::Publisher research_arrival_publisher_, research_cycle_publisher_;\n'
        '    autoware_perception_msgs::TrackedObjects research_guarded_objects_;')
    header_text = replace_once(header_text, '        detection_input_receipt_.Commit();',
        '        detection_input_receipt_.Commit();\n'
        '        research_arrival_publisher_.publish(msg->header);')
    (include / header.name).write_text(header_text)
    source = wrapper.read_text()
    source = replace_once(source, '    ProcessYAML();', '    ProcessYAML();\n'
        '    private_nh.param<bool>("research_direction_guard", research_direction_guard_enabled_, false);\n'
        '    bool measurement_clock = false;\n'
        '    private_nh.param<bool>("research_guard_measurement_clock", measurement_clock, false);\n'
        '    ajou_evaluation::DirectionGuardConfig guard_config;\n'
        '    guard_config.measurement_time_decision = measurement_clock;\n'
        '    research_direction_guard_ = ajou_evaluation::VehicleMotionDirectionGuard(guard_config);\n'
        '    research_guard_publisher_ = private_nh.advertise<autoware_perception_msgs::TrackedObjects>(\n'
        '        "diagnostics/direction_guarded_objects", 20);\n'
        '    research_arrival_publisher_ = private_nh.advertise<std_msgs::Header>("diagnostics/input_arrived", 20);\n'
        '    research_cycle_publisher_ = private_nh.advertise<std_msgs::Header>("diagnostics/cycle_complete", 20);')
    source = replace_once(source, '    o_tracked_objects_.objects.clear();',
        '    o_tracked_objects_.objects.clear();\n'
        '    research_guarded_objects_.header = o_tracked_objects_.header;\n'
        '    research_guarded_objects_.objects.clear();')
    source = replace_once(source, '        const auto canonical = vehicle_orientation_canonicalizer_.Apply(',
        '        bool research_guarded = false;\n'
        '        if (research_direction_guard_enabled_) {\n'
        '            const auto guarded = research_direction_guard_.Apply(\n'
        '                track.track_id, track_structs.time_stamp, track.initialization_time,\n'
        '                track.state_vec(S_YAW), output_velocity_x, output_velocity_y,\n'
        '                is_vehicle, track.center_history);\n'
        '            output_velocity_x = guarded.vx; output_velocity_y = guarded.vy;\n'
        '            research_guarded = guarded.guarded;\n'
        '        }\n'
        '        const auto canonical = vehicle_orientation_canonicalizer_.Apply(')
    source = replace_once(source, '        o_tracked_objects_.objects.push_back(output);',
        '        o_tracked_objects_.objects.push_back(output);\n'
        '        if (research_guarded) research_guarded_objects_.objects.push_back(output);')
    source = replace_once(source, '    vehicle_orientation_canonicalizer_.RetainOnly(active_track_ids);',
        '    vehicle_orientation_canonicalizer_.RetainOnly(active_track_ids);\n'
        '    research_direction_guard_.RetainOnly(active_track_ids);')
    source = replace_once(source, '        p_track_objects_.publish(o_tracked_objects_);',
        '        p_track_objects_.publish(o_tracked_objects_);\n'
        '        research_guard_publisher_.publish(research_guarded_objects_);')
    source = replace_once(source, '        Publish();',
        '        Publish();\n'
        '        std_msgs::Header cycle; cycle.stamp = ros::Time::now();\n'
        '        research_cycle_publisher_.publish(cycle);')
    isolated_wrapper = output / 'research_node.cpp'
    isolated_wrapper.write_text(source)
    cmake = ROOT / 'base_ws/build_aw149/ekf_multi_object_tracking/CMakeFiles/ekf_multi_object_tracking.dir'
    values = {}
    for line in (cmake / 'flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1); values[key] = shlex.split(value)
    link = shlex.split((cmake / 'link.txt').read_text())
    libraries = link[link.index('-o') + 2:]
    flags = values['CXX_DEFINES'] + values['CXX_INCLUDES']
    flags = [f for f in flags if f != '-I' + str(TRACKER / 'include')]
    flags = ['-I' + str(include), *flags, '-O2', '-std=c++17', '-DNDEBUG']
    commands = []
    wrapper_object = output / 'research_node.o'
    command = ['/usr/bin/c++', *flags, '-c', str(isolated_wrapper), '-o', str(wrapper_object)]
    subprocess.run(command, check=True); commands.append(command)
    binaries = {}
    for mode, selected_source in [('baseline', core), ('innovation', innovation_source)]:
        obj = output / (mode + '_core.o')
        command = ['/usr/bin/c++', *flags, '-c', str(selected_source), '-o', str(obj)]
        subprocess.run(command, check=True); commands.append(command)
        binary = output / (mode + '_node')
        command = ['/usr/bin/c++', '-rdynamic', str(wrapper_object), str(obj), '-o', str(binary), *libraries]
        subprocess.run(command, check=True); commands.append(command)
        binaries[mode] = str(binary.resolve())
    if inputs != {p: sha(Path(p)) for p in inputs}:
        raise RuntimeError('Inputs changed during isolated build')
    receipt = dict(inputs_sha256=inputs, binaries=binaries,
        binaries_sha256={k: sha(Path(v)) for k, v in binaries.items()}, commands=commands,
        scope='Isolated actual ROS node + native core. Private guard opt-in; no install or production mutation.',
        production_modified=False)
    (output / 'build_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--innovation-source', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.output, args.innovation_source), indent=2))
