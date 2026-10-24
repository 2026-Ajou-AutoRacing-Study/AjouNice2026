#!/usr/bin/env python3
"""Fixed-input, current-source Cruise replay, not a closed-loop planner certificate.

The original bag supplies optimizer trajectory, odometry and acceleration. Only
PredictedObjects is replaced. A private localhost ROS master provides read-only
parameter transport and diagnostic loopback; no installed binary is rebuilt,
canonical topic published, simulator started, or vehicle command generated.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
PLANNING = ROOT / 'planning_ws'
MODULE = PLANNING / 'src/autoware_motion_velocity_obstacle_cruise_module'
CONFIG = PLANNING / 'src/autoware_launch/config/ros1/planning/scenario_planning'
TRAJECTORY = '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory'
ODOMETRY = '/localization/kinematic_state'
ACCELERATION = '/localization/acceleration'
PREDICTION = '/perception/object_recognition/objects'
HARNESS = r'''
#include "obstacle_cruise_module.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
using namespace autoware::motion_velocity_planner;
using Json=nlohmann::json;
int main(int argc,char **argv) {
  ros::init(argc,argv,"shape_guard_cruise_harness");
  if(argc<3) return 2;
  ros::NodeHandle node("/shape_guard_cruise");
  ros::Time::setNow(ros::Time(1));
  ObstacleCruiseModule module;
  module.init(node,"shape_guard_cruise");
  auto data=std::make_shared<PlannerData>(node);
  Json selection; size_t diagnostics=0;
  auto sub=node.subscribe<std_msgs::String>("debug/obstacle_cruise/selection",10,
    [&](const std_msgs::String::ConstPtr &msg){selection=Json::parse(msg->data);++diagnostics;});
  const auto connect=ros::WallTime::now()+ros::WallDuration(.5);
  while(ros::WallTime::now()<connect){ros::spinOnce();ros::WallDuration(.002).sleep();}
  rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
  rosbag::View view(bag);
  std::ofstream output(argv[2]);
  size_t cases=0;
  for(const auto &msg:view) {
    if(msg.getTopic()=="/input/odom") data->current_odometry=*msg.instantiate<nav_msgs::Odometry>();
    else if(msg.getTopic()=="/input/accel")
      data->current_acceleration=*msg.instantiate<geometry_msgs::AccelWithCovarianceStamped>();
    else if(msg.getTopic()=="/input/objects")
      data->process_predicted_objects(*msg.instantiate<autoware_perception_msgs::PredictedObjects>());
    else if(msg.getTopic()=="/input/trajectory") {
      const auto trajectory=msg.instantiate<autoware_planning_msgs::Trajectory>();
      ros::Time::setNow(msg.getTime());
      data->is_driving_forward=autoware::motion_utils::isDrivingForwardWithTwist(
        trajectory->points).value_or(data->is_driving_forward);
      const auto before=diagnostics;
      const auto result=module.plan(trajectory->points,trajectory->points,data);
      const auto deadline=ros::WallTime::now()+ros::WallDuration(1.0);
      while(diagnostics==before && ros::WallTime::now()<deadline) {
        ros::spinOnce();ros::WallDuration(.0002).sleep();
      }
      if(diagnostics!=before+1) throw std::runtime_error("Missing/duplicate actual module diagnostic");
      Json row{{"index",cases++},{"decision_stamp",msg.getTime().toSec()},
        {"trajectory_stamp",trajectory->header.stamp.toSec()},
        {"selection",selection},{"stop_points",result.stop_points.size()},
        {"slowdown_intervals",result.slowdown_intervals.size()},
        {"clear_limit",bool(result.velocity_limit_clear_command)}};
      row["limit"]=nullptr;
      if(result.velocity_limit) {
        const auto &limit=*result.velocity_limit;
        row["limit"]={{"max_velocity",limit.max_velocity},{"use_constraints",limit.use_constraints},
          {"min_acceleration",limit.constraints.min_acceleration},
          {"max_acceleration",limit.constraints.max_acceleration},
          {"min_jerk",limit.constraints.min_jerk},{"max_jerk",limit.constraints.max_jerk}};
      }
      output<<row.dump()<<'\n';
    }
  }
  return cases ? 0 : 3;
}
'''


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    with Path(path).open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def wire(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    return buffer.getvalue()


def latest_index(times, stamp):
    """Only past receipts are legal; never use nearest/future observations."""
    from bisect import bisect_right
    index = bisect_right(times, stamp) - 1
    return index if index >= 0 else None


def source_audit(bag_path):
    import rosbag
    with rosbag.Bag(str(bag_path)) as bag:
        topics = {key: dict(type=info.msg_type, messages=info.message_count)
                  for key, info in bag.get_type_and_topic_info().topics.items()}
        result = dict(path=str(bag_path), sha256=sha(bag_path), start=bag.get_start_time(),
                      end=bag.get_end_time(), topics=topics)
    recorded_commit = 'e4e96ce8efe72606ae97e5e8d0774f0a9db66772'
    result['recorded_commit'] = recorded_commit
    result['recorded_commit_available_locally'] = subprocess.run(
        ['git', '-C', str(ROOT), 'cat-file', '-e', recorded_commit], capture_output=True).returncode == 0
    result['missing_exact_recording'] = ['full planner parameter snapshot', 'lanelet map binary',
        'LC candidate/state/approval diagnostics', 'rotary object/decision diagnostics']
    result['scope'] = ('Current-source fixed-optimizer-path Cruise module sensitivity; '
                       'not reproduction of the recorded executable or closed-loop vehicle behavior.')
    return result


def source_receipt():
    selected = list(MODULE.rglob('*.cpp')) + list(MODULE.rglob('*.hpp'))
    selected += list((PLANNING / 'src/autoware_motion_velocity_planner_common').rglob('*.cpp'))
    selected += list((PLANNING / 'src/autoware_motion_velocity_planner_common').rglob('*.hpp'))
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(selected)}


def catalog_input_audit(pack_path):
    """Check frozen 14-scene bags without inventing an ego path for sensor-only scenes."""
    import rosbag
    with gzip.open(pack_path, 'rt') as handle:
        pack = json.load(handle)
    result = dict(pack=str(pack_path), pack_sha256=sha(pack_path), scenes={})
    for name, case in pack['cases'].items():
        path = Path(case['raw_bag'])
        row = dict(bag=str(path), available=path.is_file())
        if path.is_file():
            with rosbag.Bag(str(path)) as bag:
                topics = bag.get_type_and_topic_info().topics
                row['fixed_cruise_inputs'] = {topic: topics[topic].message_count if topic in topics else 0
                                            for topic in (TRAJECTORY, ODOMETRY, ACCELERATION)}
                row['has_all_fixed_cruise_inputs'] = all(row['fixed_cruise_inputs'].values())
                row['map_binary_topics'] = [topic for topic, info in topics.items()
                                            if info.msg_type == 'autoware_map_msgs/LaneletMapBin']
                row['route_topics'] = [topic for topic, info in topics.items()
                                      if info.msg_type == 'autoware_planning_msgs/LaneletRoute']
        result['scenes'][name] = row
    result['scope'] = 'Availability only; absence is not evidence of a safe or unchanged planner decision.'
    return result


def compile_harness(directory):
    """Recompile actual Cruise sources into private objects; reuse dependency ABI only."""
    directory.mkdir(exist_ok=True)
    source = directory / 'cruise_harness.cpp'
    if not source.exists() or source.read_text() != HARNESS:
        source.write_text(HARNESS)
    build = PLANNING / 'build_isolated/autoware_motion_velocity_obstacle_cruise_module/CMakeFiles'
    flags = (build / 'autoware_motion_velocity_obstacle_cruise_module.dir/flags.make').read_text()
    includes = shlex.split(re.search(r'^CXX_INCLUDES = (.*)$', flags, re.M).group(1))
    sources = [source] + sorted((MODULE / 'src').rglob('*.cpp'))
    commands = []
    def compile_one(item):
        index, path = item
        obj = directory / ('source_%02d.o' % index)
        cmd = ['/usr/bin/c++', '-std=c++17', '-O1', '-DNDEBUG', '-w'] + includes + ['-c', str(path), '-o', str(obj)]
        commands.append(cmd)
        if obj.exists() and obj.stat().st_mtime_ns > path.stat().st_mtime_ns:
            return obj
        with (directory / ('compile_%02d.log' % index)).open('w') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
        return obj
    with ThreadPoolExecutor(max_workers=2) as workers:
        objects = list(workers.map(compile_one, enumerate(sources)))
    link = shlex.split((build / 'test_pid_target_switch.dir/link.txt').read_text())
    # Preserve dependency link flags, not the installed/possibly stale Cruise library or GTest.
    dependencies = link[link.index('-o') + 2:]
    dependencies = [arg for arg in dependencies if 'gtest' not in arg or arg.startswith('-Wl,')]
    dependencies = [arg for arg in dependencies if not arg.endswith('/libautoware_motion_velocity_obstacle_cruise_module.so')]
    binary = directory / 'cruise_harness'
    cmd = ['/usr/bin/c++'] + list(map(str, objects)) + ['-o', str(binary)] + dependencies + [
        '-L/opt/ros/noetic/lib', '-lrosbag', '-lrosbag_storage',
        '-L' + str(PLANNING / 'install_isolated/lib'), '-lautoware_utils_geometry',
        '-lautoware_utils_math', '-lautoware_utils_system', '-lautoware_utils_visualization',
        '-lvehicle_info_utils']
    with (directory / 'link.log').open('w') as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    commands.append(cmd)
    write_json(directory / 'build_receipt.json', dict(commands=commands, harness_sha256=sha(source),
        binary_sha256=sha(binary), actual_cruise_sources=[str(p) for p in sources[1:]],
        source_sha256=source_receipt(),
        dependency_sha256={arg: sha(arg) for arg in dependencies if arg.startswith('/') and '.so' in arg},
        dependency_abi='Existing local planning dependency shared libraries; not certified against recording.'))
    return binary


def verify_build(directory):
    """Reject a stale/tampered executable, source snapshot, harness, or linked dependency."""
    receipt = json.loads((directory / 'build_receipt.json').read_text())
    if receipt['binary_sha256'] != sha(directory / 'cruise_harness'):
        raise ValueError('Reused native binary SHA mismatch')
    if receipt['harness_sha256'] != hashlib.sha256(HARNESS.encode()).hexdigest():
        raise ValueError('Reused native harness does not match this analyzer')
    if sha(directory / 'cruise_harness.cpp') != receipt['harness_sha256']:
        raise ValueError('Reused harness source SHA mismatch')
    # v1 receipts kept the complete source snapshot alongside the build rather
    # than duplicating it in the build receipt; retain an equally strict check.
    sources = receipt.get('source_sha256')
    if sources is None:
        sources = json.loads((directory.parent / 'source_receipt.json').read_text())['source_sha256']
    if sources != source_receipt():
        raise ValueError('Reused binary was compiled against different planner sources')
    for path, digest in receipt['dependency_sha256'].items():
        if sha(path) != digest:
            raise ValueError('Native dependency SHA mismatch: ' + path)
    return dict(binary_sha256=receipt['binary_sha256'], source_snapshot_matches=True,
                dependency_sha256_checked=len(receipt['dependency_sha256']))


def validate_aa(report, comparison, before, after):
    if before not in report['modes'] or after not in report['modes']:
        raise ValueError('A/A pair is absent from the requested modes')
    left, right = report['modes'][before], report['modes'][after]
    if left['predicted_input_sha256'] != right['predicted_input_sha256']:
        raise ValueError('A/A prediction payload is not identical')
    if left['fixed_input_sha256'] != right['fixed_input_sha256'] or not comparison['exact_rows']:
        raise ValueError('A/A actual Cruise response was not exact; candidate comparison invalid')
    return dict(before=before, after=after, predicted_payload_exact=True, actual_cruise_response_exact=True)


def prepare_cases(source_bag, objects_bag, objects_topic, output):
    import rosbag
    streams = {ODOMETRY: [], ACCELERATION: [], TRAJECTORY: []}
    def deserialize(raw):
        return raw[-1]().deserialize(raw[1])
    with rosbag.Bag(str(source_bag)) as bag:
        for topic, raw, stamp in bag.read_messages(topics=list(streams), raw=True):
            streams[topic].append((stamp.to_nsec(), raw))
    objects = []
    with rosbag.Bag(str(objects_bag)) as bag:
        for _, raw, stamp in bag.read_messages(topics=[objects_topic], raw=True):
            if raw[0] != 'autoware_perception_msgs/PredictedObjects':
                raise ValueError('Expected actual PredictedObjects, not tracker rows or geometric proxy')
            objects.append((stamp.to_nsec(), raw))
    if not objects:
        raise ValueError('No prediction inputs')
    times = {key: [row[0] for row in rows] for key, rows in streams.items()}
    object_times = [row[0] for row in objects]
    if any(a > b for a, b in zip(object_times, object_times[1:])):
        raise ValueError('Prediction receive order regressed')
    fixed_hash, object_hash = hashlib.sha256(), hashlib.sha256()
    rows, skips = [], Counter()
    import genpy
    with rosbag.Bag(str(output), 'w') as bag:
        for stamp, raw_trajectory in streams[TRAJECTORY]:
            oi = latest_index(object_times, stamp)
            di = latest_index(times[ODOMETRY], stamp)
            ai = latest_index(times[ACCELERATION], stamp)
            if None in (oi, di, ai):
                skips['not_all_past_inputs_available'] += 1
                continue
            trajectory = deserialize(raw_trajectory)
            if len(trajectory.points) < 2:
                skips['short_optimizer_trajectory'] += 1
                continue
            odom, accel, predicted = (deserialize(streams[ODOMETRY][di][1]),
                                      deserialize(streams[ACCELERATION][ai][1]), deserialize(objects[oi][1]))
            if predicted.header.frame_id != 'map':
                raise ValueError('Cruise replay requires original map-frame predictions')
            fixed_hash.update(str(stamp).encode())
            for message in (odom, accel, trajectory):
                fixed_hash.update(wire(message))
            object_hash.update(wire(predicted))
            receipt = genpy.Time(stamp // 1000000000, stamp % 1000000000)
            for topic, message in (('/input/odom', odom), ('/input/accel', accel),
                                   ('/input/objects', predicted), ('/input/trajectory', trajectory)):
                bag.write(topic, message, receipt)
            rows.append(dict(stamp=stamp * 1.e-9, objects_stamp=predicted.header.stamp.to_sec(),
                             object_receipt_age_sec=(stamp - objects[oi][0]) * 1.e-9,
                             object_count=len(predicted.objects)))
    return dict(cases=len(rows), skipped=dict(skips), fixed_input_sha256=fixed_hash.hexdigest(),
                predicted_input_sha256=object_hash.hexdigest(), cases_metadata=rows)


def compare_rows(before, after):
    if len(before) != len(after) or any(a['decision_stamp'] != b['decision_stamp'] for a, b in zip(before, after)):
        raise ValueError('Planner decision cohort changed')
    changed = Counter()
    details, deltas = [], []
    for a, b in zip(before, after):
        kinds = []
        for name, left, right in (
            ('candidate_set', sorted(a['selection']['candidate_uuids']), sorted(b['selection']['candidate_uuids'])),
            ('selected_target', a['selection']['selected_uuids'], b['selection']['selected_uuids']),
            ('velocity_limit', a['limit'], b['limit']),
            ('clear_limit', a['clear_limit'], b['clear_limit'])):
            if left != right:
                changed[name] += 1
                kinds.append(name)
        if kinds:
            details.append(dict(index=a['index'], decision_stamp=a['decision_stamp'], changed=kinds,
                before_selected=a['selection']['selected_uuids'], after_selected=b['selection']['selected_uuids'],
                before_limit=a['limit'], after_limit=b['limit']))
        if a['limit'] is not None and b['limit'] is not None:
            deltas.append(b['limit']['max_velocity'] - a['limit']['max_velocity'])
    return dict(cases=len(before), exact_rows=before == after, changed=dict(changed), details=details,
                both_limited_cycles=len(deltas), lower_requested_limit_cycles=sum(v < 0 for v in deltas),
                higher_requested_limit_cycles=sum(v > 0 for v in deltas),
                requested_limit_delta_mps=dict(minimum=min(deltas), maximum=max(deltas),
                    mean=sum(deltas) / len(deltas)) if deltas else None,
                requested_limit_delta_distributions=velocity_delta_distributions(before, after),
                braking_scope='VelocityLimit request only; no smoother, final trajectory, actuator command or closed loop.')


def distribution(values):
    """Linear-interpolated quantiles, matching the usual NumPy percentile rule."""
    values = sorted(values)
    if not values:
        return dict(count=0)
    def percentile(fraction):
        position = fraction * (len(values) - 1)
        index = int(position)
        return values[index] + (values[min(index + 1, len(values) - 1)] - values[index]) * (position - index)
    return dict(count=len(values), mean=sum(values) / len(values), minimum=values[0],
                p50=percentile(.5), p90=percentile(.9), p95=percentile(.95), maximum=values[-1])


def velocity_delta_distributions(before, after):
    """Signed ON-minus-OFF request differences; never fabricate an absent speed cap."""
    if len(before) != len(after) or any(a['decision_stamp'] != b['decision_stamp'] for a, b in zip(before, after)):
        raise ValueError('Planner decision cohort changed')
    all_cycles, both_limited, changed = [], [], []
    presence_changes = 0
    for a, b in zip(before, after):
        if a['limit'] is None and b['limit'] is None:
            all_cycles.append(0.)  # No request changed; not a commanded zero-speed limit.
        elif a['limit'] is None or b['limit'] is None:
            presence_changes += 1
        else:
            delta = b['limit']['max_velocity'] - a['limit']['max_velocity']
            both_limited.append(delta)
            all_cycles.append(delta)
            if delta:
                changed.append(delta)
    def signed_absolute(values):
        return dict(signed_mps=distribution(values), absolute_mps=distribution([abs(v) for v in values]))
    return dict(changed_velocity_limit_cycles=signed_absolute(changed),
                both_limited_cycles=signed_absolute(both_limited),
                all_decision_cycles=signed_absolute(all_cycles),
                limit_presence_change_cycles=presence_changes,
                denominator=('All-decision differences count two absent limits as zero change, NOT a 0m/s command. '
                             'A limit-presence transition has no numeric delta and is counted separately. '
                             'Signed delta is candidate minus baseline; no final control/braking claim.'))


def summarize_rows(rows, bag_start):
    """Actual diagnostic reasons plus bounded geometric event windows, not actor GT."""
    import math
    limits = [r['limit']['max_velocity'] for r in rows if r['limit'] is not None]
    reasons = Counter(o['filter_reason'] for r in rows for o in r['selection']['inputs'])
    episodes = {}
    for name, start, end, center in (('obstacle_0134', 29.6, 31.4, [71., 403.]),
                                     ('truck_0156', 117.25, 118.25, [202., 643.])):
        selected = [r for r in rows if start <= r['decision_stamp'] - bag_start <= end]
        objects = [dict(decision_offset_sec=r['decision_stamp'] - bag_start, **o)
                   for r in selected for o in r['selection']['inputs']
                   if math.dist([o['x'], o['y']], center) < 8.]
        episodes[name] = dict(decision_cycles=len(selected), objects_near_event=objects,
            filter_reasons=dict(Counter(o['filter_reason'] for o in objects)),
            actual_selected_object_samples=sum(o['selected'] for o in objects),
            note='8m geometric window is descriptive correspondence, not an identity/GT oracle.')
    return dict(cycles=len(rows), filter_reasons=dict(reasons),
                limit_cycles=len(limits), minimum_requested_velocity_limit_mps=min(limits) if limits else None,
                maximum_requested_velocity_limit_mps=max(limits) if limits else None, episodes=episodes)


def parameters():
    paths = [ROOT / 'configs/planning/vehicle_info.yaml', CONFIG / 'common/common.param.yaml',
             CONFIG / 'common/nearest_search.param.yaml',
             CONFIG / 'lane_driving/motion_planning/motion_velocity_planner/motion_velocity_planner.param.yaml',
             CONFIG / 'lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml']
    result = {}
    for path in paths:
        result.update(yaml.safe_load(path.read_text()))
    return result, {str(path.relative_to(ROOT)): sha(path) for path in paths}


def private_environment(output):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    if port == 11311:
        raise RuntimeError('Never connect to the live ROS master')
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
               ROS_HOSTNAME='127.0.0.1', ROS_HOME=str(output / 'ros_home'),
               ROS_LOG_DIR=str(output / 'ros_home/log'))
    return port, env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--objects', action='append', default=[], metavar='MODE=BAG')
    parser.add_argument('--objects-topic', default='/shape_guard/predicted_objects')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--build', type=Path, help='Reuse an earlier private harness build')
    parser.add_argument('--catalog', type=Path, help='Also audit original bags of the frozen 14-scene core pack')
    parser.add_argument('--aa-pair', help='Required in run mode: two identical-input modes, e.g. off_a,off_b')
    parser.add_argument('--require-ready-signals', action='store_true',
                        help='Wait for sibling replay summary.json and successful owned-process cleanup.json')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    audit = source_audit(args.bag)
    write_json(args.output / 'input_audit.json', audit)
    if args.catalog:
        write_json(args.output / 'catalog_input_audit.json', catalog_input_audit(args.catalog))
    initial = source_receipt()
    params, config_sha = parameters()
    write_json(args.output / 'source_receipt.json', dict(source_sha256=initial, config_sha256=config_sha,
                                                      parameter_values=params))
    if not args.run:
        print(json.dumps(dict(audit=str(args.output / 'input_audit.json'))))
        return
    modes = dict(entry.split('=', 1) for entry in args.objects)
    if len(modes) != len(args.objects) or not modes or any(not re.fullmatch('[a-z][a-z0-9_]*', mode) for mode in modes):
        raise ValueError('Unique safe MODE=BAG inputs required')
    aa_pair = args.aa_pair.split(',') if args.aa_pair else []
    if len(aa_pair) != 2 or aa_pair[0] == aa_pair[1] or not all(mode in modes for mode in aa_pair):
        raise ValueError('Run requires --aa-pair with two distinct requested identical-input modes')
    binary = args.build / 'cruise_harness' if args.build else compile_harness(args.output / 'build')
    if args.build and not binary.exists():
        binary = compile_harness(args.build)
    if not binary.is_file():
        raise FileNotFoundError(binary)
    build_validation = verify_build(binary.parent)
    port, env = private_environment(args.output)
    master_log = (args.output / 'master.log').open('w')
    master = subprocess.Popen(['/opt/ros/noetic/bin/roscore', '-p', str(port)], env=env,
                              stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
    report = dict(scope=audit['scope'], private_master_uri=env['ROS_MASTER_URI'],
                  build_validation=build_validation, modes={})
    try:
        import xmlrpc.client
        server = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 20
        while True:
            try:
                if server.getUri('/shape_guard_orchestrator')[0] == 1:
                    break
            except OSError:
                pass
            if master.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Private master failed')
            time.sleep(.05)
        server.setParam('/shape_guard_orchestrator', '/use_sim_time', True)
        for name, value in params.items():
            server.setParam('/shape_guard_orchestrator', '/shape_guard_cruise/' + name, value)
        for mode, path in modes.items():
            provenance = dict(bag=str(path), topic=args.objects_topic)
            if args.require_ready_signals:
                parent = Path(path).parent
                deadline = time.monotonic() + 900
                last_notice = 0.
                while not all((parent / name).is_file() for name in ('summary.json', 'cleanup.json')):
                    if time.monotonic() > deadline:
                        raise RuntimeError('Timed out waiting for closed upstream replay: ' + path)
                    if time.monotonic() - last_notice > 15:
                        print('Waiting for closed, cleaned replay: ' + mode, flush=True)
                        last_notice = time.monotonic()
                    time.sleep(.2)
                upstream = json.loads((parent / 'summary.json').read_text())
                cleanup = json.loads((parent / 'cleanup.json').read_text())
                if not cleanup['exited'] or not all(cleanup['exited']) or cleanup['production_touched']:
                    raise ValueError('Upstream replay has not completed isolated cleanup')
                if upstream['clock_mode'] != 'receipt':
                    raise ValueError('Original receipt-paced replay is required for this comparison')
                provenance.update(clock_mode=upstream['clock_mode'],
                    summary_sha256=sha(parent / 'summary.json'), bag_sha256=sha(path),
                    input_state_fingerprint=upstream['input_state_fingerprint'],
                    schedule_fingerprint=upstream['schedule_fingerprint'])
            directory = args.output / mode
            directory.mkdir()
            cases = prepare_cases(args.bag, Path(path), args.objects_topic, directory / 'cases.bag')
            write_json(directory / 'case_receipt.json', cases)
            with (directory / 'native.log').open('w') as log:
                subprocess.run([str(binary), str(directory / 'cases.bag'), str(directory / 'rows.jsonl')],
                               env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
            rows = [json.loads(line) for line in (directory / 'rows.jsonl').read_text().splitlines()]
            if len(rows) != cases['cases']:
                raise RuntimeError('Incomplete actual Cruise execution')
            write_json(directory / 'actual_response_summary.json', summarize_rows(rows, audit['start']))
            report['modes'][mode] = dict(cases=cases['cases'], fixed_input_sha256=cases['fixed_input_sha256'],
                predicted_input_sha256=cases['predicted_input_sha256'],
                rows_sha256=sha(directory / 'rows.jsonl'), limit_cycles=sum(r['limit'] is not None for r in rows),
                selected_cycles=sum(bool(r['selection']['selected_uuids']) for r in rows),
                input_provenance=provenance)
        names = list(modes)
        if len({r['fixed_input_sha256'] for r in report['modes'].values()}) != 1:
            raise RuntimeError('Non-object fixed inputs changed')
        before = [json.loads(line) for line in (args.output / names[0] / 'rows.jsonl').read_text().splitlines()]
        for mode in names[1:]:
            after = [json.loads(line) for line in (args.output / mode / 'rows.jsonl').read_text().splitlines()]
            write_json(args.output / ('comparison_' + names[0] + '_' + mode + '.json'), compare_rows(before, after))
        aa_rows = [[json.loads(line) for line in (args.output / mode / 'rows.jsonl').read_text().splitlines()]
                   for mode in aa_pair]
        report['aa_gate'] = validate_aa(report, compare_rows(*aa_rows), *aa_pair)
        if verify_build(binary.parent) != build_validation:
            raise RuntimeError('Native build changed during qualification')
        if source_receipt() != initial:
            raise RuntimeError('Planner source changed during qualification')
        report['complete'] = True
        write_json(args.output / 'report.json', report)
        print(json.dumps(report))
    finally:
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGTERM)
                master.wait(timeout=5)
        master_log.close()


if __name__ == '__main__':
    main()
