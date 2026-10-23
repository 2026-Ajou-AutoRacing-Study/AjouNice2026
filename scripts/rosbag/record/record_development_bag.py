#!/usr/bin/env python3
"""Standalone full-stack developer recording, with original sensor messages."""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]
TOPICS = [
    r'^/sensor/(gps|imu|lidar[^/]*(/.*)?)$',
    r'^/sensor/camera/[^/]+/(compressed(/.*)?|camera_info)$',
    r'^/(Ego_topic|Competition_topic|Object_topic|CollisionData|VehicleCollisionData|IntscnTL_topic|GetTrafficLightStatus)$',
    r'^/localization(/.*)?$', r'^/competition/raw_twist$',
    r'^/perception/object_recognition/(detection/objects|tracking/measurement_objects|tracking/objects|objects)$',
    r'^/perception/(bevfusion|base5_trial)/(status|latency_ms|frame_timing)$',
    r'^/perception/traffic_light_recognition(/.*)?$',
    r'^/(map_based_prediction|tracked_object_motion_policy)/diagnostics(/.*)?$',
    r'^/planning/mission_planning/route$', r'^/planning/route_state$',
    r'^/planning/scenario_planning/.*(path|path_with_lane_id|trajectory|velocity_limit|clear_velocity_limit)$',
    r'^/planning/trajectory$', r'^/planning/validation_status$',
    r'^/planning/planning_factors/.*$',
    r'^/planning/(cooperate_status|auto_mode_status|debug/objects_of_interest)/.*$',
    r'^/planning/.*/debug/.*(safety_checks|processing_time.*|calculation_time|processing_time_ms_diag)$',
    r'^/planning/.*/processing_time_ms$',
    r'^/planning/.*/debug/obstacle_cruise/(selection|input_trajectory|planning_info)$',
    r'^/planning/.*/debug/roundabout/(ego_ttc|object_ttc)$',
    r'^/(control|purepursuit_autoware)(/.*)?$', r'^/ctrl_cmd$',
    r'^/(tf|tf_static|clock|diagnostics|rosout|rosout_agg)$',
]
REQUIRED = [
    '/sensor/lidar1', '/sensor/imu', '/sensor/gps',
    *['/sensor/camera/'+name+'/compressed' for name in ('front', 'front_left', 'front_right', 'back')],
    '/localization/kinematic_state', '/planning/trajectory', '/ctrl_cmd',
    '/perception/object_recognition/objects',
]
DIAGNOSTICS = [
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_left/safety_checks',
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_right/safety_checks',
    '/planning/planning_factors/obstacle_cruise',
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_cruise/selection',
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_cruise/input_trajectory',
]


def scrub(value):
    if isinstance(value, dict):
        return {k: ('[REDACTED]' if re.search(r'token|password|secret|credential|api.key', str(k), re.I)
                    else scrub(v)) for k, v in value.items()}
    if isinstance(value, list):
        return [scrub(v) for v in value]
    return value


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def audit(path, patterns):
    import rosbag
    with rosbag.Bag(str(path)) as bag:
        topics = {k: dict(messages=v.message_count, type=v.msg_type)
                  for k, v in bag.get_type_and_topic_info().topics.items()}
        duration = bag.get_end_time()-bag.get_start_time()
    missing = lambda names: [n for n in names if topics.get(n, {}).get('messages', 0) == 0]
    return dict(duration_sec=duration, topics=topics,
                missing_core=missing(REQUIRED), missing_diagnostics=missing(DIAGNOSTICS),
                unmatched_patterns=[p for p in patterns if not any(re.match(p, t) for t in topics)],
                note='Message counts prove recording, not driving safety or sensor synchronization. '
                     'Optional/inactive/old-node diagnostics may be absent; see recorder.log for buffer drops.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=0, help='Seconds; 0 records until Ctrl-C')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'rosbags/eval/development')
    parser.add_argument('--include-decoded-images', action='store_true', help='Also save Image topics; increases disk/CPU load')
    parser.add_argument('--minimum-free-gib', type=float, default=10)
    parser.add_argument('--list', action='store_true', help='Print regexes only; no ROS or files')
    args = parser.parse_args()
    if args.duration < 0 or not 2 <= args.minimum_free_gib <= 100000:
        parser.error('duration >= 0 and minimum-free-gib >= 2 are required')
    patterns = TOPICS + ([r'^/sensor/camera/[^/]+(/image_raw)?$'] if args.include_decoded_images else [])
    if args.list:
        print('\n'.join(patterns))
        return 0
    import rosgraph
    import rospy
    master = rosgraph.Master('/development_bag_recorder')
    master.getPid()  # Fail before creating files when ROS is unavailable.
    parent = args.output_dir.expanduser().resolve()
    existing = parent
    while not existing.exists():
        existing = existing.parent
    if shutil.disk_usage(existing).free < args.minimum_free_gib * 1024**3:
        parser.error('Not enough free disk; recording was not started')
    directory = parent / ('dev_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    directory.mkdir(parents=True, exist_ok=False)
    bag_path = directory/'development.bag'
    cmd = ['rosbag', 'record', '--lz4', '--buffsize=1024', '--regex', '-O', str(bag_path)]
    if args.duration:
        cmd.append('--duration='+str(args.duration))
    cmd += patterns
    def git(*parts):
        return subprocess.run(['git', '-C', str(ROOT), *parts], capture_output=True, text=True).stdout.strip()
    parameters = {}
    for name in ('/use_sim_time', '/lanelet2_map_loader', '/map_based_prediction',
                 '/tracked_object_motion_policy', '/purepursuit_autoware',
                 '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner',
                 '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner'):
        try:
            parameters[name] = scrub(rospy.get_param(name))
        except KeyError:
            parameters[name] = None
    write_json(directory/'metadata.json', dict(command=cmd, git_commit=git('rev-parse', 'HEAD'),
        git_status=git('status', '--short'), advertised_topics=master.getPublishedTopics('/'),
        parameters=parameters, raw_images='Original CompressedImage bytes; no re-encoding. Decoded images opt-in.',
        started_at=datetime.datetime.now().astimezone().isoformat()))
    print('개발용 녹화 폴더: '+str(directory), flush=True)
    print('원본 압축 카메라·LiDAR PointCloud/패킷·GPS/IMU 포함. recorder.log의 Recording을 확인한 뒤 주행하세요.', flush=True)
    print('Ctrl-C로 마감. 디스크 2GiB 미만이면 녹화만 종료하며 차량/스택은 제어하지 않습니다.', flush=True)
    previous = {}
    stopping = False
    with (directory/'recorder.log').open('w') as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def stop(signum=None, frame=None):
            nonlocal stopping
            if not stopping and process.poll() is None:
                stopping = True
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, stop)
        try:
            with (directory/'recorder.log').open() as monitor:
                while process.poll() is None:
                    for line in monitor:
                        if any(word in line.lower() for word in ('recording to', 'warn', 'error', 'dropping')):
                            print(line.rstrip(), flush=True)
                    if shutil.disk_usage(directory).free < 2*1024**3:
                        print('WARNING: 디스크 부족으로 녹화 마감. 차량은 직접 안전하게 정지하세요.', flush=True)
                        stop()
                    time.sleep(.5)
            result = process.returncode
        finally:
            stop()
            process.wait()  # Do not force-kill an unfinalized bag.
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    try:
        report = audit(bag_path, patterns)
        write_json(directory/'recording_audit.json', report)
        print('누락 core: '+str(report['missing_core']), flush=True)
        print('누락 진단: '+str(report['missing_diagnostics']), flush=True)
        print('마감 완료: '+str(bag_path), flush=True)
    except Exception as error:
        write_json(directory/'recording_audit.json', dict(audit_failed=str(error), recorder_exit=result))
        print('WARNING: bag 검사 실패. 원본/active/log를 보존했습니다: '+str(error), flush=True)
        return 1
    return 0 if result in (0, -signal.SIGINT, 130) else 1


if __name__ == '__main__':
    raise SystemExit(main())
