#!/usr/bin/env python3
"""Standalone full-stack developer recording, with original sensor messages."""
import argparse
import datetime
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[3]
TOPICS = [
    r'^/sensor/(gps|imu|lidar[^/]*(/.*)?)$',
    r'^/sensor/camera/[^/]+/(compressed(/.*)?|camera_info)$',
    r'^/(Ego_topic|Competition_topic|Object_topic|CollisionData|VehicleCollisionData|IntscnTL_topic|GetTrafficLightStatus)$',
    r'^/localization(/.*)?$', r'^/competition/raw_twist$',
    r'^/perception/object_recognition/(detection/objects|tracking/measurement_objects|tracking/objects|objects)$',
    r'^/perception/(bevfusion|base5_trial)/(status|latency_ms|frame_timing)$',
    r'^/perception/traffic_light_recognition(/.*)?$',
    r'^/traffic_light_node/debug_(selection|classifier|runtime|policy_reason|observed_label|raw_label)$',
    r'^/(map_based_prediction|tracked_object_motion_policy)/diagnostics(/.*)?$',
    r'^/perception/object_recognition/tracking/diagnostics(/.*)?$',
    r'^/planning/mission_planning/route$', r'^/planning/route_state$',
    r'^/map/vector_map$', r'^/system/operation_mode/state$',
    r'^/vehicle/status/(velocity_status|steering_status)$',
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
    '/sensor/lidar1', '/sensor/lidar1/packets', '/sensor/imu', '/sensor/gps',
    *['/sensor/camera/'+name+'/compressed' for name in ('front', 'front_left', 'front_right', 'back')],
    '/localization/kinematic_state', '/planning/trajectory', '/ctrl_cmd',
    '/perception/object_recognition/objects',
    '/perception/object_recognition/detection/objects',
    '/perception/object_recognition/tracking/measurement_objects',
    '/perception/object_recognition/tracking/objects',
]

PARAMETERS = (
    '/use_sim_time', '/configure', '/topic_name', '/ekf_multi_object_tracking',
    '/lanelet2_map_loader', '/map_based_prediction',
    '/tracked_object_motion_policy', '/purepursuit_autoware', '/longi_pi_autoware', '/localization',
    '/lidar/lidar_driver', '/lidar/lidar_transform', '/bevfusion_ros_bridge',
    '/base5_trial/bevfusion_ros_bridge', '/traffic_light_node', '/traffic_light_map_roi',
    '/traffic_light_route_group_id',
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner',
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner',
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner',
    '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer',
    '/planning/scenario_planning/velocity_smoother',
)


def local_module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
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


class SnapshotTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.
        if connection.sock is not None:
            connection.sock.settimeout(2.)
        return connection


def bounded_ros_readers(master_uri):
    """Read-only Master/parameter API with per-call timeout, no ROS node needed."""
    proxy = xmlrpc.client.ServerProxy(master_uri, transport=SnapshotTransport(), allow_none=True)
    caller = '/development_bag_recorder'
    class Reader:
        master_uri = ''
        def call(self, method, *args):
            code, _, value = getattr(proxy, method)(caller, *args)
            if code != 1:
                raise RuntimeError('ROS snapshot request failed: ' + method)
            return value
        def getPid(self):
            return self.call('getPid')
        def getPublishedTopics(self, namespace):
            return self.call('getPublishedTopics', namespace)
        def getSystemState(self):
            return self.call('getSystemState')
        def get_param(self, name):
            code, _, value = proxy.getParam(caller, name)
            if code == -1:
                raise KeyError(name)
            if code != 1:
                raise RuntimeError('ROS parameter snapshot failed')
            return value
    reader = Reader()
    reader.master_uri = master_uri
    return reader


def audit(path, patterns):
    return local_module('development_recording_audit').audit(path, patterns, REQUIRED, DIAGNOSTICS)


def snapshot(master, rospy):
    """Only read ROS state. Never change a parameter, publisher, or node."""
    result = dict(captured_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(), parameters={}, errors={})
    # Optional, isolated and hard-bounded. Failure never gates bag or driving.
    try:
        result['loaded_binary_receipts'] = local_module('loaded_library_receipt').collect_bounded(
            getattr(master, 'master_uri', None))
    except Exception as error:
        result['loaded_binary_receipts'] = dict(status='collector_unavailable',
                                                targets={}, error_type=type(error).__name__)
    for name in PARAMETERS:
        try:
            result['parameters'][name] = scrub(rospy.get_param(name))
        except KeyError:
            result['parameters'][name] = None
        except Exception as error:
            result['errors'][name] = type(error).__name__
            if isinstance(error, (OSError, EOFError, xmlrpc.client.ProtocolError)):
                result['errors']['snapshot_incomplete'] = 'Master unavailable; remaining reads skipped'
                return result
    for field, method in (('advertised_topics', master.getPublishedTopics),
                          ('system_state', master.getSystemState)):
        try:
            result[field] = method('/') if field == 'advertised_topics' else method()
        except Exception as error:
            result['errors'][field] = type(error).__name__
    return result


def telemetry_command(directory, session_id, args):
    command = [sys.executable, str(Path(__file__).with_name('record_host_telemetry.py')),
               '--output', str(directory / 'host_algorithm.jsonl'), '--role', 'algorithm',
               '--session-id', session_id, '--interval', str(args.host_interval)]
    for port in sorted(set(args.udp_port or [2368])):
        command += ['--udp-port', str(port)]
    for pid in args.telemetry_pid:
        command += ['--pid', str(pid)]
    if args.no_host_gpu:
        command.append('--no-gpu')
    return command


def stop_child(process):
    """Stop only our bounded telemetry subprocess, never a driving process."""
    if process is None:
        return None
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGTERM)
        except ProcessLookupError:
            pass  # Child may finish between poll and signal; still reap it.
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except ProcessLookupError:
                pass
            process.wait()
            return dict(exit_code=process.returncode, forced=True)
    return dict(exit_code=process.returncode, forced=False)


def recorder_log_summary(path):
    result = dict(warning_lines=0, error_lines=0, drop_warning_lines=0)
    for line in path.read_text(errors='replace').splitlines():
        lower = line.lower()
        result['warning_lines'] += int('warn' in lower)
        result['error_lines'] += int('error' in lower)
        result['drop_warning_lines'] += int('dropping' in lower or 'buffer exceeded' in lower)
    result['note'] = 'Log warnings are not an exact frame/packet loss counter.'
    return result


def telemetry_audit(path):
    result = dict(file=str(path.name), samples=0, invalid_lines=0, ended=False)
    if not path.exists():
        return dict(result, status='missing')
    for line in path.read_text(errors='replace').splitlines():
        try:
            row = json.loads(line)
            kind = row.get('record_type')
            result['samples'] += int(kind == 'sample')
            result['ended'] |= kind == 'end'
        except (ValueError, AttributeError):
            result['invalid_lines'] += 1
    result['status'] = 'recorded' if result['samples'] else 'no_samples'
    return result


def diagnostic_readiness(state):
    advertised = {item[0] for item in state.get('advertised_topics', [])}
    checks = local_module('development_recording_audit').capabilities(
        {name: {'messages': 1} for name in advertised})
    # This snapshot is NOT a bag count. Keep this distinction in both key and note.
    return dict(advertised_groups={name: check['recorded'] for name, check in checks.items()},
                note='Advertisement only: a topic may never publish. The post-recording audit '
                     'checks actual message counts. Missing diagnostics never stop the driving stack.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=0, help='Seconds; 0 records until Ctrl-C')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'rosbags/eval/development')
    parser.add_argument('--include-decoded-images', action='store_true', help='Also save Image topics; increases disk/CPU load')
    parser.add_argument('--minimum-free-gib', type=float, default=10)
    parser.add_argument('--list', action='store_true', help='Print regexes only; no ROS or files')
    parser.add_argument('--audit-only', type=Path, help='Print closed bag audit JSON; no master, nodes, or files created')
    parser.add_argument('--check', action='store_true', help='Read-only live ROS advertisement/parameter check; no recording or files')
    parser.add_argument('--session-id', help='Shared label for algorithm and MORAI host logs (letters/numbers/._-)')
    parser.add_argument('--no-host-telemetry', action='store_true', help='Disable this PC resource/UDP sidecar')
    parser.add_argument('--host-interval', type=float, default=1., help='Host sampling interval, seconds (default 1)')
    parser.add_argument('--no-host-gpu', action='store_true', help='Do not query nvidia-smi')
    parser.add_argument('--telemetry-pid', type=int, action='append', default=[], help='Optional process CPU/RSS PID; repeatable')
    parser.add_argument('--udp-port', type=int, action='append', help='Socket drop/queue port; repeatable, default 2368')
    args = parser.parse_args()
    if args.duration < 0 or not 2 <= args.minimum_free_gib <= 100000:
        parser.error('duration >= 0 and minimum-free-gib >= 2 are required')
    if not math.isfinite(args.host_interval) or not .25 <= args.host_interval <= 60:
        parser.error('host-interval must be finite and 0.25..60 seconds')
    if args.session_id and not re.fullmatch(r'[A-Za-z0-9_.-]{1,80}', args.session_id):
        parser.error('session-id must use 1..80 letters/numbers/._-')
    if any(pid <= 0 for pid in args.telemetry_pid) or any(not 1 <= p <= 65535 for p in args.udp_port or []):
        parser.error('telemetry-pid must be positive and udp-port must be 1..65535')
    patterns = TOPICS + ([r'^/sensor/camera/[^/]+(/image_raw)?$'] if args.include_decoded_images else [])
    if args.list:
        print('\n'.join(patterns))
        return 0
    if args.audit_only:
        print(json.dumps(audit(args.audit_only.expanduser().resolve(), patterns), ensure_ascii=False, indent=2))
        return 0
    import rosgraph
    master = bounded_ros_readers(rosgraph.Master('/development_bag_recorder').master_uri)
    master.getPid()  # Fail before creating files when ROS is unavailable.
    if args.check:
        state = snapshot(master, master)
        print(json.dumps(dict(readiness=diagnostic_readiness(state), ros_state=state),
                         ensure_ascii=False, indent=2))
        return 0
    parent = args.output_dir.expanduser().resolve()
    existing = parent
    while not existing.exists():
        existing = existing.parent
    if shutil.disk_usage(existing).free < args.minimum_free_gib * 1024**3:
        parser.error('Not enough free disk; recording was not started')
    session_id = args.session_id or ('lan_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
    directory = parent / ('dev_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    directory.mkdir(parents=True, exist_ok=False)
    bag_path = directory/'development.bag'
    cmd = ['rosbag', 'record', '--lz4', '--buffsize=1024', '--regex', '-O', str(bag_path)]
    if args.duration:
        cmd.append('--duration='+str(args.duration))
    cmd += patterns
    def git(*parts):
        try:
            return subprocess.run(['git', '-C', str(ROOT), *parts], capture_output=True,
                                  text=True, timeout=5).stdout.strip()
        except subprocess.TimeoutExpired:
            return '[git snapshot timeout]'
    state = snapshot(master, master)
    readiness = diagnostic_readiness(state)
    write_json(directory/'ros_state_start.json', state)
    provenance = {}
    for file in (Path(__file__), Path(__file__).with_name('development_recording_audit.py'),
                 Path(__file__).with_name('loaded_library_receipt.py'),
                 Path(__file__).with_name('record_host_telemetry.py'), ROOT/'docs/common_alias.txt',
                 ROOT/'configs/perception/bevfusion_base5_production.yaml',
                 ROOT/'base_ws/src/sensor_interface/launch/lidar.launch'):
        provenance[str(file.relative_to(ROOT))] = hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None
    host_command = None if args.no_host_telemetry else telemetry_command(directory, session_id, args)
    write_json(directory/'metadata.json', dict(command=cmd, session_id=session_id, git_commit=git('rev-parse', 'HEAD'),
        git_status=git('status', '--short'), source_sha256=provenance,
        loaded_binary_receipts=state.get('loaded_binary_receipts'),
        advertised_topics=state.get('advertised_topics'), parameters=state['parameters'],
        diagnostic_advertisement_readiness=readiness,
        host_telemetry_command=host_command,
        raw_images='Original CompressedImage bytes; no re-encoding. Decoded images opt-in.',
        started_at=datetime.datetime.now().astimezone().isoformat(),
        limits='ROS parameters and source hashes alone do not identify installed binaries. Optional '
               'loaded_binary_receipts identify allowlisted local backing disk files only, not memory '
               'bytes, active branches or GPU backend. Clock alignment and source physical timestamps '
               'are unverified.'))
    print('개발용 녹화 폴더: '+str(directory), flush=True)
    print('양쪽 PC 공통 session-id: ' + session_id, flush=True)
    print('원본 압축 카메라·LiDAR PointCloud/패킷·GPS/IMU 포함. recorder.log의 Recording을 확인한 뒤 주행하세요.', flush=True)
    print('Ctrl-C로 마감. 디스크 2GiB 미만이면 녹화만 종료하며 차량/스택은 제어하지 않습니다.', flush=True)
    print('최신 노드는 녹화 구독에 따라 상세 진단을 자동 활성화합니다. 별도 시작 옵션이 필요하지 않습니다.', flush=True)
    print('이미 켠 주행 노드를 이 녹화기가 재시작하거나 설정 변경하지 않습니다.', flush=True)
    for name in ('detector_stage_timing', 'postprocessing_stage_timing', 'tracker_assignment_trace'):
        if not readiness['advertised_groups'][name]:
            print('WARNING: ' + name + ' 진단 토픽 미광고: 구버전 또는 미실행 노드일 수 있습니다. 녹화는 계속하며 주행 노드는 재시작하지 않습니다.', flush=True)
    previous = {}
    stopping = False
    host_process = None
    host_start_error = None
    host_result = None
    with (directory/'recorder.log').open('w') as log:
        if host_command:
            try:
                with (directory/'host_telemetry.log').open('w') as host_log:
                    host_process = subprocess.Popen(host_command, stdout=host_log, stderr=subprocess.STDOUT)
            except OSError as error:
                host_start_error = type(error).__name__
                print('WARNING: 호스트 계측 실행 실패; bag 녹화는 계속합니다.', flush=True)
        try:
            process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            stop_child(host_process)
            raise
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
            host_result = stop_child(host_process)
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    write_json(directory/'ros_state_end.json', snapshot(master, master))
    try:
        report = audit(bag_path, patterns)
        report['recorder_exit'] = result
        report['recorder_log'] = recorder_log_summary(directory/'recorder.log')
        report['host_telemetry'] = dict(telemetry_audit(directory/'host_algorithm.jsonl'),
                                       process=host_result, start_error=host_start_error,
                                       disabled=args.no_host_telemetry)
        write_json(directory/'recording_audit.json', report)
        print('누락 core: '+str(report['missing_core']), flush=True)
        print('누락 진단: '+str(report['missing_diagnostics']), flush=True)
        for name, status in report['analysis_capabilities'].items():
            print('기록 ' + name + ': ' + ('있음' if status['recorded'] else '부족/미발행'), flush=True)
        print('호스트 계측: ' + report['host_telemetry']['status'] + '. MORAI PC 로그는 별도 수집해 같은 폴더로 전달하세요.', flush=True)
        print('마감 완료: '+str(bag_path), flush=True)
    except Exception as error:
        write_json(directory/'recording_audit.json', dict(audit_failed=str(error), recorder_exit=result))
        print('WARNING: bag 검사 실패. 원본/active/log를 보존했습니다: '+str(error), flush=True)
        return 1
    return 0 if result in (0, -signal.SIGINT, 130) else 1


if __name__ == '__main__':
    raise SystemExit(main())
