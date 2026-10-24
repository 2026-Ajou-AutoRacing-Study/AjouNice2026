#!/usr/bin/env python3
"""Development-only, stationary MORAI traffic-light evidence capture.

Run in the sourced ROS environment. Owns only its two roslaunch children and a
brake/P publisher, never starts planning or driving. JSON commands on stdin:
{"action":"status"}, {"action":"capture","name":"sunny13_front"},
{"action":"load","source":"/absolute/scenario.json","pose":{...}},
{"action":"stop"}. Captures are observations, not traffic-state ground truth.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
ALLOWED_OUTPUT_ROOT = ROOT / 'artifacts/evaluation'
sys.path[:0] = [str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'),
               str(ROOT / 'base_ws/src/control/trajectory_batch_runner/src')]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def normalized_scene(scene, pose=None):
    # Preserve native scene schema; stationary ego only. Never mutate source.
    scene = json.loads(json.dumps(scene))
    if not isinstance(scene, dict) or not isinstance(scene.get('egoVehicle'), dict):
        raise ValueError('Scene must contain egoVehicle')
    ego = scene['egoVehicle']
    initial = ego.get('initPosition')
    if not isinstance(initial, dict) or not isinstance(initial.get('pos'), dict):
        raise ValueError('Scene must contain an ego initPosition/pos')
    if pose is not None and (not isinstance(pose, dict) or set(pose) - {'x','y','z','yaw'}):
        raise ValueError('Pose must contain only x/y/z/yaw')
    pose = pose or {}
    def finite(value):
        if isinstance(value, bool):
            raise ValueError('Boolean is not a pose coordinate')
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError('Pose coordinates must be numeric') from error
        if not math.isfinite(number):
            raise ValueError('Non-finite pose')
        return number
    # Validate the complete effective pose, not only explicitly overridden axes.
    for axis in ('x', 'y', 'z'):
        value = finite(pose.get(axis, initial['pos'].get(axis)))
        initial['pos'][axis] = value
        initial['pos']['_' + axis] = str(value)
    rotation = initial.get('rot')
    if not isinstance(rotation, dict) or 'yaw' not in rotation:
        raise ValueError('Scene must contain ego rotation/yaw')
    yaw = finite(pose.get('yaw', rotation['yaw']))
    if 'yaw' in pose:
        rotation['yaw'] = str(yaw)
    ego.update(beforeControlMode=2,currentControlMode=2,currentVelocity=0.0,customValue=0.0)
    initial['gear'] = 1
    saved = initial.get('vehicleSaveData', {})
    if not isinstance(saved, dict) or not isinstance(saved.get('vehicle', {}), dict):
        raise ValueError('vehicleSaveData/vehicle must be objects')
    for container in (ego, initial, saved, saved.get('vehicle', {})):
        for key in ('velocity', 'lastVelocity', 'currentVelocity'):
            if key in container:
                value = container[key]
                container[key] = ({axis: 0.0 for axis in value} if isinstance(value, dict) else 0.0)
    if isinstance(saved.get('inputData'), list) and len(saved['inputData']) > 6:
        saved['inputData'][6] = 1
    for key in ('vehicleList', 'pedestrianList', 'obstacleList', 'objectList', 'spawnPointList'):
        if key in scene:
            scene[key] = []
    return scene


def validate_output_directory(output, allowed_root=ALLOWED_OUTPUT_ROOT):
    requested = Path(output).expanduser()
    resolved = requested.resolve()
    try:
        resolved.relative_to(Path(allowed_root).absolute())
    except ValueError:
        raise ValueError('Output must remain below artifacts/evaluation')
    if requested.exists() or requested.is_symlink() or resolved.exists():
        raise FileExistsError('Output directory must be fresh')
    return resolved


def validate_scenario_source(source, scenario_dir):
    source = Path(source).resolve(strict=True)
    directory = Path(scenario_dir).resolve(strict=True)
    if not directory.is_dir() or source.parent != directory or source.suffix != '.json' or not source.is_file():
        raise ValueError('Source must be a JSON file directly inside the selected simulator map folder')
    return source


def validate_idle_timeout(value):
    if not math.isfinite(value) or not 0 < value <= 3600:
        raise ValueError('Idle timeout must be finite and in (0, 3600] seconds')
    return value


class CommandLineReader:
    """Raw fd reads plus a complete-line queue; never mix select with readline.

    poll returns ('line', text), ('timeout', None), or ('eof', None). Queued lines
    are delivered before another select, including when one write batches JSONL.
    EOF is sticky and a final unterminated line is delivered once before EOF.
    """
    def __init__(self, fd, max_line_bytes=1024 * 1024):
        self.fd = fd
        self.max_line_bytes = max_line_bytes
        self.pending = bytearray()
        self.lines = deque()
        self.eof = False

    def poll(self, timeout):
        if self.lines:
            return 'line', self.lines.popleft().decode('utf-8')
        if self.eof:
            return 'eof', None
        readable, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not readable:
            return 'timeout', None
        chunk = os.read(self.fd, 65536)
        if not chunk:
            self.eof = True
            if self.pending:
                line = bytes(self.pending)
                self.pending.clear()
                return 'line', line.decode('utf-8')
            return 'eof', None
        self.pending.extend(chunk)
        while b'\n' in self.pending:
            line, _, remainder = self.pending.partition(b'\n')
            if len(line) > self.max_line_bytes:
                raise ValueError('Stdin command exceeds maximum line size')
            self.lines.append(bytes(line))
            self.pending = bytearray(remainder)
        if len(self.pending) > self.max_line_bytes:
            raise ValueError('Stdin command exceeds maximum line size')
        if self.lines:
            return 'line', self.lines.popleft().decode('utf-8')
        return 'timeout', None


def cleanup_owned(hold, processes, handles, stationary, status, event, stop_process, shutdown):
    """Always attempt every owned cleanup; return failures without fabricating a stop."""
    failures = []
    def record(value):
        try:
            event(value)
        except Exception as error:
            failures.append('cleanup event: ' + str(error))
    if hold is not None:
        try:
            receipt = stationary()
            record({'event':'shutdown_stationary','stationary_verified':True,
                    'stationary':receipt,'status':status()})
        except Exception as error:
            failures.append('shutdown stationary gate: ' + str(error))
            record({'event':'shutdown_stationary_failed','stationary_verified':False,'error':str(error)})
        try:
            hold.stop()
        except Exception as error:
            failures.append('brake hold cleanup: ' + str(error))
    for process in reversed(processes):
        try:
            stop_process(process)
        except Exception as error:
            failures.append('owned process cleanup: ' + str(error))
    for handle in handles:
        try:
            handle.close()
        except Exception as error:
            failures.append('owned log cleanup: ' + str(error))
    try:
        shutdown('Owned stationary probe finished')
    except Exception as error:
        failures.append('ROS shutdown: ' + str(error))
    if failures:
        record({'event':'cleanup_failed','stationary_verified':False,'errors':list(failures)})
    return failures


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scenario-dir', required=True, type=Path)
    parser.add_argument('--idle-timeout', type=float, default=3600)
    parser.add_argument('--bootstrap-scene', type=Path,
        help='Explicit stationary-scene/network restore before initial telemetry gate')
    return parser


def main():
    parser = build_argument_parser()
    args = parser.parse_args()
    try:
        args.output = validate_output_directory(args.output)
        validate_idle_timeout(args.idle_timeout)
        if not args.scenario_dir.is_dir():
            raise ValueError('Scenario directory must exist')
        args.scenario_dir = args.scenario_dir.resolve()
        if args.bootstrap_scene is not None:
            args.bootstrap_scene = validate_scenario_source(args.bootstrap_scene, args.scenario_dir)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.output.mkdir(parents=True)
    import rospy
    from sensor_msgs.msg import CompressedImage
    from planning_scenario_validator.runtime import LiveMonitor, BrakeHold, stop_group
    from trajectory_batch_runner.morai_udp_client import build_scenario_load_payload
    network = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json'
    sensor = ROOT / 'configs/sensors/Sensor_competition_local.json'
    rospy.init_node('traffic_light_stationary_probe', disable_signals=True)
    publishers = dict(rospy.get_master().getSystemState()[2][0])
    if publishers.get('/ctrl_cmd'):
        raise RuntimeError('Another control publisher exists; nothing started')
    # Refuse existing gateway nodes even before they have produced a topic.
    import rosnode
    conflicts = set(rosnode.get_node_names()) & {
        '/morai_ctrl_cmd_udp_sender', '/morai_sensor_udp_receiver', '/morai_ego_udp_receiver'}
    if conflicts:
        raise RuntimeError('Gateway already owned: ' + repr(conflicts))
    processes, handles = [], []
    hold = None
    monitor = LiveMonitor()
    monitor.events_path = args.output / 'safety_events.jsonl'
    latest = {}
    rospy.Subscriber('/sensor/camera/front/compressed', CompressedImage,
        lambda msg: latest.update(message=msg, receipt=time.monotonic()), queue_size=1)
    def event(value):
        value['wall_time'] = time.time()
        with (args.output / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(value, ensure_ascii=False) + '\n')
        print(json.dumps(value, ensure_ascii=False), flush=True)
    def status():
        ego = monitor.ego
        return {'gear': monitor.gear, 'control_mode': monitor.control_mode,
                'speed_mps': monitor.raw_ego_speed_mps() if ego else None,
                'position': ({k: getattr(ego.position, k) for k in ('x','y','z')} if ego else None),
                'heading': ego.heading if ego else None,
                'ego_receipt_age_sec': (time.monotonic()-monitor.stamps['ego'] if 'ego' in monitor.stamps else None),
                'image_receipt_age_sec': (time.monotonic() - latest['receipt'] if latest else None)}
    def stationary():
        try:
            return monitor.wait_stationary(after=time.monotonic(), required=('ego','gear','control_mode'),
                check_preparation_collision=False, timeout=12, hold_sec=1.0)
        except Exception as error:
            event({'event':'stationary_gate_failed','stationary_verified':False,'error':str(error)})
            raise
    def stage_send(source, pose=None):
        source = validate_scenario_source(source, args.scenario_dir)
        original = source.read_bytes()
        scene = normalized_scene(json.loads(original), pose)
        stem = 'tlp_' + uuid.uuid4().hex[:16]
        payload = json.dumps(scene, ensure_ascii=False, indent=2, allow_nan=False).encode()
        installed = args.scenario_dir / (stem + '.json')
        with installed.open('xb') as stream:
            stream.write(payload)
        with (args.output / (stem + '.json')).open('xb') as stream:
            stream.write(payload)
        sidecar = args.scenario_dir / 'EgoNetwork' / (stem + '_MN.json')
        with sidecar.open('xb') as stream:
            stream.write(network.read_bytes())
        request = {'delete_all':False, 'load_network_connection_data':True,
            'load_ego_vehicle_data':True, 'load_surrounding_vehicle_data':True,
            'load_pedestrian_data':True, 'load_obstacle_data':True, 'set_pause':False}
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            load_after = time.monotonic()
            sock.sendto(build_scenario_load_payload(stem, request), ('127.0.0.1',9095))
        event({'event':'load_requested','source':str(source),'source_sha256':sha(original),
            'installed':str(installed),'installed_sha256':sha(payload),
            'requested_pose':scene['egoVehicle']['initPosition']['pos']})
        return scene, load_after
    try:
        commands = [
            ['roslaunch','sensor_interface','morai_udp_interface.launch',
             'repo_root:=' + str(ROOT), 'network_config:=' + str(network),
             'sensor_config:=' + str(sensor), 'enable_lidar:=false','enable_control:=false'],
            ['roslaunch','planning_scenario_validator','parking_hold.launch',
             'repo_root:=' + str(ROOT), 'network_config:=' + str(network)]]
        for index, command in enumerate(commands):
            handle = (args.output / ('launch_%d.log' % index)).open('ab', buffering=0)
            handles.append(handle)
            processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        hold = BrakeHold()
        hold.start()
        if args.bootstrap_scene:
            stage_send(args.bootstrap_scene)
        receipt = stationary()
        event({'event':'ready','stationary_verified':True,'stationary':receipt,'status':status(),
               'network_sha256':sha(network.read_bytes()), 'sensor_sha256':sha(sensor.read_bytes())})
        last_command = time.monotonic()
        reader = CommandLineReader(sys.stdin.fileno())
        while time.monotonic() - last_command < args.idle_timeout:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Owned gateway exited')
            kind, line = reader.poll(min(.5, max(0.0, args.idle_timeout - (time.monotonic() - last_command))))
            if kind == 'timeout':
                continue
            if kind == 'eof':
                event({'event':'stdin_eof'})
                break
            last_command = time.monotonic()
            try:
                command = json.loads(line)
                if not isinstance(command, dict):
                    raise ValueError('Command must be a JSON object')
                action = command['action']
                if action == 'stop':
                    event({'event':'stop_requested'})
                    break
                if action == 'status':
                    event({'event':'status', **status()})
                elif action == 'load':
                    stationary()
                    scene, load_after = stage_send(command['source'], command.get('pose'))
                    # Wait for measured pose, not merely an old stationary packet.
                    target = scene['egoVehicle']['initPosition']['pos']
                    deadline = time.monotonic() + 15
                    while time.monotonic() < deadline:
                        ego = monitor.ego
                        if (ego and monitor.stamps.get('ego',0) > load_after and
                            math.hypot(ego.position.x-float(target['x']), ego.position.y-float(target['y'])) < 1):
                            break
                        time.sleep(.1)
                    else:
                        raise RuntimeError('Scenario pose readback failed')
                    event({'event':'loaded','requested_pose':scene['egoVehicle']['initPosition']['pos'],
                        'stationary':stationary(),'status':status()})
                elif action == 'capture':
                    receipt = stationary()
                    name = command['name']
                    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
                        raise ValueError('Use a simple capture name')
                    image = latest.copy()
                    if not image or time.monotonic()-image['receipt'] > 1:
                        raise RuntimeError('No fresh camera image')
                    msg = image['message']
                    data = bytes(msg.data)
                    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                        raise RuntimeError('Expected original JPEG')
                    target = args.output / (name + '.jpg')
                    with target.open('xb') as stream:
                        stream.write(data)
                    record = {'event':'captured', 'name':name,'image_path':str(target),
                        'sha256':sha(data),'header_stamp':{'secs':msg.header.stamp.secs,'nsecs':msg.header.stamp.nsecs},
                        'format':msg.format,'stationary':receipt,'status':status(),
                        'human_gt':None,'weather_annotation':command.get('weather_annotation','UNKNOWN')}
                    with target.with_suffix('.json').open('x') as stream:
                        json.dump(record,stream,ensure_ascii=False,indent=2)
                    event(record)
                else:
                    raise ValueError('Unknown action')
            except Exception as error:
                event({'event':'command_failed','error':str(error)})
        else:
            event({'event':'idle_timeout','idle_timeout_sec':args.idle_timeout})
    except BaseException as error:
        try:
            event({'event':'probe_failed','error':str(error),'stationary_verified':False})
        except Exception:
            pass
        raise
    finally:
        active_error = sys.exc_info()[0] is not None
        failures = cleanup_owned(hold, processes, handles, stationary, status, event,
                                 stop_group, rospy.signal_shutdown)
        if failures and not active_error:
            raise RuntimeError('Probe cleanup did not verify a normal stationary finish: ' + '; '.join(failures))


if __name__ == '__main__':
    main()
