#!/usr/bin/env python3
"""Shared perception RViz layout with frozen E12 results and Lanelet map.

Only offline visualization support is started. No inference, tracking, control,
UDP, live localization, or MORAI. Raw bags are strictly topic-whitelisted.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'
SENSOR_TOPICS = (
    '/sensor/lidar1', '/sensor/camera/front/compressed',
    '/sensor/camera/back/compressed', '/sensor/camera/front_left/compressed',
    '/sensor/camera/front_right/compressed',
)
MARKER_NAMES = ('gt', 'detections', 'tracks', 'velocity', 'prediction3',
                'prediction5', 'prefix5', 'gt_future', 'hud')
MARKER_TOPICS = tuple('/e12_view/' + name for name in MARKER_NAMES)
FROZEN_TF = '/e12_view/tf'
COMMON_RVIZ = ROOT / 'base_ws/src/perception_object_pipeline/rviz/object_pipeline_live.rviz'


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('A finite positive number is required')
    return result


def scene_paths(case):
    return OUTPUT / 'rviz_common_v1' / case


def read_scene(path):
    scene = json.loads(path.read_text())
    if scene.get('schema') != 'e12_postprocessing_common_rviz_scene_v1' or scene.get('complete') is not True:
        raise ValueError('A completed E12 viewer export is required')
    for key in ('raw_bag', 'marker_bag'):
        value = Path(scene[key])
        if not value.is_absolute() or not value.is_file():
            raise ValueError(f'{key} is not an existing absolute file: {value}')
    if scene['fixed_frame'] != 'map':
        raise ValueError('The common RViz requires the audited map-coordinate export')
    if set(scene['topics']) != set(MARKER_TOPICS + (FROZEN_TF,)):
        raise ValueError('Unexpected marker topic contract')
    for key in ('start_offset_sec', 'duration_sec'):
        value = float(scene[key])
        if not math.isfinite(value) or value < 0 or (key == 'duration_sec' and value == 0):
            raise ValueError(f'Invalid {key}')
    if scene.get('marker_bag_sha256'):
        digest = hashlib.sha256()
        with Path(scene['marker_bag']).open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024**2), b''):
                digest.update(block)
        if digest.hexdigest() != scene['marker_bag_sha256']:
            raise ValueError('Marker export changed; use a fresh export rather than the frozen receipt')
    # This is an offline view check, not a production model startup gate.
    for relative in ('configs/sensors/Sensor_competition_lan.json',
                     'configs/tf/morai_lanelet_static_tf.yaml'):
        path = ROOT / relative
        expected = scene.get('source_sha256', {}).get(str(path))
        if expected and hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Offline overlay/static-TF configuration differs: ' + relative)
    return scene


def play_command(scene, rate, autoplay=False, duration=None):
    # A short sensor pre-roll avoids losing the first marker when rosbag's
    # topic-filtered view starts after the raw bag's first (excluded) message.
    start = max(0., scene['start_offset_sec'] - .25)
    length = scene['duration_sec'] + scene['start_offset_sec'] - start
    length = min(length, duration) if duration else length
    command = ['rosbag', 'play', '--clock', '--rate', str(rate),
               '--start', str(start), '--duration', str(length)]
    if not autoplay:
        command.append('--pause')
    if duration is None:
        command.append('--keep-alive')
    # Remap ONLY the derived frozen TF; never replay /tf from the raw bag.
    command += [scene['raw_bag'], scene['marker_bag'], FROZEN_TF + ':=/tf', '--topics']
    return command + list(SENSOR_TOPICS) + list(MARKER_TOPICS) + [FROZEN_TF]


def rviz_configuration(frame='map'):
    if frame != 'map':
        raise ValueError('Common view uses map, not a sensor-fixed camera')
    config = yaml.safe_load(COMMON_RVIZ.read_text())
    topics = {
        '/perception/object_recognition/visualization/detections': 'detections',
        '/perception/object_recognition/visualization/tracks': 'tracks',
        '/perception/object_recognition/visualization/predictions': 'prediction3',
        '/evaluation/visualization/gt_boxes': 'gt',
        '/evaluation/visualization/gt_paths': 'gt_future',
    }

    def remap(displays):
        for display in displays:
            name = topics.get(display.get('Marker Topic'))
            if name:
                display['Marker Topic'] = '/e12_view/' + name
                display['Namespaces'] = {'e12_' + name: True}
                if name == 'tracks':
                    display['Name'] = 'Tracks (cyan / coast orange)'
                elif name == 'gt_future':
                    display['Name'] = 'Matched GT Future Paths (white)'
            remap(display.get('Displays', []))

    manager = config['Visualization Manager']
    remap(manager['Displays'])
    # Preserve shared camera docks, map, vehicle, follower, zoom and layout.
    optional = [('velocity', 'Velocity arrows'), ('prediction5', 'Candidate 5s (orange)'),
                ('prefix5', 'Prefix-stable 5s RESEARCH (blue)'), ('hud', 'Replay notes')]
    manager['Displays'].append(dict(Class='rviz/Group', Name='E12 comparison (optional)',
        Enabled=False, Displays=[dict(Class='rviz/MarkerArray', Name=title, Enabled=True,
            **{'Marker Topic': '/e12_view/' + name, 'Namespaces': {'e12_' + name: True},
               'Queue Size': 5, 'Value': True}) for name, title in optional]))
    config['Preferences']['PromptSaveOnExit'] = False
    return config


def support_launch():
    """Reuse shared map/vehicle launch files, with numeric TF from config."""
    launch = ET.Element('launch')
    lanelet = ET.SubElement(launch, 'include', file='$(find lanelet_map_visualizer)/launch/kcity_lanelet_map.launch')
    ET.SubElement(lanelet, 'arg', name='publish_vector_map', value='false')
    vehicle = ET.SubElement(launch, 'include', file='$(find ioniq5)/launch/display_ioniq5.launch')
    ET.SubElement(vehicle, 'arg', name='start_rviz', value='false')

    def static(name, parent, child, xyz, rpy):
        args = ' '.join(str(value) for value in [*xyz, *reversed(rpy), parent, child])
        ET.SubElement(launch, 'node', pkg='tf2_ros', type='static_transform_publisher',
            name=name, args=args, output='screen')

    alignment = yaml.safe_load((ROOT / 'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    static('e12_map_to_morai', alignment['parent'], alignment['child'], alignment['xyz'], alignment['rpy'])
    sensor = json.loads((ROOT / 'configs/sensors/Sensor_competition_lan.json').read_text())
    for item in sensor['cameraList'] + sensor['Lidar3DList']:
        cfg = item.get('cc', item.get('lc'))
        child = cfg['rosConfig']['frameID']
        static('e12_base_to_' + child, 'base_link', child,
            [item['pos'][axis] for axis in ('x', 'y', 'z')],
            [math.radians(float(item['rot'][axis])) for axis in ('roll', 'pitch', 'yaw')])
    return ET.tostring(launch, encoding='unicode')


BOOTSTRAP_CLOCK = '''
import sys, time, rospy
from rosgraph_msgs.msg import Clock
rospy.init_node('e12_rviz_bootstrap_clock', disable_signals=True)
publisher = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
start = time.monotonic()
while time.monotonic()-start < 30 and not rospy.is_shutdown():
    publisher.publish(Clock(clock=rospy.Time.from_sec(float(sys.argv[1])+time.monotonic()-start)))
    time.sleep(.025)
'''


def private_environment(directory, port):
    env = dict(os.environ)
    env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
               ROS_HOME=str(directory / 'ros_home'), ROS_LOG_DIR=str(directory / 'ros_logs'),
               DISABLE_ROS1_EOL_WARNINGS='1')
    env.pop('ROS_HOSTNAME', None)
    return env


def stop_owned(process, group=False):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            if group:
                os.killpg(process.pid, sig)
            else:
                process.send_signal(sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def run(scene, case, rate, headless=False, autoplay=False, duration=None):
    for executable in ('roscore', 'rosbag', 'rosparam', 'roslaunch') + (() if headless else ('rviz',)):
        if not shutil.which(executable):
            raise RuntimeError(f'{executable} not found; use the .sh wrapper to source ROS')
    if not headless and not os.environ.get('DISPLAY'):
        raise RuntimeError('DISPLAY is empty. Run this in a local desktop terminal.')
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory = OUTPUT / 'rviz_sessions' / f'{case}_{stamp}_{os.getpid()}'
    directory.mkdir(parents=True, exist_ok=False)
    config = directory / 'view.rviz'
    config.write_text(yaml.safe_dump(rviz_configuration(scene['fixed_frame']), sort_keys=False))
    launch = directory / 'support.launch'
    launch.write_text(support_launch())
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = private_environment(directory, port)
    command = play_command(scene, rate, autoplay, duration)
    (directory / 'session.json').write_text(json.dumps(dict(
        case=case, rate=rate, scene=scene, command=command,
        master=env['ROS_MASTER_URI'], common_rviz_source=str(COMMON_RVIZ),
        live_stack_started=False), indent=2))
    core = rviz = player = support = bootstrap = None
    try:
        with (directory / 'roscore.log').open('w') as log:
            core = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = False
        for _ in range(50):
            if core.poll() is not None:
                raise RuntimeError(f'Private roscore exited; see {directory}')
            try:
                probe = subprocess.run(['rosparam', 'list'], env=env, timeout=1,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if probe.returncode == 0:
                    ready = True
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(.1)
        if not ready:
            raise RuntimeError('Private roscore readiness timed out')
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], env=env, check=True, timeout=5)
        # The shared map publisher sleeps in ROS time before latching its map.
        # Seed this private clock well BEFORE the recording, then hand it to
        # rosbag. This avoids its startup sleep hanging while replay is paused.
        with (directory / 'bootstrap.log').open('w') as log:
            bootstrap = subprocess.Popen([sys.executable, '-c', BOOTSTRAP_CLOCK,
                str(scene['first_receipt'] - 60)], env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
        with (directory / 'support.log').open('w') as log:
            support = subprocess.Popen(['roslaunch', str(launch)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        probe_code = ("import rospy; from visualization_msgs.msg import MarkerArray; "
            "rospy.init_node('e12_map_ready', disable_signals=True); "
            "m=rospy.wait_for_message('/map/lanelet_map', MarkerArray, timeout=15); "
            "assert m.markers and all(x.header.frame_id=='map' for x in m.markers); "
            "print(len(m.markers))")
        ready = subprocess.run([sys.executable, '-c', probe_code], env=env,
            capture_output=True, text=True, timeout=20)
        if ready.returncode or support.poll() is not None:
            raise RuntimeError(f'Lanelet map support failed; see {directory}/support.log\n{ready.stderr}')
        print('Common Lanelet map ready: ' + ready.stdout.strip() + ' markers.', flush=True)
        if not headless:
            with (directory / 'rviz.log').open('w') as log:
                rviz = subprocess.Popen(['rviz', '-d', str(config)], env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(.5)
        stop_owned(bootstrap, group=True)
        print(f'\n{case} | {rate:g}x | private ROS {port} | {directory}', flush=True)
        print('Space: play/pause. Ctrl+C or close RViz: close this replay only.', flush=True)
        # Keep the player in the terminal foreground group: rosbag reads Space
        # from the user TTY. Only roscore/RViz get separate owned process groups.
        player = subprocess.Popen(command, env=env)
        while player.poll() is None:
            if core.poll() is not None:
                raise RuntimeError('Private ROS master exited')
            if support.poll() is not None:
                raise RuntimeError(f'Common visualization support exited; see {directory}/support.log')
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError(f'RViz exited with {rviz.returncode}; see {directory}/rviz.log')
                break
            time.sleep(.15)
        if player.poll() not in (None, 0):
            raise RuntimeError(f'rosbag player exited with {player.returncode}')
    except KeyboardInterrupt:
        pass
    finally:
        # No pkill/killall, no existing master/parameter changes.
        stop_owned(player)
        stop_owned(rviz, group=True)
        stop_owned(support, group=True)
        stop_owned(bootstrap, group=True)
        stop_owned(core, group=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=('H04', 'H03', 'R01'), nargs='?', default='H04')
    parser.add_argument('rate', type=positive, nargs='?', default=1.)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--headless', action='store_true', help='Owned-master smoke test, no RViz')
    parser.add_argument('--autoplay', action='store_true')
    parser.add_argument('--duration', type=positive, help='Limit bag seconds and exit at end')
    args = parser.parse_args()
    scene_file = scene_paths(args.case) / 'scene.json'
    if not scene_file.is_file():
        raise SystemExit(f'Missing local export: {scene_file}\n'
            f'Prepare: /usr/bin/python3 scripts/evaluation/prepare_e12_common_rviz.py '
            f'--case {args.case} --output {scene_file.parent}')
    scene = read_scene(scene_file)
    if args.check_only:
        print(json.dumps(dict(case=args.case, raw_bag=scene['raw_bag'],
            marker_bag=scene['marker_bag'], command=play_command(scene, args.rate)), indent=2))
        return
    if args.headless and not (args.autoplay and args.duration):
        parser.error('--headless requires --autoplay and --duration to bound the smoke test')
    run(scene, args.case, args.rate, args.headless, args.autoplay, args.duration)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
