#!/usr/bin/env python3
"""Private-master, sensor-only RViz playback of frozen E12 offline results.

No inference, tracking, control, TF, UDP, or MORAI nodes are launched. The raw
bag is strictly topic-whitelisted so existing live ROS sessions are untouched.
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


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('A finite positive number is required')
    return result


def scene_paths(case):
    return OUTPUT / 'rviz_exports_v1' / case


def read_scene(path):
    scene = json.loads(path.read_text())
    if scene.get('schema') != 'e12_postprocessing_rviz_scene_v1' or scene.get('complete') is not True:
        raise ValueError('A completed E12 viewer export is required')
    for key in ('raw_bag', 'marker_bag'):
        value = Path(scene[key])
        if not value.is_absolute() or not value.is_file():
            raise ValueError(f'{key} is not an existing absolute file: {value}')
    if scene['fixed_frame'] != 'lidar1':
        raise ValueError('Only the audited current lidar1 coordinate view is supported')
    if set(scene['topics']) != set(MARKER_TOPICS):
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
    command += [scene['raw_bag'], scene['marker_bag'], '--topics']
    return command + list(SENSOR_TOPICS) + list(MARKER_TOPICS)


def rviz_configuration(frame):
    displays = [dict(Class='rviz/Grid', Name='10m grid', Enabled=True,
        **{'Cell Size': 10, 'Plane Cell Count': 20, 'Reference Frame': frame,
           'Color': '90; 90; 90', 'Alpha': .4, 'Value': True}),
        dict(Class='rviz/PointCloud2', Name='Original LiDAR', Enabled=True,
             Topic='/sensor/lidar1', **{'Queue Size': 2, 'Decay Time': .12,
             'Position Transformer': 'XYZ', 'Color Transformer': 'Intensity',
             'Size (Pixels)': 2, 'Style': 'Points', 'Alpha': .5, 'Value': True})]
    titles = ('GT (magenta)', 'E12 detections (green)', 'Tracks (cyan / coast orange)',
              'Track velocity', 'Current 3s (yellow)', 'Candidate 5s (orange)',
              'Prefix-stable 5s RESEARCH (blue)', 'Valid GT future (white)', 'Scene notes')
    for name, topic, title in zip(MARKER_NAMES, MARKER_TOPICS, titles):
        displays.append(dict(Class='rviz/MarkerArray', Name=title,
            Enabled=name != 'prefix5', **{'Marker Topic': topic,
            'Queue Size': 2, 'Value': name != 'prefix5'}))
    for camera in ('front', 'back', 'front_left', 'front_right'):
        displays.append(dict(Class='rviz/Image', Name='Camera ' + camera,
            Enabled=camera == 'front', **{'Image Topic': '/sensor/camera/' + camera,
            'Transport Hint': 'compressed', 'Queue Size': 2, 'Normalize Range': True,
            'Min Value': 0, 'Max Value': 1, 'Median window': 5,
            'Value': camera == 'front'}))
    return {'Panels': [{'Class': 'rviz/Displays', 'Name': 'Displays'},
                       {'Class': 'rviz/Views', 'Name': 'Views'}],
        'Preferences': {'PromptSaveOnExit': False},
        'Visualization Manager': {'Class': '', 'Name': 'root', 'Enabled': True,
            'Displays': displays,
            'Global Options': {'Fixed Frame': frame, 'Frame Rate': 30,
                               'Background Color': '35; 35; 35'},
            'Tools': [{'Class': 'rviz/Interact'}, {'Class': 'rviz/MoveCamera'},
                      {'Class': 'rviz/Select'}, {'Class': 'rviz/FocusCamera'}],
            'Views': {'Current': {'Class': 'rviz/Orbit', 'Name': 'Current View',
                'Target Frame': frame, 'Distance': 180, 'Pitch': 1.25, 'Yaw': 3.14159,
                'Focal Point': {'X': 0, 'Y': 0, 'Z': 0}, 'Near Clip Distance': .05}},
            'Value': True},
        'Window Geometry': {'Width': 1600, 'Height': 950, 'X': 40, 'Y': 30}}


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
    for executable in ('roscore', 'rosbag', 'rosparam') + (() if headless else ('rviz',)):
        if not shutil.which(executable):
            raise RuntimeError(f'{executable} not found; use the .sh wrapper to source ROS')
    if not headless and not os.environ.get('DISPLAY'):
        raise RuntimeError('DISPLAY is empty. Run this in a local desktop terminal.')
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory = OUTPUT / 'rviz_sessions' / f'{case}_{stamp}_{os.getpid()}'
    directory.mkdir(parents=True, exist_ok=False)
    config = directory / 'view.rviz'
    config.write_text(yaml.safe_dump(rviz_configuration(scene['fixed_frame']), sort_keys=False))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = private_environment(directory, port)
    command = play_command(scene, rate, autoplay, duration)
    (directory / 'session.json').write_text(json.dumps(dict(
        case=case, rate=rate, scene=scene, command=command,
        master=env['ROS_MASTER_URI'], live_stack_started=False), indent=2))
    core = rviz = player = None
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
        if not headless:
            with (directory / 'rviz.log').open('w') as log:
                rviz = subprocess.Popen(['rviz', '-d', str(config)], env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'\n{case} | {rate:g}x | private ROS {port} | {directory}', flush=True)
        print('Space: play/pause. Ctrl+C or close RViz: close this replay only.', flush=True)
        # Keep the player in the terminal foreground group: rosbag reads Space
        # from the user TTY. Only roscore/RViz get separate owned process groups.
        player = subprocess.Popen(command, env=env)
        while player.poll() is None:
            if core.poll() is not None:
                raise RuntimeError('Private ROS master exited')
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
            f'Prepare: /usr/bin/python3 scripts/evaluation/export_e12_postprocessing_view.py '
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
