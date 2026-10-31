#!/usr/bin/env python3
"""Record small GNSS timing topics and analyze; never owns vehicle control."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xmlrpc.client


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = Path(__file__).with_name('analyze_gnss_delay.py')
TOPICS = [
    '/Ego_topic', '/Competition_topic', '/competition/raw_twist',
    '/sensor/gps', '/sensor/imu', '/localization/imu',
    '/localization/gt/twist', '/localization/gt/orientation',
    '/localization/gnss/odom', '/localization/odometry/local',
    '/localization/odom', '/localization/kinematic_state',
    '/localization/control_kinematic_state', '/localization/constraints/lateral_velocity',
    '/localization/initialpose', '/tf', '/tf_static', '/rosout', '/clock',
]
REQUIRED = {'/Ego_topic', '/sensor/gps', '/Competition_topic'}
PARAMETERS = [
    '/use_sim_time', '/morai_gps_to_local_odometry', '/morai_gt_motion_adapter',
    '/gnss_imu_gt_local_ekf', '/gnss_imu_gt_global_ekf',
    '/smooth_control_state_publisher',
]


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.
        return connection


def inspect_master(uri):
    """Read-only preflight and a bounded allowlisted parameter snapshot."""
    proxy = xmlrpc.client.ServerProxy(uri, transport=BoundedTransport(), allow_none=True)
    caller = '/gnss_delay_recorder_preflight'
    code, _, _ = proxy.getPid(caller)
    if code != 1:
        raise RuntimeError('ROS master is unavailable; run rc and the full stack first')
    code, _, topics = proxy.getPublishedTopics(caller, '/')
    if code != 1:
        raise RuntimeError('Cannot read ROS published topics')
    names = {row[0] for row in topics}
    missing = sorted(REQUIRED - names)
    if missing:
        raise RuntimeError('Missing ' + ', '.join(missing) +
                           '; development calibration requires Ego oracle + raw GPS + Competition. '
                           'It cannot calibrate a no-Ego competition recording.')
    params = {}
    for name in PARAMETERS:
        code, _, value = proxy.getParam(caller, name)
        params[name] = value if code == 1 else None
    return {'master_uri': uri, 'advertised_topics': topics, 'parameters': params}


def allocate_output(explicit=None, root=None):
    path = (Path(explicit).expanduser().resolve() if explicit else
            Path(root or ROOT / 'planner_logs/analysis/gnss_delay/manual') /
            (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]))
    path.mkdir(parents=True, exist_ok=False)
    return path


def recorder_command(bag, duration):
    return ['rosbag', 'record', '--lz4', '--buffsize=64',
            '--duration=' + str(duration), '-O', str(bag)] + TOPICS


def analyzer_command(bag, output, heading, acquisition):
    return [sys.executable, str(ANALYZER), str(bag), '--output-dir', str(output),
            '--heading-offset-deg', str(heading), '--acquisition', acquisition]


def stop_owned(process, timeout=15.):
    if process.poll() is not None:
        return
    for sig, wait in [(signal.SIGINT, timeout), (signal.SIGTERM, 3.), (signal.SIGKILL, 3.)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Owned recorder did not exit; raw evidence left in place')


def record(bag, duration, output):
    log = output / 'recorder.log'
    env = dict(os.environ, ROS_LOG_DIR=str(output / 'ros_logs'), ROS_HOME=str(output / 'ros_home'))
    with log.open('x') as stream:
        process = subprocess.Popen(recorder_command(bag, duration), stdout=stream,
                                   stderr=subprocess.STDOUT, env=env, start_new_session=True)
        original = {}
        interrupted = []
        def handle(signum, _frame):
            interrupted.append(signum)
            try:
                os.killpg(process.pid, signal.SIGINT if len(interrupted) == 1 else signal.SIGTERM)
            except ProcessLookupError:
                pass
        try:
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
                original[sig] = signal.signal(sig, handle)
            print('GNSS recording started. Drive normally; include acceleration and different speeds.', flush=True)
            print('Ctrl-C ends ONLY this recorder, then analyzes. It does NOT stop the car.', flush=True)
            print('Recording: ' + str(bag), flush=True)
            code = process.wait(timeout=duration + 30.)
        finally:
            stop_owned(process)
            for sig, handler in original.items():
                signal.signal(sig, handler)
    if not bag.is_file():
        raise RuntimeError('No finalized bag; inspect ' + str(log) + '. Active files are preserved.')
    if code not in (0, 130, -signal.SIGINT):
        raise RuntimeError('Recorder failed (exit ' + str(code) + '); inspect ' + str(log))
    return {'exit_code': code, 'interrupted': bool(interrupted), 'log': str(log)}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=90., help='wall seconds, default90; Ctrl-C finishes early')
    parser.add_argument('--output-dir', type=Path, help='new directory only, never overwrite')
    parser.add_argument('--output-root', type=Path, help='parent for automatic unique result directories')
    parser.add_argument('--bag', type=Path, help='analyze an existing bag instead of connecting to ROS')
    parser.add_argument('--heading-offset-deg', type=float, default=-1.347)
    parser.add_argument('--acquisition', choices=['MR', 'MRP', 'LAN_FIELD', 'OFFLINE_REPLAY'], default='MR')
    parser.add_argument('--dry-run', action='store_true', help='print plan, no ROS or files')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error('--duration must be finite and in [1, 600] seconds')
    if not math.isfinite(args.heading_offset_deg):
        parser.error('--heading-offset-deg must be finite')
    if args.bag and not args.bag.expanduser().is_file():
        parser.error('input bag does not exist')
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.dry_run:
        print(json.dumps({'mode': 'analyze_existing' if args.bag else 'record_then_analyze',
                          'duration_sec': args.duration, 'topics': TOPICS,
                          'heading_offset_deg': args.heading_offset_deg,
                          'controls_vehicle': False, 'starts_master': False}, indent=2))
        return 0
    snapshot = None
    if not args.bag:
        snapshot = inspect_master(os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311'))
        if shutil.disk_usage(ROOT).free < 512 * 1024**2:
            raise RuntimeError('Need at least512MiB free for a new calibration recording')
    output = allocate_output(args.output_dir, args.output_root)
    bag = args.bag.expanduser().resolve() if args.bag else output / 'gnss_delay.bag'
    provenance = {'schema_version': 1, 'started_at': datetime.now(timezone.utc).isoformat(),
                  'mode': 'existing_bag' if args.bag else 'manual_stack_observation',
                  'bag': str(bag), 'heading_offset_deg': args.heading_offset_deg,
                  'acquisition': args.acquisition, 'controls_vehicle': False,
                  'snapshot': snapshot, 'topics_requested': TOPICS}
    (output / 'provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    if not args.bag:
        receipt = record(bag, args.duration, output)
        (output / 'recording_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    command = analyzer_command(bag, output / 'analysis', args.heading_offset_deg, args.acquisition)
    print('Calculating relative GNSS delay...', flush=True)
    result = subprocess.run(command, timeout=180.)
    print('Results: ' + str(output / 'analysis'), flush=True)
    return result.returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print('GNSS measurement failed: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
