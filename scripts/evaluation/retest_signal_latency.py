#!/usr/bin/env python3
"""Fresh A/B/C/D actual-validator trials with lightweight host telemetry.

No production defaults, models, raw scenes or existing evidence are changed.
Stops on the first nonzero trial result; the existing validator owns stop/P.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]


def retry_config(source, mission, record_front_jpeg=False, minimum_free_disk_gib=None):
    cfg = yaml.safe_load(source.read_text())
    cfg['campaign_name'] = 'traffic_signal_latency_retest_'+mission
    cfg['resume'] = False
    if minimum_free_disk_gib is not None:
        if not math.isfinite(minimum_free_disk_gib) or minimum_free_disk_gib < 1.:
            raise ValueError('Short trial reserve must remain at least 1 GiB')
        cfg['recording']['minimum_free_disk_gib'] = minimum_free_disk_gib
    if not record_front_jpeg:
        cfg['recording']['selected_topics'] = [
            topic for topic in cfg['recording']['selected_topics']
            if topic != '/sensor/camera/front/compressed']
    elif '/sensor/camera/front/compressed' not in cfg['recording']['selected_topics']:
        cfg['recording']['selected_topics'].append('/sensor/camera/front/compressed')
    for node in ('behavior_planning/behavior_path_planner',
                 'behavior_planning/behavior_velocity_planner',
                 'motion_planning/motion_velocity_planner', 'motion_planning/velocity_smoother'):
        cfg['recording']['selected_topics'].append(
            '/planning/scenario_planning/lane_driving/'+node+'/debug/processing_time_ms_diag')
    return cfg


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trials', type=Path, required=True)
    p.add_argument('--map-file', type=Path, required=True)
    p.add_argument('--simulator-pid', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--record-front-jpeg', action='store_true',
                   help='Keep original camera JPEGs for stop-position visual review (requires disk budget)')
    p.add_argument('--missions', default='ABCD',
                   help='Unique mission letters, in order; retry a permissive pass as a fresh trial')
    p.add_argument('--minimum-free-disk-gib', type=float,
                   help='Explicit short-trial storage budget; >=1 GiB reserve, source default unchanged')
    p.add_argument('--single-config', type=Path,
                   help='One explicit alternate approach/corridor config; requires one mission letter')
    a = p.parse_args()
    if not a.missions or len(set(a.missions)) != len(a.missions) or set(a.missions)-set('ABCD'):
        p.error('Use nonempty unique A/B/C/D mission letters')
    if a.single_config is not None and len(a.missions) != 1:
        p.error('An explicit approach override requires exactly one mission')
    a.output = a.output.resolve()
    a.output.relative_to(ROOT/'artifacts/evaluation')
    if a.output.exists() or not Path('/proc', str(a.simulator_pid)).is_dir():
        p.error('Require a fresh output and a running simulator PID')
    a.output.mkdir(parents=True)
    (a.output/'configs').mkdir()
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    sources = {}
    for mission in a.missions:
        source = a.single_config if a.single_config is not None else a.trials/(mission+'_focused.yaml')
        sources[str(source.resolve())] = sha(source)
        (a.output/'configs'/(mission+'.yaml')).write_text(
            yaml.safe_dump(retry_config(source, mission, a.record_front_jpeg,
                                        a.minimum_free_disk_gib), sort_keys=False))
    receipt = dict(map_file=str(a.map_file.resolve()), map_sha256=sha(a.map_file),
        launcher_sha256=sha(Path(__file__)),
        host_sampler_sha256=sha(ROOT/'scripts/rosbag/record/record_host_telemetry.py'),
        source_configs=sources, debug_cpp=False, camera_full_bag=a.record_front_jpeg,
        missions=a.missions,
        minimum_free_disk_gib=a.minimum_free_disk_gib,
        telemetry_interval_sec=1., simulator_pid=a.simulator_pid,
        source_geometry=str((a.trials/'groups.json').resolve()),
        geometry_sha256=sha(a.trials/'groups.json'),
        scope='OD_GT + real localization/planner/controller + traffic v5; not BEVFusion full-stack certification')
    (a.output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    current = None
    def interrupted(signum, frame):
        # The finally block delivers exactly one graceful termination request.
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    telemetry_log = (a.output/'host_stderr.log').open('x')
    telemetry = subprocess.Popen([sys.executable, str(ROOT/'scripts/rosbag/record/record_host_telemetry.py'),
        '--output', str(a.output/'host.jsonl'), '--role', 'local_morai_and_ros',
        '--session-id', a.output.name, '--interval', '1', '--pid', str(a.simulator_pid), '--no-gpu'],
        stdout=telemetry_log, stderr=subprocess.STDOUT)
    def event(**body):
        body.update(wall_time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())
        with (a.output/'events.jsonl').open('a') as f:
            f.write(json.dumps(body)+'\n')
    try:
        for mission in a.missions:
            if telemetry.poll() is not None:
                raise RuntimeError('Host telemetry stopped; do not run an uninstrumented trial')
            command = [sys.executable, str(ROOT/'scripts/evaluation/run_traffic_light_compliance_trial.py'),
                str(a.output/'configs'/(mission+'.yaml')), '--output', str(a.output/mission),
                '--geometry', str(a.trials/'groups.json'), '--map-file', str(a.map_file),
                '--mode', 'production', '--experiment', 'natural']
            event(event='trial_start', mission=mission, command=command)
            with (a.output/(mission+'.log')).open('x') as log:
                current = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                code = current.wait()
            event(event='trial_end', mission=mission, returncode=code)
            current = None
            if code != 0:
                return code
        return 0
    finally:
        if current is not None and current.poll() is None:
            current.send_signal(signal.SIGTERM)
            # No force kill: the validator must retain its verified stop/P cleanup.
            current.wait()
        telemetry.send_signal(signal.SIGINT)
        telemetry.wait(timeout=10)
        telemetry_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
