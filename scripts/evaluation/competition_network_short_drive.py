#!/usr/bin/env python3
"""Local-only smoke: stock controller/UDP, bounded drive, trajectory-timeout brake, verified P.

Only explicitly supplied owned launches are stopped. No scenario/GT API,
velocity-limit request or production parameter tuning. NOT a stopping-distance test.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--controller-launch-pid', type=int, required=True)
    parser.add_argument('--planning-launch-pid', type=int, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        parser.error('Requires explicit --execute; this test moves MORAI')
    cmdline = Path('/proc/%d/cmdline' % args.controller_launch_pid).read_bytes().replace(b'\0', b' ')
    if b'roslaunch trajectory_follower pp_pi_autoware.launch' not in cmdline:
        parser.error('Not the expected owned controller launch process')
    planning_cmd = Path('/proc/%d/cmdline' % args.planning_launch_pid).read_bytes().replace(b'\0', b' ')
    if b'roslaunch ajounice_planning_integration competition_planning.launch' not in planning_cmd:
        parser.error('Not the expected owned planning launch process')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import rospy
    from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int8
    from autoware_planning_msgs.msg import Trajectory
    sys.path.insert(0, str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'))
    from planning_scenario_validator.runtime import BrakeHold
    rospy.init_node('competition_network_short_drive', disable_signals=True)
    latest, observed = {}, {}
    observed_peak = [0.]

    def receive(key, msg):
        latest[key], observed[key] = msg, time.monotonic()
        if key == 'native' and math.isfinite(msg.velocity.x):
            observed_peak[0] = max(observed_peak[0], abs(msg.velocity.x))

    subscriptions = [rospy.Subscriber(topic, cls, lambda m, key=key: receive(key, m), queue_size=1)
        for key, topic, cls in (
            ('native', '/Competition_topic', EgoVehicleStatus), ('gear', '/vehicle/status/gear', Int8),
            ('pose', '/localization/kinematic_state', Odometry), ('cmd', '/ctrl_cmd', CtrlCmd),
            ('trajectory', '/planning/trajectory', Trajectory))]
    processes, log_handles, samples = [], [], []
    result = {'pass': False, 'drive_limit_seconds': 1.5, 'native_speed_abort_mps': 2., 'samples': samples}
    hold, sender, planner = None, None, None
    result['stop_method'] = 'stock controller trajectory-timeout brake'

    def fresh(key):
        if key not in latest or time.monotonic() - observed[key] >= .5:
            return False
        if key in ('native', 'pose', 'trajectory'):
            age = rospy.Time.now().to_sec() - latest[key].header.stamp.to_sec()
            return -.1 <= age < .5
        return True

    def speed():
        return abs(latest['native'].velocity.x) if fresh('native') else math.inf

    def wait_for(predicate, timeout, action=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if action:
                action()
            if predicate():
                return
            time.sleep(.04)
        raise RuntimeError('Readiness/stop condition timed out')

    def launch(command, name):
        log = (args.output_dir / (name + '.log')).open('w')
        log_handles.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)

    def stopped_held(require_park=False):
        return speed() < .03 and (not require_park or (fresh('gear') and latest['gear'].data == 1))

    try:
        result['stage'] = 'preflight'
        wait_for(lambda: all(fresh(k) for k in ('native', 'gear', 'pose', 'cmd', 'trajectory')), 8)
        pubs, subs, _ = rospy.get_master().getSystemState()[2]
        if dict(pubs).get('/ctrl_cmd', []) != ['/longi_pi_autoware']:
            raise RuntimeError('Unexpected command publisher')
        if any(dict(pubs).get(t) or dict(subs).get(t) for t in ('/Ego_topic', '/Object_topic')):
            raise RuntimeError('Forbidden GT dependency present')
        if not stopped_held(True):
            raise RuntimeError('Vehicle not stationary in P')
        pose = latest['pose'].pose.pose.position
        if math.hypot(pose.x+.130667330614472, pose.y-117.580315248218) > 3:
            raise RuntimeError('Not at official START')
        # Verify the exact STOP path before enabling any UDP control.
        result['stage'] = 'parked_watchdog_stop'
        os.kill(args.planning_launch_pid, signal.SIGINT)
        wait_for(lambda: fresh('cmd') and latest['cmd'].accel < .01 and latest['cmd'].brake > .99, 8)
        result['parked_watchdog_verified'] = True
        result['stage'] = 'planning_restart'
        planner = launch(['roslaunch', 'ajounice_planning_integration', 'competition_planning.launch'], 'planning')
        wait_for(lambda: fresh('trajectory') and fresh('cmd') and latest['cmd'].accel > .05, 30)
        started = time.monotonic()
        result['stage'] = 'bounded_drive'
        sender = launch(['roslaunch', 'sensor_interface', 'morai_udp_interface.launch',
            'enable_sensors:=false', 'enable_lidar:=false', 'enable_ego_network:=false',
            'network_config:=' + str(ROOT / 'configs/network/ego_network_competition_local.json')], 'drive_sender')
        while time.monotonic() - started < 1.5:
            if sender.poll() is not None:
                raise RuntimeError('Drive sender exited')
            if not all(fresh(k) for k in ('native', 'pose', 'cmd', 'trajectory')):
                result['drive_end'] = 'input_stale'
                break
            v = speed()
            samples.append({'seconds': time.monotonic()-started, 'native_speed_mps': v,
                            'accel': latest['cmd'].accel, 'brake': latest['cmd'].brake})
            if v > 2:
                result['drive_end'] = 'speed_bound'
                break
            time.sleep(.04)
        else:
            result['drive_end'] = 'time_bound'
        result['stage'] = 'watchdog_stop_after_drive'
        stop(planner)
        wait_for(lambda: fresh('cmd') and latest['cmd'].accel < .01 and latest['cmd'].brake > .99, 8)
        wait_for(stopped_held, 8)
        result['watchdog_stop_verified'] = True
    except Exception as error:
        result['error'] = str(error)
    finally:
        # Remove only the explicitly supplied, owned controller before BrakeHold
        # becomes the sole ROS command publisher; never compete with its accel.
        try:
            os.kill(args.controller_launch_pid, signal.SIGINT)
            wait_for(lambda: '/longi_pi_autoware' not in dict(rospy.get_master().getSystemState()[2][0]).get('/ctrl_cmd', []), 5)
            hold = BrakeHold()
            hold.start()
            if sender is not None:
                wait_for(stopped_held, 8)
                time.sleep(.5)
                wait_for(stopped_held, 2)
                stop(sender)
                sender = launch(['roslaunch', str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch'),
                    'network_config:=' + str(ROOT / 'configs/network/ego_network_competition_local.json')], 'park_sender')
                wait_for(lambda: stopped_held(True), 8)
                time.sleep(.5)
                wait_for(lambda: stopped_held(True), 2)
            result['park_verified'] = stopped_held(True)
            result['peak_sampled_drive_window_speed_mps'] = max((x['native_speed_mps'] for x in samples), default=0)
            result['peak_native_speed_mps'] = observed_peak[0]
            result['pass'] = bool(result.get('watchdog_stop_verified') and result['park_verified']
                                  and result['peak_native_speed_mps'] > .1 and 'error' not in result)
        except Exception as error:
            result['stop_error'] = str(error)
            result['pass'] = False
            if sender is not None and hold is not None:
                # Do not tear down braking transport on an unverified stop.
                # A reconnect/manual intervention may restore fresh status.
                print('STOP UNVERIFIED: keeping brake transport; waiting for fresh stationary status', flush=True)
                while not stopped_held():
                    time.sleep(.1)
                stop(sender)
                sender = launch(['roslaunch', str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch'),
                    'network_config:=' + str(ROOT / 'configs/network/ego_network_competition_local.json')], 'park_retry')
                while not stopped_held(True):
                    time.sleep(.1)
        # All tests are local/low-speed; keep braking through shutdown before
        # stopping transport. Never report success without observed stationary P.
        if hold is not None:
            time.sleep(.5)
        for process in reversed(processes):
            stop(process)
        if hold is not None:
            hold.stop()
        for sub in subscriptions:
            sub.unregister()
        for log in log_handles:
            log.close()
        (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
