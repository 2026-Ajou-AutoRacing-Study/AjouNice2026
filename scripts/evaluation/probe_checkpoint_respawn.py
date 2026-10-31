#!/usr/bin/env python3
"""Parked native Spawn probe; no controller or automatic driving is started.

Requires an owned UDP receiver with enable_control=false and no other stack.
Writes evidence only; never changes production parameters or installed files.
"""
import json
import argparse
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
    parser.add_argument('--nearby-once', action='store_true', help='Drive until 8m or 5m/s, then brake and return once to CP1')
    parser.add_argument('--moving-respawn', action='store_true', help='One low-speed CP1 return with controller left running')
    args = parser.parse_args()
    if args.moving_respawn:
        args.nearby_once = True
    import rospy
    import tf
    from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
    try:
        from trajectory_follower.msg import TerminalState
    except ImportError:
        TerminalState = None  # Older installed controller: command + rosout remain the evidence.
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int8, String
    from autoware_planning_msgs.msg import Trajectory
    sys.path.insert(0, str(ROOT/'base_ws/src/planning_addons/planning_scenario_validator/src'))
    from planning_scenario_validator.runtime import BrakeHold
    out = ROOT/'planner_logs/analysis'/time.strftime('respawn_parked_%y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    rospy.init_node('parked_respawn_probe', disable_signals=True)
    latest, receipt, samples, children, handles = {}, {}, [], [], []
    listener = tf.TransformListener()

    def callback(key, value):
        latest[key], receipt[key] = value, time.monotonic()

    subs = [rospy.Subscriber(topic, cls, lambda msg, key=key: callback(key, msg), queue_size=1)
            for key, topic, cls in (
                ('ego', '/Ego_topic', EgoVehicleStatus), ('gear', '/vehicle/status/gear', Int8),
                ('global', '/localization/kinematic_state', Odometry),
                ('smooth', '/localization/control_kinematic_state', Odometry),
                ('route', '/competition_route_requester/request_status', String),
                ('command', '/ctrl_cmd', CtrlCmd),
                ('trajectory', '/planning/trajectory', Trajectory))]
    if TerminalState is not None:
        subs.append(rospy.Subscriber('/control/terminal_state', TerminalState,
                                    lambda msg: callback('control',msg), queue_size=1))

    def launch(name, args):
        handle = (out/(name+'.log')).open('w')
        handles.append(handle)
        proc = subprocess.Popen(args, cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc

    def snapshot(cp, elapsed):
        row = dict(checkpoint=cp, elapsed=elapsed, wall=time.time())
        ego = latest.get('ego')
        if ego:
            row['speed'] = abs(ego.velocity.x)
            row['native_xy'] = [ego.position.x, ego.position.y]
            try:
                trans, rot = listener.lookupTransform('map', 'morai_map', rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                x, y = ego.position.x, ego.position.y
                target = [trans[0]+math.cos(yaw)*x-math.sin(yaw)*y,
                          trans[1]+math.sin(yaw)*x+math.cos(yaw)*y]
                row['target_map_xy'] = target
                for key in ('global', 'smooth'):
                    if key in latest:
                        p = latest[key].pose.pose.position
                        row[key+'_error_m'] = math.hypot(p.x-target[0], p.y-target[1])
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                pass
        row['gear'] = latest['gear'].data if 'gear' in latest else None
        if 'command' in latest:
            row['brake'] = latest['command'].brake
            row['accel'] = latest['command'].accel
        if 'control' in latest:
            row['control_reason'] = latest['control'].reason
            row['control_level'] = latest['control'].level
        row['route'] = latest['route'].data if 'route' in latest else None
        tr = latest.get('trajectory')
        if tr and ego:
            row['trajectory_points'] = len(tr.points)
            row['trajectory_age'] = rospy.Time.now().to_sec()-tr.header.stamp.to_sec()
            if tr.points and 'target_map_xy' in row:
                x, y = row['target_map_xy']
                row['trajectory_distance_m'] = min(math.hypot(p.pose.position.x-x, p.pose.position.y-y) for p in tr.points)
        row['receipt_age'] = {key: time.monotonic()-value for key, value in receipt.items()}
        return row

    hold = BrakeHold()
    spawn = None
    drive_sender = controller = park_sender = None
    result = {'scope': 'parked native respawn, no controller; not closed-loop driving', 'samples': samples}
    result['installed_terminal_state_type_available'] = TerminalState is not None
    try:
        hold.start()
        park_sender = launch('park_sender', ['rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
               '_network_config:='+str(ROOT/'configs/network/ego_network_competition_local.json'), '_gear:=1'])
        time.sleep(2)
        if 'ego' not in latest or abs(latest['ego'].velocity.x) > .05:
            raise RuntimeError('Fresh stationary Ego feedback required')
        spawn = subprocess.Popen(['/home/luv/Applications/MORAI_Spawn_Linux'], stdin=subprocess.PIPE,
                                 stdout=(out/'spawn.log').open('w'), stderr=subprocess.STDOUT, text=True)
        spawn.stdin.write('0\n'); spawn.stdin.flush()
        time.sleep(2)
        launch('sensing', ['roslaunch', 'bringup', 'sensing.launch'])
        launch('localization', ['roslaunch', 'bringup', 'localization_gnss_imu_gt_ctra.launch',
               'gt_source:=competition', 'initial_position_source:=official_start',
               'use_heading_correction:=true', 'heading_yaw_offset_deg:=-1.347',
               'publish_sensor_tf:=false', 'publish_control_state:=true',
               'use_quantized_gnss_control_state:=false', 'control_translation_tau_sec:=0.5', 'control_yaw_tau_sec:=0.5'])
        launch('planning', ['roslaunch', 'ajounice_planning_integration', 'competition_planning.launch'])
        launch('tracking_gt', ['roslaunch', 'perception_object_pipeline', 'tracking_prediction.launch',
                              'use_morai_gt_detection:=true'])
        launch('bag', ['rosbag', 'record', '-O', str(out/'probe.bag'), '/Ego_topic', '/Competition_topic',
               '/sensor/gps', '/localization/kinematic_state', '/localization/control_kinematic_state',
               '/competition_route_requester/request_status', '/planning/trajectory', '/tf', '/tf_static',
               '/vehicle/status/gear', '/ctrl_cmd', '/control/terminal_state', '/rosout'])
        # A respawn result is uninterpretable if startup itself never worked.
        started = time.monotonic()
        while time.monotonic()-started < 100:
            samples.append(snapshot('startup', time.monotonic()-started))
            status = json.loads(latest['route'].data) if 'route' in latest else {}
            if status.get('verified_active') and latest.get('trajectory') and latest['trajectory'].points:
                break
            if status.get('uncertain'):
                raise RuntimeError('Startup route reconciliation required; respawn not attempted')
            time.sleep(.2)
        else:
            raise RuntimeError('No verified startup route/trajectory; respawn not attempted')
        stages = ((1, 15),) if args.nearby_once else ((0, 3), (1, 22), (5, 22), (12, 25), (0, 22))
        for cp, duration in stages:
            if cp != 0 or samples:
                spawn.stdin.write(str(cp)+'\n'); spawn.stdin.flush()
            started = time.monotonic()
            while time.monotonic()-started < duration:
                row = snapshot(cp, time.monotonic()-started)
                samples.append(row)
                if row.get('speed', 0) > .15:
                    raise RuntimeError('Unexpected movement during parked probe')
                time.sleep(.1)
            print(json.dumps(dict(checkpoint=cp, final=samples[-1])), flush=True)
        if args.nearby_once:
            row = samples[-1]
            if row.get('trajectory_distance_m', 999) > .5 or row.get('smooth_error_m', 999) > .25:
                raise RuntimeError('CP1 baseline not aligned; no driving')
            origin = row['native_xy']
            result['scope'] = 'One nearby return to CP1 after real planner/controller driving; stop request at 8m or 5m/s, with additional braking travel; lightweight GT'
            if args.moving_respawn:
                launch('trial_speed_cap', ['/usr/bin/python3', str(ROOT/'scripts/evaluation/publish_traffic_braking_trial_speed_cap.py'),
                    '--max-velocity-mps','4', '--max-seconds','180', '--output',
                    str(ROOT/'artifacts/evaluation'/('moving_respawn_cap_'+out.name))])
                time.sleep(1)
            hold.stop()
            controller = launch('controller', ['roslaunch', 'trajectory_follower', 'pp_pi_autoware.launch',
                                'kinematic_state_topic:=/localization/control_kinematic_state'])
            time.sleep(2)
            stop_owned(park_sender)
            drive_sender = launch('drive_sender', ['rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
                                   '_network_config:='+str(ROOT/'configs/network/ego_network_competition_local.json'), '_gear:=4'])
            started = time.monotonic()
            while time.monotonic()-started < (10 if args.moving_respawn else 7):
                row = snapshot('drive', time.monotonic()-started)
                samples.append(row)
                if row['receipt_age'].get('ego', 999) > .5 or row['receipt_age'].get('smooth',999) > .5:
                    raise RuntimeError('Stale drive input')
                distance = math.dist(row['native_xy'], origin)
                if args.moving_respawn and row.get('speed', 0) > 5.5:
                    raise RuntimeError('Trial speed cap not effective; respawn not attempted')
                if distance >= 8 or (not args.moving_respawn and row.get('speed', 0) >= 5):
                    break
                time.sleep(.04)
            if args.moving_respawn:
                result['scope'] = 'One CP1 respawn while moving; controller/localization/planning kept running; lightweight GT'
                result['before_return'] = snapshot('before_moving_return', 0)
                if result['before_return'].get('speed',0) < 1 or distance < 7:
                    raise RuntimeError('Moving respawn requires speed >=1m/s and >=7m actual progress; not attempted')
                result['spawn_command_wall'] = time.time()
                spawn.stdin.write('1\n'); spawn.stdin.flush()
                started = time.monotonic()
                while time.monotonic()-started < 8:
                    row = snapshot('moving_return_cp1', time.monotonic()-started)
                    samples.append(row)
                    if any(row['receipt_age'].get(key,999)>.5 for key in ('ego','smooth','command')):
                        raise RuntimeError('Stale input after moving respawn; emergency test stop')
                    if row.get('speed',0)>7 or math.dist(row['native_xy'],origin)>18:
                        result['observation_end'] = 'bounded speed/distance; test brake follows'
                        break
                    time.sleep(.02)
                result['after_return'] = samples[-1]
                result['completed'] = True
                print('MOVING_RESULT '+json.dumps(result['after_return']), flush=True)
                return
            stop_owned(controller)
            hold = BrakeHold(); hold.start()
            wait_stationary(latest, receipt)
            result['distance_before_return_m'] = math.dist(snapshot('stopped', 0)['native_xy'], origin)
            if result['distance_before_return_m'] < 2:
                raise RuntimeError('Insufficient actual travel; nearby return not attempted')
            stop_owned(drive_sender)
            park_sender = launch('park_after_drive', ['rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
                                    '_network_config:='+str(ROOT/'configs/network/ego_network_competition_local.json'), '_gear:=1'])
            time.sleep(1)
            result['before_return'] = snapshot('before_return', 0)
            spawn.stdin.write('1\n'); spawn.stdin.flush()
            started = time.monotonic()
            while time.monotonic()-started < 15:
                samples.append(snapshot('nearby_return_cp1', time.monotonic()-started))
                time.sleep(.1)
            result['after_return'] = samples[-1]
            print('NEARBY_RESULT '+json.dumps(result['after_return']), flush=True)
        result['completed'] = True
    except BaseException as error:
        result.update(completed=False, error=repr(error))
        print(repr(error), flush=True)
    finally:
        if args.nearby_once:
            stop_owned(controller)
            if hold.thread is None:
                hold = BrakeHold(); hold.start()
            if drive_sender is not None and drive_sender.poll() is None:
                wait_stationary(latest, receipt)
                stop_owned(drive_sender)
                park_sender = launch('emergency_park', ['rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
                    '_network_config:='+str(ROOT/'configs/network/ego_network_competition_local.json'), '_gear:=1'])
                time.sleep(1)
            result['final_park'] = snapshot('final_park',0)
        # Keep the brake publisher until the P sender has finished; no controller ran.
        if spawn:
            try:
                spawn.stdin.write('q\n'); spawn.stdin.flush(); spawn.wait(timeout=3)
            except Exception:
                spawn.terminate()
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=3)
        hold.stop()
        for handle in handles:
            handle.close()
        (out/'result.json').write_text(json.dumps(result, indent=2))
        print('EVIDENCE '+str(out), flush=True)


def stop_owned(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)


def wait_stationary(latest, receipt):
    # Keep braking transport alive if feedback disappears; never tear it down moving.
    stable = None
    while True:
        now = time.monotonic()
        ego = latest.get('ego')
        good = ego and now-receipt.get('ego', 0) < .5 and abs(ego.velocity.x) < .03
        stable = (stable or now) if good else None
        if stable and now-stable >= .5:
            return
        time.sleep(.05)


if __name__ == '__main__':
    main()
