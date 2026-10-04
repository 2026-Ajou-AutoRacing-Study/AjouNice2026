#!/usr/bin/env python3
"""Record a bounded simulator trial; never issues vehicle control commands.

Run from the AW149 ROS environment. A completed observation is NOT a mission pass.
The JSON/HTML keeps collision reports distinct from proven physical ego contacts.
"""
import argparse
import html
import json
import math
import signal
import subprocess
import time
from pathlib import Path

import rospy
from morai_msgs.msg import CollisionData, EgoVehicleStatus
from autoware_planning_msgs.msg import Trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=180)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('planning_trial_observer', anonymous=True)
    started = time.monotonic()
    state = dict(ego_samples=0, trajectory_samples=0, collision_reports=0,
                 near_ego_collision_reports=0,
                 max_speed_mps=0., distance_m=0., events=[], mission_pass=None)
    last = [None]

    def ego(msg):
        xy = (msg.position.x, msg.position.y)
        if last[0] is not None:
            step = math.hypot(xy[0]-last[0][0], xy[1]-last[0][1])
            if step < 5:
                state['distance_m'] += step
            else:
                state['events'].append(dict(t=time.monotonic()-started, teleport_m=step))
        last[0] = xy
        state['ego_xy'] = xy
        state['ego_samples'] += 1
        state['max_speed_mps'] = max(state['max_speed_mps'], math.sqrt(
            msg.velocity.x**2+msg.velocity.y**2+msg.velocity.z**2))

    def trajectory(msg):
        state['trajectory_samples'] += 1
        state['trajectory_points'] = len(msg.points)

    def collision(msg):
        if msg.collision_object:
            state['collision_reports'] += 1
            # Raw MORAI reports may include distant NPC contacts. Keep this
            # proximity screen separate from geometric/physical confirmation.
            if last[0] is not None:
                near = [x for x in msg.collision_object if math.hypot(
                    x.position.x-last[0][0], x.position.y-last[0][1]) < 15.0]
                if near:
                    state['near_ego_collision_reports'] += 1
                    if state['near_ego_collision_reports'] == 1:
                        state['events'].append(dict(t=time.monotonic()-started,
                            near_ego_collision_ids=[x.unique_id for x in near]))
            if state['collision_reports'] == 1:
                state['events'].append(dict(t=time.monotonic()-started,
                    collision_ids=[x.unique_id for x in msg.collision_object]))

    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, ego, queue_size=10)
    rospy.Subscriber('/planning/trajectory', Trajectory, trajectory, queue_size=2)
    rospy.Subscriber('/CollisionData', CollisionData, collision, queue_size=10)
    pattern = r'/(planning/.*|perception/.*|evaluation/.*|localization/.*|control/.*|purepursuit_autoware/.*|longi_pi_autoware/.*|competition_route_requester/.*|tf|tf_static|Ego_topic|Object_topic|CollisionData|ctrl_cmd|rosout|rosout_agg|vehicle/.*)'
    log = (args.output/'recorder.log').open('w')
    recorder = subprocess.Popen(['rosbag', 'record', '--lz4', '--buffsize=512', '-O',
                                 str(args.output/'planning.bag'), '-e', pattern],
                                stdout=log, stderr=subprocess.STDOUT)
    state['git_sha'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    with (args.output/'rosparams.yaml').open('w') as params:
        subprocess.run(['rosparam', 'dump', '/dev/stdout'], stdout=params, check=True)
    try:
        while not rospy.is_shutdown() and time.monotonic()-started < args.seconds:
            if recorder.poll() is not None:
                raise RuntimeError('rosbag recorder exited unexpectedly')
            state['elapsed_sec'] = time.monotonic()-started
            print('OBS', json.dumps({key: state.get(key) for key in (
                'elapsed_sec', 'distance_m', 'max_speed_mps', 'near_ego_collision_reports',
                'trajectory_points')}, ensure_ascii=False), flush=True)
            time.sleep(5)
    finally:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=30)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=5)
        log.close()
        (args.output/'summary.json').write_text(json.dumps(state, indent=2))
        (args.output/'report.html').write_text('<!doctype html><meta charset="utf-8">'
            '<h1>Planning trial observation — not a mission pass</h1><pre>'
            + html.escape(json.dumps(state, indent=2)) + '</pre>')


if __name__ == '__main__':
    main()
