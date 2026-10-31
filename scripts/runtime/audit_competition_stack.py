#!/usr/bin/env python3
"""Bounded, read-only parked-stack audit; never publishes commands or changes params."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error('duration must be 1..60 seconds')
    import rospy
    from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
    from std_msgs.msg import Int8
    from autoware_planning_msgs.msg import Trajectory
    rospy.init_node('competition_parked_stack_audit', anonymous=True, disable_signals=True)
    headed = (
        '/localization/kinematic_state', '/localization/control_kinematic_state',
        '/perception/object_recognition/detection/objects',
        '/perception/object_recognition/tracking/objects',
        '/perception/object_recognition/objects', '/planning/trajectory',
    )
    times, ages = defaultdict(list), defaultdict(list)
    latest = {}

    def receive(topic, msg):
        times[topic].append(time.monotonic())
        if topic in headed:
            _, sec, nsec = struct.unpack_from('<III', msg._buff)
            ages[topic].append(rospy.Time.now().to_sec() - (sec + nsec * 1e-9))
        else:
            latest[topic] = msg

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, lambda m, key=topic: receive(key, m), queue_size=1)
                     for topic in headed + ('/perception/traffic_light_recognition/traffic_signals',)]
    for topic, cls in (('/ctrl_cmd', CtrlCmd), ('/vehicle/status/gear', Int8), ('/Competition_topic', EgoVehicleStatus)):
        subscriptions.append(rospy.Subscriber(topic, cls, lambda m, key=topic: receive(key, m), queue_size=1))
    end = time.monotonic() + args.duration
    while time.monotonic() < end and not rospy.is_shutdown():
        time.sleep(.05)
    for sub in subscriptions:
        sub.unregister()
    report = dict(motion_command_sent=False, topics={}, pressure={})
    for topic in headed + ('/perception/traffic_light_recognition/traffic_signals', '/ctrl_cmd'):
        ticks, delays = times[topic], sorted(ages[topic])
        report['topics'][topic] = dict(count=len(ticks), hz=(len(ticks)-1)/(ticks[-1]-ticks[0]) if len(ticks)>1 else 0.)
        if delays:
            report['topics'][topic].update(age_p50_ms=1000*delays[len(delays)//2],
                age_p95_ms=1000*delays[min(len(delays)-1, int(.95*len(delays)))], age_max_ms=1000*delays[-1])
    for key in ('cpu', 'memory', 'io'):
        report['pressure'][key] = Path('/proc/pressure/' + key).read_text().strip()
    report['meminfo'] = '\n'.join(line for line in Path('/proc/meminfo').read_text().splitlines()
                                  if line.startswith(('MemAvailable:', 'SwapFree:')))
    pubs, subs, _ = rospy.get_master().getSystemState()[2]
    report['forbidden_publishers'] = {t: dict(pubs).get(t, []) for t in ('/Ego_topic', '/Object_topic')}
    report['forbidden_subscribers'] = {t: dict(subs).get(t, []) for t in ('/Ego_topic', '/Object_topic')}
    report['control_sender_present'] = any('/morai_ctrl_cmd_udp_sender' in nodes for _, nodes in subs)
    for topic in ('/vehicle/status/gear', '/ctrl_cmd', '/Competition_topic'):
        if topic in latest:
            msg = latest[topic]
            if topic.endswith('/gear'):
                report['gear'] = msg.data
            elif topic == '/ctrl_cmd':
                report['controller_output'] = dict(accel=msg.accel, brake=msg.brake, steering=msg.steering, cmd_type=msg.longlCmdType)
            else:
                report['native_speed_mps'] = msg.velocity.x
    try:
        trajectory = rospy.wait_for_message('/planning/trajectory', Trajectory, timeout=3)
        report['trajectory_points'] = len(trajectory.points)
        report['trajectory_max_speed_mps'] = max((p.longitudinal_velocity_mps for p in trajectory.points), default=0.)
    except rospy.ROSException:
        report['trajectory_points'] = 0
    report['graph_connected'] = all(report['topics'][t]['count'] for t in report['topics'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps(report, indent=2))
    rospy.signal_shutdown('parked read-only audit complete')


if __name__ == '__main__':
    main()
