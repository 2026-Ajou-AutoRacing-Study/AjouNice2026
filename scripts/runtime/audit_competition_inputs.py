#!/usr/bin/env python3
"""Read-only live input audit; never sends CtrlCmd, loads scenes or changes params."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error('duration must be 1..60 seconds')
    import rospy
    from morai_msgs.msg import EgoVehicleStatus, GPSMessage
    rospy.init_node('competition_input_audit', anonymous=True, disable_signals=True)
    stamps, latest = defaultdict(list), {}
    topics = {
        '/Competition_topic': EgoVehicleStatus,
        '/CollisionData': rospy.AnyMsg,
        '/sensor/gps': GPSMessage,
        '/sensor/imu': rospy.AnyMsg,
        '/sensor/lidar1': rospy.AnyMsg,
        '/sensor/camera/front/compressed': rospy.AnyMsg,
        '/sensor/camera/front_left/compressed': rospy.AnyMsg,
        '/sensor/camera/front_right/compressed': rospy.AnyMsg,
        '/sensor/camera/back/compressed': rospy.AnyMsg,
    }

    def observe(topic, message):
        stamps[topic].append(time.monotonic())
        if topic in ('/Competition_topic', '/sensor/gps'):
            latest[topic] = message

    subscriptions = [rospy.Subscriber(topic, cls, lambda msg, key=topic: observe(key, msg), queue_size=1)
                     for topic, cls in topics.items()]
    end = time.monotonic() + args.duration
    while time.monotonic() < end and not rospy.is_shutdown():
        time.sleep(.03)
    for sub in subscriptions:
        sub.unregister()
    publishers = dict(rospy.get_master().getSystemState()[2][0])
    report = {'motion_command_sent': False, 'topics': {}}
    for topic in topics:
        times = stamps[topic]
        report['topics'][topic] = dict(count=len(times), hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.)
    report['forbidden_publishers'] = {topic: publishers.get(topic, []) for topic in ('/Ego_topic', '/Object_topic')}
    if '/Competition_topic' in latest:
        msg = latest['/Competition_topic']
        report['competition'] = dict(raw_heading_deg=msg.heading, corrected_heading_deg=msg.heading-1.347,
                                     vx_mps=msg.velocity.x, gear=getattr(msg, 'gear', None),
                                     ctrl_mode=getattr(msg, 'ctrl_mode', None))
    if '/sensor/gps' in latest:
        msg = latest['/sensor/gps']
        report['gps'] = dict(latitude=msg.latitude, longitude=msg.longitude, altitude=msg.altitude)
    report['required_input_pass'] = all(stamps[t] for t in topics if t != '/CollisionData')
    report['no_forbidden_publishers'] = not any(report['forbidden_publishers'].values())
    # Collision can be event-driven; absent packets at rest are not proof of failure.
    report['collision_note'] = 'No collision packets at rest may be normal; no impact was induced.'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output:
            json.dump(report, output, indent=2)
            output.write('\n')
    print(json.dumps(report, indent=2))
    rospy.signal_shutdown('read-only audit complete')
    return 0 if report['required_input_pass'] and report['no_forbidden_publishers'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
