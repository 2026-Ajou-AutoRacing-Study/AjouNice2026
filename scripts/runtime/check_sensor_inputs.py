#!/usr/bin/env python3
"""Read-only, bounded ROS input-flow check. Never publishes or changes params."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time

# Receive rates are diagnostic references, not hard competition acceptance limits.
TOPICS = {
    'vehicle': [('Vehicle', '/Competition_topic', 'morai_msgs/EgoVehicleStatus', 50)],
    'cameras': [(name, '/sensor/camera/'+direction+'/compressed', 'sensor_msgs/CompressedImage', 30)
                for name, direction in [('Front', 'front'), ('Left', 'front_left'),
                                        ('Right', 'front_right'), ('Rear', 'back')]],
    'lidar': [('LiDAR', '/sensor/lidar1', 'sensor_msgs/PointCloud2', 15)],
    'gnss': [('GNSS', '/sensor/gps', 'morai_msgs/GPSMessage', 30)],
    'imu': [('IMU', '/sensor/imu', 'sensor_msgs/Imu', 50)],
}
IPS = {'competition': '192.168.0.10', 'teamlead': '10.42.0.1', 'local': '127.0.0.1'}


def evaluate(times, now, expected_hz, *, type_ok=True, invalid=False):
    if not type_ok:
        return 'FAIL', 'wrong message type', 0.0, None
    if not times:
        return 'FAIL', 'no new data', 0.0, None
    age = max(0.0, now-times[-1])
    hz = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.0
    if len(times)<2 or age>1.0:
        return 'FAIL', 'stale / single message', hz, age
    if invalid:
        return 'WARN', 'invalid / shadow coordinates', hz, age
    if hz<expected_hz*.5:
        return 'WARN', 'low rate; check load', hz, age
    return 'PASS', 'receiving', hz, age


def gps_invalid(message):
    lat, lon = float(message.latitude), float(message.longitude)
    return not (math.isfinite(lat) and math.isfinite(lon) and
                -90<=lat<=90 and -180<=lon<=180 and (lat!=0 or lon!=0))


def network_rows(profile, get_param):
    rows = []
    try:
        result = subprocess.run(['ip', '-j', '-4', 'addr'], capture_output=True, text=True, timeout=2, check=True)
        addresses = {a['local'] for interface in json.loads(result.stdout)
                     for a in interface.get('addr_info', []) if a.get('family')=='inet'}
        expected = IPS[profile]
        rows.append(('PASS' if expected in addresses else 'FAIL', 'NET',
                     'local IP '+expected+(' OK' if expected in addresses else ' absent')))
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        rows.append(('FAIL', 'NET', 'IP check unavailable: '+str(error)))
    for label, param in [('UDP config', '/morai_ego_udp_receiver/network_config'),
                         ('Sensor config', '/morai_sensor_udp_receiver/sensor_config')]:
        path = get_param(param, '')
        rows.append(('PASS' if path and Path(path).is_file() else 'WARN', label,
                     str(path) if path else 'param unavailable (receiver not started?)'))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('item', nargs='?', choices=['all', 'network']+list(TOPICS), default='all')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--profile', choices=IPS, default='competition')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=30:
        parser.error('--seconds must be between 1 and 30')
    socket.setdefaulttimeout(2)
    try:
        import rospy
        import rosgraph
        import roslib.message
        rosgraph.Master('/sensor_input_check').getPid()
    except Exception as error:
        print('FAIL ROS master / environment unavailable: '+str(error))
        return 2
    rospy.init_node('sensor_input_check', anonymous=True, disable_signals=True)
    rows = []
    if args.item in ('all', 'network'):
        rows.extend(network_rows(args.profile, rospy.get_param))
    selected = [t for group in (TOPICS if args.item=='all' else [args.item])
                for t in TOPICS.get(group, [])]
    states = {topic: dict(times=deque(maxlen=4096), type_ok=True, invalid=False, details='')
              for _, topic, _, _ in selected}
    lock = threading.Lock()
    def callback(msg, spec):
        label, topic, expected_type, _ = spec
        with lock:
            state = states[topic]
            state['times'].append(time.monotonic())
            state['type_ok'] = msg._connection_header.get('type')==expected_type
            if topic=='/sensor/gps' and state['type_ok']:
                try:
                    cls = roslib.message.get_message_class(expected_type)
                    state['invalid'] = gps_invalid(cls().deserialize(msg._buff))
                except Exception:
                    state['details'] = 'coordinate validity unavailable'
    subscribers = [rospy.Subscriber(spec[1], rospy.AnyMsg, callback, callback_args=spec,
                                   queue_size=1, buff_size=16*1024*1024) for spec in selected]
    if selected:
        print('Checking live inputs for %.1fs (read-only)...'%args.seconds, flush=True)
        deadline = time.monotonic()+args.seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            time.sleep(min(.05, max(0, deadline-time.monotonic())))
    with lock:
        now = time.monotonic()
        for label, topic, _, rate in selected:
            state = states[topic]
            level, reason, hz, age = evaluate(state['times'], now, rate,
                                             type_ok=state['type_ok'], invalid=state['invalid'])
            if state['details'] and level=='PASS':
                level, reason = 'WARN', state['details']
            rows.append((level, label, '%.1fHz age=%s %s'%(hz, '-' if age is None else '%.2fs'%age, reason)))
    for sub in subscribers:
        sub.unregister()
    for level, label, text in rows:
        print('%-4s %-13s %s'%(level, label, text))
    failure = any(row[0]=='FAIL' for row in rows)
    warning = any(row[0]=='WARN' for row in rows)
    print('NOT READY: resolve input failures' if failure else
          'CHECK: flow present; review warnings' if warning else 'INPUT FLOW OK (not a driving approval)')
    print('No control sent. Collision is event-only; Ego/Object GT not required.')
    if any(label=='Vehicle' and level=='FAIL' for level, label, _ in rows):
        print('Competition may wait for the judge relay. Recheck after staff starts it.')
    rospy.signal_shutdown('read-only check complete')
    return 1 if failure else 0


if __name__=='__main__':
    raise SystemExit(main())
