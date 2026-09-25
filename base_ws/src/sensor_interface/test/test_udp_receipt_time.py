#!/usr/bin/env python3
"""Isolated installed-binary UDP timestamp regression; no production ROS master.

Run after sourcing install_aw149. Uses temporary configs and loopback ports.
"""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import xmlrpc.client


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    root = Path(__file__).resolve().parents[4]
    master_port = port()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % master_port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    processes = []
    with tempfile.TemporaryDirectory(prefix='udp-receipt-test-') as temp:
        os.environ['ROS_LOG_DIR'] = temp
        try:
            processes.append(subprocess.Popen(['roscore', '-p', str(master_port)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                             start_new_session=True))
            for _ in range(100):
                try:
                    xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')
                    break
                except OSError:
                    time.sleep(.05)
            import rospy
            from geometry_msgs.msg import TwistStamped
            from morai_msgs.msg import EgoVehicleStatus, GPSMessage, ObjectStatusList, CollisionData
            from sensor_msgs.msg import Imu, CompressedImage
            rospy.init_node('receipt_test', disable_signals=True)
            endpoints = {}

            def rewrite(value):
                if isinstance(value, dict):
                    if 'udpConfig' in value:
                        udp_port = port()
                        value['udpConfig']['destinationIP'] = '127.0.0.1'
                        value['udpConfig']['destinationPort'] = udp_port
                        endpoints[value.get('rosConfig', {}).get('Topic', '')] = udp_port
                    for child in value.values():
                        rewrite(child)
                elif isinstance(value, list):
                    for child in value:
                        rewrite(child)

            for node, config, parameter in [
                ('morai_sensor_udp_receiver', 'configs/sensors/Sensor_competition_lan.json', 'sensor_config'),
                ('morai_ego_udp_receiver', 'configs/network/ego_network_competition_lan.json', 'network_config'),
            ]:
                data = json.loads((root / config).read_text())
                rewrite(data)
                path = Path(temp) / (node + '.json')
                path.write_text(json.dumps(data))
                rospy.set_param('/' + node + '/' + parameter, str(path))
                processes.append(subprocess.Popen(
                    [str(root / 'base_ws/install_aw149/lib/sensor_interface' / node), '__name:=' + node],
                    stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True))
            received = {}
            subscribers = []
            for topic, cls in [('/Ego_topic', EgoVehicleStatus), ('/Competition_topic', EgoVehicleStatus),
                               ('/Object_topic', ObjectStatusList), ('/CollisionData', CollisionData),
                               ('/competition/raw_twist', TwistStamped),
                               ('/sensor/imu', Imu), ('/sensor/gps', GPSMessage),
                               ('/sensor/camera/front/compressed', CompressedImage)]:
                subscribers.append(rospy.Subscriber(topic, cls,
                    lambda m, key=topic: received.setdefault(key, (m, time.time()))))
            time.sleep(1)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            for offset in [120, -120]:
                # Drain the prior phase's ROS callbacks before resetting observations.
                time.sleep(.3)
                received.clear()
                started = time.time()
                remote = int(started) + offset
                packets = {}
                for topic, size in [('/Ego_topic', 229), ('/Competition_topic', 181)]:
                    packet = bytearray(size)
                    struct.pack_into('<ii', packet, 27, remote, 0)
                    packet[-2:] = b'\r\n'
                    packets[topic] = packet
                # Competition packet offsets: vel_xyz=101, ang_vel_xyz=113.
                struct.pack_into('<fff', packets['/Competition_topic'], 101, 36.0, 0.0, 0.0)
                struct.pack_into('<fff', packets['/Competition_topic'], 113, 0.0, 0.0, 0.25)
                imu = bytearray(115)
                struct.pack_into('<ii', imu, 25, remote, 0)
                struct.pack_into('<d', imu, 33, 1.0)
                imu[-2:] = b'\r\n'
                packets['/sensor/imu'] = imu
                for topic, size, stamp_offset in [('/Object_topic', 2160, 30),
                                                   ('/CollisionData', 181, 31)]:
                    packet = bytearray(size)
                    struct.pack_into('<ii', packet, stamp_offset, remote, 0)
                    packet[-2:] = b'\r\n'
                    packets[topic] = packet
                packets['/sensor/gps'] = b'$GPGGA,120000,3723.0,N,12700.0,E,1,10,1.0,10.0,M,0,M,,*00'
                packets['/sensor/camera/front/compressed'] = (
                    b'MOR' + struct.pack('<iiii', remote, 0, 0, 4) + b'\xff\xd8\xff\xd9EI')
                for _ in range(40):
                    for topic, packet in packets.items():
                        sock.sendto(packet, ('127.0.0.1', endpoints[topic]))
                    if all(t in received for t in packets):
                        break
                    time.sleep(.05)
                for topic in packets:
                    assert topic in received, 'no output: ' + topic
                    message, observed = received[topic]
                    stamp = message.header.stamp.to_sec()
                    assert started <= stamp <= observed, (topic, stamp, started, observed)
                    assert observed - stamp < .5, (topic, observed - stamp)
                assert '/competition/raw_twist' in received
                twist, observed = received['/competition/raw_twist']
                assert abs(twist.twist.linear.x - 10.0) < 1.0e-6
                assert abs(twist.twist.angular.z - 0.25) < 1.0e-6
                assert twist.header.frame_id == 'base_link'
                assert observed - twist.header.stamp.to_sec() < .5
                print('PASS: remote clock offset %+d s, all streams use ROS receipt time' % offset)
            sock.close()
            rospy.signal_shutdown('test complete')
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


if __name__ == '__main__':
    main()
