#!/usr/bin/env python3
"""Isolated real-binary UDP gate test. Never uses MORAI or the production master."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[2]


def reserve_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    return sock


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def wait_for(predicate, detail, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError(detail)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receiver', type=Path, default=ROOT / 'base_ws/install_aw149/lib/sensor_interface/morai_ego_udp_receiver')
    parser.add_argument('--sender', type=Path, default=ROOT / 'base_ws/install_aw149/lib/sensor_interface/morai_ctrl_cmd_udp_sender')
    args = parser.parse_args()
    for path in (args.receiver, args.sender):
        if not path.is_file():
            parser.error('Binary missing: ' + str(path))
    with socket.socket() as temporary:
        temporary.bind(('127.0.0.1', 0))
        master_port = temporary.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % master_port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    processes = []
    with tempfile.TemporaryDirectory(prefix='competition-udp-test-') as folder:
        temp = Path(folder)
        os.environ['ROS_LOG_DIR'] = str(temp / 'roslogs')
        try:
            master = subprocess.Popen(['roscore', '-p', str(master_port)],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(master)
            rpc = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])

            def master_ready():
                try:
                    return rpc.getPid('/competition_test')[0] == 1
                except OSError:
                    return False

            wait_for(master_ready, 'isolated master unavailable', 8)
            import rospy
            from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList, CollisionData
            from geometry_msgs.msg import TwistStamped
            rospy.init_node('competition_udp_test', disable_signals=True)
            source = json.loads((ROOT / 'configs/network/ego_network_competition_local.json').read_text())
            base = source['egoNetworkData'][0]['listEgoNetworkInfo']
            template = {entry['PUBSUB_TYPE']: entry for entry in base}
            classes = {770: EgoVehicleStatus, 772: ObjectStatusList, 261: CollisionData, 295: EgoVehicleStatus}
            sizes = {770: 229, 772: 2160, 261: 181, 295: 181}
            # Also prove legacy defaults and each switch independently still work.
            for ego_on, object_on in ((True, True), (False, True), (True, False), (False, False)):
                phase = 'p%d%d' % (ego_on, object_on)
                sockets, ports, topics, entries, received, subscriptions = {}, {}, {}, [], {}, []
                enabled = {770: ego_on, 772: object_on, 261: True, 295: True}
                process = None
                try:
                    for kind in classes:
                        sockets[kind] = reserve_udp()
                        ports[kind] = sockets[kind].getsockname()[1]
                        topics[kind] = '/competition_test/%s/t%d' % (phase, kind)
                        entry = copy.deepcopy(template[kind])
                        config = entry['config']
                        config['commType'] = 1
                        config['udpConfig'].update(destinationIP='127.0.0.1', destinationPort=ports[kind])
                        config['rosConfig']['Topic'] = topics[kind]
                        entries.append(entry)
                        subscriptions.append(rospy.Subscriber(topics[kind], classes[kind],
                            lambda msg, k=kind: received.__setitem__(k, msg)))
                    # Nonreceiver entries must be ignored, even with unusable endpoint data.
                    extra = copy.deepcopy(template[771])
                    extra['config']['commType'] = 1
                    extra['config']['udpConfig']['destinationIP'] = 'not-a-receiver-address'
                    entries.append(extra)
                    path = temp / (phase + '.json')
                    path.write_text(json.dumps({'egoNetworkData': [{'listEgoNetworkInfo': entries}]}))
                    name = 'receiver_' + phase
                    rospy.set_param('/' + name + '/network_config', str(path))
                    # First phase omits flags: development defaults must remain ON.
                    if phase != 'p11':
                        rospy.set_param('/' + name + '/enable_ego_vehicle_status', ego_on)
                        rospy.set_param('/' + name + '/enable_object_info', object_on)
                    for kind in classes:
                        if enabled[kind]:
                            sockets[kind].close()
                    with (temp / (phase + '.log')).open('w') as log:
                        process = subprocess.Popen([str(args.receiver), '__name:=' + name],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                        processes.append(process)
                        def advertised():
                            assert process.poll() is None, (temp / (phase + '.log')).read_text()
                            pubs = dict(rpc.getSystemState('/competition_test')[2][0])
                            return all('/' + name in pubs.get(topics[k], []) for k in classes if enabled[k])
                        wait_for(advertised, 'allowed publishers missing')
                        time.sleep(.2)
                        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                            for _ in range(20):
                                for kind in classes:
                                    packet = bytearray(sizes[kind])
                                    packet[-2:] = b'\r\n'
                                    if kind in (770, 295):
                                        struct.pack_into('<ii', packet, 27, int(time.time()) + 120, 0)
                                        struct.pack_into('<fff', packet, 101, 36., 0., 0.)
                                    sender.sendto(packet, ('127.0.0.1', ports[kind]))
                                time.sleep(.025)
                        wait_for(lambda: all(k in received for k in classes if enabled[k]), 'allowed packet missing')
                        pubs = dict(rpc.getSystemState('/competition_test')[2][0])
                        for kind in classes:
                            if not enabled[kind]:
                                assert kind not in received, 'forbidden ROS output'
                                assert '/' + name not in pubs.get(topics[kind], []), 'forbidden publisher'
                                # This socket occupied the forbidden endpoint BEFORE receiver startup.
                                # Starting successfully proves receiver did not try to bind that port.
                                sockets[kind].settimeout(.2)
                                assert sockets[kind].recv(4096), 'test did not send forbidden packets'
                        assert abs(received[295].velocity.x - 10.) < 1e-6
                        assert abs(time.time() - received[295].header.stamp.to_sec()) < 2.
                        print('PASS %s: enabled streams delivered; disabled ports unbound/publishers absent' % phase)
                    stop(process)
                finally:
                    if process is not None:
                        stop(process)
                    for sub in subscriptions:
                        sub.unregister()
                    for sock in sockets.values():
                        sock.close()

            # A missing mandatory channel must still fail startup, not silently pass.
            with reserve_udp() as held:
                entry = copy.deepcopy(template[295])
                entry['config']['udpConfig'].update(destinationIP='127.0.0.1', destinationPort=held.getsockname()[1])
                path = temp / 'missing_collision.json'
                path.write_text(json.dumps({'egoNetworkData': [{'listEgoNetworkInfo': [entry]}]}))
                held.close()
                rospy.set_param('/missing_required/network_config', str(path))
                rospy.set_param('/missing_required/enable_ego_vehicle_status', False)
                rospy.set_param('/missing_required/enable_object_info', False)
                with (temp / 'missing.log').open('w') as log:
                    process = subprocess.Popen([str(args.receiver), '__name:=missing_required'],
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    processes.append(process)
                    assert process.wait(timeout=5) != 0, 'missing Collision accepted'
                print('PASS missing mandatory Collision: nonzero startup failure retained')

            # CtrlCmd is sent only to a private random-port loopback collector.
            with reserve_udp() as collector, reserve_udp() as local:
                remote_port, local_port = collector.getsockname()[1], local.getsockname()[1]
                local.close()
                entry = copy.deepcopy(template[771])
                entry['EgoCtrlConfig']['udpConfig'].update(hostIP='127.0.0.1', hostPort=remote_port,
                    destinationIP='127.0.0.1', destinationPort=local_port)
                path = temp / 'sender.json'
                path.write_text(json.dumps({'egoNetworkData': [{'listEgoNetworkInfo': [entry]}]}))
                rospy.set_param('/test_ctrl/network_config', str(path))
                with (temp / 'sender.log').open('w') as log:
                    process = subprocess.Popen([str(args.sender), '__name:=test_ctrl'],
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    processes.append(process)
                    publisher = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
                    wait_for(lambda: publisher.get_num_connections() > 0, 'test control subscriber missing')
                    collector.settimeout(.15)
                    try:
                        collector.recv(4096)
                        raise AssertionError('sender emitted command without ROS input')
                    except socket.timeout:
                        pass
                    message = CtrlCmd()
                    message.longlCmdType = 1
                    message.accel, message.brake, message.steering = .2, .3, math.radians(10)
                    publisher.publish(message)
                    collector.settimeout(2)
                    packet, peer = collector.recvfrom(4096)
                    assert len(packet) == 55 and packet[-2:] == b'\r\n'
                    assert struct.unpack_from('<bbb', packet, 30) == (2, 4, 1)
                    assert abs(struct.unpack_from('<f', packet, 49)[0] - .25) < 1e-6
                    assert peer[1] == local_port
                    publisher.unregister()
                    stop(process)
                    print('PASS control: no unsolicited command; mode=2/type=1, unchanged steering and endpoints')
            rospy.signal_shutdown('isolated UDP test complete')
        finally:
            for process in reversed(processes):
                stop(process)


if __name__ == '__main__':
    main()
