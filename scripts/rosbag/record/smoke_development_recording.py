#!/usr/bin/env python3
"""Synthetic functional smoke on a private localhost ROS master, never a vehicle test."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
JPEG_SENTINEL = b'unchanged-synthetic-compressed-bytes'


def publisher():
    import rospy
    from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from velodyne_msgs.msg import VelodynePacket, VelodyneScan
    rospy.init_node('synthetic_development_inputs', anonymous=True)
    messages = {}
    for camera in ('front', 'front_left', 'front_right', 'back'):
        msg = CompressedImage(format='jpeg', data=JPEG_SENTINEL)
        msg.header.frame_id = camera
        messages['/sensor/camera/' + camera + '/compressed'] = msg
    scan = VelodyneScan()
    scan.header.frame_id = 'lidar1'
    scan.packets = [VelodynePacket() for _ in range(50)]
    messages['/sensor/lidar1/packets'] = scan
    messages['/sensor/lidar1'] = PointCloud2()
    messages['/sensor/imu'] = Imu()
    messages['/localization/kinematic_state'] = Odometry()
    # String stand-ins deliberately test selection only, not message semantics.
    for topic in ('/sensor/gps', '/Competition_topic', '/planning/trajectory', '/ctrl_cmd',
                  '/perception/object_recognition/detection/objects',
                  '/perception/object_recognition/tracking/measurement_objects',
                  '/perception/object_recognition/tracking/objects',
                  '/perception/object_recognition/objects'):
        messages[topic] = String(data='synthetic-topic-selection-only')
    messages['/perception/bevfusion/frame_timing'] = String(data=json.dumps({
        'server_inference_ms': 60., 'counters': {'callbacks': 1, 'camera_sync_reject': 0}}))
    pubs = {topic: rospy.Publisher(topic, type(msg), queue_size=5) for topic, msg in messages.items()}
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for topic, msg in messages.items():
            if hasattr(msg, 'header'):
                msg.header.stamp = now
            if topic == '/sensor/lidar1/packets':
                for packet in msg.packets:
                    packet.stamp = now
            pubs[topic].publish(msg)
        rate.sleep()


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publisher', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT/'planner_logs/analysis/development_recorder_smoke')
    args = parser.parse_args()
    if args.publisher:
        publisher()
        return 0
    directory = args.output_dir.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    uri = 'http://127.0.0.1:' + str(port)
    env = dict(os.environ, ROS_MASTER_URI=uri, ROS_IP='127.0.0.1',
               ROS_HOME=str(directory/'ros_home'), ROS_LOG_DIR=str(directory/'ros_logs'))
    env.pop('ROS_HOSTNAME', None)
    core = pub = recording = None
    try:
        with (directory/'core.log').open('w') as log:
            core = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = False
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            try:
                ready = xmlrpc.client.ServerProxy(uri).getPid('/private_smoke')[0] == 1
            except OSError:
                pass
            if ready:
                break
            if core.poll() is not None:
                raise RuntimeError('Private master exited')
            time.sleep(.1)
        if not ready:
            raise RuntimeError('Private master not ready')
        with (directory/'publisher.log').open('w') as log:
            pub = subprocess.Popen([sys.executable, str(Path(__file__)), '--publisher'], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(1)
        with (directory/'recorder_console.log').open('w') as log:
            recording = subprocess.Popen([sys.executable, str(HERE/'record_development_bag.py'),
                '--duration', '8', '--minimum-free-gib', '2', '--no-host-gpu',
                '--session-id', 'private-synthetic-smoke', '--output-dir', str(directory/'recordings')],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            recording.wait(timeout=35)
        if recording.returncode:
            raise RuntimeError('Recorder failed; inspect recorder_console.log')
        if pub.poll() is not None:
            raise RuntimeError('Synthetic publisher exited unexpectedly')
        folders = list((directory/'recordings').iterdir())
        if len(folders) != 1:
            raise RuntimeError('Expected one uniquely named recording')
        folder = folders[0]
        report = json.loads((folder/'recording_audit.json').read_text())
        assert report['missing_core'] == [], report['missing_core']
        assert report['analysis_capabilities']['raw_sensor_replay']['recorded']
        assert report['analysis_capabilities']['detector_stage_timing']['recorded']
        assert report['lidar_packets_per_scan_histogram'].get('50', 0) > 0
        assert report['host_telemetry']['samples'] >= 2
        assert report['host_telemetry']['ended']
        import rosbag
        original_bytes = 0
        with rosbag.Bag(str(folder/'development.bag')) as bag:
            for _, msg, _ in bag.read_messages(topics=['/sensor/camera/front/compressed']):
                assert bytes(msg.data) == JPEG_SENTINEL
                original_bytes += 1
        assert original_bytes > 0
        result = dict(status='functional_smoke_pass', ros_master_uri=uri,
                      bag=str(folder/'development.bag'), original_compressed_messages=original_bytes,
                      telemetry=report['host_telemetry'],
                      limits='Synthetic serializers/selection/lifecycle only; not real LAN, GPU, '
                             'sensor synchronization, Windows, or driving performance certification.')
        (directory/'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        return 0
    finally:
        # Only subprocess groups created above; never contact the default master.
        stop(recording)
        stop(pub)
        stop(core)


if __name__ == '__main__':
    raise SystemExit(main())
