#!/usr/bin/env python3
"""Private-master startup/map transport smoke; NO fake driving inputs, NO driving claim."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

root = Path(__file__).resolve().parents[1]
out = root/'logs/install-runtime'
out.mkdir(parents=True, exist_ok=True)
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
                  ROS_HOME=tempfile.mkdtemp(prefix='ajounice-install-ros-'))
processes, handles = [], []


def start(command, name):
    handle = (out/(name+'.log')).open('w')
    handles.append(handle)
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process


try:
    master_process = start(['roscore', '-p', str(port)], 'master')
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    for attempt in range(100):
        try:
            assert master.getPid('/install_smoke')[0] == 1
            break
        except Exception:
            time.sleep(.1)
    else:
        raise RuntimeError('private master unavailable')
    # Existing test vehicle only for startup. NOT an AjouNice approved vehicle config.
    vehicle = root/'install_isolated/share/autoware_vehicle_info_utils/config/vehicle_info.yaml'
    launch = start(['roslaunch', 'ajounice_planning_integration', 'planning.launch',
                    'repo_root:='+str(root.parent), 'vehicle_param_path:='+str(vehicle),
                    'start_requester:=false'], 'planning')
    import rospy
    from autoware_map_msgs.msg import LaneletMapBin
    rospy.init_node('install_smoke', anonymous=True, disable_signals=True)
    map_message = rospy.wait_for_message('/map/vector_map', LaneletMapBin, timeout=45)
    assert map_message.header.frame_id == 'map' and len(map_message.data)>1000
    assert not bytes(map_message.data[:64]).lstrip().startswith(b'<'), 'raw XML masquerades as binary map'
    time.sleep(12)
    assert launch.poll() is None, 'planning launch terminated'
    system = master.getSystemState('/install_smoke')[2]
    pubs, subs, services = [dict(x) for x in system]
    assert len(pubs.get('/map/vector_map', [])) == 1, 'duplicate map publisher'
    for service in ('/planning/set_waypoint_route', '/planning/set_lanelet_route'):
        assert service in services, 'missing RouteSelector service '+service
    for topic in ('/localization/kinematic_state', '/perception/object_recognition/objects'):
        assert topic in subs, 'missing canonical subscriber '+topic
        assert topic not in pubs, 'smoke must not fabricate production inputs'
    text = (out/'planning.log').read_text(errors='replace')
    assert 'process has died' not in text and 'Failed to load library' not in text
    result = dict(scope='FULL_GRAPH_STARTUP_AND_BINARY_MAP_ONLY_NO_ROUTE_OR_DRIVING_INPUTS',
                  binary_map_bytes=len(map_message.data), map_publishers=pubs['/map/vector_map'],
                  services=sorted(services), publishers=pubs, subscribers=subs,
                  test_vehicle_only=str(vehicle), ros_home=os.environ['ROS_HOME'])
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({key:result[key] for key in ('scope', 'binary_map_bytes', 'map_publishers', 'ros_home')}, indent=2))
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    for handle in handles:
        handle.close()
