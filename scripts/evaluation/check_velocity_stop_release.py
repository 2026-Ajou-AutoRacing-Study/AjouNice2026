"""Exercise stop/release transitions against the installed ROS1 velocity smoother."""
import argparse
import json
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path

import rosparam
import rospy
import yaml
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
args.output.mkdir(parents=True, exist_ok=False)
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11475'
os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
with socket.socket() as sock:
    assert sock.connect_ex(('127.0.0.1', 11475)) != 0, 'private master occupied'
processes, handles, outputs, results = [], [], [], []

def spawn(command, name):
    handle = (args.output / (name + '.log')).open('w')
    handles.append(handle)
    processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                      start_new_session=True))

try:
    spawn(['roscore', '-p', '11475'], 'master')
    for _ in range(100):
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/test')[0] == 1:
                break
        except OSError:
            pass
        time.sleep(.05)
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('velocity_stop_release_check', disable_signals=True)
    common = root / 'planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/common'
    for config in [root / 'configs/planning/vehicle_info.yaml', common / 'common.param.yaml',
                   common / 'nearest_search.param.yaml',
                   common / 'autoware_velocity_smoother/velocity_smoother.param.yaml',
                   common / 'autoware_velocity_smoother/JerkFiltered.param.yaml']:
        rosparam.upload_params('/smoother', yaml.safe_load(config.read_text()))
    rospy.set_param('/smoother/algorithm_type', 'JerkFiltered')
    rospy.set_param('/smoother/publish_debug_trajs', False)
    spawn(['rosrun', 'autoware_velocity_smoother', 'velocity_smoother_node',
           '__name:=smoother'], 'smoother')
    clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
    odom_pub = rospy.Publisher('/localization/kinematic_state', Odometry, queue_size=1, latch=True)
    accel_pub = rospy.Publisher('/smoother/input/acceleration', AccelWithCovarianceStamped,
                                queue_size=1, latch=True)
    mode_pub = rospy.Publisher('/smoother/input/operation_mode_state', OperationModeState,
                               queue_size=1, latch=True)
    path_pub = rospy.Publisher('/smoother/input/trajectory', Trajectory, queue_size=1)
    sub = rospy.Subscriber('/smoother/output/trajectory', Trajectory, outputs.append, queue_size=100)
    time.sleep(1)
    tick, position = 0, 0.

    def step(target, measured):
        global tick, position
        tick += 1
        position += measured * .1
        stamp = rospy.Time.from_sec(1000 + tick * .1)
        clock.publish(Clock(stamp))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'map'
        odom.pose.pose.position.x = position
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = measured
        odom_pub.publish(odom)
        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel_pub.publish(accel)
        mode_pub.publish(OperationModeState(mode=OperationModeState.AUTONOMOUS,
                                            is_autoware_control_enabled=True))
        time.sleep(.03)
        path = Trajectory()
        path.header.stamp, path.header.frame_id = stamp, 'map'
        for i in range(501):
            p = TrajectoryPoint()
            p.pose.position.x = position - 5 + i * .2
            p.pose.orientation.w = 1.
            p.longitudinal_velocity_mps = target
            path.points.append(p)
        before = len(outputs)
        path_pub.publish(path)
        deadline = time.monotonic() + 3
        while len(outputs) == before and time.monotonic() < deadline:
            time.sleep(.01)
        assert len(outputs) > before, 'no smoother output'
        closest = min(outputs[-1].points, key=lambda p: abs(p.pose.position.x - position))
        return closest.longitudinal_velocity_mps

    for _ in range(3):
        speed = step(8., 5.)
    results.append(dict(case='normal_motion', speed=speed, passed=speed > 4.))
    for _ in range(3):
        speed = step(0., 5.)
    results.append(dict(case='upstream_stop_while_still_moving', speed=speed, passed=abs(speed) < .01))
    speed = step(8., 5.)
    results.append(dict(case='release_stop_from_measured_motion', speed=speed, passed=speed > 4.))
    for _ in range(3):
        speed = step(0., 0.)
    results.append(dict(case='stationary_stop_remains_stop', speed=speed, passed=abs(speed) < .01))
    speed = step(8., 0.)
    results.append(dict(case='stationary_release_engages', speed=speed, passed=speed > .1))
    (args.output / 'results.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    assert all(r['passed'] for r in results), 'stop/release transition regression'
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    for handle in handles:
        handle.close()
