"""Check command invalidation when a static scene changes without a new PO path."""
import json
import io
import zlib
import copy
import math
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path

import rosbag
import rosparam
import rospy
import yaml
from autoware_planning_msgs.msg import StaticObstacle, StaticObstacleScene, Trajectory, TrajectoryPoint
from geometry_msgs.msg import AccelWithCovarianceStamped, Point32
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock

root = Path(__file__).resolve().parents[2]
out = root / "planner_logs/obstacle_scene_regression/latest_scene_cases"
out.mkdir(parents=True, exist_ok=True)
os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11474"
os.environ["ROS_LOG_DIR"] = str(out / "ros")
with socket.socket() as sock:
    assert sock.connect_ex(("127.0.0.1", 11474)) != 0, "private master port occupied"
processes, handles, received, results, snapshots = [], [], [], [], []

def spawn(command, name):
    handle = (out / (name + ".log")).open("w")
    handles.append(handle)
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process

def wait_output(before):
    deadline = time.monotonic() + 3
    while len(received) <= before and time.monotonic() < deadline:
        time.sleep(.01)
    assert len(received) > before, "no output"
    return received[-1]

try:
    spawn(["roscore", "-p", "11474"], "master")
    for _ in range(100):
        try:
            if xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getUri("/check")[0] == 1:
                break
        except OSError:
            pass
        time.sleep(.05)
    rospy.set_param("/use_sim_time", True)
    rospy.init_node("latest_static_scene_check", disable_signals=True)
    ns = "/validator"
    params = yaml.safe_load((root / "configs/planning/vehicle_info.yaml").read_text())
    params.update(default_handling_type=0, publish_diag=False, diag_error_count_threshold=1,
                  display_on_terminal=False, soft_stop_deceleration=-2.0, soft_stop_jerk_lim=-.5,
                  th_traffic_light_timeout=.5, require_obstacle_pointcloud=False, launch_modules=[])
    rosparam.upload_params(ns, params)
    spawn(["rosrun", "autoware_planning_validator", "planning_validator_full_node", "__name:=validator"], "validator")
    pubs = []
    with rosbag.Bag(str(root / "planner_logs/bags/planning_rviz_20260922_204201.bag")) as bag:
        first = {}
        topics = {"/map/vector_map": "input/lanelet_map_bin", "/planning/mission_planning/route": "input/route"}
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            first.setdefault(topic, message)
            if len(first) == 2:
                break
        for topic, message in first.items():
            pub = rospy.Publisher(ns + "/" + topics[topic], type(message), queue_size=1, latch=True)
            pubs.append(pub)
            pub.publish(message)
    clock = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
    odom_pub = rospy.Publisher(ns + "/input/kinematics", Odometry, queue_size=1, latch=True)
    accel_pub = rospy.Publisher(ns + "/input/acceleration", AccelWithCovarianceStamped, queue_size=1, latch=True)
    scene_pub = rospy.Publisher("/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene", StaticObstacleScene, queue_size=1)
    trajectory_pub = rospy.Publisher(ns + "/input/trajectory", Trajectory, queue_size=1)
    sub = rospy.Subscriber(ns + "/output/trajectory", Trajectory, received.append, queue_size=100)
    def receive_snapshot(message):
        for status in message.status:
            if status.name == 'static_scene_check':
                snapshots.append({v.key:v.value for v in status.values})
    sub_snapshot = rospy.Subscriber(ns + '/debug/static_scene_check', DiagnosticArray,
                                    receive_snapshot, queue_size=100)
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x, odom.pose.pose.position.y = 70., 370.
    odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = math.sqrt(.5)
    odom.twist.twist.linear.x = 2.
    trajectory = Trajectory()
    trajectory.header.frame_id = "map"
    for index in range(326):
        point = TrajectoryPoint()
        point.pose.position.x, point.pose.position.y = 70., 365. + index * .2
        point.pose.orientation = odom.pose.pose.orientation
        point.longitudinal_velocity_mps = 5.
        trajectory.points.append(point)
    time.sleep(1)
    clock.publish(Clock(rospy.Time(1000)))
    odom_pub.publish(odom)
    accel_pub.publish(AccelWithCovarianceStamped())
    scene = StaticObstacleScene()
    scene.header.frame_id, scene.header.stamp = "map", rospy.Time(1000)
    scene_pub.publish(scene)
    for _ in range(15):
        trajectory.header.stamp = rospy.Time(1000)
        trajectory_pub.publish(trajectory)
        time.sleep(.15)
        if received:
            break
    assert received and all(p.longitudinal_velocity_mps == 5 for p in received[-1].points)
    results.append({"case": "initial_free_path", "passed": True})
    obstacle = StaticObstacle(hard_margin=.1, soft_margin=.2)
    obstacle.polygon.points = [Point32(x=x, y=y) for x, y in [(69.,390.),(71.,390.),(71.,395.),(69.,395.)]]
    scene.obstacles = [obstacle]
    scene.header.stamp = rospy.Time.from_sec(1000.1)
    before = len(received)
    clock.publish(Clock(scene.header.stamp))
    scene_pub.publish(scene)  # Deliberately do not send another trajectory.
    result = wait_output(before)
    stopped = [p.pose.position.y for p in result.points if abs(p.longitudinal_velocity_mps) < .01]
    assert stopped and min(stopped) < 386
    assert result.points[25].longitudinal_velocity_mps > 0
    assert [p.pose for p in result.points] == [p.pose for p in trajectory.points]
    results.append({"case": "new_obstacle_without_new_path", "passed": True, "stop_y": min(stopped)})
    before = len(received)
    trajectory_pub.publish(trajectory)
    result = wait_output(before)
    assert any(p.longitudinal_velocity_mps == 0 for p in result.points)
    results.append({"case": "stale_positive_path_cannot_restore_motion", "passed": True})
    detour = copy.deepcopy(trajectory)
    detour.header.stamp = scene.header.stamp
    for point in detour.points:
        phase = max(0., min(1., (point.pose.position.y - 374.) / 10.))
        point.pose.position.x = 70. - 2. * (1. - math.cos(math.pi * phase))
        yaw = math.atan2(1., -.2 * math.pi * math.sin(math.pi * phase))
        point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
    before = len(received)
    trajectory_pub.publish(detour)
    result = wait_output(before)
    assert all(p.longitudinal_velocity_mps == 5 for p in result.points)
    results.append({"case": "new_safe_detour_replaces_braking_path", "passed": True})
    scene.obstacles[0].polygon.points = [Point32(x=x,y=y) for x,y in [(69.,372.),(71.,372.),(71.,376.),(69.,376.)]]
    scene.header.stamp = rospy.Time.from_sec(1000.2)
    before = len(received)
    clock.publish(Clock(scene.header.stamp))
    scene_pub.publish(scene)
    result = wait_output(before)
    assert all(p.longitudinal_velocity_mps == 0 for p in result.points)
    results.append({"case": "current_footprint_overlap_emergency_stop", "passed": True})
    scene.obstacles = []
    scene.header.stamp = rospy.Time.from_sec(1000.3)
    clock.publish(Clock(scene.header.stamp))
    scene_pub.publish(scene)
    time.sleep(.1)
    before = len(received)
    trajectory.header.stamp = scene.header.stamp
    trajectory_pub.publish(trajectory)
    result = wait_output(before)
    assert all(p.longitudinal_velocity_mps == 5 for p in result.points)
    results.append({"case": "cleared_scene_accepts_fresh_path", "passed": True})
    deadline = time.monotonic() + 2
    while len(snapshots) < len(received) and time.monotonic() < deadline:
        time.sleep(.01)
    indexed = {int(s['publication_index']):s for s in snapshots}
    for message in received:
        assert message.header.seq in indexed, (message.header.seq, list(indexed))
        snapshot = indexed[message.header.seq]
        normalized = copy.deepcopy(message)
        normalized.header.seq = 0
        buf = io.BytesIO()
        normalized.serialize(buf)
        assert zlib.crc32(buf.getvalue()) == int(snapshot['trajectory_crc32'])
    results.append({"case": "each_output_has_exact_scene_snapshot", "passed": True,
                    "outputs_checked": len(received)})
    (out / "results.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
finally:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in reversed(processes):
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    for handle in handles:
        handle.close()
