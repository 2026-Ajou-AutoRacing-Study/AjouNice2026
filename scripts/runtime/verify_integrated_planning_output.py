#!/usr/bin/env python3
"""Drive the installed AjouNice planning launch with conservative synthetic inputs."""

import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

import roslaunch
import rospkg
import rospy
import tf2_ros
import yaml
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_planning_msgs.msg import LaneletRoute, RouteState, Trajectory
from autoware_planning_validator.msg import PlanningValidatorStatus
from geometry_msgs.msg import AccelWithCovarianceStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


repo = Path(os.environ["AJOUNICE_ROOT"]).resolve()
result_path = Path(os.environ["AW149_OUTPUT_PROBE_RESULT"]).resolve()
route_path = repo / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v61_competition_route.yaml"
route_config = yaml.safe_load(route_path.read_text())
selected = route_config["route_a"]


def make_pose(value):
    pose = Pose()
    pose.position.x = float(value["x"])
    pose.position.y = float(value["y"])
    pose.position.z = float(value.get("z", 0.0))
    q = quaternion_from_euler(0.0, 0.0, math.radians(float(value["yaw_deg"])))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


packages = rospkg.RosPack()
launch_file = Path(packages.get_path("ajounice_planning_integration")) / "launch/morai_planning_stack.launch"
launch_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(launch_id)
parent = roslaunch.parent.ROSLaunchParent(
    launch_id,
    [(str(launch_file), [
        "start_localization:=false",
        "start_sensor_tf:=false",
        "start_object_pipeline:=false",
        "start_detector:=false",
        "start_traffic_light:=false",
        "start_requester:=false",
        "start_controller:=false",
    ])],
)
deaths = []


class Monitor(roslaunch.pmon.ProcessListener):
    def process_died(self, name, exit_code):
        deaths.append({"name": name, "exit_code": exit_code})


parent.process_listeners = [Monitor()]
rospy.init_node("aw149_integrated_output_probe", anonymous=True)
latest = {}
counts = {"map": 0, "route": 0, "route_state": 0, "final": 0, "validation": 0}


def receive(message, key):
    latest[key] = message
    counts[key] += 1


subscriptions = [
    rospy.Subscriber("/map/vector_map", LaneletMapBin, receive, callback_args="map", queue_size=2),
    rospy.Subscriber("/planning/mission_planning/route", LaneletRoute, receive, callback_args="route", queue_size=2),
    rospy.Subscriber("/planning/route_state", RouteState, receive, callback_args="route_state", queue_size=2),
    rospy.Subscriber("/planning/trajectory", Trajectory, receive, callback_args="final", queue_size=10),
    rospy.Subscriber("/planning/validation_status", PlanningValidatorStatus, receive, callback_args="validation", queue_size=10),
]


def publisher(topic, message_type):
    return rospy.Publisher(topic, message_type, queue_size=1, latch=True)


odom_pub = publisher("/localization/kinematic_state", Odometry)
accel_pub = publisher("/localization/acceleration", AccelWithCovarianceStamped)
objects_pub = publisher("/perception/object_recognition/objects", PredictedObjects)
signals_pub = publisher("/perception/traffic_light_recognition/traffic_signals", TrafficLightGroupArray)
grid_pub = publisher("/perception/occupancy_grid_map/map", OccupancyGrid)
cloud_pub = publisher("/perception/obstacle_segmentation/pointcloud", PointCloud2)
mode_pub = publisher("/system/operation_mode/state", OperationModeState)
broadcaster = tf2_ros.TransformBroadcaster()

odom = Odometry(header=Header(frame_id="map"), child_frame_id="base_link")
odom.pose.pose = make_pose(route_config["start"]["map_pose"])
odom.twist.twist.linear.x = 2.0
grid = OccupancyGrid(header=Header(frame_id="map"))
grid.info.resolution = 1.0
grid.info.width = 500
grid.info.height = 500
grid.info.origin.position.x = odom.pose.pose.position.x - 250.0
grid.info.origin.position.y = odom.pose.pose.position.y - 250.0
grid.info.origin.orientation.w = 1.0
grid.data = [0] * (grid.info.width * grid.info.height)


def tick():
    now = rospy.Time.now()
    odom.header.stamp = now
    odom_pub.publish(odom)
    accel_pub.publish(AccelWithCovarianceStamped(header=Header(frame_id="base_link", stamp=now)))
    objects_pub.publish(PredictedObjects(header=Header(frame_id="map", stamp=now)))
    signals = TrafficLightGroupArray()
    signals.stamp.sec, signals.stamp.nanosec = now.secs, now.nsecs
    signals_pub.publish(signals)
    grid.header.stamp = now
    grid_pub.publish(grid)
    far_point = [(odom.pose.pose.position.x + 100.0, odom.pose.pose.position.y + 100.0, 1.0)]
    cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id="map", stamp=now), far_point))
    mode_pub.publish(OperationModeState(
        mode=OperationModeState.AUTONOMOUS,
        is_autoware_control_enabled=True,
        is_autonomous_mode_available=True,
    ))
    transform = TransformStamped(header=Header(frame_id="map", stamp=now), child_frame_id="base_link")
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation
    broadcaster.sendTransform(transform)
    parent.spin_once()
    if deaths:
        raise RuntimeError("integrated launch process died: " + str(deaths))
    time.sleep(0.05)


result = {
    "scope": "AJOUNICE_INSTALLED_LAUNCH_SYNTHETIC_INPUT_OUTPUT_ONLY",
    "pass": False,
    "launch": str(launch_file),
    "route_fixture": str(route_path),
    "counts": counts,
    "process_deaths_before_shutdown": deaths,
    "not_proven": [
        "live MORAI topic, frame, timestamp, or unit semantics",
        "production perception accuracy",
        "vehicle controller handoff",
        "closed-loop driving",
    ],
}

try:
    parent.start()
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline and "map" not in latest:
        tick()
    if "map" not in latest:
        raise RuntimeError("installed integration launch did not publish v61 map")

    rospy.wait_for_service("/planning/set_waypoint_route", timeout=60.0)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        tick()
        if "route_state" in latest and latest["route_state"].state == RouteState.UNSET:
            break
    if "route_state" not in latest or latest["route_state"].state != RouteState.UNSET:
        raise RuntimeError("mission planner did not reach UNSET readiness")

    request = SetWaypointRouteRequest()
    request.header = Header(frame_id="map", stamp=rospy.Time.now())
    request.uuid.uuid = list(uuid.UUID(selected["uuid"]).bytes)
    request.allow_modification = bool(selected["allow_modification"])
    for reference in selected["waypoint_refs"]:
        request.waypoints.append(make_pose(route_config["poses"][reference]))
    request.goal_pose = make_pose(route_config["poses"][selected["goal_ref"]])
    response = rospy.ServiceProxy("/planning/set_waypoint_route", SetWaypointRoute)(request)
    if not response.status.success:
        raise RuntimeError("route rejected: " + str(response))

    started = time.monotonic()
    deadline = started + 120.0
    while time.monotonic() < deadline:
        tick()
        if counts["final"] >= 10 and counts["validation"] >= 10:
            break
    if counts["final"] < 10 or counts["validation"] < 10:
        raise RuntimeError("insufficient integrated outputs: " + str(counts))

    expected_lanelets = selected["expected_path_lanelet_ids"]
    observed_lanelets = [segment.preferred_primitive.id for segment in latest["route"].segments]
    if observed_lanelets != expected_lanelets:
        raise RuntimeError("integrated route mismatch")
    trajectory = latest["final"]
    if trajectory.header.frame_id != "map" or len(trajectory.points) < 3:
        raise RuntimeError("invalid integrated trajectory shape")
    if not all(math.isfinite(value) for point in trajectory.points for value in (
        point.pose.position.x,
        point.pose.position.y,
        point.longitudinal_velocity_mps,
        point.acceleration_mps2,
    )):
        raise RuntimeError("non-finite integrated trajectory")
    flags = {
        name: getattr(latest["validation"], name)
        for name in latest["validation"].__slots__
        if name.startswith("is_valid_")
    }
    if not flags or not all(flags.values()):
        raise RuntimeError("validator rejected integrated trajectory: " + str(flags))

    result.update({
        "pass": True,
        "counts": counts.copy(),
        "route_lanelet_count": len(observed_lanelets),
        "final_points": len(trajectory.points),
        "max_velocity_mps": max(point.longitudinal_velocity_mps for point in trajectory.points),
        "validation_flags": flags,
        "duration_seconds": time.monotonic() - started,
    })
finally:
    result["counts"] = counts.copy()
    result["process_deaths_before_shutdown"] = deaths[:]
    result_path.write_text(json.dumps(result, indent=2) + "\n")
    parent.shutdown()

print(json.dumps(result, indent=2))
sys.exit(0 if result["pass"] else 1)
