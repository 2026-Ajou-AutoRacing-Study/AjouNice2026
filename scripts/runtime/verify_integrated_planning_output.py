#!/usr/bin/env python3
"""Drive the installed AjouNice planning launch with conservative synthetic inputs."""

import csv
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
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_planning_msgs.msg import LaneletRoute, Path as PlanningPath, RouteState, Trajectory
from autoware_planning_validator.msg import PlanningValidatorStatus
from competition_route_requester.srv import RequestRoute
from geometry_msgs.msg import AccelWithCovarianceStamped, Point, Pose, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf.transformations import quaternion_from_euler


repo = Path(os.environ["AJOUNICE_ROOT"]).resolve()
result_path = Path(os.environ["AW149_OUTPUT_PROBE_RESULT"]).resolve()
route_path = repo / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v72_competition_route.yaml"
route_config = yaml.safe_load(route_path.read_text())
selected = route_config["route_a"]
use_requester = os.environ.get("AW149_OUTPUT_PROBE_USE_REQUESTER", "0") == "1"
route_case = os.environ.get("AW149_OUTPUT_PROBE_ROUTE_CASE", "route_a_only")
transition_speed_mps = float(os.environ.get("AW149_OUTPUT_PROBE_TRANSITION_SPEED_MPS", "2.0"))
input_period_s = float(os.environ.get("AW149_OUTPUT_PROBE_INPUT_PERIOD_S", "0.1"))
force_fallback = os.environ.get("AW149_OUTPUT_PROBE_FORCE_FALLBACK", "0") == "1"
force_b_rejection = os.environ.get("AW149_OUTPUT_PROBE_FORCE_B_REJECTION", "0") == "1"
limited_perception = os.environ.get("AW149_OUTPUT_PROBE_LIMITED_PERCEPTION", "0") == "1"
refresh_route_a = os.environ.get("AW149_OUTPUT_PROBE_REFRESH_ROUTE_A", "0") == "1"
full_lap = os.environ.get("AW149_OUTPUT_PROBE_FULL_LAP", "0") == "1"
full_lap_step_m = float(os.environ.get("AW149_OUTPUT_PROBE_FULL_LAP_STEP_M", "5.0"))
b_rejection_speed_mps = float(os.environ.get("AW149_OUTPUT_PROBE_B_REJECTION_SPEED_MPS", "1000.0"))
if not math.isfinite(input_period_s) or input_period_s <= 0.0 or input_period_s > 0.25:
    raise ValueError("AW149_OUTPUT_PROBE_INPUT_PERIOD_S must be in (0, 0.25]")
if route_case not in (
        "route_a_only", "route_a_to_b", "route_a_to_fallback", "route_a_to_b_with_fallback"):
    raise ValueError(
        "AW149_OUTPUT_PROBE_ROUTE_CASE must be route_a_only, route_a_to_b, "
        "route_a_to_fallback, or route_a_to_b_with_fallback")
if route_case != "route_a_only" and not use_requester:
    raise ValueError("route transition probes require AW149_OUTPUT_PROBE_USE_REQUESTER=1")
if refresh_route_a and (not use_requester or route_case != "route_a_only"):
    raise ValueError("AW149_OUTPUT_PROBE_REFRESH_ROUTE_A requires requester route_a_only")
if full_lap and route_case != "route_a_only":
    raise ValueError("AW149_OUTPUT_PROBE_FULL_LAP requires route_a_only")
if not math.isfinite(full_lap_step_m) or full_lap_step_m < 0.25:
    raise ValueError("AW149_OUTPUT_PROBE_FULL_LAP_STEP_M must be at least 0.25")
if force_fallback and route_case != "route_a_to_b_with_fallback":
    raise ValueError("AW149_OUTPUT_PROBE_FORCE_FALLBACK requires route_a_to_b_with_fallback")
if force_b_rejection and route_case != "route_a_to_b_with_fallback":
    raise ValueError("AW149_OUTPUT_PROBE_FORCE_B_REJECTION requires route_a_to_b_with_fallback")
if force_fallback and force_b_rejection:
    raise ValueError("forced fallback and forced B rejection are mutually exclusive")
if not math.isfinite(b_rejection_speed_mps) or b_rejection_speed_mps <= 0.0:
    raise ValueError("AW149_OUTPUT_PROBE_B_REJECTION_SPEED_MPS must be positive and finite")
transition_to_b = (route_case in ("route_a_to_b", "route_a_to_b_with_fallback") and
                   not force_fallback and not force_b_rejection)
transition_to_fallback = route_case == "route_a_to_fallback" or force_fallback or force_b_rejection


def make_pose(value):
    pose = Pose()
    pose.position.x = float(value["x"])
    pose.position.y = float(value["y"])
    pose.position.z = float(value.get("z", 0.0))
    q = quaternion_from_euler(0.0, 0.0, math.radians(float(value["yaw_deg"])))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def message_stamp(value):
    return value.to_sec() if hasattr(value, "to_sec") else value.sec + value.nanosec * 1e-9


def positions_in_box(message, x_min, x_max, y_min, y_max):
    return [
        point.pose.position
        for point in message.points
        if x_min <= point.pose.position.x <= x_max
        and y_min <= point.pose.position.y <= y_max
    ]


def maximum_nearest_distance(source, target):
    if not source or not target:
        return None
    return max(
        min(math.hypot(point.x - other.x, point.y - other.y) for other in target)
        for point in source
    )


def primitive_signature(route):
    return [sorted(primitive.id for primitive in segment.primitives) for segment in route.segments]


def strict_common_span(original, target, target_first=False):
    original_signatures = primitive_signature(original)
    target_signatures = primitive_signature(target)
    index_pairs = (
        ((original_index, target_index)
         for target_index in range(len(target_signatures))
         for original_index in range(len(original_signatures)))
        if target_first else
        ((original_index, target_index)
         for original_index in range(len(original_signatures))
         for target_index in range(len(target_signatures)))
    )
    for original_index, target_index in index_pairs:
        original_value = original_signatures[original_index]
        target_value = target_signatures[target_index]
        if original_value != target_value:
            continue
        length = 1
        while (original_index + length < len(original_signatures) and
               target_index + length < len(target_signatures) and
               original_signatures[original_index + length] == target_signatures[target_index + length]):
            length += 1
        return {
            "original_start_index": original_index,
            "target_start_index": target_index,
            "contiguous_segment_count": length,
            "first_signature": original_value,
            "last_signature": original_signatures[original_index + length - 1],
        }
    return None


def preferred_signature_mismatches(original, target):
    target_by_preferred = {}
    for segment in target.segments:
        target_by_preferred.setdefault(segment.preferred_primitive.id, []).append(
            sorted(primitive.id for primitive in segment.primitives)
        )
    mismatches = []
    for segment in original.segments:
        preferred = segment.preferred_primitive.id
        candidates = target_by_preferred.get(preferred, [])
        original_signature = sorted(primitive.id for primitive in segment.primitives)
        if candidates and original_signature not in candidates:
            mismatches.append({
                "preferred": preferred,
                "route_a_primitives": original_signature,
                "route_b_primitive_candidates": candidates,
            })
    return mismatches


packages = rospkg.RosPack()
launch_file = Path(packages.get_path("ajounice_planning_integration")) / "launch/morai_planning_stack.launch"
requester_launch = Path(packages.get_path("competition_route_requester")) / "launch/competition_route_requester.launch"
# Production waits for an explicit RViz request. Requester probes use a private
# config with automatic initial dispatch so they remain non-interactive.
launch_requester_in_stack = False
launch_specs = [(str(launch_file), [
    "start_localization:=false",
    "start_sensor_tf:=false",
    "start_object_pipeline:=false",
    "start_detector:=false",
    "start_traffic_light:=false",
    "start_requester:=" + ("true" if launch_requester_in_stack else "false"),
    "start_controller:=false",
    "use_ground_pointcloud:=" + ("false" if limited_perception else "true"),
    "use_occupancy_grid:=" + ("false" if limited_perception else "true"),
])]
if use_requester and not launch_requester_in_stack:
    requester_config = yaml.safe_load(
        (Path(packages.get_path("competition_route_requester")) / "config/competition_route_requester.param.yaml").read_text()
    )
    requester_config["route_operation_case"] = route_case
    # Production uses an explicit RViz command. The automated output-only
    # probe deliberately enables the initial request in its private config.
    requester_config["auto_request_initial_route"] = True
    requester_config["route_definition_file"] = str(route_path)
    requester_config_path = result_path.parent / "competition_route_requester.param.yaml"
    requester_config_path.write_text(yaml.safe_dump(requester_config, sort_keys=False))
    launch_specs.append((str(requester_launch), [
        "repo_root:=" + str(repo),
        "config_file:=" + str(requester_config_path),
    ]))
launch_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(launch_id)
parent = roslaunch.parent.ROSLaunchParent(launch_id, launch_specs)
deaths = []


class Monitor(roslaunch.pmon.ProcessListener):
    def process_died(self, name, exit_code):
        deaths.append({"name": name, "exit_code": exit_code})


parent.process_listeners = [Monitor()]
rospy.init_node("aw149_integrated_output_probe", anonymous=True)
latest = {}
counts = {
    "map": 0,
    "route": 0,
    "route_state": 0,
    "behavior_path": 0,
    "behavior_velocity": 0,
    "path_smoothed": 0,
    "path_optimized": 0,
    "motion_velocity": 0,
    "scenario_selected": 0,
    "velocity_smoothed": 0,
    "final": 0,
    "validation": 0,
    "request_status": 0,
}
request_status_history = []


def receive(message, key):
    latest[key] = message
    counts[key] += 1
    if key == "request_status":
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        compact = {name: status.get(name) for name in (
            "phase", "route_key", "detail", "verified_active", "uncertain", "generation")}
        if not request_status_history or compact != request_status_history[-1]:
            request_status_history.append(compact)


def requester_verified(route_key="route_a"):
    if "request_status" not in latest:
        return False
    try:
        status = json.loads(latest["request_status"].data)
    except (TypeError, ValueError):
        return False
    return status.get("phase") == "VERIFIED" and status.get("verified_active") == route_key


subscriptions = [
    rospy.Subscriber("/map/vector_map", LaneletMapBin, receive, callback_args="map", queue_size=2),
    rospy.Subscriber("/planning/mission_planning/route", LaneletRoute, receive, callback_args="route", queue_size=2),
    rospy.Subscriber("/planning/route_state", RouteState, receive, callback_args="route_state", queue_size=2),
    rospy.Subscriber("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
                     PathWithLaneId, receive, callback_args="behavior_path", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/lane_driving/behavior_planning/path",
                     PlanningPath, receive, callback_args="behavior_velocity", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path",
                     PlanningPath, receive, callback_args="path_smoothed", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
                     Trajectory, receive, callback_args="path_optimized", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/lane_driving/trajectory",
                     Trajectory, receive, callback_args="motion_velocity", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/scenario_selector/trajectory",
                     Trajectory, receive, callback_args="scenario_selected", queue_size=10),
    rospy.Subscriber("/planning/scenario_planning/velocity_smoother/trajectory",
                     Trajectory, receive, callback_args="velocity_smoothed", queue_size=10),
    rospy.Subscriber("/planning/trajectory", Trajectory, receive, callback_args="final", queue_size=10),
    rospy.Subscriber("/planning/validation_status", PlanningValidatorStatus, receive, callback_args="validation", queue_size=10),
    rospy.Subscriber("/competition_route_requester/request_status", String, receive, callback_args="request_status", queue_size=10),
]


def publisher(topic, message_type):
    return rospy.Publisher(topic, message_type, queue_size=1, latch=True)


odom_pub = publisher("/localization/kinematic_state", Odometry)
accel_pub = publisher("/localization/acceleration", AccelWithCovarianceStamped)
objects_pub = publisher("/perception/object_recognition/objects", PredictedObjects)
signals_pub = publisher("/perception/traffic_light_recognition/traffic_signals", TrafficLightGroupArray)
grid_pub = None if limited_perception else publisher(
    "/perception/occupancy_grid_map/map", OccupancyGrid)
cloud_pub = None if limited_perception else publisher(
    "/perception/obstacle_segmentation/pointcloud", PointCloud2)
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
    if not limited_perception:
        grid.header.stamp = now
        grid_pub.publish(grid)
        far_point = [(odom.pose.pose.position.x + 100.0, odom.pose.pose.position.y + 100.0, 1.0)]
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(frame_id="map", stamp=now), far_point))
    mode = OperationModeState(
        mode=OperationModeState.AUTONOMOUS,
        is_autoware_control_enabled=True,
        is_autonomous_mode_available=True,
    )
    mode.stamp.sec, mode.stamp.nanosec = now.secs, now.nsecs
    mode_pub.publish(mode)
    transform = TransformStamped(header=Header(frame_id="map", stamp=now), child_frame_id="base_link")
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation
    broadcaster.sendTransform(transform)
    parent.spin_once()
    if deaths:
        raise RuntimeError("integrated launch process died: " + str(deaths))
    time.sleep(input_period_s)


def run_full_lap():
    course_path = (
        repo
        / "base_ws/src/control/trajectory_generator/traj_sample/competition_route/"
        "200889_200889_v.csv"
    )
    with course_path.open(newline="") as stream:
        course = list(csv.DictReader(stream))
    if len(course) < 2:
        raise RuntimeError("competition course is empty: " + str(course_path))

    goal = route_config["poses"][selected["goal_ref"]]
    goal_index = min(
        range(len(course)),
        key=lambda index: (
            (float(course[index]["x"]) - float(goal["x"])) ** 2
            + (float(course[index]["y"]) - float(goal["y"])) ** 2
        ),
    )
    goal_error = math.hypot(
        float(course[goal_index]["x"]) - float(goal["x"]),
        float(course[goal_index]["y"]) - float(goal["y"]),
    )
    if goal_error > 0.25:
        raise RuntimeError("competition course misses Route A goal by {:.3f} m".format(goal_error))

    spacing = math.hypot(
        float(course[1]["x"]) - float(course[0]["x"]),
        float(course[1]["y"]) - float(course[0]["y"]),
    )
    if spacing <= 0.0:
        raise RuntimeError("competition course has invalid point spacing")
    stride = max(1, int(round(full_lap_step_m / spacing)))
    sample_indices = list(range(0, goal_index + 1, stride))
    if sample_indices[-1] != goal_index:
        sample_indices.append(goal_index)

    expected_sections = {
        lane_id: index for index, lane_id in enumerate(selected["expected_path_lanelet_ids"])
    }
    maximum_route_section = -1
    route_section_regressions = []
    worst_course_error = {"value_m": 0.0, "sample_index": 0, "course_row": 0}
    worst_optimizer_shift = {"value_m": 0.0, "sample_index": 0, "course_row": 0}
    maximum_first_point_error = 0.0
    samples = 0

    for sample_index, course_index in enumerate(sample_indices):
        row = course[course_index]
        pose = Pose()
        pose.position.x = float(row["x"])
        pose.position.y = float(row["y"])
        pose.position.z = float(row["z"])
        quaternion = quaternion_from_euler(0.0, 0.0, float(row["yaw"]))
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        odom.pose.pose = pose
        odom.twist.twist.linear.x = 0.0 if course_index == goal_index else 5.0

        previous = {name: counts[name] for name in (
            "behavior_path", "path_smoothed", "path_optimized", "final", "validation")}
        requested_stamp = rospy.Time.now().to_sec()
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            tick()
            fresh = all(counts[name] > previous[name] for name in previous)
            if not fresh:
                continue
            if latest["final"].header.stamp.to_sec() < requested_stamp:
                continue
            if message_stamp(latest["validation"].stamp) < requested_stamp:
                continue
            break
        else:
            raise RuntimeError(
                "full-lap sample {} at course row {} timed out: {}".format(
                    sample_index, course_index, counts
                )
            )

        flags = {
            name: getattr(latest["validation"], name)
            for name in latest["validation"].__slots__
            if name.startswith("is_valid_")
        }
        remaining_distance = (goal_index - course_index) * spacing
        required_flags = dict(flags)
        if remaining_distance <= 25.0:
            # A route-ending trajectory is expected to be shorter than the
            # normal forward-horizon requirement.
            required_flags.pop("is_valid_forward_trajectory_length", None)
        if not required_flags or not all(required_flags.values()):
            raise RuntimeError(
                "full-lap validator rejected sample {}: {}".format(sample_index, flags)
            )

        local_start = max(0, course_index - int(round(10.0 / spacing)))
        local_end = min(goal_index + 1, course_index + int(round(220.0 / spacing)))
        local_course = [
            Point(x=float(course[index]["x"]), y=float(course[index]["y"]))
            for index in range(local_start, local_end, max(1, int(round(0.5 / spacing))))
        ]
        final_positions = [point.pose.position for point in latest["final"].points]
        course_error = maximum_nearest_distance(final_positions, local_course)
        if course_error is None or course_error > 3.0:
            raise RuntimeError(
                "full-lap trajectory is {:.3f} m from course at sample {}".format(
                    float("nan") if course_error is None else course_error, sample_index
                )
            )
        if course_error > worst_course_error["value_m"]:
            worst_course_error = {
                "value_m": course_error,
                "sample_index": sample_index,
                "course_row": course_index,
            }

        first_error = math.hypot(
            final_positions[0].x - pose.position.x,
            final_positions[0].y - pose.position.y,
        )
        maximum_first_point_error = max(maximum_first_point_error, first_error)
        if first_error > 10.0:
            raise RuntimeError(
                "full-lap trajectory starts {:.3f} m from ego at sample {}".format(
                    first_error, sample_index
                )
            )

        smoothed_positions = [point.pose.position for point in latest["path_smoothed"].points]
        optimized_positions = [point.pose.position for point in latest["path_optimized"].points]
        optimizer_shift = maximum_nearest_distance(optimized_positions, smoothed_positions)
        if optimizer_shift is None or optimizer_shift > 3.0:
            raise RuntimeError(
                "full-lap PO shift is {:.3f} m at sample {}".format(
                    float("nan") if optimizer_shift is None else optimizer_shift, sample_index
                )
            )
        if optimizer_shift > worst_optimizer_shift["value_m"]:
            worst_optimizer_shift = {
                "value_m": optimizer_shift,
                "sample_index": sample_index,
                "course_row": course_index,
            }

        lane_ids = set()
        for point in latest["behavior_path"].points[:5]:
            lane_ids.update(int(lane_id) for lane_id in point.lane_ids)
        sections = [expected_sections[lane_id] for lane_id in lane_ids if lane_id in expected_sections]
        if sections:
            current_section = max(sections)
            if current_section + 1 < maximum_route_section:
                route_section_regressions.append({
                    "sample_index": sample_index,
                    "course_row": course_index,
                    "previous_section": maximum_route_section,
                    "current_section": current_section,
                })
            maximum_route_section = max(maximum_route_section, current_section)
        samples += 1
        progress_interval = max(1, len(sample_indices) // 10)
        if sample_index % progress_interval == 0 or sample_index + 1 == len(sample_indices):
            rospy.loginfo(
                "AW149 full-lap %.1f%% | %.1f/%.1f m | course_error=%.3f m "
                "PO_shift=%.3f m | route_section=%d",
                100.0 * course_index / goal_index,
                course_index * spacing,
                goal_index * spacing,
                course_error,
                optimizer_shift,
                maximum_route_section,
            )

    if route_section_regressions:
        raise RuntimeError("BPP route section regressed: " + str(route_section_regressions[:3]))
    if maximum_route_section < len(expected_sections) - 2:
        raise RuntimeError(
            "BPP did not reach final route sections: {} / {}".format(
                maximum_route_section, len(expected_sections) - 1
            )
        )
    return {
        "course_file": str(course_path),
        "course_length_to_goal_m": goal_index * spacing,
        "route_goal_course_error_m": goal_error,
        "step_m": stride * spacing,
        "validated_samples": samples,
        "completion_ratio": 1.0,
        "maximum_final_to_course_error": worst_course_error,
        "maximum_po_to_smoother_shift": worst_optimizer_shift,
        "maximum_final_first_point_to_ego_m": maximum_first_point_error,
        "maximum_bpp_route_section": maximum_route_section,
        "expected_final_route_section": len(expected_sections) - 1,
        "route_section_regressions": route_section_regressions,
    }


result = {
    "scope": "AJOUNICE_INSTALLED_LAUNCH_SYNTHETIC_INPUT_OUTPUT_ONLY",
    "pass": False,
    "launch": str(launch_file),
    "route_fixture": str(route_path),
    "counts": counts,
    "route_request_mode": "competition_route_requester" if use_requester else "direct_service_probe",
    "route_operation_case": route_case,
    "forced_fallback_branch": force_fallback,
    "forced_b_rejection_branch": force_b_rejection,
    "synthetic_input_period_seconds": input_period_s,
    "limited_perception": limited_perception,
    "refresh_route_a": refresh_route_a,
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
        mission_unset = "route_state" in latest and latest["route_state"].state == RouteState.UNSET
        requester_started = use_requester and any(
            status.get("route_key") == "route_a" and status.get("phase") in
            ("DISPATCHED", "VERIFIED", "RECONCILIATION_REQUIRED")
            for status in request_status_history
        )
        if mission_unset or requester_started:
            break
    if not mission_unset and not requester_started:
        raise RuntimeError("mission planner did not reach UNSET readiness")

    if not use_requester:
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
        outputs_ready = counts["final"] >= 10 and counts["validation"] >= 10
        if outputs_ready and (not use_requester or requester_verified("route_a")):
            break
    if counts["final"] < 10 or counts["validation"] < 10:
        raise RuntimeError("insufficient integrated outputs: " + str(counts))

    if use_requester:
        if "request_status" not in latest:
            raise RuntimeError("competition_route_requester published no status")
        requester_status = json.loads(latest["request_status"].data)
        if not requester_verified("route_a"):
            raise RuntimeError("competition_route_requester did not verify Route A: " + str(requester_status))
        result["requester_status"] = requester_status

        if refresh_route_a:
            previous_generation = requester_status.get("generation", 0)
            previous_route_stamp = latest["route"].header.stamp.to_sec()
            previous_route_count = counts["route"]
            previous_final_count = counts["final"]
            odom.pose.pose = make_pose(route_config["start"]["map_pose"])
            odom.twist.twist.linear.x = 0.0
            for _ in range(5):
                tick()
            requested_at = rospy.Time.now().to_sec()
            response = rospy.ServiceProxy(
                "/competition_route_requester/request", RequestRoute)("route_a")
            if not response.accepted:
                raise RuntimeError("Route A refresh request rejected: " + response.detail)
            started_refresh = time.monotonic()
            deadline = started_refresh + 120.0
            refreshed_status = None
            while time.monotonic() < deadline:
                tick()
                status = json.loads(latest["request_status"].data)
                if (status.get("phase") == "VERIFIED" and
                        status.get("verified_active") == "route_a" and
                        status.get("generation", 0) > previous_generation):
                    refreshed_status = status
                    break
            if refreshed_status is None:
                raise RuntimeError("competition_route_requester did not verify refreshed Route A")
            refreshed_route_stamp = latest["route"].header.stamp.to_sec()
            if (counts["route"] <= previous_route_count or
                    refreshed_route_stamp <= previous_route_stamp or
                    refreshed_route_stamp < requested_at):
                raise RuntimeError("Route A refresh did not publish a newly stamped route")
            if counts["final"] <= previous_final_count:
                raise RuntimeError("Route A refresh did not produce a new final trajectory")
            result["route_a_refresh"] = {
                "accepted": True,
                "previous_generation": previous_generation,
                "verified_generation": refreshed_status["generation"],
                "previous_route_stamp": previous_route_stamp,
                "refreshed_route_stamp": refreshed_route_stamp,
                "route_messages_after_request": counts["route"] - previous_route_count,
                "final_messages_after_request": counts["final"] - previous_final_count,
                "verification_seconds": time.monotonic() - started_refresh,
            }
            result["requester_status"] = refreshed_status

        if force_b_rejection:
            odom.pose.pose = make_pose(route_config["poses"]["wp_200178"])
            odom.twist.twist.linear.x = b_rejection_speed_mps
            generation_before = requester_status.get("generation", 0)
            started_rejection = time.monotonic()
            deadline = started_rejection + 30.0
            rejected = None
            while time.monotonic() < deadline:
                tick()
                rejected = next((status for status in reversed(request_status_history)
                    if status.get("phase") == "REJECTED" and status.get("route_key") == "route_b" and
                    status.get("generation", 0) > generation_before), None)
                if rejected is not None:
                    break
            if rejected is None:
                raise RuntimeError("Route B did not produce an explicit safety rejection")
            if rejected.get("uncertain") or rejected.get("verified_active") != "route_a":
                raise RuntimeError("explicit Route B rejection corrupted requester state: " + str(rejected))
            result["route_b_explicit_rejection"] = rejected
            result["route_b_rejection_seconds"] = time.monotonic() - started_rejection
            result["route_b_rejection_speed_mps"] = b_rejection_speed_mps

        if transition_to_b:
            route_a_counts = counts.copy()
            route_a_message = latest["route"]
            odom.pose.pose = make_pose(route_config["poses"]["wp_200178"])
            odom.twist.twist.linear.x = transition_speed_mps
            started_b = time.monotonic()
            deadline = started_b + 120.0
            while time.monotonic() < deadline:
                tick()
                if requester_verified("route_b"):
                    break
            if not requester_verified("route_b"):
                status = json.loads(latest["request_status"].data)
                raise RuntimeError("competition_route_requester did not verify Route B: " + str(status))
            result["route_a_counts_at_transition"] = route_a_counts
            result["route_b_verification_seconds"] = time.monotonic() - started_b
            result["route_b_transition_speed_mps"] = transition_speed_mps
            result["requester_status"] = json.loads(latest["request_status"].data)
            result["route_a_to_b_original_first_common_span"] = strict_common_span(
                route_a_message, latest["route"])
            result["route_a_to_b_target_first_common_span"] = strict_common_span(
                route_a_message, latest["route"], target_first=True)
            mismatches = preferred_signature_mismatches(route_a_message, latest["route"])
            result["shared_preferred_signature_mismatch_count"] = len(mismatches)
            result["shared_preferred_signature_mismatch_sample"] = mismatches[:10]
            selected = route_config["route_b"]

        if transition_to_fallback:
            route_a_counts = counts.copy()
            route_a_message = latest["route"]
            cutoff_ref = route_config["fallback_route"]["hard_cutoff_pose_ref"]
            odom.pose.pose = make_pose(route_config["poses"][cutoff_ref])
            odom.twist.twist.linear.x = transition_speed_mps
            started_fallback = time.monotonic()
            deadline = started_fallback + 120.0
            while time.monotonic() < deadline:
                tick()
                if requester_verified("fallback_route"):
                    break
            if not requester_verified("fallback_route"):
                status = json.loads(latest["request_status"].data)
                raise RuntimeError(
                    "competition_route_requester did not verify fallback route: " + str(status))
            result["route_a_counts_at_transition"] = route_a_counts
            result["fallback_verification_seconds"] = time.monotonic() - started_fallback
            result["fallback_transition_speed_mps"] = transition_speed_mps
            result["requester_status"] = json.loads(latest["request_status"].data)
            result["route_a_to_fallback_original_first_common_span"] = strict_common_span(
                route_a_message, latest["route"])
            result["route_a_to_fallback_target_first_common_span"] = strict_common_span(
                route_a_message, latest["route"], target_first=True)
            mismatches = preferred_signature_mismatches(route_a_message, latest["route"])
            result["shared_preferred_signature_mismatch_count"] = len(mismatches)
            result["shared_preferred_signature_mismatch_sample"] = mismatches[:10]
            selected = route_config["fallback_route"]

    if full_lap:
        result["full_lap"] = run_full_lap()

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
        "request_status_history": request_status_history,
    })
    if "path_smoothed" in latest and "path_optimized" in latest:
        # lanelet 200392 left-turn window used by the boundary-fix regression.
        corner = (25.0, 42.0, 180.0, 200.0)
        smoothed = positions_in_box(latest["path_smoothed"], *corner)
        optimized = positions_in_box(latest["path_optimized"], *corner)
        result["lanelet_200392_corner_geometry"] = {
            "window_xy": {"x": list(corner[:2]), "y": list(corner[2:])},
            "smoothed_points": len(smoothed),
            "optimized_points": len(optimized),
            "optimized_to_smoothed_max_nearest_m": maximum_nearest_distance(
                optimized, latest["path_smoothed"].points and [
                    point.pose.position for point in latest["path_smoothed"].points
                ]),
            "smoothed_to_optimized_max_nearest_m": maximum_nearest_distance(
                smoothed, latest["path_optimized"].points and [
                    point.pose.position for point in latest["path_optimized"].points
                ]),
            "message_stamp_delta_seconds": abs(
                latest["path_smoothed"].header.stamp.to_sec()
                - latest["path_optimized"].header.stamp.to_sec()
            ),
        }
        result["path_optimizer_lat_error_weight"] = rospy.get_param(
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/"
            "mpt/weight/lat_error_weight",
            None,
        )
finally:
    result["counts"] = counts.copy()
    result["process_deaths_before_shutdown"] = deaths[:]
    result["request_status_history"] = request_status_history
    if "final" in latest:
        result["latest_final_stamp"] = latest["final"].header.stamp.to_sec()
        result["latest_final_age_seconds"] = rospy.Time.now().to_sec() - latest["final"].header.stamp.to_sec()
    if "validation" in latest:
        result["latest_validation_stamp"] = message_stamp(latest["validation"].stamp)
        result["latest_validation_flags"] = {
            name: getattr(latest["validation"], name)
            for name in latest["validation"].__slots__
            if name.startswith("is_valid_")
        }
    result_path.write_text(json.dumps(result, indent=2) + "\n")
    parent.shutdown()

print(json.dumps(result, indent=2))
sys.exit(0 if result["pass"] else 1)
