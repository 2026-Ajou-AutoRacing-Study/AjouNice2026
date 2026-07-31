#!/usr/bin/env python3
"""Drive the confirmed K-City course in Autoware planning simulator.

The route API cannot express a closed loop with identical start and goal as one
useful request, so this script runs the confirmed course as two continuous legs:
the open loop from the start to the last lanelet, then the final return lanelet.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ElementTree
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import RouteState as AdapiRouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray
from autoware_planning_msgs.msg import Trajectory
from autoware_system_msgs.msg import AutowareState
from tier4_external_api_msgs.srv import Engage

from autoware_route_probe import RouteProbe, load_lanelets, pose_at_fraction, route_from_path


Point3 = Tuple[float, float, float]


def read_preferred_course(path: Path) -> List[int]:
    with path.open(newline="", encoding="utf-8") as fp:
        return [int(row["lanelet_id"]) for row in csv.DictReader(fp) if row["role"] == "preferred_course"]


def lanelet_speed_limits(map_path: Path) -> Dict[int, float]:
    from lxml import etree

    root = etree.parse(str(map_path)).getroot()
    speeds: Dict[int, float] = {}
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag") if tag.get("k")}
        if tags.get("type") == "lanelet" and tags.get("subtype") == "road" and "speed_limit" in tags:
            speeds[int(relation.get("id"))] = float(tags["speed_limit"]) / 3.6
    return speeds


def traffic_light_regulatory_ids(map_path: Path) -> List[int]:
    root = ElementTree.parse(map_path).getroot()
    ids = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") == "regulatory_element" and tags.get("subtype") == "traffic_light":
            ids.append(int(relation.get("id")))
    return sorted(ids)


def dist2_point_to_segment(point: Point3, a: Point3, b: Point3) -> float:
    px, py, _ = point
    ax, ay, _ = a
    bx, by, _ = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return (px - ax) ** 2 + (py - ay) ** 2
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    cx = ax + ratio * dx
    cy = ay + ratio * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def nearest_lanelet(
    point: Point3,
    route_ids: Iterable[int],
    lanelet_points: Dict[int, List[Point3]],
) -> Tuple[Optional[int], float]:
    best_id: Optional[int] = None
    best_dist2 = float("inf")
    for lanelet_id in route_ids:
        points = lanelet_points.get(lanelet_id)
        if not points:
            continue
        if len(points) == 1:
            dist2 = dist2_point_to_segment(point, points[0], points[0])
            if dist2 < best_dist2:
                best_id, best_dist2 = lanelet_id, dist2
            continue
        for a, b in zip(points, points[1:]):
            dist2 = dist2_point_to_segment(point, a, b)
            if dist2 < best_dist2:
                best_id, best_dist2 = lanelet_id, dist2
    return best_id, math.sqrt(best_dist2)


def speed_stats(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, max(0, round(0.95 * (len(ordered) - 1))))
    return {
        "max_mps": max(values),
        "max_kph": max(values) * 3.6,
        "mean_mps": sum(values) / len(values),
        "mean_kph": sum(values) / len(values) * 3.6,
        "p95_mps": ordered[p95_index],
        "p95_kph": ordered[p95_index] * 3.6,
        "samples": len(values),
    }


class CourseDriveProbe(RouteProbe):
    def __init__(self, green_traffic_light_ids: Sequence[int] = ()) -> None:
        super().__init__()
        self.autonomous_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        self.enable_control_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/enable_autoware_control"
        )
        self.stop_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_stop"
        )
        self.legacy_engage_client = self.create_client(Engage, "/api/autoware/set/engage")
        self.operation_mode: Optional[OperationModeState] = None
        self.route_state: Optional[AdapiRouteState] = None
        self.autoware_state: Optional[AutowareState] = None
        self.latest_trajectory: Optional[Trajectory] = None
        self.trajectory_maxima: List[float] = []
        self.green_traffic_light_ids = tuple(sorted(set(green_traffic_light_ids)))
        self.traffic_signal_pub = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            10,
        )
        self.traffic_signal_timer = self.create_timer(0.1, self._publish_green_traffic_lights)
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._operation_mode_cb,
            10,
        )
        self.create_subscription(AdapiRouteState, "/api/routing/state", self._route_state_cb, 10)
        self.create_subscription(AutowareState, "/autoware/state", self._autoware_state_cb, 10)

    def _publish_green_traffic_lights(self) -> None:
        if not self.green_traffic_light_ids:
            return
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        for regulatory_id in self.green_traffic_light_ids:
            group = TrafficLightGroup()
            group.traffic_light_group_id = regulatory_id
            element = TrafficLightElement()
            element.color = TrafficLightElement.GREEN
            element.shape = TrafficLightElement.CIRCLE
            element.status = TrafficLightElement.SOLID_ON
            element.confidence = 1.0
            group.elements.append(element)
            msg.traffic_light_groups.append(group)
        self.traffic_signal_pub.publish(msg)

    def _operation_mode_cb(self, msg: OperationModeState) -> None:
        self.operation_mode = msg

    def _route_state_cb(self, msg: AdapiRouteState) -> None:
        self.route_state = msg

    def _autoware_state_cb(self, msg: AutowareState) -> None:
        self.autoware_state = msg

    def _trajectory_cb(self, msg: Trajectory) -> None:
        super()._trajectory_cb(msg)
        self.latest_trajectory = msg
        if msg.points:
            self.trajectory_maxima.append(max(point.longitudinal_velocity_mps for point in msg.points))

    def change_to_autonomous(self, timeout_sec: float) -> Dict[str, object]:
        if not self.autonomous_client.wait_for_service(timeout_sec=timeout_sec):
            return {"success": False, "code": None, "message": "service_not_available"}
        try:
            response = self.call_service(self.autonomous_client, ChangeOperationMode.Request(), timeout_sec)
            return {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}

    def enable_autoware_control(self, timeout_sec: float) -> Dict[str, object]:
        if not self.enable_control_client.wait_for_service(timeout_sec=timeout_sec):
            return {"success": False, "code": None, "message": "service_not_available"}
        try:
            response = self.call_service(self.enable_control_client, ChangeOperationMode.Request(), timeout_sec)
            return {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}

    def change_to_stop(self, timeout_sec: float) -> Dict[str, object]:
        if not self.stop_client.wait_for_service(timeout_sec=timeout_sec):
            return {"success": False, "code": None, "message": "service_not_available"}
        try:
            response = self.call_service(self.stop_client, ChangeOperationMode.Request(), timeout_sec)
            return {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}

    def legacy_engage(self, timeout_sec: float) -> Dict[str, object]:
        if not self.legacy_engage_client.wait_for_service(timeout_sec=timeout_sec):
            return {"success": False, "code": None, "message": "service_not_available"}
        request = Engage.Request()
        request.engage = True
        try:
            response = self.call_service(self.legacy_engage_client, request, timeout_sec)
            return {
                "success": response.status.code in (response.status.SUCCESS, response.status.IGNORED),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}

    def engage_autonomous(self, timeout_sec: float) -> Dict[str, object]:
        availability_wait = self.spin_until(
            lambda: self.operation_mode is not None and self.operation_mode.is_autonomous_mode_available,
            timeout_sec,
        )
        enable_status = None
        if self.operation_mode is not None and not self.operation_mode.is_autoware_control_enabled:
            enable_status = self.enable_autoware_control(2.0)
        autonomous_status = self.change_to_autonomous(2.0)
        legacy_status = None
        if not autonomous_status.get("success"):
            legacy_status = self.legacy_engage(2.0)
            self.spin_until(
                lambda: self.operation_mode is not None
                and self.operation_mode.mode == OperationModeState.AUTONOMOUS,
                3.0,
            )
        return {
            "availability_wait_success": availability_wait,
            "enable_autoware_control": enable_status,
            "change_to_autonomous": autonomous_status,
            "legacy_engage": legacy_status,
        }

    def is_arrived(self) -> bool:
        return (
            self.route_state is not None
            and self.route_state.state == AdapiRouteState.ARRIVED
        ) or (
            self.autoware_state is not None
            and self.autoware_state.state == AutowareState.ARRIVED_GOAL
        )


def make_sample_row(
    leg_name: str,
    elapsed: float,
    lanelet_id: Optional[int],
    lanelet_distance: float,
    speed_limit_mps: Optional[float],
    odom,
    goal_distance: Optional[float],
) -> Dict[str, object]:
    pos = odom.pose.pose.position
    twist = odom.twist.twist.linear
    speed_mps = math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)
    return {
        "leg": leg_name,
        "time_sec": round(elapsed, 3),
        "x": pos.x,
        "y": pos.y,
        "z": pos.z,
        "speed_mps": speed_mps,
        "speed_kph": speed_mps * 3.6,
        "nearest_lanelet": lanelet_id if lanelet_id is not None else "",
        "nearest_lanelet_distance_m": lanelet_distance,
        "speed_limit_mps": speed_limit_mps if speed_limit_mps is not None else "",
        "speed_limit_kph": speed_limit_mps * 3.6 if speed_limit_mps is not None else "",
        "goal_distance_m": goal_distance if goal_distance is not None else "",
    }


def write_json(path: Path, payload: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def write_samples(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "leg",
        "time_sec",
        "x",
        "y",
        "z",
        "speed_mps",
        "speed_kph",
        "nearest_lanelet",
        "nearest_lanelet_distance_m",
        "speed_limit_mps",
        "speed_limit_kph",
        "goal_distance_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_samples(rows: Sequence[Dict[str, object]]) -> Dict[str, object]:
    speeds = [float(row["speed_mps"]) for row in rows]
    by_limit: Dict[str, List[float]] = defaultdict(list)
    overspeed = 0
    for row in rows:
        if row["speed_limit_mps"] == "":
            continue
        limit = float(row["speed_limit_mps"])
        speed = float(row["speed_mps"])
        by_limit[str(round(limit * 3.6))].append(speed)
        if speed > limit + 0.5:
            overspeed += 1

    accel_values = []
    previous_by_leg: Dict[str, Dict[str, object]] = {}
    for row in rows:
        leg = str(row["leg"])
        prev = previous_by_leg.get(leg)
        if prev is not None:
            dt = float(row["time_sec"]) - float(prev["time_sec"])
            if dt > 1e-6:
                accel_values.append((float(row["speed_mps"]) - float(prev["speed_mps"])) / dt)
        previous_by_leg[leg] = row

    return {
        "overall_speed": speed_stats(speeds),
        "speed_by_limit_kph": {limit: speed_stats(values) for limit, values in sorted(by_limit.items())},
        "overspeed_samples_gt_limit_plus_0_5_mps": overspeed,
        "acceleration": {
            "max_mps2": max(accel_values) if accel_values else None,
            "min_mps2": min(accel_values) if accel_values else None,
        },
    }


def run_leg(
    node: CourseDriveProbe,
    name: str,
    path: List[int],
    lanelets,
    lanelet_points: Dict[int, List[Point3]],
    speed_limits: Dict[int, float],
    args,
    initialize: bool,
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    spec = route_from_path(name, path, lanelets, required_lanelet=path[-1], waypoint_lanelets=path[1:-1])
    result: Dict[str, object] = {
        "name": name,
        "path": path,
        "start_lanelet": path[0],
        "goal_lanelet": path[-1],
        "waypoint_count": len(path[1:-1]),
    }
    started = time.monotonic()

    clear_status = node.clear_route(args.service_timeout)
    if not bool(clear_status["success"]):
        waited_for_arrival = node.spin_until(node.is_arrived, args.clear_retry_timeout)
        retry_status = node.clear_route(args.service_timeout)
        stop_status = None
        retry_after_stop_status = None
        if not bool(retry_status["success"]):
            stop_status = node.change_to_stop(2.0)
            retry_after_stop_status = node.clear_route(args.service_timeout)
        clear_status = {
            "success": bool(
                retry_status["success"]
                or (retry_after_stop_status is not None and retry_after_stop_status["success"])
            ),
            "initial": clear_status,
            "waited_for_arrival": waited_for_arrival,
            "retry": retry_status,
            "change_to_stop": stop_status,
            "retry_after_stop": retry_after_stop_status,
        }
    init_status = None
    if initialize:
        init_status = node.initialize_pose(spec.start_pose, args.service_timeout)

    node.latest_route = None
    node.latest_trajectory_points = 0
    node.latest_trajectory = None
    node.trajectory_maxima.clear()
    route_status = node.set_route(spec, args.service_timeout)
    route_received = False
    if bool(route_status["success"]):
        route_received = node.spin_until(
            lambda: node.latest_route is not None and bool(node.latest_route.segments),
            args.route_timeout,
        )
    trajectory_received = False
    if bool(route_status["success"]) and route_received:
        trajectory_received = node.spin_until(lambda: node.latest_trajectory_points > 0, args.trajectory_timeout)

    route_ids = node.route_ids()
    if not (bool(route_status["success"]) and route_received and trajectory_received):
        result.update(
            {
                "clear_status": clear_status,
                "initialization_status": init_status,
                "route_status": route_status,
                "route_received": route_received,
                "trajectory_received": trajectory_received,
                "trajectory_points": node.latest_trajectory_points,
                "route_ids": route_ids,
                "route_id_count": len(route_ids),
                "autonomous_status": None,
                "operation_mode": {},
                "route_state": int(node.route_state.state) if node.route_state is not None else None,
                "autoware_state": int(node.autoware_state.state) if node.autoware_state is not None else None,
                "reached_goal": False,
                "stop_reason": "route_or_trajectory_not_ready",
                "duration_sec": round(time.monotonic() - started, 3),
                "final_goal_distance_m": node.odom_distance_to(spec.goal_pose),
                "sample_count": 0,
                "sample_summary": summarize_samples([]),
            }
        )
        return result, []

    autonomous_status = node.engage_autonomous(10.0)

    samples: List[Dict[str, object]] = []
    last_motion_time = started
    has_moved = False
    reached_goal = False
    stop_reason = "timeout"

    while time.monotonic() - started < args.max_leg_duration:
        rclpy.spin_once(node, timeout_sec=args.sample_period)
        if node.latest_odom is None:
            continue

        elapsed = time.monotonic() - started
        goal_distance = node.odom_distance_to(spec.goal_pose)
        odom = node.latest_odom
        pos = odom.pose.pose.position
        twist = odom.twist.twist.linear
        speed = math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)
        if speed > 0.2:
            last_motion_time = time.monotonic()
            has_moved = True
        lanelet_id, lanelet_distance = nearest_lanelet(
            (pos.x, pos.y, pos.z),
            route_ids or path,
            lanelet_points,
        )
        row = make_sample_row(
            name,
            elapsed,
            lanelet_id,
            lanelet_distance,
            speed_limits.get(lanelet_id) if lanelet_id is not None else None,
            odom,
            goal_distance,
        )
        samples.append(row)

        if node.is_arrived() or (
            has_moved
            and elapsed > args.min_drive_duration
            and goal_distance is not None
            and goal_distance < args.goal_threshold
        ):
            reached_goal = True
            stop_reason = "goal_reached"
            break
        if has_moved and elapsed > args.min_drive_duration and time.monotonic() - last_motion_time > args.stuck_timeout:
            stop_reason = "stuck_or_not_progressing"
            break

    result.update(
        {
            "clear_status": clear_status,
            "initialization_status": init_status,
            "route_status": route_status,
            "route_received": route_received,
            "trajectory_received": trajectory_received,
            "trajectory_points": node.latest_trajectory_points,
            "trajectory_max_speed_mps_observed": max(node.trajectory_maxima) if node.trajectory_maxima else None,
            "trajectory_max_speed_kph_observed": max(node.trajectory_maxima) * 3.6 if node.trajectory_maxima else None,
            "route_ids": route_ids,
            "route_id_count": len(route_ids),
            "autonomous_status": autonomous_status,
            "operation_mode": {
                "mode": int(node.operation_mode.mode) if node.operation_mode is not None else None,
                "is_autoware_control_enabled": bool(node.operation_mode.is_autoware_control_enabled)
                if node.operation_mode is not None
                else None,
                "is_autonomous_mode_available": bool(node.operation_mode.is_autonomous_mode_available)
                if node.operation_mode is not None
                else None,
            },
            "route_state": int(node.route_state.state) if node.route_state is not None else None,
            "autoware_state": int(node.autoware_state.state) if node.autoware_state is not None else None,
            "reached_goal": reached_goal,
            "stop_reason": stop_reason,
            "duration_sec": round(time.monotonic() - started, 3),
            "final_goal_distance_m": node.odom_distance_to(spec.goal_pose),
            "sample_count": len(samples),
            "sample_summary": summarize_samples(samples),
        }
    )
    return result, samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--course-csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--samples-csv", required=True, type=Path)
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    parser.add_argument("--service-timeout", type=float, default=20.0)
    parser.add_argument("--route-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=20.0)
    parser.add_argument("--max-leg-duration", type=float, default=240.0)
    parser.add_argument("--sample-period", type=float, default=0.2)
    parser.add_argument("--goal-threshold", type=float, default=5.0)
    parser.add_argument("--stuck-timeout", type=float, default=8.0)
    parser.add_argument("--min-drive-duration", type=float, default=5.0)
    parser.add_argument("--clear-retry-timeout", type=float, default=20.0)
    parser.add_argument("--publish-all-traffic-lights-green", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    course = read_preferred_course(args.course_csv)
    if len(course) < 2:
        raise RuntimeError("Expected at least two preferred course lanelets")
    if course[0] != course[-1]:
        course.append(course[0])

    _, _, lanelets, _ = load_lanelets(str(args.map), args.origin_lat, args.origin_lon)
    missing = [lanelet_id for lanelet_id in course if lanelet_id not in lanelets]
    if missing:
        raise RuntimeError(f"Course lanelets missing from map: {missing}")
    lanelet_points = {lanelet_id: info.centerline for lanelet_id, info in lanelets.items()}
    speed_limits = lanelet_speed_limits(args.map)

    payload: Dict[str, object] = {
        "map": str(args.map),
        "course_csv": str(args.course_csv),
        "course_lanelets": course,
        "legs": [],
        "started_at_unix": time.time(),
    }
    write_json(args.output, payload)

    all_samples: List[Dict[str, object]] = []
    rclpy.init()
    green_ids = traffic_light_regulatory_ids(args.map) if args.publish_all_traffic_lights_green else []
    node = CourseDriveProbe(green_ids)
    unexpected_error = None
    try:
        payload["phase"] = "waiting_for_adapi_services"
        write_json(args.output, payload)
        node.wait_for_services(args.service_timeout)
        legs = [
            ("full_open_loop_to_200892", course[:-1], True),
            ("final_return_200892_to_200889", [course[-2], course[-1]], False),
        ]
        for name, path, initialize in legs:
            payload["phase"] = f"running_{name}"
            write_json(args.output, payload)
            leg_result, samples = run_leg(
                node,
                name,
                path,
                lanelets,
                lanelet_points,
                speed_limits,
                args,
                initialize,
            )
            payload["legs"].append(leg_result)
            all_samples.extend(samples)
            payload["sample_summary"] = summarize_samples(all_samples)
            payload["completed"] = all(leg.get("reached_goal") for leg in payload["legs"])
            write_json(args.output, payload)
            write_samples(args.samples_csv, all_samples)
    except Exception as exc:
        unexpected_error = f"{type(exc).__name__}: {exc}"
        payload["error"] = unexpected_error
    finally:
        payload["phase"] = "finished"
        payload["finished_at_unix"] = time.time()
        payload["sample_summary"] = summarize_samples(all_samples)
        payload["completed"] = len(payload["legs"]) == 2 and all(leg.get("reached_goal") for leg in payload["legs"])
        write_json(args.output, payload)
        write_samples(args.samples_csv, all_samples)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if unexpected_error is None and payload.get("completed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
