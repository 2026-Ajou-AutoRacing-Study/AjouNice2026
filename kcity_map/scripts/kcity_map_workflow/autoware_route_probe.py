#!/usr/bin/env python3
"""Exercise Autoware routing against a Lanelet2 map and record coverage.

Run this inside the Autoware ROS 2 container after sourcing ROS and Autoware.
The script intentionally uses Autoware's public AD API services for route
requests, then verifies coverage from the published LaneletRoute message.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set

import lanelet2
import rclpy
from autoware_adapi_v1_msgs.srv import ClearRoute, InitializeLocalization, SetRoutePoints
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class LaneletInfo:
    lanelet_id: int
    centerline: List[tuple]
    centroid: tuple
    length: float


@dataclass
class RouteSpec:
    name: str
    start_lanelet: int
    goal_lanelet: int
    intended_lanelets: List[int]
    start_pose: Pose
    goal_pose: Pose
    waypoint_lanelets: List[int] = field(default_factory=list)
    waypoint_poses: List[Pose] = field(default_factory=list)
    required_lanelet: Optional[int] = None


def make_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def distance(a: tuple, b: tuple) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def pose_at_fraction(points: Sequence[tuple], fraction: float) -> Pose:
    if len(points) == 1:
        x, y, z = points[0]
        return make_pose(x, y, z, 0.0)

    segments = []
    total = 0.0
    for a, b in zip(points, points[1:]):
        seg = distance(a, b)
        segments.append(seg)
        total += seg

    target = max(0.0, min(1.0, fraction)) * total
    walked = 0.0
    for idx, seg in enumerate(segments):
        if walked + seg >= target or idx == len(segments) - 1:
            a = points[idx]
            b = points[idx + 1]
            ratio = 0.0 if seg <= 1e-9 else (target - walked) / seg
            x = a[0] + (b[0] - a[0]) * ratio
            y = a[1] + (b[1] - a[1]) * ratio
            z = a[2] + (b[2] - a[2]) * ratio
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return make_pose(x, y, z, yaw)
        walked += seg

    a, b = points[-2], points[-1]
    return make_pose(b[0], b[1], b[2], math.atan2(b[1] - a[1], b[0] - a[0]))


def lanelet_attr(lanelet, key: str, default: str = "") -> str:
    return lanelet.attributes[key] if key in lanelet.attributes else default


def load_lanelets(map_path: str, origin_lat: float, origin_lon: float):
    origin = lanelet2.io.Origin(origin_lat, origin_lon)
    projector = lanelet2.projection.UtmProjector(origin)
    lanelet_map = lanelet2.io.load(map_path, projector)
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)

    lanelets: Dict[int, LaneletInfo] = {}
    lanelet_objs = {}
    for lanelet in lanelet_map.laneletLayer:
        if lanelet_attr(lanelet, "subtype") != "road":
            continue
        if "participant:vehicle" in lanelet.attributes and lanelet.attributes["participant:vehicle"] != "yes":
            continue
        points = [(float(p.x), float(p.y), float(p.z)) for p in lanelet.centerline]
        if not points:
            continue
        length = sum(distance(a, b) for a, b in zip(points, points[1:]))
        centroid = (
            sum(p[0] for p in points) / len(points),
            sum(p[1] for p in points) / len(points),
            sum(p[2] for p in points) / len(points),
        )
        lanelet_id = int(lanelet.id)
        lanelets[lanelet_id] = LaneletInfo(lanelet_id, points, centroid, length)
        lanelet_objs[lanelet_id] = lanelet
    return lanelet_map, graph, lanelets, lanelet_objs


def following_ids(graph, lanelet_objs: Dict[int, object], lanelet_id: int) -> List[int]:
    return [int(l.id) for l in graph.following(lanelet_objs[lanelet_id])]


def previous_ids(graph, lanelet_objs: Dict[int, object], lanelet_id: int) -> List[int]:
    return [int(l.id) for l in graph.previous(lanelet_objs[lanelet_id])]


def choose_next(
    candidates: List[int],
    lanelets: Dict[int, LaneletInfo],
    seen: Set[int],
    prefer_uncovered: Optional[Set[int]] = None,
) -> Optional[int]:
    fresh = [candidate for candidate in candidates if candidate not in seen and candidate in lanelets]
    if not fresh:
        return None
    if prefer_uncovered:
        uncovered = [candidate for candidate in fresh if candidate in prefer_uncovered]
        if uncovered:
            fresh = uncovered
    return max(fresh, key=lambda lanelet_id: lanelets[lanelet_id].length)


def build_forward_path(
    start_id: int,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    max_lanelets: int,
    prefer_uncovered: Optional[Set[int]] = None,
) -> List[int]:
    path = [start_id]
    seen = {start_id}
    current = start_id
    for _ in range(max(1, max_lanelets) - 1):
        next_id = choose_next(following_ids(graph, lanelet_objs, current), lanelets, seen, prefer_uncovered)
        if next_id is None:
            break
        path.append(next_id)
        seen.add(next_id)
        current = next_id
    return path


def build_backward_path(
    goal_id: int,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    max_lanelets: int,
    prefer_uncovered: Optional[Set[int]] = None,
) -> List[int]:
    reverse_path = [goal_id]
    seen = {goal_id}
    current = goal_id
    for _ in range(max(1, max_lanelets) - 1):
        prev_id = choose_next(previous_ids(graph, lanelet_objs, current), lanelets, seen, prefer_uncovered)
        if prev_id is None:
            break
        reverse_path.append(prev_id)
        seen.add(prev_id)
        current = prev_id
    return list(reversed(reverse_path))


def route_from_path(
    name: str,
    path: List[int],
    lanelets: Dict[int, LaneletInfo],
    required_lanelet: Optional[int] = None,
    waypoint_lanelets: Optional[List[int]] = None,
) -> RouteSpec:
    start_id = path[0]
    goal_id = path[-1]
    goal_fraction = 0.75 if start_id == goal_id else 0.50
    waypoints = list(waypoint_lanelets or [])
    return RouteSpec(
        name=name,
        start_lanelet=start_id,
        goal_lanelet=goal_id,
        intended_lanelets=path,
        start_pose=pose_at_fraction(lanelets[start_id].centerline, 0.20),
        goal_pose=pose_at_fraction(lanelets[goal_id].centerline, goal_fraction),
        waypoint_lanelets=waypoints,
        waypoint_poses=[pose_at_fraction(lanelets[lanelet_id].centerline, 0.50) for lanelet_id in waypoints],
        required_lanelet=required_lanelet,
    )


def farthest_sample(lanelet_ids: Iterable[int], lanelets: Dict[int, LaneletInfo], count: int) -> List[int]:
    ids = list(lanelet_ids)
    if len(ids) <= count:
        return ids
    start = min(ids, key=lambda lanelet_id: (lanelets[lanelet_id].centroid[0], lanelets[lanelet_id].centroid[1]))
    selected = [start]
    remaining = set(ids)
    remaining.remove(start)
    while remaining and len(selected) < count:
        next_id = max(
            remaining,
            key=lambda lanelet_id: min(
                distance(lanelets[lanelet_id].centroid, lanelets[selected_id].centroid)
                for selected_id in selected
            ),
        )
        selected.append(next_id)
        remaining.remove(next_id)
    return selected


def make_sample_routes(
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    count: int,
    max_lanelets: int,
    excluded_starts: Set[int],
    excluded_goals: Set[int],
) -> List[RouteSpec]:
    candidate_paths = []
    for lanelet_id in lanelets:
        if lanelet_id in excluded_starts:
            continue
        path = build_forward_path(lanelet_id, graph, lanelets, lanelet_objs, max_lanelets)
        while len(path) > 1 and path[-1] in excluded_goals:
            path = path[:-1]
        if len(path) >= 3:
            candidate_paths.append(path)
    if not candidate_paths:
        candidate_paths = [[lanelet_id] for lanelet_id in lanelets]

    candidate_start_ids = [path[0] for path in candidate_paths]
    chosen = farthest_sample(candidate_start_ids, lanelets, count)
    path_by_start = {path[0]: path for path in candidate_paths}
    routes = []
    for idx, start_id in enumerate(chosen, 1):
        path = path_by_start[start_id]
        routes.append(route_from_path(f"sample_{idx:02d}", path, lanelets))
    return routes


def make_smoke_routes(
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    max_lanelets: int,
    excluded_starts: Set[int],
    excluded_goals: Set[int],
) -> List[RouteSpec]:
    return make_sample_routes(graph, lanelets, lanelet_objs, 3, max_lanelets, excluded_starts, excluded_goals)


class RouteProbe(Node):
    def __init__(self) -> None:
        super().__init__("kcity_route_probe")
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clear_client = self.create_client(ClearRoute, "/api/routing/clear_route")
        self.init_client = self.create_client(InitializeLocalization, "/api/localization/initialize")
        self.set_route_client = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", reliable)
        self.route_sub = self.create_subscription(LaneletRoute, "/planning/mission_planning/route", self._route_cb, transient)
        self.trajectory_sub = self.create_subscription(Trajectory, "/planning/trajectory", self._trajectory_cb, reliable)
        self.odom_sub = self.create_subscription(Odometry, "/localization/kinematic_state", self._odom_cb, reliable)
        self.latest_route: Optional[LaneletRoute] = None
        self.latest_trajectory_points = 0
        self.latest_odom: Optional[Odometry] = None

    def _route_cb(self, msg: LaneletRoute) -> None:
        self.latest_route = msg

    def _trajectory_cb(self, msg: Trajectory) -> None:
        self.latest_trajectory_points = len(msg.points)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def wait_for_services(self, timeout_sec: float) -> None:
        for client, name in [
            (self.clear_client, "/api/routing/clear_route"),
            (self.init_client, "/api/localization/initialize"),
            (self.set_route_client, "/api/routing/set_route_points"),
        ]:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"service not available: {name}")

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def call_service(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise TimeoutError(f"service call timed out: {client.srv_name}")

    def pose_msg(self, pose: Pose) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = pose
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[14] = 0.25
        covariance[35] = 0.01
        msg.pose.covariance = covariance
        return msg

    def publish_initial_pose(self, pose: Pose, repeats: int = 10) -> None:
        for _ in range(repeats):
            self.initial_pose_pub.publish(self.pose_msg(pose))
            rclpy.spin_once(self, timeout_sec=0.1)

    def odom_distance_to(self, pose: Pose) -> Optional[float]:
        if self.latest_odom is None:
            return None
        pos = self.latest_odom.pose.pose.position
        return math.hypot(pos.x - pose.position.x, pos.y - pose.position.y)

    def initialize_pose(self, pose: Pose, timeout_sec: float) -> Dict[str, object]:
        request = InitializeLocalization.Request()
        request.pose = [self.pose_msg(pose)]
        status = {"success": False, "code": None, "message": ""}
        try:
            response = self.call_service(self.init_client, request, timeout_sec)
            status = {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            status = {"success": False, "code": None, "message": f"service_exception: {exc}"}
        self.publish_initial_pose(pose)
        self.spin_until(lambda: self.odom_distance_to(pose) is not None and self.odom_distance_to(pose) < 2.0, 3.0)
        status["odom_distance_m"] = self.odom_distance_to(pose)
        return status

    def clear_route(self, timeout_sec: float) -> Dict[str, object]:
        request = ClearRoute.Request()
        try:
            response = self.call_service(self.clear_client, request, timeout_sec)
            status = {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            status = {"success": False, "code": None, "message": f"service_exception: {exc}"}
        self.latest_route = None
        self.latest_trajectory_points = 0
        self.spin_until(lambda: False, 0.25)
        return status

    def set_route(self, spec: RouteSpec, timeout_sec: float) -> Dict[str, object]:
        request = SetRoutePoints.Request()
        request.header.frame_id = "map"
        request.header.stamp = self.get_clock().now().to_msg()
        request.option.allow_goal_modification = True
        request.goal = spec.goal_pose
        request.waypoints = list(spec.waypoint_poses)
        try:
            response = self.call_service(self.set_route_client, request, timeout_sec)
            return {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}

    def route_ids(self) -> List[int]:
        if self.latest_route is None:
            return []
        ids = []
        for segment in self.latest_route.segments:
            ids.append(int(segment.preferred_primitive.id))
            ids.extend(int(primitive.id) for primitive in segment.primitives)
        deduped = []
        seen = set()
        for lanelet_id in ids:
            if lanelet_id not in seen:
                deduped.append(lanelet_id)
                seen.add(lanelet_id)
        return deduped

    def run_route(self, index: int, spec: RouteSpec, args) -> Dict[str, object]:
        started = time.monotonic()
        clear_status = self.clear_route(args.service_timeout)
        init_status = self.initialize_pose(spec.start_pose, args.service_timeout)
        self.latest_route = None
        self.latest_trajectory_points = 0
        route_status = self.set_route(spec, args.service_timeout)
        route_received = False
        if bool(route_status["success"]):
            route_received = self.spin_until(lambda: self.latest_route is not None and bool(self.latest_route.segments), args.route_timeout)
        trajectory_received = False
        if bool(route_status["success"]) and route_received:
            trajectory_received = self.spin_until(lambda: self.latest_trajectory_points > 0, args.trajectory_timeout)
        route_ids = self.route_ids()
        intended = set(spec.intended_lanelets)
        covered_intended = [lanelet_id for lanelet_id in spec.intended_lanelets if lanelet_id in route_ids]
        if spec.required_lanelet is not None:
            target_covered = spec.required_lanelet in route_ids
        else:
            target_covered = spec.start_lanelet in route_ids or spec.goal_lanelet in route_ids or bool(intended.intersection(route_ids))
        passed = (
            bool(route_status["success"])
            and route_received
            and target_covered
            and (trajectory_received or not args.require_trajectory)
        )
        return {
            "index": index,
            "name": spec.name,
            "start_lanelet": spec.start_lanelet,
            "goal_lanelet": spec.goal_lanelet,
            "required_lanelet": spec.required_lanelet,
            "waypoint_lanelets": spec.waypoint_lanelets,
            "waypoint_count": len(spec.waypoint_lanelets),
            "intended_lanelets": spec.intended_lanelets,
            "intended_count": len(spec.intended_lanelets),
            "covered_intended": covered_intended,
            "covered_intended_count": len(covered_intended),
            "clear_status": clear_status,
            "initialization_status": init_status,
            "route_status": route_status,
            "route_received": route_received,
            "route_ids": route_ids,
            "route_id_count": len(route_ids),
            "trajectory_received": trajectory_received,
            "trajectory_points": self.latest_trajectory_points,
            "target_covered": target_covered,
            "pass": passed,
            "duration_sec": round(time.monotonic() - started, 3),
        }


def write_json(path: Path, payload: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def trim_excluded_goal(path: List[int], excluded_goals: Set[int]) -> List[int]:
    while len(path) > 1 and path[-1] in excluded_goals:
        path = path[:-1]
    return path


def build_through_path(
    target_id: int,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    pending: Set[int],
    max_lanelets: int,
    excluded_starts: Set[int],
    excluded_goals: Set[int],
) -> List[int]:
    total_budget = max(2, max_lanelets)
    backward_budget = max(1, (total_budget + 1) // 2)
    backward = build_backward_path(target_id, graph, lanelets, lanelet_objs, backward_budget, pending)
    forward_budget = max(1, total_budget - len(backward) + 1)
    forward = build_forward_path(target_id, graph, lanelets, lanelet_objs, forward_budget, pending)
    combined = trim_excluded_goal(backward + forward[1:], excluded_goals)
    if len(combined) >= 2 and combined[0] not in excluded_starts:
        return combined

    forward = trim_excluded_goal(build_forward_path(target_id, graph, lanelets, lanelet_objs, total_budget, pending), excluded_goals)
    if len(forward) >= 2 and forward[0] not in excluded_starts:
        return forward

    backward = trim_excluded_goal(build_backward_path(target_id, graph, lanelets, lanelet_objs, total_budget, pending), excluded_goals)
    if len(backward) >= 2 and backward[0] not in excluded_starts:
        return backward
    return [target_id]


def coverage_route_for_target(
    target_id: int,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    pending: Set[int],
    max_lanelets: int,
    route_index: int,
    excluded_starts: Set[int],
    excluded_goals: Set[int],
    strategy: str,
) -> RouteSpec:
    if strategy == "micro":
        return route_from_path(f"coverage_{route_index:04d}", [target_id], lanelets, required_lanelet=target_id)
    if strategy == "through":
        path = build_through_path(
            target_id,
            graph,
            lanelets,
            lanelet_objs,
            pending,
            max_lanelets,
            excluded_starts,
            excluded_goals,
        )
        return route_from_path(
            f"coverage_{route_index:04d}",
            path,
            lanelets,
            required_lanelet=target_id,
            waypoint_lanelets=path[1:-1],
        )
    path = build_forward_path(target_id, graph, lanelets, lanelet_objs, max_lanelets, pending)
    path = trim_excluded_goal(path, excluded_goals)
    if path[0] in excluded_starts or len(path) == 1:
        backward = build_backward_path(target_id, graph, lanelets, lanelet_objs, max_lanelets, pending)
        forward = build_forward_path(target_id, graph, lanelets, lanelet_objs, max_lanelets, pending)
        combined = backward + forward[1:]
        combined = trim_excluded_goal(combined, excluded_goals)
        if combined and combined[0] not in excluded_starts and len(combined) >= 2:
            path = combined
    if len(path) == 1:
        path = build_backward_path(target_id, graph, lanelets, lanelet_objs, max_lanelets, pending)
        path = trim_excluded_goal(path, excluded_goals)
    return route_from_path(
        f"coverage_{route_index:04d}",
        path,
        lanelets,
        required_lanelet=target_id,
        waypoint_lanelets=path[1:-1],
    )


def run_specs(
    node: RouteProbe,
    specs: Sequence[RouteSpec],
    args,
    payload: Dict[str, object],
    road_ids: Set[int],
) -> Dict[str, object]:
    covered: Set[int] = set()
    results = []
    for idx, spec in enumerate(specs, 1):
        result = node.run_route(idx, spec, args)
        results.append(result)
        if result["pass"]:
            covered.update(lanelet_id for lanelet_id in result["route_ids"] if lanelet_id in road_ids)
        payload["results"] = results
        payload["summary"].update(
            {
                "executed_tests": len(results),
                "passed_tests": sum(1 for item in results if item["pass"]),
                "failed_tests": sum(1 for item in results if not item["pass"]),
                "covered_lanelets": len(covered),
                "uncovered_lanelets": len(road_ids - covered),
            }
        )
        write_json(Path(args.output), payload)
        print(
            f"[{idx}/{len(specs)}] {spec.name} start={spec.start_lanelet} "
            f"goal={spec.goal_lanelet} pass={result['pass']} "
            f"route_ids={result['route_id_count']} covered={len(covered)}/{len(road_ids)}",
            flush=True,
        )
        if args.stop_on_failure and not result["pass"]:
            break
    payload["summary"]["uncovered_ids"] = sorted(road_ids - covered)
    return payload


def run_coverage(
    node: RouteProbe,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    args,
    payload: Dict[str, object],
    road_ids: Set[int],
    coverage_scope: Set[int],
    initial_covered: Set[int],
    excluded_starts: Set[int],
    excluded_goals: Set[int],
) -> Dict[str, object]:
    pending: Set[int] = set(coverage_scope) - initial_covered
    covered: Set[int] = set(initial_covered) & road_ids
    failed_targets: Set[int] = set()
    results = []
    route_index = 1
    while pending and (args.max_tests <= 0 or len(results) < args.max_tests):
        target_id = max(
            pending,
            key=lambda lanelet_id: (
                len(build_forward_path(lanelet_id, graph, lanelets, lanelet_objs, args.max_route_lanelets, pending)),
                lanelets[lanelet_id].length,
            ),
        )
        spec = coverage_route_for_target(
            target_id,
            graph,
            lanelets,
            lanelet_objs,
            pending,
            args.max_route_lanelets,
            route_index,
            excluded_starts,
            excluded_goals,
            args.coverage_strategy,
        )
        result = node.run_route(route_index, spec, args)
        results.append(result)
        newly_covered = {lanelet_id for lanelet_id in result["route_ids"] if lanelet_id in road_ids} if result["pass"] else set()
        covered.update(newly_covered)
        pending -= newly_covered & coverage_scope
        if target_id not in covered:
            failed_targets.add(target_id)
            pending.discard(target_id)
        payload["results"] = results
        payload["summary"].update(
            {
                "executed_tests": len(results),
                "passed_tests": sum(1 for item in results if item["pass"]),
                "failed_tests": sum(1 for item in results if not item["pass"]),
                "covered_lanelets": len(covered),
                "uncovered_lanelets": len(road_ids - covered),
                "coverage_scope_lanelets": len(coverage_scope),
                "covered_scope_lanelets": len(coverage_scope & covered),
                "uncovered_scope_lanelets": len(coverage_scope - covered),
                "pending_lanelets": len(pending),
                "failed_target_ids": sorted(failed_targets),
                "covered_ids": sorted(covered),
                "uncovered_ids": sorted(coverage_scope - covered),
            }
        )
        write_json(Path(args.output), payload)
        print(
            f"[coverage {route_index}] target={target_id} start={spec.start_lanelet} "
            f"goal={spec.goal_lanelet} pass={result['pass']} "
            f"route_ids={result['route_id_count']} new={len(newly_covered)} "
            f"covered={len(covered)}/{len(road_ids)} pending={len(pending)}",
            flush=True,
        )
        if args.stop_on_failure and not result["pass"]:
            break
        route_index += 1
    payload["summary"]["covered_ids"] = sorted(covered)
    payload["summary"]["uncovered_ids"] = sorted(coverage_scope - covered)
    payload["summary"]["all_uncovered_ids"] = sorted(road_ids - covered)
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="/home/aw/autoware_data/maps/kcity-v24/lanelet2_map.osm")
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    parser.add_argument("--mode", choices=["smoke", "sample20", "coverage"], required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--sample-count", type=int, default=20)
    parser.add_argument("--max-route-lanelets", type=int, default=40)
    parser.add_argument("--max-tests", type=int, default=0, help="0 means no explicit cap")
    parser.add_argument("--coverage-strategy", choices=["path", "micro", "through"], default="path")
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--route-timeout", type=float, default=10.0)
    parser.add_argument("--trajectory-timeout", type=float, default=10.0)
    parser.add_argument("--require-trajectory", action="store_true", default=True)
    parser.add_argument("--no-require-trajectory", dest="require_trajectory", action="store_false")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--exclude-start-lanelets", default="")
    parser.add_argument("--exclude-goal-lanelets", default="")
    parser.add_argument("--target-lanelets", default="")
    parser.add_argument("--target-lanelets-file", default="")
    parser.add_argument("--initial-covered-lanelets", default="")
    parser.add_argument("--initial-covered-lanelets-file", default="")
    return parser.parse_args()


def parse_id_set(value: str) -> Set[int]:
    if not value.strip():
        return set()
    return {int(part.strip()) for part in value.split(",") if part.strip()}


def parse_id_file(path: str) -> Set[int]:
    if not path:
        return set()
    text = Path(path).read_text(encoding="utf-8")
    normalized = text.replace(",", "\n").replace(" ", "\n").replace("\t", "\n")
    return {int(part.strip()) for part in normalized.splitlines() if part.strip()}


def main() -> int:
    args = parse_args()
    _, graph, lanelets, lanelet_objs = load_lanelets(args.map, args.origin_lat, args.origin_lon)
    road_ids = set(lanelets)
    excluded_starts = parse_id_set(args.exclude_start_lanelets)
    excluded_goals = parse_id_set(args.exclude_goal_lanelets)
    target_lanelets = (parse_id_set(args.target_lanelets) | parse_id_file(args.target_lanelets_file)) & road_ids
    initial_covered = (parse_id_set(args.initial_covered_lanelets) | parse_id_file(args.initial_covered_lanelets_file)) & road_ids
    coverage_scope = target_lanelets if target_lanelets else set(road_ids)
    unknown_targets = sorted((parse_id_set(args.target_lanelets) | parse_id_file(args.target_lanelets_file)) - road_ids)
    payload: Dict[str, object] = {
        "summary": {
            "map": args.map,
            "mode": args.mode,
            "origin_lat": args.origin_lat,
            "origin_lon": args.origin_lon,
            "total_road_lanelets": len(road_ids),
            "coverage_scope_lanelets": len(coverage_scope),
            "initial_covered_lanelets": len(initial_covered),
            "unknown_target_lanelets": unknown_targets,
            "excluded_start_lanelets": sorted(excluded_starts),
            "excluded_goal_lanelets": sorted(excluded_goals),
            "started_at_unix": time.time(),
        },
        "results": [],
    }
    write_json(Path(args.output), payload)

    rclpy.init()
    node = RouteProbe()
    try:
        node.wait_for_services(args.service_timeout)
        try:
            if args.mode == "smoke":
                specs = make_smoke_routes(
                    graph, lanelets, lanelet_objs, args.max_route_lanelets, excluded_starts, excluded_goals
                )
                payload = run_specs(node, specs, args, payload, road_ids)
            elif args.mode == "sample20":
                specs = make_sample_routes(
                    graph,
                    lanelets,
                    lanelet_objs,
                    args.sample_count,
                    args.max_route_lanelets,
                    excluded_starts,
                    excluded_goals,
                )
                payload = run_specs(node, specs, args, payload, road_ids)
            else:
                payload = run_coverage(
                    node,
                    graph,
                    lanelets,
                    lanelet_objs,
                    args,
                    payload,
                    road_ids,
                    coverage_scope,
                    initial_covered,
                    excluded_starts,
                    excluded_goals,
                )
        except KeyboardInterrupt:
            payload["summary"]["interrupted"] = True
    finally:
        payload["summary"]["finished_at_unix"] = time.time()
        write_json(Path(args.output), payload)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
