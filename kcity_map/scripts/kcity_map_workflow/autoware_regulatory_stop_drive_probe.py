#!/usr/bin/env python3
"""Drive through Lanelet2 stop rules and verify planned and actual stopping.

Run inside the Autoware planning-simulator container after sourcing the
Autoware installation.  Traffic-light tests continuously inject a red signal,
hold it after the vehicle stops, then inject green and verify that the vehicle
resumes.  Stop-sign tests rely on the stop-line module's own release logic.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import lanelet2
import rclpy
from autoware_adapi_v1_msgs.srv import AcceptStart
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray
from autoware_planning_msgs.msg import Trajectory

from autoware_course_drive import CourseDriveProbe
from autoware_route_probe import (
    LaneletInfo,
    build_backward_path,
    build_forward_path,
    load_lanelets,
    make_pose,
    pose_at_fraction,
    route_from_path,
)


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class StopTarget:
    lanelet_id: int
    regulatory_element_id: int
    stop_line_id: int
    rule_type: str
    traffic_light_way_ids: Tuple[int, ...]


def parse_ids(value: str) -> Set[int]:
    return {int(part.strip()) for part in value.split(",") if part.strip()}


def load_targets(path: Path, selected_lanelets: Set[int], expected_rule_type: str) -> List[StopTarget]:
    targets = []
    with path.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            rule_type = row.get("rule_type", "")
            if rule_type != expected_rule_type:
                continue
            for lanelet_text in row["lanelet_id"].split():
                lanelet_id = int(lanelet_text)
                if selected_lanelets and lanelet_id not in selected_lanelets:
                    continue
                traffic_light_way_ids = tuple(
                    int(value) for value in row.get("traffic_light_way_ids", "").split() if value
                )
                targets.append(
                    StopTarget(
                        lanelet_id=lanelet_id,
                        regulatory_element_id=int(row["regulatory_element_id"]),
                        stop_line_id=int(row["stop_line_id"]),
                        rule_type=rule_type,
                        traffic_light_way_ids=traffic_light_way_ids,
                    )
                )
    return targets


def load_traffic_light_regulatory_groups(path: str) -> Tuple[Set[int], Dict[int, Set[int]]]:
    root = ElementTree.parse(path).getroot()
    regulatory_ways: Dict[int, Set[int]] = {}
    way_to_regulatory_ids: Dict[int, Set[int]] = {}
    for relation in root.findall("relation"):
        relation_tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if relation_tags.get("type") == "regulatory_element" and relation_tags.get("subtype") == "traffic_light":
            relation_id = int(relation.get("id"))
            way_ids = {
                int(member.get("ref"))
                for member in relation.findall("member")
                if member.get("type") == "way" and member.get("role") == "refers"
            }
            regulatory_ways[relation_id] = way_ids
            for way_id in way_ids:
                way_to_regulatory_ids.setdefault(way_id, set()).add(relation_id)
    equivalents = {}
    for relation_id, way_ids in regulatory_ways.items():
        equivalents[relation_id] = set().union(
            *(way_to_regulatory_ids[way_id] for way_id in way_ids)
        ) if way_ids else {relation_id}
    return set(regulatory_ways), equivalents


def distance_xy(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def point_to_segment_distance(point: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
    px, py = float(point[0]), float(point[1])
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denominator))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def point_to_polyline_distance(point: Sequence[float], line: Sequence[Point3]) -> float:
    if not line:
        return float("inf")
    if len(line) == 1:
        return distance_xy(point, line[0])
    return min(point_to_segment_distance(point, a, b) for a, b in zip(line, line[1:]))


def midpoint(points: Sequence[Point3]) -> Point3:
    if len(points) == 1:
        return points[0]
    lengths = [distance_xy(a, b) for a, b in zip(points, points[1:])]
    target = sum(lengths) / 2.0
    walked = 0.0
    for index, length in enumerate(lengths):
        if walked + length >= target:
            ratio = 0.0 if length <= 1e-9 else (target - walked) / length
            a, b = points[index], points[index + 1]
            return (
                a[0] + (b[0] - a[0]) * ratio,
                a[1] + (b[1] - a[1]) * ratio,
                a[2] + (b[2] - a[2]) * ratio,
            )
        walked += length
    return points[-1]


def stop_line_points(lanelet_map, stop_line_id: int) -> List[Point3]:
    line = lanelet_map.lineStringLayer.get(stop_line_id)
    return [(float(point.x), float(point.y), float(point.z)) for point in line]


def concat_path_points(path: Sequence[int], lanelets: Dict[int, LaneletInfo]) -> List[Point3]:
    points: List[Point3] = []
    for lanelet_id in path:
        for point in lanelets[lanelet_id].centerline:
            if not points or distance_xy(points[-1], point) > 0.01:
                points.append(point)
    return points


def cumulative_lengths(points: Sequence[Point3]) -> List[float]:
    values = [0.0]
    for a, b in zip(points, points[1:]):
        values.append(values[-1] + distance_xy(a, b))
    return values


def project_arc(points: Sequence[Point3], cumulative: Sequence[float], point: Sequence[float]) -> float:
    best_distance = float("inf")
    best_arc = 0.0
    px, py = float(point[0]), float(point[1])
    for index, (a, b) in enumerate(zip(points, points[1:])):
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        dx = bx - ax
        dy = by - ay
        denominator = dx * dx + dy * dy
        ratio = 0.0 if denominator <= 1e-9 else max(
            0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denominator)
        )
        cx = ax + ratio * dx
        cy = ay + ratio * dy
        distance = math.hypot(px - cx, py - cy)
        if distance < best_distance:
            best_distance = distance
            best_arc = cumulative[index] + ratio * math.sqrt(denominator)
    return best_arc


def route_path_for_target(
    target_id: int,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    max_lanelets_each_side: int,
) -> List[int]:
    backward = build_backward_path(
        target_id,
        graph,
        lanelets,
        lanelet_objs,
        max_lanelets_each_side,
    )
    forward = build_forward_path(
        target_id,
        graph,
        lanelets,
        lanelet_objs,
        max_lanelets_each_side,
    )
    path = backward + forward[1:]
    deduped = []
    for lanelet_id in path:
        if not deduped or deduped[-1] != lanelet_id:
            deduped.append(lanelet_id)
    return deduped


def trajectory_zero_distance(trajectory: Optional[Trajectory], stop_line: Sequence[Point3]) -> Optional[float]:
    if trajectory is None:
        return None
    distances = []
    for point in trajectory.points:
        if abs(float(point.longitudinal_velocity_mps)) > 0.05:
            continue
        position = point.pose.position
        distances.append(point_to_polyline_distance((position.x, position.y), stop_line))
    return min(distances) if distances else None


def planning_factor_distance(
    factor: Optional[PlanningFactorArray],
    stop_line: Sequence[Point3],
) -> Optional[float]:
    if factor is None:
        return None
    distances = []
    for item in factor.factors:
        if int(item.behavior) != 3:
            continue
        for control_point in item.control_points:
            position = control_point.pose.position
            distances.append(point_to_polyline_distance((position.x, position.y), stop_line))
    return min(distances) if distances else None


class RegulatoryStopDriveProbe(CourseDriveProbe):
    def __init__(self) -> None:
        super().__init__()
        self.traffic_light_factor: Optional[PlanningFactorArray] = None
        self.stop_line_factor: Optional[PlanningFactorArray] = None
        self.debug_traffic_signal: Optional[TrafficLightGroup] = None
        self.signal_pub = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            10,
        )
        self.create_subscription(
            PlanningFactorArray,
            "/planning/planning_factors/traffic_light",
            self._traffic_light_factor_cb,
            10,
        )
        self.create_subscription(
            PlanningFactorArray,
            "/planning/planning_factors/stop_line",
            self._stop_line_factor_cb,
            10,
        )
        self.create_subscription(
            TrafficLightGroup,
            "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal",
            self._debug_traffic_signal_cb,
            10,
        )
        self.signal_ids: Tuple[int, ...] = ()
        self.signal_states: Dict[int, int] = {}
        self.all_traffic_light_regulatory_ids: Set[int] = set()
        self.equivalent_traffic_light_regulatory_ids: Dict[int, Set[int]] = {}
        self.accept_start_client = self.create_client(AcceptStart, "/api/motion/accept_start")
        self.signal_color = TrafficLightElement.UNKNOWN
        self.signal_timer = self.create_timer(0.1, self._publish_signal)
        self.min_factor_distance_m: Optional[float] = None
        self.min_zero_trajectory_distance_m: Optional[float] = None
        self.active_stop_line: List[Point3] = []

    def _trajectory_cb(self, msg: Trajectory) -> None:
        super()._trajectory_cb(msg)
        distance = trajectory_zero_distance(msg, self.active_stop_line)
        if distance is not None:
            self.min_zero_trajectory_distance_m = (
                distance
                if self.min_zero_trajectory_distance_m is None
                else min(self.min_zero_trajectory_distance_m, distance)
            )

    def _traffic_light_factor_cb(self, msg: PlanningFactorArray) -> None:
        self.traffic_light_factor = msg
        self._record_factor(msg)

    def _stop_line_factor_cb(self, msg: PlanningFactorArray) -> None:
        self.stop_line_factor = msg
        self._record_factor(msg)

    def _debug_traffic_signal_cb(self, msg: TrafficLightGroup) -> None:
        if msg.elements:
            self.debug_traffic_signal = msg

    def _record_factor(self, msg: PlanningFactorArray) -> None:
        distance = planning_factor_distance(msg, self.active_stop_line)
        if distance is not None:
            self.min_factor_distance_m = (
                distance if self.min_factor_distance_m is None else min(self.min_factor_distance_m, distance)
            )

    def reset_observations(self, stop_line: Sequence[Point3]) -> None:
        self.active_stop_line = list(stop_line)
        self.traffic_light_factor = None
        self.stop_line_factor = None
        self.debug_traffic_signal = None
        self.latest_trajectory = None
        self.min_factor_distance_m = None
        self.min_zero_trajectory_distance_m = None

    def set_signal(self, ids: Iterable[int], color: int) -> None:
        self.signal_ids = tuple(sorted(set(int(value) for value in ids)))
        self.signal_color = color
        self.signal_states = {signal_id: color for signal_id in self.signal_ids}
        for _ in range(5):
            self._publish_signal()
            rclpy.spin_once(self, timeout_sec=0.02)

    def set_signal_states(self, states: Dict[int, int]) -> None:
        self.signal_states = dict(sorted((int(key), int(value)) for key, value in states.items()))
        self.signal_ids = tuple(self.signal_states)
        for _ in range(5):
            self._publish_signal()
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_signal(self) -> None:
        if not self.signal_states:
            return
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        for signal_id, color in self.signal_states.items():
            group = TrafficLightGroup()
            group.traffic_light_group_id = signal_id
            element = TrafficLightElement()
            element.color = color
            element.shape = TrafficLightElement.CIRCLE
            element.status = TrafficLightElement.SOLID_ON
            element.confidence = 1.0
            group.elements.append(element)
            msg.traffic_light_groups.append(group)
        self.signal_pub.publish(msg)

    def accept_start(self, timeout_sec: float) -> Dict[str, object]:
        if not self.accept_start_client.wait_for_service(timeout_sec=timeout_sec):
            return {"success": False, "code": None, "message": "service_not_available"}
        try:
            response = self.call_service(self.accept_start_client, AcceptStart.Request(), timeout_sec)
            return {
                "success": bool(response.status.success),
                "code": int(response.status.code),
                "message": response.status.message,
            }
        except Exception as exc:
            return {"success": False, "code": None, "message": f"service_exception: {exc}"}


def speed_mps(odom) -> float:
    twist = odom.twist.twist.linear
    return math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)


def prepare_route(node: RegulatoryStopDriveProbe, spec, args) -> Dict[str, object]:
    stop_status = node.change_to_stop(3.0)
    node.spin_until(
        lambda: node.operation_mode is not None and node.operation_mode.mode == OperationModeState.STOP,
        args.mode_transition_timeout,
    )
    clear_attempts = []
    clear_status = {"success": False, "code": None, "message": "not_attempted"}
    for _ in range(args.clear_attempts):
        clear_status = node.clear_route(args.service_timeout)
        clear_attempts.append(clear_status)
        if clear_status.get("success"):
            break
        node.spin_until(lambda: False, args.clear_retry_interval)
    init_status = node.initialize_pose(spec.start_pose, args.service_timeout)
    node.latest_route = None
    node.latest_trajectory_points = 0
    route_status = node.set_route(spec, args.service_timeout)
    route_received = False
    if route_status.get("success"):
        route_received = node.spin_until(
            lambda: node.latest_route is not None and bool(node.latest_route.segments),
            args.route_timeout,
        )
    trajectory_received = False
    if route_received:
        trajectory_received = node.spin_until(
            lambda: node.latest_trajectory is not None and bool(node.latest_trajectory.points),
            args.trajectory_timeout,
        )
    return {
        "change_to_stop": stop_status,
        "clear_route": clear_status,
        "clear_attempts": clear_attempts,
        "initialize": init_status,
        "set_route": route_status,
        "route_received": route_received,
        "trajectory_received": trajectory_received,
        "route_ids": node.route_ids(),
    }


def run_target(
    node: RegulatoryStopDriveProbe,
    target: StopTarget,
    lanelet_map,
    graph,
    lanelets: Dict[int, LaneletInfo],
    lanelet_objs: Dict[int, object],
    args,
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    if args.route_strategy == "target_forward":
        path = build_forward_path(
            target.lanelet_id,
            graph,
            lanelets,
            lanelet_objs,
            args.max_lanelets_each_side,
        )
    else:
        path = route_path_for_target(
            target.lanelet_id,
            graph,
            lanelets,
            lanelet_objs,
            args.max_lanelets_each_side,
        )
    if len(path) < 2:
        return ({"pass": False, "failure": "route_path_too_short", "path": path}, [])

    spec = route_from_path(
        f"regulatory_stop_{target.lanelet_id}",
        path,
        lanelets,
        required_lanelet=target.lanelet_id,
        waypoint_lanelets=path[1:-1],
    )
    stop_line = stop_line_points(lanelet_map, target.stop_line_id)
    stop_midpoint = midpoint(stop_line)
    target_points = lanelets[target.lanelet_id].centerline
    approach_start = target_points[-2]
    approach_end = target_points[-1]
    approach_length = distance_xy(approach_start, approach_end)
    approach_unit = (
        (approach_end[0] - approach_start[0]) / approach_length,
        (approach_end[1] - approach_start[1]) / approach_length,
    )

    def signed_stop_distance(position) -> float:
        return (
            (stop_midpoint[0] - float(position.x)) * approach_unit[0]
            + (stop_midpoint[1] - float(position.y)) * approach_unit[1]
        )

    node.reset_observations(stop_line)
    if args.signal_id_mode == "relation":
        signal_ids = tuple(
            sorted(
                node.equivalent_traffic_light_regulatory_ids.get(
                    target.regulatory_element_id,
                    {target.regulatory_element_id},
                )
            )
        )
    elif args.signal_id_mode == "ways":
        signal_ids = target.traffic_light_way_ids
    else:
        signal_ids = (target.regulatory_element_id,) + target.traffic_light_way_ids
    if target.rule_type == "traffic_light":
        states = {
            signal_id: TrafficLightElement.GREEN
            for signal_id in node.all_traffic_light_regulatory_ids
        }
        for signal_id in signal_ids:
            states[signal_id] = TrafficLightElement.RED
        node.set_signal_states(states)
    else:
        node.set_signal_states(
            {
                signal_id: TrafficLightElement.GREEN
                for signal_id in node.all_traffic_light_regulatory_ids
            }
        )

    setup = prepare_route(node, spec, args)
    node.spin_until(
        lambda: (
            node.min_factor_distance_m is not None
            and node.min_zero_trajectory_distance_m is not None
        ),
        args.planning_observation_timeout,
    )
    planned_factor = (
        node.min_factor_distance_m is not None
        and node.min_factor_distance_m <= args.planning_factor_tolerance_m
    )
    planned_zero = (
        node.min_zero_trajectory_distance_m is not None
        and node.min_zero_trajectory_distance_m <= args.trajectory_stop_tolerance_m
    )
    red_debug_signal = {
        "id": int(node.debug_traffic_signal.traffic_light_group_id),
        "colors": [int(element.color) for element in node.debug_traffic_signal.elements],
    } if node.debug_traffic_signal is not None else None

    engage_status = node.engage_autonomous(args.service_timeout)
    samples: List[Dict[str, object]] = []
    started = time.monotonic()
    moved = False
    stopped_since: Optional[float] = None
    actual_stop = False
    crossed_while_red = False
    stop_signed_distance = None
    stop_euclidean_distance = None
    min_speed_near_stop = float("inf")
    max_speed_before_stop = 0.0
    approached_target = False

    while time.monotonic() - started < args.stop_timeout:
        rclpy.spin_once(node, timeout_sec=args.sample_period)
        if node.latest_odom is None:
            continue
        odom = node.latest_odom
        position = odom.pose.pose.position
        speed = speed_mps(odom)
        signed_distance = signed_stop_distance(position)
        euclidean_distance = point_to_polyline_distance((position.x, position.y), stop_line)
        max_speed_before_stop = max(max_speed_before_stop, speed)
        if speed > args.moved_speed_mps:
            moved = True
        if abs(signed_distance) <= args.near_stop_distance_m:
            min_speed_near_stop = min(min_speed_near_stop, speed)
        if 0.0 <= signed_distance <= args.approach_detection_distance_m:
            approached_target = True
        if (
            target.rule_type == "traffic_light"
            and moved
            and approached_target
            and signed_distance < -args.crossing_tolerance_m
        ):
            crossed_while_red = True

        samples.append(
            {
                "lanelet_id": target.lanelet_id,
                "phase": "stop_approach",
                "time_sec": round(time.monotonic() - started, 3),
                "x": position.x,
                "y": position.y,
                "speed_mps": speed,
                "signed_distance_to_stop_m": signed_distance,
                "euclidean_distance_to_stop_m": euclidean_distance,
            }
        )

        stopped_now = (
            moved
            and speed <= args.stopped_speed_mps
            and 0.0 <= signed_distance <= args.near_stop_distance_m
            and euclidean_distance <= args.near_stop_distance_m
        )
        if stopped_now:
            if stopped_since is None:
                stopped_since = time.monotonic()
            elif time.monotonic() - stopped_since >= args.stop_hold_sec:
                actual_stop = True
                stop_signed_distance = signed_distance
                stop_euclidean_distance = euclidean_distance
                break
        else:
            stopped_since = None
        if crossed_while_red:
            break

    if target.rule_type == "traffic_light":
        node.set_signal_states(
            {
                signal_id: TrafficLightElement.GREEN
                for signal_id in node.all_traffic_light_regulatory_ids | set(signal_ids)
            }
        )

    node.spin_until(
        lambda: (
            node.debug_traffic_signal is not None
            and any(element.color == TrafficLightElement.GREEN for element in node.debug_traffic_signal.elements)
        ),
        args.green_debug_timeout,
    )
    green_debug_signal = {
        "id": int(node.debug_traffic_signal.traffic_light_group_id),
        "colors": [int(element.color) for element in node.debug_traffic_signal.elements],
    } if node.debug_traffic_signal is not None else None
    node.spin_until(lambda: False, args.green_settle_sec)
    green_factor_distance = planning_factor_distance(node.traffic_light_factor, stop_line)
    green_zero_distance = trajectory_zero_distance(node.latest_trajectory, stop_line)
    release_engage_status = node.engage_autonomous(args.service_timeout)
    accept_start_status = node.accept_start(args.service_timeout)
    operation_mode_after_green = {
        "mode": int(node.operation_mode.mode) if node.operation_mode is not None else None,
        "is_autoware_control_enabled": bool(node.operation_mode.is_autoware_control_enabled)
        if node.operation_mode is not None
        else None,
        "is_autonomous_mode_available": bool(node.operation_mode.is_autonomous_mode_available)
        if node.operation_mode is not None
        else None,
    }

    resumed = False
    reached_goal = False
    release_started = time.monotonic()
    while time.monotonic() - release_started < args.release_timeout:
        rclpy.spin_once(node, timeout_sec=args.sample_period)
        if node.latest_odom is None:
            continue
        odom = node.latest_odom
        position = odom.pose.pose.position
        speed = speed_mps(odom)
        signed_distance = signed_stop_distance(position)
        euclidean_distance = point_to_polyline_distance((position.x, position.y), stop_line)
        if speed > args.resume_speed_mps or signed_distance < -args.crossing_tolerance_m:
            resumed = True
        goal_distance = node.odom_distance_to(spec.goal_pose)
        if node.is_arrived() or (goal_distance is not None and goal_distance < args.goal_threshold_m):
            reached_goal = True
            break
        samples.append(
            {
                "lanelet_id": target.lanelet_id,
                "phase": "release",
                "time_sec": round(time.monotonic() - release_started, 3),
                "x": position.x,
                "y": position.y,
                "speed_mps": speed,
                "signed_distance_to_stop_m": signed_distance,
                "euclidean_distance_to_stop_m": euclidean_distance,
            }
        )
        if resumed and signed_distance < -args.release_crossing_distance_m:
            break

    final_planned_factor = (
        node.min_factor_distance_m is not None
        and node.min_factor_distance_m <= args.planning_factor_tolerance_m
    )
    final_planned_zero = (
        node.min_zero_trajectory_distance_m is not None
        and node.min_zero_trajectory_distance_m <= args.trajectory_stop_tolerance_m
    )
    cleanup_stop_status = node.change_to_stop(3.0)
    node.spin_until(
        lambda: node.operation_mode is not None and node.operation_mode.mode == OperationModeState.STOP,
        args.mode_transition_timeout,
    )
    cleanup_clear_status = node.clear_route(args.service_timeout)

    result_pass = (
        bool(setup["set_route"].get("success"))
        and setup["route_received"]
        and setup["trajectory_received"]
        and target.lanelet_id in setup["route_ids"]
        and final_planned_factor
        and final_planned_zero
        and actual_stop
        and not crossed_while_red
        and (resumed or not args.require_resume)
    )
    return (
        {
            "lanelet_id": target.lanelet_id,
            "regulatory_element_id": target.regulatory_element_id,
            "stop_line_id": target.stop_line_id,
            "rule_type": target.rule_type,
            "traffic_light_way_ids": list(target.traffic_light_way_ids),
            "signal_ids_published": list(signal_ids) if target.rule_type == "traffic_light" else [],
            "background_green_signal_count": len(node.all_traffic_light_regulatory_ids),
            "signal_id_mode": args.signal_id_mode,
            "red_debug_signal": red_debug_signal,
            "green_debug_signal": green_debug_signal,
            "green_factor_distance_m": green_factor_distance,
            "green_stop_factor_released": green_factor_distance is None,
            "green_zero_trajectory_distance_m": green_zero_distance,
            "release_engage_status": release_engage_status,
            "accept_start_status": accept_start_status,
            "operation_mode_after_green": operation_mode_after_green,
            "path": path,
            "route_strategy": args.route_strategy,
            "setup": setup,
            "engage_status": engage_status,
            "planned_stop_factor_before_engage": planned_factor,
            "planned_stop_factor": final_planned_factor,
            "min_planning_factor_distance_m": node.min_factor_distance_m,
            "planned_zero_velocity_before_engage": planned_zero,
            "planned_zero_velocity": final_planned_zero,
            "min_zero_trajectory_distance_m": node.min_zero_trajectory_distance_m,
            "actual_stop": actual_stop,
            "crossed_while_red": crossed_while_red,
            "stop_signed_distance_m": stop_signed_distance,
            "stop_euclidean_distance_m": stop_euclidean_distance,
            "min_speed_near_stop_mps": min_speed_near_stop if math.isfinite(min_speed_near_stop) else None,
            "max_speed_before_stop_mps": max_speed_before_stop,
            "resumed_after_release": resumed,
            "reached_goal_after_release": reached_goal,
            "cleanup_stop_status": cleanup_stop_status,
            "cleanup_clear_status": cleanup_clear_status,
            "sample_count": len(samples),
            "pass": result_pass,
        },
        samples,
    )


def write_json(path: Path, payload: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def write_samples(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "lanelet_id",
        "phase",
        "time_sec",
        "x",
        "y",
        "speed_mps",
        "signed_distance_to_stop_m",
        "euclidean_distance_to_stop_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--fix-csv", required=True, type=Path)
    parser.add_argument("--rule-type", choices=["traffic_light", "stop_sign"], required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--samples-csv", required=True, type=Path)
    parser.add_argument("--target-lanelets", default="")
    parser.add_argument("--signal-id-mode", choices=["relation", "ways", "both"], default="relation")
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    parser.add_argument("--max-lanelets-each-side", type=int, default=4)
    parser.add_argument("--route-strategy", choices=["through", "target_forward"], default="through")
    parser.add_argument("--service-timeout", type=float, default=20.0)
    parser.add_argument("--route-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=20.0)
    parser.add_argument("--planning-observation-timeout", type=float, default=8.0)
    parser.add_argument("--planning-factor-tolerance-m", type=float, default=15.0)
    parser.add_argument("--trajectory-stop-tolerance-m", type=float, default=12.0)
    parser.add_argument("--stop-timeout", type=float, default=60.0)
    parser.add_argument("--release-timeout", type=float, default=30.0)
    parser.add_argument("--green-debug-timeout", type=float, default=3.0)
    parser.add_argument("--green-settle-sec", type=float, default=2.0)
    parser.add_argument("--mode-transition-timeout", type=float, default=5.0)
    parser.add_argument("--clear-attempts", type=int, default=3)
    parser.add_argument("--clear-retry-interval", type=float, default=1.0)
    parser.add_argument("--sample-period", type=float, default=0.1)
    parser.add_argument("--moved-speed-mps", type=float, default=0.5)
    parser.add_argument("--stopped-speed-mps", type=float, default=0.12)
    parser.add_argument("--resume-speed-mps", type=float, default=0.5)
    parser.add_argument("--near-stop-distance-m", type=float, default=12.0)
    parser.add_argument("--approach-detection-distance-m", type=float, default=30.0)
    parser.add_argument("--crossing-tolerance-m", type=float, default=1.0)
    parser.add_argument("--release-crossing-distance-m", type=float, default=5.0)
    parser.add_argument("--stop-hold-sec", type=float, default=1.5)
    parser.add_argument("--goal-threshold-m", type=float, default=5.0)
    parser.add_argument("--require-resume", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    selected_lanelets = parse_ids(args.target_lanelets)
    targets = load_targets(args.fix_csv, selected_lanelets, args.rule_type)
    lanelet_map, graph, lanelets, lanelet_objs = load_lanelets(
        args.map,
        args.origin_lat,
        args.origin_lon,
    )
    missing_lanelets = sorted({target.lanelet_id for target in targets} - set(lanelets))
    targets = [target for target in targets if target.lanelet_id in lanelets]
    traffic_light_regulatory_ids, equivalent_regulatory_ids = load_traffic_light_regulatory_groups(args.map)

    payload: Dict[str, object] = {
        "map": args.map,
        "fix_csv": str(args.fix_csv),
        "rule_type": args.rule_type,
        "target_count": len(targets),
        "missing_lanelets": missing_lanelets,
        "results": [],
        "started_at_unix": time.time(),
    }
    write_json(args.output, payload)
    all_samples: List[Dict[str, object]] = []

    rclpy.init()
    node = RegulatoryStopDriveProbe()
    node.all_traffic_light_regulatory_ids = traffic_light_regulatory_ids
    node.equivalent_traffic_light_regulatory_ids = equivalent_regulatory_ids
    try:
        node.wait_for_services(args.service_timeout)
        for index, target in enumerate(targets, 1):
            result, samples = run_target(
                node,
                target,
                lanelet_map,
                graph,
                lanelets,
                lanelet_objs,
                args,
            )
            payload["results"].append(result)
            all_samples.extend(samples)
            payload["executed_tests"] = len(payload["results"])
            payload["passed_tests"] = sum(1 for item in payload["results"] if item.get("pass"))
            payload["failed_tests"] = sum(1 for item in payload["results"] if not item.get("pass"))
            write_json(args.output, payload)
            write_samples(args.samples_csv, all_samples)
            print(
                f"[{index}/{len(targets)}] lanelet={target.lanelet_id} "
                f"factor={result.get('planned_stop_factor')} zero={result.get('planned_zero_velocity')} "
                f"stopped={result.get('actual_stop')} resumed={result.get('resumed_after_release')} "
                f"pass={result.get('pass')}",
                flush=True,
            )
            if args.stop_on_failure and not result.get("pass"):
                break
    finally:
        node.set_signal_states(
            {
                signal_id: TrafficLightElement.GREEN
                for signal_id in node.all_traffic_light_regulatory_ids
            }
        )
        node.change_to_stop(3.0)
        payload["finished_at_unix"] = time.time()
        write_json(args.output, payload)
        write_samples(args.samples_csv, all_samples)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if payload.get("failed_tests", 0) == 0 and payload.get("executed_tests") == len(targets) else 2


if __name__ == "__main__":
    raise SystemExit(main())
