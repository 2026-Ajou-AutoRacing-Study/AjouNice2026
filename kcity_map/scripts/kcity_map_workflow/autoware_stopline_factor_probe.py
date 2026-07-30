#!/usr/bin/env python3
"""Probe Autoware stop-line behavior factors for verified K-City stop signs.

Run this inside the Autoware ROS 2 container after sourcing ROS and Autoware.
The input fix CSV is produced by fix_v41_stop_lines_to_v42.py.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set

import lanelet2
import rclpy
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from visualization_msgs.msg import MarkerArray

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from autoware_route_probe import (  # noqa: E402
    RouteProbe,
    RouteSpec,
    build_forward_path,
    coverage_route_for_target,
    following_ids,
    load_lanelets,
    parse_id_set,
    previous_ids,
    route_from_path,
    write_json,
)


@dataclass(frozen=True)
class StopLineTarget:
    lanelet_id: int
    regulatory_element_id: int
    stop_line_id: int
    traffic_sign_way_id: int
    rule_type: str
    gis_sign_id: str
    gis_sign_remark: str
    stop_sign_distance_m: float


def parse_float(value: str, default: float = float("nan")) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load_targets(path: Path, selected_lanelets: Set[int], selected_stop_lines: Set[int]) -> List[StopLineTarget]:
    targets: List[StopLineTarget] = []
    with path.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            stop_line_id = int(row["stop_line_id"])
            if selected_stop_lines and stop_line_id not in selected_stop_lines:
                continue
            for lanelet_text in row["lanelet_id"].split():
                lanelet_id = int(lanelet_text)
                if selected_lanelets and lanelet_id not in selected_lanelets:
                    continue
                targets.append(
                    StopLineTarget(
                        lanelet_id=lanelet_id,
                        regulatory_element_id=int(row["regulatory_element_id"]),
                        stop_line_id=stop_line_id,
                        traffic_sign_way_id=int(row["traffic_sign_way_id"]),
                        rule_type=row["rule_type"],
                        gis_sign_id=row["gis_sign_id"],
                        gis_sign_remark=row["gis_sign_remark"],
                        stop_sign_distance_m=parse_float(row["stop_sign_distance_m"]),
                    )
                )
    return targets


def distance_xy(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def midpoint(points: Sequence[Sequence[float]]) -> tuple:
    if not points:
        return (float("nan"), float("nan"), float("nan"))
    if len(points) == 1:
        return tuple(points[0])
    segment_lengths = [distance_xy(a, b) for a, b in zip(points, points[1:])]
    total = sum(segment_lengths)
    if total <= 1e-9:
        return tuple(points[0])
    target = total * 0.5
    walked = 0.0
    for index, length in enumerate(segment_lengths):
        if walked + length >= target or index == len(segment_lengths) - 1:
            ratio = 0.0 if length <= 1e-9 else (target - walked) / length
            a = points[index]
            b = points[index + 1]
            return (
                float(a[0]) + (float(b[0]) - float(a[0])) * ratio,
                float(a[1]) + (float(b[1]) - float(a[1])) * ratio,
                float(a[2]) + (float(b[2]) - float(a[2])) * ratio,
            )
        walked += length
    return tuple(points[-1])


def stop_line_midpoints(lanelet_map, stop_line_ids: Set[int]) -> Dict[int, tuple]:
    midpoints = {}
    for stop_line_id in stop_line_ids:
        line = lanelet_map.lineStringLayer.get(stop_line_id)
        points = [(float(point.x), float(point.y), float(point.z)) for point in line]
        midpoints[stop_line_id] = midpoint(points)
    return midpoints


def planning_factor_distances(msg: Optional[PlanningFactorArray], expected: tuple) -> List[float]:
    if msg is None:
        return []
    distances = []
    for factor in msg.factors:
        for control_point in factor.control_points:
            pos = control_point.pose.position
            distances.append(distance_xy((pos.x, pos.y), expected))
    return distances


def stop_planning_factor_distances(msg: Optional[PlanningFactorArray], expected: tuple) -> List[float]:
    if msg is None:
        return []
    distances = []
    for factor in msg.factors:
        if int(factor.behavior) != 3:
            continue
        for control_point in factor.control_points:
            pos = control_point.pose.position
            distances.append(distance_xy((pos.x, pos.y), expected))
    return distances


def marker_distances(msg: Optional[MarkerArray], expected: tuple) -> List[float]:
    if msg is None:
        return []
    distances = []
    for marker in msg.markers:
        pos = marker.pose.position
        distances.append(distance_xy((pos.x, pos.y), expected))
        for point in marker.points:
            distances.append(distance_xy((point.x, point.y), expected))
    return distances


class StopLineFactorProbe(RouteProbe):
    def __init__(self) -> None:
        super().__init__()
        self.factor_sub = self.create_subscription(
            PlanningFactorArray,
            "/planning/planning_factors/stop_line",
            self._factor_cb,
            10,
        )
        self.wall_sub = self.create_subscription(
            MarkerArray,
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/virtual_wall/stop_line",
            self._wall_cb,
            10,
        )
        self.latest_factor: Optional[PlanningFactorArray] = None
        self.latest_wall: Optional[MarkerArray] = None
        self.factor_samples = 0
        self.wall_samples = 0

    def _factor_cb(self, msg: PlanningFactorArray) -> None:
        self.latest_factor = msg
        self.factor_samples += 1

    def _wall_cb(self, msg: MarkerArray) -> None:
        self.latest_wall = msg
        self.wall_samples += 1

    def reset_stopline_observations(self) -> None:
        self.latest_factor = None
        self.latest_wall = None
        self.factor_samples = 0
        self.wall_samples = 0


def bool_from_expectation(value: str) -> Optional[bool]:
    if value == "present":
        return True
    if value == "absent":
        return False
    return None


def expectation_pass(expectation: Optional[bool], observed: bool) -> bool:
    if expectation is None:
        return True
    return observed is expectation


def run_target(
    node: StopLineFactorProbe,
    route_index: int,
    spec: RouteSpec,
    target: StopLineTarget,
    expected_midpoint: tuple,
    args,
) -> Dict[str, object]:
    node.reset_stopline_observations()
    result = node.run_route(route_index, spec, args)
    node.spin_until(
        lambda: node.factor_samples >= args.min_factor_samples and node.wall_samples >= args.min_wall_samples,
        args.factor_timeout,
    )

    all_factor_distances = planning_factor_distances(node.latest_factor, expected_midpoint)
    stop_factor_distances = stop_planning_factor_distances(node.latest_factor, expected_midpoint)
    wall_distances = marker_distances(node.latest_wall, expected_midpoint)
    min_all_factor = min(all_factor_distances) if all_factor_distances else None
    min_stop_factor = min(stop_factor_distances) if stop_factor_distances else None
    min_wall = min(wall_distances) if wall_distances else None
    near_stop_factor = min_stop_factor is not None and min_stop_factor <= args.near_tolerance
    near_wall = min_wall is not None and min_wall <= args.near_tolerance
    expected_factor = bool_from_expectation(args.expect_target_factor)

    factor_details = []
    if node.latest_factor is not None:
        for factor in node.latest_factor.factors:
            factor_details.append(
                {
                    "module": factor.module,
                    "behavior": int(factor.behavior),
                    "detail": factor.detail,
                    "control_point_count": len(factor.control_points),
                }
            )

    passed = (
        bool(result["pass"])
        and expectation_pass(expected_factor, near_stop_factor)
        and (not args.require_target_wall or near_wall)
    )
    return {
        **result,
        "lanelet_id": target.lanelet_id,
        "regulatory_element_id": target.regulatory_element_id,
        "stop_line_id": target.stop_line_id,
        "traffic_sign_way_id": target.traffic_sign_way_id,
        "rule_type": target.rule_type,
        "gis_sign_id": target.gis_sign_id,
        "gis_sign_remark": target.gis_sign_remark,
        "gis_stop_sign_distance_m": target.stop_sign_distance_m,
        "expected_stop_line_midpoint": {
            "x": expected_midpoint[0],
            "y": expected_midpoint[1],
            "z": expected_midpoint[2],
        },
        "factor_samples": node.factor_samples,
        "wall_samples": node.wall_samples,
        "factor_count": len(node.latest_factor.factors) if node.latest_factor is not None else 0,
        "stop_factor_count": sum(1 for factor in node.latest_factor.factors if int(factor.behavior) == 3)
        if node.latest_factor is not None
        else 0,
        "wall_marker_count": len(node.latest_wall.markers) if node.latest_wall is not None else 0,
        "min_any_factor_distance_m": round(min_all_factor, 3) if min_all_factor is not None else None,
        "min_stop_factor_distance_m": round(min_stop_factor, 3) if min_stop_factor is not None else None,
        "min_wall_distance_m": round(min_wall, 3) if min_wall is not None else None,
        "near_target_stop_factor": near_stop_factor,
        "near_target_wall": near_wall,
        "expected_target_factor": args.expect_target_factor,
        "factor_details": factor_details,
        "stopline_probe_pass": passed,
    }


def approach_specs_for_target(
    target: StopLineTarget,
    graph,
    lanelets,
    lanelet_objs,
    pending: Set[int],
    max_route_lanelets: int,
    route_index: int,
) -> List[RouteSpec]:
    specs: List[RouteSpec] = []
    target_id = target.lanelet_id
    forward = build_forward_path(target_id, graph, lanelets, lanelet_objs, max_route_lanelets, pending)
    if len(forward) >= 2:
        specs.append(
            route_from_path(
                f"approach_{route_index:04d}_target_forward",
                forward,
                lanelets,
                required_lanelet=target_id,
                waypoint_lanelets=forward[1:-1],
            )
        )

    for previous_id in previous_ids(graph, lanelet_objs, target_id):
        if previous_id not in lanelets:
            continue
        tail_budget = max(1, max_route_lanelets - 1)
        tail = build_forward_path(target_id, graph, lanelets, lanelet_objs, tail_budget, pending)
        path = [previous_id] + tail
        if len(path) < 2:
            continue
        specs.append(
            route_from_path(
                f"approach_{route_index:04d}_prev_{previous_id}",
                path,
                lanelets,
                required_lanelet=target_id,
                waypoint_lanelets=path[1:-1],
            )
        )

    if not specs:
        specs.append(route_from_path(f"approach_{route_index:04d}_single", [target_id], lanelets, required_lanelet=target_id))
    return specs


def run_target_with_attempts(
    node: StopLineFactorProbe,
    route_index: int,
    specs: Sequence[RouteSpec],
    target: StopLineTarget,
    expected_midpoint: tuple,
    args,
) -> Dict[str, object]:
    attempts = []
    selected: Optional[Dict[str, object]] = None
    for attempt_index, spec in enumerate(specs, 1):
        result = run_target(node, route_index * 100 + attempt_index, spec, target, expected_midpoint, args)
        attempts.append(
            {
                "attempt_index": attempt_index,
                "name": spec.name,
                "start_lanelet": spec.start_lanelet,
                "goal_lanelet": spec.goal_lanelet,
                "intended_lanelets": spec.intended_lanelets,
                "route_ids": result["route_ids"],
                "route_pass": result["pass"],
                "near_target_stop_factor": result["near_target_stop_factor"],
                "min_stop_factor_distance_m": result["min_stop_factor_distance_m"],
                "factor_count": result["factor_count"],
                "stopline_probe_pass": result["stopline_probe_pass"],
            }
        )
        selected = result
        if result["stopline_probe_pass"]:
            break
    assert selected is not None
    selected["attempts"] = attempts
    selected["attempt_count"] = len(attempts)
    return selected


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    parser.add_argument("--fix-csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--target-lanelets", default="")
    parser.add_argument("--target-stop-lines", default="")
    parser.add_argument("--max-route-lanelets", type=int, default=8)
    parser.add_argument("--coverage-strategy", choices=["path", "micro", "through", "approach"], default="through")
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--route-timeout", type=float, default=10.0)
    parser.add_argument("--trajectory-timeout", type=float, default=10.0)
    parser.add_argument("--factor-timeout", type=float, default=4.0)
    parser.add_argument("--min-factor-samples", type=int, default=1)
    parser.add_argument("--min-wall-samples", type=int, default=0)
    parser.add_argument("--near-tolerance", type=float, default=15.0)
    parser.add_argument("--expect-target-factor", choices=["present", "absent", "any"], default="present")
    parser.add_argument("--require-target-wall", action="store_true")
    parser.add_argument("--require-trajectory", action="store_true", default=True)
    parser.add_argument("--no-require-trajectory", dest="require_trajectory", action="store_false")
    parser.add_argument("--stop-on-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    selected_lanelets = parse_id_set(args.target_lanelets)
    selected_stop_lines = parse_id_set(args.target_stop_lines)
    targets = load_targets(Path(args.fix_csv), selected_lanelets, selected_stop_lines)
    lanelet_map, graph, lanelets, lanelet_objs = load_lanelets(args.map, args.origin_lat, args.origin_lon)
    stop_midpoints = stop_line_midpoints(lanelet_map, {target.stop_line_id for target in targets})

    missing_lanelets = sorted({target.lanelet_id for target in targets} - set(lanelets))
    targets = [target for target in targets if target.lanelet_id in lanelets]
    payload: Dict[str, object] = {
        "summary": {
            "map": args.map,
            "fix_csv": args.fix_csv,
            "target_count": len(targets),
            "missing_lanelets": missing_lanelets,
            "expect_target_factor": args.expect_target_factor,
            "near_tolerance_m": args.near_tolerance,
            "started_at_unix": time.time(),
        },
        "results": [],
    }
    write_json(Path(args.output), payload)

    rclpy.init()
    node = StopLineFactorProbe()
    try:
        node.wait_for_services(args.service_timeout)
        pending = {target.lanelet_id for target in targets}
        results = []
        for route_index, target in enumerate(targets, 1):
            if args.coverage_strategy == "approach":
                specs = approach_specs_for_target(
                    target,
                    graph,
                    lanelets,
                    lanelet_objs,
                    pending,
                    args.max_route_lanelets,
                    route_index,
                )
                result = run_target_with_attempts(
                    node, route_index, specs, target, stop_midpoints[target.stop_line_id], args
                )
            else:
                spec = coverage_route_for_target(
                    target.lanelet_id,
                    graph,
                    lanelets,
                    lanelet_objs,
                    pending,
                    args.max_route_lanelets,
                    route_index,
                    set(),
                    set(),
                    args.coverage_strategy,
                )
                result = run_target(node, route_index, spec, target, stop_midpoints[target.stop_line_id], args)
            results.append(result)
            payload["results"] = results
            payload["summary"].update(
                {
                    "executed_tests": len(results),
                    "passed_tests": sum(1 for item in results if item["stopline_probe_pass"]),
                    "failed_tests": sum(1 for item in results if not item["stopline_probe_pass"]),
                    "near_target_stop_factor_tests": sum(1 for item in results if item["near_target_stop_factor"]),
                    "near_target_wall_tests": sum(1 for item in results if item["near_target_wall"]),
                }
            )
            write_json(Path(args.output), payload)
            print(
                f"[{route_index}/{len(targets)}] lanelet={target.lanelet_id} "
                f"stop_line={target.stop_line_id} route_pass={result['pass']} "
                f"near_factor={result['near_target_stop_factor']} "
                f"min_factor={result['min_stop_factor_distance_m']} "
                f"probe_pass={result['stopline_probe_pass']}",
                flush=True,
            )
            pending.discard(target.lanelet_id)
            if args.stop_on_failure and not result["stopline_probe_pass"]:
                break
    finally:
        payload["summary"]["finished_at_unix"] = time.time()
        write_json(Path(args.output), payload)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
