#!/usr/bin/env python3

import json
import math
import os
import sys
from copy import deepcopy

import rospy
from autoware_internal_planning_msgs.msg import PathWithLaneId, PlanningFactorArray
from autoware_planning_msgs.msg import LaneletRoute, Path, RouteState
from std_msgs.msg import String
from tier4_planning_msgs.msg import StopReasonArray


STOP_REASON_BY_MODULE = {
    "blind_spot": "BlindSpot",
    "crosswalk": "Crosswalk",
    "detection_area": "DetectionArea",
    "intersection": "Intersection",
    "merge_from_private": "MergeFromPrivateRoad",
    "no_drivable_lane": "NoDrivableLane",
    "no_stopping_area": "NoStoppingArea",
    "stop_line": "StopLine",
    "traffic_light": "TrafficLight",
    "virtual_traffic_light": "VirtualTrafficLight",
    "walkway": "Walkway",
}


def read_json(path):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        return json.load(stream)


def distance(first, second):
    return math.sqrt(
        (first.x - second.x) ** 2
        + (first.y - second.y) ** 2
        + (first.z - second.z) ** 2
    )


def finite_path_points(points):
    return all(
        math.isfinite(value)
        for item in points
        for value in (
            item.pose.position.x,
            item.pose.position.y,
            item.pose.position.z,
            item.longitudinal_velocity_mps,
        )
    )


def cumulative_lengths(points):
    values = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        values.append(values[-1] + distance(first.pose.position, second.pose.position))
    return values


def point_segment_distance(point, first, second):
    dx = second.x - first.x
    dy = second.y - first.y
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(point.x - first.x, point.y - first.y)
    ratio = max(
        0.0,
        min(1.0, ((point.x - first.x) * dx + (point.y - first.y) * dy) / length_sq),
    )
    projected_x = first.x + ratio * dx
    projected_y = first.y + ratio * dy
    return math.hypot(point.x - projected_x, point.y - projected_y)


def maximum_geometry_error(source, target):
    if not source or not target:
        return math.inf
    if len(target) == 1:
        return max(distance(item.pose.position, target[0].pose.position) for item in source)
    return max(
        min(
            point_segment_distance(
                item.pose.position, first.pose.position, second.pose.position
            )
            for first, second in zip(target[:-1], target[1:])
        )
        for item in source
    )


def unique_lane_sequence(path):
    result = []
    for item in path.points:
        for lane_id in item.lane_ids:
            lane_id = int(lane_id)
            if not result or result[-1] != lane_id:
                result.append(lane_id)
    return result


def json_position_distance(first, second):
    return math.sqrt(
        (float(first["x"]) - float(second["x"])) ** 2
        + (float(first["y"]) - float(second["y"])) ** 2
        + (float(first.get("z", 0.0)) - float(second.get("z", 0.0))) ** 2
    )


class MissionBppBvpPipelineValidator:
    def __init__(self):
        self.reference_path = rospy.get_param("~reference_json")
        self.reference = read_json(self.reference_path)
        self.expected_module = str(rospy.get_param("~expected_module", "stop_line"))
        self.timeout = float(rospy.get_param("~timeout", 30.0))
        self.stop_pose_tolerance = float(
            rospy.get_param("~stop_pose_tolerance", 1.5)
        )
        self.stop_station_tolerance = float(
            rospy.get_param("~stop_station_tolerance", 2.0)
        )
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 1.0))
        self.geometry_tolerance = float(
            rospy.get_param("~geometry_tolerance", 0.25)
        )
        self.shutdown_on_result = bool(
            rospy.get_param("~shutdown_on_result", True)
        )
        self.report_path = os.path.expanduser(
            rospy.get_param("~report_json", "/tmp/mission_bpp_bvp.report.json")
        )
        self.started_at = rospy.Time.now()
        self.route = None
        self.route_state = None
        self.bpp_path = None
        self.bvp_path = None
        self.status = None
        self.factors = None
        self.stop_reasons = None
        self.done = False
        self.exit_code = 2

        self.result_pub = rospy.Publisher(
            "/validation/mission_bpp_bvp/result", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/planning/mission_planning/route",
            LaneletRoute,
            self.on_route,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/mission_planning/state",
            RouteState,
            self.on_route_state,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
            PathWithLaneId,
            self.on_bpp_path,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path",
            Path,
            self.on_bvp_path,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/debug/behavior_velocity_planner/status",
            String,
            self.on_status,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/planning_factors/behavior_velocity_planner",
            PlanningFactorArray,
            self.on_factors,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/status/stop_reasons",
            StopReasonArray,
            self.on_stop_reasons,
            queue_size=1,
        )
        rospy.Timer(rospy.Duration(0.1), self.on_timer)

    def on_route(self, message):
        self.route = deepcopy(message)

    def on_route_state(self, message):
        self.route_state = deepcopy(message)

    def on_bpp_path(self, message):
        if message.points:
            self.bpp_path = deepcopy(message)

    def on_bvp_path(self, message):
        if message.points:
            self.bvp_path = deepcopy(message)

    def on_status(self, message):
        self.status = str(message.data)

    def on_factors(self, message):
        self.factors = deepcopy(message)

    def on_stop_reasons(self, message):
        self.stop_reasons = deepcopy(message)

    def ready(self):
        return all(
            item is not None
            for item in (
                self.route,
                self.route_state,
                self.bpp_path,
                self.bvp_path,
                self.status,
                self.factors,
                self.stop_reasons,
            )
        )

    def expected_route_sequence(self):
        return [
            int(segment["preferred_primitive"]["id"])
            for segment in self.reference["scenario"]["route"]["segments"]
        ]

    def live_route_sequence(self):
        return [int(item.preferred_primitive.id) for item in self.route.segments]

    def reference_stop(self):
        summary = self.reference.get("summary", {})
        pose_value = summary.get("control_point_pose")
        station = summary.get("control_point_station_m")
        if pose_value is None:
            station = summary.get("first_zero_station_m")
            index = summary.get("first_zero_index")
            points = self.reference.get("output_path", {}).get("points", [])
            if index is not None and 0 <= int(index) < len(points):
                pose_value = points[int(index)]["pose"]
        if pose_value is None or station is None:
            return None, None
        position = pose_value["position"]
        return (
            (float(position["x"]), float(position["y"]), float(position.get("z", 0.0))),
            float(station),
        )

    def live_stop(self, expected_position):
        goal = self.bvp_path.points[-1].pose.position
        lengths = cumulative_lengths(self.bvp_path.points)
        candidates = []
        for index, item in enumerate(self.bvp_path.points):
            if float(item.longitudinal_velocity_mps) > 0.01:
                continue
            if distance(item.pose.position, goal) <= 2.0:
                continue
            pose = item.pose.position
            error = math.sqrt(
                (pose.x - expected_position[0]) ** 2
                + (pose.y - expected_position[1]) ** 2
                + (pose.z - expected_position[2]) ** 2
            )
            candidates.append((error, lengths[index], index, pose))
        return min(candidates, key=lambda value: value[0]) if candidates else None

    def validate(self):
        checks = []

        def check(name, passed, detail):
            checks.append({"name": name, "passed": bool(passed), "detail": detail})

        check(
            "route_state",
            self.route_state.state == RouteState.SET,
            "state={}".format(self.route_state.state),
        )
        expected_route = self.expected_route_sequence()
        live_route = self.live_route_sequence()
        check(
            "route_sequence",
            live_route == expected_route,
            "reference={} live={}".format(expected_route, live_route),
        )

        bpp_points = self.bpp_path.points
        bvp_points = self.bvp_path.points
        check("bpp_path", len(bpp_points) >= 2, "points={}".format(len(bpp_points)))
        check("bvp_path", len(bvp_points) >= 2, "points={}".format(len(bvp_points)))
        check("bpp_finite", finite_path_points([item.point for item in bpp_points]), "finite")
        check("bvp_finite", finite_path_points(bvp_points), "finite")

        goal_value = self.reference["scenario"]["route"]["goal_pose"]["position"]
        endpoint = bvp_points[-1].pose.position
        goal_error = math.sqrt(
            (endpoint.x - float(goal_value["x"])) ** 2
            + (endpoint.y - float(goal_value["y"])) ** 2
        )
        reference_points = self.reference.get("output_path", {}).get("points", [])
        reference_goal_error = math.inf
        if reference_points:
            reference_goal_error = json_position_distance(
                reference_points[-1]["pose"]["position"], goal_value
            )
        reference_reaches_goal = reference_goal_error <= self.goal_tolerance
        if reference_reaches_goal:
            check(
                "goal_reached",
                goal_error <= self.goal_tolerance,
                "reference_error={:.3f} m live_error={:.3f} m".format(
                    reference_goal_error, goal_error
                ),
            )
        else:
            check(
                "local_path_horizon",
                True,
                "ROS2 reference ends {:.3f} m before goal; live goal error={:.3f} m"
                .format(reference_goal_error, goal_error),
            )

        bpp_as_path = [item.point for item in bpp_points]
        geometry_error = max(
            maximum_geometry_error(bpp_as_path, bvp_points),
            maximum_geometry_error(bvp_points, bpp_as_path),
        )
        check(
            "bpp_bvp_geometry",
            geometry_error <= self.geometry_tolerance,
            "max_error={:.3f} m".format(geometry_error),
        )

        factor_modules = [str(item.module) for item in self.factors.factors]
        check(
            "planning_factor",
            self.expected_module in factor_modules,
            "expected={} active={}".format(self.expected_module, factor_modules),
        )
        check(
            "module_status",
            self.expected_module in self.status,
            self.status,
        )
        expected_reason = STOP_REASON_BY_MODULE.get(self.expected_module)
        reasons = [str(item.reason) for item in self.stop_reasons.stop_reasons]
        check(
            "stop_reason",
            expected_reason is None or expected_reason in reasons,
            "expected={} active={}".format(expected_reason, reasons),
        )

        expected_stop, expected_station = self.reference_stop()
        live_stop = self.live_stop(expected_stop) if expected_stop is not None else None
        if live_stop is None:
            stop_pose_error = math.inf
            live_station = None
            station_error = math.inf
        else:
            stop_pose_error, live_station, _index, _pose = live_stop
            station_error = abs(live_station - expected_station)
        check(
            "stop_pose",
            stop_pose_error <= self.stop_pose_tolerance,
            "error={:.3f} m".format(stop_pose_error),
        )
        check(
            "stop_station",
            station_error <= self.stop_station_tolerance,
            "reference={:.3f} m live={} error={:.3f} m".format(
                expected_station,
                "{:.3f} m".format(live_station) if live_station is not None else "none",
                station_error,
            ),
        )

        report = {
            "format": "mission_bpp_bvp_integration_report",
            "scenario": self.reference["scenario"].get("name", "unnamed"),
            "reference": self.reference_path,
            "expected_module": self.expected_module,
            "passed": all(item["passed"] for item in checks),
            "checks": checks,
            "diagnostics": {
                "expected_route_sequence": expected_route,
                "live_route_sequence": live_route,
                "bpp_lane_sequence": unique_lane_sequence(self.bpp_path),
                "bpp_point_count": len(bpp_points),
                "bvp_point_count": len(bvp_points),
                "goal_error_m": goal_error,
                "reference_goal_error_m": reference_goal_error,
                "reference_reaches_goal": reference_reaches_goal,
                "bpp_bvp_geometry_max_error_m": geometry_error,
                "reference_stop_station_m": expected_station,
                "live_stop_station_m": live_station,
                "stop_station_error_m": station_error,
                "stop_pose_error_m": stop_pose_error,
                "planning_factor_modules": factor_modules,
                "stop_reasons": reasons,
                "module_status": self.status,
            },
            "tolerances": {
                "goal_m": self.goal_tolerance,
                "geometry_m": self.geometry_tolerance,
                "stop_pose_m": self.stop_pose_tolerance,
                "stop_station_m": self.stop_station_tolerance,
            },
        }
        return report

    def finish(self, report):
        self.done = True
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.report_path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")

        self.exit_code = 0 if report["passed"] else 1
        verdict = "PASS" if report["passed"] else "FAIL"
        summary = "MISSION+BPP+BVP INTEGRATION: {} | {} | report={}".format(
            verdict, report["scenario"], self.report_path
        )
        self.result_pub.publish(String(data=summary))
        log = rospy.loginfo if report["passed"] else rospy.logerr
        log(summary)
        for item in report["checks"]:
            log("[%s] %s: %s", "PASS" if item["passed"] else "FAIL", item["name"], item["detail"])
        if self.shutdown_on_result:
            rospy.signal_shutdown("integration validation complete")

    def on_timer(self, _event):
        if self.done:
            return
        if self.ready():
            report = self.validate()
            factor_modules = report["diagnostics"]["planning_factor_modules"]
            if self.expected_module in factor_modules or (
                rospy.Time.now() - self.started_at
            ).to_sec() >= self.timeout:
                self.finish(report)
                return
        if (rospy.Time.now() - self.started_at).to_sec() >= self.timeout:
            if self.ready():
                self.finish(self.validate())
            else:
                missing = [
                    name
                    for name, value in (
                        ("route", self.route),
                        ("route_state", self.route_state),
                        ("bpp_path", self.bpp_path),
                        ("bvp_path", self.bvp_path),
                        ("status", self.status),
                        ("planning_factors", self.factors),
                        ("stop_reasons", self.stop_reasons),
                    )
                    if value is None
                ]
                self.finish(
                    {
                        "format": "mission_bpp_bvp_integration_report",
                        "scenario": self.reference["scenario"].get("name", "unnamed"),
                        "reference": self.reference_path,
                        "expected_module": self.expected_module,
                        "passed": False,
                        "checks": [
                            {
                                "name": "pipeline_outputs",
                                "passed": False,
                                "detail": "timed out waiting for {}".format(missing),
                            }
                        ],
                        "diagnostics": {"missing": missing},
                    }
                )


def main():
    rospy.init_node("mission_bpp_bvp_pipeline_validator")
    validator = MissionBppBvpPipelineValidator()
    rospy.spin()
    sys.exit(validator.exit_code)


if __name__ == "__main__":
    main()
