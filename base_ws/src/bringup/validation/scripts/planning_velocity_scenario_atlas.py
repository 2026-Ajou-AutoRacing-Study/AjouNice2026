#!/usr/bin/env python3

import bisect
import csv
import json
import math
import os
import sys
from datetime import datetime, timezone

import rospy
import roslib.packages
import rospkg
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


NORMAL_COLOR = (0.12, 0.86, 0.30, 1.0)
BVP_COLOR = (1.0, 0.34, 0.02, 1.0)
MVP_COLOR = (1.0, 0.86, 0.02, 1.0)
STOP_COLOR = (1.0, 0.03, 0.03, 1.0)
BASE_COLOR = (0.35, 0.38, 0.43, 0.85)
TEXT_COLOR = (0.94, 0.96, 1.0, 1.0)


def read_json(path):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        return json.load(stream)


def rgba(values):
    return ColorRGBA(
        r=float(values[0]),
        g=float(values[1]),
        b=float(values[2]),
        a=float(values[3]),
    )


def point_position(value):
    return value["pose"]["position"]


def cumulative_stations(points):
    stations = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        first_position = point_position(first)
        second_position = point_position(second)
        stations.append(
            stations[-1]
            + math.hypot(
                float(second_position["x"]) - float(first_position["x"]),
                float(second_position["y"]) - float(first_position["y"]),
            )
        )
    return stations


def interpolate(stations, values, query):
    if query <= stations[0]:
        return float(values[0])
    if query >= stations[-1]:
        return float(values[-1])
    upper = bisect.bisect_right(stations, query)
    lower = upper - 1
    span = stations[upper] - stations[lower]
    ratio = 0.0 if span <= 1.0e-9 else (query - stations[lower]) / span
    return float(values[lower]) + ratio * (float(values[upper]) - float(values[lower]))


def clip_after_effect_stop(stations, baselines, outputs, post_stop_length):
    stop_station = next(
        (
            station
            for station, baseline, output in zip(stations, baselines, outputs)
            if baseline > 0.1 and output <= 0.05
        ),
        None,
    )
    if stop_station is None:
        return len(stations), None
    cutoff = stop_station + post_stop_length
    count = bisect.bisect_right(stations, cutoff)
    return max(count, 2), stop_station


def bvp_profile(path, title, module, reason, post_stop_length):
    document = read_json(path)
    input_points = document["input_path"]["points"]
    output_points = document["output_path"]["points"]
    input_stations = cumulative_stations(input_points)
    output_stations = cumulative_stations(output_points)
    input_velocities = [
        float(point["longitudinal_velocity_mps"]) for point in input_points
    ]
    output_velocities = [
        float(point["longitudinal_velocity_mps"]) for point in output_points
    ]
    baselines = [
        interpolate(input_stations, input_velocities, station)
        for station in output_stations
    ]
    count, stop_station = clip_after_effect_stop(
        output_stations, baselines, output_velocities, post_stop_length
    )
    return {
        "title": title,
        "planner": "BVP",
        "module": module,
        "reason": reason,
        "stations": output_stations[:count],
        "baselines": baselines[:count],
        "outputs": output_velocities[:count],
        "stop_station": stop_station,
        "source": os.path.abspath(os.path.expanduser(path)),
    }


def load_mvp_profiles(path):
    profiles = {}
    with open(os.path.expanduser(path), "r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            profiles.setdefault(row["case"], []).append(row)
    return profiles


def mvp_profile(rows, title, module, reason, post_stop_length):
    stations = [float(row["station_m"]) for row in rows]
    outputs = [float(row["longitudinal_velocity_mps"]) for row in rows]
    baselines = [5.0 for _ in rows]
    count, stop_station = clip_after_effect_stop(
        stations, baselines, outputs, post_stop_length
    )
    return {
        "title": title,
        "planner": "MVP",
        "module": module,
        "reason": reason,
        "stations": stations[:count],
        "baselines": baselines[:count],
        "outputs": outputs[:count],
        "stop_station": stop_station,
        "source": "motion_velocity_planner validation profile",
    }


def load_map_route(map_file, lane_ids, projector_type):
    package_dir = rospkg.RosPack().get_path("behavior_path_planner")
    source_helper = os.path.join(package_dir, "scripts", "bpp_map_tools.py")
    if os.path.isfile(source_helper):
        helper_dir = os.path.dirname(source_helper)
    else:
        helper_paths = roslib.packages.find_node(
            "behavior_path_planner", "bpp_map_tools.py"
        )
        if not helper_paths:
            raise RuntimeError("behavior_path_planner/bpp_map_tools.py was not found")
        helper_dir = os.path.dirname(helper_paths[0])
    if helper_dir not in sys.path:
        sys.path.insert(0, helper_dir)

    from bpp_map_tools import CenterlineMap, stitch_route_centerlines

    centerline_map = CenterlineMap.load(
        os.path.expanduser(map_file), "", False, projector_type
    )
    points, used_lane_ids = stitch_route_centerlines(centerline_map, lane_ids)
    if len(points) < 2:
        raise RuntimeError(
            "failed to build the K-City route from lanelets {}".format(lane_ids)
        )
    return points, used_lane_ids


def polyline_stations(points):
    stations = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        stations.append(
            stations[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return stations


def interpolate_xyz(stations, points, query):
    if query <= stations[0]:
        return points[0]
    if query >= stations[-1]:
        return points[-1]
    upper = bisect.bisect_right(stations, query)
    lower = upper - 1
    span = stations[upper] - stations[lower]
    ratio = 0.0 if span <= 1.0e-9 else (query - stations[lower]) / span
    return tuple(
        float(points[lower][axis])
        + ratio * (float(points[upper][axis]) - float(points[lower][axis]))
        for axis in range(3)
    )


class PlanningVelocityScenarioAtlas:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.gap = max(float(rospy.get_param("~scenario_gap_m", 25.0)), 5.0)
        self.post_stop_length = max(
            float(rospy.get_param("~post_stop_length_m", 18.0)), 5.0
        )
        self.velocity_epsilon = max(
            float(rospy.get_param("~velocity_change_epsilon_mps", 0.1)), 0.0
        )
        self.output_dir = os.path.expanduser(
            rospy.get_param(
                "~output_dir",
                "~/AJOUNICE2026/local_autoware_planning_sim/results/"
                "velocity_scenario_atlas",
            )
        )
        self.map_file = os.path.expanduser(rospy.get_param("~map_file"))
        self.projector_type = rospy.get_param("~projector_type", "MGRS")
        self.route_lane_ids = [
            int(lane_id) for lane_id in rospy.get_param("~route_lane_ids")
        ]
        self.route_start_station = max(
            float(rospy.get_param("~route_start_station_m", 0.0)), 0.0
        )
        self.route_points, self.used_route_lane_ids = load_map_route(
            self.map_file, self.route_lane_ids, self.projector_type
        )
        self.route_stations = polyline_stations(self.route_points)
        mvp_profiles_path = rospy.get_param("~mvp_profiles_csv")
        mvp_profiles = load_mvp_profiles(mvp_profiles_path)
        self.scenarios = [
            bvp_profile(
                rospy.get_param("~speed_bump_reference"),
                "1. SPEED BUMP",
                "speed_bump",
                "map speed bump regulation",
                self.post_stop_length,
            ),
            bvp_profile(
                rospy.get_param("~crosswalk_reference"),
                "2. CROSSWALK",
                "crosswalk",
                "pedestrian crossing",
                self.post_stop_length,
            ),
            bvp_profile(
                rospy.get_param("~traffic_light_reference"),
                "3. TRAFFIC LIGHT",
                "traffic_light",
                "red signal",
                self.post_stop_length,
            ),
            bvp_profile(
                rospy.get_param("~intersection_reference"),
                "4. INTERSECTION",
                "intersection",
                "collision object",
                self.post_stop_length,
            ),
            mvp_profile(
                mvp_profiles["static_object_30m"],
                "5. STATIC OBJECT",
                "obstacle_stop",
                "stopped vehicle 30 m ahead",
                self.post_stop_length,
            ),
            mvp_profile(
                mvp_profiles["slow_dynamic_object_30m_0p5mps"],
                "6. SLOW OBJECT",
                "dynamic_obstacle_stop",
                "vehicle at 0.5 m/s",
                self.post_stop_length,
            ),
            mvp_profile(
                mvp_profiles["dynamic_object_30m_2mps"],
                "7. CRUISE FOLLOW",
                "obstacle_cruise",
                "vehicle at 2.0 m/s",
                self.post_stop_length,
            ),
        ]
        self.publisher = rospy.Publisher(
            "/validation/planning_velocity_scenario_atlas/markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.trajectory_topic = rospy.get_param(
            "~trajectory_topic",
            "/validation/planning_velocity_scenario_atlas/trajectory",
        )
        self.trajectory_publisher = rospy.Publisher(
            self.trajectory_topic,
            Trajectory,
            queue_size=1,
            latch=True,
        )
        self.rows, self.summaries = self.compose_atlas()
        self.place_atlas_on_map()
        self.write_results()
        rospy.sleep(0.3)
        self.publisher.publish(self.make_markers())
        self.trajectory_publisher.publish(self.make_trajectory())
        rospy.loginfo(
            "planning velocity scenario atlas ready: scenarios=%d points=%d "
            "trajectory=%s output=%s",
            len(self.scenarios),
            len(self.rows),
            self.trajectory_topic,
            self.output_dir,
        )

    def point_color(self, planner, baseline, output):
        if baseline > 0.1 and output <= 0.05:
            return "stop", STOP_COLOR
        if output < baseline - self.velocity_epsilon:
            return ("bvp", BVP_COLOR) if planner == "BVP" else ("mvp", MVP_COLOR)
        return "normal", NORMAL_COLOR

    def compose_atlas(self):
        rows = []
        summaries = []
        cursor = 0.0
        global_index = 0
        for scenario_index, scenario in enumerate(self.scenarios, start=1):
            local_origin = scenario["stations"][0]
            scenario_start = cursor
            colors = []
            for local_index, (station, baseline, output) in enumerate(
                zip(
                    scenario["stations"],
                    scenario["baselines"],
                    scenario["outputs"],
                )
            ):
                local_station = station - local_origin
                atlas_station = scenario_start + local_station
                color_name, _ = self.point_color(
                    scenario["planner"], baseline, output
                )
                colors.append(color_name)
                rows.append(
                    {
                        "global_point_index": global_index,
                        "scenario_index": scenario_index,
                        "scenario": scenario["title"],
                        "planner": scenario["planner"],
                        "module": scenario["module"],
                        "reason": scenario["reason"],
                        "local_station_m": round(local_station, 6),
                        "atlas_station_m": round(atlas_station, 6),
                        "baseline_velocity_mps": round(float(baseline), 6),
                        "output_velocity_mps": round(float(output), 6),
                        "velocity_reduction_mps": round(
                            max(float(baseline) - float(output), 0.0), 6
                        ),
                        "color_reason": color_name,
                    }
                )
                global_index += 1
            scenario_end = rows[-1]["atlas_station_m"]
            modified_outputs = [
                float(output)
                for baseline, output in zip(
                    scenario["baselines"], scenario["outputs"]
                )
                if output < baseline - self.velocity_epsilon
            ]
            summaries.append(
                {
                    "scenario_index": scenario_index,
                    "scenario": scenario["title"],
                    "planner": scenario["planner"],
                    "module": scenario["module"],
                    "reason": scenario["reason"],
                    "start_station_m": scenario_start,
                    "end_station_m": scenario_end,
                    "baseline_max_velocity_mps": max(scenario["baselines"]),
                    "affected_min_velocity_mps": (
                        min(modified_outputs) if modified_outputs else max(scenario["outputs"])
                    ),
                    "has_stop": "stop" in colors,
                    "source": scenario["source"],
                }
            )
            cursor = float(scenario_end) + self.gap
        return rows, summaries

    def place_atlas_on_map(self):
        atlas_length = float(self.rows[-1]["atlas_station_m"])
        route_length = float(self.route_stations[-1])
        available_length = route_length - self.route_start_station
        if available_length <= 1.0:
            raise RuntimeError(
                "route_start_station_m {:.2f} exceeds route length {:.2f}".format(
                    self.route_start_station, route_length
                )
            )
        self.map_distance_scale = min(1.0, available_length / atlas_length)
        for row in self.rows:
            map_station = self.route_start_station + (
                float(row["atlas_station_m"]) * self.map_distance_scale
            )
            x, y, z = interpolate_xyz(
                self.route_stations, self.route_points, map_station
            )
            row["map_station_m"] = round(map_station, 6)
            row["position_x_m"] = round(x, 6)
            row["position_y_m"] = round(y, 6)
            row["position_z_m"] = round(z, 6)

        for summary in self.summaries:
            scenario_rows = [
                row
                for row in self.rows
                if row["scenario_index"] == summary["scenario_index"]
            ]
            summary["map_start_station_m"] = scenario_rows[0]["map_station_m"]
            summary["map_end_station_m"] = scenario_rows[-1]["map_station_m"]

    def marker(self, namespace, marker_id, marker_type):
        value = Marker()
        value.header.frame_id = self.frame_id
        value.header.stamp = rospy.Time.now()
        value.ns = namespace
        value.id = marker_id
        value.type = marker_type
        value.action = Marker.ADD
        value.pose.orientation.w = 1.0
        value.lifetime = rospy.Duration(0)
        return value

    def ros_point(self, row, z=0.0):
        return Point(
            x=float(row["position_x_m"]),
            y=float(row["position_y_m"]),
            z=float(row.get("position_z_m", 0.0)) + float(z),
        )

    def row_normal(self, row_index):
        previous = self.rows[max(row_index - 1, 0)]
        following = self.rows[min(row_index + 1, len(self.rows) - 1)]
        dx = float(following["position_x_m"]) - float(previous["position_x_m"])
        dy = float(following["position_y_m"]) - float(previous["position_y_m"])
        length = math.hypot(dx, dy)
        if length <= 1.0e-6:
            return 0.0, 1.0
        return -dy / length, dx / length

    def make_trajectory(self):
        trajectory = Trajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = self.frame_id
        elapsed = 0.0
        for row_index, row in enumerate(self.rows):
            point = TrajectoryPoint()
            point.pose.position = self.ros_point(row)

            previous = self.rows[max(row_index - 1, 0)]
            following = self.rows[min(row_index + 1, len(self.rows) - 1)]
            dx = float(following["position_x_m"]) - float(previous["position_x_m"])
            dy = float(following["position_y_m"]) - float(previous["position_y_m"])
            yaw = math.atan2(dy, dx)
            point.pose.orientation.z = math.sin(yaw * 0.5)
            point.pose.orientation.w = math.cos(yaw * 0.5)
            point.longitudinal_velocity_mps = float(row["output_velocity_mps"])
            point.time_from_start = rospy.Duration.from_sec(elapsed)
            trajectory.points.append(point)

            if row_index >= len(self.rows) - 1:
                continue
            next_row = self.rows[row_index + 1]
            distance = math.hypot(
                float(next_row["position_x_m"]) - float(row["position_x_m"]),
                float(next_row["position_y_m"]) - float(row["position_y_m"]),
            )
            nominal_velocity = max(
                0.5
                * (
                    float(row["baseline_velocity_mps"])
                    + float(next_row["baseline_velocity_mps"])
                ),
                0.1,
            )
            elapsed += distance / nominal_velocity
        return trajectory

    def make_markers(self):
        markers = MarkerArray()
        base = self.marker("atlas_base", 0, Marker.LINE_STRIP)
        base.scale.x = 2.2
        base.color = rgba(BASE_COLOR)
        base.points = [self.ros_point(row, 0.0) for row in self.rows]
        markers.markers.append(base)

        route_context = self.marker("kcity_route_context", 0, Marker.LINE_STRIP)
        route_context.scale.x = 0.45
        route_context.color = rgba((0.50, 0.53, 0.58, 0.45))
        route_context.points = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]) + 0.15)
            for point in self.route_points
        ]
        markers.markers.append(route_context)

        colored = self.marker("velocity_reason_segments", 0, Marker.LINE_LIST)
        colored.scale.x = 1.65
        points = self.marker("velocity_reason_points", 0, Marker.SPHERE_LIST)
        points.scale.x = points.scale.y = points.scale.z = 2.5
        for first, second in zip(self.rows[:-1], self.rows[1:]):
            if first["scenario_index"] != second["scenario_index"]:
                continue
            _, first_color = self.point_color(
                first["planner"],
                first["baseline_velocity_mps"],
                first["output_velocity_mps"],
            )
            _, second_color = self.point_color(
                second["planner"],
                second["baseline_velocity_mps"],
                second["output_velocity_mps"],
            )
            colored.points.extend([self.ros_point(first, 0.35), self.ros_point(second, 0.35)])
            colored.colors.extend([rgba(first_color), rgba(second_color)])
        for row in self.rows:
            _, marker_color = self.point_color(
                row["planner"],
                row["baseline_velocity_mps"],
                row["output_velocity_mps"],
            )
            points.points.append(self.ros_point(row, 0.55))
            points.colors.append(rgba(marker_color))
        markers.markers.extend([colored, points])

        connectors = self.marker("label_connectors", 0, Marker.LINE_LIST)
        connectors.scale.x = 0.7
        connectors.color = rgba((0.62, 0.66, 0.72, 0.72))
        for summary in self.summaries:
            scenario_rows = [
                row
                for row in self.rows
                if row["scenario_index"] == summary["scenario_index"]
            ]
            anchor = scenario_rows[len(scenario_rows) // 2]
            anchor_index = int(anchor["global_point_index"])
            label = self.marker("scenario_labels", summary["scenario_index"], Marker.TEXT_VIEW_FACING)
            label.pose.position = self.ros_point(anchor, 4.0)
            normal_x, normal_y = self.row_normal(anchor_index)
            label_offset = 42.0 if summary["scenario_index"] % 2 else -42.0
            label.pose.position.x += normal_x * label_offset
            label.pose.position.y += normal_y * label_offset
            label.scale.z = 8.0
            label.color = rgba(BVP_COLOR if summary["planner"] == "BVP" else MVP_COLOR)
            label.text = (
                "{}\n{} | {}\n{}\n{:.2f} -> {:.2f} m/s".format(
                    summary["scenario"],
                    summary["planner"],
                    summary["module"],
                    summary["reason"],
                    summary["baseline_max_velocity_mps"],
                    summary["affected_min_velocity_mps"],
                )
            )
            markers.markers.append(label)
            connectors.points.extend(
                [self.ros_point(anchor, 0.8), Point(
                    x=label.pose.position.x,
                    y=label.pose.position.y,
                    z=0.8,
                )]
            )

            start = self.marker("scenario_boundaries", summary["scenario_index"], Marker.CUBE)
            start.pose.position = self.ros_point(scenario_rows[0], 1.4)
            start.scale.x = 2.4
            start.scale.y = 2.4
            start.scale.z = 2.8
            start.color = rgba(BVP_COLOR if summary["planner"] == "BVP" else MVP_COLOR)
            markers.markers.append(start)

        markers.markers.append(connectors)

        legend = self.marker("legend", 0, Marker.TEXT_VIEW_FACING)
        first = self.rows[0]
        normal_x, normal_y = self.row_normal(0)
        legend.pose.position = self.ros_point(first, 7.0)
        legend.pose.position.x -= normal_x * 42.0
        legend.pose.position.y -= normal_y * 42.0
        legend.scale.z = 8.0
        legend.color = rgba(TEXT_COLOR)
        legend.text = (
            "K-CITY VELOCITY REASON ATLAS\n"
            "GREEN normal | ORANGE BVP slowdown | YELLOW MVP slowdown | RED stop"
        )
        markers.markers.append(legend)
        return markers

    def write_results(self):
        os.makedirs(self.output_dir, exist_ok=True)
        csv_path = os.path.join(self.output_dir, "velocity_scenario_atlas.points.csv")
        json_path = os.path.join(self.output_dir, "velocity_scenario_atlas.summary.json")
        with open(csv_path, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        with open(json_path, "w", encoding="utf-8") as stream:
            json.dump(
                {
                    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
                    "format": "planning_velocity_scenario_atlas",
                    "point_count": len(self.rows),
                    "scenario_count": len(self.summaries),
                    "map": {
                        "file": self.map_file,
                        "projector_type": self.projector_type,
                        "route_lane_ids": self.used_route_lane_ids,
                        "route_length_m": self.route_stations[-1],
                        "atlas_route_start_station_m": self.route_start_station,
                        "distance_scale": self.map_distance_scale,
                    },
                    "color_legend": {
                        "normal": "green",
                        "bvp_slowdown": "orange",
                        "mvp_slowdown": "yellow",
                        "stop": "red",
                    },
                    "scenarios": self.summaries,
                },
                stream,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")


def main():
    rospy.init_node("planning_velocity_scenario_atlas")
    PlanningVelocityScenarioAtlas()
    rospy.spin()


if __name__ == "__main__":
    main()
