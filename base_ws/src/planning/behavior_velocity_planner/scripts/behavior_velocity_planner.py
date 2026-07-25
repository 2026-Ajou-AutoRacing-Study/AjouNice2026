#!/usr/bin/env python3

import importlib
import math
import threading
import time
from copy import deepcopy

import rospy
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from autoware_internal_planning_msgs.msg import PathWithLaneId, PlanningFactorArray
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import (
    PredictedObjects,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from autoware_planning_msgs.msg import Path
from tier4_planning_msgs.msg import StopReasonArray
from tier4_v2x_msgs.msg import (
    InfrastructureCommandArray,
    VirtualTrafficLightStateArray,
)

from behavior_velocity_planner_common import BehaviorVelocityMap, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


DEFAULT_PLUGINS = {
    "blind_spot": "behavior_velocity_blind_spot_module:BlindSpotModuleManager",
    "crosswalk": "behavior_velocity_crosswalk_module:CrosswalkModuleManager",
    "detection_area": "behavior_velocity_detection_area_module:DetectionAreaModuleManager",
    "intersection": "behavior_velocity_intersection_module:IntersectionModuleManager",
    "merge_from_private": "behavior_velocity_merge_from_private_module:MergeFromPrivateModuleManager",
    "no_drivable_lane": "behavior_velocity_no_drivable_lane_module:NoDrivableLaneModuleManager",
    "no_stopping_area": "behavior_velocity_no_stopping_area_module:NoStoppingAreaModuleManager",
    "occlusion_spot": "behavior_velocity_occlusion_spot_module:OcclusionSpotModuleManager",
    "run_out": "behavior_velocity_run_out_module:RunOutModuleManager",
    "speed_bump": "behavior_velocity_speed_bump_module:SpeedBumpModuleManager",
    "stop_line": "behavior_velocity_stop_line_module:StopLineModuleManager",
    "traffic_light": "behavior_velocity_traffic_light_module:TrafficLightModuleManager",
    "virtual_traffic_light": "behavior_velocity_virtual_traffic_light_module:VirtualTrafficLightModuleManager",
    "walkway": "behavior_velocity_walkway_module:WalkwayModuleManager",
}


class BehaviorVelocityPlanner:
    def __init__(self):
        self.lock = threading.RLock()
        self.data = PlannerData()
        self.managers = []

        self.map_file = rospy.get_param("~map_file", "")
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.flatten_map_z = bool(rospy.get_param("~flatten_map_z", False))
        self.max_processing_length = max(
            float(rospy.get_param("~max_processing_length", 350.0)), 0.0
        )
        self.debug_processing_time = bool(
            rospy.get_param("~debug_processing_time", False)
        )

        self.path_pub = rospy.Publisher("~output/path", Path, queue_size=1)
        self.stop_reasons_pub = rospy.Publisher(
            "~output/stop_reasons", StopReasonArray, queue_size=1
        )
        self.planning_factors_pub = rospy.Publisher(
            "~output/planning_factors", PlanningFactorArray, queue_size=1
        )
        self.markers_pub = rospy.Publisher(
            "~debug/markers", MarkerArray, queue_size=1
        )
        self.status_pub = rospy.Publisher("~debug/status", String, queue_size=1)
        self.traffic_signal_pub = rospy.Publisher(
            "~output/traffic_signal", TrafficLightGroup, queue_size=1
        )
        self.infrastructure_commands_pub = rospy.Publisher(
            "~output/infrastructure_commands",
            InfrastructureCommandArray,
            queue_size=1,
        )

        self.subscribers = [
            rospy.Subscriber(
                "~input/path_with_lane_id",
                PathWithLaneId,
                self.on_path,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/vector_map", LaneletMapBin, self.on_vector_map, queue_size=1
            ),
            rospy.Subscriber(
                "~input/vehicle_odometry", Odometry, self.on_odometry, queue_size=1
            ),
            rospy.Subscriber(
                "~input/acceleration",
                AccelWithCovarianceStamped,
                self.on_acceleration,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/dynamic_objects",
                PredictedObjects,
                self.on_dynamic_objects,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/no_ground_pointcloud",
                PointCloud2,
                self.on_no_ground_pointcloud,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/compare_map_filtered_pointcloud",
                PointCloud2,
                self.on_compare_map_filtered_pointcloud,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/traffic_signals",
                TrafficLightGroupArray,
                self.on_traffic_signals,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/occupancy_grid",
                OccupancyGrid,
                self.on_occupancy_grid,
                queue_size=1,
            ),
            rospy.Subscriber(
                "~input/virtual_traffic_light_states",
                VirtualTrafficLightStateArray,
                self.on_virtual_traffic_light_states,
                queue_size=1,
            ),
        ]

        self.load_map_file()
        self.load_plugins()

    def load_map_file(self):
        if not self.map_file:
            rospy.logwarn("behavior_velocity_planner: map_file is empty; waiting for LaneletMapBin")
            return
        try:
            lanelet_map = BehaviorVelocityMap.load(
                self.map_file,
                self.projector_file,
                self.projector_type,
                self.flatten_map_z,
            )
            self.set_map(lanelet_map)
            rospy.loginfo(
                "behavior_velocity_planner: loaded %d road lanelets, %d intersection lanelets, %d stop-line ways, %d speed bumps, %d traffic lights, %d crosswalks, and %d walkways from %s",
                lanelet_map.road_lanelet_count(),
                len(lanelet_map.intersection_lanelet_candidates()),
                lanelet_map.stop_line_count(),
                lanelet_map.speed_bump_count(),
                lanelet_map.traffic_light_count(),
                lanelet_map.pedestrian_crossing_count("crosswalk"),
                lanelet_map.pedestrian_crossing_count("walkway"),
                self.map_file,
            )
        except Exception as error:
            rospy.logerr("behavior_velocity_planner: failed to load map: %s", error)

    def load_plugins(self):
        requested = rospy.get_param(
            "~launch_modules",
            [
                "stop_line",
                "speed_bump",
                "traffic_light",
                "crosswalk",
                "walkway",
                "blind_spot",
                "intersection",
                "detection_area",
                "merge_from_private",
                "virtual_traffic_light",
                "occlusion_spot",
                "run_out",
                "no_stopping_area",
                "no_drivable_lane",
            ],
        )
        if isinstance(requested, str):
            requested = [requested]
        configured = dict(DEFAULT_PLUGINS)
        configured.update(rospy.get_param("~module_plugins", {}))

        for module_name in requested:
            specification = configured.get(module_name, module_name)
            try:
                module_path, class_name = specification.split(":", 1)
                plugin_module = importlib.import_module(module_path)
                manager_class = getattr(plugin_module, class_name)
                manager = manager_class()
                if self.data.lanelet_map is not None:
                    manager.on_map_updated(self.data.lanelet_map)
                self.managers.append(manager)
                rospy.loginfo(
                    "behavior_velocity_planner: loaded module '%s' (%s)",
                    module_name,
                    specification,
                )
            except Exception as error:
                rospy.logerr(
                    "behavior_velocity_planner: failed to load module '%s': %s",
                    module_name,
                    error,
                )

    def set_map(self, lanelet_map):
        with self.lock:
            self.data.lanelet_map = lanelet_map
            for manager in self.managers:
                manager.on_map_updated(lanelet_map)

    def on_vector_map(self, message):
        if not message.data:
            return
        try:
            lanelet_map = BehaviorVelocityMap.from_osm_bytes(
                message.data,
                source=message.name_map or "LaneletMapBin",
                projector_file=self.projector_file,
                projector_type=self.projector_type,
                flatten_z=self.flatten_map_z,
            )
            self.set_map(lanelet_map)
            rospy.loginfo(
                "behavior_velocity_planner: loaded %d road lanelets, %d intersection lanelets, %d stop-line ways, %d speed bumps, %d traffic lights, %d crosswalks, and %d walkways from LaneletMapBin",
                lanelet_map.road_lanelet_count(),
                len(lanelet_map.intersection_lanelet_candidates()),
                lanelet_map.stop_line_count(),
                lanelet_map.speed_bump_count(),
                lanelet_map.traffic_light_count(),
                lanelet_map.pedestrian_crossing_count("crosswalk"),
                lanelet_map.pedestrian_crossing_count("walkway"),
            )
        except Exception as error:
            rospy.logerr_throttle(
                5.0, "behavior_velocity_planner: LaneletMapBin parse failed: %s", error
            )

    def on_odometry(self, message):
        with self.lock:
            self.data.odometry = message

    def on_acceleration(self, message):
        with self.lock:
            self.data.acceleration = message

    def on_dynamic_objects(self, message):
        with self.lock:
            self.data.dynamic_objects = message

    def on_no_ground_pointcloud(self, message):
        with self.lock:
            self.data.no_ground_pointcloud = message

    def on_compare_map_filtered_pointcloud(self, message):
        with self.lock:
            self.data.compare_map_filtered_pointcloud = message

    def on_traffic_signals(self, message):
        with self.lock:
            self.data.traffic_signals = message

    def on_occupancy_grid(self, message):
        with self.lock:
            self.data.occupancy_grid = message

    def on_virtual_traffic_light_states(self, message):
        with self.lock:
            self.data.virtual_traffic_light_states = message

    def on_path(self, path_with_lane_id):
        if not path_with_lane_id.points:
            return

        processing_started = time.perf_counter()
        module_times = []
        with self.lock:
            self.data.current_time = rospy.Time.now().to_sec()
            processing_path, retained_points = self.processing_path(path_with_lane_id)
            output_path = to_path(processing_path)
            results = []
            for manager in self.managers:
                try:
                    module_started = time.perf_counter()
                    results.append(
                        manager.modify_path_velocity(
                            processing_path, output_path, self.data
                        )
                    )
                    module_times.append(
                        (manager.module_name, time.perf_counter() - module_started)
                    )
                except Exception as error:
                    rospy.logerr_throttle(
                        2.0,
                        "behavior_velocity_planner: module '%s' failed: %s",
                        manager.module_name,
                        error,
                    )

            if retained_points:
                output_path.points.extend(
                    deepcopy(point.point) for point in retained_points
                )

        self.publish_results(output_path, results)
        if self.debug_processing_time:
            rospy.loginfo(
                "behavior_velocity_planner: cycle %.1f ms input=%d processed=%d "
                "modules=[%s]",
                (time.perf_counter() - processing_started) * 1000.0,
                len(path_with_lane_id.points),
                len(processing_path.points),
                ", ".join(
                    "{}:{:.1f}".format(name, elapsed * 1000.0)
                    for name, elapsed in module_times
                ),
            )

    def processing_path(self, path_with_lane_id):
        if self.max_processing_length <= 0.0 or len(path_with_lane_id.points) < 3:
            return path_with_lane_id, []

        points = path_with_lane_id.points
        accumulated = 0.0
        end_index = len(points)
        for index, (first, second) in enumerate(zip(points[:-1], points[1:]), start=1):
            first_position = first.point.pose.position
            second_position = second.point.pose.position
            accumulated += math.hypot(
                second_position.x - first_position.x,
                second_position.y - first_position.y,
            )
            if accumulated >= self.max_processing_length:
                end_index = index + 1
                break

        if end_index >= len(points):
            return path_with_lane_id, []

        processing_path = deepcopy(path_with_lane_id)
        processing_path.points = deepcopy(points[:end_index])
        return processing_path, points[end_index:]

    def publish_results(self, output_path, results):
        stop_reasons = StopReasonArray()
        stop_reasons.header = deepcopy(output_path.header)
        planning_factors = PlanningFactorArray()
        planning_factors.header = deepcopy(output_path.header)
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header = deepcopy(output_path.header)
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        statuses = []
        traffic_signal = None
        infrastructure_commands = InfrastructureCommandArray()
        infrastructure_commands.stamp = rospy.Time.now()
        for result in results:
            stop_reasons.stop_reasons.extend(result.stop_reasons)
            planning_factors.factors.extend(result.planning_factors)
            marker_array.markers.extend(result.markers)
            if result.status:
                statuses.append(result.status)
            if traffic_signal is None and result.traffic_signal is not None:
                traffic_signal = result.traffic_signal
            infrastructure_commands.commands.extend(result.infrastructure_commands)

        self.path_pub.publish(output_path)
        self.stop_reasons_pub.publish(stop_reasons)
        self.planning_factors_pub.publish(planning_factors)
        self.markers_pub.publish(marker_array)
        self.status_pub.publish(String(data="; ".join(statuses) or "no active modules"))
        if traffic_signal is not None:
            self.traffic_signal_pub.publish(traffic_signal)
        self.infrastructure_commands_pub.publish(infrastructure_commands)


def main():
    rospy.init_node("behavior_velocity_planner")
    BehaviorVelocityPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
