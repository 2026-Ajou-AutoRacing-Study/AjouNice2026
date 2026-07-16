import math
from dataclasses import dataclass

import rospy

from behavior_velocity_planner_common.geometry import cumulative_lengths, point_at_station
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.module_utils import (
    ego_station,
    object_position,
    object_speed,
    polygon_marker,
    sphere_marker,
    text_marker,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_positions,
    set_velocity_between_indices,
    yaw_from_quaternion,
)


@dataclass
class OcclusionSpotParameters:
    detection_method: str = "occupancy_grid"
    use_object_info: bool = True
    use_partition_lanelet: bool = True
    pedestrian_vel: float = 1.5
    pedestrian_radius: float = 0.3
    detection_area_offset: float = 30.0
    detection_area_length: float = 100.0
    stuck_vehicle_vel: float = 1.0
    lateral_distance: float = 1.5
    search_dist: float = 10.0
    safety_ratio: float = 0.8
    max_slow_down_jerk: float = -0.3
    max_slow_down_accel: float = -1.5
    min_allowed_velocity: float = 1.0
    safe_margin: float = 2.0
    min_occlusion_spot_size: float = 1.0
    slice_length: float = 10.0
    min_longitudinal_offset: float = 1.0
    max_lateral_distance: float = 5.0
    base_link_to_front: float = 3.79
    path_interpolation_ds: float = 0.1
    free_space_max: int = 43
    occupied_min: int = 57

    @classmethod
    def from_ros(cls, prefix="~occlusion_spot"):
        parameter = cls()
        key_map = {
            "detection_area_offset": "threshold/detection_area_offset",
            "detection_area_length": "threshold/detection_area_length",
            "stuck_vehicle_vel": "threshold/stuck_vehicle_vel",
            "lateral_distance": "threshold/lateral_distance",
            "search_dist": "threshold/search_dist",
            "safety_ratio": "motion/safety_ratio",
            "max_slow_down_jerk": "motion/max_slow_down_jerk",
            "max_slow_down_accel": "motion/max_slow_down_accel",
            "min_allowed_velocity": "motion/min_allowed_velocity",
            "safe_margin": "motion/safe_margin",
            "min_occlusion_spot_size": "detection_area/min_occlusion_spot_size",
            "slice_length": "detection_area/slice_length",
            "min_longitudinal_offset": "detection_area/min_longitudinal_offset",
            "max_lateral_distance": "detection_area/max_lateral_distance",
            "base_link_to_front": "vehicle/base_link_to_front",
            "path_interpolation_ds": "motion/path_interpolation_ds",
            "free_space_max": "grid/free_space_max",
            "occupied_min": "grid/occupied_min",
        }
        for name in parameter.__dataclass_fields__:
            current = getattr(parameter, name)
            key = key_map.get(name, name)
            value = rospy.get_param("{}/{}".format(prefix, key), current)
            setattr(parameter, name, type(current)(value))
        return parameter


class OcclusionSpotModuleManager(SceneModuleManagerInterface):
    module_name = "occlusion_spot"

    def __init__(self, parameters=None):
        self.parameters = parameters or OcclusionSpotParameters.from_ros()

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        del path_with_lane_id
        result = SceneModuleResult(module_name=self.module_name)
        if len(output_path.points) < 2:
            result.status = "occlusion_spot: input path is too short"
            return result
        path_xyz = path_positions(output_path)
        lengths = cumulative_lengths(path_xyz)
        current_station = ego_station(output_path, planner_data.odometry)
        candidate = self._find_candidate(path_xyz, lengths, current_station, planner_data)
        if candidate is None:
            result.status = "occlusion_spot: clear"
            return result

        collision_station = candidate["station"]
        longitudinal = max(collision_station - current_station, 0.0)
        lateral_time = max(
            candidate["lateral"] - self.parameters.pedestrian_radius, 0.0
        ) / max(self.parameters.pedestrian_vel, 0.1)
        safe_velocity = max(
            self.parameters.min_allowed_velocity,
            self.parameters.safety_ratio
            * max(longitudinal - self.parameters.safe_margin, 0.0)
            / max(lateral_time, 0.1),
        )
        measured_speed = ego_speed(planner_data.odometry)
        profile_velocity = max(
            float(point.longitudinal_velocity_mps) for point in output_path.points
        )
        target_velocity = min(safe_velocity, profile_velocity)
        if target_velocity >= profile_velocity - 1.0e-3:
            result.status = "occlusion_spot: visible but current profile is safe"
            result.markers.extend(self._markers(output_path.header, candidate, target_velocity))
            return result

        # Autoware evaluates the possible collision point at the vehicle front.
        # Convert it back to the base-link station before applying the speed cap.
        slow_start = max(
            0.0,
            collision_station - self.parameters.base_link_to_front,
        )
        processing_end = min(
            lengths[-1],
            current_station
            + self.parameters.detection_area_offset
            - self.parameters.base_link_to_front
            + self.parameters.min_longitudinal_offset,
        )
        self._interpolate_processing_range(output_path, 0.0, processing_end)
        insert_path_point(output_path, processing_end)
        path_xyz = path_positions(output_path)
        lengths = cumulative_lengths(path_xyz)
        output_path.points = [
            point
            for point, station in zip(output_path.points, lengths)
            if station <= processing_end + 1.0e-3
        ]
        start_index, _ = insert_path_point(output_path, slow_start)
        end_index = len(output_path.points) - 1
        set_velocity_between_indices(output_path, start_index, end_index, target_velocity)

        # With a stopped ego, the upstream path contains historical points.  Keep
        # those at the safe speed, anchor the profile at the measured ego speed,
        # and recreate Autoware's jerk-limited rise toward min_allowed_velocity.
        if measured_speed <= self.parameters.min_allowed_velocity:
            ego_index, _ = insert_path_point(output_path, current_station)
            output_path.points[ego_index].longitudinal_velocity_mps = min(
                float(output_path.points[ego_index].longitudinal_velocity_mps),
                measured_speed,
            )
            restart_gain = max(abs(self.parameters.max_slow_down_jerk) * 0.47, 0.01)
            positions = path_positions(output_path)
            stations = cumulative_lengths(positions)
            for index in range(ego_index + 1, len(output_path.points)):
                distance = max(stations[index] - current_station, 0.0)
                restart_velocity = min(target_velocity, restart_gain * distance * distance)
                output_path.points[index].longitudinal_velocity_mps = min(
                    float(output_path.points[index].longitudinal_velocity_mps),
                    restart_velocity,
                )
        detail = "source={} lateral={:.2f}m ttv={:.2f}s safe_velocity={:.2f}mps".format(
            candidate["source"], candidate["lateral"], lateral_time, target_velocity
        )
        result.modified = True
        result.markers.extend(self._markers(output_path.header, candidate, target_velocity))
        result.status = "occlusion_spot: SLOW_DOWN " + detail
        return result

    def _find_candidate(self, path_xyz, lengths, current_station, planner_data):
        first = current_station + self.parameters.min_longitudinal_offset
        last = min(
            lengths[-1],
            current_station
            + self.parameters.detection_area_offset
            + self.parameters.detection_area_length,
        )
        candidates = []
        if self.parameters.detection_method == "occupancy_grid":
            grid = planner_data.occupancy_grid
            if grid is not None:
                station = first
                step = max(float(grid.info.resolution), 0.5)
                while station <= last:
                    center = point_at_station(path_xyz, station)
                    ahead = point_at_station(path_xyz, min(station + 0.5, lengths[-1]))
                    if center is None or ahead is None:
                        break
                    yaw = math.atan2(ahead[1] - center[1], ahead[0] - center[0])
                    lateral = self.parameters.lateral_distance
                    while lateral <= self.parameters.max_lateral_distance + 1.0e-6:
                        for sign in (-1.0, 1.0):
                            point = (
                                center[0] - math.sin(yaw) * lateral * sign,
                                center[1] + math.cos(yaw) * lateral * sign,
                                center[2],
                            )
                            if self._occluded(grid, point):
                                collision_station = min(
                                    station
                                    + self.parameters.pedestrian_radius
                                    + self.parameters.path_interpolation_ds,
                                    last,
                                )
                                candidates.append(
                                    {
                                        "source": "occupancy_grid",
                                        "point": point,
                                        "collision_point": point_at_station(
                                            path_xyz, collision_station
                                        ),
                                        "station": collision_station,
                                        "lateral": lateral,
                                        "object": None,
                                    }
                                )
                        lateral += step
                    if candidates:
                        break
                    station += step

        if self.parameters.use_object_info:
            for detected in getattr(planner_data.dynamic_objects, "objects", []):
                if object_speed(detected) > self.parameters.stuck_vehicle_vel:
                    continue
                position = object_position(detected)
                projection = self._project(path_xyz, position)
                if projection is None:
                    continue
                station, lateral = projection
                if (
                    first <= station <= last
                    and self.parameters.lateral_distance
                    <= lateral
                    <= self.parameters.max_lateral_distance
                ):
                    collision_station = min(station + self.parameters.search_dist, last)
                    candidates.append(
                        {
                            "source": "predicted_object",
                            "point": position,
                            "collision_point": point_at_station(path_xyz, collision_station),
                            "station": collision_station,
                            "lateral": lateral,
                            "object": detected,
                        }
                    )
        return min(candidates, key=lambda item: item["station"]) if candidates else None

    def _interpolate_processing_range(self, path, first_station, last_station):
        step = max(self.parameters.path_interpolation_ds, 0.05)
        station = max(first_station, 0.0)
        while station <= last_station + 1.0e-6:
            insert_path_point(path, station)
            station += step

    def _occluded(self, grid, point):
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return False
        origin = grid.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        dx = point[0] - origin.position.x
        dy = point[1] - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        x = int(math.floor(local_x / resolution))
        y = int(math.floor(local_y / resolution))
        width = int(grid.info.width)
        height = int(grid.info.height)
        if not (0 <= x < width and 0 <= y < height):
            return False
        radius = max(int(math.ceil(self.parameters.min_occlusion_spot_size / resolution)), 1)
        values = []
        for cy in range(max(0, y - radius // 2), min(height, y + radius // 2 + 1)):
            for cx in range(max(0, x - radius // 2), min(width, x + radius // 2 + 1)):
                values.append(int(grid.data[cy * width + cx]))
        return bool(values) and all(
            value < 0
            or self.parameters.free_space_max < value < self.parameters.occupied_min
            for value in values
        )

    @staticmethod
    def _project(path_xyz, point):
        from behavior_velocity_planner_common.geometry import project_point_to_polyline

        projection = project_point_to_polyline(path_xyz, point)
        if projection is None:
            return None
        return float(projection[0]), float(projection[1])

    @staticmethod
    def _markers(header, candidate, velocity):
        x, y, z = candidate["point"]
        size = 1.0
        polygon = (
            (x - size, y - size, z),
            (x + size, y - size, z),
            (x + size, y + size, z),
            (x - size, y + size, z),
        )
        return [
            polygon_marker(
                header,
                "occlusion_spot/hidden_area",
                0,
                polygon,
                (0.55, 0.1, 0.85, 0.42),
            ),
            sphere_marker(
                header,
                "occlusion_spot/collision",
                0,
                candidate["collision_point"],
                (1.0, 0.1, 0.1, 0.95),
                1.1,
            ),
            text_marker(
                header,
                "occlusion_spot/label",
                0,
                candidate["point"],
                "OCCLUSION {:.1f} m/s".format(velocity),
                (1.0, 0.9, 0.2, 1.0),
            ),
        ]
