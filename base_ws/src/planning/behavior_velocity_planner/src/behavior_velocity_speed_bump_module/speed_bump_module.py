from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor

from behavior_velocity_planner_common.geometry import (
    polyline_polygon_intersections,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.path_utils import (
    insert_path_point,
    path_with_lane_id_positions,
    project_pose_to_path,
    set_velocity_between_indices,
)


@dataclass
class SpeedBumpParameters:
    slow_start_margin: float = 1.0
    slow_end_margin: float = 1.0
    print_debug_info: bool = False
    min_height: float = 0.05
    max_height: float = 0.30
    min_speed: float = 1.39
    max_speed: float = 2.78
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    vehicle_width: float = 1.9

    @classmethod
    def from_ros(cls, prefix="~speed_bump"):
        parameter = cls()
        direct_fields = (
            "slow_start_margin",
            "slow_end_margin",
            "print_debug_info",
            "base_link_to_front",
            "base_link_to_rear",
            "vehicle_width",
        )
        speed_fields = ("min_height", "max_height", "min_speed", "max_speed")
        for field_name in direct_fields:
            current = getattr(parameter, field_name)
            value = rospy.get_param("{}/{}".format(prefix, field_name), current)
            if isinstance(current, bool):
                value = (
                    value.lower() in ("true", "1", "yes", "on")
                    if isinstance(value, str)
                    else bool(value)
                )
            setattr(parameter, field_name, type(current)(value))
        for field_name in speed_fields:
            current = getattr(parameter, field_name)
            value = rospy.get_param(
                "{}/speed_calculation/{}".format(prefix, field_name), current
            )
            setattr(parameter, field_name, type(current)(value))
        return parameter


class SpeedBumpModuleManager(SceneModuleManagerInterface):
    module_name = "speed_bump"

    def __init__(self, parameters=None):
        self.parameters = parameters or SpeedBumpParameters.from_ros()
        self.lanelet_map = None

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "speed_bump: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "speed_bump: input path is too short"
            return result

        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        route_lane_ids = {
            int(lane_id)
            for point in path_with_lane_id.points
            for lane_id in point.lane_ids
        }
        ego_station = self._ego_station(output_path, planner_data.odometry)
        active = []

        for speed_bump in lanelet_map.speed_bump_candidates():
            if speed_bump.lane_ids and route_lane_ids.isdisjoint(speed_bump.lane_ids):
                continue
            intersections = polyline_polygon_intersections(
                path_xyz, speed_bump.points
            )
            if len(intersections) < 2:
                continue
            entry_station = intersections[0]["station"]
            exit_station = intersections[-1]["station"]
            slow_start_station = max(
                0.0,
                entry_station
                - self.parameters.base_link_to_front
                - self.parameters.slow_start_margin,
            )
            slow_end_station = (
                exit_station
                + self.parameters.base_link_to_rear
                + self.parameters.slow_end_margin
            )
            slow_speed = self.calculate_slow_speed(speed_bump.tags)

            start_index, start_pose = insert_path_point(
                output_path, slow_start_station
            )
            end_index, end_pose = insert_path_point(output_path, slow_end_station)
            set_velocity_between_indices(
                output_path, start_index, end_index, slow_speed
            )

            result.modified = True
            result.planning_factors.append(
                self._make_planning_factor(
                    output_path.header,
                    start_pose,
                    end_pose,
                    slow_start_station - ego_station,
                    slow_end_station - ego_station,
                    slow_speed,
                    speed_bump,
                )
            )
            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    speed_bump,
                    start_pose,
                    end_pose,
                    slow_speed,
                )
            )
            active.append(
                "{}:{:.2f}m/s".format(speed_bump.regulatory_id, slow_speed)
            )

        if active:
            result.status = "speed_bump: " + ", ".join(active)
        else:
            result.status = "speed_bump: no speed bump on path"
        return result

    def calculate_slow_speed(self, tags):
        if "slow_down_speed" in tags:
            return max(float(tags["slow_down_speed"]) / 3.6, 0.0)

        minimum_height = float(self.parameters.min_height)
        maximum_height = float(self.parameters.max_height)
        height = float(tags.get("height", minimum_height))
        height = max(minimum_height, min(maximum_height, height))
        if maximum_height <= minimum_height:
            return max(float(self.parameters.min_speed), 0.0)
        ratio = (height - minimum_height) / (maximum_height - minimum_height)
        speed = self.parameters.max_speed + ratio * (
            self.parameters.min_speed - self.parameters.max_speed
        )
        return max(float(speed), 0.0)

    @staticmethod
    def _ego_station(output_path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(output_path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _control_point(pose, velocity, distance):
        control_point = ControlPoint()
        control_point.pose = deepcopy(pose)
        control_point.velocity = float(velocity)
        control_point.shift_length = 0.0
        control_point.distance = float(distance)
        return control_point

    def _make_planning_factor(
        self,
        header,
        start_pose,
        end_pose,
        start_distance,
        end_distance,
        slow_speed,
        speed_bump,
    ):
        factor = PlanningFactor()
        factor.module = "speed_bump"
        factor.is_driving_forward = True
        factor.control_points = [
            self._control_point(start_pose, slow_speed, start_distance),
            self._control_point(end_pose, slow_speed, end_distance),
        ]
        factor.behavior = PlanningFactor.SLOW_DOWN
        factor.detail = "speed_bump_id={} polygon_id={} speed={:.3f}".format(
            speed_bump.regulatory_id, speed_bump.polygon_id, slow_speed
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = True
        factor.safety_factors.detail = "map speed bump"
        return factor

    def _make_markers(self, header, speed_bump, start_pose, end_pose, slow_speed):
        marker_id = int(speed_bump.regulatory_id % 2000000000)
        outline = self._base_marker(header, marker_id, Marker.LINE_STRIP)
        outline.ns = "bvp_speed_bump/polygon"
        outline.scale.x = 0.25
        outline.color.r = 1.0
        outline.color.g = 0.65
        outline.color.b = 0.0
        outline.color.a = 1.0
        outline.points = [self._to_point(point) for point in speed_bump.points]
        outline.points.append(self._to_point(speed_bump.points[0]))

        fill = self._base_marker(header, marker_id + 1, Marker.TRIANGLE_LIST)
        fill.ns = "bvp_speed_bump/area"
        fill.color.r = 1.0
        fill.color.g = 0.55
        fill.color.b = 0.0
        fill.color.a = 0.35
        anchor = speed_bump.points[0]
        for index in range(1, len(speed_bump.points) - 1):
            fill.points.extend(
                [
                    self._to_point(anchor),
                    self._to_point(speed_bump.points[index]),
                    self._to_point(speed_bump.points[index + 1]),
                ]
            )

        start = self._pose_marker(
            header, marker_id + 2, "bvp_speed_bump/slow_start", start_pose, 0.1, 0.9, 1.0
        )
        end = self._pose_marker(
            header, marker_id + 3, "bvp_speed_bump/slow_end", end_pose, 1.0, 0.1, 0.8
        )

        text = self._base_marker(header, marker_id + 4, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_speed_bump/status"
        text.pose = deepcopy(start_pose)
        text.pose.position.z += 2.0
        text.scale.z = 0.65
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "SPEED BUMP {}\n{:.2f} m/s".format(
            speed_bump.regulatory_id, slow_speed
        )
        return [outline, fill, start, end, text]

    @staticmethod
    def _pose_marker(header, marker_id, namespace, pose, red, green, blue):
        marker = SpeedBumpModuleManager._base_marker(
            header, marker_id, Marker.SPHERE
        )
        marker.ns = namespace
        marker.pose = deepcopy(pose)
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        return marker

    @staticmethod
    def _to_point(point):
        output = Point()
        output.x = float(point[0])
        output.y = float(point[1])
        output.z = float(point[2]) if len(point) > 2 else 0.0
        return output

    @staticmethod
    def _base_marker(header, marker_id, marker_type):
        marker = Marker()
        marker.header = deepcopy(header)
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.5)
        return marker
