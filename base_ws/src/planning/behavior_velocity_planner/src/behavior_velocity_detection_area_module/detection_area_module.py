import math
from dataclasses import dataclass, field

import rospy
from sensor_msgs import point_cloud2

from autoware_internal_planning_msgs.msg import PlanningFactor, SafetyFactor
from autoware_perception_msgs.msg import ObjectClassification
from tier4_planning_msgs.msg import StopReason

from behavior_velocity_planner_common.geometry import first_polyline_intersection
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.module_utils import (
    ego_station,
    line_marker,
    make_planning_factor,
    make_stop_reason,
    object_label,
    object_position,
    polygon_marker,
    route_lane_ids,
    sphere_marker,
    text_marker,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_positions,
    set_velocity_from_index,
)


LABEL_NAMES = {
    "unknown": ObjectClassification.UNKNOWN,
    "car": ObjectClassification.CAR,
    "truck": ObjectClassification.TRUCK,
    "bus": ObjectClassification.BUS,
    "trailer": ObjectClassification.TRAILER,
    "motorcycle": ObjectClassification.MOTORCYCLE,
    "bicycle": ObjectClassification.BICYCLE,
    "pedestrian": ObjectClassification.PEDESTRIAN,
    "animal": ObjectClassification.ANIMAL,
    "hazard": ObjectClassification.HAZARD,
    "over_drivable": ObjectClassification.OVER_DRIVABLE,
    "under_drivable": ObjectClassification.UNDER_DRIVABLE,
}


@dataclass
class DetectionAreaParameters:
    stop_margin: float = 0.0
    pointcloud: bool = True
    target_labels: set = field(default_factory=set)
    use_dead_line: bool = False
    dead_line_margin: float = 5.0
    unstoppable_policy: str = "stop_after_stopline"
    max_deceleration: float = 1.0
    delay_response_time: float = 0.5
    state_clear_time: float = 2.0
    hold_stop_margin_distance: float = 0.0
    distance_to_judge_over_stop_line: float = 0.5
    suppress_pass_judge_when_stopping: bool = False
    base_link_to_front: float = 3.79

    @classmethod
    def from_ros(cls, prefix="~detection_area"):
        parameter = cls()
        scalar_fields = [
            name
            for name in parameter.__dataclass_fields__
            if name not in ("pointcloud", "target_labels")
        ]
        for name in scalar_fields:
            current = getattr(parameter, name)
            value = rospy.get_param("{}/{}".format(prefix, name), current)
            setattr(parameter, name, type(current)(value))
        parameter.pointcloud = bool(
            rospy.get_param(
                "{}/target_filtering/pointcloud".format(prefix), parameter.pointcloud
            )
        )
        parameter.target_labels = {
            value
            for name, value in LABEL_NAMES.items()
            if bool(
                rospy.get_param(
                    "{}/target_filtering/{}".format(prefix, name), False
                )
            )
        }
        if parameter.unstoppable_policy not in ("go", "force_stop", "stop_after_stopline"):
            parameter.unstoppable_policy = "stop_after_stopline"
        return parameter


class DetectionAreaModuleManager(SceneModuleManagerInterface):
    module_name = "detection_area"

    def __init__(self, parameters=None):
        self.parameters = parameters or DetectionAreaParameters.from_ros()
        self.lanelet_map = None
        self.states = {}

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.states.clear()

    def reset(self):
        self.states.clear()

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "detection_area: waiting for Lanelet2 map"
            return result
        if len(output_path.points) < 2:
            result.status = "detection_area: input path is too short"
            return result

        path_xyz = path_positions(output_path)
        route_ids = route_lane_ids(path_with_lane_id)
        current_station = ego_station(output_path, planner_data.odometry)
        now = float(planner_data.current_time)
        active = []
        for area in lanelet_map.detection_area_candidates():
            if area.lane_ids and route_ids.isdisjoint(area.lane_ids):
                continue
            if area.stop_line is None:
                continue
            crossing = first_polyline_intersection(path_xyz, area.stop_line.points)
            if crossing is None:
                continue
            line_station = float(crossing["station"])
            nominal_stop = max(
                0.0,
                line_station
                - self.parameters.stop_margin
                - self.parameters.base_link_to_front,
            )
            if (
                self.parameters.use_dead_line
                and current_station > line_station + self.parameters.dead_line_margin
            ):
                active.append("{}:DEAD_LINE_PASSED".format(area.regulatory_id))
                continue

            detection = self._detect(area, planner_data)
            state = self.states.setdefault(
                area.regulatory_id, {"last_detected": None, "target": None}
            )
            if detection is not None:
                state["last_detected"] = now
                state["target"] = detection
            elif (
                state["last_detected"] is not None
                and now - state["last_detected"] < self.parameters.state_clear_time
            ):
                detection = state["target"]
            else:
                state.update(last_detected=None, target=None)

            result.markers.extend(self._markers(output_path.header, area, detection))
            if detection is None:
                active.append("{}:GO".format(area.regulatory_id))
                continue

            stop_station = nominal_stop
            speed = ego_speed(planner_data.odometry)
            required = speed * self.parameters.delay_response_time + speed**2 / (
                2.0 * max(self.parameters.max_deceleration, 1.0e-3)
            )
            distance = nominal_stop - current_station
            already_stopped = speed < 0.01
            if (
                already_stopped
                and distance < self.parameters.hold_stop_margin_distance
                and distance > -self.parameters.distance_to_judge_over_stop_line
            ):
                stop_station = current_station
            elif required > max(distance, 0.0) and not (
                self.parameters.suppress_pass_judge_when_stopping and speed < 0.5
            ):
                if self.parameters.unstoppable_policy == "go":
                    active.append("{}:PASS_UNSTOPPABLE".format(area.regulatory_id))
                    continue
                if self.parameters.unstoppable_policy == "stop_after_stopline":
                    stop_station = min(
                        max(current_station + required, nominal_stop),
                        path_xyz and self._path_length(path_xyz) or nominal_stop,
                    )

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            target_point = detection["point"]
            detail = "regulatory_id={} source={} policy={}".format(
                area.regulatory_id,
                detection["source"],
                self.parameters.unstoppable_policy,
            )
            result.modified = True
            result.stop_reasons.append(
                make_stop_reason(
                    StopReason.DETECTION_AREA,
                    stop_pose,
                    stop_station - current_station,
                    [target_point],
                )
            )
            result.planning_factors.append(
                make_planning_factor(
                    self.module_name,
                    output_path.header,
                    stop_pose,
                    stop_station - current_station,
                    PlanningFactor.STOP,
                    detail,
                    detected_object=detection.get("object"),
                    safety_points=[target_point],
                    safety_type=(
                        SafetyFactor.POINTCLOUD
                        if detection["source"] == "pointcloud"
                        else SafetyFactor.OBJECT
                    ),
                )
            )
            active.append("{}:STOP({})".format(area.regulatory_id, detection["source"]))

        result.status = (
            "detection_area: " + ", ".join(active)
            if active
            else "detection_area: no regulatory area on path"
        )
        return result

    def _detect(self, area, planner_data):
        if self.parameters.pointcloud:
            for cloud in (
                planner_data.no_ground_pointcloud,
                planner_data.compare_map_filtered_pointcloud,
            ):
                if cloud is None:
                    continue
                for point in point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True
                ):
                    xyz = (float(point[0]), float(point[1]), float(point[2]))
                    if any(self._inside(xyz, polygon) for polygon in area.polygons):
                        return {"source": "pointcloud", "point": xyz, "object": None}
        for detected in getattr(planner_data.dynamic_objects, "objects", []):
            if object_label(detected) not in self.parameters.target_labels:
                continue
            xyz = object_position(detected)
            if any(self._inside(xyz, polygon) for polygon in area.polygons):
                return {"source": "object", "point": xyz, "object": detected}
        return None

    def _markers(self, header, area, detection):
        output = []
        for index, polygon in enumerate(area.polygons):
            output.append(
                polygon_marker(
                    header,
                    "detection_area/polygon",
                    area.regulatory_id * 10 + index,
                    polygon,
                    (1.0, 0.65, 0.0, 0.28),
                )
            )
        output.append(
            line_marker(
                header,
                "detection_area/stop_line",
                area.regulatory_id,
                area.stop_line.points,
                (1.0, 0.1, 0.1, 1.0),
                0.45,
            )
        )
        center = area.polygons[0][0]
        label = "DETECTION AREA {}".format(area.regulatory_id)
        if detection is not None:
            label += " STOP"
            output.append(
                sphere_marker(
                    header,
                    "detection_area/obstacle",
                    area.regulatory_id,
                    detection["point"],
                    (1.0, 0.0, 0.0, 0.95),
                    0.9,
                )
            )
        output.append(
            text_marker(
                header,
                "detection_area/label",
                area.regulatory_id,
                center,
                label,
                (1.0, 0.9, 0.2, 1.0),
            )
        )
        return output

    @staticmethod
    def _inside(point, polygon):
        x, y = point[0], point[1]
        inside = False
        previous = polygon[-1]
        for current in polygon:
            if (previous[1] > y) != (current[1] > y):
                boundary = (current[0] - previous[0]) * (y - previous[1]) / (
                    current[1] - previous[1]
                ) + previous[0]
                if x < boundary:
                    inside = not inside
            previous = current
        return inside

    @staticmethod
    def _path_length(points):
        return sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(points[:-1], points[1:])
        )
