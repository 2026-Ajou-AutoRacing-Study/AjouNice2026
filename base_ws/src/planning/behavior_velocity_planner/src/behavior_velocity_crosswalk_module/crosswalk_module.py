import math
from copy import deepcopy
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import (
    ControlPoint,
    PlanningFactor,
    SafetyFactor,
)
from autoware_perception_msgs.msg import ObjectClassification
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    first_polyline_intersection,
    point_in_polygon,
    polyline_polygon_intersections,
    project_point_to_polyline,
    segment_intersection,
)
from behavior_velocity_planner_common.interfaces import (
    SceneModuleManagerInterface,
    SceneModuleResult,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_with_lane_id_positions,
    project_pose_to_path,
    set_velocity_between_indices,
    set_velocity_from_index,
)


@dataclass
class CrosswalkParameters:
    stop_distance_from_crosswalk: float = 3.5
    stop_distance_from_object_preferred: float = 3.0
    stop_distance_from_crosswalk_limit: float = 3.0
    min_acc_preferred: float = -1.0
    min_jerk_preferred: float = -1.0
    crosswalk_attention_range: float = 2.0
    ego_pass_first_margin: float = 4.0
    ego_pass_later_margin: float = 13.0
    ego_pass_first_additional_margin: float = 0.5
    ego_pass_later_additional_margin: float = 0.5
    ego_min_assumed_speed: float = 2.0
    min_object_velocity: float = 1.39
    stop_object_velocity_threshold: float = 0.25
    min_slow_down_velocity: float = 2.78
    enable_no_stop_decision: bool = True
    min_acc_for_no_stop_decision: float = -1.5
    min_jerk_for_no_stop_decision: float = -1.5
    crossing_angle_threshold: float = 0.5
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    max_detection_distance: float = 200.0
    target_unknown: bool = True
    target_bicycle: bool = True
    target_motorcycle: bool = True
    target_pedestrian: bool = True

    @classmethod
    def from_ros(cls, prefix="~crosswalk"):
        parameter = cls()
        names = {
            "stop_distance_from_crosswalk": "stop_position/stop_distance_from_crosswalk",
            "stop_distance_from_object_preferred": "stop_position/stop_distance_from_object_preferred",
            "stop_distance_from_crosswalk_limit": "stop_position/stop_distance_from_crosswalk_limit",
            "min_acc_preferred": "stop_position/min_acc_preferred",
            "min_jerk_preferred": "stop_position/min_jerk_preferred",
            "crosswalk_attention_range": "object_filtering/crosswalk_attention_range",
            "ego_pass_first_margin": "pass_judge/ego_pass_first_margin",
            "ego_pass_later_margin": "pass_judge/ego_pass_later_margin",
            "ego_pass_first_additional_margin": "pass_judge/ego_pass_first_additional_margin",
            "ego_pass_later_additional_margin": "pass_judge/ego_pass_later_additional_margin",
            "ego_min_assumed_speed": "pass_judge/ego_min_assumed_speed",
            "min_object_velocity": "pass_judge/min_object_velocity",
            "stop_object_velocity_threshold": "pass_judge/stop_object_velocity_threshold",
            "min_slow_down_velocity": "slow_down/min_slow_down_velocity",
            "enable_no_stop_decision": "pass_judge/no_stop_decision/enable",
            "min_acc_for_no_stop_decision": "pass_judge/no_stop_decision/min_acc",
            "min_jerk_for_no_stop_decision": "pass_judge/no_stop_decision/min_jerk",
            "crossing_angle_threshold": "object_filtering/crossing_angle_threshold",
            "target_unknown": "object_filtering/target_object/unknown",
            "target_bicycle": "object_filtering/target_object/bicycle",
            "target_motorcycle": "object_filtering/target_object/motorcycle",
            "target_pedestrian": "object_filtering/target_object/pedestrian",
        }
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            value = rospy.get_param(
                "{}/{}".format(prefix, names.get(field_name, field_name)), current
            )
            if isinstance(current, bool):
                value = (
                    value.lower() in ("true", "1", "yes", "on")
                    if isinstance(value, str)
                    else bool(value)
                )
            setattr(parameter, field_name, type(current)(value))
        return parameter


class CrosswalkModuleManager(SceneModuleManagerInterface):
    module_name = "crosswalk"

    def __init__(self, parameters=None):
        self.parameters = parameters or CrosswalkParameters.from_ros()
        self.lanelet_map = None

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "crosswalk: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "crosswalk: input path is too short"
            return result

        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        route_lane_ids = {
            int(lane_id)
            for point in path_with_lane_id.points
            for lane_id in point.lane_ids
        }
        ego_station = self._ego_station(output_path, planner_data.odometry)
        crossings = []
        for crossing in lanelet_map.pedestrian_crossing_candidates("crosswalk"):
            if crossing.road_lane_ids and route_lane_ids.isdisjoint(
                crossing.road_lane_ids
            ):
                continue
            intersections = polyline_polygon_intersections(path_xyz, crossing.points)
            if len(intersections) < 2:
                continue
            entry_station = float(intersections[0]["station"])
            if entry_station - ego_station > self.parameters.max_detection_distance:
                continue
            crossings.append(
                (entry_station, float(intersections[-1]["station"]), crossing)
            )

        crossings.sort(key=lambda item: item[0])
        statuses = []
        for entry_station, exit_station, crossing in crossings:
            default_stop_station = self._default_stop_station(
                crossing, path_xyz, entry_station
            )
            map_slowed = self._apply_map_slowdown(
                output_path, crossing, entry_station, exit_station, ego_station, result
            )
            collisions = self._yield_collisions(
                planner_data.dynamic_objects,
                path_xyz,
                crossing,
                entry_station,
                exit_station,
                ego_station,
                ego_speed(planner_data.odometry),
            )
            if not collisions:
                result.markers.extend(
                    self._make_markers(
                        output_path.header, crossing, None, None, "CLEAR"
                    )
                )
                statuses.append(
                    "{}:{}".format(crossing.lanelet_id, "SLOW" if map_slowed else "CLEAR")
                )
                continue

            nearest = min(collisions, key=lambda item: item["ego_station"])
            stop_station = self._select_stop_station(
                default_stop_station,
                entry_station,
                nearest["ego_station"],
                ego_station,
                planner_data,
            )
            if stop_station is None:
                statuses.append("{}:NO_STOP_DECISION".format(crossing.lanelet_id))
                continue

            stop_index, stop_pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, stop_index, 0.0)
            distance = stop_station - ego_station
            collision_point = self._to_point(nearest["point"])
            result.modified = True
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance, collision_point)
            )
            result.planning_factors.append(
                self._make_stop_factor(
                    output_path.header,
                    stop_pose,
                    distance,
                    crossing,
                    nearest,
                )
            )
            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    crossing,
                    stop_pose,
                    nearest["point"],
                    "YIELD",
                )
            )
            result.status = (
                "crosswalk: lanelet={} YIELD ttc={:.2f}s ttv={:.2f}s "
                "stop_distance={:.2f}m"
            ).format(
                crossing.lanelet_id,
                nearest["ttc"],
                nearest["ttv"],
                distance,
            )
            return result

        result.status = (
            "crosswalk: " + ", ".join(statuses)
            if statuses
            else "crosswalk: no crosswalk on path"
        )
        return result

    def _default_stop_station(self, crossing, path_xyz, entry_station):
        intersections = []
        for stop_line in crossing.stop_lines:
            item = first_polyline_intersection(path_xyz, stop_line.points)
            if item is not None:
                intersections.append(float(item["station"]))
        if intersections:
            return max(0.0, min(intersections) - self.parameters.base_link_to_front)
        return max(
            0.0,
            entry_station
            - self.parameters.stop_distance_from_crosswalk
            - self.parameters.base_link_to_front,
        )

    def _yield_collisions(
        self,
        objects_message,
        ego_path,
        crossing,
        entry_station,
        exit_station,
        ego_station,
        vehicle_speed,
    ):
        if objects_message is None:
            return []
        near = max(0.0, entry_station - self.parameters.crosswalk_attention_range)
        far = exit_station + self.parameters.crosswalk_attention_range
        collisions = []
        for detected in objects_message.objects:
            if not self._is_target_object(detected):
                continue
            collision = self._object_collision(detected, ego_path, crossing, near, far)
            if collision is None:
                continue
            distance_to_collision = max(
                0.0,
                collision["ego_station"]
                - ego_station
                + self.parameters.base_link_to_rear,
            )
            ttc = distance_to_collision / max(
                float(vehicle_speed), self.parameters.ego_min_assumed_speed
            )
            object_speed = self._object_speed(detected)
            ttv = collision["object_distance"] / max(
                object_speed, self.parameters.min_object_velocity
            )
            if (
                ttc
                + self.parameters.ego_pass_first_margin
                + self.parameters.ego_pass_first_additional_margin
                < ttv
            ):
                continue
            if (
                ttv
                + self.parameters.ego_pass_later_margin
                + self.parameters.ego_pass_later_additional_margin
                < ttc
            ):
                continue
            collision.update(
                {
                    "ttc": ttc,
                    "ttv": ttv,
                    "object": detected,
                    "object_speed": object_speed,
                }
            )
            collisions.append(collision)
        return collisions

    def _object_collision(self, detected, ego_path, crossing, near, far):
        pose = detected.kinematics.initial_pose_with_covariance.pose
        current = (pose.position.x, pose.position.y, pose.position.z)
        current_projection = project_point_to_polyline(ego_path, current)
        if point_in_polygon(current, crossing.points) and current_projection is not None:
            if near <= current_projection[0] <= far:
                return {
                    "ego_station": float(current_projection[0]),
                    "object_distance": 0.0,
                    "point": current_projection[4],
                }

        predicted_paths = list(detected.kinematics.predicted_paths)
        if not predicted_paths:
            predicted_paths = [self._linear_prediction(detected)]
        best = None
        ego_lengths = cumulative_lengths(ego_path)
        for predicted in predicted_paths:
            object_path = [
                (item.position.x, item.position.y, item.position.z)
                for item in predicted.path
            ]
            if len(object_path) < 2:
                continue
            object_lengths = cumulative_lengths(object_path)
            for object_index in range(len(object_path) - 1):
                object_segment_length = (
                    object_lengths[object_index + 1] - object_lengths[object_index]
                )
                object_yaw = math.atan2(
                    object_path[object_index + 1][1] - object_path[object_index][1],
                    object_path[object_index + 1][0] - object_path[object_index][0],
                )
                for ego_index in range(len(ego_path) - 1):
                    intersection = segment_intersection(
                        object_path[object_index],
                        object_path[object_index + 1],
                        ego_path[ego_index],
                        ego_path[ego_index + 1],
                    )
                    if intersection is None:
                        continue
                    object_ratio, ego_ratio, point = intersection
                    ego_station = ego_lengths[ego_index] + (
                        ego_lengths[ego_index + 1] - ego_lengths[ego_index]
                    ) * ego_ratio
                    if not near <= ego_station <= far:
                        continue
                    ego_yaw = math.atan2(
                        ego_path[ego_index + 1][1] - ego_path[ego_index][1],
                        ego_path[ego_index + 1][0] - ego_path[ego_index][0],
                    )
                    angle = abs(math.atan2(
                        math.sin(object_yaw - ego_yaw),
                        math.cos(object_yaw - ego_yaw),
                    ))
                    angle = min(angle, abs(math.pi - angle))
                    if angle < self.parameters.crossing_angle_threshold:
                        continue
                    object_distance = (
                        object_lengths[object_index]
                        + object_segment_length * object_ratio
                    )
                    candidate = {
                        "ego_station": float(ego_station),
                        "object_distance": float(object_distance),
                        "point": point,
                    }
                    if best is None or candidate["object_distance"] < best["object_distance"]:
                        best = candidate
        return best

    @staticmethod
    def _linear_prediction(detected):
        class Prediction:
            path = []

        prediction = Prediction()
        pose = detected.kinematics.initial_pose_with_covariance.pose
        twist = detected.kinematics.initial_twist_with_covariance.twist
        prediction.path = []
        for step in range(17):
            item = deepcopy(pose)
            time = 0.5 * step
            item.position.x += twist.linear.x * time
            item.position.y += twist.linear.y * time
            prediction.path.append(item)
        return prediction

    def _is_target_object(self, detected):
        enabled = {
            ObjectClassification.UNKNOWN: self.parameters.target_unknown,
            ObjectClassification.BICYCLE: self.parameters.target_bicycle,
            ObjectClassification.MOTORCYCLE: self.parameters.target_motorcycle,
            ObjectClassification.PEDESTRIAN: self.parameters.target_pedestrian,
        }
        labels = sorted(
            detected.classification,
            key=lambda item: float(item.probability),
            reverse=True,
        )
        return bool(labels and enabled.get(int(labels[0].label), False))

    @staticmethod
    def _object_speed(detected):
        velocity = detected.kinematics.initial_twist_with_covariance.twist.linear
        return math.hypot(float(velocity.x), float(velocity.y))

    def _select_stop_station(
        self,
        default_stop_station,
        entry_station,
        collision_station,
        ego_station,
        planner_data,
    ):
        preferred_station = min(
            default_stop_station,
            collision_station
            - self.parameters.stop_distance_from_object_preferred
            - self.parameters.base_link_to_front,
        )
        limit_station = (
            entry_station
            - self.parameters.stop_distance_from_crosswalk_limit
            - self.parameters.base_link_to_front
        )
        speed = ego_speed(planner_data.odometry)
        acceleration = 0.0
        if planner_data.acceleration is not None:
            acceleration = float(planner_data.acceleration.accel.accel.linear.x)
        weak_distance = self._deceleration_distance(
            speed,
            acceleration,
            self.parameters.min_acc_preferred,
            self.parameters.min_jerk_preferred,
        )
        preferred_distance = preferred_station - ego_station
        limit_distance = limit_station - ego_station
        if weak_distance < preferred_distance:
            selected_distance = preferred_distance
        elif weak_distance < limit_distance:
            selected_distance = weak_distance
        else:
            selected_distance = limit_distance
        if self.parameters.enable_no_stop_decision:
            strong_distance = self._deceleration_distance(
                speed,
                acceleration,
                self.parameters.min_acc_for_no_stop_decision,
                self.parameters.min_jerk_for_no_stop_decision,
            )
            if max(selected_distance, 0.1) < strong_distance:
                return None
        return max(0.0, ego_station + selected_distance)

    @staticmethod
    def _deceleration_distance(speed, acceleration, min_acceleration, min_jerk):
        speed = max(float(speed), 0.0)
        if speed <= 1.0e-6:
            return 0.0
        target_acceleration = min(float(min_acceleration), -1.0e-3)
        jerk = min(float(min_jerk), -1.0e-3)
        acceleration = min(float(acceleration), 0.0)
        ramp_time = max((target_acceleration - acceleration) / jerk, 0.0)
        ramp_distance = max(
            speed * ramp_time
            + 0.5 * acceleration * ramp_time**2
            + jerk * ramp_time**3 / 6.0,
            0.0,
        )
        speed_after_ramp = max(
            speed + acceleration * ramp_time + 0.5 * jerk * ramp_time**2,
            0.0,
        )
        return ramp_distance + speed_after_ramp**2 / (-2.0 * target_acceleration)

    def _apply_map_slowdown(
        self, output_path, crossing, entry_station, exit_station, ego_station, result
    ):
        if "safety_slow_down_speed" not in crossing.tags:
            return False
        velocity = max(float(crossing.tags["safety_slow_down_speed"]), 0.0)
        margin = self.parameters.base_link_to_front + float(
            crossing.tags.get("safety_slow_down_distance", 2.0)
        )
        start_index, start_pose = insert_path_point(
            output_path, max(0.0, entry_station - margin)
        )
        end_index, _ = insert_path_point(
            output_path, exit_station + self.parameters.base_link_to_rear
        )
        set_velocity_between_indices(output_path, start_index, end_index, velocity)
        result.modified = True
        result.planning_factors.append(
            self._make_slow_factor(
                output_path.header,
                start_pose,
                entry_station - margin - ego_station,
                crossing,
                velocity,
            )
        )
        return True

    @staticmethod
    def _ego_station(path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _make_stop_reason(stop_pose, distance, collision_point):
        stop_factor = StopFactor()
        stop_factor.stop_pose = deepcopy(stop_pose)
        stop_factor.dist_to_stop_pose = float(distance)
        stop_factor.stop_factor_points = [deepcopy(collision_point)]
        reason = StopReason()
        reason.reason = StopReason.CROSSWALK
        reason.stop_factors = [stop_factor]
        return reason

    def _make_stop_factor(
        self, header, stop_pose, distance, crossing, collision
    ):
        factor = PlanningFactor()
        factor.module = "crosswalk"
        factor.is_driving_forward = True
        factor.behavior = PlanningFactor.STOP
        factor.control_points = [self._control_point(stop_pose, 0.0, distance)]
        factor.detail = (
            "crosswalk_id={} object={} ttc={:.3f} ttv={:.3f}"
        ).format(
            crossing.lanelet_id,
            self._uuid_text(collision["object"].object_id),
            collision["ttc"],
            collision["ttv"],
        )
        safety = SafetyFactor()
        safety.type = SafetyFactor.OBJECT
        safety.object_id = deepcopy(collision["object"].object_id)
        safety.ttc_begin = float(collision["ttc"])
        safety.ttc_end = float(collision["ttc"])
        safety.points = [self._to_point(collision["point"])]
        safety.is_safe = False
        safety.detail = "crosswalk user"
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.factors = [safety]
        factor.safety_factors.is_safe = False
        factor.safety_factors.detail = "yield to crosswalk user"
        return factor

    def _make_slow_factor(self, header, pose, distance, crossing, velocity):
        factor = PlanningFactor()
        factor.module = "crosswalk"
        factor.is_driving_forward = True
        factor.behavior = PlanningFactor.SLOW_DOWN
        factor.control_points = [self._control_point(pose, velocity, distance)]
        factor.detail = "crosswalk_id={} map_slowdown={:.3f}".format(
            crossing.lanelet_id, velocity
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = True
        factor.safety_factors.detail = "Lanelet2 safety_slow_down_speed"
        return factor

    @staticmethod
    def _control_point(pose, velocity, distance):
        point = ControlPoint()
        point.pose = deepcopy(pose)
        point.velocity = float(velocity)
        point.distance = float(distance)
        point.shift_length = 0.0
        return point

    def _make_markers(self, header, crossing, stop_pose, collision, state):
        marker_id = int(crossing.lanelet_id % 2000000000)
        outline = self._base_marker(header, marker_id, Marker.LINE_STRIP)
        outline.ns = "bvp_crosswalk/polygon"
        outline.scale.x = 0.24
        outline.color.r = 0.15
        outline.color.g = 0.9
        outline.color.b = 1.0
        outline.color.a = 1.0
        outline.points = [self._to_point(item) for item in crossing.points]
        outline.points.append(self._to_point(crossing.points[0]))
        markers = [outline]
        if collision is not None:
            collision_marker = self._base_marker(
                header, marker_id + 1, Marker.SPHERE
            )
            collision_marker.ns = "bvp_crosswalk/collision"
            collision_marker.pose.position = self._to_point(collision)
            collision_marker.scale.x = collision_marker.scale.y = collision_marker.scale.z = 0.8
            collision_marker.color.r = 1.0
            collision_marker.color.g = 0.1
            collision_marker.color.b = 0.1
            collision_marker.color.a = 1.0
            markers.append(collision_marker)
        if stop_pose is not None:
            stop = self._base_marker(header, marker_id + 2, Marker.CUBE)
            stop.ns = "bvp_crosswalk/stop_pose"
            stop.pose = deepcopy(stop_pose)
            stop.scale.x = 0.5
            stop.scale.y = 2.5
            stop.scale.z = 0.25
            stop.color.r = 1.0
            stop.color.g = 0.0
            stop.color.b = 0.25
            stop.color.a = 1.0
            markers.append(stop)
        text = self._base_marker(header, marker_id + 3, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_crosswalk/status"
        center = crossing.points[len(crossing.points) // 2]
        text.pose.position = self._to_point(center)
        text.pose.position.z += 2.2
        text.scale.z = 0.65
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "CROSSWALK {}\n{}".format(crossing.lanelet_id, state)
        markers.append(text)
        return markers

    @staticmethod
    def _uuid_text(value):
        return "".join("{:02x}".format(int(item)) for item in value.uuid)

    @staticmethod
    def _to_point(value):
        point = Point()
        point.x = float(value[0])
        point.y = float(value[1])
        point.z = float(value[2]) if len(value) > 2 else 0.0
        return point

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
