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
from autoware_perception_msgs.msg import ObjectClassification, PredictedPath
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    point_at_station,
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
    set_velocity_from_index,
    yaw_from_quaternion,
)


DECISION_SAFE = "Safe"
DECISION_STUCK = "StuckStop"
DECISION_YIELD_STUCK = "YieldStuckStop"
DECISION_COLLISION = "NonOccludedCollisionStop"
DECISION_FIRST_WAIT = "FirstWaitBeforeOcclusion"
DECISION_PEEKING = "PeekingTowardOcclusion"
DECISION_OCCLUDED_COLLISION = "OccludedCollisionStop"
DECISION_FULLY_PRIORITIZED = "FullyPrioritized"
DECISION_OVER_PASS = "OverPassJudgeLine"
DECISION_INTERNAL_ERROR = "InternalError"


@dataclass
class IntersectionParameters:
    attention_area_length: float = 200.0
    attention_area_margin: float = 0.75
    attention_area_angle_threshold: float = 0.785
    default_stopline_margin: float = 3.0
    pass_judge_line_margin: float = 1.0
    max_accel: float = -2.8
    max_jerk: float = -5.0
    base_link_to_front: float = 3.79
    base_link_to_rear: float = 1.10
    vehicle_width: float = 1.90
    footprint_search_step: float = 0.10
    collision_detection_hold_time: float = 1.0
    min_predicted_path_confidence: float = 0.05
    collision_start_margin_time: float = 3.0
    collision_end_margin_time: float = 2.0
    fully_prioritized_start_margin_time: float = 2.0
    fully_prioritized_end_margin_time: float = 0.0
    default_intersection_velocity: float = 2.778
    minimum_ego_velocity: float = 1.0
    target_unknown: bool = False
    target_car: bool = True
    target_truck: bool = True
    target_bus: bool = True
    target_trailer: bool = True
    target_motorcycle: bool = True
    target_bicycle: bool = True
    stuck_vehicle_velocity_threshold: float = 0.833
    stuck_vehicle_detect_distance: float = 5.0
    stuck_straight: bool = True
    stuck_left: bool = True
    stuck_right: bool = True
    yield_stuck_distance_threshold: float = 5.0
    yield_stuck_straight: bool = False
    yield_stuck_left: bool = True
    yield_stuck_right: bool = True
    occlusion_enable: bool = True
    occlusion_free_space_max: int = 43
    occlusion_occupied_min: int = 58
    occlusion_unknown_is_occupied: bool = False
    occlusion_detection_hold_time: float = 0.5
    temporal_stop_time_before_peeking: float = 1.0
    peeking_offset: float = 1.0
    creep_velocity: float = 1.0

    @classmethod
    def from_ros(cls, prefix="~intersection"):
        parameter = cls()
        names = {
            "attention_area_length": "common/attention_area_length",
            "attention_area_margin": "common/attention_area_margin",
            "attention_area_angle_threshold": "common/attention_area_angle_threshold",
            "default_stopline_margin": "common/default_stopline_margin",
            "pass_judge_line_margin": "common/pass_judge_line_margin",
            "max_accel": "common/max_accel",
            "max_jerk": "common/max_jerk",
            "collision_detection_hold_time": "collision_detection/collision_detection_hold_time",
            "min_predicted_path_confidence": "collision_detection/min_predicted_path_confidence",
            "collision_start_margin_time": "collision_detection/not_prioritized/collision_start_margin_time",
            "collision_end_margin_time": "collision_detection/not_prioritized/collision_end_margin_time",
            "fully_prioritized_start_margin_time": "collision_detection/fully_prioritized/collision_start_margin_time",
            "fully_prioritized_end_margin_time": "collision_detection/fully_prioritized/collision_end_margin_time",
            "default_intersection_velocity": "collision_detection/velocity_profile/default_velocity",
            "minimum_ego_velocity": "collision_detection/velocity_profile/minimum_default_velocity",
            "target_unknown": "collision_detection/target_type/unknown",
            "target_car": "collision_detection/target_type/car",
            "target_truck": "collision_detection/target_type/truck",
            "target_bus": "collision_detection/target_type/bus",
            "target_trailer": "collision_detection/target_type/trailer",
            "target_motorcycle": "collision_detection/target_type/motorcycle",
            "target_bicycle": "collision_detection/target_type/bicycle",
            "stuck_vehicle_velocity_threshold": "stuck_vehicle/stuck_vehicle_velocity_threshold",
            "stuck_vehicle_detect_distance": "stuck_vehicle/stuck_vehicle_detect_dist",
            "stuck_straight": "stuck_vehicle/turn_direction/straight",
            "stuck_left": "stuck_vehicle/turn_direction/left",
            "stuck_right": "stuck_vehicle/turn_direction/right",
            "yield_stuck_distance_threshold": "yield_stuck/distance_threshold",
            "yield_stuck_straight": "yield_stuck/turn_direction/straight",
            "yield_stuck_left": "yield_stuck/turn_direction/left",
            "yield_stuck_right": "yield_stuck/turn_direction/right",
            "occlusion_enable": "occlusion/enable",
            "occlusion_free_space_max": "occlusion/free_space_max",
            "occlusion_occupied_min": "occlusion/occupied_min",
            "occlusion_unknown_is_occupied": "occlusion/unknown_is_occupied",
            "occlusion_detection_hold_time": "occlusion/occlusion_detection_hold_time",
            "temporal_stop_time_before_peeking": "occlusion/temporal_stop_time_before_peeking",
            "peeking_offset": "occlusion/peeking_offset",
            "creep_velocity": "occlusion/creep_velocity_without_traffic_light",
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


class IntersectionModuleManager(SceneModuleManagerInterface):
    module_name = "intersection"

    def __init__(self, parameters=None):
        self.parameters = parameters or IntersectionParameters.from_ros()
        self.lanelet_map = None
        self.collision_last_seen = {}
        self.occlusion_first_seen = {}
        self.occlusion_stop_started = {}

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.reset()

    def reset(self):
        self.collision_last_seen.clear()
        self.occlusion_first_seen.clear()
        self.occlusion_stop_started.clear()

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "intersection: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "intersection: input path is too short"
            return result

        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        path_lengths = cumulative_lengths(path_xyz)
        route_lane_ids = {
            int(lane_id)
            for point in path_with_lane_id.points
            for lane_id in point.lane_ids
        }
        candidates = lanelet_map.intersection_lanelet_candidates(route_lane_ids)
        if not candidates:
            result.status = "intersection: no turn_direction lanelet on path"
            return result

        ego_station = self._ego_station(output_path, planner_data.odometry)
        now = float(planner_data.current_time)
        statuses = []
        for target in candidates:
            interval = self._lane_interval(path_with_lane_id, path_lengths, target.lanelet_id)
            if interval is None or interval[1] < ego_station - self.parameters.base_link_to_rear:
                continue
            conflicts = self._objective_conflicts(lanelet_map, target)
            if not conflicts:
                statuses.append("{}:{}".format(target.lanelet_id, DECISION_INTERNAL_ERROR))
                continue
            geometry = self._intersection_geometry(
                path_xyz, output_path, target, conflicts, interval, lanelet_map
            )
            if geometry is None:
                statuses.append("{}:{}".format(target.lanelet_id, DECISION_INTERNAL_ERROR))
                continue

            decision = self._decide(
                target,
                conflicts,
                geometry,
                path_xyz,
                output_path,
                ego_station,
                planner_data,
                lanelet_map,
                now,
            )
            statuses.append("{}:{}".format(target.lanelet_id, decision["state"]))
            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    target,
                    conflicts,
                    geometry,
                    decision,
                    lanelet_map,
                )
            )
            if decision["stop_station"] is None:
                continue

            stop_index, stop_pose = insert_path_point(
                output_path, decision["stop_station"]
            )
            if decision["state"] == DECISION_PEEKING:
                set_velocity_from_index(
                    output_path, stop_index, self.parameters.creep_velocity
                )
                peeking_index, peeking_pose = insert_path_point(
                    output_path, decision["peeking_station"]
                )
                set_velocity_from_index(output_path, peeking_index, 0.0)
                stop_index, stop_pose = peeking_index, peeking_pose
            else:
                set_velocity_from_index(output_path, stop_index, 0.0)

            distance = float(decision["stop_station"] - ego_station)
            factor_point = decision.get("factor_point") or geometry["entry_point"]
            result.stop_reasons.append(
                self._make_stop_reason(stop_pose, distance, factor_point)
            )
            result.planning_factors.append(
                self._make_planning_factor(
                    output_path.header,
                    stop_pose,
                    distance,
                    target,
                    decision,
                    factor_point,
                )
            )
            result.modified = True

        result.status = (
            "intersection: " + ", ".join(statuses)
            if statuses
            else "intersection: route intersection is behind ego"
        )
        return result

    def _decide(
        self,
        target,
        conflicts,
        geometry,
        path_xyz,
        output_path,
        ego_station,
        planner_data,
        lanelet_map,
        now,
    ):
        stop_station = geometry["default_stop_station"]
        if self._over_pass_judge_line(output_path, ego_station, stop_station, planner_data):
            return self._decision(DECISION_OVER_PASS)

        objects = list(getattr(planner_data.dynamic_objects, "objects", []) or [])
        stuck = self._stuck_object(
            objects, path_xyz, geometry, target.turn_direction
        )
        if stuck is not None:
            return self._decision(
                DECISION_STUCK,
                stop_station,
                "stuck stop",
                stuck,
                self._object_position(stuck),
            )

        yield_stuck = self._yield_stuck_object(
            objects, conflicts, geometry, target.turn_direction
        )
        if yield_stuck is not None:
            return self._decision(
                DECISION_YIELD_STUCK,
                stop_station,
                "yield stuck stop",
                yield_stuck,
                self._object_position(yield_stuck),
            )

        fully_prioritized = self._is_fully_prioritized(
            target, lanelet_map, planner_data.traffic_signals
        )
        collision = None
        if not self._straight_controlled_by_traffic_light(target, lanelet_map):
            collision = self._collision_object(
                objects,
                path_xyz,
                geometry,
                ego_station,
                fully_prioritized,
            )
        lane_id = target.lanelet_id
        if collision is not None:
            self.collision_last_seen[lane_id] = now
        elif now - self.collision_last_seen.get(lane_id, -1.0e9) <= self.parameters.collision_detection_hold_time:
            collision = {"object": None, "point": geometry["entry_point"], "ttc": 0.0}
        if collision is not None:
            return self._decision(
                DECISION_COLLISION,
                stop_station,
                "collision stop",
                collision.get("object"),
                collision.get("point"),
                collision.get("ttc"),
            )

        occlusion_state = self._occlusion_decision(
            target,
            conflicts,
            geometry,
            ego_station,
            planner_data,
            now,
        )
        if occlusion_state is not None:
            return occlusion_state
        if fully_prioritized:
            return self._decision(DECISION_FULLY_PRIORITIZED)
        return self._decision(DECISION_SAFE)

    @staticmethod
    def _decision(
        state,
        stop_station=None,
        detail="",
        detected_object=None,
        factor_point=None,
        ttc=0.0,
        peeking_station=None,
    ):
        return {
            "state": state,
            "stop_station": stop_station,
            "detail": detail,
            "object": detected_object,
            "factor_point": factor_point,
            "ttc": float(ttc),
            "peeking_station": peeking_station,
        }

    @staticmethod
    def _lane_interval(path, path_lengths, lanelet_id):
        indices = [
            index
            for index, point in enumerate(path.points)
            if int(lanelet_id) in [int(item) for item in point.lane_ids]
        ]
        if not indices:
            return None
        first = max(indices[0] - 1, 0)
        last = min(indices[-1] + 1, len(path_lengths) - 1)
        return float(path_lengths[first]), float(path_lengths[last])

    def _objective_conflicts(self, lanelet_map, target):
        ego_related = set(target.previous_ids)
        for previous_id in target.previous_ids:
            previous = lanelet_map.road_lanelets.get(previous_id)
            if previous is not None:
                ego_related.update(previous.following_ids)
        right_of_way_yields = set()
        for relation in lanelet_map.right_of_way_for_lanelet(target.lanelet_id):
            if target.lanelet_id in relation.right_of_way_lane_ids:
                right_of_way_yields.update(relation.yield_lane_ids)
        return [
            conflict
            for conflict in lanelet_map.conflicting_road_lanelets(target.lanelet_id)
            if conflict.lanelet_id not in ego_related
            and conflict.lanelet_id not in right_of_way_yields
        ]

    def _intersection_geometry(
        self, path_xyz, output_path, target, conflicts, interval, lanelet_map
    ):
        intersections = []
        for conflict in conflicts:
            values = polyline_polygon_intersections(path_xyz, conflict.polygon)
            intersections.extend(
                value
                for value in values
                if interval[0] - self.parameters.base_link_to_front
                <= value["station"]
                <= interval[1] + self.parameters.base_link_to_rear
            )
        if not intersections:
            return None
        first_center_entry = min(value["station"] for value in intersections)
        last_center_exit = max(value["station"] for value in intersections)
        footprint_entry = self._first_footprint_conflict_station(
            path_xyz,
            conflicts,
            max(0.0, interval[0] - self.parameters.base_link_to_front),
            min(cumulative_lengths(path_xyz)[-1], interval[1]),
        )
        if footprint_entry is None:
            footprint_entry = first_center_entry - self.parameters.base_link_to_front
        default_stop_station = max(
            0.0, footprint_entry - self.parameters.default_stopline_margin
        )
        entry_point = point_at_station(path_xyz, first_center_entry)
        area = lanelet_map.intersection_areas.get(target.intersection_area_id)
        return {
            "target_interval": interval,
            "first_conflict_station": float(first_center_entry),
            "last_conflict_station": float(last_center_exit),
            "footprint_entry_station": float(footprint_entry),
            "default_stop_station": float(default_stop_station),
            "default_stop_point": point_at_station(path_xyz, default_stop_station),
            "entry_point": entry_point,
            "intersection_area": area,
        }

    def _first_footprint_conflict_station(self, path_xyz, conflicts, start, end):
        station = float(start)
        step = max(float(self.parameters.footprint_search_step), 0.02)
        while station <= end + 1.0e-6:
            footprint = self._vehicle_footprint(path_xyz, station)
            if any(self._polygons_intersect(footprint, item.polygon) for item in conflicts):
                return station
            station += step
        return None

    def _vehicle_footprint(self, path_xyz, station):
        center = point_at_station(path_xyz, station)
        following = point_at_station(path_xyz, station + 0.05)
        yaw = math.atan2(following[1] - center[1], following[0] - center[0])
        half_width = 0.5 * self.parameters.vehicle_width
        local = (
            (self.parameters.base_link_to_front, half_width),
            (self.parameters.base_link_to_front, -half_width),
            (-self.parameters.base_link_to_rear, -half_width),
            (-self.parameters.base_link_to_rear, half_width),
        )
        return tuple(
            (
                center[0] + longitudinal * math.cos(yaw) - lateral * math.sin(yaw),
                center[1] + longitudinal * math.sin(yaw) + lateral * math.cos(yaw),
                center[2],
            )
            for longitudinal, lateral in local
        )

    def _stuck_object(self, objects, path_xyz, geometry, turn_direction):
        enabled = {
            "straight": self.parameters.stuck_straight,
            "left": self.parameters.stuck_left,
            "right": self.parameters.stuck_right,
        }.get(turn_direction, False)
        if not enabled:
            return None
        for detected in objects:
            if not self._is_target_object(detected):
                continue
            if self._object_speed(detected) > self.parameters.stuck_vehicle_velocity_threshold:
                continue
            position = self._object_position(detected)
            projection = project_point_to_polyline(path_xyz, position)
            if projection is None:
                continue
            station, distance = float(projection[0]), float(projection[1])
            width = max(float(detected.shape.dimensions.y), 1.0)
            if distance > 0.5 * (self.parameters.vehicle_width + width):
                continue
            if not (
                geometry["first_conflict_station"]
                <= station
                <= geometry["last_conflict_station"]
                + self.parameters.stuck_vehicle_detect_distance
            ):
                continue
            return detected
        return None

    def _yield_stuck_object(self, objects, conflicts, geometry, turn_direction):
        enabled = {
            "straight": self.parameters.yield_stuck_straight,
            "left": self.parameters.yield_stuck_left,
            "right": self.parameters.yield_stuck_right,
        }.get(turn_direction, False)
        if not enabled:
            return None
        entry = geometry["entry_point"]
        for detected in objects:
            if not self._is_target_object(detected):
                continue
            if self._object_speed(detected) > self.parameters.stuck_vehicle_velocity_threshold:
                continue
            position = self._object_position(detected)
            if not any(point_in_polygon(position, item.polygon) for item in conflicts):
                continue
            distance = math.hypot(position[0] - entry[0], position[1] - entry[1])
            if distance <= self.parameters.yield_stuck_distance_threshold:
                return detected
        return None

    def _collision_object(
        self, objects, path_xyz, geometry, ego_station, fully_prioritized
    ):
        start_margin = (
            self.parameters.fully_prioritized_start_margin_time
            if fully_prioritized
            else self.parameters.collision_start_margin_time
        )
        end_margin = (
            self.parameters.fully_prioritized_end_margin_time
            if fully_prioritized
            else self.parameters.collision_end_margin_time
        )
        ego_velocity = max(
            self.parameters.default_intersection_velocity,
            self.parameters.minimum_ego_velocity,
        )
        ego_enter = max(
            geometry["first_conflict_station"] - ego_station, 0.0
        ) / ego_velocity
        ego_exit = max(
            geometry["last_conflict_station"] - ego_station, 0.0
        ) / ego_velocity
        for detected in objects:
            if not self._is_target_object(detected):
                continue
            for prediction in self._predicted_paths(detected):
                if float(prediction.confidence) < self.parameters.min_predicted_path_confidence:
                    continue
                passage = self._object_passage_interval(
                    detected, prediction, path_xyz, geometry
                )
                if passage is None:
                    continue
                object_enter, object_exit, collision_point = passage
                if object_exit + end_margin < ego_enter:
                    continue
                if object_enter - start_margin > ego_exit:
                    continue
                return {
                    "object": detected,
                    "point": collision_point,
                    "ttc": max(ego_enter, 0.0),
                }
        return None

    def _object_passage_interval(self, detected, prediction, path_xyz, geometry):
        width = max(float(detected.shape.dimensions.y), 0.5)
        threshold = 0.5 * (self.parameters.vehicle_width + width)
        time_step = self._duration_seconds(prediction.time_step)
        if time_step <= 1.0e-6:
            time_step = 0.5
        passages = []
        for index, pose in enumerate(prediction.path):
            position = (pose.position.x, pose.position.y, pose.position.z)
            projection = project_point_to_polyline(path_xyz, position)
            if projection is None:
                continue
            station, distance = float(projection[0]), float(projection[1])
            if distance > threshold:
                continue
            if not (
                geometry["first_conflict_station"] - self.parameters.base_link_to_rear
                <= station
                <= geometry["last_conflict_station"] + self.parameters.base_link_to_front
            ):
                continue
            passages.append((index * time_step, position))
        if not passages:
            return None
        return passages[0][0], passages[-1][0], passages[0][1]

    def _predicted_paths(self, detected):
        predictions = list(detected.kinematics.predicted_paths)
        if predictions:
            return sorted(predictions, key=lambda item: item.confidence, reverse=True)
        prediction = PredictedPath()
        prediction.confidence = 1.0
        prediction.time_step = rospy.Duration(0.5)
        pose = detected.kinematics.initial_pose_with_covariance.pose
        yaw = yaw_from_quaternion(pose.orientation)
        velocity = detected.kinematics.initial_twist_with_covariance.twist.linear
        for step in range(41):
            item = deepcopy(pose)
            elapsed = 0.5 * step
            item.position.x += (
                velocity.x * math.cos(yaw) - velocity.y * math.sin(yaw)
            ) * elapsed
            item.position.y += (
                velocity.x * math.sin(yaw) + velocity.y * math.cos(yaw)
            ) * elapsed
            prediction.path.append(item)
        return [prediction]

    def _occlusion_decision(
        self, target, conflicts, geometry, ego_station, planner_data, now
    ):
        if not self.parameters.occlusion_enable or planner_data.occupancy_grid is None:
            self.occlusion_first_seen.pop(target.lanelet_id, None)
            self.occlusion_stop_started.pop(target.lanelet_id, None)
            return None
        if not self._has_occluded_cell(planner_data.occupancy_grid, conflicts, geometry):
            self.occlusion_first_seen.pop(target.lanelet_id, None)
            self.occlusion_stop_started.pop(target.lanelet_id, None)
            return None
        lane_id = target.lanelet_id
        first_seen = self.occlusion_first_seen.setdefault(lane_id, now)
        if now - first_seen < self.parameters.occlusion_detection_hold_time:
            return None
        stop_station = geometry["default_stop_station"]
        stopped_near_line = (
            abs(stop_station - ego_station) <= 1.0
            and ego_speed(planner_data.odometry) <= 0.1
        )
        if stopped_near_line:
            stopped_since = self.occlusion_stop_started.setdefault(lane_id, now)
        else:
            self.occlusion_stop_started.pop(lane_id, None)
            stopped_since = now
        if now - stopped_since < self.parameters.temporal_stop_time_before_peeking:
            return self._decision(
                DECISION_FIRST_WAIT,
                stop_station,
                "occlusion first stop",
                factor_point=geometry["entry_point"],
            )
        peeking_station = max(
            stop_station,
            geometry["footprint_entry_station"] + self.parameters.peeking_offset,
        )
        return self._decision(
            DECISION_PEEKING,
            stop_station,
            "occlusion peeking stop",
            factor_point=geometry["entry_point"],
            peeking_station=peeking_station,
        )

    def _has_occluded_cell(self, grid, conflicts, geometry):
        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return False
        origin = grid.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        polygons = [item.polygon for item in conflicts]
        if geometry["intersection_area"] is not None:
            polygons.append(geometry["intersection_area"].points)
        for index, value in enumerate(grid.data):
            cell = int(value)
            occluded = (
                self.parameters.occlusion_free_space_max
                <= cell
                < self.parameters.occlusion_occupied_min
            )
            if cell < 0 and self.parameters.occlusion_unknown_is_occupied:
                occluded = True
            if not occluded:
                continue
            row, column = divmod(index, width)
            if row >= height:
                break
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            point = (
                origin.position.x + local_x * cos_yaw - local_y * sin_yaw,
                origin.position.y + local_x * sin_yaw + local_y * cos_yaw,
                origin.position.z,
            )
            if any(point_in_polygon(point, polygon) for polygon in polygons):
                return True
        return False

    def _over_pass_judge_line(self, output_path, ego_station, stop_station, planner_data):
        speed = ego_speed(planner_data.odometry)
        acceleration = 0.0
        if planner_data.acceleration is not None:
            acceleration = float(planner_data.acceleration.accel.accel.linear.x)
        braking_distance = self._braking_distance(speed, acceleration)
        pass_judge_station = max(
            0.0,
            stop_station - braking_distance - self.parameters.pass_judge_line_margin,
        )
        return ego_station > stop_station and ego_station > pass_judge_station

    def _braking_distance(self, speed, acceleration):
        speed = max(float(speed), 0.0)
        acceleration = min(float(acceleration), 0.0)
        jerk = min(float(self.parameters.max_jerk), -1.0e-3)
        target_acceleration = min(float(self.parameters.max_accel), -1.0e-3)
        ramp_time = max((target_acceleration - acceleration) / jerk, 0.0)
        distance = max(
            speed * ramp_time
            + 0.5 * acceleration * ramp_time**2
            + jerk * ramp_time**3 / 6.0,
            0.0,
        )
        velocity = max(
            speed + acceleration * ramp_time + 0.5 * jerk * ramp_time**2,
            0.0,
        )
        return distance + velocity**2 / (-2.0 * target_acceleration)

    def _is_target_object(self, detected):
        enabled = {
            ObjectClassification.UNKNOWN: self.parameters.target_unknown,
            ObjectClassification.CAR: self.parameters.target_car,
            ObjectClassification.TRUCK: self.parameters.target_truck,
            ObjectClassification.BUS: self.parameters.target_bus,
            ObjectClassification.TRAILER: self.parameters.target_trailer,
            ObjectClassification.MOTORCYCLE: self.parameters.target_motorcycle,
            ObjectClassification.BICYCLE: self.parameters.target_bicycle,
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
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    @staticmethod
    def _object_position(detected):
        position = detected.kinematics.initial_pose_with_covariance.pose.position
        return float(position.x), float(position.y), float(position.z)

    @staticmethod
    def _duration_seconds(duration):
        if hasattr(duration, "to_sec"):
            return float(duration.to_sec())
        return float(getattr(duration, "secs", 0.0)) + float(
            getattr(duration, "nsecs", 0.0)
        ) * 1.0e-9

    @staticmethod
    def _ego_station(path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _straight_controlled_by_traffic_light(target, lanelet_map):
        if target.turn_direction != "straight":
            return False
        return any(
            target.lanelet_id in traffic_light.lane_ids
            for traffic_light in lanelet_map.traffic_light_candidates()
        )

    @staticmethod
    def _is_fully_prioritized(target, lanelet_map, traffic_signals):
        if traffic_signals is None:
            return False
        relevant_groups = {
            light.regulatory_id
            for light in lanelet_map.traffic_light_candidates()
            if target.lanelet_id in light.lane_ids
        }
        for group in traffic_signals.traffic_light_groups:
            if int(group.traffic_light_group_id) not in relevant_groups:
                continue
            for element in group.elements:
                if int(element.color) == int(element.GREEN):
                    return True
        return False

    @staticmethod
    def _make_stop_reason(stop_pose, distance, factor_point):
        factor = StopFactor()
        factor.stop_pose = deepcopy(stop_pose)
        factor.dist_to_stop_pose = float(distance)
        factor.stop_factor_points = [IntersectionModuleManager._to_point(factor_point)]
        reason = StopReason()
        reason.reason = StopReason.INTERSECTION
        reason.stop_factors = [factor]
        return reason

    def _make_planning_factor(
        self, header, stop_pose, distance, target, decision, factor_point
    ):
        control = ControlPoint()
        control.pose = deepcopy(stop_pose)
        control.velocity = 0.0
        control.distance = float(distance)
        factor = PlanningFactor()
        factor.module = "intersection"
        factor.is_driving_forward = True
        factor.control_points = [control]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "lanelet_id={} {}".format(
            target.lanelet_id, decision["detail"]
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = False
        factor.safety_factors.detail = decision["state"]
        detected = decision.get("object")
        if detected is not None:
            safety = SafetyFactor()
            safety.type = SafetyFactor.OBJECT
            safety.object_id = deepcopy(detected.object_id)
            predictions = self._predicted_paths(detected)
            if predictions:
                safety.predicted_path = deepcopy(predictions[0])
            safety.ttc_begin = float(decision.get("ttc", 0.0))
            safety.ttc_end = float(decision.get("ttc", 0.0))
            safety.points = [self._to_point(factor_point)]
            safety.is_safe = False
            safety.detail = decision["state"]
            factor.safety_factors.factors = [safety]
        return factor

    def _make_markers(
        self, header, target, conflicts, geometry, decision, lanelet_map
    ):
        marker_id = int(target.lanelet_id % 2000000000)
        markers = []
        target_fill = self._polygon_marker(
            header,
            "bvp_intersection/target",
            marker_id,
            target.polygon,
            (0.15, 0.95, 0.35, 0.22),
        )
        markers.append(target_fill)
        for index, conflict in enumerate(conflicts):
            markers.append(
                self._polygon_marker(
                    header,
                    "bvp_intersection/conflict",
                    marker_id + 100 + index,
                    conflict.polygon,
                    (1.0, 0.45, 0.05, 0.16),
                )
            )
        if geometry["intersection_area"] is not None:
            markers.append(
                self._polygon_marker(
                    header,
                    "bvp_intersection/area",
                    marker_id + 500,
                    geometry["intersection_area"].points,
                    (1.0, 0.85, 0.0, 0.14),
                )
            )
        if decision["stop_station"] is not None:
            stop = self._base_marker(header, marker_id + 600, Marker.CUBE)
            stop.ns = "bvp_intersection/stop"
            stop.pose.position = self._to_point(geometry["default_stop_point"])
            stop.scale.x = 0.45
            stop.scale.y = 3.0
            stop.scale.z = 0.3
            stop.color.r = 1.0
            stop.color.g = 0.05
            stop.color.b = 0.15
            stop.color.a = 1.0
            markers.append(stop)
        detected = decision.get("object")
        if detected is not None:
            predictions = self._predicted_paths(detected)
            if predictions:
                path = self._base_marker(header, marker_id + 700, Marker.LINE_STRIP)
                path.ns = "bvp_intersection/object_prediction"
                path.scale.x = 0.35
                path.color.r = 1.0
                path.color.g = 0.1
                path.color.b = 0.1
                path.color.a = 1.0
                path.points = [
                    deepcopy(pose.position) for pose in predictions[0].path
                ]
                markers.append(path)
        text = self._base_marker(header, marker_id + 800, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_intersection/status"
        text.pose.position = self._to_point(geometry["entry_point"])
        text.pose.position.z += 2.5
        text.scale.z = 0.65
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "INTERSECTION {}\n{}".format(target.lanelet_id, decision["state"])
        markers.append(text)
        return markers

    @staticmethod
    def _polygon_marker(header, namespace, marker_id, polygon, color):
        marker = IntersectionModuleManager._base_marker(
            header, marker_id, Marker.TRIANGLE_LIST
        )
        marker.ns = namespace
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        anchor = polygon[0]
        for index in range(1, len(polygon) - 1):
            marker.points.extend(
                [
                    IntersectionModuleManager._to_point(anchor),
                    IntersectionModuleManager._to_point(polygon[index]),
                    IntersectionModuleManager._to_point(polygon[index + 1]),
                ]
            )
        return marker

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

    @staticmethod
    def _polygons_intersect(first, second):
        first_edges = list(zip(first, first[1:] + first[:1]))
        second_edges = list(zip(second, second[1:] + second[:1]))
        if any(
            segment_intersection(first_a, first_b, second_a, second_b) is not None
            for first_a, first_b in first_edges
            for second_a, second_b in second_edges
        ):
            return True
        return point_in_polygon(first[0], second) or point_in_polygon(second[0], first)
