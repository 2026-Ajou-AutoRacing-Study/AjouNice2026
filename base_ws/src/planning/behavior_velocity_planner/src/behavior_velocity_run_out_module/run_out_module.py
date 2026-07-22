import math
from dataclasses import dataclass, field

import rospy

from autoware_internal_planning_msgs.msg import PlanningFactor
from autoware_perception_msgs.msg import ObjectClassification
from tier4_planning_msgs.msg import StopReason

from behavior_velocity_planner_common.geometry import project_point_to_polyline
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
    object_speed,
    sphere_marker,
    text_marker,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_positions,
    set_velocity_between_indices,
    set_velocity_from_index,
)


@dataclass
class RunOutParameters:
    time_margin: float = 0.5
    slowdown_on_time_buffer: float = 0.1
    slowdown_off_time_buffer: float = 0.5
    slowdown_distance_buffer: float = 2.0
    slowdown_deceleration_limit: float = 4.0
    stop_on_time_buffer: float = 0.3
    stop_off_time_buffer: float = 1.0
    stop_distance_buffer: float = 2.5
    stop_deceleration_limit: float = 5.0
    stopped_velocity_threshold: float = 0.25
    min_slowdown_velocity: float = 1.0
    ego_width: float = 1.9
    base_link_to_front: float = 3.79
    expected_ego_acceleration: float = 0.7
    path_interpolation_ds: float = 0.1
    target_labels: set = field(
        default_factory=lambda: {
            ObjectClassification.PEDESTRIAN,
            ObjectClassification.BICYCLE,
            ObjectClassification.MOTORCYCLE,
        }
    )

    @classmethod
    def from_ros(cls, prefix="~run_out"):
        parameter = cls()
        key_map = {
            "time_margin": "collision/time_margin",
            "slowdown_on_time_buffer": "slowdown/on_time_buffer",
            "slowdown_off_time_buffer": "slowdown/off_time_buffer",
            "slowdown_distance_buffer": "slowdown/distance_buffer",
            "slowdown_deceleration_limit": "slowdown/deceleration_limit",
            "stop_on_time_buffer": "stop/on_time_buffer",
            "stop_off_time_buffer": "stop/off_time_buffer",
            "stop_distance_buffer": "stop/distance_buffer",
            "stop_deceleration_limit": "stop/deceleration_limit",
        }
        for name in parameter.__dataclass_fields__:
            if name == "target_labels":
                continue
            current = getattr(parameter, name)
            value = rospy.get_param(
                "{}/{}".format(prefix, key_map.get(name, name)), current
            )
            setattr(parameter, name, type(current)(value))
        names = rospy.get_param(
            "{}/objects/target_labels".format(prefix),
            ["PEDESTRIAN", "BICYCLE", "MOTORCYCLE"],
        )
        lookup = {
            "PEDESTRIAN": ObjectClassification.PEDESTRIAN,
            "BICYCLE": ObjectClassification.BICYCLE,
            "MOTORCYCLE": ObjectClassification.MOTORCYCLE,
        }
        parameter.target_labels = {
            lookup[name.upper()] for name in names if name.upper() in lookup
        }
        return parameter


class RunOutModuleManager(SceneModuleManagerInterface):
    """ROS 1 BVP adapter for Autoware's current Motion Velocity run-out plugin."""

    module_name = "run_out"

    def __init__(self, parameters=None):
        self.parameters = parameters or RunOutParameters.from_ros()
        self.first_collision_time = None
        self.last_collision_time = None
        self.last_candidate = None

    def reset(self):
        self.first_collision_time = None
        self.last_collision_time = None
        self.last_candidate = None

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        del path_with_lane_id
        result = SceneModuleResult(module_name=self.module_name)
        if len(output_path.points) < 2:
            result.status = "run_out: input path is too short"
            return result
        path_xyz = path_positions(output_path)
        current_station = ego_station(output_path, planner_data.odometry)
        speed = max(ego_speed(planner_data.odometry), 0.0)
        profile_velocity = max(
            float(point.longitudinal_velocity_mps) for point in output_path.points
        )
        now = float(planner_data.current_time)
        candidate = self._collision(
            path_xyz,
            current_station,
            speed,
            profile_velocity,
            planner_data,
        )
        if candidate is not None:
            if self.first_collision_time is None:
                self.first_collision_time = now
            self.last_collision_time = now
            self.last_candidate = candidate
        elif self.last_collision_time is not None:
            hold = max(
                self.parameters.slowdown_off_time_buffer,
                self.parameters.stop_off_time_buffer,
            )
            if now - self.last_collision_time <= hold:
                candidate = self.last_candidate
            else:
                self.reset()

        if candidate is None:
            result.status = "run_out: clear"
            return result

        elapsed = max(now - (self.first_collision_time or now), 0.0)
        collision_station = candidate["station"]
        distance = collision_station - current_station
        if elapsed >= self.parameters.stop_on_time_buffer:
            stop_station = max(
                current_station,
                collision_station
                - self.parameters.stop_distance_buffer
                - self.parameters.base_link_to_front,
            )
            index, pose = insert_path_point(output_path, stop_station)
            set_velocity_from_index(output_path, index, 0.0)
            required_deceleration = speed**2 / max(2.0 * (stop_station - current_station), 0.1)
            detail = "collision t_ego={:.2f}s t_object={:.2f}s decel={:.2f}mps2".format(
                candidate["ego_time"], candidate["object_time"], required_deceleration
            )
            result.modified = True
            result.stop_reasons.append(
                make_stop_reason(
                    StopReason.OBSTACLE_STOP,
                    pose,
                    stop_station - current_station,
                    [candidate["point"]],
                )
            )
            result.planning_factors.append(
                make_planning_factor(
                    self.module_name,
                    output_path.header,
                    pose,
                    stop_station - current_station,
                    PlanningFactor.STOP,
                    detail,
                    detected_object=candidate["object"],
                    safety_points=[candidate["point"]],
                    ttc_begin=candidate["ego_time"],
                    ttc_end=candidate["object_time"],
                )
            )
            decision = "STOP"
        elif elapsed >= self.parameters.slowdown_on_time_buffer:
            slow_station = max(current_station, collision_station - 15.0)
            start_index, pose = insert_path_point(output_path, slow_station)
            end_index, _ = insert_path_point(output_path, collision_station)
            target_velocity = max(
                self.parameters.min_slowdown_velocity,
                min(speed * 0.5, speed - 0.5),
            )
            set_velocity_between_indices(
                output_path, start_index, end_index, target_velocity
            )
            detail = "preventive slowdown t_ego={:.2f}s t_object={:.2f}s".format(
                candidate["ego_time"], candidate["object_time"]
            )
            result.modified = True
            result.planning_factors.append(
                make_planning_factor(
                    self.module_name,
                    output_path.header,
                    pose,
                    slow_station - current_station,
                    PlanningFactor.SLOW_DOWN,
                    detail,
                    velocity=target_velocity,
                    detected_object=candidate["object"],
                    safety_points=[candidate["point"]],
                    ttc_begin=candidate["ego_time"],
                    ttc_end=candidate["object_time"],
                )
            )
            decision = "SLOW_DOWN"
        else:
            decision = "MONITORING"

        result.markers.extend(self._markers(output_path.header, candidate, decision))
        result.status = "run_out: {} object={} delta_t={:.2f}s".format(
            decision,
            candidate["label"],
            abs(candidate["ego_time"] - candidate["object_time"]),
        )
        return result

    def _collision(
        self,
        path_xyz,
        current_station,
        ego_velocity,
        profile_velocity,
        planner_data,
    ):
        candidates = []
        # Autoware keeps collision geometry preprocessing internal. The continuous
        # projection below evaluates the full polyline without resampling the public
        # output path, which would only slow every downstream planner.
        for detected in getattr(planner_data.dynamic_objects, "objects", []):
            label = object_label(detected)
            if label not in self.parameters.target_labels:
                continue
            if object_speed(detected) < self.parameters.stopped_velocity_threshold:
                continue
            half_width = max(float(detected.shape.dimensions.y) * 0.5, 0.3)
            collision_width = self.parameters.ego_width * 0.5 + half_width
            predictions = detected.kinematics.predicted_paths
            if not predictions:
                predictions = [self._linear_prediction(detected)]
            predictions = [
                max(predictions, key=lambda item: float(item.confidence))
            ]
            for prediction in predictions:
                time_step = max(prediction.time_step.to_sec(), 0.1)
                for index, pose in enumerate(prediction.path):
                    point = (
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                    )
                    projection = project_point_to_polyline(path_xyz, point)
                    if projection is None:
                        continue
                    station = float(projection[0])
                    lateral = float(projection[1])
                    if station <= current_station or lateral > collision_width:
                        continue
                    object_time = index * time_step
                    ego_time = self._ego_arrival_time(
                        station - current_station,
                        ego_velocity,
                        profile_velocity,
                    )
                    if abs(ego_time - object_time) <= self.parameters.time_margin:
                        candidates.append(
                            {
                                "station": station,
                                "point": point,
                                "ego_time": ego_time,
                                "object_time": object_time,
                                "object": detected,
                                "prediction": prediction,
                                "label": label,
                            }
                        )
                        break
        return min(candidates, key=lambda item: item["station"]) if candidates else None

    def _ego_arrival_time(self, distance, initial_velocity, target_velocity):
        """Approximate the velocity-smoother time law used by ROS 2 Autoware."""
        distance = max(float(distance), 0.0)
        initial_velocity = max(float(initial_velocity), 0.0)
        target_velocity = max(float(target_velocity), initial_velocity, 0.1)
        acceleration = max(self.parameters.expected_ego_acceleration, 1.0e-3)
        time_to_target = max((target_velocity - initial_velocity) / acceleration, 0.0)
        distance_to_target = (
            initial_velocity * time_to_target
            + 0.5 * acceleration * time_to_target * time_to_target
        )
        if distance <= distance_to_target:
            return (
                -initial_velocity
                + math.sqrt(initial_velocity * initial_velocity + 2.0 * acceleration * distance)
            ) / acceleration
        return time_to_target + (distance - distance_to_target) / target_velocity

    @staticmethod
    def _linear_prediction(detected):
        from autoware_perception_msgs.msg import PredictedPath
        from geometry_msgs.msg import Pose

        prediction = PredictedPath()
        prediction.confidence = 1.0
        prediction.time_step = rospy.Duration.from_sec(0.5)
        initial = object_position(detected)
        velocity = detected.kinematics.initial_twist_with_covariance.twist.linear
        for index in range(21):
            pose = Pose()
            pose.position.x = initial[0] + velocity.x * 0.5 * index
            pose.position.y = initial[1] + velocity.y * 0.5 * index
            pose.position.z = initial[2] + velocity.z * 0.5 * index
            pose.orientation.w = 1.0
            prediction.path.append(pose)
        return prediction

    @staticmethod
    def _markers(header, candidate, decision):
        prediction_points = [
            (pose.position.x, pose.position.y, pose.position.z)
            for pose in candidate["prediction"].path
        ]
        return [
            line_marker(
                header,
                "run_out/predicted_path",
                0,
                prediction_points,
                (1.0, 0.45, 0.05, 1.0),
                0.35,
            ),
            sphere_marker(
                header,
                "run_out/collision",
                0,
                candidate["point"],
                (1.0, 0.05, 0.05, 1.0),
                1.2,
            ),
            text_marker(
                header,
                "run_out/label",
                0,
                candidate["point"],
                "RUN OUT {}".format(decision),
                (1.0, 0.95, 0.2, 1.0),
            ),
        ]
