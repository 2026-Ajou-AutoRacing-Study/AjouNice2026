import math
from copy import deepcopy
from dataclasses import dataclass, field

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor
from autoware_perception_msgs.msg import TrafficLightElement
from tier4_planning_msgs.msg import StopFactor, StopReason

from behavior_velocity_planner_common.geometry import first_polyline_intersection
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
)


APPROACH = "APPROACH"
GO_OUT = "GO_OUT"
NOT_YELLOW = "NOT_YELLOW"
YELLOW_FROM_GREEN = "YELLOW_FROM_GREEN"
YELLOW_FROM_NON_GREEN = "YELLOW_FROM_NON_GREEN"


@dataclass
class TrafficLightParameters:
    stop_margin: float = 0.0
    tl_state_timeout: float = 1.0
    stop_time_hysteresis: float = 0.1
    yellow_lamp_period: float = 2.75
    yellow_light_stop_velocity: float = 1.0
    enable_pass_judge: bool = True
    enable_arrow_aware_yellow_passing: bool = False
    enable_rtc: bool = False
    min_behind_distance_to_stop: float = 0.5
    max_behind_distance_to_stop: float = 1.0
    v2i_use_remaining_time: bool = False
    v2i_last_time_allowed_to_pass: float = 2.0
    v2i_velocity_threshold: float = 0.5
    v2i_required_time_to_departure: float = 3.0
    base_link_to_front: float = 3.79
    max_detection_distance: float = 200.0
    passed_deadline: float = -2.0
    restart_length: float = 1.0
    max_stop_acceleration: float = -2.8
    max_stop_jerk: float = -5.0
    delay_response_time: float = 0.5
    is_simulation: bool = False

    @classmethod
    def from_ros(cls, prefix="~traffic_light"):
        parameter = cls()
        nested_fields = {
            "min_behind_distance_to_stop": "restart_suppression/min_behind_distance_to_stop",
            "max_behind_distance_to_stop": "restart_suppression/max_behind_distance_to_stop",
            "v2i_use_remaining_time": "v2i/use_remaining_time",
            "v2i_last_time_allowed_to_pass": "v2i/last_time_allowed_to_pass",
            "v2i_velocity_threshold": "v2i/velocity_threshold",
            "v2i_required_time_to_departure": "v2i/required_time_to_departure",
        }
        for field_name in parameter.__dataclass_fields__:
            current = getattr(parameter, field_name)
            ros_name = nested_fields.get(field_name, field_name)
            value = rospy.get_param("{}/{}".format(prefix, ros_name), current)
            if isinstance(current, bool):
                value = (
                    value.lower() in ("true", "1", "yes", "on")
                    if isinstance(value, str)
                    else bool(value)
                )
            setattr(parameter, field_name, type(current)(value))
        return parameter


@dataclass
class TrafficLightState:
    mode: str = APPROACH
    stop_signal_since: float = None
    previous_was_stop: bool = False
    has_received_signal: bool = False
    previous_elements: tuple = field(default_factory=tuple)
    yellow_transition: str = NOT_YELLOW


class TrafficLightModuleManager(SceneModuleManagerInterface):
    module_name = "traffic_light"

    def __init__(self, parameters=None):
        self.parameters = parameters or TrafficLightParameters.from_ros()
        self.lanelet_map = None
        self.states = {}

    def on_map_updated(self, lanelet_map):
        self.lanelet_map = lanelet_map
        self.reset()

    def reset(self):
        self.states = {}

    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        result = SceneModuleResult(module_name=self.module_name)
        lanelet_map = planner_data.lanelet_map or self.lanelet_map
        if lanelet_map is None:
            result.status = "traffic_light: waiting for Lanelet2 map"
            return result
        if len(path_with_lane_id.points) < 2 or len(output_path.points) < 2:
            result.status = "traffic_light: input path is too short"
            return result

        path_xyz = path_with_lane_id_positions(path_with_lane_id)
        ordered_lane_ids = []
        for path_point in path_with_lane_id.points:
            for lane_id in path_point.lane_ids:
                lane_id = int(lane_id)
                if lane_id not in ordered_lane_ids:
                    ordered_lane_ids.append(lane_id)
        ego_station = self._ego_station(output_path, planner_data.odometry)
        candidates = []
        seen_lane_lines = set()

        for traffic_light in lanelet_map.traffic_light_candidates():
            matching_lane_ids = [
                lane_id for lane_id in ordered_lane_ids if lane_id in traffic_light.lane_ids
            ]
            if not matching_lane_ids:
                continue
            lane_id = matching_lane_ids[0]
            lane_line_key = (lane_id, traffic_light.stop_line_id)
            if lane_line_key in seen_lane_lines:
                continue
            intersection = first_polyline_intersection(
                path_xyz, traffic_light.stop_line_points
            )
            if intersection is None:
                continue
            distance = float(intersection["station"]) - ego_station
            if distance > self.parameters.max_detection_distance:
                continue
            seen_lane_lines.add(lane_line_key)
            candidates.append(
                (float(intersection["station"]), lane_id, traffic_light, intersection)
            )

        candidates.sort(key=lambda item: item[0])
        statuses = []
        for collision_station, lane_id, traffic_light, intersection in candidates:
            state = self.states.setdefault(lane_id, TrafficLightState())
            stop_station = max(
                0.0,
                collision_station
                - self.parameters.stop_margin
                - self.parameters.base_link_to_front,
            )
            distance_to_stop = stop_station - ego_station

            if state.mode == APPROACH and distance_to_stop < self.parameters.passed_deadline:
                state.mode = GO_OUT
                state.stop_signal_since = None
                state.previous_was_stop = False
                statuses.append("{}:GO_OUT".format(traffic_light.regulatory_id))
                continue
            if state.mode == GO_OUT:
                if distance_to_stop > self.parameters.restart_length:
                    state.mode = APPROACH
                else:
                    statuses.append("{}:GO_OUT".format(traffic_light.regulatory_id))
                    continue

            should_stop, signal_label, signal = self._signal_requires_stop(
                traffic_light,
                lane_id,
                state,
                planner_data.traffic_signals,
                float(planner_data.current_time),
            )
            if result.traffic_signal is None and signal is not None:
                result.traffic_signal = deepcopy(signal)

            if should_stop:
                if state.stop_signal_since is None:
                    state.stop_signal_since = float(planner_data.current_time)
                stop_duration = max(
                    float(planner_data.current_time) - state.stop_signal_since, 0.0
                )
                to_be_stopped = state.previous_was_stop or (
                    stop_duration > self.parameters.stop_time_hysteresis
                )
            else:
                state.stop_signal_since = None
                state.previous_was_stop = False
                to_be_stopped = False

            if self.parameters.v2i_use_remaining_time and signal is not None:
                to_be_stopped = to_be_stopped or self._v2i_requires_stop(
                    signal,
                    traffic_light.turn_direction(lane_id),
                    distance_to_stop,
                    planner_data,
                )

            stop_pose = None
            if to_be_stopped:
                restart_station = self._restart_suppression_station(
                    distance_to_stop, ego_station, planner_data.odometry
                )
                target_station = (
                    restart_station if restart_station is not None else stop_station
                )
                if not self._is_passthrough(distance_to_stop, state, planner_data):
                    stop_index, stop_pose = insert_path_point(output_path, target_station)
                    set_velocity_from_index(output_path, stop_index, 0.0)
                    state.previous_was_stop = True
                    result.modified = True
                    result.stop_reasons.append(
                        self._make_stop_reason(
                            stop_pose,
                            target_station - ego_station,
                            self._to_point(intersection["point"]),
                        )
                    )
                    result.planning_factors.append(
                        self._make_planning_factor(
                            output_path.header,
                            stop_pose,
                            target_station - ego_station,
                            traffic_light,
                            lane_id,
                            signal_label,
                        )
                    )
                    result.markers.extend(
                        self._make_markers(
                            output_path.header,
                            traffic_light,
                            intersection["point"],
                            stop_pose,
                            signal_label,
                        )
                    )
                    result.status = (
                        "traffic_light: group={} lane={} signal={} stop_distance={:.2f}m"
                    ).format(
                        traffic_light.regulatory_id,
                        lane_id,
                        signal_label,
                        target_station - ego_station,
                    )
                    return result
                signal_label += "_PASS_JUDGE"
                state.previous_was_stop = False

            result.markers.extend(
                self._make_markers(
                    output_path.header,
                    traffic_light,
                    intersection["point"],
                    stop_pose,
                    signal_label,
                )
            )
            statuses.append("{}:{}".format(traffic_light.regulatory_id, signal_label))

        result.status = (
            "traffic_light: " + ", ".join(statuses)
            if statuses
            else "traffic_light: no traffic light on path"
        )
        return result

    def _signal_requires_stop(
        self, traffic_light, lane_id, state, message, current_time
    ):
        signal = self._find_signal(message, traffic_light.regulatory_id)
        if signal is None:
            should_stop = state.has_received_signal or not self.parameters.is_simulation
            return should_stop, "MISSING" if should_stop else "MISSING_SIM_PASS", None

        state.has_received_signal = True
        stamp = self._stamp_to_sec(getattr(message, "stamp", None))
        if current_time - stamp > self.parameters.tl_state_timeout:
            state.previous_elements = self._element_keys(signal.elements)
            return True, "TIMEOUT", signal

        current_elements = self._element_keys(signal.elements)
        is_yellow = any(
            color == TrafficLightElement.AMBER
            and status in (TrafficLightElement.SOLID_ON, 0)
            for color, _shape, status in current_elements
        )
        if is_yellow:
            if state.yellow_transition == NOT_YELLOW:
                previous_green = any(
                    color == TrafficLightElement.GREEN
                    and shape == TrafficLightElement.CIRCLE
                    for color, shape, _status in state.previous_elements
                )
                state.yellow_transition = (
                    YELLOW_FROM_GREEN if previous_green else YELLOW_FROM_NON_GREEN
                )
        else:
            state.yellow_transition = NOT_YELLOW

        if (
            is_yellow
            and self.parameters.enable_arrow_aware_yellow_passing
            and traffic_light.turn_direction(lane_id) in ("left", "right")
            and traffic_light.has_static_arrow
            and state.yellow_transition == YELLOW_FROM_GREEN
        ):
            state.previous_elements = current_elements
            return False, "AMBER_ARROW_PASS", signal

        should_stop = self.is_traffic_signal_stop(
            traffic_light.turn_direction(lane_id), signal.elements
        )
        label = self._signal_label(signal.elements)
        state.previous_elements = current_elements
        return should_stop, label, signal

    @staticmethod
    def _find_signal(message, regulatory_id):
        if message is None:
            return None
        return next(
            (
                group
                for group in message.traffic_light_groups
                if int(group.traffic_light_group_id) == int(regulatory_id)
            ),
            None,
        )

    @staticmethod
    def _stamp_to_sec(stamp):
        if stamp is None:
            return 0.0
        return float(stamp.to_sec())

    @staticmethod
    def _element_keys(elements):
        return tuple(
            (int(element.color), int(element.shape), int(element.status))
            for element in elements
        )

    @staticmethod
    def _contains(elements, shape, color):
        return any(
            int(element.shape) == int(shape) and int(element.color) == int(color)
            for element in elements
        )

    @classmethod
    def is_traffic_signal_stop(cls, turn_direction, elements):
        if cls._contains(elements, TrafficLightElement.CIRCLE, TrafficLightElement.GREEN):
            return False
        arrow_by_direction = {
            "right": TrafficLightElement.RIGHT_ARROW,
            "left": TrafficLightElement.LEFT_ARROW,
            "straight": TrafficLightElement.UP_ARROW,
        }
        arrow = arrow_by_direction.get(str(turn_direction))
        if arrow is not None and cls._contains(
            elements, arrow, TrafficLightElement.GREEN
        ):
            return False
        return True

    @staticmethod
    def _signal_label(elements):
        names = {
            TrafficLightElement.UNKNOWN: "UNKNOWN",
            TrafficLightElement.RED: "RED",
            TrafficLightElement.AMBER: "AMBER",
            TrafficLightElement.GREEN: "GREEN",
            TrafficLightElement.WHITE: "WHITE",
        }
        colors = [names.get(int(element.color), "COLOR_{}".format(element.color)) for element in elements]
        return "+".join(colors) if colors else "EMPTY"

    def _v2i_requires_stop(
        self, signal, turn_direction, distance_to_stop, planner_data
    ):
        velocity = ego_speed(planner_data.odometry)
        passing_time = (
            distance_to_stop / velocity
            if velocity > self.parameters.v2i_velocity_threshold
            else self.parameters.v2i_required_time_to_departure
        )
        threshold = passing_time + self.parameters.v2i_last_time_allowed_to_pass
        for prediction in signal.predictions:
            if not self._is_red_stop(
                turn_direction, prediction.simultaneous_elements
            ):
                continue
            remaining = self._stamp_to_sec(prediction.predicted_stamp) - float(
                planner_data.current_time
            )
            if remaining < threshold:
                return True
        return False

    @classmethod
    def _is_red_stop(cls, turn_direction, elements):
        if not cls._contains(elements, TrafficLightElement.CIRCLE, TrafficLightElement.RED):
            return False
        return cls.is_traffic_signal_stop(turn_direction, elements)

    def _is_passthrough(self, distance_to_stop, state, planner_data):
        velocity = ego_speed(planner_data.odometry)
        acceleration = 0.0
        if planner_data.acceleration is not None:
            acceleration = float(
                planner_data.acceleration.accel.accel.linear.x
            )
        judge_distance = self.calc_judge_line_distance(
            velocity,
            acceleration,
            self.parameters.max_stop_acceleration,
            self.parameters.max_stop_jerk,
            self.parameters.delay_response_time,
        )
        stoppable = (
            judge_distance < distance_to_stop
            or velocity < self.parameters.yellow_light_stop_velocity
        )
        reachable = distance_to_stop < velocity * self.parameters.yellow_lamp_period
        return (
            self.parameters.enable_pass_judge
            and not stoppable
            and not state.previous_was_stop
            and reachable
        )

    @staticmethod
    def calc_judge_line_distance(velocity, acceleration, max_acceleration, max_jerk, delay):
        velocity = float(velocity)
        if velocity <= 0.0:
            return 0.0
        x1 = velocity * float(delay)
        v2 = velocity + (
            float(max_acceleration) ** 2 - float(acceleration) ** 2
        ) / (2.0 * float(max_jerk))
        if v2 <= 0.0:
            discriminant = max(
                float(acceleration) ** 2
                - 2.0 * float(max_jerk) * velocity,
                0.0,
            )
            t2 = -(
                float(max_acceleration) + math.sqrt(discriminant)
            ) / float(max_jerk)
            x2 = (
                velocity * t2
                + float(acceleration) * t2**2 / 2.0
                + float(max_jerk) * t2**3 / 6.0
            )
            return max(0.0, x1 + x2)
        t2 = (
            float(max_acceleration) - float(acceleration)
        ) / float(max_jerk)
        x2 = (
            velocity * t2
            + float(acceleration) * t2**2 / 2.0
            + float(max_jerk) * t2**3 / 6.0
        )
        x3 = -(v2**2) / (2.0 * float(max_acceleration))
        return max(0.0, x1 + x2 + x3)

    def _restart_suppression_station(self, distance_to_stop, ego_station, odometry):
        if odometry is None or ego_speed(odometry) > 0.01:
            return None
        if (
            self.parameters.min_behind_distance_to_stop
            < distance_to_stop
            < self.parameters.max_behind_distance_to_stop
        ):
            return max(float(ego_station), 0.0)
        return None

    @staticmethod
    def _ego_station(output_path, odometry):
        if odometry is None:
            return 0.0
        projection = project_pose_to_path(output_path, odometry.pose.pose)
        return float(projection[0]) if projection is not None else 0.0

    @staticmethod
    def _to_point(value):
        point = Point()
        point.x = float(value[0])
        point.y = float(value[1])
        point.z = float(value[2]) if len(value) > 2 else 0.0
        return point

    @staticmethod
    def _make_stop_reason(stop_pose, distance, collision_point):
        factor = StopFactor()
        factor.stop_pose = deepcopy(stop_pose)
        factor.dist_to_stop_pose = float(distance)
        factor.stop_factor_points = [deepcopy(collision_point)]
        reason = StopReason()
        reason.reason = StopReason.TRAFFIC_LIGHT
        reason.stop_factors = [factor]
        return reason

    @staticmethod
    def _make_planning_factor(header, stop_pose, distance, traffic_light, lane_id, signal):
        control_point = ControlPoint()
        control_point.pose = deepcopy(stop_pose)
        control_point.velocity = 0.0
        control_point.shift_length = 0.0
        control_point.distance = float(distance)
        factor = PlanningFactor()
        factor.module = "traffic_light"
        factor.is_driving_forward = True
        factor.control_points = [control_point]
        factor.behavior = PlanningFactor.STOP
        factor.detail = "traffic_light_id={} lane_id={} signal={}".format(
            traffic_light.regulatory_id, lane_id, signal
        )
        factor.safety_factors.header = deepcopy(header)
        factor.safety_factors.is_safe = True
        factor.safety_factors.detail = "traffic signal stop"
        return factor

    def _make_markers(self, header, traffic_light, collision_xyz, stop_pose, signal):
        marker_id = int(traffic_light.regulatory_id % 2000000000)
        line = self._base_marker(header, marker_id, Marker.LINE_STRIP)
        line.ns = "bvp_traffic_light/stop_line"
        line.scale.x = 0.28
        line.color.r = 1.0
        line.color.g = 0.6
        line.color.a = 1.0
        line.points = [self._to_point(item) for item in traffic_light.stop_line_points]

        bulb = self._base_marker(header, marker_id + 1, Marker.SPHERE)
        bulb.ns = "bvp_traffic_light/signal"
        bulb.pose.position = self._to_point(collision_xyz)
        bulb.pose.position.z += 1.8
        bulb.scale.x = bulb.scale.y = bulb.scale.z = 0.9
        red, green, blue = self._signal_rgb(signal)
        bulb.color.r = red
        bulb.color.g = green
        bulb.color.b = blue
        bulb.color.a = 1.0

        markers = [line, bulb]
        if stop_pose is not None:
            stop = self._base_marker(header, marker_id + 2, Marker.SPHERE)
            stop.ns = "bvp_traffic_light/stop_pose"
            stop.pose = deepcopy(stop_pose)
            stop.scale.x = stop.scale.y = stop.scale.z = 0.8
            stop.color.r = 1.0
            stop.color.g = 0.05
            stop.color.b = 0.25
            stop.color.a = 1.0
            markers.append(stop)

        text = self._base_marker(header, marker_id + 3, Marker.TEXT_VIEW_FACING)
        text.ns = "bvp_traffic_light/status"
        text.pose.position = self._to_point(collision_xyz)
        text.pose.position.z += 3.0
        text.scale.z = 0.65
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "TRAFFIC LIGHT {}\n{}".format(
            traffic_light.regulatory_id, signal
        )
        markers.append(text)
        return markers

    @staticmethod
    def _signal_rgb(signal):
        signal = str(signal)
        if "RED" in signal or "TIMEOUT" in signal or "MISSING" in signal:
            return 1.0, 0.05, 0.05
        if "AMBER" in signal:
            return 1.0, 0.72, 0.0
        if "GREEN" in signal:
            return 0.1, 1.0, 0.25
        return 0.75, 0.75, 0.75

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
