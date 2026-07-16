from dataclasses import dataclass

import rospy

from autoware_internal_planning_msgs.msg import PlanningFactor
from tier4_planning_msgs.msg import StopReason
from tier4_v2x_msgs.msg import InfrastructureCommand, KeyValue

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
    route_lane_ids,
    text_marker,
)
from behavior_velocity_planner_common.path_utils import (
    ego_speed,
    insert_path_point,
    path_positions,
    set_velocity_from_index,
)


@dataclass
class VirtualTrafficLightParameters:
    max_delay_sec: float = 3.0
    near_line_distance: float = 1.0
    dead_line_margin: float = 1.0
    max_yaw_deviation_deg: float = 90.0
    check_timeout_after_stop_line: bool = True
    hold_stop_margin_distance: float = 0.0
    base_link_to_front: float = 3.79

    @classmethod
    def from_ros(cls, prefix="~virtual_traffic_light"):
        parameter = cls()
        for name in parameter.__dataclass_fields__:
            current = getattr(parameter, name)
            value = rospy.get_param("{}/{}".format(prefix, name), current)
            setattr(parameter, name, type(current)(value))
        return parameter


class VirtualTrafficLightModuleManager(SceneModuleManagerInterface):
    module_name = "virtual_traffic_light"

    def __init__(self, parameters=None):
        self.parameters = parameters or VirtualTrafficLightParameters.from_ros()
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
            result.status = "virtual_traffic_light: waiting for Lanelet2 map"
            return result
        if len(output_path.points) < 2:
            result.status = "virtual_traffic_light: input path is too short"
            return result

        route_ids = route_lane_ids(path_with_lane_id)
        path_xyz = path_positions(output_path)
        current_station = ego_station(output_path, planner_data.odometry)
        now = float(planner_data.current_time)
        active = []
        for element in lanelet_map.virtual_traffic_light_candidates():
            if element.lane_ids and route_ids.isdisjoint(element.lane_ids):
                continue
            start = self._station(path_xyz, element.start_line)
            end_candidates = [
                (self._station(path_xyz, line), line)
                for line in element.end_lines
            ]
            end_candidates = [item for item in end_candidates if item[0] is not None]
            if start is None or not end_candidates:
                continue
            end_station, end_line = min(end_candidates, key=lambda item: item[0])
            stop_station = self._station(path_xyz, element.stop_line)
            module_state = self.states.setdefault(element.regulatory_id, "NONE")

            if current_station < start - self.parameters.base_link_to_front:
                module_state = "NONE"
            elif current_station > end_station + self.parameters.dead_line_margin:
                module_state = "FINALIZED"
            elif module_state == "NONE":
                module_state = "REQUESTING"
            incoming = self._incoming_state(element, planner_data)
            command_state = getattr(InfrastructureCommand, module_state)
            decision = "GO"
            decision_station = None
            decision_line = None

            if module_state not in ("NONE", "FINALIZED") and stop_station is None:
                decision = "NO_STOP_LINE"
            elif module_state not in ("NONE", "FINALIZED"):
                fresh = incoming is not None and self._fresh(incoming, now)
                approved = incoming is not None and bool(incoming.approval)
                before_stop = current_station < (
                    stop_station - self.parameters.base_link_to_front
                    + self.parameters.dead_line_margin
                )
                if not fresh or not approved:
                    module_state = "REQUESTING"
                    command_state = InfrastructureCommand.REQUESTING
                    decision = "STOP_NO_APPROVAL" if fresh else "STOP_TIMEOUT"
                    decision_station = max(
                        0.0, stop_station - self.parameters.base_link_to_front
                    )
                    decision_line = element.stop_line
                elif before_stop:
                    module_state = "REQUESTING"
                    command_state = InfrastructureCommand.REQUESTING
                    decision = "APPROVED"
                else:
                    module_state = "PASSING"
                    command_state = InfrastructureCommand.PASSING
                    if self.parameters.check_timeout_after_stop_line and not fresh:
                        decision = "STOP_TIMEOUT_AFTER_LINE"
                        decision_station = max(current_station, 0.0)
                        decision_line = element.stop_line
                    elif not bool(incoming.is_finalized):
                        decision = "STOP_FOR_FINALIZATION"
                        decision_station = max(
                            0.0, end_station - self.parameters.base_link_to_front
                        )
                        decision_line = end_line
                        distance_to_end = decision_station - current_station
                        if (
                            abs(distance_to_end) < self.parameters.near_line_distance
                            and ego_speed(planner_data.odometry) < 0.01
                        ):
                            module_state = "FINALIZING"
                            command_state = InfrastructureCommand.FINALIZING

            self.states[element.regulatory_id] = module_state
            result.infrastructure_commands.append(
                self._command(element, command_state, now, lanelet_map)
            )
            result.markers.extend(
                self._markers(
                    output_path.header,
                    element,
                    module_state,
                    incoming,
                )
            )
            if decision_station is not None:
                stop_index, stop_pose = insert_path_point(output_path, decision_station)
                set_velocity_from_index(output_path, stop_index, 0.0)
                factor_points = decision_line.points if decision_line is not None else ()
                detail = "regulatory_id={} instrument_id={} state={} {}".format(
                    element.regulatory_id,
                    element.instrument.line_id,
                    module_state,
                    decision,
                )
                result.modified = True
                result.stop_reasons.append(
                    make_stop_reason(
                        StopReason.VIRTUAL_TRAFFIC_LIGHT,
                        stop_pose,
                        decision_station - current_station,
                        factor_points,
                    )
                )
                result.planning_factors.append(
                    make_planning_factor(
                        self.module_name,
                        output_path.header,
                        stop_pose,
                        decision_station - current_station,
                        PlanningFactor.STOP,
                        detail,
                        safety_points=factor_points,
                    )
                )
            active.append(
                "{}:{}({})".format(element.regulatory_id, module_state, decision)
            )

        result.status = (
            "virtual_traffic_light: " + ", ".join(active)
            if active
            else "virtual_traffic_light: no regulatory element on path"
        )
        return result

    @staticmethod
    def _station(path_xyz, line):
        if line is None:
            return None
        crossing = first_polyline_intersection(path_xyz, line.points)
        return float(crossing["station"]) if crossing is not None else None

    @staticmethod
    def _incoming_state(element, planner_data):
        states = getattr(planner_data.virtual_traffic_light_states, "states", [])
        target_id = str(element.instrument.line_id)
        for state in states:
            if str(state.id) == target_id:
                return state
        return None

    def _fresh(self, state, now):
        stamp = state.stamp.to_sec()
        return stamp > 0.0 and now - stamp <= self.parameters.max_delay_sec

    @staticmethod
    def _command(element, state, now, lanelet_map):
        command = InfrastructureCommand()
        command.stamp = rospy.Time.from_sec(max(now, 0.0))
        command.type = str(element.instrument.tags.get("type", "virtual_traffic_light"))
        command.id = str(element.instrument.line_id)
        command.state = int(state)
        tags = []
        for key, value in sorted(element.instrument.tags.items()):
            if key == "type":
                continue
            tags.append(KeyValue(key=str(key), value=str(value)))
        if element.lane_ids:
            lane_id = element.lane_ids[0]
            tags.append(KeyValue(key="lane_id", value=str(lane_id)))
            lane = lanelet_map.road_lanelets.get(lane_id)
            direction = lane.turn_direction if lane is not None else "straight"
            tags.append(KeyValue(key="turn_direction", value=str(direction)))
        command.custom_tags = tags
        return command

    @staticmethod
    def _markers(header, element, state, incoming):
        approval = bool(incoming.approval) if incoming is not None else False
        label_point = element.instrument.points[0]
        output = [
            line_marker(
                header,
                "virtual_traffic_light/instrument",
                element.regulatory_id,
                element.instrument.points,
                (0.75, 0.2, 1.0, 1.0),
                0.6,
            ),
            line_marker(
                header,
                "virtual_traffic_light/start",
                element.regulatory_id,
                element.start_line.points,
                (0.15, 0.8, 1.0, 1.0),
                0.4,
            ),
        ]
        if element.stop_line is not None:
            output.append(
                line_marker(
                    header,
                    "virtual_traffic_light/stop",
                    element.regulatory_id,
                    element.stop_line.points,
                    (1.0, 0.15, 0.1, 1.0),
                    0.55,
                )
            )
        for index, line in enumerate(element.end_lines):
            output.append(
                line_marker(
                    header,
                    "virtual_traffic_light/end",
                    element.regulatory_id * 10 + index,
                    line.points,
                    (0.1, 1.0, 0.4, 1.0),
                    0.45,
                )
            )
        output.append(
            text_marker(
                header,
                "virtual_traffic_light/label",
                element.regulatory_id,
                label_point,
                "VTL {} approval={}".format(state, approval),
                (1.0, 1.0, 0.2, 1.0),
            )
        )
        return output
