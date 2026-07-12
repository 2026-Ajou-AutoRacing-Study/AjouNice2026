#!/usr/bin/env python3

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is provided by ROS Noetic.
    yaml = None

try:
    from lanelet2 import core, geometry, io, projection, routing, traffic_rules

    LANELET2_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - reported clearly when the node starts.
    core = geometry = io = projection = routing = traffic_rules = None
    LANELET2_IMPORT_ERROR = exc


MGRS_GRID_SIZE = 100000.0


class RoutePlanningError(RuntimeError):
    pass


@dataclass
class RouteSectionData:
    preferred_id: int
    primitive_ids: list


@dataclass
class RoutePlan:
    path_ids: list
    route_lane_ids: set
    sections: list
    goal_position: tuple
    goal_yaw: object = None


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _first_osm_node(path):
    for _event, element in ET.iterparse(path, events=("end",)):
        if element.tag != "node":
            continue
        elevation = 0.0
        for child in element:
            if child.tag == "tag" and child.attrib.get("k") == "ele":
                elevation = float(child.attrib.get("v", 0.0))
                break
        return (
            float(element.attrib["lat"]),
            float(element.attrib["lon"]),
            elevation,
        )
    raise RoutePlanningError("Lanelet2 OSM does not contain a node")


def _projector_configuration(projector_type, projector_file):
    configured_type = str(projector_type or "MGRS")
    origin = None
    if projector_file:
        if yaml is None:
            raise RoutePlanningError("PyYAML is required to read the projector file")
        with open(projector_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        configured_type = str(config.get("projector_type", configured_type))
        map_origin = config.get("map_origin")
        if map_origin:
            origin = (
                float(map_origin["latitude"]),
                float(map_origin["longitude"]),
                float(map_origin.get("altitude", 0.0)),
            )
    return configured_type, origin


class Lanelet2RoutePlanner:
    """Lanelet2 route planner following Autoware mission-planner semantics."""

    def __init__(
        self,
        map_file,
        projector_type="MGRS",
        projector_file="",
        goal_angle_threshold_deg=45.0,
        enable_correct_goal_pose=False,
        consider_no_drivable_lanes=False,
        check_footprint_inside_lanes=True,
        vehicle_front_offset=3.79,
        vehicle_rear_offset=1.10,
        vehicle_left_offset=0.948,
        vehicle_right_offset=0.948,
        max_search_range=20.0,
        preferred_primitive_type="",
    ):
        if LANELET2_IMPORT_ERROR is not None:
            raise RoutePlanningError(
                "lanelet2 Python bindings are unavailable: {}. "
                "Install ros-noetic-lanelet2 and ros-noetic-lanelet2-python.".format(
                    LANELET2_IMPORT_ERROR
                )
            )
        if not map_file:
            raise RoutePlanningError("map_file is required")
        if not os.path.isfile(map_file):
            raise RoutePlanningError("map file does not exist: {}".format(map_file))

        self.map_file = map_file
        self.projector_type, configured_origin = _projector_configuration(
            projector_type, projector_file
        )
        self.goal_angle_threshold = math.radians(float(goal_angle_threshold_deg))
        self.enable_correct_goal_pose = bool(enable_correct_goal_pose)
        self.consider_no_drivable_lanes = bool(consider_no_drivable_lanes)
        self.check_footprint_inside_lanes = bool(check_footprint_inside_lanes)
        self.vehicle_front_offset = float(vehicle_front_offset)
        self.vehicle_rear_offset = float(vehicle_rear_offset)
        self.vehicle_left_offset = float(vehicle_left_offset)
        self.vehicle_right_offset = float(vehicle_right_offset)
        self.max_search_range = float(max_search_range)
        self.preferred_primitive_type = str(preferred_primitive_type)
        self.external_offset_x = 0.0
        self.external_offset_y = 0.0
        self.preferred_lane_ids = []
        self.route_lane_ids = set()

        if os.path.splitext(map_file)[1].lower() not in (".osm", ".xml"):
            raise RoutePlanningError(
                "this ROS1 port currently requires a Lanelet2 .osm/.xml map file"
            )

        first_node = _first_osm_node(map_file)
        origin_values = configured_origin or first_node
        origin = io.Origin(origin_values[0], origin_values[1])
        projector_name = self.projector_type.lower().replace("_", "")

        if projector_name == "mgrs":
            # Autoware MGRS coordinates are the UTM coordinates inside the current
            # 100 km grid square. Lanelet2 can build the graph in full UTM; only
            # external ROS poses need the deterministic grid offset.
            self.projector = projection.UtmProjector(origin, False, False)
            first_projected = self.projector.forward(
                core.GPSPoint(first_node[0], first_node[1], first_node[2])
            )
            self.external_offset_x = (
                math.floor(first_projected.x / MGRS_GRID_SIZE) * MGRS_GRID_SIZE
            )
            self.external_offset_y = (
                math.floor(first_projected.y / MGRS_GRID_SIZE) * MGRS_GRID_SIZE
            )
        elif projector_name in ("localcartesianutm", "localutm"):
            if configured_origin is None:
                raise RoutePlanningError(
                    "LocalCartesianUTM requires map_origin in projector_file"
                )
            self.projector = projection.UtmProjector(origin, True, False)
        elif projector_name == "utm":
            self.projector = projection.UtmProjector(origin, False, False)
        elif projector_name in ("localcartesian", "enu"):
            if configured_origin is None:
                raise RoutePlanningError("LocalCartesian requires map_origin in projector_file")
            self.projector = projection.LocalCartesianProjector(origin)
        else:
            raise RoutePlanningError(
                "unsupported projector_type '{}': use MGRS, UTM, "
                "LocalCartesianUTM, or LocalCartesian".format(self.projector_type)
            )

        self.map = io.load(map_file, self.projector)
        self.traffic_rules = traffic_rules.create(
            traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle
        )
        self.graph = routing.RoutingGraph(self.map, self.traffic_rules)
        validity_errors = list(self.graph.checkValidity(False))
        if validity_errors:
            raise RoutePlanningError(
                "Lanelet2 routing graph is invalid: {}".format(validity_errors[0])
            )

    @property
    def lanelet_count(self):
        return len(self.map.laneletLayer)

    def clear_route(self):
        self.preferred_lane_ids = []
        self.route_lane_ids = set()

    def commit_route(self, plan):
        self.preferred_lane_ids = list(plan.path_ids)
        self.route_lane_ids = set(plan.route_lane_ids)

    def commit_message_route(self, segments):
        self.preferred_lane_ids = [int(segment.preferred_primitive.id) for segment in segments]
        self.route_lane_ids = {
            int(primitive.id)
            for segment in segments
            for primitive in segment.primitives
            if int(primitive.id) != 0
        }

    def to_internal_point(self, pose):
        return core.BasicPoint2d(
            float(pose.position.x) + self.external_offset_x,
            float(pose.position.y) + self.external_offset_y,
        )

    def to_external_xyz(self, point):
        return (
            float(point.x) - self.external_offset_x,
            float(point.y) - self.external_offset_y,
            float(point.z),
        )

    def _is_road(self, lanelet):
        return dict(lanelet.attributes).get("subtype", "") == "road"

    def _nearby_road_lanelets(self, point):
        candidates = geometry.findWithin2d(
            self.map.laneletLayer, point, self.max_search_range
        )
        return [lanelet for _distance, lanelet in candidates if self._is_road(lanelet)]

    def _centerline_projection(self, lanelet, point):
        centerline = list(lanelet.centerline)
        if not centerline:
            raise RoutePlanningError("lanelet {} has no centerline".format(lanelet.id))
        if len(centerline) == 1:
            only = centerline[0]
            return (
                only.x,
                only.y,
                only.z,
                0.0,
                math.hypot(point.x - only.x, point.y - only.y),
                0,
                0.0,
            )

        best = None
        for segment_index, (start, end) in enumerate(zip(centerline[:-1], centerline[1:])):
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq < 1.0e-12:
                ratio = 0.0
            else:
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        ((point.x - start.x) * dx + (point.y - start.y) * dy)
                        / length_sq,
                    ),
                )
            x = start.x + ratio * dx
            y = start.y + ratio * dy
            z = start.z + ratio * (end.z - start.z)
            distance = math.hypot(point.x - x, point.y - y)
            candidate = (x, y, z, math.atan2(dy, dx), distance, segment_index, ratio)
            if best is None or candidate[4] < best[4]:
                best = candidate
        return best

    def lanelet_yaw(self, lanelet, point):
        return self._centerline_projection(lanelet, point)[3]

    def _closest_lanelet(self, lanelets, pose):
        point = self.to_internal_point(pose)
        pose_yaw = yaw_from_quaternion(pose.orientation)
        return min(
            lanelets,
            key=lambda lanelet: (
                self._centerline_projection(lanelet, point)[4],
                abs(normalize_angle(self.lanelet_yaw(lanelet, point) - pose_yaw)),
                lanelet.id,
            ),
        )

    def _start_lanelets(self, pose):
        point = self.to_internal_point(pose)
        candidates = self._nearby_road_lanelets(point)
        containing = [lanelet for lanelet in candidates if geometry.inside(lanelet, point)]
        if containing:
            return containing
        if not candidates:
            return []
        return [self._closest_lanelet(candidates, pose)]

    def _goal_lanelet(self, pose):
        point = self.to_internal_point(pose)
        candidates = self._nearby_road_lanelets(point)
        if not candidates:
            return None

        containing = [lanelet for lanelet in candidates if geometry.inside(lanelet, point)]
        selection = containing or candidates
        preferred = [
            lanelet for lanelet in selection if int(lanelet.id) in self.preferred_lane_ids
        ]
        if preferred:
            return self._closest_lanelet(preferred, pose)
        return self._closest_lanelet(selection, pose)

    def _plan_between(self, start_pose, goal_pose):
        start_lanelets = self._start_lanelets(start_pose)
        goal_lanelet = self._goal_lanelet(goal_pose)
        if not start_lanelets:
            raise RoutePlanningError("failed to find a road lanelet near the start checkpoint")
        if goal_lanelet is None:
            raise RoutePlanningError("failed to find a road lanelet near the goal checkpoint")

        start_point = self.to_internal_point(start_pose)
        start_yaw = yaw_from_quaternion(start_pose.orientation)
        best = None
        for start_lanelet in start_lanelets:
            lane_yaw = self.lanelet_yaw(start_lanelet, start_point)
            angle_difference = abs(normalize_angle(lane_yaw - start_yaw))
            if angle_difference > math.pi * 0.5:
                continue
            route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)
            if route is None:
                continue
            path = list(route.shortestPath())
            if self.consider_no_drivable_lanes and any(
                dict(lanelet.attributes).get("no_drivable_lane", "no") == "yes"
                for lanelet in path
            ):
                continue
            if int(start_lanelet.id) in self.preferred_lane_ids:
                return path
            cost = float(route.length2d()) + 1000.0 * angle_difference
            if best is None or cost < best[0]:
                best = (cost, path)

        if best is None:
            raise RoutePlanningError("Lanelet2 could not find a direction-valid route")
        return best[1]

    def _all_neighbors(self, lanelet):
        left = [relation.lanelet for relation in self.graph.leftRelations(lanelet)]
        right = [relation.lanelet for relation in self.graph.rightRelations(lanelet)]
        ordered = list(reversed(left)) + [lanelet] + right
        result = []
        seen = set()
        for neighbor in ordered:
            if int(neighbor.id) in seen:
                continue
            seen.add(int(neighbor.id))
            result.append(neighbor)
        return result

    def _candidate_connects(self, candidate_id, route_ids, candidate_ids, boundary_ids, backward):
        frontier = [self.map.laneletLayer[int(candidate_id)]]
        visited = {int(candidate_id)}
        while frontier:
            current = frontier.pop(0)
            next_lanelets = (
                self.graph.previous(current) if backward else self.graph.following(current)
            )
            for next_lanelet in next_lanelets:
                lanelet_id = int(next_lanelet.id)
                if lanelet_id in route_ids:
                    return True
                if lanelet_id in boundary_ids:
                    continue
                if lanelet_id in candidate_ids and lanelet_id not in visited:
                    visited.add(lanelet_id)
                    frontier.append(next_lanelet)
        return False

    def _route_lane_ids(self, path_lanelets):
        route_ids = {int(lanelet.id) for lanelet in path_lanelets}
        candidate_ids = set()
        for lanelet in path_lanelets:
            for relation in self.graph.rightRelations(lanelet):
                relation_name = str(relation.relationType)
                if relation_name == "Right":
                    route_ids.add(int(relation.lanelet.id))
                elif relation_name == "AdjacentRight":
                    candidate_ids.add(int(relation.lanelet.id))
            for relation in self.graph.leftRelations(lanelet):
                relation_name = str(relation.relationType)
                if relation_name == "Left":
                    route_ids.add(int(relation.lanelet.id))
                elif relation_name == "AdjacentLeft":
                    candidate_ids.add(int(relation.lanelet.id))

        start_ids = {int(lanelet.id) for lanelet in self._all_neighbors(path_lanelets[0])}
        goal_ids = {int(lanelet.id) for lanelet in self._all_neighbors(path_lanelets[-1])}
        for candidate_id in candidate_ids:
            if candidate_id in start_ids or candidate_id in goal_ids:
                continue
            connects_before = self._candidate_connects(
                candidate_id, route_ids, candidate_ids, start_ids, True
            )
            connects_after = self._candidate_connects(
                candidate_id, route_ids, candidate_ids, goal_ids, False
            )
            if connects_before and connects_after:
                route_ids.add(candidate_id)
        return route_ids

    def _make_sections(self, path_lanelets, route_ids):
        sections = []
        seen_primitives = set()
        for main_lanelet in path_lanelets:
            primitive_ids = [
                int(neighbor.id)
                for neighbor in self._all_neighbors(main_lanelet)
                if int(neighbor.id) in route_ids
            ]
            if int(main_lanelet.id) not in primitive_ids:
                primitive_ids.append(int(main_lanelet.id))

            # A lateral transition inside one parallel lane group is represented
            # by one Autoware RouteSection. The destination lane becomes the
            # preferred primitive instead of duplicating all parallel lanes in
            # consecutive sections.
            if sections and set(sections[-1].primitive_ids) == set(primitive_ids):
                sections[-1].preferred_id = int(main_lanelet.id)
                continue

            for primitive_id in primitive_ids:
                if primitive_id in seen_primitives:
                    raise RoutePlanningError("loop detected within the planned route")
                seen_primitives.add(primitive_id)
            sections.append(RouteSectionData(int(main_lanelet.id), primitive_ids))
        return sections

    def _near_goal_lanelets(self, goal_lanelet):
        result = [goal_lanelet]
        distance = 0.0
        current = goal_lanelet
        while distance < self.vehicle_rear_offset:
            previous = list(self.graph.previous(current))
            if not previous:
                break
            current = previous[0]
            result.insert(0, current)
            distance += float(geometry.length2d(current))
        distance = 0.0
        current = goal_lanelet
        while distance < self.vehicle_front_offset:
            following = list(self.graph.following(current))
            if not following:
                break
            current = following[0]
            result.append(current)
            distance += float(geometry.length2d(current))
        return result

    def _footprint_is_inside(self, pose, lanelets):
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        corners = (
            (self.vehicle_front_offset, self.vehicle_left_offset),
            (self.vehicle_front_offset, -self.vehicle_right_offset),
            (-self.vehicle_rear_offset, -self.vehicle_right_offset),
            (-self.vehicle_rear_offset, self.vehicle_left_offset),
        )
        for longitudinal, lateral in corners:
            external_x = pose.position.x + longitudinal * cos_yaw - lateral * sin_yaw
            external_y = pose.position.y + longitudinal * sin_yaw + lateral * cos_yaw
            point = core.BasicPoint2d(
                external_x + self.external_offset_x,
                external_y + self.external_offset_y,
            )
            if not any(geometry.inside(lanelet, point) for lanelet in lanelets):
                return False
        return True

    def _validated_goal(self, goal_pose, goal_lanelet):
        point = self.to_internal_point(goal_pose)
        projection_result = self._centerline_projection(goal_lanelet, point)
        corrected_yaw = None
        if self.enable_correct_goal_pose:
            external_x = projection_result[0] - self.external_offset_x
            external_y = projection_result[1] - self.external_offset_y
            corrected_yaw = projection_result[3]
            point = core.BasicPoint2d(projection_result[0], projection_result[1])
        else:
            external_x = float(goal_pose.position.x)
            external_y = float(goal_pose.position.y)

        goal_yaw = corrected_yaw
        if goal_yaw is None:
            goal_yaw = yaw_from_quaternion(goal_pose.orientation)
        lane_yaw = self.lanelet_yaw(goal_lanelet, point)
        if abs(normalize_angle(lane_yaw - goal_yaw)) >= self.goal_angle_threshold:
            raise RoutePlanningError("goal orientation does not match the goal lanelet")
        if not geometry.inside(goal_lanelet, point):
            raise RoutePlanningError("goal pose is outside the selected road lanelet")
        if self.check_footprint_inside_lanes and not self._footprint_is_inside(
            goal_pose, self._near_goal_lanelets(goal_lanelet)
        ):
            raise RoutePlanningError("goal vehicle footprint exceeds the road lanelets")

        refined = self._centerline_projection(goal_lanelet, point)
        return (external_x, external_y, float(refined[2])), corrected_yaw

    def plan(self, start_pose, waypoints, goal_pose):
        checkpoints = [start_pose] + list(waypoints) + [goal_pose]
        all_path = []
        for index in range(1, len(checkpoints)):
            path = self._plan_between(checkpoints[index - 1], checkpoints[index])
            for lanelet in path:
                if all_path and int(all_path[-1].id) == int(lanelet.id):
                    continue
                all_path.append(lanelet)
        if not all_path:
            raise RoutePlanningError("planned route is empty")

        route_ids = self._route_lane_ids(all_path)
        sections = self._make_sections(all_path, route_ids)
        goal_position, corrected_yaw = self._validated_goal(goal_pose, all_path[-1])
        return RoutePlan(
            path_ids=[int(lanelet.id) for lanelet in all_path],
            route_lane_ids=route_ids,
            sections=sections,
            goal_position=goal_position,
            goal_yaw=corrected_yaw,
        )

    def centerline_points(self, lane_ids):
        points = []
        for lane_id in lane_ids:
            try:
                lanelet = self.map.laneletLayer[int(lane_id)]
            except Exception:
                continue
            lane_points = [self.to_external_xyz(point) for point in lanelet.centerline]
            if points and lane_points:
                forward_distance = math.hypot(
                    points[-1][0] - lane_points[0][0], points[-1][1] - lane_points[0][1]
                )
                reverse_distance = math.hypot(
                    points[-1][0] - lane_points[-1][0], points[-1][1] - lane_points[-1][1]
                )
                if reverse_distance < forward_distance:
                    lane_points.reverse()
                if math.hypot(
                    points[-1][0] - lane_points[0][0], points[-1][1] - lane_points[0][1]
                ) < 1.0e-3:
                    lane_points = lane_points[1:]
            points.extend(lane_points)
        return points

    def route_change_distance(self, old_lane_ids, new_lane_ids, pose):
        if not old_lane_ids or not new_lane_ids:
            return 0.0
        point = self.to_internal_point(pose)
        old_lanelets = [self.map.laneletLayer[int(lane_id)] for lane_id in old_lane_ids]
        current_index = min(
            range(len(old_lanelets)),
            key=lambda index: self._centerline_projection(old_lanelets[index], point)[4],
        )
        current_id = int(old_lanelets[current_index].id)
        try:
            new_index = list(new_lane_ids).index(current_id)
        except ValueError:
            return 0.0

        distance = 0.0
        old_index = current_index
        while old_index < len(old_lane_ids) and new_index < len(new_lane_ids):
            if int(old_lane_ids[old_index]) != int(new_lane_ids[new_index]):
                break
            lanelet = self.map.laneletLayer[int(old_lane_ids[old_index])]
            if old_index == current_index:
                centerline = list(lanelet.centerline)
                total = float(geometry.length2d(lanelet))
                traversed = 0.0
                projection_result = self._centerline_projection(lanelet, point)
                target_segment = projection_result[5]
                target_ratio = projection_result[6]
                for segment_index, (start, end) in enumerate(
                    zip(centerline[:-1], centerline[1:])
                ):
                    segment = math.hypot(end.x - start.x, end.y - start.y)
                    if segment_index == target_segment:
                        traversed += target_ratio * segment
                        break
                    traversed += segment
                distance += max(0.0, total - traversed)
            else:
                distance += float(geometry.length2d(lanelet))
            old_index += 1
            new_index += 1
        if old_index == len(old_lane_ids) and new_index == len(new_lane_ids):
            return float("inf")
        return distance
