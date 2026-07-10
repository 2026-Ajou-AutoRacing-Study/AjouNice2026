#!/usr/bin/env python3

import json
import math
import os
from collections import defaultdict, deque

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Path
from std_msgs.msg import Float32


def clamp(value, low, high):
    return max(low, min(high, value))


def distance2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def parse_link_ids(value):
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_from_points(points, index):
    if len(points) < 2:
        return 0.0

    current = points[index]
    if index < len(points) - 1:
        target = points[index + 1]
    else:
        target = points[index - 1]

    return math.atan2(target[1] - current[1], target[0] - current[0])


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MGeoTrajectoryGenerator:
    def __init__(self):
        self.link_set_path = rospy.get_param(
            "~link_set_path",
            "/home/lenovo/AjouNice2026/kcity_map/mgeo_map/link_set.json",
        )
        self.path_topic = rospy.get_param("~path_topic", "/recorded_trajectory_path")
        self.preview_path_topic = rospy.get_param("~preview_path_topic", "/mgeo_trajectory/preview_path")
        self.target_speed_topic = rospy.get_param("~target_speed_topic", "/recorded_trajectory/target_speed")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))

        self.route_link_ids = parse_link_ids(rospy.get_param("~route_link_ids", ""))
        self.start_link_id = rospy.get_param("~start_link_id", "")
        self.goal_link_id = rospy.get_param("~goal_link_id", "")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.auto_start_from_ego = as_bool(rospy.get_param("~auto_start_from_ego", True))
        self.auto_start_timeout = float(rospy.get_param("~auto_start_timeout", 5.0))
        self.auto_start_heading_weight = float(rospy.get_param("~auto_start_heading_weight", 8.0))
        self.nearest_point_stride = max(1, int(rospy.get_param("~nearest_point_stride", 5)))
        self.route_length_m = float(rospy.get_param("~route_length_m", 180.0))
        self.max_links = int(rospy.get_param("~max_links", 40))
        self.point_spacing = float(rospy.get_param("~point_spacing", 1.0))
        self.duplicate_distance = float(rospy.get_param("~duplicate_distance", 0.1))
        self.offset_x = float(rospy.get_param("~offset_x", 0.0))
        self.offset_y = float(rospy.get_param("~offset_y", 0.0))
        self.offset_z = float(rospy.get_param("~offset_z", 0.0))
        self.offset_yaw = math.radians(float(rospy.get_param("~offset_yaw_deg", 0.0)))
        self.ego_offset_x = float(rospy.get_param("~ego_offset_x", 0.0))
        self.ego_offset_y = float(rospy.get_param("~ego_offset_y", 0.0))
        self.ego_offset_z = float(rospy.get_param("~ego_offset_z", 0.0))
        self.preview_offset_x = float(rospy.get_param("~preview_offset_x", self.offset_x))
        self.preview_offset_y = float(rospy.get_param("~preview_offset_y", self.offset_y))
        self.preview_offset_z = float(rospy.get_param("~preview_offset_z", self.offset_z))
        self.preview_offset_yaw = math.radians(float(rospy.get_param("~preview_offset_yaw_deg", math.degrees(self.offset_yaw))))

        self.default_target_speed = float(rospy.get_param("~default_target_speed", 3.0))
        self.use_link_speed = as_bool(rospy.get_param("~use_link_speed", False))
        self.max_target_speed = float(rospy.get_param("~max_target_speed", 8.0))
        self.speed_unit_scale = float(rospy.get_param("~speed_unit_scale", 0.27778))

        self.links = self.load_links()
        self.links_by_id = {link["idx"]: link for link in self.links}
        self.outgoing_by_node = self.build_outgoing_map()
        self.start_link_point_index = 0
        self.selected_link_ids = self.select_route()
        self.points = self.resample_points(self.collect_points(self.selected_link_ids, self.transform_point))
        self.preview_points = self.resample_points(self.collect_points(self.selected_link_ids, self.transform_preview_point))
        self.target_speed = self.compute_target_speed(self.selected_link_ids)

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.preview_path_pub = rospy.Publisher(self.preview_path_topic, Path, queue_size=1, latch=True)
        self.speed_pub = rospy.Publisher(self.target_speed_topic, Float32, queue_size=1)

        rospy.loginfo(
            "MGeo trajectory loaded: links=%d selected=%d control_points=%d preview_points=%d speed=%.2f path=%s preview=%s",
            len(self.links),
            len(self.selected_link_ids),
            len(self.points),
            len(self.preview_points),
            self.target_speed,
            self.path_topic,
            self.preview_path_topic,
        )

    def load_links(self):
        if not os.path.exists(self.link_set_path):
            raise RuntimeError("MGeo link_set file does not exist: %s" % self.link_set_path)

        with open(self.link_set_path, "r", encoding="utf-8") as json_file:
            links = json.load(json_file)

        if not isinstance(links, list):
            raise RuntimeError("MGeo link_set must be a JSON list: %s" % self.link_set_path)

        valid_links = []
        for link in links:
            if not isinstance(link, dict):
                continue
            if link.get("idx") and isinstance(link.get("points"), list) and len(link["points"]) >= 2:
                valid_links.append(link)
        return valid_links

    def build_outgoing_map(self):
        outgoing = defaultdict(list)
        for link in self.links:
            outgoing[link.get("from_node_idx")].append(link["idx"])
        for node_links in outgoing.values():
            node_links.sort()
        return outgoing

    def select_route(self):
        if self.route_link_ids:
            missing = [link_id for link_id in self.route_link_ids if link_id not in self.links_by_id]
            if missing:
                raise RuntimeError("Unknown MGeo route link ids: %s" % ", ".join(missing))
            return self.route_link_ids

        start = self.start_link_id
        if not start and self.auto_start_from_ego:
            start, self.start_link_point_index = self.find_start_link_from_ego()
        if not start:
            start = self.links[0]["idx"] if self.links else ""
        if start not in self.links_by_id:
            raise RuntimeError("Unknown start_link_id: %s" % start)

        if self.goal_link_id:
            route = self.find_link_route(start, self.goal_link_id)
            if route:
                return route
            raise RuntimeError("No MGeo route found from %s to %s" % (start, self.goal_link_id))

        return self.follow_stable_successors(start, self.max_links, self.route_length_m)

    def find_start_link_from_ego(self):
        try:
            rospy.loginfo("Waiting for ego pose on %s for automatic MGeo route start", self.ego_topic)
            ego = rospy.wait_for_message(self.ego_topic, EgoVehicleStatus, timeout=self.auto_start_timeout)
        except Exception as exc:
            rospy.logwarn("Could not get ego pose for automatic route start: %s. Falling back to first MGeo link.", exc)
            return (self.links[0]["idx"] if self.links else ""), 0

        ego_point = (
            float(ego.position.x) + self.ego_offset_x,
            float(ego.position.y) + self.ego_offset_y,
            float(ego.position.z) + self.ego_offset_z,
        )
        ego_yaw = math.radians(float(ego.heading))

        best = None
        for link in self.links:
            points = self.transformed_link_points(link)
            for index in range(0, len(points), self.nearest_point_stride):
                point = points[index]
                if index < len(points) - 1:
                    link_yaw = yaw_from_points(points, index)
                else:
                    link_yaw = yaw_from_points(points, len(points) - 1)

                heading_error = abs(normalize_angle(ego_yaw - link_yaw))
                distance = distance2d(ego_point, point)
                cost = distance + self.auto_start_heading_weight * heading_error
                if heading_error > math.radians(100.0):
                    cost += 50.0

                if best is None or cost < best["cost"]:
                    best = {
                        "link_id": link["idx"],
                        "index": index,
                        "distance": distance,
                        "heading_error": heading_error,
                        "cost": cost,
                    }

        if best is None:
            return (self.links[0]["idx"] if self.links else ""), 0

        rospy.loginfo(
            "Auto-selected MGeo start link %s point=%d dist=%.2fm heading_error=%.1fdeg",
            best["link_id"],
            best["index"],
            best["distance"],
            math.degrees(best["heading_error"]),
        )
        return best["link_id"], best["index"]

    def find_link_route(self, start_link_id, goal_link_id):
        queue = deque([(start_link_id, [start_link_id])])
        visited = {start_link_id}
        while queue:
            link_id, route = queue.popleft()
            if link_id == goal_link_id:
                return route
            link = self.links_by_id[link_id]
            for next_link_id in self.outgoing_by_node.get(link.get("to_node_idx"), []):
                if next_link_id in visited:
                    continue
                visited.add(next_link_id)
                queue.append((next_link_id, route + [next_link_id]))
        return []

    def follow_first_successors(self, start_link_id, max_links):
        route = []
        visited = set()
        current = start_link_id
        while current and current not in visited and len(route) < max_links:
            route.append(current)
            visited.add(current)
            link = self.links_by_id[current]
            candidates = [item for item in self.outgoing_by_node.get(link.get("to_node_idx"), []) if item not in visited]
            current = candidates[0] if candidates else ""
        return route

    def follow_stable_successors(self, start_link_id, max_links, route_length_m):
        route = []
        visited = set()
        current = start_link_id
        total_length = 0.0
        while current and current not in visited and len(route) < max_links:
            route.append(current)
            visited.add(current)
            total_length += self.link_length(current)
            if route_length_m > 0.0 and total_length >= route_length_m:
                break

            link = self.links_by_id[current]
            candidates = [item for item in self.outgoing_by_node.get(link.get("to_node_idx"), []) if item not in visited]
            if not candidates:
                break

            current_yaw = self.link_end_yaw(current)
            current = min(candidates, key=lambda link_id: self.successor_cost(link_id, current_yaw))
        return route

    def successor_cost(self, link_id, previous_yaw):
        yaw_error = abs(normalize_angle(self.link_start_yaw(link_id) - previous_yaw))
        link = self.links_by_id[link_id]
        lane_change_penalty = 5.0 if str(link.get("lane_change_dir", "")).strip() not in ("", "0", "None", "none") else 0.0
        return yaw_error + lane_change_penalty

    def transformed_link_points(self, link):
        return [
            self.transform_point(float(point[0]), float(point[1]), float(point[2]))
            for point in link.get("points", [])
            if len(point) >= 3
        ]

    def link_start_yaw(self, link_id):
        points = self.transformed_link_points(self.links_by_id[link_id])
        return yaw_from_points(points, 0)

    def link_end_yaw(self, link_id):
        points = self.transformed_link_points(self.links_by_id[link_id])
        return yaw_from_points(points, len(points) - 1)

    def link_length(self, link_id):
        points = self.transformed_link_points(self.links_by_id[link_id])
        return sum(distance2d(points[index - 1], points[index]) for index in range(1, len(points)))

    def collect_points(self, link_ids, transform_fn):
        points = []
        for route_index, link_id in enumerate(link_ids):
            raw_points = self.links_by_id[link_id].get("points", [])
            if route_index == 0 and self.start_link_point_index > 0 and not self.start_link_id and not self.route_link_ids:
                raw_points = raw_points[max(0, self.start_link_point_index - 2) :]

            for raw_point in raw_points:
                if len(raw_point) < 3:
                    continue
                point = transform_fn(float(raw_point[0]), float(raw_point[1]), float(raw_point[2]))
                if points and distance2d(points[-1], point) < self.duplicate_distance:
                    continue
                points.append(point)
        return points

    def transform_with_offset(self, x, y, z, offset_x, offset_y, offset_z, offset_yaw):
        cos_yaw = math.cos(offset_yaw)
        sin_yaw = math.sin(offset_yaw)
        return (
            cos_yaw * x - sin_yaw * y + offset_x,
            sin_yaw * x + cos_yaw * y + offset_y,
            z + offset_z,
        )

    def transform_point(self, x, y, z):
        return self.transform_with_offset(x, y, z, self.offset_x, self.offset_y, self.offset_z, self.offset_yaw)

    def transform_preview_point(self, x, y, z):
        return self.transform_with_offset(
            x,
            y,
            z,
            self.preview_offset_x,
            self.preview_offset_y,
            self.preview_offset_z,
            self.preview_offset_yaw,
        )

    def resample_points(self, points):
        if self.point_spacing <= 0.0 or len(points) < 2:
            return points

        resampled = [points[0]]
        carry = 0.0
        last = points[0]
        spacing = self.point_spacing

        for next_point in points[1:]:
            segment = distance2d(last, next_point)
            if segment < 1e-6:
                last = next_point
                continue

            while carry + segment >= spacing:
                ratio = (spacing - carry) / segment
                new_point = (
                    last[0] + (next_point[0] - last[0]) * ratio,
                    last[1] + (next_point[1] - last[1]) * ratio,
                    last[2] + (next_point[2] - last[2]) * ratio,
                )
                resampled.append(new_point)
                last = new_point
                segment = distance2d(last, next_point)
                carry = 0.0
                if segment < 1e-6:
                    break

            carry += segment
            last = next_point

        if distance2d(resampled[-1], points[-1]) > self.duplicate_distance:
            resampled.append(points[-1])
        return resampled

    def compute_target_speed(self, link_ids):
        if not self.use_link_speed:
            return clamp(self.default_target_speed, 0.0, self.max_target_speed)

        speeds = []
        for link_id in link_ids:
            try:
                speed = float(self.links_by_id[link_id].get("max_speed", 0.0))
            except (TypeError, ValueError):
                speed = 0.0
            if speed > 0.0:
                speeds.append(speed * self.speed_unit_scale)

        if not speeds:
            return clamp(self.default_target_speed, 0.0, self.max_target_speed)
        return clamp(min(speeds), 0.0, self.max_target_speed)

    def build_path(self, points):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id

        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_from_points(points, index))
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            path.poses.append(pose)

        return path

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            path = self.build_path(self.points)
            preview_path = self.build_path(self.preview_points)
            self.path_pub.publish(path)
            self.preview_path_pub.publish(preview_path)
            self.speed_pub.publish(Float32(data=self.target_speed))
            rospy.loginfo_throttle(
                5.0,
                "Publishing MGeo trajectory: %d links, %d poses, route_start=%s, speed=%.2f",
                len(self.selected_link_ids),
                len(path.poses),
                self.selected_link_ids[0] if self.selected_link_ids else "none",
                self.target_speed,
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mgeo_trajectory_generator")
    try:
        MGeoTrajectoryGenerator().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("mgeo_trajectory_generator failed: %s", exc)
        raise
