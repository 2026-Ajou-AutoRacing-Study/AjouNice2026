#!/usr/bin/env python3

import csv
import glob
import math
import os

import rospy
import tf.transformations
from geometry_msgs.msg import Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, low, high):
    return max(low, min(high, value))


def norm2d(x, y):
    return math.sqrt(x * x + y * y)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_pose(pose):
    q = pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


class OnlineStats:
    def __init__(self):
        self.count = 0
        self.sum_sq = 0.0
        self.sum_abs = 0.0
        self.max_abs = 0.0

    def add(self, value):
        abs_value = abs(value)
        self.count += 1
        self.sum_sq += value * value
        self.sum_abs += abs_value
        self.max_abs = max(self.max_abs, abs_value)

    def rms(self):
        if self.count == 0:
            return 0.0
        return math.sqrt(self.sum_sq / float(self.count))

    def mean_abs(self):
        if self.count == 0:
            return 0.0
        return self.sum_abs / float(self.count)


class TrackingValidator:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/recorded_trajectory_path")
        self.target_speed_topic = rospy.get_param("~target_speed_topic", "/recorded_trajectory/target_speed")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.ctrl_topic = rospy.get_param("~ctrl_topic", "/ctrl_cmd")
        self.marker_topic = rospy.get_param("~marker_topic", "/validation/markers")
        self.output_csv = os.path.expanduser(
            rospy.get_param("~output_csv", "~/catkin_ws/src/ajou_control/validation_tracking.csv")
        )
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.6))
        self.steer_saturation_ratio = float(rospy.get_param("~steer_saturation_ratio", 0.95))
        self.nearest_search_window = int(rospy.get_param("~nearest_search_window", 120))
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 1.0))
        self.ego_offset_x = float(rospy.get_param("~ego_offset_x", 0.0))
        self.ego_offset_y = float(rospy.get_param("~ego_offset_y", 0.0))
        self.ego_offset_z = float(rospy.get_param("~ego_offset_z", 0.0))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.driven_path_decimation = max(1, int(rospy.get_param("~driven_path_decimation", 3)))
        self.history_csvs = self.resolve_history_csvs(
            rospy.get_param("~history_csvs", ""),
            rospy.get_param("~history_glob", ""),
        )

        self.path = None
        self.path_points = []
        self.path_yaws = []
        self.cumulative_s = []
        self.total_length = 0.0
        self.last_nearest_index = 0
        self.ego = None
        self.ctrl = None
        self.target_speed = 0.0

        self.lateral_stats = OnlineStats()
        self.heading_stats = OnlineStats()
        self.speed_stats = OnlineStats()
        self.ade_stats = OnlineStats()
        self.saturated_count = 0

        self.last_sample_time = None
        self.last_lat_vel = None
        self.last_long_vel = None
        self.sample_count = 0
        self.driven_points = []
        self.previous_runs = [self.load_history_csv(path) for path in self.history_csvs]
        self.previous_runs = [run for run in self.previous_runs if run]

        self.publishers = {}
        for topic in [
            "/validation/lateral/lat_err",
            "/validation/lateral/heading_err",
            "/validation/lateral/steering_angle",
            "/validation/lateral/lat_vel",
            "/validation/lateral/lat_acc",
            "/validation/longitudinal/long_vel",
            "/validation/longitudinal/target_vel",
            "/validation/longitudinal/long_vel_err",
            "/validation/longitudinal/long_acc",
            "/validation/metrics/mean_ade",
            "/validation/metrics/fde",
            "/validation/metrics/lateral_rms",
            "/validation/metrics/heading_rms",
            "/validation/metrics/speed_rms",
            "/validation/metrics/progress",
            "/validation/metrics/steer_saturation_rate",
            "/validation/lateral_error",
            "/validation/heading_error",
            "/validation/speed_error",
            "/validation/progress",
        ]:
            self.publishers[topic] = rospy.Publisher(topic, Float32, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.target_speed_topic, Float32, self.target_speed_callback, queue_size=1)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        rospy.Subscriber(self.ctrl_topic, CtrlCmd, self.ctrl_callback, queue_size=1)

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.csv_file = open(self.output_csv, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time",
                "nearest_index",
                "progress",
                "ego_x",
                "ego_y",
                "path_x",
                "path_y",
                "lateral_error",
                "heading_error_rad",
                "steering_angle",
                "lat_vel",
                "lat_acc",
                "long_vel",
                "target_speed",
                "long_vel_error",
                "long_acc",
                "ade",
                "mean_ade",
                "fde",
                "lateral_rms",
                "heading_rms",
                "speed_rms",
                "steering_saturated",
                "steer_saturation_rate",
            ]
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo("Tracking validator writing %s", self.output_csv)
        rospy.loginfo("Loaded %d previous validation run(s) for RViz comparison", len(self.previous_runs))

    def resolve_history_csvs(self, history_csvs, history_glob):
        paths = []
        for path in str(history_csvs).split(","):
            path = os.path.expanduser(path.strip())
            if path:
                paths.append(path)
        if history_glob:
            paths.extend(glob.glob(os.path.expanduser(str(history_glob))))
        return [path for path in paths if os.path.abspath(path) != os.path.abspath(self.output_csv)]

    def load_history_csv(self, path):
        if not os.path.exists(path):
            rospy.logwarn("Previous validation CSV does not exist: %s", path)
            return []

        points = []
        with open(path, "r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                try:
                    points.append((float(row["ego_x"]), float(row["ego_y"])))
                except (KeyError, TypeError, ValueError):
                    continue
        rospy.loginfo("Loaded %d ego samples from %s", len(points), path)
        return points

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.loginfo("Closed validation CSV: %s", self.output_csv)

    def path_callback(self, msg):
        self.path = msg
        self.path_points = []
        self.path_yaws = []
        self.cumulative_s = []
        total = 0.0

        last = None
        for pose_stamped in msg.poses:
            pose = pose_stamped.pose
            point = (pose.position.x, pose.position.y)
            if last is not None:
                total += norm2d(point[0] - last[0], point[1] - last[1])
            self.path_points.append(point)
            self.path_yaws.append(yaw_from_pose(pose))
            self.cumulative_s.append(total)
            last = point

        self.total_length = total
        self.last_nearest_index = min(self.last_nearest_index, max(0, len(self.path_points) - 1))

    def target_speed_callback(self, msg):
        self.target_speed = float(msg.data)

    def ego_callback(self, msg):
        self.ego = msg

    def ctrl_callback(self, msg):
        self.ctrl = msg

    def current_ego(self):
        pos = self.ego.position
        yaw = math.radians(float(self.ego.heading))
        vx = float(self.ego.velocity.x) * self.velocity_scale
        vy = float(self.ego.velocity.y) * self.velocity_scale
        long_vel = math.cos(yaw) * vx + math.sin(yaw) * vy
        lat_vel = -math.sin(yaw) * vx + math.cos(yaw) * vy
        return pos.x + self.ego_offset_x, pos.y + self.ego_offset_y, yaw, long_vel, lat_vel

    def nearest_index(self, x, y):
        if not self.path_points:
            return 0

        start = max(0, self.last_nearest_index - self.nearest_search_window)
        end = min(len(self.path_points), self.last_nearest_index + self.nearest_search_window)
        if end <= start:
            start, end = 0, len(self.path_points)

        best_index = start
        best_dist_sq = float("inf")
        for index in range(start, end):
            px, py = self.path_points[index]
            dist_sq = (px - x) ** 2 + (py - y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = index

        self.last_nearest_index = best_index
        return best_index

    def signed_lateral_error(self, x, y, path_index):
        px, py = self.path_points[path_index]
        yaw = self.path_yaws[path_index]
        dx = x - px
        dy = y - py
        return -math.sin(yaw) * dx + math.cos(yaw) * dy

    def acceleration(self, now, long_vel, lat_vel):
        if self.last_sample_time is None:
            self.last_sample_time = now
            self.last_long_vel = long_vel
            self.last_lat_vel = lat_vel
            return 0.0, 0.0

        dt = max(1e-3, now - self.last_sample_time)
        long_acc = (long_vel - self.last_long_vel) / dt
        lat_acc = (lat_vel - self.last_lat_vel) / dt
        self.last_sample_time = now
        self.last_long_vel = long_vel
        self.last_lat_vel = lat_vel
        return long_acc, lat_acc

    def publish_float(self, topic, value):
        self.publishers[topic].publish(Float32(data=float(value)))

    def marker_base(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def line_strip_marker(self, marker_id, namespace, points, width, rgba, z_offset=0.4):
        marker = self.marker_base(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.points = [Point(x, y, z_offset) for x, y in points]
        return marker

    def sphere_marker(self, marker_id, namespace, x, y, scale, rgba, z_offset=1.0):
        marker = self.marker_base(marker_id, namespace, Marker.SPHERE)
        marker.pose.position = Point(x, y, z_offset)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def text_marker(self, marker_id, x, y, lateral_error, heading_error, mean_ade):
        marker = self.marker_base(marker_id, "validation_summary_text", Marker.TEXT_VIEW_FACING)
        marker.pose.position = Point(x, y, 3.0)
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "lat %.2f m | head %.2f deg | mean ADE %.2f m" % (
            lateral_error,
            math.degrees(heading_error),
            mean_ade,
        )
        return marker

    def publish_markers(self, x, y, px, py, lateral_error, heading_error, mean_ade):
        markers = MarkerArray()
        if self.path_points:
            markers.markers.append(
                self.line_strip_marker(0, "validation_target_path", self.path_points, 0.25, (0.0, 0.65, 1.0, 0.95), 0.35)
            )

        if len(self.driven_points) >= 2:
            markers.markers.append(
                self.line_strip_marker(1, "validation_current_ego_path", self.driven_points, 0.32, (1.0, 0.85, 0.0, 0.95), 0.65)
            )

        marker_id = 10
        palette = [
            (0.75, 0.75, 0.75, 0.65),
            (0.45, 1.0, 0.45, 0.65),
            (1.0, 0.45, 0.45, 0.65),
            (0.8, 0.45, 1.0, 0.65),
        ]
        for index, run in enumerate(self.previous_runs):
            if len(run) < 2:
                continue
            markers.markers.append(
                self.line_strip_marker(
                    marker_id,
                    "validation_previous_ego_path_%d" % index,
                    run,
                    0.18,
                    palette[index % len(palette)],
                    0.55 + 0.05 * index,
                )
            )
            marker_id += 1

        markers.markers.append(self.sphere_marker(2, "validation_ego_position", x, y, 1.1, (1.0, 1.0, 1.0, 1.0), 1.0))
        markers.markers.append(self.sphere_marker(3, "validation_nearest_target", px, py, 0.9, (1.0, 0.0, 0.0, 1.0), 0.9))
        markers.markers.append(
            self.line_strip_marker(4, "validation_ego_to_target_error", [(x, y), (px, py)], 0.22, (1.0, 0.0, 1.0, 1.0), 1.0)
        )
        markers.markers.append(self.text_marker(5, x, y, lateral_error, heading_error, mean_ade))
        self.marker_pub.publish(markers)

    def evaluate_once(self):
        if self.ego is None or len(self.path_points) < 2:
            return None

        now = rospy.Time.now().to_sec()
        x, y, ego_yaw, long_vel, lat_vel = self.current_ego()
        nearest = self.nearest_index(x, y)
        px, py = self.path_points[nearest]
        path_yaw = self.path_yaws[nearest]
        lateral_error = self.signed_lateral_error(x, y, nearest)
        heading_error = normalize_angle(ego_yaw - path_yaw)
        long_vel_error = long_vel - self.target_speed
        long_acc, lat_acc = self.acceleration(now, long_vel, lat_vel)
        ade = norm2d(x - px, y - py)
        fde = norm2d(x - self.path_points[-1][0], y - self.path_points[-1][1])
        progress = 0.0
        if self.total_length > 1e-6:
            progress = clamp(self.cumulative_s[nearest] / self.total_length, 0.0, 1.0)

        front_steer = self.ctrl.front_steer if self.ctrl is not None else 0.0
        saturated = abs(front_steer) >= self.max_steer_rad * self.steer_saturation_ratio

        self.lateral_stats.add(lateral_error)
        self.heading_stats.add(heading_error)
        self.speed_stats.add(long_vel_error)
        self.ade_stats.add(ade)
        if saturated:
            self.saturated_count += 1

        self.sample_count += 1
        if self.sample_count % self.driven_path_decimation == 0:
            self.driven_points.append((x, y))

        saturation_rate = 0.0
        if self.lateral_stats.count > 0:
            saturation_rate = float(self.saturated_count) / float(self.lateral_stats.count)

        mean_ade = self.ade_stats.mean_abs()
        self.writer.writerow(
            [
                "%.6f" % now,
                nearest,
                "%.6f" % progress,
                "%.6f" % x,
                "%.6f" % y,
                "%.6f" % px,
                "%.6f" % py,
                "%.6f" % lateral_error,
                "%.6f" % heading_error,
                "%.6f" % front_steer,
                "%.6f" % lat_vel,
                "%.6f" % lat_acc,
                "%.6f" % long_vel,
                "%.6f" % self.target_speed,
                "%.6f" % long_vel_error,
                "%.6f" % long_acc,
                "%.6f" % ade,
                "%.6f" % mean_ade,
                "%.6f" % fde,
                "%.6f" % self.lateral_stats.rms(),
                "%.6f" % self.heading_stats.rms(),
                "%.6f" % self.speed_stats.rms(),
                int(saturated),
                "%.6f" % saturation_rate,
            ]
        )
        self.csv_file.flush()

        self.publish_float("/validation/lateral/lat_err", lateral_error)
        self.publish_float("/validation/lateral/heading_err", heading_error)
        self.publish_float("/validation/lateral/steering_angle", front_steer)
        self.publish_float("/validation/lateral/lat_vel", lat_vel)
        self.publish_float("/validation/lateral/lat_acc", lat_acc)
        self.publish_float("/validation/longitudinal/long_vel", long_vel)
        self.publish_float("/validation/longitudinal/target_vel", self.target_speed)
        self.publish_float("/validation/longitudinal/long_vel_err", long_vel_error)
        self.publish_float("/validation/longitudinal/long_acc", long_acc)
        self.publish_float("/validation/metrics/mean_ade", mean_ade)
        self.publish_float("/validation/metrics/fde", fde)
        self.publish_float("/validation/metrics/lateral_rms", self.lateral_stats.rms())
        self.publish_float("/validation/metrics/heading_rms", self.heading_stats.rms())
        self.publish_float("/validation/metrics/speed_rms", self.speed_stats.rms())
        self.publish_float("/validation/metrics/progress", progress)
        self.publish_float("/validation/metrics/steer_saturation_rate", saturation_rate)
        self.publish_float("/validation/lateral_error", lateral_error)
        self.publish_float("/validation/heading_error", heading_error)
        self.publish_float("/validation/speed_error", long_vel_error)
        self.publish_float("/validation/progress", progress)
        self.publish_markers(x, y, px, py, lateral_error, heading_error, mean_ade)

        return {
            "nearest": nearest,
            "progress": progress,
            "lateral": lateral_error,
            "lateral_rms": self.lateral_stats.rms(),
            "lateral_max": self.lateral_stats.max_abs,
            "heading_rms": self.heading_stats.rms(),
            "speed_rms": self.speed_stats.rms(),
            "saturation_rate": saturation_rate,
            "mean_ade": mean_ade,
            "fde": fde,
            "long_acc": long_acc,
            "lat_acc": lat_acc,
        }

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            result = self.evaluate_once()
            if result is None:
                rospy.loginfo_throttle(
                    1.0,
                    "Tracking validator waiting: path_ready=%s ego_ready=%s",
                    self.path is not None,
                    self.ego is not None,
                )
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "Validation n=%d progress=%.1f%% lat=%.2f lat_rms=%.2f heading_rms=%.3f speed_rms=%.2f mean_ade=%.2f fde=%.2f steer_sat=%.1f%%",
                    result["nearest"],
                    result["progress"] * 100.0,
                    result["lateral"],
                    result["lateral_rms"],
                    result["heading_rms"],
                    result["speed_rms"],
                    result["mean_ade"],
                    result["fde"],
                    result["saturation_rate"] * 100.0,
                )
            rate.sleep()


def main():
    rospy.init_node("tracking_validator")
    TrackingValidator().run()


if __name__ == "__main__":
    main()
