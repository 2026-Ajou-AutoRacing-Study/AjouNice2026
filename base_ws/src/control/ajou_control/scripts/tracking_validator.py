#!/usr/bin/env python3

import csv
import math
import os

import rospy
import tf.transformations
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path
from std_msgs.msg import Float32


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
        self.output_csv = os.path.expanduser(
            rospy.get_param("~output_csv", "~/catkin_ws/src/ajou_control/validation_tracking.csv")
        )
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.6))
        self.steer_saturation_ratio = float(rospy.get_param("~steer_saturation_ratio", 0.95))
        self.nearest_search_window = int(rospy.get_param("~nearest_search_window", 120))

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
        self.saturated_count = 0

        self.lateral_error_pub = rospy.Publisher("/validation/lateral_error", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/validation/heading_error", Float32, queue_size=1)
        self.speed_error_pub = rospy.Publisher("/validation/speed_error", Float32, queue_size=1)
        self.progress_pub = rospy.Publisher("/validation/progress", Float32, queue_size=1)

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
                "speed",
                "target_speed",
                "speed_error",
                "front_steer",
                "steering_saturated",
            ]
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo("Tracking validator writing %s", self.output_csv)

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
        speed = norm2d(self.ego.velocity.x, self.ego.velocity.y)
        return pos.x, pos.y, yaw, speed

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

    def evaluate_once(self):
        if self.ego is None or len(self.path_points) < 2:
            return None

        x, y, ego_yaw, speed = self.current_ego()
        nearest = self.nearest_index(x, y)
        px, py = self.path_points[nearest]
        path_yaw = self.path_yaws[nearest]
        lateral_error = self.signed_lateral_error(x, y, nearest)
        heading_error = normalize_angle(ego_yaw - path_yaw)
        speed_error = speed - self.target_speed
        progress = 0.0
        if self.total_length > 1e-6:
            progress = clamp(self.cumulative_s[nearest] / self.total_length, 0.0, 1.0)

        front_steer = self.ctrl.front_steer if self.ctrl is not None else 0.0
        saturated = abs(front_steer) >= self.max_steer_rad * self.steer_saturation_ratio

        self.lateral_stats.add(lateral_error)
        self.heading_stats.add(heading_error)
        self.speed_stats.add(speed_error)
        if saturated:
            self.saturated_count += 1

        stamp = rospy.Time.now().to_sec()
        self.writer.writerow(
            [
                "%.6f" % stamp,
                nearest,
                "%.6f" % progress,
                "%.6f" % x,
                "%.6f" % y,
                "%.6f" % px,
                "%.6f" % py,
                "%.6f" % lateral_error,
                "%.6f" % heading_error,
                "%.6f" % speed,
                "%.6f" % self.target_speed,
                "%.6f" % speed_error,
                "%.6f" % front_steer,
                int(saturated),
            ]
        )
        self.csv_file.flush()

        self.lateral_error_pub.publish(Float32(data=lateral_error))
        self.heading_error_pub.publish(Float32(data=heading_error))
        self.speed_error_pub.publish(Float32(data=speed_error))
        self.progress_pub.publish(Float32(data=progress))

        saturation_rate = 0.0
        if self.lateral_stats.count > 0:
            saturation_rate = float(self.saturated_count) / float(self.lateral_stats.count)

        return {
            "nearest": nearest,
            "progress": progress,
            "lateral": lateral_error,
            "lateral_rms": self.lateral_stats.rms(),
            "lateral_max": self.lateral_stats.max_abs,
            "heading_rms": self.heading_stats.rms(),
            "speed_rms": self.speed_stats.rms(),
            "saturation_rate": saturation_rate,
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
                    "Validation n=%d progress=%.1f%% lateral=%.2f rms=%.2f max=%.2f heading_rms=%.3f speed_rms=%.2f steer_sat=%.1f%%",
                    result["nearest"],
                    result["progress"] * 100.0,
                    result["lateral"],
                    result["lateral_rms"],
                    result["lateral_max"],
                    result["heading_rms"],
                    result["speed_rms"],
                    result["saturation_rate"] * 100.0,
                )
            rate.sleep()


def main():
    rospy.init_node("tracking_validator")
    TrackingValidator().run()


if __name__ == "__main__":
    main()
