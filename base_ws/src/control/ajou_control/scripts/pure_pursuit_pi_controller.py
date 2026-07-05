#!/usr/bin/env python3

import math

import rospy
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


class PurePursuitPIController:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/recorded_trajectory_path")
        self.target_speed_topic = rospy.get_param("~target_speed_topic", "/recorded_trajectory/target_speed")
        self.use_target_speed_topic = bool(rospy.get_param("~use_target_speed_topic", False))
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.ctrl_topic = rospy.get_param("~ctrl_topic", "/ctrl_cmd")
        self.debug_topic = rospy.get_param("~debug_topic", "/pure_pursuit/debug_markers")

        self.control_rate = float(rospy.get_param("~control_rate", 30.0))
        self.wheel_base = float(rospy.get_param("~wheel_base", 2.8))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 4.0))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 18.0))
        self.lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.8))
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.6))
        self.max_steer_rate = float(rospy.get_param("~max_steer_rate", 0.35))
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))

        self.default_target_speed = float(rospy.get_param("~default_target_speed", 5.0))
        self.max_target_speed = float(rospy.get_param("~max_target_speed", 12.0))
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 1.0))
        self.kp = float(rospy.get_param("~kp", 0.25))
        self.ki = float(rospy.get_param("~ki", 0.04))
        self.integral_limit = float(rospy.get_param("~integral_limit", 8.0))
        self.accel_feedforward = float(rospy.get_param("~accel_feedforward", 0.08))
        self.min_drive_accel = float(rospy.get_param("~min_drive_accel", 0.18))
        self.speed_deadband = float(rospy.get_param("~speed_deadband", 0.15))
        self.brake_gain = float(rospy.get_param("~brake_gain", 0.25))
        self.stop_distance = float(rospy.get_param("~stop_distance", 2.5))
        self.nearest_search_window = int(rospy.get_param("~nearest_search_window", 80))
        self.reacquire_distance = float(rospy.get_param("~reacquire_distance", 12.0))

        self.path = None
        self.path_points = []
        self.ego = None
        self.target_speed_mps = self.default_target_speed
        self.last_nearest_index = 0
        self.speed_integral = 0.0
        self.last_time = None
        self.last_steer = 0.0

        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        if self.use_target_speed_topic:
            rospy.Subscriber(self.target_speed_topic, Float32, self.target_speed_callback, queue_size=1)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        self.ctrl_pub = rospy.Publisher(self.ctrl_topic, CtrlCmd, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, MarkerArray, queue_size=1)

        rospy.loginfo(
            "PurePursuitPIController 설정: path=%s speed=%s use_speed_topic=%s ego=%s ctrl=%s debug=%s",
            self.path_topic,
            self.target_speed_topic,
            self.use_target_speed_topic,
            self.ego_topic,
            self.ctrl_topic,
            self.debug_topic,
        )

    def path_callback(self, msg):
        self.path = msg
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self.last_nearest_index = min(self.last_nearest_index, max(0, len(self.path_points) - 1))

    def target_speed_callback(self, msg):
        self.target_speed_mps = clamp(float(msg.data), 0.0, self.max_target_speed)

    def ego_callback(self, msg):
        self.ego = msg

    def current_pose(self):
        position = self.ego.position
        yaw = math.radians(float(self.ego.heading))
        raw_speed = norm2d(self.ego.velocity.x, self.ego.velocity.y)
        speed = raw_speed * self.velocity_scale
        return position.x, position.y, yaw, speed, raw_speed

    def nearest_index(self, x, y):
        if not self.path_points:
            return 0

        start = max(0, self.last_nearest_index - self.nearest_search_window)
        end = min(len(self.path_points), self.last_nearest_index + self.nearest_search_window)
        if end <= start or self.distance_to_index(self.last_nearest_index, x, y) > self.reacquire_distance:
            start, end = 0, len(self.path_points)

        best_index = start
        best_dist_sq = float("inf")
        for index in range(start, end):
            px, py, _ = self.path_points[index]
            dist_sq = (px - x) ** 2 + (py - y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = index

        self.last_nearest_index = best_index
        return best_index

    def distance_to_index(self, index, x, y):
        if not self.path_points:
            return float("inf")
        index = max(0, min(index, len(self.path_points) - 1))
        px, py, _ = self.path_points[index]
        return norm2d(px - x, py - y)

    def target_index(self, nearest_index, lookahead):
        if not self.path_points:
            return 0

        accumulated = 0.0
        last_x, last_y, _ = self.path_points[nearest_index]
        for index in range(nearest_index + 1, len(self.path_points)):
            px, py, _ = self.path_points[index]
            accumulated += norm2d(px - last_x, py - last_y)
            if accumulated >= lookahead:
                return index
            last_x, last_y = px, py

        return len(self.path_points) - 1

    def build_safe_stop(self):
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.0
        cmd.brake = 1.0
        cmd.front_steer = 0.0
        return cmd

    def make_sphere_marker(self, marker_id, namespace, x, y, z, scale, red, green, blue):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        return marker

    def make_line_marker(self, marker_id, namespace, start_x, start_y, end_x, end_y, red, green, blue):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.points = [
            Point(start_x, start_y, 1.2),
            Point(end_x, end_y, 1.2),
        ]
        return marker

    def make_heading_marker(self, marker_id, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_ego_heading"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.9
        marker.scale.z = 0.9
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [
            Point(x, y, 1.5),
            Point(x + 5.0 * math.cos(yaw), y + 5.0 * math.sin(yaw), 1.5),
        ]
        return marker

    def publish_debug_markers(self, x, y, yaw, nearest, target, tx, ty):
        if not self.path_points:
            return

        nx, ny, nz = self.path_points[nearest]
        _, _, tz = self.path_points[target]
        markers = MarkerArray()
        markers.markers.append(self.make_sphere_marker(0, "pure_pursuit_ego", x, y, 1.4, 1.4, 1.0, 1.0, 1.0))
        markers.markers.append(self.make_sphere_marker(1, "pure_pursuit_nearest", nx, ny, nz + 1.2, 1.2, 1.0, 0.0, 0.0))
        markers.markers.append(self.make_sphere_marker(2, "pure_pursuit_target", tx, ty, tz + 1.6, 1.6, 0.0, 0.4, 1.0))
        markers.markers.append(self.make_line_marker(3, "pure_pursuit_lookahead", x, y, tx, ty, 1.0, 1.0, 0.0))
        markers.markers.append(self.make_line_marker(4, "pure_pursuit_cte", x, y, nx, ny, 1.0, 0.0, 1.0))
        markers.markers.append(self.make_heading_marker(5, x, y, yaw))
        self.debug_pub.publish(markers)

    def build_command(self):
        if self.ego is None or len(self.path_points) < 2:
            return self.build_safe_stop(), "대기 중"

        now = rospy.Time.now()
        if self.last_time is None:
            dt = 1.0 / max(self.control_rate, 1.0)
        else:
            dt = max(1e-3, (now - self.last_time).to_sec())
        self.last_time = now

        x, y, yaw, speed, raw_speed = self.current_pose()
        nearest = self.nearest_index(x, y)
        cte = self.distance_to_index(nearest, x, y)
        goal_distance = norm2d(self.path_points[-1][0] - x, self.path_points[-1][1] - y)

        if nearest >= len(self.path_points) - 2 and goal_distance < self.stop_distance:
            self.speed_integral = 0.0
            return self.build_safe_stop(), "완료"

        lookahead = clamp(
            self.min_lookahead + self.lookahead_gain * speed,
            self.min_lookahead,
            self.max_lookahead,
        )
        target = self.target_index(nearest, lookahead)
        tx, ty, _ = self.path_points[target]
        self.publish_debug_markers(x, y, yaw, nearest, target, tx, ty)

        dx = tx - x
        dy = ty - y
        alpha = normalize_angle(math.atan2(dy, dx) - yaw)
        target_distance = max(norm2d(dx, dy), 1e-3)
        steer = self.steering_sign * math.atan2(2.0 * self.wheel_base * math.sin(alpha), target_distance)
        steer = clamp(steer, -self.max_steer_rad, self.max_steer_rad)
        max_steer_delta = self.max_steer_rate * dt
        steer = clamp(steer, self.last_steer - max_steer_delta, self.last_steer + max_steer_delta)
        self.last_steer = steer

        target_speed = clamp(self.target_speed_mps, 0.0, self.max_target_speed)
        speed_error = target_speed - speed
        self.speed_integral = clamp(
            self.speed_integral + speed_error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        pi_output = self.kp * speed_error + self.ki * self.speed_integral

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.front_steer = steer
        if speed_error > self.speed_deadband:
            cmd.accel = clamp(max(self.min_drive_accel, self.accel_feedforward + pi_output), 0.0, 1.0)
            cmd.brake = 0.0
        elif speed_error < -self.speed_deadband:
            cmd.accel = 0.0
            cmd.brake = clamp(-pi_output * self.brake_gain, 0.0, 1.0)
        else:
            cmd.accel = clamp(self.accel_feedforward, 0.0, 1.0)
            cmd.brake = 0.0

        return cmd, "추종 중 n=%d target=%d cte=%.2f ego=(%.1f,%.1f) tgt=(%.1f,%.1f) alpha=%.2f ld=%.2f speed=%.2f(raw %.2f)/%.2f accel=%.2f brake=%.2f steer=%.3f" % (
            nearest,
            target,
            cte,
            x,
            y,
            tx,
            ty,
            alpha,
            lookahead,
            speed,
            raw_speed,
            target_speed,
            cmd.accel,
            cmd.brake,
            steer,
        )

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            cmd, state = self.build_command()
            self.ctrl_pub.publish(cmd)
            rospy.loginfo_throttle(1.0, "Pure pursuit PI 상태: %s", state)
            rate.sleep()


def main():
    rospy.init_node("pure_pursuit_pi_controller")
    PurePursuitPIController().run()


if __name__ == "__main__":
    main()
