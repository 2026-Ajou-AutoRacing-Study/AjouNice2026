#!/usr/bin/env python3
"""Publish a compact, north-up localization minimap as sensor_msgs/Image."""

import math
import threading

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def marker_points_xy(marker):
    """Apply the marker's planar pose and return its points as an Nx2 array."""
    if not marker.points:
        return np.empty((0, 2), dtype=np.float64)

    yaw = quaternion_to_yaw(marker.pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    tx = marker.pose.position.x
    ty = marker.pose.position.y
    points = np.empty((len(marker.points), 2), dtype=np.float64)

    for index, point in enumerate(marker.points):
        points[index, 0] = cosine * point.x - sine * point.y + tx
        points[index, 1] = sine * point.x + cosine * point.y + ty
    return points


class LocalizationMinimapPublisher:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map/lanelet_map")
        self.state_topic = rospy.get_param(
            "~state_topic", "/localization/kinematic_state"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/minimap/image"
        )
        self.width = max(240, int(rospy.get_param("~width", 480)))
        self.height = max(240, int(rospy.get_param("~height", 480)))
        self.margin = max(12, int(rospy.get_param("~margin", 24)))
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 10.0)))
        self.path_min_distance = max(
            0.01, float(rospy.get_param("~path_min_distance", 0.20))
        )
        self.max_path_points = max(
            100, int(rospy.get_param("~max_path_points", 20000))
        )

        self.lock = threading.Lock()
        self.background = None
        self.bounds = None
        self.state = None
        self.path = []
        self.last_state_stamp = None
        self.map_frame = "map"

        self.publisher = rospy.Publisher(
            self.output_topic, Image, queue_size=1, latch=True
        )
        self.map_subscriber = rospy.Subscriber(
            self.map_topic, MarkerArray, self.map_callback, queue_size=1
        )
        self.state_subscriber = rospy.Subscriber(
            self.state_topic, Odometry, self.state_callback, queue_size=20
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.timer_callback
        )

        rospy.loginfo(
            "Localization minimap: map=%s state=%s output=%s size=%dx%d",
            self.map_topic,
            self.state_topic,
            self.output_topic,
            self.width,
            self.height,
        )

    @staticmethod
    def marker_color(marker):
        namespace = marker.ns.lower()
        if "center" in namespace:
            return (40, 195, 225)
        if "stop" in namespace:
            return (60, 80, 235)
        if "crosswalk" in namespace:
            return (120, 180, 120)
        return (145, 150, 155)

    def map_callback(self, message):
        polylines = []
        all_points = []
        frame_id = None

        for marker in message.markers:
            if marker.action not in (Marker.ADD, Marker.MODIFY):
                continue
            points = marker_points_xy(marker)
            if points.size == 0:
                continue
            if frame_id is None and marker.header.frame_id:
                frame_id = marker.header.frame_id
            all_points.append(points)

            if marker.type == Marker.LINE_STRIP and len(points) >= 2:
                polylines.append((points, self.marker_color(marker), False))
            elif marker.type == Marker.LINE_LIST and len(points) >= 2:
                for index in range(0, len(points) - 1, 2):
                    polylines.append(
                        (points[index : index + 2], self.marker_color(marker), False)
                    )
            elif marker.type == Marker.POINTS:
                polylines.append((points, self.marker_color(marker), True))

        if not all_points:
            rospy.logwarn_throttle(
                5.0, "Localization minimap received an empty Lanelet MarkerArray"
            )
            return

        combined = np.concatenate(all_points, axis=0)
        finite = np.isfinite(combined).all(axis=1)
        combined = combined[finite]
        if combined.size == 0:
            rospy.logwarn("Localization minimap map has no finite coordinates")
            return

        min_x, min_y = np.min(combined, axis=0)
        max_x, max_y = np.max(combined, axis=0)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        padding_x = max(5.0, span_x * 0.03)
        padding_y = max(5.0, span_y * 0.03)
        bounds = (
            min_x - padding_x,
            min_y - padding_y,
            max_x + padding_x,
            max_y + padding_y,
        )

        background = np.full((self.height, self.width, 3), (28, 31, 34), np.uint8)
        for points, color, draw_points in polylines:
            pixels = self.world_points_to_pixels(points, bounds)
            if draw_points:
                for pixel in pixels:
                    cv2.circle(background, tuple(pixel), 1, color, -1, cv2.LINE_AA)
            elif len(pixels) >= 2:
                cv2.polylines(
                    background,
                    [pixels],
                    False,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        cv2.rectangle(
            background,
            (0, 0),
            (self.width - 1, self.height - 1),
            (92, 98, 104),
            1,
        )

        with self.lock:
            self.bounds = bounds
            self.background = background
            self.map_frame = frame_id or "map"

        rospy.loginfo(
            "Localization minimap cached Lanelet map: %.1f x %.1f m, %d primitives",
            bounds[2] - bounds[0],
            bounds[3] - bounds[1],
            len(polylines),
        )

    def state_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation)
        velocity = message.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        stamp = message.header.stamp

        with self.lock:
            if (
                self.last_state_stamp is not None
                and not stamp.is_zero()
                and stamp < self.last_state_stamp
            ):
                self.path = []

            if not self.path or math.hypot(
                position.x - self.path[-1][0], position.y - self.path[-1][1]
            ) >= self.path_min_distance:
                self.path.append((position.x, position.y))
                if len(self.path) > self.max_path_points:
                    self.path = self.path[-self.max_path_points :]

            self.state = {
                "x": position.x,
                "y": position.y,
                "yaw": yaw,
                "speed": speed,
                "stamp": stamp,
                "frame_id": message.header.frame_id,
            }
            if not stamp.is_zero():
                self.last_state_stamp = stamp

    def world_points_to_pixels(self, points, bounds):
        min_x, min_y, max_x, max_y = bounds
        usable_width = max(1, self.width - 2 * self.margin)
        usable_height = max(1, self.height - 2 * self.margin)
        scale = min(
            usable_width / max(max_x - min_x, 1e-6),
            usable_height / max(max_y - min_y, 1e-6),
        )
        drawn_width = (max_x - min_x) * scale
        drawn_height = (max_y - min_y) * scale
        offset_x = (self.width - drawn_width) * 0.5
        offset_y = (self.height - drawn_height) * 0.5

        pixels = np.empty_like(points, dtype=np.int32)
        pixels[:, 0] = np.rint(offset_x + (points[:, 0] - min_x) * scale)
        pixels[:, 1] = np.rint(
            self.height - offset_y - (points[:, 1] - min_y) * scale
        )
        return pixels

    def draw_vehicle(self, image, x, y, yaw, bounds):
        center = self.world_points_to_pixels(
            np.array([[x, y]], dtype=np.float64), bounds
        )[0]
        length = 12.0
        half_width = 6.0
        forward = np.array([math.cos(yaw), -math.sin(yaw)])
        left = np.array([math.sin(yaw), math.cos(yaw)])
        triangle = np.array(
            [
                center + forward * length,
                center - forward * length * 0.65 + left * half_width,
                center - forward * length * 0.65 - left * half_width,
            ],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(image, triangle, (30, 110, 255), cv2.LINE_AA)
        cv2.polylines(image, [triangle], True, (245, 245, 245), 1, cv2.LINE_AA)
        cv2.circle(image, tuple(center), 2, (255, 255, 255), -1, cv2.LINE_AA)

    def timer_callback(self, _event):
        with self.lock:
            if self.background is None:
                return
            image = self.background.copy()
            bounds = self.bounds
            state = None if self.state is None else dict(self.state)
            path = list(self.path)

        if len(path) >= 2:
            path_pixels = self.world_points_to_pixels(
                np.asarray(path, dtype=np.float64), bounds
            )
            cv2.polylines(
                image, [path_pixels], False, (255, 165, 35), 2, cv2.LINE_AA
            )

        title = "GLOBAL MINIMAP"
        speed_text = "waiting for localization"
        if state is not None:
            self.draw_vehicle(
                image, state["x"], state["y"], state["yaw"], bounds
            )
            speed_text = "v = %.1f km/h" % (state["speed"] * 3.6)

        cv2.rectangle(image, (0, 0), (self.width, 54), (18, 20, 23), -1)
        cv2.putText(
            image,
            title,
            (14, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            speed_text,
            (14, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (80, 205, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "north-up",
            (self.width - 82, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (190, 195, 200),
            1,
            cv2.LINE_AA,
        )

        output = Image()
        output.header.stamp = (
            state["stamp"]
            if state is not None and not state["stamp"].is_zero()
            else rospy.Time.now()
        )
        output.header.frame_id = self.map_frame
        output.height = self.height
        output.width = self.width
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = self.width * 3
        output.data = image.tobytes()
        self.publisher.publish(output)


def main():
    rospy.init_node("localization_minimap_publisher")
    LocalizationMinimapPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
