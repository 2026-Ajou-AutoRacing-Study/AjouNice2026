#!/usr/bin/env python3
"""Synchronize Sensor_4 ROS topics and publish Docker BEVFusion results for RViz."""

import collections
import hashlib
import json
import math
import os
import threading
import time

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from bevfusion_ros.transport import TcpTransport


DEFAULT_CAMERA_ORDER = [
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
]


class BevFusionRosBridge:
    def __init__(self):
        self.sensor_config = os.path.abspath(
            os.path.expanduser(rospy.get_param("~sensor_config"))
        )
        self.operation = rospy.get_param("~operation", "inference")
        self.intrinsic_source = rospy.get_param("~intrinsic_source", "horizontal_fov")
        self.camera_order = rospy.get_param("~camera_order", DEFAULT_CAMERA_ORDER)
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance_sec", 0.10))
        self.camera_buffer_size = int(rospy.get_param("~camera_buffer_size", 20))
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.2))
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime_sec", 0.5))
        self.transport = TcpTransport(
            rospy.get_param("~server_host", "127.0.0.1"),
            rospy.get_param("~server_port", 29500),
            rospy.get_param("~connect_timeout_sec", 2.0),
            rospy.get_param("~request_timeout_sec", 30.0),
        )
        self.camera_topics, self.lidar_topic = self._load_topics()
        self.camera_buffers = {
            frame: collections.deque(maxlen=self.camera_buffer_size)
            for frame in self.camera_order
        }
        self.buffer_lock = threading.Lock()
        self.pending_condition = threading.Condition()
        self.pending_frame = None
        self.sequence = 0
        self.dropped_pending = 0
        self.last_marker_count = 0
        self.shutting_down = False

        self.marker_pub = rospy.Publisher(
            "/perception/bevfusion/markers", MarkerArray, queue_size=1
        )
        self.latency_pub = rospy.Publisher(
            "/perception/bevfusion/latency_ms", Float32, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "/perception/bevfusion/status", String, queue_size=1, latch=True
        )
        self.projection_pubs = {
            frame: rospy.Publisher(
                "/perception/bevfusion/projection/{}/compressed".format(frame),
                CompressedImage,
                queue_size=1,
            )
            for frame in self.camera_order
        }
        self.subscribers = []
        for frame, topic in self.camera_topics.items():
            self.subscribers.append(
                rospy.Subscriber(
                    topic,
                    CompressedImage,
                    self._camera_callback,
                    callback_args=frame,
                    queue_size=5,
                    buff_size=8 * 1024 * 1024,
                )
            )
        self.subscribers.append(
            rospy.Subscriber(
                self.lidar_topic,
                PointCloud2,
                self._lidar_callback,
                queue_size=1,
                buff_size=32 * 1024 * 1024,
            )
        )
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)
        self._publish_status("waiting_for_synchronized_sensor_frame")
        rospy.loginfo(
            "BEVFusion bridge: operation=%s, lidar=%s, cameras=%s",
            self.operation,
            self.lidar_topic,
            self.camera_topics,
        )

    def _load_topics(self):
        with open(self.sensor_config, "rb") as config_file:
            config_bytes = config_file.read()
        self.sensor_config_sha256 = hashlib.sha256(config_bytes).hexdigest()
        config = json.loads(config_bytes.decode("utf-8"))
        cameras = {}
        for sensor in config.get("cameraList", []):
            ros_config = sensor.get("cc", {}).get("rosConfig", {})
            frame = ros_config.get("frameID", "")
            topic = ros_config.get("Topic", "")
            if frame and topic:
                cameras[frame] = topic
        missing = [frame for frame in self.camera_order if frame not in cameras]
        if missing:
            raise RuntimeError("Sensor config is missing cameras: {}".format(missing))
        lidars = config.get("Lidar3DList", [])
        if len(lidars) != 1:
            raise RuntimeError("Expected exactly one 3D LiDAR")
        lidar_topic = lidars[0].get("lc", {}).get("rosConfig", {}).get("Topic", "")
        if not lidar_topic:
            raise RuntimeError("LiDAR topic is missing from sensor config")
        return {frame: cameras[frame] for frame in self.camera_order}, lidar_topic

    def _camera_callback(self, message, frame):
        with self.buffer_lock:
            self.camera_buffers[frame].append(message)

    def _lidar_callback(self, message):
        lidar_stamp = stamp_seconds(message)
        selected = {}
        differences = {}
        with self.buffer_lock:
            for frame, buffer in self.camera_buffers.items():
                if not buffer:
                    return
                nearest = min(buffer, key=lambda item: abs(stamp_seconds(item) - lidar_stamp))
                difference = abs(stamp_seconds(nearest) - lidar_stamp)
                if difference > self.sync_tolerance:
                    rospy.logwarn_throttle(
                        2.0,
                        "BEVFusion sync rejected: %s differs by %.3f sec",
                        frame,
                        difference,
                    )
                    return
                selected[frame] = nearest
                differences[frame] = difference
        with self.pending_condition:
            if self.pending_frame is not None:
                self.dropped_pending += 1
            self.pending_frame = (message, selected, differences)
            self.pending_condition.notify()

    def _worker_loop(self):
        while not rospy.is_shutdown() and not self.shutting_down:
            with self.pending_condition:
                while self.pending_frame is None and not self.shutting_down:
                    self.pending_condition.wait(timeout=0.5)
                if self.shutting_down:
                    return
                frame = self.pending_frame
                self.pending_frame = None
            try:
                self._process_frame(*frame)
            except Exception as error:
                self._publish_status("error: {}".format(error))
                rospy.logwarn_throttle(2.0, "BEVFusion request failed: %s", error)
                time.sleep(0.2)

    def _process_frame(self, lidar_message, images, differences):
        start = time.monotonic()
        points = pointcloud_to_xyzir_time(lidar_message)
        self.sequence += 1
        metadata = {
            "operation": self.operation,
            "sequence": self.sequence,
            "stamp": stamp_seconds(lidar_message),
            "lidar_frame": lidar_message.header.frame_id,
            "camera_order": self.camera_order,
            "sensor_config_sha256": self.sensor_config_sha256,
            "intrinsic_source": self.intrinsic_source,
            "sync_difference_sec": differences,
            "points_blob": "points",
            "point_dim": 5,
            "point_count": int(points.shape[0]),
            "score_threshold": self.score_threshold,
            "max_detections": self.max_detections,
        }
        blobs = {"points": points.astype("<f4", copy=False).tobytes()}
        for frame, message in images.items():
            blobs["image/{}".format(frame)] = bytes(message.data)
        response, response_blobs = self.transport.request(metadata, blobs)
        latency_ms = (time.monotonic() - start) * 1000.0
        self.latency_pub.publish(Float32(data=latency_ms))
        if response.get("operation") == "projection":
            self._publish_projection(response, response_blobs)
        else:
            self._publish_markers(response, lidar_message.header.stamp)
        self._publish_status(
            "ok operation={} sequence={} latency_ms={:.1f} dropped_pending={}".format(
                response.get("operation"), self.sequence, latency_ms, self.dropped_pending
            )
        )

    def _publish_projection(self, response, blobs):
        stamp = rospy.Time.from_sec(float(response.get("stamp", 0.0)))
        for frame, publisher in self.projection_pubs.items():
            name = "projection/{}".format(frame)
            if name not in blobs:
                continue
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = frame
            message.format = "jpeg"
            message.data = blobs[name]
            publisher.publish(message)

    def _publish_markers(self, response, stamp):
        markers = []
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)
        frame_id = response.get("frame_id", "lidar1")
        detections = response.get("detections", [])
        for index, detection in enumerate(detections):
            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = stamp
            box.ns = "bevfusion_boxes"
            box.id = index
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = detection["center"]
            yaw = float(detection["yaw"])
            box.pose.orientation.z = math.sin(yaw * 0.5)
            box.pose.orientation.w = math.cos(yaw * 0.5)
            box.scale.x, box.scale.y, box.scale.z = [max(0.01, float(v)) for v in detection["size"]]
            red, green, blue = class_color(detection.get("label", 0))
            box.color.r, box.color.g, box.color.b, box.color.a = red, green, blue, 0.35
            box.lifetime = rospy.Duration(self.marker_lifetime)
            markers.append(box)

            label = Marker()
            label.header = box.header
            label.ns = "bevfusion_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = box.pose.position.x
            label.pose.position.y = box.pose.position.y
            label.pose.position.z = box.pose.position.z + box.scale.z * 0.5 + 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = "{} {:.2f}".format(
                detection.get("class_name", "unknown"), detection.get("score", 0.0)
            )
            label.lifetime = box.lifetime
            markers.append(label)
        self.last_marker_count = len(detections)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def shutdown(self):
        self.shutting_down = True
        self.transport.close()
        with self.pending_condition:
            self.pending_condition.notify_all()


def stamp_seconds(message):
    stamp = message.header.stamp
    if stamp.is_zero():
        return rospy.Time.now().to_sec()
    return stamp.to_sec()


def pointcloud_to_xyzir_time(message):
    available = {field.name for field in message.fields}
    required = ["x", "y", "z"]
    if not all(name in available for name in required):
        raise ValueError("PointCloud2 must contain x, y, z")
    intensity_name = "intensity" if "intensity" in available else None
    time_name = next((name for name in ("time", "t", "timestamp") if name in available), None)
    selected_fields = required[:]
    if intensity_name:
        selected_fields.append(intensity_name)
    if time_name:
        selected_fields.append(time_name)
    rows = []
    for values in point_cloud2.read_points(
        message, field_names=selected_fields, skip_nans=True
    ):
        cursor = 3
        intensity = float(values[cursor]) if intensity_name else 0.0
        cursor += 1 if intensity_name else 0
        relative_time = float(values[cursor]) if time_name else 0.0
        rows.append((values[0], values[1], values[2], intensity, relative_time))
    if not rows:
        return np.empty((0, 5), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)


def class_color(label):
    palette = [
        (0.1, 0.8, 0.2), (0.9, 0.5, 0.1), (0.6, 0.3, 0.9),
        (0.1, 0.5, 0.9), (0.8, 0.7, 0.1), (0.9, 0.2, 0.2),
        (0.8, 0.1, 0.6), (0.2, 0.8, 0.8), (0.95, 0.95, 0.95),
        (0.7, 0.4, 0.1),
    ]
    return palette[int(label) % len(palette)]


def main():
    rospy.init_node("bevfusion_ros_bridge")
    BevFusionRosBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
