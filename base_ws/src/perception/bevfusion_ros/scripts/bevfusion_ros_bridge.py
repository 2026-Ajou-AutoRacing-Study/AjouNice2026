#!/usr/bin/env python3
"""Synchronize Sensor_4 ROS topics and publish Docker BEVFusion results for RViz."""

import collections
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from autoware_perception_msgs.msg import DetectedObjects
from perception_object_pipeline import make_detected_objects
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from bevfusion_ros.transport import TcpTransport


DEFAULT_CAMERA_ORDER = [
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
]

POINTFIELD_TO_DTYPE = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


class BevFusionRosBridge:
    def __init__(self):
        self.sensor_config = os.path.abspath(
            os.path.expanduser(rospy.get_param("~sensor_config"))
        )
        self.operation = rospy.get_param("~operation", "inference")
        self.intrinsic_source = rospy.get_param("~intrinsic_source", "horizontal_fov")
        self.camera_order = rospy.get_param("~camera_order", DEFAULT_CAMERA_ORDER)
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance_sec", 0.10))
        self.camera_buffer_size = int(rospy.get_param("~camera_buffer_size", 30))
        self.preprocess_workers = max(
            1, int(rospy.get_param("~preprocess_workers", 4))
        )
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.2))
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime_sec", 0.5))
        self.transport = TcpTransport(
            rospy.get_param("~server_host", "127.0.0.1"),
            rospy.get_param("~server_port", 29500),
            rospy.get_param("~connect_timeout_sec", 2.0),
            rospy.get_param("~request_timeout_sec", 30.0),
        )
        (
            self.camera_topics,
            self.camera_shapes,
            self.lidar_topic,
        ) = self._load_topics()
        self.camera_buffers = {
            frame: collections.deque(maxlen=self.camera_buffer_size)
            for frame in self.camera_order
        }
        self.buffer_lock = threading.Lock()
        self.pipeline_condition = threading.Condition()
        self.raw_pending = None
        self.prepared_pending = None
        self.sequence = 0
        self.dropped_raw = 0
        self.dropped_prepared = 0
        self.last_marker_count = 0
        self.shutting_down = False

        self.marker_pub = rospy.Publisher(
            "/perception/bevfusion/markers", MarkerArray, queue_size=1
        )
        self.objects_pub = rospy.Publisher(
            rospy.get_param(
                "~output_objects",
                "/perception/object_recognition/detection/objects",
            ),
            DetectedObjects,
            queue_size=1,
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
        self.camera_executor = ThreadPoolExecutor(
            max_workers=self.preprocess_workers,
            thread_name_prefix="bevfusion_camera",
        )
        self.preprocess_worker = threading.Thread(
            target=self._preprocess_loop, daemon=True
        )
        self.inference_worker = threading.Thread(
            target=self._inference_loop, daemon=True
        )
        self.preprocess_worker.start()
        self.inference_worker.start()
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
        shapes = {}
        for sensor in config.get("cameraList", []):
            ros_config = sensor.get("cc", {}).get("rosConfig", {})
            frame = ros_config.get("frameID", "")
            topic = ros_config.get("Topic", "")
            if frame and topic:
                cameras[frame] = topic
                shapes[frame] = (
                    int(sensor["cc"]["cameraResHeight"]),
                    int(sensor["cc"]["cameraResWidth"]),
                )
        missing = [frame for frame in self.camera_order if frame not in cameras]
        if missing:
            raise RuntimeError("Sensor config is missing cameras: {}".format(missing))
        lidars = config.get("Lidar3DList", [])
        if len(lidars) != 1:
            raise RuntimeError("Expected exactly one 3D LiDAR")
        lidar_topic = lidars[0].get("lc", {}).get("rosConfig", {}).get("Topic", "")
        if not lidar_topic:
            raise RuntimeError("LiDAR topic is missing from sensor config")
        return (
            {frame: cameras[frame] for frame in self.camera_order},
            {frame: shapes[frame] for frame in self.camera_order},
            lidar_topic,
        )

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
        with self.pipeline_condition:
            if self.raw_pending is not None:
                self.dropped_raw += 1
            self.raw_pending = (
                message,
                selected,
                differences,
                time.monotonic(),
            )
            self.pipeline_condition.notify_all()

    def _preprocess_loop(self):
        while not rospy.is_shutdown() and not self.shutting_down:
            with self.pipeline_condition:
                while self.raw_pending is None and not self.shutting_down:
                    self.pipeline_condition.wait(timeout=0.5)
                if self.shutting_down:
                    return
                frame = self.raw_pending
                self.raw_pending = None
            try:
                prepared = self._prepare_frame(*frame)
                with self.pipeline_condition:
                    if self.prepared_pending is not None:
                        self.dropped_prepared += 1
                    self.prepared_pending = prepared
                    self.pipeline_condition.notify_all()
            except Exception as error:
                self._publish_status("error: {}".format(error))
                rospy.logwarn_throttle(
                    2.0, "BEVFusion preprocessing failed: %s", error
                )

    def _inference_loop(self):
        while not rospy.is_shutdown() and not self.shutting_down:
            with self.pipeline_condition:
                while self.prepared_pending is None and not self.shutting_down:
                    self.pipeline_condition.wait(timeout=0.5)
                if self.shutting_down:
                    return
                prepared = self.prepared_pending
                self.prepared_pending = None
            try:
                self._request_and_publish(prepared)
            except Exception as error:
                self._publish_status("error: {}".format(error))
                rospy.logwarn_throttle(2.0, "BEVFusion request failed: %s", error)
                time.sleep(0.2)

    def _prepare_frame(
        self, lidar_message, images, differences, received_at
    ):
        start = time.monotonic()
        conversion_start = time.monotonic()
        points = pointcloud_to_xyzi(lidar_message)
        conversion_ms = (time.monotonic() - conversion_start) * 1000.0
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
            "point_dim": 4,
            "point_count": int(points.shape[0]),
            "score_threshold": self.score_threshold,
            "max_detections": self.max_detections,
        }
        blobs = {"points": points.astype("<f4", copy=False).tobytes()}
        if self.operation == "inference":
            resized = dict(self.camera_executor.map(
                self._preprocess_camera,
                [(frame, images[frame]) for frame in self.camera_order],
            ))
            for frame, image in resized.items():
                blobs["image/{}".format(frame)] = image.tobytes()
            metadata.update({
                "image_encoding": "rgb8",
                "image_height": 256,
                "image_width": 704,
                "image_channels": 3,
            })
        else:
            metadata["image_encoding"] = "compressed"
            for frame, message in images.items():
                blobs["image/{}".format(frame)] = bytes(message.data)
        metadata["pointcloud_ms"] = conversion_ms
        metadata["host_preprocess_ms"] = (
            time.monotonic() - start
        ) * 1000.0
        return metadata, blobs, lidar_message.header.stamp, received_at

    def _preprocess_camera(self, argument):
        frame, message = argument
        image = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise ValueError("could not decode image for {}".format(frame))
        if image.shape[:2] != self.camera_shapes[frame]:
            raise ValueError(
                "{} image shape {} differs from Sensor.json {}".format(
                    frame, image.shape[:2], self.camera_shapes[frame]
                )
            )
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width = image.shape[:2]
        scale = 0.55 * 1280.0 / width
        new_width, new_height = int(width * scale), int(height * scale)
        image = cv2.resize(
            image,
            (new_width, new_height),
            interpolation=cv2.INTER_CUBIC,
        )
        crop_top = new_height - 256
        crop_left = max(0, new_width - 704) // 2
        image = image[crop_top:crop_top + 256, crop_left:crop_left + 704]
        if image.shape[:2] != (256, 704):
            raise ValueError(
                "{} preprocessed shape is {}".format(frame, image.shape[:2])
            )
        return frame, np.ascontiguousarray(image)

    def _request_and_publish(self, prepared):
        metadata, blobs, stamp, received_at = prepared
        request_start = time.monotonic()
        response, response_blobs = self.transport.request(metadata, blobs)
        transport_ms = (time.monotonic() - request_start) * 1000.0
        latency_ms = (time.monotonic() - received_at) * 1000.0
        self.latency_pub.publish(Float32(data=latency_ms))
        if response.get("operation") == "projection":
            self._publish_projection(response, response_blobs)
        else:
            self._publish_detections(response, stamp)
        self._publish_status(
            "ok operation={} sequence={} latency_ms={:.1f} host_preprocess_ms={:.1f} "
            "pointcloud_ms={:.1f} server_preprocess_ms={:.1f} "
            "inference_ms={:.1f} request_ms={:.1f} dropped_raw={} "
            "dropped_prepared={}".format(
                response.get("operation"),
                metadata.get("sequence", 0),
                latency_ms,
                float(metadata.get("host_preprocess_ms", 0.0)),
                float(metadata.get("pointcloud_ms", 0.0)),
                float(response.get("preprocess_ms", 0.0)),
                float(response.get("inference_ms", 0.0)),
                transport_ms,
                self.dropped_raw,
                self.dropped_prepared,
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

    def _publish_detections(self, response, stamp):
        markers = []
        frame_id = response.get("frame_id", "lidar1")
        detections = response.get("detections", [])
        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        header = delete_all.header
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)
        boxes = [
            list(detection["center"])
            + list(detection["size"])
            + [float(detection["yaw"])]
            for detection in detections
        ]
        self.objects_pub.publish(
            make_detected_objects(
                header,
                boxes,
                [float(detection.get("score", 0.0)) for detection in detections],
                [str(detection.get("class_name", "unknown")) for detection in detections],
            )
        )
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
            label.text = "{} {:.6f}".format(
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
        with self.pipeline_condition:
            self.pipeline_condition.notify_all()
        self.camera_executor.shutdown(wait=False)


def stamp_seconds(message):
    stamp = message.header.stamp
    if stamp.is_zero():
        return rospy.Time.now().to_sec()
    return stamp.to_sec()


def pointcloud_to_xyzi(message):
    """Vectorize PointCloud2 into Base3's Nx4 feature contract."""
    fields = {field.name: field for field in message.fields}
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError(
                "PointCloud2 field {} is missing; fields={}".format(
                    required, sorted(fields)
                )
            )
    names = ["x", "y", "z"]
    if "intensity" in fields:
        names.append("intensity")
    endian = ">" if message.is_bigendian else "<"
    formats, offsets = [], []
    for name in names:
        field = fields[name]
        if field.datatype not in POINTFIELD_TO_DTYPE:
            raise ValueError(
                "unsupported PointField datatype for {}: {}".format(
                    name, field.datatype
                )
            )
        formats.append(endian + POINTFIELD_TO_DTYPE[field.datatype])
        offsets.append(int(field.offset))
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(message.point_step),
    })
    count = int(message.width) * int(message.height)
    if count <= 0:
        return np.empty((0, 4), dtype=np.float32)
    raw = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.zeros((count, 4), dtype=np.float32)
    points[:, 0] = raw["x"]
    points[:, 1] = raw["y"]
    points[:, 2] = raw["z"]
    if "intensity" in names:
        points[:, 3] = raw["intensity"].astype(np.float32, copy=False)
    return points


def class_color(label):
    palette = [
        (0.1, 0.8, 0.2),
        (0.95, 0.75, 0.1),
        (0.6, 0.3, 0.9),
    ]
    return palette[int(label) % len(palette)]


def main():
    rospy.init_node("bevfusion_ros_bridge")
    BevFusionRosBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
