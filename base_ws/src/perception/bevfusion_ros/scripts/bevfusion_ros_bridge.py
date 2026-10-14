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
from perception_object_pipeline.rolling_lidar import (
    PlanarPoseBuffer,
    deskew_points_to_reference_pose,
    lidar_world_pose_at_time,
    scan_time_contract,
    transform_points_between_lidar_times,
)
from nav_msgs.msg import Odometry
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from bevfusion_ros.score_thresholds import (
    build_score_thresholds,
    filter_detections,
    server_score_floor,
)
from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms
from bevfusion_ros.transport import TcpTransport
from bevfusion_ros.frame_timing import FrameTimingLedger
from bevfusion_ros.box_center_origin import LEGACY_OUTPUT, response_for_ros, validate_output_origin


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
        self.box_center_output_origin = validate_output_origin(
            rospy.get_param('~box_center_output_origin', LEGACY_OUTPUT))
        self.intrinsic_source = rospy.get_param("~intrinsic_source", "horizontal_fov")
        self.camera_order = rospy.get_param("~camera_order", DEFAULT_CAMERA_ORDER)
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance_sec", 0.10))
        self.camera_buffer_size = int(rospy.get_param("~camera_buffer_size", 30))
        self.preprocess_workers = max(
            1, int(rospy.get_param("~preprocess_workers", 4))
        )
        # Production E25 defaults are unchanged. Base5 selects its learned
        # deterministic crop via a separate runtime configuration.
        self.image_final_height = int(rospy.get_param('~image_final_height', 256))
        self.image_final_width = int(rospy.get_param('~image_final_width', 704))
        self.image_resize = float(rospy.get_param('~image_resize', 0.55))
        self.image_reference_width = float(rospy.get_param('~image_reference_width', 1280))
        self.image_encoding = str(rospy.get_param('~image_encoding', 'rgb8'))
        if self.image_encoding not in ('rgb8', 'compressed'):
            raise ValueError('image_encoding must be rgb8 or compressed')
        self.point_intensity_source = str(rospy.get_param('~point_intensity_source', 'native'))
        if self.point_intensity_source not in ('native', 'ros255'):
            raise ValueError('point_intensity_source must be native or ros255')
        if (self.image_final_height <= 0 or self.image_final_width <= 0 or
                self.image_final_height % 32 or self.image_final_width % 32 or
                not math.isfinite(self.image_resize) or self.image_resize <= 0 or
                not math.isfinite(self.image_reference_width) or self.image_reference_width <= 0):
            raise ValueError('Invalid deterministic image resize/crop contract')
        self.lidar_deskew_mode = str(
            rospy.get_param("~lidar_deskew_mode", "off")
        ).strip().lower()
        self.lidar_deskew_reference = str(
            rospy.get_param("~lidar_deskew_reference", "end")
        ).strip().lower()
        self.lidar_deskew_max_point_time_span_sec = float(
            rospy.get_param("~lidar_deskew_max_point_time_span_sec", 0.15)
        )
        self.lidar_deskew_maximum_extrapolation_sec = float(
            rospy.get_param("~lidar_deskew_maximum_extrapolation_sec", 0.05)
        )
        self.lidar_deskew_pose_topic = str(
            rospy.get_param(
                "~lidar_deskew_pose_topic", "/localization/kinematic_state"
            )
        )
        self.lidar_deskew_pose_type = str(
            rospy.get_param("~lidar_deskew_pose_type", "odometry")
        ).strip().lower()
        self.temporal_sweeps = max(
            0, int(rospy.get_param("~temporal_sweeps", 0)))
        self.temporal_feature_cache = bool(
            rospy.get_param("~temporal_feature_cache", False))
        self.query_memory_context = bool(
            rospy.get_param("~query_memory_context", False))
        self.query_memory_diagnostics_path = os.path.abspath(
            os.path.expanduser(str(rospy.get_param(
                "~query_memory_diagnostics_path", "")))) \
            if rospy.get_param("~query_memory_diagnostics_path", "") else ""
        self.query_memory_diagnostics_handle = None
        if self.query_memory_diagnostics_path:
            parent = os.path.dirname(self.query_memory_diagnostics_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self.query_memory_diagnostics_handle = open(
                self.query_memory_diagnostics_path, "a", buffering=1)
        self.temporal_maximum_age_sec = float(
            rospy.get_param("~temporal_maximum_age_sec", 0.30))
        if self.temporal_feature_cache and self.temporal_sweeps <= 0:
            raise ValueError(
                "temporal_feature_cache requires temporal_sweeps > 0")
        if self.lidar_deskew_mode not in ("off", "ego_pose_2d"):
            raise ValueError("lidar_deskew_mode must be off or ego_pose_2d")
        if self.lidar_deskew_reference not in ("end", "median", "header"):
            raise ValueError("lidar_deskew_reference must be end, median, or header")
        if self.lidar_deskew_pose_type not in ("odometry", "ego_status"):
            raise ValueError("lidar_deskew_pose_type must be odometry or ego_status")
        self.score_threshold, self.class_score_thresholds = build_score_thresholds(
            rospy.get_param("~score_threshold", 0.10),
            rospy.get_param("~vehicle_score_threshold", 0.40),
            rospy.get_param("~pedestrian_score_threshold", 0.15),
            rospy.get_param("~obstacle_score_threshold", 0.10),
        )
        self.model_score_threshold = server_score_floor(
            self.score_threshold, self.class_score_thresholds
        )
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.rotated_bev_nms_enabled = bool(
            rospy.get_param("~rotated_bev_nms_enabled", True)
        )
        self.rotated_bev_nms_iou_threshold = float(
            rospy.get_param("~rotated_bev_nms_iou_threshold", 0.50)
        )
        if not 0.0 <= self.rotated_bev_nms_iou_threshold <= 1.0:
            raise ValueError(
                "rotated_bev_nms_iou_threshold must be in [0, 1]")
        self.publish_detector_velocity = bool(
            rospy.get_param("~publish_detector_velocity", False)
        )
        self.detector_velocity_noise_std_mps = float(
            rospy.get_param("~detector_velocity_noise_std_mps", 1.0)
        )
        self.detector_maximum_velocity_mps = float(
            rospy.get_param("~detector_maximum_velocity_mps", 60.0)
        )
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
            self.lidar_position,
            self.lidar_yaw,
        ) = self._load_topics()
        self.ego_pose_buffer = PlanarPoseBuffer(duration_sec=2.0)
        self.temporal_sweep_buffer = collections.deque(
            maxlen=max(1, self.temporal_sweeps))
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
        self.dropped_deskew_timing = 0
        self.dropped_deskew_pose = 0
        self.nms_suppressed_total = 0
        self.last_marker_count = 0
        self.shutting_down = False
        self.frame_timing = None
        self.frame_timing_pub = None
        if bool(rospy.get_param('~publish_frame_timing', False)):
            self.frame_timing = FrameTimingLedger(capacity=16)
            self.frame_timing_pub = rospy.Publisher(
                '/perception/bevfusion/frame_timing', String, queue_size=1)

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
        if (
            self.lidar_deskew_mode == "ego_pose_2d"
            or self.temporal_sweeps > 0
            or self.query_memory_context
        ):
            pose_message_type = (
                Odometry
                if self.lidar_deskew_pose_type == "odometry"
                else EgoVehicleStatus
            )
            self.subscribers.append(
                rospy.Subscriber(
                    self.lidar_deskew_pose_topic,
                    pose_message_type,
                    self._pose_callback,
                    queue_size=100,
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
            "BEVFusion bridge: operation=%s, lidar=%s, cameras=%s, "
            "temporal_sweeps=%d, temporal_feature_cache=%s, "
            "rotated_bev_nms=%s@%.2f",
            self.operation,
            self.lidar_topic,
            self.camera_topics,
            self.temporal_sweeps,
            self.temporal_feature_cache,
            self.rotated_bev_nms_enabled,
            self.rotated_bev_nms_iou_threshold,
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
        lidar_position = tuple(
            float(lidars[0].get("pos", {}).get(axis, 0.0))
            for axis in ("x", "y", "z")
        )
        lidar_yaw = math.radians(
            float(lidars[0].get("rot", {}).get("yaw", 0.0))
        )
        return (
            {frame: cameras[frame] for frame in self.camera_order},
            {frame: shapes[frame] for frame in self.camera_order},
            lidar_topic,
            lidar_position,
            lidar_yaw,
        )

    def _pose_callback(self, message):
        stamp = stamp_seconds(message)
        if self.lidar_deskew_pose_type == "odometry":
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
        else:
            position = message.position
            yaw = math.radians(float(message.heading))
        self.ego_pose_buffer.add(
            stamp, position.x, position.y, position.z, yaw
        )

    def _camera_callback(self, message, frame):
        with self.buffer_lock:
            self.camera_buffers[frame].append(message)

    def _lidar_callback(self, message):
        ledger = getattr(self, 'frame_timing', None)
        ticket = ledger.begin(message.header.stamp.to_nsec(), message.header.frame_id,
                              rospy.Time.now().to_nsec()) if ledger else None
        lidar_stamp = stamp_seconds(message)
        timing = None
        point_time_offsets = None
        reference_stamp = lidar_stamp
        if self.lidar_deskew_mode == "ego_pose_2d":
            try:
                point_time_offsets = pointcloud_field_array(
                    message, "time", np.float64
                )
            except ValueError as error:
                if ledger:
                    ledger.count('deskew_timing_reject')
                self.dropped_deskew_timing += 1
                rospy.logwarn_throttle(2.0, "BEVFusion deskew rejected: %s", error)
                return
            timing = scan_time_contract(
                point_time_offsets,
                self.lidar_deskew_max_point_time_span_sec,
                self.lidar_deskew_reference,
            )
            if timing is None:
                if ledger:
                    ledger.count('deskew_timing_reject')
                self.dropped_deskew_timing += 1
                rospy.logwarn_throttle(
                    2.0, "BEVFusion deskew rejected invalid point-time span"
                )
                return
            reference_stamp += timing["reference_offset_sec"]
        selected = {}
        differences = {}
        with self.buffer_lock:
            for frame, buffer in self.camera_buffers.items():
                if not buffer:
                    if ledger:
                        ledger.count('camera_empty_reject')
                    return
                nearest = min(
                    buffer,
                    key=lambda item: abs(stamp_seconds(item) - reference_stamp),
                )
                difference = abs(stamp_seconds(nearest) - reference_stamp)
                if difference > self.sync_tolerance:
                    if ledger:
                        ledger.count('camera_sync_reject')
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
                if ledger:
                    ledger.discard(self.raw_pending[-1], 'raw_overwrite')
            self.raw_pending = (
                message,
                selected,
                differences,
                timing,
                point_time_offsets,
                reference_stamp,
                time.monotonic(),
            )
            if ledger:
                ledger.accept(ticket, self.raw_pending[-1], reference_stamp, differences)
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
                ledger = getattr(self, 'frame_timing', None)
                if ledger:
                    ledger.stage(frame[-1], 'preprocess_start')
                prepared = self._prepare_frame(*frame)
                if ledger:
                    ledger.stage(frame[-1], 'preprocess_end')
                with self.pipeline_condition:
                    if self.prepared_pending is not None:
                        self.dropped_prepared += 1
                        if ledger:
                            ledger.discard(self.prepared_pending[-1], 'prepared_overwrite')
                    self.prepared_pending = prepared
                    if ledger:
                        ledger.stage(prepared[-1], 'prepared_enqueued')
                    self.pipeline_condition.notify_all()
            except Exception as error:
                if getattr(self, 'frame_timing', None):
                    self.frame_timing.discard(frame[-1], 'preprocess_error')
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
        self, lidar_message, images, differences, timing, point_time_offsets,
        reference_stamp, received_at
    ):
        start = time.monotonic()
        conversion_start = time.monotonic()
        points = pointcloud_to_xyzi(lidar_message)
        conversion_ms = (time.monotonic() - conversion_start) * 1000.0
        deskew_ms = 0.0
        deskew_displacement = np.empty(0, dtype=np.float64)
        if self.lidar_deskew_mode == "ego_pose_2d":
            deskew_start = time.monotonic()
            deskewed = deskew_points_to_reference_pose(
                points,
                point_time_offsets,
                stamp_seconds(lidar_message),
                timing["reference_offset_sec"],
                self.ego_pose_buffer.snapshot(),
                lidar_position=self.lidar_position,
                lidar_yaw=self.lidar_yaw,
                maximum_extrapolation_sec=(
                    self.lidar_deskew_maximum_extrapolation_sec
                ),
            )
            if deskewed is None:
                self.dropped_deskew_pose += 1
                raise ValueError("deskew pose coverage is insufficient")
            points, deskew_displacement, _reference_pose = deskewed
            deskew_ms = (time.monotonic() - deskew_start) * 1000.0
        points_for_history = np.asarray(points, dtype=np.float32)
        temporal_lidar_pose_world = None
        if self.temporal_feature_cache or self.query_memory_context:
            pose_samples = self.ego_pose_buffer.snapshot()
            temporal_lidar_pose_world = lidar_world_pose_at_time(
                pose_samples,
                reference_stamp,
                lidar_position=self.lidar_position,
                lidar_yaw=self.lidar_yaw,
                maximum_extrapolation_sec=(
                    self.lidar_deskew_maximum_extrapolation_sec),
            )
            if temporal_lidar_pose_world is None:
                raise ValueError("temporal model pose coverage is insufficient")
        if self.temporal_feature_cache:
            points = np.concatenate(
                [points_for_history,
                 np.zeros((len(points_for_history), 1), np.float32)],
                axis=1,
            )
        elif self.temporal_sweeps > 0:
            point_sets = [np.concatenate(
                [points_for_history, np.zeros((len(points_for_history), 1), np.float32)],
                axis=1,
            )]
            pose_samples = self.ego_pose_buffer.snapshot()
            for source_stamp, source_points in reversed(self.temporal_sweep_buffer):
                age = float(reference_stamp) - float(source_stamp)
                if age <= 1.0e-6 or age > self.temporal_maximum_age_sec:
                    continue
                aligned = transform_points_between_lidar_times(
                    source_points,
                    source_stamp,
                    reference_stamp,
                    pose_samples,
                    lidar_position=self.lidar_position,
                    lidar_yaw=self.lidar_yaw,
                    maximum_extrapolation_sec=(
                        self.lidar_deskew_maximum_extrapolation_sec),
                )
                if aligned is None:
                    continue
                point_sets.append(np.concatenate(
                    [aligned.astype(np.float32, copy=False),
                     np.full((len(aligned), 1), age, np.float32)],
                    axis=1,
                ))
                if len(point_sets) >= self.temporal_sweeps + 1:
                    break
            points = np.concatenate(point_sets, axis=0)
            if (self.temporal_sweep_buffer and
                    reference_stamp <= self.temporal_sweep_buffer[-1][0]):
                self.temporal_sweep_buffer.clear()
            self.temporal_sweep_buffer.append(
                (float(reference_stamp), points_for_history.copy()))
        self.sequence += 1
        metadata = {
            "operation": self.operation,
            "sequence": self.sequence,
            "stamp": reference_stamp,
            "lidar_frame": lidar_message.header.frame_id,
            "camera_order": self.camera_order,
            "sensor_config_sha256": self.sensor_config_sha256,
            "intrinsic_source": self.intrinsic_source,
            "sync_difference_sec": differences,
            "points_blob": "points",
            "point_dim": int(points.shape[1]),
            "point_count": int(points.shape[0]),
            "point_intensity_source": self.point_intensity_source,
            "model_image_final_dim": [self.image_final_height, self.image_final_width],
            "score_threshold": self.model_score_threshold,
            "max_detections": self.max_detections,
            "lidar_deskew_mode": self.lidar_deskew_mode,
            "lidar_deskew_ms": deskew_ms,
            "lidar_deskew_span_sec": 0.0 if timing is None else timing["span_sec"],
            "lidar_deskew_displacement_p95_m": (
                0.0
                if deskew_displacement.size == 0
                else float(np.percentile(deskew_displacement, 95.0))
            ),
        }
        if temporal_lidar_pose_world is not None:
            metadata["temporal_feature_cache"] = self.temporal_feature_cache
            metadata["query_memory_context"] = self.query_memory_context
            metadata["lidar_pose_world"] = [
                float(value) for value in temporal_lidar_pose_world
            ]
        blobs = {"points": points.astype("<f4", copy=False).tobytes()}
        if self.operation == "inference" and self.image_encoding == 'rgb8':
            resized = dict(self.camera_executor.map(
                self._preprocess_camera,
                [(frame, images[frame]) for frame in self.camera_order],
            ))
            for frame, image in resized.items():
                blobs["image/{}".format(frame)] = image.tobytes()
            metadata.update({
                "image_encoding": "rgb8",
                "image_height": self.image_final_height,
                "image_width": self.image_final_width,
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
        return metadata, blobs, rospy.Time.from_sec(reference_stamp), received_at

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
        scale = self.image_resize * self.image_reference_width / width
        new_width, new_height = int(width * scale), int(height * scale)
        image = cv2.resize(
            image,
            (new_width, new_height),
            interpolation=cv2.INTER_CUBIC,
        )
        crop_top = new_height - self.image_final_height
        crop_left = max(0, new_width - self.image_final_width) // 2
        if crop_top < 0 or crop_left + self.image_final_width > new_width:
            raise ValueError('Resize cannot produce requested model crop')
        image = image[crop_top:crop_top + self.image_final_height,
                      crop_left:crop_left + self.image_final_width]
        if image.shape[:2] != (self.image_final_height, self.image_final_width):
            raise ValueError(
                "{} preprocessed shape is {}".format(frame, image.shape[:2])
            )
        return frame, np.ascontiguousarray(image)

    def _request_and_publish(self, prepared):
        ledger = getattr(self, 'frame_timing', None)
        if ledger:
            ledger.bind_request(prepared[-1])
        try:
            self._request_and_publish_body(prepared)
        except Exception:
            if ledger:
                ledger.discard(prepared[-1], 'request_error')
            raise
        finally:
            if ledger:
                ledger.unbind_request()

    def _request_and_publish_body(self, prepared):
        metadata, blobs, stamp, received_at = prepared
        ledger = getattr(self, 'frame_timing', None)
        request_start = time.monotonic()
        if ledger:
            ledger.stage(received_at, 'request_start', request_start)
        response, response_blobs = self.transport.request(metadata, blobs)
        transport_ms = (time.monotonic() - request_start) * 1000.0
        latency_ms = (time.monotonic() - received_at) * 1000.0
        self.latency_pub.publish(Float32(data=latency_ms))
        if ledger:
            ledger.stage(received_at, 'response_returned', request_start+transport_ms/1000.)
        if response.get("operation") == "projection":
            self._publish_projection(response, response_blobs)
        else:
            ros_response = response_for_ros(
                response, getattr(self, 'box_center_output_origin', LEGACY_OUTPUT))
            self._publish_detections(ros_response, stamp)
            self._write_query_memory_diagnostics(response, metadata)
        self._publish_status(
            "ok operation={} sequence={} latency_ms={:.1f} host_preprocess_ms={:.1f} "
            "pointcloud_ms={:.1f} server_preprocess_ms={:.1f} "
            "inference_ms={:.1f} request_ms={:.1f} dropped_raw={} "
            "dropped_prepared={} deskew_mode={} deskew_ms={:.2f} "
            "deskew_span_ms={:.2f} deskew_displacement_p95_m={:.3f} "
            "dropped_deskew_timing={} dropped_deskew_pose={} "
            "nms_suppressed_total={}".format(
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
                metadata.get("lidar_deskew_mode", "off"),
                float(metadata.get("lidar_deskew_ms", 0.0)),
                float(metadata.get("lidar_deskew_span_sec", 0.0)) * 1000.0,
                float(metadata.get("lidar_deskew_displacement_p95_m", 0.0)),
                self.dropped_deskew_timing,
                self.dropped_deskew_pose,
                self.nms_suppressed_total,
            )
        )
        if ledger:
            self._publish_frame_timing(prepared, response, latency_ms)

    def _publish_frame_timing(self, prepared, response, latency_ms):
        # Diagnostics must never turn a successful detector frame into an
        # inference error or alter the legacy Float32/metadata time definitions.
        metadata, _blobs, stamp, received_at = prepared
        try:
            record = self.frame_timing.finish(received_at, metadata.get('sequence', 0),
                stamp.to_nsec(), rospy.Time.now().to_nsec(), metadata, response, latency_ms)
            if record is not None:
                self.frame_timing_pub.publish(String(data=json.dumps(record, allow_nan=False, separators=(',', ':'))))
        except Exception as error:
            self.frame_timing.count('diagnostic_error')
            rospy.logwarn_throttle(2., 'BEVFusion frame timing diagnostic failed: %s', error)

    def _write_query_memory_diagnostics(self, response, request_metadata):
        if self.query_memory_diagnostics_handle is None:
            return
        diagnostics = response.get("query_memory_diagnostics")
        if not diagnostics:
            return
        record = {
            "sequence": int(request_metadata.get("sequence", 0)),
            "stamp": float(response.get("stamp", 0.0)),
            "diagnostics": diagnostics,
        }
        self.query_memory_diagnostics_handle.write(
            json.dumps(record, sort_keys=True) + "\n")

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
        detections = filter_detections(
            response.get("detections", []),
            self.score_threshold,
            self.class_score_thresholds,
        )
        if self.rotated_bev_nms_enabled:
            detection_count_before_nms = len(detections)
            detections = class_aware_rotated_bev_nms(
                detections, self.rotated_bev_nms_iou_threshold)
            self.nms_suppressed_total += detection_count_before_nms - len(detections)
        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        header = delete_all.header
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)
        boxes = []
        for detection in detections:
            box_values = (
                list(detection["center"])
                + list(detection["size"])
                + [float(detection["yaw"])]
            )
            velocity = detection.get("velocity")
            if velocity is not None and len(velocity) >= 2:
                box_values.extend([float(velocity[0]), float(velocity[1])])
            boxes.append(box_values)
        self.objects_pub.publish(
            make_detected_objects(
                header,
                boxes,
                [float(detection.get("score", 0.0)) for detection in detections],
                [str(detection.get("class_name", "unknown")) for detection in detections],
                include_velocity=self.publish_detector_velocity,
                velocity_classes=("vehicle",),
                velocity_noise_std_mps=self.detector_velocity_noise_std_mps,
                maximum_velocity_mps=self.detector_maximum_velocity_mps,
            )
        )
        if getattr(self, 'frame_timing', None):
            self.frame_timing.output_published(rospy.Time.now().to_nsec())
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
        if getattr(self, 'frame_timing', None):
            self.frame_timing.clear()
        self.transport.close()
        if self.query_memory_diagnostics_handle is not None:
            self.query_memory_diagnostics_handle.close()
            self.query_memory_diagnostics_handle = None
        with self.pipeline_condition:
            self.pipeline_condition.notify_all()
        self.camera_executor.shutdown(wait=False)


def stamp_seconds(message):
    stamp = message.header.stamp
    if stamp.is_zero():
        return rospy.Time.now().to_sec()
    return stamp.to_sec()


def pointcloud_field_array(message, name, output_dtype=None):
    """Return one PointCloud2 field as a strided vector without Python loops."""
    fields = {field.name: field for field in message.fields}
    if name not in fields:
        raise ValueError(
            "PointCloud2 field {} is missing; fields={}".format(
                name, sorted(fields)
            )
        )
    field = fields[name]
    if field.datatype not in POINTFIELD_TO_DTYPE or int(field.count) != 1:
        raise ValueError(
            "unsupported PointField datatype/count for {}: {}/{}".format(
                name, field.datatype, field.count
            )
        )
    endian = ">" if message.is_bigendian else "<"
    count = int(message.width) * int(message.height)
    if count <= 0:
        return np.empty(0, dtype=output_dtype or np.float32)
    values = np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=np.dtype(endian + POINTFIELD_TO_DTYPE[field.datatype]),
        buffer=message.data,
        offset=int(field.offset),
        strides=(int(message.row_step), int(message.point_step)),
    ).reshape(-1)
    if output_dtype is not None:
        return values.astype(output_dtype, copy=False)
    return values


def pointcloud_to_xyzi_time(message):
    """Vectorize PointCloud2 into model XYZI and optional timing arrays."""
    points = pointcloud_to_xyzi(message)
    fields = {field.name for field in message.fields}
    count = len(points)
    offsets = (
        pointcloud_field_array(message, "time", np.float64)
        if "time" in fields
        else np.full(count, np.nan, dtype=np.float64)
    )
    return points, offsets


def pointcloud_to_xyzi(message):
    fields = {field.name for field in message.fields}
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError("PointCloud2 field {} is missing".format(required))
    count = int(message.width) * int(message.height)
    points = np.zeros((count, 4), dtype=np.float32)
    for index, name in enumerate(("x", "y", "z")):
        points[:, index] = pointcloud_field_array(message, name, np.float32)
    if "intensity" in fields:
        points[:, 3] = pointcloud_field_array(message, "intensity", np.float32)
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
