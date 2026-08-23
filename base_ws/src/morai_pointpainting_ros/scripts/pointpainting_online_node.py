#!/usr/bin/env python3
"""
ROS1 Noetic online PointPainting node for MORAI.

Data flow:
4 camera Image topics
    -> latest-before-LiDAR temporal matching
    -> LRASPP semantic segmentation
LiDAR PointCloud2
    -> camera projection and semantic painting
    -> 9-channel MORAI painted points (or legacy 10-channel points)
    -> OpenPCDet PointPillars
    -> custom 3D detection array + RViz markers

The LiDAR subscriber never performs heavy work. It stores only the newest scan.
A worker thread processes one scan at a time and overwrites stale queued scans,
which prevents latency from growing without bound.
"""

from __future__ import annotations

import copy
import csv
import hashlib
import json
import math
import os
import sys
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import cv2
import rospy
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from PIL import Image as PILImage
from sensor_msgs.msg import (
    CompressedImage,
    Image,
    PointCloud2,
    PointField,
)
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from morai_msgs.msg import EgoVehicleStatus

from morai_pointpainting_ros.msg import (
    PointPaintingDetection,
    PointPaintingDetectionArray,
)
from autoware_perception_msgs.msg import DetectedObjects
from perception_object_pipeline import make_detected_objects
from perception_object_pipeline.rolling_lidar import (
    PlanarPoseBuffer,
    deskew_points_to_reference_pose,
    representative_detection_time_offset,
    transform_boxes_between_lidar_times,
    transform_points_between_lidar_times,
)


LEGACY_SEMANTIC_NAMES = (
    "background",
    "person",
    "vehicle",
    "two_wheeler",
    "dog",
    "unknown",
)

MORAI_4CLASS_SEMANTIC_NAMES = (
    "background",
    "pedestrian",
    "vehicle",
    "obstacle",
    "unknown",
)

R_OPTICAL_FROM_BODY = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)

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


def expand_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def file_identity(path: Optional[Path]) -> Optional[Dict[str, object]]:
    """Return a reproducible artifact identity outside measured frame timing."""
    if path is None:
        return None
    identity: Dict[str, object] = {"path": str(path)}
    try:
        stat = path.stat()
        digest = hashlib.sha256()
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
        identity.update(
            {
                "size_bytes": int(stat.st_size),
                "sha256": digest.hexdigest(),
            }
        )
    except OSError as error:
        identity["identity_error"] = str(error)
    return identity


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ],
        dtype=np.float64,
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ],
        dtype=np.float64,
    )
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return rz @ ry @ rx


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half = 0.5 * float(yaw)
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half),
    )


def stamp_to_sec(stamp) -> float:
    value = float(stamp.to_sec())
    if value <= 0.0:
        value = float(rospy.get_rostime().to_sec())
    return value


def cloud_to_xyzi(msg: PointCloud2, include_time: bool = False):
    fields = {field.name: field for field in msg.fields}

    for required in ("x", "y", "z"):
        if required not in fields:
            raise RuntimeError(
                "PointCloud2 field '{}' is missing. fields={}".format(
                    required,
                    sorted(fields.keys()),
                )
            )

    names = ["x", "y", "z"]
    if "intensity" in fields:
        names.append("intensity")
    has_time = include_time and "time" in fields
    if has_time:
        names.append("time")

    endian = ">" if msg.is_bigendian else "<"
    dtype_names = []
    dtype_formats = []
    dtype_offsets = []

    for name in names:
        field = fields[name]
        if field.datatype not in POINTFIELD_TO_DTYPE:
            raise RuntimeError(
                "Unsupported PointField datatype for {}: {}".format(
                    name,
                    field.datatype,
                )
            )
        dtype_names.append(name)
        dtype_formats.append(endian + POINTFIELD_TO_DTYPE[field.datatype])
        dtype_offsets.append(int(field.offset))

    dtype = np.dtype(
        {
            "names": dtype_names,
            "formats": dtype_formats,
            "offsets": dtype_offsets,
            "itemsize": int(msg.point_step),
        }
    )

    count = int(msg.width) * int(msg.height)
    if count <= 0:
        empty_points = np.empty((0, 4), dtype=np.float32)
        if include_time:
            return empty_points, np.empty((0,), dtype=np.float32)
        return empty_points

    raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    output = np.zeros((count, 4), dtype=np.float32)
    output[:, 0] = raw["x"]
    output[:, 1] = raw["y"]
    output[:, 2] = raw["z"]

    if "intensity" in names:
        output[:, 3] = raw["intensity"].astype(np.float32, copy=False)

    if include_time:
        offsets = (
            raw["time"].astype(np.float32, copy=False)
            if has_time
            else np.full((count,), np.nan, dtype=np.float32)
        )
        return output, offsets
    return output


def make_painted_cloud(
    points: np.ndarray,
    header: Header,
    semantic_names: Sequence[str],
) -> PointCloud2:
    names = [
        "x",
        "y",
        "z",
        "intensity",
    ] + list(semantic_names)
    if points.ndim != 2 or points.shape[1] != len(names):
        raise RuntimeError(
            "Painted points must have shape (N, {}), got {}".format(
                len(names), points.shape
            )
        )

    output = PointCloud2()
    output.header = copy.copy(header)
    output.height = 1
    output.width = int(len(points))
    output.fields = [
        PointField(name, index * 4, PointField.FLOAT32, 1)
        for index, name in enumerate(names)
    ]
    output.is_bigendian = False
    output.point_step = 4 * len(names)
    output.row_step = output.point_step * output.width
    output.is_dense = bool(np.isfinite(points).all())
    output.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return output


def box_corners(box: Sequence[float]) -> np.ndarray:
    x, y, z, dx, dy, dz, yaw = [float(value) for value in box[:7]]
    corners = np.array(
        [
            [dx / 2, dy / 2, -dz / 2],
            [dx / 2, -dy / 2, -dz / 2],
            [-dx / 2, -dy / 2, -dz / 2],
            [-dx / 2, dy / 2, -dz / 2],
            [dx / 2, dy / 2, dz / 2],
            [dx / 2, -dy / 2, dz / 2],
            [-dx / 2, -dy / 2, dz / 2],
            [-dx / 2, dy / 2, dz / 2],
        ],
        dtype=np.float32,
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array(
        [[cosine, -sine], [sine, cosine]],
        dtype=np.float32,
    )
    corners[:, :2] = corners[:, :2] @ rotation.T
    corners += np.array([x, y, z], dtype=np.float32)
    return corners


def add_marker_point(marker: Marker, xyz: Sequence[float]) -> None:
    marker.points.append(
        Point(
            x=float(xyz[0]),
            y=float(xyz[1]),
            z=float(xyz[2]),
        )
    )


def make_markers(
    boxes: np.ndarray,
    scores: np.ndarray,
    names: Sequence[str],
    header: Header,
) -> MarkerArray:
    output = MarkerArray()

    delete_all = Marker()
    delete_all.header = copy.copy(header)
    delete_all.action = Marker.DELETEALL
    output.markers.append(delete_all)

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    for index, (box, score, name) in enumerate(zip(boxes, scores, names)):
        corners = box_corners(box)

        line = Marker()
        line.header = copy.copy(header)
        line.ns = "pointpainting_boxes"
        line.id = int(index)
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.08
        line.color.r = 1.0
        line.color.g = 0.1
        line.color.b = 0.1
        line.color.a = 1.0
        line.lifetime = rospy.Duration(0.25)

        for start, end in edges:
            add_marker_point(line, corners[start])
            add_marker_point(line, corners[end])

        output.markers.append(line)

        label = Marker()
        label.header = copy.copy(header)
        label.ns = "pointpainting_labels"
        label.id = int(index)
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.x = float(box[0])
        label.pose.position.y = float(box[1])
        label.pose.position.z = float(box[2] + box[5] / 2.0 + 0.3)
        label.scale.z = 0.35
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "{} {:.2f}".format(name, score)
        label.lifetime = rospy.Duration(0.25)
        output.markers.append(label)

    return output


class PointPaintingOnlineNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.openpcdet_root = expand_path(
            rospy.get_param("~openpcdet_root", "~/OpenPCDet")
        )
        self.tools_dir = self.openpcdet_root / "tools"
        self.cfg_file = expand_path(
            rospy.get_param(
                "~cfg_file",
                "~/OpenPCDet/tools/cfgs/custom_models/"
                "pointpillar_morai_pointpainting.yaml",
            )
        )
        self.checkpoint = expand_path(
            rospy.get_param(
                "~checkpoint",
                "~/OpenPCDet/output/custom_models/"
                "pointpillar_morai_pointpainting/"
                "pointpainting_20epoch/ckpt/checkpoint_epoch_20.pth",
            )
        )
        segmentation_checkpoint_value = str(
            rospy.get_param("~segmentation_checkpoint", "")
        ).strip()
        self.segmentation_checkpoint = (
            expand_path(segmentation_checkpoint_value)
            if segmentation_checkpoint_value
            else None
        )
        self.semantic_profile = str(
            rospy.get_param(
                "~semantic_profile",
                "morai_4class" if self.segmentation_checkpoint else "legacy_voc",
            )
        ).strip().lower()
        if self.semantic_profile not in ("morai_4class", "legacy_voc"):
            raise ValueError(
                "semantic_profile must be morai_4class or legacy_voc"
            )

        self.lidar_topic = rospy.get_param("~lidar_topic", "/sensor/lidar1")
        self.detections_topic = rospy.get_param(
            "~detections_topic",
            "/pointpainting/detections",
        )
        self.markers_topic = rospy.get_param(
            "~markers_topic",
            "/pointpainting/detection_markers",
        )
        self.painted_points_topic = rospy.get_param(
            "~painted_points_topic",
            "/pointpainting/painted_points",
        )

        self.score_threshold = float(
            rospy.get_param("~score_threshold", 0.10)
        )
        self.max_detection_range = float(
            rospy.get_param("~max_detection_range", 30.0)
        )
        self.publish_detector_velocity = bool(
            rospy.get_param("~publish_detector_velocity", False)
        )
        self.detector_velocity_classes = tuple(
            str(value).strip().lower()
            for value in rospy.get_param(
                "~detector_velocity_classes", ["vehicle"]
            )
        )
        self.detector_velocity_noise_std_mps = float(
            rospy.get_param("~detector_velocity_noise_std_mps", 1.0)
        )
        self.detector_maximum_velocity_mps = float(
            rospy.get_param("~detector_maximum_velocity_mps", 30.0)
        )
        self.max_camera_age_ms = float(
            rospy.get_param("~max_camera_age_ms", 100.0)
        )
        self.max_camera_span_ms = float(
            rospy.get_param("~max_camera_span_ms", 80.0)
        )
        self.camera_buffer_size = int(
            rospy.get_param("~camera_buffer_size", 30)
        )
        self.camera_cpu_workers = int(
            rospy.get_param("~camera_cpu_workers", 1)
        )
        self.publish_painted_points = bool(
            rospy.get_param("~publish_painted_points", False)
        )
        self.lidar_measurement_stamp_mode = str(
            rospy.get_param(
                "~lidar_measurement_stamp_mode",
                "object_points_median",
            )
        ).strip().lower()
        self.lidar_timing_box_extent_scale = float(
            rospy.get_param("~lidar_timing_box_extent_scale", 0.65)
        )
        self.lidar_timing_minimum_points_per_box = int(
            rospy.get_param("~lidar_timing_minimum_points_per_box", 3)
        )
        self.lidar_deskew_mode = str(
            rospy.get_param("~lidar_deskew_mode", "off")
        ).strip().lower()
        self.lidar_deskew_reference = str(
            rospy.get_param("~lidar_deskew_reference", "point_median")
        ).strip().lower()
        self.lidar_deskew_camera_reference = str(
            rospy.get_param("~lidar_deskew_camera_reference", "reference")
        ).strip().lower()
        self.ego_topic = str(rospy.get_param("~ego_topic", "/Ego_topic"))
        self.ego_pose_buffer_duration_sec = float(
            rospy.get_param("~ego_pose_buffer_duration_sec", 2.0)
        )
        self.lidar_deskew_maximum_extrapolation_sec = float(
            rospy.get_param("~lidar_deskew_maximum_extrapolation_sec", 0.08)
        )
        self.requested_temporal_num_sweeps = int(
            rospy.get_param("~temporal_num_sweeps", 0)
        )
        self.temporal_maximum_gap_sec = float(
            rospy.get_param("~temporal_maximum_gap_sec", 0.75)
        )
        self.temporal_lidar_scan_period_sec = float(
            rospy.get_param("~temporal_lidar_scan_period_sec", 1.0 / 15.0)
        )
        self.temporal_time_lag_mode = str(
            rospy.get_param("~temporal_time_lag_mode", "sweep_constant")
        ).strip().lower()
        self.use_amp = bool(rospy.get_param("~use_amp", True))
        self.segmentation_short_side = int(
            rospy.get_param("~segmentation_short_side", 520)
        )
        self.segmentation_preprocess_backend = str(
            rospy.get_param(
                "~segmentation_preprocess_backend",
                "torchvision",
            )
        ).strip().lower()
        self.warmup_segmentation = bool(
            rospy.get_param("~warmup_segmentation", True)
        )
        self.warmup_full_pipeline = bool(
            rospy.get_param("~warmup_full_pipeline", True)
        )
        self.warmup_iterations = int(
            rospy.get_param("~warmup_iterations", 2)
        )
        self.warmup_point_count = int(
            rospy.get_param("~warmup_point_count", 12000)
        )
        self.real_input_warmup = bool(
            rospy.get_param("~real_input_warmup", True)
        )
        self.real_input_warmup_iterations = int(
            rospy.get_param("~real_input_warmup_iterations", 2)
        )
        benchmark_output_value = str(
            rospy.get_param("~benchmark_output_dir", "")
        ).strip()
        self.benchmark_output_dir = (
            expand_path(benchmark_output_value)
            if benchmark_output_value
            else None
        )
        self.benchmark_label = str(
            rospy.get_param("~benchmark_label", "pointpainting")
        )
        self.benchmark_exclude_frames = int(
            rospy.get_param("~benchmark_exclude_frames", 10)
        )
        self.detailed_timing = bool(
            rospy.get_param(
                "~detailed_timing",
                self.benchmark_output_dir is not None,
            )
        )
        if self.segmentation_short_side < 224:
            raise ValueError(
                "segmentation_short_side must be >= 224"
            )
        if self.segmentation_preprocess_backend not in {
            "torchvision",
            "opencv",
        }:
            raise ValueError(
                "segmentation_preprocess_backend must be torchvision or "
                "opencv"
            )
        if self.camera_cpu_workers < 1 or self.camera_cpu_workers > 4:
            raise ValueError("camera_cpu_workers must be between 1 and 4")
        if self.detector_velocity_noise_std_mps < 0.0:
            raise ValueError("detector_velocity_noise_std_mps must be non-negative")
        if self.detector_maximum_velocity_mps <= 0.0:
            raise ValueError("detector_maximum_velocity_mps must be positive")
        if self.lidar_measurement_stamp_mode not in {
            "header",
            "point_median",
            "object_points_median",
        }:
            raise ValueError(
                "lidar_measurement_stamp_mode must be header, point_median, "
                "or object_points_median"
            )
        if self.lidar_timing_box_extent_scale <= 0.0:
            raise ValueError("lidar_timing_box_extent_scale must be positive")
        if self.lidar_timing_minimum_points_per_box < 1:
            raise ValueError(
                "lidar_timing_minimum_points_per_box must be >= 1"
            )
        if self.lidar_deskew_mode not in {"off", "ego_pose_2d"}:
            raise ValueError("lidar_deskew_mode must be off or ego_pose_2d")
        if self.lidar_deskew_reference not in {"header", "point_median"}:
            raise ValueError(
                "lidar_deskew_reference must be header or point_median"
            )
        if self.lidar_deskew_camera_reference not in {"header", "reference"}:
            raise ValueError(
                "lidar_deskew_camera_reference must be header or reference"
            )
        if self.ego_pose_buffer_duration_sec < 0.2:
            raise ValueError("ego_pose_buffer_duration_sec must be >= 0.2")
        if self.lidar_deskew_maximum_extrapolation_sec < 0.0:
            raise ValueError(
                "lidar_deskew_maximum_extrapolation_sec must be non-negative"
            )
        if self.requested_temporal_num_sweeps < 0:
            raise ValueError("temporal_num_sweeps must be >= 0")
        if self.temporal_maximum_gap_sec <= 0.0:
            raise ValueError("temporal_maximum_gap_sec must be positive")
        if self.temporal_lidar_scan_period_sec <= 0.0:
            raise ValueError("temporal_lidar_scan_period_sec must be positive")
        if self.temporal_time_lag_mode not in {
            "sweep_constant",
            "azimuth_approx",
        }:
            raise ValueError(
                "temporal_time_lag_mode must be sweep_constant or azimuth_approx"
            )
        if self.warmup_iterations < 1:
            raise ValueError("warmup_iterations must be >= 1")
        if self.warmup_point_count < 1000:
            raise ValueError("warmup_point_count must be >= 1000")
        if self.real_input_warmup_iterations < 1:
            raise ValueError(
                "real_input_warmup_iterations must be >= 1"
            )
        if self.benchmark_exclude_frames < 0:
            raise ValueError("benchmark_exclude_frames must be >= 0")

        self.normalize_intensity = bool(
            rospy.get_param("~normalize_intensity", True)
        )
        self.remove_ego_points = bool(
            rospy.get_param("~remove_ego_points", True)
        )
        self.ego_box = np.asarray(
            rospy.get_param(
                "~ego_box",
                [-1.5, 3.5, -1.25, 1.25, -1.0, 2.2],
            ),
            dtype=np.float32,
        )

        self.lidar_config = rospy.get_param("~lidar")
        self.camera_configs = rospy.get_param("~cameras")
        self.camera_names = [
            "front",
            "front_left",
            "front_right",
            "back",
        ]
        self.camera_executor = (
            ThreadPoolExecutor(
                max_workers=self.camera_cpu_workers,
                thread_name_prefix="pointpainting_camera",
            )
            if self.camera_cpu_workers > 1
            else None
        )

        missing_cameras = [
            name
            for name in self.camera_names
            if name not in self.camera_configs
        ]
        if missing_cameras:
            raise RuntimeError(
                "Camera config missing: {}".format(missing_cameras)
            )

        self.lidar_position = np.asarray(
            self.lidar_config["position"],
            dtype=np.float64,
        )
        self.r_vehicle_lidar = rotation_matrix(
            *self.lidar_config["rotation"]
        )
        self.lidar_yaw_rad = math.radians(
            float(self.lidar_config["rotation"][2])
        )
        self.ego_pose_buffer = PlanarPoseBuffer(
            self.ego_pose_buffer_duration_sec
        )

        self.camera_buffers = {
            name: deque(maxlen=self.camera_buffer_size)
            for name in self.camera_names
        }
        self.camera_lock = threading.Lock()

        self.latest_lidar = None
        self.lidar_lock = threading.Lock()
        self.worker_event = threading.Event()

        self.dropped_lidar_frames = 0
        self.skipped_lidar_frames = 0
        self.processed_frames = 0
        self.lidar_input_frames = 0
        self.real_input_warmup_frames = 0
        self.skip_reasons = {
            "camera_sync": 0,
            "empty_lidar": 0,
            "temporal_history": 0,
            "processing_error": 0,
        }
        self.camera_input_frames = {
            name: 0 for name in self.camera_names
        }
        self.camera_first_wall_time = {
            name: None for name in self.camera_names
        }
        self.camera_last_wall_time = {
            name: None for name in self.camera_names
        }
        self.lidar_first_wall_time = None
        self.lidar_last_wall_time = None
        self.publish_first_wall_time = None
        self.publish_last_wall_time = None
        self.benchmark_rows = []
        self.benchmark_report_written = False
        self.real_input_warmup_complete = not self.real_input_warmup
        self.online_ready_logged = False
        self.logged_segmentation_shapes = set()

        self._load_segmentation_model()
        self._load_detection_model()
        if self.warmup_full_pipeline:
            self._warmup_full_pipeline()

        self.detections_pub = rospy.Publisher(
            self.detections_topic,
            PointPaintingDetectionArray,
            queue_size=1,
        )
        self.objects_pub = rospy.Publisher(
            "~output/objects",
            DetectedObjects,
            queue_size=1,
        )
        self.markers_pub = rospy.Publisher(
            self.markers_topic,
            MarkerArray,
            queue_size=1,
        )
        self.painted_pub = rospy.Publisher(
            self.painted_points_topic,
            PointCloud2,
            queue_size=1,
        )

        self.camera_subscribers = []
        self.ego_subscriber = rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self._ego_callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        for camera_name in self.camera_names:
            camera_config = self.camera_configs[camera_name]
            topic = camera_config["topic"]
            compressed = bool(
                camera_config.get(
                    "compressed",
                    str(topic).endswith("/compressed"),
                )
            )
            message_type = CompressedImage if compressed else Image

            subscriber = rospy.Subscriber(
                topic,
                message_type,
                self._camera_callback,
                callback_args=camera_name,
                queue_size=2,
                buff_size=16 * 1024 * 1024,
                tcp_nodelay=True,
            )
            self.camera_subscribers.append(subscriber)
            rospy.loginfo(
                "Camera %-12s <- %s (%s)",
                camera_name,
                topic,
                "CompressedImage" if compressed else "Image",
            )

        self.lidar_subscriber = rospy.Subscriber(
            self.lidar_topic,
            PointCloud2,
            self._lidar_callback,
            queue_size=1,
            buff_size=64 * 1024 * 1024,
            tcp_nodelay=True,
        )

        self.worker = threading.Thread(
            target=self._worker_loop,
            name="pointpainting_worker",
            daemon=True,
        )
        self.worker.start()
        rospy.on_shutdown(self._write_benchmark_report)
        rospy.on_shutdown(self._shutdown_camera_executor)

        rospy.loginfo("LiDAR       <- %s", self.lidar_topic)
        rospy.loginfo(
            "Ego pose    <- %s (deskew=%s reference=%s camera=%s)",
            self.ego_topic,
            self.lidar_deskew_mode,
            self.lidar_deskew_reference,
            self.lidar_deskew_camera_reference,
        )
        rospy.loginfo("Detections  -> %s", self.detections_topic)
        rospy.loginfo("Markers     -> %s", self.markers_topic)
        rospy.loginfo(
            "score>=%.2f range<=%.1fm camera_age<=%.1fms "
            "AMP=%s seg_short_side=%d preprocess=%s camera_workers=%d "
            "native_semantic_map=True "
            "full_warmup=%s warmup_iterations=%d "
            "real_input_warmup=%s real_input_warmup_iterations=%d",
            self.score_threshold,
            self.max_detection_range,
            self.max_camera_age_ms,
            self.use_amp,
            self.segmentation_short_side,
            self.segmentation_preprocess_backend,
            self.camera_cpu_workers,
            self.warmup_full_pipeline,
            self.warmup_iterations,
            self.real_input_warmup,
            self.real_input_warmup_iterations,
        )
        if self.benchmark_output_dir is not None:
            rospy.loginfo(
                "Benchmark metrics -> %s (detailed_timing=%s, exclude=%d)",
                self.benchmark_output_dir,
                self.detailed_timing,
                self.benchmark_exclude_frames,
            )
        if self.real_input_warmup:
            rospy.loginfo(
                "Waiting for the first synchronized real sensor set "
                "before publishing detections."
            )
        else:
            self._log_online_ready()

    def _load_segmentation_model(self) -> None:
        from torchvision import transforms

        torch.backends.cudnn.benchmark = True
        from torchvision.models.segmentation import (
            lraspp_mobilenet_v3_large,
        )

        try:
            from torchvision.models.segmentation import (
                LRASPP_MobileNet_V3_Large_Weights,
            )
        except ImportError:
            LRASPP_MobileNet_V3_Large_Weights = None

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        if self.device.type != "cuda":
            raise RuntimeError(
                "CUDA GPU is required for the competition pipeline."
            )

        # Training and online inference both preserve aspect ratio, resize the
        # shorter side, convert RGB uint8 to [0, 1], then apply ImageNet
        # normalization. This preprocessing is shared by every semantic
        # profile, including the MORAI 4-class checkpoint.
        self.segmentation_preprocess = transforms.Compose(
            [
                transforms.Resize(
                    self.segmentation_short_side,
                    antialias=True,
                ),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225],
                ),
            ]
        )
        self.segmentation_mean_cpu = torch.tensor(
            [0.485, 0.456, 0.406], dtype=torch.float32
        ).reshape(3, 1, 1)
        self.segmentation_std_cpu = torch.tensor(
            [0.229, 0.224, 0.225], dtype=torch.float32
        ).reshape(3, 1, 1)

        if self.semantic_profile == "morai_4class":
            if self.segmentation_checkpoint is None:
                raise ValueError(
                    "morai_4class requires segmentation_checkpoint"
                )
            if not self.segmentation_checkpoint.exists():
                raise FileNotFoundError(
                    "Segmentation checkpoint not found: {}".format(
                        self.segmentation_checkpoint
                    )
                )

            try:
                self.segmentation_model = lraspp_mobilenet_v3_large(
                    pretrained=False,
                    pretrained_backbone=False,
                    num_classes=4,
                )
            except TypeError:
                self.segmentation_model = lraspp_mobilenet_v3_large(
                    weights=None,
                    weights_backbone=None,
                    num_classes=4,
                )

            checkpoint = torch.load(
                str(self.segmentation_checkpoint),
                map_location="cpu",
            )
            if not isinstance(checkpoint, dict):
                raise RuntimeError(
                    "Unexpected segmentation checkpoint type: {}".format(
                        type(checkpoint).__name__
                    )
                )
            state_dict = checkpoint.get(
                "model_state_dict",
                checkpoint.get("state_dict", checkpoint),
            )
            state_dict = {
                (key[7:] if key.startswith("module.") else key): value
                for key, value in state_dict.items()
            }
            self.segmentation_model.load_state_dict(
                state_dict,
                strict=True,
            )
            self.segmentation_model = self.segmentation_model.to(
                self.device
            ).eval().to(memory_format=torch.channels_last)
            self.semantic_names = MORAI_4CLASS_SEMANTIC_NAMES
            self.segmentation_indices = None
            torchvision_api = "MORAI 4-class checkpoint"
        elif LRASPP_MobileNet_V3_Large_Weights is not None:
            weights = LRASPP_MobileNet_V3_Large_Weights.DEFAULT

            self.segmentation_model = lraspp_mobilenet_v3_large(
                weights=weights
            ).to(self.device).eval().to(
                memory_format=torch.channels_last
            )
            categories = [
                str(value).lower()
                for value in weights.meta["categories"]
            ]
            torchvision_api = "weights enum"
        else:
            self.segmentation_model = lraspp_mobilenet_v3_large(
                pretrained=True
            ).to(self.device).eval().to(
                memory_format=torch.channels_last
            )

            categories = [
                "__background__",
                "aeroplane",
                "bicycle",
                "bird",
                "boat",
                "bottle",
                "bus",
                "car",
                "cat",
                "chair",
                "cow",
                "diningtable",
                "dog",
                "horse",
                "motorbike",
                "person",
                "pottedplant",
                "sheep",
                "sofa",
                "train",
                "tvmonitor",
            ]
            torchvision_api = "legacy pretrained=True"

        if self.semantic_profile == "legacy_voc":
            def category_index(*candidates: str) -> int:
                for candidate in candidates:
                    if candidate in categories:
                        return categories.index(candidate)
                raise RuntimeError(
                    "Segmentation category not found: {} in {}".format(
                        candidates,
                        categories,
                    )
                )

            self.segmentation_indices = {
                "person": category_index("person"),
                "car": category_index("car"),
                "bus": category_index("bus"),
                "bicycle": category_index("bicycle", "bike"),
                "motorbike": category_index(
                    "motorbike",
                    "motorcycle",
                ),
                "dog": category_index("dog"),
            }
            self.semantic_names = LEGACY_SEMANTIC_NAMES

        if self.warmup_segmentation:
            warmup_shapes = [
                (720, 1280),
                (480, 640),
            ]
            with torch.inference_mode():
                for original_height, original_width in warmup_shapes:
                    warmup_height = self.segmentation_short_side
                    warmup_width = int(
                        round(
                            original_width
                            * warmup_height
                            / original_height
                        )
                    )
                    dummy = torch.zeros(
                        (2, 3, warmup_height, warmup_width),
                        device=self.device,
                        dtype=torch.float32,
                    ).contiguous(
                        memory_format=torch.channels_last
                    )
                    with torch.cuda.amp.autocast(
                        enabled=self.use_amp
                    ):
                        self.segmentation_model(dummy)
            torch.cuda.synchronize()

        rospy.loginfo(
            "Segmentation model loaded on %s (%s), "
            "short_side=%d, channels_last=True, warmup=%s, profile=%s",
            self.device,
            torchvision_api,
            self.segmentation_short_side,
            self.warmup_segmentation,
            self.semantic_profile,
        )

    def _load_detection_model(self) -> None:
        if not self.tools_dir.exists():
            raise FileNotFoundError(
                "OpenPCDet tools directory not found: {}".format(
                    self.tools_dir
                )
            )
        if not self.cfg_file.exists():
            raise FileNotFoundError(
                "OpenPCDet config not found: {}".format(self.cfg_file)
            )
        if not self.checkpoint.exists():
            raise FileNotFoundError(
                "Checkpoint not found: {}".format(self.checkpoint)
            )

        # OpenPCDet checkpoints contain optimizer/version metadata serialized
        # before PyTorch 2.6. They are trusted team artifacts, so preserve the
        # legacy loader behavior required by OpenPCDet's unpatched loader.
        os.environ.setdefault("TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD", "1")

        sys.path.insert(0, str(self.openpcdet_root))
        sys.path.insert(0, str(self.tools_dir))
        os.chdir(str(self.tools_dir))

        import yaml

        from pcdet.config import cfg, cfg_from_yaml_file
        from pcdet.datasets import DatasetTemplate
        from pcdet.models import build_network
        from pcdet.utils import common_utils

        class OnlineDataset(DatasetTemplate):
            def __init__(self, dataset_cfg, class_names, root_path, logger):
                super().__init__(
                    dataset_cfg=dataset_cfg,
                    class_names=class_names,
                    training=False,
                    root_path=root_path,
                    logger=logger,
                )

            def __len__(self):
                return 1

            def __getitem__(self, index):
                raise IndexError(index)

        # OpenPCDet의 _BASE_CONFIG_는 현재 작업 디렉터리를
        # 기준으로 해석된다. 저장소 config와 OpenPCDet config를
        # 모두 지원하도록 실제 base config가 존재하는 기준 폴더를 찾는다.
        config_load_dir = self.tools_dir

        with self.cfg_file.open("r", encoding="utf-8") as config_file:
            raw_config = yaml.safe_load(config_file) or {}

        base_config_value = (
            raw_config
            .get("DATA_CONFIG", {})
            .get("_BASE_CONFIG_")
        )

        if base_config_value:
            base_config_path = Path(
                str(base_config_value)
            ).expanduser()

            if base_config_path.is_absolute():
                if not base_config_path.exists():
                    raise FileNotFoundError(
                        "Base config not found: {}".format(
                            base_config_path
                        )
                    )
            else:
                candidate_directories = (
                    self.cfg_file.parent,
                    self.tools_dir,
                    self.openpcdet_root,
                )

                resolved_directory = next(
                    (
                        directory
                        for directory in candidate_directories
                        if (directory / base_config_path).exists()
                    ),
                    None,
                )

                if resolved_directory is None:
                    searched = [
                        str(directory / base_config_path)
                        for directory in candidate_directories
                    ]
                    raise FileNotFoundError(
                        "Base config not found. searched={}".format(
                            searched
                        )
                    )

                config_load_dir = resolved_directory

        rospy.loginfo(
            "OpenPCDet config load cwd: %s",
            config_load_dir,
        )

        try:
            os.chdir(str(config_load_dir))
            cfg_from_yaml_file(str(self.cfg_file), cfg)
        finally:
            # 이후 OpenPCDet 동작은 기존처럼 tools 기준으로 유지한다.
            os.chdir(str(self.tools_dir))

        self.pcdet_cfg = cfg
        self.pcdet_load_data_to_gpu = None

        logger = common_utils.create_logger()
        self.detection_dataset = OnlineDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            root_path=self.openpcdet_root,
            logger=logger,
        )
        self.class_names = list(cfg.CLASS_NAMES)
        configured_features = list(
            cfg.DATA_CONFIG.POINT_FEATURE_ENCODING.used_feature_list
        )
        single_sweep_features = [
            "x",
            "y",
            "z",
            "intensity",
        ] + list(self.semantic_names)
        if configured_features == single_sweep_features:
            inferred_temporal_sweeps = 1
        elif configured_features == single_sweep_features + ["time_lag_sec"]:
            inferred_temporal_sweeps = 3
        else:
            raise RuntimeError(
                "Point feature contract mismatch: config={} runtime={}".format(
                    configured_features,
                    single_sweep_features,
                )
            )
        if self.requested_temporal_num_sweeps:
            self.temporal_num_sweeps = self.requested_temporal_num_sweeps
        else:
            self.temporal_num_sweeps = inferred_temporal_sweeps
        if inferred_temporal_sweeps == 1 and self.temporal_num_sweeps != 1:
            raise RuntimeError(
                "temporal_num_sweeps={} requires time_lag_sec in config".format(
                    self.temporal_num_sweeps
                )
            )
        if inferred_temporal_sweeps > 1 and self.temporal_num_sweeps < 2:
            raise RuntimeError(
                "time_lag_sec config requires temporal_num_sweeps >= 2"
            )
        self.temporal_history = deque(
            maxlen=max(0, self.temporal_num_sweeps - 1)
        )
        self.point_cloud_range = np.asarray(
            cfg.DATA_CONFIG.POINT_CLOUD_RANGE,
            dtype=np.float32,
        )

        self.detection_model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(cfg.CLASS_NAMES),
            dataset=self.detection_dataset,
        )
        self.detection_model.load_params_from_file(
            filename=str(self.checkpoint),
            logger=logger,
            to_cpu=True,
        )
        self.detection_model.cuda().eval()

        from pcdet.models import load_data_to_gpu
        self.pcdet_load_data_to_gpu = load_data_to_gpu

        rospy.loginfo(
            "PointPillars loaded: %s (features=%s temporal_sweeps=%d)",
            self.checkpoint,
            configured_features,
            self.temporal_num_sweeps,
        )

    def _estimated_temporal_point_offsets(self, points: np.ndarray) -> np.ndarray:
        azimuth = np.mod(
            np.arctan2(points[:, 1], points[:, 0]), 2.0 * np.pi
        )
        clockwise_phase = np.mod(-azimuth, 2.0 * np.pi)
        return np.asarray(
            clockwise_phase
            / (2.0 * np.pi)
            * self.temporal_lidar_scan_period_sec,
            dtype=np.float32,
        )

    def _warmup_temporal_points(self, painted: np.ndarray) -> np.ndarray:
        if self.temporal_num_sweeps == 1:
            return painted
        sweeps = []
        for sweep_index in range(self.temporal_num_sweeps):
            lag = np.full(
                len(painted),
                -self.temporal_lidar_scan_period_sec * float(sweep_index),
                dtype=np.float32,
            )
            sweeps.append(np.concatenate((painted, lag[:, None]), axis=1))
        return np.ascontiguousarray(np.concatenate(sweeps), dtype=np.float32)

    def _temporal_points(
        self,
        painted: np.ndarray,
        timestamp: float,
    ) -> Optional[np.ndarray]:
        if self.temporal_num_sweeps == 1:
            return painted

        if self.temporal_history:
            newest_timestamp = float(self.temporal_history[-1][0])
            gap = float(timestamp) - newest_timestamp
            if gap <= 0.0 or gap > self.temporal_maximum_gap_sec:
                self.temporal_history.clear()

        if len(self.temporal_history) < self.temporal_num_sweeps - 1:
            self.temporal_history.append(
                (float(timestamp), np.array(painted, copy=True))
            )
            return None

        pose_samples = self.ego_pose_buffer.snapshot()
        current_lag = np.zeros(len(painted), dtype=np.float32)
        if self.temporal_time_lag_mode == "azimuth_approx":
            current_lag += self._estimated_temporal_point_offsets(painted)
        temporal_sweeps = [
            np.concatenate((painted, current_lag[:, None]), axis=1)
        ]
        history = list(self.temporal_history)[-(self.temporal_num_sweeps - 1):]
        for source_timestamp, source_points in reversed(history):
            transformed = transform_points_between_lidar_times(
                source_points,
                source_timestamp,
                timestamp,
                pose_samples,
                lidar_position=self.lidar_position,
                lidar_yaw=self.lidar_yaw_rad,
                maximum_extrapolation_sec=(
                    self.lidar_deskew_maximum_extrapolation_sec
                ),
            )
            if transformed is None:
                self.temporal_history.clear()
                self.temporal_history.append(
                    (float(timestamp), np.array(painted, copy=True))
                )
                return None
            lag = np.full(
                len(source_points),
                float(source_timestamp) - float(timestamp),
                dtype=np.float32,
            )
            if self.temporal_time_lag_mode == "azimuth_approx":
                lag += self._estimated_temporal_point_offsets(source_points)
            temporal_sweeps.append(
                np.concatenate((transformed, lag[:, None]), axis=1)
            )

        self.temporal_history.append(
            (float(timestamp), np.array(painted, copy=True))
        )
        return np.ascontiguousarray(
            np.concatenate(temporal_sweeps, axis=0), dtype=np.float32
        )

    def _warmup_full_pipeline(self) -> None:
        """Warm all expensive online stages before subscribing to sensors."""
        rospy.loginfo(
            "Full pipeline warm-up started: iterations=%d points=%d",
            self.warmup_iterations,
            self.warmup_point_count,
        )

        rng = np.random.default_rng(20260725)
        dummy_images = {}
        for camera_name in self.camera_names:
            config = self.camera_configs[camera_name]
            height = int(config["height"])
            width = int(config["width"])

            # A non-uniform image exercises resize, normalization, model forward,
            # softmax and GPU-to-CPU transfer instead of only allocating zeros.
            image = rng.integers(
                0,
                256,
                size=(height, width, 3),
                dtype=np.uint8,
            )
            dummy_images[camera_name] = image

        point_range = self.point_cloud_range.astype(np.float32)
        minimum = point_range[:3]
        maximum = point_range[3:6]

        xyz = rng.uniform(
            minimum,
            maximum,
            size=(self.warmup_point_count, 3),
        ).astype(np.float32)
        intensity = rng.random(
            (self.warmup_point_count, 1),
            dtype=np.float32,
        )
        dummy_lidar = np.concatenate(
            [xyz, intensity],
            axis=1,
        ).astype(np.float32, copy=False)

        if self.remove_ego_points:
            ego = self.ego_box
            outside_ego = ~(
                (dummy_lidar[:, 0] >= ego[0])
                & (dummy_lidar[:, 0] <= ego[1])
                & (dummy_lidar[:, 1] >= ego[2])
                & (dummy_lidar[:, 1] <= ego[3])
                & (dummy_lidar[:, 2] >= ego[4])
                & (dummy_lidar[:, 2] <= ego[5])
            )
            dummy_lidar = dummy_lidar[outside_ego]

        if len(dummy_lidar) < 1000:
            raise RuntimeError(
                "Too few dummy points after ego filtering: {}".format(
                    len(dummy_lidar)
                )
            )

        timings = []
        with torch.inference_mode():
            for iteration in range(self.warmup_iterations):
                torch.cuda.synchronize()
                warmup_start = time.perf_counter()

                score_maps = self._segment_4cam(dummy_images)
                painted = self._paint_lidar(
                    dummy_lidar,
                    score_maps,
                )
                self._detect(
                    self._warmup_temporal_points(painted),
                    "warmup_{}".format(iteration),
                )

                torch.cuda.synchronize()
                elapsed_ms = (
                    time.perf_counter() - warmup_start
                ) * 1000.0
                timings.append(elapsed_ms)
                rospy.loginfo(
                    "Full pipeline warm-up %d/%d: %.1f ms",
                    iteration + 1,
                    self.warmup_iterations,
                    elapsed_ms,
                )

        rospy.loginfo(
            "Full pipeline warm-up complete: last=%.1f ms",
            timings[-1],
        )

    def _log_online_ready(self) -> None:
        if self.online_ready_logged:
            return
        self.online_ready_logged = True
        rospy.loginfo("PointPainting online node ready")

    def _warmup_real_sensor_input(
        self,
        images_rgb: Dict[str, np.ndarray],
        lidar: np.ndarray,
    ) -> None:
        """Warm input-dependent CUDA paths without publishing test results."""
        rospy.loginfo(
            "Real-input warm-up started: iterations=%d lidar_points=%d",
            self.real_input_warmup_iterations,
            len(lidar),
        )

        with torch.inference_mode():
            for iteration in range(self.real_input_warmup_iterations):
                torch.cuda.synchronize()
                warmup_start = time.perf_counter()

                score_maps = self._segment_4cam(images_rgb)
                painted = self._paint_lidar(lidar, score_maps)
                self._detect(
                    self._warmup_temporal_points(painted),
                    "real_warmup_{}".format(iteration),
                )

                torch.cuda.synchronize()
                elapsed_ms = (
                    time.perf_counter() - warmup_start
                ) * 1000.0
                rospy.loginfo(
                    "Real-input warm-up %d/%d: %.1f ms",
                    iteration + 1,
                    self.real_input_warmup_iterations,
                    elapsed_ms,
                )

        # Sensor callbacks continue while warm-up runs. Discard the stale LiDAR
        # scan accumulated during initialization, then start runtime statistics
        # from the next scan so cold-start work is not reported as live drops.
        with self.lidar_lock:
            self.latest_lidar = None
            self.dropped_lidar_frames = 0
            self.lidar_input_frames = 0
            self.lidar_first_wall_time = None
            self.lidar_last_wall_time = None

        self.skipped_lidar_frames = 0
        self.processed_frames = 0
        self.real_input_warmup_frames += 1
        self.skip_reasons = {
            "camera_sync": 0,
            "empty_lidar": 0,
            "temporal_history": 0,
            "processing_error": 0,
        }
        self.publish_first_wall_time = None
        self.publish_last_wall_time = None
        self.benchmark_rows = []
        self.temporal_history.clear()
        with self.camera_lock:
            for camera_name in self.camera_names:
                self.camera_input_frames[camera_name] = 0
                self.camera_first_wall_time[camera_name] = None
                self.camera_last_wall_time[camera_name] = None
        self.real_input_warmup_complete = True

        rospy.loginfo(
            "Real-input warm-up complete; "
            "dropped/skipped/processed counters reset."
        )
        self._log_online_ready()

    def _camera_callback(self, msg: Image, camera_name: str) -> None:
        wall_time = time.perf_counter()
        timestamp = stamp_to_sec(msg.header.stamp)
        with self.camera_lock:
            self.camera_input_frames[camera_name] += 1
            if self.camera_first_wall_time[camera_name] is None:
                self.camera_first_wall_time[camera_name] = wall_time
            self.camera_last_wall_time[camera_name] = wall_time
            self.camera_buffers[camera_name].append(
                (timestamp, msg)
            )

    def _ego_callback(self, msg: EgoVehicleStatus) -> None:
        self.ego_pose_buffer.add(
            stamp_to_sec(msg.header.stamp),
            msg.position.x,
            msg.position.y,
            msg.position.z,
            math.radians(msg.heading),
        )

    def _lidar_callback(self, msg: PointCloud2) -> None:
        enqueue_time = time.perf_counter()
        with self.lidar_lock:
            self.lidar_input_frames += 1
            if self.lidar_first_wall_time is None:
                self.lidar_first_wall_time = enqueue_time
            self.lidar_last_wall_time = enqueue_time
            if self.latest_lidar is not None:
                self.dropped_lidar_frames += 1
            self.latest_lidar = (msg, enqueue_time)
        self.worker_event.set()

    def _worker_loop(self) -> None:
        while not rospy.is_shutdown():
            self.worker_event.wait(timeout=0.1)
            self.worker_event.clear()

            with self.lidar_lock:
                item = self.latest_lidar
                self.latest_lidar = None

            if item is None:
                continue

            msg, enqueue_time = item
            try:
                self._process_lidar(msg, enqueue_time)
            except Exception as error:
                self.skipped_lidar_frames += 1
                self.skip_reasons["processing_error"] += 1
                rospy.logerr_throttle(
                    1.0,
                    "PointPainting frame failed: {}".format(error),
                )

            with self.lidar_lock:
                if self.latest_lidar is not None:
                    self.worker_event.set()

    def _select_camera_messages(
        self,
        lidar_timestamp: float,
    ) -> Tuple[Optional[Dict[str, object]], List[float]]:
        selected = {}
        selected_timestamps = []

        with self.camera_lock:
            for camera_name in self.camera_names:
                candidates = [
                    item
                    for item in self.camera_buffers[camera_name]
                    if item[0] <= lidar_timestamp + 1e-6
                ]
                if not candidates:
                    return None, []

                timestamp, msg = candidates[-1]
                age_ms = (lidar_timestamp - timestamp) * 1000.0
                if age_ms < -1e-3 or age_ms > self.max_camera_age_ms:
                    return None, []

                selected[camera_name] = msg
                selected_timestamps.append(timestamp)

        span_ms = (
            max(selected_timestamps) - min(selected_timestamps)
        ) * 1000.0
        if span_ms > self.max_camera_span_ms:
            return None, []

        ages_ms = [
            (lidar_timestamp - timestamp) * 1000.0
            for timestamp in selected_timestamps
        ]
        return selected, ages_ms

    def _decode_images(
        self,
        messages: Dict[str, object],
    ) -> Dict[str, np.ndarray]:
        def decode(camera_name: str) -> Tuple[str, np.ndarray]:
            message = messages[camera_name]

            if isinstance(message, CompressedImage):
                image_bgr = cv2.imdecode(
                    np.frombuffer(message.data, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
            elif isinstance(message, Image):
                image_bgr = self.bridge.imgmsg_to_cv2(
                    message,
                    desired_encoding="bgr8",
                )
            else:
                raise TypeError(
                    "Unsupported camera message type for {}: {}".format(
                        camera_name,
                        type(message).__name__,
                    )
                )

            if image_bgr is None or image_bgr.size == 0:
                raise RuntimeError(
                    "Decoded empty camera image: {}".format(camera_name)
                )

            image_rgb = image_bgr[:, :, ::-1].copy()
            return camera_name, image_rgb

        if self.camera_executor is None:
            decoded = [decode(name) for name in self.camera_names]
        else:
            decoded = list(
                self.camera_executor.map(decode, self.camera_names)
            )

        return dict(decoded)

    def _selected_score_maps(
        self,
        names: Sequence[str],
        images_rgb: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
        preprocess_start = time.perf_counter()

        def preprocess(name: str) -> torch.Tensor:
            if self.segmentation_preprocess_backend == "opencv":
                return self._opencv_segmentation_preprocess(
                    images_rgb[name]
                )
            return self.segmentation_preprocess(
                PILImage.fromarray(images_rgb[name])
            )

        if self.camera_executor is None:
            tensors = [preprocess(name) for name in names]
        else:
            tensors = list(
                self.camera_executor.map(preprocess, names)
            )
        self._segmentation_stage_ms["camera_preprocess_ms"] += (
            time.perf_counter() - preprocess_start
        ) * 1000.0

        h2d_start = time.perf_counter()
        batch = torch.stack(tensors, dim=0).to(
            self.device,
            non_blocking=True,
        ).contiguous(
            memory_format=torch.channels_last
        )
        if self.detailed_timing:
            torch.cuda.synchronize()
        self._segmentation_stage_ms["camera_h2d_ms"] += (
            time.perf_counter() - h2d_start
        ) * 1000.0

        group_name = "+".join(names)

        # Log each real runtime batch shape once after warm-up completes.
        if (
            self.real_input_warmup_complete
            and group_name not in self.logged_segmentation_shapes
        ):
            rospy.loginfo(
                "Segmentation input %-24s shape=%s",
                group_name,
                tuple(batch.shape),
            )
            self.logged_segmentation_shapes.add(group_name)

        inference_start = time.perf_counter()
        with torch.inference_mode():
            with torch.cuda.amp.autocast(enabled=self.use_amp):
                logits = self.segmentation_model(batch)["out"]

        if self.detailed_timing:
            torch.cuda.synchronize()
        self._segmentation_stage_ms["segmentation_inference_ms"] += (
            time.perf_counter() - inference_start
        ) * 1000.0

        postprocess_start = time.perf_counter()
        with torch.inference_mode():
            # Keep the semantic score map at the resized inference resolution.
            # _project_to_camera() derives fx, fy, cx and cy from the score-map
            # dimensions, so projecting directly into this map is geometrically
            # equivalent to first upsampling it to the original camera image.
            # Avoiding the extra full-resolution interpolation and softmax greatly
            # reduces GPU memory traffic in the online pipeline.
            probabilities = torch.softmax(
                logits.float(),
                dim=1,
            )

        if self.semantic_profile == "morai_4class":
            scores = probabilities.permute(0, 2, 3, 1).cpu().numpy()
            result = {
                name: scores[array_index].astype(np.float32, copy=False)
                for array_index, name in enumerate(names)
            }
            self._segmentation_stage_ms["segmentation_postprocess_ms"] += (
                time.perf_counter() - postprocess_start
            ) * 1000.0
            return result

        index = self.segmentation_indices
        person = probabilities[:, index["person"]]
        vehicle = (
            probabilities[:, index["car"]]
            + probabilities[:, index["bus"]]
        )
        two_wheeler = (
            probabilities[:, index["bicycle"]]
            + probabilities[:, index["motorbike"]]
        )
        dog = probabilities[:, index["dog"]]
        background = torch.clamp(
            1.0 - person - vehicle - two_wheeler - dog,
            min=0.0,
            max=1.0,
        )

        scores = torch.stack(
            [
                background,
                person,
                vehicle,
                two_wheeler,
                dog,
            ],
            dim=-1,
        ).cpu().numpy().astype(np.float32)

        result = {
            name: scores[array_index]
            for array_index, name in enumerate(names)
        }
        self._segmentation_stage_ms["segmentation_postprocess_ms"] += (
            time.perf_counter() - postprocess_start
        ) * 1000.0
        return result

    def _opencv_segmentation_preprocess(
        self,
        image_rgb: np.ndarray,
    ) -> torch.Tensor:
        """Approximate torchvision Resize/ToTensor/Normalize with OpenCV.

        The output geometry and RGB/ImageNet normalization are identical to
        the training path. Pixel values can still differ slightly because PIL
        antialiased bilinear resize and OpenCV INTER_LINEAR use different
        resampling kernels; this backend therefore remains opt-in.
        """
        height, width = image_rgb.shape[:2]
        if height <= width:
            output_height = self.segmentation_short_side
            output_width = int(
                self.segmentation_short_side * width / height
            )
        else:
            output_width = self.segmentation_short_side
            output_height = int(
                self.segmentation_short_side * height / width
            )

        resized = cv2.resize(
            image_rgb,
            (output_width, output_height),
            interpolation=cv2.INTER_LINEAR,
        )
        tensor = (
            torch.from_numpy(resized)
            .permute(2, 0, 1)
            .to(dtype=torch.float32)
            .div_(255.0)
        )
        return tensor.sub_(self.segmentation_mean_cpu).div_(
            self.segmentation_std_cpu
        )

    def _segment_4cam(
        self,
        images_rgb: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
        self._segmentation_stage_ms = {
            "camera_preprocess_ms": 0.0,
            "camera_h2d_ms": 0.0,
            "segmentation_inference_ms": 0.0,
            "segmentation_postprocess_ms": 0.0,
        }
        score_maps = {}
        score_maps.update(
            self._selected_score_maps(
                ["front", "back"],
                images_rgb,
            )
        )
        score_maps.update(
            self._selected_score_maps(
                ["front_left", "front_right"],
                images_rgb,
            )
        )
        return score_maps

    def _project_to_camera(
        self,
        lidar_xyz: np.ndarray,
        camera_name: str,
        image_shape: Sequence[int],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        config = self.camera_configs[camera_name]
        camera_position = np.asarray(
            config["position"],
            dtype=np.float64,
        )
        r_vehicle_camera = rotation_matrix(
            *config["rotation"]
        )

        points_vehicle = (
            self.r_vehicle_lidar @ lidar_xyz.T
        ).T + self.lidar_position

        points_camera_body = (
            r_vehicle_camera.T
            @ (points_vehicle - camera_position).T
        ).T

        points_optical = (
            R_OPTICAL_FROM_BODY @ points_camera_body.T
        ).T

        x_opt = points_optical[:, 0]
        y_opt = points_optical[:, 1]
        z_opt = points_optical[:, 2]

        height = int(image_shape[0])
        width = int(image_shape[1])
        fov = float(config["fov"])

        focal = width / (
            2.0 * math.tan(math.radians(fov / 2.0))
        )
        fx = focal
        fy = focal
        cx = width / 2.0
        cy = height / 2.0

        valid_depth = (z_opt > 0.5) & (z_opt < 100.0)
        depth_indices = np.flatnonzero(valid_depth)

        u_depth = (
            fx * x_opt[valid_depth] / z_opt[valid_depth] + cx
        )
        v_depth = (
            fy * y_opt[valid_depth] / z_opt[valid_depth] + cy
        )

        valid_image = (
            (u_depth >= 0.0)
            & (u_depth < width)
            & (v_depth >= 0.0)
            & (v_depth < height)
        )

        visible_indices = depth_indices[valid_image]
        u = u_depth[valid_image].astype(np.int32)
        v = v_depth[valid_image].astype(np.int32)
        return visible_indices, u, v

    def _paint_lidar(
        self,
        lidar: np.ndarray,
        score_maps: Dict[str, np.ndarray],
    ) -> np.ndarray:
        lidar_xyz = lidar[:, :3].astype(
            np.float64,
            copy=False,
        )

        known_semantic_count = len(self.semantic_names) - 1
        semantic_sum = np.zeros(
            (len(lidar), known_semantic_count),
            dtype=np.float32,
        )
        camera_count = np.zeros(
            len(lidar),
            dtype=np.int32,
        )

        for camera_name in self.camera_names:
            visible_indices, u, v = self._project_to_camera(
                lidar_xyz,
                camera_name,
                score_maps[camera_name].shape,
            )
            camera_scores = score_maps[camera_name][v, u]
            semantic_sum[visible_indices] += camera_scores
            camera_count[visible_indices] += 1

        seen = camera_count > 0
        final_semantic = np.zeros(
            (len(lidar), len(self.semantic_names)),
            dtype=np.float32,
        )
        final_semantic[seen, :known_semantic_count] = (
            semantic_sum[seen]
            / camera_count[seen, None]
        )
        final_semantic[~seen, -1] = 1.0

        return np.concatenate(
            [lidar[:, :4], final_semantic],
            axis=1,
        ).astype(np.float32, copy=False)

    def _preprocess_lidar(self, msg: PointCloud2):
        points, point_time_offsets = cloud_to_xyzi(msg, include_time=True)
        deskew = {
            "applied": False,
            "reference_time_offset_sec": 0.0,
            "point_displacement_mean_m": 0.0,
            "point_displacement_p95_m": 0.0,
            "point_displacement_max_m": 0.0,
        }
        if len(points) == 0:
            return points, point_time_offsets, deskew

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        point_time_offsets = point_time_offsets[finite]
        if len(points) == 0:
            return points, point_time_offsets, deskew

        if self.lidar_deskew_mode == "ego_pose_2d":
            finite_time = point_time_offsets[np.isfinite(point_time_offsets)]
            reference_offset = 0.0
            if self.lidar_deskew_reference == "point_median" and finite_time.size:
                reference_offset = float(np.median(finite_time))
            corrected = deskew_points_to_reference_pose(
                points,
                point_time_offsets,
                stamp_to_sec(msg.header.stamp),
                reference_offset,
                self.ego_pose_buffer.snapshot(),
                lidar_position=self.lidar_position,
                lidar_yaw=self.lidar_yaw_rad,
                maximum_extrapolation_sec=(
                    self.lidar_deskew_maximum_extrapolation_sec
                ),
            )
            if corrected is not None:
                points, displacement, _ = corrected
                deskew = {
                    "applied": True,
                    "reference_time_offset_sec": reference_offset,
                    "point_displacement_mean_m": float(np.mean(displacement)),
                    "point_displacement_p95_m": float(
                        np.percentile(displacement, 95)
                    ),
                    "point_displacement_max_m": float(np.max(displacement)),
                }
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "LiDAR ego deskew skipped: insufficient timed ego poses.",
                )

        if self.normalize_intensity:
            intensity = points[:, 3]
            max_value = float(np.max(intensity)) if len(intensity) else 0.0
            if max_value > 1.5:
                divisor = 255.0 if max_value <= 255.0 else max_value
                points[:, 3] = np.clip(
                    intensity / divisor,
                    0.0,
                    1.0,
                )

        r = self.point_cloud_range
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        inside = (
            (x >= r[0])
            & (y >= r[1])
            & (z >= r[2])
            & (x <= r[3])
            & (y <= r[4])
            & (z <= r[5])
        )

        if self.remove_ego_points:
            ego = self.ego_box
            in_ego = (
                (x >= ego[0])
                & (x <= ego[1])
                & (y >= ego[2])
                & (y <= ego[3])
                & (z >= ego[4])
                & (z <= ego[5])
            )
            inside &= ~in_ego

        return (
            np.ascontiguousarray(points[inside], dtype=np.float32),
            np.ascontiguousarray(point_time_offsets[inside], dtype=np.float32),
            deskew,
        )

    def _detect(
        self,
        painted: np.ndarray,
        frame_id: str,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, float]]:
        stage_ms = {
            "pointpillars_preprocess_ms": 0.0,
            "pointpillars_h2d_ms": 0.0,
            "pointpillars_inference_ms": 0.0,
            "pointpillars_postprocess_ms": 0.0,
        }

        preprocess_start = time.perf_counter()
        sample = self.detection_dataset.prepare_data(
            data_dict={
                "points": painted,
                "frame_id": frame_id,
            }
        )
        batch = self.detection_dataset.collate_batch([sample])
        stage_ms["pointpillars_preprocess_ms"] = (
            time.perf_counter() - preprocess_start
        ) * 1000.0

        h2d_start = time.perf_counter()
        self.pcdet_load_data_to_gpu(batch)
        if self.detailed_timing:
            torch.cuda.synchronize()
        stage_ms["pointpillars_h2d_ms"] = (
            time.perf_counter() - h2d_start
        ) * 1000.0

        with torch.no_grad():
            inference_start = time.perf_counter()
            if self.detailed_timing:
                for module in self.detection_model.module_list:
                    batch = module(batch)
                torch.cuda.synchronize()
                stage_ms["pointpillars_inference_ms"] = (
                    time.perf_counter() - inference_start
                ) * 1000.0

                postprocess_start = time.perf_counter()
                predictions, _ = self.detection_model.post_processing(batch)
                torch.cuda.synchronize()
            else:
                predictions, _ = self.detection_model(batch)
                stage_ms["pointpillars_inference_ms"] = (
                    time.perf_counter() - inference_start
                ) * 1000.0
                postprocess_start = time.perf_counter()

        prediction = predictions[0]
        boxes = prediction["pred_boxes"].detach().cpu().numpy()
        scores = prediction["pred_scores"].detach().cpu().numpy()
        labels = prediction["pred_labels"].detach().cpu().numpy()

        distance = np.linalg.norm(boxes[:, :2], axis=1)
        valid = (
            (scores >= self.score_threshold)
            & (distance <= self.max_detection_range)
        )

        boxes = np.ascontiguousarray(boxes[valid], dtype=np.float32)
        scores = np.asarray(scores[valid], dtype=np.float32)
        labels = np.asarray(labels[valid], dtype=np.int32)

        if len(scores):
            order = np.argsort(-scores)
            boxes = boxes[order]
            scores = scores[order]
            labels = labels[order]

        stage_ms["pointpillars_postprocess_ms"] = (
            time.perf_counter() - postprocess_start
        ) * 1000.0
        return boxes, scores, labels, stage_ms

    @staticmethod
    def _wall_rate(count: int, first_time, last_time):
        if count < 2 or first_time is None or last_time is None:
            return None
        duration = float(last_time) - float(first_time)
        if duration <= 0.0:
            return None
        return float(count - 1) / duration

    @staticmethod
    def _distribution(rows, key):
        values = np.asarray(
            [float(row[key]) for row in rows if key in row],
            dtype=np.float64,
        )
        values = values[np.isfinite(values)]
        if not len(values):
            return None
        return {
            "count": int(len(values)),
            "mean": float(np.mean(values)),
            "p50": float(np.percentile(values, 50)),
            "p90": float(np.percentile(values, 90)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)),
            "max": float(np.max(values)),
        }

    def _shutdown_camera_executor(self) -> None:
        if self.camera_executor is not None:
            self.camera_executor.shutdown(wait=True)
            self.camera_executor = None

    def _write_benchmark_report(self) -> None:
        if (
            self.benchmark_output_dir is None
            or self.benchmark_report_written
        ):
            return
        self.benchmark_report_written = True

        try:
            self.benchmark_output_dir.mkdir(parents=True, exist_ok=True)
            rows = list(self.benchmark_rows)
            excluded = min(self.benchmark_exclude_frames, len(rows))
            analyzed_rows = rows[excluded:]

            csv_path = self.benchmark_output_dir / "frame_metrics.csv"
            if rows:
                fieldnames = list(rows[0].keys())
                with csv_path.open("w", encoding="utf-8", newline="") as file:
                    writer = csv.DictWriter(file, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(rows)

            timing_keys = (
                "camera_match_ms",
                "lidar_preprocess_ms",
                "camera_decode_ms",
                "camera_preprocess_ms",
                "camera_h2d_ms",
                "segmentation_inference_ms",
                "segmentation_postprocess_ms",
                "segmentation_total_ms",
                "painting_ms",
                "pointpillars_preprocess_ms",
                "pointpillars_h2d_ms",
                "pointpillars_inference_ms",
                "pointpillars_postprocess_ms",
                "pointpillars_total_ms",
                "message_publish_ms",
                "total_compute_ms",
                "queue_delay_ms",
                "arrival_to_publish_ms",
                "sim_sensor_age_at_publish_ms",
                "camera_age_max_ms",
                "camera_span_ms",
                "measurement_time_offset_ms",
            )
            timing_summary = {
                key: self._distribution(analyzed_rows, key)
                for key in timing_keys
            }

            queue_growth_ms = None
            if len(analyzed_rows) >= 8:
                quarter = max(1, len(analyzed_rows) // 4)
                first_queue = [
                    row["queue_delay_ms"]
                    for row in analyzed_rows[:quarter]
                ]
                last_queue = [
                    row["queue_delay_ms"]
                    for row in analyzed_rows[-quarter:]
                ]
                queue_growth_ms = float(
                    np.median(last_queue) - np.median(first_queue)
                )

            camera_rates = {
                name: self._wall_rate(
                    self.camera_input_frames[name],
                    self.camera_first_wall_time[name],
                    self.camera_last_wall_time[name],
                )
                for name in self.camera_names
            }
            lidar_rate = self._wall_rate(
                self.lidar_input_frames,
                self.lidar_first_wall_time,
                self.lidar_last_wall_time,
            )
            publish_rate = self._wall_rate(
                self.processed_frames,
                self.publish_first_wall_time,
                self.publish_last_wall_time,
            )
            input_count = int(self.lidar_input_frames)
            publish_count = int(self.processed_frames)

            summary = {
                "schema_version": 2,
                "label": self.benchmark_label,
                "generated_at": datetime.now().astimezone().isoformat(),
                "artifacts": {
                    "pointpillars_config": file_identity(self.cfg_file),
                    "pointpillars_checkpoint": file_identity(
                        self.checkpoint
                    ),
                    "segmentation_checkpoint": file_identity(
                        self.segmentation_checkpoint
                    ),
                },
                "runtime": {
                    "semantic_profile": self.semantic_profile,
                    "segmentation_short_side": self.segmentation_short_side,
                    "segmentation_preprocess_backend": (
                        self.segmentation_preprocess_backend
                    ),
                    "camera_cpu_workers": self.camera_cpu_workers,
                    "use_amp": self.use_amp,
                    "detailed_timing": self.detailed_timing,
                    "score_threshold": self.score_threshold,
                    "maximum_detection_range_m": self.max_detection_range,
                    "lidar_measurement_stamp_mode": (
                        self.lidar_measurement_stamp_mode
                    ),
                    "lidar_deskew_mode": self.lidar_deskew_mode,
                    "lidar_deskew_reference": self.lidar_deskew_reference,
                    "lidar_deskew_camera_reference": (
                        self.lidar_deskew_camera_reference
                    ),
                },
                "lidar_deskew": {
                    "applied_frames": int(
                        sum(
                            int(row.get("lidar_deskew_applied", 0))
                            for row in analyzed_rows
                        )
                    ),
                    "reference_offset_ms": self._distribution(
                        analyzed_rows,
                        "lidar_deskew_reference_offset_ms",
                    ),
                    "point_displacement_mean_m": self._distribution(
                        analyzed_rows,
                        "lidar_deskew_point_displacement_mean_m",
                    ),
                    "point_displacement_p95_m": self._distribution(
                        analyzed_rows,
                        "lidar_deskew_point_displacement_p95_m",
                    ),
                    "point_displacement_max_m": self._distribution(
                        analyzed_rows,
                        "lidar_deskew_point_displacement_max_m",
                    ),
                },
                "counts": {
                    "camera_input_frames": dict(self.camera_input_frames),
                    "lidar_input_frames": input_count,
                    "published_frames": publish_count,
                    "dropped_lidar_frames": int(self.dropped_lidar_frames),
                    "skipped_lidar_frames": int(self.skipped_lidar_frames),
                    "skip_reasons": dict(self.skip_reasons),
                    "real_input_warmup_frames": int(
                        self.real_input_warmup_frames
                    ),
                    "recorded_rows": len(rows),
                    "excluded_published_frames": excluded,
                    "analyzed_published_frames": len(analyzed_rows),
                },
                "rates_hz": {
                    "camera_input": camera_rates,
                    "lidar_input": lidar_rate,
                    "detection_publish": publish_rate,
                },
                "ratios": {
                    "publish_per_lidar_input": (
                        float(publish_count) / input_count
                        if input_count else None
                    ),
                    "drop_per_lidar_input": (
                        float(self.dropped_lidar_frames) / input_count
                        if input_count else None
                    ),
                    "skip_per_lidar_input": (
                        float(self.skipped_lidar_frames) / input_count
                        if input_count else None
                    ),
                },
                "queue_delay_growth_ms": queue_growth_ms,
                "timing_ms": timing_summary,
            }
            summary_path = self.benchmark_output_dir / "summary.json"
            with summary_path.open("w", encoding="utf-8") as file:
                json.dump(summary, file, indent=2, sort_keys=True)
                file.write("\n")
            rospy.loginfo(
                "PointPainting benchmark report: %s (%d frames)",
                summary_path,
                len(analyzed_rows),
            )
        except Exception as error:
            rospy.logerr("Failed to write benchmark report: %s", error)

    def _make_detection_array(
        self,
        header: Header,
        boxes: np.ndarray,
        scores: np.ndarray,
        labels: np.ndarray,
        camera_age_max_ms: float,
        camera_span_ms: float,
        queue_delay_ms: float,
        segmentation_ms: float,
        painting_ms: float,
        detection_ms: float,
        total_ms: float,
    ) -> PointPaintingDetectionArray:
        output = PointPaintingDetectionArray()
        output.header = copy.copy(header)
        output.camera_age_max_ms = float(camera_age_max_ms)
        output.camera_span_ms = float(camera_span_ms)
        output.queue_delay_ms = float(queue_delay_ms)
        output.segmentation_ms = float(segmentation_ms)
        output.painting_ms = float(painting_ms)
        output.detection_ms = float(detection_ms)
        output.total_ms = float(total_ms)
        output.dropped_lidar_frames = int(
            self.dropped_lidar_frames
        )
        output.skipped_lidar_frames = int(
            self.skipped_lidar_frames
        )

        for index, (box, score, label) in enumerate(
            zip(boxes, scores, labels)
        ):
            class_index = int(label) - 1
            class_name = (
                self.class_names[class_index]
                if 0 <= class_index < len(self.class_names)
                else "unknown"
            )

            detection = PointPaintingDetection()
            detection.detection_id = int(index)
            detection.class_id = int(label)
            detection.class_name = str(class_name)
            detection.score = float(score)
            detection.pose = Pose()
            detection.pose.position.x = float(box[0])
            detection.pose.position.y = float(box[1])
            detection.pose.position.z = float(box[2])
            detection.pose.orientation = quaternion_from_yaw(
                float(box[6])
            )
            detection.size = Vector3(
                x=float(box[3]),
                y=float(box[4]),
                z=float(box[5]),
            )
            detection.distance = float(
                np.linalg.norm(box[:2])
            )
            output.detections.append(detection)

        return output

    def _process_lidar(
        self,
        msg: PointCloud2,
        enqueue_time: float,
    ) -> None:
        total_start = time.perf_counter()
        queue_delay_ms = (
            total_start - enqueue_time
        ) * 1000.0
        lidar_timestamp = stamp_to_sec(msg.header.stamp)

        lidar_preprocess_start = time.perf_counter()
        lidar, point_time_offsets, deskew = self._preprocess_lidar(msg)
        lidar_preprocess_ms = (
            time.perf_counter() - lidar_preprocess_start
        ) * 1000.0
        if len(lidar) == 0:
            self.skipped_lidar_frames += 1
            self.skip_reasons["empty_lidar"] += 1
            rospy.logwarn_throttle(
                1.0,
                "Skipping empty LiDAR scan.",
            )
            return

        camera_reference_timestamp = lidar_timestamp
        if (
            deskew["applied"]
            and self.lidar_deskew_camera_reference == "reference"
        ):
            camera_reference_timestamp += deskew["reference_time_offset_sec"]
        camera_match_start = time.perf_counter()
        camera_messages, camera_ages_ms = self._select_camera_messages(
            camera_reference_timestamp
        )
        camera_match_ms = (
            time.perf_counter() - camera_match_start
        ) * 1000.0
        if camera_messages is None:
            self.skipped_lidar_frames += 1
            self.skip_reasons["camera_sync"] += 1
            rospy.logwarn_throttle(
                1.0,
                "Skipping LiDAR: no causal 4-camera set "
                "within age/span limits.",
            )
            return

        camera_timestamps = [
            stamp_to_sec(camera_messages[name].header.stamp)
            for name in self.camera_names
        ]
        camera_age_max_ms = max(camera_ages_ms)
        camera_span_ms = (
            max(camera_timestamps) - min(camera_timestamps)
        ) * 1000.0

        camera_decode_start = time.perf_counter()
        images_rgb = self._decode_images(camera_messages)
        camera_decode_ms = (
            time.perf_counter() - camera_decode_start
        ) * 1000.0

        if not self.real_input_warmup_complete:
            self._warmup_real_sensor_input(images_rgb, lidar)
            return

        score_maps = self._segment_4cam(images_rgb)
        segmentation_stage_ms = dict(self._segmentation_stage_ms)
        segmentation_ms = float(sum(segmentation_stage_ms.values()))

        painting_start = time.perf_counter()
        painted = self._paint_lidar(lidar, score_maps)
        painting_ms = (
            time.perf_counter() - painting_start
        ) * 1000.0

        temporal_points = self._temporal_points(painted, lidar_timestamp)
        if temporal_points is None:
            self.skipped_lidar_frames += 1
            self.skip_reasons["temporal_history"] += 1
            rospy.loginfo_throttle(
                1.0,
                "Waiting for %d-sweep temporal history (%d/%d past sweeps).",
                self.temporal_num_sweeps,
                len(self.temporal_history),
                self.temporal_num_sweeps - 1,
            )
            return

        boxes, scores, labels, detection_stage_ms = self._detect(
            temporal_points,
            str(msg.header.seq),
        )
        detection_ms = float(sum(detection_stage_ms.values()))

        measurement_time_offset_sec = 0.0
        if self.lidar_measurement_stamp_mode == "point_median":
            finite_time = point_time_offsets[np.isfinite(point_time_offsets)]
            if finite_time.size:
                measurement_time_offset_sec = float(np.median(finite_time))
        elif self.lidar_measurement_stamp_mode == "object_points_median":
            measurement_time_offset_sec = representative_detection_time_offset(
                lidar,
                point_time_offsets,
                boxes,
                extent_scale=self.lidar_timing_box_extent_scale,
                minimum_points_per_box=(
                    self.lidar_timing_minimum_points_per_box
                ),
            )
        if deskew["applied"]:
            source_time = (
                lidar_timestamp + deskew["reference_time_offset_sec"]
            )
            target_time = lidar_timestamp + measurement_time_offset_sec
            transformed_boxes = transform_boxes_between_lidar_times(
                boxes,
                source_time,
                target_time,
                self.ego_pose_buffer.snapshot(),
                lidar_position=self.lidar_position,
                lidar_yaw=self.lidar_yaw_rad,
                maximum_extrapolation_sec=(
                    self.lidar_deskew_maximum_extrapolation_sec
                ),
            )
            if transformed_boxes is None:
                # Keep the coordinate frame and timestamp coherent when a
                # late pose sample is unavailable.
                measurement_time_offset_sec = deskew[
                    "reference_time_offset_sec"
                ]
            else:
                boxes = np.ascontiguousarray(
                    transformed_boxes, dtype=np.float32
                )
        measurement_header = copy.deepcopy(msg.header)
        if self.lidar_measurement_stamp_mode != "header":
            measurement_header.stamp = rospy.Time.from_sec(
                max(0.0, lidar_timestamp + measurement_time_offset_sec)
            )

        prepublish_total_ms = (
            time.perf_counter() - total_start
        ) * 1000.0

        message_publish_start = time.perf_counter()
        output = self._make_detection_array(
            header=measurement_header,
            boxes=boxes,
            scores=scores,
            labels=labels,
            camera_age_max_ms=camera_age_max_ms,
            camera_span_ms=camera_span_ms,
            queue_delay_ms=queue_delay_ms,
            segmentation_ms=segmentation_ms,
            painting_ms=painting_ms,
            detection_ms=detection_ms,
            total_ms=prepublish_total_ms,
        )
        names = [
            self.class_names[int(label) - 1].lower()
            if 0 < int(label) <= len(self.class_names)
            else "unknown"
            for label in labels
        ]
        self.objects_pub.publish(
            make_detected_objects(
                measurement_header,
                boxes,
                scores,
                names,
                include_velocity=self.publish_detector_velocity,
                velocity_classes=self.detector_velocity_classes,
                velocity_noise_std_mps=self.detector_velocity_noise_std_mps,
                maximum_velocity_mps=self.detector_maximum_velocity_mps,
            )
        )
        self.detections_pub.publish(output)

        self.markers_pub.publish(
            make_markers(
                boxes,
                scores,
                names,
                measurement_header,
            )
        )

        if self.publish_painted_points:
            painted_header = copy.deepcopy(msg.header)
            if deskew["applied"]:
                painted_header.stamp = rospy.Time.from_sec(
                    lidar_timestamp + deskew["reference_time_offset_sec"]
                )
            self.painted_pub.publish(
                make_painted_cloud(
                    painted,
                    painted_header,
                    self.semantic_names,
                )
            )

        publish_wall_time = time.perf_counter()
        message_publish_ms = (
            publish_wall_time - message_publish_start
        ) * 1000.0
        total_compute_ms = (
            publish_wall_time - total_start
        ) * 1000.0
        arrival_to_publish_ms = queue_delay_ms + total_compute_ms
        ros_now = rospy.Time.now().to_sec()
        sim_sensor_age_at_publish_ms = (
            (ros_now - lidar_timestamp) * 1000.0
            if ros_now > 0.0 and lidar_timestamp > 0.0
            else float("nan")
        )

        self.processed_frames += 1
        if self.publish_first_wall_time is None:
            self.publish_first_wall_time = publish_wall_time
        self.publish_last_wall_time = publish_wall_time

        pointpillars_total_ms = float(sum(detection_stage_ms.values()))
        row = {
            "frame_index": int(self.processed_frames),
            "source_seq": int(msg.header.seq),
            "source_stamp_sec": float(lidar_timestamp),
            "measurement_stamp_sec": float(
                measurement_header.stamp.to_sec()
            ),
            "measurement_time_offset_ms": float(
                measurement_time_offset_sec * 1000.0
            ),
            "lidar_points": int(len(lidar)),
            "detection_count": int(len(boxes)),
            "camera_match_ms": float(camera_match_ms),
            "lidar_preprocess_ms": float(lidar_preprocess_ms),
            "lidar_deskew_applied": int(deskew["applied"]),
            "lidar_deskew_reference_offset_ms": float(
                deskew["reference_time_offset_sec"] * 1000.0
            ),
            "lidar_deskew_point_displacement_mean_m": float(
                deskew["point_displacement_mean_m"]
            ),
            "lidar_deskew_point_displacement_p95_m": float(
                deskew["point_displacement_p95_m"]
            ),
            "lidar_deskew_point_displacement_max_m": float(
                deskew["point_displacement_max_m"]
            ),
            "camera_decode_ms": float(camera_decode_ms),
            "camera_preprocess_ms": float(
                segmentation_stage_ms["camera_preprocess_ms"]
            ),
            "camera_h2d_ms": float(
                segmentation_stage_ms["camera_h2d_ms"]
            ),
            "segmentation_inference_ms": float(
                segmentation_stage_ms["segmentation_inference_ms"]
            ),
            "segmentation_postprocess_ms": float(
                segmentation_stage_ms["segmentation_postprocess_ms"]
            ),
            "segmentation_total_ms": float(segmentation_ms),
            "painting_ms": float(painting_ms),
            "pointpillars_preprocess_ms": float(
                detection_stage_ms["pointpillars_preprocess_ms"]
            ),
            "pointpillars_h2d_ms": float(
                detection_stage_ms["pointpillars_h2d_ms"]
            ),
            "pointpillars_inference_ms": float(
                detection_stage_ms["pointpillars_inference_ms"]
            ),
            "pointpillars_postprocess_ms": float(
                detection_stage_ms["pointpillars_postprocess_ms"]
            ),
            "pointpillars_total_ms": pointpillars_total_ms,
            "message_publish_ms": float(message_publish_ms),
            "total_compute_ms": float(total_compute_ms),
            "queue_delay_ms": float(queue_delay_ms),
            "arrival_to_publish_ms": float(arrival_to_publish_ms),
            "sim_sensor_age_at_publish_ms": float(
                sim_sensor_age_at_publish_ms
            ),
            "camera_age_max_ms": float(camera_age_max_ms),
            "camera_span_ms": float(camera_span_ms),
            "dropped_lidar_frames_cumulative": int(
                self.dropped_lidar_frames
            ),
            "skipped_lidar_frames_cumulative": int(
                self.skipped_lidar_frames
            ),
        }
        if self.benchmark_output_dir is not None:
            self.benchmark_rows.append(row)
        if (
            self.processed_frames <= 5
            or self.processed_frames % 10 == 0
        ):
            rospy.loginfo(
                "frame=%d lidar=%d painted=%d boxes=%d "
                "camera_age=%.1fms queue=%.1fms "
                "seg=%.1fms paint=%.1fms det=%.1fms "
                "total=%.1fms dropped=%d skipped=%d",
                self.processed_frames,
                len(lidar),
                len(painted),
                len(boxes),
                camera_age_max_ms,
                queue_delay_ms,
                segmentation_ms,
                painting_ms,
                detection_ms,
                total_compute_ms,
                self.dropped_lidar_frames,
                self.skipped_lidar_frames,
            )


def main() -> None:
    rospy.init_node(
        "pointpainting_online",
        anonymous=False,
    )
    try:
        PointPaintingOnlineNode()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("PointPainting startup failed: %s", error)
        raise


if __name__ == "__main__":
    main()
