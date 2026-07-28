#!/usr/bin/env python3
"""
ROS1 Noetic online PointPainting node for MORAI.

Data flow:
4 camera Image topics
    -> latest-before-LiDAR temporal matching
    -> LRASPP semantic segmentation
LiDAR PointCloud2
    -> camera projection and semantic painting
    -> 10-channel painted points
    -> OpenPCDet PointPillars
    -> custom 3D detection array + RViz markers

The LiDAR subscriber never performs heavy work. It stores only the newest scan.
A worker thread processes one scan at a time and overwrites stale queued scans,
which prevents latency from growing without bound.
"""

from __future__ import annotations

import copy
import math
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
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

from morai_pointpainting_ros.msg import (
    PointPaintingDetection,
    PointPaintingDetectionArray,
)


SEMANTIC_NAMES = (
    "background",
    "person",
    "vehicle",
    "two_wheeler",
    "dog",
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


def cloud_to_xyzi(msg: PointCloud2) -> np.ndarray:
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
        return np.empty((0, 4), dtype=np.float32)

    raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    output = np.zeros((count, 4), dtype=np.float32)
    output[:, 0] = raw["x"]
    output[:, 1] = raw["y"]
    output[:, 2] = raw["z"]

    if "intensity" in names:
        output[:, 3] = raw["intensity"].astype(np.float32, copy=False)

    return output


def make_painted_cloud(points: np.ndarray, header: Header) -> PointCloud2:
    if points.ndim != 2 or points.shape[1] != 10:
        raise RuntimeError("Painted points must have shape (N, 10), got {}".format(points.shape))

    names = [
        "x",
        "y",
        "z",
        "intensity",
        "background",
        "person",
        "vehicle",
        "two_wheeler",
        "dog",
        "unknown",
    ]

    output = PointCloud2()
    output.header = copy.copy(header)
    output.height = 1
    output.width = int(len(points))
    output.fields = [
        PointField(name, index * 4, PointField.FLOAT32, 1)
        for index, name in enumerate(names)
    ]
    output.is_bigendian = False
    output.point_step = 40
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
            rospy.get_param("~score_threshold", 0.20)
        )
        self.max_detection_range = float(
            rospy.get_param("~max_detection_range", 20.0)
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
        self.publish_painted_points = bool(
            rospy.get_param("~publish_painted_points", False)
        )
        self.use_amp = bool(rospy.get_param("~use_amp", True))
        self.segmentation_short_side = int(
            rospy.get_param("~segmentation_short_side", 520)
        )
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
        if self.segmentation_short_side < 224:
            raise ValueError(
                "segmentation_short_side must be >= 224"
            )
        if self.warmup_iterations < 1:
            raise ValueError("warmup_iterations must be >= 1")
        if self.warmup_point_count < 1000:
            raise ValueError("warmup_point_count must be >= 1000")
        if self.real_input_warmup_iterations < 1:
            raise ValueError(
                "real_input_warmup_iterations must be >= 1"
            )

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

        rospy.loginfo("LiDAR       <- %s", self.lidar_topic)
        rospy.loginfo("Detections  -> %s", self.detections_topic)
        rospy.loginfo("Markers     -> %s", self.markers_topic)
        rospy.loginfo(
            "score>=%.2f range<=%.1fm camera_age<=%.1fms "
            "AMP=%s seg_short_side=%d native_semantic_map=True "
            "full_warmup=%s warmup_iterations=%d "
            "real_input_warmup=%s real_input_warmup_iterations=%d",
            self.score_threshold,
            self.max_detection_range,
            self.max_camera_age_ms,
            self.use_amp,
            self.segmentation_short_side,
            self.warmup_full_pipeline,
            self.warmup_iterations,
            self.real_input_warmup,
            self.real_input_warmup_iterations,
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

        if LRASPP_MobileNet_V3_Large_Weights is not None:
            weights = LRASPP_MobileNet_V3_Large_Weights.DEFAULT

            # Apply the launch parameter to the actual segmentation input size.
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
            self.segmentation_preprocess = transforms.Compose(
                [
                    transforms.Resize(self.segmentation_short_side),
                    transforms.ToTensor(),
                    transforms.Normalize(
                        mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225],
                    ),
                ]
            )
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
            "short_side=%d, channels_last=True, warmup=%s",
            self.device,
            torchvision_api,
            self.segmentation_short_side,
            self.warmup_segmentation,
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

        sys.path.insert(0, str(self.openpcdet_root))
        sys.path.insert(0, str(self.tools_dir))
        os.chdir(str(self.tools_dir))

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

        cfg_from_yaml_file(str(self.cfg_file), cfg)
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
            "PointPillars loaded: %s",
            self.checkpoint,
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
                    painted,
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
                    painted,
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

        self.skipped_lidar_frames = 0
        self.processed_frames = 0
        self.real_input_warmup_complete = True

        rospy.loginfo(
            "Real-input warm-up complete; "
            "dropped/skipped/processed counters reset."
        )
        self._log_online_ready()

    def _camera_callback(self, msg: Image, camera_name: str) -> None:
        timestamp = stamp_to_sec(msg.header.stamp)
        with self.camera_lock:
            self.camera_buffers[camera_name].append(
                (timestamp, msg)
            )

    def _lidar_callback(self, msg: PointCloud2) -> None:
        enqueue_time = time.perf_counter()
        with self.lidar_lock:
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
        images_rgb = {}

        for camera_name in self.camera_names:
            message = messages[camera_name]

            if isinstance(message, CompressedImage):
                image_bgr = self.bridge.compressed_imgmsg_to_cv2(
                    message,
                    desired_encoding="bgr8",
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
            images_rgb[camera_name] = image_rgb

        return images_rgb

    def _selected_score_maps(
        self,
        names: Sequence[str],
        images_rgb: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
        tensors = [
            self.segmentation_preprocess(
                PILImage.fromarray(images_rgb[name])
            )
            for name in names
        ]

        batch = torch.stack(tensors, dim=0).to(
            self.device,
            non_blocking=True,
        ).contiguous(
            memory_format=torch.channels_last
        )

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

        with torch.inference_mode():
            with torch.cuda.amp.autocast(enabled=self.use_amp):
                logits = self.segmentation_model(batch)["out"]

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

        return {
            name: scores[array_index]
            for array_index, name in enumerate(names)
        }

    def _segment_4cam(
        self,
        images_rgb: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
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

        semantic_sum = np.zeros(
            (len(lidar), 5),
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
            (len(lidar), 6),
            dtype=np.float32,
        )
        final_semantic[seen, :5] = (
            semantic_sum[seen]
            / camera_count[seen, None]
        )
        final_semantic[~seen, 5] = 1.0

        return np.concatenate(
            [lidar[:, :4], final_semantic],
            axis=1,
        ).astype(np.float32, copy=False)

    def _preprocess_lidar(self, msg: PointCloud2) -> np.ndarray:
        points = cloud_to_xyzi(msg)
        if len(points) == 0:
            return points

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if len(points) == 0:
            return points

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

        return np.ascontiguousarray(
            points[inside],
            dtype=np.float32,
        )

    def _detect(
        self,
        painted: np.ndarray,
        frame_id: str,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        sample = self.detection_dataset.prepare_data(
            data_dict={
                "points": painted,
                "frame_id": frame_id,
            }
        )
        batch = self.detection_dataset.collate_batch([sample])
        self.pcdet_load_data_to_gpu(batch)

        with torch.no_grad():
            predictions, _ = self.detection_model(batch)

        prediction = predictions[0]
        boxes = prediction["pred_boxes"].detach().cpu().numpy()
        scores = prediction["pred_scores"].detach().cpu().numpy()
        labels = prediction["pred_labels"].detach().cpu().numpy()

        distance = np.linalg.norm(boxes[:, :2], axis=1)
        valid = (
            (scores >= self.score_threshold)
            & (distance <= self.max_detection_range)
        )

        boxes = np.ascontiguousarray(boxes[valid, :7], dtype=np.float32)
        scores = np.asarray(scores[valid], dtype=np.float32)
        labels = np.asarray(labels[valid], dtype=np.int32)

        if len(scores):
            order = np.argsort(-scores)
            boxes = boxes[order]
            scores = scores[order]
            labels = labels[order]

        return boxes, scores, labels

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
        lidar_timestamp = stamp_to_sec(msg.header.stamp)

        camera_messages, camera_ages_ms = (
            self._select_camera_messages(lidar_timestamp)
        )
        if camera_messages is None:
            self.skipped_lidar_frames += 1
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

        lidar = self._preprocess_lidar(msg)
        if len(lidar) == 0:
            self.skipped_lidar_frames += 1
            rospy.logwarn_throttle(
                1.0,
                "Skipping empty LiDAR scan.",
            )
            return

        images_rgb = self._decode_images(camera_messages)

        if not self.real_input_warmup_complete:
            self._warmup_real_sensor_input(images_rgb, lidar)
            return

        torch.cuda.synchronize()
        segmentation_start = time.perf_counter()
        score_maps = self._segment_4cam(images_rgb)
        torch.cuda.synchronize()
        segmentation_ms = (
            time.perf_counter() - segmentation_start
        ) * 1000.0

        painting_start = time.perf_counter()
        painted = self._paint_lidar(lidar, score_maps)
        painting_ms = (
            time.perf_counter() - painting_start
        ) * 1000.0

        torch.cuda.synchronize()
        detection_start = time.perf_counter()
        boxes, scores, labels = self._detect(
            painted,
            str(msg.header.seq),
        )
        torch.cuda.synchronize()
        detection_ms = (
            time.perf_counter() - detection_start
        ) * 1000.0

        total_ms = (
            time.perf_counter() - total_start
        ) * 1000.0
        queue_delay_ms = (
            total_start - enqueue_time
        ) * 1000.0

        output = self._make_detection_array(
            header=msg.header,
            boxes=boxes,
            scores=scores,
            labels=labels,
            camera_age_max_ms=camera_age_max_ms,
            camera_span_ms=camera_span_ms,
            queue_delay_ms=queue_delay_ms,
            segmentation_ms=segmentation_ms,
            painting_ms=painting_ms,
            detection_ms=detection_ms,
            total_ms=total_ms,
        )
        self.detections_pub.publish(output)

        names = [
            self.class_names[int(label) - 1]
            if 0 < int(label) <= len(self.class_names)
            else "unknown"
            for label in labels
        ]
        self.markers_pub.publish(
            make_markers(
                boxes,
                scores,
                names,
                msg.header,
            )
        )

        if self.publish_painted_points:
            self.painted_pub.publish(
                make_painted_cloud(painted, msg.header)
            )

        self.processed_frames += 1
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
                total_ms,
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
