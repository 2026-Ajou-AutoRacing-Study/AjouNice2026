#!/usr/bin/env python3
"""Online four-camera Fast-BEV inference and T&P-compatible ROS outputs."""

from __future__ import annotations

import copy
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


# Make the repository-owned Fast-BEV/MMDetection3D fork importable before any
# OpenMMLab modules are loaded.  The launch wrapper also exports PYTHONPATH,
# but deriving the checkout root here keeps direct rosrun usage deterministic.
SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_REPO_ROOT = SCRIPT_PATH.parents[4]
for import_root in (DEFAULT_REPO_ROOT, DEFAULT_REPO_ROOT / "third_party/fastbev"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import cv2
import message_filters
import numpy as np
import rospy
import torch
from geometry_msgs.msg import Pose, Vector3
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from mmcv import Config
from mmcv.runner import load_checkpoint, wrap_fp16_model
from mmcv.utils import import_modules_from_strings
from mmdet.datasets.pipelines import Compose
from mmdet3d.core.bbox import get_box_type
from mmdet3d.models import build_model
from morai_pointpainting_ros.msg import (
    PointPaintingDetection,
    PointPaintingDetectionArray,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


CAMERA_NAMES = ("front", "front_left", "front_right", "back")
OPTICAL_TO_PHYSICAL = np.asarray(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)
JSK_CLASS_LABELS = {"vehicle": 1, "pedestrian": 3, "obstacle": 0}
MARKER_COLORS = {
    "vehicle": (0.1, 0.7, 1.0),
    "pedestrian": (1.0, 0.3, 0.2),
    "obstacle": (1.0, 0.8, 0.1),
}


def _stamp_sec(message: CompressedImage) -> float:
    return message.header.stamp.to_sec()


def _rpy_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    rx = np.asarray(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)],
         [0, math.sin(roll), math.cos(roll)]], dtype=np.float64)
    ry = np.asarray(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
         [-math.sin(pitch), 0, math.cos(pitch)]], dtype=np.float64)
    rz = np.asarray(
        [[math.cos(yaw), -math.sin(yaw), 0],
         [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _camera_calibration(sensor_config: Path) -> Dict[str, Dict[str, np.ndarray]]:
    """Build the exact Sensor_6 camera-to-LiDAR calibration used in training."""
    with sensor_config.open("r", encoding="utf-8") as stream:
        raw = json.load(stream)
    cameras = {}
    for item in raw["cameraList"]:
        sensor_name = item["cc"]["sensorName"]
        name = sensor_name[len("Camera_"):] if sensor_name.startswith("Camera_") else sensor_name
        cameras[name] = item
    missing = set(CAMERA_NAMES) - set(cameras)
    if missing:
        raise ValueError(f"Sensor config is missing cameras: {sorted(missing)}")
    if len(raw.get("Lidar3DList", [])) != 1:
        raise ValueError("Fast-BEV Sensor_6 calibration requires one 3D LiDAR")

    lidar = raw["Lidar3DList"][0]
    lidar_rotation = _rpy_matrix(
        *(float(lidar["rot"][key]) for key in ("roll", "pitch", "yaw")))
    lidar_translation = [float(lidar["pos"][key]) for key in "xyz"]
    lidar_to_ego = _transform(lidar_rotation, lidar_translation)

    result = {}
    for name in CAMERA_NAMES:
        item = cameras[name]
        cc = item["cc"]
        width = int(cc["cameraResWidth"])
        height = int(cc["cameraResHeight"])
        horizontal_fov = float(cc["cameraFOV"])
        focal = width / (2.0 * math.tan(math.radians(horizontal_fov) / 2.0))
        principal = cc["principalPoint"]
        intrinsic = np.asarray(
            [[focal, 0.0, float(principal["x"])],
             [0.0, focal, float(principal["y"])],
             [0.0, 0.0, 1.0]], dtype=np.float32)
        physical_rotation = _rpy_matrix(
            *(float(item["rot"][key]) for key in ("roll", "pitch", "yaw")))
        camera_rotation = physical_rotation @ OPTICAL_TO_PHYSICAL
        camera_translation = [float(item["pos"][key]) for key in "xyz"]
        camera_to_ego = _transform(camera_rotation, camera_translation)
        camera_to_lidar = np.linalg.inv(lidar_to_ego) @ camera_to_ego
        result[name] = {
            "intrinsic": intrinsic,
            "sensor2lidar_rotation": camera_to_lidar[:3, :3].astype(np.float32),
            "sensor2lidar_translation": camera_to_lidar[:3, 3].astype(np.float32),
            "resolution": np.asarray([width, height], dtype=np.int32),
        }
    return result


def _projection_metadata(calibration: Dict[str, Dict[str, np.ndarray]]) -> dict:
    extrinsics, augmentations, extras = [], [], []
    for name in CAMERA_NAMES:
        camera = calibration[name]
        rotation = camera["sensor2lidar_rotation"]
        translation = camera["sensor2lidar_translation"]
        intrinsic = camera["intrinsic"]

        lidar2cam_rotation = np.linalg.inv(rotation)
        lidar2cam_translation = translation @ lidar2cam_rotation.T
        # Mirror NuScenesDataset.get_data_info exactly.  It performs these
        # homogeneous products in float64 and MoraiMultiViewDataset casts the
        # resulting projection to float32 afterwards.
        lidar2cam = np.eye(4)
        lidar2cam[:3, :3] = lidar2cam_rotation.T
        lidar2cam[3, :3] = -lidar2cam_translation
        viewpad = np.eye(4)
        viewpad[:3, :3] = intrinsic
        extrinsics.append((viewpad @ lidar2cam.T).astype(np.float32))
        augmentations.append({
            "intrin": intrinsic.copy(),
            "rot": rotation.copy(),
            "tran": translation.copy(),
            "post_rot": np.eye(3),
            "post_tran": np.zeros(3),
        })
        extras.append({
            "sensor2lidar_rotation": rotation.copy(),
            "sensor2lidar_translation": translation.copy(),
            "cam_intrinsic": intrinsic.copy(),
        })
    return {
        "extrinsic": extrinsics,
        "intrinsic": np.eye(4, dtype=np.float32),
        "lidar2img_aug": augmentations,
        "lidar2img_extra": extras,
    }


class FastBEVOnlineNode:
    def __init__(self) -> None:
        self.repo_root = Path(
            rospy.get_param("~repo_root", str(DEFAULT_REPO_ROOT))).expanduser().resolve()
        self.config_path = Path(rospy.get_param("~config")).expanduser().resolve()
        self.checkpoint_path = Path(rospy.get_param("~checkpoint")).expanduser().resolve()
        self.sensor_config_path = Path(
            rospy.get_param("~sensor_config")).expanduser().resolve()
        self.output_frame_id = str(rospy.get_param("~output_frame_id", "velodyne"))
        self.use_fp16 = bool(rospy.get_param("~fp16", True))
        self.cudnn_benchmark = bool(rospy.get_param("~cudnn_benchmark", True))
        self.warmup_iterations = max(0, int(rospy.get_param("~warmup_iterations", 2)))
        self.sync_queue_size = max(2, int(rospy.get_param("~sync_queue_size", 12)))
        self.sync_slop_sec = max(0.0, float(rospy.get_param("~sync_slop_sec", 0.05)))
        self.max_detection_range = float(rospy.get_param("~max_detection_range", 50.0))

        for path, label in (
            (self.config_path, "Fast-BEV config"),
            (self.checkpoint_path, "Fast-BEV checkpoint"),
            (self.sensor_config_path, "MORAI sensor config"),
        ):
            if not path.is_file():
                raise FileNotFoundError(f"{label} not found: {path}")
        if not torch.cuda.is_available():
            raise RuntimeError("Fast-BEV online inference requires a CUDA GPU")

        self.class_names = list(rospy.get_param(
            "~class_names", ["vehicle", "pedestrian", "obstacle"]))
        thresholds = rospy.get_param("~score_thresholds", {})
        self.score_thresholds = {
            name: float(thresholds.get(name, 0.2)) for name in self.class_names}

        self.calibration = _camera_calibration(self.sensor_config_path)
        self.projection_metadata = _projection_metadata(self.calibration)
        self._load_model_and_pipeline()

        self.detections_pub = rospy.Publisher(
            rospy.get_param("~detections_topic", "/fastbev/detections"),
            PointPaintingDetectionArray, queue_size=1)
        self.jsk_pub = rospy.Publisher(
            rospy.get_param("~jsk_boxes_topic", "/hmi/perc/jsk_pillar_objects"),
            BoundingBoxArray, queue_size=1)
        self.markers_pub = rospy.Publisher(
            rospy.get_param("~markers_topic", "/fastbev/detection_markers"),
            MarkerArray, queue_size=1)

        camera_params = rospy.get_param("~cameras")
        topics = [camera_params[name]["topic"] for name in CAMERA_NAMES]
        self.camera_subscribers = [
            message_filters.Subscriber(topic, CompressedImage, queue_size=2)
            for topic in topics
        ]
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.camera_subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)

        self._condition = threading.Condition()
        self._pending = None
        self._shutdown = False
        self.received_sets = 0
        self.processed_sets = 0
        self.dropped_sets = 0
        self.skipped_sets = 0
        self._warmup_complete = self.warmup_iterations == 0
        self.worker = threading.Thread(target=self._worker_loop, name="fastbev_worker")
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("Fast-BEV online node ready")
        rospy.loginfo("  config: %s", self.config_path)
        rospy.loginfo("  checkpoint: %s", self.checkpoint_path)
        rospy.loginfo("  cameras: %s", ", ".join(topics))
        rospy.loginfo("  output frame: %s", self.output_frame_id)
        rospy.loginfo("  precision: %s", "FP16" if self.use_fp16 else "FP32")
        if not self._warmup_complete:
            rospy.loginfo(
                "Waiting for the first synchronized real 4-camera set for %d warm-up iterations",
                self.warmup_iterations)

    def _load_model_and_pipeline(self) -> None:
        fastbev_root = self.repo_root / "third_party/fastbev"
        for import_root in (self.repo_root, fastbev_root):
            if str(import_root) not in sys.path:
                sys.path.insert(0, str(import_root))
        self.cfg = Config.fromfile(str(self.config_path))
        import_modules_from_strings(**self.cfg.custom_imports)
        self.box_type_3d, self.box_mode_3d = get_box_type("LiDAR")
        pipeline_cfg = copy.deepcopy(self.cfg.data.test.pipeline)
        pipeline_cfg[0] = dict(type="MoraiMultiViewArrayPipeline", n_images=4)
        self.pipeline = Compose(pipeline_cfg)

        self.cfg.model.train_cfg = None
        model = build_model(self.cfg.model, test_cfg=self.cfg.get("test_cfg"))
        if self.use_fp16:
            wrap_fp16_model(model)
        checkpoint = load_checkpoint(model, str(self.checkpoint_path), map_location="cpu")
        checkpoint_classes = checkpoint.get("meta", {}).get("CLASSES")
        if checkpoint_classes:
            if list(checkpoint_classes) != self.class_names:
                raise ValueError(
                    f"Checkpoint classes {list(checkpoint_classes)} != ROS classes "
                    f"{self.class_names}")
        model.CLASSES = self.class_names
        torch.backends.cudnn.benchmark = self.cudnn_benchmark
        self.model = model.cuda().eval()

    def _synchronized_callback(self, *messages: CompressedImage) -> None:
        received_at = time.perf_counter()
        self.received_sets += 1
        with self._condition:
            if self._pending is not None:
                self.dropped_sets += 1
            self._pending = (messages, received_at)
            self._condition.notify()

    def _worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self._condition:
                while self._pending is None and not self._shutdown:
                    self._condition.wait(timeout=0.2)
                if self._shutdown:
                    return
                messages, received_at = self._pending
                self._pending = None
            try:
                self._process(messages, received_at)
            except Exception as error:  # keep the ROS node alive for transient bad frames
                self.skipped_sets += 1
                rospy.logerr_throttle(1.0, "Fast-BEV frame failed: %s", error)

    def _decode_images(self, messages: Sequence[CompressedImage]) -> List[np.ndarray]:
        images = []
        for name, message in zip(CAMERA_NAMES, messages):
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Failed to decode {name} compressed image")
            expected = self.calibration[name]["resolution"]
            if image.shape[1] != expected[0] or image.shape[0] != expected[1]:
                raise ValueError(
                    f"{name} resolution {image.shape[1]}x{image.shape[0]} != "
                    f"Sensor_6 {expected[0]}x{expected[1]}")
            images.append(image)
        return images

    def _prepare(self, images: Sequence[np.ndarray], sample_idx: str):
        results = {
            "sample_idx": sample_idx,
            "camera_images": list(images),
            "lidar2img": copy.deepcopy(self.projection_metadata),
            "box_type_3d": self.box_type_3d,
            "box_mode_3d": self.box_mode_3d,
        }
        prepared = self.pipeline(results)
        image_container = prepared["img"]
        meta_container = prepared["img_metas"]
        image_tensor = getattr(image_container, "data", image_container)
        image_meta = getattr(meta_container, "data", meta_container)
        if image_tensor.ndim != 4 or image_tensor.shape[0] != 4:
            raise AssertionError(f"Unexpected online image tensor shape: {image_tensor.shape}")
        return image_tensor.unsqueeze(0).cuda(non_blocking=True), [image_meta]

    def _infer(self, image_tensor, image_metas):
        with torch.no_grad():
            result = self.model(
                return_loss=False,
                rescale=True,
                img=image_tensor,
                img_metas=image_metas,
            )[0]
        boxes = result["boxes_3d"].tensor.detach().float().cpu().numpy()[:, :7]
        scores = result["scores_3d"].detach().float().cpu().numpy()
        labels = result["labels_3d"].detach().long().cpu().numpy()
        valid = np.zeros(len(scores), dtype=bool)
        for index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            if not 0 <= int(label) < len(self.class_names):
                continue
            class_name = self.class_names[int(label)]
            valid[index] = (
                float(score) >= self.score_thresholds[class_name]
                and float(np.linalg.norm(box[:2])) <= self.max_detection_range
            )
        boxes, scores, labels = boxes[valid], scores[valid], labels[valid]
        if len(scores):
            order = np.argsort(-scores)
            boxes, scores, labels = boxes[order], scores[order], labels[order]
        return boxes, scores, labels

    def _header(self, messages: Sequence[CompressedImage]) -> Header:
        newest = max(messages, key=_stamp_sec)
        header = Header()
        header.seq = newest.header.seq
        header.stamp = newest.header.stamp
        header.frame_id = self.output_frame_id
        return header

    def _publish(self, header, boxes, scores, labels, camera_span_ms,
                 queue_delay_ms, preprocess_ms, inference_ms, total_ms) -> None:
        detections = PointPaintingDetectionArray()
        detections.header = header
        detections.camera_age_max_ms = float(camera_span_ms)
        detections.camera_span_ms = float(camera_span_ms)
        detections.queue_delay_ms = float(queue_delay_ms)
        detections.segmentation_ms = 0.0
        detections.painting_ms = float(preprocess_ms)
        detections.detection_ms = float(inference_ms)
        detections.total_ms = float(total_ms)
        # These legacy field names are kept for wire compatibility.  For this
        # camera-only detector they count dropped/skipped synchronized sets.
        detections.dropped_lidar_frames = int(self.dropped_sets)
        detections.skipped_lidar_frames = int(self.skipped_sets)

        jsk = BoundingBoxArray()
        jsk.header = header
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            class_name = self.class_names[int(label)]
            qx, qy, qz, qw = _quaternion_from_yaw(float(box[6]))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, box[:3])
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
            size = Vector3(x=float(box[3]), y=float(box[4]), z=float(box[5]))

            detection = PointPaintingDetection()
            detection.detection_id = index
            detection.class_id = int(label) + 1
            detection.class_name = class_name
            detection.score = float(score)
            detection.pose = pose
            detection.size = size
            detection.distance = float(np.linalg.norm(box[:2]))
            detections.detections.append(detection)

            tracked_input = BoundingBox()
            tracked_input.header = header
            tracked_input.pose = pose
            tracked_input.dimensions = size
            tracked_input.value = float(score)
            tracked_input.label = JSK_CLASS_LABELS.get(class_name, 0)
            jsk.boxes.append(tracked_input)

            marker = Marker()
            marker.header = header
            marker.ns = f"fastbev_{class_name}"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale = size
            red, green, blue = MARKER_COLORS.get(class_name, (0.8, 0.8, 0.8))
            marker.color.r, marker.color.g, marker.color.b = red, green, blue
            marker.color.a = min(1.0, max(0.25, float(score)))
            marker.lifetime = rospy.Duration(0.2)
            markers.markers.append(marker)

        self.detections_pub.publish(detections)
        self.jsk_pub.publish(jsk)
        self.markers_pub.publish(markers)

    def _process(self, messages, received_at: float) -> None:
        total_start = time.perf_counter()
        timestamps = [_stamp_sec(message) for message in messages]
        camera_span_ms = (max(timestamps) - min(timestamps)) * 1000.0
        queue_delay_ms = (total_start - received_at) * 1000.0

        preprocess_start = time.perf_counter()
        images = self._decode_images(messages)
        image_tensor, image_metas = self._prepare(
            images, f"ros_{messages[0].header.seq}")
        torch.cuda.synchronize()
        preprocess_ms = (time.perf_counter() - preprocess_start) * 1000.0

        if not self._warmup_complete:
            for _ in range(self.warmup_iterations):
                self._infer(image_tensor, image_metas)
            torch.cuda.synchronize()
            self._warmup_complete = True
            rospy.loginfo("Real-input Fast-BEV warm-up complete")

        torch.cuda.synchronize()
        inference_start = time.perf_counter()
        boxes, scores, labels = self._infer(image_tensor, image_metas)
        torch.cuda.synchronize()
        inference_ms = (time.perf_counter() - inference_start) * 1000.0
        total_ms = (time.perf_counter() - total_start) * 1000.0
        header = self._header(messages)
        self._publish(
            header, boxes, scores, labels, camera_span_ms, queue_delay_ms,
            preprocess_ms, inference_ms, total_ms)
        self.processed_sets += 1
        if self.processed_sets <= 5 or self.processed_sets % 20 == 0:
            rospy.loginfo(
                "frame=%d boxes=%d camera_span=%.1fms queue=%.1fms "
                "preprocess=%.1fms inference=%.1fms total=%.1fms "
                "dropped=%d skipped=%d",
                self.processed_sets, len(boxes), camera_span_ms, queue_delay_ms,
                preprocess_ms, inference_ms, total_ms,
                self.dropped_sets, self.skipped_sets)

    def _on_shutdown(self) -> None:
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()


def main() -> None:
    rospy.init_node("fastbev_online")
    FastBEVOnlineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
