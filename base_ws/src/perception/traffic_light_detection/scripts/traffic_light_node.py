#!/usr/bin/env python3
"""
traffic_light_node.py (v14 - raw 라벨 디버그 토픽 추가, 스무딩 파라미터 재조정용)

v13 -> v14 변경점 (2026-08-19, 스무딩 창(5)/과반수(3) 값 재조정을 위한 실측 준비):
  - 지금까지 사용해온 스무딩 파라미터(window_size=5, majority_count=3)는 실측
    근거 없는 잠정치. 이걸 재조정하려면 raw(스무딩 전) 데이터가 매 프레임 얼마나
    오래(몇 프레임) 지속되는지 알아야 하는데, 기존 구조는 최종(스무딩 후) 데이터만
    외부에서 관측 가능했음.
  - 수정: raw_color_results(보정까지는 거치되 스무딩 전)를 매 프레임
    ~debug_raw_label(std_msgs/String) 토픽으로 추가 발행. 이 토픽을 별도
    로거로 구독해서, raw 라벨이 몇 프레임 연속으로
    유지되는지 실측 가능하게 함.
"""
import os
import sys
import time as _time_module
import json
import hashlib
import threading
import math
from collections import deque

_user_site = os.path.expanduser(f"~/.local/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages")
if (os.environ.get("TRAFFIC_LIGHT_ISOLATE_PYTHON", "false").lower() != "true"
        and os.path.isdir(_user_site) and _user_site not in sys.path):
    sys.path.insert(0, _user_site)
_this_dir = os.path.dirname(os.path.abspath(__file__))
if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt64
from ultralytics import YOLO

from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)

from light_color_utils import classify_crop, pick_ego_lane_box
from ros_time import builtin_time_from_ros
from signal_smoother import SignalSmoother
from signal_phase_policy import SignalPhasePolicy
from traffic_light_map_selector import (MISSION_GROUP_IDS, associate_boxes, select_packet,
                                        SELECTOR_CONTRACT_VERSION, selector_config)
from traffic_light_near_rescue import (near_rescue_config, near_rescue_decision,
                                       accept_near_rescue_selection)
from traffic_light_debug_view import render_node_overlay
from traffic_light_observation_handoff import detection_miss_context, low_confidence_context, no_lamps_context
from traffic_light_visual_continuation import VisualContinuation
from traffic_light_stamped_vote import StampedPhaseVote
from traffic_light_unreadable_evidence import unreadable_color_evidence

COLOR_MAP = {
    "red": TrafficLightElement.RED,
    "yellow": TrafficLightElement.AMBER,
    "green": TrafficLightElement.GREEN,
}

INVALID_EXTERNAL_GROUP_ID = 0
DEBUG_DRAW_COLOR = {
    "red": (0, 0, 255),
    "yellow": (0, 220, 255),
    "green": (0, 255, 0),
    "off": (255, 255, 255),
    "mixed": (255, 0, 255),
}

HZ_LOG_INTERVAL = 30

EXCLUDED_MAP_ERROR_IDS = {201395, 201396}


def _detector_runtime_device(model, requested_device):
    """Read the inference backend, not Ultralytics' retained outer module.

    Ultralytics may deepcopy the outer module when it creates a predictor, so
    outer Model.device can still be CPU after predict(device=0) runs on CUDA.
    This is diagnostic-only: never move a model or select an inference device.
    """
    predictor = getattr(model, "predictor", None)
    for source, owner in (("predictor.device", predictor),
                          ("predictor.model.device", getattr(predictor, "model", None)),
                          ("model.device", model)):
        device = getattr(owner, "device", None)
        if device is not None:
            return str(device), source
    return str(requested_device), "requested_device_fallback"


def _pick_draw_color(color_results):
    present = {c for c, _shape, _is_left in color_results}
    if len(present) == 0:
        return "off"
    if len(present) == 1:
        return next(iter(present))
    return "mixed"


def _raw_label_for(color_results):
    if not color_results:
        return "off"
    parts = []
    for color, shape, is_left in color_results:
        if shape == "arrow":
            parts.append(f"{color}_arrow_left" if is_left else f"{color}_arrow_right")
        else:
            parts.append(f"{color}_{shape}")
    return "+".join(sorted(parts))


_SHORT_TOKEN = {
    ("red", "circle", None): "RC",
    ("yellow", "circle", None): "YC",
    ("green", "circle", None): "GC",
    ("green", "arrow", True): "GLA",
    ("green", "arrow", False): "GRA",
}


def _short_label(color_results):
    if not color_results:
        return "OFF"
    return "+".join(sorted(_SHORT_TOKEN.get(tuple(r), "?") for r in color_results))


class _StatusLog:
    """[Traffic] 상태를 1초에 한 줄로만 출력한다.

    한 주기 안에서 가장 심각한 사건의 레벨로 찍어, rosconsole 기본 색상
    (DEBUG 초록 / INFO 흰색 / WARN 노랑 / ERROR 빨강)이 그대로 의미를 갖게 한다.
    """

    INFO, WARN, ERROR = 0, 1, 2

    def __init__(self, period_sec=1.0):
        self.period = float(period_sec)
        self._last_emit = 0.0
        self._level = self.INFO
        self._note = ""

    def note(self, level, text):
        if level >= self._level:
            self._level = level
            self._note = text

    def tick(self, now, section, phase=None):
        if now - self._last_emit < self.period:
            return
        self._last_emit = now
        body = section if phase is None else f"{section} | {phase}"
        if self._note:
            body = f"{body} | {self._note}"
        if self._level >= self.ERROR:
            rospy.logerr(f"[Traffic][ERROR] {body}")
        elif self._level == self.WARN:
            rospy.logwarn(f"[Traffic][WARN] {body}")
        else:
            rospy.loginfo(f"[Traffic] {body}")
        self._level = self.INFO
        self._note = ""


class TrafficLightNode:
    def __init__(self):
        weights_path = rospy.get_param("~weights_path")
        image_topic = rospy.get_param("~image_topic", "/sensor/camera/front/compressed")
        self.conf_threshold = rospy.get_param("~conf", 0.25)
        # Keep traffic-light inference independent from the detector runtime.
        # A GPU-capable Python can be selected by launch argument on hosts whose
        # system PyTorch does not support the installed GPU architecture.
        self.inference_device = rospy.get_param("~device", "cpu")
        self.detector_imgsz = int(rospy.get_param("~detector_imgsz", 640))
        if not 320 <= self.detector_imgsz <= 1920 or self.detector_imgsz % 32:
            raise ValueError("detector_imgsz must be a multiple of 32 within 320..1920")
        self.near_rescue_config = near_rescue_config({
            "enabled": rospy.get_param("~near_rescue_enabled", False),
        })
        self.center_tolerance_ratio = rospy.get_param("~center_tolerance_ratio", 0.20)
        self.y_weight = rospy.get_param("~y_weight", 0.6)
        self.fallback_group_id = rospy.get_param("~fallback_group_id", 0)
        self.publish_rate_limit_hz = rospy.get_param("~max_rate_hz", 10.0)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", False)
        self.debug_jpeg_quality = rospy.get_param("~debug_jpeg_quality", 60)
        self.selection_mode = rospy.get_param("~selection_mode", "legacy_center")
        if self.selection_mode not in ("legacy_center", "map_association"):
            raise ValueError("selection_mode must be legacy_center or map_association")
        self.classifier_mode = rospy.get_param("~classifier_mode", "hsv")
        if self.classifier_mode not in ("hsv", "cnn"):
            raise ValueError("classifier_mode must be hsv or cnn")
        self.cnn_crop_margin_px = int(rospy.get_param("~cnn_crop_margin_px", 12))
        if not 0 <= self.cnn_crop_margin_px <= 64:
            raise ValueError("cnn_crop_margin_px must be within 0..64")
        self._cnn_startup = None
        if self.classifier_mode == "cnn":
            from traffic_light_cnn_classifier import initialize_classifier
            self._cnn_startup = initialize_classifier(
                rospy.get_param("~cnn_checkpoint_path", ""),
                device=rospy.get_param("~cnn_device", "cpu"),
                min_bit_confidence=float(rospy.get_param("~cnn_min_bit_confidence", .6)),
                warmup=True,
            )
            if not self._cnn_startup.ready:
                rospy.logerr("[Traffic] CNN unavailable; missing observations, no HSV fallback: %s",
                             self._cnn_startup.reason)
        self._roi_lock = threading.Lock()
        self._roi_condition = threading.Condition(self._roi_lock)
        self._roi_packets = deque(maxlen=20)
        self._selected_head_id = None
        self.map_packet_max_image_delta_sec = float(rospy.get_param("~map_packet_max_image_delta_sec", 0.))
        self.map_packet_max_age_sec = float(rospy.get_param("~map_packet_max_age_sec", .5))
        self.map_packet_wait_sec = float(rospy.get_param("~map_packet_wait_sec", .03))
        if not 0 <= self.map_packet_wait_sec <= .1 or not 0 <= self.map_packet_max_image_delta_sec <= .05:
            raise ValueError("map packet wait/delta exceeds bounded candidate limits")

        self.debug_save_crop_dir = rospy.get_param("~debug_save_crop_dir", "")
        self._save_crop_count = 0
        if self.debug_save_crop_dir:
            os.makedirs(self.debug_save_crop_dir, exist_ok=True)
            rospy.loginfo(
                f"[Traffic] 충돌 크롭 저장: {self.debug_save_crop_dir}"
            )

        self.status = _StatusLog(float(rospy.get_param("~log_interval_sec", 1.0)))

        self.planner_compliance_observations = rospy.get_param("~planner_compliance_observations", False)
        if type(self.planner_compliance_observations) is not bool:
            raise ValueError("planner_compliance_observations must be an explicit bool")
        self.preserve_unreadable_color_evidence = rospy.get_param('~preserve_unreadable_color_evidence', False)
        if type(self.preserve_unreadable_color_evidence) is not bool:
            raise ValueError('preserve_unreadable_color_evidence must be an explicit bool')
        if self.preserve_unreadable_color_evidence and not self.planner_compliance_observations:
            raise ValueError('unreadable color evidence requires observed-only handoff, not legacy corrections')
        self.stamped_vote_window_size = rospy.get_param('~stamped_vote_window_size', 0)
        if type(self.stamped_vote_window_size) is not int or self.stamped_vote_window_size not in (0, 3, 7):
            raise ValueError('stamped_vote_window_size must be 0 (disabled), 3, or 7 (experiment)')
        self.stamped_vote_max_age_sec = rospy.get_param('~stamped_vote_max_age_sec', .5)
        self.stamped_vote_startup_passthrough = rospy.get_param('~stamped_vote_startup_passthrough', False)
        if type(self.stamped_vote_startup_passthrough) is not bool:
            raise ValueError('stamped_vote_startup_passthrough must be an explicit bool')
        self._stamped_vote = None
        if self.stamped_vote_window_size:
            if not self.planner_compliance_observations or self.selection_mode != 'map_association':
                raise ValueError('stamped vote requires observed-only map_association; not legacy corrections')
            self._stamped_vote = StampedPhaseVote(self.stamped_vote_window_size, self.stamped_vote_max_age_sec,
                                                  self.stamped_vote_startup_passthrough)
        self.detection_miss_hold_sec = float(rospy.get_param("~detection_miss_hold_sec", .2))
        if not 0 <= self.detection_miss_hold_sec <= .2:
            rospy.logwarn("[Traffic] invalid detection hold; disabling optional dropout hold")
            self.detection_miss_hold_sec = 0.
        self.low_confidence_hold_sec = float(rospy.get_param("~low_confidence_hold_sec", .3))
        if not 0 <= self.low_confidence_hold_sec <= .3:
            rospy.logwarn("[Traffic] invalid low confidence hold; disabling optional hold")
            self.low_confidence_hold_sec = 0.
        self.no_lamps_hold_sec = float(rospy.get_param('~no_lamps_hold_sec', .2))
        if not math.isfinite(self.no_lamps_hold_sec) or not 0 <= self.no_lamps_hold_sec <= .3:
            raise ValueError('no_lamps_hold_sec must be finite in [0,0.3]')
        self.visual_continuation_sec=float(rospy.get_param('~visual_continuation_sec',0.))
        if not math.isfinite(self.visual_continuation_sec) or not 0<=self.visual_continuation_sec<=.3:
            raise ValueError('visual_continuation_sec must be finite in [0,0.3]')
        self._visual_continuation=VisualContinuation()
        smoothing_window_size = int(rospy.get_param("~smoothing_window_size", 5))
        smoothing_majority_count = rospy.get_param("~smoothing_majority_count", None)
        if smoothing_majority_count is not None:
            smoothing_majority_count = int(smoothing_majority_count)
        self.signal_smoother = SignalSmoother(
            window_size=smoothing_window_size,
            majority_count=smoothing_majority_count,
        )
        rospy.logdebug(
            f"[Traffic] 스무딩 window={smoothing_window_size} "
            f"majority={self.signal_smoother.majority_count}"
        )

        self.route_group_id_topic = rospy.get_param(
            "~route_group_id_topic",
            "/perception/traffic_light_recognition/route_group_id",
        )
        self.route_group_id_stale_sec = float(
            rospy.get_param("~route_group_id_stale_sec", 2.0)
        )
        signal_profile_path = rospy.get_param("~signal_profile_path")
        self.signal_phase_policy = SignalPhasePolicy.from_yaml(signal_profile_path)
        self._external_route_group_id = None
        self._external_route_group_id_stamp = None
        self._last_policy_group_id = None
        rospy.logdebug(f"[Traffic] 프로파일: {signal_profile_path}")

        if not os.path.exists(weights_path):
            rospy.logerr(f"[Traffic][ERROR] 가중치 파일 없음: {weights_path}")

        rospy.loginfo(f"[Traffic] 기동: YOLO {os.path.basename(weights_path)} device={self.inference_device}")
        self.model = YOLO(weights_path)
        self.runtime_metadata = {
            "selection_mode": self.selection_mode, "classifier_mode": self.classifier_mode,
            "detector_weights_path": os.path.abspath(os.path.expanduser(weights_path)),
            "detector_requested_device": str(self.inference_device), "detector_imgsz": self.detector_imgsz,
            "cnn_crop_margin_px": self.cnn_crop_margin_px,
            "planner_compliance_observations": self.planner_compliance_observations,
            "preserve_unreadable_color_evidence": self.preserve_unreadable_color_evidence,
            "stamped_vote_window_size": self.stamped_vote_window_size,
            "stamped_vote_max_age_sec": self.stamped_vote_max_age_sec,
            "stamped_vote_startup_passthrough": self.stamped_vote_startup_passthrough,
            "low_confidence_hold_sec": self.low_confidence_hold_sec,
            "no_lamps_hold_sec": self.no_lamps_hold_sec,
            "no_lamps_hold_source_age_cap_sec": .3,
            "low_confidence_hold_source_age_cap_sec": .5,
            "low_confidence_hold_min_speed_mps": 1.,
            "visual_continuation_sec": self.visual_continuation_sec,
            "near_rescue": dict(enabled=self.near_rescue_config.enabled,
                                imgsz=self.near_rescue_config.imgsz,
                                max_depth_m=self.near_rescue_config.max_depth_m,
                                min_projected_width_px=self.near_rescue_config.min_projected_width_px),
            "cnn_startup_ready": self._cnn_startup.ready if self._cnn_startup else None,
            "cnn_metadata": self._cnn_startup.metadata if self._cnn_startup else None,
        }
        try:
            with open(weights_path, "rb") as stream:
                self.runtime_metadata["detector_weights_sha256"] = hashlib.sha256(stream.read()).hexdigest()
        except OSError:
            self.runtime_metadata["detector_weights_sha256"] = None

        self.pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/perception/traffic_light_recognition/traffic_signals"),
            TrafficLightGroupArray,
            queue_size=1,
        )
        self.sub_route_group_id = rospy.Subscriber(
            self.route_group_id_topic,
            UInt64,
            self.route_group_id_callback,
            queue_size=1,
        )
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(
                "~debug_image/compressed", CompressedImage, queue_size=1
            )

        self.raw_debug_pub = rospy.Publisher("~debug_raw_label", String, queue_size=10)
        self.observed_debug_pub = rospy.Publisher("~debug_observed_label", String, queue_size=10)
        self.policy_debug_pub = rospy.Publisher("~debug_policy_reason", String, queue_size=10)
        self.selection_debug_pub = rospy.Publisher("~debug_selection", String, queue_size=10)
        self.classifier_debug_pub = rospy.Publisher("~debug_classifier", String, queue_size=10)
        self.runtime_debug_pub = rospy.Publisher("~debug_runtime", String, queue_size=1, latch=True)
        self._publish_runtime_debug()
        self.sub_map_rois = None
        if self.selection_mode == "map_association":
            self.sub_map_rois = rospy.Subscriber(
                rospy.get_param("~map_roi_topic", "/perception/traffic_light_recognition/map_rois"),
                String, self.map_roi_callback, queue_size=10)

        self._last_stamp = rospy.Time(0)
        self._min_period = rospy.Duration(1.0 / self.publish_rate_limit_hz) if self.publish_rate_limit_hz > 0 else rospy.Duration(0)

        self._n_processed = 0
        self._proc_time_sum = 0.0
        self._proc_time_max = 0.0
        self._hz_window_start = time.time()
        self._n_conflict_fixed = 0

        self.sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(
            f"[Traffic] 구독 {image_topic}"
            + (" / 디버그 영상 ON" if self.publish_debug_image else "")
        )

    def route_group_id_callback(self, msg):
        group_id = int(msg.data)
        self._external_route_group_id = (
            None if group_id == INVALID_EXTERNAL_GROUP_ID else group_id
        )
        self._external_route_group_id_stamp = rospy.Time.now()

    def map_roi_callback(self, msg):
        try:
            packet = json.loads(msg.data)
            if not isinstance(packet, dict):
                raise ValueError("ROI packet must be an object")
            with self._roi_condition:
                self._roi_packets.append(packet)
                self._roi_condition.notify_all()
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(5., "[Traffic] invalid map ROI packet: %s", error)

    def _map_packet_for_image(self, msg, group_id, width, height):
        deadline = time.monotonic() + self.map_packet_wait_sec
        with self._roi_condition:
            while True:
                packet = select_packet(self._roi_packets, msg.header.stamp.to_nsec(), group_id,
                    width, height, rospy.Time.now().to_nsec(), self.map_packet_max_image_delta_sec,
                    self.map_packet_max_age_sec, msg.header.frame_id)
                remaining = deadline-time.monotonic()
                if packet is not None or group_id <= 0 or msg.header.stamp.to_nsec() <= 0 or remaining <= 0:
                    return packet
                # A sibling ROS callback can deliver the exact image-stamped packet.
                self._roi_condition.wait(remaining)

    def _resolve_group_id(self):
        """Use the system-Python route matcher; return zero when it is stale."""
        if (
            self._external_route_group_id is not None
            and self._external_route_group_id_stamp is not None
        ):
            age = (rospy.Time.now() - self._external_route_group_id_stamp).to_sec()
            if 0 <= age <= self.route_group_id_stale_sec:
                if self.selection_mode == "map_association" and self._external_route_group_id not in MISSION_GROUP_IDS:
                    return 0
                return self._external_route_group_id
        return 0 if self.selection_mode == "map_association" else self.fallback_group_id

    def _log_hz_if_needed(self):
        self._n_processed += 1
        if self._n_processed % HZ_LOG_INTERVAL == 0:
            elapsed = time.time() - self._hz_window_start
            hz = HZ_LOG_INTERVAL / elapsed if elapsed > 0 else 0.0
            avg_ms = (self._proc_time_sum / HZ_LOG_INTERVAL) * 1000.0
            max_ms = self._proc_time_max * 1000.0
            rospy.logdebug(
                f"[Traffic] {hz:.1f}Hz 평균 {avg_ms:.1f}ms 최대 {max_ms:.1f}ms"
            )
            self._hz_window_start = time.time()
            self._proc_time_sum = 0.0
            self._proc_time_max = 0.0

    def _publish_runtime_debug(self, source_stamp=None, frame_timing=None, frame_decision=None):
        report = dict(self.runtime_metadata)
        report["detector_actual_device"], report["detector_actual_device_source"] = _detector_runtime_device(
            self.model, self.inference_device)
        report["source_stamp_ns"] = source_stamp.to_nsec() if source_stamp is not None else None
        report["frame_timing"] = frame_timing
        report["frame_decision"] = frame_decision
        report["cuda_memory"] = None
        if "cuda" in report["detector_actual_device"]:
            try:
                import torch
                device = report["detector_actual_device"]
                report["cuda_memory"] = {"device": device,
                    "allocated_bytes": torch.cuda.memory_allocated(device),
                    "reserved_bytes": torch.cuda.memory_reserved(device),
                    "max_allocated_bytes": torch.cuda.max_memory_allocated(device)}
            except (RuntimeError, ValueError, ImportError) as error:
                report["cuda_memory_error"] = str(error)
        self.runtime_debug_pub.publish(String(data=json.dumps(report, allow_nan=False)))

    def _maybe_save_conflict_crop(self, crop, raw_color_results):
        if not self.debug_save_crop_dir:
            return
        self._save_crop_count += 1
        summary = "+".join(f"{c}_{s}" for c, s, _l in raw_color_results).replace("/", "-")
        fname = f"conflict_{int(_time_module.time() * 1000)}_{self._save_crop_count:04d}_{summary}.png"
        fpath = os.path.join(self.debug_save_crop_dir, fname)
        cv2.imwrite(fpath, crop)

    def image_callback(self, msg):
        now = rospy.Time.now()
        if now - self._last_stamp < self._min_period:
            return
        self._last_stamp = now

        t_start = time.time()
        t_monotonic = time.monotonic()

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("cv2.imdecode 실패 (None)")
        except Exception as e:
            self.status.note(_StatusLog.ERROR, f"이미지 디코딩 실패: {e}")
            self.status.tick(time.time(), "—")
            return

        h, w = img.shape[:2]
        t_decoded = time.monotonic()

        results = self.model.predict(
            source=img,
            conf=self.conf_threshold,
            device=self.inference_device,
            imgsz=self.detector_imgsz,
            verbose=False,
        )
        boxes = self._read_detector_boxes(results, w, h)
        t_detected = time.monotonic()
        rescue_timing_sec = 0.
        rescue_debug = {"attempted": False, "reason": "not_map_association"}

        group_array = TrafficLightGroupArray()
        source_stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else now
        group_array.stamp = builtin_time_from_ros(source_stamp)

        debug_img = img if self.publish_debug_image else None

        group = TrafficLightGroup()
        group.traffic_light_group_id = self._resolve_group_id()
        if group.traffic_light_group_id != self._last_policy_group_id:
            self.signal_smoother.reset()
            self.signal_phase_policy.reset_transition()
            self._last_policy_group_id = group.traffic_light_group_id
            self._selected_head_id = None
            if hasattr(self,'_visual_continuation'):self._visual_continuation.reset()
            if getattr(self, '_stamped_vote', None) is not None:
                self._stamped_vote.reset()

        previous_head_id = self._selected_head_id
        packet = None
        selection = {"status": "legacy", "reason": "legacy_center"}
        if self.selection_mode == "legacy_center":
            best = pick_ego_lane_box(boxes, self.center_tolerance_ratio, self.y_weight)
        else:
            packet = self._map_packet_for_image(msg, group.traffic_light_group_id, w, h)
            if packet is None or packet.get("status") != "ready":
                best = None
                selection = {"status": "missing", "reason": "map_packet_missing_or_stale" if packet is None
                             else packet.get("reason", "map_projection_missing")}
                rescue_debug["reason"] = selection["reason"]
            else:
                try:
                    if packet.get("selector_contract_version") != SELECTOR_CONTRACT_VERSION:
                        raise ValueError("selector_contract_version_mismatch")
                    if not isinstance(packet.get("selector_config"), dict):
                        raise ValueError("missing_selector_configuration")
                    config = selector_config(packet.get("selector_config"))
                    best, selection = associate_boxes(boxes, packet.get("rois", []),
                        group.traffic_light_group_id, self._selected_head_id, config=config)
                    selection["projection_reason"] = packet.get("reason")
                    decision = near_rescue_decision(selection, packet.get("rois", []),
                        group.traffic_light_group_id, config=getattr(self, "near_rescue_config", None))
                    if packet["source_stamp_ns"] != msg.header.stamp.to_nsec():
                        decision.update(retry=False, reason="retry_requires_exact_image_stamp")
                    rescue_debug = dict(decision, attempted=False)
                    if decision["retry"] and self.detector_imgsz != decision["imgsz"]:
                        # The SAME original image and exact/fresh map packet are
                        # reused. No confidence, classifier or signal-policy relaxation.
                        rescue_started = time.monotonic()
                        rescue_debug.update(attempted=True, primary_reason=selection["reason"])
                        try:
                            retry_results = self.model.predict(source=img, conf=self.conf_threshold,
                                device=self.inference_device, imgsz=decision["imgsz"], verbose=False)
                            retry_boxes = self._read_detector_boxes(retry_results, w, h)
                            retry_best, retry_selection = associate_boxes(boxes + retry_boxes,
                                packet.get("rois", []), group.traffic_light_group_id,
                                self._selected_head_id, config=config)
                            context_reason = self._near_retry_context_reason(packet, group.traffic_light_group_id)
                            accepted = (context_reason is None and
                                        accept_near_rescue_selection(retry_selection, decision))
                            rescue_debug.update(secondary_detection_count=len(retry_boxes),
                                accepted=accepted, context_rejection=context_reason,
                                result=dict(retry_selection))
                            if accepted:
                                best, selection = retry_best, retry_selection
                            # Primary boxes remain present for identity checks/debug.
                            boxes = boxes + retry_boxes
                        except Exception as error:
                            rescue_debug.update(accepted=False, error_type=type(error).__name__)
                            self.status.note(_StatusLog.WARN, "근접 재검출 실패; missing 유지")
                        rescue_timing_sec = time.monotonic() - rescue_started
                    elif decision["retry"]:
                        rescue_debug["reason"] = "primary_already_retry_scale"
                    selection["selector_contract_version"] = SELECTOR_CONTRACT_VERSION
                    selection["selector_config"] = config.to_dict()
                    selection["projection_reason"] = packet.get("reason")
                    selection["eligible_projected_head_ids"] = [roi.get("head_id") for roi in packet.get("rois", [])
                                                               if roi.get("eligible")]
                except (ValueError, TypeError, KeyError, IndexError) as error:
                    best, selection = None, {"status": "missing", "reason": "invalid_map_rois:" + str(error)}
                selection["projection_source_stamp_ns"] = packet["source_stamp_ns"]
            self._selected_head_id = selection.get("head_id")
        visual_debug=dict(status='disabled',reason='visual_continuation_disabled')
        if self.selection_mode=='map_association' and getattr(self,'visual_continuation_sec',0.)>0:
            if not hasattr(self,'_visual_continuation'):self._visual_continuation=VisualContinuation()
            if best is not None:
                self._visual_continuation.remember(best,selection,group.traffic_light_group_id,
                    msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec())
            else:
                best,visual_debug=self._visual_continuation.select(boxes,packet,selection,
                    group.traffic_light_group_id,msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec(),
                    self.visual_continuation_sec)
                if (best is None and visual_debug.get('retry_imgsz')==640
                        and self.near_rescue_config.enabled and self.detector_imgsz!=640):
                    try:
                        retry=self.model.predict(source=img,conf=self.conf_threshold,device=self.inference_device,
                                                 imgsz=640,verbose=False)
                        boxes=boxes+self._read_detector_boxes(retry,w,h)
                        best,visual_debug=self._visual_continuation.select(boxes,packet,selection,
                            group.traffic_light_group_id,msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec(),
                            self.visual_continuation_sec)
                        visual_debug['retry640_attempted']=True
                    except Exception as error:
                        best=None;self._visual_continuation.reset()
                        visual_debug.update(status='missing',reason='visual_retry_error:'+type(error).__name__)
                if best is not None:
                    if self._near_retry_context_reason(packet,group.traffic_light_group_id) is not None:
                        best=None;self._visual_continuation.reset()
                        visual_debug.update(status='missing',reason='visual_context_changed_before_crop')
                    else:
                        selection=dict(visual_debug)
                        self._selected_head_id=selection['head_id']
        hold_context = detection_miss_context(packet, selection, previous_head_id,
            group.traffic_light_group_id, msg.header.stamp.to_nsec(), rospy.Time.now().to_nsec(),
            self.map_packet_max_age_sec)
        if hold_context["eligible"] and self._near_retry_context_reason(packet, group.traffic_light_group_id) is not None:
            hold_context.update(eligible=False, reason="route_or_packet_changed_before_handoff")
        if hold_context["eligible"]:
            # Identity continuity only. This is not a fresh detection or lamp.
            self._selected_head_id = previous_head_id
        selection.update(mode=self.selection_mode, group_id=group.traffic_light_group_id,
                         source_stamp_ns=msg.header.stamp.to_nsec(), detection_count=len(boxes),
                         near_rescue=rescue_debug, detection_miss_hold=hold_context)
        selection['visual_continuation']=visual_debug
        self.selection_debug_pub.publish(String(data=json.dumps(selection, allow_nan=False)))
        t_selected = time.monotonic()
        label_text = "off"
        draw_color_name = "off"
        box_coords = None
        det_conf = 0.0

        raw_color_results = []
        raw_det_conf = 0.0
        classifier_debug = {"mode": self.classifier_mode, "known": False, "reason": "no_selected_crop",
                            "source_stamp_ns": msg.header.stamp.to_nsec(),
                            "group_id": group.traffic_light_group_id,
                            "head_id": selection.get("head_id")}

        if best is not None:
            x1, y1, x2, y2, det_conf = best[4]
            margin = self.cnn_crop_margin_px if self.classifier_mode == "cnn" else 0
            x1c, y1c = max(0, int(x1)-margin), max(0, int(y1)-margin)
            x2c, y2c = min(w, int(x2)+margin), min(h, int(y2)+margin)
            crop = img[y1c:y2c, x1c:x2c]
            box_coords = (x1c, y1c, x2c, y2c)
            raw_det_conf = det_conf
            classifier_debug.update(detector_bbox_xyxy=[x1, y1, x2, y2],
                                    crop_xyxy=list(box_coords), crop_margin_px=margin)

            if crop.size > 0:
                if self.classifier_mode == "hsv":
                    raw_color_results, _counts = classify_crop(crop)
                    classifier_debug.update(known=True, reason="hsv_observation", counts=_counts)
                elif self._cnn_startup.ready:
                    observation = self._cnn_startup.classifier.classify_bgr(crop)
                    raw_color_results = list(observation.tuples) if observation.known else []
                    classifier_debug.update(observation.to_dict())
                    raw_det_conf = min(raw_det_conf, observation.confidence) if observation.known else 0.
                else:
                    classifier_debug["reason"] = self._cnn_startup.reason

                self._maybe_save_conflict_crop(crop, raw_color_results)

        low_confidence_hold = low_confidence_context(packet, selection, classifier_debug, previous_head_id,
            group.traffic_light_group_id, msg.header.stamp.to_nsec(), rospy.Time.now().to_nsec(),
            self.map_packet_max_age_sec)
        if (low_confidence_hold["eligible"] and
                self._near_retry_context_reason(packet, group.traffic_light_group_id) is not None):
            low_confidence_hold.update(eligible=False, reason="route_or_packet_changed_before_handoff")
        classifier_debug["low_confidence_hold"] = low_confidence_hold
        dark_hold = no_lamps_context(packet, selection, classifier_debug, previous_head_id,
            group.traffic_light_group_id, msg.header.stamp.to_nsec(), rospy.Time.now().to_nsec(),
            self.map_packet_max_age_sec, include_soft=True)
        if (dark_hold['eligible'] and
                self._near_retry_context_reason(packet, group.traffic_light_group_id) is not None):
            dark_hold.update(eligible=False, reason='route_or_packet_changed_before_handoff')
        classifier_debug['no_lamps_hold'] = dark_hold
        self.classifier_debug_pub.publish(String(data=json.dumps(classifier_debug, allow_nan=False)))
        t_classified = time.monotonic()
        self.observed_debug_pub.publish(String(data=_raw_label_for(raw_color_results)))
        detected_color_results = list(raw_color_results)
        fixed, policy_reason = self.signal_phase_policy.apply(
            raw_color_results, group.traffic_light_group_id,
            observed_only=self.planner_compliance_observations,
        )
        if fixed != raw_color_results:
            self._n_conflict_fixed += 1
            self.status.note(
                _StatusLog.WARN,
                f"보정 {_short_label(raw_color_results)}->{_short_label(fixed)}",
            )
        raw_color_results = fixed

        self.raw_debug_pub.publish(String(data=_raw_label_for(raw_color_results)))

        stamped_vote_debug = {'enabled': False}
        if getattr(self, '_stamped_vote', None) is not None:
            # Vote CURRENT observed-only normalized frames, never the repeated
            # output of production's detector/CNN-gap holds. A retained head
            # without a current crop is identity loss for this experiment.
            identity_valid = (best is not None and selection.get('status') == 'matched'
                and selection.get('reason') in ('map_head_association', 'visual_continuation_match')
                and group.traffic_light_group_id in MISSION_GROUP_IDS
                and self._near_retry_context_reason(packet, group.traffic_light_group_id) is None)
            voted = self._stamped_vote.update(raw_color_results, normalized_reason=policy_reason,
                group_id=group.traffic_light_group_id, head_id=selection.get('head_id'),
                source_stamp_ns=msg.header.stamp.to_nsec(), now_ns=rospy.Time.now().to_nsec(),
                confidence=raw_det_conf, identity_valid=identity_valid)
            color_results, det_conf, transition_reason = voted['lamps'], voted['confidence'], voted['reason']
            observation_timing, stamped_vote_debug = voted['observation_timing'], voted['debug']
        else:
            color_results, det_conf, transition_reason = self.signal_phase_policy.process_normalized(
                raw_color_results, policy_reason, group.traffic_light_group_id,
                (rospy.Time.now().to_sec() if self.planner_compliance_observations else now.to_sec()),
                raw_det_conf, self.signal_smoother,
                source_stamp_ns=msg.header.stamp.to_nsec(),
                use_smoothing=not self.planner_compliance_observations,
                observation_head_id=selection.get("head_id") or hold_context.get("head_id"),
                detection_miss_hold_eligible=hold_context["eligible"],
                detection_miss_hold_sec=getattr(self, "detection_miss_hold_sec", .2),
                low_confidence_hold_eligible=low_confidence_hold["eligible"],
                low_confidence_hold_sec=getattr(self, "low_confidence_hold_sec", .3),
                low_confidence_confident_red=low_confidence_hold.get("confident_red", False),
                no_lamps_hold_eligible=dark_hold['eligible'],
                no_lamps_hold_sec=getattr(self, 'no_lamps_hold_sec', 0.),
                cnn_gap_confident_lamps=dark_hold.get('confident_lamps', ()),
            )
            observation_timing = self.signal_phase_policy.output_observation_metadata(group.traffic_light_group_id)
        adopted_ns = observation_timing.get("adopted_observation_source_stamp_ns")
        if observation_timing.get("output_is_observed"):
            # A held/smoothed winner is not a new observation of this JPEG.
            # Preserve integer ns; from_sec(float) would lose epoch precision.
            output_stamp = (msg.header.stamp if adopted_ns == msg.header.stamp.to_nsec() else
                            rospy.Time(adopted_ns // 1000000000, adopted_ns % 1000000000)
                            if adopted_ns is not None else rospy.Time(0))
        else:
            # Synthetic fallback RED is a fresh policy decision, not lamp GT.
            output_stamp = source_stamp
        group_array.stamp = builtin_time_from_ros(output_stamp)
        output_now_ns = rospy.Time.now().to_nsec()
        last_trusted_ns = observation_timing.get("last_trusted_observation_source_stamp_ns")
        observation_timing.update(
            output_source_stamp_ns=output_stamp.to_nsec(),
            output_observation_age_sec=((output_now_ns-adopted_ns)/1e9 if adopted_ns is not None else None),
            last_trusted_observation_age_sec=((output_now_ns-last_trusted_ns)/1e9
                                             if last_trusted_ns is not None else None),
            output_stamp_basis=("adopted_lamp_observation" if observation_timing.get("output_is_observed")
                                else "current_image_stamped_vote_unknown" if stamped_vote_debug['enabled']
                                else "current_image_policy_fallback"))
        if policy_reason.endswith(":fail_open"):
            self.status.note(_StatusLog.WARN, "검출 없음 -> fallback")
        elif transition_reason.startswith("observation_guard"):
            self.status.note(_StatusLog.WARN, "관측 가드 만료")
        self.policy_debug_pub.publish(String(data=f"{policy_reason}|{transition_reason}"))

        wire_unreadable_color_evidence = []
        if (getattr(self, 'preserve_unreadable_color_evidence', False)
                and self.planner_compliance_observations and not raw_color_results and not color_results
                and best is not None and selection.get('status') == 'matched'
                and selection.get('reason') in ('map_head_association', 'visual_continuation_match')
                and group.traffic_light_group_id in MISSION_GROUP_IDS
                and self._near_retry_context_reason(packet, group.traffic_light_group_id) is None):
            wire_unreadable_color_evidence = unreadable_color_evidence(
                detected_color_results, classifier_debug, raw_det_conf)
        labels_for_debug = []
        for color, shape, is_left in color_results:
            elem = TrafficLightElement()
            elem.color = COLOR_MAP.get(color, TrafficLightElement.UNKNOWN)
            if shape == "circle":
                elem.shape = TrafficLightElement.CIRCLE
                labels_for_debug.append(color)
            elif shape == "arrow":
                if is_left is False:
                    elem.shape = TrafficLightElement.RIGHT_ARROW
                    labels_for_debug.append(f"{color}_right")
                else:
                    elem.shape = TrafficLightElement.LEFT_ARROW
                    labels_for_debug.append(f"{color}_left")
            else:
                elem.shape = TrafficLightElement.CIRCLE
                labels_for_debug.append(color)
            elem.status = TrafficLightElement.SOLID_ON
            elem.confidence = det_conf
            group.elements.append(elem)
        for evidence in wire_unreadable_color_evidence:
            elem = TrafficLightElement()
            elem.color = COLOR_MAP[evidence['color']]
            elem.shape = TrafficLightElement.UNKNOWN
            elem.status = TrafficLightElement.UNKNOWN
            elem.confidence = evidence['confidence']
            group.elements.append(elem)
        if labels_for_debug:
            label_text = "+".join(labels_for_debug)
        # 디버그 영상은 '인식 결과'를 보여준다. 박스 색 = 실제로 검출한 램프 색,
        # 텍스트 = 인식->최종판정(정책 보정 포함)을 병기, 텍스트 색 = 최종 판정 색.
        detected_text = _short_label(detected_color_results)
        decided_text = _short_label(color_results)
        label_text = decided_text if detected_text == decided_text else f"{detected_text}->{decided_text}"
        draw_color_name = _pick_draw_color(detected_color_results)
        decision_color_name = _pick_draw_color(color_results)

        group_array.traffic_light_groups.append(group)

        self.pub.publish(group_array)
        t_published = time.monotonic()
        self._publish_runtime_debug(msg.header.stamp, frame_timing={
            "clock": "monotonic_callback_wall_not_CUDA_event_time",
            "decode_sec": t_decoded-t_monotonic,
            "detector_and_box_readback_sec": t_detected-t_decoded,
            "selection_including_map_wait_sec": t_selected-t_detected,
            "near_rescue_detector_and_association_sec": rescue_timing_sec,
            "crop_and_classifier_sec": t_classified-t_selected,
            "policy_and_publish_sec": t_published-t_classified,
            "callback_to_output_sec": t_published-t_monotonic,
            "header_to_callback_sec": (now-msg.header.stamp).to_sec(),
            "includes_debug_image_encoding": False}, frame_decision={
            "group_id": group.traffic_light_group_id,
            "head_id": selection.get("head_id"),
            "selection_status": selection.get("status"),
            "near_rescue": rescue_debug,
            "detection_miss_hold": hold_context,
            "low_confidence_hold": low_confidence_hold,
            "no_lamps_hold": dark_hold,
            "visual_continuation": visual_debug,
            "classifier_known": classifier_debug.get("known"),
            "observed_tuples": detected_color_results,
            "normalized_tuples": raw_color_results,
            "final_tuples": color_results,
            "final_confidence": det_conf,
            "planner_compliance_observations": self.planner_compliance_observations,
            "preserve_unreadable_color_evidence": getattr(self, 'preserve_unreadable_color_evidence', False),
            "wire_unreadable_color_evidence": wire_unreadable_color_evidence,
            "raw_observation_source_stamp_ns": msg.header.stamp.to_nsec(),
            "raw_observation_confidence": raw_det_conf,
            "stamped_vote": stamped_vote_debug,
            "observation_timing": observation_timing,
            "policy_reason": policy_reason,
            "transition_reason": transition_reason})

        if self.debug_pub is not None and debug_img is not None:
            try:
                # Diagnostic rendering/encoding is opt-in and AFTER canonical
                # publishing. It cannot edit the raw detector/classifier input.
                overlay = render_node_overlay(debug_img, boxes,
                    classifier_debug.get("detector_bbox_xyxy"), box_coords, {
                        "group_id": group.traffic_light_group_id,
                        "head_id": selection.get("head_id"),
                        "classifier_mode": self.classifier_mode,
                        "classifier_known": classifier_debug.get("known"),
                        "observed_lamps": _short_label(detected_color_results),
                        "normalized_lamps": _short_label(raw_color_results),
                        "final_lamps": _short_label(color_results),
                        "confidence": det_conf,
                        "selection_status": selection.get("status"),
                        "selection_reason": selection.get("reason"),
                        "source_stamp_ns": msg.header.stamp.to_nsec(),
                        "image_age_sec": (rospy.Time.now().to_nsec()-msg.header.stamp.to_nsec())/1e9,
                        "observation_timing": observation_timing,
                        "near_retry": ("accepted" if rescue_debug.get("accepted") else
                                       "attempted" if rescue_debug.get("attempted") else "OFF"),
                        "policy_reason": policy_reason,
                        "transition_reason": transition_reason,
                        "planner_parameters": self._debug_planner_parameters(),
                    })
                ok, enc = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, self.debug_jpeg_quality])
                if ok:
                    debug_msg = CompressedImage()
                    # This image still depicts the ORIGINAL JPEG, not the time
                    # at which its overlay finished encoding.
                    debug_msg.header = msg.header
                    debug_msg.format = "jpeg"
                    debug_msg.data = enc.tobytes()
                    self.debug_pub.publish(debug_msg)
            except Exception as error:
                rospy.logwarn_throttle(5., "[Traffic] debug overlay failed (standard output already published): %s", type(error).__name__)

        proc_time = time.time() - t_start
        self._proc_time_sum += proc_time
        self._proc_time_max = max(self._proc_time_max, proc_time)
        self._log_hz_if_needed()
        self.status.tick(
            time.time(),
            self.signal_phase_policy.section_label(group.traffic_light_group_id),
            _short_label(color_results),
        )

    @staticmethod
    def _read_detector_boxes(results, width, height):
        boxes = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            confidence = float(box.conf[0])
            boxes.append((((x1+x2)/2)/width, ((y1+y2)/2)/height,
                          (x2-x1)/width, (y2-y1)/height,
                          (x1, y1, x2, y2, confidence)))
        return boxes

    def _debug_planner_parameters(self):
        """Read-only cached settings, NOT an inferred live planner GO decision."""
        clock = time.monotonic()
        cached = getattr(self, "_debug_planner_config_cache", None)
        last = getattr(self, "_debug_planner_config_clock", None)
        if cached is None or last is None or clock-last >= 2. or clock < last:
            prefix = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light/"
            get_param = getattr(rospy, "get_param", lambda _key, default: default)
            cached = {key: get_param(prefix+key, None) for key in (
                "enable_signal_compliance_guard", "enable_yellow_passthrough", "yellow_light_is_go",
                "enable_arrow_aware_yellow_passing", "tl_state_timeout",
                "signal_compliance_enable_timed_yellow", "signal_compliance_min_yellow_sec",
                "signal_compliance_yellow_time_margin_sec", "signal_compliance_required_green_observations")}
            self._debug_planner_config_cache = cached
            self._debug_planner_config_clock = clock
        return dict(cached)

    def _near_retry_context_reason(self, packet, group_id):
        """Discard a delayed retry if route/freshness changed during inference."""
        if self._resolve_group_id() != group_id:
            return "route_changed_during_retry"
        try:
            age = (rospy.Time.now().to_nsec() - int(packet["generated_stamp_ns"])) / 1e9
            if not 0 <= age <= getattr(self, "map_packet_max_age_sec", .5):
                return "map_packet_expired_during_retry"
        except (KeyError, TypeError, ValueError, OverflowError):
            return "invalid_retry_packet_generation_stamp"
        return None


def main():
    rospy.init_node("traffic_light_node")
    TrafficLightNode()
    rospy.spin()


if __name__ == "__main__":
    main()
