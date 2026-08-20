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
    로거(raw_streak_analyzer.py)로 구독해서, raw 라벨이 몇 프레임 연속으로
    유지되는지 실측 가능하게 함.
"""
import os
import sys
import time as _time_module

_user_site = os.path.expanduser(f"~/.local/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages")
if os.path.isdir(_user_site) and _user_site not in sys.path:
    sys.path.insert(0, _user_site)
_this_dir = os.path.dirname(os.path.abspath(__file__))
if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)

from light_color_utils import classify_crop, pick_ego_lane_box
from lanelet_route_matcher import LaneletRouteGroupIdMatcher
from signal_smoother import SignalSmoother
from signal_conflict_resolver import has_impossible_conflict, resolve_conflict

COLOR_MAP = {
    "red": TrafficLightElement.RED,
    "yellow": TrafficLightElement.AMBER,
    "green": TrafficLightElement.GREEN,
}

DEBUG_DRAW_COLOR = {
    "red": (0, 0, 255),
    "yellow": (0, 220, 255),
    "green": (0, 255, 0),
    "off": (255, 255, 255),
    "mixed": (255, 0, 255),
}

HZ_LOG_INTERVAL = 30

EXCLUDED_MAP_ERROR_IDS = {201395, 201396}


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


class TrafficLightNode:
    def __init__(self):
        weights_path = rospy.get_param("~weights_path")
        image_topic = rospy.get_param("~image_topic", "/sensor/camera/front/compressed")
        self.conf_threshold = rospy.get_param("~conf", 0.25)
        self.center_tolerance_ratio = rospy.get_param("~center_tolerance_ratio", 0.20)
        self.y_weight = rospy.get_param("~y_weight", 0.6)
        self.fallback_group_id = rospy.get_param("~fallback_group_id", 0)
        self.publish_rate_limit_hz = rospy.get_param("~max_rate_hz", 10.0)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.debug_jpeg_quality = rospy.get_param("~debug_jpeg_quality", 60)

        self.debug_save_crop_dir = rospy.get_param("~debug_save_crop_dir", "")
        self._save_crop_count = 0
        if self.debug_save_crop_dir:
            os.makedirs(self.debug_save_crop_dir, exist_ok=True)
            rospy.loginfo(
                f"[traffic_light_node] 무효 조합 타겟 크롭 저장 활성화: {self.debug_save_crop_dir}"
            )

        smoothing_window_size = int(rospy.get_param("~smoothing_window_size", 5))
        smoothing_majority_count = rospy.get_param("~smoothing_majority_count", None)
        if smoothing_majority_count is not None:
            smoothing_majority_count = int(smoothing_majority_count)
        self.signal_smoother = SignalSmoother(
            window_size=smoothing_window_size,
            majority_count=smoothing_majority_count,
        )
        rospy.loginfo(
            f"[traffic_light_node] 신호 스무딩: window={smoothing_window_size}, "
            f"majority={self.signal_smoother.majority_count}"
        )

        lanelet2_map_file = rospy.get_param("~lanelet2_map_file", "")
        lanelet2_projector_file = rospy.get_param("~lanelet2_projector_file", "")
        lanelet2_projector_type = rospy.get_param("~lanelet2_projector_type", "LocalCartesianUTM")
        route_topic = rospy.get_param("~route_topic", "/planning/mission_planning/route")
        odom_topic = rospy.get_param("~odometry_topic", "/localization/kinematic_state")

        if not os.path.exists(weights_path):
            rospy.logerr(f"[traffic_light_node] 가중치 파일 없음: {weights_path}")

        rospy.loginfo(f"[traffic_light_node] YOLO 가중치 로드: {weights_path}")
        self.model = YOLO(weights_path)

        if not lanelet2_map_file:
            rospy.logwarn(
                "[traffic_light_node] ~lanelet2_map_file 파라미터 없음 "
                f"-> route 기반 매칭 비활성화, group_id는 항상 {self.fallback_group_id}"
            )

        self.route_matcher = LaneletRouteGroupIdMatcher(
            map_file=lanelet2_map_file,
            projector_type=lanelet2_projector_type,
            projector_file=lanelet2_projector_file,
            excluded_group_ids=EXCLUDED_MAP_ERROR_IDS,
        )
        self.route_matcher.start_subscribers(route_topic, odom_topic)

        self.pub = rospy.Publisher(
            "/perception/traffic_light_recognition/traffic_signals",
            TrafficLightGroupArray,
            queue_size=1,
        )
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(
                "~debug_image/compressed", CompressedImage, queue_size=1
            )

        self.raw_debug_pub = rospy.Publisher("~debug_raw_label", String, queue_size=10)

        self._last_stamp = rospy.Time(0)
        self._min_period = rospy.Duration(1.0 / self.publish_rate_limit_hz) if self.publish_rate_limit_hz > 0 else rospy.Duration(0)

        self._n_processed = 0
        self._proc_time_sum = 0.0
        self._proc_time_max = 0.0
        self._hz_window_start = time.time()
        self._n_conflict_fixed = 0

        self.sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(
            f"[traffic_light_node] 구독: {image_topic} (CompressedImage) / "
            f"발행: /perception/traffic_light_recognition/traffic_signals, ~debug_raw_label"
            + (" / 디버그 이미지: ~debug_image/compressed" if self.publish_debug_image else "")
        )

    def _resolve_group_id(self):
        gid = self.route_matcher.get_group_id()
        return gid if gid is not None else self.fallback_group_id

    def _log_hz_if_needed(self):
        self._n_processed += 1
        if self._n_processed % HZ_LOG_INTERVAL == 0:
            elapsed = time.time() - self._hz_window_start
            hz = HZ_LOG_INTERVAL / elapsed if elapsed > 0 else 0.0
            avg_ms = (self._proc_time_sum / HZ_LOG_INTERVAL) * 1000.0
            max_ms = self._proc_time_max * 1000.0
            rospy.loginfo(
                f"[traffic_light_node] 실시간성: 최근 {HZ_LOG_INTERVAL}프레임 "
                f"평균 {hz:.1f} Hz, 프레임당 처리시간 평균 {avg_ms:.1f}ms / 최대 {max_ms:.1f}ms"
            )
            self._hz_window_start = time.time()
            self._proc_time_sum = 0.0
            self._proc_time_max = 0.0

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

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("cv2.imdecode 실패 (None)")
        except Exception as e:
            rospy.logwarn(f"[traffic_light_node] 이미지 디코딩 실패: {e}")
            return

        h, w = img.shape[:2]

        results = self.model.predict(source=img, conf=self.conf_threshold, verbose=False)
        r = results[0]

        boxes = []
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            det_conf = float(box.conf[0])
            cx = ((x1 + x2) / 2) / w
            cy = ((y1 + y2) / 2) / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            boxes.append((cx, cy, bw, bh, (x1, y1, x2, y2, det_conf)))

        group_array = TrafficLightGroupArray()
        group_array.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else now

        debug_img = img if self.publish_debug_image else None

        group = TrafficLightGroup()
        group.traffic_light_group_id = self._resolve_group_id()

        best = pick_ego_lane_box(boxes, self.center_tolerance_ratio, self.y_weight)
        label_text = "off"
        draw_color_name = "off"
        box_coords = None
        det_conf = 0.0

        raw_color_results = []
        raw_det_conf = 0.0

        if best is not None:
            x1, y1, x2, y2, det_conf = best[4]
            x1c, y1c = max(0, int(x1)), max(0, int(y1))
            x2c, y2c = min(w, int(x2)), min(h, int(y2))
            crop = img[y1c:y2c, x1c:x2c]
            box_coords = (x1c, y1c, x2c, y2c)
            raw_det_conf = det_conf

            if crop.size > 0:
                raw_color_results, _counts = classify_crop(crop)

                if has_impossible_conflict(raw_color_results):
                    self._maybe_save_conflict_crop(crop, raw_color_results)

        if has_impossible_conflict(raw_color_results):
            fixed = resolve_conflict(raw_color_results)
            if fixed != raw_color_results:
                self._n_conflict_fixed += 1
                rospy.logwarn_throttle(
                    2.0,
                    f"[traffic_light_node] 물리적으로 불가능한 신호 조합 보정(누적 {self._n_conflict_fixed}회): "
                    f"{raw_color_results} -> {fixed}",
                )
            raw_color_results = fixed

        self.raw_debug_pub.publish(String(data=_raw_label_for(raw_color_results)))

        smoothed = self.signal_smoother.update(
            raw_color_results, payload=(raw_color_results, raw_det_conf)
        )
        color_results, det_conf = smoothed if smoothed is not None else ([], 0.0)

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
        if labels_for_debug:
            label_text = "+".join(labels_for_debug)
        draw_color_name = _pick_draw_color(color_results)

        group_array.traffic_light_groups.append(group)

        if debug_img is not None and box_coords is not None:
            x1c, y1c, x2c, y2c = box_coords
            box_color = DEBUG_DRAW_COLOR[draw_color_name]
            cv2.rectangle(debug_img, (x1c, y1c), (x2c, y2c), box_color, 2)
            cv2.putText(
                debug_img,
                f"{label_text} ({det_conf:.2f}) gid={group.traffic_light_group_id}",
                (x1c, max(0, y1c - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2, cv2.LINE_AA,
            )

        self.pub.publish(group_array)

        if self.debug_pub is not None and debug_img is not None:
            ok, enc = cv2.imencode(".jpg", debug_img, [cv2.IMWRITE_JPEG_QUALITY, self.debug_jpeg_quality])
            if ok:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = now
                debug_msg.format = "jpeg"
                debug_msg.data = enc.tobytes()
                self.debug_pub.publish(debug_msg)

        proc_time = time.time() - t_start
        self._proc_time_sum += proc_time
        self._proc_time_max = max(self._proc_time_max, proc_time)
        self._log_hz_if_needed()


def main():
    rospy.init_node("traffic_light_node")
    TrafficLightNode()
    rospy.spin()


if __name__ == "__main__":
    main()
