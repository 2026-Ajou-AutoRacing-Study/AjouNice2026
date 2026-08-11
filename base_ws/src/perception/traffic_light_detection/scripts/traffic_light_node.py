#!/usr/bin/env python3
"""
traffic_light_node.py (v2 - group_id 동적 매칭 추가)

카메라 이미지 -> YOLO26(신호등 위치 탐지) -> 우리 차로 필터링 -> 색상/모양 분류
-> autoware_perception_msgs/TrafficLightGroupArray 로 발행.

v1 대비 변경점:
  - traffic_light_group_id 고정값(기본 0) 대신, /Ego_topic(차량 현재 위치)을 구독해서
    build_traffic_light_group_id_table.py로 미리 생성한 테이블(traffic_light_group_id_table.json)에서
    가장 가까운 lanelet2 regulatory element id를 찾아 동적으로 채움.
  - 가장 가까운 신호등까지 거리가 ~max_group_id_match_dist(기본 80m)보다 크면
    "지금 어떤 신호등 구역에도 속하지 않는다"고 판단, 매칭 실패로 처리하고
    폴백값(~fallback_group_id, 기본 0)을 사용.
  - 테이블 로드/매칭 실패 시 항상 안전하게 폴백값으로 동작(노드가 죽지 않음).

토픽:
  구독: /sensor/camera/front/compressed (sensor_msgs/CompressedImage)
  구독: /Ego_topic (morai_msgs/EgoVehicleStatus) - group_id 매칭용 차량 위치
  발행: /perception/traffic_light_recognition/traffic_signals (autoware_perception_msgs/TrafficLightGroupArray)
  발행(디버그): ~debug_image/compressed (sensor_msgs/CompressedImage)

주의(팀 확인 필요, 2026-08-11):
  - 룩업 테이블은 lanelet2 좌표 - (mgeo 원점 - lanelet2 원점) 오프셋으로 계산됨.
    Ego_topic이 mgeo local 좌표계와 동일한 프레임을 쓴다는 가정 하에 만들어졌고,
    아직 실측으로 이 가정을 완전히 검증하지 않음 (구간별 정지선 Ego 좌표와
    테이블 매칭 거리로 검증 권장, build_traffic_light_group_id_table.py 참고).
  - id 201395, 201396은 lanelet2 맵 자체 데이터 오류(전구 위치가 도로 밖에 찍힘)로
    확인되어 테이블에서 제외됨. 맵 수정 전까지 이 두 신호등 근처에서는 매칭이
    안 되고 폴백값이 나갈 수 있음.
  - 화살표 방향(LEFT_ARROW/RIGHT_ARROW)은 신뢰도가 낮을 때(면적 작음) 방향을 못 정함.
    K-City 샘플 시나리오는 좌회전 신호만 있는 걸로 조사돼 있어, 방향 불명 화살표는
    기본적으로 LEFT_ARROW로 채움.
"""
import json
import math
import os
import sys

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
from morai_msgs.msg import EgoVehicleStatus
from ultralytics import YOLO

from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)

from light_color_utils import classify_crop, pick_ego_lane_box


COLOR_MAP = {
    "red": TrafficLightElement.RED,
    "yellow": TrafficLightElement.AMBER,
    "green": TrafficLightElement.GREEN,
}

HZ_LOG_INTERVAL = 30


class TrafficLightGroupIdMatcher:
    """차량 위치 기반 가장 가까운 lanelet2 신호등 group_id 매칭."""

    def __init__(self, table_path, max_match_dist, fallback_group_id):
        self.max_match_dist = max_match_dist
        self.fallback_group_id = fallback_group_id
        self.entries = []  # [(group_id, x, y), ...]
        self.loaded = False

        if not os.path.exists(table_path):
            rospy.logwarn(
                f"[traffic_light_node] group_id 테이블 없음: {table_path} "
                f"-> 항상 고정값({fallback_group_id}) 사용"
            )
            return

        try:
            with open(table_path) as f:
                data = json.load(f)
            self.entries = [
                (e["group_id"], e["ego_frame_x"], e["ego_frame_y"])
                for e in data.get("traffic_lights", [])
            ]
            self.loaded = len(self.entries) > 0
            rospy.loginfo(f"[traffic_light_node] group_id 테이블 로드: {len(self.entries)}개 신호등")
        except Exception as e:
            rospy.logwarn(f"[traffic_light_node] group_id 테이블 로드 실패: {e} -> 고정값 사용")

        self._latest_ego_xy = None

    def update_ego_position(self, x, y):
        self._latest_ego_xy = (x, y)

    def get_group_id(self):
        """현재 ego 위치 기준 가장 가까운 group_id 반환. 매칭 실패 시 fallback."""
        if not self.loaded or self._latest_ego_xy is None:
            return self.fallback_group_id

        ex, ey = self._latest_ego_xy
        best_id, best_dist = None, None
        for gid, x, y in self.entries:
            d = math.hypot(ex - x, ey - y)
            if best_dist is None or d < best_dist:
                best_dist, best_id = d, gid

        if best_id is not None and best_dist <= self.max_match_dist:
            return best_id
        return self.fallback_group_id


class TrafficLightNode:
    def __init__(self):
        weights_path = rospy.get_param("~weights_path")
        image_topic = rospy.get_param("~image_topic", "/sensor/camera/front/compressed")
        ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.conf_threshold = rospy.get_param("~conf", 0.25)
        self.center_tolerance_ratio = rospy.get_param("~center_tolerance_ratio", 0.20)
        self.y_weight = rospy.get_param("~y_weight", 0.6)
        self.fallback_group_id = rospy.get_param("~fallback_group_id", 0)
        self.max_group_id_match_dist = rospy.get_param("~max_group_id_match_dist", 80.0)
        table_path = rospy.get_param(
            "~group_id_table_path",
            os.path.join(_this_dir, "..", "traffic_light_group_id_table.json"),
        )
        self.publish_rate_limit_hz = rospy.get_param("~max_rate_hz", 10.0)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.debug_jpeg_quality = rospy.get_param("~debug_jpeg_quality", 60)

        if not os.path.exists(weights_path):
            rospy.logerr(f"[traffic_light_node] 가중치 파일 없음: {weights_path}")

        rospy.loginfo(f"[traffic_light_node] YOLO 가중치 로드: {weights_path}")
        self.model = YOLO(weights_path)

        self.group_id_matcher = TrafficLightGroupIdMatcher(
            table_path, self.max_group_id_match_dist, self.fallback_group_id
        )

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

        self._last_stamp = rospy.Time(0)
        self._min_period = rospy.Duration(1.0 / self.publish_rate_limit_hz) if self.publish_rate_limit_hz > 0 else rospy.Duration(0)

        self._n_processed = 0
        self._proc_time_sum = 0.0
        self._proc_time_max = 0.0
        self._hz_window_start = time.time()

        self.sub_ego = rospy.Subscriber(ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        self.sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo(
            f"[traffic_light_node] 구독: {image_topic} (CompressedImage), {ego_topic} (EgoVehicleStatus) / "
            f"발행: /perception/traffic_light_recognition/traffic_signals"
            + (" / 디버그 이미지: ~debug_image/compressed" if self.publish_debug_image else "")
        )

    def ego_callback(self, msg):
        self.group_id_matcher.update_ego_position(msg.position.x, msg.position.y)

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

        best = pick_ego_lane_box(boxes, self.center_tolerance_ratio, self.y_weight)
        if best is not None:
            x1, y1, x2, y2, det_conf = best[4]
            x1c, y1c = max(0, int(x1)), max(0, int(y1))
            x2c, y2c = min(w, int(x2)), min(h, int(y2))
            crop = img[y1c:y2c, x1c:x2c]

            group = TrafficLightGroup()
            group.traffic_light_group_id = self.group_id_matcher.get_group_id()

            label_text = "off"
            if crop.size > 0:
                color_results, _counts = classify_crop(crop)
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

            group_array.traffic_light_groups.append(group)

            if debug_img is not None:
                cv2.rectangle(debug_img, (x1c, y1c), (x2c, y2c), (0, 255, 255), 2)
                cv2.putText(
                    debug_img, f"{label_text} ({det_conf:.2f}) gid={group.traffic_light_group_id}",
                    (x1c, max(0, y1c - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
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
