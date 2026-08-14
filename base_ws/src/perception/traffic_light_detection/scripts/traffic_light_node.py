#!/usr/bin/env python3
"""
traffic_light_node.py (v6 - 혼합 신호(red+green 등) 디버그 표시 구분)

카메라 이미지 -> YOLO26(신호등 위치 탐지) -> 우리 차로 필터링 -> 색상/모양 분류
-> autoware_perception_msgs/TrafficLightGroupArray 로 발행.

v6 -> v7 변경점 (2026-08-14, 판단팀 연동 실주행 검증 중 발견한 버그 수정):
  - 버그: group_id 발행을 "카메라가 신호등 박스를 찾았을 때"에만 했음. 그러다 보니
    route 기반으로는 이미 다음 신호등(예: D구간)으로 전환됐는데 카메라가 아직 그
    신호등을 시야에 못 담은 구간(정지선까지 60~100m 남은 정상 주행 구간)에서는
    group 자체가 통째로 안 나갔음. 판단팀은 이 경우 signal=None으로 보고,
    is_simulation=false 설정상 곧바로 "MISSING"(정지 필요)으로 오판해서 정지선
    100m 밖에서도 불필요한 정지 판단이 발생하는 게 실주행 로그로 확인됨.
  - 수정: group_id는 카메라 인식 여부와 무관하게 매 프레임 항상 발행. 카메라가
    신호등을 못 찾은 프레임에서는 elements가 빈 group을 보냄. 판단팀은 group
    자체는 찾되(signal is not None) elements가 비어있으면 정지 신호로 해석 안
    하므로, 불필요한 정지 판단이 사라짐.

v5 -> v6 변경점 (2026-08-14):
  - 디버그 박스/텍스트 색이 여러 색(예: red+green_left)이 동시에 검출됐을 때도
    무조건 우선순위(red)로만 표시되던 것을, "혼합 상태"를 별도 색(마젠타)으로
    명확히 구분하도록 변경. 단일 색만 검출된 경우는 기존처럼 그 색 그대로 표시.

v4 -> v5 변경점 (2026-08-14):
  - 디버그 이미지의 박스/텍스트 색상이 항상 고정(노란 박스+초록 텍스트)이었는데,
    실제로 인식된 신호등 색상(빨강/노랑/초록)에 맞춰 동적으로 바뀌도록 변경.

v3 -> v4 변경점 (2026-08-14, 팀장 요청: 매칭 로직을 하나로 단순화):
  - group_id 매칭을 route 기반(LaneletRouteGroupIdMatcher) 하나만 사용하도록 단순화.
    기존 v2/v3의 유클리드 최근접 폴백은 완전히 제거함.
  - route 매칭이 실패하면 group_id는 그냥 fallback_group_id(기본 0)로 나감.
  - "유효 신호등 id" 판단: lanelet_route_matcher.py가 lanelet2 맵 자체에서
    subtype=traffic_light인 regulatory element를 직접 판별함. 알려진 맵 오류
    2건(201395, 201396)만 별도로 계속 제외.

토픽:
  구독: /sensor/camera/front/compressed (sensor_msgs/CompressedImage)
  구독: /planning/mission_planning/route (autoware_planning_msgs/LaneletRoute)
  구독: /localization/kinematic_state (nav_msgs/Odometry)
  발행: /perception/traffic_light_recognition/traffic_signals (autoware_perception_msgs/TrafficLightGroupArray)
  발행(디버그): ~debug_image/compressed (sensor_msgs/CompressedImage)

주의(팀 확인 필요):
  - route 기반 매칭은 lanelet2 python bindings와 맵 파일(~lanelet2_map_file),
    projector 파일(~lanelet2_projector_file)이 반드시 필요함. 셋 중 하나라도
    없으면 route 매칭이 비활성화되고 group_id는 항상 fallback_group_id(0)로 나감.
  - id 201395, 201396은 lanelet2 맵 자체 데이터 오류(전구 위치가 도로 밖에 찍힘)로
    확인되어 매칭 대상에서 제외됨.
  - 화살표 방향(LEFT_ARROW/RIGHT_ARROW)은 신뢰도가 낮을 때(면적 작음) 방향을 못 정함.
    K-City 샘플 시나리오는 좌회전 신호만 있는 걸로 조사돼 있어, 방향 불명 화살표는
    기본적으로 LEFT_ARROW로 채움.
"""
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
from ultralytics import YOLO

from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)

from light_color_utils import classify_crop, pick_ego_lane_box
from lanelet_route_matcher import LaneletRouteGroupIdMatcher

COLOR_MAP = {
    "red": TrafficLightElement.RED,
    "yellow": TrafficLightElement.AMBER,
    "green": TrafficLightElement.GREEN,
}

# 디버그 박스/텍스트에 쓸 BGR 색상 (OpenCV는 BGR 순서)
DEBUG_DRAW_COLOR = {
    "red": (0, 0, 255),
    "yellow": (0, 220, 255),
    "green": (0, 255, 0),
    "off": (255, 255, 255),
    "mixed": (255, 0, 255),  # 마젠타 - 두 가지 이상 색이 동시에 검출된 "혼합" 상태 표시
}


HZ_LOG_INTERVAL = 30


def _pick_draw_color(color_results):
    """color_results: [(color, shape, is_left), ...] -> 대표로 그릴 색 이름.
    단일 색만 검출되면 그 색 그대로, 두 가지 이상 색이 동시에 검출되면 'mixed'."""
    present = {c for c, _shape, _is_left in color_results}
    if len(present) == 0:
        return "off"
    if len(present) == 1:
        return next(iter(present))
    return "mixed"


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

        # --- route 기반 group_id 매칭 설정 (유일한 매칭 경로) ---
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
            excluded_group_ids={201395, 201396},
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

        self._last_stamp = rospy.Time(0)
        self._min_period = rospy.Duration(1.0 / self.publish_rate_limit_hz) if self.publish_rate_limit_hz > 0 else rospy.Duration(0)

        self._n_processed = 0
        self._proc_time_sum = 0.0
        self._proc_time_max = 0.0
        self._hz_window_start = time.time()

        self.sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(
            f"[traffic_light_node] 구독: {image_topic} (CompressedImage) / "
            f"발행: /perception/traffic_light_recognition/traffic_signals"
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

        # group_id는 카메라 인식 여부와 무관하게 항상 발행한다.
        # (route 기반으로 "지금 봐야 할 신호등"이 정해져 있으면, 설령 카메라가 아직
        #  그 신호등을 못 찾았어도 elements가 빈 group을 계속 보내야 판단팀이
        #  "신호 데이터 자체를 못 받음(MISSING)"으로 오판해서 불필요하게 정지하는
        #  것을 막을 수 있다. 판단팀은 group을 못 찾을 때만 MISSING으로 처리하고,
        #  is_simulation=false라 MISSING은 곧바로 정지 판단으로 이어지기 때문에
        #  이 부분이 중요함 - 2026-08-14 판단팀 연동 실주행 검증 중 발견)
        group = TrafficLightGroup()
        group.traffic_light_group_id = self._resolve_group_id()

        best = pick_ego_lane_box(boxes, self.center_tolerance_ratio, self.y_weight)
        label_text = "off"
        draw_color_name = "off"
        box_coords = None
        det_conf = 0.0

        if best is not None:
            x1, y1, x2, y2, det_conf = best[4]
            x1c, y1c = max(0, int(x1)), max(0, int(y1))
            x2c, y2c = min(w, int(x2)), min(h, int(y2))
            crop = img[y1c:y2c, x1c:x2c]
            box_coords = (x1c, y1c, x2c, y2c)

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
                            # is_left True 또는 None(불확실) -> K-City 샘플 시나리오는
                            # 좌회전 신호만 있는 걸로 조사됨 -> 기본 LEFT_ARROW
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
