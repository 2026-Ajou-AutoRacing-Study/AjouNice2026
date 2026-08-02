#!/usr/bin/env python3
"""
ekf_tracking_node.py

jaeyoungjo99/ekf-multi-object-tracking(실제 EKF 기반 트래커)의 출력을
tracking_node.py(GT)/pointpainting_tracking_node.py와 동일한 인터페이스
(/tracking/objects, TrackedObjectArray)로 변환하는 브릿지.

배경 (2026-07-30 소스 직접 확인 결과, ekf_multi_object_tracking_node.cpp):
    이 트래커는 실제 데이터연관 기반 EKF 트래킹(진짜 지속되는 track_id)과
    저속 검출 -> 고속 출력(config.yaml output_period_lidar: false)을 모두
    제공하지만, 메인 출력 토픽(jsk_all_track, jsk_recognition_msgs/
    BoundingBoxArray)에는 position/heading/dimensions와 velocity "크기"
    (box.value)만 있고 track_id/yaw_rate/acceleration은 없다
    (jsk_recognition_msgs::BoundingBox 자체에 그런 필드가 없고, label은
    이미 class로 쓰이고 있음).

    다만 같은 노드가 함께 발행하는 시각화용 MarkerArray(vis_all_track)의
    ns=="track_info" 마커에는:
      - marker.id 가 그대로 진짜 track_id (문자열 파싱 불필요)
      - marker.pose.position.x/y 가 jsk box와 완전히 같은 값
        (소스 확인: 둘 다 track.state_vec(S_X)/state_vec(S_Y)에서 그대로 옴)
      - marker.text 안에 "YawVel : X deg/s", "Accel : X m/s^2" 형태로
        EKF가 이미 추정한 yaw_rate/가속도 크기가 들어있음
    이걸 이용해서 box<->marker를 (x, y) 근접 매칭하면, 정규식 파싱만으로
    실제 track_id/yaw_rate/가속도를 복원할 수 있다.

    (C++ 소스를 고쳐서 이 값들을 별도 토픽으로 깔끔하게 내보내는 게 정석이지만,
    지금 빌드 환경이 막혀 있어 소스 수정 없이 붙일 수 있는 이 방법을 우선 택함.
    나중에 빌드 가능해지면 소스에 track_id/yaw_rate 전용 필드를 추가해서
    이 정규식 파싱 의존을 없애는 걸 권장 - TODO로 남겨둠.)

주의:
    - track_info 마커와 jsk box가 "같은 publish 주기"에서 나온다는 보장이
      필요하므로, MarkerArray는 별도 콜백으로 최신 것만 캐시해두고, jsk box
      콜백에서 그 캐시가 너무 오래되지 않았을 때만(~marker_max_age) 사용한다.
      매칭 안 되면(마커가 없거나 오래됐거나 위치가 안 맞으면) track_id는
      순서 기반 임시값으로, yaw_rate/가속도는 0으로 폴백한다 - 그래도
      prediction_node.py가 자체적으로 yaw_rate를 재추정하므로 파이프라인
      자체는 안 죽지만, 그 경우 EKF 트래킹의 이득(진짜 ID, 진짜 yaw_rate)을
      못 쓰게 되므로 rospy.logwarn_throttle로 알림.
    - velocity/acceleration은 "크기"만 주어지므로 heading 방향으로 투영한다고
      가정한다 (pointpainting_tracking_node.py와 동일한 단순화).

입력:
    ~jsk_track_topic (기본 /app/perc/jsk_all_track, jsk_recognition_msgs/BoundingBoxArray)
    ~marker_topic    (기본 /app/perc/vis_all_track, visualization_msgs/MarkerArray)
출력:
    ~output_topic (기본 /tracking/objects, TrackedObjectArray)
       -> tracking_node.py(GT)/pointpainting_tracking_node.py와 완전히 동일한
          토픽/타입이므로 prediction_node.py는 수정 없이 그대로 동작한다.
"""

import math
import re

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import Marker, MarkerArray

from tracking_prediction.msg import TrackedObject, TrackedObjectArray

# ekf-multi-object-tracking 실제 소스(ekf_multi_object_tracking.hpp) 확인된 클래스 enum:
# UNKNOWN=0, CAR=1, TRUCK=2, PEDESTRIAN=3, BICYCLE=4
CLASS_LABEL_TO_NAME = {
    0: "obstacle",    # UNKNOWN
    1: "vehicle",     # CAR
    2: "vehicle",     # TRUCK
    3: "pedestrian",  # PEDESTRIAN
    4: "vehicle",     # BICYCLE (CTRV 예측 목적상 vehicle로 취급, 별도 클래스 없음)
}

# ekf_multi_object_tracking_node.cpp의 display_text 포맷 문자열에서 파싱:
# "...\nYawVel : X.XXXdeg/s\n...", "...\nAccel  : X.XXXm/s^2\n..."
YAWVEL_RE = re.compile(r"YawVel\s*:\s*(-?[\d.]+)\s*deg/s")
ACCEL_RE = re.compile(r"Accel\s*:\s*(-?[\d.]+)\s*m/s\^2")

# box<->marker 위치 매칭 허용 오차(m). 둘 다 같은 state_vec(S_X,S_Y)에서
# 그대로 오므로 이론상 완전히 같은 값이어야 함 - float 오차 대비용으로만 여유를 둠.
POSITION_MATCH_TOLERANCE = 0.05


def _quaternion_to_yaw(q):
    """geometry_msgs/Quaternion -> yaw(radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EkfTrackingBridge(object):
    def __init__(self):
        jsk_track_topic = rospy.get_param("~jsk_track_topic", "/app/perc/jsk_all_track")
        marker_topic = rospy.get_param("~marker_topic", "/app/perc/vis_all_track")
        output_topic = rospy.get_param("~output_topic", "/tracking/objects")
        # 이보다 오래된 marker 캐시는 못 믿고 폴백 (box와 marker가 다른 주기에서 왔을 위험)
        self._marker_max_age = rospy.get_param("~marker_max_age", 0.2)

        self._pub = rospy.Publisher(output_topic, TrackedObjectArray, queue_size=10)
        rospy.Subscriber(marker_topic, MarkerArray, self._marker_callback, queue_size=10)
        rospy.Subscriber(jsk_track_topic, BoundingBoxArray, self._box_callback, queue_size=10)

        # [(x, y, track_id, yaw_rate_rad, accel_mag), ...] - 최신 marker 캐시
        self._latest_track_info_markers = []
        self._latest_marker_stamp = None

        self._fallback_track_id = 0

        # gap 진단용: _box_callback이 실제로 얼마나 자주 불리는지 직접 로그.
        # jsk_all_track 자체가 끊기는지, 아니면 콜백은 정상인데 뭔가 다른 이유로
        # publish가 안 되는지 확실히 구분하기 위함(2026-07-31, /tracking/objects
        # 2~3초 gap 원인 조사).
        self._last_box_cb_time = None
        self._gap_warn_threshold = rospy.get_param("~gap_warn_threshold", 0.5)

        rospy.loginfo(
            "[ekf_tracking_node] 시작: jsk=%s, marker=%s -> %s",
            jsk_track_topic, marker_topic, output_topic,
        )

    def _marker_callback(self, msg):
        parsed = []
        for m in msg.markers:
            if m.ns != "track_info":
                continue
            if m.action == Marker.DELETE:
                continue
            yawvel_match = YAWVEL_RE.search(m.text)
            accel_match = ACCEL_RE.search(m.text)
            yaw_rate_rad = math.radians(float(yawvel_match.group(1))) if yawvel_match else 0.0
            accel_mag = float(accel_match.group(1)) if accel_match else 0.0
            parsed.append((m.pose.position.x, m.pose.position.y, m.id, yaw_rate_rad, accel_mag))

        self._latest_track_info_markers = parsed
        self._latest_marker_stamp = rospy.Time.now().to_sec()

    def _find_marker_match(self, x, y):
        if self._latest_marker_stamp is None:
            return None
        if rospy.Time.now().to_sec() - self._latest_marker_stamp > self._marker_max_age:
            return None  # marker 캐시가 너무 오래됨 - 매칭 신뢰 못 함

        best = None
        best_dist = POSITION_MATCH_TOLERANCE
        for (mx, my, track_id, yaw_rate_rad, accel_mag) in self._latest_track_info_markers:
            dist = math.hypot(x - mx, y - my)
            if dist < best_dist:
                best_dist = dist
                best = (track_id, yaw_rate_rad, accel_mag)
        return best

    def _box_callback(self, msg):
        now = rospy.Time.now().to_sec()
        if self._last_box_cb_time is not None:
            gap = now - self._last_box_cb_time
            if gap > self._gap_warn_threshold:
                rospy.logwarn(
                    "[ekf_tracking_node][GAP진단] jsk_all_track 콜백 간격 %.3fs "
                    "(sim_time=%.3f, boxes=%d) - jsk_all_track 자체가 끊겼음",
                    gap, now, len(msg.boxes),
                )
        self._last_box_cb_time = now

        out = TrackedObjectArray()
        out.header = msg.header

        objects = []
        for box in msg.boxes:
            x, y = box.pose.position.x, box.pose.position.y
            heading = _quaternion_to_yaw(box.pose.orientation)
            v = box.value  # 속도 "크기" (방향은 heading으로 가정)

            match = self._find_marker_match(x, y)
            if match is not None:
                track_id, yaw_rate_rad, accel_mag = match
            else:
                # marker 매칭 실패 - 임시 순서 기반 ID, yaw_rate/accel은 0으로 폴백.
                # prediction_node.py가 자체적으로 yaw_rate를 재추정하므로 파이프라인은
                # 안 죽지만, 이 경우 EKF 추정치(진짜 ID/yaw_rate) 이득을 못 씀.
                self._fallback_track_id += 1
                track_id = self._fallback_track_id
                yaw_rate_rad = 0.0
                accel_mag = 0.0
                rospy.logwarn_throttle(
                    5.0,
                    "[ekf_tracking_node] track_info marker 매칭 실패 - "
                    "임시 ID/yaw_rate=0으로 폴백 (marker_topic 발행 여부 확인 필요)",
                )

            t = TrackedObject()
            t.track_id = track_id
            t.class_label = CLASS_LABEL_TO_NAME.get(box.label, "obstacle")
            t.position.x = x
            t.position.y = y
            t.position.z = box.pose.position.z
            t.heading = heading
            t.velocity.x = v * math.cos(heading)
            t.velocity.y = v * math.sin(heading)
            t.velocity.z = 0.0
            t.acceleration.x = accel_mag * math.cos(heading)
            t.acceleration.y = accel_mag * math.sin(heading)
            t.acceleration.z = 0.0
            t.size = box.dimensions

            objects.append(t)

        out.objects = objects
        self._pub.publish(out)


def main():
    rospy.init_node("ekf_tracking_node")
    EkfTrackingBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
