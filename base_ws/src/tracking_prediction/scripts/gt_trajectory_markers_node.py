#!/usr/bin/env python3
"""
gt_trajectory_markers_node.py

MORAI GT(/Object_topic)를 직접 받아서, 각 객체(unique_id 기준, 진짜 지속 ID)의
"최근 실제 이동 경로"를 노란색 LINE_STRIP으로 그린다.

목적: /prediction/markers(파란색, CTRA 예측 궤적)와 같은 화면에 겹쳐서,
      "예측이 실제 경로랑 얼마나 다른지"를 눈으로 바로 비교하기 위함
      (변지훈님 요청: "상대차 앞에 미래 궤적 깔아주고, 실제 경로랑 비교").

주의: 이 노드는 dummy tracking을 거치지 않고 /Object_topic을 직접 구독해서
      MORAI의 진짜 unique_id로 히스토리를 쌓는다 (평가 스크립트와 같은 방식).
      따라서 dummy tracking 파이프라인과는 독립적이며, "정답이 실제로 어디로
      갔는지"를 보여주는 순수 참고용 시각화다.

출력: ~output_topic (기본 /gt/markers, visualization_msgs/MarkerArray)
"""

import math
from collections import defaultdict, deque

import rospy
from geometry_msgs.msg import Point
from morai_msgs.msg import ObjectStatusList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

GT_COLOR = ColorRGBA(1.0, 0.9, 0.1, 0.9)  # 노란색 - 예측(파란색)과 확실히 구분되게
HISTORY_SECONDS = 3.0  # 최근 몇 초 동안의 이동 경로를 그릴지
# MORAI가 사라진 NPC의 unique_id를 다른 위치의 새 차량에 재사용하는 경우가 있음
# (실측으로 확인: 같은 uid가 한 프레임 사이 최대 246m 순간이동). 이 거리 이상 튀면
# "같은 차가 아니다"로 보고 히스토리를 리셋해서 엉뚱한 직선이 그려지지 않게 함
TELEPORT_THRESHOLD_M = 15.0


class GtTrajectoryMarkersNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param("~output_topic", "/gt/markers")
        # 2026-08-01: GT 원본 rate(~50Hz) 그대로 매번 전체 차량 궤적(최대 19개 LINE_STRIP,
        # 각 최대 150점)을 다시 그려서 발행하니 RViz가 버벅인다는 피드백으로 추가.
        # 트레일은 눈으로 보기에 10Hz 정도면 충분히 부드러워서 발행만 제한함
        # (히스토리 갱신 자체는 매 메시지마다 그대로 해서 트레일 정확도는 안 떨어짐).
        self._publish_hz = rospy.get_param("~publish_hz", 10.0)
        self._last_publish_time = 0.0

        self._pub = rospy.Publisher(output_topic, MarkerArray, queue_size=10)
        rospy.Subscriber(input_topic, ObjectStatusList, self._callback, queue_size=10)

        # unique_id별 (t, x, y, z) 히스토리
        self._history = defaultdict(lambda: deque(maxlen=500))

        rospy.loginfo(
            "[gt_trajectory_markers_node] %s -> %s (publish_hz=%s)",
            input_topic, output_topic, self._publish_hz,
        )

    def _update_history(self, uid, t, x, y, z):
        hist = self._history[uid]

        if hist:
            _, px, py, pz = hist[-1]
            dist = math.hypot(x - px, y - py)
            if dist > TELEPORT_THRESHOLD_M:
                # unique_id 재사용으로 인한 순간이동으로 판단 -> 히스토리 리셋
                hist.clear()

        hist.append((t, x, y, z))
        # 오래된 기록 제거
        while hist and (t - hist[0][0]) > HISTORY_SECONDS:
            hist.popleft()

    def _callback(self, msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # 히스토리는 매 메시지마다 갱신(트레일 정확도 유지) - 마커 생성/발행만 throttle
        seen_uids = set()
        for obj in msg.npc_list:
            seen_uids.add(obj.unique_id)
            self._update_history(
                obj.unique_id, stamp, obj.position.x, obj.position.y, obj.position.z
            )

        now = rospy.Time.now().to_sec()
        if self._publish_hz > 0 and (now - self._last_publish_time) < (1.0 / self._publish_hz):
            return
        self._last_publish_time = now

        markers = MarkerArray()

        clear = Marker()
        clear.header = msg.header
        clear.ns = "gt_trajectory"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for uid in seen_uids:
            hist = self._history[uid]
            if len(hist) < 2:
                continue

            line = Marker()
            line.header = msg.header
            line.ns = "gt_trajectory"
            line.id = uid
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.15
            line.color = GT_COLOR
            line.lifetime = rospy.Duration(0.5)
            line.points = [Point(x, y, z) for (_, x, y, z) in hist]
            markers.markers.append(line)

        self._pub.publish(markers)


def main():
    rospy.init_node("gt_trajectory_markers_node")
    GtTrajectoryMarkersNode()
    rospy.spin()


if __name__ == "__main__":
    main()
