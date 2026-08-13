#!/usr/bin/env python3
"""
future_rollout_node.py

jaeyoungjo99/ekf-multi-object-tracking(CTRV, prediction_model=1)의 출력에는
현재 필터링된 상태(위치/속도/yaw/yaw_rate)만 있고, 미래 3초 궤적은 없다.
이 노드는 그 상태를 복사해서 CTRV로 0.1초씩 반복 전개(rollout)해서
PredictedPathArray 역할을 하는 MarkerArray(파란 궤적)를 만든다.

입력 파싱 방식:
    /app/perc/vis_all_track (MarkerArray) 안에서 ns=="track_info",
    type==TEXT_VIEW_FACING인 마커만 사용한다. 이 마커 하나에:
        - marker.id            -> track_id
        - marker.pose.position -> 현재 (x, y)  (z는 텍스트 표시용 +3.0 오프셋이라 무시)
        - marker.pose.orientation -> yaw
        - marker.text 안의 "Vel", "YawVel" 문자열 -> 속도(m/s), yaw_rate(deg/s)
    를 전부 담고 있어서, BoundingBoxArray와 별도로 대조할 필요가 없다
    (ekf_multi_object_tracking_node.cpp 소스 확인: vis_track_info.id = track.track_id,
     vis_track_info.pose.position = state_vec(S_X,S_Y), text 포맷 고정).

CTRV 롤아웃 (가속도 사용 안 함, prediction_model=1과 일치):
    x_next = x + v*cos(yaw)*dt
    y_next = y + v*sin(yaw)*dt
    yaw_next = yaw + yaw_rate*dt
    이걸 dt=0.1초씩 30번 반복 (horizon 3.0초)

출력: ~output_topic (기본 /app/perc/predicted_path, visualization_msgs/MarkerArray)
      각 트랙마다 LINE_STRIP(파란 궤적) + 시간이 갈수록 커지는 SPHERE
"""

import math
import re

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

DT = 0.1          # 롤아웃 스텝 간격(초)
NUM_STEPS = 30     # 3.0초 / 0.1초
TEXT_NS = "track_info"

VEL_RE = re.compile(r"Vel\s*:\s*(-?[\d.]+)m/s")
YAWVEL_RE = re.compile(r"YawVel\s*:\s*(-?[\d.]+)deg/s")


def _quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FutureRolloutNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/app/perc/vis_all_track")
        output_topic = rospy.get_param("~output_topic", "/app/perc/predicted_path")
        self._dt = rospy.get_param("~dt", DT)
        self._num_steps = rospy.get_param("~num_steps", NUM_STEPS)
        self._publish_rate_limit = rospy.get_param("~publish_rate_limit", 10.0)  # RViz 렉 방지, 기본 10Hz
        self._last_publish_time = 0.0

        self._pub = rospy.Publisher(output_topic, MarkerArray, queue_size=10)
        rospy.Subscriber(input_topic, MarkerArray, self._callback, queue_size=10)

        rospy.loginfo(
            "[future_rollout_node] %s -> %s (dt=%.2fs x %d steps = %.1fs horizon, CTRV)",
            input_topic, output_topic, self._dt, self._num_steps, self._dt * self._num_steps,
        )

    def _parse_track_info_marker(self, marker):
        """ns=="track_info" 텍스트 마커 하나에서 (track_id, x, y, yaw, v, yaw_rate_rad) 추출."""
        vel_match = VEL_RE.search(marker.text)
        yawvel_match = YAWVEL_RE.search(marker.text)
        if vel_match is None or yawvel_match is None:
            return None

        v = float(vel_match.group(1))
        yaw_rate_deg = float(yawvel_match.group(1))
        yaw_rate = math.radians(yaw_rate_deg)

        x = marker.pose.position.x
        y = marker.pose.position.y
        yaw = _quaternion_to_yaw(marker.pose.orientation)

        return marker.id, x, y, yaw, v, yaw_rate

    def _ctrv_rollout(self, x, y, yaw, v, yaw_rate):
        """CTRV로 dt*num_steps초까지 num_steps개 지점을 반복 전개."""
        points = []
        cur_x, cur_y, cur_yaw = x, y, yaw
        for _ in range(self._num_steps):
            cur_x += v * math.cos(cur_yaw) * self._dt
            cur_y += v * math.sin(cur_yaw) * self._dt
            cur_yaw += yaw_rate * self._dt
            points.append((cur_x, cur_y, cur_yaw))
        return points

    def _callback(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self._last_publish_time < 1.0 / self._publish_rate_limit:
            return  # RViz 렉 방지를 위해 시각화 발행 속도만 제한 (내부 계산 자체는 그대로)
        self._last_publish_time = now

        out = MarkerArray()

        clear = Marker()
        clear.header = msg.markers[0].header if msg.markers else None
        clear.ns = "predicted_path"
        clear.action = Marker.DELETEALL
        if clear.header is not None:
            out.markers.append(clear)

        for marker in msg.markers:
            if marker.ns != TEXT_NS or marker.type != Marker.TEXT_VIEW_FACING:
                continue

            parsed = self._parse_track_info_marker(marker)
            if parsed is None:
                continue
            track_id, x, y, yaw, v, yaw_rate = parsed

            trajectory = self._ctrv_rollout(x, y, yaw, v, yaw_rate)

            line = Marker()
            line.header = marker.header
            line.ns = "predicted_path"
            line.id = track_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.2
            line.color = ColorRGBA(0.2, 0.6, 1.0, 0.9)  # 파란색
            line.lifetime = rospy.Duration(0.3)
            line.points = [Point(px, py, marker.pose.position.z - 3.0) for (px, py, _) in trajectory]
            out.markers.append(line)
            # 구슬(SPHERE) 마커는 트랙당 30개씩 발행되어 RViz 렉의 주원인이었음 -> 제거.
            # 선(LINE_STRIP) 하나로도 궤적 방향/형태 확인에 충분함.

        self._pub.publish(out)


def main():
    rospy.init_node("future_rollout_node")
    FutureRolloutNode()
    rospy.spin()


if __name__ == "__main__":
    main()
