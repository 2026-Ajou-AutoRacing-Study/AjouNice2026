#!/usr/bin/env python3
"""
pointpainting_tracking_node.py

tracking_node.py(GT용)와 출력 인터페이스(/tracking/objects, TrackedObjectArray)를
완전히 동일하게 맞춘, PointPainting 실제 검출 결과 전용 tracking 노드.

이 노드가 필요한 이유:
    PointPainting(morai_pointpainting_ros/PointPaintingDetectionArray)은
    한 프레임의 순간적인 위치(pose)만 주고, velocity/acceleration은 아예
    주지 않는다 (실제 3D detector라 당연함 - GT처럼 미리 알 수 없음).
    반면 우리 CTRA prediction_node.py는 TrackedObject.velocity/acceleration을
    그대로 사용하므로, 이 노드에서 직접 속도와 yaw_rate를
    "직전 프레임과 위치가 가장 가까운 같은 클래스 검출값"의 유한차분으로
    추정해서 채워 넣는다 (prediction_node.py의 yaw_rate 추정과 같은 방식을
    velocity에도 적용한 것).

    dummy ID는 tracking_node.py와 마찬가지로 매 프레임 보이는 순서대로
    부여한다 (진짜 데이터연관/재식별 로직 없음 - 나중에 필요하면 교체).

입력: ~input_topic (기본 /pointpainting/detections, PointPaintingDetectionArray)
출력: ~output_topic (기본 /tracking/objects, TrackedObjectArray)
       -> tracking_node.py(GT용)와 완전히 동일한 토픽/타입이므로
          prediction_node.py는 입력 소스가 GT든 PointPainting이든
          수정 없이 그대로 동작한다. launch 파일에서 어느 tracking 노드를
          띄우느냐만 선택하면 됨.
"""

import math

import rospy
from morai_pointpainting_ros.msg import PointPaintingDetectionArray

from tracking_prediction.msg import TrackedObject, TrackedObjectArray

# 직전 프레임에서 같은 객체로 볼 최대 거리(m). tracking_node.py의 GT 버전과
# 달리, 여기서는 detection_id(dummy)가 아니라 클래스+최근접 위치로 매칭한다.
MATCH_DIST_THRESHOLD = 3.0


def _quaternion_to_yaw(q):
    """geometry_msgs/Quaternion -> yaw(radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PointPaintingTrackingNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/pointpainting/detections")
        output_topic = rospy.get_param("~output_topic", "/tracking/objects")

        self._pub = rospy.Publisher(output_topic, TrackedObjectArray, queue_size=10)
        self._sub = rospy.Subscriber(
            input_topic, PointPaintingDetectionArray, self._callback, queue_size=10
        )

        # 직전 프레임의 (class_name별) [(x, y, heading, stamp), ...] - 속도/yaw_rate
        # 추정용. GT tracking_node.py는 velocity를 그대로 받아 쓰지만, 여기서는
        # PointPainting이 안 주기 때문에 직접 계산해야 한다.
        self._prev_by_class = {}
        self._prev_stamp = None

        rospy.loginfo(
            "[pointpainting_tracking_node] 시작: %s -> %s", input_topic, output_topic
        )

    def _estimate_velocity_and_heading_rate(self, x, y, heading, dt, prev_list):
        """직전 프레임 최근접 매칭으로 (vx, vy 대신) 속도 스칼라/방향, yaw_rate 추정."""
        if dt <= 0 or not prev_list:
            return 0.0, 0.0  # v, omega

        best = None
        best_dist = MATCH_DIST_THRESHOLD
        for (px, py, pheading) in prev_list:
            dist = math.hypot(x - px, y - py)
            if dist < best_dist:
                best_dist = dist
                best = (px, py, pheading)

        if best is None:
            return 0.0, 0.0

        px, py, pheading = best
        v = math.hypot(x - px, y - py) / dt  # 위치차분/시간 = 속도 크기(m/s)

        dtheta = heading - pheading
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
        omega = dtheta / dt

        return v, omega

    def _callback(self, msg):
        now_stamp = msg.header.stamp.to_sec()
        dt = 0.0
        if self._prev_stamp is not None:
            dt = now_stamp - self._prev_stamp

        out = TrackedObjectArray()
        out.header = msg.header

        objects = []
        next_prev_by_class = {}
        track_id = 0

        for det in msg.detections:
            track_id += 1

            heading = _quaternion_to_yaw(det.pose.orientation)
            x, y = det.pose.position.x, det.pose.position.y

            prev_list = self._prev_by_class.get(det.class_name, [])
            v, omega = self._estimate_velocity_and_heading_rate(
                x, y, heading, dt, prev_list
            )

            t = TrackedObject()
            t.track_id = track_id
            t.class_label = det.class_name
            t.position.x = x
            t.position.y = y
            t.position.z = det.pose.position.z
            t.heading = heading
            # 속도 벡터로 환산 (heading 방향으로 v를 투영)
            t.velocity.x = v * math.cos(heading)
            t.velocity.y = v * math.sin(heading)
            t.velocity.z = 0.0
            # PointPainting은 가속도 정보가 없음 -> 0으로 둠 (CTRA에서 a=0으로 처리됨).
            # omega(yaw_rate)는 prediction_node.py가 또 자체적으로 재추정하므로
            # 여기서 heading만 정확히 넘겨주면 충분함.
            t.acceleration.x = 0.0
            t.acceleration.y = 0.0
            t.acceleration.z = 0.0
            t.size = det.size

            objects.append(t)
            next_prev_by_class.setdefault(det.class_name, []).append((x, y, heading))

        out.objects = objects
        self._pub.publish(out)

        self._prev_by_class = next_prev_by_class
        self._prev_stamp = now_stamp


def main():
    rospy.init_node("pointpainting_tracking_node")
    PointPaintingTrackingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
