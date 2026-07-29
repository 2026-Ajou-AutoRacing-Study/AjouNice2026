#!/usr/bin/env python3
"""
visualization_node.py

/tracking/objects, /prediction/objects를 RViz에서 볼 수 있도록
visualization_msgs/MarkerArray로 변환한다.

RViz 확인 항목(변지훈님 요청):
    - 상대 차량 앞에 "미래 궤적"이 실제로 깔리는지
    - 이 궤적이 실제 GT 궤적(또는 실제 이동 경로)과 얼마나 다른지 육안으로 비교
    - lanelet map을 같이 켜서 도로 형상 대비 궤적이 자연스러운지 확인

출력:
    /tracking/markers   (visualization_msgs/MarkerArray) - 현재 위치 박스
    /prediction/markers  (visualization_msgs/MarkerArray) - 예측 궤적(LINE_STRIP + 각 horizon 지점 SPHERE)
"""

import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tracking_prediction.msg import TrackedObjectArray, PredictedObjectArray

CLASS_COLOR = {
    "vehicle": ColorRGBA(0.2, 0.6, 1.0, 0.6),
    "pedestrian": ColorRGBA(1.0, 0.6, 0.1, 0.8),
    "obstacle": ColorRGBA(0.8, 0.2, 0.2, 0.6),
}
DEFAULT_COLOR = ColorRGBA(0.6, 0.6, 0.6, 0.6)


def _yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class VisualizationNode(object):
    def __init__(self):
        tracking_topic = rospy.get_param("~tracking_topic", "/tracking/objects")
        prediction_topic = rospy.get_param("~prediction_topic", "/prediction/objects")
        tracking_markers_topic = rospy.get_param("~tracking_markers_topic", "/tracking/markers")
        prediction_markers_topic = rospy.get_param("~prediction_markers_topic", "/prediction/markers")
        self._frame_id = rospy.get_param("~frame_id", "")  # 비어있으면 입력 메시지의 frame_id 그대로 사용

        self._tracking_pub = rospy.Publisher(tracking_markers_topic, MarkerArray, queue_size=10)
        self._prediction_pub = rospy.Publisher(prediction_markers_topic, MarkerArray, queue_size=10)

        rospy.Subscriber(tracking_topic, TrackedObjectArray, self._tracking_cb, queue_size=10)
        rospy.Subscriber(prediction_topic, PredictedObjectArray, self._prediction_cb, queue_size=10)

        rospy.loginfo(
            "[visualization_node] %s -> %s, %s -> %s",
            tracking_topic, tracking_markers_topic,
            prediction_topic, prediction_markers_topic,
        )

    def _tracking_cb(self, msg):
        markers = MarkerArray()
        frame_id = self._frame_id or msg.header.frame_id

        # 이전에 그렸던 마커보다 개수가 줄면 남은 마커가 화면에 계속 남으므로 매번 전체 삭제 후 다시 그림
        clear = Marker()
        clear.header = msg.header
        clear.header.frame_id = frame_id
        clear.ns = "tracking"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for obj in msg.objects:
            color = CLASS_COLOR.get(obj.class_label, DEFAULT_COLOR)
            qx, qy, qz, qw = _yaw_to_quaternion(obj.heading)

            box = Marker()
            box.header = msg.header
            box.header.frame_id = frame_id
            box.ns = "tracking"
            box.id = obj.track_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position = obj.position
            box.pose.orientation.x = qx
            box.pose.orientation.y = qy
            box.pose.orientation.z = qz
            box.pose.orientation.w = qw
            box.scale = obj.size
            box.color = color
            box.lifetime = rospy.Duration(0.3)
            markers.markers.append(box)

            label = Marker()
            label.header = box.header
            label.ns = "tracking_label"
            label.id = obj.track_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                obj.position.x, obj.position.y, obj.position.z + obj.size.z / 2.0 + 0.5
            )
            label.scale.z = 0.6
            label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            label.text = f"id={obj.track_id} {obj.class_label}"
            label.lifetime = rospy.Duration(0.3)
            markers.markers.append(label)

        self._tracking_pub.publish(markers)

    def _prediction_cb(self, msg):
        markers = MarkerArray()
        frame_id = self._frame_id or msg.header.frame_id

        clear = Marker()
        clear.header = msg.header
        clear.header.frame_id = frame_id
        clear.ns = "prediction"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for obj in msg.objects:
            color = CLASS_COLOR.get(obj.class_label, DEFAULT_COLOR)

            # 예측 궤적을 잇는 선 (미래로 갈수록 옅어지게)
            line = Marker()
            line.header = msg.header
            line.header.frame_id = frame_id
            line.ns = "prediction_line"
            line.id = obj.track_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0  # 유효한 쿼터니언 필요 (기본값 0,0,0,0은 무효)
            line.scale.x = 0.2
            line.color = ColorRGBA(color.r, color.g, color.b, 0.9)
            line.lifetime = rospy.Duration(0.3)
            line.points = [Point(p.position.x, p.position.y, p.position.z) for p in obj.trajectory]
            markers.markers.append(line)

            # 각 horizon 지점에 작은 구슬 표시 (시간이 지날수록 커지게 해서 "미래로 갈수록"을 표현)
            for i, point in enumerate(obj.trajectory):
                sphere = Marker()
                sphere.header = line.header
                sphere.ns = "prediction_point"
                sphere.id = obj.track_id * 100 + i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = point.position
                sphere.pose.orientation.w = 1.0  # 유효한 쿼터니언 필요
                scale = 0.3 + 0.15 * i
                sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
                sphere.color = ColorRGBA(color.r, color.g, color.b, 0.5)
                sphere.lifetime = rospy.Duration(0.3)
                markers.markers.append(sphere)

        self._prediction_pub.publish(markers)


def main():
    rospy.init_node("visualization_node")
    VisualizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
