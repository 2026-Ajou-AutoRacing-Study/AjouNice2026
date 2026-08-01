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
        # 2026-08-01 추가/최종: 실사용 horizon으로 1.5s를 채택함(미션 무관 단일 고정,
        # docs/entry_zone_ctrv_ctra_summary.md, docs/highway_ctrv_ctra_summary.md). 0으로
        # 두면 prediction_node가 발행하는 horizons 배열 전체를 선+구슬로 다 그리고,
        # 값을 주면(예: 1.5) 그 시점 하나만 큰 구슬로 강조해서 그림 - "우리가 쓰기로 한 값"만
        # 깔끔하게 눈으로 확인하기 위함.
        self._only_horizon = rospy.get_param("~only_horizon", 0.0)
        self._only_horizon_tol = rospy.get_param("~only_horizon_tolerance", 0.05)

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

        if self._only_horizon > 0.0:
            # 마커 개수를 줄이기 위해 차량별로 따로 발행하지 않고, 전체 차량의 곡선을
            # LINE_LIST 하나, 중간 웨이포인트 점들을 SPHERE_LIST 하나에 몰아서 발행함
            # (2026-08-01: 마커가 콜백당 수십 개씩 나가서 RViz가 버벅인다는 피드백으로 변경.
            # nav_msgs/Path는 여러 차량 궤적을 하나로 강제 연결해버려서(원래 단일 경로용)
            # 대신 이 방식 채택 - 차량별 분리는 유지하면서 마커 수는 크게 줄임).
            # 끝점(only_horizon)만 차량별로 큰 구슬+라벨을 개별 마커로 남겨둠(수가 적어 부담 없음).
            line_list = Marker()
            line_list.header = msg.header
            line_list.header.frame_id = frame_id
            line_list.ns = "prediction_curve"
            line_list.id = 0
            line_list.type = Marker.LINE_LIST
            line_list.action = Marker.ADD
            line_list.pose.orientation.w = 1.0
            line_list.scale.x = 0.25
            line_list.lifetime = rospy.Duration(0.5)

            dot_list = Marker()
            dot_list.header = line_list.header
            dot_list.ns = "prediction_point"
            dot_list.id = 0
            dot_list.type = Marker.SPHERE_LIST
            dot_list.action = Marker.ADD
            dot_list.pose.orientation.w = 1.0
            dot_list.scale.x = dot_list.scale.y = dot_list.scale.z = 0.35
            dot_list.lifetime = rospy.Duration(0.5)

            for obj in msg.objects:
                color = CLASS_COLOR.get(obj.class_label, DEFAULT_COLOR)
                pts = sorted(
                    (p for p in obj.trajectory if p.time_offset <= self._only_horizon + self._only_horizon_tol),
                    key=lambda p: p.time_offset,
                )
                if not pts:
                    continue
                end_point = pts[-1]

                # LINE_LIST는 점 2개당 선분 1개 - 연속 구간마다 (i, i+1) 쌍을 추가
                for i in range(len(pts) - 1):
                    p0, p1 = pts[i].position, pts[i + 1].position
                    line_list.points.append(Point(p0.x, p0.y, p0.z))
                    line_list.points.append(Point(p1.x, p1.y, p1.z))
                    line_list.colors.append(ColorRGBA(color.r, color.g, color.b, 0.9))
                    line_list.colors.append(ColorRGBA(color.r, color.g, color.b, 0.9))

                # 끝점 제외한 중간 웨이포인트만 작은 구슬(경로 위 실제 샘플 표시용)
                for p in pts[:-1]:
                    dot_list.points.append(Point(p.position.x, p.position.y, p.position.z))
                    dot_list.colors.append(ColorRGBA(color.r, color.g, color.b, 0.5))

                # 끝점(only_horizon)은 차량별로 크게 강조 + 라벨 (개수 적어 개별 마커로 둬도 무방)
                end_sphere = Marker()
                end_sphere.header = line_list.header
                end_sphere.ns = "prediction_end"
                end_sphere.id = obj.track_id
                end_sphere.type = Marker.SPHERE
                end_sphere.action = Marker.ADD
                end_sphere.pose.position = end_point.position
                end_sphere.pose.orientation.w = 1.0
                end_sphere.scale.x = end_sphere.scale.y = end_sphere.scale.z = 1.0
                end_sphere.color = ColorRGBA(color.r, color.g, color.b, 0.95)
                end_sphere.lifetime = rospy.Duration(0.5)
                markers.markers.append(end_sphere)

                label = Marker()
                label.header = line_list.header
                label.ns = "prediction_label"
                label.id = obj.track_id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = Point(
                    end_point.position.x, end_point.position.y, end_point.position.z + 1.0
                )
                label.scale.z = 0.6
                label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                label.text = f"+{end_point.time_offset:.1f}s"
                label.lifetime = rospy.Duration(0.5)
                markers.markers.append(label)

            if line_list.points:
                markers.markers.append(line_list)
            if dot_list.points:
                markers.markers.append(dot_list)

            self._prediction_pub.publish(markers)
            return

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
