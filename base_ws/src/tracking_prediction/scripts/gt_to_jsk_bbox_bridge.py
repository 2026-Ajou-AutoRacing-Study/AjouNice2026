#!/usr/bin/env python3
"""
gt_to_jsk_bbox_bridge.py

MORAI GT(/Object_topic, morai_msgs/ObjectStatusList)를
jaeyoungjo99/ekf-multi-object-tracking이 요구하는 입력 포맷
(jsk_recognition_msgs/BoundingBoxArray, 기본 토픽명 /hmi/perc/jsk_pillar_objects)
으로 변환한다.

나중에 실제 OD(PointPainting 등) 결과로 교체할 때는, 이 노드 대신
같은 출력 토픽/타입으로 발행하는 다른 브릿지 노드를 launch에서
바꿔 끼우면 된다 (localization의 Ego_topic/실제추정치 통일 패턴과 동일).

class label 매핑(추정, ekf tracker가 label을 어떻게 쓰는지 원문 확인 안 됨):
    0 = vehicle, 1 = pedestrian, 2 = obstacle
"""

import math

import rospy
from morai_msgs.msg import ObjectStatusList
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

MORAI_HEADING_IN_DEGREES = True

CLASS_LABEL = {
    "vehicle": 0,
    "pedestrian": 1,
    "obstacle": 2,
}


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def _yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GtToJskBridge(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param("~output_topic", "/hmi/perc/jsk_pillar_objects")
        self._frame_id = rospy.get_param("~frame_id", "")  # 비어있으면 입력 메시지의 frame_id 그대로 사용

        # OD가 실제로는 GT(약 48.7Hz)보다 훨씬 느릴 것이므로, 낮은 hz를 라이브로
        # 재현해서 테스트할 수 있게 발행 속도 제한 옵션 추가 (기본값 0=제한 없음=GT 그대로)
        self._simulated_od_hz = rospy.get_param("~simulated_od_hz", 0.0)
        self._last_publish_time = 0.0

        self._pub = rospy.Publisher(output_topic, BoundingBoxArray, queue_size=10)
        rospy.Subscriber(input_topic, ObjectStatusList, self._callback, queue_size=10)

        rospy.loginfo("[gt_to_jsk_bbox_bridge] %s -> %s", input_topic, output_topic)

    def _make_box(self, obj, header, class_label):
        box = BoundingBox()
        box.header = header

        box.pose.position.x = obj.position.x
        box.pose.position.y = obj.position.y
        box.pose.position.z = obj.position.z

        yaw = _heading_to_rad(obj.heading)
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        box.pose.orientation.x = qx
        box.pose.orientation.y = qy
        box.pose.orientation.z = qz
        box.pose.orientation.w = qw

        box.dimensions = obj.size
        box.value = 1.0  # 신뢰도(likelihood) - GT라서 항상 1.0
        box.label = CLASS_LABEL.get(class_label, 0)
        return box

    def _callback(self, msg):
        if self._simulated_od_hz > 0:
            now = rospy.Time.now().to_sec()
            if now - self._last_publish_time < 1.0 / self._simulated_od_hz:
                return  # 실제 OD가 이 정도 hz로만 검출을 준다고 가정하고 나머지는 무시
            self._last_publish_time = now

        out = BoundingBoxArray()
        out.header = msg.header
        if self._frame_id:
            out.header.frame_id = self._frame_id

        boxes = []
        for obj in msg.npc_list:
            boxes.append(self._make_box(obj, out.header, "vehicle"))
        for obj in msg.pedestrian_list:
            boxes.append(self._make_box(obj, out.header, "pedestrian"))
        for obj in msg.obstacle_list:
            boxes.append(self._make_box(obj, out.header, "obstacle"))

        out.boxes = boxes
        self._pub.publish(out)


def main():
    rospy.init_node("gt_to_jsk_bbox_bridge")
    GtToJskBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
