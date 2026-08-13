#!/usr/bin/env python3
"""
tracking_node.py

지금 단계 목표:
    실제 Object Detection(BEVFusion/pointpainting)이 아직 완성 전이므로,
    MORAI가 발행하는 GT(/Object_topic, morai_msgs/ObjectStatusList)를
    "이미 검출된 3D bbox들"인 것처럼 취급해서 표준 트래킹 출력 토픽으로 재발행한다.

    실제 3D detector는 프레임마다 안정적인 ID를 안 주기 때문에, 진짜 데이터연관
    (재식별) 로직 없이 "이번 프레임에 보인 순서대로 1, 2, 3 ... " 임의 ID를
    부여한다 (dummy tracking). 나중에 real MOT(데이터연관+EKF)로 교체할 때
    이 노드의 입력/출력 인터페이스는 그대로 유지하고 내부 로직만 바꾸면 된다.

    - 입력: /Object_topic (morai_msgs/ObjectStatusList) 또는 실제 OD 결과가
      나중에 이 노드 대신 같은 이름의 출력 토픽에 publish 되도록 launch에서
      노드 자체를 스위칭한다 (localization의 Ego_topic/추정치 방식과 동일).
    - 출력: ~output_topic (기본값 /tracking/objects, TrackedObjectArray)

주의:
    - MORAI heading은 degree로 발행된다고 가정하고 radian으로 변환한다.
      (실측으로 재확인 필요, 이전 조사에서 확정 못함 - MORAI_HEADING_IN_DEGREES)
    - position/velocity/acceleration은 입력 프레임을 그대로 통과시킨다.
      (world 또는 ego-relative 여부는 /Object_topic을 그대로 쓰는 launch 설정에 따름)
"""

import math

import rospy
from morai_msgs.msg import ObjectStatusList

from tracking_prediction.msg import TrackedObject, TrackedObjectArray

# MORAI heading 단위 가정 (morai_to_nuscenes.py에서와 동일한 가정 유지)
MORAI_HEADING_IN_DEGREES = True


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


class DummyTrackingNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param("~output_topic", "/tracking/objects")

        self._pub = rospy.Publisher(output_topic, TrackedObjectArray, queue_size=10)
        self._sub = rospy.Subscriber(
            input_topic, ObjectStatusList, self._callback, queue_size=10
        )

        rospy.loginfo(
            "[tracking_node] dummy tracking 시작: %s -> %s", input_topic, output_topic
        )

    def _make_tracked_object(self, obj, track_id, class_label):
        t = TrackedObject()
        t.track_id = track_id
        t.class_label = class_label
        t.position.x = obj.position.x
        t.position.y = obj.position.y
        t.position.z = obj.position.z
        t.heading = _heading_to_rad(obj.heading)
        # MORAI ObjectStatus.velocity는 km/h로 발행됨 (실측으로 확인) -> m/s로 변환
        KMH_TO_MS = 1.0 / 3.6
        t.velocity.x = obj.velocity.x * KMH_TO_MS
        t.velocity.y = obj.velocity.y * KMH_TO_MS
        t.velocity.z = obj.velocity.z * KMH_TO_MS
        t.acceleration = obj.acceleration
        t.size = obj.size
        return t

    def _callback(self, msg):
        out = TrackedObjectArray()
        out.header = msg.header
        if not out.header.stamp:
            out.header.stamp = rospy.Time.now()

        track_id = 0
        objects = []

        # dummy 배정: 순서대로 1,2,3... (재식별/매칭 로직 없음)
        for obj in msg.npc_list:
            track_id += 1
            objects.append(self._make_tracked_object(obj, track_id, "vehicle"))

        for obj in msg.pedestrian_list:
            track_id += 1
            objects.append(self._make_tracked_object(obj, track_id, "pedestrian"))

        for obj in msg.obstacle_list:
            track_id += 1
            objects.append(self._make_tracked_object(obj, track_id, "obstacle"))

        out.objects = objects
        self._pub.publish(out)


def main():
    rospy.init_node("tracking_node")
    DummyTrackingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
