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

class label 매핑 (2026-07-30 소스 확인 완료 - 이전엔 추정치였음):
    ekf-multi-object-tracking의 실제 enum(ekf_multi_object_tracking.hpp,
    ObjectClass)은 UNKNOWN=0, CAR=1, TRUCK=2, PEDESTRIAN=3, BICYCLE=4.
    이전 버전은 이걸 몰라서 vehicle=0/pedestrian=1/obstacle=2로 잘못 추정해뒀었고,
    그 상태로는 vehicle이 UNKNOWN(0)으로, pedestrian이 CAR(1)로 오분류되고 있었음.

exclude_ids / teleport_guard (2026-07-31 추가):
    sensor_cam_raw_roundabout1.bag 재생 중 /tracking/objects가 순간 약 2~3초씩
    끊기는 현상을 발견함. 원인 확인(diagnose_object_topic_gap.py로 bag 원본은
    깨끗함을 먼저 확인, diagnose_teleport_near_gap.py로 재확인): NPC가 로터리를
    한 바퀴 돌 때마다 시작지점으로 순간이동(텔레포트)하는데, 이게 처음엔
    eval 스크립트들이 걸러온 junk ID(16,18,20,21,23,25,27,30,31,34,35,40)만의
    문제인 줄 알았으나, 실제로는 "정상" 순환 차량들(1,13,38,42,44,46,48 등)도
    한 바퀴마다 똑같이 순간이동함이 확인됨 - 즉 특정 ID만의 문제가 아니라
    이 시뮬레이션의 NPC 루프 방식 자체(한 바퀴 완주 시 스폰 지점으로 리셋)의
    특성. EKF의 max_association_dist_m(5.0m)을 넘는 순간이동이라 트랙을
    잃고, output_confirmed_track:true라 재확인될 때까지 전체 출력이 끊김.
    실제 검출기라면 물리적으로 불가능한 현상(차가 순간이동할 수 없음)이라
    GT bag/시뮬레이터의 아티팩트로 판단.

    조치 두 단계:
      1) ~exclude_ids: 애초에 junk로 알려진 ID는 트래커 입력에서 아예 제외
         (여전히 유효 - 다른 이유로도 junk이므로 유지).
      2) ~teleport_guard_dist_m (기본 5.0m, EKF의 max_association_dist_m과 동일):
         ID에 상관없이, 직전 프레임 대비 이 거리 이상 순간이동한 객체는 "그
         프레임만" 발행에서 제외(ID 자체를 영구 제외하지 않음 - 다음 프레임에
         점프가 없으면 정상적으로 다시 포함됨). 이게 일반적이고 근본적인 해결.
"""

import math

import rospy
from morai_msgs.msg import ObjectStatusList
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

MORAI_HEADING_IN_DEGREES = True

CLASS_LABEL = {
    "vehicle": 1,     # CAR
    "pedestrian": 3,  # PEDESTRIAN
    "obstacle": 0,    # UNKNOWN
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

        # 순간이동(텔레포트) 등 junk로 판정된 unique_id를 트래커 입력에서 제외.
        # 콤마구분 문자열(launch param) 또는 리스트 둘 다 허용.
        exclude_ids_param = rospy.get_param("~exclude_ids", "")
        if isinstance(exclude_ids_param, str):
            self._exclude_ids = set(
                int(x) for x in exclude_ids_param.split(",") if x.strip()
            )
        else:
            self._exclude_ids = set(int(x) for x in exclude_ids_param)
        if self._exclude_ids:
            rospy.loginfo("[gt_to_jsk_bbox_bridge] 제외할 unique_id: %s", sorted(self._exclude_ids))

        # ID 무관 순간이동(텔레포트) 가드: 직전 위치 대비 이 거리(m) 이상 점프하면
        # 그 프레임만 스킵. EKF의 max_association_dist_m과 같은 값(5.0m) 기본.
        self._teleport_guard_dist = float(rospy.get_param("~teleport_guard_dist_m", 5.0))
        # {unique_id: (x, y)} - 리스트 종류(npc/pedestrian/obstacle) 구분 없이 unique_id로 추적
        self._last_pos = {}
        self._teleport_skip_count = 0

        self._pub = rospy.Publisher(output_topic, BoundingBoxArray, queue_size=10)
        rospy.Subscriber(input_topic, ObjectStatusList, self._callback, queue_size=10)

        rospy.loginfo("[gt_to_jsk_bbox_bridge] %s -> %s", input_topic, output_topic)

    def _passes_teleport_guard(self, obj):
        """직전 프레임 대비 teleport_guard_dist 이상 점프했으면 False(스킵).
        last_pos는 항상 갱신 - 텔레포트된 새 위치에 정착하면 바로 다음 프레임부터
        다시 정상 발행되게 하기 위함(영구 제외 아님, 그 프레임만 제외)."""
        uid = obj.unique_id
        x, y = obj.position.x, obj.position.y
        prev = self._last_pos.get(uid)
        self._last_pos[uid] = (x, y)
        if prev is None:
            return True
        dist = math.hypot(x - prev[0], y - prev[1])
        if dist >= self._teleport_guard_dist:
            self._teleport_skip_count += 1
            rospy.logwarn_throttle(
                2.0,
                "[gt_to_jsk_bbox_bridge] uid=%d 순간이동 감지(%.1fm >= %.1fm) - 이 프레임 스킵 "
                "(누적 %d회)",
                uid, dist, self._teleport_guard_dist, self._teleport_skip_count,
            )
            return False
        return True

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
            if obj.unique_id in self._exclude_ids:
                continue
            if not self._passes_teleport_guard(obj):
                continue
            boxes.append(self._make_box(obj, out.header, "vehicle"))
        for obj in msg.pedestrian_list:
            if obj.unique_id in self._exclude_ids:
                continue
            if not self._passes_teleport_guard(obj):
                continue
            boxes.append(self._make_box(obj, out.header, "pedestrian"))
        for obj in msg.obstacle_list:
            if obj.unique_id in self._exclude_ids:
                continue
            if not self._passes_teleport_guard(obj):
                continue
            boxes.append(self._make_box(obj, out.header, "obstacle"))

        out.boxes = boxes
        self._pub.publish(out)


def main():
    rospy.init_node("gt_to_jsk_bbox_bridge")
    GtToJskBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
