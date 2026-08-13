#!/usr/bin/env python3
"""
predicted_objects_autoware_bridge.py

목적(2026-08-02): T&P 파이프라인(우리 커스텀 TrackedObjectArray/PredictedObjectArray)을
판단팀이 실제로 구독하는 Autoware 표준 형식으로 변환해서 다시 발행한다.

배경: 판단팀 코드(behavior_path_planner/behavior_velocity_planner/motion_velocity_planner,
총 59개 파일)를 직접 읽어서 확인한 결과:
    - 토픽명이 다름: 우리는 /prediction/objects, 판단팀은
      /perception/object_recognition/objects 를 구독함
      (behavior_path_planner_impl.launch:109의 remap 확인).
    - 메시지 타입이 다름: 우리는 tracking_prediction/PredictedObjectArray,
      판단팀은 autoware_perception_msgs/PredictedObjects.
    - kinematics.predicted_paths[].path 가 "여러 시점이 이어진 경로"를 요구함
      (bpp_dynamic_avoidance_tools.py의 _predicted_path_cuts_in()이
      `for pose in predicted.path`로 경로 전체를 순회하며 cut-in을 판단).
      우리가 최종 채택한 단일 horizon(1.5s) 점 하나만으로는 부족함 - 이 브릿지를
      쓰려면 prediction_node.py의 ~horizons를 여러 값의 리스트로(예:
      [0.3,0.6,0.9,1.2,1.5]) 띄워야 함. **주의: 이건 "실사용 horizon=1.5s"
      최종 결정(docs/tp_final_report.md)을 뒤집는 게 아니다** - 1.5s를 경로의
      마지막 점으로 그대로 포함시키는 것뿐이고, 안전마진 판단 자체는 여전히
      1.5s 시점 값을 기준으로 함.

변환 매핑:
    - track_id(uint32) -> object_id(UUID): track_id를 16바이트 뒤쪽 4바이트에
      박아넣어 결정적으로 생성한다(같은 track_id는 항상 같은 UUID로 매핑됨.
      진짜 랜덤 UUID 스펙은 아니지만, 이 세션 내에서 object_id로 구분만 되면
      충분함).
    - class_label(string: vehicle/pedestrian/obstacle) -> classification[0]
      (ObjectClassification label enum + probability=1.0).
    - /tracking/objects의 최신 상태(position/heading/velocity/acceleration/size)
      -> kinematics.initial_pose/twist/acceleration_with_covariance, shape.
      covariance는 전부 0으로 채움(우리는 불확실성 추정치가 없음 - 판단팀 코드가
      covariance를 실제로 읽는지는 확인 못 함, 필요해지면 추가 확인 필요).
      twist/accel은 판단팀 코드(bpp_dynamic_avoidance_tools.py의 _object_velocity,
      pose.orientation과 결합해서 씀)를 보고 "객체 자신의 heading 기준 전후/좌우
      성분(body frame)"으로 넣어야 한다는 걸 확인해서, world-frame 벡터를
      heading으로 투영/직교분해해서 twist.linear.x(전진)/y(횡)로 채운다.
    - /prediction/objects의 trajectory[] -> kinematics.predicted_paths[0].path
      (heading -> quaternion 변환). time_step은 horizons가 등간격이라고 가정하고
      첫 두 점 간격으로 계산한다(등간격이 아니면 경고 로그 - 판단팀 cut-in 판단이
      부정확해질 수 있음).

주의(타이밍): /tracking/objects와 /prediction/objects는 서로 다른 콜백에서
들어오지만, prediction_node.py가 /tracking/objects를 받은 그 콜백 안에서 곧바로
동기적으로 /prediction/objects를 발행하므로(중간 큐잉/딜레이 없음), 실질적으로
같은 프레임끼리 짝지어진다고 가정한다. 이 브릿지는 track_id별 "가장 최근에 받은"
tracking 상태를 캐싱해뒀다가 매칭하는 방식이라, 이론상 아주 드물게 한 프레임
어긋날 수 있다 - 문제가 되면 message_filters.ApproximateTimeSynchronizer로 교체할 것.
"""

import math

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjectKinematics,
    PredictedObjects,
    PredictedPath,
    Shape,
)

from tracking_prediction.msg import PredictedObjectArray, TrackedObjectArray

CLASS_LABEL_TO_AUTOWARE = {
    "vehicle": ObjectClassification.CAR,
    "pedestrian": ObjectClassification.PEDESTRIAN,
    "obstacle": ObjectClassification.UNKNOWN,
}

_ZERO_COVARIANCE_36 = [0.0] * 36


def _yaw_to_quaternion(yaw):
    """yaw(rad) 단일 회전 -> geometry_msgs/Quaternion (roll=pitch=0 가정,
    지상 차량 예측이라 이 프로젝트 전체에서 쓰는 가정과 동일)."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _project_forward(vec2_like, heading):
    """world-frame Vector3를 heading 방향(전진 성분)으로 투영."""
    return vec2_like.x * math.cos(heading) + vec2_like.y * math.sin(heading)


def _project_lateral(vec2_like, heading):
    """world-frame Vector3를 heading에 수직인 방향(횡 성분)으로 투영."""
    return -vec2_like.x * math.sin(heading) + vec2_like.y * math.cos(heading)


def _uuid_from_track_id(track_id):
    b = [0] * 12 + [
        (track_id >> 24) & 0xFF,
        (track_id >> 16) & 0xFF,
        (track_id >> 8) & 0xFF,
        track_id & 0xFF,
    ]
    return UUID(uuid=b)


class PredictedObjectsAutowareBridge(object):
    def __init__(self):
        tracking_topic = rospy.get_param("~tracking_topic", "/tracking/objects")
        prediction_topic = rospy.get_param("~prediction_topic", "/prediction/objects")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/object_recognition/objects"
        )

        self._pub = rospy.Publisher(output_topic, PredictedObjects, queue_size=10)
        rospy.Subscriber(
            tracking_topic, TrackedObjectArray, self._tracking_callback, queue_size=10
        )
        rospy.Subscriber(
            prediction_topic, PredictedObjectArray, self._prediction_callback, queue_size=10
        )

        self._latest_tracked_by_id = {}

        rospy.loginfo(
            "[predicted_objects_autoware_bridge] %s (+ %s 최신상태) -> %s",
            prediction_topic, tracking_topic, output_topic,
        )

    def _tracking_callback(self, msg):
        for obj in msg.objects:
            self._latest_tracked_by_id[obj.track_id] = obj

    def _build_shape(self, tracked):
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=tracked.size.x, y=tracked.size.y, z=tracked.size.z)
        return shape

    def _build_kinematics(self, tracked, pred_obj):
        kin = PredictedObjectKinematics()

        pose = Pose()
        pose.position = Point(
            x=tracked.position.x, y=tracked.position.y, z=tracked.position.z
        )
        pose.orientation = _yaw_to_quaternion(tracked.heading)
        kin.initial_pose_with_covariance.pose = pose
        kin.initial_pose_with_covariance.covariance = list(_ZERO_COVARIANCE_36)

        v_fwd = _project_forward(tracked.velocity, tracked.heading)
        v_lat = _project_lateral(tracked.velocity, tracked.heading)
        kin.initial_twist_with_covariance.twist.linear = Vector3(x=v_fwd, y=v_lat, z=0.0)
        kin.initial_twist_with_covariance.covariance = list(_ZERO_COVARIANCE_36)

        a_fwd = _project_forward(tracked.acceleration, tracked.heading)
        a_lat = _project_lateral(tracked.acceleration, tracked.heading)
        kin.initial_acceleration_with_covariance.accel.linear = Vector3(
            x=a_fwd, y=a_lat, z=0.0
        )
        kin.initial_acceleration_with_covariance.covariance = list(_ZERO_COVARIANCE_36)

        path = PredictedPath()
        points = list(pred_obj.trajectory)
        path.path = []
        for pt in points:
            p = Pose()
            p.position = Point(x=pt.position.x, y=pt.position.y, z=pt.position.z)
            p.orientation = _yaw_to_quaternion(pt.heading)
            path.path.append(p)

        if len(points) >= 2:
            gaps = [
                points[i + 1].time_offset - points[i].time_offset
                for i in range(len(points) - 1)
            ]
            step = gaps[0]
            if max(gaps) - min(gaps) > 1e-3:
                rospy.logwarn_throttle(
                    5.0,
                    "[predicted_objects_autoware_bridge] ~horizons가 등간격이 아님(%s) - "
                    "time_step을 첫 간격(%.3fs)으로 근사함. 판단팀 cut-in 판단이 "
                    "부정확해질 수 있으니 등간격 리스트 사용을 권장함." % (gaps, step),
                )
            path.time_step = rospy.Duration(step)
        elif len(points) == 1:
            path.time_step = rospy.Duration(points[0].time_offset)
        else:
            path.time_step = rospy.Duration(0.0)
        path.confidence = 1.0

        kin.predicted_paths = [path]
        return kin

    def _prediction_callback(self, msg):
        out = PredictedObjects()
        out.header = msg.header

        objects = []
        for pred_obj in msg.objects:
            tracked = self._latest_tracked_by_id.get(pred_obj.track_id)
            if tracked is None:
                rospy.logwarn_throttle(
                    5.0,
                    "[predicted_objects_autoware_bridge] track_id=%d의 /tracking/objects "
                    "상태를 아직 못 받아서 이번 프레임에서 건너뜀" % pred_obj.track_id,
                )
                continue

            ao = PredictedObject()
            ao.object_id = _uuid_from_track_id(pred_obj.track_id)
            ao.existence_probability = 1.0
            ao.classification = [
                ObjectClassification(
                    label=CLASS_LABEL_TO_AUTOWARE.get(
                        pred_obj.class_label, ObjectClassification.UNKNOWN
                    ),
                    probability=1.0,
                )
            ]
            ao.kinematics = self._build_kinematics(tracked, pred_obj)
            ao.shape = self._build_shape(tracked)
            objects.append(ao)

        out.objects = objects
        self._pub.publish(out)


def main():
    rospy.init_node("predicted_objects_autoware_bridge")
    PredictedObjectsAutowareBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
