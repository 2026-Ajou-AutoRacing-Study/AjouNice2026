#!/usr/bin/env python3
"""
prediction_node.py

CTRA(Constant Turn Rate and Acceleration) 물리 모델로 각 추적 객체의
미래 위치를 여러 시점(horizon)에 대해 예측한다.

CTRA 상태: s = [x, y, theta, v, omega, a]
    - v, a: TrackedObject.velocity / .acceleration 벡터를 heading 방향으로
      투영해서 스칼라로 계산 (MORAI GT가 벡터로 주기 때문)
    - omega(yaw rate): GT에 직접 없음 -> 직전 프레임과 "가장 가까운 위치"의
      객체를 단순 최근접 매칭해서 heading 변화량으로 유한차분 추정.
      (주의: 이 매칭은 tracking_node의 dummy track_id와 무관한, 이 노드
       내부에서만 쓰는 보조 추정치. track_id가 dummy라 프레임 간 지속되지
       않기 때문에 여기서 별도로 처리함)

수식 근거: Autoware.Auto motion_model 문서(CATR)
    https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/motion-model-design.html
    - omega == 0 (직진): 고전 등가속도 공식(거리 = v*t + 1/2*a*t^2)으로 환원
    - omega != 0 (회전): 1/omega, 1/omega^2 항이 있어 omega가 0에 가까우면
      수치적으로 불안정 -> OMEGA_EPS 이하면 강제로 직진 공식 사용
"""

import math

import rospy
from geometry_msgs.msg import Point

from tracking_prediction.msg import (
    TrackedObjectArray,
    PredictedObject,
    PredictedObjectArray,
    PredictedTrajectoryPoint,
)

# omega가 이 값보다 작으면 수치안정성을 위해 직진(등가속도) 공식으로 강제 전환
OMEGA_EPS = 1e-3
# 감쇠 가속도 모델의 lambda (클수록 감쇠 빠름). 2026-07-27 오프라인 비교실험
# 절충안으로 0.4 채택 (mean 개선 vs median 유지 트레이드오프). ROS 파라미터로 덮어쓰기 가능.
DECAY_LAMBDA = 0.4

# 최근접 매칭 시 "같은 객체"로 볼 최대 거리(m). 이보다 멀면 새 객체로 간주하고
# yaw_rate를 추정할 과거 정보가 없다고 보고 0으로 둔다.
MATCH_DIST_THRESHOLD = 3.0


def _project_to_scalar(vec2_like, heading):
    """velocity/acceleration 벡터(x,y)를 heading 방향으로 투영한 스칼라값."""
    return vec2_like.x * math.cos(heading) + vec2_like.y * math.sin(heading)


def ctra_predict(x, y, theta, v, omega, a, dt, lam=DECAY_LAMBDA):
    """
    CTRA 모델로 dt초 뒤의 (x, y, theta)를 예측한다.
    가속도는 상수가 아니라 a(t) = a * exp(-lam*t)로 감쇠한다고 가정
    (Autoware map_based_prediction 방식). lam=0에 가까울수록 기존
    등가속도 CTRA와 동일해짐 (수식 검증 완료).

    오프라인 비교 실험 결과(2026-07-27), lam이 클수록 평균오차는
    개선되나 중앙값은 소폭 악화되는 트레이드오프가 있어, 절충값으로
    lam=0.4를 기본값으로 채택함 (~lambda 0.3~0.5 권장 구간).
    """
    if abs(omega) < OMEGA_EPS:
        if lam < 1e-6:
            dist = v * dt + 0.5 * a * dt * dt
        else:
            dist = v * dt + (a / lam) * (dt - (1.0 / lam) * (1 - math.exp(-lam * dt)))
        x_next = x + dist * math.cos(theta)
        y_next = y + dist * math.sin(theta)
        theta_next = theta
        return x_next, y_next, theta_next

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    # 등속(v0) 부분: 기존 CTRA와 동일한 형태
    x_const = v * (sin_tn - sin_t) / omega
    y_const = v * (cos_t - cos_tn) / omega

    if lam < 1e-6:
        x_decay = (
            (a * dt * sin_tn) / omega - (a * cos_t) / (omega ** 2) + (a * cos_tn) / (omega ** 2)
        )
        y_decay = (
            -(a * dt * cos_tn) / omega - (a * sin_t) / (omega ** 2) + (a * sin_tn) / (omega ** 2)
        )
        return x + x_const + x_decay, y + y_const + y_decay, theta_next

    denom = lam * lam + omega * omega
    ic = (
        math.exp(-lam * dt) * (-lam * cos_tn + omega * sin_tn)
        - (-lam * cos_t + omega * sin_t)
    ) / denom
    is_ = (
        math.exp(-lam * dt) * (-lam * sin_tn - omega * cos_tn)
        - (-lam * sin_t - omega * cos_t)
    ) / denom

    x_decay = (a / lam) * ((sin_tn - sin_t) / omega - ic)
    y_decay = (a / lam) * ((cos_t - cos_tn) / omega - is_)

    return x + x_const + x_decay, y + y_const + y_decay, theta_next


class CtraPredictionNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/tracking/objects")
        output_topic = rospy.get_param("~output_topic", "/prediction/objects")
        # 기본 예측 horizon: 0.5, 1.0, 1.5, 2.0, 3.0초 뒤
        self._horizons = rospy.get_param(
            "~horizons", [0.5, 1.0, 1.5, 2.0, 3.0]
        )
        self._decay_lambda = rospy.get_param("~decay_lambda", DECAY_LAMBDA)

        self._pub = rospy.Publisher(output_topic, PredictedObjectArray, queue_size=10)
        self._sub = rospy.Subscriber(
            input_topic, TrackedObjectArray, self._callback, queue_size=10
        )

        # 직전 프레임의 (class_label별) [(x, y, heading, stamp), ...] 저장
        self._prev_by_class = {}
        self._prev_stamp = None

        rospy.loginfo(
            "[prediction_node] CTRA(감쇠가속도) 예측 시작: %s -> %s (horizons=%s, decay_lambda=%s)",
            input_topic,
            output_topic,
            self._horizons,
            self._decay_lambda,
        )

    def _estimate_yaw_rate(self, obj, dt, prev_list):
        """직전 프레임에서 위치가 가장 가까운 객체를 찾아 yaw_rate를 유한차분으로 추정."""
        if dt <= 0 or not prev_list:
            return 0.0

        best = None
        best_dist = MATCH_DIST_THRESHOLD
        for (px, py, pheading) in prev_list:
            dist = math.hypot(obj.position.x - px, obj.position.y - py)
            if dist < best_dist:
                best_dist = dist
                best = pheading

        if best is None:
            return 0.0

        dtheta = obj.heading - best
        # -pi ~ pi로 정규화 (급격한 wrap-around 방지)
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
        return dtheta / dt

    def _callback(self, msg):
        now_stamp = msg.header.stamp.to_sec()
        dt = 0.0
        if self._prev_stamp is not None:
            dt = now_stamp - self._prev_stamp

        out = PredictedObjectArray()
        out.header = msg.header

        predicted_objects = []
        next_prev_by_class = {}

        for obj in msg.objects:
            prev_list = self._prev_by_class.get(obj.class_label, [])
            omega = self._estimate_yaw_rate(obj, dt, prev_list)

            v = math.hypot(obj.velocity.x, obj.velocity.y)
            # 부호(전진/후진)를 heading 투영으로 결정
            if _project_to_scalar(obj.velocity, obj.heading) < 0:
                v = -v
            a = _project_to_scalar(obj.acceleration, obj.heading)

            traj_points = []
            for horizon in self._horizons:
                px, py, ptheta = ctra_predict(
                    obj.position.x, obj.position.y, obj.heading, v, omega, a, horizon,
                    lam=self._decay_lambda,
                )
                point = PredictedTrajectoryPoint()
                point.time_offset = horizon
                point.position = Point(x=px, y=py, z=obj.position.z)
                point.heading = ptheta
                traj_points.append(point)

            pred_obj = PredictedObject()
            pred_obj.track_id = obj.track_id
            pred_obj.class_label = obj.class_label
            pred_obj.trajectory = traj_points
            predicted_objects.append(pred_obj)

            next_prev_by_class.setdefault(obj.class_label, []).append(
                (obj.position.x, obj.position.y, obj.heading)
            )

        out.objects = predicted_objects
        self._pub.publish(out)

        self._prev_by_class = next_prev_by_class
        self._prev_stamp = now_stamp


def main():
    rospy.init_node("prediction_node")
    CtraPredictionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
