#!/usr/bin/env python3
"""
extrapolation_node.py

배경: 판단/제어 모듈은 최소 20Hz, 권장 30~40Hz, 최대 50Hz로 현재 위치 정보가
필요한데, 실제 OD는 8~15Hz(실측 10~12Hz)로만 나온다. 그 사이 빈 구간을
CTRV로 외삽(extrapolation)해서 채워 재발행한다.

중요한 설계 원칙 (2026-07-31 결정):
    이 노드는 tracking(`/tracking/objects`, 실측 rate 그대로)과 prediction
    (`prediction_node.py`) 사이에 끼어들지 않는다. prediction_node.py는 지금처럼
    `/tracking/objects`(실측 rate)를 그대로 구독해서 horizon 예측을 계속한다.
    이 노드는 tracking 출력을 별도로 구독해서 고rate 버전만 새 토픽
    (`/tracking/objects_highrate`)으로 추가 발행한다.

    이유: prediction의 CTRV horizon 예측이 이미 외삽된(추정치가 섞인) 값을
    시작점으로 삼으면 외삽오차가 먼저 깔리고 그 위에 horizon 예측오차가
    또 쌓여 오차가 누적된다. 반면 판단/제어가 필요로 하는 건 "지금 이 순간의
    위치를 더 자주 알려달라"는 것뿐이므로, 별도 고rate 스트림으로 분리하는 게
    안전하다 (eval_extrapolation_rate_gain.py로 이 분리 설계 기준 외삽오차를
    사전 검증함 - docs/tp_final_report.md 4/4-1절).

외삽 방법: CTRV 롤아웃 (모델은 CTRV로 고정 - 2026-07-30 CTRV vs CTRA 결정과 동일).
    실측 메시지가 들어올 때마다 각 객체의 (x, y, heading, v, omega)를 갱신해
    "마지막으로 확인된 운동 상태"로 저장해두고, 그 다음 실측이 올 때까지
    Timer(주기 1/target_hz)가 매 tick마다 그 상태를 기준으로
    dt=(now - 마지막 실측 시각)만큼 CTRV로 외삽해 재발행한다.
    omega 추정은 prediction_node.py의 _estimate_yaw_rate와 동일한 방식
    (직전 실측 프레임과 최근접 위치 매칭 후 유한차분)을 그대로 사용한다.

안전장치: 마지막 실측 이후 ~max_extrapolation_gap(기본 0.3s)를 넘으면 그
객체는 외삽을 멈추고 발행 대상에서 제외한다(추적이 끊겼는데 계속 미래로
외삽하며 발행하면 위험하기 때문).

사용법(파라미터):
    ~input_topic (기본 /tracking/objects)
    ~output_topic (기본 /tracking/objects_highrate)
    ~target_hz (기본 30.0 - 판단팀 요구범위 20~50 중 권장값)
    ~max_extrapolation_gap (기본 0.3초)
"""

import math

import rospy
from tracking_prediction.msg import TrackedObject, TrackedObjectArray

OMEGA_EPS = 1e-3
MATCH_DIST_THRESHOLD = 3.0


def _project_to_scalar(vec2_like, heading):
    return vec2_like.x * math.cos(heading) + vec2_like.y * math.sin(heading)


def ctrv_predict(x, y, theta, v, omega, dt):
    """prediction_node.py의 ctrv_predict와 동일 (외삽도 같은 CTRV 모델로 통일)."""
    if abs(omega) < OMEGA_EPS:
        return x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), theta
    theta_next = theta + omega * dt
    x_next = x + (v / omega) * (math.sin(theta_next) - math.sin(theta))
    y_next = y + (v / omega) * (math.cos(theta) - math.cos(theta_next))
    return x_next, y_next, theta_next


class ExtrapolationNode(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/tracking/objects")
        output_topic = rospy.get_param("~output_topic", "/tracking/objects_highrate")
        self._target_hz = float(rospy.get_param("~target_hz", 30.0))
        self._max_gap = float(rospy.get_param("~max_extrapolation_gap", 0.3))

        self._pub = rospy.Publisher(output_topic, TrackedObjectArray, queue_size=10)
        self._sub = rospy.Subscriber(
            input_topic, TrackedObjectArray, self._on_tracking, queue_size=10
        )

        # track별 최신 운동상태: {track_id: dict(x,y,theta,v,omega,class_label,
        #                                         velocity, acceleration, size, t)}
        self._states = {}
        self._frame_id = ""
        # yaw_rate 추정용 직전 프레임 (class_label별 (x,y,heading) 목록) - prediction_node와 동일 방식
        self._prev_by_class = {}
        self._prev_stamp = None

        period = 1.0 / self._target_hz
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "[extrapolation_node] %s -> %s (target_hz=%.1f, max_extrapolation_gap=%.2fs)",
            input_topic, output_topic, self._target_hz, self._max_gap,
        )

    def _estimate_yaw_rate(self, obj, dt, prev_list):
        if dt <= 0 or not prev_list:
            return 0.0
        best, best_dist = None, MATCH_DIST_THRESHOLD
        for (px, py, pheading) in prev_list:
            dist = math.hypot(obj.position.x - px, obj.position.y - py)
            if dist < best_dist:
                best_dist, best = dist, pheading
        if best is None:
            return 0.0
        dtheta = obj.heading - best
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
        return dtheta / dt

    def _on_tracking(self, msg):
        now_stamp = msg.header.stamp.to_sec()
        dt = (now_stamp - self._prev_stamp) if self._prev_stamp is not None else 0.0
        self._frame_id = msg.header.frame_id

        next_prev_by_class = {}
        for obj in msg.objects:
            prev_list = self._prev_by_class.get(obj.class_label, [])
            omega = self._estimate_yaw_rate(obj, dt, prev_list)

            v = math.hypot(obj.velocity.x, obj.velocity.y)
            if _project_to_scalar(obj.velocity, obj.heading) < 0:
                v = -v

            self._states[obj.track_id] = {
                "x": obj.position.x, "y": obj.position.y, "z": obj.position.z,
                "theta": obj.heading, "v": v, "omega": omega,
                "class_label": obj.class_label,
                "velocity": obj.velocity, "acceleration": obj.acceleration,
                "size": obj.size, "t": now_stamp,
            }
            next_prev_by_class.setdefault(obj.class_label, []).append(
                (obj.position.x, obj.position.y, obj.heading)
            )

        # 이번 실측에 없던(사라진) track은 상태에서 제거해 계속 외삽하지 않게 함
        seen_ids = {obj.track_id for obj in msg.objects}
        for tid in list(self._states.keys()):
            if tid not in seen_ids:
                del self._states[tid]

        self._prev_by_class = next_prev_by_class
        self._prev_stamp = now_stamp

    def _on_timer(self, _event):
        if not self._states:
            return
        now = rospy.Time.now().to_sec()

        out = TrackedObjectArray()
        out.header.stamp = rospy.Time.from_sec(now)
        out.header.frame_id = self._frame_id

        objects = []
        for track_id, st in self._states.items():
            dt = now - st["t"]
            if dt < 0:
                dt = 0.0
            if dt > self._max_gap:
                # 실측이 너무 오래 끊김 - 이 track은 이번 tick에서 발행하지 않음
                continue
            x, y, theta = ctrv_predict(st["x"], st["y"], st["theta"], st["v"], st["omega"], dt)

            obj = TrackedObject()
            obj.track_id = track_id
            obj.class_label = st["class_label"]
            obj.position.x, obj.position.y, obj.position.z = x, y, st["z"]
            obj.heading = theta
            obj.velocity = st["velocity"]
            obj.acceleration = st["acceleration"]
            obj.size = st["size"]
            objects.append(obj)

        out.objects = objects
        self._pub.publish(out)


def main():
    rospy.init_node("extrapolation_node")
    ExtrapolationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
