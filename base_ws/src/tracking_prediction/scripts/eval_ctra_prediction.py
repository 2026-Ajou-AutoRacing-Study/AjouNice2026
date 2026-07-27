#!/usr/bin/env python3
"""
eval_ctra_prediction.py

CTRA 예측이 실제로 얼마나 정확한지, rosbag에 녹화된 MORAI GT(/Object_topic)만으로
오프라인으로 검증하는 스크립트. 실시간 노드(tracking_node/prediction_node)는
건드리지 않고, 같은 ctra_predict() 함수를 그대로 가져다 쓴다.

핵심 아이디어:
    - GT의 ObjectStatus.unique_id는 MORAI가 주는 "진짜" 지속 ID이므로
      (dummy tracking과 달리) 여기서는 그대로 사용해서 시간축을 따라
      같은 객체를 정확히 추적한다. (tracking_node가 dummy ID를 쓰는 건
      "실제 검출기는 ID가 없다"를 파이프라인 검증용으로 흉내내기 위함이고,
      이 평가 스크립트는 순수하게 CTRA 수식 자체의 정확도만 재는 것이 목적이라
      GT의 진짜 ID를 쓰는 게 맞다.)
    - 각 시점 t에서 그 객체의 (x,y,heading,v,a,omega)를 가지고 t+horizon 시점을
      예측한 뒤, 실제 t+horizon 시점 GT 위치와 비교해서 오차(m)를 계산한다.
    - omega(yaw rate)는 GT에 없으므로 같은 unique_id의 연속된 두 프레임의
      heading 차이로 유한차분 추정한다 (실제 ID가 있으므로 tracking_node의
      최근접 매칭 같은 우회가 필요 없음).
    - 회전 여부(|omega| 임계값 초과)로 구간을 나눠서 "직진 vs 회전(교차로/로터리
      추정 구간)"의 오차를 따로 리포트한다. 좌표 기반 지오펜스가 없어도
      이 기준이 "회전 중 CTRA가 얼마나 나빠지는지"를 실질적으로 대변한다.

사용법:
    python3 eval_ctra_prediction.py <bag_path> \
        --horizons 0.5 1.0 1.5 2.0 3.0 \
        --turn-threshold-deg-per-sec 5.0
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

# prediction_node.py와 동일 로직 (모듈로 분리해서 재사용하는 게 이상적이나,
# 지금은 화요일 마감 우선순위상 함수만 복사해서 독립 실행 가능하게 둠.
# 나중에 공통 유틸 모듈로 뽑아내는 걸 권장)
OMEGA_EPS = 1e-3
MORAI_HEADING_IN_DEGREES = True


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctra_predict(x, y, theta, v, omega, a, dt):
    if abs(omega) < OMEGA_EPS:
        dist = v * dt + 0.5 * a * dt * dt
        x_next = x + dist * math.cos(theta)
        y_next = y + dist * math.sin(theta)
        theta_next = theta
        return x_next, y_next, theta_next

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    x_next = x + (
        (a * dt * sin_tn) / omega
        - (v * sin_t) / omega
        + (v * sin_tn) / omega
        - (a * cos_t) / (omega ** 2)
        + (a * cos_tn) / (omega ** 2)
    )
    y_next = y - (
        (a * dt * cos_tn) / omega
        - (v * cos_t) / omega
        + (v * cos_tn) / omega
        + (a * sin_t) / (omega ** 2)
        - (a * sin_tn) / (omega ** 2)
    )
    return x_next, y_next, theta_next


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _extract_object_series(bag_path, topic="/Object_topic"):
    """unique_id별로 (timestamp, x, y, heading, vx, vy, ax, ay) 시계열을 뽑는다."""
    series = defaultdict(list)

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for group, label in (
                (msg.npc_list, "vehicle"),
                (msg.pedestrian_list, "pedestrian"),
                (msg.obstacle_list, "obstacle"),
            ):
                KMH_TO_MS = 1.0 / 3.6
                for obj in group:
                    series[obj.unique_id].append(
                        {
                            "t": stamp,
                            "x": obj.position.x,
                            "y": obj.position.y,
                            "heading": _heading_to_rad(obj.heading),
                            # MORAI ObjectStatus.velocity는 km/h로 발행됨 (실측 확인) -> m/s 변환
                            "vx": obj.velocity.x * KMH_TO_MS,
                            "vy": obj.velocity.y * KMH_TO_MS,
                            "ax": obj.acceleration.x,
                            "ay": obj.acceleration.y,
                            "class": label,
                        }
                    )

    for uid in series:
        series[uid].sort(key=lambda r: r["t"])

    return series


def _nearest_record(records, target_t, max_dt=0.2):
    """target_t에 가장 가까운 record를 찾는다 (max_dt 초과시 None)."""
    best = None
    best_diff = max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff = diff
            best = r
    return best


def evaluate(bag_path, horizons, turn_threshold_rad_per_sec):
    series = _extract_object_series(bag_path)

    # horizon별 오차 리스트, "직진"/"회전" 구간별로 분리해서 저장
    errors = {h: {"straight": [], "turn": []} for h in horizons}

    for uid, records in series.items():
        for i in range(1, len(records)):
            prev, cur = records[i - 1], records[i]
            dt = cur["t"] - prev["t"]
            if dt <= 0:
                continue

            dtheta = cur["heading"] - prev["heading"]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            omega = dtheta / dt

            v = math.hypot(cur["vx"], cur["vy"])
            if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
                v = -v
            a = _project_to_scalar(cur["ax"], cur["ay"], cur["heading"])

            bucket = "turn" if abs(omega) > turn_threshold_rad_per_sec else "straight"

            for horizon in horizons:
                px, py, _ = ctra_predict(
                    cur["x"], cur["y"], cur["heading"], v, omega, a, horizon
                )

                gt_future = _nearest_record(records, cur["t"] + horizon)
                if gt_future is None:
                    continue

                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors[horizon][bucket].append(err)

    return errors


def _print_report(errors, horizons):
    print("=" * 70)
    print("CTRA 예측 오차 평가 결과 (m, 위치오차)")
    print("=" * 70)
    for h in horizons:
        for bucket in ("straight", "turn"):
            vals = errors[h][bucket]
            if not vals:
                print(f"  horizon={h:>4}s  [{bucket:8s}]  샘플 없음")
                continue
            arr = np.array(vals)
            print(
                f"  horizon={h:>4}s  [{bucket:8s}]  n={len(arr):5d}  "
                f"mean={arr.mean():.3f}  median={np.median(arr):.3f}  "
                f"p95={np.percentile(arr, 95):.3f}  max={arr.max():.3f}"
            )
    print("=" * 70)
    print("주의: 'turn' 구간은 yaw_rate 임계값 초과 프레임 기준 (교차로/로터리 추정 구간)")
    print("      'straight'보다 'turn'에서 오차가 확연히 크면 map-based 도입을 검토")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="MORAI /Object_topic이 녹화된 rosbag 경로")
    parser.add_argument(
        "--horizons",
        type=float,
        nargs="+",
        default=[0.5, 1.0, 1.5, 2.0, 3.0],
        help="평가할 예측 시점들(초)",
    )
    parser.add_argument(
        "--turn-threshold-deg-per-sec",
        type=float,
        default=5.0,
        help="이 값(deg/s) 초과 yaw_rate면 '회전' 구간으로 분류",
    )
    args = parser.parse_args()

    turn_threshold_rad = math.radians(args.turn_threshold_deg_per_sec)
    errors = evaluate(args.bag_path, args.horizons, turn_threshold_rad)
    _print_report(errors, args.horizons)


if __name__ == "__main__":
    main()
