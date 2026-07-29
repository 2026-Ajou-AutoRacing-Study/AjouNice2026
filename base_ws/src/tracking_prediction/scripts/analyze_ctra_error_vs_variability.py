#!/usr/bin/env python3
"""
analyze_ctra_error_vs_variability.py

변지훈님 지적에 대한 후속 검증:
    "CTRA 성능은 회전/직진이 아니라, yaw_rate/가속도가 예측구간 내내
     일정한가(가정 성립 여부)에 달려있다."

기존 eval_ctra_prediction_v2.py는 예측 "시작 시점"의 yaw_rate가 임계값을
넘는지로 직진/회전을 이분류했다. 이 스크립트는 그 대신, 예측이 커버하는
horizon 구간 "동안" 실제 yaw_rate/가속도가 얼마나 변했는지(변동성)를
직접 재서 오차와의 상관관계를 본다.

핵심 지표 (각 예측 인스턴스마다 계산):
    - omega_variability: horizon 구간 동안 실제 GT yaw_rate의 표준편차
    - accel_variability : horizon 구간 동안 실제 GT 가속도(속도 유한차분)의 표준편차
    - error(FDE)         : 그 구간 마지막 시점(horizon)에서의 CTRA 예측 오차

이 세 값의 상관관계(피어슨 상관계수)를 계산하고 산점도를 그린다.
"회전/직진"이 아니라 "변동성 자체"가 오차와 강하게 상관되면,
변지훈님 가설이 정량적으로 뒷받침된다.

사용법:
    python3 analyze_ctra_error_vs_variability.py <bag_path> --label roundabout
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

OMEGA_EPS = 1e-3
MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6  # MORAI ObjectStatus.velocity는 km/h 단위 (실측 확인됨)


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
    series = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for obj in msg.npc_list:
                series[obj.unique_id].append(
                    {
                        "t": stamp,
                        "x": obj.position.x,
                        "y": obj.position.y,
                        "heading": _heading_to_rad(obj.heading),
                        "vx": obj.velocity.x * KMH_TO_MS,
                        "vy": obj.velocity.y * KMH_TO_MS,
                    }
                )
    for uid in series:
        series[uid].sort(key=lambda r: r["t"])
    return series


def _nearest_record(records, target_t, max_dt=0.2):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def _instantaneous_omega_v_a(records, i):
    """records[i] 시점의 순간 yaw_rate(omega), 속도(v), 가속도(a)를 유한차분으로 계산."""
    if i == 0 or i >= len(records):
        return None
    prev, cur = records[i - 1], records[i]
    dt = cur["t"] - prev["t"]
    if dt <= 0:
        return None

    dtheta = cur["heading"] - prev["heading"]
    dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
    omega = dtheta / dt

    v_cur = math.hypot(cur["vx"], cur["vy"])
    if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
        v_cur = -v_cur
    v_prev = math.hypot(prev["vx"], prev["vy"])
    if _project_to_scalar(prev["vx"], prev["vy"], prev["heading"]) < 0:
        v_prev = -v_prev
    a = (v_cur - v_prev) / dt

    return omega, v_cur, a


def analyze(bag_path, horizon):
    series = _extract_object_series(bag_path)

    omega_variability = []
    accel_variability = []
    errors = []

    for uid, records in series.items():
        for i in range(1, len(records)):
            iv = _instantaneous_omega_v_a(records, i)
            if iv is None:
                continue
            omega0, v0, a0 = iv
            cur = records[i]

            # CTRA로 horizon 뒤 예측
            px, py, _ = ctra_predict(
                cur["x"], cur["y"], cur["heading"], v0, omega0, a0, horizon
            )
            gt_future = _nearest_record(records, cur["t"] + horizon)
            if gt_future is None:
                continue
            error = math.hypot(px - gt_future["x"], py - gt_future["y"])

            # horizon 구간 동안 실제 omega/a가 얼마나 변했는지(표준편차) 측정
            window_omegas, window_accels = [], []
            for j in range(i, len(records)):
                if records[j]["t"] - cur["t"] > horizon:
                    break
                iv_j = _instantaneous_omega_v_a(records, j)
                if iv_j is None:
                    continue
                window_omegas.append(iv_j[0])
                window_accels.append(iv_j[2])

            if len(window_omegas) < 2:
                continue

            omega_variability.append(np.std(window_omegas))
            accel_variability.append(np.std(window_accels))
            errors.append(error)

    return np.array(omega_variability), np.array(accel_variability), np.array(errors)


def _print_and_plot(label, horizon, omega_var, accel_var, errors):
    print("=" * 70)
    print(f"[{label}] horizon={horizon}s, 표본수={len(errors)}")
    print("=" * 70)

    corr_omega = np.corrcoef(omega_var, errors)[0, 1]
    corr_accel = np.corrcoef(accel_var, errors)[0, 1]

    print(f"오차 vs yaw_rate 변동성(표준편차)  피어슨 상관계수: {corr_omega:.3f}")
    print(f"오차 vs 가속도 변동성(표준편차)    피어슨 상관계수: {corr_accel:.3f}")
    print("(1에 가까울수록 강한 양의 상관 = 변동성이 클수록 오차도 커진다는 뜻)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    kfont_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
    try:
        kfont = fm.FontProperties(fname=kfont_path)
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    ax = axes[0]
    ax.scatter(omega_var, errors, s=8, alpha=0.3, color="#2b6cb0")
    ax.set_xlabel("yaw_rate 변동성 (rad/s, std)", fontproperties=kfont)
    ax.set_ylabel(f"{horizon}s 뒤 예측 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 오차 vs yaw_rate 변동성 (r={corr_omega:.2f})", fontproperties=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.scatter(accel_var, errors, s=8, alpha=0.3, color="#c05621")
    ax.set_xlabel("가속도 변동성 (m/s², std)", fontproperties=kfont)
    ax.set_ylabel(f"{horizon}s 뒤 예측 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 오차 vs 가속도 변동성 (r={corr_accel:.2f})", fontproperties=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"variability_correlation_{label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"그래프 저장됨: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--horizon", type=float, default=2.0, help="분석할 예측 horizon(초), 기본 2.0s")
    args = parser.parse_args()

    omega_var, accel_var, errors = analyze(args.bag_path, args.horizon)
    _print_and_plot(args.label, args.horizon, omega_var, accel_var, errors)


if __name__ == "__main__":
    main()
