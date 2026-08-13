#!/usr/bin/env python3
"""
analyze_ctra_error_vs_variability_v2.py

v1(analyze_ctra_error_vs_variability.py) 대비 개선점:
    1. omega/a를 raw 유한차분 그대로 안 쓰고, 이동평균(window=SMOOTH_WINDOW)으로
       스무딩해서 미분 노이즈를 줄인다. (v1은 노이즈가 변동성 지표를 오염시켰을
       가능성이 높았음)
    2. "horizon 구간 전체의 표준편차" 대신, CTRA가 실제로 가정하는 것과 더
       정확히 일치하는 지표인 "t0 시점 값(omega0, a0) 대비 평균절대편차
       (mean absolute deviation from t0)"로 교체.
       -> CTRA는 "t0 값이 계속 유지된다"고 가정하므로, "t0 대비 얼마나
          벗어났는가"가 "가정이 깨졌는가"를 더 직접적으로 나타낸다.

사용법:
    python3 analyze_ctra_error_vs_variability_v2.py <bag_path> --label roundabout --horizon 2.0
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

OMEGA_EPS = 1e-3
MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
SMOOTH_WINDOW = 5  # 이동평균 창 크기 (프레임 수)


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctra_predict(x, y, theta, v, omega, a, dt):
    if abs(omega) < OMEGA_EPS:
        dist = v * dt + 0.5 * a * dt * dt
        x_next = x + dist * math.cos(theta)
        y_next = y + dist * math.sin(theta)
        return x_next, y_next, theta

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


def _raw_omega_v_a(records, i):
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


def _build_smoothed_series(records):
    """각 프레임의 (omega, v, a)를 raw로 구한 뒤, 이동평균으로 스무딩해서 반환."""
    raw = [None] * len(records)
    for i in range(len(records)):
        raw[i] = _raw_omega_v_a(records, i)

    smoothed = [None] * len(records)
    half = SMOOTH_WINDOW // 2
    for i in range(len(records)):
        window = [raw[j] for j in range(max(0, i - half), min(len(records), i + half + 1)) if raw[j] is not None]
        if not window:
            continue
        omega_s = np.mean([w[0] for w in window])
        v_s = np.mean([w[1] for w in window])
        a_s = np.mean([w[2] for w in window])
        smoothed[i] = (omega_s, v_s, a_s)

    return smoothed


def analyze(bag_path, horizon):
    series = _extract_object_series(bag_path)

    omega_deviation = []
    accel_deviation = []
    errors = []

    for uid, records in series.items():
        smoothed = _build_smoothed_series(records)

        for i in range(1, len(records)):
            if smoothed[i] is None:
                continue
            omega0, v0, a0 = smoothed[i]
            cur = records[i]

            px, py, _ = ctra_predict(
                cur["x"], cur["y"], cur["heading"], v0, omega0, a0, horizon
            )
            gt_future = _nearest_record(records, cur["t"] + horizon)
            if gt_future is None:
                continue
            error = math.hypot(px - gt_future["x"], py - gt_future["y"])

            # horizon 구간 동안, "t0 값(omega0, a0) 대비" 스무딩된 실제값이
            # 얼마나 벗어났는지(평균절대편차) 측정
            omega_devs, accel_devs = [], []
            for j in range(i, len(records)):
                if records[j]["t"] - cur["t"] > horizon:
                    break
                if smoothed[j] is None:
                    continue
                omega_devs.append(abs(smoothed[j][0] - omega0))
                accel_devs.append(abs(smoothed[j][2] - a0))

            if len(omega_devs) < 2:
                continue

            omega_deviation.append(np.mean(omega_devs))
            accel_deviation.append(np.mean(accel_devs))
            errors.append(error)

    return np.array(omega_deviation), np.array(accel_deviation), np.array(errors)


def _print_and_plot(label, horizon, omega_dev, accel_dev, errors):
    print("=" * 70)
    print(f"[{label}] horizon={horizon}s, 표본수={len(errors)} (스무딩창={SMOOTH_WINDOW})")
    print("=" * 70)

    corr_omega = np.corrcoef(omega_dev, errors)[0, 1]
    corr_accel = np.corrcoef(accel_dev, errors)[0, 1]

    print(f"오차 vs yaw_rate의 t0대비 평균편차   피어슨 상관계수: {corr_omega:.3f}")
    print(f"오차 vs 가속도의 t0대비 평균편차     피어슨 상관계수: {corr_accel:.3f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    ax = axes[0]
    ax.scatter(omega_dev, errors, s=8, alpha=0.3, color="#2b6cb0")
    ax.set_xlabel("yaw_rate의 t0 대비 평균편차 (rad/s)", fontproperties=kfont)
    ax.set_ylabel(f"{horizon}s 뒤 예측 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 오차 vs yaw_rate 이탈도 (r={corr_omega:.2f})", fontproperties=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.scatter(accel_dev, errors, s=8, alpha=0.3, color="#c05621")
    ax.set_xlabel("가속도의 t0 대비 평균편차 (m/s²)", fontproperties=kfont)
    ax.set_ylabel(f"{horizon}s 뒤 예측 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 오차 vs 가속도 이탈도 (r={corr_accel:.2f})", fontproperties=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"deviation_correlation_{label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"그래프 저장됨: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--horizon", type=float, default=2.0)
    args = parser.parse_args()

    omega_dev, accel_dev, errors = analyze(args.bag_path, args.horizon)
    _print_and_plot(args.label, args.horizon, omega_dev, accel_dev, errors)


if __name__ == "__main__":
    main()
