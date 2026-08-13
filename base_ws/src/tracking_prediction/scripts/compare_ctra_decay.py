#!/usr/bin/env python3
"""
compare_ctra_decay.py

기존 CTRA(가속도 a가 예측구간 내내 상수)와, Autoware map_based_prediction
방식의 "감쇠 가속도" CTRA(a(t) = a0 * exp(-lambda*t))를 같은 GT 데이터로
비교한다.

감쇠 모델 유도:
    v(t) = v0 + (a0/lambda) * (1 - exp(-lambda*t))
    (lambda -> 0 극한을 취하면 기존 등가속도 공식 v0 + a0*t로 정확히 환원됨,
     즉 lambda가 작을수록 "거의 등가속도"에 가까워짐 - sanity check로 확인함)

    직진(omega=0):
        distance(T) = v0*T + (a0/lambda) * (T - (1/lambda)*(1 - exp(-lambda*T)))

    회전(omega!=0): v(t)*cos(theta0+omega*t), v(t)*sin(...)를 각각 적분.
        상수항(v0) 부분은 기존 CTRA와 동일한 형태.
        감쇠항(a0/lambda 부분)은 지수함수*삼각함수 적분 공식을 사용:
        ∫exp(-λt)cos(θ0+ωt)dt, ∫exp(-λt)sin(θ0+ωt)dt

여러 lambda 후보값으로 테스트해서, 어떤 값이 기존 CTRA보다 FDE를
줄여주는지 확인한다.

사용법:
    python3 compare_ctra_decay.py <bag_path> --label roundabout --horizon 2.0
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

OMEGA_EPS = 1e-3
MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
SMOOTH_WINDOW = 5
LAMBDA_CANDIDATES = [0.2, 0.4, 0.5, 1.0, 2.0, 5.0]


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctra_predict_constant_a(x, y, theta, v, omega, a, dt):
    """기존 CTRA: 가속도가 예측구간 내내 일정하다고 가정."""
    if abs(omega) < OMEGA_EPS:
        dist = v * dt + 0.5 * a * dt * dt
        return x + dist * math.cos(theta), y + dist * math.sin(theta)

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    x_next = x + (
        (a * dt * sin_tn) / omega - (v * sin_t) / omega + (v * sin_tn) / omega
        - (a * cos_t) / (omega ** 2) + (a * cos_tn) / (omega ** 2)
    )
    y_next = y - (
        (a * dt * cos_tn) / omega - (v * cos_t) / omega + (v * cos_tn) / omega
        + (a * sin_t) / (omega ** 2) - (a * sin_tn) / (omega ** 2)
    )
    return x_next, y_next


def ctra_predict_decay_a(x, y, theta, v, omega, a, dt, lam):
    """감쇠가속도 CTRA: a(t) = a * exp(-lam*t)."""
    if abs(omega) < OMEGA_EPS:
        if lam < 1e-6:
            dist = v * dt + 0.5 * a * dt * dt
        else:
            dist = v * dt + (a / lam) * (dt - (1.0 / lam) * (1 - math.exp(-lam * dt)))
        return x + dist * math.cos(theta), y + dist * math.sin(theta)

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    # 상수항(v0) 부분: 기존 CTRA와 동일
    x_const = v * (sin_tn - sin_t) / omega
    y_const = v * (cos_t - cos_tn) / omega

    if lam < 1e-6:
        # lambda가 0에 가까우면 감쇠 없음 -> 기존 등가속도와 동일
        return ctra_predict_constant_a(x, y, theta, v, omega, a, dt)

    denom = lam * lam + omega * omega

    # I_c(T) = ∫0^T exp(-lam*t) cos(theta0+omega*t) dt
    ic = (
        math.exp(-lam * dt) * (-lam * cos_tn + omega * sin_tn)
        - (-lam * cos_t + omega * sin_t)
    ) / denom
    # I_s(T) = ∫0^T exp(-lam*t) sin(theta0+omega*t) dt
    is_ = (
        math.exp(-lam * dt) * (-lam * sin_tn - omega * cos_tn)
        - (-lam * sin_t - omega * cos_t)
    ) / denom

    # 감쇠항(a0/lam) 부분
    x_decay = (a / lam) * ((sin_tn - sin_t) / omega - ic)
    y_decay = (a / lam) * ((cos_t - cos_tn) / omega - is_)

    return x + x_const + x_decay, y + y_const + y_decay


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
    raw = [_raw_omega_v_a(records, i) for i in range(len(records))]
    smoothed = [None] * len(records)
    half = SMOOTH_WINDOW // 2
    for i in range(len(records)):
        window = [raw[j] for j in range(max(0, i - half), min(len(records), i + half + 1)) if raw[j] is not None]
        if not window:
            continue
        smoothed[i] = (np.mean([w[0] for w in window]), np.mean([w[1] for w in window]), np.mean([w[2] for w in window]))
    return smoothed


def analyze(bag_path, horizon):
    series = _extract_object_series(bag_path)

    errors_const = []
    errors_decay = {lam: [] for lam in LAMBDA_CANDIDATES}

    for uid, records in series.items():
        smoothed = _build_smoothed_series(records)
        for i in range(1, len(records)):
            if smoothed[i] is None:
                continue
            omega0, v0, a0 = smoothed[i]
            cur = records[i]
            gt_future = _nearest_record(records, cur["t"] + horizon)
            if gt_future is None:
                continue

            px, py = ctra_predict_constant_a(cur["x"], cur["y"], cur["heading"], v0, omega0, a0, horizon)
            errors_const.append(math.hypot(px - gt_future["x"], py - gt_future["y"]))

            for lam in LAMBDA_CANDIDATES:
                px_d, py_d = ctra_predict_decay_a(cur["x"], cur["y"], cur["heading"], v0, omega0, a0, horizon, lam)
                errors_decay[lam].append(math.hypot(px_d - gt_future["x"], py_d - gt_future["y"]))

    return np.array(errors_const), {lam: np.array(v) for lam, v in errors_decay.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--horizon", type=float, default=2.0)
    args = parser.parse_args()

    errors_const, errors_decay = analyze(args.bag_path, args.horizon)

    print("=" * 70)
    print(f"[{args.label}] horizon={args.horizon}s, 표본수={len(errors_const)}")
    print("=" * 70)
    print(f"기존 CTRA(등가속도)          FDE mean={errors_const.mean():.3f}  median={np.median(errors_const):.3f}")
    for lam, errs in errors_decay.items():
        improve = (errors_const.mean() - errs.mean()) / errors_const.mean() * 100
        print(f"감쇠 CTRA(lambda={lam:>4})     FDE mean={errs.mean():.3f}  median={np.median(errs):.3f}  (개선율 {improve:+.1f}%)")
    print("=" * 70)


if __name__ == "__main__":
    main()
