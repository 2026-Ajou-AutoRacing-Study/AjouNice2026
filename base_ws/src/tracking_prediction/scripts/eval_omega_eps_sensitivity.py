#!/usr/bin/env python3
"""
eval_omega_eps_sensitivity.py

가설: 로터리(analyze_jitter_outliers.py)와 하이웨이(analyze_jitter_outliers.py --label highway)
양쪽에서 발견된 극단치 오차는, 로터리 급커브 같은 "실제로 크게 도는" 상황이 아니라
"속도(v)는 큰데 추정된 yaw_rate가 작게(노이즈 수준으로) 나온" 상황에서 CTRV의
(v/yaw_rate) 항이 지나치게 커지는 게 원인일 수 있다.

현재 코드(ctrv_predict)는 |yaw_rate| < 1e-4 rad/s(약 0.006deg/s)일 때만 직진 공식으로
전환하는데, 실제 이상치들의 yaw_rate는 2~4.5deg/s(약 0.035~0.079 rad/s) 수준으로 이
임계값보다 훨씬 커서 회전 공식이 그대로 쓰인다. 이 스크립트는 그 임계값(OMEGA_EPS)을
훨씬 크게 올렸을 때(예: 0.05~0.15 rad/s) tail(p90/p95/p99)이 실제로 개선되는지,
그리고 그 대가로 typical(median)이 나빠지는지를 직접 비교해서 검증한다.

비교 대상 horizon: 2.0s, 3.0s (tail이 가장 크게 벌어지는 구간)

사용법:
    python3 eval_omega_eps_sensitivity.py <bag_path> --label highway
"""

import argparse
import math
import random
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
TELEPORT_THRESHOLD_M = 15.0

OD_HZ = 10.0
TP_CENTER_ERROR_MEAN = 0.145

TEST_HORIZONS = [1.0, 2.0, 3.0]
OMEGA_EPS_CANDIDATES = [1e-4, 0.02, 0.05, 0.08, 0.10, 0.15]  # rad/s


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt, omega_eps):
    if abs(yaw_rate) < omega_eps:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + yaw_rate * dt
    yaw_next = yaw + yaw_rate * dt
    x_next = x + (v / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
    y_next = y + (v / yaw_rate) * (math.cos(yaw) - math.cos(yaw_next))
    return x_next, y_next, yaw_next


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _extract_full_rate_series(bag_path, exclude_ids, topic="/Object_topic"):
    series = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for obj in msg.npc_list:
                if obj.unique_id in exclude_ids:
                    continue
                series[obj.unique_id].append(
                    {
                        "t": stamp, "x": obj.position.x, "y": obj.position.y,
                        "heading": _heading_to_rad(obj.heading),
                        "vx": obj.velocity.x * KMH_TO_MS, "vy": obj.velocity.y * KMH_TO_MS,
                    }
                )
    for uid in series:
        series[uid].sort(key=lambda r: r["t"])
    return _split_teleport_segments(series)


def _split_teleport_segments(series):
    new_series = {}
    for uid, records in series.items():
        if not records:
            continue
        seg_idx = 0
        cur_seg = [records[0]]
        for i in range(1, len(records)):
            dist = math.hypot(records[i]["x"] - records[i - 1]["x"], records[i]["y"] - records[i - 1]["y"])
            if dist > TELEPORT_THRESHOLD_M:
                new_series[f"{uid}_{seg_idx}"] = cur_seg
                seg_idx += 1
                cur_seg = []
            cur_seg.append(records[i])
        if cur_seg:
            new_series[f"{uid}_{seg_idx}"] = cur_seg
    return new_series


def _nearest_record(records, target_t, max_dt=0.3):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def _downsample(records, target_hz):
    if not records:
        return []
    interval = 1.0 / target_hz
    sampled = [records[0]]
    last_t = records[0]["t"]
    for r in records[1:]:
        if r["t"] - last_t >= interval:
            sampled.append(r)
            last_t = r["t"]
    return sampled


def _apply_detection_noise(records, pos_std, rng):
    noisy = []
    for r in records:
        n = dict(r)
        n["x"] = r["x"] + rng.gauss(0, pos_std)
        n["y"] = r["y"] + rng.gauss(0, pos_std)
        noisy.append(n)
    return noisy


def _instantaneous_v_yawrate(prev, cur):
    dt = cur["t"] - prev["t"]
    if dt <= 0:
        return None
    dtheta = cur["heading"] - prev["heading"]
    dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
    yaw_rate = dtheta / dt
    v = math.hypot(cur["vx"], cur["vy"])
    if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
        v = -v
    return v, yaw_rate


def evaluate(series, gt_lookup, horizons, omega_eps, od_hz, pos_std, seed=42):
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in horizons}

    for uid, full_records in series.items():
        gt_records = gt_lookup[uid]
        working = _downsample(full_records, od_hz)
        working = _apply_detection_noise(working, pos_std, rng)
        if len(working) < 2:
            continue

        for i in range(1, len(working)):
            iv = _instantaneous_v_yawrate(working[i - 1], working[i])
            if iv is None:
                continue
            v, yaw_rate = iv
            cur = working[i]
            for h in horizons:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h, omega_eps)
                gt_future = _nearest_record(gt_records, cur["t"] + h, max_dt=0.3)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"[{args.label}] 평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    pos_std = TP_CENTER_ERROR_MEAN / math.sqrt(math.pi / 2.0)
    gt_lookup = series

    print("\n" + "=" * 100)
    print(f"[{args.label}] OMEGA_EPS(rad/s)별 tail 변화 (OD {OD_HZ:.0f}Hz + 검출오차 sigma={pos_std:.3f}m)")
    print("=" * 100)
    header = f"{'omega_eps':>10} {'horizon':>8} {'n':>7} {'mean':>8} {'p50':>8} {'p90':>8} {'p95':>8} {'p99':>8} {'max':>9}"
    print(header)

    for omega_eps in OMEGA_EPS_CANDIDATES:
        errors_by_horizon = evaluate(series, gt_lookup, TEST_HORIZONS, omega_eps, OD_HZ, pos_std, seed=args.seed)
        for h in TEST_HORIZONS:
            vals = errors_by_horizon[h]
            if not vals:
                continue
            arr = np.array(vals)
            print(
                f"{omega_eps:>10.4f} {h:>8.1f} {len(arr):>7d} {arr.mean():>8.3f} "
                f"{np.percentile(arr,50):>8.3f} {np.percentile(arr,90):>8.3f} "
                f"{np.percentile(arr,95):>8.3f} {np.percentile(arr,99):>8.3f} {arr.max():>9.3f}"
            )
        print("-" * 100)

    print("=" * 100)
    print("omega_eps=0.0001(=현재 기본값)과 비교해서, 값을 올렸을 때")
    print("  - p95/p99/max(최악의 경우)가 줄어들면: 가설 지지 (작은 omega 노이즈 증폭이 원인)")
    print("  - mean/p50(전형적인 경우)이 크게 나빠지면: 부작용 (실제 회전 상황까지 직진으로 오판)")
    print("두 효과를 같이 보고 적정 omega_eps를 판단할 것.")


if __name__ == "__main__":
    main()
