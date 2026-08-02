#!/usr/bin/env python3
"""
eval_omega_clip_sensitivity.py

배경 (2026-08-01, task #1 방향 전환): lanelet map 중심선으로 궤적을 구속해서
heavy-tail을 줄이는 방안(map 기반 확장)은 모델이 무거워진다는 이유로 보류하고,
map 없이 순수 CTRV/CTRA 파라미터 튜닝만으로 horizon을 더 늘릴 수 있는지 먼저 시도함.

가설: `eval_omega_eps_sensitivity.py`는 "|yaw_rate|가 너무 작을 때"(noise를
회전으로 오판하는 하한 문제)만 다뤘고, 실제로는 반대 방향 문제도 있을 수 있다 -
직전/직후 프레임 간 heading을 유한차분으로 추정할 때 검출 노이즈 때문에 순간적으로
비현실적으로 큰 yaw_rate가 튀어나오면, CTRV의 (v/yaw_rate) 항이 실제로는 거의
직진인 차량을 급격한 원호로 예측해버려 heavy-tail 오차의 원인이 될 수 있다.
이 스크립트는 yaw_rate의 절대값에 물리적으로 타당한 상한(OMEGA_MAX_CLIP)을 씌워서
(그 이상은 clip) tail(p90/p95/p99)이 개선되는지, 그리고 실제 회전 구간(로터리
급커브)의 typical 오차가 나빠지는 부작용이 있는지를 확인한다.

`eval_omega_eps_sensitivity.py`(하한 sweep)와 대칭 관계 - 하한은 이미 무효 판정
(가설 기각) 났으므로, 이번엔 상한을 sweep한다.

목표: 로터리 0.3s / 하이웨이 1.7s인 현재 "3.0m 이내 90%" 안전마진 horizon을
map 없이 이 clip만으로 더 늘릴 수 있는지 확인 (docs/tp_final_report.md 2-3절
기준과 동일한 방식으로 비교).

사용법:
    python3 eval_omega_clip_sensitivity.py <bag_path> --label highway
    python3 eval_omega_clip_sensitivity.py <bag_path> --label roundabout
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
OMEGA_EPS = 1e-4  # 하한은 기존 기본값 그대로 유지 (이미 검증된 값)

TEST_HORIZONS = [0.3, 0.5, 1.0, 1.5, 1.7, 2.0, 3.0]
SAFETY_THRESHOLD_M = 3.0
SAFETY_PCTL = 90

# 상한 후보(rad/s). None = clip 없음(현재 기본 동작).
# 참고 스케일: 0.5 rad/s ~= 28.6 deg/s (일반 승용차 급커브에서도 흔치 않은 수준),
# 0.2 rad/s ~= 11.5 deg/s, 0.1 rad/s ~= 5.7 deg/s.
OMEGA_MAX_CANDIDATES = [None, 0.5, 0.3, 0.2, 0.15, 0.1, 0.05]


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt, omega_eps, omega_max_clip):
    if omega_max_clip is not None:
        if yaw_rate > omega_max_clip:
            yaw_rate = omega_max_clip
        elif yaw_rate < -omega_max_clip:
            yaw_rate = -omega_max_clip
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


def evaluate(series, gt_lookup, horizons, omega_max_clip, od_hz, pos_std, seed=42):
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
                px, py, _ = ctrv_predict(
                    cur["x"], cur["y"], cur["heading"], v, yaw_rate, h, OMEGA_EPS, omega_max_clip
                )
                gt_future = _nearest_record(gt_records, cur["t"] + h, max_dt=0.3)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def _find_safety_horizon(errors_by_horizon, horizons, threshold_m, pctl):
    """threshold_m 이내를 pctl% 확률로 만족하는 가장 긴 horizon (짧은 순 -> 첫 위반 직전)."""
    best = None
    for h in sorted(horizons):
        vals = errors_by_horizon.get(h, [])
        if not vals:
            continue
        p = np.percentile(np.array(vals), pctl)
        if p <= threshold_m:
            best = h
        else:
            break
    return best


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

    print("\n" + "=" * 110)
    print(f"[{args.label}] omega 상한 clip(rad/s)별 tail 변화 (OD {OD_HZ:.0f}Hz + 검출오차 sigma={pos_std:.3f}m)")
    print("=" * 110)
    header = f"{'omega_max':>10} {'horizon':>8} {'n':>7} {'mean':>8} {'p50':>8} {'p90':>8} {'p95':>8} {'p99':>8} {'max':>9}"
    print(header)

    summary_rows = []
    for omega_max in OMEGA_MAX_CANDIDATES:
        errors_by_horizon = evaluate(series, gt_lookup, TEST_HORIZONS, omega_max, OD_HZ, pos_std, seed=args.seed)
        label_val = "clip없음" if omega_max is None else f"{omega_max:.2f}"
        for h in TEST_HORIZONS:
            vals = errors_by_horizon[h]
            if not vals:
                continue
            arr = np.array(vals)
            print(
                f"{label_val:>10} {h:>8.1f} {len(arr):>7d} {arr.mean():>8.3f} "
                f"{np.percentile(arr,50):>8.3f} {np.percentile(arr,90):>8.3f} "
                f"{np.percentile(arr,95):>8.3f} {np.percentile(arr,99):>8.3f} {arr.max():>9.3f}"
            )
        print("-" * 110)
        safety_h = _find_safety_horizon(errors_by_horizon, TEST_HORIZONS, SAFETY_THRESHOLD_M, SAFETY_PCTL)
        summary_rows.append((label_val, safety_h))

    print("=" * 110)
    print(f"[{args.label}] omega 상한 clip별 '{SAFETY_THRESHOLD_M}m 이내 {SAFETY_PCTL}%' 안전마진 horizon (map 없이 CTRV만으로 확장 가능한지 확인)")
    print("=" * 110)
    for label_val, safety_h in summary_rows:
        print(f"  omega_max={label_val:>8} -> safety_horizon = {safety_h if safety_h is not None else '0.3s 미만(전 구간 실패)'}")

    print("\n판단 기준:")
    print("  - clip을 걸었을 때 safety_horizon이 늘어나면: 가설 지지 (노이즈성 큰 yaw_rate가 heavy-tail 원인 중 하나)")
    print("  - 전혀 안 늘어나면: 이 방향도 기각, map 기반 접근이나 planner 쪽 처리로 넘겨야 함")
    print("  - mean/p50(전형적인 경우)이 clip 때문에 나빠지면: 진짜 급회전 구간까지 깎아버리는 부작용 - 부작용과 tail 개선을 같이 보고 판단")


if __name__ == "__main__":
    main()
