#!/usr/bin/env python3
"""
eval_fine_horizon.py

CTRV 예측을 0.1초 간격으로 0.1~3.0초까지 촘촘하게 평가한다.
목표:
    1. horizon이 늘어날수록 오차가 어떻게 "퍼지는지" 곡선으로 확인
    2. 오차가 임계값(threshold)을 넘는 지점을 찾아서 "권장 최대 horizon"을 제안
    3. 지터(jitter): 같은 절대 시각에 대한 예측이, 예측을 만든 시점(t0)이
       달라짐에 따라 얼마나 흔들리는지 측정
       - 예: t=10.0s 시점의 위치를, t0=9.0s일 때(1.0s짜리 예측)와
         t0=9.5s일 때(0.5s짜리 예측) 각각 예측했을 때 그 두 값이 서로
         얼마나 다른지 -> 이게 크면 "예측선이 부들거린다"는 뜻

--exclude-ids로 하이웨이쪽(로터리 무관) 차량 제외 가능
  (find_roundabout_and_filter.py로 뽑은 목록을 그대로 넣으면 됨)

사용법:
    python3 eval_fine_horizon.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40 \
        --error-threshold 3.0
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]  # 0.1, 0.2, ..., 3.0


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt):
    """CTRV: 가속도 없이 등속+등yaw_rate로 dt초 뒤 위치 예측."""
    if abs(yaw_rate) < 1e-4:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + yaw_rate * dt
    yaw_next = yaw + yaw_rate * dt
    x_next = x + (v / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
    y_next = y + (v / yaw_rate) * (math.cos(yaw) - math.cos(yaw_next))
    return x_next, y_next, yaw_next


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _extract_object_series(bag_path, exclude_ids, topic="/Object_topic"):
    series = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for obj in msg.npc_list:
                if obj.unique_id in exclude_ids:
                    continue
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


def _instantaneous_v_yawrate(records, i):
    if i == 0 or i >= len(records):
        return None
    prev, cur = records[i - 1], records[i]
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


def evaluate_fine_horizon(series):
    """각 horizon마다 오차 리스트 계산 (ADE/FDE용 촘촘한 곡선)."""
    errors_by_horizon = {h: [] for h in HORIZONS}

    for uid, records in series.items():
        for i in range(1, len(records)):
            iv = _instantaneous_v_yawrate(records, i)
            if iv is None:
                continue
            v, yaw_rate = iv
            cur = records[i]

            for h in HORIZONS:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h)
                gt_future = _nearest_record(records, cur["t"] + h)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def evaluate_jitter(series):
    """
    지터: 같은 절대 시각 T를 서로 다른 t0(예측 시작점)에서 예측했을 때
    (예: t0=T-0.5s일 때와 t0=T-1.0s일 때) 그 예측 위치가 얼마나 다른지.

    구현: 각 시점 i에서 1.0초 뒤 예측 위치(t0=cur.t 기준)와, 그로부터 0.5초
    지난 시점 i'(cur.t+0.5초에 가장 가까운 프레임)에서 0.5초 뒤 예측 위치
    (같은 절대시각 T=cur.t+1.0s를 가리킴)를 비교한다.
    """
    jitter_errors = []

    for uid, records in series.items():
        for i in range(1, len(records)):
            iv = _instantaneous_v_yawrate(records, i)
            if iv is None:
                continue
            v0, yawrate0 = iv
            cur = records[i]
            target_t = cur["t"] + 1.0

            # 예측 A: t0=cur.t 기준으로 1.0초 뒤 예측
            pxA, pyA, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v0, yawrate0, 1.0)

            # 예측 B: 0.5초 뒤 시점(cur.t+0.5)에서, 같은 target_t까지 0.5초 예측
            later = _nearest_record(records, cur["t"] + 0.5, max_dt=0.1)
            if later is None:
                continue
            later_idx = records.index(later)
            iv_later = _instantaneous_v_yawrate(records, later_idx)
            if iv_later is None:
                continue
            v1, yawrate1 = iv_later
            pxB, pyB, _ = ctrv_predict(later["x"], later["y"], later["heading"], v1, yawrate1, 0.5)

            jitter = math.hypot(pxA - pxB, pyA - pyB)
            jitter_errors.append(jitter)

    return np.array(jitter_errors)


def find_recommended_horizon(errors_by_horizon, threshold):
    """평균 오차가 threshold(m)를 처음 넘는 horizon을 찾는다."""
    for h in HORIZONS:
        vals = errors_by_horizon[h]
        if not vals:
            continue
        if np.mean(vals) > threshold:
            return h
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="", help="쉼표로 구분된 제외할 unique_id 목록")
    parser.add_argument("--error-threshold", type=float, default=3.0, help="권장 horizon 판단 기준(m)")
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    if exclude_ids:
        print(f"제외된 unique_id: {sorted(exclude_ids)}")

    series = _extract_object_series(args.bag_path, exclude_ids)
    print(f"평가 대상 unique_id 수: {len(series)}")

    errors_by_horizon = evaluate_fine_horizon(series)

    print("\n" + "=" * 70)
    print(f"[{args.label}] horizon별 오차 (0.1s 간격)")
    print("=" * 70)
    for h in HORIZONS:
        vals = errors_by_horizon[h]
        if not vals:
            continue
        arr = np.array(vals)
        print(f"  horizon={h:>4.1f}s  n={len(arr):5d}  mean={arr.mean():.3f}  median={np.median(arr):.3f}  p95={np.percentile(arr,95):.3f}")

    recommended = find_recommended_horizon(errors_by_horizon, args.error_threshold)
    print("\n" + "=" * 70)
    if recommended is not None:
        print(f"평균 오차가 {args.error_threshold}m를 처음 넘는 horizon: {recommended}s")
        print(f"-> 이 임계값 기준이면, 권장 최대 horizon은 {recommended - 0.1:.1f}s 이하")
    else:
        print(f"3.0초까지 평균 오차가 {args.error_threshold}m를 넘지 않음 (전체 구간 안전)")
    print("=" * 70)

    print("\n지터(예측 안정성) 분석 중...")
    jitter = evaluate_jitter(series)
    if len(jitter) > 0:
        print(f"지터 표본수: {len(jitter)}")
        print(f"지터(1.0s 뒤 예측이 0.5초 후 재예측시 얼마나 바뀌는지) mean={jitter.mean():.3f}m  median={np.median(jitter):.3f}m  p95={np.percentile(jitter,95):.3f}m")
    else:
        print("지터 계산할 표본이 없습니다.")

    # 그래프 저장
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
    means = [np.mean(errors_by_horizon[h]) if errors_by_horizon[h] else np.nan for h in HORIZONS]
    p95s = [np.percentile(errors_by_horizon[h], 95) if errors_by_horizon[h] else np.nan for h in HORIZONS]
    ax.plot(HORIZONS, means, marker="o", markersize=3, color="#2b6cb0", label="평균")
    ax.plot(HORIZONS, p95s, marker="o", markersize=3, color="#c05621", label="p95")
    ax.axhline(args.error_threshold, color="red", linestyle="--", alpha=0.6, label=f"임계값 {args.error_threshold}m")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] horizon별 오차 곡선 (0.1s 간격)", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    if len(jitter) > 0:
        ax.hist(jitter, bins=50, color="#805ad5", alpha=0.8)
    ax.set_xlabel("지터 (m)", fontproperties=kfont)
    ax.set_ylabel("빈도", fontproperties=kfont)
    ax.set_title(f"[{args.label}] 지터 분포 (예측 안정성)", fontproperties=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"fine_horizon_{args.label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


if __name__ == "__main__":
    main()
