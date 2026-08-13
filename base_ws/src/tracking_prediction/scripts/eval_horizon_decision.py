#!/usr/bin/env python3
"""
eval_horizon_decision.py

기존 eval_fine_horizon.py를 확장:
    1. 각 horizon(0.1~3.0s, 30개)마다 전체 분포 통계(p25/p50/p75/p90/p95/p99/max)를 계산
    2. 박스플롯: horizon이 늘어날수록 오차 분포가 얼마나 "퍼지는지" 한눈에 보여줌
    3. 퍼센타일 곡선 그래프: p50/p90/p95 각각을 horizon에 따라 선으로 그려서
       "몇 % 확률로 오차가 얼마 이내인지"를 horizon별로 비교 가능
    4. 결정표: "오차를 X m 이내로 유지하고 싶다 + Y% 확률로 그러길 원한다"는
       조건들을 여러 개 놓고, 각 조건을 만족하는 최대 horizon을 자동 계산
       (예: "95% 확률로 2m 이내를 원하면 -> horizon은 얼마까지 가능한가")

사용법:
    python3 eval_horizon_decision.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]  # 0.1 ~ 3.0

# 결정표에 쓸 (오차 임계값(m), 원하는 확률(percentile)) 조합들
DECISION_CRITERIA = [
    (1.0, 50), (1.0, 90), (1.0, 95),
    (2.0, 50), (2.0, 90), (2.0, 95),
    (3.0, 50), (3.0, 90), (3.0, 95),
    (5.0, 90), (5.0, 95),
]


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt):
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
                        "t": stamp, "x": obj.position.x, "y": obj.position.y,
                        "heading": _heading_to_rad(obj.heading),
                        "vx": obj.velocity.x * KMH_TO_MS, "vy": obj.velocity.y * KMH_TO_MS,
                    }
                )
    for uid in series:
        series[uid].sort(key=lambda r: r["t"])
    return _split_teleport_segments(series)


TELEPORT_THRESHOLD_M = 15.0  # 이 이상 순간이동하면 unique_id 재사용(다른 차)으로 간주


def _split_teleport_segments(series):
    """같은 unique_id라도 순간이동(재사용)이 감지되면 별개 궤적으로 쪼갠다."""
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


def evaluate(series):
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


def print_distribution_table(errors_by_horizon):
    print("=" * 100)
    print("horizon별 오차 분포 (전체 백분위수, 단위: m)")
    print("=" * 100)
    header = f"{'horizon':>8} {'n':>6} {'mean':>7} {'p25':>7} {'p50':>7} {'p75':>7} {'p90':>7} {'p95':>7} {'p99':>7} {'max':>8}"
    print(header)
    for h in HORIZONS:
        vals = errors_by_horizon[h]
        if not vals:
            continue
        arr = np.array(vals)
        print(
            f"{h:>8.1f} {len(arr):>6d} {arr.mean():>7.3f} "
            f"{np.percentile(arr,25):>7.3f} {np.percentile(arr,50):>7.3f} {np.percentile(arr,75):>7.3f} "
            f"{np.percentile(arr,90):>7.3f} {np.percentile(arr,95):>7.3f} {np.percentile(arr,99):>7.3f} "
            f"{arr.max():>8.3f}"
        )
    print("=" * 100)


def print_decision_table(errors_by_horizon):
    print("\n" + "=" * 70)
    print("Horizon 결정표: '오차 X m 이내를 Y% 확률로 원한다' 기준 최대 horizon")
    print("=" * 70)
    for threshold, pct in DECISION_CRITERIA:
        best_h = None
        for h in HORIZONS:
            vals = errors_by_horizon[h]
            if not vals:
                continue
            p = np.percentile(vals, pct)
            if p <= threshold:
                best_h = h
            else:
                break  # horizon이 커질수록 오차도 커지는 경향이므로, 한번 넘으면 이후도 대부분 넘음
        if best_h is not None:
            print(f"  {threshold:>4.1f}m 이내를 {pct:>2d}% 확률로 원하면  ->  horizon <= {best_h:.1f}s 가능")
        else:
            print(f"  {threshold:>4.1f}m 이내를 {pct:>2d}% 확률로 원하면  ->  0.1s부터 이미 불가능")
    print("=" * 70)


def make_plots(label, errors_by_horizon):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    box_horizons = HORIZONS[::3]
    box_data = [errors_by_horizon[h] for h in box_horizons]

    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))

    ax = axes[0]
    ax.boxplot(box_data, labels=[f"{h:.1f}" for h in box_horizons], showfliers=False)
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] horizon별 오차 분포 (박스플롯, 이상치 제외)", fontproperties=kfont)
    ax.grid(alpha=0.3, axis="y")

    ax = axes[1]
    p50s = [np.percentile(errors_by_horizon[h], 50) if errors_by_horizon[h] else np.nan for h in HORIZONS]
    p90s = [np.percentile(errors_by_horizon[h], 90) if errors_by_horizon[h] else np.nan for h in HORIZONS]
    p95s = [np.percentile(errors_by_horizon[h], 95) if errors_by_horizon[h] else np.nan for h in HORIZONS]
    ax.plot(HORIZONS, p50s, marker="o", markersize=3, label="p50(중앙값)", color="#2f855a")
    ax.plot(HORIZONS, p90s, marker="o", markersize=3, label="p90", color="#dd6b20")
    ax.plot(HORIZONS, p95s, marker="o", markersize=3, label="p95", color="#c53030")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 퍼센타일별 오차 곡선", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out1 = f"horizon_distribution_{label}.png"
    plt.savefig(out1, dpi=150)
    print(f"\n그래프 저장됨: {out1}")

    fig2, ax2 = plt.subplots(figsize=(7, 5.5))
    sample_horizons = [0.5, 1.0, 1.5, 2.0, 3.0]
    colors = ["#2b6cb0", "#2f855a", "#dd6b20", "#c53030", "#805ad5"]
    for h, c in zip(sample_horizons, colors):
        vals = np.sort(errors_by_horizon[h])
        if len(vals) == 0:
            continue
        cdf = np.arange(1, len(vals) + 1) / len(vals)
        ax2.plot(vals, cdf, label=f"{h}s", color=c)
    ax2.set_xlabel("오차 (m)", fontproperties=kfont)
    ax2.set_ylabel("누적 확률 (이 오차 이하일 확률)", fontproperties=kfont)
    ax2.set_title(f"[{label}] horizon별 오차 누적분포(CDF)", fontproperties=kfont)
    ax2.legend(title="horizon", prop=kfont)
    ax2.set_xlim(0, 10)
    ax2.grid(alpha=0.3)
    plt.tight_layout()
    out2 = f"horizon_cdf_{label}.png"
    plt.savefig(out2, dpi=150)
    print(f"그래프 저장됨: {out2}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_object_series(args.bag_path, exclude_ids)
    print(f"평가 대상 unique_id 수: {len(series)}")

    errors_by_horizon = evaluate(series)
    print_distribution_table(errors_by_horizon)
    print_decision_table(errors_by_horizon)
    make_plots(args.label, errors_by_horizon)


if __name__ == "__main__":
    main()
