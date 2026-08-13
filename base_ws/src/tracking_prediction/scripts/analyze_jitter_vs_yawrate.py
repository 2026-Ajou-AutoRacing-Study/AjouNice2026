#!/usr/bin/env python3
"""
analyze_jitter_vs_yawrate.py

지터(예측 안정성)가 큰 순간이 실제로 "회전 중"(|yaw_rate| 큰 순간)인지
상관관계로 확인한다. eval_fine_horizon.py의 evaluate_jitter()와 같은 방식으로
지터를 계산하되, 그 순간의 |yaw_rate|도 같이 기록해서 산점도+상관계수를 낸다.

사용법:
    python3 analyze_jitter_vs_yawrate.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6


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
    return series


def _nearest_record(records, target_t, max_dt=0.1):
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


def analyze(series):
    jitters = []
    abs_yaw_rates = []

    for uid, records in series.items():
        for i in range(1, len(records)):
            iv = _instantaneous_v_yawrate(records, i)
            if iv is None:
                continue
            v0, yawrate0 = iv
            cur = records[i]

            pxA, pyA, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v0, yawrate0, 1.0)

            later = _nearest_record(records, cur["t"] + 0.5)
            if later is None:
                continue
            later_idx = records.index(later)
            iv_later = _instantaneous_v_yawrate(records, later_idx)
            if iv_later is None:
                continue
            v1, yawrate1 = iv_later
            pxB, pyB, _ = ctrv_predict(later["x"], later["y"], later["heading"], v1, yawrate1, 0.5)

            jitter = math.hypot(pxA - pxB, pyA - pyB)
            jitters.append(jitter)
            abs_yaw_rates.append(abs(yawrate0))

    return np.array(jitters), np.array(abs_yaw_rates)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_object_series(args.bag_path, exclude_ids)

    jitters, abs_yaw_rates = analyze(series)
    print(f"표본수: {len(jitters)}")

    corr = np.corrcoef(abs_yaw_rates, jitters)[0, 1]
    print(f"지터 vs |yaw_rate| 피어슨 상관계수: {corr:.3f}")

    # yaw_rate 구간별 지터 평균도 같이 출력 (직관적 확인용)
    bins = [0, math.radians(2), math.radians(5), math.radians(10), math.radians(20), math.radians(90)]
    bin_labels = ["0~2deg/s", "2~5deg/s", "5~10deg/s", "10~20deg/s", "20deg/s~"]
    print("\nyaw_rate 구간별 평균 지터:")
    for i in range(len(bins) - 1):
        mask = (abs_yaw_rates >= bins[i]) & (abs_yaw_rates < bins[i + 1])
        if mask.sum() == 0:
            continue
        print(f"  {bin_labels[i]:>12}: n={mask.sum():5d}  mean_jitter={jitters[mask].mean():.3f}m  median_jitter={np.median(jitters[mask]):.3f}m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(np.degrees(abs_yaw_rates), jitters, s=5, alpha=0.3, color="#805ad5")
    ax.set_xlabel("|yaw_rate| (deg/s)", fontproperties=kfont)
    ax.set_ylabel("지터 (m)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] 지터 vs 회전 정도 (r={corr:.2f})", fontproperties=kfont)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    out_path = f"jitter_vs_yawrate_{args.label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


if __name__ == "__main__":
    main()
