#!/usr/bin/env python3
"""
eval_vehicle_spacing.py

목적:
    prediction_node.py의 yaw_rate 추정은 "직전 프레임에서 가장 가까운 위치의
    같은 class_label 객체"를 최근접 매칭한다 (MATCH_DIST_THRESHOLD=3.0m).
    로터리에서 차 3대가 "일정한 간격"으로 돈다고 했는데, 그 간격이 실제로
    3m보다 충분히 넉넉한지 확인해야 오매칭 위험을 판단할 수 있다.

    MORAI를 직접 띄우지 않아도 /Object_topic은 이미 rosbag 안에 기록되어
    있는 raw 데이터이므로, bag 파일만으로 프레임마다 차량들 간 실제 거리를
    그대로 계산할 수 있다 (재생/시뮬레이션 불필요).

측정 방법:
    각 /Object_topic 프레임(콜백 1회)마다, 그 순간 존재하는 모든 npc_list
    차량들 사이의 모든 쌍(pair) 거리를 계산하고, 그중 최소값(그 프레임에서
    "가장 가까운 두 차" 간 거리)을 기록한다. 이 최소값의 분포가
    MATCH_DIST_THRESHOLD=3.0m 대비 얼마나 여유가 있는지를 보여준다.

    또한 프레임당 동시 존재 차량 수 분포도 같이 출력해서, "3대가 로터리를
    돈다"는 가정이 이 bag에서 실제로 맞는지도 같이 확인한다.

사용법:
    python3 eval_vehicle_spacing.py <bag_path> --label roundabout
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag


def _pairwise_min_and_all(objs):
    """한 프레임의 (uid, x, y) 목록에서 모든 쌍의 거리와 그중 최소값을 반환."""
    dists = []
    n = len(objs)
    for i in range(n):
        for j in range(i + 1, n):
            _, xi, yi = objs[i]
            _, xj, yj = objs[j]
            dists.append(math.hypot(xi - xj, yi - yj))
    if not dists:
        return None, []
    return min(dists), dists


def analyze(bag_path, topic="/Object_topic"):
    frame_rows = []  # (t, n_vehicles, min_pair_dist)
    all_pair_dists = []
    close_frames = []  # 상세 확인용: min_pair_dist가 임계치 이하인 프레임

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            objs = [(o.unique_id, o.position.x, o.position.y) for o in msg.npc_list]
            n = len(objs)
            min_dist, dists = _pairwise_min_and_all(objs)
            frame_rows.append({"t": stamp, "n_vehicles": n, "min_pair_dist": min_dist})
            all_pair_dists.extend(dists)
            if min_dist is not None:
                close_frames.append((stamp, min_dist, objs))

    return frame_rows, all_pair_dists, close_frames


def print_vehicle_count_distribution(frame_rows):
    counts = [r["n_vehicles"] for r in frame_rows]
    print("\n" + "=" * 70)
    print("프레임당 동시 존재 npc_list 차량 수 분포")
    print("=" * 70)
    unique_vals, freq = np.unique(counts, return_counts=True)
    total = len(counts)
    for v, f in zip(unique_vals, freq):
        print(f"  차량 수 {v:>3}대: {f:>6}프레임 ({100*f/total:5.1f}%)")


def print_spacing_summary(frame_rows, match_threshold=3.0):
    valid = [r["min_pair_dist"] for r in frame_rows if r["min_pair_dist"] is not None]
    if not valid:
        print("\n(동시에 2대 이상 존재하는 프레임이 없어 간격을 계산할 수 없음)")
        return

    arr = np.array(valid)
    print("\n" + "=" * 70)
    print("프레임별 '가장 가까운 두 차량' 간 거리(min pairwise distance) 통계")
    print("=" * 70)
    print(f"  표본 프레임 수: {len(arr)}")
    print(f"  최소: {arr.min():.2f}m")
    print(f"  p1  : {np.percentile(arr, 1):.2f}m")
    print(f"  p5  : {np.percentile(arr, 5):.2f}m")
    print(f"  p10 : {np.percentile(arr, 10):.2f}m")
    print(f"  p50 : {np.percentile(arr, 50):.2f}m")
    print(f"  mean: {arr.mean():.2f}m")
    print(f"  최대: {arr.max():.2f}m")

    for thr in (match_threshold, 5.0, 8.0):
        frac = 100.0 * np.mean(arr < thr)
        print(f"  -> 간격이 {thr:.1f}m 미만인 프레임 비율: {frac:.2f}%")


def save_close_frames_csv(close_frames, path, threshold=3.0):
    import csv

    rows = [
        {"t": t, "min_pair_dist": d}
        for (t, d, _objs) in close_frames
        if d is not None and d < threshold
    ]
    rows.sort(key=lambda r: r["min_pair_dist"])
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t", "min_pair_dist"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n간격 {threshold}m 미만 프레임 상세 CSV 저장됨: {path} (n={len(rows)})")
    if rows:
        print("  (t는 bag 내부 타임스탬프. rviz로 재생 중 이 시각 근처를 seek하면")
        print("   실제로 어떤 상황인지 육안으로도 확인 가능)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--match-threshold", type=float, default=3.0,
                         help="prediction_node.py의 MATCH_DIST_THRESHOLD와 동일한 값(기본 3.0m)")
    args = parser.parse_args()

    frame_rows, all_pair_dists, close_frames = analyze(args.bag_path)

    print(f"\n총 프레임 수: {len(frame_rows)}")
    print_vehicle_count_distribution(frame_rows)
    print_spacing_summary(frame_rows, args.match_threshold)

    csv_path = f"vehicle_spacing_close_frames_{args.label}.csv"
    save_close_frames_csv(close_frames, csv_path, args.match_threshold)


if __name__ == "__main__":
    main()
