#!/usr/bin/env python3
"""
diagnose_teleport_near_gap.py

가설: /tracking/objects의 ~2.3초 gap이 EKF 트래커의 max_association_dist_m=5.0m
때문일 수 있다 - 여러 NPC가 동시에 "텔레포트"(위치가 순간적으로 5m 이상, 특히
15m 이상 점프)하면 EKF가 기존 track과 매칭을 못 해 전부 track을 잃고,
output_confirmed_track:true라 재확인될 때까지 아무것도 출력하지 않을 수 있다.

이 스크립트는 /Object_topic 전체에서, 같은 unique_id를 가진 객체가 연속
프레임 사이에 threshold(m) 이상 순간이동한 모든 사례의 시각을 찾아서,
동시에 여러 대가 몰려서 점프하는 구간(=클러스터)이 있는지 보여준다.

사용법:
    python3 diagnose_teleport_near_gap.py <bag_path> [--threshold 5.0] [--cluster-window 1.0]
"""

import argparse
import math
from collections import defaultdict

import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--threshold", type=float, default=5.0, help="이 이상 순간이동하면 텔레포트로 간주(m)")
    parser.add_argument("--cluster-window", type=float, default=1.0, help="이 시간(초) 이내에 몰린 텔레포트를 하나의 클러스터로 묶음")
    args = parser.parse_args()

    series = defaultdict(list)
    t0 = None
    with rosbag.Bag(args.bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=["/Object_topic"]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            if t0 is None:
                t0 = stamp
            for obj in msg.npc_list:
                series[obj.unique_id].append((stamp, obj.position.x, obj.position.y))

    jumps = []  # (t, uid, dist)
    for uid, records in series.items():
        records.sort(key=lambda r: r[0])
        for i in range(1, len(records)):
            t_prev, x_prev, y_prev = records[i - 1]
            t_cur, x_cur, y_cur = records[i]
            dist = math.hypot(x_cur - x_prev, y_cur - y_prev)
            if dist >= args.threshold:
                jumps.append((t_cur, uid, dist))

    jumps.sort()
    print(f"[텔레포트 후보] threshold={args.threshold}m 이상 순간이동: 총 {len(jumps)}건\n")

    if not jumps:
        print("텔레포트 후보가 없습니다 - 다른 원인을 찾아야 합니다.")
        return

    # 클러스터링: cluster_window 이내로 몰린 것끼리 묶기
    clusters = []
    cur = [jumps[0]]
    for j in jumps[1:]:
        if j[0] - cur[-1][0] <= args.cluster_window:
            cur.append(j)
        else:
            clusters.append(cur)
            cur = [j]
    clusters.append(cur)

    print(f"클러스터(동시에 몰린 텔레포트) 수: {len(clusters)}")
    print(f"{'t(bag 시작 후)':>16} | {'포함 uid 수':>10} | {'uid 목록'}")
    for c in clusters:
        t_rel = c[0][0] - t0
        uids = sorted({j[1] for j in c})
        print(f"{t_rel:>16.3f} | {len(uids):>10d} | {uids}")


if __name__ == "__main__":
    main()
