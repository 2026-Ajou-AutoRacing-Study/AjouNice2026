#!/usr/bin/env python3
"""
analyze_jitter_outliers.py

analyze_jitter_vs_yawrate.py에서 yaw_rate 구간별 지터를 봤더니, 특히
10~20deg/s, 20deg/s~ 구간에서 mean(2.0~2.9m)이 median(0.16~0.36m)보다
10배 넘게 커서 "회전 자체"보다는 "일부 소수 트랙/순간의 극단적 이상치"가
지터/오차 분포를 지배하고 있을 가능성이 높다고 판단했다. 이 스크립트는 그
이상치들이 정확히 어떤 unique_id, 어떤 시각에서 나오는지 상위 N개를 뽑아서
보여준다 (rviz로 그 구간을 직접 재생해서 눈으로 확인하기 위한 좌표용).

analyze_jitter_vs_yawrate.py 대비 수정한 점:
    원본은 _split_teleport_segments()를 안 거쳐서, 같은 unique_id가 재사용
    (순간이동)된 경우 그 경계에서 튀는 값이 "회전 때문"으로 잘못 섞여 들어갈
    수 있었다. 이 스크립트는 eval_final_horizon_decision.py와 동일하게
    텔레포트 분리를 거친 뒤 분석해서, 진짜 회전/추정 불안정 문제와 데이터
    경계 문제를 분리한다.

지터 정의(analyze_jitter_vs_yawrate.py와 동일):
    t0 시점 상태로 1.0s 뒤(t0+1.0)를 예측한 결과(A)와, t0+0.5 시점 상태로
    0.5s 뒤(역시 t0+1.0)를 예측한 결과(B)를 비교. 같은 목표 시점을 다른
    시점의 정보로 예측했을 때 결과가 얼마나 달라지는지(지터, 안정성)를 본다.

사용법:
    python3 analyze_jitter_outliers.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40 \
        --top-n 30
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
TELEPORT_THRESHOLD_M = 15.0


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


def _nearest_record_with_index(records, target_t, max_dt=0.1):
    best, best_diff, best_idx = None, max_dt, None
    for idx, r in enumerate(records):
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best, best_idx = diff, r, idx
    return best, best_idx


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
    """샘플별로 (uid, t, x, y, v, yaw_rate, jitter)를 전부 기록해서 반환."""
    rows = []

    for uid, records in series.items():
        for i in range(1, len(records)):
            iv = _instantaneous_v_yawrate(records, i)
            if iv is None:
                continue
            v0, yawrate0 = iv
            cur = records[i]

            pxA, pyA, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v0, yawrate0, 1.0)

            later, later_idx = _nearest_record_with_index(records, cur["t"] + 0.5, max_dt=0.1)
            if later is None:
                continue
            iv_later = _instantaneous_v_yawrate(records, later_idx)
            if iv_later is None:
                continue
            v1, yawrate1 = iv_later
            pxB, pyB, _ = ctrv_predict(later["x"], later["y"], later["heading"], v1, yawrate1, 0.5)

            jitter = math.hypot(pxA - pxB, pyA - pyB)
            rows.append(
                {
                    "uid": uid, "t": cur["t"], "x": cur["x"], "y": cur["y"],
                    "v": v0, "yaw_rate_deg": math.degrees(yawrate0), "jitter": jitter,
                }
            )

    return rows


def print_top_outliers(rows, top_n):
    rows_sorted = sorted(rows, key=lambda r: r["jitter"], reverse=True)
    top = rows_sorted[:top_n]

    print("\n" + "=" * 100)
    print(f"지터 상위 {top_n}개 샘플 (rviz로 직접 확인할 좌표/시각)")
    print("=" * 100)
    header = f"{'uid':>12} {'t(s)':>14} {'x':>9} {'y':>9} {'v(m/s)':>8} {'yaw_rate(deg/s)':>16} {'jitter(m)':>10}"
    print(header)
    for r in top:
        print(
            f"{r['uid']:>12} {r['t']:>14.3f} {r['x']:>9.2f} {r['y']:>9.2f} "
            f"{r['v']:>8.2f} {r['yaw_rate_deg']:>16.2f} {r['jitter']:>10.3f}"
        )
    print("=" * 100)
    return top


def print_uid_concentration(rows, top_n):
    """상위 N개가 소수 uid에 몰려있는지 확인 (이상치가 '특정 트랙 문제'인지 판단용)."""
    rows_sorted = sorted(rows, key=lambda r: r["jitter"], reverse=True)
    top = rows_sorted[:top_n]

    counts = defaultdict(int)
    for r in top:
        counts[r["uid"]] += 1

    print(f"\n상위 {top_n}개 중 uid별 등장 횟수 (소수 uid에 몰려있으면 '특정 트랙 문제'일 가능성):")
    for uid, cnt in sorted(counts.items(), key=lambda kv: kv[1], reverse=True):
        print(f"  {uid:>12}: {cnt}회")
    n_unique = len(counts)
    print(f"-> 상위 {top_n}개가 서로 다른 uid {n_unique}개에서 나옴 "
          f"({'소수 트랙에 집중됨' if n_unique <= max(3, top_n // 5) else '여러 트랙에 고르게 분산됨'})")


def save_csv(rows, path):
    import csv

    rows_sorted = sorted(rows, key=lambda r: r["jitter"], reverse=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["uid", "t", "x", "y", "v", "yaw_rate_deg", "jitter"])
        writer.writeheader()
        writer.writerows(rows_sorted)
    print(f"\n전체 샘플 CSV 저장됨: {path} (n={len(rows_sorted)})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--top-n", type=int, default=30)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    rows = analyze(series)
    print(f"표본수: {len(rows)}")

    print_top_outliers(rows, args.top_n)
    print_uid_concentration(rows, args.top_n)

    csv_path = f"jitter_outliers_{args.label}.csv"
    save_csv(rows, csv_path)


if __name__ == "__main__":
    main()
