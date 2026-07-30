#!/usr/bin/env python3
"""
eval_robustness_with_noise.py

지금까지의 평가는 완벽한 GT로만 했음 - 실전에서는 실제 OD의 검출 노이즈,
불규칙한 끊김(가려짐)이 섞인다. 이 스크립트는 GT에 이런 현실적인 결함을
합성으로 추가해서, horizon 선택이 노이즈가 있어도 여전히 안전한지 확인한다.

추가하는 결함 종류:
    1. 위치 노이즈: 검출 위치에 가우시안 노이즈 추가 (표준편차 조절 가능)
    2. heading 노이즈: 방향 추정에도 가우시안 노이즈 추가
    3. 불규칙한 끊김(occlusion): 일정 확률로 몇 프레임 연속 "안 보임" 처리
       (균일한 hz 저하가 아니라, burst 형태로 몇 프레임씩 뭉텅이로 빠짐)

사용법:
    python3 eval_robustness_with_noise.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40 \
        --pos-noise-std 0.3 --heading-noise-std 3.0 \
        --occlusion-prob 0.05 --occlusion-burst-len 5
"""

import argparse
import math
import random
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
EVAL_HORIZONS = [0.5, 1.0, 1.5, 2.0, 3.0]
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


def _extract_series(bag_path, exclude_ids, topic="/Object_topic"):
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


def _apply_detection_noise(records, pos_std, heading_std_deg, rng):
    noisy = []
    heading_std = math.radians(heading_std_deg)
    for r in records:
        n = dict(r)
        n["x"] = r["x"] + rng.gauss(0, pos_std)
        n["y"] = r["y"] + rng.gauss(0, pos_std)
        n["heading"] = r["heading"] + rng.gauss(0, heading_std)
        noisy.append(n)
    return noisy


def _apply_occlusion_bursts(records, occlusion_prob, burst_len, rng):
    kept = []
    i = 0
    while i < len(records):
        if rng.random() < occlusion_prob:
            i += burst_len
        else:
            kept.append(records[i])
            i += 1
    return kept


def _nearest_record(records, target_t, max_dt=0.3):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def evaluate(series, pos_std, heading_std_deg, occlusion_prob, burst_len, seed=42):
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in EVAL_HORIZONS}

    for uid, gt_records in series.items():
        observed = _apply_detection_noise(gt_records, pos_std, heading_std_deg, rng)
        observed = _apply_occlusion_bursts(observed, occlusion_prob, burst_len, rng)
        if len(observed) < 2:
            continue

        for i in range(1, len(observed)):
            prev, cur = observed[i - 1], observed[i]
            dt = cur["t"] - prev["t"]
            if dt <= 0:
                continue
            dtheta = cur["heading"] - prev["heading"]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            yaw_rate = dtheta / dt
            v = math.hypot(cur["vx"], cur["vy"])
            if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
                v = -v

            for h in EVAL_HORIZONS:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h)
                gt_future = _nearest_record(gt_records, cur["t"] + h)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--pos-noise-std", type=float, default=0.0)
    parser.add_argument("--heading-noise-std", type=float, default=0.0)
    parser.add_argument("--occlusion-prob", type=float, default=0.0)
    parser.add_argument("--occlusion-burst-len", type=int, default=5)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_series(args.bag_path, exclude_ids)

    print(f"조건: 위치노이즈 std={args.pos_noise_std}m, heading노이즈 std={args.heading_noise_std}deg, "
          f"끊김확률={args.occlusion_prob}, 끊김길이={args.occlusion_burst_len}프레임")
    print("=" * 70)

    errors_by_horizon = evaluate(
        series, args.pos_noise_std, args.heading_noise_std,
        args.occlusion_prob, args.occlusion_burst_len,
    )

    for h in EVAL_HORIZONS:
        vals = errors_by_horizon[h]
        if not vals:
            print(f"  horizon={h}s  표본 없음")
            continue
        arr = np.array(vals)
        print(f"  horizon={h:>4.1f}s  n={len(arr):5d}  mean={arr.mean():.3f}  median={np.median(arr):.3f}  p90={np.percentile(arr,90):.3f}  p95={np.percentile(arr,95):.3f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
