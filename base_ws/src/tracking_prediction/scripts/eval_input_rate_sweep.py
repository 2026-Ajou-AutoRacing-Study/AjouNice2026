#!/usr/bin/env python3
"""
eval_input_rate_sweep.py


사용법:
    python3 eval_input_rate_sweep.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
CANDIDATE_HZ = [5, 7, 8, 9, 10, 15, 20, 30]
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


def _nearest_record(records, target_t, max_dt):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def _downsample(records, target_hz):
    """실측 48.7Hz GT를 target_hz로 듬성듬성 샘플링해서 'OD가 이 속도로 검출을 준다면'을 흉내냄."""
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


def evaluate_at_rate(series, target_hz):
    """target_hz로 다운샘플링한 뒤, 원본(고속) GT를 정답으로 삼아 CTRV 오차를 horizon별로 계산."""
    errors_by_horizon = {h: [] for h in EVAL_HORIZONS}
    # 검출 시점 사이 "중간 지점"을 얼마나 잘 메우는지(extrapolation 효과 근사) 측정용
    midpoint_errors = []

    for uid, full_records in series.items():
        sampled = _downsample(full_records, target_hz)
        if len(sampled) < 2:
            continue

        for i in range(1, len(sampled)):
            prev, cur = sampled[i - 1], sampled[i]
            dt = cur["t"] - prev["t"]
            if dt <= 0:
                continue

            dtheta = cur["heading"] - prev["heading"]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            yaw_rate = dtheta / dt
            v = math.hypot(cur["vx"], cur["vy"])
            if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
                v = -v

            # (A) horizon별 오차: 이 검출 시점(cur.t) 기준으로 CTRV 예측 vs 실제(고속GT)
            for h in EVAL_HORIZONS:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h)
                gt_future = _nearest_record(full_records, cur["t"] + h, max_dt=0.3)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

            # (B) extrapolation 효과 근사: "검출과 검출 사이 중간 시점"을 이 상태로
            # 얼마나 잘 채우는지 (dt/2 뒤 예측 vs 그 중간 시점의 실제 고속GT)
            mid_dt = dt / 2.0
            px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, mid_dt)
            gt_mid = _nearest_record(full_records, cur["t"] + mid_dt, max_dt=0.3)
            if gt_mid is not None:
                midpoint_errors.append(math.hypot(px - gt_mid["x"], py - gt_mid["y"]))

    return errors_by_horizon, np.array(midpoint_errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    print("\n" + "=" * 100)
    print("입력(OD 추정) hz별 horizon 오차 (mean, 단위: m)")
    print("=" * 100)
    header = f"{'입력 hz':>8}" + "".join(f"{h:>9.1f}s" for h in EVAL_HORIZONS) + f"{'중간지점 오차(extrapolation 효과)':>30}"
    print(header)

    results = {}
    for hz in CANDIDATE_HZ:
        errors_by_horizon, midpoint_errors = evaluate_at_rate(series, hz)
        results[hz] = (errors_by_horizon, midpoint_errors)
        row = f"{hz:>8}"
        for h in EVAL_HORIZONS:
            vals = errors_by_horizon[h]
            row += f"{np.mean(vals):>10.3f}" if vals else f"{'N/A':>10}"
        mid_mean = midpoint_errors.mean() if len(midpoint_errors) else float("nan")
        row += f"{mid_mean:>30.3f}"
        print(row)
    print("=" * 100)
    print("'중간지점 오차'는 검출과 검출 사이(예: 8Hz면 0.0625초 간격)의 실제 위치를,")
    print("직전 검출 상태만으로 CTRV 외삽했을 때 얼마나 정확히 메우는지를 나타냄.")
    print("이 값이 작을수록 'extrapolation이 그 사이 구간을 잘 메워준다'는 뜻.")

    # 그래프
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    ax = axes[0]
    for h, color in zip(EVAL_HORIZONS, ["#2b6cb0", "#2f855a", "#dd6b20", "#c53030", "#805ad5"]):
        means = [np.mean(results[hz][0][h]) if results[hz][0][h] else np.nan for hz in CANDIDATE_HZ]
        ax.plot(CANDIDATE_HZ, means, marker="o", label=f"horizon={h}s", color=color)
    ax.set_xlabel("입력(OD 추정) hz", fontproperties=kfont)
    ax.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] 입력 hz에 따른 horizon별 예측 오차", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    mid_means = [results[hz][1].mean() if len(results[hz][1]) else np.nan for hz in CANDIDATE_HZ]
    ax.plot(CANDIDATE_HZ, mid_means, marker="o", color="#805ad5")
    ax.set_xlabel("입력(OD 추정) hz", fontproperties=kfont)
    ax.set_ylabel("중간지점 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] Extrapolation 효과 (검출 사이 구간 메움 정확도)", fontproperties=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"input_rate_sweep_{args.label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


if __name__ == "__main__":
    main()
