#!/usr/bin/env python3
"""
eval_od_hz_sensitivity.py

eval_final_horizon_decision.py는 OD 주기를 10Hz "고정값"으로 놓고 최종 결정을 냈다.
하지만 실제 PointPainting/BEVFusion 등 OD가 정확히 몇 hz로 들어올지는 사전에 확정하기
어렵고(하드웨어/후처리 부하에 따라 변동), "10Hz 근처(8~15Hz)"로만 예상되는 상황이다.

이 스크립트는 그 불확실 구간(8~15Hz)을 전부 스윕해서:
    - OD hz가 낮아지거나 높아짐에 따라 "결정표"(오차 Xm 이내를 Y% 확률로 원하면
      horizon이 얼마까지 가능한가)가 어떻게 바뀌는지 표로 보여주고
    - 어떤 hz가 오더라도 안전한 "보수적(worst-case) horizon" 하나를 자동 추천한다.

즉 eval_final_horizon_decision.py가 "10Hz라고 가정하면 최종 horizon은?"에 답한다면,
이 스크립트는 "10Hz 근처 어디서 오더라도 안전한 horizon은?"에 답한다.

검출 위치오차는 eval_final_horizon_decision.py와 동일하게 실측 PointPainting TP
중심오차(mean=0.145m)를 Rayleigh 역산한 가우시안 노이즈로 고정 적용한다 (hz만 스윕,
노이즈 크기 자체를 스윕하고 싶으면 --tp-center-error-mean-list 사용).

사용법:
    python3 eval_od_hz_sensitivity.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40

    # hz 범위나 검출오차도 같이 바꿔가며 보고 싶으면:
    python3 eval_od_hz_sensitivity.py <bag_path> --label roundabout \
        --od-hz-list 8,9,10,11,12,13,14,15 \
        --tp-center-error-mean-list 0.1,0.145,0.2
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

# OD가 "10Hz 근처"로 예상되지만 정확한 값은 불확실 -> 이 범위를 스윕해서 민감도 확인
OD_HZ_LIST_DEFAULT = [8, 9, 10, 11, 12, 13, 14, 15]

# 실측 PointPainting TP 중심오차 mean (기본값). 여러 값을 같이 보려면 --tp-center-error-mean-list
TP_CENTER_ERROR_MEAN_DEFAULT = 0.145

HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]  # 0.1 ~ 3.0

# 결정표에 쓸 (오차 임계값(m), 원하는 확률(percentile)) 조합
DECISION_CRITERIA = [
    (1.0, 90), (1.0, 95),
    (2.0, 90), (2.0, 95),
    (3.0, 90), (3.0, 95),
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


def evaluate_scenario(series, od_hz, pos_std, heading_std_deg=0.0, seed=42, max_dt=0.3):
    """od_hz로 다운샘플 + (pos_std, heading_std_deg) 노이즈를 적용한 뒤, 원본 고속GT를
    정답으로 삼아 horizon별 오차를 계산 (eval_final_horizon_decision.py의 combined 시나리오와 동일)."""
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in HORIZONS}

    for uid, full_records in series.items():
        working = _downsample(full_records, od_hz)
        if pos_std > 0 or heading_std_deg > 0:
            working = _apply_detection_noise(working, pos_std, heading_std_deg, rng)
        if len(working) < 2:
            continue

        for i in range(1, len(working)):
            iv = _instantaneous_v_yawrate(working[i - 1], working[i])
            if iv is None:
                continue
            v, yaw_rate = iv
            cur = working[i]
            for h in HORIZONS:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h)
                gt_future = _nearest_record(full_records, cur["t"] + h, max_dt=max_dt)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def max_horizon_for_criterion(errors_by_horizon, threshold, pct):
    best_h = None
    for h in HORIZONS:
        vals = errors_by_horizon[h]
        if not vals:
            continue
        p = np.percentile(vals, pct)
        if p <= threshold:
            best_h = h
        else:
            break
    return best_h


def print_hz_sensitivity_table(tp_mean, results_by_hz):
    """results_by_hz: {hz: errors_by_horizon}"""
    print("\n" + "=" * 100)
    print(f"OD hz 민감도 표 (검출오차 mean={tp_mean:.3f}m 고정) - 기준별 '허용 가능한 최대 horizon(s)'")
    print("=" * 100)
    hz_list = sorted(results_by_hz.keys())
    header = f"{'기준(오차/확률)':>18}" + "".join(f"{hz:>7}Hz" for hz in hz_list) + f"{'보수적 추천':>12}"
    print(header)
    for threshold, pct in DECISION_CRITERIA:
        row_vals = []
        for hz in hz_list:
            h = max_horizon_for_criterion(results_by_hz[hz], threshold, pct)
            row_vals.append(h)
        conservative = min([v for v in row_vals if v is not None], default=None)
        label = f"{threshold:.1f}m@{pct}%"
        row = f"{label:>18}"
        for v in row_vals:
            row += f"{v:>8.1f}" if v is not None else f"{'N/A':>8}"
        row += f"{conservative:>11.1f}s" if conservative is not None else f"{'N/A':>12}"
        print(row)
    print("=" * 100)
    print("'보수적 추천' = 이 hz 범위(8~15Hz) 어디서 OD가 들어와도 기준을 만족하는 최대 horizon")
    print("(= 각 hz별 최대 horizon 중 가장 작은 값. 실제 OD hz를 확신할 수 없을 때 이 값을 쓰면 안전)")


def print_mean_error_table(results_by_hz):
    print("\n" + "=" * 100)
    print("OD hz별 대표 horizon 평균 오차 (단위: m)")
    print("=" * 100)
    hz_list = sorted(results_by_hz.keys())
    compare_horizons = [0.5, 1.0, 1.5, 2.0, 3.0]
    header = f"{'hz':>6}" + "".join(f"{h:>9.1f}s" for h in compare_horizons)
    print(header)
    for hz in hz_list:
        row = f"{hz:>6}"
        for h in compare_horizons:
            vals = results_by_hz[hz][h]
            row += f"{np.mean(vals):>10.3f}" if vals else f"{'N/A':>10}"
        print(row)
    print("=" * 100)


def make_plots(label, tp_mean, results_by_hz):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    hz_list = sorted(results_by_hz.keys())

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # (1) 대표 horizon들의 평균 오차 vs hz
    ax = axes[0]
    compare_horizons = [0.5, 1.0, 1.5, 2.0, 3.0]
    colors = ["#2b6cb0", "#2f855a", "#dd6b20", "#c53030", "#805ad5"]
    for h, c in zip(compare_horizons, colors):
        means = [np.mean(results_by_hz[hz][h]) if results_by_hz[hz][h] else np.nan for hz in hz_list]
        ax.plot(hz_list, means, marker="o", label=f"horizon={h}s", color=c)
    ax.set_xlabel("OD 입력 hz", fontproperties=kfont)
    ax.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 검출오차 mean={tp_mean:.3f}m 고정, OD hz 스윕", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    # (2) 주요 결정기준의 '허용 최대 horizon' vs hz
    ax = axes[1]
    for threshold, pct in [(2.0, 90), (2.0, 95), (1.0, 90), (1.0, 95)]:
        hs = [max_horizon_for_criterion(results_by_hz[hz], threshold, pct) for hz in hz_list]
        ax.plot(hz_list, hs, marker="o", label=f"{threshold:.1f}m@{pct}%")
    ax.set_xlabel("OD 입력 hz", fontproperties=kfont)
    ax.set_ylabel("허용 가능한 최대 horizon (s)", fontproperties=kfont)
    ax.set_title(f"[{label}] hz에 따른 결정 horizon 변화", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"od_hz_sensitivity_{label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--od-hz-list", default=",".join(str(h) for h in OD_HZ_LIST_DEFAULT),
                         help=f"스윕할 OD hz 목록, 콤마구분 (기본 {OD_HZ_LIST_DEFAULT})")
    parser.add_argument("--tp-center-error-mean-list", default=str(TP_CENTER_ERROR_MEAN_DEFAULT),
                         help=f"스윕할 검출오차 mean(m) 목록, 콤마구분 (기본 {TP_CENTER_ERROR_MEAN_DEFAULT}m 단일값)")
    parser.add_argument("--heading-noise-std", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    od_hz_list = [float(x) for x in args.od_hz_list.split(",") if x.strip()]
    tp_mean_list = [float(x) for x in args.tp_center_error_mean_list.split(",") if x.strip()]

    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")
    print(f"스윕할 OD hz: {od_hz_list}")
    print(f"스윕할 검출오차(TP 중심오차 mean): {tp_mean_list}")

    for tp_mean in tp_mean_list:
        pos_std = tp_mean / math.sqrt(math.pi / 2.0)
        print(
            f"\n### 검출오차 mean={tp_mean:.3f}m -> Rayleigh 역산 sigma={pos_std:.4f}m 조건으로 hz 스윕 ###"
        )
        results_by_hz = {}
        for hz in od_hz_list:
            results_by_hz[hz] = evaluate_scenario(
                series, hz, pos_std, args.heading_noise_std, seed=args.seed
            )
        print_mean_error_table(results_by_hz)
        print_hz_sensitivity_table(tp_mean, results_by_hz)
        label = args.label if len(tp_mean_list) == 1 else f"{args.label}_tp{tp_mean:.3f}"
        make_plots(label, tp_mean, results_by_hz)


if __name__ == "__main__":
    main()
