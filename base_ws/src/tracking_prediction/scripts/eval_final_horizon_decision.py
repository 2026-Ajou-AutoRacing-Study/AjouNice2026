#!/usr/bin/env python3
"""
eval_final_horizon_decision.py

지금까지의 개별 평가를 하나로 합쳐서 "실전 조건"에 가까운 최종 horizon 결정을 내린다.

    1. eval_input_rate_sweep.py  -> OD 입력 hz를 스윕해서 10Hz가 실용적인 선택임을 확인 (선행 작업)
    2. eval_robustness_with_noise.py -> 임의의 노이즈 크기로 강건성을 정성적으로 확인 (선행 작업)
    3. 이 스크립트 -> 위 둘을 "실측값"으로 고정해서 하나의 시나리오로 합침
        - OD 주기: 10Hz 고정 (스윕 결과로 결정된 값)
        - 검출 위치 오차: 실제 PointPainting TP(참양성) 중심 오차 실측치
          mean = 0.145m 을 그대로 사용 (아래 "노이즈 변환" 참고)

    최종적으로 "10Hz + 실측 검출오차"가 섞인 조건에서 horizon별 오차 분포/퍼센타일과
    "오차 X m 이내를 Y% 확률로 원한다" 결정표를 뽑고, 어느 정도가 rate 때문이고
    어느 정도가 검출오차 때문인지 분해해서 보여준다 (raw / 10Hz만 / 10Hz+검출오차 3단 비교).

노이즈 변환 (mean 중심오차 -> 가우시안 표준편차):
    PointPainting 평가에서 나온 "중심 오차"는 (dx, dy) 두 축 오차의 유클리드 거리
    (Euclidean norm)의 평균이다. 각 축에 독립 등방(isotropic) 가우시안 노이즈
    N(0, sigma^2)를 가정하면, 그 노름은 Rayleigh 분포를 따르고
        E[norm] = sigma * sqrt(pi/2)
    이 성립한다 (Rayleigh 분포의 평균 공식). 따라서 실측 mean 중심오차로부터
        sigma = mean / sqrt(pi/2)
    로 역산해서, 시뮬레이션에 넣을 가우시안 노이즈의 표준편차를 얻는다.
    (mean=0.145m 이면 sigma ≈ 0.1157m)

heading 노이즈는 이번 PointPainting 실측에 포함되어 있지 않아 기본값 0으로 둔다
(측정되면 --heading-noise-std로 갱신 가능). occlusion(끊김)도 이번 "결합" 시나리오의
핵심은 아니라서 기본 off이며, 필요하면 --occlusion-prob 등으로 켤 수 있다
(eval_robustness_with_noise.py와 동일한 옵션을 그대로 이식).

사용법:
    python3 eval_final_horizon_decision.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40

    # 실측 중심오차가 갱신되면:
    python3 eval_final_horizon_decision.py <bag_path> --label roundabout \
        --tp-center-error-mean 0.145
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

# --- 1) OD 주기: eval_input_rate_sweep.py 스윕 결과로 결정된 고정값 ---
OD_HZ_DEFAULT = 10.0

# --- 2) 검출오차: 실제 PointPainting TP 중심오차 실측치 (mean, 단위 m) ---
TP_CENTER_ERROR_MEAN_DEFAULT = 0.145

# horizon 그리드: eval_horizon_decision.py와 동일 (0.1 ~ 3.0s, 0.1 간격)
HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]

# 최종 결정표에 쓸 (오차 임계값(m), 원하는 확률(percentile)) 조합
DECISION_CRITERIA = [
    (1.0, 50), (1.0, 90), (1.0, 95),
    (2.0, 50), (2.0, 90), (2.0, 95),
    (3.0, 50), (3.0, 90), (3.0, 95),
    (5.0, 90), (5.0, 95),
]

# raw / 10Hz만 / 10Hz+검출오차 3단 비교에 쓸 대표 horizon
COMPARE_HORIZONS = [0.5, 1.0, 1.5, 2.0, 3.0]


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


def _nearest_record(records, target_t, max_dt=0.3):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def _downsample(records, target_hz):
    """실측 GT를 target_hz로 듬성듬성 샘플링해서 'OD가 이 속도로 검출을 준다면'을 흉내냄.
    (eval_input_rate_sweep.py와 동일 로직)"""
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
    """검출 위치/heading에 가우시안 노이즈 추가. (eval_robustness_with_noise.py와 동일 로직)
    속도(vx, vy)는 기존 스크립트들과 동일하게 GT 값을 그대로 사용한다 (트래킹의 속도
    추정 자체는 이번 평가 범위 밖 - tracking_and_prediction_v1.md 참고)."""
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
    if occlusion_prob <= 0:
        return records
    kept = []
    i = 0
    while i < len(records):
        if rng.random() < occlusion_prob:
            i += burst_len
        else:
            kept.append(records[i])
            i += 1
    return kept


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


def evaluate_scenario(series, gt_lookup, horizons, od_hz=None, pos_std=0.0,
                       heading_std_deg=0.0, occlusion_prob=0.0, occlusion_burst_len=5,
                       seed=42, max_dt=0.3):
    """하나의 시나리오(rate/노이즈 조합)에 대해 horizon별 오차 리스트를 계산.

    od_hz=None 이면 다운샘플 없이 원본(고속) 시퀀스를 그대로 씀 (= 'raw' 베이스라인).
    gt_lookup은 항상 원본 고속 GT 시퀀스(정답)로 고정한다.
    """
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in horizons}

    for uid, full_records in series.items():
        gt_records = gt_lookup[uid]

        working = _downsample(full_records, od_hz) if od_hz else list(full_records)
        if pos_std > 0 or heading_std_deg > 0:
            working = _apply_detection_noise(working, pos_std, heading_std_deg, rng)
        if occlusion_prob > 0:
            working = _apply_occlusion_bursts(working, occlusion_prob, occlusion_burst_len, rng)

        if len(working) < 2:
            continue

        for i in range(1, len(working)):
            iv = _instantaneous_v_yawrate(working[i - 1], working[i])
            if iv is None:
                continue
            v, yaw_rate = iv
            cur = working[i]
            for h in horizons:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, yaw_rate, h)
                gt_future = _nearest_record(gt_records, cur["t"] + h, max_dt=max_dt)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def print_distribution_table(title, errors_by_horizon):
    print("=" * 100)
    print(title)
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
    print("최종 Horizon 결정표 (10Hz + 실측 PointPainting 검출오차 결합 시나리오)")
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


def print_decomposition_table(raw_errors, rate_only_errors, combined_errors):
    print("\n" + "=" * 90)
    print("오차 분해: raw(고속GT) vs 10Hz만 vs 10Hz+검출오차(최종)  [mean 오차, 단위 m]")
    print("=" * 90)
    header = f"{'horizon':>8} {'raw':>10} {'10Hz만':>10} {'10Hz+검출오차':>14} {'rate 기여':>10} {'검출오차 기여':>12}"
    print(header)
    for h in COMPARE_HORIZONS:
        raw_v = raw_errors.get(h, [])
        rate_v = rate_only_errors.get(h, [])
        comb_v = combined_errors.get(h, [])
        raw_m = np.mean(raw_v) if raw_v else float("nan")
        rate_m = np.mean(rate_v) if rate_v else float("nan")
        comb_m = np.mean(comb_v) if comb_v else float("nan")
        rate_contrib = rate_m - raw_m
        noise_contrib = comb_m - rate_m
        print(
            f"{h:>8.1f} {raw_m:>10.3f} {rate_m:>10.3f} {comb_m:>14.3f} "
            f"{rate_contrib:>10.3f} {noise_contrib:>12.3f}"
        )
    print("=" * 90)
    print("'rate 기여' = 10Hz만 - raw  (다운샘플링만으로 늘어난 오차)")
    print("'검출오차 기여' = 10Hz+검출오차 - 10Hz만  (실측 PointPainting 노이즈가 추가로 늘린 오차)")


def make_plots(label, combined_errors, raw_errors, rate_only_errors):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    # --- 그림 1: 최종(combined) 시나리오의 박스플롯 + 퍼센타일 곡선 ---
    box_horizons = HORIZONS[::3]
    box_data = [combined_errors[h] for h in box_horizons]

    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))

    ax = axes[0]
    ax.boxplot(box_data, labels=[f"{h:.1f}" for h in box_horizons], showfliers=False)
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 10Hz+검출오차 결합 시나리오 horizon별 오차 분포", fontproperties=kfont)
    ax.grid(alpha=0.3, axis="y")

    ax = axes[1]
    p50s = [np.percentile(combined_errors[h], 50) if combined_errors[h] else np.nan for h in HORIZONS]
    p90s = [np.percentile(combined_errors[h], 90) if combined_errors[h] else np.nan for h in HORIZONS]
    p95s = [np.percentile(combined_errors[h], 95) if combined_errors[h] else np.nan for h in HORIZONS]
    ax.plot(HORIZONS, p50s, marker="o", markersize=3, label="p50(중앙값)", color="#2f855a")
    ax.plot(HORIZONS, p90s, marker="o", markersize=3, label="p90", color="#dd6b20")
    ax.plot(HORIZONS, p95s, marker="o", markersize=3, label="p95", color="#c53030")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 퍼센타일별 오차 곡선 (최종 결합 시나리오)", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out1 = f"final_horizon_distribution_{label}.png"
    plt.savefig(out1, dpi=150)
    print(f"\n그래프 저장됨: {out1}")

    # --- 그림 2: raw / 10Hz만 / 10Hz+검출오차 비교 (오차 분해) ---
    fig2, ax2 = plt.subplots(figsize=(8, 5.5))
    means_raw = [np.mean(raw_errors[h]) if raw_errors[h] else np.nan for h in HORIZONS]
    means_rate = [np.mean(rate_only_errors[h]) if rate_only_errors[h] else np.nan for h in HORIZONS]
    means_comb = [np.mean(combined_errors[h]) if combined_errors[h] else np.nan for h in HORIZONS]
    ax2.plot(HORIZONS, means_raw, marker="o", markersize=3, label="raw(고속GT)", color="#2b6cb0")
    ax2.plot(HORIZONS, means_rate, marker="o", markersize=3, label="10Hz만(다운샘플)", color="#dd6b20")
    ax2.plot(HORIZONS, means_comb, marker="o", markersize=3, label="10Hz+검출오차(최종)", color="#c53030")
    ax2.set_xlabel("horizon (s)", fontproperties=kfont)
    ax2.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax2.set_title(f"[{label}] 오차 원인 분해: rate vs 검출오차", fontproperties=kfont)
    ax2.legend(prop=kfont)
    ax2.grid(alpha=0.3)
    plt.tight_layout()
    out2 = f"final_horizon_comparison_{label}.png"
    plt.savefig(out2, dpi=150)
    print(f"그래프 저장됨: {out2}")

    # --- 그림 3: 최종 시나리오의 horizon별 CDF ---
    fig3, ax3 = plt.subplots(figsize=(7, 5.5))
    sample_horizons = [0.5, 1.0, 1.5, 2.0, 3.0]
    colors = ["#2b6cb0", "#2f855a", "#dd6b20", "#c53030", "#805ad5"]
    for h, c in zip(sample_horizons, colors):
        vals = np.sort(combined_errors[h])
        if len(vals) == 0:
            continue
        cdf = np.arange(1, len(vals) + 1) / len(vals)
        ax3.plot(vals, cdf, label=f"{h}s", color=c)
    ax3.set_xlabel("오차 (m)", fontproperties=kfont)
    ax3.set_ylabel("누적 확률 (이 오차 이하일 확률)", fontproperties=kfont)
    ax3.set_title(f"[{label}] 최종 결합 시나리오 horizon별 오차 CDF", fontproperties=kfont)
    ax3.legend(title="horizon", prop=kfont)
    ax3.set_xlim(0, 10)
    ax3.grid(alpha=0.3)
    plt.tight_layout()
    out3 = f"final_horizon_cdf_{label}.png"
    plt.savefig(out3, dpi=150)
    print(f"그래프 저장됨: {out3}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--od-hz", type=float, default=OD_HZ_DEFAULT,
                         help=f"OD 입력 주기 가정 (기본 {OD_HZ_DEFAULT}Hz, eval_input_rate_sweep.py 결과로 결정된 값)")
    parser.add_argument("--tp-center-error-mean", type=float, default=TP_CENTER_ERROR_MEAN_DEFAULT,
                         help=f"실측 PointPainting TP 중심오차 평균(m), 기본 {TP_CENTER_ERROR_MEAN_DEFAULT}m")
    parser.add_argument("--pos-noise-std", type=float, default=None,
                         help="직접 표준편차(m)를 지정하고 싶을 때 사용 (지정 시 --tp-center-error-mean 무시)")
    parser.add_argument("--heading-noise-std", type=float, default=0.0,
                         help="heading 노이즈 표준편차(deg). 이번 PointPainting 실측에는 포함 안 됨 (기본 0)")
    parser.add_argument("--occlusion-prob", type=float, default=0.0)
    parser.add_argument("--occlusion-burst-len", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    # mean 중심오차 -> 가우시안 표준편차 (Rayleigh 역산), --pos-noise-std로 override 가능
    if args.pos_noise_std is not None:
        pos_std = args.pos_noise_std
        print(f"\n[검출오차 설정] --pos-noise-std 직접 지정: sigma = {pos_std:.4f}m")
    else:
        pos_std = args.tp_center_error_mean / math.sqrt(math.pi / 2.0)
        print(
            f"\n[검출오차 설정] 실측 PointPainting TP 중심오차 mean = {args.tp_center_error_mean:.3f}m "
            f"-> Rayleigh 역산 sigma = mean / sqrt(pi/2) = {pos_std:.4f}m"
        )
    print(f"[OD 주기 설정] {args.od_hz:.1f}Hz 고정 (eval_input_rate_sweep.py 스윕 결과 기반)")
    if args.heading_noise_std > 0:
        print(f"[heading 노이즈] std = {args.heading_noise_std:.2f}deg (실측 아님, 수동 지정)")
    if args.occlusion_prob > 0:
        print(f"[occlusion] prob={args.occlusion_prob}, burst_len={args.occlusion_burst_len}프레임")

    gt_lookup = series  # 항상 원본 고속 GT를 정답으로 사용

    # (A) raw: 다운샘플/노이즈 없음 - 베이스라인
    raw_errors = evaluate_scenario(series, gt_lookup, HORIZONS, od_hz=None, seed=args.seed)

    # (B) 10Hz만: OD 주기 효과만 분리
    rate_only_errors = evaluate_scenario(series, gt_lookup, HORIZONS, od_hz=args.od_hz, seed=args.seed)

    # (C) 10Hz + 실측 검출오차: 최종 결합 시나리오
    combined_errors = evaluate_scenario(
        series, gt_lookup, HORIZONS,
        od_hz=args.od_hz, pos_std=pos_std, heading_std_deg=args.heading_noise_std,
        occlusion_prob=args.occlusion_prob, occlusion_burst_len=args.occlusion_burst_len,
        seed=args.seed,
    )

    print_distribution_table(
        f"[{args.label}] 최종 결합 시나리오 (OD {args.od_hz:.0f}Hz + 검출오차 sigma={pos_std:.3f}m) horizon별 오차 분포",
        combined_errors,
    )
    print_decision_table(combined_errors)
    print_decomposition_table(raw_errors, rate_only_errors, combined_errors)
    make_plots(args.label, combined_errors, raw_errors, rate_only_errors)


if __name__ == "__main__":
    main()
