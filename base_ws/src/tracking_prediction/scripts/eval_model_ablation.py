#!/usr/bin/env python3
"""
eval_model_ablation.py

eval_final_horizon_decision_ctra.py에서 "CTRA+감쇠가속도+스무딩"이 기존 CTRV보다
오히려 나빠지는 결과가 나왔는데, 그 실험은 CTRA 전환과 스무딩을 동시에 바꿔서
어느 쪽이 원인인지 구분이 안 됐다. 이 스크립트는 두 축을 분리해서 2x2(모델 x
스무딩 여부) 전부 비교한다.

    모델: CTRV(가속도 없음) vs CTRA-감쇠(가속도 있음, lambda 감쇠)
    스무딩: 없음(raw 단일프레임 유한차분) vs 있음(다운샘플된 시퀀스 내 5프레임 이동평균)

4가지 조합 전부 "동일한" OD 10Hz 다운샘플 + 동일한 노이즈 realization(같은 seed로
한 번만 생성한 뒤 재사용)에 대해 평가해서 공정하게 비교한다.

가설:
    - CTRA는 CTRV보다 1/omega, 1/omega^2 항에 가속도(a)가 추가로 곱해져서, 이미
      불안정한 소(小)omega 영역(로터리 전환 구간)에서 오차가 더 증폭될 수 있음.
    - 스무딩은 노이즈를 줄이지만, 회전이 "막 시작/끝나는" 전환 순간 자체를
      평균으로 뭉개버려서 그 순간의 예측을 오히려 부정확하게 만들 수도 있음.
    - 이 스크립트로 두 효과를 분리해서 실제로 어느 쪽이 개선/악화의 원인인지 확인.

사용법:
    python3 eval_model_ablation.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40
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
OMEGA_EPS = 1e-3

OD_HZ_DEFAULT = 10.0
TP_CENTER_ERROR_MEAN_DEFAULT = 0.145
LAMBDA_DECAY_DEFAULT = 0.4
SMOOTH_WINDOW_DEFAULT = 5

HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]
COMPARE_HORIZONS = [0.5, 1.0, 1.5, 2.0, 3.0]

ACCEL_CLIP_DEFAULT = 4.0  # m/s^2, 일반 주행 가감속의 물리적으로 합리적인 범위

VARIANTS = ["ctrv_raw", "ctrv_smooth", "ctra_raw", "ctra_smooth", "ctra_clip"]
VARIANT_LABELS = {
    "ctrv_raw": "CTRV/raw",
    "ctrv_smooth": "CTRV/smooth",
    "ctra_raw": "CTRA/raw",
    "ctra_smooth": "CTRA/smooth",
    "ctra_clip": "CTRA/clip",
}


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt):
    if abs(yaw_rate) < 1e-4:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
    yaw_next = yaw + yaw_rate * dt
    x_next = x + (v / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
    y_next = y + (v / yaw_rate) * (math.cos(yaw) - math.cos(yaw_next))
    return x_next, y_next


def ctra_predict_decay_a(x, y, theta, v, omega, a, dt, lam):
    if abs(omega) < OMEGA_EPS:
        if lam < 1e-6:
            dist = v * dt + 0.5 * a * dt * dt
        else:
            dist = v * dt + (a / lam) * (dt - (1.0 / lam) * (1 - math.exp(-lam * dt)))
        return x + dist * math.cos(theta), y + dist * math.sin(theta)

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    x_const = v * (sin_tn - sin_t) / omega
    y_const = v * (cos_t - cos_tn) / omega

    if lam < 1e-6:
        x_next = x + x_const + (
            (a * dt * sin_tn) / omega - (a * cos_t) / (omega ** 2) + (a * cos_tn) / (omega ** 2)
        )
        y_next = y + y_const - (
            (a * dt * cos_tn) / omega + (a * sin_t) / (omega ** 2) - (a * sin_tn) / (omega ** 2)
        )
        return x_next, y_next

    denom = lam * lam + omega * omega
    ic = (
        math.exp(-lam * dt) * (-lam * cos_tn + omega * sin_tn)
        - (-lam * cos_t + omega * sin_t)
    ) / denom
    is_ = (
        math.exp(-lam * dt) * (-lam * sin_tn - omega * cos_tn)
        - (-lam * sin_t - omega * cos_t)
    ) / denom

    x_decay = (a / lam) * ((sin_tn - sin_t) / omega - ic)
    y_decay = (a / lam) * ((cos_t - cos_tn) / omega - is_)

    return x + x_const + x_decay, y + y_const + y_decay


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


def _raw_omega_v_a(records, i):
    if i == 0 or i >= len(records):
        return None
    prev, cur = records[i - 1], records[i]
    dt = cur["t"] - prev["t"]
    if dt <= 0:
        return None
    dtheta = cur["heading"] - prev["heading"]
    dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
    omega = dtheta / dt
    v_cur = math.hypot(cur["vx"], cur["vy"])
    if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
        v_cur = -v_cur
    v_prev = math.hypot(prev["vx"], prev["vy"])
    if _project_to_scalar(prev["vx"], prev["vy"], prev["heading"]) < 0:
        v_prev = -v_prev
    a = (v_cur - v_prev) / dt
    return omega, v_cur, a


def _build_smoothed_series(records, smooth_window):
    raw = [_raw_omega_v_a(records, i) for i in range(len(records))]
    smoothed = [None] * len(records)
    half = smooth_window // 2
    for i in range(len(records)):
        window = [raw[j] for j in range(max(0, i - half), min(len(records), i + half + 1)) if raw[j] is not None]
        if not window:
            continue
        smoothed[i] = (
            np.mean([w[0] for w in window]),
            np.mean([w[1] for w in window]),
            np.mean([w[2] for w in window]),
        )
    return smoothed


def build_noisy_working_series(series, od_hz, pos_std, heading_std_deg, seed):
    """모든 variant가 공유할 '동일한' 다운샘플+노이즈 데이터를 한 번만 생성."""
    rng = random.Random(seed)
    working = {}
    for uid in sorted(series.keys()):
        full_records = series[uid]
        w = _downsample(full_records, od_hz)
        if pos_std > 0 or heading_std_deg > 0:
            w = _apply_detection_noise(w, pos_std, heading_std_deg, rng)
        working[uid] = w
    return working


def evaluate_variant(working_series, gt_lookup, variant, lam, smooth_window, accel_clip=None):
    errors_by_horizon = {h: [] for h in HORIZONS}
    use_ctra = variant.startswith("ctra")
    use_smooth = variant in ("ctrv_smooth", "ctra_smooth", "ctra_clip")
    use_clip = variant == "ctra_clip"

    for uid, working in working_series.items():
        if len(working) < 2:
            continue
        gt_records = gt_lookup[uid]

        smoothed = _build_smoothed_series(working, smooth_window) if use_smooth else None

        for i in range(1, len(working)):
            if use_smooth:
                if smoothed[i] is None:
                    continue
                omega, v, a = smoothed[i]
            else:
                iv = _raw_omega_v_a(working, i)
                if iv is None:
                    continue
                omega, v, a = iv

            if use_clip and accel_clip is not None:
                a = max(-accel_clip, min(accel_clip, a))

            cur = working[i]
            for h in HORIZONS:
                if use_ctra:
                    px, py = ctra_predict_decay_a(cur["x"], cur["y"], cur["heading"], v, omega, a, h, lam)
                else:
                    px, py = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, omega, h)
                gt_future = _nearest_record(gt_records, cur["t"] + h, max_dt=0.3)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def print_stat_table(title, stat_fn, results):
    print("\n" + "=" * 100)
    print(title)
    print("=" * 100)
    header = f"{'horizon':>8}" + "".join(f"{VARIANT_LABELS[v]:>14}" for v in VARIANTS)
    print(header)
    for h in COMPARE_HORIZONS:
        row = f"{h:>8.1f}"
        for v in VARIANTS:
            vals = results[v][h]
            row += f"{stat_fn(vals):>14.3f}" if vals else f"{'N/A':>14}"
        print(row)
    print("=" * 100)


def make_plots(label, results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    colors = {
        "ctrv_raw": "#c53030", "ctrv_smooth": "#dd6b20", "ctra_raw": "#2b6cb0",
        "ctra_smooth": "#805ad5", "ctra_clip": "#2f855a",
    }

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    ax = axes[0]
    for v in VARIANTS:
        means = [np.mean(results[v][h]) if results[v][h] else np.nan for h in HORIZONS]
        ax.plot(HORIZONS, means, marker="o", markersize=3, label=VARIANT_LABELS[v], color=colors[v])
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 모델x스무딩 2x2 평균 오차", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    for v in VARIANTS:
        p95s = [np.percentile(results[v][h], 95) if results[v][h] else np.nan for h in HORIZONS]
        ax.plot(HORIZONS, p95s, marker="o", markersize=3, label=VARIANT_LABELS[v], color=colors[v])
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("p95 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 모델x스무딩 2x2 p95 오차(꼬리)", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"model_ablation_{label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--od-hz", type=float, default=OD_HZ_DEFAULT)
    parser.add_argument("--tp-center-error-mean", type=float, default=TP_CENTER_ERROR_MEAN_DEFAULT)
    parser.add_argument("--pos-noise-std", type=float, default=None)
    parser.add_argument("--heading-noise-std", type=float, default=0.0)
    parser.add_argument("--lambda-decay", type=float, default=LAMBDA_DECAY_DEFAULT)
    parser.add_argument("--smooth-window", type=int, default=SMOOTH_WINDOW_DEFAULT)
    parser.add_argument("--accel-clip", type=float, default=ACCEL_CLIP_DEFAULT,
                         help=f"CTRA/clip 변형에서 가속도 추정치를 이 값(m/s^2)으로 clip (기본 {ACCEL_CLIP_DEFAULT})")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    if args.pos_noise_std is not None:
        pos_std = args.pos_noise_std
    else:
        pos_std = args.tp_center_error_mean / math.sqrt(math.pi / 2.0)
    print(f"[검출오차] mean={args.tp_center_error_mean:.3f}m -> sigma={pos_std:.4f}m, OD={args.od_hz:.0f}Hz")
    print(f"[CTRA] lambda={args.lambda_decay}, smooth_window={args.smooth_window}프레임(다운샘플 기준), accel_clip=±{args.accel_clip}m/s²")
    print("5가지 조합(CTRV raw/smooth, CTRA raw/smooth/clip) 전부 동일한 노이즈 realization으로 평가합니다.\n")

    working_series = build_noisy_working_series(series, args.od_hz, pos_std, args.heading_noise_std, args.seed)

    results = {}
    for v in VARIANTS:
        results[v] = evaluate_variant(working_series, series, v, args.lambda_decay, args.smooth_window, args.accel_clip)

    print_stat_table(f"[{args.label}] 평균(mean) 오차 비교 (단위 m)", lambda vals: np.mean(vals), results)
    print_stat_table(f"[{args.label}] p90 오차 비교 (단위 m)", lambda vals: np.percentile(vals, 90), results)
    print_stat_table(f"[{args.label}] p95 오차 비교 (단위 m)", lambda vals: np.percentile(vals, 95), results)
    print_stat_table(f"[{args.label}] 중앙값(p50) 오차 비교 (단위 m)", lambda vals: np.percentile(vals, 50), results)

    make_plots(args.label, results)


if __name__ == "__main__":
    main()
