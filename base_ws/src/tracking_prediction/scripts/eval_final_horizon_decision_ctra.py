#!/usr/bin/env python3
"""
eval_final_horizon_decision_ctra.py

eval_final_horizon_decision.py(CTRV, 단일프레임 유한차분)의 후속 실험.
그동안 알아낸 것들을 전부 합쳐서 "지금까지 찾은 개선안이 실제로 얼마나
효과 있는지"를 같은 조건(OD 10Hz + 실측 PointPainting 검출오차)에서 직접 비교한다.

지금까지 발견한 것 정리:
    1. eval_final_horizon_decision.py: p25는 3초까지도 낮은데 p90/95는 처음부터
       붕괴 - "전체가 서서히 나빠지는" 게 아니라 "일부가 처음부터 갈라져 나감".
    2. analyze_jitter_outliers.py: 그 이상치들이 로터리의 특정 좌표(급커브/합류부로
       추정)에 몰려있음 - 여러 차량이 각자 다른 시간에 같은 지점을 지나며 똑같이 튐.
    3. analyze_ctra_error_vs_variability_v2.py: 오차는 yaw_rate 변동성보다
       "가속도가 t0 값에서 얼마나 벗어나는지"와 더 강하게 상관됨 (팀 자체 분석 r=0.94).
    4. compare_ctra_decay.py: 등가속도 CTRA보다 감쇠가속도 CTRA(a(t)=a0*exp(-lambda*t))가
       FDE를 더 줄임 (팀 검증값 lambda=0.4).
    5. compare_ctra_decay.py / analyze_ctra_error_vs_variability_v2.py 둘 다
       raw 단일프레임 유한차분 대신 5프레임 이동평균으로 omega/v/a를 스무딩함
       -> 미분 노이즈(지터의 원인 중 하나)를 줄임.

이 스크립트가 하는 일:
    OD 10Hz 다운샘플 + 실측 검출오차(mean=0.145m, Rayleigh 역산) 조건은 그대로 두고,
    "예측 모델"만 두 가지로 바꿔서 같은 데이터에 직접 비교한다.
        A) ctrv_raw       : 기존 eval_final_horizon_decision.py와 동일 (CTRV, 스무딩 없음)
        B) ctra_smoothed  : CTRA + 감쇠가속도(lambda) + omega/v/a 5프레임 이동평균 스무딩

    중요: 스무딩 윈도우는 "다운샘플된(10Hz) 시퀀스 자체"에 적용한다. 원본 48.7Hz
    시퀀스에 적용하면 실제 OD가 10Hz로만 데이터를 준다는 가정과 모순되기 때문
    (실제 시스템은 10Hz로 들어온 값들 안에서만 스무딩할 수 있음). 스무딩 윈도우는
    가운데(미래 포함) 방식이라 오프라인 리서치용이며, 실제 온라인 EKF에 넣을 땐
    과거 프레임만 쓰는 causal 윈도우로 바꿔야 한다 (팀 기존 스크립트들과 동일한 제약).

사용법:
    python3 eval_final_horizon_decision_ctra.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40

    # lambda나 스무딩 창을 바꿔보고 싶으면:
    python3 eval_final_horizon_decision_ctra.py <bag_path> --label roundabout \
        --lambda-decay 0.4 --smooth-window 5
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
LAMBDA_DECAY_DEFAULT = 0.4  # 팀 기존 검증값 (compare_ctra_decay.py)
SMOOTH_WINDOW_DEFAULT = 5

HORIZONS = [round(0.1 * i, 1) for i in range(1, 31)]

DECISION_CRITERIA = [
    (1.0, 50), (1.0, 90), (1.0, 95),
    (2.0, 50), (2.0, 90), (2.0, 95),
    (3.0, 50), (3.0, 90), (3.0, 95),
    (5.0, 90), (5.0, 95),
]

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


def ctra_predict_decay_a(x, y, theta, v, omega, a, dt, lam):
    """감쇠가속도 CTRA: a(t) = a * exp(-lam*t). (compare_ctra_decay.py와 동일 공식)"""
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
        # 감쇠 없음(lambda~0) -> 기존 등가속도 CTRA 공식으로 정확히 환원 (omega!=0 분기)
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
    """다운샘플(+노이즈)된 시퀀스 자체에서 omega/v/a를 5프레임(기본) 이동평균으로 스무딩.
    (compare_ctra_decay.py의 _build_smoothed_series와 동일 로직, OD가 실제로
    줄 수 있는 데이터 안에서만 스무딩한다는 점만 다름)"""
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


def evaluate_ctrv_raw(series, od_hz, pos_std, heading_std_deg, seed):
    """베이스라인: 기존 eval_final_horizon_decision.py와 동일한 CTRV(스무딩 없음)."""
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in HORIZONS}

    for uid, full_records in series.items():
        working = _downsample(full_records, od_hz)
        if pos_std > 0 or heading_std_deg > 0:
            working = _apply_detection_noise(working, pos_std, heading_std_deg, rng)
        if len(working) < 2:
            continue

        for i in range(1, len(working)):
            iv = _raw_omega_v_a(working, i)
            if iv is None:
                continue
            omega, v, _a = iv
            cur = working[i]
            for h in HORIZONS:
                px, py, _ = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, omega, h)
                gt_future = _nearest_record(full_records, cur["t"] + h, max_dt=0.3)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_horizon[h].append(err)

    return errors_by_horizon


def evaluate_ctra_smoothed(series, od_hz, pos_std, heading_std_deg, lam, smooth_window, seed):
    """개선안: CTRA + 감쇠가속도 + (다운샘플된 시퀀스 내에서의) omega/v/a 스무딩."""
    rng = random.Random(seed)
    errors_by_horizon = {h: [] for h in HORIZONS}

    for uid, full_records in series.items():
        working = _downsample(full_records, od_hz)
        if pos_std > 0 or heading_std_deg > 0:
            working = _apply_detection_noise(working, pos_std, heading_std_deg, rng)
        if len(working) < 2:
            continue

        smoothed = _build_smoothed_series(working, smooth_window)

        for i in range(1, len(working)):
            if smoothed[i] is None:
                continue
            omega0, v0, a0 = smoothed[i]
            cur = working[i]
            for h in HORIZONS:
                px, py = ctra_predict_decay_a(cur["x"], cur["y"], cur["heading"], v0, omega0, a0, h, lam)
                gt_future = _nearest_record(full_records, cur["t"] + h, max_dt=0.3)
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


def print_decision_table(title, errors_by_horizon):
    print("\n" + "=" * 70)
    print(title)
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
                break
        if best_h is not None:
            print(f"  {threshold:>4.1f}m 이내를 {pct:>2d}% 확률로 원하면  ->  horizon <= {best_h:.1f}s 가능")
        else:
            print(f"  {threshold:>4.1f}m 이내를 {pct:>2d}% 확률로 원하면  ->  0.1s부터 이미 불가능")
    print("=" * 70)


def print_ab_comparison(ctrv_errors, ctra_errors):
    print("\n" + "=" * 100)
    print("A/B 비교: CTRV(스무딩 없음) vs CTRA+감쇠가속도+스무딩  [mean/p90/p95, 단위 m]")
    print("=" * 100)
    header = f"{'horizon':>8} {'CTRV mean':>10} {'CTRA mean':>10} {'개선율':>8} {'CTRV p90':>10} {'CTRA p90':>10} {'CTRV p95':>10} {'CTRA p95':>10}"
    print(header)
    for h in COMPARE_HORIZONS:
        cv, ca = ctrv_errors.get(h, []), ctra_errors.get(h, [])
        if not cv or not ca:
            continue
        cv_mean, ca_mean = np.mean(cv), np.mean(ca)
        improve = (cv_mean - ca_mean) / cv_mean * 100 if cv_mean > 0 else float("nan")
        cv_p90, ca_p90 = np.percentile(cv, 90), np.percentile(ca, 90)
        cv_p95, ca_p95 = np.percentile(cv, 95), np.percentile(ca, 95)
        print(
            f"{h:>8.1f} {cv_mean:>10.3f} {ca_mean:>10.3f} {improve:>7.1f}% "
            f"{cv_p90:>10.3f} {ca_p90:>10.3f} {cv_p95:>10.3f} {ca_p95:>10.3f}"
        )
    print("=" * 100)


def make_plots(label, ctrv_errors, ctra_errors):
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
    means_ctrv = [np.mean(ctrv_errors[h]) if ctrv_errors[h] else np.nan for h in HORIZONS]
    means_ctra = [np.mean(ctra_errors[h]) if ctra_errors[h] else np.nan for h in HORIZONS]
    ax.plot(HORIZONS, means_ctrv, marker="o", markersize=3, label="CTRV(기존)", color="#c53030")
    ax.plot(HORIZONS, means_ctra, marker="o", markersize=3, label="CTRA+감쇠+스무딩(개선안)", color="#2f855a")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] 평균 오차: CTRV vs CTRA 개선안", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    p95_ctrv = [np.percentile(ctrv_errors[h], 95) if ctrv_errors[h] else np.nan for h in HORIZONS]
    p95_ctra = [np.percentile(ctra_errors[h], 95) if ctra_errors[h] else np.nan for h in HORIZONS]
    ax.plot(HORIZONS, p95_ctrv, marker="o", markersize=3, label="CTRV(기존) p95", color="#c53030")
    ax.plot(HORIZONS, p95_ctra, marker="o", markersize=3, label="CTRA 개선안 p95", color="#2f855a")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("p95 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] p95 오차(꼬리): CTRV vs CTRA 개선안", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"ctrv_vs_ctra_{label}.png"
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
    parser.add_argument("--lambda-decay", type=float, default=LAMBDA_DECAY_DEFAULT,
                         help=f"CTRA 감쇠계수 (기본 {LAMBDA_DECAY_DEFAULT}, 팀 compare_ctra_decay.py 검증값)")
    parser.add_argument("--smooth-window", type=int, default=SMOOTH_WINDOW_DEFAULT,
                         help=f"omega/v/a 이동평균 스무딩 창(다운샘플된 프레임 수, 기본 {SMOOTH_WINDOW_DEFAULT})")
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
    print(f"[CTRA 개선안] lambda={args.lambda_decay}, smooth_window={args.smooth_window}프레임(다운샘플 기준)")

    ctrv_errors = evaluate_ctrv_raw(series, args.od_hz, pos_std, args.heading_noise_std, args.seed)
    ctra_errors = evaluate_ctra_smoothed(
        series, args.od_hz, pos_std, args.heading_noise_std, args.lambda_decay, args.smooth_window, args.seed
    )

    print_distribution_table(f"[{args.label}] CTRA+감쇠+스무딩 (개선안) horizon별 오차 분포", ctra_errors)
    print_decision_table(f"[{args.label}] CTRA 개선안 Horizon 결정표", ctra_errors)
    print_ab_comparison(ctrv_errors, ctra_errors)
    make_plots(args.label, ctrv_errors, ctra_errors)


if __name__ == "__main__":
    main()
