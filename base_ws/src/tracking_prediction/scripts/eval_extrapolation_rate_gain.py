#!/usr/bin/env python3
"""
eval_extrapolation_rate_gain.py

배경: 판단(의사결정) 모듈이 동작하려면 입력이 mean 20Hz는 나와야 하는데, OD는
10Hz(스윕 결과로는 8~15Hz 범위) 밖에 못 준다. 그 차이를 tracking/prediction의
extrapolation(CTRV로 실시간 롤아웃)으로 메워서 20Hz를 만들어 낼 계획인데,
"그렇게 메운 값이 얼마나 부정확한지"를 수치로 확인해야 한다.

방법:
    OD가 od_hz로 들어온다고 하면, 실제 검출 사이 간격(interval = 1/od_hz) 안에서
    20Hz(target_hz)를 만족하려면 그 간격을 steps = ceil(interval * target_hz)개의
    조각으로 나눠서, 그 사이사이를 CTRV 외삽으로 채워야 한다. 예를 들어 OD가
    10Hz(interval=0.1s)면 20Hz를 만들기 위해 2조각(steps=2)으로 나눠 그 중간
    (0.05s 뒤)에 외삽 1개를 끼워 넣어야 한다.

    이 외삽 지점들 중, 마지막 실제 검출로부터 가장 멀리 떨어진 지점
    (= 다음 실제 검출이 오기 직전, 정보가 가장 오래된 시점)이 제일 부정확할
    것이므로 그 오차를 "최악의 경우"로 리포트한다. 평균/percentile도 같이 낸다.

    모델은 CTRV로 고정한다 (CTRV vs CTRA 비교 결과 CTRV로 최종 결정했으므로).
    검출오차(실측 PointPainting TP 중심오차 mean=0.145m)도 그대로 반영한다.

사용법:
    python3 eval_extrapolation_rate_gain.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40 \
        --target-hz 20
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

OD_HZ_LIST_DEFAULT = [8, 9, 10, 11, 12, 13, 14, 15]
TARGET_HZ_DEFAULT = 20.0
TP_CENTER_ERROR_MEAN_DEFAULT = 0.145


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctrv_predict(x, y, yaw, v, yaw_rate, dt):
    if abs(yaw_rate) < 1e-4:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
    yaw_next = yaw + yaw_rate * dt
    x_next = x + (v / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
    y_next = y + (v / yaw_rate) * (math.cos(yaw) - math.cos(yaw_next))
    return x_next, y_next


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


def _raw_omega_v(records, i):
    if i == 0 or i >= len(records):
        return None
    prev, cur = records[i - 1], records[i]
    dt = cur["t"] - prev["t"]
    if dt <= 0:
        return None
    dtheta = cur["heading"] - prev["heading"]
    dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
    omega = dtheta / dt
    v = math.hypot(cur["vx"], cur["vy"])
    if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
        v = -v
    return omega, v


def evaluate_extrapolation_offsets(series, od_hz, offsets, pos_std, heading_std_deg, seed):
    """od_hz로 다운샘플+노이즈 적용한 뒤, 각 실제 검출 시점 기준으로 offsets(초) 뒤를
    CTRV로 외삽했을 때의 오차를 offset별로 계산 (정답은 원본 고속 GT)."""
    rng = random.Random(seed)
    errors_by_offset = {off: [] for off in offsets}

    for uid, full_records in series.items():
        working = _downsample(full_records, od_hz)
        if pos_std > 0 or heading_std_deg > 0:
            working = _apply_detection_noise(working, pos_std, heading_std_deg, rng)
        if len(working) < 2:
            continue

        for i in range(1, len(working)):
            iv = _raw_omega_v(working, i)
            if iv is None:
                continue
            omega, v = iv
            cur = working[i]
            for off in offsets:
                px, py = ctrv_predict(cur["x"], cur["y"], cur["heading"], v, omega, off)
                gt_future = _nearest_record(full_records, cur["t"] + off, max_dt=0.05)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors_by_offset[off].append(err)

    return errors_by_offset


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--od-hz-list", default=",".join(str(h) for h in OD_HZ_LIST_DEFAULT))
    parser.add_argument("--target-hz", type=float, default=TARGET_HZ_DEFAULT)
    parser.add_argument("--tp-center-error-mean", type=float, default=TP_CENTER_ERROR_MEAN_DEFAULT)
    parser.add_argument("--pos-noise-std", type=float, default=None)
    parser.add_argument("--heading-noise-std", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    od_hz_list = [float(x) for x in args.od_hz_list.split(",") if x.strip()]

    series = _extract_full_rate_series(args.bag_path, exclude_ids)
    print(f"평가 대상 궤적 수(텔레포트 분리 포함): {len(series)}")

    if args.pos_noise_std is not None:
        pos_std = args.pos_noise_std
    else:
        pos_std = args.tp_center_error_mean / math.sqrt(math.pi / 2.0)
    print(f"[검출오차] mean={args.tp_center_error_mean:.3f}m -> sigma={pos_std:.4f}m")
    print(f"[목표] 판단 모듈이 필요로 하는 출력: {args.target_hz:.0f}Hz (mean)\n")

    print("=" * 110)
    print(f"[{args.label}] OD hz별 {args.target_hz:.0f}Hz 달성에 필요한 extrapolation 및 그 오차")
    print("=" * 110)
    header = (
        f"{'OD hz':>7} {'검출간격(s)':>10} {'필요 조각수':>10} {'실제달성hz':>10} "
        f"{'최악 외삽gap(s)':>14} {'최악gap mean err':>16} {'최악gap p90 err':>15} {'최악gap p95 err':>15}"
    )
    print(header)

    rows = []
    for od_hz in od_hz_list:
        interval = 1.0 / od_hz
        steps = max(1, math.ceil(interval * args.target_hz))
        sub_dt = interval / steps
        achieved_hz = od_hz * steps

        # steps개 조각 중, 실제 검출로부터 가장 먼(다음 검출 직전) 외삽 지점
        worst_offset = round(sub_dt * (steps - 1), 4) if steps > 1 else 0.0
        # 참고용으로 전체 offset도 계산(그래프/추가분석 대비)
        offsets = sorted(set([round(sub_dt * k, 4) for k in range(1, steps)] + [worst_offset])) if steps > 1 else [0.0]
        if not offsets:
            offsets = [sub_dt]

        errors_by_offset = evaluate_extrapolation_offsets(
            series, od_hz, offsets, pos_std, args.heading_noise_std, args.seed
        )
        worst_vals = errors_by_offset.get(worst_offset, [])
        if worst_vals:
            mean_e = np.mean(worst_vals)
            p90_e = np.percentile(worst_vals, 90)
            p95_e = np.percentile(worst_vals, 95)
        else:
            mean_e = p90_e = p95_e = float("nan")

        rows.append((od_hz, interval, steps, achieved_hz, worst_offset, mean_e, p90_e, p95_e))
        print(
            f"{od_hz:>7.0f} {interval:>10.4f} {steps:>10d} {achieved_hz:>10.1f} "
            f"{worst_offset:>14.4f} {mean_e:>16.3f} {p90_e:>15.3f} {p95_e:>15.3f}"
        )
    print("=" * 110)
    print("'필요 조각수' = 검출 간격 하나를 몇 개로 쪼개야 목표 hz를 만족하는지 (조각수-1개가 실제 extrapolation)")
    print("'최악 외삽gap' = 다음 실제 검출이 오기 직전, 마지막 실제 검출로부터 가장 오래 외삽한 시점")
    print("이 gap에서의 오차가 작을수록 '20Hz를 extrapolation으로 채워도 안전하다'는 뜻")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    try:
        kfont = fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    except Exception:
        kfont = None
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    ax = axes[0]
    hz_vals = [r[0] for r in rows]
    mean_errs = [r[5] for r in rows]
    p90_errs = [r[6] for r in rows]
    p95_errs = [r[7] for r in rows]
    ax.plot(hz_vals, mean_errs, marker="o", label="mean", color="#2f855a")
    ax.plot(hz_vals, p90_errs, marker="o", label="p90", color="#dd6b20")
    ax.plot(hz_vals, p95_errs, marker="o", label="p95", color="#c53030")
    ax.set_xlabel("OD 입력 hz", fontproperties=kfont)
    ax.set_ylabel("최악 외삽gap 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] {args.target_hz:.0f}Hz 달성 시 최악 외삽 오차", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    ax = axes[1]
    worst_gaps = [r[4] for r in rows]
    ax.plot(hz_vals, worst_gaps, marker="o", color="#2b6cb0")
    ax.set_xlabel("OD 입력 hz", fontproperties=kfont)
    ax.set_ylabel("최악 외삽 gap (s)", fontproperties=kfont)
    ax.set_title(f"[{args.label}] OD hz별 최악 외삽 시간간격", fontproperties=kfont)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = f"extrapolation_rate_gain_{args.label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n그래프 저장됨: {out_path}")


if __name__ == "__main__":
    main()
