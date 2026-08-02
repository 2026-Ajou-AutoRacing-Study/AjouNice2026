#!/usr/bin/env python3
"""
eval_highway_ctrv_ctra.py

2026-08-01: 로터리 진입구간(eval_entry_zone_ctrv_ctra.py)에서 다듬은 평가 방법론
(생존편향 제거한 동일 표본집합, threshold별 커버리지, ADE vs FDE 비교)을 하이웨이에도
적용한다. 하이웨이는 로터리처럼 "진입 게이트/원형 교차로" 개념이 없으므로, 원 피팅이나
접근/이탈 판정 로직은 쓰지 않고 대신 **하이웨이 bag에 있는 모든 순환 차량(NPC)의 모든
샘플 시점**을 그대로 평가 대상으로 삼는다 (기존 tp_final_report.md 2-3절의 하이웨이
분석과 같은 범위, 다만 이번엔 생존편향 제거 + 다중 threshold + ADE까지 추가).

방법:
    1. /Object_topic에서 NPC(순환 차량) 궤적을 뽑는다 (텔레포트 지점은 분리).
    2. --sample-interval(기본 0.1s=10Hz) 간격으로 시점을 뽑아, 그 시점에 존재하는
       모든 NPC 위치를 표본으로 모은다 (로터리처럼 "게이트 근처 1대만" 고르지 않음).
    3. 그 표본 전체에 대해 CTRV/CTRA로 여러 horizon 예측 후 오차를 모은다.
    4. 가장 긴 horizon까지 GT가 있는 표본만으로 고정한(생존편향 제거) 뒤,
       mean/p50/p90/p95, threshold별(1/2/3/5m) 커버리지, ADE vs FDE 비교표를 출력한다.

사용법:
    python3 eval_highway_ctrv_ctra.py <highway_bag_path> --label highway
"""

import argparse
import csv
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6
TELEPORT_THRESHOLD_M = 15.0
OMEGA_EPS = 1e-4
DECAY_LAMBDA_DEFAULT = 0.4

HORIZONS_DEFAULT = [0.3, 0.5, 0.7, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _norm_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ctrv_predict(x, y, yaw, v, yaw_rate, dt):
    if abs(yaw_rate) < OMEGA_EPS:
        return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
    yaw_next = yaw + yaw_rate * dt
    x_next = x + (v / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
    y_next = y + (v / yaw_rate) * (math.cos(yaw) - math.cos(yaw_next))
    return x_next, y_next


def ctra_predict(x, y, theta, v, omega, a, dt, lam=DECAY_LAMBDA_DEFAULT):
    """감쇠가속도 CTRA (eval_entry_zone_ctrv_ctra.py와 동일 공식, lam=0.4 기본)."""
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
        x_decay = (a * dt * sin_tn) / omega - (a * cos_t) / (omega ** 2) + (a * cos_tn) / (omega ** 2)
        y_decay = -(a * dt * cos_tn) / omega - (a * sin_t) / (omega ** 2) + (a * sin_tn) / (omega ** 2)
        return x + x_const + x_decay, y + y_const + y_decay

    denom = lam * lam + omega * omega
    ic = (math.exp(-lam * dt) * (-lam * cos_tn + omega * sin_tn) - (-lam * cos_t + omega * sin_t)) / denom
    is_ = (math.exp(-lam * dt) * (-lam * sin_tn - omega * cos_tn) - (-lam * sin_t - omega * cos_t)) / denom
    x_decay = (a / lam) * ((sin_tn - sin_t) / omega - ic)
    y_decay = (a / lam) * ((cos_t - cos_tn) / omega - is_)
    return x + x_const + x_decay, y + y_const + y_decay


# ---------------------------------------------------------------------------
# 데이터 로딩 (eval_entry_zone_ctrv_ctra.py와 동일 로직)
# ---------------------------------------------------------------------------

def _extract_npc_series(bag_path, exclude_ids, topic="/Object_topic"):
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


# ---------------------------------------------------------------------------
# 샘플 수집: 로터리처럼 게이트 근처 1대만 고르지 않고, 매 시점 모든 NPC를 다 씀
# ---------------------------------------------------------------------------

def collect_all_samples(npc_series, sample_interval):
    t_values = sorted({rec["t"] for records in npc_series.values() for rec in records})
    sampled_t = []
    last_t = None
    for t in t_values:
        if last_t is None or t - last_t >= sample_interval:
            sampled_t.append(t)
            last_t = t

    samples = []
    for uid, records in npc_series.items():
        # 이 궤적(segment)에서 sampled_t와 가장 가까운(0.05s 이내) 레코드들만 모음
        idx = 0
        for t in sampled_t:
            best, best_diff = None, 0.05
            # records가 시간순 정렬돼 있으므로 선형 탐색으로 충분히 빠름(세그먼트당 길이 제한적)
            for rec in records:
                diff = abs(rec["t"] - t)
                if diff < best_diff:
                    best_diff, best = diff, rec
            if best is not None:
                samples.append((uid, best, records))
    samples.sort(key=lambda s: s[1]["t"])
    return samples, sampled_t


def count_episodes(samples, max_gap_s=0.5):
    if not samples:
        return 0, []
    episodes = []
    cur_uid, cur_t0, cur_t1, cur_n = samples[0][0], samples[0][1]["t"], samples[0][1]["t"], 1
    for uid, rec, _ in samples[1:]:
        t = rec["t"]
        if uid == cur_uid and t - cur_t1 <= max_gap_s:
            cur_t1, cur_n = t, cur_n + 1
        else:
            episodes.append((cur_uid, cur_t0, cur_t1, cur_n))
            cur_uid, cur_t0, cur_t1, cur_n = uid, t, t, 1
    episodes.append((cur_uid, cur_t0, cur_t1, cur_n))
    return len(episodes), episodes


def export_raw_csv(output_path, horizons, samples):
    """오차 통계로 요약하지 않고, GT/CTRV예측/CTRA예측 실제 좌표값을 그대로 CSV로 뽑는다
    (2026-08-01: 팀장님이 오차 대신 원본 값을 직접 보고 판단하고 싶다는 요청, 로터리
    버전 eval_entry_zone_ctrv_ctra.py의 export_raw_csv와 동일 형식 - 하이웨이는 접근/이탈
    구분이 없어 group 컬럼은 전부 "all")."""
    rows_written = 0
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "group", "uid", "t", "horizon",
            "gt_now_x", "gt_now_y",
            "gt_future_x", "gt_future_y",
            "ctrv_pred_x", "ctrv_pred_y", "ctrv_error_m",
            "ctra_pred_x", "ctra_pred_y", "ctra_error_m",
        ])
        for uid, rec, full_records in samples:
            idx = full_records.index(rec) if rec in full_records else None
            if idx is None or idx == 0:
                continue
            prev = full_records[idx - 1]
            dt = rec["t"] - prev["t"]
            if dt <= 0:
                continue
            dtheta = _norm_angle(rec["heading"] - prev["heading"])
            omega = dtheta / dt
            v_prev = math.hypot(prev["vx"], prev["vy"])
            v_cur = math.hypot(rec["vx"], rec["vy"])
            if _project_to_scalar(rec["vx"], rec["vy"], rec["heading"]) < 0:
                v_cur = -v_cur
            a = (v_cur - v_prev) / dt

            for h in horizons:
                gt_future = _nearest_record(full_records, rec["t"] + h, max_dt=0.1)
                if gt_future is None:
                    continue
                cvx, cvy = ctrv_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, h)
                cax, cay = ctra_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, a, h)
                ctrv_err = math.hypot(cvx - gt_future["x"], cvy - gt_future["y"])
                ctra_err = math.hypot(cax - gt_future["x"], cay - gt_future["y"])
                writer.writerow([
                    "all", uid, f"{rec['t']:.3f}", f"{h:.2f}",
                    f"{rec['x']:.3f}", f"{rec['y']:.3f}",
                    f"{gt_future['x']:.3f}", f"{gt_future['y']:.3f}",
                    f"{cvx:.3f}", f"{cvy:.3f}", f"{ctrv_err:.3f}",
                    f"{cax:.3f}", f"{cay:.3f}", f"{ctra_err:.3f}",
                ])
                rows_written += 1
    print(f"[CSV] 원본값 {rows_written}행을 {output_path}에 저장함 "
          f"(GT/CTRV예측/CTRA예측 위치 + 오차, horizon={horizons})")


# ---------------------------------------------------------------------------
# CTRV/CTRA 예측 오차 평가 (FDE + ADE, eval_entry_zone_ctrv_ctra.py와 동일)
# ---------------------------------------------------------------------------

def _nearest_record(records, target_t, max_dt=0.1):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def filter_samples_with_full_horizon(samples, max_horizon, max_dt=0.1):
    out = []
    for uid, rec, full_records in samples:
        if _nearest_record(full_records, rec["t"] + max_horizon, max_dt=max_dt) is not None:
            out.append((uid, rec, full_records))
    return out


def evaluate_samples(samples, horizons):
    errs_ctrv = {h: [] for h in horizons}
    errs_ctra = {h: [] for h in horizons}

    for uid, rec, full_records in samples:
        idx = full_records.index(rec) if rec in full_records else None
        if idx is None or idx == 0:
            continue
        prev = full_records[idx - 1]
        dt = rec["t"] - prev["t"]
        if dt <= 0:
            continue
        dtheta = _norm_angle(rec["heading"] - prev["heading"])
        omega = dtheta / dt
        v_prev = math.hypot(prev["vx"], prev["vy"])
        v_cur = math.hypot(rec["vx"], rec["vy"])
        if _project_to_scalar(rec["vx"], rec["vy"], rec["heading"]) < 0:
            v_cur = -v_cur
        a = (v_cur - v_prev) / dt

        for h in horizons:
            gt_future = _nearest_record(full_records, rec["t"] + h, max_dt=0.1)
            if gt_future is None:
                continue
            cx_, cy_ = ctrv_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, h)
            errs_ctrv[h].append(math.hypot(cx_ - gt_future["x"], cy_ - gt_future["y"]))
            ax_, ay_ = ctra_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, a, h)
            errs_ctra[h].append(math.hypot(ax_ - gt_future["x"], ay_ - gt_future["y"]))

    return errs_ctrv, errs_ctra


def evaluate_samples_ade(samples, horizons, step=0.1, max_dt=0.05):
    ade_ctrv = {h: [] for h in horizons}
    ade_ctra = {h: [] for h in horizons}

    for uid, rec, full_records in samples:
        idx = full_records.index(rec) if rec in full_records else None
        if idx is None or idx == 0:
            continue
        prev = full_records[idx - 1]
        dt = rec["t"] - prev["t"]
        if dt <= 0:
            continue
        dtheta = _norm_angle(rec["heading"] - prev["heading"])
        omega = dtheta / dt
        v_prev = math.hypot(prev["vx"], prev["vy"])
        v_cur = math.hypot(rec["vx"], rec["vy"])
        if _project_to_scalar(rec["vx"], rec["vy"], rec["heading"]) < 0:
            v_cur = -v_cur
        a = (v_cur - v_prev) / dt

        for h in horizons:
            n_steps = max(1, round(h / step))
            step_ts = [min(k * step, h) for k in range(1, n_steps + 1)]
            errs_v, errs_a, ok = [], [], True
            for tk in step_ts:
                gt_k = _nearest_record(full_records, rec["t"] + tk, max_dt=max_dt)
                if gt_k is None:
                    ok = False
                    break
                cx_, cy_ = ctrv_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, tk)
                errs_v.append(math.hypot(cx_ - gt_k["x"], cy_ - gt_k["y"]))
                ax_, ay_ = ctra_predict(rec["x"], rec["y"], rec["heading"], v_cur, omega, a, tk)
                errs_a.append(math.hypot(ax_ - gt_k["x"], ay_ - gt_k["y"]))
            if ok and errs_v:
                ade_ctrv[h].append(sum(errs_v) / len(errs_v))
                ade_ctra[h].append(sum(errs_a) / len(errs_a))

    return ade_ctrv, ade_ctra


def _stats(vals):
    if not vals:
        return None
    arr = np.array(vals)
    return arr.mean(), np.percentile(arr, 50), np.percentile(arr, 90), np.percentile(arr, 95)


def _coverage(vals, threshold):
    if not vals:
        return None
    arr = np.array(vals)
    return float((arr <= threshold).mean() * 100.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="highway")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--horizons", default=",".join(str(h) for h in HORIZONS_DEFAULT))
    parser.add_argument("--sample-interval", type=float, default=0.1)
    parser.add_argument("--ade-step", type=float, default=0.1)
    parser.add_argument("--error-threshold", default="1.0,2.0,3.0,5.0")
    parser.add_argument(
        "--export-csv", default="",
        help="지정하면 오차 통계 대신/추가로 GT·CTRV예측·CTRA예측 실제 좌표값을 이 경로에 "
             "CSV로 그대로 저장함 (--horizons로 원하는 horizon만 지정, "
             "예: --horizons 1.0,1.3,1.5,1.7,2.0 --export-csv raw_highway.csv)",
    )
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]
    thresholds = [float(x) for x in args.error_threshold.split(",") if x.strip()]

    npc_series = _extract_npc_series(args.bag_path, exclude_ids)
    print(f"[{args.label}] NPC 궤적 수(텔레포트 분리 포함): {len(npc_series)}")

    samples, sampled_t = collect_all_samples(npc_series, args.sample_interval)
    print(f"[샘플링] {args.sample_interval:.3f}s 간격(~{1.0/args.sample_interval:.1f}Hz), 시점 {len(sampled_t)}개")
    print(f"[샘플 수] 전체 NPC x 시점 = {len(samples)}개 (독립 이벤트 수 아님)")
    if not samples:
        print("샘플이 없습니다.")
        return

    n_ep, episodes = count_episodes(samples)
    print(f"[실제 통과 이벤트 수] {n_ep}건 (같은 차량이 연속으로 찍힌 구간을 하나로 묶은 것)")
    for uid, t0, t1, n in episodes:
        print(f"    uid={uid}, 지속={t1 - t0:.2f}s, 프레임수={n}")

    if args.export_csv:
        export_raw_csv(args.export_csv, horizons, samples)

    errs_ctrv, errs_ctra = evaluate_samples(samples, horizons)

    print("\n" + "=" * 100)
    print(f"[{args.label}] CTRV vs CTRA (단위: m, 원본 표본 - horizon마다 표본 구성 다름, 참고용)")
    print("=" * 100)
    print(f"{'horizon':>8} | {'CTRV mean/p50/p90/p95':^34} | {'CTRA mean/p50/p90/p95':^34}")
    print("-" * 100)
    for h in horizons:
        sv = _stats(errs_ctrv[h])
        sa = _stats(errs_ctra[h])
        sv_str = f"{sv[0]:.3f}/{sv[1]:.3f}/{sv[2]:.3f}/{sv[3]:.3f}" if sv else "N/A"
        sa_str = f"{sa[0]:.3f}/{sa[1]:.3f}/{sa[2]:.3f}/{sa[3]:.3f}" if sa else "N/A"
        print(f"{h:>8.2f} | {sv_str:^34} | {sa_str:^34}")
    print("=" * 100)

    # 생존편향 제거: 가장 긴 horizon까지 GT가 있는 표본만으로 고정
    max_h = max(horizons)
    samples_fx = filter_samples_with_full_horizon(samples, max_h)
    print(f"\n[동일 표본집합] max_horizon={max_h:.1f}s까지 GT가 있는 표본만 사용: "
          f"{len(samples)} -> {len(samples_fx)}")

    errs_ctrv_fx, _ = evaluate_samples(samples_fx, horizons)

    print("\n" + "=" * 80)
    print(f"[{args.label}] horizon별 오차 mean/p50/p90/p95(m) - 동일 표본집합(n 고정), CTRV")
    print(f"(n={len(samples_fx)}, 모든 horizon에서 동일 - 생존편향 없음)")
    print("=" * 80)
    print(f"{'horizon':>8} | {'mean/p50/p90/p95':^30}")
    print("-" * 80)
    for h in horizons:
        s = _stats(errs_ctrv_fx[h])
        s_str = f"{s[0]:.3f}/{s[1]:.3f}/{s[2]:.3f}/{s[3]:.3f}" if s else "N/A"
        print(f"{h:>8.2f} | {s_str:^30}")
    print("=" * 80)

    for th in thresholds:
        print("\n" + "=" * 60)
        print(f"[{args.label}] horizon별 {th:.1f}m 이내 FDE 커버리지(%) - 동일 표본집합, CTRV")
        print("=" * 60)
        print(f"{'horizon':>8} | {'coverage':>10}")
        print("-" * 60)
        for h in horizons:
            c = _coverage(errs_ctrv_fx[h], th)
            c_s = f"{c:.1f}%" if c is not None else "N/A"
            print(f"{h:>8.2f} | {c_s:>10}")
        print("=" * 60)

    # ADE vs FDE
    ade_ctrv_fx, _ = evaluate_samples_ade(samples_fx, horizons, step=args.ade_step)
    print("\n" + "=" * 60)
    print(f"[{args.label}] ADE vs FDE 비교 (mean, m) - 동일 표본집합")
    print("=" * 60)
    print(f"{'horizon':>8} | {'FDE mean':>10} | {'ADE mean':>10} | {'ADE/FDE':>8}")
    print("-" * 60)
    for h in horizons:
        s_fde = _stats(errs_ctrv_fx[h])
        s_ade = _stats(ade_ctrv_fx[h])
        fde_m = s_fde[0] if s_fde else None
        ade_m = s_ade[0] if s_ade else None
        ratio = f"{ade_m / fde_m:.2f}" if (fde_m and ade_m and fde_m > 0) else "N/A"
        fde_s = f"{fde_m:.2f}" if fde_m is not None else "N/A"
        ade_s = f"{ade_m:.2f}" if ade_m is not None else "N/A"
        print(f"{h:>8.2f} | {fde_s:>10} | {ade_s:>10} | {ratio:>8}")
    print("=" * 60)

    for th in thresholds:
        print("\n" + "=" * 60)
        print(f"[{args.label}] horizon별 {th:.1f}m 이내 ADE 커버리지(%) - 동일 표본집합")
        print("=" * 60)
        print(f"{'horizon':>8} | {'coverage':>10}")
        print("-" * 60)
        for h in horizons:
            c = _coverage(ade_ctrv_fx[h], th)
            c_s = f"{c:.1f}%" if c is not None else "N/A"
            print(f"{h:>8.2f} | {c_s:>10}")
        print("=" * 60)


if __name__ == "__main__":
    main()
