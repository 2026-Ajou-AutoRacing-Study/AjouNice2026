#!/usr/bin/env python3
"""
eval_entry_zone_ctrv_ctra.py

2026-07-30: 로터리 미션의 실제 쓰임새는 "ego가 로터리 진입 직전, 다가오는 차량 1대와
멀어지는 차량 1대의 예측경로만 보고 진입각을 판단"하는 것이라는 지적이 나왔음.
지금까지의 CTRV vs CTRA 비교(eval_model_ablation.py)는 로터리 전체 궤적·모든 시점을
다 넣고 평가한 것이라 범위가 더 넓었음. 이 스크립트는 실제 필요 범위(ego 진입 직전
구간의 접근/이탈 차량 2대)로만 좁혀서 CTRV vs CTRA를 여러 horizon에서 한 번에
비교한다.

방법:
    1. NPC(순환 차량) 궤적 전체 점들로 로터리 원(중심, 반지름)을 최소자승 원피팅
       (Kasa method)으로 구한다 - 정확한 진입 게이트 좌표를 몰라도 데이터로 추정.
    2. /Ego_topic에서 ego 궤적을 뽑아, ego가 원에 가까워지는(진입권 밖에서 접근 중)
       구간을 "진입 판단 구간"으로 식별한다 (거리 기준 + 아직 회전 시작 전).
    3. 그 구간의 각 시점마다, 원 중심 기준 각도로 ego와 모든 순환 NPC의 위치를
       비교해서 "ego 진입점 방향으로 다가오는 차량"(approaching, 아직 안 지나감)과
       "이미 지나가서 멀어지는 차량"(departing) 각 1대씩만 고른다.
    4. 그 2대(시점별로 매번 다시 고름)에 대해서만 CTRV/CTRA로 여러 horizon 예측 후
       오차를 모아서 mean/p90/p95 표로 한눈에 비교한다.

가정/한계 (명시):
    - 원 피팅은 전체 NPC 포인트로 하므로 진입/이탈 직선 구간 포인트가 약간 섞여
      들어가지만, 원형 구간 포인트가 압도적으로 많아 큰 영향은 없을 것으로 가정.
    - "진입 판단 구간"은 ego가 원 반경 밖 [+entry-band-min, +entry-band-max]m
      거리대에 있고 아직 급격히 꺾지 않은(|yaw_rate|<entry-yawrate-max) 구간의
      마지막 --decision-window초로 정의 (정확한 게이트 좌표 대신 데이터로 근사).
    - ego heading/velocity 단위는 ObjectStatus와 동일(heading deg, velocity km/h)
      하다고 가정 - EgoVehicleStatus.msg에 별도 단위 표기가 없어 기존 관례를 따름
      (다르면 --ego-heading-rad 등으로 조정 필요, 결과가 이상하면 확인 필요).

사용법:
    python3 eval_entry_zone_ctrv_ctra.py <bag_path> --label roundabout \
        --exclude-ids 16,18,20,21,23,25,27,30,31,34,35,40
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

HORIZONS_DEFAULT = [0.3, 0.5, 1.0, 1.5, 1.7, 2.0, 3.0]


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _signed_longitudinal_speed(record):
    """Return speed magnitude with the heading-direction sign convention."""
    speed = math.hypot(record["vx"], record["vy"])
    if _project_to_scalar(record["vx"], record["vy"], record["heading"]) < 0:
        return -speed
    return speed


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
    """감쇠가속도 CTRA (docs/ctrv_vs_ctra_decision_summary.md와 동일 공식, lam=0.4 기본)."""
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
# 데이터 로딩
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


def _extract_ego_series(bag_path, topic="/Ego_topic"):
    records = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            records.append(
                {
                    "t": stamp, "x": msg.position.x, "y": msg.position.y,
                    "heading": _heading_to_rad(msg.heading),
                    "vx": msg.velocity.x * KMH_TO_MS, "vy": msg.velocity.y * KMH_TO_MS,
                }
            )
    records.sort(key=lambda r: r["t"])
    return records


# ---------------------------------------------------------------------------
# 원 피팅 (Kasa least-squares)
# ---------------------------------------------------------------------------

def fit_circle(points):
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    A = np.column_stack([2 * xs, 2 * ys, np.ones_like(xs)])
    b = xs ** 2 + ys ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = math.sqrt(c + cx ** 2 + cy ** 2)
    return cx, cy, r


# ---------------------------------------------------------------------------
# 진입 구간 탐지 + approaching/departing 선정
# ---------------------------------------------------------------------------

def _yaw_rate_series(records):
    """records(시간순)에 대해 각 i번째의 순간 yaw_rate/속도를 곁들여 반환."""
    out = []
    for i in range(1, len(records)):
        prev, cur = records[i - 1], records[i]
        dt = cur["t"] - prev["t"]
        if dt <= 0:
            continue
        dtheta = _norm_angle(cur["heading"] - prev["heading"])
        yaw_rate = dtheta / dt
        v = math.hypot(cur["vx"], cur["vy"])
        if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
            v = -v
        out.append({**cur, "yaw_rate": yaw_rate, "v": v})
    return out


def detect_entry_windows(ego_records, cx, cy, r, entry_band_min, entry_band_max,
                          entry_yawrate_max_deg, decision_window_s):
    ego = _yaw_rate_series(ego_records)
    windows = []
    was_outside_band = False
    for rec in ego:
        dist = math.hypot(rec["x"] - cx, rec["y"] - cy) - r
        in_band = entry_band_min <= dist <= entry_band_max
        straight = abs(math.degrees(rec["yaw_rate"])) < entry_yawrate_max_deg
        if in_band and straight:
            was_outside_band = True
        elif was_outside_band and dist < entry_band_min:
            # 방금 band 안쪽 경계를 통과 = 진입 커밋 시점 -> 그 직전 decision_window_s를 구간으로
            t_commit = rec["t"]
            windows.append((t_commit - decision_window_s, t_commit))
            was_outside_band = False
    return windows


def rotation_direction(npc_series, cx, cy):
    """순환 NPC들의 평균 각속도 부호로 회전방향(+1 CCW / -1 CW) 추정."""
    signs = []
    for records in npc_series.values():
        if len(records) < 5:
            continue
        thetas = [math.atan2(r["y"] - cy, r["x"] - cx) for r in records[:: max(1, len(records) // 20)]]
        deltas = [_norm_angle(thetas[i + 1] - thetas[i]) for i in range(len(thetas) - 1)]
        if deltas:
            signs.append(1 if sum(deltas) > 0 else -1)
    if not signs:
        return 1
    return 1 if sum(signs) >= 0 else -1


def pick_approaching_departing(npc_series, t, cx, cy, r, rot_sign, ego_theta, ring_tol=8.0,
                                max_gap_rad=None):
    """t 시점 근처에서 원 위(ring_tol 이내)에 있는 NPC들 중 ego_theta 기준
    approaching(아직 안 지나감, 다가오는 중) 1대 / departing(방금 지나감) 1대를 고른다.
    max_gap_rad를 넘어서면(=ego 게이트에서 너무 멀면) 후보에서 아예 제외한다 -
    이게 없으면 순환 차량이 3대뿐이라 게이트 근처에 아무도 없을 때 훨씬 먼 차를
    "그나마 가장 가까운 방향"이라고 잘못 고르게 됨."""
    best_approach, best_approach_gap = None, None
    best_depart, best_depart_gap = None, None

    for uid, records in npc_series.items():
        # t에 가장 가까운 레코드 탐색 (선형 - 궤적 수가 적어 충분히 빠름)
        best_rec, best_diff = None, 0.1
        for rec in records:
            diff = abs(rec["t"] - t)
            if diff < best_diff:
                best_diff, best_rec = diff, rec
        if best_rec is None:
            continue
        dist_to_ring = abs(math.hypot(best_rec["x"] - cx, best_rec["y"] - cy) - r)
        if dist_to_ring > ring_tol:
            continue

        theta_npc = math.atan2(best_rec["y"] - cy, best_rec["x"] - cx)
        delta = _norm_angle(theta_npc - ego_theta)  # ego 기준 상대각
        # rot_sign>0(CCW)이면 approaching은 delta<0(아직 ego 각도에 못 미침, 다가오는 중)
        signed = delta * rot_sign
        if signed < 0:  # 아직 ego 위치에 도달 전 = approaching
            gap = -signed
            if max_gap_rad is not None and gap > max_gap_rad:
                continue
            if best_approach_gap is None or gap < best_approach_gap:
                best_approach_gap, best_approach = gap, (uid, best_rec, records)
        elif signed > 0:  # 이미 지나감 = departing
            gap = signed
            if max_gap_rad is not None and gap > max_gap_rad:
                continue
            if best_depart_gap is None or gap < best_depart_gap:
                best_depart_gap, best_depart = gap, (uid, best_rec, records)

    return best_approach, best_depart


# ---------------------------------------------------------------------------
# CTRV/CTRA 예측 오차 평가 (approaching/departing 샘플만)
# ---------------------------------------------------------------------------

def _nearest_record(records, target_t, max_dt=0.1):
    best, best_diff = None, max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff, best = diff, r
    return best


def filter_samples_with_full_horizon(samples, max_horizon, max_dt=0.1):
    """samples 중 t+max_horizon 시점까지 실측 GT가 존재하는 것만 남긴다.
    horizon별로 비교할 때 표본 구성이 달라지면(긴 horizon일수록 GT를 못 찾는
    표본이 먼저 탈락) '표본이 쉬워서 좋아 보이는' 생존 편향이 생기므로,
    모든 horizon에서 동일한(=apples-to-apples) 표본 집합으로 비교하기 위한 필터."""
    out = []
    for uid, rec, full_records in samples:
        if _nearest_record(full_records, rec["t"] + max_horizon, max_dt=max_dt) is not None:
            out.append((uid, rec, full_records))
    return out


def evaluate_samples(samples, horizons):
    """samples: [(uid, rec_at_t, full_records), ...] (approaching/departing 시점별로 모은 것)"""
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
        # Keep the previous and current speeds in the same signed convention.
        # Applying a sign only to v_cur creates a false direction reversal and
        # can produce physically impossible acceleration estimates.
        v_prev = _signed_longitudinal_speed(prev)
        v_cur = _signed_longitudinal_speed(rec)
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
    """ADE(h): FDE(h)는 t+h 한 점만 비교하지만, ADE(h)는 0~h를 step 간격(기본 0.1s)으로
    쪼갠 중간 시점들 각각의 오차를 평균낸다 - "h초 동안 예측 경로 전체가 실제 경로를
    얼마나 잘 따라가는가"를 봄. 중간 시점 중 하나라도 실측 GT를 못 찾으면(max_dt=0.05
    이내에 없으면) 그 샘플은 해당 horizon에서 통째로 제외한다(부분평균으로 인한
    편향을 피하기 위함 - FDE의 filter_samples_with_full_horizon과 같은 원칙)."""
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
        v_prev = _signed_longitudinal_speed(prev)
        v_cur = _signed_longitudinal_speed(rec)
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


def export_raw_csv(output_path, horizons, approach_samples, depart_samples):
    """오차 통계로 요약하지 않고, GT/CTRV예측/CTRA예측 실제 좌표값을 그대로 CSV로 뽑는다
    (2026-08-01: 팀장님이 오차 대신 원본 값을 직접 보고 판단하고 싶다는 요청).
    approaching/departing 각 샘플 x horizon 조합마다 한 행씩 기록."""
    groups = (("approaching", approach_samples), ("departing", depart_samples))
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
        for group_name, samples in groups:
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
                v_prev = _signed_longitudinal_speed(prev)
                v_cur = _signed_longitudinal_speed(rec)
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
                        group_name, uid, f"{rec['t']:.3f}", f"{h:.2f}",
                        f"{rec['x']:.3f}", f"{rec['y']:.3f}",
                        f"{gt_future['x']:.3f}", f"{gt_future['y']:.3f}",
                        f"{cvx:.3f}", f"{cvy:.3f}", f"{ctrv_err:.3f}",
                        f"{cax:.3f}", f"{cay:.3f}", f"{ctra_err:.3f}",
                    ])
                    rows_written += 1
    print(f"[CSV] 원본값 {rows_written}행을 {output_path}에 저장함 "
          f"(GT/CTRV예측/CTRA예측 위치 + 오차, horizon={horizons})")


def _draw_car(ax, x, y, heading, color, label=None, length=4.5, width=2.0, zorder=5, alpha=1.0,
              draw_arrow=True):
    """heading(rad) 방향을 바라보는 자동차 모양 사각형을 (x,y) 중심에 그린다."""
    import matplotlib.patches as mpatches
    import matplotlib.transforms as mtransforms

    rect = mpatches.FancyBboxPatch(
        (-length / 2, -width / 2), length, width,
        boxstyle="round,pad=0.0,rounding_size=0.3",
        facecolor=color, edgecolor="black", linewidth=1.0, alpha=alpha, zorder=zorder,
    )
    t = mtransforms.Affine2D().rotate(heading).translate(x, y) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)
    if draw_arrow:
        # 진행방향 화살표(앞쪽) - 실제 heading 데이터가 있는 차량(접근/이탈)만 표시.
        # ego는 정차 상태라 이 화살표가 "임의로 원 중심을 향하게 계산한 방향"이라
        # 실측 heading으로 오해될 수 있어 ego에는 그리지 않음(호출부에서 draw_arrow=False).
        ax.annotate(
            "", xy=(x + math.cos(heading) * length * 0.9, y + math.sin(heading) * length * 0.9),
            xytext=(x, y),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.5), zorder=zorder,
        )
    if label:
        ax.annotate(label, (x, y), xytext=(6, 10), textcoords="offset points", fontsize=9,
                    fontweight="bold", color=color, zorder=zorder + 1)


def save_measurement_plot(label, npc_series, cx, cy, r, ego_x, ego_y, approach_samples, depart_samples):
    """어디를 측정했는지 한눈에 보이는 그림: NPC 실제 이동 궤적(옅은 선), 차 모양 아이콘
    (ego/접근/이탈), 그리고 실제 샘플이 찍힌 범위를 ego 주변만 확대해서 그린다.
    도로 형상/차선은 실측 데이터가 없어 그리지 않음(장식용 요소 배제)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 10))

    # 배경: 실제 NPC 지나간 자리(옅은 선) - "차가 실제로 다닌 자리"만 데이터 그대로 표시
    # (예전엔 회색 띠+노란 점선으로 "도로처럼" 그렸으나, 실제 차선폭/중앙선 데이터가
    # 아니라 순전히 장식이라 오해 소지가 있어 제거 - 2026-07-31)
    for records in npc_series.values():
        xs = [rec["x"] for rec in records]
        ys = [rec["y"] for rec in records]
        ax.plot(xs, ys, color="white", linewidth=0.4, alpha=0.25, zorder=2)

    # approaching / departing 샘플 위치 (옅은 점 - "이 구간에서 측정했다"는 범위 표시용)
    if approach_samples:
        axs = [rec["x"] for _, rec, _ in approach_samples]
        ays = [rec["y"] for _, rec, _ in approach_samples]
        ax.scatter(axs, ays, c="deepskyblue", s=14, alpha=0.6, zorder=3,
                   label=f"approaching car positions over time (n={len(approach_samples)})")
    if depart_samples:
        dxs = [rec["x"] for _, rec, _ in depart_samples]
        dys = [rec["y"] for _, rec, _ in depart_samples]
        ax.scatter(dxs, dys, c="orange", s=14, alpha=0.6, zorder=3,
                   label=f"departing car positions over time (n={len(depart_samples)})")

    # 대표 예시 1개씩 뽑아서 실제 "차 모양"으로 그림 (감 잡기용)
    if approach_samples:
        _, rec, _ = approach_samples[len(approach_samples) // 2]
        _draw_car(ax, rec["x"], rec["y"], rec["heading"], "deepskyblue", label="approaching car\n(example instant)")
    if depart_samples:
        _, rec, _ = depart_samples[len(depart_samples) // 2]
        _draw_car(ax, rec["x"], rec["y"], rec["heading"], "orange", label="departing car\n(example instant)")

    # ego: 정차 상태(실제 heading 데이터 아님) - 화살표 없이 위치만 표시
    ego_heading = math.atan2(cy - ego_y, cx - ego_x)
    _draw_car(ax, ego_x, ego_y, ego_heading, "red", label="EGO (parked at entry gate)", zorder=6,
              draw_arrow=False)

    # ego 주변만 확대 (전체 원 대신 실제로 의미있는 구간만 보이게)
    zoom = 45.0
    ax.set_xlim(ego_x - zoom, ego_x + zoom)
    ax.set_ylim(ego_y - zoom * 0.8, ego_y + zoom * 1.2)

    ax.set_aspect("equal", adjustable="box")
    ax.set_facecolor("#1e1e1e")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"[{label}] EGO entry-gate measurement zone (zoomed)")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)

    out_path = f"entry_zone_measurement_{label}.png"
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\n측정 범위 그림 저장됨: {out_path}")


def count_episodes(samples, max_gap_s=0.5):
    """samples(시간순, (uid, rec, records) 튜플)를 같은 차량이 연속으로 찍힌
    구간끼리 묶어 '실제 통과 이벤트' 수를 센다. uid가 바뀌거나 시간 간격이
    max_gap_s를 넘으면 새 이벤트로 취급."""
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


def _stats(vals):
    if not vals:
        return None
    arr = np.array(vals)
    return arr.mean(), np.percentile(arr, 50), np.percentile(arr, 90), np.percentile(arr, 95)


def _coverage(vals, threshold):
    """threshold(m) 이내 오차인 샘플 비율(%). '이 horizon을 실사용 가능한가'를
    판단하는 실용적 지표 - 이상적인 90/95% 신뢰수준 기준 대신, 몇 %가 커버되는지
    그대로 보여줘서 절충값을 고를 수 있게 함."""
    if not vals:
        return None
    arr = np.array(vals)
    return float((arr <= threshold).mean() * 100.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--exclude-ids", default="")
    parser.add_argument("--horizons", default=",".join(str(h) for h in HORIZONS_DEFAULT))
    parser.add_argument("--entry-band-min", type=float, default=0.0, help="원 밖 이 거리(m)부터 진입권으로 인정")
    parser.add_argument("--entry-band-max", type=float, default=25.0, help="원 밖 이 거리(m)까지 진입권으로 인정")
    parser.add_argument("--entry-yawrate-max-deg", type=float, default=8.0, help="이 미만이면 '아직 직진 중'으로 판단")
    parser.add_argument("--decision-window", type=float, default=2.0, help="진입 커밋 직전 몇 초를 판단구간으로 볼지")
    parser.add_argument("--ring-tol", type=float, default=8.0, help="원 중심 기준 이 거리(m) 이내면 '순환 중'으로 인정")
    parser.add_argument(
        "--max-gap-deg", type=float, default=25.0,
        help="ego 게이트 각도 기준 이보다 먼 각도 차이의 차량은 후보에서 제외(게이트 근처만 측정)",
    )
    parser.add_argument(
        "--error-threshold", default="1.0,2.0,3.0,5.0",
        help="이 오차(m) 이내를 '쓸만하다'로 보고, horizon별 커버리지(%%)를 같이 출력. "
             "콤마로 여러 기준 동시 비교 가능 (기본: 1.0,2.0,3.0,5.0)",
    )
    parser.add_argument(
        "--sample-interval", type=float, default=0.1,
        help="GT(native ~49.7Hz)에서 평가용으로 뽑아낼 시점 간격(초). "
             "0.1=10Hz. 실제 OD 관측 rate(8~12Hz)와 비교하려면 0.125(8Hz)~0.083(12Hz)로 바꿔서 재실행.",
    )
    parser.add_argument(
        "--ade-step", type=float, default=0.1,
        help="ADE(h) 계산 시 0~h 구간을 쪼갤 간격(초). 기본 0.1s (=예측경로 위 0.1s마다 오차 확인 후 평균)",
    )
    parser.add_argument(
        "--export-csv", default="",
        help="지정하면 오차 통계 대신/추가로 GT·CTRV예측·CTRA예측 실제 좌표값을 이 경로에 "
             "CSV로 그대로 저장함 (--horizons로 원하는 horizon만 지정해서 뽑을 수 있음, "
             "예: --horizons 1.0,1.3,1.5,1.7,2.0 --export-csv raw_roundabout.csv)",
    )
    args = parser.parse_args()

    exclude_ids = set(int(x) for x in args.exclude_ids.split(",") if x.strip())
    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]

    npc_series = _extract_npc_series(args.bag_path, exclude_ids)
    print(f"[{args.label}] NPC 궤적 수(텔레포트 분리 포함): {len(npc_series)}")

    all_points = [(r["x"], r["y"]) for records in npc_series.values() for r in records]
    cx, cy, r = fit_circle(all_points)
    print(f"[로터리 원 피팅] 중심=({cx:.2f}, {cy:.2f}), 반지름={r:.2f}m (n={len(all_points)}점)")

    rot_sign = rotation_direction(npc_series, cx, cy)
    print(f"[회전방향] {'CCW(+)' if rot_sign > 0 else 'CW(-)'}")

    ego_records = _extract_ego_series(args.bag_path)
    if not ego_records:
        print("[에러] /Ego_topic에서 데이터를 못 읽었습니다 - 토픽명/타입 확인 필요.")
        return
    print(f"[ego] 레코드 수: {len(ego_records)}")

    ego_x = sum(r["x"] for r in ego_records) / len(ego_records)
    ego_y = sum(r["y"] for r in ego_records) / len(ego_records)
    ego_dist = math.hypot(ego_x - cx, ego_y - cy) - r
    print(f"[진단] ego 위치는 전체 구간 고정: ({ego_x:.2f}, {ego_y:.2f}), 원 경계 기준 거리={ego_dist:.1f}m")
    print("ego가 진입 지점에 정지해서 촬영한 데이터로 보고, 이 고정 위치를 게이트로 써서 전체 시간에 걸쳐 평가합니다.")

    ego_theta = math.atan2(ego_y - cy, ego_x - cx)
    t_values = sorted({rec["t"] for records in npc_series.values() for rec in records})
    # 너무 촘촘하면(native ~49.7Hz) 느려지므로 --sample-interval 간격으로만 샘플링해서 평가
    sampled_t = []
    last_t = None
    for t in t_values:
        if last_t is None or t - last_t >= args.sample_interval:
            sampled_t.append(t)
            last_t = t
    print(f"[샘플링] {args.sample_interval:.3f}s 간격(~{1.0 / args.sample_interval:.1f}Hz), "
          f"시점 {len(sampled_t)}개")

    samples = []
    approach_samples, depart_samples = [], []
    for t in sampled_t:
        approach, depart = pick_approaching_departing(
            npc_series, t, cx, cy, r, rot_sign, ego_theta, ring_tol=args.ring_tol,
            max_gap_rad=math.radians(args.max_gap_deg),
        )
        if approach is not None:
            samples.append(approach)
            approach_samples.append(approach)
        if depart is not None:
            samples.append(depart)
            depart_samples.append(depart)

    print(f"[샘플 수] approaching+departing 합쳐서 {len(samples)}개 "
          f"({1.0 / args.sample_interval:.1f}Hz 시점 기준, 독립 이벤트 수 아님)")
    if not samples:
        print("샘플이 없습니다 - ring-tol 등 파라미터 조정 필요.")
        return

    n_app_ep, app_episodes = count_episodes(approach_samples)
    n_dep_ep, dep_episodes = count_episodes(depart_samples)
    print(f"[실제 통과 이벤트 수] approaching={n_app_ep}건, departing={n_dep_ep}건 "
          f"(같은 차량이 연속으로 찍힌 구간을 하나로 묶은 것)")
    for kind, eps in (("approaching", app_episodes), ("departing", dep_episodes)):
        for uid, t0, t1, n in eps:
            print(f"    [{kind}] uid={uid}, 지속={t1 - t0:.2f}s, 프레임수={n}")

    save_measurement_plot(args.label, npc_series, cx, cy, r, ego_x, ego_y, approach_samples, depart_samples)

    if args.export_csv:
        export_raw_csv(args.export_csv, horizons, approach_samples, depart_samples)

    errs_ctrv, errs_ctra = evaluate_samples(samples, horizons)
    errs_ctrv_app, errs_ctra_app = evaluate_samples(approach_samples, horizons)
    errs_ctrv_dep, errs_ctra_dep = evaluate_samples(depart_samples, horizons)

    print("\n" + "=" * 100)
    print(f"[{args.label}] 진입구간 접근/이탈 차량만 대상 - CTRV vs CTRA (단위: m, 합산)")
    print("=" * 100)
    print(f"{'horizon':>8} | {'CTRV mean/p50/p90/p95':^34} | {'CTRA mean/p50/p90/p95':^34}")
    print("-" * 100)
    for h in horizons:
        sv = _stats(errs_ctrv[h])
        sa = _stats(errs_ctra[h])
        sv_str = f"{sv[0]:.3f}/{sv[1]:.3f}/{sv[2]:.3f}/{sv[3]:.3f}" if sv else "N/A"
        sa_str = f"{sa[0]:.3f}/{sa[1]:.3f}/{sa[2]:.3f}/{sa[3]:.3f}" if sa else "N/A"
        print(f"{h:>8.1f} | {sv_str:^34} | {sa_str:^34}")
    print("=" * 100)
    print("CTRA 열이 CTRV보다 작을수록, '진입구간 한정이라면 CTRA가 더 낫다'는 뜻.")

    print("\n" + "=" * 110)
    print(f"[{args.label}] 다가오는 차량(approaching) vs 멀어지는 차량(departing) 오차 - CTRV 기준 (단위: m)")
    print(f"(approaching n={len(approach_samples)}, departing n={len(depart_samples)})")
    print("=" * 110)
    print(f"{'horizon':>8} | {'approaching mean/p50/p90/p95':^36} | {'departing mean/p50/p90/p95':^36}")
    print("-" * 110)
    for h in horizons:
        sa = _stats(errs_ctrv_app[h])
        sd = _stats(errs_ctrv_dep[h])
        sa_str = f"{sa[0]:.3f}/{sa[1]:.3f}/{sa[2]:.3f}/{sa[3]:.3f}" if sa else "N/A"
        sd_str = f"{sd[0]:.3f}/{sd[1]:.3f}/{sd[2]:.3f}/{sd[3]:.3f}" if sd else "N/A"
        print(f"{h:>8.1f} | {sa_str:^36} | {sd_str:^36}")
    print("=" * 110)

    thresholds = [float(x) for x in args.error_threshold.split(",") if x.strip()]
    th = thresholds[-1]  # 기존 단일-threshold 로직(표본 손실 경고용)은 대표로 마지막 값 사용
    print("\n" + "=" * 100)
    print(f"[{args.label}] horizon별 {th:.1f}m 이내 커버리지(%) - CTRV 기준 (표본이 horizon마다 "
          f"달라지는 원본 표 - 아래 '동일 표본집합' 표와 비교용으로만 참고)")
    print("=" * 100)
    print(f"{'horizon':>8} | {'전체':>8} | {'n(전체)':>8} | {'approaching':>12} | {'n(app)':>7} | "
          f"{'departing':>12} | {'n(dep)':>7}")
    print("-" * 100)
    for h in horizons:
        n_all, n_app, n_dep = len(errs_ctrv[h]), len(errs_ctrv_app[h]), len(errs_ctrv_dep[h])
        c_all = _coverage(errs_ctrv[h], th)
        c_app = _coverage(errs_ctrv_app[h], th)
        c_dep = _coverage(errs_ctrv_dep[h], th)
        c_all_s = f"{c_all:.1f}%" if c_all is not None else "N/A"
        c_app_s = f"{c_app:.1f}%" if c_app is not None else "N/A"
        c_dep_s = f"{c_dep:.1f}%" if c_dep is not None else "N/A"
        print(f"{h:>8.2f} | {c_all_s:>8} | {n_all:>8} | {c_app_s:>12} | {n_app:>7} | "
              f"{c_dep_s:>12} | {n_dep:>7}")
    print("=" * 100)
    print(f"주의: n(전체)/n(app)/n(dep)가 horizon이 커질수록 줄어드는 이유는 "
          f"_nearest_record(t+h)가 해당 NPC의 실측 구간(다음 텔레포트 전까지)을 넘는 h에서는 "
          f"미래 GT를 못 찾아 그 샘플을 통째로 버리기 때문. 특히 departing은 게이트를 막 지나친 "
          f"직후라 텔레포트(한 바퀴 완주 리셋) 지점에 더 가까운 경우가 많아 표본 손실이 훨씬 큼. "
          f"이러면 horizon이 커질수록 '남아있는 표본만 유독 쉬워서' 커버리지가 착시적으로 좋아 보일 "
          f"수 있음 - 아래 '동일 표본집합' 표로 이 편향을 제거하고 다시 비교할 것.")

    # 생존편향 제거: 가장 긴 horizon(=max(horizons))까지 GT가 존재하는 표본만으로
    # 모든 horizon을 재평가 -> horizon 간 표본 구성이 항상 동일해서 공정 비교 가능.
    max_h = max(horizons)
    samples_fx = filter_samples_with_full_horizon(samples, max_h)
    app_fx = filter_samples_with_full_horizon(approach_samples, max_h)
    dep_fx = filter_samples_with_full_horizon(depart_samples, max_h)
    print(f"\n[동일 표본집합 비교] max_horizon={max_h:.1f}s까지 GT가 있는 표본만 사용: "
          f"전체 {len(samples)}->{len(samples_fx)}, approaching {len(approach_samples)}->{len(app_fx)}, "
          f"departing {len(depart_samples)}->{len(dep_fx)}")

    errs_ctrv_fx, _ = evaluate_samples(samples_fx, horizons)
    errs_ctrv_app_fx, _ = evaluate_samples(app_fx, horizons)
    errs_ctrv_dep_fx, _ = evaluate_samples(dep_fx, horizons)

    # --- 상세 지표: mean/p50/p90/p95 (동일 표본집합, threshold와 무관하게 한 번만) ---
    print("\n" + "=" * 116)
    print(f"[{args.label}] horizon별 오차 mean/p50/p90/p95(m) - 동일 표본집합(n 고정) 기준, CTRV")
    print(f"(전체 n={len(samples_fx)}, approaching n={len(app_fx)}, departing n={len(dep_fx)} - "
          f"모든 horizon에서 이 n 그대로 고정, 생존편향 없음)")
    print("=" * 116)
    print(f"{'horizon':>8} | {'전체 mean/p50/p90/p95':^26} | {'approaching mean/p50/p90/p95':^26} | "
          f"{'departing mean/p50/p90/p95':^26}")
    print("-" * 116)
    for h in horizons:
        s_all = _stats(errs_ctrv_fx[h])
        s_app = _stats(errs_ctrv_app_fx[h])
        s_dep = _stats(errs_ctrv_dep_fx[h])
        s_all_s = f"{s_all[0]:.2f}/{s_all[1]:.2f}/{s_all[2]:.2f}/{s_all[3]:.2f}" if s_all else "N/A"
        s_app_s = f"{s_app[0]:.2f}/{s_app[1]:.2f}/{s_app[2]:.2f}/{s_app[3]:.2f}" if s_app else "N/A"
        s_dep_s = f"{s_dep[0]:.2f}/{s_dep[1]:.2f}/{s_dep[2]:.2f}/{s_dep[3]:.2f}" if s_dep else "N/A"
        print(f"{h:>8.2f} | {s_all_s:^26} | {s_app_s:^26} | {s_dep_s:^26}")
    print("=" * 116)

    # --- 커버리지: 여러 threshold를 한 번에 비교 (동일 표본집합) ---
    for th_i in thresholds:
        print("\n" + "=" * 100)
        print(f"[{args.label}] horizon별 {th_i:.1f}m 이내 커버리지(%) - 동일 표본집합(n 고정) 기준, CTRV")
        print("=" * 100)
        print(f"{'horizon':>8} | {'전체':>8} | {'approaching':>12} | {'departing':>12}")
        print("-" * 100)
        for h in horizons:
            c_all = _coverage(errs_ctrv_fx[h], th_i)
            c_app = _coverage(errs_ctrv_app_fx[h], th_i)
            c_dep = _coverage(errs_ctrv_dep_fx[h], th_i)
            c_all_s = f"{c_all:.1f}%" if c_all is not None else "N/A"
            c_app_s = f"{c_app:.1f}%" if c_app is not None else "N/A"
            c_dep_s = f"{c_dep:.1f}%" if c_dep is not None else "N/A"
            print(f"{h:>8.2f} | {c_all_s:>8} | {c_app_s:>12} | {c_dep_s:>12}")
        print("=" * 100)
    print("\n이 표들이 편향 없는 진짜 추세임 - 위(표본이 horizon마다 달라지는 원본 표)와 비교해서 "
          "얼마나 차이나는지 확인할 것. threshold(m)를 낮출수록(예: 1.0m) 요구조건이 빡빡해지므로 "
          "커버리지가 전반적으로 낮아지는 게 정상 - 그 낮아지는 정도로 '얼마나 엄격한 기준을 감당할 "
          "수 있는지' 감을 잡을 것.")

    # ------------------------------------------------------------------
    # ADE(h): FDE(h)(t+h 한 점만)와 달리, 0~h를 --ade-step 간격으로 쪼갠 중간 시점들의
    # 오차 평균 - "예측 경로 전체가 실제 경로를 얼마나 잘 따라가는가". 표본집합은 FDE와
    # 동일한 samples_fx/app_fx/dep_fx(생존편향 제거된 것)를 그대로 재사용 - ADE는 중간점도
    # 다 있어야 하므로 추가로 몇 개 더 빠질 수 있어 아래 n으로 확인.
    # ------------------------------------------------------------------
    ade_ctrv_fx, _ = evaluate_samples_ade(samples_fx, horizons, step=args.ade_step)
    ade_ctrv_app_fx, _ = evaluate_samples_ade(app_fx, horizons, step=args.ade_step)
    ade_ctrv_dep_fx, _ = evaluate_samples_ade(dep_fx, horizons, step=args.ade_step)

    print("\n" + "#" * 116)
    print(f"[{args.label}] ADE(h) - 0~h를 {args.ade_step:.2f}s 간격으로 쪼갠 중간 시점 오차 평균 "
          f"(FDE(h)=t+h 한 점만 비교와 다름), 동일 표본집합(생존편향 제거), CTRV")
    print("#" * 116)

    print("\n" + "=" * 132)
    print(f"[{args.label}] horizon별 ADE mean/p50/p90/p95(m) + n (동일 표본집합 기준, CTRV)")
    print("=" * 132)
    print(f"{'horizon':>8} | {'전체 mean/p50/p90/p95 (n)':^30} | {'approaching mean/p50/p90/p95 (n)':^30} | "
          f"{'departing mean/p50/p90/p95 (n)':^30}")
    print("-" * 132)
    for h in horizons:
        s_all = _stats(ade_ctrv_fx[h])
        s_app = _stats(ade_ctrv_app_fx[h])
        s_dep = _stats(ade_ctrv_dep_fx[h])
        s_all_s = (f"{s_all[0]:.2f}/{s_all[1]:.2f}/{s_all[2]:.2f}/{s_all[3]:.2f} (n={len(ade_ctrv_fx[h])})"
                   if s_all else f"N/A (n={len(ade_ctrv_fx[h])})")
        s_app_s = (f"{s_app[0]:.2f}/{s_app[1]:.2f}/{s_app[2]:.2f}/{s_app[3]:.2f} (n={len(ade_ctrv_app_fx[h])})"
                   if s_app else f"N/A (n={len(ade_ctrv_app_fx[h])})")
        s_dep_s = (f"{s_dep[0]:.2f}/{s_dep[1]:.2f}/{s_dep[2]:.2f}/{s_dep[3]:.2f} (n={len(ade_ctrv_dep_fx[h])})"
                   if s_dep else f"N/A (n={len(ade_ctrv_dep_fx[h])})")
        print(f"{h:>8.2f} | {s_all_s:^30} | {s_app_s:^30} | {s_dep_s:^30}")
    print("=" * 132)

    # ADE vs FDE 직접 비교 (전체, mean 기준) - 경로 전체 관점과 끝점 관점이 얼마나 다른지 한눈에
    print("\n" + "=" * 60)
    print(f"[{args.label}] ADE vs FDE 비교 (전체, mean, m) - 동일 표본집합")
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
    print("ADE는 항상 FDE보다 작거나 같음(경로 중간엔 아직 덜 벌어진 시점들이 섞여 평균이 낮아짐).")
    print("ADE/FDE 비율이 1에 가까울수록 '경로 내내 끝점 수준으로 벌어져 있다'는 뜻이고,")
    print("비율이 작을수록(예: 0.3~0.5) '끝점 근처에서만 갑자기 벌어진다'는 뜻 - 궤적 형태 파악에 참고.")

    # threshold별 ADE 커버리지 (FDE와 동일한 thresholds 재사용)
    for th_i in thresholds:
        print("\n" + "=" * 100)
        print(f"[{args.label}] horizon별 {th_i:.1f}m 이내 ADE 커버리지(%) - 동일 표본집합 기준, CTRV")
        print("=" * 100)
        print(f"{'horizon':>8} | {'전체':>8} | {'approaching':>12} | {'departing':>12}")
        print("-" * 100)
        for h in horizons:
            c_all = _coverage(ade_ctrv_fx[h], th_i)
            c_app = _coverage(ade_ctrv_app_fx[h], th_i)
            c_dep = _coverage(ade_ctrv_dep_fx[h], th_i)
            c_all_s = f"{c_all:.1f}%" if c_all is not None else "N/A"
            c_app_s = f"{c_app:.1f}%" if c_app is not None else "N/A"
            c_dep_s = f"{c_dep:.1f}%" if c_dep is not None else "N/A"
            print(f"{h:>8.2f} | {c_all_s:>8} | {c_app_s:>12} | {c_dep_s:>12}")
        print("=" * 100)


if __name__ == "__main__":
    main()
