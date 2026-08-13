#!/usr/bin/env python3
"""
eval_ctra_prediction.py (v2)

CTRA 예측 정확도를 GT rosbag으로 오프라인 평가한다.
v1 대비 추가된 것:
    - --label로 시나리오 이름 태깅 (예: roundabout, highway) 후 JSON으로 결과 저장
    - ADE(Average Displacement Error) / FDE(Final Displacement Error) 계산
      * ADE: 한 번의 예측(모든 horizon)에서 나온 오차들의 평균
      * FDE: 그 예측의 마지막(가장 먼 미래) horizon에서의 오차
    - matplotlib으로 horizon별 오차 꺾은선 그래프 + ADE/FDE 막대그래프 PNG 저장

핵심 개념 (노션/발표용 재확인):
    "t0 시점의 객체 상태(위치/heading/속도/yaw_rate/가속도)로,
     t0+horizon초 뒤의 위치를 CTRA로 예측 -> 그 시점 실제 GT 위치와의
     유클리드 거리(m) = 오차"
    GT의 진짜 unique_id로 시간축을 따라가며 매칭하므로, 이 오차는
    "CTRA 수식 자체의 정확도"를 순수하게 측정한 것이다 (dummy tracking의
    영향은 없음 - 그건 라이브 파이프라인에서 별도 검증 필요).

사용법:
    python3 eval_ctra_prediction.py <bag_path> --label roundabout
    python3 eval_ctra_prediction.py <bag_path2> --label highway
    # 두 결과를 합쳐서 비교 그래프를 그리려면:
    python3 plot_ctra_results.py results_roundabout.json results_highway.json
"""

import argparse
import json
import math
from collections import defaultdict

import numpy as np
import rosbag

OMEGA_EPS = 1e-3
MORAI_HEADING_IN_DEGREES = True
KMH_TO_MS = 1.0 / 3.6  # MORAI ObjectStatus.velocity는 km/h로 발행됨 (실측 확인됨)


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def ctra_predict(x, y, theta, v, omega, a, dt):
    if abs(omega) < OMEGA_EPS:
        dist = v * dt + 0.5 * a * dt * dt
        x_next = x + dist * math.cos(theta)
        y_next = y + dist * math.sin(theta)
        theta_next = theta
        return x_next, y_next, theta_next

    theta_next = theta + omega * dt
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    sin_tn, cos_tn = math.sin(theta_next), math.cos(theta_next)

    x_next = x + (
        (a * dt * sin_tn) / omega
        - (v * sin_t) / omega
        + (v * sin_tn) / omega
        - (a * cos_t) / (omega ** 2)
        + (a * cos_tn) / (omega ** 2)
    )
    y_next = y - (
        (a * dt * cos_tn) / omega
        - (v * cos_t) / omega
        + (v * cos_tn) / omega
        + (a * sin_t) / (omega ** 2)
        - (a * sin_tn) / (omega ** 2)
    )
    return x_next, y_next, theta_next


def _project_to_scalar(vx, vy, heading):
    return vx * math.cos(heading) + vy * math.sin(heading)


def _extract_object_series(bag_path, topic="/Object_topic"):
    """unique_id별로 시계열을 뽑는다. (GT의 진짜 ID 사용 - dummy tracking 무관)"""
    series = defaultdict(list)

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for group, label in (
                (msg.npc_list, "vehicle"),
                (msg.pedestrian_list, "pedestrian"),
                (msg.obstacle_list, "obstacle"),
            ):
                for obj in group:
                    series[obj.unique_id].append(
                        {
                            "t": stamp,
                            "x": obj.position.x,
                            "y": obj.position.y,
                            "heading": _heading_to_rad(obj.heading),
                            "vx": obj.velocity.x * KMH_TO_MS,
                            "vy": obj.velocity.y * KMH_TO_MS,
                            "ax": obj.acceleration.x,
                            "ay": obj.acceleration.y,
                            "class": label,
                        }
                    )

    for uid in series:
        series[uid].sort(key=lambda r: r["t"])

    return series


def _nearest_record(records, target_t, max_dt=0.2):
    best = None
    best_diff = max_dt
    for r in records:
        diff = abs(r["t"] - target_t)
        if diff < best_diff:
            best_diff = diff
            best = r
    return best


def evaluate(bag_path, horizons, turn_threshold_rad_per_sec):
    series = _extract_object_series(bag_path)

    errors = {h: {"straight": [], "turn": []} for h in horizons}
    ade_fde = {"straight": {"ade": [], "fde": []}, "turn": {"ade": [], "fde": []}}

    for uid, records in series.items():
        for i in range(1, len(records)):
            prev, cur = records[i - 1], records[i]
            dt = cur["t"] - prev["t"]
            if dt <= 0:
                continue

            dtheta = cur["heading"] - prev["heading"]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            omega = dtheta / dt

            v = math.hypot(cur["vx"], cur["vy"])
            if _project_to_scalar(cur["vx"], cur["vy"], cur["heading"]) < 0:
                v = -v
            a = _project_to_scalar(cur["ax"], cur["ay"], cur["heading"])

            bucket = "turn" if abs(omega) > turn_threshold_rad_per_sec else "straight"

            instance_errors = []
            for horizon in horizons:
                px, py, _ = ctra_predict(
                    cur["x"], cur["y"], cur["heading"], v, omega, a, horizon
                )
                gt_future = _nearest_record(records, cur["t"] + horizon)
                if gt_future is None:
                    continue
                err = math.hypot(px - gt_future["x"], py - gt_future["y"])
                errors[horizon][bucket].append(err)
                instance_errors.append(err)

            if len(instance_errors) == len(horizons):
                ade_fde[bucket]["ade"].append(sum(instance_errors) / len(instance_errors))
                ade_fde[bucket]["fde"].append(instance_errors[-1])

    return errors, ade_fde


def _print_report(errors, horizons):
    print("=" * 70)
    print("CTRA 예측 오차 평가 결과 (m, 위치오차) - horizon별")
    print("=" * 70)
    for h in horizons:
        for bucket in ("straight", "turn"):
            vals = errors[h][bucket]
            if not vals:
                print(f"  horizon={h:>4}s  [{bucket:8s}]  샘플 없음")
                continue
            arr = np.array(vals)
            print(
                f"  horizon={h:>4}s  [{bucket:8s}]  n={len(arr):5d}  "
                f"mean={arr.mean():.3f}  median={np.median(arr):.3f}  "
                f"p95={np.percentile(arr, 95):.3f}  max={arr.max():.3f}"
            )
    print("=" * 70)


def _print_ade_fde(ade_fde, horizons):
    print()
    print("=" * 70)
    print(f"ADE / FDE 요약 (horizon 전체={horizons}, 단위: m)")
    print("=" * 70)
    for bucket in ("straight", "turn"):
        ade_vals = np.array(ade_fde[bucket]["ade"])
        fde_vals = np.array(ade_fde[bucket]["fde"])
        if len(ade_vals) == 0:
            print(f"  [{bucket:8s}]  샘플 없음")
            continue
        print(
            f"  [{bucket:8s}]  n={len(ade_vals):5d}  "
            f"ADE(mean)={ade_vals.mean():.3f}  ADE(median)={np.median(ade_vals):.3f}  "
            f"FDE(mean)={fde_vals.mean():.3f}  FDE(median)={np.median(fde_vals):.3f}"
        )
    print("=" * 70)
    print("ADE: 궤적 전체(각 horizon)의 평균 오차 / FDE: 마지막(가장 먼) horizon의 오차")


def _make_plots(label, errors, ade_fde, horizons):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm

    # 나눔고딕이 있으면 사용 (없으면 기본 폰트, 한글 깨질 수 있음)
    KFONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
    kfont = fm.FontProperties(fname=KFONT_PATH)
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # (1) horizon별 평균오차 꺾은선
    ax = axes[0]
    for bucket, color in (("straight", "#2b6cb0"), ("turn", "#c05621")):
        means = [np.mean(errors[h][bucket]) if errors[h][bucket] else np.nan for h in horizons]
        ax.plot(horizons, means, marker="o", color=color, label=f"{bucket} (직진/회전)")
    ax.set_xlabel("horizon (s)", fontproperties=kfont)
    ax.set_ylabel("평균 오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] horizon별 평균 오차", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3)

    # (2) ADE/FDE 막대그래프
    ax = axes[1]
    buckets = ["straight", "turn"]
    ade_means = [np.mean(ade_fde[b]["ade"]) if ade_fde[b]["ade"] else 0 for b in buckets]
    fde_means = [np.mean(ade_fde[b]["fde"]) if ade_fde[b]["fde"] else 0 for b in buckets]
    x = np.arange(len(buckets))
    width = 0.35
    ax.bar(x - width / 2, ade_means, width, label="ADE", color="#2f855a")
    ax.bar(x + width / 2, fde_means, width, label="FDE", color="#9b2c2c")
    ax.set_xticks(x)
    ax.set_xticklabels(buckets, fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title(f"[{label}] ADE / FDE 요약", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3, axis="y")

    plt.tight_layout()
    out_path = f"ctra_eval_{label}.png"
    plt.savefig(out_path, dpi=150)
    print(f"그래프 저장됨: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="MORAI /Object_topic이 녹화된 rosbag 경로")
    parser.add_argument("--label", default="result", help="시나리오 이름 (예: roundabout, highway)")
    parser.add_argument("--horizons", type=float, nargs="+", default=[0.5, 1.0, 1.5, 2.0, 3.0])
    parser.add_argument("--turn-threshold-deg-per-sec", type=float, default=5.0)
    parser.add_argument("--no-plot", action="store_true", help="그래프 생성 건너뛰기")
    args = parser.parse_args()

    turn_threshold_rad = math.radians(args.turn_threshold_deg_per_sec)
    errors, ade_fde = evaluate(args.bag_path, args.horizons, turn_threshold_rad)

    _print_report(errors, args.horizons)
    _print_ade_fde(ade_fde, args.horizons)

    # JSON으로 저장 (나중에 여러 시나리오 비교용)
    result = {
        "label": args.label,
        "bag_path": args.bag_path,
        "horizons": args.horizons,
        "errors": {str(h): errors[h] for h in args.horizons},
        "ade_fde": ade_fde,
    }
    json_path = f"results_{args.label}.json"
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n결과 JSON 저장됨: {json_path}")

    if not args.no_plot:
        _make_plots(args.label, errors, ade_fde, args.horizons)


if __name__ == "__main__":
    main()
