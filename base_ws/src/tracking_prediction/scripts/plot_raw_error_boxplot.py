#!/usr/bin/env python3
"""
plot_raw_error_boxplot.py

eval_entry_zone_ctrv_ctra.py / eval_highway_ctrv_ctra.py의 --export-csv로 뽑은 원본
CSV(GT/CTRV예측/CTRA예측 좌표 + 오차)를 읽어서, horizon별 예측 오차(m)의 "전체 분포"를
박스플롯 + 원본 샘플 점(jitter)으로 그린다.

목적(2026-08-02, 팀장님 요청): p90/p95 같은 요약 숫자 하나만 보여주는 대신, 실제 예측경로가
실제경로랑 얼마나/어떻게 차이나는지 원본 오차값을 눈으로 직접 보고 판단할 수 있게.

CTRA 오차는 horizon이 길어질수록 CTRV보다 수십~수백 배 커진다(omega가 노이즈 수준일 때
1/omega^2 항이 CTRV의 1/omega 항보다 노이즈를 훨씬 크게 증폭시키기 때문 - 자세한 설명은
docs/ctrv_vs_ctra_decision_summary.md 참고). 그래서 y축은 로그 스케일을 쓴다 - 선형 스케일이면
CTRA 꼬리값 때문에 CTRV 쪽 분포가 아예 안 보이게 눌린다.

사용법:
    python3 plot_raw_error_boxplot.py raw_roundabout_full.csv --label roundabout
    python3 plot_raw_error_boxplot.py raw_highway_full.csv --label highway

CSV의 group 컬럼(로터리는 approaching/departing, 하이웨이는 all) 값마다 별도 PNG 하나씩
저장한다 (예: error_boxplot_roundabout_approaching.png).

주의: 로터리 NPC 궤적은 한 바퀴(텔레포트 전까지) 약 2.7~2.9초 정도만 이어지므로, horizon이
그보다 길면(예: 3.0s 이상) 미래 GT를 못 찾아 그 horizon의 샘플이 거의/전혀 없을 수 있다.
이 경우 해당 horizon 박스가 비어있거나 아예 안 그려지는데, 이는 버그가 아니라 "그 시점까지
같은 차를 추적할 데이터 자체가 없다"는 뜻 - 그 자체로 유의미한 정보임.
"""
import argparse
import csv
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CTRV_COLOR = "#2ca8ff"
CTRA_COLOR = "#ff5c5c"


def load_csv(path):
    """group -> horizon -> {"ctrv": [...], "ctra": [...]} 형태로 적재."""
    data = defaultdict(lambda: defaultdict(lambda: {"ctrv": [], "ctra": []}))
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            group = row["group"]
            horizon = float(row["horizon"])
            data[group][horizon]["ctrv"].append(float(row["ctrv_error_m"]))
            data[group][horizon]["ctra"].append(float(row["ctra_error_m"]))
    return data


def _box(ax, series, positions, width, color):
    return ax.boxplot(
        series, positions=positions, widths=width, patch_artist=True,
        showfliers=False, manage_ticks=False,
        boxprops=dict(facecolor=color, alpha=0.55, edgecolor=color),
        medianprops=dict(color="white", linewidth=1.5),
        whiskerprops=dict(color=color), capprops=dict(color=color),
    )


def plot_group(label, group, horizon_data, out_path):
    horizons = sorted(horizon_data.keys())
    ctrv_series = [horizon_data[h]["ctrv"] for h in horizons]
    ctra_series = [horizon_data[h]["ctra"] for h in horizons]
    counts = [len(horizon_data[h]["ctrv"]) for h in horizons]

    fig, ax = plt.subplots(figsize=(15, 7))

    step = min(np.diff(horizons)) if len(horizons) > 1 else 0.25
    width = step * 0.32
    pos_ctrv = [h - width * 0.65 for h in horizons]
    pos_ctra = [h + width * 0.65 for h in horizons]

    _box(ax, ctrv_series, pos_ctrv, width, CTRV_COLOR)
    _box(ax, ctra_series, pos_ctra, width, CTRA_COLOR)

    rng = np.random.default_rng(0)
    for h, errs, base in zip(horizons, ctrv_series, pos_ctrv):
        if not errs:
            continue
        jitter = rng.uniform(-width * 0.35, width * 0.35, size=len(errs))
        ax.scatter([base + j for j in jitter], errs, s=6, color=CTRV_COLOR,
                   alpha=0.35, zorder=3, linewidths=0)
    for h, errs, base in zip(horizons, ctra_series, pos_ctra):
        if not errs:
            continue
        jitter = rng.uniform(-width * 0.35, width * 0.35, size=len(errs))
        ax.scatter([base + j for j in jitter], errs, s=6, color=CTRA_COLOR,
                   alpha=0.35, zorder=3, linewidths=0)

    ax.set_yscale("log")
    ax.set_xlabel("horizon (s)")
    ax.set_ylabel("prediction error: predicted vs actual position (m, log scale)")
    ax.set_xticks(horizons)
    ax.set_xticklabels([f"{h:.2f}\n(n={n})" for h, n in zip(horizons, counts)], fontsize=8)
    ax.set_title(f"[{label} / {group}] CTRV vs CTRA prediction error distribution "
                 f"(box=IQR/median, dots=all raw samples)")
    ax.grid(True, which="both", axis="y", alpha=0.2)

    handles = [
        plt.Line2D([0], [0], marker="s", color="none", markerfacecolor=CTRV_COLOR, markersize=10, label="CTRV"),
        plt.Line2D([0], [0], marker="s", color="none", markerfacecolor=CTRA_COLOR, markersize=10, label="CTRA"),
    ]
    ax.legend(handles=handles, loc="upper left")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"저장됨: {out_path} (horizons={horizons}, n per horizon={counts})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", help="--export-csv로 만든 원본 CSV 경로")
    parser.add_argument("--label", required=True, help="출력 파일명에 쓸 라벨 (예: roundabout, highway)")
    args = parser.parse_args()

    data = load_csv(args.csv_path)
    if not data:
        print(f"[경고] {args.csv_path}에서 읽은 행이 없습니다.")
        return
    for group, horizon_data in data.items():
        out_path = f"error_boxplot_{args.label}_{group}.png"
        plot_group(args.label, group, horizon_data, out_path)


if __name__ == "__main__":
    main()
