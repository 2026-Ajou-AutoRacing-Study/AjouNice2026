#!/usr/bin/env python3
"""
plot_ctra_results.py

여러 개의 results_<label>.json (eval_ctra_prediction.py가 저장한 것)을 모아서
시나리오별(roundabout, highway 등) ADE/FDE를 한 그래프에 비교한다.

사용법:
    python3 plot_ctra_results.py results_roundabout.json results_highway.json
"""

import argparse
import json

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("json_paths", nargs="+", help="results_*.json 파일들")
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm

    KFONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
    kfont = fm.FontProperties(fname=KFONT_PATH)
    plt.rcParams["axes.unicode_minus"] = False

    results = []
    for path in args.json_paths:
        with open(path) as f:
            results.append(json.load(f))

    # 시나리오 x (직진/회전) 조합별 ADE, FDE 평균 계산
    labels = []
    ade_vals = []
    fde_vals = []
    for r in results:
        for bucket in ("straight", "turn"):
            ade_list = r["ade_fde"][bucket]["ade"]
            fde_list = r["ade_fde"][bucket]["fde"]
            if not ade_list:
                continue
            labels.append(f"{r['label']}\n({bucket})")
            ade_vals.append(np.mean(ade_list))
            fde_vals.append(np.mean(fde_list))

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.6), 6))
    ax.bar(x - width / 2, ade_vals, width, label="ADE", color="#2f855a")
    ax.bar(x + width / 2, fde_vals, width, label="FDE", color="#9b2c2c")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontproperties=kfont)
    ax.set_ylabel("오차 (m)", fontproperties=kfont)
    ax.set_title("시나리오별 CTRA 예측 정확도 (ADE / FDE)", fontproperties=kfont)
    ax.legend(prop=kfont)
    ax.grid(alpha=0.3, axis="y")

    for i, (a, f) in enumerate(zip(ade_vals, fde_vals)):
        ax.text(i - width / 2, a, f"{a:.2f}", ha="center", va="bottom", fontsize=8)
        ax.text(i + width / 2, f, f"{f:.2f}", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    plt.savefig("ctra_scenario_comparison.png", dpi=150)
    print("저장됨: ctra_scenario_comparison.png")


if __name__ == "__main__":
    main()
