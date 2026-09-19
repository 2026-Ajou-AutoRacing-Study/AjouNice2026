#!/usr/bin/env python3
"""Summarize an equal-bag highway tracker/Map-CV ablation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


BAGS = ("vehicle_highway_merge", "vehicle_highway1")


def mean(values):
    return sum(values) / len(values)


def extract(case_dir: Path):
    values = {
        key: []
        for key in (
            "tracking_f1",
            "center_p50_m",
            "speed_mae_mps",
            "ave_mps",
            "longitudinal_velocity_mae_mps",
            "lateral_velocity_mae_mps",
            "yaw_mae_deg",
            "all_top1_fde_1s_m",
            "all_top1_fde_2s_m",
            "all_top1_fde_3s_m",
            "all_coverage_3s",
            "stable_top1_fde_1s_m",
            "stable_top1_fde_2s_m",
            "stable_top1_fde_3s_m",
            "stable_minfde_k_1s_m",
            "stable_minfde_k_2s_m",
            "stable_minfde_k_3s_m",
        )
    }
    config = None
    per_bag = {}
    for bag in BAGS:
        bag_dir = case_dir / bag
        regular = json.loads((bag_dir / "summary.json").read_text())
        stable = json.loads((bag_dir / "summary_cv_stable.json").read_text())
        bag_config = json.loads((bag_dir / "tracker_config.json").read_text())
        if config is None:
            config = bag_config
        elif config != bag_config:
            raise ValueError(f"inconsistent tracker config in {case_dir}")

        tracking = regular["tracking"]
        all_h = regular["prediction"]["multimodal"]["horizons"]
        stable_h = stable["prediction"]["multimodal"]["horizons"]
        row = {
            "tracking_f1": tracking["f1"],
            "center_p50_m": tracking["bbox_center_absolute_distance_m"]["p50"],
            "speed_mae_mps": tracking["absolute_speed_error_mps"]["mean"],
            "ave_mps": tracking["velocity_l2_error_mps"]["mean"],
            "longitudinal_velocity_mae_mps": tracking[
                "absolute_longitudinal_velocity_error_mps"
            ]["mean"],
            "lateral_velocity_mae_mps": tracking[
                "absolute_lateral_velocity_error_mps"
            ]["mean"],
            "yaw_mae_deg": tracking["absolute_yaw_error_deg"]["mean"],
            "all_top1_fde_1s_m": all_h["1"]["top1_fde_m"]["mean"],
            "all_top1_fde_2s_m": all_h["2"]["top1_fde_m"]["mean"],
            "all_top1_fde_3s_m": all_h["3"]["top1_fde_m"]["mean"],
            "all_coverage_3s": all_h["3"]["top1_coverage"],
            "stable_top1_fde_1s_m": stable_h["1"]["top1_fde_m"]["mean"],
            "stable_top1_fde_2s_m": stable_h["2"]["top1_fde_m"]["mean"],
            "stable_top1_fde_3s_m": stable_h["3"]["top1_fde_m"]["mean"],
            "stable_minfde_k_1s_m": stable_h["1"]["min_fde_k_m"]["mean"],
            "stable_minfde_k_2s_m": stable_h["2"]["min_fde_k_m"]["mean"],
            "stable_minfde_k_3s_m": stable_h["3"]["min_fde_k_m"]["mean"],
        }
        per_bag[bag] = row
        for key, value in row.items():
            values[key].append(value)
    return {
        "config": config,
        "macro": {key: mean(rows) for key, rows in values.items()},
        "per_bag": per_bag,
    }


def markdown(results):
    lines = [
        "# Highway tracker ablation",
        "",
        "Equal-weight macro of `vehicle_highway_merge` and `vehicle_highway1`.",
        "Playback must be 1.0x for the internal 30 Hz scheduler.",
        "",
        "| Case | F1 ↑ | Center p50 ↓ | Speed MAE ↓ | AVE ↓ | Long/Lat MAE ↓ | Yaw MAE ↓ | "
        "stable minFDE-K 1/2/3s ↓ | all Top-1 FDE 1/2/3s ↓ |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, result in results.items():
        row = result["macro"]
        stable = "/".join(f"{row[f'stable_minfde_k_{h}s_m']:.3f}" for h in (1, 2, 3))
        all_fde = "/".join(f"{row[f'all_top1_fde_{h}s_m']:.3f}" for h in (1, 2, 3))
        lines.append(
            f"| {name} | {row['tracking_f1']:.4f} | {row['center_p50_m']:.3f} | "
            f"{row['speed_mae_mps']:.3f} | {row['ave_mps']:.3f} | "
            f"{row['longitudinal_velocity_mae_mps']:.3f}/"
            f"{row['lateral_velocity_mae_mps']:.3f} | "
            f"{row['yaw_mae_deg']:.2f} | {stable} | {all_fde} |"
        )
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    args = parser.parse_args()

    results = {}
    for case_dir in sorted(path for path in args.root.iterdir() if path.is_dir()):
        if all((case_dir / bag / "summary_cv_stable.json").is_file() for bag in BAGS):
            results[case_dir.name] = extract(case_dir)
    if not results:
        raise SystemExit("no complete two-bag cases found")
    args.output_json.write_text(json.dumps(results, indent=2) + "\n")
    args.output_markdown.write_text(markdown(results))
    print(args.output_markdown)


if __name__ == "__main__":
    main()
