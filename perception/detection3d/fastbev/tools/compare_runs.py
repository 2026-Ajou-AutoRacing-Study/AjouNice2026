#!/usr/bin/env python3
"""Create a compact Markdown comparison from two Fast-BEV test reports."""

import argparse
import json
from pathlib import Path


PRIMARY_METRICS = tuple(
    ["center/mAP"]
    + [
        f"center/{class_name}_AP_{threshold}m"
        for class_name in ("vehicle", "pedestrian", "obstacle")
        for threshold in ("0.5", "1", "2", "4")
    ]
    + [
        f"center/{class_name}_mean_AP"
        for class_name in ("vehicle", "pedestrian", "obstacle")
    ]
    + [
        "tp/vehicle_mATE",
        "tp/vehicle_mASE",
        "tp/vehicle_mAOE",
        "tp/pedestrian_mATE",
        "tp/pedestrian_mASE",
        "tp/pedestrian_mAOE",
        "tp/obstacle_mATE",
        "tp/obstacle_mASE",
        "tp/obstacle_mAOE",
        "iou/mAP_0.25",
        "iou/mAR_0.25",
        "iou/mAP_0.50",
        "iou/mAR_0.50",
    ])


def load_report(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def display(value):
    if value is None:
        return "—"
    return f"{float(value):.6f}"


def delta(left, right):
    if left is None or right is None:
        return "—"
    return f"{float(right) - float(left):+.6f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--baseline-name", default="PIL baseline")
    parser.add_argument("--candidate-name", default="OpenCV candidate")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    baseline = load_report(args.baseline)
    candidate = load_report(args.candidate)
    baseline_metrics = baseline["metrics"]
    candidate_metrics = candidate["metrics"]
    lines = [
        "# Fast-BEV experiment comparison",
        "",
        f"- Baseline checkpoint: `{baseline['checkpoint']}`",
        f"- Candidate checkpoint: `{candidate['checkpoint']}`",
        f"- Evaluation frames: {candidate['dataset_frames']}",
        "",
        "## Accuracy",
        "",
        f"| Metric | {args.baseline_name} | {args.candidate_name} | Delta |",
        "|---|---:|---:|---:|",
    ]
    for key in PRIMARY_METRICS:
        left = baseline_metrics.get(key)
        right = candidate_metrics.get(key)
        lines.append(
            f"| `{key}` | {display(left)} | {display(right)} | "
            f"{delta(left, right)} |")

    lines.extend([
        "",
        "## Runtime",
        "",
        f"| Metric | {args.baseline_name} | {args.candidate_name} | Delta |",
        "|---|---:|---:|---:|",
    ])
    runtime_keys = sorted(set(baseline.get("runtime", {})) | set(
        candidate.get("runtime", {})))
    for key in runtime_keys:
        left = baseline.get("runtime", {}).get(key)
        right = candidate.get("runtime", {}).get(key)
        lines.append(
            f"| `{key}` | {display(left)} | {display(right)} | "
            f"{delta(left, right)} |")

    output = "\n".join(lines) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")


if __name__ == "__main__":
    main()
