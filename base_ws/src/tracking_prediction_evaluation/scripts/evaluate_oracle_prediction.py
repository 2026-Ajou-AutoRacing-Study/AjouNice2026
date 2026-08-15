#!/usr/bin/env python3
"""Evaluate CTRV/CTRA using causal MORAI GT history (E0 oracle benchmark)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.evaluator import (
    EvaluationConfig,
    build_summary,
    evaluate_segments,
    write_report,
)
from tracking_prediction_evaluation.models import default_predictors
from tracking_prediction_evaluation.trajectory import read_morai_trajectories


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/Object_topic")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--classes",
        nargs="+",
        choices=("vehicle", "pedestrian", "obstacle"),
        default=("vehicle", "pedestrian"),
    )
    parser.add_argument(
        "--horizons", nargs="+", type=float, default=(0.5, 1.0, 1.5, 2.0, 3.0)
    )
    parser.add_argument("--evaluation-rate-hz", type=float, default=10.0)
    parser.add_argument("--history-sec", type=float, default=0.5)
    parser.add_argument("--trajectory-step-sec", type=float, default=0.1)
    parser.add_argument("--maximum-assignment-distance-m", type=float, default=2.0)
    parser.add_argument("--maximum-gap-sec", type=float, default=0.5)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--worst-sample-count", type=int, default=100)
    return parser.parse_args()


def main():
    args = parse_args()
    config = EvaluationConfig(
        horizons=tuple(sorted(set(args.horizons))),
        trajectory_step_sec=args.trajectory_step_sec,
        evaluation_rate_hz=args.evaluation_rate_hz,
        history_sec=args.history_sec,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
    )
    segments = read_morai_trajectories(
        args.bag,
        topic=args.topic,
        include_classes=args.classes,
        maximum_assignment_distance_m=args.maximum_assignment_distance_m,
        maximum_gap_sec=args.maximum_gap_sec,
    )
    predictors = default_predictors()
    samples = evaluate_segments(segments, predictors, config)
    summary = build_summary(
        samples,
        config,
        source={
            "bag": str(args.bag.resolve()),
            "topic": args.topic,
            "classes": list(args.classes),
            "trajectory_segment_count": len(segments),
            "models": [predictor.name for predictor in predictors],
        },
    )
    summary_path, worst_path = write_report(
        args.output_dir,
        summary,
        samples,
        worst_sample_count=args.worst_sample_count,
    )
    print(json.dumps({
        "evaluation": summary["evaluation"],
        "segments": len(segments),
        "samples": len(samples),
        "summary": str(summary_path),
        "worst_samples": str(worst_path),
        "paired_comparison": summary["paired_comparison"],
    }, indent=2))


if __name__ == "__main__":
    main()
