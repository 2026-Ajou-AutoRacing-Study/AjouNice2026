#!/usr/bin/env python3
"""Break MORAI obstacle errors down by geometry and ego-frame distance."""

import argparse
import json
import pickle
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.detection3d.common.obstacle_shape import (
    SHAPE_CLASSES,
    classify_obstacle_shape,
)


DEFAULT_INFOS = Path(
    "/home/luv/ajounice_data/morai_od/processed/"
    "fastbev_sunny13_1500_v1/fastbev_infos/morai_infos_test.pkl")
DISTANCE_BINS = (
    ("0_10m", 0.0, 10.0),
    ("10_20m", 10.0, 20.0),
    ("20_30m", 20.0, 30.0),
    ("30_50m", 30.0, 50.0),
)
MATCH_THRESHOLDS = (0.5, 1.0, 2.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--infos", type=Path, default=DEFAULT_INFOS)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--score-threshold", type=float, required=True)
    parser.add_argument("--obstacle-label", type=int, default=2)
    parser.add_argument("--shape-ratio-threshold", type=float, default=2.0)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path)
    return parser.parse_args()


def angle_error(gt_yaw, pred_yaw, period):
    difference = (pred_yaw - gt_yaw + period / 2.0) % period - period / 2.0
    return float(abs(difference))


def scale_error(gt_size, pred_size):
    gt_size = np.maximum(np.asarray(gt_size, dtype=np.float64), 1e-8)
    pred_size = np.maximum(np.asarray(pred_size, dtype=np.float64), 1e-8)
    intersection = np.prod(np.minimum(gt_size, pred_size))
    union = np.prod(gt_size) + np.prod(pred_size) - intersection
    return float(1.0 - intersection / max(union, 1e-8))


def greedy_match(gt_boxes, pred_boxes, pred_scores, threshold):
    """Return GT-to-pred matches and unmatched prediction indices."""
    order = np.argsort(-pred_scores)
    available = set(range(len(gt_boxes)))
    matches = {}
    matched_predictions = set()
    for pred_index in order:
        if not available:
            break
        candidates = np.asarray(sorted(available), dtype=np.int64)
        distances = np.linalg.norm(
            gt_boxes[candidates, :2] - pred_boxes[pred_index, :2], axis=1)
        nearest_offset = int(np.argmin(distances))
        distance = float(distances[nearest_offset])
        if distance < threshold:
            gt_index = int(candidates[nearest_offset])
            matches[gt_index] = (int(pred_index), distance)
            matched_predictions.add(int(pred_index))
            available.remove(gt_index)
    unmatched = sorted(set(range(len(pred_boxes))) - matched_predictions)
    return matches, unmatched


def mean_or_none(values):
    return float(np.mean(values)) if values else None


def median_or_none(values):
    return float(np.median(values)) if values else None


def summarize_group(records):
    matched = [record for record in records if record.get("match_2m")]
    sizes = np.asarray([record["gt_size"] for record in records])
    return {
        "gt": len(records),
        "recall_0.5m": (sum(record["match_0.5m"] for record in records) /
                        len(records) if records else None),
        "recall_1m": (sum(record["match_1m"] for record in records) /
                      len(records) if records else None),
        "recall_2m": (len(matched) / len(records) if records else None),
        "miss_2m": len(records) - len(matched),
        "mATE_2m_matches": mean_or_none([
            record["center_error"] for record in matched]),
        "mASE_2m_matches": mean_or_none([
            record["scale_error"] for record in matched]),
        "mAOE_2pi_2m_matches": mean_or_none([
            record["yaw_error_2pi"] for record in matched]),
        "mAOE_pi_symmetric_2m_matches": mean_or_none([
            record["yaw_error_pi"] for record in matched]),
        "median_gt_lwh": (np.median(sizes, axis=0).tolist()
                          if len(sizes) else None),
        "mean_num_lidar_pts": mean_or_none([
            record["num_lidar_pts"] for record in records]),
    }


def percentage(value):
    return "—" if value is None else f"{100.0 * value:.1f}%"


def number(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def markdown_table(title, summaries):
    lines = [
        f"## {title}", "",
        "| Group | GT | R@0.5m | R@1m | R@2m | mATE | mASE | "
        "mAOE | π-sym mAOE |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, summary in summaries.items():
        lines.append(
            f"| {name} | {summary['gt']} | "
            f"{percentage(summary['recall_0.5m'])} | "
            f"{percentage(summary['recall_1m'])} | "
            f"{percentage(summary['recall_2m'])} | "
            f"{number(summary['mATE_2m_matches'])} | "
            f"{number(summary['mASE_2m_matches'])} | "
            f"{number(summary['mAOE_2pi_2m_matches'])} | "
            f"{number(summary['mAOE_pi_symmetric_2m_matches'])} |")
    return lines


def main():
    args = parse_args()
    if not 0.0 <= args.score_threshold <= 1.0:
        raise ValueError("--score-threshold must be in [0, 1]")
    with args.infos.expanduser().resolve().open("rb") as stream:
        payload = pickle.load(stream)
    with args.predictions.expanduser().resolve().open("rb") as stream:
        predictions = pickle.load(stream)
    infos = payload["infos"] if isinstance(payload, dict) else payload
    if len(infos) != len(predictions):
        raise AssertionError(
            f"infos={len(infos)} predictions={len(predictions)}")

    records = []
    prediction_count = 0
    false_positive_2m = 0
    per_frame_false_positives = []
    confusion = Counter()
    for frame_index, (info, prediction) in enumerate(zip(infos, predictions)):
        valid = np.asarray(info["valid_flag"], dtype=bool)
        names = np.asarray(info["gt_names"])[valid]
        boxes = np.asarray(info["gt_boxes"], dtype=np.float64)[valid, :7]
        lidar_points = np.asarray(info["num_lidar_pts"])[valid]
        obstacle_mask = names == "obstacle"
        gt_boxes = boxes[obstacle_mask]
        gt_lidar_points = lidar_points[obstacle_mask]

        labels = np.asarray(prediction["labels"])
        scores = np.asarray(prediction["scores"], dtype=np.float64)
        pred_boxes_all = np.asarray(prediction["boxes"], dtype=np.float64)[:, :7]
        pred_mask = ((labels == args.obstacle_label) &
                     (scores >= args.score_threshold))
        pred_boxes = pred_boxes_all[pred_mask]
        pred_scores = scores[pred_mask]
        prediction_count += len(pred_boxes)

        matches = {}
        unmatched_2m = []
        for threshold in MATCH_THRESHOLDS:
            current, unmatched = greedy_match(
                gt_boxes, pred_boxes, pred_scores, threshold)
            matches[threshold] = current
            if threshold == 2.0:
                unmatched_2m = unmatched
        false_positive_2m += len(unmatched_2m)
        per_frame_false_positives.append(len(unmatched_2m))

        for gt_index, gt_box in enumerate(gt_boxes):
            shape = classify_obstacle_shape(
                gt_box[3:6], args.shape_ratio_threshold)
            distance = float(np.linalg.norm(gt_box[:2]))
            record = {
                "frame_index": frame_index,
                "token": str(info.get("token", frame_index)),
                "shape": shape,
                "distance": distance,
                "gt_size": gt_box[3:6].tolist(),
                "num_lidar_pts": int(gt_lidar_points[gt_index]),
            }
            for threshold in MATCH_THRESHOLDS:
                record[f"match_{threshold:g}m"] = gt_index in matches[threshold]
            if record["match_2m"]:
                pred_index, center_error = matches[2.0][gt_index]
                pred_box = pred_boxes[pred_index]
                pred_shape = classify_obstacle_shape(
                    pred_box[3:6], args.shape_ratio_threshold)
                confusion[(shape, pred_shape)] += 1
                record.update({
                    "center_error": center_error,
                    "scale_error": scale_error(gt_box[3:6], pred_box[3:6]),
                    "yaw_error_2pi": angle_error(
                        gt_box[6], pred_box[6], 2.0 * np.pi),
                    "yaw_error_pi": angle_error(
                        gt_box[6], pred_box[6], np.pi),
                    "predicted_shape": pred_shape,
                    "score": float(pred_scores[pred_index]),
                })
            records.append(record)

    shape_groups = {
        shape: summarize_group([
            record for record in records if record["shape"] == shape])
        for shape in SHAPE_CLASSES
    }
    distance_groups = {
        name: summarize_group([
            record for record in records
            if lower <= record["distance"] < upper])
        for name, lower, upper in DISTANCE_BINS
    }
    shape_distance = {
        shape: {
            name: summarize_group([
                record for record in records
                if record["shape"] == shape and
                lower <= record["distance"] < upper])
            for name, lower, upper in DISTANCE_BINS
        }
        for shape in SHAPE_CLASSES
    }
    worst_matches = sorted(
        (record for record in records if record.get("match_2m")),
        key=lambda record: record["center_error"], reverse=True)[:20]
    misses = [record for record in records if not record["match_2m"]]
    report = {
        "schema": "morai_obstacle_error_analysis_v1",
        "infos": str(args.infos.expanduser().resolve()),
        "predictions": str(args.predictions.expanduser().resolve()),
        "frames": len(infos),
        "score_threshold": args.score_threshold,
        "center_match_thresholds_m": list(MATCH_THRESHOLDS),
        "shape_ratio_threshold": args.shape_ratio_threshold,
        "overall": summarize_group(records),
        "predictions_above_threshold": prediction_count,
        "false_positives_at_2m": false_positive_2m,
        "frames_with_false_positives_at_2m": sum(
            value > 0 for value in per_frame_false_positives),
        "by_shape": shape_groups,
        "by_distance": distance_groups,
        "by_shape_and_distance": shape_distance,
        "matched_shape_confusion": {
            gt: {pred: int(confusion[(gt, pred)]) for pred in SHAPE_CLASSES}
            for gt in SHAPE_CLASSES
        },
        "misses_2m": misses,
        "worst_20_center_matches": worst_matches,
    }
    output_json = args.output_json.expanduser().resolve()
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    if args.output_markdown:
        lines = [
            "# Fast-BEV obstacle error decomposition", "",
            f"- Frames: `{len(infos)}`",
            f"- Validation-selected score threshold: "
            f"`{args.score_threshold:.5f}`",
            f"- Obstacle GT: `{len(records)}`",
            f"- Predictions above threshold: `{prediction_count}`",
            f"- False positives at 2 m matching: `{false_positive_2m}`",
            "",
        ]
        lines += markdown_table("By obstacle shape", shape_groups)
        lines += [""] + markdown_table("By ego-frame distance", distance_groups)
        lines += ["", "## Matched GT shape vs predicted box shape", "",
                  "| GT \\ Predicted | Compact | Tall | Long |",
                  "|---|---:|---:|---:|"]
        for gt in SHAPE_CLASSES:
            row = report["matched_shape_confusion"][gt]
            lines.append(
                f"| {gt} | {row['compact_obstacle']} | "
                f"{row['tall_obstacle']} | {row['long_obstacle']} |")
        output_markdown = args.output_markdown.expanduser().resolve()
        output_markdown.parent.mkdir(parents=True, exist_ok=True)
        output_markdown.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print("markdown:", output_markdown)
    print("json:", output_json)
    print("obstacle GT:", len(records))
    print("shape counts:", {key: value["gt"]
                             for key, value in shape_groups.items()})


if __name__ == "__main__":
    main()
