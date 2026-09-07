#!/usr/bin/env python3
"""Offline oracle audit of Vehicle detection versus Ego interaction relevance.

This is an experiment-only evaluator.  It never changes runtime topics or
production filtering.  Relevance is derived retrospectively from MORAI GT Ego
and actor trajectories, then detector recall is reported per relevance slice.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        action="append",
        nargs=4,
        metavar=("MODEL", "SCENE", "RAW_BAG", "DETECTION_BAG"),
        required=True,
    )
    parser.add_argument(
        "--tracking-input",
        action="append",
        nargs=4,
        metavar=("MODEL", "SCENE", "RAW_BAG", "PIPELINE_BAG"),
        default=[],
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--horizon-sec", type=float, default=3.0)
    parser.add_argument("--step-sec", type=float, default=0.1)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--match-distance-m", type=float, default=1.0)
    return parser.parse_args()


def configure_imports(repo_root):
    source = repo_root / "base_ws/src/tracking_prediction_evaluation/src"
    sys.path.insert(0, str(source))


def direction_bin(ego, state):
    dx = state.x - ego.x
    dy = state.y - ego.y
    cosine = math.cos(ego.yaw)
    sine = math.sin(ego.yaw)
    x = cosine * dx + sine * dy
    y = -sine * dx + cosine * dy
    if x >= abs(y):
        return "front"
    if -x >= abs(y):
        return "rear"
    return "left" if y >= 0.0 else "right"


def greedy_matches(gt_items, predictions, maximum_distance):
    candidates = []
    for prediction_index, prediction in enumerate(predictions):
        if prediction.class_name != "vehicle":
            continue
        for gt_index, (_, state) in enumerate(gt_items):
            distance = math.hypot(prediction.x - state.x, prediction.y - state.y)
            if distance <= maximum_distance:
                candidates.append(
                    (-prediction.score, distance, prediction_index, gt_index)
                )
    matches = {}
    used_predictions = set()
    used_gt = set()
    for negative_score, distance, prediction_index, gt_index in sorted(candidates):
        if prediction_index in used_predictions or gt_index in used_gt:
            continue
        used_predictions.add(prediction_index)
        used_gt.add(gt_index)
        matches[gt_index] = {
            "score": -negative_score,
            "center_error_m": distance,
        }
    return matches


def future_interaction(segment, ego_points, timestamp, horizon_sec, step_sec):
    from tracking_prediction_evaluation.pipeline_evaluator import interpolate_ego
    from tracking_prediction_evaluation.trajectory import interpolate_point

    times = np.arange(0.0, horizon_sec + step_sec * 0.5, step_sec)
    samples = []
    for offset in times:
        future_time = timestamp + float(offset)
        ego = interpolate_ego(ego_points, future_time, 0.15)
        actor = interpolate_point(segment.points, future_time, 0.15)
        if ego is None or actor is None:
            continue
        samples.append((float(offset), ego, actor))
    if not samples or samples[0][0] > step_sec * 0.5:
        return None

    ego_xy = np.asarray([(item[1].x, item[1].y) for item in samples])
    actor_xy = np.asarray([(item[2].x, item[2].y) for item in samples])
    synchronous = np.linalg.norm(actor_xy - ego_xy, axis=1)
    synchronous_index = int(np.argmin(synchronous))

    pairwise = np.linalg.norm(
        ego_xy[:, np.newaxis, :] - actor_xy[np.newaxis, :, :], axis=2
    )
    path_index = np.unravel_index(int(np.argmin(pairwise)), pairwise.shape)
    path_time_gap = abs(samples[path_index[0]][0] - samples[path_index[1]][0])
    current_distance = float(synchronous[0])
    final_distance = float(synchronous[-1])

    return {
        "available_horizon_sec": samples[-1][0],
        "current_distance_m": current_distance,
        "final_distance_m": final_distance,
        "distance_delta_m": final_distance - current_distance,
        "synchronous_min_distance_m": float(synchronous[synchronous_index]),
        "synchronous_min_time_sec": samples[synchronous_index][0],
        "path_min_distance_m": float(pairwise[path_index]),
        "path_min_time_gap_sec": float(path_time_gap),
    }


def relevance_label(interaction):
    """Nominal experimental tiers, intentionally not a production contract."""
    synchronous = interaction["synchronous_min_distance_m"]
    path_distance = interaction["path_min_distance_m"]
    current = interaction["current_distance_m"]
    # Geometric path overlap deliberately ignores the observed arrival-time
    # gap.  Ego may have yielded precisely because the actor was important;
    # using the safely realised time gap would then label that actor as
    # irrelevant.  Synchronous proximity is retained for close actors whose
    # short recorded paths have not yet geometrically crossed.
    if synchronous <= 8.0 or path_distance <= 3.5:
        return "conflict_critical"
    if synchronous <= 15.0 or current <= 15.0 or path_distance <= 5.0:
        return "planning_relevant"
    return "peripheral"


def trend_label(delta_m):
    if delta_m <= -2.0:
        return "approaching"
    if delta_m >= 2.0:
        return "receding"
    return "stable"


def distance_bin(value, edges, labels):
    for edge, label in zip(edges, labels):
        if value < edge:
            return label
    return labels[-1]


def load_context(repo_root, raw_bag):
    from tracking_prediction_evaluation.bbox_reference import (
        transform_segments_to_calibrated_bbox_center,
    )
    from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points
    from tracking_prediction_evaluation.static_transform import (
        load_morai_to_map_transform,
    )
    from tracking_prediction_evaluation.trajectory import (
        read_morai_trajectories,
        transform_segments_planar,
    )

    translation_xyz, transform_yaw = load_morai_to_map_transform(
        repo_root / "configs/tf/morai_lanelet_static_tf.yaml"
    )
    segments = read_morai_trajectories(
        raw_bag, topic="/Object_topic", include_classes=("vehicle",)
    )
    segments, _ = transform_segments_to_calibrated_bbox_center(segments)
    segments = transform_segments_planar(segments, translation_xyz, transform_yaw)
    ego_points = tuple(
        sorted(
            read_ego_points(raw_bag, "/Ego_topic", translation_xyz, transform_yaw),
            key=lambda item: item.timestamp,
        )
    )
    return segments, ego_points


def audit_input(
    repo_root,
    model,
    scene,
    raw_bag,
    detection_bag,
    context,
    args,
):
    from tracking_prediction_evaluation.detection_evaluator import (
        build_evaluation_frames,
        read_detection_frames,
    )

    segments, ego_points = context
    detection_frames = read_detection_frames(
        detection_bag, "/perception/object_recognition/detection/objects"
    )
    frames, _ = build_evaluation_frames(
        segments,
        ego_points,
        detection_frames,
        maximum_range_m=args.maximum_range_m,
    )
    records = []
    for frame in frames:
        gt_items = [
            (segment, state)
            for segment, state in frame.gt
            if segment.class_name == "vehicle"
        ]
        matches = greedy_matches(gt_items, frame.predictions, args.match_distance_m)
        for gt_index, (segment, state) in enumerate(gt_items):
            interaction = future_interaction(
                segment,
                ego_points,
                frame.timestamp,
                args.horizon_sec,
                args.step_sec,
            )
            if interaction is None:
                continue
            match = matches.get(gt_index)
            record = {
                "model": model,
                "scene": scene,
                "timestamp": frame.timestamp,
                "segment_key": segment.key,
                "direction": direction_bin(frame.ego, state),
                "relevance": relevance_label(interaction),
                "trend": trend_label(interaction["distance_delta_m"]),
                "matched_1m": match is not None,
                "score": float(match["score"]) if match else 0.0,
                "center_error_m": match["center_error_m"] if match else None,
            }
            record.update(interaction)
            records.append(record)
    return records


def audit_tracking_input(model, scene, pipeline_bag, context, args):
    from tracking_prediction_evaluation.evaluator import estimate_motion_state
    from tracking_prediction_evaluation.pipeline_evaluator import (
        active_gt,
        interpolate_ego,
        is_cv_stable_horizon,
        match_estimates,
        read_output_frames,
    )

    segments, ego_points = context
    tracking_frames, prediction_frames = read_output_frames(
        pipeline_bag,
        "/perception/object_recognition/tracking/objects",
        "/perception/object_recognition/objects",
    )
    records = []
    for timestamp, estimates in sorted(tracking_frames.items()):
        ego = interpolate_ego(ego_points, timestamp, 0.15)
        gt_items = active_gt(
            segments, timestamp, 0.15, ego, args.maximum_range_m
        )
        visible = tuple(
            estimate
            for estimate in estimates
            if ego is None
            or math.hypot(estimate.x - ego.x, estimate.y - ego.y)
            <= args.maximum_range_m
        )
        matches, _, _ = match_estimates(gt_items, visible, 2.0)
        match_by_gt = {
            gt_index: (visible[estimate_index], distance)
            for gt_index, estimate_index, distance in matches
        }
        for gt_index, (segment, state) in enumerate(gt_items):
            interaction = future_interaction(
                segment,
                ego_points,
                timestamp,
                args.horizon_sec,
                args.step_sec,
            )
            if interaction is None:
                continue
            match = match_by_gt.get(gt_index)
            speed_error = None
            track_id = None
            center_error = None
            if match is not None:
                estimate, center_error = match
                track_id = estimate.object_id.hex()
                history = [
                    point
                    for point in segment.points
                    if timestamp - 0.5 <= point.timestamp <= timestamp + 1.0e-9
                ]
                if history and abs(history[-1].timestamp - timestamp) > 1.0e-9:
                    history.append(state)
                motion = estimate_motion_state(history)
                if motion is not None:
                    speed_error = abs(estimate.speed - motion.speed)
            record = {
                "model": model,
                "scene": scene,
                "timestamp": timestamp,
                "segment_key": segment.key,
                "direction": direction_bin(ego, state),
                "relevance": relevance_label(interaction),
                "trend": trend_label(interaction["distance_delta_m"]),
                "matched_2m": match is not None,
                "center_error_m": center_error,
                "speed_error_mps": speed_error,
                "track_id": track_id,
            }
            record.update(interaction)
            records.append(record)
    prediction_records = []
    for timestamp, estimates in sorted(prediction_frames.items()):
        ego = interpolate_ego(ego_points, timestamp, 0.15)
        gt_items = active_gt(
            segments, timestamp, 0.15, ego, args.maximum_range_m
        )
        visible = tuple(
            estimate
            for estimate in estimates
            if ego is None
            or math.hypot(estimate.x - ego.x, estimate.y - ego.y)
            <= args.maximum_range_m
        )
        matches, _, _ = match_estimates(gt_items, visible, 2.0)
        match_by_gt = {
            gt_index: visible[estimate_index]
            for gt_index, estimate_index, _ in matches
        }
        for gt_index, (segment, state) in enumerate(gt_items):
            interaction = future_interaction(
                segment,
                ego_points,
                timestamp,
                args.horizon_sec,
                args.step_sec,
            )
            if interaction is None:
                continue
            estimate = match_by_gt.get(gt_index)
            paths = estimate.predicted_paths if estimate is not None else ()
            for horizon in (1.0, 2.0, 3.0):
                from tracking_prediction_evaluation.trajectory import interpolate_point

                future = interpolate_point(segment.points, timestamp + horizon, 0.15)
                if future is None:
                    continue
                fdes = []
                for path in paths:
                    sampled = sample_path(path.points, path.step_sec, horizon)
                    if sampled is not None:
                        fdes.append(math.hypot(sampled[0] - future.x, sampled[1] - future.y))
                prediction_records.append(
                    {
                        "model": model,
                        "scene": scene,
                        "timestamp": timestamp,
                        "segment_key": segment.key,
                        "horizon_sec": horizon,
                        "relevance": relevance_label(interaction),
                        "trend": trend_label(interaction["distance_delta_m"]),
                        "matched_path": bool(fdes),
                        "min_fde_m": min(fdes) if fdes else None,
                        "path_count": len(fdes),
                        "cv_stable": is_cv_stable_horizon(
                            segment, timestamp, horizon, 0.5, 0.15
                        ),
                    }
                )
    return records, prediction_records


def sample_path(points, step_sec, horizon_sec):
    if not points or step_sec <= 0.0:
        return None
    index = horizon_sec / step_sec
    low = int(math.floor(index + 1.0e-9))
    high = int(math.ceil(index - 1.0e-9))
    if low < 0 or high >= len(points):
        return None
    if low == high:
        return points[low]
    ratio = index - low
    return (
        points[low][0] + ratio * (points[high][0] - points[low][0]),
        points[low][1] + ratio * (points[high][1] - points[low][1]),
    )


def tracking_outages(records, relevance):
    by_segment = defaultdict(list)
    for row in records:
        by_segment[row["segment_key"]].append(row)
    durations = []
    for rows in by_segment.values():
        rows.sort(key=lambda row: row["timestamp"])
        positive_deltas = [
            second["timestamp"] - first["timestamp"]
            for first, second in zip(rows, rows[1:])
            if 0.0 < second["timestamp"] - first["timestamp"] <= 0.2
        ]
        nominal_dt = float(np.median(positive_deltas)) if positive_deltas else 0.0
        start = None
        last = None
        for row in rows:
            contiguous = last is not None and row["timestamp"] - last <= 0.2
            is_outage = row["relevance"] == relevance and not row["matched_2m"]
            if is_outage and (start is None or not contiguous):
                if start is not None:
                    durations.append(last - start + nominal_dt)
                start = row["timestamp"]
            elif not is_outage and start is not None:
                durations.append(last - start + nominal_dt)
                start = None
            last = row["timestamp"]
        if start is not None:
            durations.append(last - start + nominal_dt)
    return durations


def tracking_statistics(records, all_records, relevance):
    if not records:
        return {"count": 0}
    matched = np.asarray([row["matched_2m"] for row in records], dtype=bool)
    center = np.asarray(
        [row["center_error_m"] for row in records if row["center_error_m"] is not None]
    )
    speed = np.asarray(
        [row["speed_error_mps"] for row in records if row["speed_error_mps"] is not None]
    )
    outages = np.asarray(tracking_outages(all_records, relevance), dtype=np.float64)
    switches = 0
    previous = {}
    for row in sorted(records, key=lambda item: item["timestamp"]):
        if not row["matched_2m"]:
            continue
        key = row["segment_key"]
        old = previous.get(key)
        if old is not None and old[0] != row["track_id"] and row["timestamp"] - old[1] <= 0.5:
            switches += 1
        previous[key] = (row["track_id"], row["timestamp"])
    return {
        "count": len(records),
        "coverage": float(matched.mean()),
        "miss_count": int((~matched).sum()),
        "center_error_p50_m": float(np.median(center)) if len(center) else None,
        "center_error_p90_m": float(np.percentile(center, 90)) if len(center) else None,
        "speed_error_p50_mps": float(np.median(speed)) if len(speed) else None,
        "speed_error_p90_mps": float(np.percentile(speed, 90)) if len(speed) else None,
        "outage_p95_sec": float(np.percentile(outages, 95)) if len(outages) else 0.0,
        "outage_max_sec": float(np.max(outages)) if len(outages) else 0.0,
        "id_switches": switches,
    }


def summarize_tracking(records):
    output = {}
    for relevance in ("conflict_critical", "planning_relevant", "peripheral"):
        rows = [row for row in records if row["relevance"] == relevance]
        if rows:
            output[relevance] = tracking_statistics(rows, records, relevance)
    return output


def summarize_prediction(records):
    output = {}
    for relevance in ("conflict_critical", "planning_relevant", "peripheral"):
        relevance_rows = [row for row in records if row["relevance"] == relevance]
        if not relevance_rows:
            continue
        output[relevance] = {"all": {}, "cv_stable": {}}
        for slice_name in ("all", "cv_stable"):
            for horizon in (1.0, 2.0, 3.0):
                rows = [
                    row
                    for row in relevance_rows
                    if row["horizon_sec"] == horizon
                    and (slice_name == "all" or row["cv_stable"])
                ]
                if not rows:
                    continue
                errors = np.asarray(
                    [
                        row["min_fde_m"]
                        for row in rows
                        if row["min_fde_m"] is not None
                    ],
                    dtype=np.float64,
                )
                output[relevance][slice_name][str(int(horizon)) + "s"] = {
                    "eligible": len(rows),
                    "coverage": float(len(errors) / len(rows)),
                    "min_fde_p50_m": (
                        float(np.percentile(errors, 50)) if len(errors) else None
                    ),
                    "min_fde_p90_m": (
                        float(np.percentile(errors, 90)) if len(errors) else None
                    ),
                    "miss_rate_2m": (
                        float(np.mean(errors > 2.0)) if len(errors) else None
                    ),
                }
    return output


def statistics(records):
    if not records:
        return {"count": 0}
    matched = np.asarray([row["matched_1m"] for row in records], dtype=bool)
    errors = np.asarray(
        [row["center_error_m"] for row in records if row["center_error_m"] is not None],
        dtype=np.float64,
    )
    return {
        "count": len(records),
        "share": None,
        "recall_1m": float(matched.mean()),
        "miss_count": int((~matched).sum()),
        "score_mean_including_misses": float(
            np.mean([row["score"] for row in records])
        ),
        "matched_center_error_p50_m": float(np.median(errors)) if len(errors) else None,
    }


def grouped(records, key_function, order):
    buckets = defaultdict(list)
    for row in records:
        buckets[key_function(row)].append(row)
    total = len(records)
    output = {}
    for key in order:
        if key not in buckets:
            continue
        output[key] = statistics(buckets[key])
        output[key]["share"] = len(buckets[key]) / max(total, 1)
    return output


def summarize(records):
    return {
        "overall": statistics(records),
        "relevance": grouped(
            records,
            lambda row: row["relevance"],
            ("conflict_critical", "planning_relevant", "peripheral"),
        ),
        "trend": grouped(
            records,
            lambda row: row["trend"],
            ("approaching", "stable", "receding"),
        ),
        "direction": grouped(
            records,
            lambda row: row["direction"],
            ("front", "left", "right", "rear"),
        ),
        "synchronous_min_distance": grouped(
            records,
            lambda row: distance_bin(
                row["synchronous_min_distance_m"],
                (5.0, 10.0, 15.0, 20.0, math.inf),
                ("0-5m", "5-10m", "10-15m", "15-20m", "20m+"),
            ),
            ("0-5m", "5-10m", "10-15m", "15-20m", "20m+"),
        ),
    }


def markdown_table(title, mapping):
    lines = [
        "## " + title,
        "",
        "| Slice | GT | Share | Recall@1m | Miss | Score mean | Center p50 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, item in mapping.items():
        center = item["matched_center_error_p50_m"]
        lines.append(
            "| {} | {} | {:.1%} | {:.1%} | {} | {:.3f} | {} |".format(
                name,
                item["count"],
                item["share"],
                item["recall_1m"],
                item["miss_count"],
                item["score_mean_including_misses"],
                "{:.3f} m".format(center) if center is not None else "—",
            )
        )
    return lines


def main():
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[3]
    configure_imports(repo_root)
    contexts = {}
    records = []
    for model, scene, raw_bag_value, detection_bag_value in args.input:
        raw_bag = Path(raw_bag_value).resolve()
        detection_bag = Path(detection_bag_value).resolve()
        context = contexts.get(raw_bag)
        if context is None:
            context = load_context(repo_root, raw_bag)
            contexts[raw_bag] = context
        records.extend(
            audit_input(
                repo_root,
                model,
                scene,
                raw_bag,
                detection_bag,
                context,
                args,
            )
        )

    tracking_records = []
    prediction_records = []
    for model, scene, raw_bag_value, pipeline_bag_value in args.tracking_input:
        raw_bag = Path(raw_bag_value).resolve()
        pipeline_bag = Path(pipeline_bag_value).resolve()
        context = contexts.get(raw_bag)
        if context is None:
            context = load_context(repo_root, raw_bag)
            contexts[raw_bag] = context
        new_tracking, new_prediction = audit_tracking_input(
            model, scene, pipeline_bag, context, args
        )
        tracking_records.extend(new_tracking)
        prediction_records.extend(new_prediction)

    by_source = defaultdict(list)
    for row in records:
        by_source[(row["model"], row["scene"])].append(row)
    report = {
        "protocol": {
            "status": "experiment_only",
            "horizon_sec": args.horizon_sec,
            "step_sec": args.step_sec,
            "maximum_range_m": args.maximum_range_m,
            "match_distance_m": args.match_distance_m,
            "critical": "sync_min<=8m OR asynchronous GT swept-path min<=3.5m",
            "relevant": "sync/current<=15m OR GT swept-path min<=5m",
            "warning": "Retrospective GT oracle, not a production filter.",
        },
        "sources": {
            "{}::{}".format(model, scene): summarize(rows)
            for (model, scene), rows in sorted(by_source.items())
        },
    }
    tracking_by_source = defaultdict(list)
    for row in tracking_records:
        tracking_by_source[(row["model"], row["scene"])].append(row)
    report["tracking_sources"] = {
        "{}::{}".format(model, scene): summarize_tracking(rows)
        for (model, scene), rows in sorted(tracking_by_source.items())
    }
    prediction_by_source = defaultdict(list)
    for row in prediction_records:
        prediction_by_source[(row["model"], row["scene"])].append(row)
    report["prediction_sources"] = {
        "{}::{}".format(model, scene): summarize_prediction(rows)
        for (model, scene), rows in sorted(prediction_by_source.items())
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "interaction_audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (args.output_dir / "interaction_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in records:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    with (args.output_dir / "interaction_tracking_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in tracking_records:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    with (args.output_dir / "interaction_prediction_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in prediction_records:
            stream.write(json.dumps(row, sort_keys=True) + "\n")

    lines = ["# Vehicle interaction relevance audit.", ""]
    for source, summary in report["sources"].items():
        lines.extend(markdown_table(source + " / relevance", summary["relevance"]))
        lines.append("")
        lines.extend(markdown_table(source + " / trend", summary["trend"]))
        lines.append("")
        lines.extend(
            markdown_table(
                source + " / synchronous future minimum distance",
                summary["synchronous_min_distance"],
            )
        )
        lines.append("")
    (args.output_dir / "interaction_audit.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
