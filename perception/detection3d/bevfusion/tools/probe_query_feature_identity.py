#!/usr/bin/env python3
"""Measure whether Q1 query features help recover the same GT instance.

This is a diagnostic only.  GT instance IDs label query pairs after inference;
they are never supplied to the detector or its online association path.
"""

from __future__ import annotations

import argparse
import json
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch
from mmengine.config import Config
from mmengine.runner import Runner
from scipy.optimize import linear_sum_assignment

from perception.detection3d.bevfusion.plugin.morai_query_memory import (
    planar_source_to_target_matrix,
    transform_query_centers,
)


def _percentiles(values):
    array = np.asarray(values, dtype=np.float64)
    if not len(array):
        return {"count": 0}
    return {
        "count": int(len(array)),
        "mean": float(array.mean()),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
    }


def _load_infos(path: Path):
    with path.open("rb") as handle:
        payload = pickle.load(handle)
    items = payload.get("data_list", payload.get("infos", payload))
    return {
        (str(item["scene_id"]), int(item["sequence_frame_index"])): item
        for item in items
    }


def _assign_queries(frame, info, minimum_score, maximum_distance):
    query_indices = torch.where(frame.scores >= minimum_score)[0]
    instances = [
        instance for instance in info.get("instances", [])
        if int(instance.get("bbox_label_3d", -1)) == 0
        and "instance_id" in instance
    ]
    query_to_id = torch.full(
        (frame.centers_xy.shape[0],), -1, dtype=torch.long,
        device=frame.centers_xy.device)
    if not len(query_indices) or not instances:
        return query_to_id, {}
    gt_centers = torch.as_tensor(
        [instance["bbox_3d"][:2] for instance in instances],
        device=frame.centers_xy.device, dtype=frame.centers_xy.dtype)
    distances = torch.cdist(frame.centers_xy[query_indices], gt_centers)
    rows, columns = linear_sum_assignment(distances.detach().cpu().numpy())
    gt_velocity = {}
    for row, column in zip(rows, columns):
        if float(distances[row, column]) > maximum_distance:
            continue
        query_to_id[query_indices[row]] = int(instances[column]["instance_id"])
        gt_velocity[int(instances[column]["instance_id"])] = np.asarray(
            instances[column]["bbox_3d"][7:9], dtype=np.float64)
    return query_to_id, gt_velocity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("infos", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--minimum-score", type=float, default=0.20)
    parser.add_argument("--assignment-distance", type=float, default=2.0)
    parser.add_argument("--association-distance", type=float, default=6.0)
    args = parser.parse_args()

    infos = _load_infos(args.infos)
    cfg = Config.fromfile(str(args.config))
    cfg.load_from = str(args.checkpoint)
    cfg.work_dir = "/tmp/ajounice-query-feature-probe"
    cfg.log_level = "WARNING"
    cfg.visualizer.vis_backends = []
    cfg.test_dataloader.num_workers = 0
    cfg.test_dataloader.persistent_workers = False
    runner = Runner.from_cfg(cfg)
    runner.load_or_resume()
    model = runner.model
    model.eval()
    head = model.bbox_head

    hits = defaultdict(int)
    opportunities = 0
    candidate_counts = []
    same_cosines = []
    other_cosines = []
    pair_speed_errors = []
    pair_vector_errors = []
    previous = None
    frame_count = 0

    with torch.no_grad():
        for batch in runner.test_dataloader:
            sample = batch["data_samples"][0]
            meta = sample.metainfo
            model.test_step(batch)
            frame = head.query_memory_bank._frames[-1]
            key = (str(meta["scene_id"]), int(meta["sequence_frame_index"]))
            info = infos[key]
            query_to_id, gt_velocity = _assign_queries(
                frame, info, args.minimum_score, args.assignment_distance)
            current = {
                "frame": frame,
                "ids": query_to_id,
                "gt_velocity": gt_velocity,
                "scene": key[0],
            }
            frame_count += 1
            if previous is None or previous["scene"] != current["scene"]:
                previous = current
                continue
            dt = frame.timestamp_sec - previous["frame"].timestamp_sec
            if dt <= 0.0 or dt > head.query_memory_bank.reset_gap_sec:
                previous = current
                continue

            transform = planar_source_to_target_matrix(
                previous["frame"].lidar_pose_world,
                frame.lidar_pose_world,
                device=frame.centers_xy.device)
            previous_centers = transform_query_centers(
                previous["frame"].centers_xy, transform)
            previous_valid = previous["frame"].scores >= args.minimum_score
            current_valid = frame.scores >= args.minimum_score
            for current_index in torch.where(current_valid & (query_to_id >= 0))[0]:
                instance_id = int(query_to_id[current_index])
                target_indices = torch.where(
                    previous_valid & (previous["ids"] == instance_id))[0]
                if len(target_indices) != 1:
                    continue
                distances = torch.linalg.vector_norm(
                    previous_centers - frame.centers_xy[current_index], dim=-1)
                candidates = torch.where(
                    previous_valid
                    & (distances <= args.association_distance))[0]
                target_index = int(target_indices[0])
                if target_index not in candidates.tolist() or not len(candidates):
                    continue
                opportunities += 1
                candidate_counts.append(len(candidates))
                current_feature = torch.nn.functional.normalize(
                    frame.features[:, current_index].float(), dim=0)
                candidate_features = torch.nn.functional.normalize(
                    previous["frame"].features[:, candidates].float(), dim=0)
                cosine = current_feature @ candidate_features
                target_offset = candidates.tolist().index(target_index)
                same_cosines.append(float(cosine[target_offset]))
                other_cosines.extend(
                    float(value) for index, value in enumerate(cosine)
                    if index != target_offset)
                hits["distance"] += int(
                    int(candidates[torch.argmin(distances[candidates])])
                    == target_index)
                hits["cosine"] += int(
                    int(candidates[torch.argmax(cosine)]) == target_index)
                for weight in (0.25, 0.5, 1.0, 2.0):
                    score = cosine - weight * (
                        distances[candidates] / args.association_distance)
                    hits[f"hybrid_{weight:g}"] += int(
                        int(candidates[torch.argmax(score)]) == target_index)

                pair_velocity = (
                    frame.centers_xy[current_index]
                    - previous_centers[target_index]) / dt
                target_velocity = current["gt_velocity"].get(instance_id)
                if target_velocity is not None:
                    estimate = pair_velocity.detach().cpu().numpy()
                    pair_speed_errors.append(abs(
                        np.linalg.norm(estimate)
                        - np.linalg.norm(target_velocity)))
                    pair_vector_errors.append(np.linalg.norm(
                        estimate - target_velocity))
            previous = current

    result = {
        "frames": frame_count,
        "association_opportunities": opportunities,
        "top1_accuracy": {
            name: (count / opportunities if opportunities else None)
            for name, count in sorted(hits.items())
        },
        "candidate_count": _percentiles(candidate_counts),
        "same_id_cosine": _percentiles(same_cosines),
        "different_id_cosine": _percentiles(other_cosines),
        "gt_id_query_pair_speed_mae_mps": _percentiles(pair_speed_errors),
        "gt_id_query_pair_vector_error_mps": _percentiles(pair_vector_errors),
        "contract": {
            "minimum_score": args.minimum_score,
            "assignment_distance_m": args.assignment_distance,
            "association_distance_m": args.association_distance,
            "gt_identity_used_by_model": False,
        },
    }
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
