#!/usr/bin/env python3
"""Diagnose class-wise yaw ambiguity in a Fast-BEV checkpoint."""

import argparse
import math
from collections import defaultdict

import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model
from mmcv.utils import import_modules_from_strings
from mmdet3d.datasets import build_dataloader, build_dataset
from mmdet3d.models import build_model


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--distance-threshold", type=float, default=2.0)
    parser.add_argument("--fp16", action="store_true")
    return parser.parse_args()


def wrapped_abs(delta, period=2.0 * math.pi):
    return abs((delta + period / 2.0) % period - period / 2.0)


def percentile_summary(values):
    values = np.asarray(values, dtype=np.float64)
    if not len(values):
        return "n=0"
    degrees = np.degrees(values)
    return (
        f"n={len(values)}, mean={degrees.mean():.2f} deg, "
        f"median={np.median(degrees):.2f} deg, "
        f"p90={np.percentile(degrees, 90):.2f} deg"
    )


def normalize_result(result):
    if "pts_bbox" in result:
        result = result["pts_bbox"]
    elif "img_bbox" in result:
        result = result["img_bbox"]
    return (
        result["boxes_3d"].tensor.detach().cpu().numpy()[:, :7],
        result["scores_3d"].detach().cpu().numpy(),
        result["labels_3d"].detach().cpu().numpy(),
    )


def main():
    args = parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("A CUDA GPU is required")

    cfg = Config.fromfile(args.config)
    import_modules_from_strings(**cfg.custom_imports)
    cfg.data.test.test_mode = True
    dataset = build_dataset(cfg.data.test)
    loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=0,
        num_gpus=1,
        dist=False,
        shuffle=False,
    )

    cfg.model.train_cfg = None
    model = build_model(cfg.model, test_cfg=cfg.get("test_cfg"))
    if args.fp16:
        wrap_fp16_model(model)
    checkpoint = load_checkpoint(model, args.checkpoint, map_location="cpu")
    model.CLASSES = checkpoint.get("meta", {}).get("CLASSES", dataset.CLASSES)
    model = MMDataParallel(model.cuda(), device_ids=[torch.cuda.current_device()])
    model.eval()

    samples = []
    for info, batch in zip(dataset.data_infos, loader):
        with torch.no_grad():
            result = model(return_loss=False, rescale=True, **batch)[0]
        pred_boxes, pred_scores, pred_labels = normalize_result(result)
        valid = (
            np.asarray(info["valid_flag"], dtype=bool)
            if dataset.use_valid_flag
            else np.asarray(info["num_lidar_pts"]) > 0
        )
        gt_names = np.asarray(info["gt_names"])[valid]
        gt_labels = np.asarray([
            dataset.CLASSES.index(name) if name in dataset.CLASSES else -1
            for name in gt_names
        ])
        known = gt_labels >= 0
        samples.append({
            "gt_boxes": np.asarray(info["gt_boxes"])[valid][known, :7],
            "gt_labels": gt_labels[known],
            "pred_boxes": pred_boxes,
            "pred_scores": pred_scores,
            "pred_labels": pred_labels,
        })

    for class_index, class_name in enumerate(dataset.CLASSES):
        ranked = []
        for sample_index, sample in enumerate(samples):
            for pred_index in np.flatnonzero(sample["pred_labels"] == class_index):
                ranked.append((
                    float(sample["pred_scores"][pred_index]),
                    sample_index,
                    int(pred_index),
                ))
        ranked.sort(reverse=True)
        taken = set()
        errors = defaultdict(list)
        for _score, sample_index, pred_index in ranked:
            sample = samples[sample_index]
            candidates = np.flatnonzero(sample["gt_labels"] == class_index)
            candidates = [
                int(index) for index in candidates
                if (sample_index, int(index)) not in taken
            ]
            if not candidates:
                continue
            pred_box = sample["pred_boxes"][pred_index]
            distances = [
                np.linalg.norm(sample["gt_boxes"][index, :2] - pred_box[:2])
                for index in candidates
            ]
            best_offset = int(np.argmin(distances))
            if distances[best_offset] >= args.distance_threshold:
                continue
            gt_index = candidates[best_offset]
            taken.add((sample_index, gt_index))
            gt_box = sample["gt_boxes"][gt_index]
            delta = float(pred_box[6] - gt_box[6])
            aspect = max(gt_box[3], gt_box[4]) / max(
                min(gt_box[3], gt_box[4]), 1e-8)
            distance = float(np.linalg.norm(gt_box[:2]))
            errors["directed"].append(wrapped_abs(delta))
            errors["gt_yaw"].append(float(gt_box[6]))
            errors["pred_yaw"].append(float(pred_box[6]))
            errors["axial"].append(wrapped_abs(delta, math.pi))
            errors["plus_90"].append(wrapped_abs(delta + math.pi / 2.0))
            errors["minus_90"].append(wrapped_abs(delta - math.pi / 2.0))
            errors["plus_180"].append(wrapped_abs(delta + math.pi))
            bucket = "near_square" if aspect <= 1.25 else "elongated"
            errors[bucket].append(wrapped_abs(delta))
            for lower, upper in ((0, 10), (10, 20), (20, 30), (30, 50)):
                if lower <= distance < upper:
                    errors[f"range_{lower}_{upper}m"].append(
                        wrapped_abs(delta))
                    break

        print(f"[{class_name}]")
        for name in (
            "directed", "axial", "plus_90", "minus_90", "plus_180",
            "near_square", "elongated", "range_0_10m", "range_10_20m",
            "range_20_30m", "range_30_50m",
        ):
            print(f"  {name}: {percentile_summary(errors[name])}")
        for name in ("gt_yaw", "pred_yaw"):
            angles = np.asarray(errors[name], dtype=np.float64)
            resultant = abs(np.mean(np.exp(1j * angles))) if len(angles) else 0.0
            print(
                f"  {name}_circular_concentration: {resultant:.4f} "
                "(0=uniform, 1=single direction)"
            )


if __name__ == "__main__":
    main()
