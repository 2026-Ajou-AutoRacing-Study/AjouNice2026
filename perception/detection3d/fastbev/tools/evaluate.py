#!/usr/bin/env python3
"""Evaluate MORAI Fast-BEV accuracy and single-frame throughput."""

import argparse
import json
import os
import statistics
import time
from pathlib import Path

import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model
from mmcv.utils import import_modules_from_strings
from mmdet3d.datasets import build_dataloader, build_dataset
from mmdet3d.models import build_model


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "configs/fastbev_m0_morai_4cam_overfit20.py"
)
DEFAULT_CHECKPOINT = (
    Path(__file__).resolve().parents[4]
    / "artifacts/detection3d/fastbev/runs/overfit20_v1/epoch_30.pth"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="benchmark inference with MMCV FP16 model wrapping",
    )
    parser.add_argument(
        "--cudnn-benchmark",
        action="store_true",
        help="enable cuDNN autotuning for the fixed input shape",
    )
    parser.add_argument(
        "--metrics",
        default="center,tp,iou",
        help="comma-separated MORAI metrics: center,tp,iou",
    )
    parser.add_argument(
        "--no-distance-bins",
        action="store_true",
        help="skip the 0-10/10-20/20-30/30-50 m diagnostics",
    )
    parser.add_argument(
        "--output-json",
        help="optional path for machine-readable metrics and runtime",
    )
    parser.add_argument(
        "--wandb",
        action="store_true",
        help="append test metrics to the selected W&B training run",
    )
    parser.add_argument("--wandb-run-id", default=os.getenv("WANDB_RUN_ID"))
    parser.add_argument(
        "--wandb-entity", default=os.getenv("WANDB_ENTITY", "AjouNice2026"))
    parser.add_argument(
        "--wandb-project", default=os.getenv("WANDB_PROJECT", "morai-fastbev"))
    return parser.parse_args()


def summarize(name, durations):
    values_ms = np.asarray(durations, dtype=np.float64) * 1000.0
    mean_sec = statistics.fmean(durations)
    print(f"{name} frames: {len(durations)}")
    print(f"{name} FPS: {1.0 / mean_sec:.3f}")
    print(f"{name} mean ms: {mean_sec * 1000.0:.3f}")
    print(f"{name} p50 ms: {np.percentile(values_ms, 50):.3f}")
    print(f"{name} p95 ms: {np.percentile(values_ms, 95):.3f}")


def main():
    args = parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("A CUDA GPU is required for Fast-BEV evaluation")
    if args.warmup < 0:
        raise ValueError("--warmup must be non-negative")
    torch.backends.cudnn.benchmark = args.cudnn_benchmark

    cfg = Config.fromfile(args.config)
    import_modules_from_strings(**cfg.custom_imports)
    cfg.data.test.test_mode = True

    dataset = build_dataset(cfg.data.test)
    if args.warmup >= len(dataset):
        raise ValueError("--warmup must be smaller than the dataset")
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

    results = []
    inference_times = []
    end_to_end_times = []
    iterator = iter(loader)
    torch.cuda.reset_peak_memory_stats()

    for index in range(len(dataset)):
        torch.cuda.synchronize()
        frame_start = time.perf_counter()
        batch = next(iterator)
        torch.cuda.synchronize()
        inference_start = time.perf_counter()
        with torch.no_grad():
            result = model(return_loss=False, rescale=True, **batch)
        torch.cuda.synchronize()
        frame_end = time.perf_counter()
        results.extend(result)

        if index >= args.warmup:
            inference_times.append(frame_end - inference_start)
            end_to_end_times.append(frame_end - frame_start)

    print("dataset frames:", len(dataset))
    print("warmup frames:", args.warmup)
    print("frame definition: one synchronized four-camera set")
    print("precision:", "FP16" if args.fp16 else "FP32")
    print("cudnn benchmark:", args.cudnn_benchmark)
    summarize("inference", inference_times)
    summarize("end-to-end", end_to_end_times)
    print(
        "peak CUDA MiB:",
        round(torch.cuda.max_memory_allocated() / 1024**2, 1),
    )
    # Print throughput before evaluation so timing results remain available if
    # a dataset-specific metric adapter fails.
    distance_bins = () if args.no_distance_bins else (
        (0.0, 10.0), (10.0, 20.0), (20.0, 30.0), (30.0, 50.0))
    metrics = dataset.evaluate(
        results,
        metric=args.metrics,
        distance_bins=distance_bins,
    )
    print("metrics:")
    for key in sorted(metrics):
        print(f"  {key}: {metrics[key]:.6f}")
    payload = {
        "evaluation_protocol": "morai_nuscenes_style_v1",
        "config": str(Path(args.config).expanduser().resolve()),
        "checkpoint": str(Path(args.checkpoint).expanduser().resolve()),
        "dataset_frames": len(dataset),
        "warmup_frames": args.warmup,
        "precision": "FP16" if args.fp16 else "FP32",
        "runtime": {
            "inference_fps": 1.0 / statistics.fmean(inference_times),
            "inference_mean_ms": statistics.fmean(inference_times) * 1000.0,
            "inference_p50_ms": float(
                np.percentile(inference_times, 50) * 1000.0),
            "inference_p95_ms": float(
                np.percentile(inference_times, 95) * 1000.0),
            "end_to_end_fps": 1.0 / statistics.fmean(end_to_end_times),
            "peak_cuda_mib": torch.cuda.max_memory_allocated() / 1024**2,
        },
        "metrics": {key: float(value) for key, value in metrics.items()},
    }
    if args.output_json:
        output_path = Path(args.output_json).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print("metrics JSON:", output_path)
    if args.wandb:
        if not args.wandb_run_id:
            raise ValueError("--wandb requires --wandb-run-id or WANDB_RUN_ID")
        import wandb

        run = wandb.init(
            entity=args.wandb_entity,
            project=args.wandb_project,
            id=args.wandb_run_id,
            resume="allow",
            dir=str(Path(args.checkpoint).expanduser().resolve().parent),
        )

        def wandb_metric(key):
            if key.startswith("tp/") or key.startswith("iou/"):
                return True
            if key.startswith("center/"):
                return "_best_" not in key and not key.endswith((
                    "_GT", "_predictions", "_ground_truth"))
            if key.startswith("range/"):
                return key.endswith(("_mean_AP", "/mAP"))
            return False

        # Keep the complete 377-metric report in JSON, but append only the
        # dashboard-sized final-test subset to W&B. This avoids hundreds of
        # automatically generated panels while preserving detailed offline
        # diagnostics.
        log_values = {
            f"test/{key}": value
            for key, value in payload["metrics"].items()
            if wandb_metric(key)
        }
        log_values.update({
            f"test/runtime/{key}": value
            for key, value in payload["runtime"].items()
        })
        log_values["test/dataset_frames"] = payload["dataset_frames"]
        run.log(log_values)
        run.summary["test/checkpoint"] = payload["checkpoint"]
        run.finish()
        print("W&B test metrics appended to run:", args.wandb_run_id)
    print("Fast-BEV MORAI evaluation + FPS: OK")


if __name__ == "__main__":
    main()
