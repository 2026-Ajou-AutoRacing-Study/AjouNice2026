#!/usr/bin/env python3
"""Benchmark one PointPainting training batch size on a single GPU.

The timing includes DataLoader iteration, host-to-device transfer, forward,
backward, gradient clipping, and the optimizer step.  Invoke this script in a
fresh process for every batch size so CUDA peak-memory accounting and OOM
results remain independent.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

import torch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--openpcdet-root", default="/home/luv/OpenPCDet")
    parser.add_argument("--cfg-file", required=True)
    parser.add_argument("--data-path", required=True)
    parser.add_argument("--batch-size", type=int, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--warmup-iters", type=int, default=15)
    parser.add_argument("--measure-iters", type=int, default=60)
    parser.add_argument("--amp", action="store_true")
    parser.add_argument("--output-json")
    return parser.parse_args()


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def main() -> None:
    args = parse_args()
    if args.batch_size <= 0 or args.measure_iters <= 0 or args.warmup_iters < 0:
        raise ValueError("batch size and iteration counts must be positive")

    openpcdet_root = Path(args.openpcdet_root).expanduser().resolve()
    cfg_file = Path(args.cfg_file).expanduser().resolve()
    data_path = Path(args.data_path).expanduser().resolve()
    sys.path.insert(0, str(openpcdet_root))

    from pcdet.config import cfg, cfg_from_yaml_file
    from pcdet.datasets import build_dataloader
    from pcdet.models import build_network, model_fn_decorator
    from pcdet.utils import common_utils
    from tools.train_utils.optimization import build_optimizer

    os.chdir(openpcdet_root / "tools")
    cfg_from_yaml_file(str(cfg_file), cfg)
    cfg.DATA_CONFIG.DATA_PATH = str(data_path)
    logger = common_utils.create_logger(
        f"/tmp/pointpainting_batch_benchmark_bs{args.batch_size}.log",
        rank=1,
    )
    dataset, loader, _ = build_dataloader(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        batch_size=args.batch_size,
        dist=False,
        workers=args.workers,
        logger=logger,
        training=True,
        total_epochs=1,
    )

    model = build_network(cfg.MODEL, len(cfg.CLASS_NAMES), dataset).cuda().train()
    optimizer = build_optimizer(model, cfg.OPTIMIZATION)
    model_func = model_fn_decorator()
    scaler = torch.amp.GradScaler("cuda", enabled=args.amp)
    iterator = iter(loader)
    timings: list[float] = []
    losses: list[float] = []
    total_iters = args.warmup_iters + args.measure_iters

    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    for iteration in range(total_iters):
        started = time.perf_counter()
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            batch = next(iterator)

        optimizer.zero_grad()
        with torch.amp.autocast("cuda", enabled=args.amp):
            loss, _, _ = model_func(model, batch)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), cfg.OPTIMIZATION.GRAD_NORM_CLIP)
        scaler.step(optimizer)
        scaler.update()
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - started
        if iteration >= args.warmup_iters:
            timings.append(elapsed)
            losses.append(float(loss.detach().cpu()))

    mean_sec = statistics.fmean(timings)
    result = {
        "batch_size": args.batch_size,
        "workers": args.workers,
        "amp": args.amp,
        "dataset_frames": len(dataset),
        "warmup_iters": args.warmup_iters,
        "measure_iters": args.measure_iters,
        "iteration_sec_mean": mean_sec,
        "iteration_sec_p50": percentile(timings, 0.50),
        "iteration_sec_p95": percentile(timings, 0.95),
        "samples_per_sec": args.batch_size / mean_sec,
        "loss_mean": statistics.fmean(losses),
        "loss_min": min(losses),
        "loss_max": max(losses),
        "peak_allocated_mib": torch.cuda.max_memory_allocated() / 2**20,
        "peak_reserved_mib": torch.cuda.max_memory_reserved() / 2**20,
    }
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output_json:
        output = Path(args.output_json).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
