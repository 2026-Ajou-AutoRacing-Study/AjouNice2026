#!/usr/bin/env python3
"""Repeatable Base3 LRASPP batch-size benchmark with bottleneck isolation."""

from __future__ import annotations

import argparse
import importlib.util
import json
import random
import statistics
import time
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader


def load_training_module():
    path = Path(__file__).with_name("train_lraspp_morai_4class_v2.py")
    spec = importlib.util.spec_from_file_location("lraspp_train", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--train-csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--batch-sizes", default="8,12,16,24,32")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--short-side", type=int, default=384)
    parser.add_argument("--warmup-steps", type=int, default=20)
    parser.add_argument("--measure-steps", type=int, default=100)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--seed", type=int, default=260830)
    return parser.parse_args()


def next_batch(iterator, loader):
    try:
        return next(iterator), iterator
    except StopIteration:
        iterator = iter(loader)
        return next(iterator), iterator


def make_components(module, args, batch_size, seed):
    module.seed_everything(seed)
    dataset = module.MoraiSegDataset(
        args.dataset_root, args.train_csv, args.short_side
    )
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=True,
        drop_last=True,
        collate_fn=module.segmentation_collate,
        persistent_workers=args.workers > 0,
    )
    device = torch.device("cuda")
    model = module.build_model().to(device).to(
        memory_format=torch.channels_last
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4)
    criterion = nn.CrossEntropyLoss(ignore_index=255)
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    return loader, model, optimizer, criterion, scaler, device


def train_step(model, optimizer, criterion, scaler, images, masks):
    optimizer.zero_grad(set_to_none=True)
    with torch.amp.autocast("cuda", enabled=True):
        output = model(images)["out"]
        loss = criterion(output, masks)
    scaler.scale(loss).backward()
    scaler.step(optimizer)
    scaler.update()
    return float(loss.item())


def benchmark_end_to_end(module, args, batch_size, seed):
    loader, model, optimizer, criterion, scaler, device = make_components(
        module, args, batch_size, seed
    )
    iterator = iter(loader)
    for _ in range(args.warmup_steps):
        (images, masks), iterator = next_batch(iterator, loader)
        images = images.to(
            device, non_blocking=True, memory_format=torch.channels_last
        )
        masks = masks.to(device, non_blocking=True)
        train_step(model, optimizer, criterion, scaler, images, masks)
    torch.cuda.synchronize(device)
    torch.cuda.reset_peak_memory_stats(device)

    data_wait = 0.0
    compute = 0.0
    losses = []
    started = time.perf_counter()
    for _ in range(args.measure_steps):
        data_started = time.perf_counter()
        (images, masks), iterator = next_batch(iterator, loader)
        data_wait += time.perf_counter() - data_started

        compute_started = time.perf_counter()
        images = images.to(
            device, non_blocking=True, memory_format=torch.channels_last
        )
        masks = masks.to(device, non_blocking=True)
        losses.append(train_step(
            model, optimizer, criterion, scaler, images, masks
        ))
        torch.cuda.synchronize(device)
        compute += time.perf_counter() - compute_started
    elapsed = time.perf_counter() - started
    images_seen = batch_size * args.measure_steps
    result = {
        "mode": "end_to_end",
        "batch_size": batch_size,
        "images": images_seen,
        "elapsed_sec": elapsed,
        "images_per_sec": images_seen / elapsed,
        "data_wait_sec_per_step": data_wait / args.measure_steps,
        "compute_sec_per_step": compute / args.measure_steps,
        "compute_ms_per_image": 1000.0 * compute / images_seen,
        "peak_cuda_gib": torch.cuda.max_memory_allocated(device) / 1024**3,
        "loss_first": losses[0],
        "loss_last": losses[-1],
    }
    del loader, model, optimizer, criterion, scaler, images, masks
    torch.cuda.empty_cache()
    return result


def benchmark_compute_only(module, args, batch_size, seed):
    loader, model, optimizer, criterion, scaler, device = make_components(
        module, args, batch_size, seed
    )
    images, masks = next(iter(loader))
    images = images.to(
        device, non_blocking=True, memory_format=torch.channels_last
    )
    masks = masks.to(device, non_blocking=True)
    for _ in range(args.warmup_steps):
        train_step(model, optimizer, criterion, scaler, images, masks)
    torch.cuda.synchronize(device)
    torch.cuda.reset_peak_memory_stats(device)

    losses = []
    started = time.perf_counter()
    for _ in range(args.measure_steps):
        losses.append(train_step(
            model, optimizer, criterion, scaler, images, masks
        ))
    torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - started
    images_seen = batch_size * args.measure_steps
    result = {
        "mode": "compute_only",
        "batch_size": batch_size,
        "images": images_seen,
        "elapsed_sec": elapsed,
        "images_per_sec": images_seen / elapsed,
        "compute_ms_per_image": 1000.0 * elapsed / images_seen,
        "peak_cuda_gib": torch.cuda.max_memory_allocated(device) / 1024**3,
        "loss_first": losses[0],
        "loss_last": losses[-1],
    }
    del loader, model, optimizer, criterion, scaler, images, masks
    torch.cuda.empty_cache()
    return result


def summarize(rows):
    summary = []
    keys = (
        "images_per_sec", "data_wait_sec_per_step",
        "compute_ms_per_image", "peak_cuda_gib",
    )
    for mode in ("end_to_end", "compute_only"):
        for batch_size in sorted({row["batch_size"] for row in rows}):
            selected = [
                row for row in rows
                if row["mode"] == mode and row["batch_size"] == batch_size
            ]
            record = {"mode": mode, "batch_size": batch_size}
            for key in keys:
                values = [row[key] for row in selected if key in row]
                if values:
                    record[f"median_{key}"] = statistics.median(values)
            summary.append(record)
    return summary


def main():
    args = parse_args()
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is required")
    torch.backends.cudnn.benchmark = True
    module = load_training_module()
    batch_sizes = [int(value) for value in args.batch_sizes.split(",")]
    jobs = [
        (repeat, batch_size)
        for repeat in range(args.repeats)
        for batch_size in batch_sizes
    ]
    random.Random(args.seed).shuffle(jobs)
    rows = []
    print("job order:", jobs, flush=True)
    for index, (repeat, batch_size) in enumerate(jobs, 1):
        seed = args.seed + repeat * 1009 + batch_size
        print(
            f"[{index}/{len(jobs)}] repeat={repeat + 1} batch={batch_size}",
            flush=True,
        )
        for function in (benchmark_compute_only, benchmark_end_to_end):
            row = function(module, args, batch_size, seed)
            row["repeat"] = repeat + 1
            rows.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)
    payload = {
        "config": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        "runs": rows,
        "summary": summarize(rows),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8"
    )
    print("summary:")
    print(json.dumps(payload["summary"], indent=2, sort_keys=True))
    print("output:", args.output)


if __name__ == "__main__":
    main()
