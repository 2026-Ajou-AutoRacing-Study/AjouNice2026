#!/usr/bin/env python3
"""Run one native Fast-BEV MORAI train step on the configured CUDA GPU."""

import argparse
from pathlib import Path

import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import build_optimizer
from mmcv.utils import import_modules_from_strings
from mmdet3d.datasets import build_dataloader, build_dataset
from mmdet3d.models import build_model


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "fastbev_plugin/configs/fastbev_m0_morai_4cam_s384x704.py"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--no-backward", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("A CUDA GPU is required for the native train smoke")

    cfg = Config.fromfile(args.config)
    import_modules_from_strings(**cfg.custom_imports)
    dataset = build_dataset(cfg.data.train)
    loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=0,
        num_gpus=1,
        dist=False,
        shuffle=False,
    )
    batch = next(iter(loader))

    model = build_model(
        cfg.model,
        train_cfg=cfg.get("train_cfg"),
        test_cfg=cfg.get("test_cfg"),
    )
    model.init_weights()
    optimizer = build_optimizer(model, cfg.optimizer)
    model = MMDataParallel(model.cuda(), device_ids=[torch.cuda.current_device()])
    model.train()

    torch.cuda.reset_peak_memory_stats()
    losses = model(return_loss=True, **batch)
    reduced = {
        key: sum(value) if isinstance(value, (list, tuple)) else value
        for key, value in losses.items()
    }
    total_loss = sum(reduced.values())
    if not bool(torch.isfinite(total_loss).all()):
        raise RuntimeError(f"Non-finite loss: {reduced}")

    if not args.no_backward:
        optimizer.zero_grad()
        total_loss.backward()
        optimizer.step()
    torch.cuda.synchronize()

    print("dataset length:", len(dataset))
    print("camera count:", cfg.model.num_cameras)
    print("optimizer:", type(optimizer).__name__)
    for key, value in reduced.items():
        print(f"{key}: {float(value.detach().cpu()):.6f}")
    print("total loss:", float(total_loss.detach().cpu()))
    print(
        "peak CUDA MiB:",
        round(torch.cuda.max_memory_allocated() / 1024**2, 1),
    )
    print("backward + optimizer step:", not args.no_backward)
    print("Fast-BEV native MORAI train smoke: OK")


if __name__ == "__main__":
    main()
