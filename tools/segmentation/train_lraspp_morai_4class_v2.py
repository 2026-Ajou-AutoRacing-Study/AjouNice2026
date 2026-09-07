#!/usr/bin/env python3
"""
Fine-tune torchvision LRASPP-MobileNetV3-Large on MORAI 4-class masks.

Target classes
--------------
0 Background
1 Pedestrian
2 Vehicle
3 Obstacle

Designed for the user's OpenPCDet environment:
- Python 3.8
- PyTorch 1.10.x
- torchvision 0.11.x
- CUDA / RTX 2080 Ti

The frozen dataset and split manifests are read-only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import time
from pathlib import Path

import numpy as np
from PIL import Image

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, Sampler
from torchvision.models.segmentation import (
    LRASPP_MobileNet_V3_Large_Weights,
    lraspp_mobilenet_v3_large,
)
from torchvision.transforms import functional as TF
from torchvision.transforms.functional import InterpolationMode


NUM_CLASSES = 4
CLASS_NAMES = ["background", "pedestrian", "vehicle", "obstacle"]
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", required=True, type=Path)
    p.add_argument("--train-csv", required=True, type=Path)
    p.add_argument("--val-csv", required=True, type=Path)
    p.add_argument("--output-dir", required=True, type=Path)
    p.add_argument(
        "--resume",
        type=Path,
        default=None,
        help="Resume model/optimizer/scheduler and history from a checkpoint.",
    )
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--batch-size", type=int, default=4)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--short-side", type=int, default=384)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--weight-decay", type=float, default=1e-4)
    p.add_argument(
        "--class-weight-mode", choices=("none", "enet"), default="none",
        help="ENet inverse-log pixel-frequency weighting improves rare classes.",
    )
    p.add_argument(
        "--scheduler", choices=("none", "cosine"), default="none"
    )
    p.add_argument(
        "--warmup-epochs",
        type=int,
        default=0,
        help=(
            "Linear LR warm-up epochs before cosine annealing. "
            "Only used with --scheduler cosine."
        ),
    )
    p.add_argument(
        "--sampling-mode",
        choices=("natural", "class_clip_aware"),
        default="natural",
        help=(
            "class_clip_aware draws an exact class-source mixture while "
            "cycling scenes uniformly inside every source pool."
        ),
    )
    p.add_argument("--sample-ratio-vehicle", type=float, default=0.10)
    p.add_argument("--sample-ratio-pedestrian", type=float, default=0.15)
    p.add_argument("--sample-ratio-obstacle", type=float, default=0.30)
    p.add_argument("--sample-ratio-empty", type=float, default=0.45)
    p.add_argument(
        "--checkpoint-interval",
        type=int,
        default=0,
        help="Also retain checkpoint_epoch_N.pth every N epochs; 0 disables it.",
    )
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--amp", action="store_true")
    p.add_argument("--wandb-project", default="")
    p.add_argument("--wandb-entity", default="")
    p.add_argument("--wandb-run-name", default="")
    p.add_argument(
        "--wandb-run-id",
        default="",
        help="Existing W&B run id to resume, for example 9fdubfb6.",
    )
    p.add_argument("--wandb-mode", choices=("online", "offline", "disabled"),
                   default="online")
    p.add_argument("--max-train-batches", type=int, default=0,
                   help="0 = all batches; useful for a very fast smoke test.")
    p.add_argument("--max-val-batches", type=int, default=0,
                   help="0 = all batches.")
    return p.parse_args()


def seed_everything(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def read_manifest(path: Path):
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        raise RuntimeError(f"Empty manifest: {path}")
    return rows


def class_weights_from_manifest(rows, mode):
    if mode == "none":
        return None, None
    columns = [
        "background_pixels", "pedestrian_pixels", "vehicle_pixels",
        "obstacle_pixels",
    ]
    counts = np.zeros(NUM_CLASSES, dtype=np.float64)
    for row in rows:
        for index, column in enumerate(columns):
            if column not in row or row[column] == "":
                raise RuntimeError(
                    f"{mode} class weighting requires CSV column {column}"
                )
            counts[index] += float(row[column])
    frequencies = counts / counts.sum()
    # ENet weighting: bounded inverse-log frequency.  This avoids the extreme
    # gradients produced by raw inverse frequency on small obstacle pixels.
    weights = 1.0 / np.log(1.02 + frequencies)
    weights /= weights.mean()
    return weights.astype(np.float32), counts.astype(np.int64)


class MoraiSegDataset(Dataset):
    def __init__(self, dataset_root: Path, manifest_csv: Path, short_side: int):
        self.dataset_root = dataset_root.resolve()
        self.rows = read_manifest(manifest_csv.resolve())
        self.short_side = int(short_side)

    def __len__(self):
        return len(self.rows)

    def _resize_pair(self, image: Image.Image, mask: Image.Image):
        w, h = image.size
        if min(w, h) == self.short_side:
            return image, mask

        scale = self.short_side / float(min(w, h))
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))

        image = TF.resize(
            image,
            [new_h, new_w],
            interpolation=InterpolationMode.BILINEAR,
        )
        mask = TF.resize(
            mask,
            [new_h, new_w],
            interpolation=InterpolationMode.NEAREST,
        )
        return image, mask

    def __getitem__(self, idx):
        row = self.rows[idx]
        image_path = self.dataset_root / row["image"]
        mask_path = self.dataset_root / row["mask"]

        image = Image.open(image_path).convert("RGB")
        mask = Image.open(mask_path).convert("L")

        if image.size != mask.size:
            raise RuntimeError(
                f"Image/mask size mismatch:\n{image_path}\n{mask_path}"
            )

        image, mask = self._resize_pair(image, mask)

        image = TF.to_tensor(image)
        image = TF.normalize(image, IMAGENET_MEAN, IMAGENET_STD)

        mask = torch.from_numpy(np.array(mask, dtype=np.int64)).long()
        ids = torch.unique(mask)
        if torch.any(ids < 0) or torch.any(ids >= NUM_CLASSES):
            raise RuntimeError(
                f"Invalid class IDs {ids.tolist()} in {mask_path}"
            )

        return image, mask


class SceneBalancedClassSampler(Sampler):
    """Sample a fixed class mixture without letting large clips dominate.

    A row containing more than one foreground class belongs to each matching
    source pool.  The requested ratios describe the *source pool draws*, not
    mutually exclusive labels.  Inside a pool, scenes are visited in shuffled
    round-robin order, then one eligible camera frame from that scene is drawn.
    This keeps a large/pixel-heavy scene from dominating merely because it has
    more eligible rows.
    """

    POOL_COLUMNS = {
        "vehicle": "vehicle_pixels",
        "pedestrian": "pedestrian_pixels",
        "obstacle": "obstacle_pixels",
    }

    def __init__(self, rows, ratios, seed=42, num_samples=None):
        self.rows = rows
        self.ratios = dict(ratios)
        self.seed = int(seed)
        self.epoch = 0
        self.num_samples = int(num_samples or len(rows))
        if self.num_samples <= 0:
            raise ValueError("sampler num_samples must be positive")
        if any(value < 0 for value in self.ratios.values()):
            raise ValueError("sampling ratios must be non-negative")
        ratio_sum = sum(self.ratios.values())
        if not math.isclose(ratio_sum, 1.0, rel_tol=0.0, abs_tol=1.0e-8):
            raise ValueError(
                f"sampling ratios must sum to 1.0, got {ratio_sum:.12f}"
            )

        self.scene_pools = {}
        for pool_name in self.ratios:
            scene_to_indices = {}
            for index, row in enumerate(rows):
                if self._belongs(row, pool_name):
                    scene_to_indices.setdefault(row["scene_key"], []).append(index)
            if self.ratios[pool_name] > 0 and not scene_to_indices:
                raise ValueError(f"sampling pool is empty: {pool_name}")
            self.scene_pools[pool_name] = scene_to_indices

        self.draw_counts = self._apportion_counts()

    def _belongs(self, row, pool_name):
        if pool_name == "empty":
            return all(int(row[column]) == 0 for column in self.POOL_COLUMNS.values())
        return int(row[self.POOL_COLUMNS[pool_name]]) > 0

    def _apportion_counts(self):
        raw = {
            name: self.num_samples * ratio
            for name, ratio in self.ratios.items()
        }
        counts = {name: int(math.floor(value)) for name, value in raw.items()}
        remaining = self.num_samples - sum(counts.values())
        order = sorted(
            raw,
            key=lambda name: (raw[name] - counts[name], name),
            reverse=True,
        )
        for name in order[:remaining]:
            counts[name] += 1
        return counts

    def set_epoch(self, epoch):
        self.epoch = int(epoch)

    def __len__(self):
        return self.num_samples

    def __iter__(self):
        rng = random.Random(self.seed + 1000003 * self.epoch)
        sampled = []
        for pool_name, draw_count in self.draw_counts.items():
            scene_to_indices = self.scene_pools[pool_name]
            scenes = list(scene_to_indices)
            if not scenes:
                continue
            scene_order = []
            while len(scene_order) < draw_count:
                cycle = scenes.copy()
                rng.shuffle(cycle)
                scene_order.extend(cycle)
            for scene in scene_order[:draw_count]:
                sampled.append(rng.choice(scene_to_indices[scene]))
        rng.shuffle(sampled)
        if len(sampled) != self.num_samples:
            raise RuntimeError(
                f"sampler produced {len(sampled)} != {self.num_samples} indices"
            )
        return iter(sampled)

    def summary(self):
        return {
            name: {
                "ratio": self.ratios[name],
                "draws_per_epoch": self.draw_counts[name],
                "eligible_rows": sum(
                    len(indices) for indices in self.scene_pools[name].values()
                ),
                "eligible_scenes": len(self.scene_pools[name]),
            }
            for name in self.ratios
        }


def segmentation_collate(batch):
    """
    Preserve each camera's aspect ratio, then pad only inside the batch.

    Image padding value = 0 in normalized tensor space (approximately mean RGB).
    Mask padding value = 255 and is ignored by CrossEntropyLoss/metrics.

    This keeps training preprocessing consistent with the online PointPainting
    node, which also resizes by short side while preserving aspect ratio.
    """
    images, masks = zip(*batch)

    max_h = max(int(x.shape[-2]) for x in images)
    max_w = max(int(x.shape[-1]) for x in images)

    padded_images = []
    padded_masks = []

    for image, mask in zip(images, masks):
        h, w = int(image.shape[-2]), int(image.shape[-1])
        pad_right = max_w - w
        pad_bottom = max_h - h

        # torchvision F.pad uses [left, top, right, bottom].
        image = TF.pad(
            image,
            [0, 0, pad_right, pad_bottom],
            fill=0.0,
        )
        mask = TF.pad(
            mask,
            [0, 0, pad_right, pad_bottom],
            fill=255,
        )

        padded_images.append(image)
        padded_masks.append(mask)

    return torch.stack(padded_images, 0), torch.stack(padded_masks, 0)


def build_model():
    # torchvision 0.11.x: load VOC-pretrained LRASPP first, then replace only
    # the two final class-projection convolutions. This keeps the pretrained
    # MobileNetV3 backbone and LR-ASPP feature layers.
    model = lraspp_mobilenet_v3_large(
        weights=LRASPP_MobileNet_V3_Large_Weights.DEFAULT
    )

    if not hasattr(model, "classifier"):
        raise RuntimeError("Unexpected LRASPP model: no classifier")

    head = model.classifier
    if not hasattr(head, "low_classifier") or not hasattr(head, "high_classifier"):
        raise RuntimeError(
            "Unexpected torchvision LRASPPHead layout; "
            "expected low_classifier/high_classifier."
        )

    low_old = head.low_classifier
    high_old = head.high_classifier

    head.low_classifier = nn.Conv2d(
        low_old.in_channels, NUM_CLASSES,
        kernel_size=low_old.kernel_size,
        stride=low_old.stride,
        padding=low_old.padding,
        bias=(low_old.bias is not None),
    )
    head.high_classifier = nn.Conv2d(
        high_old.in_channels, NUM_CLASSES,
        kernel_size=high_old.kernel_size,
        stride=high_old.stride,
        padding=high_old.padding,
        bias=(high_old.bias is not None),
    )

    return model


def update_confusion(conf, pred, target, num_classes=NUM_CLASSES):
    pred = pred.view(-1)
    target = target.view(-1)
    valid = (target >= 0) & (target < num_classes)
    encoded = num_classes * target[valid] + pred[valid]
    bins = torch.bincount(encoded, minlength=num_classes ** 2)
    conf += bins.reshape(num_classes, num_classes).cpu()
    return conf


def metrics_from_confusion(conf):
    conf = conf.double()
    tp = torch.diag(conf)
    fp = conf.sum(0) - tp
    fn = conf.sum(1) - tp
    denom = tp + fp + fn
    iou = torch.where(denom > 0, tp / denom, torch.nan)
    precision_denom = tp + fp
    recall_denom = tp + fn
    dice_denom = 2 * tp + fp + fn
    precision = torch.where(
        precision_denom > 0, tp / precision_denom, torch.nan
    )
    recall = torch.where(recall_denom > 0, tp / recall_denom, torch.nan)
    dice = torch.where(dice_denom > 0, 2 * tp / dice_denom, torch.nan)
    row_sum = conf.sum(1, keepdim=True)
    normalized_conf = torch.where(row_sum > 0, conf / row_sum, 0.0)

    total = conf.sum()
    pixel_acc = (tp.sum() / total).item() if total > 0 else float("nan")
    miou = torch.nanmean(iou).item()
    foreground_miou = torch.nanmean(iou[1:]).item()

    def class_values(values):
        return {
            CLASS_NAMES[i]: (
                float(values[i].item())
                if not torch.isnan(values[i]) else None
            )
            for i in range(NUM_CLASSES)
        }

    return {
        "pixel_accuracy": pixel_acc,
        "miou_4class": miou,
        "foreground_miou": foreground_miou,
        "class_iou": class_values(iou),
        "class_precision": class_values(precision),
        "class_recall": class_values(recall),
        "class_dice": class_values(dice),
        "confusion_matrix": conf.to(torch.int64).tolist(),
        "confusion_matrix_row_normalized": normalized_conf.tolist(),
    }


def train_one_epoch(model, loader, optimizer, scaler, criterion, device,
                    amp, max_batches):
    model.train()
    total_loss = 0.0
    seen = 0
    processed_images = 0
    torch.cuda.synchronize(device)
    started_at = time.perf_counter()

    for batch_idx, (images, masks) in enumerate(loader):
        if max_batches and batch_idx >= max_batches:
            break

        images = images.to(
            device, non_blocking=True,
            memory_format=torch.channels_last
        )
        masks = masks.to(device, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)

        with torch.amp.autocast("cuda", enabled=amp):
            output = model(images)["out"]
            if output.shape[1] != NUM_CLASSES:
                raise RuntimeError(
                    f"Expected 4 output channels, got {output.shape}"
                )
            loss = criterion(output, masks)

        if not torch.isfinite(loss):
            raise RuntimeError(f"Non-finite loss: {loss.item()}")

        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()

        total_loss += float(loss.item())
        seen += 1
        processed_images += int(images.shape[0])

        if batch_idx == 0:
            print(
                "  first train batch:",
                "image", tuple(images.shape),
                "mask", tuple(masks.shape),
                "output", tuple(output.shape),
                "loss", f"{loss.item():.6f}",
            )

    torch.cuda.synchronize(device)
    elapsed_sec = time.perf_counter() - started_at
    print(
        "  train performance:",
        f"batches={seen}",
        f"images={processed_images}",
        f"elapsed={elapsed_sec:.3f}s",
        f"images_per_sec={processed_images / max(elapsed_sec, 1e-9):.3f}",
    )
    return total_loss / max(seen, 1), {
        "elapsed_sec": elapsed_sec,
        "images": processed_images,
        "images_per_sec": processed_images / max(elapsed_sec, 1e-9),
    }


@torch.no_grad()
def validate(model, loader, criterion, device, amp, max_batches):
    model.eval()
    total_loss = 0.0
    seen = 0
    conf = torch.zeros((NUM_CLASSES, NUM_CLASSES), dtype=torch.int64)

    for batch_idx, (images, masks) in enumerate(loader):
        if max_batches and batch_idx >= max_batches:
            break

        images = images.to(
            device, non_blocking=True,
            memory_format=torch.channels_last
        )
        masks = masks.to(device, non_blocking=True)

        with torch.amp.autocast("cuda", enabled=amp):
            output = model(images)["out"]
            loss = criterion(output, masks)

        pred = output.argmax(1)
        conf = update_confusion(conf, pred.cpu(), masks.cpu())

        total_loss += float(loss.item())
        seen += 1

        if batch_idx == 0:
            print(
                "  first val batch:",
                "image", tuple(images.shape),
                "mask", tuple(masks.shape),
                "output", tuple(output.shape),
                "loss", f"{loss.item():.6f}",
            )

    metrics = metrics_from_confusion(conf)
    metrics["loss"] = total_loss / max(seen, 1)
    return metrics


def save_checkpoint(path, model, optimizer, scheduler, scaler, epoch, metrics,
                    args, class_weights):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "epoch": epoch,
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "scheduler_state_dict": (
                scheduler.state_dict() if scheduler is not None else None
            ),
            "scaler_state_dict": scaler.state_dict(),
            "scheduler_step_timing": "post_epoch",
            "metrics": metrics,
            "num_classes": NUM_CLASSES,
            "class_names": CLASS_NAMES,
            "short_side": args.short_side,
            "architecture": "lraspp_mobilenet_v3_large",
            "class_weights": (
                class_weights.tolist() if class_weights is not None else None
            ),
        },
        path,
    )


def define_wandb_metrics(run):
    """Use the same explicit epoch axis as the PointPainting trainer.

    W&B's private ``_step`` is intentionally not used as the chart axis.  One
    committed record represents one complete Train N -> Val N cycle, so every
    train and validation scalar for an epoch stays aligned in the dashboard.
    """
    run.define_metric("epoch", hidden=True)
    run.define_metric(
        "train/loss", step_metric="epoch", summary="min", overwrite=True
    )
    run.define_metric(
        "train/lr", step_metric="epoch", summary="last", overwrite=True
    )
    run.define_metric(
        "val/loss", step_metric="epoch", summary="min", overwrite=True
    )
    run.define_metric(
        "val/pixel_accuracy",
        step_metric="epoch",
        summary="max",
        overwrite=True,
    )
    run.define_metric(
        "val/miou_4class",
        step_metric="epoch",
        summary="max",
        overwrite=True,
    )
    run.define_metric(
        "val/miou_foreground",
        step_metric="epoch",
        summary="max",
        overwrite=True,
    )
    run.define_metric(
        "train/images_per_sec",
        step_metric="epoch",
        summary="last",
        overwrite=True,
    )
    run.define_metric(
        "performance/peak_cuda_memory_gib",
        step_metric="epoch",
        summary="max",
        overwrite=True,
    )
    for name in CLASS_NAMES:
        for metric in ("iou", "precision", "recall", "dice"):
            run.define_metric(
                f"val/{metric}/{name}",
                step_metric="epoch",
                summary="max",
                overwrite=True,
            )


def wandb_epoch_payload(epoch, train_loss, learning_rate, train_performance,
                        peak_cuda_gib, val_metrics):
    """Flatten a completed Train N -> Val N cycle into one W&B record."""
    return {
        "epoch": int(epoch),
        "train/loss": float(train_loss),
        "train/lr": float(learning_rate),
        "train/images_per_sec": float(train_performance["images_per_sec"]),
        "performance/peak_cuda_memory_gib": float(peak_cuda_gib),
        "val/loss": float(val_metrics["loss"]),
        "val/pixel_accuracy": float(val_metrics["pixel_accuracy"]),
        "val/miou_4class": float(val_metrics["miou_4class"]),
        "val/miou_foreground": float(val_metrics["foreground_miou"]),
        **dict(
            item
            for metric, values in (
                ("iou", val_metrics["class_iou"]),
                ("precision", val_metrics["class_precision"]),
                ("recall", val_metrics["class_recall"]),
                ("dice", val_metrics["class_dice"]),
            )
            for name, value in values.items()
            for item in [(
                f"val/{metric}/{name}",
                float(value) if value is not None else float("nan"),
            )]
        ),
    }


def wandb_confusion_table(wandb, val_metrics):
    table = wandb.Table(columns=[
        "true_class", "predicted_class", "pixel_count", "row_fraction"
    ])
    counts = val_metrics["confusion_matrix"]
    fractions = val_metrics["confusion_matrix_row_normalized"]
    for true_index, true_name in enumerate(CLASS_NAMES):
        for pred_index, pred_name in enumerate(CLASS_NAMES):
            table.add_data(
                true_name,
                pred_name,
                int(counts[true_index][pred_index]),
                float(fractions[true_index][pred_index]),
            )
    return table


def main():
    args = parse_args()
    if args.warmup_epochs < 0 or args.warmup_epochs >= args.epochs:
        raise SystemExit(
            "--warmup-epochs must satisfy 0 <= warmup_epochs < epochs"
        )
    seed_everything(args.seed)

    if not torch.cuda.is_available():
        raise SystemExit("CUDA is required for this training run.")

    device = torch.device("cuda")
    torch.backends.cudnn.benchmark = True

    print("torch:", torch.__version__)
    import torchvision
    print("torchvision:", torchvision.__version__)
    print("gpu:", torch.cuda.get_device_name(0))
    print("target classes:", dict(enumerate(CLASS_NAMES)))

    train_ds = MoraiSegDataset(
        args.dataset_root, args.train_csv, args.short_side
    )
    val_ds = MoraiSegDataset(
        args.dataset_root, args.val_csv, args.short_side
    )

    print("train images:", len(train_ds))
    print("val images:", len(val_ds))

    class_weights, class_pixel_counts = class_weights_from_manifest(
        train_ds.rows, args.class_weight_mode
    )
    if class_pixel_counts is not None:
        print("train class pixels:", {
            name: int(class_pixel_counts[index])
            for index, name in enumerate(CLASS_NAMES)
        })
        print("loss class weights:", {
            name: float(class_weights[index])
            for index, name in enumerate(CLASS_NAMES)
        })

    train_sampler = None
    if args.sampling_mode == "class_clip_aware":
        train_sampler = SceneBalancedClassSampler(
            train_ds.rows,
            ratios={
                "vehicle": args.sample_ratio_vehicle,
                "pedestrian": args.sample_ratio_pedestrian,
                "obstacle": args.sample_ratio_obstacle,
                "empty": args.sample_ratio_empty,
            },
            seed=args.seed,
            num_samples=len(train_ds),
        )
        print("train sampler:", json.dumps(
            train_sampler.summary(), indent=2, ensure_ascii=False
        ))

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=(train_sampler is None),
        sampler=train_sampler,
        num_workers=args.workers,
        pin_memory=True,
        drop_last=False,
        collate_fn=segmentation_collate,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=True,
        drop_last=False,
        collate_fn=segmentation_collate,
    )

    model = build_model().to(device).to(memory_format=torch.channels_last)

    # Smoke-test baseline: unweighted CE. Class rebalancing should only be
    # introduced after confirming the full pipeline and inspecting per-class IoU.
    criterion = nn.CrossEntropyLoss(
        weight=(
            torch.as_tensor(class_weights, device=device)
            if class_weights is not None else None
        ),
        ignore_index=255,
    )

    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.lr,
        weight_decay=args.weight_decay,
    )
    scheduler = None
    if args.scheduler == "cosine":
        if args.warmup_epochs:
            cosine_steps = max(1, args.epochs - args.warmup_epochs)

            def warmup_cosine_factor(step):
                if step <= args.warmup_epochs:
                    return 0.1 + 0.9 * step / args.warmup_epochs
                progress = min(
                    (step - args.warmup_epochs) / cosine_steps,
                    1.0,
                )
                return 0.01 + 0.99 * 0.5 * (
                    1.0 + math.cos(math.pi * progress)
                )

            scheduler = torch.optim.lr_scheduler.LambdaLR(
                optimizer, lr_lambda=warmup_cosine_factor
            )
        else:
            scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                optimizer,
                T_max=args.epochs,
                eta_min=args.lr * 0.01,
            )

    scaler = torch.amp.GradScaler("cuda", enabled=args.amp)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    history_path = args.output_dir / "history.json"
    history = []
    start_epoch = 1
    best_4class_miou = -1.0
    best_foreground_miou = -1.0

    if args.resume is not None:
        if not args.resume.is_file():
            raise SystemExit(f"Resume checkpoint does not exist: {args.resume}")
        try:
            checkpoint = torch.load(
                args.resume, map_location=device, weights_only=False
            )
        except TypeError:
            checkpoint = torch.load(args.resume, map_location=device)
        model.load_state_dict(checkpoint["model_state_dict"])
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        if scheduler is not None and checkpoint.get("scheduler_state_dict"):
            scheduler.load_state_dict(checkpoint["scheduler_state_dict"])
            if checkpoint.get("scheduler_step_timing") != "post_epoch":
                # Checkpoints produced before resume support were saved before
                # scheduler.step(). Advance once to reconstruct the LR that
                # would have been used by the following epoch.
                scheduler.step()
        if checkpoint.get("scaler_state_dict"):
            scaler.load_state_dict(checkpoint["scaler_state_dict"])

        completed_epoch = int(checkpoint["epoch"])
        start_epoch = completed_epoch + 1
        if history_path.is_file():
            history = [
                row for row in json.loads(history_path.read_text(
                    encoding="utf-8"
                ))
                if int(row["epoch"]) <= completed_epoch
            ]
        if history:
            best_4class_miou = max(
                float(row["miou_4class"]) for row in history
            )
            best_foreground_miou = max(
                float(row["foreground_miou"]) for row in history
            )
        else:
            metrics = checkpoint.get("metrics", {})
            best_4class_miou = float(metrics.get("miou_4class", -1.0))
            best_foreground_miou = float(
                metrics.get("foreground_miou", -1.0)
            )
        print(
            "resumed:", args.resume,
            f"completed_epoch={completed_epoch}",
            f"next_epoch={start_epoch}",
            f"best_4class_miou={best_4class_miou:.6f}",
            f"best_foreground_miou={best_foreground_miou:.6f}",
        )

    if start_epoch > args.epochs:
        raise SystemExit(
            f"Checkpoint already completed epoch {start_epoch - 1}, "
            f"but --epochs is {args.epochs}."
        )

    config = vars(args).copy()
    config = {k: str(v) if isinstance(v, Path) else v for k, v in config.items()}
    (args.output_dir / "run_config.json").write_text(
        json.dumps(config, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    wandb_run = None
    if args.wandb_project and args.wandb_mode != "disabled":
        import wandb
        wandb_run = wandb.init(
            project=args.wandb_project,
            entity=args.wandb_entity or None,
            name=args.wandb_run_name or args.output_dir.name,
            id=args.wandb_run_id or None,
            resume="must" if args.wandb_run_id else None,
            mode=args.wandb_mode,
            config={
                **config,
                "class_names": CLASS_NAMES,
                "class_weights": (
                    class_weights.tolist() if class_weights is not None else None
                ),
                "class_pixel_counts": (
                    class_pixel_counts.tolist()
                    if class_pixel_counts is not None else None
                ),
            },
        )
        define_wandb_metrics(wandb_run)
        print("wandb run:", wandb_run.url)

    for epoch in range(start_epoch, args.epochs + 1):
        print(f"\nEpoch {epoch}/{args.epochs}")
        epoch_learning_rate = optimizer.param_groups[0]["lr"]
        print(f"  learning_rate: {epoch_learning_rate:.6e}")

        torch.cuda.reset_peak_memory_stats(device)

        if train_sampler is not None:
            train_sampler.set_epoch(epoch)

        train_loss, train_performance = train_one_epoch(
            model, train_loader, optimizer, scaler,
            criterion, device, args.amp, args.max_train_batches
        )
        val_metrics = validate(
            model, val_loader, criterion,
            device, args.amp, args.max_val_batches
        )
        peak_cuda_gib = torch.cuda.max_memory_allocated(device) / (1024 ** 3)
        print("  peak CUDA memory:", f"{peak_cuda_gib:.3f} GiB")

        row = {
            "epoch": epoch,
            "train_loss": train_loss,
            "train_performance": train_performance,
            "peak_cuda_memory_gib": peak_cuda_gib,
            **val_metrics,
        }
        history.append(row)

        print(f"  train_loss: {train_loss:.6f}")
        print(f"  val_loss  : {val_metrics['loss']:.6f}")
        print(f"  val_mIoU  : {val_metrics['miou_4class']:.6f}")
        print(f"  fg_mIoU   : {val_metrics['foreground_miou']:.6f}")
        print(f"  pixel_acc : {val_metrics['pixel_accuracy']:.6f}")
        for name in CLASS_NAMES:
            print(
                f"  {name:10s}",
                f"IoU={val_metrics['class_iou'][name]}",
                f"P={val_metrics['class_precision'][name]}",
                f"R={val_metrics['class_recall'][name]}",
                f"Dice={val_metrics['class_dice'][name]}",
            )

        if wandb_run is not None:
            payload = wandb_epoch_payload(
                epoch,
                train_loss,
                epoch_learning_rate,
                train_performance,
                peak_cuda_gib,
                val_metrics,
            )
            payload["val/confusion_matrix"] = wandb_confusion_table(
                wandb, val_metrics
            )
            wandb_run.log(payload, commit=True)

        if scheduler is not None:
            scheduler.step()

        save_checkpoint(
            args.output_dir / "last.pth",
            model, optimizer, scheduler, scaler, epoch, val_metrics,
            args, class_weights,
        )
        if (
            args.checkpoint_interval > 0
            and epoch % args.checkpoint_interval == 0
        ):
            save_checkpoint(
                args.output_dir / f"checkpoint_epoch_{epoch}.pth",
                model, optimizer, scheduler, scaler, epoch, val_metrics,
                args, class_weights,
            )

        if val_metrics["miou_4class"] > best_4class_miou:
            best_4class_miou = val_metrics["miou_4class"]
            save_checkpoint(
                args.output_dir / "best_4class_miou.pth",
                model, optimizer, scheduler, scaler, epoch, val_metrics,
                args, class_weights,
            )
            print("  saved new best_4class_miou.pth")
            if wandb_run is not None:
                wandb_run.summary["best/4class_epoch"] = epoch
                wandb_run.summary["best/val_miou_4class"] = (
                    best_4class_miou
                )

        if val_metrics["foreground_miou"] > best_foreground_miou:
            best_foreground_miou = val_metrics["foreground_miou"]
            save_checkpoint(
                args.output_dir / "best_foreground_miou.pth",
                model, optimizer, scheduler, scaler, epoch, val_metrics,
                args, class_weights,
            )
            print("  saved new best_foreground_miou.pth")
            if wandb_run is not None:
                wandb_run.summary["best/epoch"] = epoch
                wandb_run.summary["best/foreground_epoch"] = epoch
                wandb_run.summary["best/val_miou_foreground"] = (
                    best_foreground_miou
                )

        (args.output_dir / "history.json").write_text(
            json.dumps(history, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    print("\nTraining finished.")
    print("best 4-class mIoU:", best_4class_miou)
    print("best foreground mIoU:", best_foreground_miou)
    print("output:", args.output_dir)
    if wandb_run is not None:
        wandb_run.finish()


if __name__ == "__main__":
    main()
