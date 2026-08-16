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
import random
from pathlib import Path

import numpy as np
from PIL import Image

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset
from torchvision.models.segmentation import lraspp_mobilenet_v3_large
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
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--batch-size", type=int, default=4)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--short-side", type=int, default=384)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--weight-decay", type=float, default=1e-4)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--amp", action="store_true")
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
    model = lraspp_mobilenet_v3_large(pretrained=True)

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

    total = conf.sum()
    pixel_acc = (tp.sum() / total).item() if total > 0 else float("nan")
    miou = torch.nanmean(iou).item()

    return {
        "pixel_accuracy": pixel_acc,
        "miou_4class": miou,
        "class_iou": {
            CLASS_NAMES[i]: (
                float(iou[i].item()) if not torch.isnan(iou[i]) else None
            )
            for i in range(NUM_CLASSES)
        },
    }


def train_one_epoch(model, loader, optimizer, scaler, criterion, device,
                    amp, max_batches):
    model.train()
    total_loss = 0.0
    seen = 0

    for batch_idx, (images, masks) in enumerate(loader):
        if max_batches and batch_idx >= max_batches:
            break

        images = images.to(
            device, non_blocking=True,
            memory_format=torch.channels_last
        )
        masks = masks.to(device, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)

        with torch.cuda.amp.autocast(enabled=amp):
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

        if batch_idx == 0:
            print(
                "  first train batch:",
                "image", tuple(images.shape),
                "mask", tuple(masks.shape),
                "output", tuple(output.shape),
                "loss", f"{loss.item():.6f}",
            )

    return total_loss / max(seen, 1)


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

        with torch.cuda.amp.autocast(enabled=amp):
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


def save_checkpoint(path, model, optimizer, epoch, metrics, args):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "epoch": epoch,
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "metrics": metrics,
            "num_classes": NUM_CLASSES,
            "class_names": CLASS_NAMES,
            "short_side": args.short_side,
            "architecture": "lraspp_mobilenet_v3_large",
        },
        path,
    )


def main():
    args = parse_args()
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

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
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
    criterion = nn.CrossEntropyLoss(ignore_index=255)

    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.lr,
        weight_decay=args.weight_decay,
    )

    scaler = torch.cuda.amp.GradScaler(enabled=args.amp)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = vars(args).copy()
    config = {k: str(v) if isinstance(v, Path) else v for k, v in config.items()}
    (args.output_dir / "run_config.json").write_text(
        json.dumps(config, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    best_miou = -1.0
    history = []

    for epoch in range(1, args.epochs + 1):
        print(f"\nEpoch {epoch}/{args.epochs}")

        train_loss = train_one_epoch(
            model, train_loader, optimizer, scaler,
            criterion, device, args.amp, args.max_train_batches
        )
        val_metrics = validate(
            model, val_loader, criterion,
            device, args.amp, args.max_val_batches
        )

        row = {
            "epoch": epoch,
            "train_loss": train_loss,
            **val_metrics,
        }
        history.append(row)

        print(f"  train_loss: {train_loss:.6f}")
        print(f"  val_loss  : {val_metrics['loss']:.6f}")
        print(f"  val_mIoU  : {val_metrics['miou_4class']:.6f}")
        print(f"  pixel_acc : {val_metrics['pixel_accuracy']:.6f}")
        for name, value in val_metrics["class_iou"].items():
            print(f"  IoU {name:10s}: {value}")

        save_checkpoint(
            args.output_dir / "last.pth",
            model, optimizer, epoch, val_metrics, args
        )

        if val_metrics["miou_4class"] > best_miou:
            best_miou = val_metrics["miou_4class"]
            save_checkpoint(
                args.output_dir / "best_miou.pth",
                model, optimizer, epoch, val_metrics, args
            )
            print("  saved new best_miou.pth")

        (args.output_dir / "history.json").write_text(
            json.dumps(history, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    print("\nTraining finished.")
    print("best val mIoU:", best_miou)
    print("output:", args.output_dir)


if __name__ == "__main__":
    main()
