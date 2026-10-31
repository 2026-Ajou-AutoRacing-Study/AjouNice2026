#!/usr/bin/env python3
"""Research-only full ResNet18 fine-tuning of four independent lamp bits.

No downloads, ROS, simulator calls, production edits, or automatic promotion.
Manual QC acceptance is required by default. Explicit weak-native-label mode
permits unreviewed records, never explicitly QC-rejected records. Unknown labels
are excluded by the strict four-bit dataset contract. Val alone chooses the best
epoch; the locked test split is evaluated once, after loading that checkpoint.
Explicitly manually reviewed no-lamp background negatives are supported separately and
retain native_gt_verified:false; unknown/unreadable signals are not negatives.
Manual review may be an assistant review; it does not certify independent human GT.
Signal-head training spatial augmentation only expands the entire approved QC
ROI. Manual no-lamp backgrounds keep their exact approved ROI, so neighboring
signals cannot enter through context expansion. Both permit photometric changes.
Torch/Pillow/NumPy/torchvision are lazy imports for actual training only.
"""

import argparse
import json
import math
import os
from pathlib import Path
import random

from train_traffic_light_crop_probe import (
    BASELINE_PATH, CLASSES, HEIGHT, WIDTH, _baseline, annotation_source, evaluation_metrics, hsv_bits,
    letterbox_geometry, load_dataset, metric_summary, sha256, train_pos_weight, validate_crop,
)

DEFAULT_CONFIGURATION = {"epochs": 30, "batch_size": 16, "device": "cpu", "seed": 0,
                         "warmup_epochs": 2, "backbone_lr": 1e-4, "head_lr": 1e-3,
                         "weight_decay": 1e-4, "dropout": 0., "blur_probability": 0.}


def quality_gate(records, allow_weak_native_labels=False):
    if type(allow_weak_native_labels) is not bool:
        raise ValueError("Weak-label permission must be an explicit boolean")
    if not records:
        raise ValueError("Dataset must not be empty")
    weak, native_count, background_count = 0, 0, 0
    for row in records:
        source = annotation_source(row, allow_manual_background=True)
        native_count += source == "native_verified"
        background_count += source == "manual_background_no_lamps"
        accepted = row.get("manual_qc_accepted")
        if accepted is False:
            raise ValueError("Explicit manual QC rejection is never allowed")
        if accepted is not True:
            if not allow_weak_native_labels:
                raise ValueError("Every record requires manual_qc_accepted:true by default")
            if accepted is not None:
                raise ValueError("manual_qc_accepted must be true, false, or absent/null")
            weak += 1
    return {"manual_qc_accepted_count": len(records) - weak, "unreviewed_record_count": weak,
            "native_verified_record_count": native_count, "manual_background_record_count": background_count,
            "allow_weak_native_labels": bool(allow_weak_native_labels),
            "mode": "weak_native_label_research_only" if weak else "manual_QC_accepted_research_only",
            "native_state_is_not_independent_visibility_certification": True}


def lr_multiplier(epoch, epochs=30, warmup_epochs=2, minimum=.01):
    if (type(epoch) is not int or type(epochs) is not int or type(warmup_epochs) is not int
            or epochs < 1 or not 0 <= epoch < epochs or not 0 <= warmup_epochs <= epochs
            or not math.isfinite(minimum) or not 0 < minimum <= 1):
        raise ValueError("Invalid warmup/cosine schedule parameters")
    if epoch < warmup_epochs:
        return (epoch + 1) / warmup_epochs
    progress = (epoch - warmup_epochs) / max(1, epochs - warmup_epochs - 1)
    return minimum + (1 - minimum) * .5 * (1 + math.cos(math.pi * progress))


def validation_is_better(macro_f1, bce, best=None):
    if not math.isfinite(macro_f1) or not 0 <= macro_f1 <= 1 or not math.isfinite(bce) or bce < 0:
        raise ValueError("Validation selection metrics must be finite")
    return (best is None or macro_f1 > best["macro_f1"]
            or (macro_f1 == best["macro_f1"] and bce < best["bce"]))


def training_augmentation_contract(blur_probability=0.):
    """One exact contract shared by provenance, checkpoints and final reports."""
    return {"spatial_mode": "signal_expansion_only_manual_background_fixed",
            "signal_spatial_mode": "expansion_only_asymmetric_context_padding",
            "signal_annotation_rule": "all_accepted_records_except_manual_background_no_lamps",
            "manual_background_annotation_kind": "manual_background_no_lamps",
            "manual_background_spatial_mode": "fixed_approved_QC_ROI_no_expansion",
            "photometric_scope": "both_signal_and_manual_background_training_records",
            "reference": "entire_approved_QC_integer_ROI_always_contained",
            "padding_side_order": ["left", "top", "right", "bottom"],
            "independent_per_side_fraction_range": [0., .1],
            "padding_fraction_range_scope": "signal_records_only_manual_background_uses_zero_padding",
            "fraction_reference": "left_right_use_original_width_top_bottom_use_original_height",
            "pixel_rounding": "floor_nonnegative_padding_before_image_bounds_clipping",
            "image_bounds_clipping": True, "approved_ROI_can_be_cut": False,
            "policy_revision_reason": "pre_fit_QC_preserve_signal_edges_and_exclude_neighbor_lamps_from_background",
            "brightness_range": [.9, 1.1], "contrast_range": [.9, 1.1],
            "blur_probability": blur_probability, "blur_radius": .4,
            "flip": False, "hue_shift": False}


def augmentation_parameters(row, rng, *, training=False, blur_probability=0.):
    """Preserve signal ROIs; never spatially expand manual no-lamp backgrounds."""
    if not math.isfinite(blur_probability) or not 0 <= blur_probability <= .05:
        raise ValueError("Optional Gaussian blur probability must be in [0,.05]")
    width, height = row["image_width"], row["image_height"]
    box = list(validate_crop(row["crop_integer_xyxy"], width, height))
    if not training:
        return {"crop_xyxy": box, "brightness": 1., "contrast": 1., "gaussian_blur": False,
                "horizontal_flip": False, "vertical_flip": False, "hue_shift": 0.,
                "spatial_mode": "fixed_approved_QC_ROI", "padding_fraction_ltrb": [0., 0., 0., 0.],
                "padding_pixels_ltrb": [0, 0, 0, 0], "approved_ROI_contained": True}
    if row["split"] != "train":
        raise ValueError("Training augmentation is forbidden for validation/test")
    x1, y1, x2, y2 = box
    crop_width, crop_height = x2 - x1, y2 - y1
    if row.get("annotation_kind") == "manual_background_no_lamps":
        fractions = [0., 0., 0., 0.]
        left, top, right, bottom = box
        spatial_mode = "fixed_approved_manual_background_ROI"
    else:
        fractions = [rng.uniform(0., .1) for _ in range(4)]
        padding = [math.floor(fraction * dimension) for fraction, dimension in
                   zip(fractions, (crop_width, crop_height, crop_width, crop_height))]
        left, top = max(0, x1 - padding[0]), max(0, y1 - padding[1])
        right, bottom = min(width, x2 + padding[2]), min(height, y2 + padding[3])
        spatial_mode = "expansion_only_asymmetric_context_padding"
    return {"crop_xyxy": [left, top, right, bottom],
            "brightness": rng.uniform(.9, 1.1), "contrast": rng.uniform(.9, 1.1),
            "gaussian_blur": blur_probability > 0 and rng.random() < blur_probability,
            "horizontal_flip": False, "vertical_flip": False, "hue_shift": 0.,
            "spatial_mode": spatial_mode, "padding_fraction_ltrb": fractions,
            "padding_pixels_ltrb": [x1 - left, y1 - top, right - x2, bottom - y2],
            "approved_ROI_contained": left <= x1 < x2 <= right and top <= y1 < y2 <= bottom}


def selection_split_rows(records):
    """The training-loop interface deliberately returns no test rows."""
    return ([row for row in records if row["split"] == "train"],
            [row for row in records if row["split"] == "val"])


def freeze_batchnorm_running_stats(model, torch_module):
    for module in model.modules():
        if isinstance(module, torch_module.nn.modules.batchnorm._BatchNorm):
            module.eval()  # Affine parameters retain requires_grad=True.


def validate_config(*, epochs=30, batch_size=16, warmup_epochs=2, backbone_lr=1e-4,
                    head_lr=1e-3, weight_decay=1e-4, dropout=0., blur_probability=0., device="cpu", seed=0):
    if (type(epochs) is not int or epochs < 1 or type(batch_size) is not int or batch_size < 1
            or type(warmup_epochs) is not int or not 0 <= warmup_epochs <= epochs or type(seed) is not int or seed < 0):
        raise ValueError("Invalid epoch/batch/warmup/seed configuration")
    if any(not math.isfinite(value) or value <= 0 for value in (backbone_lr, head_lr)):
        raise ValueError("Learning rates must be finite and positive")
    if not math.isfinite(weight_decay) or weight_decay < 0 or dropout not in (0., .1):
        raise ValueError("Weight decay must be nonnegative; dropout is 0 or .1")
    if not math.isfinite(blur_probability) or not 0 <= blur_probability <= .05:
        raise ValueError("Blur probability must be in [0,.05]")
    if device != "cpu" and not (isinstance(device, str) and device.startswith("cuda:") and device[5:].isdigit()):
        raise ValueError("Device must be cpu or an explicit cuda:N; no implicit fallback")


def _fixed_crop(row):
    from PIL import Image
    path = Path(row["resolved_image_path"])
    if sha256(path) != row["image_sha256"]:
        raise ValueError("Original JPEG changed after dataset validation: {}".format(row["image_path"]))
    with Image.open(path) as source:
        return source.convert("RGB").crop(tuple(row["crop_integer_xyxy"]))


def _input_tensor(row, torch, rng, *, training=False, blur_probability=0.):
    import numpy as np
    from PIL import Image, ImageEnhance, ImageFilter
    path = Path(row["resolved_image_path"])
    if sha256(path) != row["image_sha256"]:
        raise ValueError("Original JPEG changed after dataset validation")
    augment = augmentation_parameters(row, rng, training=training, blur_probability=blur_probability)
    with Image.open(path) as source:
        crop = source.convert("RGB").crop(tuple(augment["crop_xyxy"]))
    if training:
        crop = ImageEnhance.Contrast(crop).enhance(augment["contrast"])
        crop = ImageEnhance.Brightness(crop).enhance(augment["brightness"])
        if augment["gaussian_blur"]:
            crop = crop.filter(ImageFilter.GaussianBlur(radius=.4))
    resized, offset = letterbox_geometry(*crop.size)
    canvas = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    resampling = getattr(Image, "Resampling", Image)
    canvas.paste(crop.resize(resized, resampling.BILINEAR), offset)
    tensor = torch.from_numpy(np.asarray(canvas).copy()).permute(2, 0, 1).float() / 255.
    mean = torch.tensor([.485, .456, .406], dtype=torch.float32)[:, None, None]
    std = torch.tensor([.229, .224, .225], dtype=torch.float32)[:, None, None]
    return (tensor - mean) / std


def _evaluate_rows(model, rows, torch, device, batch_size):
    """Fixed crops only. Caller decides when the single test pass is allowed."""
    model.eval()
    probabilities, loss_sum = [], 0.
    with torch.inference_mode():
        for start in range(0, len(rows), batch_size):
            batch = rows[start:start + batch_size]
            inputs = torch.stack([_input_tensor(row, torch, None) for row in batch]).to(device)
            target = torch.tensor([row["bits"] for row in batch], dtype=torch.float32, device=device)
            logits = model(inputs)
            if not torch.isfinite(logits).all():
                raise ValueError("Non-finite evaluation logits")
            loss_sum += float(torch.nn.functional.binary_cross_entropy_with_logits(logits, target, reduction="sum").item())
            probabilities.extend(torch.sigmoid(logits).cpu().tolist())
    predicted = [[int(value >= .5) for value in row] for row in probabilities]
    return {"probabilities": probabilities, "predictions": predicted,
            "bce": loss_sum / (len(rows) * len(CLASSES)),
            "metrics": metric_summary([row["bits"] for row in rows], predicted)}


def train(records, backbone_weights, output, provenance, quality, *, epochs=30, batch_size=16,
          device="cpu", seed=0, warmup_epochs=2, backbone_lr=1e-4, head_lr=1e-3,
          weight_decay=1e-4, dropout=0., blur_probability=0.):
    """Actual full-backbone training. Never called by pure regression tests."""
    validate_config(epochs=epochs, batch_size=batch_size, device=device, seed=seed,
                    warmup_epochs=warmup_epochs, backbone_lr=backbone_lr, head_lr=head_lr,
                    weight_decay=weight_decay, dropout=dropout, blur_probability=blur_probability)
    os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
    import numpy as np
    import torch
    import torchvision
    from torchvision.models import resnet18
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(4)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    if device.startswith("cuda:"):
        if not torch.cuda.is_available():
            raise RuntimeError("Requested CUDA is unavailable; no CPU fallback")
        torch.cuda.manual_seed_all(seed)
    model = resnet18(weights=None)  # Explicit local state only; never download.
    state = torch.load(str(backbone_weights), map_location="cpu", weights_only=True)
    model.load_state_dict(state, strict=True)
    model.fc = (torch.nn.Linear(512, 4) if dropout == 0 else
                torch.nn.Sequential(torch.nn.Dropout(dropout), torch.nn.Linear(512, 4)))
    model.requires_grad_(True)
    model = model.float().to(device)
    backbone_parameters = [parameter for name, parameter in model.named_parameters() if not name.startswith("fc.")]
    optimizer = torch.optim.AdamW([
        {"params": backbone_parameters, "lr": backbone_lr},
        {"params": list(model.fc.parameters()), "lr": head_lr},
    ], weight_decay=weight_decay)
    positive_weights = train_pos_weight(records)
    criterion = torch.nn.BCEWithLogitsLoss(pos_weight=torch.tensor(positive_weights, dtype=torch.float32, device=device))
    training_rows, validation_rows = selection_split_rows(records)
    best = None
    output = Path(output)
    with (output / "epochs.jsonl").open("x", encoding="utf-8") as log:
        for epoch in range(epochs):
            multiplier = lr_multiplier(epoch, epochs, warmup_epochs)
            for parameter_group, base_lr in zip(optimizer.param_groups, (backbone_lr, head_lr)):
                parameter_group["lr"] = base_lr * multiplier
            model.train()
            freeze_batchnorm_running_stats(model, torch)
            order = list(range(len(training_rows)))
            random.Random(seed + epoch * 1009).shuffle(order)
            rng = random.Random(seed + epoch * 1009 + 1)
            loss_sum = 0.
            for start in range(0, len(order), batch_size):
                batch = [training_rows[index] for index in order[start:start + batch_size]]
                inputs = torch.stack([_input_tensor(row, torch, rng, training=True,
                                                   blur_probability=blur_probability) for row in batch]).to(device)
                target = torch.tensor([row["bits"] for row in batch], dtype=torch.float32, device=device)
                optimizer.zero_grad(set_to_none=True)
                loss = criterion(model(inputs), target)
                if not torch.isfinite(loss):
                    raise ValueError("Non-finite training loss")
                loss.backward()
                optimizer.step()
                loss_sum += float(loss.detach().item()) * len(batch)
            validation = _evaluate_rows(model, validation_rows, torch, device, batch_size)
            f1, bce = validation["metrics"]["macro_f1"], validation["bce"]
            improved = validation_is_better(f1, bce, best)
            if improved:
                best = {"epoch": epoch + 1, "macro_f1": f1, "bce": bce}
                checkpoint = {"research_only": True, "not_production": True, "architecture": "resnet18",
                    "training_scope": "full_backbone_finetune_BN_running_stats_frozen_affine_trainable_FP32",
                    "model_state_dict": {key: value.detach().cpu().clone() for key, value in model.state_dict().items()},
                    "classes": list(CLASSES), "threshold": .5, "dropout": dropout,
                    "best_validation": best, "provenance": provenance, "quality_gate": quality,
                    "input_size_wh": [WIDTH, HEIGHT], "normalization": "ImageNet",
                    "preprocessing": "inference_fixed_approved_ROI_then_aspect_preserving_RGB_black_letterbox",
                    "train_only_augmentation": training_augmentation_contract(blur_probability)}
                torch.save(checkpoint, output / "best.pt")
            row = {"epoch": epoch + 1, "train_weighted_bce": loss_sum / len(training_rows),
                   "val_unweighted_bce": bce, "val_macro_f1": f1,
                   "val_metrics": validation["metrics"], "backbone_lr": backbone_lr * multiplier,
                   "head_lr": head_lr * multiplier, "checkpoint_updated": improved,
                   "test_evaluated": False}
            log.write(json.dumps(row, allow_nan=False) + "\n")
            log.flush()
    # Only the validation-chosen checkpoint may see the held-out test images.
    chosen = torch.load(str(output / "best.pt"), map_location="cpu", weights_only=True)
    model.load_state_dict(chosen["model_state_dict"], strict=True)
    model.to(device).eval()
    final = {}
    for split, rows in (("train", training_rows), ("val", validation_rows),
                        ("test", [row for row in records if row["split"] == "test"])):
        evaluated = _evaluate_rows(model, rows, torch, device, batch_size)
        for row, probability, predicted in zip(rows, evaluated["probabilities"], evaluated["predictions"]):
            final[row["record_index"]] = (probability, predicted)
    baseline = _baseline()
    hsv_predictions = []
    for row in records:
        original = np.asarray(_fixed_crop(row)).copy()
        observed, _ = baseline.classify_crop(original[:, :, ::-1].copy())
        hsv_predictions.append(hsv_bits(observed))
    probabilities = [final[row["record_index"]][0] for row in records]
    predictions = [final[row["record_index"]][1] for row in records]
    return {"best_validation": chosen["best_validation"], "probabilities": probabilities,
            "predictions": predictions, "hsv_predictions": hsv_predictions,
            "runtime": {"torch": str(torch.__version__), "torchvision": str(torchvision.__version__),
                        "numpy": str(np.__version__), "opencv": str(baseline.cv2.__version__)},
            "train_pos_weight": dict(zip(CLASSES, positive_weights)), "test_evaluation_pass_count": 1,
            "test_batch_forward_count": math.ceil(sum(row["split"] == "test" for row in records) / batch_size)}


def execute(dataset, backbone_weights, output=None, *, allow_weak_native_labels=False,
            validate_only=False, **configuration):
    configuration = {**DEFAULT_CONFIGURATION, **configuration}
    validate_config(**configuration)
    weights = Path(backbone_weights).expanduser().resolve(strict=True)
    if not weights.is_file():
        raise ValueError("Explicit local pretrained ResNet18 state_dict required")
    if output is not None and (Path(output).exists() or Path(output).is_symlink()):
        raise FileExistsError("Output must be a fresh research-only directory")
    records, dataset_metadata = load_dataset(dataset, allow_manual_background=True)
    quality = quality_gate(records, allow_weak_native_labels)
    provenance = {"dataset": dataset_metadata, "backbone_weights_path": str(weights),
                  "annotation_counts": {"native_verified": quality["native_verified_record_count"],
                                        "manual_background_no_lamps": quality["manual_background_record_count"]},
                  "backbone_weights_sha256": sha256(weights), "script_sha256": sha256(__file__),
                  "train_only_augmentation": training_augmentation_contract(configuration["blur_probability"]),
                  "hsv_source_sha256": sha256(BASELINE_PATH),
                  "frozen_probe_helper_sha256": sha256(Path(__file__).with_name("train_traffic_light_crop_probe.py"))}
    if validate_only:
        return {"research_only": True, "training_executed": False, "provenance": provenance, "quality_gate": quality}
    if output is None:
        raise ValueError("A fresh --output directory is required for training")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    result = train(records, weights, output, provenance, quality, **configuration)
    if sha256(dataset_metadata["path"]) != dataset_metadata["sha256"] or sha256(weights) != provenance["backbone_weights_sha256"]:
        raise ValueError("Dataset/checkpoint source changed during training")
    predictions = result["predictions"]
    report = {"research_only": True, "not_production": True, "scope": "full_ResNet18_backbone_finetune_not_frozen_feature_probe",
        "quality_gate": quality, "provenance": provenance, "configuration": configuration,
        "selection": {"criterion": "highest_val_macro_f1_then_lowest_unweighted_val_BCE", "threshold": .5,
                      "best_validation": result["best_validation"], "test_used_for_selection": False,
                      "test_evaluation_pass_count": result["test_evaluation_pass_count"],
                      "test_batch_forward_count": result["test_batch_forward_count"]},
        "training_contract": {"precision": "FP32", "AMP": False, "compile": False,
            "optimizer": "AdamW", "backbone_trainable": True, "BN_running_stats_frozen": True,
            "BN_affine_trainable": True, "train_pos_weight": result["train_pos_weight"],
            "lr_schedule": "linear_epoch_warmup_then_cosine", "warmup_epochs": configuration["warmup_epochs"],
            "minimum_lr_multiplier": .01,
            "deterministic_algorithms": True, "TF32": False},
        "preprocessing": {"input_size_wh": [WIDTH, HEIGHT], "JPEG_decoder": "Pillow_RGB_for_both_CNN_and_HSV",
            "letterbox": "aspect_preserving_RGB_black_bilinear",
            "normalization": "ImageNet",
            "train_only_augmentation": training_augmentation_contract(configuration["blur_probability"]),
            "val_test": "fixed_QC_ROI_no_augmentation",
            "HSV_input": "same_fixed_original_ROI_BGR_before_CNN_letterbox"},
        "limitations": ["Native state/QC does not certify real-world or production safety performance",
                        "Pose-group split is not automatically independent geography",
                        "Four clean known lamp bits only; unknown/unreadable need separate evaluation"],
        "runtime": result["runtime"], "cnn": evaluation_metrics(records, predictions),
        "existing_hsv_baseline": evaluation_metrics(records, result["hsv_predictions"]),
        "checkpoint": {"path": "best.pt", "sha256": sha256(output / "best.pt")}}
    with (output / "predictions.jsonl").open("x", encoding="utf-8") as stream:
        for row, probability, predicted, hsv in zip(records, result["probabilities"], predictions, result["hsv_predictions"]):
            item = {key: row[key] for key in ("record_index", "image_path", "image_sha256", "crop_xyxy", "split",
                                             "group_id", "environment", "labels", "native_gt_verified")}
            item.update(research_only=True, manual_qc_accepted=row.get("manual_qc_accepted"),
                        annotation_kind=row.get("annotation_kind"),
                        manual_no_lamp_qc_accepted=row.get("manual_no_lamp_qc_accepted"),
                        cnn_probabilities=dict(zip(CLASSES, probability)), cnn_labels=dict(zip(CLASSES, predicted)),
                        hsv_labels=dict(zip(CLASSES, hsv)))
            stream.write(json.dumps(item, allow_nan=False) + "\n")
    with (output / "metrics.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--backbone-weights", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", default="cpu", help="cpu or explicit cuda:N")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--backbone-lr", type=float, default=1e-4)
    parser.add_argument("--head-lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--warmup-epochs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dropout", type=float, choices=(0., .1), default=0.)
    parser.add_argument("--blur-probability", type=float, default=0.)
    parser.add_argument("--allow-weak-native-labels", action="store_true", help="Explicit research-only unreviewed-label opt in")
    parser.add_argument("--validate-only", action="store_true", help="No Torch import/training")
    args = vars(parser.parse_args(argv))
    result = execute(**args)
    print(json.dumps({"research_only": True, "training_executed": not args["validate_only"],
                      "output": str(args["output"]) if args["output"] else None,
                      "quality_gate": result["quality_gate"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
