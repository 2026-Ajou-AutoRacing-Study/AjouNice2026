#!/usr/bin/env python3
"""Research-only frozen ResNet18 features + four-bit linear-head pilot.

Never downloads weights, changes production, starts ROS, or labels camera data.
Input JSON: {"records": [{"image_path": "relative.jpg", "image_sha256": "...",
"crop_xyxy": [x1,y1,x2,y2], "labels": {"red":0,"yellow":0,"green":1,
"left_arrow":0}, "split":"train|val|test", "group_id":"pose-group",
"environment":"Sunny13", "native_gt_verified":true}, ...]}.
group_id MUST identify a pose/frame leakage group, not merely a signal ID.
All environments/phases/repeated frames from one pose belong to one split.
Native state verification is not proof that every lamp is visible in the crop.
The Python loader has an explicit manual-background opt-in; this frozen pilot's
CLI deliberately does not enable it. Background is never converted to native GT.
Torch, torchvision, NumPy and OpenCV are imported only for actual training.
"""

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
BASELINE_PATH = ROOT / "base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py"
CLASSES = ("red", "yellow", "green", "left_arrow")
SPLITS = ("train", "val", "test")
WIDTH, HEIGHT = 224, 128


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_labels(labels):
    if not isinstance(labels, dict) or set(labels) != set(CLASSES):
        raise ValueError("all four exact label bits must be known")
    if any(type(labels[key]) not in (int, bool) or labels[key] not in (0, 1) for key in CLASSES):
        raise ValueError("label bits must be bool or integer 0/1, never unknown/probability")
    return [int(labels[key]) for key in CLASSES]


def annotation_source(row, allow_manual_background=False):
    """Do not convert manually reviewed negatives into fabricated native GT.

    Unknown/unreadable signal crops are NOT background. The opt-in contract
    requires an explicit manual no-lamps claim and both exact QC flags. Pixel
    visibility itself cannot be certified by this metadata-only validator.
    Manual review is not an independent human-GT certification.
    """
    if type(allow_manual_background) is not bool:
        raise ValueError("allow_manual_background must be an explicit boolean")
    native = row.get("native_gt_verified")
    kind = row.get("annotation_kind")
    if native is True:
        if kind == "manual_background_no_lamps":
            raise ValueError("Manual background must retain native_gt_verified:false, never fabricated true")
        return "native_verified"
    if not allow_manual_background:
        raise ValueError("native_gt_verified:true is required for every record")
    if native is not False:
        raise ValueError("Manual background requires explicit native_gt_verified:false")
    if kind != "manual_background_no_lamps":
        raise ValueError("Unverified/unknown signal is not manual_background_no_lamps")
    if row.get("manual_qc_accepted") is not True or row.get("manual_no_lamp_qc_accepted") is not True:
        raise ValueError("Manual background requires both manual QC flags exactly true")
    if any(validate_labels(row.get("labels"))):
        raise ValueError("Manual background requires all four known label bits zero")
    return "manual_background_no_lamps"


def validate_crop(box, width, height):
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError("crop_xyxy must have four source-image pixel coordinates")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in box):
        raise ValueError("crop coordinates must be finite numbers")
    x1, y1, x2, y2 = box
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError("crop is empty or outside source image bounds")
    # This matches the node's positive-coordinate int truncation, without an
    # implicit clamp. CNN and HSV both receive this exact original-image ROI.
    integer_box = tuple(int(x) for x in box)
    if integer_box[2] <= integer_box[0] or integer_box[3] <= integer_box[1]:
        raise ValueError("crop becomes empty under integer pixel slicing")
    return integer_box


def image_size(path):
    from PIL import Image  # Lazy; pure validation tests inject a size reader.
    with Image.open(path) as image:
        image.verify()
    with Image.open(path) as image:
        return image.size


def train_pos_weight(records):
    rows = [row["bits"] for row in records if row["split"] == "train"]
    if not rows:
        raise ValueError("train split must not be empty")
    positives = [sum(row[index] for row in rows) for index in range(4)]
    for key, positive in zip(CLASSES, positives):
        if positive == 0 or positive == len(rows):
            raise ValueError("train needs both positive and negative examples for {}".format(key))
    return [(len(rows) - positive) / positive for positive in positives]


def load_dataset(path, size_reader=image_size, *, allow_manual_background=False):
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    document = json.loads(payload)
    records = document.get("records") if isinstance(document, dict) else None
    if not isinstance(records, list) or not records:
        raise ValueError("dataset must contain a nonempty records list")
    prepared, groups, frames, paths, cached = [], {}, {}, {}, {}
    annotation_counts = Counter()
    for index, row in enumerate(records):
        if not isinstance(row, dict):
            raise ValueError("record must be an object")
        annotation_counts[annotation_source(row, allow_manual_background)] += 1
        split, group, environment = row.get("split"), row.get("group_id"), row.get("environment")
        if split not in SPLITS:
            raise ValueError("split must be train, val or test")
        if not isinstance(group, str) or not group.strip() or group != group.strip():
            raise ValueError("group_id must be a nonempty pose/frame-group string")
        if not isinstance(environment, str) or not environment.strip():
            raise ValueError("environment must be a nonempty string")
        relative = row.get("image_path")
        if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError("image_path must be a safe relative path")
        image = (path.parent / relative).resolve(strict=True)
        try:
            image.relative_to(path.parent)
        except ValueError as error:
            raise ValueError("image path escapes dataset directory") from error
        expected = row.get("image_sha256")
        if not image.is_file() or not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise ValueError("image file and lowercase SHA256 are required")
        if image not in cached:
            actual = sha256(image)
            size = size_reader(image)
            if (not isinstance(size, (list, tuple)) or len(size) != 2
                    or any(type(v) is not int or v <= 0 for v in size)):
                raise ValueError("decoded image dimensions must be positive integers")
            cached[image] = (actual, size)
        actual, (width, height) = cached[image]
        if actual != expected:
            raise ValueError("image SHA256 mismatch: {}".format(relative))
        for mapping, key, label in ((groups, group, "pose/frame group"),
                                    (frames, actual, "identical image bytes"),
                                    (paths, str(image), "image path")):
            if key in mapping and mapping[key] != split:
                raise ValueError("{} crosses train/val/test splits".format(label))
            mapping[key] = split
        item = dict(row)
        item.update(record_index=index, resolved_image_path=str(image),
                    crop_integer_xyxy=list(validate_crop(row.get("crop_xyxy"), width, height)),
                    bits=validate_labels(row.get("labels")), image_width=width, image_height=height)
        prepared.append(item)
    counts = Counter(row["split"] for row in prepared)
    if any(counts[split] == 0 for split in SPLITS):
        raise ValueError("train, val and test must all be nonempty")
    if len({row["group_id"] for row in prepared if row["split"] == "train"}) < 2:
        raise ValueError("train requires at least two distinct pose/frame groups")
    weights = train_pos_weight(prepared)
    return prepared, {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest(),
                      "record_count": len(prepared), "unique_image_sha256_count": len(frames),
                      "split_counts": dict(counts), "group_to_split": groups,
                      "annotation_counts": {key: annotation_counts[key] for key in ("native_verified", "manual_background_no_lamps")},
                      "allow_manual_background": allow_manual_background,
                      "pos_weight_from_train_only": dict(zip(CLASSES, weights))}


def letterbox_geometry(width, height):
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("positive integer crop dimensions required")
    scale = min(WIDTH / width, HEIGHT / height)
    resized = (max(1, min(WIDTH, round(width * scale))), max(1, min(HEIGHT, round(height * scale))))
    return resized, ((WIDTH - resized[0]) // 2, (HEIGHT - resized[1]) // 2)


def metric_summary(truth, prediction):
    if len(truth) != len(prediction):
        raise ValueError("truth/prediction count mismatch")
    for row in list(truth) + list(prediction):
        if len(row) != 4 or any(type(bit) not in (int, bool) or bit not in (0, 1) for bit in row):
            raise ValueError("metrics require four known binary bits per row")
    count = len(truth)
    per_class = {}
    for index, key in enumerate(CLASSES):
        tp = sum(bool(a[index]) and bool(b[index]) for a, b in zip(truth, prediction))
        fp = sum(not a[index] and bool(b[index]) for a, b in zip(truth, prediction))
        fn = sum(bool(a[index]) and not b[index] for a, b in zip(truth, prediction))
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.0
        per_class[key] = {"precision": precision, "recall": recall, "f1": f1,
                          "tp": tp, "fp": fp, "fn": fn, "tn": count - tp - fp - fn,
                          "positive_support": tp + fn}
    exact = sum(list(a) == list(b) for a, b in zip(truth, prediction))
    return {"count": count, "exact_set_correct": exact,
            "exact_set_accuracy": exact / count if count else None,
            "per_class": per_class, "zero_division": 0,
            "macro_f1": sum(item["f1"] for item in per_class.values()) / 4 if count else None}


def evaluation_metrics(records, predictions):
    if len(records) != len(predictions):
        raise ValueError("record/prediction count mismatch")
    result = {}
    for split in SPLITS:
        indices = [i for i, row in enumerate(records) if row["split"] == split]
        def subset(selected):
            return metric_summary([records[i]["bits"] for i in selected], [predictions[i] for i in selected])
        item = {"overall": subset(indices)}
        for field in ("environment", "group_id"):
            item["by_" + field] = {
                key: subset([i for i in indices if records[i][field] == key])
                for key in sorted({records[i][field] for i in indices})}
        result[split] = item
    return result


def hsv_bits(observations):
    bits = {key: 0 for key in CLASSES}
    for color, shape, left in observations:
        if color in ("red", "yellow", "green") and shape == "circle":
            bits[color] = 1
        if color == "green" and shape == "arrow" and left is not False:
            bits["left_arrow"] = 1
    return [bits[key] for key in CLASSES]


def _baseline():
    spec = importlib.util.spec_from_file_location("traffic_crop_probe_hsv", BASELINE_PATH)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def run_pilot(records, backbone_weights):
    import numpy as np
    from PIL import Image
    import torch
    import torchvision
    from torchvision.models import resnet18
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.set_num_threads(4)
    torch.use_deterministic_algorithms(True)
    backbone = resnet18(weights=None)  # Explicitly no download/pretrained registry.
    state = torch.load(str(backbone_weights), map_location="cpu", weights_only=True)
    backbone.load_state_dict(state, strict=True)
    backbone.fc = torch.nn.Identity()
    backbone.requires_grad_(False)
    backbone.eval()
    baseline = _baseline()
    mean = torch.tensor([.485, .456, .406]).reshape(3, 1, 1)
    std = torch.tensor([.229, .224, .225]).reshape(3, 1, 1)
    features, baseline_predictions, batch = [], [], []
    with torch.inference_mode():
        for index, row in enumerate(records):
            image_path = Path(row["resolved_image_path"])
            if sha256(image_path) != row["image_sha256"]:
                raise ValueError("image changed after validation: {}".format(row["image_path"]))
            with Image.open(image_path) as source:
                crop = source.convert("RGB").crop(tuple(row["crop_integer_xyxy"]))
            original = np.asarray(crop).copy()
            observations, _ = baseline.classify_crop(original[:, :, ::-1].copy())
            baseline_predictions.append(hsv_bits(observations))
            resized, offset = letterbox_geometry(*crop.size)
            canvas = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
            canvas.paste(crop.resize(resized, Image.Resampling.BILINEAR), offset)
            tensor = torch.from_numpy(np.asarray(canvas).copy()).permute(2, 0, 1).float() / 255
            batch.append((tensor - mean) / std)
            if len(batch) == 32 or index == len(records) - 1:
                features.append(backbone(torch.stack(batch)).clone())
                batch = []
    # Clone outside inference_mode: frozen feature constants participate in the
    # linear head's backward pass even though the backbone has no gradients.
    features = torch.cat(features).clone()
    labels = torch.tensor([row["bits"] for row in records], dtype=torch.float32)
    train = [i for i, row in enumerate(records) if row["split"] == "train"]
    torch.manual_seed(0)
    head = torch.nn.Linear(512, 4)
    optimizer = torch.optim.AdamW(head.parameters(), lr=.01, weight_decay=.001)
    loss_function = torch.nn.BCEWithLogitsLoss(pos_weight=torch.tensor(train_pos_weight(records)))
    loss_history = []
    head.train()
    for iteration in range(100):
        optimizer.zero_grad(set_to_none=True)
        loss = loss_function(head(features[train]), labels[train])
        if not torch.isfinite(loss):
            raise ValueError("non-finite training loss")
        loss.backward()
        optimizer.step()
        if iteration in (0, 24, 49, 74, 99):
            loss_history.append({"iteration": iteration + 1, "train_loss": float(loss.detach())})
    head.eval()
    with torch.no_grad():
        probabilities = torch.sigmoid(head(features)).tolist()
    predictions = [[int(value >= .5) for value in row] for row in probabilities]
    return {"probabilities": probabilities, "predictions": predictions,
            "hsv_predictions": baseline_predictions, "head_state_dict": head.state_dict(),
            "loss_history": loss_history, "torch_version": torch.__version__,
            "torchvision_version": torchvision.__version__}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--backbone-weights", type=Path, required=True,
                        help="Explicit existing local ResNet18 state_dict; never auto-download")
    parser.add_argument("--output", type=Path, help="Fresh research-only output directory")
    parser.add_argument("--save-head", action="store_true", help="Save research_only candidate_head.pt")
    parser.add_argument("--validate-only", action="store_true", help="Validate data/weights paths only; no Torch import or training")
    args = parser.parse_args(argv)
    if not args.validate_only and args.output is None:
        parser.error("--output is required for the pilot")
    weights = args.backbone_weights.expanduser().resolve(strict=True)
    if not weights.is_file():
        parser.error("--backbone-weights must be an existing local file")
    if args.output is not None and args.output.exists():
        parser.error("output must be a fresh directory")
    records, dataset = load_dataset(args.dataset)
    provenance = {"dataset": dataset, "backbone_weights_path": str(weights),
                  "backbone_weights_sha256": sha256(weights), "script_sha256": sha256(__file__),
                  "hsv_source_path": str(BASELINE_PATH), "hsv_source_sha256": sha256(BASELINE_PATH)}
    if args.validate_only:
        print(json.dumps({"research_only": True, "training_executed": False, **provenance}, indent=2))
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    result = run_pilot(records, weights)
    report = {
        "research_only": True, "not_production": True,
        "scope": "Frozen pretrained ResNet18 features plus newly trained linear head; not a trained traffic-light CNN or housing detector",
        "native_gt_visibility_limit": "native_gt_verified is a caller manifest claim. Actual visible lamp/ROI correctness is NOT independently verified; these pilot scores are not deployment accuracy or a safety certification.",
        "split_limit": "Pose/frame-group separation does not certify independent geography, simulator rendering or real-world generalization.",
        "provenance": provenance,
        "training": {"seed": 0, "device": "cpu", "threads": 4, "backbone": "resnet18",
                     "backbone_frozen_eval": True, "head": "Linear(512,4)",
                     "iterations": 100, "optimizer": "AdamW", "learning_rate": .01,
                     "weight_decay": .001, "loss": "BCEWithLogitsLoss",
                     "pos_weight": dataset["pos_weight_from_train_only"],
                     "threshold": .5, "validation_used_for_selection": False,
                     "loss_history": result["loss_history"]},
        "preprocessing": {"width": WIDTH, "height": HEIGHT, "resize": "aspect_preserving_bilinear",
                          "letterbox": "centered_black_RGB", "normalization": "ImageNet",
                          "crop_rule": "int_truncation_positive_xyxy_no_clamp",
                          "augmentations": [], "hsv_input": "identical_original_ROI_BGR_before_CNN_letterbox"},
        "runtime": {"torch": result["torch_version"], "torchvision": result["torchvision_version"]},
        "linear_head": evaluation_metrics(records, result["predictions"]),
        "existing_hsv_baseline": evaluation_metrics(records, result["hsv_predictions"]),
    }
    with (args.output / "metrics.json").open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    with (args.output / "predictions.jsonl").open("x") as stream:
        for row, probabilities, predicted, hsv in zip(records, result["probabilities"], result["predictions"], result["hsv_predictions"]):
            record = {key: row[key] for key in ("record_index", "image_path", "image_sha256", "crop_xyxy",
                      "crop_integer_xyxy", "labels", "split", "group_id", "environment", "native_gt_verified")}
            record.update(research_only=True, cnn_probabilities=dict(zip(CLASSES, probabilities)),
                          cnn_labels=dict(zip(CLASSES, predicted)), hsv_labels=dict(zip(CLASSES, hsv)))
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
    if args.save_head:
        import torch
        torch.save({"research_only": True, "not_production": True, "classes": CLASSES,
                    "head_state_dict": result["head_state_dict"], "threshold": .5,
                    "provenance": provenance, "preprocessing": report["preprocessing"]},
                   args.output / "candidate_head.pt")
    print(json.dumps({"research_only": True, "output": str(args.output), "records": len(records)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
