#!/usr/bin/env python3
"""Verified multi-housing YOLO dataset export; never edit/re-encode originals."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import shutil

SPLITS = ("train", "val", "test")
CLASS_NAME = "traffic_light"
MISSION_HEAD_IDS = {"C1256W000077", "C1256W000091", "C1256W000016", "C1256W000018", "C1256W000052"}


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def image_size(path):
    from PIL import Image
    with Image.open(path) as image:
        if image.format != "JPEG":
            raise ValueError("Only original JPEG inputs are supported")
        image.verify()
    with Image.open(path) as image:
        return image.size


def validate_box(box, width, height):
    if not isinstance(box, dict) or type(box.get("class_id")) is not int or box["class_id"] != 0:
        raise ValueError("Every housing box needs integer class_id:0")
    if box.get("box_qc_accepted") is not True:
        raise ValueError("Every tight housing box requires box_qc_accepted:true")
    xyxy = box.get("xyxy")
    if (not isinstance(xyxy, list) or len(xyxy) != 4 or
            any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in xyxy)):
        raise ValueError("Box xyxy must contain four finite pixel coordinates")
    x1, y1, x2, y2 = xyxy
    if not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height:
        raise ValueError("Housing box is empty or outside source JPEG")
    role = box.get("head_role", "unknown")
    if role not in ("near", "far", "other", "unknown"):
        raise ValueError("head_role must be near/far/other/unknown, never inferred")
    return {**box, "xyxy": [float(value) for value in xyxy], "head_role": role}


def load_dataset(path, *, development_pilot=False, size_reader=image_size):
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    document = json.loads(payload)
    rows = document.get("records")
    if not isinstance(rows, list) or not rows:
        raise ValueError("A nonempty detection records list is required")
    records, groups, hashes, paths, cache = [], {}, {}, set(), {}
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or row.get("annotation_complete") is not True:
            raise ValueError("Every full image requires annotation_complete:true; no partial box labels")
        split, group, environment = (row.get(key) for key in ("split", "group_id", "environment"))
        if split not in SPLITS or not isinstance(group, str) or not group.strip() or group != group.strip():
            raise ValueError("Explicit train/val/test split and pose group are required")
        if not isinstance(environment, str) or not environment.strip():
            raise ValueError("Explicit environment or UNKNOWN is required")
        relative = row.get("image_path")
        if (not isinstance(relative, str) or not relative or Path(relative).is_absolute()
                or ".." in Path(relative).parts or Path(relative).suffix.lower() not in (".jpg", ".jpeg")):
            raise ValueError("image_path must be a safe relative original JPEG")
        image = (path.parent / relative).resolve(strict=True)
        try:
            image.relative_to(path.parent)
        except ValueError as error:
            raise ValueError("Image escapes dataset artifact root") from error
        if image in paths:
            raise ValueError("Each source JPEG path must appear once with ALL housing boxes")
        paths.add(image)
        expected = row.get("image_sha256")
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise ValueError("Original JPEG SHA256 is required")
        if image not in cache:
            cache[image] = (digest(image), size_reader(image))
        actual, (width, height) = cache[image]
        if actual != expected:
            raise ValueError("Source JPEG SHA256 mismatch")
        if type(width) is not int or type(height) is not int or min(width, height) < 1:
            raise ValueError("Invalid JPEG dimensions")
        if row.get("width") != width or row.get("height") != height:
            raise ValueError("Declared dimensions differ from original JPEG")
        for mapping, key in ((groups, group), (hashes, actual)):
            if key in mapping and mapping[key] != split:
                raise ValueError("Pose group or identical JPEG bytes cross splits")
            mapping[key] = split
        boxes = row.get("boxes")
        if not isinstance(boxes, list):
            raise ValueError("boxes must be a complete list, including [] for verified negatives")
        boxes = [validate_box(box, width, height) for box in boxes]
        if "label_sha256" in row:
            parts = Path(relative).parts
            if len(parts) != 3 or parts[0] != "images" or parts[1] != split:
                raise ValueError("Exported YOLO image/label layout is inconsistent")
            label = path.parent / "labels" / split / (Path(relative).stem + ".txt")
            expected_text = "".join(normalized_label(box, width, height) + "\n" for box in boxes)
            if digest(label) != row["label_sha256"] or label.read_text() != expected_text:
                raise ValueError("YOLO normalized labels differ from verified manifest boxes")
        records.append({**row, "record_index": index, "resolved_image_path": str(image), "boxes": boxes})
    counts = Counter(row["split"] for row in records)
    required = ("train", "val") if development_pilot else SPLITS
    if any(not counts[split] for split in required):
        raise ValueError("Required splits are empty; absent test needs explicit --development-pilot")
    if len({row["group_id"] for row in records if row["split"] == "train"}) < 2:
        raise ValueError("At least two distinct train pose groups are required")
    if not any(row["boxes"] for row in records if row["split"] == "train"):
        raise ValueError("Training needs at least one accepted housing box")
    return records, {"source_path": str(path), "source_sha256": hashlib.sha256(payload).hexdigest(),
                     "split_image_counts": dict(counts), "group_to_split": groups,
                     "split_environment_image_counts":{split:dict(Counter(row["environment"] for row in records if row["split"]==split)) for split in SPLITS},
                     "split_box_counts":{split:sum(len(row["boxes"]) for row in records if row["split"]==split) for split in SPLITS},
                     "box_count": sum(len(row["boxes"]) for row in records),
                     "unique_JPEG_sha256_count": len(hashes), "development_pilot": development_pilot}


def normalized_label(box, width, height):
    x1, y1, x2, y2 = box["xyxy"]
    return "0 {:.10f} {:.10f} {:.10f} {:.10f}".format((x1 + x2) / (2 * width),
            (y1 + y2) / (2 * height), (x2 - x1) / width, (y2 - y1) / height)


def write_data_yaml(path, dataset_root, include_test=True):
    # JSON strings are valid YAML scalars, including paths with spaces.
    text = "path: {}\ntrain: images/train\nval: images/val\n".format(json.dumps(str(Path(dataset_root).resolve())))
    if include_test:
        text += "test: images/test\n"
    text += "names:\n  0: traffic_light\n"
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(text)


def export_dataset(source, output, *, development_pilot=False):
    output = Path(output)
    if output.exists() or output.is_symlink():
        raise FileExistsError("YOLO dataset output directory must be fresh")
    records, metadata = load_dataset(source, development_pilot=development_pilot)
    output.mkdir(parents=True, exist_ok=False)
    for split in SPLITS:
        (output / "images" / split).mkdir(parents=True)
        (output / "labels" / split).mkdir(parents=True)
    exported = []
    for row in records:
        stem = "{:06d}_{}".format(row["record_index"], row["image_sha256"][:12])
        relative = "images/{}/{}.jpg".format(row["split"], stem)
        shutil.copyfile(row["resolved_image_path"], output / relative)
        if digest(output / relative) != row["image_sha256"]:
            raise ValueError("Byte-for-byte JPEG export failed")
        label = output / "labels" / row["split"] / (stem + ".txt")
        with label.open("x", encoding="utf-8") as stream:
            stream.write("".join(normalized_label(box, row["width"], row["height"]) + "\n" for box in row["boxes"]))
        item = {key: value for key, value in row.items() if key != "resolved_image_path"}
        item.update(image_path=relative, source_image_path=row["image_path"], label_sha256=digest(label))
        exported.append(item)
    manifest = {"schema_version": 1, "research_only": True, "class_names": {"0": CLASS_NAME},
                "source": metadata, "records": exported, "image_encoding": "unchanged_original_JPEG_bytes",
                "export_source_sha256": digest(__file__)}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
    write_data_yaml(output / "data.yaml", output, include_test=metadata["split_image_counts"].get("test", 0) > 0)
    return manifest


def iou(a, b):
    intersection = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.


def match_detections(truth, predictions, threshold=.5):
    """Confidence-ordered one-to-one matching; do not count duplicate detections twice."""
    used, matches, false_positive = set(), [], []
    for index in sorted(range(len(predictions)), key=lambda i: predictions[i]["confidence"], reverse=True):
        candidates = [(iou(box["xyxy"], predictions[index]["xyxy"]), j) for j, box in enumerate(truth) if j not in used]
        overlap, target = max(candidates, default=(0., None))
        if target is not None and overlap >= threshold:
            used.add(target)
            matches.append({"prediction_index": index, "truth_index": target, "iou": overlap})
        else:
            false_positive.append(index)
    return {"matches": matches, "false_positive_indices": false_positive,
            "missed_truth_indices": [i for i in range(len(truth)) if i not in used]}


def detection_summary(rows):
    tp = sum(len(row["matching"]["matches"]) for row in rows)
    fp = sum(len(row["matching"]["false_positive_indices"]) for row in rows)
    fn = sum(len(row["matching"]["missed_truth_indices"]) for row in rows)
    bins = {}
    for row in rows:
        matched = {item["truth_index"] for item in row["matching"]["matches"]}
        for index, box in enumerate(row["truth_boxes"]):
            x1, y1, x2, y2 = box["xyxy"]
            categories = ["role:" + box["head_role"], "area:" + ("small_lt32sq" if (x2-x1)*(y2-y1) < 1024 else "not_small"),
                          "height:" + ("le8" if y2-y1 <= 8 else "le16" if y2-y1 <= 16 else "gt16")]
            native = box.get("native_id")
            if native:categories.append("native:"+native)
            categories.append("mission:" + ("allowed5" if native in MISSION_HEAD_IDS else "other_front_car_head" if native else "native_id_unknown"))
            for category in categories:
                item = bins.setdefault(category, {"gt": 0, "matched": 0})
                item["gt"] += 1
                item["matched"] += index in matched
    for item in bins.values():
        item["recall"] = item["matched"] / item["gt"] if item["gt"] else None
    return {"images": len(rows), "tp": tp, "fp": fp, "fn": fn,
            "unmatched_predictions_per_image":fp/len(rows) if rows else None,
            "precision_at_fixed_conf": tp / (tp+fp) if tp+fp else None,
            "recall_at_fixed_conf": tp / (tp+fn) if tp+fn else None, "GT_box_bins": bins,
            "matched_non_mission_head_count_not_detector_FP": bins.get("mission:other_front_car_head",{}).get("matched",0),
            "route_association_evaluated": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--development-pilot", action="store_true")
    args = parser.parse_args()
    manifest = export_dataset(args.dataset, args.output, development_pilot=args.development_pilot)
    print(json.dumps({"output": str(args.output.resolve()), **manifest["source"]}))


if __name__ == "__main__":
    main()
