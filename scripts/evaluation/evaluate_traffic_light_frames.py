#!/usr/bin/env python3
"""Offline YOLO housing + existing HSV/picker diagnostics, not a CNN or scorer.

Read a sample_traffic_light_bag_frames.py manifest and verify original JPEG
hashes before inference. Originals are never rewritten. A fresh output directory
receives results.jsonl, summary.json, and review_contact_sheet.jpg. The contact
sheet is a derived review artifact, never ground truth. No ROS is imported and
no live topics, policies, smoothing, or production settings are changed.

Optional human GT JSON is a flat image_path -> annotation mapping. Annotations
are attached separately and are not scored by this diagnostic runner.
"""

import argparse
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WEIGHTS = ROOT / "base_ws/src/perception/traffic_light_detection/weights/best.pt"


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def read_manifest(path, max_frames=None):
    """Validate paths and every selected JPEG hash before model loading."""
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    manifest = json.loads(payload)
    frames = manifest.get("frames")
    if manifest.get("schema_version") != 1 or not isinstance(frames, list) or not frames:
        raise ValueError("Expected a nonempty schema_version=1 sampler manifest")
    if manifest.get("frame_count", len(frames)) != len(frames):
        raise ValueError("Manifest frame_count does not match frames")
    if max_frames is not None and (isinstance(max_frames, bool)
                                  or not isinstance(max_frames, int) or max_frames <= 0):
        raise ValueError("max_frames must be a positive integer or None")
    seen = set()
    selected = []
    for index, frame in enumerate(frames):
        relative = frame.get("image_path")
        if not isinstance(relative, str) or not relative:
            raise ValueError("Every frame needs a relative image_path")
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts or relative in seen:
            raise ValueError("Unsafe or duplicate manifest image_path")
        seen.add(relative)
        image_path = (path.parent / relative_path).resolve(strict=True)
        try:
            image_path.relative_to(path.parent)
        except ValueError as error:
            raise ValueError("Image path escapes manifest directory") from error
        if not image_path.is_file() or not re.fullmatch(r"[0-9a-f]{64}", str(frame.get("sha256", ""))):
            raise ValueError("Image must be a regular file with a SHA256 digest")
        if max_frames is None or index < max_frames:
            image_payload = image_path.read_bytes()
            if hashlib.sha256(image_payload).hexdigest() != frame["sha256"]:
                raise ValueError("Image SHA256 mismatch: {}".format(relative))
            if not image_payload.startswith(b"\xff\xd8\xff") or not image_payload.endswith(b"\xff\xd9"):
                raise ValueError("Manifest image is not a complete JPEG: {}".format(relative))
            selected.append((frame, image_path))
    return manifest, selected, hashlib.sha256(payload).hexdigest()


def load_human_annotations(path, known_paths):
    if path is None:
        return {}, None
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    annotations = json.loads(payload)
    if not isinstance(annotations, dict) or any(not isinstance(key, str) for key in annotations):
        raise ValueError("Human GT must be an image_path -> annotation JSON object")
    unknown = set(annotations) - set(known_paths)
    if unknown:
        raise ValueError("Human GT contains unknown image paths: {}".format(sorted(unknown)))
    return annotations, {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest(),
                         "status": "supplied_by_caller_not_independently_verified"}


@contextmanager
def offline_environment():
    """Isolate Ultralytics settings; restore caller environment on every exit."""
    keys = ("YOLO_OFFLINE", "YOLO_CONFIG_DIR", "MPLCONFIGDIR")
    previous = {key: os.environ.get(key) for key in keys}
    with tempfile.TemporaryDirectory(prefix="ajounice-traffic-offline-") as directory:
        root = Path(directory)
        (root / "yolo").mkdir()
        (root / "matplotlib").mkdir()
        os.environ.update(YOLO_OFFLINE="true", YOLO_CONFIG_DIR=str(root / "yolo"),
                          MPLCONFIGDIR=str(root / "matplotlib"))
        try:
            yield
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


def load_yolo(weights_path):
    from ultralytics import YOLO  # Lazy: tests and --help do not require YOLO/Torch.
    return YOLO(str(weights_path))


def model_names(model):
    names = model.names
    normalized = {int(key): value for key, value in names.items()} if isinstance(names, dict) else dict(enumerate(names))
    if normalized != {0: "traffic_light"}:
        raise ValueError("Expected the single-class traffic_light housing detector, got {!r}".format(names))
    return normalized


def _tolist(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return value.tolist() if hasattr(value, "tolist") else list(value)


def prediction_boxes(results):
    if len(results) != 1:
        raise ValueError("Expected exactly one YOLO result for one image")
    boxes = results[0].boxes
    if boxes is None:
        return []
    coordinates, confidence, classes = (_tolist(getattr(boxes, name)) for name in ("xyxy", "conf", "cls"))
    if not len(coordinates) == len(confidence) == len(classes):
        raise ValueError("YOLO result box arrays have inconsistent lengths")
    records = []
    for xyxy, score, class_id in zip(coordinates, confidence, classes):
        xyxy, score, class_id = list(map(float, xyxy)), float(score), float(class_id)
        if (len(xyxy) != 4 or not all(math.isfinite(value) for value in xyxy + [score, class_id])
                or xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1] or not 0 <= score <= 1
                or class_id != 0):
            raise ValueError("Invalid/non-housing YOLO box")
        records.append({"xyxy": xyxy, "confidence": score, "class_id": 0})
    return records


def observation_label(observation):
    if not observation:
        return "no_lamp_observation"
    return "+".join("{}_{}".format(color, shape if shape != "arrow" else
                                   ("arrow_right" if left is False else "arrow_left"))
                    for color, shape, left in observation)


def review_tile(image, diagnosis, frame_index, image_path, has_gt):
    """Derived annotated thumbnail + selected crop; return local pixel geometry."""
    import cv2
    import numpy as np
    tile = np.full((240, 480, 3), 24, np.uint8)
    annotated = image.copy()
    selected = diagnosis["selected_index"]
    for box in diagnosis["boxes"]:
        x1, y1, x2, y2 = box["xyxy_integer_clamped_as_node"]
        color = (0, 255, 255) if box["index"] == selected else (170, 170, 170)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(annotated, str(box["index"]), (x1, max(12, y1)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)

    def fit(source, panel):
        left, top, right, bottom = panel
        height, width = source.shape[:2]
        scale = min((right - left) / width, (bottom - top) / height)
        target_width = max(1, min(right - left, round(width * scale)))
        target_height = max(1, min(bottom - top, round(height * scale)))
        x = left + (right - left - target_width) // 2
        y = top + (bottom - top - target_height) // 2
        resized = cv2.resize(source, (target_width, target_height), interpolation=cv2.INTER_AREA
                             if scale < 1 else cv2.INTER_NEAREST)
        tile[y:y + target_height, x:x + target_width] = resized
        return {"rendered_xyxy_in_tile": [x, y, x + target_width, y + target_height],
                "source_dimensions_wh": [width, height]}

    full_panel, crop_panel = [4, 30, 324, 210], [332, 30, 476, 210]
    geometry = {"tile_dimensions_wh": [480, 240], "full_image_panel_xyxy_in_tile": full_panel,
                "crop_panel_xyxy_in_tile": crop_panel, "full_image_render": fit(annotated, full_panel),
                "selected_crop_source_xyxy": None, "crop_render": None}
    if selected is not None:
        box = diagnosis["boxes"][selected]
        x1, y1, x2, y2 = box["xyxy_integer_clamped_as_node"]
        crop = image[y1:y2, x1:x2]
        geometry["selected_crop_source_xyxy"] = [x1, y1, x2, y2]
        if crop.size:
            geometry["crop_render"] = fit(crop, crop_panel)
    label = observation_label(diagnosis["baseline_observation"])
    title = "{:04d} {}".format(frame_index, Path(image_path).name)
    footer = "obs={} | pick={} | GT={}".format(label, selected, "SUPPLIED" if has_gt else "UNLABELED")
    cv2.putText(tile, title[:70], (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .48, (230, 230, 230), 1)
    cv2.putText(tile, footer[:80], (5, 228), cv2.FONT_HERSHEY_SIMPLEX, .38, (230, 230, 230), 1)
    return tile, geometry


def evaluate_manifest(manifest_path, output_dir, *, weights=DEFAULT_WEIGHTS, conf=.25,
                      imgsz=640, device="cpu", max_frames=None, human_gt_json=None,
                      contact_sheet_max_frames=150, contact_sheet_columns=4, model=None):
    """Evaluate immutable images; injectable model permits pure offline tests."""
    if not math.isfinite(conf) or not 0 <= conf <= 1:
        raise ValueError("conf must be finite and in [0, 1]")
    for name, value in (("imgsz", imgsz), ("contact_sheet_max_frames", contact_sheet_max_frames),
                        ("contact_sheet_columns", contact_sheet_columns)):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError("{} must be a positive integer".format(name))
    output = Path(output_dir)
    if output.exists() or output.is_symlink():
        raise FileExistsError("Output directory must not exist: {}".format(output))
    manifest_path = Path(manifest_path).resolve(strict=True)
    manifest, frames, manifest_sha = read_manifest(manifest_path, max_frames)
    weights = Path(weights).resolve(strict=True)
    if not weights.is_file() or weights.suffix != ".pt":
        raise ValueError("A provisioned local .pt checkpoint is required")
    weights_sha = digest(weights)
    human, human_source = load_human_annotations(human_gt_json,
                                               [frame["image_path"] for frame in manifest["frames"]])
    import cv2
    import numpy as np
    from traffic_light_crop_diagnostics import diagnose_image, provenance
    output.mkdir(parents=True, exist_ok=False)
    tiles, labels, selected_counts = [], Counter(), Counter()
    boxes_total, gt_count = 0, 0
    run_started = time.perf_counter()
    supplied_model = model is not None
    with offline_environment():
        if model is None:
            model = load_yolo(weights)
        names = model_names(model)
        with (output / "results.jsonl").open("x", encoding="utf-8") as results_stream:
            for index, (frame, image_path) in enumerate(frames):
                started = time.perf_counter()
                payload = image_path.read_bytes()
                if hashlib.sha256(payload).hexdigest() != frame["sha256"]:
                    raise ValueError("Image changed after preflight: {}".format(image_path))
                image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("JPEG decode failed: {}".format(image_path))
                height, width = image.shape[:2]
                if (frame.get("width", width), frame.get("height", height)) != (width, height):
                    raise ValueError("Decoded dimensions differ from sampler manifest")
                predict_started = time.perf_counter()
                predictions = model.predict(source=image.copy(), conf=conf, imgsz=imgsz,
                                            device=device, verbose=False, save=False, stream=False)
                boxes = prediction_boxes(predictions)
                predict_seconds = time.perf_counter() - predict_started
                human_gt = human.get(frame["image_path"])
                gt_count += human_gt is not None
                diagnosis = diagnose_image(image, boxes, human_gt=human_gt)
                label = observation_label(diagnosis["baseline_observation"])
                labels[label] += 1
                selected_counts[diagnosis["crop_status"]] += 1
                boxes_total += len(boxes)
                geometry = None
                if index < contact_sheet_max_frames:
                    tile, geometry = review_tile(image, diagnosis, index, frame["image_path"], human_gt is not None)
                    column, row = index % contact_sheet_columns, index // contact_sheet_columns
                    geometry["tile_xyxy_contact_sheet"] = [column * 480, row * 240,
                                                           (column + 1) * 480, (row + 1) * 240]
                    tiles.append(tile)
                row = {"schema_version": 1, "frame_index": index, "image_path": frame["image_path"],
                       "image_sha256": frame["sha256"], "source_frame": frame,
                       "scope": "raw_baseline_observation_no_policy_or_smoothing_not_ground_truth",
                       "human_ground_truth": human_gt,
                       "human_ground_truth_status": "not_provided" if human_gt is None else "supplied_unverified",
                       "observation_label": label, "diagnosis": diagnosis, "review_geometry": geometry,
                       "timing_seconds": {"predict_and_box_copy": predict_seconds,
                                          "frame_wall_including_diagnostics": time.perf_counter() - started}}
                results_stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    sheet_rows = (len(tiles) + contact_sheet_columns - 1) // contact_sheet_columns
    sheet = np.full((sheet_rows * 240, contact_sheet_columns * 480, 3), 24, np.uint8)
    for index, tile in enumerate(tiles):
        x, y = index % contact_sheet_columns * 480, index // contact_sheet_columns * 240
        sheet[y:y + 240, x:x + 480] = tile
    encoded, jpeg = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not encoded:
        raise ValueError("Contact sheet JPEG encoding failed")
    with (output / "review_contact_sheet.jpg").open("xb") as stream:
        stream.write(jpeg.tobytes())
    if digest(manifest_path) != manifest_sha or digest(weights) != weights_sha:
        raise ValueError("Manifest/checkpoint changed during run; no success summary written")
    summary = {
        "schema_version": 1, "status": "complete",
        "evaluation_mode": "predictions_only" if not gt_count else "predictions_with_supplied_human_annotations_not_scored",
        "accuracy_computed": False,
        "accuracy_not_computed_reason": "No human GT supplied" if not gt_count else "This baseline diagnostic runner does not score annotations",
        "scope": "offline_single_class_YOLO_housing_plus_existing_HSV_picker_not_CNN_no_policy_no_smoothing",
        "manifest": {"path": str(manifest_path), "sha256": manifest_sha,
                     "source": manifest.get("source"), "annotations": manifest.get("annotations")},
        "model": {"weights_path": str(weights), "weights_sha256": weights_sha, "names": names,
                  "conf": conf, "imgsz": imgsz, "device": str(device),
                  "implementation": "supplied_predictor" if supplied_model else "ultralytics_YOLO",
                  "ultralytics_version": getattr(sys.modules.get("ultralytics"), "__version__", None)},
        "provenance": {"runner_source_path": str(Path(__file__).resolve()), "runner_source_sha256": digest(__file__),
                       "python_version": platform.python_version(), "baseline_diagnostics": provenance()},
        "frame_count": len(frames), "manifest_frame_count": len(manifest["frames"]),
        "bbox_count": boxes_total, "crop_status_counts": dict(selected_counts),
        "raw_observation_counts_not_accuracy": dict(labels),
        "human_gt_source": human_source, "frames_with_human_gt": gt_count,
        "runtime_seconds": time.perf_counter() - run_started,
        "timing_scope": "offline_wall_time_not_production_latency_or_10Hz_cadence",
        "contact_sheet": {"image_path": "review_contact_sheet.jpg", "sha256": digest(output / "review_contact_sheet.jpg"),
                          "dimensions_wh": [int(sheet.shape[1]), int(sheet.shape[0])],
                          "tile_dimensions_wh": [480, 240], "columns": contact_sheet_columns,
                          "shown_frame_indices": list(range(len(tiles))), "max_frames": contact_sheet_max_frames,
                          "scope": "derived_review_only_original_JPEGs_unchanged"},
        "results": {"path": "results.jsonl", "sha256": digest(output / "results.jsonl")},
    }
    with (output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--conf", type=float, default=.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--human-gt-json", type=Path)
    parser.add_argument("--contact-sheet-max-frames", type=int, default=150)
    parser.add_argument("--contact-sheet-columns", type=int, default=4)
    args = vars(parser.parse_args(argv))
    manifest, output = args.pop("manifest"), args.pop("output")
    summary = evaluate_manifest(manifest, output, **args)
    print(json.dumps({"summary": str((output / "summary.json").resolve()),
                      "frame_count": summary["frame_count"], "evaluation_mode": summary["evaluation_mode"]}))


if __name__ == "__main__":
    main()
