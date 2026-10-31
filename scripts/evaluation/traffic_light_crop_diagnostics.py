#!/usr/bin/env python3
"""Inspect the existing traffic-light color baseline without ROS or YOLO.

This is observation-stage diagnostics, not a replacement classifier, a lamp-ROI
detector, or a source of ground truth. Border/interior statistics refer only to
the geometric crop border. They do not identify lamp or housing pixels.

Examples::

    python traffic_light_crop_diagnostics.py --crop crop.png
    python traffic_light_crop_diagnostics.py --image frame.jpg \
        --boxes-json boxes.json --output new_report.json

Boxes JSON is a list of [x1, y1, x2, y2, confidence] (confidence may be omitted),
or dictionaries with ``xyxy`` and optional ``confidence``/``class_id`` fields.
Coordinates are float camera-image coordinates, not normalized coordinates.
Output files use exclusive creation; an existing file is never overwritten.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import platform
from pathlib import Path
import sys

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
UTILITY_PATH = (
    REPO_ROOT / "base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py"
)


def _load_baseline():
    """Load the real source module without creating a production __pycache__."""
    spec = importlib.util.spec_from_file_location("traffic_light_color_baseline", UTILITY_PATH)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


BASELINE = _load_baseline()


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance():
    """Record executable baseline provenance, not a production-runtime claim."""
    return {
        "utility_source_path": str(UTILITY_PATH),
        "utility_source_sha256": _sha256(UTILITY_PATH),
        "diagnostic_source_sha256": _sha256(__file__),
        "python_version": platform.python_version(),
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "baseline_constants": {
            "upscale_factor": BASELINE.UPSCALE_FACTOR,
            "min_lit_pixels": BASELINE.MIN_LIT_PIXELS,
            "min_pixels_for_shape_analysis": BASELINE.MIN_PIXELS_FOR_SHAPE_ANALYSIS,
            "circle_threshold": BASELINE.CIRCLE_THRESHOLD,
            "opening_kernel": BASELINE._OPEN_KERNEL.tolist(),
            "hsv_ranges_inclusive": {
                color: [[lower.tolist(), upper.tolist()] for lower, upper in ranges]
                for color, ranges in BASELINE.COLOR_RANGES.items()
            },
        },
    }


def _validate_image(image):
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ValueError("image must be a uint8 NumPy BGR array")
    if image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
        raise ValueError("image must have positive height/width and exactly three BGR channels")


def _distribution(hsv):
    distributions = {}
    for index, (name, maximum, bins) in enumerate((("h", 180, 18), ("s", 256, 16), ("v", 256, 16))):
        values = hsv[:, :, index]
        histogram, edges = np.histogram(values, bins=bins, range=(0, maximum))
        distributions[name] = {
            "min": int(values.min()),
            "max": int(values.max()),
            "mean": float(values.mean()),
            "percentiles_5_50_95": np.percentile(values, [5, 50, 95]).tolist(),
            "histogram_counts": histogram.tolist(),
            "histogram_bin_edges": edges.tolist(),
        }
    return distributions


def _fraction(numerator, denominator):
    return float(numerator) / denominator if denominator else None


def _green_diagnostics(mask, border_fraction):
    height, width = mask.shape
    count = int(np.count_nonzero(mask))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, BASELINE._OPEN_KERNEL)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    minimum_area = BASELINE.MIN_PIXELS_FOR_SHAPE_ANALYSIS / 2.0
    contour_records = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 4 * math.pi * area / perimeter**2 if perimeter else None
        eligible = area >= minimum_area and perimeter > 0
        shape = None
        if eligible:
            shape = "circle" if circularity >= BASELINE.CIRCLE_THRESHOLD else "arrow_left"
        contour_records.append({
            "area": area,
            "perimeter": perimeter,
            "circularity": circularity,
            "bounding_xywh": list(cv2.boundingRect(contour)),
            "passes_baseline_contour_filter": eligible,
            "shape_if_analyzed": shape,
        })

    border_width = max(1, int(math.ceil(min(height, width) * border_fraction)))
    border = np.ones(mask.shape, dtype=bool)
    if height > 2 * border_width and width > 2 * border_width:
        border[border_width:-border_width, border_width:-border_width] = False
    green = mask != 0
    border_area = int(np.count_nonzero(border))
    interior_area = height * width - border_area
    border_green = int(np.count_nonzero(green & border))
    interior_green = count - border_green

    if count < BASELINE.MIN_LIT_PIXELS:
        decision_path = "color_presence_gate_not_met"
    elif count < BASELINE.MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        decision_path = "default_circle_below_shape_analysis_pixel_minimum"
    elif not contours:
        decision_path = "default_circle_no_contours_after_opening"
    elif not any(record["passes_baseline_contour_filter"] for record in contour_records):
        decision_path = "default_circle_no_eligible_contours"
    else:
        decision_path = "contour_circularity"

    return {
        "pre_open_pixels": count,
        "post_open_pixels": int(np.count_nonzero(opened)),
        "presence_gate_met": count >= BASELINE.MIN_LIT_PIXELS,
        "shape_decision_path": decision_path,
        "baseline_shape_results": (
            [list(result) for result in BASELINE.classify_shapes_multi(mask, count)]
            if count >= BASELINE.MIN_LIT_PIXELS else []
        ),
        "contour_coordinate_system": "upscaled_crop_pixels",
        "contours": contour_records,
        "contour_scope": "all_opened_external_contours_even_if_baseline_short_circuits",
        "geometric_border_not_lamp_roi": {
            "requested_short_side_fraction": border_fraction,
            "border_width_upscaled_pixels": border_width,
            "border_area_pixels": border_area,
            "interior_area_pixels": interior_area,
            "border_green_pixels": border_green,
            "interior_green_pixels": interior_green,
            "border_green_occupancy": _fraction(border_green, border_area),
            "interior_green_occupancy": _fraction(interior_green, interior_area),
            "share_of_green_in_border": _fraction(border_green, count),
            "share_of_green_in_interior": _fraction(interior_green, count),
        },
    }


def diagnose_crop(crop_bgr, human_gt=None, border_fraction=0.10):
    """Return JSON-serializable baseline observations and explanatory statistics."""
    _validate_image(crop_bgr)
    if not 0 < border_fraction <= 0.5:
        raise ValueError("border_fraction must be in (0, 0.5]")
    observations, counts = BASELINE.classify_crop(crop_bgr)
    upscaled = cv2.resize(
        crop_bgr, None, fx=BASELINE.UPSCALE_FACTOR, fy=BASELINE.UPSCALE_FACTOR,
        interpolation=cv2.INTER_CUBIC,
    )
    hsv = cv2.cvtColor(upscaled, cv2.COLOR_BGR2HSV)
    return {
        "schema_version": 1,
        "scope": "baseline_observation_only_no_policy_or_smoothing",
        "provenance": provenance(),
        "original_dimensions_wh": [int(crop_bgr.shape[1]), int(crop_bgr.shape[0])],
        "upscaled_dimensions_wh": [int(upscaled.shape[1]), int(upscaled.shape[0])],
        "baseline_observation": [list(result) for result in observations],
        "baseline_color_counts": counts,
        "hsv_distribution_upscaled": _distribution(hsv),
        "green_mask": _green_diagnostics(BASELINE.get_color_mask(hsv, "green"), border_fraction),
        "human_ground_truth": human_gt,
        "ground_truth_status": "not_provided" if human_gt is None else "provided_by_caller_unverified",
    }


def _normalize_box(box):
    if isinstance(box, dict):
        xyxy = box["xyxy"]
        confidence = box.get("confidence")
        class_id = box.get("class_id")
    else:
        if len(box) not in (4, 5):
            raise ValueError("boxes must contain four coordinates and optional confidence")
        xyxy, confidence = box[:4], box[4] if len(box) == 5 else None
        class_id = None
    if len(xyxy) != 4:
        raise ValueError("xyxy must have four coordinates")
    coordinates = [float(value) for value in xyxy]
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("box coordinates must be finite")
    if coordinates[2] <= coordinates[0] or coordinates[3] <= coordinates[1]:
        raise ValueError("box x2/y2 must exceed x1/y1")
    if confidence is not None:
        confidence = float(confidence)
        if not math.isfinite(confidence):
            raise ValueError("confidence must be finite when provided")
    return coordinates, confidence, class_id


def diagnose_image(image_bgr, boxes_xyxy, selected_index=None, center_tolerance_ratio=0.20,
                   y_weight=0.6, human_gt=None, border_fraction=0.10):
    """Inspect selection and its crop using actual baseline picker semantics.

    ``selected_index=None`` uses the current picker; an explicit index is a
    diagnostic override and is recorded alongside the picker's own selection.
    Confidence is retained for provenance but does not enter the picker score.
    """
    _validate_image(image_bgr)
    height, width = image_bgr.shape[:2]
    records, picker_boxes = [], []
    for index, box in enumerate(boxes_xyxy):
        coordinates, confidence, class_id = _normalize_box(box)
        x1, y1, x2, y2 = coordinates
        cx, cy = (x1 + x2) / (2 * width), (y1 + y2) / (2 * height)
        picker_boxes.append((cx, cy, (x2 - x1) / width, (y2 - y1) / height, index))
        records.append({
            "index": index,
            "xyxy_float_original": coordinates,
            "confidence": confidence,
            "class_id": class_id,
            "center_xy_normalized": [cx, cy],
            "in_center_window": abs(cx - 0.5) <= center_tolerance_ratio,
            "selection_score": abs(cx - 0.5) + y_weight * cy,
            "xyxy_integer_clamped_as_node": [
                max(0, int(x1)), max(0, int(y1)), min(width, int(x2)), min(height, int(y2)),
            ],
        })
    picked = BASELINE.pick_ego_lane_box(picker_boxes, center_tolerance_ratio, y_weight)
    baseline_index = None if picked is None else picked[4]
    if selected_index is not None:
        if isinstance(selected_index, bool) or not isinstance(selected_index, (int, np.integer)):
            raise ValueError("selected_index must be an integer")
        if not 0 <= selected_index < len(records):
            raise ValueError("selected_index is outside the box list")
        chosen = int(selected_index)
    else:
        chosen = baseline_index
    crop_diagnostics = None
    crop_status = "no_selected_box"
    if chosen is not None:
        x1, y1, x2, y2 = records[chosen]["xyxy_integer_clamped_as_node"]
        crop = image_bgr[y1:y2, x1:x2]
        crop_status = "empty_crop_as_node" if not crop.size else "classified"
        if crop.size:
            crop_diagnostics = diagnose_crop(crop, human_gt, border_fraction)
    return {
        "schema_version": 1,
        "scope": "baseline_selection_and_observation_only_no_policy_or_smoothing",
        "provenance": provenance(),
        "image_dimensions_wh": [width, height],
        "selection_parameters": {"center_tolerance_ratio": center_tolerance_ratio, "y_weight": y_weight},
        "boxes": records,
        "baseline_picker_selected_index": baseline_index,
        "selected_index": chosen,
        "selection_mode": "baseline_picker" if selected_index is None else "caller_override",
        "crop_status": crop_status,
        "baseline_observation": [] if crop_diagnostics is None else crop_diagnostics["baseline_observation"],
        "crop_diagnostics": crop_diagnostics,
        "human_ground_truth": human_gt,
        "ground_truth_status": "not_provided" if human_gt is None else "provided_by_caller_unverified",
    }


def write_json_exclusive(report, output_path):
    """Write only a new file. Do not create directories or overwrite evidence."""
    with Path(output_path).open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--crop", type=Path, help="already selected housing crop")
    inputs.add_argument("--image", type=Path, help="full camera image; requires --boxes-json")
    parser.add_argument("--boxes-json", type=Path, help="JSON list of float xyxy boxes")
    parser.add_argument("--selected-index", type=int, help="explicit diagnostic override of baseline picker")
    parser.add_argument("--human-gt-json", type=Path, help="optional supplied annotations; never inferred")
    parser.add_argument("--center-tolerance-ratio", type=float, default=0.20)
    parser.add_argument("--y-weight", type=float, default=0.6)
    parser.add_argument("--border-fraction", type=float, default=0.10)
    parser.add_argument("--output", type=Path, help="new JSON file; omit to print to stdout")
    args = parser.parse_args(argv)
    if args.image and args.boxes_json is None:
        parser.error("--image requires --boxes-json")
    if args.crop and (args.boxes_json is not None or args.selected_index is not None):
        parser.error("--boxes-json/--selected-index require --image")
    try:
        path = args.crop or args.image
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("cannot decode input image: {}".format(path))
        gt = None if args.human_gt_json is None else json.loads(args.human_gt_json.read_text())
        if args.crop:
            report = diagnose_crop(image, gt, args.border_fraction)
        else:
            report = diagnose_image(
                image, json.loads(args.boxes_json.read_text()), args.selected_index,
                args.center_tolerance_ratio, args.y_weight, gt, args.border_fraction,
            )
        report["input_file"] = {"path": str(path.resolve()), "sha256": _sha256(path)}
        if args.output is None:
            print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
        else:
            write_json_exclusive(report, args.output)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
