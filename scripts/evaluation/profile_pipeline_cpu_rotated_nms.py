#!/usr/bin/env python3
"""Independent lazy geometry-cache NMS candidate, never patches production."""
from __future__ import annotations

import argparse
import ast
import copy
import cProfile
import gzip
import hashlib
import json
import math
from pathlib import Path
import pstats
import io
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "base_ws/src/perception/bevfusion_ros"
sys.path.insert(0, str(PACKAGE / "src"))
from bevfusion_ros import rotated_bev_nms as baseline


def candidate_indices(detections, iou_threshold):
    """Same plain-dictionary arithmetic and stage ordering; only per-frame cache."""
    threshold = float(iou_threshold)
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("rotated BEV NMS IoU threshold must be in [0, 1]")
    ranked_indices = sorted(range(len(detections)),
        key=lambda index: (-float(detections[index].get("score", 0.0)), index))
    # Geometry is lazy: malformed unused centers/yaw must not newly throw.
    geometry = [{} for _ in detections]
    kept_indices, kept_by_class = [], {}

    def size(index):
        cache = geometry[index]
        if "size" not in cache:
            cache["size"] = detections[index].get("size", ())
        return cache["size"]

    def area(index):
        cache = geometry[index]
        if "area" not in cache:
            value = size(index)
            cache["area"] = abs(float(value[0]) * float(value[1]))
        return cache["area"]

    def center(index, axis):
        cache = geometry[index]
        key = "x" if axis == 0 else "y"
        if key not in cache:
            cache[key] = float(detections[index]["center"][axis])
        return cache[key]

    def radius(index):
        cache = geometry[index]
        if "radius" not in cache:
            value = size(index)
            cache["radius"] = .5 * math.hypot(float(value[0]), float(value[1]))
        return cache["radius"]

    def polygon(index):
        cache = geometry[index]
        if "polygon" not in cache:
            detection = detections[index]
            cache["polygon"] = baseline._rectangle_corners(detection["center"], size(index), detection.get("yaw", 0.0))
        return cache["polygon"]

    def iou(first, second):
        first_size, second_size = size(first), size(second)
        if len(first_size) < 2 or len(second_size) < 2:
            return 0.0
        first_area, second_area = area(first), area(second)
        if first_area <= 0.0 or second_area <= 0.0:
            return 0.0
        center_dx = center(first, 0) - center(second, 0)
        center_dy = center(first, 1) - center(second, 1)
        first_radius, second_radius = radius(first), radius(second)
        if center_dx * center_dx + center_dy * center_dy > (first_radius + second_radius) ** 2:
            return 0.0
        first_polygon, second_polygon = polygon(first), polygon(second)
        intersection = baseline._polygon_area(baseline._polygon_intersection(first_polygon, second_polygon))
        union = first_area + second_area - intersection
        return intersection / union if union > 0.0 else 0.0

    for candidate_index in ranked_indices:
        candidate_class = baseline.normalize_class_name(detections[candidate_index].get("class_name"))
        same_class_kept = kept_by_class.setdefault(candidate_class, [])
        suppressed = False
        for kept_index in same_class_kept:
            if iou(candidate_index, kept_index) > threshold:
                suppressed = True
                break
        if not suppressed:
            kept_indices.append(candidate_index)
            same_class_kept.append(candidate_index)
    return sorted(kept_indices)


def candidate_nms(detections, threshold):
    return [detections[index] for index in candidate_indices(detections, threshold)]


def reference_index_function():
    """Exact original function, final return indexes rather than dictionaries."""
    tree = ast.parse(Path(baseline.__file__).read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "class_aware_rotated_bev_nms")
    function = copy.deepcopy(function)
    function.name = "reference_indices"
    function.body[-1] = ast.Return(value=ast.Call(func=ast.Name(id="sorted", ctx=ast.Load()),
        args=[ast.Name(id="kept_indices", ctx=ast.Load())], keywords=[]))
    tree = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(tree)
    namespace = dict(baseline.__dict__)
    exec(compile(tree, "exact_original_nms_indices", "exec"), namespace)
    return namespace["reference_indices"]


reference_indices = reference_index_function()


def operating_rows(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as stream:
        payload = json.load(stream)
    result = []
    classes = ("vehicle", "pedestrian", "obstacle")
    for row in payload["rows"]:
        if "predictions" in row:
            predictions = row["predictions"]
        else:
            predictions = [dict(center=box[:3], size=box[3:6], yaw=box[6], score=score,
                                class_name=classes[label])
                           for box, label, score in zip(row["pred_boxes"], row["pred_labels"], row["pred_scores"])]
        # Only server floor/top100: actual bridge NMS runs before class gates.
        predictions = sorted((point for point in predictions if point["score"] >= .1),
                             key=lambda point: -point["score"])[:100]
        result.append(predictions)
    return result


def stats(values):
    if not values:
        return dict(count=0)
    values = np.asarray(values, dtype=float)
    return dict(count=len(values), mean=float(values.mean()), median=float(np.median(values)),
        p90=float(np.percentile(values, 90)), p95=float(np.percentile(values, 95)),
        p99=float(np.percentile(values, 99)), max=float(values.max()))


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.repeat <= 10:
        parser.error("repeat must be 1..10")
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    args.output.mkdir(parents=True)
    report = dict(production_changed=False, baseline_sha256=digest(Path(baseline.__file__)),
        candidate_sha256=digest(Path(__file__)), cases=[],
        limits=["CPU-only offline dictionary NMS; not GPU inference, ROS bridge callback or live latency certification.",
                "Normal GC enabled; paired interleaving reduces drift but does not remove host scheduler/concurrent work variation.",
                "Plain JSON dictionary inputs; cache lives only within one invocation, no stale frame/global cache.",
                "Server score floor0.1/top100 and NMS IoU0.50 unchanged; class gates follow NMS and are not part of timing."])
    functions = {"baseline": baseline.class_aware_rotated_bev_nms, "candidate": candidate_nms}
    for path in args.predictions:
        frames = operating_rows(path)
        original = copy.deepcopy(frames)
        expected = [reference_indices(frame, .5) for frame in frames]
        for frame, wanted in zip(frames, expected):
            if candidate_indices(frame, .5) != wanted:
                raise AssertionError("Candidate retained indices differ")
        if frames != original:
            raise AssertionError("Qualification mutated original predictions")
        costs = {name: [] for name in functions}
        # Warm up without changing caches: each call creates a fresh cache.
        for frame in frames[:30]:
            for function in functions.values():
                function(frame, .5)
        for repeat in range(args.repeat):
            for index, frame in enumerate(frames):
                order = ("baseline", "candidate") if (index + repeat) % 2 else ("candidate", "baseline")
                results = {}
                for name in order:
                    started = time.perf_counter()
                    results[name] = functions[name](frame, .5)
                    costs[name].append((time.perf_counter() - started) * 1000.)
                if len(results["baseline"]) != len(results["candidate"]) or any(
                        left is not right for left, right in zip(results["baseline"], results["candidate"])):
                    raise AssertionError("Returned object identity/order differs")
        if frames != original:
            raise AssertionError("Timed calls mutated predictions")
        case = dict(path=str(path), sha256=digest(path), frames=len(frames), repeat=args.repeat,
            input_boxes=sum(map(len, frames)), kept_boxes=sum(map(len, expected)),
            box_count=stats(list(map(len, frames))),
            timing_ms={name: stats(value) for name, value in costs.items()},
            retained_indices_exact=True, returned_identity_order_exact=True, input_mutation=False)
        report["cases"].append(case)
        print(json.dumps(case), flush=True)
        for name, function in functions.items():
            profiler = cProfile.Profile()
            profiler.enable()
            for frame in frames:
                function(frame, .5)
            profiler.disable()
            text = io.StringIO()
            pstats.Stats(profiler, stream=text).strip_dirs().sort_stats("cumulative").print_stats(30)
            (args.output / (path.stem + "_" + name + "_profile.txt")).write_text(text.getvalue())
        (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
