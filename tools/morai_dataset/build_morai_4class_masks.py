#!/usr/bin/env python3
"""
Build task-aligned 4-class semantic masks from MORAI Dataset Capture.

Target IDs
---------
0: Background
1: Pedestrian
2: Vehicle
3: Obstacle

Important design:
- Raw Dataset Capture files are NEVER modified.
- MORAI semantic RGB(A) -> target ID is NOT hard-coded.
- The script discovers semantic colors from:
    BBox2D class name -> Instance color -> Instance.png pixels -> Semantic.png pixels
- Unknown BBox classes are reported and, in --strict mode, stop mask generation.
- This handles the observed case where the same MORAI class can use different
  semantic colors in different captures, as long as they map to the same target class.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is required. Install with: pip install pyyaml", file=sys.stderr)
    raise

RGBA_RE = re.compile(r"\((\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)\s*$")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input-root", required=True, type=Path,
                   help="Raw MORAI Dataset Capture root. Never modified.")
    p.add_argument("--output-root", required=True, type=Path,
                   help="Derived 4-class dataset output root.")
    p.add_argument("--class-map", required=True, type=Path,
                   help="YAML mapping from MORAI class names to target classes.")
    p.add_argument("--min-instance-semantic-ratio", type=float, default=0.98,
                   help="Minimum dominant semantic-color ratio inside an instance mask.")
    p.add_argument("--strict", action="store_true",
                   help="Fail if any BBox class is not explicitly mapped/ignored.")
    p.add_argument("--copy-images", action="store_true",
                   help="Copy matching *_Intensity.png into output/images.")
    p.add_argument("--write-preview", action="store_true",
                   help="Write human-readable RGB preview masks.")
    return p.parse_args()


def load_class_map(path: Path):
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))

    target_classes = cfg["target_classes"]
    required = {"background": 0, "pedestrian": 1, "vehicle": 2, "obstacle": 3}
    for name, idx in required.items():
        if int(target_classes.get(name, -999)) != idx:
            raise ValueError(
                f"target_classes must contain exactly {required}; got {target_classes}"
            )

    source_to_target = {}
    for target_name in ("pedestrian", "vehicle", "obstacle"):
        for source_name in cfg.get("source_classes", {}).get(target_name, []):
            key = source_name.strip().lower()
            if key in source_to_target and source_to_target[key] != target_name:
                raise ValueError(f"Source class mapped twice: {source_name}")
            source_to_target[key] = target_name

    ignored = {x.strip().lower() for x in cfg.get("ignore_bbox_classes", [])}
    return cfg, source_to_target, ignored


def parse_bbox_line(line: str):
    line = line.strip()
    if not line:
        return None
    parts = line.split()
    if not parts:
        return None

    class_name = parts[0]
    m = RGBA_RE.search(line)
    if not m:
        raise ValueError(f"Could not parse instance RGBA from line: {line}")

    rgba = tuple(int(x) for x in m.groups())
    return class_name, rgba


def dominant_semantic_color(instance_rgba, semantic_rgba, instance_color):
    color = np.asarray(instance_color, dtype=np.uint8)
    mask = np.all(instance_rgba == color, axis=2)
    pixel_count = int(mask.sum())
    if pixel_count == 0:
        return None, 0, 0.0

    pixels = semantic_rgba[mask]
    colors, counts = np.unique(pixels, axis=0, return_counts=True)
    i = int(np.argmax(counts))
    dominant = tuple(int(x) for x in colors[i])
    ratio = float(counts[i] / counts.sum())
    return dominant, pixel_count, ratio


def relative_stem(path: Path, root: Path, suffix: str):
    rel = path.relative_to(root)
    name = rel.name
    if not name.endswith(suffix):
        raise ValueError(name)
    stem = name[:-len(suffix)]
    return rel.parent, stem


def discover_color_mapping(
    input_root: Path,
    source_to_target: dict,
    ignored: set,
    min_ratio: float,
):
    observations = []
    unknown_classes = Counter()
    target_color_counts = defaultdict(Counter)

    bbox_files = sorted(input_root.rglob("*_BBox2D.txt"))

    for bbox_path in bbox_files:
        parent, stem = relative_stem(bbox_path, input_root, "_BBox2D.txt")
        base_dir = input_root / parent
        instance_path = base_dir / f"{stem}_Instance.png"
        semantic_path = base_dir / f"{stem}_Semantic.png"

        if not instance_path.exists() or not semantic_path.exists():
            observations.append({
                "bbox_file": str(bbox_path),
                "class_name": "",
                "target_class": "",
                "instance_rgba": "",
                "semantic_rgba": "",
                "instance_pixels": 0,
                "dominant_ratio": 0.0,
                "status": "missing_instance_or_semantic",
            })
            continue

        instance = np.array(Image.open(instance_path).convert("RGBA"))
        semantic = np.array(Image.open(semantic_path).convert("RGBA"))

        if instance.shape != semantic.shape:
            raise RuntimeError(
                f"Image size mismatch:\n  {instance_path}\n  {semantic_path}"
            )

        for raw_line in bbox_path.read_text(encoding="utf-8").splitlines():
            parsed = parse_bbox_line(raw_line)
            if parsed is None:
                continue

            class_name, instance_color = parsed
            key = class_name.strip().lower()

            if key in ignored:
                target_name = "ignored"
            else:
                target_name = source_to_target.get(key)

            if target_name is None:
                unknown_classes[class_name] += 1

            semantic_color, pixel_count, ratio = dominant_semantic_color(
                instance, semantic, instance_color
            )

            if pixel_count == 0:
                status = "instance_color_not_found"
            elif ratio < min_ratio:
                status = "low_semantic_consistency"
            elif target_name is None:
                status = "unknown_bbox_class"
            elif target_name == "ignored":
                status = "ignored"
            else:
                status = "ok"
                target_color_counts[target_name][semantic_color] += 1

            observations.append({
                "bbox_file": str(bbox_path.relative_to(input_root)),
                "class_name": class_name,
                "target_class": target_name or "",
                "instance_rgba": instance_color,
                "semantic_rgba": semantic_color,
                "instance_pixels": pixel_count,
                "dominant_ratio": ratio,
                "status": status,
            })

    color_to_target = {}
    conflicts = []

    target_id = {"pedestrian": 1, "vehicle": 2, "obstacle": 3}
    for target_name, counter in target_color_counts.items():
        for color, obs_count in counter.items():
            tid = target_id[target_name]
            if color in color_to_target and color_to_target[color] != tid:
                conflicts.append((color, color_to_target[color], tid))
            else:
                color_to_target[color] = tid

    if conflicts:
        raise RuntimeError(f"Conflicting semantic colors across target classes: {conflicts}")

    return observations, unknown_classes, target_color_counts, color_to_target


def write_csv(path: Path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def rgba_to_str(c):
    return "" if c is None else ",".join(str(int(x)) for x in c)


def save_mask(mask: np.ndarray, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(mask.astype(np.uint8), mode="L").save(path)


def save_preview(mask: np.ndarray, path: Path):
    # Display colors only; class IDs in the grayscale mask are the ground truth.
    palette = np.array([
        [0, 0, 0],        # background
        [255, 0, 255],    # pedestrian
        [255, 0, 0],      # vehicle
        [255, 255, 0],    # obstacle
    ], dtype=np.uint8)
    preview = palette[mask]
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(preview, mode="RGB").save(path)


def generate_masks(
    input_root: Path,
    output_root: Path,
    color_to_target: dict,
    copy_images: bool,
    write_preview: bool,
):
    masks_root = output_root / "masks"
    images_root = output_root / "images"
    previews_root = output_root / "previews"

    manifest = []
    totals = Counter()

    semantic_files = sorted(input_root.rglob("*_Semantic.png"))

    for semantic_path in semantic_files:
        parent, stem = relative_stem(semantic_path, input_root, "_Semantic.png")
        base_dir = input_root / parent
        intensity_path = base_dir / f"{stem}_Intensity.png"

        # A segmentation training sample requires an RGB/Intensity pair.
        if not intensity_path.exists():
            continue

        semantic = np.array(Image.open(semantic_path).convert("RGBA"))
        mask = np.zeros(semantic.shape[:2], dtype=np.uint8)

        for color, target_id in color_to_target.items():
            c = np.asarray(color, dtype=np.uint8)
            hit = np.all(semantic == c, axis=2)
            mask[hit] = target_id

        rel_mask = parent / f"{stem}_4class.png"
        out_mask = masks_root / rel_mask
        save_mask(mask, out_mask)

        out_img = ""
        if copy_images:
            dst_img = images_root / parent / intensity_path.name
            dst_img.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(intensity_path, dst_img)
            out_img = str(dst_img.relative_to(output_root))

        if write_preview:
            out_preview = previews_root / parent / f"{stem}_4class_preview.png"
            save_preview(mask, out_preview)

        counts = np.bincount(mask.ravel(), minlength=4)
        totals.update({i: int(counts[i]) for i in range(4)})

        manifest.append({
            "source_intensity": str(intensity_path.relative_to(input_root)),
            "source_semantic": str(semantic_path.relative_to(input_root)),
            "output_mask": str(out_mask.relative_to(output_root)),
            "copied_image": out_img,
            "background_pixels": int(counts[0]),
            "pedestrian_pixels": int(counts[1]),
            "vehicle_pixels": int(counts[2]),
            "obstacle_pixels": int(counts[3]),
        })

    return manifest, totals


def main():
    args = parse_args()
    args.input_root = args.input_root.resolve()
    args.output_root = args.output_root.resolve()
    args.class_map = args.class_map.resolve()

    if not args.input_root.exists():
        raise SystemExit(f"Input root not found: {args.input_root}")

    args.output_root.mkdir(parents=True, exist_ok=True)
    reports_root = args.output_root / "reports"
    reports_root.mkdir(parents=True, exist_ok=True)

    cfg, source_to_target, ignored = load_class_map(args.class_map)

    observations, unknown_classes, target_color_counts, color_to_target = \
        discover_color_mapping(
            args.input_root,
            source_to_target,
            ignored,
            args.min_instance_semantic_ratio,
        )

    # Always write discovery reports first.
    obs_rows = []
    for x in observations:
        y = dict(x)
        y["instance_rgba"] = rgba_to_str(y["instance_rgba"]) \
            if isinstance(y["instance_rgba"], tuple) else y["instance_rgba"]
        y["semantic_rgba"] = rgba_to_str(y["semantic_rgba"]) \
            if isinstance(y["semantic_rgba"], tuple) else y["semantic_rgba"]
        obs_rows.append(y)

    write_csv(
        reports_root / "mapping_observations.csv",
        obs_rows,
        [
            "bbox_file", "class_name", "target_class",
            "instance_rgba", "semantic_rgba",
            "instance_pixels", "dominant_ratio", "status",
        ],
    )

    unknown_rows = [
        {"class_name": name, "observations": count}
        for name, count in sorted(unknown_classes.items())
    ]
    write_csv(
        reports_root / "unknown_bbox_classes.csv",
        unknown_rows,
        ["class_name", "observations"],
    )

    resolved_rows = []
    id_to_name = {1: "pedestrian", 2: "vehicle", 3: "obstacle"}
    for color, tid in sorted(color_to_target.items(), key=lambda x: (x[1], x[0])):
        resolved_rows.append({
            "semantic_rgba": rgba_to_str(color),
            "target_id": tid,
            "target_class": id_to_name[tid],
        })
    write_csv(
        reports_root / "resolved_semantic_colors.csv",
        resolved_rows,
        ["semantic_rgba", "target_id", "target_class"],
    )

    # Preserve an exact copy of the mapping config used for this derived dataset.
    shutil.copy2(args.class_map, args.output_root / "semantic_class_mapping_used.yaml")

    if args.strict and unknown_classes:
        names = ", ".join(f"{k}({v})" for k, v in sorted(unknown_classes.items()))
        raise SystemExit(
            "STOPPED (--strict): unmapped BBox classes were found: "
            + names
            + "\nAdd them to the YAML as a target class or ignore_bbox_classes, "
              "then rerun. Raw data was not modified."
        )

    bad = [x for x in observations if x["status"] in
           {"instance_color_not_found", "low_semantic_consistency"}]
    if bad:
        raise SystemExit(
            f"STOPPED: {len(bad)} mapping observations failed consistency checks. "
            f"See {reports_root / 'mapping_observations.csv'}"
        )

    manifest, totals = generate_masks(
        args.input_root,
        args.output_root,
        color_to_target,
        args.copy_images,
        args.write_preview,
    )

    write_csv(
        args.output_root / "manifest.csv",
        manifest,
        [
            "source_intensity", "source_semantic", "output_mask", "copied_image",
            "background_pixels", "pedestrian_pixels",
            "vehicle_pixels", "obstacle_pixels",
        ],
    )

    stats = {
        "target_classes": {
            "0": "background",
            "1": "pedestrian",
            "2": "vehicle",
            "3": "obstacle",
        },
        "samples_generated": len(manifest),
        "total_pixels": {
            "background": totals[0],
            "pedestrian": totals[1],
            "vehicle": totals[2],
            "obstacle": totals[3],
        },
        "resolved_semantic_colors": {
            rgba_to_str(color): {
                "target_id": tid,
                "target_class": id_to_name[tid],
            }
            for color, tid in color_to_target.items()
        },
        "unknown_bbox_classes": dict(unknown_classes),
    }

    (args.output_root / "dataset_stats.json").write_text(
        json.dumps(stats, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print("Done.")
    print(f"  samples: {len(manifest)}")
    print(f"  output : {args.output_root}")
    print(f"  report : {reports_root / 'resolved_semantic_colors.csv'}")
    print("  class IDs: 0=Background, 1=Pedestrian, 2=Vehicle, 3=Obstacle")


if __name__ == "__main__":
    main()
