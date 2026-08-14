#!/usr/bin/env python3
"""Validate Fast-BEV range/grid consistency and MORAI GT coverage."""

import argparse
import json
import pickle
from collections import Counter
from pathlib import Path

import numpy as np
from mmcv import Config


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    return parser.parse_args()


def as_list(value):
    return value.tolist() if hasattr(value, "tolist") else list(value)


def get_infos(path):
    with Path(path).expanduser().open("rb") as stream:
        payload = pickle.load(stream)
    return payload["infos"] if isinstance(payload, dict) else payload


def get_annotations(info):
    annos = info.get("annos", info)
    boxes = annos.get("gt_boxes_upright_depth", annos.get("gt_boxes"))
    names = annos.get("class", annos.get("gt_names"))
    if boxes is None or names is None:
        raise KeyError("annotation boxes/classes are missing")
    return np.asarray(boxes), np.asarray(names).astype(str)


def find_transform(pipeline, transform_type):
    matches = [item for item in pipeline if item.get("type") == transform_type]
    if len(matches) != 1:
        raise AssertionError(
            f"expected one {transform_type}, found {len(matches)}")
    return matches[0]


def check_equal(name, actual, expected, atol=1e-6):
    if not np.allclose(actual, expected, atol=atol, rtol=0.0):
        raise AssertionError(f"{name}: {actual} != {expected}")


def coverage_for_split(ann_file, point_cloud_range):
    minimum = np.asarray(point_cloud_range[:3], dtype=np.float64)
    maximum = np.asarray(point_cloud_range[3:], dtype=np.float64)
    total = Counter()
    inside = Counter()
    sectors = Counter()
    for info in get_infos(ann_file):
        boxes, names = get_annotations(info)
        for box, name in zip(boxes, names):
            center = np.asarray(box[:3], dtype=np.float64)
            total[name] += 1
            if np.all(center >= minimum) and np.all(center <= maximum):
                inside[name] += 1
            x, y = center[:2]
            if x >= 0 and abs(x) >= abs(y):
                sectors[f"{name}/front"] += 1
            elif x < 0 and abs(x) >= abs(y):
                sectors[f"{name}/rear"] += 1
            elif y >= 0:
                sectors[f"{name}/left"] += 1
            else:
                sectors[f"{name}/right"] += 1
    classes = sorted(total)
    return {
        "frames": len(get_infos(ann_file)),
        "total": dict(total),
        "inside": dict(inside),
        "retention": {
            name: inside[name] / total[name] if total[name] else 0.0
            for name in classes
        },
        "sector_total": dict(sectors),
    }


def main():
    args = parse_args()
    cfg = Config.fromfile(str(args.config.expanduser().resolve()))
    point_cloud_range = np.asarray(cfg.point_cloud_range, dtype=np.float64)
    n_voxels = np.asarray(cfg.model.n_voxels[0], dtype=np.int64)
    voxel_size = np.asarray(cfg.model.voxel_size[0], dtype=np.float64)
    extent = point_cloud_range[3:] - point_cloud_range[:3]
    represented_extent = n_voxels * voxel_size
    # Fast-BEV's upstream M0 intentionally samples four 1.5 m Z cells (6 m)
    # around the midpoint of an 8 m annotation range.  XY must match exactly;
    # report the narrower Z sampling range without rejecting the stock design.
    check_equal("XY range extent", represented_extent[:2], extent[:2])

    anchor_range = as_list(
        cfg.model.bbox_head.anchor_generator.ranges[0])
    expected_anchor = [
        point_cloud_range[0], point_cloud_range[1], anchor_range[2],
        point_cloud_range[3], point_cloud_range[4], anchor_range[5],
    ]
    check_equal("anchor XY range", anchor_range, expected_anchor)

    train_pipeline = cfg.data.train.pipeline
    val_pipeline = cfg.data.val.pipeline
    test_pipeline = cfg.data.test.pipeline
    check_equal(
        "train ObjectRangeFilter",
        find_transform(train_pipeline, "ObjectRangeFilter")[
            "point_cloud_range"], point_cloud_range)
    for name, pipeline in (
            ("train", train_pipeline),
            ("val", val_pipeline),
            ("test", test_pipeline)):
        check_equal(
            f"{name} KittiSetOrigin",
            find_transform(pipeline, "KittiSetOrigin")[
                "point_cloud_range"], point_cloud_range)

    origin = ((point_cloud_range[:3] + point_cloud_range[3:]) / 2.0)
    represented_minimum = origin - represented_extent / 2.0
    represented_maximum = origin + represented_extent / 2.0
    payload = {
        "config": str(args.config.expanduser().resolve()),
        "point_cloud_range": point_cloud_range.tolist(),
        "n_voxels": n_voxels.tolist(),
        "voxel_size": voxel_size.tolist(),
        "origin": origin.tolist(),
        "represented_range": np.concatenate(
            [represented_minimum, represented_maximum]).tolist(),
        "xy_cells": int(n_voxels[0] * n_voxels[1]),
        "anchors": int(
            n_voxels[0] * n_voxels[1] *
            len(cfg.model.bbox_head.anchor_generator.sizes) *
            len(cfg.model.bbox_head.anchor_generator.rotations)),
        "coverage": {},
    }
    for split in ("train", "val", "test"):
        dataset_cfg = cfg.data[split]
        payload["coverage"][split] = coverage_for_split(
            dataset_cfg.ann_file, point_cloud_range)

    print(json.dumps(payload, indent=2, sort_keys=True))
    if args.output_json:
        output_path = args.output_json.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        print("report:", output_path)


if __name__ == "__main__":
    main()
