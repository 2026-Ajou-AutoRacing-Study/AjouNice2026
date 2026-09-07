#!/usr/bin/env python3
"""Create a compact MORAI actor-reference versus 3D-box geometry audit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    audit_capture_dataset,
    audit_online_detection_geometry,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--raw-bag", required=True, type=Path)
    parser.add_argument("--output-bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--reference-offset-m",
        type=float,
        default=None,
        help="Override the passenger-sized dataset median offset.",
    )
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    dataset = audit_capture_dataset(args.dataset_root)
    offset = args.reference_offset_m
    if offset is None:
        offset = dataset["passenger_sized_under_5_5m"]["longitudinal_offset_m"][
            "p50"
        ]
    online = audit_online_detection_geometry(
        args.raw_bag,
        args.output_bag,
        reference_offset_m=offset,
        detection_topic=args.detection_topic,
    )
    summary = {
        "schema_version": 1,
        "purpose": (
            "Separate MORAI actor reference-position offsets from detector "
            "bbox-center and timestamp errors."
        ),
        "capture_dataset": dataset,
        "online_detection": online,
        "interpretation": {
            "reference_offset_supported": True,
            "timestamp_correction_implied": False,
            "origin_aligned_scope": (
                "Motion-model diagnostic only; it removes both the systematic "
                "reference offset and genuine per-frame detector translation error."
            ),
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / "summary.json"
    output_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"summary: {output_path}")
    print(f"dataset passenger reference offset p50: {offset:.4f} m")
    before = online["detector_minus_object_status"]["center_distance_m"]["p50"]
    after = online["after_empirical_reference_correction"]["center_distance_m"][
        "p50"
    ]
    print(f"online center distance p50: {before:.4f} -> {after:.4f} m")


if __name__ == "__main__":
    main()
