#!/usr/bin/env python3
"""Apply class-specific score thresholds to canonical DetectedObjects bags."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


VEHICLE_LABELS = frozenset((1, 2, 3, 4, 5, 6))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument("--vehicle", type=float, default=0.40)
    parser.add_argument("--pedestrian", type=float, default=0.15)
    parser.add_argument("--obstacle", type=float, default=0.10)
    return parser.parse_args()


def classification_and_score(obj):
    if not obj.classification:
        return None, 0.0
    dominant = max(
        obj.classification, key=lambda item: float(item.probability)
    )
    label = int(dominant.label)
    if label in VEHICLE_LABELS:
        class_name = "vehicle"
    elif label == 7:
        class_name = "pedestrian"
    elif label == 0:
        class_name = "obstacle"
    else:
        return None, 0.0
    return class_name, max(
        float(obj.existence_probability), float(dominant.probability)
    )


def main():
    args = parse_args()
    thresholds = {
        "vehicle": args.vehicle,
        "pedestrian": args.pedestrian,
        "obstacle": args.obstacle,
    }
    if any(not 0.0 <= value <= 1.0 for value in thresholds.values()):
        raise ValueError("all thresholds must be in [0, 1]")
    if args.input.resolve() == args.output.resolve():
        raise ValueError("input and output bags must differ")

    import rosbag

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".active")
    if temporary.exists():
        temporary.unlink()

    counts = {
        "messages": 0,
        "before": {name: 0 for name in thresholds},
        "after": {name: 0 for name in thresholds},
    }
    try:
        with rosbag.Bag(str(args.input), "r") as source, rosbag.Bag(
            str(temporary), "w", compression="lz4"
        ) as destination:
            for topic, message, timestamp in source.read_messages():
                if topic == args.topic:
                    counts["messages"] += 1
                    retained = []
                    for obj in message.objects:
                        class_name, score = classification_and_score(obj)
                        if class_name is None:
                            continue
                        counts["before"][class_name] += 1
                        if score >= thresholds[class_name]:
                            retained.append(obj)
                            counts["after"][class_name] += 1
                    message.objects = retained
                destination.write(topic, message, timestamp)
        temporary.replace(args.output)
    finally:
        if temporary.exists():
            temporary.unlink()

    print(
        json.dumps(
            {
                "input": str(args.input.resolve()),
                "output": str(args.output.resolve()),
                "topic": args.topic,
                "thresholds": thresholds,
                **counts,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
