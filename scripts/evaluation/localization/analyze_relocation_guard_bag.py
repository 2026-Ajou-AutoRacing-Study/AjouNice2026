#!/usr/bin/env python3

"""Audit the checkpoint-relocation guard against recorded localization output."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag


def load_xy(path, topics):
    rows = {topic: [] for topic in topics}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            stamp = message.header.stamp.to_sec()
            position = message.pose.pose.position
            if stamp > 0.0 and all(
                math.isfinite(value) for value in (position.x, position.y)
            ):
                rows[topic].append((stamp, position.x, position.y))
    return {topic: np.asarray(values, dtype=float) for topic, values in rows.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--gnss-topic", default="/localization/gnss/odom")
    parser.add_argument("--state-topic", default="/localization/kinematic_state")
    parser.add_argument("--max-age-sec", type=float, default=0.1)
    parser.add_argument("--threshold-m", type=float, default=5.0)
    parser.add_argument("--consecutive", type=int, default=3)
    parser.add_argument("--cooldown-sec", type=float, default=5.0)
    parser.add_argument("--synthetic-offset-m", type=float, default=0.0)
    parser.add_argument("--synthetic-start-fraction", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows = load_xy(args.bag, (args.gnss_topic, args.state_topic))
    gnss = rows[args.gnss_topic]
    state = rows[args.state_topic]
    if len(gnss) == 0 or len(state) == 0:
        raise SystemExit("bag must contain non-empty GNSS and fused-state topics")
    start_index = int(len(gnss) * args.synthetic_start_fraction)
    indices = np.searchsorted(state[:, 0], gnss[:, 0])
    innovations = []
    ages = []
    trigger_indices = []
    count = 0
    longest = 0
    last_trigger_stamp = None
    for index, (sample, right) in enumerate(zip(gnss, indices)):
        candidates = [value for value in (right - 1, right) if 0 <= value < len(state)]
        nearest = min(candidates, key=lambda value: abs(state[value, 0] - sample[0]))
        age = abs(state[nearest, 0] - sample[0])
        if age > args.max_age_sec:
            count = 0
            continue
        gx = sample[1]
        if args.synthetic_offset_m and index >= start_index:
            gx += args.synthetic_offset_m
        innovation = math.hypot(gx - state[nearest, 1], sample[2] - state[nearest, 2])
        innovations.append(innovation)
        ages.append(age)
        in_cooldown = (
            last_trigger_stamp is not None
            and sample[0] - last_trigger_stamp < args.cooldown_sec
        )
        count = count + 1 if innovation >= args.threshold_m and not in_cooldown else 0
        longest = max(longest, count)
        if count == args.consecutive:
            trigger_indices.append(index)
            last_trigger_stamp = sample[0]
            count = 0

    values = np.asarray(innovations)
    result = {
        "bag": str(args.bag),
        "pair_count": int(len(values)),
        "max_age_sec": args.max_age_sec,
        "pair_age_p95_ms": float(np.percentile(ages, 95) * 1000.0),
        "innovation_m": {
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)),
            "max": float(np.max(values)),
        },
        "threshold_m": args.threshold_m,
        "consecutive_required": args.consecutive,
        "samples_over_threshold": int(np.count_nonzero(values >= args.threshold_m)),
        "longest_consecutive": int(longest),
        "cooldown_sec": args.cooldown_sec,
        "trigger_count": len(trigger_indices),
        "first_trigger_input_index": trigger_indices[0] if trigger_indices else None,
        "synthetic_offset_m": args.synthetic_offset_m,
        "synthetic_start_input_index": start_index if args.synthetic_offset_m else None,
    }
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
