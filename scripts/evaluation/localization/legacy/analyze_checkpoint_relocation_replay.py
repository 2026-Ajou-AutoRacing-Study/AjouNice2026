#!/usr/bin/env python3

"""Summarize a synthetic checkpoint-relocation localization replay."""

import argparse
import bisect
import json
import math
from pathlib import Path

import rosbag


def stamp_sec(message, fallback):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and not stamp.is_zero():
        return stamp.to_sec()
    return fallback.to_sec()


def load_xy(bag_path, topic):
    samples = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            samples.append(
                (
                    stamp_sec(message, bag_stamp),
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                )
            )
    return samples


def load_reset_stamps(bag_path):
    stamps = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_stamp in bag.read_messages(
            topics=["/localization/initialpose"]
        ):
            stamps.append(stamp_sec(message, bag_stamp))
    return stamps


def nearest(samples, target_stamp):
    stamps = [sample[0] for sample in samples]
    index = bisect.bisect_left(stamps, target_stamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda sample: abs(sample[0] - target_stamp))


def distance(first, second):
    if first is None or second is None:
        return None
    return math.hypot(first[1] - second[1], first[2] - second[2])


def paired_distance(first_samples, second_samples, target_stamp):
    return distance(nearest(first_samples, target_stamp), nearest(second_samples, target_stamp))


def jump(samples, target_stamp, window_sec=0.08):
    return distance(
        nearest(samples, target_stamp - window_sec),
        nearest(samples, target_stamp + window_sec),
    )


def rate_hz(samples):
    if len(samples) < 2:
        return None
    duration = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / duration if duration > 0.0 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    gnss = load_xy(args.bag, "/localization/gnss/odom")
    global_state = load_xy(args.bag, "/localization/kinematic_state")
    control_state = load_xy(args.bag, "/localization/control_kinematic_state")
    reset_stamps = load_reset_stamps(args.bag)
    if not gnss or not global_state or not control_state:
        raise SystemExit("required GNSS/global/control topics are missing")
    if len(reset_stamps) < 2:
        raise SystemExit(
            "expected initial pose and at least one relocation reset, got {}".format(
                len(reset_stamps)
            )
        )
    reset_stamp = reset_stamps[-1]
    result = {
        "bag": str(args.bag.resolve()),
        "initialpose_message_count": len(reset_stamps),
        "relocation_reset_stamp": reset_stamp,
        "gnss_global_distance_m": {
            "before_0p08s": paired_distance(gnss, global_state, reset_stamp - 0.08),
            "at_reset": paired_distance(gnss, global_state, reset_stamp),
            "after_0p10s": paired_distance(gnss, global_state, reset_stamp + 0.10),
            "after_2p00s": paired_distance(gnss, global_state, reset_stamp + 2.00),
        },
        "global_control_distance_m": {
            "before_0p08s": paired_distance(
                global_state, control_state, reset_stamp - 0.08
            ),
            "at_reset": paired_distance(global_state, control_state, reset_stamp),
            "after_0p10s": paired_distance(
                global_state, control_state, reset_stamp + 0.10
            ),
            "after_2p00s": paired_distance(
                global_state, control_state, reset_stamp + 2.00
            ),
        },
        "reset_jump_m": {
            "global": jump(global_state, reset_stamp),
            "control": jump(control_state, reset_stamp),
        },
        "rate_hz": {
            "global": rate_hz(global_state),
            "control": rate_hz(control_state),
        },
    }
    output = json.dumps(result, indent=2, sort_keys=True)
    print(output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
