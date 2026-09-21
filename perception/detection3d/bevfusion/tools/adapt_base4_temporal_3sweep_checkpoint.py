#!/usr/bin/env python3
"""Adapt a Base3 checkpoint for Base4-Temporal-3S 5-D LiDAR input."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


INPUT_WEIGHT_KEY = "pts_middle_encoder.conv_input.0.weight"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()

    checkpoint = torch.load(
        args.input,
        map_location="cpu",
        weights_only=False,
    )

    state = checkpoint.get("state_dict", checkpoint)

    if INPUT_WEIGHT_KEY not in state:
        raise KeyError(INPUT_WEIGHT_KEY)

    old = state[INPUT_WEIGHT_KEY]

    if old.shape[-1] == 5:
        print("checkpoint already has 5 input channels")
    elif old.shape[-1] == 4:
        new_shape = list(old.shape)
        new_shape[-1] = 5

        new = old.new_zeros(new_shape)
        new[..., :4] = old

        state[INPUT_WEIGHT_KEY] = new

        print(
            "{}: {} -> {}".format(
                INPUT_WEIGHT_KEY,
                tuple(old.shape),
                tuple(new.shape),
            )
        )
        print("new dt-channel weights: zero initialized")
    else:
        raise ValueError(
            "unexpected LiDAR input channel count: {}".format(old.shape[-1])
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)

    print("saved:", args.output)


if __name__ == "__main__":
    main()
