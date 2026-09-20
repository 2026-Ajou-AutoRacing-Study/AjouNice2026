#!/usr/bin/env python3
"""Expand Base3's sparse-conv input from XYZI to XYZI+time.

The first four channels are copied exactly and the new time channel starts at
zero.  Velocity-head parameters intentionally remain absent so MMEngine
initializes that new head from the model config.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


KEY = "pts_middle_encoder.conv_input.0.weight"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_checkpoint")
    parser.add_argument("output_checkpoint")
    args = parser.parse_args()

    source = Path(args.input_checkpoint)
    destination = Path(args.output_checkpoint)
    checkpoint = torch.load(source, map_location="cpu", weights_only=False)
    state_dict = checkpoint.get("state_dict", checkpoint)
    if KEY not in state_dict:
        raise KeyError(f"missing expected sparse input weight: {KEY}")
    weight = state_dict[KEY]
    if weight.shape[-1] == 5:
        expanded = weight
    elif weight.shape[-1] == 4:
        expanded = weight.new_zeros((*weight.shape[:-1], 5))
        expanded[..., :4] = weight
    else:
        raise ValueError(
            f"expected {KEY} input width 4 or 5, got {tuple(weight.shape)}"
        )
    state_dict[KEY] = expanded
    destination.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, destination)
    print({
        "input": str(source),
        "output": str(destination),
        "key": KEY,
        "old_shape": tuple(weight.shape),
        "new_shape": tuple(expanded.shape),
        "time_channel_init_abs_max": float(expanded[..., 4].abs().max()),
    })


if __name__ == "__main__":
    main()
