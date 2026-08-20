#!/usr/bin/env python3
"""Expand an OpenPCDet AnchorHeadSingle checkpoint from 7D to 9D boxes."""

import argparse
from pathlib import Path

import torch


WEIGHT_KEY = "dense_head.conv_box.weight"
BIAS_KEY = "dense_head.conv_box.bias"
VFE_WEIGHT_KEY = "vfe.pfn_layers.0.linear.weight"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--old-code-size", type=int, default=7)
    parser.add_argument("--new-code-size", type=int, default=9)
    parser.add_argument(
        "--append-point-feature",
        action="store_true",
        help="Insert one zero-initialized raw feature before PillarVFE cluster/center offsets.",
    )
    parser.add_argument("--old-raw-feature-count", type=int, default=9)
    return parser.parse_args()


def expand_anchor_channels(tensor, old_code_size, new_code_size):
    if tensor.shape[0] % old_code_size:
        raise ValueError(
            f"output channels {tensor.shape[0]} are not divisible by {old_code_size}"
        )
    anchor_count = tensor.shape[0] // old_code_size
    output = tensor.new_zeros((anchor_count * new_code_size,) + tensor.shape[1:])
    old = tensor.reshape((anchor_count, old_code_size) + tensor.shape[1:])
    new = output.reshape((anchor_count, new_code_size) + tensor.shape[1:])
    new[:, :old_code_size].copy_(old)
    return output


def insert_vfe_raw_feature(weight, old_raw_feature_count):
    if weight.ndim != 2 or weight.shape[1] <= old_raw_feature_count:
        raise ValueError("unexpected PillarVFE input weight shape")
    output = weight.new_zeros((weight.shape[0], weight.shape[1] + 1))
    output[:, :old_raw_feature_count].copy_(weight[:, :old_raw_feature_count])
    output[:, old_raw_feature_count + 1 :].copy_(weight[:, old_raw_feature_count:])
    return output


def migrate_checkpoint(
    checkpoint,
    old_code_size=7,
    new_code_size=9,
    append_point_feature=False,
    old_raw_feature_count=9,
):
    if "model_state" not in checkpoint:
        raise KeyError("OpenPCDet checkpoint has no model_state")
    state = checkpoint["model_state"]
    for key in (WEIGHT_KEY, BIAS_KEY):
        if key not in state:
            raise KeyError(f"checkpoint has no {key}")
        state[key] = expand_anchor_channels(
            state[key], old_code_size, new_code_size
        )
    if append_point_feature:
        if VFE_WEIGHT_KEY not in state:
            raise KeyError(f"checkpoint has no {VFE_WEIGHT_KEY}")
        state[VFE_WEIGHT_KEY] = insert_vfe_raw_feature(
            state[VFE_WEIGHT_KEY], old_raw_feature_count
        )
    checkpoint.pop("optimizer_state", None)
    checkpoint["epoch"] = -1
    checkpoint["it"] = 0
    checkpoint["velocity_head_migration"] = {
        "old_code_size": old_code_size,
        "new_code_size": new_code_size,
        "velocity_initialization": "zeros",
        "appended_point_feature": bool(append_point_feature),
        "old_raw_feature_count": old_raw_feature_count,
    }
    return checkpoint


def main():
    args = parse_args()
    checkpoint = torch.load(args.input, map_location="cpu", weights_only=False)
    checkpoint = migrate_checkpoint(
        checkpoint,
        args.old_code_size,
        args.new_code_size,
        append_point_feature=args.append_point_feature,
        old_raw_feature_count=args.old_raw_feature_count,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    state = checkpoint["model_state"]
    print("output:", args.output.resolve())
    print("conv_box.weight:", tuple(state[WEIGHT_KEY].shape))
    print("conv_box.bias:", tuple(state[BIAS_KEY].shape))
    if args.append_point_feature:
        print("vfe input weight:", tuple(state[VFE_WEIGHT_KEY].shape))


if __name__ == "__main__":
    main()
