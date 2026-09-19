#!/usr/bin/env python3
"""Adapt the official 10-class/5-feature BEVFusion checkpoint to MORAI."""

import argparse
import copy
from collections import OrderedDict
from pathlib import Path

import torch


GROUPS = (
    (0, 1, 2, 3, 4, 6, 7),  # nuScenes vehicles -> MORAI Vehicle
    (8,),                    # pedestrian -> Pedestrian
    (5, 9),                  # barrier + traffic cone -> Obstacle
)


def reduce_output(tensor):
    return torch.stack([tensor[list(indices)].mean(dim=0) for indices in GROUPS])


def reduce_input(tensor):
    return torch.stack(
        [tensor[:, list(indices)].mean(dim=1) for indices in GROUPS], dim=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = torch.load(args.input, map_location='cpu', weights_only=False)
    state = source.get('state_dict', source)

    # Base3 contains x, y, z, intensity; discard nuScenes' fifth time-lag input.
    state['pts_middle_encoder.conv_input.0.weight'] = (
        state['pts_middle_encoder.conv_input.0.weight'][..., :4].clone())
    for name in (
            'bbox_head.heatmap_head.1.weight',
            'bbox_head.heatmap_head.1.bias',
            'bbox_head.prediction_heads.0.heatmap.1.weight',
            'bbox_head.prediction_heads.0.heatmap.1.bias'):
        state[name] = reduce_output(state[name])
    state['bbox_head.class_encoding.weight'] = reduce_input(
        state['bbox_head.class_encoding.weight'])
    adapted = OrderedDict(
        (key, value) for key, value in state.items()
        if '.prediction_heads.0.vel.' not in key
        and key not in {
            'view_transform.dx',
            'view_transform.bx',
            'view_transform.nx',
        }
    )
    # MMEngine passes this per-module metadata to spconv's load hook. Dropping
    # it makes current-layout kernels look like legacy spconv kernels, so the
    # hook permutes them a second time and silently skips the sparse encoder.
    if hasattr(state, '_metadata'):
        adapted._metadata = copy.deepcopy(state._metadata)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        'state_dict': adapted,
        'meta': {
            'source': str(args.input),
            'class_mapping': [list(group) for group in GROUPS],
            'lidar_features': ['x', 'y', 'z', 'intensity'],
            'dataset_meta': {
                'classes': ['vehicle', 'pedestrian', 'obstacle'],
                'palette': 'random',
            },
        },
    }, args.output)
    print(f'wrote {len(adapted)} tensors to {args.output}')


if __name__ == '__main__':
    main()
