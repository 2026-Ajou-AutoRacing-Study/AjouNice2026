#!/usr/bin/env python3
"""Extract a portable Swin checkpoint from a trusted BEVFusion checkpoint."""

import argparse
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    checkpoint = torch.load(args.input, map_location='cpu', weights_only=False)
    state = checkpoint.get('state_dict', checkpoint)
    prefix = 'img_backbone.'
    extracted = {
        'backbone.' + key[len(prefix):]: value
        for key, value in state.items()
        if key.startswith(prefix)
    }
    if not extracted:
        raise RuntimeError('checkpoint contains no img_backbone parameters')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({'state_dict': extracted}, args.output)
    print(f'extracted {len(extracted)} tensors to {args.output}')


if __name__ == '__main__':
    main()
