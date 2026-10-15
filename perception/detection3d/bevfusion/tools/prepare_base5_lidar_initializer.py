#!/usr/bin/env python3
"""Audited nuScenes camera + sparse-LiDAR initializer, NOT E25 fine-tuning.

Legacy BEVFusion convolves coordinates [X,Y,Z]; rectangular Base5 swaps them
to [Y,X,Z]. Transpose the first two spatial kernel axes before loading the
encoder. Do not reuse fusion, depth, head, geometry, or optimizer states.
Actual CUDA transpose-equivariance qualification is required before training.
"""
import argparse
import copy
from collections import OrderedDict
import hashlib
import json
from pathlib import Path

import torch


def build_initializer(camera, sparse):
    state = OrderedDict((k, v.clone()) for k, v in camera['state_dict'].items())
    if not state or any(not k.startswith(('img_backbone.', 'img_neck.')) for k in state):
        raise ValueError('Camera baseline must contain only audited camera weights')
    metadata = copy.deepcopy(getattr(camera['state_dict'], '_metadata', {}))
    source = sparse['state_dict']
    source_metadata = getattr(source, '_metadata', {})
    adapted = []
    for key, value in source.items():
        if not key.startswith('pts_middle_encoder.'):
            continue
        module = key.rsplit('.', 1)[0]
        if value.ndim == 5:
            if source_metadata.get(module, {}).get('version') != 2:
                raise ValueError('Sparse kernel layout must be known spconv v2 [O,X,Y,Z,I]')
            if value.shape[1] != value.shape[2]:
                raise ValueError('Unexpected asymmetric XY kernel; audit stride/padding first')
            if key == 'pts_middle_encoder.conv_input.0.weight' and value.shape[-1] != 4:
                raise ValueError('Require XYZI-only LiDAR features')
            value = value.transpose(1, 2).contiguous()
            adapted.append(key)
        state[key] = value.clone()
    if not adapted:
        raise ValueError('No sparse kernels in nuScenes source')
    metadata.update({k: copy.deepcopy(v) for k, v in source_metadata.items()
                     if k.startswith('pts_middle_encoder')})
    state._metadata = metadata
    return dict(state_dict=state, meta=dict(
        policy='nuscenes_camera_plus_sparse_xy_transpose_v1',
        source_axes='X,Y,Z', target_axes='Y,X,Z',
        sparse_kernel_layout='spconv_v2_O_X_Y_Z_I', transposed_kernels=adapted,
        reset='fusion/BEV/depth/head/geometry/optimizer/scheduler',
        qualification='Requires actual CUDA sparse-encoder equivariance and bounded AP trial'))


def checksum(path):
    digest = hashlib.sha256()
    with path.open('rb') as file:
        for chunk in iter(lambda: file.read(4*1024**2), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=Path, required=True)
    parser.add_argument('--sparse-source', type=Path, required=True)
    parser.add_argument('--sparse-source-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Never overwrite an initializer')
    source_sha = checksum(args.sparse_source)
    if source_sha != args.sparse_source_sha256:
        raise ValueError('NuScenes source checksum differs')
    checkpoint = build_initializer(
        torch.load(args.camera, map_location='cpu', weights_only=False),
        torch.load(args.sparse_source, map_location='cpu', weights_only=False))
    checkpoint['meta'].update(camera_sha256=checksum(args.camera), sparse_source_sha256=source_sha)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    report = dict(path=str(args.output), sha256=checksum(args.output),
                  tensors=len(checkpoint['state_dict']), **checkpoint['meta'])
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
