#!/usr/bin/env python3
"""Build a full, initialized COST-only checkpoint without any learning.

Reuse an audited local camera-only initializer on the remote host rather than
uploading optimizer state. Never call this a trained or resumable checkpoint.
"""
import argparse
import hashlib
import json
from pathlib import Path

import torch
from mmengine.config import Config
from mmengine.runner import set_random_seed
from mmdet3d.apis import init_model

from base5_grid_candidates import candidate_config
from app.view_geometry import restore_view_geometry
from app.inference_optimization import validate_base5_runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('config', 'initializer', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--depth-step', type=float, choices=(.25, .5), default=.25)
    parser.add_argument('--full-shape-source', type=Path,
                        help='Trusted D50 smoke model; duplicate depth-logit rows for D25 COST only')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Fresh output required')
    torch.set_num_threads(4)
    set_random_seed(261011)
    cfg = candidate_config(Config.fromfile(str(args.config)), 'current075', args.depth_step, 512)
    validate_base5_runtime(cfg)
    model = init_model(cfg, None, device='cpu')
    model.init_weights()
    source = torch.load(args.initializer, map_location='cpu', weights_only=False)
    weights = source['state_dict']
    if not weights or any(not k.startswith(('img_backbone.', 'img_neck.')) for k in weights):
        raise ValueError('Use the audited camera-only initializer, not a full checkpoint')
    state = model.state_dict()
    required = {k for k in state if k.startswith(('img_backbone.', 'img_neck.'))}
    if set(weights) != required:
        raise ValueError('Camera initializer must contain every backbone/neck tensor')
    for name, value in weights.items():
        if name not in state or state[name].shape != value.shape:
            raise ValueError('Camera initializer does not match: ' + name)
    incompatible = model.load_state_dict(weights, strict=False)
    if incompatible.unexpected_keys:
        raise ValueError('Unexpected camera initializer keys')
    for name, value in weights.items():
        if not torch.equal(model.state_dict()[name], value):
            raise ValueError('Camera initializer changed: ' + name)
    shape_source_sha256 = None
    if args.full_shape_source:
        if args.depth_step != .25:
            raise ValueError('Shape transfer is only qualified for D50 to D25 cost probes')
        full = torch.load(args.full_shape_source, map_location='cpu', weights_only=False)
        if list(full.get('meta', {}).get('dataset_meta', {}).get('classes', [])) != list(cfg.metainfo.classes):
            raise ValueError('Full shape source class contract differs')
        state = model.state_dict()
        if set(full['state_dict']) != set(state):
            raise ValueError('Full shape source keys differ')
        transformed = []
        for key, value in full['state_dict'].items():
            if value.shape != state[key].shape:
                if key == 'view_transform.frustum':
                    # Config-owned depth/pixel coordinates, not learned weights.
                    value = state[key]
                    transformed.append(key)
                elif key not in ('view_transform.depthnet.6.weight', 'view_transform.depthnet.6.bias'):
                    raise ValueError('Unexpected shape transfer: '+key)
                else:
                    channels = model.view_transform.C
                    source_bins = value.shape[0]-channels
                    if source_bins != 198 or state[key].shape[0]-channels != 396:
                        raise ValueError('Expected D50/D25 output-bin counts')
                    value = torch.cat((value[:source_bins].repeat_interleave(2, dim=0),
                                       value[source_bins:]), dim=0)
                    transformed.append(key)
            state[key] = value
        if set(transformed) != {'view_transform.frustum','view_transform.depthnet.6.weight','view_transform.depthnet.6.bias'}:
            raise ValueError('Expected two depth-logit tensors and the config-owned frustum')
        model.load_state_dict(state, strict=True)
        shape_source_sha256 = hashlib.sha256(args.full_shape_source.read_bytes()).hexdigest()
    restore_view_geometry(model.view_transform)
    model.eval()
    meta = dict(dataset_meta=dict(classes=list(cfg.metainfo.classes)),
                cost_only=True, optimizer_updates=0, seed=261011,
                initialization='audited camera-only; other parameters freshly initialized',
                initializer_sha256=hashlib.sha256(args.initializer.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                input_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                depth_step=args.depth_step, config=cfg.pretty_text)
    if shape_source_sha256:
        meta.update(shape_source_sha256=shape_source_sha256,
                    source_checkpoint_iterations=full.get('meta',{}).get('iter'),
                    camera_override_from_full_shape_source=True,
                    initialization='D50 smoke full model; paired D25 depth-logit initialization, COST ONLY',
                    warning='Not a trained D25 model or the initialization policy for main training')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(dict(state_dict=model.state_dict(), meta=meta), args.output)
    report = dict(**{k:v for k,v in meta.items() if k != 'config'},
                  full_checkpoint_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                  state_tensors=len(model.state_dict()), camera_tensors=len(weights),
                  output_bytes=args.output.stat().st_size)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
