#!/usr/bin/env python3
"""Coordinate-audited extra nuScenes weights for a bounded Base5 trial.

Input is the official nuScenes -> MORAI3 adapter, never a trained E25.
Keep the qualified camera+sparse baseline unchanged. Transpose learned BEV
convolutions, adapt position encodings/center offsets to physical grid units,
and reset depth logits while retaining the camera context output channels.
This is a candidate initializer, not an accuracy qualification.
"""
import argparse
import copy
import hashlib
import json
from collections import OrderedDict
from pathlib import Path

import torch


SOURCE_SHA256 = '2a9ce3e01a723581f8e76b247887a704c4751f2ebcc9b36884c266466700a247'
BEV_PREFIXES = ('view_transform.downsample.', 'pts_backbone.', 'pts_neck.',
                'fusion_layer.', 'bbox_head.shared_conv.', 'bbox_head.heatmap_head.')
EXTRA_PREFIXES = ('view_transform.', 'pts_backbone.', 'pts_neck.', 'fusion_layer.', 'bbox_head.')
GEOMETRY = {'view_transform.dx', 'view_transform.bx', 'view_transform.nx', 'view_transform.frustum'}


def checksum(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as file:
        for chunk in iter(lambda: file.read(4 * 1024**2), b''):
            digest.update(chunk)
    return digest.hexdigest()


def adapt_position(weight, bias, source_origin, source_pitch, target_origin, target_pitch):
    """W(new_index * scale + offset) + b equals source physical encoding."""
    if weight.ndim != 3 or weight.shape[1:] != (2, 1):
        raise ValueError('Expected learned XY 1x1 position encoding')
    scale = weight.new_tensor(target_pitch) / weight.new_tensor(source_pitch)
    offset = (weight.new_tensor(target_origin) - weight.new_tensor(source_origin)) / weight.new_tensor(source_pitch)
    return weight * scale[None, :, None], bias + weight[:, :, 0] @ offset


def build_delta(source, target, *, source_origin=(-54., -54.), source_pitch=(.6, .6),
                target_origin=(-100.8, -33.6), target_pitch=(.4, .4),
                source_depth_bins=118, target_depth_bins=396, context_channels=80):
    state = source['state_dict']
    if tuple(state['bbox_head.heatmap_head.1.bias'].shape) != (3,):
        raise ValueError('Require the audited three-class nuScenes adapter')
    if any(p <= 0 for p in (*source_pitch, *target_pitch)):
        raise ValueError('Grid pitches must be positive')
    out = OrderedDict()
    report = dict(spatial_transpose=[], position_affine=[], center_unit_scale=[],
                  depth_context_copy=[], copied=[], reset_geometry=sorted(GEOMETRY),
                  reset_depth_logits=target_depth_bins)
    for key, value in state.items():
        if not key.startswith(EXTRA_PREFIXES) or key in GEOMETRY or '.vel.' in key:
            continue
        if key not in target:
            raise ValueError('Unexpected source key: ' + key)
        value = value.clone()
        if key in ('view_transform.depthnet.6.weight', 'view_transform.depthnet.6.bias'):
            if value.shape[0] != source_depth_bins + context_channels or target[key].shape[0] != target_depth_bins + context_channels:
                raise ValueError('Depth/context output partition differs from audited layout')
            if value.shape[1:] != target[key].shape[1:]:
                raise ValueError('Depth/context channel shapes differ')
            # No interpolation/extrapolation of depth logits, no geometry copy.
            # The fresh target output layer supplies all new depth channels.
            fresh = target[key].clone()
            fresh[target_depth_bins:] = value[source_depth_bins:]
            value = fresh
            report['depth_context_copy'].append(key)
        elif key.startswith(BEV_PREFIXES) and value.ndim == 4:
            if value.shape[-1] != value.shape[-2]:
                raise ValueError('Asymmetric BEV kernel requires separate audit: ' + key)
            value = value.transpose(-1, -2).contiguous()
            report['spatial_transpose'].append(key)
        if '.posembed.' in key:
            raise ValueError('Unknown position encoder layout: ' + key)
        if key.endswith('posembed.position_embedding_head.0.weight'):
            bias_key = key[:-6] + 'bias'
            value, bias = adapt_position(value, state[bias_key], source_origin,
                                         source_pitch, target_origin, target_pitch)
            out[bias_key] = bias
            report['position_affine'].append(key)
        elif key.endswith('posembed.position_embedding_head.0.bias'):
            continue  # paired affine conversion above owns this bias
        if key.startswith('bbox_head.prediction_heads.') and '.center.1.' in key:
            if value.shape[0] != 2:
                raise ValueError('Expected XY center output')
            scale = value.new_tensor(source_pitch) / value.new_tensor(target_pitch)
            value = value * scale.reshape(2, *([1] * (value.ndim - 1)))
            report['center_unit_scale'].append(key)
        if value.shape != target[key].shape:
            raise ValueError('Shape mismatch: ' + key)
        if torch.is_floating_point(value) and not torch.isfinite(value).all():
            raise ValueError('Nonfinite initializer: ' + key)
        out[key] = value
        report['copied'].append(key)
    expected = {key for key in target if key.startswith(EXTRA_PREFIXES) and key not in GEOMETRY}
    if set(out) != expected:
        raise ValueError('Incomplete extra-state audit: ' + repr(sorted(expected ^ set(out))))
    out._metadata = {key: copy.deepcopy(value) for key, value in getattr(state, '_metadata', {}).items()
                     if key.startswith(EXTRA_PREFIXES)}
    report.update(source_origin=list(source_origin), source_pitch=list(source_pitch),
                  target_origin=list(target_origin), target_pitch=list(target_pitch),
                  policy='nuscenes_morai3_full_xy_metric_v1', tensors=len(out))
    return dict(state_dict=out, meta=report)


def compose(baseline, delta):
    base = baseline['state_dict']
    if any(not key.startswith(('img_backbone.', 'img_neck.', 'pts_middle_encoder.')) for key in base):
        raise ValueError('Baseline must be qualified camera+sparse only')
    if set(base) & set(delta['state_dict']):
        raise ValueError('Delta must not overwrite qualified baseline')
    state = OrderedDict((key, value.clone()) for key, value in base.items())
    state.update(delta['state_dict'])
    state._metadata = copy.deepcopy(getattr(base, '_metadata', {}))
    state._metadata.update(copy.deepcopy(getattr(delta['state_dict'], '_metadata', {})))
    return dict(state_dict=state, meta=dict(delta['meta'], baseline_meta=baseline.get('meta', {})))


def qualify_extra_modules(model, source, delta):
    """Actual pretrained BEV blocks, rectangular transpose equivariance on CPU."""
    torch.set_num_threads(4)
    def module_state(state, prefix):
        result = OrderedDict((key[len(prefix):], value) for key, value in state.items()
                             if key.startswith(prefix))
        result._metadata = {key[len(prefix):]: value for key, value in getattr(state, '_metadata', {}).items()
                            if key.startswith(prefix)}
        return result
    def pair(module, prefix):
        old, new = copy.deepcopy(module).eval(), copy.deepcopy(module).eval()
        old.load_state_dict(module_state(source['state_dict'], prefix), strict=True)
        new.load_state_dict(module_state(delta['state_dict'], prefix), strict=True)
        return old, new
    errors = {}
    def compare(name, actual, expected):
        torch.testing.assert_close(actual, expected, atol=1e-4, rtol=1e-4)
        errors[name] = float((actual - expected).abs().max())
    torch.manual_seed(23456)
    with torch.no_grad():
        old, new = pair(model.view_transform.downsample, 'view_transform.downsample.')
        x = torch.randn(1, 80, 32, 48)
        compare('camera_bev_downsample', new(x.transpose(-1, -2)), old(x).transpose(-1, -2))
        old_backbone, new_backbone = pair(model.pts_backbone, 'pts_backbone.')
        old_neck, new_neck = pair(model.pts_neck, 'pts_neck.')
        x = torch.randn(1, 256, 32, 48)
        expected = old_neck(old_backbone(x))
        actual = new_neck(new_backbone(x.transpose(-1, -2)))
        for i, (new_value, old_value) in enumerate(zip(actual, expected)):
            compare('lidar_second_fpn_' + str(i), new_value, old_value.transpose(-1, -2))
        old, new = pair(model.fusion_layer, 'fusion_layer.')
        inputs = [torch.randn(1, channels, 24, 32) for channels in (80, 256)]
        compare('fusion', new([x.transpose(-1, -2) for x in inputs]), old(inputs).transpose(-1, -2))
        old, new = pair(model.bbox_head.shared_conv, 'bbox_head.shared_conv.')
        old_heatmap, new_heatmap = pair(model.bbox_head.heatmap_head, 'bbox_head.heatmap_head.')
        x = torch.randn(1, 512, 24, 32)
        compare('head_dense_heatmap', new_heatmap(new(x.transpose(-1, -2))),
                old_heatmap(old(x)).transpose(-1, -2))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--delta', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.report.exists():
        parser.error('Use fresh initializer/report paths')
    if args.delta:
        if not args.baseline or args.source or args.config:
            parser.error('Compose requires --delta --baseline only')
        delta = torch.load(args.delta, map_location='cpu', weights_only=False)
        if delta['meta'].get('source_sha256') != SOURCE_SHA256:
            raise ValueError('Untrusted delta provenance')
        result = compose(torch.load(args.baseline, map_location='cpu', weights_only=False), delta)
        result['meta'].update(baseline_sha256=checksum(args.baseline), delta_sha256=checksum(args.delta))
    else:
        if not args.source or not args.config or args.baseline:
            parser.error('Build delta requires --source --config')
        if checksum(args.source) != SOURCE_SHA256:
            raise ValueError('Require official nuScenes MORAI3 source, not E25')
        from mmengine.config import Config
        from mmengine.registry import init_default_scope
        from mmengine.utils import import_modules_from_strings
        from mmdet3d.registry import MODELS
        cfg = Config.fromfile(str(args.config))
        import_modules_from_strings(**cfg.custom_imports)
        init_default_scope('mmdet3d')
        if tuple(cfg.classes) != ('vehicle', 'pedestrian', 'obstacle'):
            raise ValueError('Candidate requires the selected merged three-class config')
        torch.manual_seed(261010)
        model = MODELS.build(cfg.model)
        model.init_weights()
        head = cfg.model.bbox_head
        pitch = [float(v) * head.test_cfg.out_size_factor for v in head.test_cfg.voxel_size]
        depth = cfg.model.view_transform.dbound
        if tuple(depth) != (1., 100., .25):
            raise ValueError('This candidate is qualified for depth25 only')
        source = torch.load(args.source, map_location='cpu', weights_only=False)
        result = build_delta(source,
                             model.state_dict(), target_origin=head.test_cfg.pc_range,
                             target_pitch=pitch)
        result['meta'].update(source_sha256=SOURCE_SHA256, target_config_sha256=checksum(args.config),
                              depth_reset_seed=261010,
                              cpu_module_equivariance_max_abs=qualify_extra_modules(model, source, result))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    torch.save(result, args.output)
    report = dict(result['meta'], path=str(args.output), sha256=checksum(args.output),
                  bytes=args.output.stat().st_size, tensors=len(result['state_dict']))
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('path', 'sha256', 'bytes', 'tensors')}))


if __name__ == '__main__':
    main()
