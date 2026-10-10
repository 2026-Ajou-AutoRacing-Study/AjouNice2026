#!/usr/bin/env python3
"""Audited conservative nuScenes camera initialization for rectangular Base5.

This is NOT full pretrained BEVFusion transfer. Legacy square BEV weights have
different learned XY orientation; shape equality is insufficient. Use this
camera-only fallback or independently qualify a full coordinate-aware adapter.
"""
import argparse
from collections import Counter, OrderedDict
import copy
import json
from pathlib import Path

import torch
from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmengine.utils import import_modules_from_strings
from mmengine.runner.checkpoint import load_checkpoint
from mmdet3d.registry import MODELS

from prepare_base5_portable_dataset import sha256


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists() or a.report.exists():
        p.error('Use fresh output/report paths')
    cfg = Config.fromfile(str(a.config))
    import_modules_from_strings(**cfg.custom_imports)
    init_default_scope('mmdet3d')
    torch.manual_seed(261010)
    model = MODELS.build(cfg.model)
    model.init_weights()
    target = model.state_dict()
    source = torch.load(a.source, map_location='cpu', weights_only=False)
    state = source.get('state_dict', source)
    loaded = OrderedDict(); dropped = []; mismatched = []
    for key, tensor in state.items():
        if not key.startswith(('img_backbone.', 'img_neck.')):
            dropped.append(key); continue
        if key not in target or target[key].shape != tensor.shape:
            mismatched.append(key); continue
        loaded[key] = tensor
    required = [key for key in target if key.startswith(('img_backbone.', 'img_neck.'))]
    missing_camera = sorted(set(required) - set(loaded))
    if missing_camera or mismatched or not loaded:
        raise ValueError('Incomplete camera transfer: ' + repr((missing_camera, mismatched)))
    if hasattr(state, '_metadata'):
        loaded._metadata = copy.deepcopy(state._metadata)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(dict(state_dict=loaded, meta=dict(initialization='nuScenes camera-only fallback',
        source_sha256=sha256(a.source), classes=list(cfg.classes),
        reset='LiDAR/BEV/fusion/depth/head/animal initialized fresh')), a.output)
    # Test the real MMEngine loader, not merely shape matching.
    load_checkpoint(model, str(a.output), map_location='cpu', strict=False)
    for key, tensor in loaded.items():
        torch.testing.assert_close(model.state_dict()[key], tensor, rtol=0, atol=0)
    reset = sorted(set(target) - set(loaded))
    report = dict(scope='qualified camera-only fallback; full encoder/fusion transfer NOT qualified',
        config=str(a.config), source=str(a.source), source_sha256=sha256(a.source),
        output=str(a.output), output_sha256=sha256(a.output), loaded=list(loaded),
        loaded_elements=sum(item.numel() for item in loaded.values()),
        total_state_elements=sum(item.numel() for item in target.values()),
        dropped=dropped, reset=reset, reset_modules=dict(Counter(key.split('.')[0] for key in reset)),
        camera_missing=missing_camera, mismatched=mismatched, actual_loader_exact=True)
    a.report.parent.mkdir(parents=True, exist_ok=True)
    a.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('scope', 'loaded_elements', 'total_state_elements', 'reset_modules', 'actual_loader_exact')}, indent=2))


if __name__ == '__main__': main()
