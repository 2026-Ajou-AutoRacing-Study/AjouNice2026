#!/usr/bin/env python3
"""CPU-only package/source/config qualification; not a GPU training certificate."""
import hashlib
import importlib.metadata
import json
from pathlib import Path


def main():
    root = Path('/opt/ajounice')
    inventory = json.loads((root / 'source_inventory.json').read_text())
    for relative, digest in inventory['files'].items():
        assert hashlib.sha256((root / relative).read_bytes()).hexdigest() == digest, relative
    import torch
    import mmcv.ops
    import spconv.pytorch
    from mmengine.config import Config
    from mmengine.utils import import_modules_from_strings
    config = Config.fromfile(str(root / 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py'))
    import_modules_from_strings(**config.custom_imports)
    assert config.model.compile_camera is False
    assert tuple(config.classes) == ('vehicle', 'pedestrian', 'obstacle')
    assert config.model.view_transform.image_size == [512, 1056]
    assert importlib.metadata.version('wandb') == '0.22.3'
    print(json.dumps(dict(passed=True, source_files=len(inventory['files']), torch=torch.__version__,
                          cuda=torch.version.cuda, compile_camera=False,
                          scope='CPU imports/config/source only; no GPU, checkpoint or dataset tested')))


if __name__ == '__main__':
    main()
