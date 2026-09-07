#!/usr/bin/env python3
"""Generate OpenPCDet info files and the GT database for a 9D dataset."""

import argparse
import sys
from pathlib import Path

import yaml
from easydict import EasyDict


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--openpcdet-root', default='/home/luv/OpenPCDet')
    parser.add_argument('--dataset-root', required=True)
    parser.add_argument('--dataset-config', required=True)
    parser.add_argument('--workers', type=int, default=4)
    return parser.parse_args()


def main():
    args = parse_args()
    openpcdet_root = Path(args.openpcdet_root).expanduser().resolve()
    dataset_root = Path(args.dataset_root).expanduser().resolve()
    sys.path.insert(0, str(openpcdet_root))

    from pcdet.datasets.custom.custom_dataset import create_custom_infos

    with Path(args.dataset_config).open(encoding='utf-8') as stream:
        dataset_cfg = EasyDict(yaml.safe_load(stream))
    dataset_cfg.DATA_PATH = str(dataset_root)

    create_custom_infos(
        dataset_cfg=dataset_cfg,
        class_names=['Pedestrian', 'Vehicle', 'Obstacle'],
        data_path=dataset_root,
        save_path=dataset_root,
        workers=args.workers,
    )


if __name__ == '__main__':
    main()
