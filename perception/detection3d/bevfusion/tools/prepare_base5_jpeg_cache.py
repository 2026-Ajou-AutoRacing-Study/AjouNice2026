#!/usr/bin/env python3
"""Create separate native JPEG Q90 train cache inside the training environment.

Original portable data remains unchanged. Default cache covers train only;
optional val/test copies are not selected by the training runner. No GPU runs.
"""
import argparse
import json
from pathlib import Path
import pickle
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from benchmark_base5_jpeg_cache import build_cache
from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import validate_jpeg_cache


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=['448x896', '512x1056'], default='512x1056')
    parser.add_argument('--splits', nargs='+', choices=['train', 'val', 'test'], default=['train'],
                        help='Generate optional extra copies; training still uses cache for train only')
    parser.add_argument('--verify', action='store_true', help='Verify existing cache only; no overwrite/resume')
    args = parser.parse_args()
    data = args.data.resolve(strict=True)
    records = []
    for split in dict.fromkeys(args.splits):
        raw = pickle.loads((data / 'infos' / args.profile / ('morai_base5_infos_' + split + '.pkl')).read_bytes())
        records.extend(raw['data_list'])
    for record in records:
        for camera in record['images'].values():
            camera['img_path'] = str(data / camera['img_path'])
    required = [camera['img_path'] for record in records for camera in record['images'].values()]
    if not args.verify:
        build_cache(data, records, args.output)
    manifest = validate_jpeg_cache(data, args.output, required, verify_hashes=True)
    print(json.dumps({key:value for key,value in manifest.items() if key != 'files'}, indent=2))


if __name__ == '__main__':
    main()
