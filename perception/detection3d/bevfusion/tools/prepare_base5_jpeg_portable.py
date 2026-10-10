#!/usr/bin/env python3
"""Export relocatable train JPEG + clean val PNG, without train PNG dependency.

Keep original source dataset untouched. LiDAR/native annotation/metadata/splits
are preserved, not just images. Run inside the BEVFusion environment. No GPU.
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import pickle
import shutil

from prepare_base5_portable_dataset import safe_relative, sha256, validate
from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import validate_jpeg_cache


def build(data, cache, output, hardlink=False):
    data, cache = Path(data).resolve(strict=True), Path(cache).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists():
        raise FileExistsError('Use a fresh JPEG portable output')
    source = validate(data, verify_hashes=True)
    if source.get('image_policy'):
        raise ValueError('Expected original PNG portable source')
    exports, train_paths, clean_paths = {}, set(), set()
    for profile in source['profiles']:
        exports[profile] = {}
        for split in ('train', 'val', 'test'):
            relative = Path('infos') / profile / ('morai_base5_infos_' + split + '.pkl')
            info = pickle.loads((data / relative).read_bytes())
            exports[profile][split] = info
            paths = train_paths if split == 'train' else clean_paths
            for row in info['data_list']:
                for camera in row['images'].values():
                    paths.add(str(safe_relative(camera['img_path'])))
    if train_paths & clean_paths:
        raise ValueError('Train and clean validation image paths overlap')
    index = validate_jpeg_cache(data, cache, [data / p for p in train_paths], verify_hashes=True)
    mapping = {p: str(Path('jpeg_train') / Path(index['files'][p]['path']).name) for p in train_paths}
    for profile, splits in exports.items():
        for row in splits['train']['data_list']:
            for camera in row['images'].values():
                p = camera['img_path']
                item = index['files'][p]
                if (camera['height'], camera['width']) != (item['height'], item['width']):
                    raise ValueError('Native camera dimensions differ')
                camera['img_path'] = mapping[p]
    output.mkdir(parents=True)
    files = {}

    def transfer(original, relative, metadata):
        destination = output / safe_relative(relative)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if hardlink:
            os.link(original, destination)
        else:
            shutil.copy2(original, destination)
        files[str(relative)] = deepcopy(metadata)

    rewritten = {str(Path('infos') / p / 'morai_base5_infos_train.pkl') for p in source['profiles']}
    for relative, metadata in source['files'].items():
        if relative in train_paths or relative in rewritten:
            continue
        transfer(data / relative, relative, metadata)
    for relative in sorted(train_paths):
        item = index['files'][relative]
        transfer(cache / item['path'], mapping[relative], dict(bytes=item['bytes'], sha256=item['sha256']))
    for profile, splits in exports.items():
        relative = Path('infos') / profile / 'morai_base5_infos_train.pkl'
        path = output / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(pickle.dumps(splits['train'], protocol=4))
        files[str(relative)] = dict(bytes=path.stat().st_size, sha256=sha256(path))
    transfer(data / 'manifest.json', 'provenance/png_portable_manifest.json',
             dict(bytes=(data / 'manifest.json').stat().st_size, sha256=sha256(data / 'manifest.json')))
    transfer(cache / 'manifest.json', 'provenance/native_jpeg_cache_manifest.json',
             dict(bytes=(cache / 'manifest.json').stat().st_size, sha256=sha256(cache / 'manifest.json')))
    result = deepcopy(source)
    result.update(files=files, bytes=sum(v['bytes'] for v in files.values()),
        copy_mode='hardlink' if hardlink else 'copy',
        image_policy=dict(train='native_jpeg90', val='original_png', test='original_png'),
        source_png_manifest_sha256=sha256(data / 'manifest.json'),
        source_jpeg_cache_manifest_sha256=sha256(cache / 'manifest.json'),
        jpeg_train_image_count=len(train_paths), original_train_png_bytes=sum(source['files'][p]['bytes'] for p in train_paths),
        jpeg_train_image_bytes=sum(index['files'][p]['bytes'] for p in train_paths),
        source_info_sha256={p:{s:sha256(data / 'infos' / p / ('morai_base5_infos_' + s + '.pkl'))
                            for s in ('train', 'val', 'test')} for p in source['profiles']})
    (output / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    validate(output, verify_hashes=True)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--cache', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--hardlink', action='store_true', help='Immutable local export only; never edit shared payload')
    a = p.parse_args()
    result = build(a.data, a.cache, a.output, a.hardlink)
    print(json.dumps({k:v for k,v in result.items() if k not in ('files','provenance','source_info_sha256')}, indent=2))


if __name__ == '__main__':
    main()
