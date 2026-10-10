"""Opt-in immutable native Q90 cache; original PNG/infos/GT remain intact."""
import hashlib
import json
from pathlib import Path

from mmcv.transforms import BaseTransform
from mmdet3d.registry import TRANSFORMS


def validate_jpeg_cache(data_root, cache_root, required_paths=(), verify_hashes=False):
    data_root, cache_root = Path(data_root).resolve(strict=True), Path(cache_root).resolve(strict=True)
    manifest = json.loads((cache_root / 'manifest.json').read_text())
    source = json.loads((data_root / 'manifest.json').read_text())
    if (manifest.get('version') != 1 or manifest.get('complete') is not True or
            manifest.get('quality') != 90 or manifest.get('native_resolution') is not True or
            manifest.get('source_color_order') != 'rgb'):
        raise ValueError('Expected complete native RGB-derived Q90 cache')
    digest = hashlib.sha256((data_root / 'manifest.json').read_bytes()).hexdigest()
    if manifest.get('source_manifest_sha256') != digest:
        raise ValueError('JPEG cache source identity differs')
    files = manifest['files']
    if manifest.get('image_count') != len(files):
        raise ValueError('JPEG cache inventory count differs')
    for required in required_paths:
        relative = str(Path(required).resolve(strict=True).relative_to(data_root))
        if relative not in files:
            raise ValueError('JPEG cache is missing required training image: ' + relative)
    for relative, item in files.items():
        original = (data_root / relative).resolve(strict=True)
        original.relative_to(data_root)
        target = (cache_root / item['path']).resolve(strict=True)
        target.relative_to(cache_root)
        if relative not in source['files'] or item['source_sha256'] != source['files'][relative]['sha256']:
            raise ValueError('JPEG cache original PNG identity differs')
        if target.stat().st_size != item['bytes']:
            raise ValueError('JPEG cache size differs')
        if verify_hashes and hashlib.sha256(target.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('JPEG cache checksum differs')
    return manifest


def cache_train_pipeline(pipeline, data_root, cache_root):
    from copy import deepcopy
    pipeline = deepcopy(pipeline)
    domain = [item for item in pipeline if item['type'] == 'MoraiOnlineImageDomainAug3D']
    if (len(domain) != 1 or domain[0]['jpeg_prob'] != 1.0 or
            list(domain[0]['jpeg_quality']) != [90, 90] or
            not domain[0].get('native_resolution') or domain[0].get('color_order') != 'rgb' or
            any(domain[0][key] != 0 for key in ('blur_prob', 'exposure_prob', 'noise_prob'))):
        raise ValueError('JPEG cache requires native Q90 100% and other pixel augmentation OFF')
    pipeline = [item for item in pipeline if item['type'] != 'MoraiOnlineImageDomainAug3D']
    index = next(i for i, item in enumerate(pipeline)
                 if item['type'] == 'BEVLoadMultiViewImageFromFiles')
    pipeline.insert(index, dict(type='MoraiUseNativeJPEGCache',
        data_root=str(Path(data_root).resolve()), cache_root=str(Path(cache_root).resolve())))
    return pipeline


@TRANSFORMS.register_module()
class MoraiUseNativeJPEGCache(BaseTransform):
    """Redirect image filenames before loading; never recompress or edit GT."""
    def __init__(self, data_root, cache_root):
        self.data_root = Path(data_root).resolve(strict=True)
        self.cache_root = Path(cache_root).resolve(strict=True)
        self.manifest = validate_jpeg_cache(self.data_root, self.cache_root)

    def transform(self, data):
        for camera in data['images'].values():
            original = Path(camera['img_path']).resolve(strict=True)
            relative = str(original.relative_to(self.data_root))
            item = self.manifest['files'].get(relative)
            if item is None:
                raise ValueError('JPEG cache misses a required camera image')
            if (camera['height'], camera['width']) != (item['height'], item['width']):
                raise ValueError('JPEG cache/native camera dimensions differ')
            camera['img_path'] = str(self.cache_root / item['path'])
        return data
